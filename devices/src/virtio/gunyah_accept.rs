// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.


//! virtio-gunyah-accept: host->guest transport for VmAccept::Sync.
//!
//! vm_control runtime-SHAREs a memparcel to the protected guest, then asks the in-VM accept
//! module -- over this device -- to `gh_rm_mem_accept` it at the attach GPA (and to release it
//! before the host unshares). Queue 0 (requestq) carries host->guest requests written into
//! guest-posted device-writable buffers; queue 1 (completionq) carries guest->host completions.
//!
//! The host side of the round trip is `vm_control::drive_guest_accept`, which sends
//! [`GunyahAcceptRequest`] over a Tube whose other end this device's worker holds.

use std::collections::BTreeMap;
use std::collections::VecDeque;
use std::io::Read;
use std::io::Write;

use anyhow::anyhow;
use anyhow::Context;
use base::error;
use base::warn;
use base::AsRawDescriptor;
use base::Event;
use base::EventToken;
use base::RawDescriptor;
use base::ReadNotifier;
use base::Tube;
use base::WaitContext;
use base::WorkerThread;
use snapshot::AnySnapshot;
use vm_control::GunyahAcceptOp;
use vm_control::GunyahAcceptRequest;
use vm_control::GunyahAcceptResponse;
use vm_memory::GuestMemory;

use super::DeviceType;
use super::Interrupt;
use super::Queue;
use super::VirtioDevice;

const QUEUE_SIZE: u16 = 16;

// Wire ops, shared with the guest module (virtio_gunyah_accept.c).
const VGA_OP_ACCEPT: u32 = 1;
const VGA_OP_RELEASE: u32 = 2;

// struct virtio_gunyah_accept_req, little-endian.
const REQ_WIRE_LEN: usize = 32;
// struct virtio_gunyah_accept_comp, little-endian.
const COMP_WIRE_LEN: usize = 8;

struct Worker {
    req_queue: Queue,
    comp_queue: Queue,
    tube: Tube,
    /// Requests received over the tube but not yet written into a guest request buffer
    /// (e.g. the guest module has not posted buffers yet).
    pending: VecDeque<GunyahAcceptRequest>,
    /// Wire req_id -> tube seq for requests the guest is currently processing.
    in_flight: BTreeMap<u32, u64>,
    next_req_id: u32,
}

impl Worker {
    /// Move pending requests into guest-posted request buffers.
    fn flush_pending(&mut self) {
        let mut needs_interrupt = false;
        while let Some(req) = self.pending.front() {
            let Some(mut avail_desc) = self.req_queue.pop() else {
                break;
            };
            let req_id = self.next_req_id;
            self.next_req_id = self.next_req_id.wrapping_add(1);

            let op = match req.op {
                GunyahAcceptOp::Accept => VGA_OP_ACCEPT,
                GunyahAcceptOp::Release => VGA_OP_RELEASE,
            };
            let mut wire = [0u8; REQ_WIRE_LEN];
            wire[0..4].copy_from_slice(&req_id.to_le_bytes());
            wire[4..8].copy_from_slice(&op.to_le_bytes());
            wire[8..12].copy_from_slice(&req.handle.to_le_bytes());
            // wire[12..16]: flags, reserved 0.
            wire[16..24].copy_from_slice(&req.gpa.to_le_bytes());
            wire[24..32].copy_from_slice(&req.size.to_le_bytes());

            let writer = &mut avail_desc.writer;
            if let Err(e) = writer.write_all(&wire) {
                warn!("gunyah-accept: failed writing request to guest buffer: {}", e);
                self.req_queue.add_used(avail_desc, 0);
                needs_interrupt = true;
                continue;
            }
            let written = writer.bytes_written();
            self.req_queue.add_used(avail_desc, written as u32);
            needs_interrupt = true;

            let req = self.pending.pop_front().unwrap();
            self.in_flight.insert(req_id, req.seq);
        }
        if needs_interrupt {
            self.req_queue.trigger_interrupt();
        }
    }

    /// Drain guest completions and relay them back over the tube.
    fn process_completions(&mut self) {
        let mut needs_interrupt = false;
        while let Some(mut avail_desc) = self.comp_queue.pop() {
            let mut wire = [0u8; COMP_WIRE_LEN];
            let reader = &mut avail_desc.reader;
            let parsed = reader.read_exact(&mut wire).is_ok();
            self.comp_queue.add_used(avail_desc, 0);
            needs_interrupt = true;
            if !parsed {
                warn!("gunyah-accept: short completion from guest");
                continue;
            }
            let req_id = u32::from_le_bytes(wire[0..4].try_into().unwrap());
            let ret = i32::from_le_bytes(wire[4..8].try_into().unwrap());
            let Some(seq) = self.in_flight.remove(&req_id) else {
                warn!("gunyah-accept: completion for unknown req_id {}", req_id);
                continue;
            };
            if let Err(e) = self.tube.send(&GunyahAcceptResponse { seq, ret }) {
                error!("gunyah-accept: failed to relay completion: {}", e);
            }
        }
        if needs_interrupt {
            self.comp_queue.trigger_interrupt();
        }
    }

    fn run(&mut self, kill_evt: Event) -> anyhow::Result<()> {
        #[derive(EventToken)]
        enum Token {
            RequestQueueAvailable,
            CompletionQueueAvailable,
            TubeReadable,
            Kill,
        }

        let wait_ctx = WaitContext::build_with(&[
            (self.req_queue.event(), Token::RequestQueueAvailable),
            (self.comp_queue.event(), Token::CompletionQueueAvailable),
            (self.tube.get_read_notifier(), Token::TubeReadable),
            (&kill_evt, Token::Kill),
        ])
        .context("failed creating WaitContext")?;

        let mut exiting = false;
        while !exiting {
            let events = wait_ctx.wait().context("failed polling for events")?;
            for event in events.iter().filter(|e| e.is_readable) {
                match event.token {
                    Token::RequestQueueAvailable => {
                        self.req_queue
                            .event()
                            .wait()
                            .context("failed reading queue Event")?;
                        // Guest (re)posted request buffers; ship anything queued.
                        self.flush_pending();
                    }
                    Token::CompletionQueueAvailable => {
                        self.comp_queue
                            .event()
                            .wait()
                            .context("failed reading queue Event")?;
                        self.process_completions();
                    }
                    Token::TubeReadable => {
                        match self.tube.recv::<GunyahAcceptRequest>() {
                            Ok(req) => {
                                self.pending.push_back(req);
                                self.flush_pending();
                            }
                            Err(e) => {
                                // vm_control end closed; nothing left to serve.
                                warn!("gunyah-accept: tube recv failed: {}", e);
                                exiting = true;
                            }
                        }
                    }
                    Token::Kill => exiting = true,
                }
            }
        }

        Ok(())
    }
}

/// The virtio-gunyah-accept device.
pub struct GunyahAccept {
    worker_thread: Option<WorkerThread<Worker>>,
    tube: Option<Tube>,
    virtio_features: u64,
}

impl GunyahAccept {
        GunyahAccept {
            worker_thread: None,
            tube: Some(tube),
            virtio_features,
        }
    }
}

impl VirtioDevice for GunyahAccept {
    fn keep_rds(&self) -> Vec<RawDescriptor> {
        }
    }

    fn device_type(&self) -> DeviceType {
        DeviceType::GunyahAccept
    }

    fn queue_max_sizes(&self) -> &[u16] {
        QUEUE_SIZES
    }

    fn features(&self) -> u64 {
        self.virtio_features
    }

    fn activate(
        &mut self,
        _interrupt: Interrupt,
        mut queues: BTreeMap<usize, Queue>,
    ) -> anyhow::Result<()> {
        }

        let req_queue = queues.remove(&0).unwrap();
        let comp_queue = queues.remove(&1).unwrap();
        let tube = self
            .tube
            .take()
            .context("gunyah-accept activated without a tube")?;

        self.worker_thread = Some(WorkerThread::start("v_gunyah_accept", move |kill_evt| {
            let mut worker = Worker {
                req_queue,
                comp_queue,
                tube,
                pending: VecDeque::new(),
                in_flight: BTreeMap::new(),
                next_req_id: 1,
            };
            if let Err(e) = worker.run(kill_evt) {
                error!("gunyah-accept worker thread failed: {:#}", e);
            }
            worker
        }));

        Ok(())
    }

    fn reset(&mut self) -> anyhow::Result<()> {
        if let Some(worker_thread) = self.worker_thread.take() {
            let worker = worker_thread.stop();
            self.tube = Some(worker.tube);
        }
        Ok(())
    }

    fn virtio_sleep(&mut self) -> anyhow::Result<Option<BTreeMap<usize, Queue>>> {
        if let Some(worker_thread) = self.worker_thread.take() {
            let worker = worker_thread.stop();
            self.tube = Some(worker.tube);
        }
            Ok(None)
    }

    fn virtio_wake(
        &mut self,
        queues_state: Option<(GuestMemory, Interrupt, BTreeMap<usize, Queue>)>,
    ) -> anyhow::Result<()> {
        if let Some((mem, interrupt, queues)) = queues_state {
            self.activate(mem, interrupt, queues)?;
        }
        Ok(())
    }

    fn virtio_snapshot(&mut self) -> anyhow::Result<AnySnapshot> {
        AnySnapshot::to_any(())
    }

    fn virtio_restore(&mut self, data: AnySnapshot) -> anyhow::Result<()> {
        let () = AnySnapshot::from_any(data)?;
        Ok(())
    }
}
