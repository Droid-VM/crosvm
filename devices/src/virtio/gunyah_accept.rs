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

// Debug-only, for the growable-pool test driver: take and drop the same host-side reference that
// a dma-buf import takes, so the "cannot release a grant something is using" path can be
// exercised without making a production pool growable. No production caller sends these.
const VGP_OP_TEST_REF: u32 = 100;
const VGP_OP_TEST_UNREF: u32 = 101;
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

            VGP_OP_QUERY => {
                if offset == 0 && len == 0 {
                    match self.mem.pool_live_grants(pool_id) {
                    }
                } else {
                    match self.mem.pool_range_backed(pool_id, offset, len) {
                        Some(backed) => (0, backed as u64),
                        None => (-libc::ENODEV, 0),
                    }
                }
            VGP_OP_TEST_REF | VGP_OP_TEST_UNREF => (self.test_ref(op, pool_id, offset, len), 0),
        if let Err(e) = self.prepare_folios(gpa, len) {
            // PrepareBlobRange cannot create a guest mapping. If it failed after partially
            // populating the sparse memfd, this is the one failure path whose state is known, so
            // it is safe to return those pages immediately.
            if let Err(punch_err) = self.punch_range(gpa, len) {
                warn!(
                    "gunyah-pool: failed to roll back prepared range {:#x}+{:#x}: {:#}",
                    gpa, len, punch_err
                );
            }
            return Err(e);
        }
            Ok(VmMemoryResponse::Err(e)) => Err(anyhow::Error::new(e)),
    /// Debug: take or drop a reference on a range, as a dma-buf import would.
    ///
    /// The real reference is taken in resource_create_blob, which only fires for a pool the GPU
    /// uses -- and those are all fully pre-shared, so nothing on device would otherwise reach the
    /// busy path at all. This makes the test driver able to, without changing which pool the GPU
    /// is given.
    fn test_ref(&mut self, op: u32, pool_id: u32, offset: u64, len: u64) -> i32 {
        let Some(base) = self.mem.pool_base(pool_id) else {
            return -libc::ENODEV;
        };
        let iov = [(GuestAddress(base.offset() + offset), len as usize)];
        if op == VGP_OP_TEST_REF {
            match self.mem.pool_ref_iovecs(&iov) {
                Ok(()) => 0,
                Err(e) => -e.as_errno(),
            }
        } else {
            self.mem.pool_unref_iovecs(&iov);
            0
        }
    }

            // RegisterMemory may have reached the VM handler before its response was lost. Do
            // not punch this range here: without an explicit "mapping was never established"
            // result, reclaiming it could race a live guest/host mapping. The prepared pages are
            // deliberately retained as the conservative recovery state.
            return -e
                .downcast_ref::<base::Error>()
                .map_or(libc::ENOMEM, |errno| errno.errno());
        // Reserve before unregistering. The reservation makes the reference check and the
        // unregister effectively one operation from the resource-create path's perspective:
        // another thread cannot add a dma-buf reference while this request is in flight.
        if let Err(e) = self.mem.pool_begin_unshare(pool_id, offset, len) {
            self.mem.pool_cancel_unshare(pool_id, offset, len);
                Ok(VmMemoryResponse::Err(e)) => Err(anyhow::Error::new(e)),
                         trying to restore the mapping",

                    // UnregisterMemory has already released the guest acceptance and the host
                    // runtime mapping. Keep the grant reserved while trying to put that mapping
                    // back; otherwise the guest would retain the old backed prefix and could
                    // allocate an address whose stage-2 mapping no longer exists.
                    match self.grant(gpa, len) {
                        Ok(()) => {
                            self.mem.pool_cancel_unshare(pool_id, offset, len);
                            return -libc::EIO;
                        }
                        Err(regrant_err) => {
                            error!(
                                "gunyah-pool: could not restore {:#x}+{:#x} after punch failure: \
                                 {:#}; dropping the grant so the guest can reconcile the range",
                                gpa, len, regrant_err
                            );
                            // The pages were not reclaimed, but the guest must stop using the
                            // range. Finishing the bookkeeping makes QUERY report it unbacked;
                            // the guest shrink path will lower its backed boundary and a later
                            // grow can retry the SHARE. This is safer than leaving a stale grant
                            // that makes the guest reuse an unmapped range.
                            if let Err(finish_err) =
                                self.mem.pool_finish_unshare(pool_id, offset, len)
                            {
                                error!(
                                    "gunyah-pool: failed to drop unreclaimable grant {:#x}+{:#}: {:?}",
                                    gpa, len, finish_err
                                );
                            }
                            return -libc::EIO;
                        }
                    }
                }

                if let Err(e) = self.mem.pool_finish_unshare(pool_id, offset, len) {
                    // The hole was punched and the guest mapping is gone. Do not clear the
                    // reservation on bookkeeping failure: making the range reusable could let a
                    // new resource alias a range whose accounting no longer matches the RM. The
                    // guest must also discard this suffix, because the backing is already gone;
                    // EUCLEAN is the private signal for that unrecoverable state.
                    error!(
                        "gunyah-pool: reclaimed {:#x}+{:#x} but could not finish bookkeeping: {:?}",
                        gpa, len, e
                    );
                    return -libc::EUCLEAN;
                // The guest mapping is still registered, so the reservation is safe to
                // release. Otherwise a transient tube/unregister failure permanently strands
                // this grant in the `releasing` state.
                self.mem.pool_cancel_unshare(pool_id, offset, len);
                -e.downcast_ref::<base::Error>()
                    .map_or(libc::EIO, |errno| errno.errno())
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
        let mut queues = BTreeMap::new();

        // Queue 2 belongs to a separate worker. It must be stopped and returned along with the
        // accept queues; otherwise wake would either leave the old pool worker owning the queue or
        // reactivate with only two queues even though activate requires all three.
        if let Some(pool_worker_thread) = self.pool_worker_thread.take() {
            let pool_worker = pool_worker_thread.stop();
            self.pool_tube = Some(pool_worker.vm_memory);
            queues.insert(2, pool_worker.pool_queue);
        }

        if let Some(worker_thread) = self.worker_thread.take() {
            let worker = worker_thread.stop();
            self.tube = Some(worker.tube);
            queues.insert(0, worker.req_queue);
            queues.insert(1, worker.comp_queue);
        }

        if queues.is_empty() {
            Ok(None)
        } else {
            Ok(Some(queues))
        }
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
