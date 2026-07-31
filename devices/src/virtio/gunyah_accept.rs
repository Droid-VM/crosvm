// SPDX-License-Identifier: GPL-3.0-or-later
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
use base::Protection;
use base::RawDescriptor;
use base::ReadNotifier;
use base::Tube;
use base::WaitContext;
use base::WorkerThread;
use snapshot::AnySnapshot;
use hypervisor::MemCacheType;
use vm_control::VmMemoryDestination;
use vm_control::VmMemoryRegionId;
use vm_control::VmMemoryRequest;
use vm_control::VmMemoryResponse;
use vm_control::VmMemorySource;
use vm_control::GunyahAcceptOp;
use vm_control::GunyahAcceptRequest;
use vm_control::GunyahAcceptResponse;
use vm_memory::GuestAddress;
use vm_memory::GuestMemory;

use super::DeviceType;
use super::Interrupt;
use super::Queue;
use super::VirtioDevice;

const QUEUE_SIZE: u16 = 16;
// Queue 0 requestq (host->guest), 1 completionq (guest->host), 2 poolq (guest->host requests).
const QUEUE_SIZES: &[u16] = &[QUEUE_SIZE, QUEUE_SIZE, QUEUE_SIZE];

// Wire ops, shared with the guest module (virtio_gunyah_accept.c).
const VGA_OP_ACCEPT: u32 = 1;
const VGA_OP_RELEASE: u32 = 2;

// struct virtio_gunyah_accept_req, little-endian.
const REQ_WIRE_LEN: usize = 32;
// struct virtio_gunyah_accept_comp, little-endian.
const COMP_WIRE_LEN: usize = 8;

// Pool ops, guest-initiated, on queue 2.
//
// A third queue rather than a new op on the existing pair, for two reasons that are both in the
// existing wire format: the completion struct is 8 bytes and has nowhere to put an offset and a
// length, and `req_id` is host-assigned, with the host dropping any completion whose id it did not
// issue (see `process_completions`). Direction is therefore carried by the queue rather than by a
// flag -- which is also what makes the validation impossible to forget, since everything arriving
// on this queue is by construction guest-originated and must be range-checked against the pool.
const VGP_OP_SHARE: u32 = 1;
const VGP_OP_UNSHARE: u32 = 2;
const VGP_OP_QUERY: u32 = 3;

// struct virtio_gunyah_pool_req, little-endian.
const POOL_REQ_WIRE_LEN: usize = 32;
// struct virtio_gunyah_pool_resp, little-endian.
const POOL_RESP_WIRE_LEN: usize = 16;

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

/// Serves guest-initiated pool grow/shrink requests on queue 2.
///
/// Deliberately a SEPARATE thread from the accept `Worker`, and this is load-bearing rather than
/// tidiness. A grow is: guest asks here -> this thread asks the vm_memory handler to
/// `runtime_share` -> that handler calls `drive_guest_accept`, which hands an ACCEPT to the accept
/// worker and waits for the guest to complete it. Run the pool queue on the accept worker's thread
/// and that is a cycle: the accept worker would be blocked waiting for the share it is itself
/// required to finish. Two threads, and the accept worker stays free to service the ACCEPT while
/// this one waits.
///
/// The same constraint applies on the guest side: whatever thread submits a pool request must not
/// be the thread that processes the accept requestq.
struct PoolWorker {
    pool_queue: Queue,
    /// To the vm_memory handler, for RegisterMemory. Distinct from the accept worker's tube.
    vm_memory: Tube,
    mem: GuestMemory,
}

impl PoolWorker {
    /// Decode, validate and answer one request. Returns the response wire bytes.
    ///
    /// Validation lives here, on the guest-originated queue, so a request can never reach
    /// `runtime_share` without having been range-checked. Host-initiated shares (VkDeviceMemory
    /// into a PCI BAR) do not come through here and are unaffected.
    fn handle(&mut self, wire: &[u8; POOL_REQ_WIRE_LEN]) -> [u8; POOL_RESP_WIRE_LEN] {
        let req_id = u32::from_le_bytes(wire[0..4].try_into().unwrap());
        let op = u32::from_le_bytes(wire[4..8].try_into().unwrap());
        let pool_id = u32::from_le_bytes(wire[8..12].try_into().unwrap());
        let offset = u64::from_le_bytes(wire[16..24].try_into().unwrap());
        let len = u64::from_le_bytes(wire[24..32].try_into().unwrap());

        let (ret, extra) = match op {
            VGP_OP_SHARE => (self.share(pool_id, offset, len), 0u64),
            VGP_OP_UNSHARE => (self.unshare(pool_id, offset, len), 0u64),
            VGP_OP_QUERY => match self.mem.pool_live_grants(pool_id) {
                Some(n) => (0, n as u64),
                None => (-libc::ENODEV, 0),
            },
            _ => {
                warn!("gunyah-pool: unknown op {}", op);
                (-libc::EINVAL, 0)
            }
        };

        let mut resp = [0u8; POOL_RESP_WIRE_LEN];
        resp[0..4].copy_from_slice(&req_id.to_le_bytes());
        resp[4..8].copy_from_slice(&ret.to_le_bytes());
        resp[8..16].copy_from_slice(&extra.to_le_bytes());
        resp
    }

    /// One RegisterMemory round trip, one step of the pool.
    ///
    /// The backing is a slice of the pool's OWN memfd rather than freshly allocated memory, and
    /// that is the crux of the whole design rather than an optimisation. The region was created at
    /// the full window size -- sparse, so host VA rather than host RAM -- so a granted range is
    /// already resolvable by `find_region`/`shm_region`, which is exactly what `create_udmabuf`
    /// needs to turn guest-supplied addresses back into (memfd, offset) pairs. Share fresh memory
    /// instead and every grant becomes a separate region that the udmabuf path would have to be
    /// taught about.
    fn grant_one(&mut self, gpa: u64, len: u64) -> anyhow::Result<()> {
        let addr = GuestAddress(gpa);
        // One call, because the offset is only meaningful against the object it came from: a
        // GuestMemory may have several backing objects.
        let (shm, shm_offset) = self
            .mem
            .offset_from_base(addr)
            .map_err(|e| anyhow!("pool address has no shm offset: {}", e))?;
        let descriptor = shm.as_raw_descriptor();
        // SAFETY: the descriptor belongs to a GuestMemory region that outlives this device, and
        // clone_descriptor duplicates it rather than taking ownership.
        let descriptor = base::clone_descriptor(&base::Descriptor(descriptor))
            .context("failed to clone the pool memfd")?;

        self.vm_memory
            .send(&VmMemoryRequest::RegisterMemory {
                source: VmMemorySource::Descriptor {
                    descriptor,
                    offset: shm_offset,
                    size: len,
                },
                dest: VmMemoryDestination::GuestPhysicalAddress(gpa),
                prot: Protection::read_write(),
                cache: MemCacheType::CacheCoherent,
                // The guest cannot touch the range until the RM has accepted it, so waiting for
                // the accept is the point: returning early would hand back an address that reads
                // as zeros rather than faulting.
                vm_accept: hypervisor::VmAccept::Sync,
            })
            .context("failed to send RegisterMemory")?;

        match self.vm_memory.recv::<VmMemoryResponse>() {
            Ok(VmMemoryResponse::RegisterMemory { .. }) => Ok(()),
            Ok(other) => Err(anyhow!("RegisterMemory refused: {:?}", other)),
            Err(e) => Err(anyhow!("RegisterMemory response: {}", e)),
        }
    }

    fn share(&mut self, pool_id: u32, offset: u64, len: u64) -> i32 {
        // Check before doing anything: a refused request must leave no trace, so the guest can
        // pick a different range without having to reconcile with the host first.
        if let Err(e) = self.mem.pool_validate_share(pool_id, offset, len) {
            warn!(
                "gunyah-pool: refusing SHARE pool={} offset={:#x} len={:#x}: {:?}",
                pool_id, offset, len, e
            );
            return -e.as_errno();
        }
        let Some(base) = self.mem.pool_base(pool_id) else {
            return -libc::ENODEV;
        };
        let step = match self.mem.pool_step(pool_id) {
            Some(s) if s != 0 => s,
            _ => return -libc::EOPNOTSUPP,
        };

        // One SHARE per step, because the reclaim label is derived per address (gpa >> 12) and a
        // single wide parcel could not be released a step at a time afterwards.
        let mut done: u64 = 0;
        while done < len {
            let gpa = base.offset() + offset + done;
            if let Err(e) = self.grant_one(gpa, step) {
                error!("gunyah-pool: grant at {:#x} failed: {:#}", gpa, e);
                // Roll back what this request managed, so a partial failure leaves the pool in
                // the state the guest already believes it is in. Anything that cannot be rolled
                // back is recorded, not silently dropped: the guest reconciles with QUERY.
                if done > 0 {
                    if let Err(e2) = self.release_range(pool_id, base.offset() + offset, done) {
                        error!(
                            "gunyah-pool: rollback of {:#x}+{:#x} failed ({:#}); \
                             those steps are stranded until the VM exits",
                            base.offset() + offset,
                            done,
                            e2
                        );
                    }
                }
                return -libc::ENOMEM;
            }
            // Record each step as it lands, so a failure part way through has an accurate table
            // to roll back from.
            if let Err(e) = self.mem.pool_mark_granted(pool_id, offset + done, step, &[0]) {
                error!("gunyah-pool: grant bookkeeping failed: {:?}", e);
                return -libc::EIO;
            }
            done += step;
        }
        0
    }

    /// Unregister a range that is already recorded as granted, and forget it.
    fn release_range(&mut self, pool_id: u32, gpa: u64, len: u64) -> anyhow::Result<()> {
        let base = self
            .mem
            .pool_base(pool_id)
            .context("no such pool")?
            .offset();
        let step = self.mem.pool_step(pool_id).unwrap_or(0);
        if step == 0 {
            return Err(anyhow!("pool {} does not grow", pool_id));
        }
        let mut off = 0u64;
        let mut first_err = None;
        while off < len {
            let req = VmMemoryRequest::UnregisterMemory(VmMemoryRegionId::from_guest_addr(GuestAddress(gpa + off)));
            let r = self
                .vm_memory
                .send(&req)
                .map_err(|e| anyhow!("send: {}", e))
                .and_then(|()| match self.vm_memory.recv::<VmMemoryResponse>() {
                    Ok(VmMemoryResponse::Ok) => Ok(()),
                    Ok(other) => Err(anyhow!("refused: {:?}", other)),
                    Err(e) => Err(anyhow!("response: {}", e)),
                });
            // Keep going on error: leaving later steps registered because an earlier one failed
            // would strand strictly more memory.
            if let Err(e) = r {
                if first_err.is_none() {
                    first_err = Some(e);
                }
            } else {
                let _ = self.mem.pool_take_granted(pool_id, gpa + off - base, step);
            }
            off += step;
        }
        match first_err {
            Some(e) => Err(e),
            None => Ok(()),
        }
    }

    fn unshare(&mut self, pool_id: u32, offset: u64, len: u64) -> i32 {
        if let Err(e) = self.mem.pool_validate_unshare(pool_id, offset, len) {
            warn!(
                "gunyah-pool: refusing UNSHARE pool={} offset={:#x} len={:#x}: {:?}",
                pool_id, offset, len, e
            );
            return -e.as_errno();
        }
        let Some(base) = self.mem.pool_base(pool_id) else {
            return -libc::ENODEV;
        };
        //
        // NOT YET CHECKED, and the guest must not be led to think otherwise: whether anything on
        // the HOST still references these pages -- a udmabuf built over them, a GPU mapping, a
        // scanout iovec. Only the host can answer that, because the guest's RESOURCE_UNREF is
        // fire-and-forget: it believes a buffer is gone while the host still holds it. Answering
        // needs the resource tables that live in the GPU device, which this worker cannot reach
        // today.
        //
        // Until it can, shrink is safe only for a range nothing was ever built over, which is
        // what the stage 4 test exercises. A consumer driver must not call it on memory it has
        // handed to the host.
        //
        match self.release_range(pool_id, base.offset() + offset, len) {
            Ok(()) => 0,
            Err(e) => {
                error!("gunyah-pool: unshare failed: {:#}", e);
                -libc::EIO
            }
        }
    }

    fn process_requests(&mut self) {
        while let Some(mut avail_desc) = self.pool_queue.pop() {
            let mut wire = [0u8; POOL_REQ_WIRE_LEN];
            let parsed = avail_desc.reader.read_exact(&mut wire).is_ok();
            let written = if parsed {
                let resp = self.handle(&wire);
                match avail_desc.writer.write_all(&resp) {
                    Ok(()) => avail_desc.writer.bytes_written(),
                    Err(e) => {
                        warn!("gunyah-pool: failed writing response: {}", e);
                        0
                    }
                }
            } else {
                warn!("gunyah-pool: short request from guest");
                0
            };
            self.pool_queue.add_used(avail_desc, written as u32);
        }
        self.pool_queue.trigger_interrupt();
    }

    fn run(&mut self, kill_evt: Event) -> anyhow::Result<()> {
        #[derive(EventToken)]
        enum Token {
            PoolQueueAvailable,
            Kill,
        }

        let wait_ctx = WaitContext::build_with(&[
            (self.pool_queue.event(), Token::PoolQueueAvailable),
            (&kill_evt, Token::Kill),
        ])
        .context("failed creating pool WaitContext")?;

        let mut exiting = false;
        while !exiting {
            for event in wait_ctx.wait()?.iter().filter(|e| e.is_readable) {
                match event.token {
                    Token::PoolQueueAvailable => {
                        self.pool_queue
                            .event()
                            .wait()
                            .context("failed reading pool queue Event")?;
                        self.process_requests();
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
    pool_worker_thread: Option<WorkerThread<PoolWorker>>,
    tube: Option<Tube>,
    /// To the vm_memory handler, used only by the pool worker. Separate from `tube` so the two
    /// round trips cannot serialise behind each other -- see PoolWorker's note on the deadlock.
    pool_tube: Option<Tube>,
    virtio_features: u64,
}

impl GunyahAccept {
    pub fn new(virtio_features: u64, tube: Tube, pool_tube: Tube) -> GunyahAccept {
        GunyahAccept {
            worker_thread: None,
            pool_worker_thread: None,
            tube: Some(tube),
            pool_tube: Some(pool_tube),
            virtio_features,
        }
    }
}

impl VirtioDevice for GunyahAccept {
    fn keep_rds(&self) -> Vec<RawDescriptor> {
        match (&self.tube, &self.pool_tube) {
            (Some(t), Some(p)) => vec![t.as_raw_descriptor(), p.as_raw_descriptor()],
            (Some(t), None) => vec![t.as_raw_descriptor()],
            (None, Some(p)) => vec![p.as_raw_descriptor()],
            (None, None) => Vec::new(),
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
        mem: GuestMemory,
        _interrupt: Interrupt,
        mut queues: BTreeMap<usize, Queue>,
    ) -> anyhow::Result<()> {
        if queues.len() != 3 {
            return Err(anyhow!("expected 3 queues, got {}", queues.len()));
        }

        let req_queue = queues.remove(&0).unwrap();
        let comp_queue = queues.remove(&1).unwrap();
        let pool_queue = queues.remove(&2).unwrap();
        let pool_tube = self
            .pool_tube
            .take()
            .context("gunyah-accept activated without a pool tube")?;
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

        self.pool_worker_thread = Some(WorkerThread::start("v_gunyah_pool", move |kill_evt| {
            let mut worker = PoolWorker {
                pool_queue,
                vm_memory: pool_tube,
                mem,
            };
            if let Err(e) = worker.run(kill_evt) {
                error!("gunyah-pool worker thread failed: {:#}", e);
            }
            worker
        }));

        Ok(())
    }

    fn reset(&mut self) -> anyhow::Result<()> {
        if let Some(t) = self.pool_worker_thread.take() {
            self.pool_tube = Some(t.stop().vm_memory);
        }
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
            return Ok(Some(BTreeMap::from([
                (0, worker.req_queue),
                (1, worker.comp_queue),
            ])));
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
