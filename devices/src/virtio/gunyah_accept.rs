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
// Debug-only, for the growable-pool test driver: take and drop the same host-side reference that
// a dma-buf import takes, so the "cannot release a grant something is using" path can be
// exercised without making a production pool growable. No production caller sends these.
const VGP_OP_TEST_REF: u32 = 100;
const VGP_OP_TEST_UNREF: u32 = 101;

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
            VGP_OP_QUERY => {
                if offset == 0 && len == 0 {
                    match self.mem.pool_live_grants(pool_id) {
                        Some(n) => (0, n as u64),
                        None => (-libc::ENODEV, 0),
                    }
                } else {
                    match self.mem.pool_range_backed(pool_id, offset, len) {
                        Some(backed) => (0, backed as u64),
                        None => (-libc::ENODEV, 0),
                    }
                }
            },
            VGP_OP_TEST_REF | VGP_OP_TEST_UNREF => (self.test_ref(op, pool_id, offset, len), 0),
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

    /// Fold the range into 2 MiB folios before it is shared.
    ///
    /// Without this the pages are whatever the fault handler produces, which is 4 KiB, and a
    /// 32 MiB grant becomes a parcel with 8192 mem_entries. That fails as an order-5 kcalloc in
    /// the host share module once memory is fragmented -- a failure that looks like "grow stopped
    /// working after a few hours of uptime" rather than like a missing preparation step.
    ///
    /// Deliberately NOT the existing PrepareBlobBacking: `folio_back_fd` ftruncates and collapses
    /// the WHOLE file, and a pool's file is the whole declared window, so using it here would
    /// populate everything the guest has not asked for -- exactly the memory this design exists
    /// to leave unallocated.
    fn prepare_folios(&mut self, gpa: u64, len: u64) -> anyhow::Result<()> {
        let (shm, shm_offset) = self
            .mem
            .offset_from_base(GuestAddress(gpa))
            .map_err(|e| anyhow!("pool address has no shm offset: {}", e))?;
        self.vm_memory
            .send(&VmMemoryRequest::PrepareBlobRange {
                descriptor: base::clone_descriptor(&base::Descriptor(shm.as_raw_descriptor()))
                    .context("failed to clone the pool memfd")?,
                offset: shm_offset,
                size: len,
            })
            .context("failed to send PrepareBlobRange")?;
        match self.vm_memory.recv::<VmMemoryResponse>() {
            Ok(VmMemoryResponse::Ok) => Ok(()),
            Ok(other) => Err(anyhow!("PrepareBlobRange refused: {:?}", other)),
            Err(e) => Err(anyhow!("PrepareBlobRange response: {}", e)),
        }
    }

    /// Share one grant: prepare the folios, hand the range to the hypervisor, wait for the guest
    /// to accept it.
    ///
    /// ONE parcel for the whole request, however many steps it spans. A grant is one memparcel
    /// regardless of length, and MAX_MEMPARCEL_PER_VM is 1024 for the entire VM -- shared with
    /// Android's own, and not returned by anything short of a reboot for what a killed VMM leaves
    /// behind. Splitting a 192 MiB request into six parcels would spend six times the quota for
    /// nothing. The cost is that the RM gives a parcel back whole, so this grant is also the unit
    /// of release; a caller that wants finer release asks for less at a time.
    ///
    /// The backing is a slice of the pool's OWN memfd rather than fresh memory, which is the crux
    /// of the design rather than an optimisation: the region was created at the full window size
    /// -- sparse, so host VA and not host RAM -- so a granted range is already resolvable by
    /// find_region/shm_region, which is what create_udmabuf needs to turn guest-supplied addresses
    /// back into (memfd, offset) pairs.
    fn grant(&mut self, gpa: u64, len: u64) -> anyhow::Result<()> {
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

        let (shm, shm_offset) = self
            .mem
            .offset_from_base(GuestAddress(gpa))
            .map_err(|e| anyhow!("pool address has no shm offset: {}", e))?;
        let descriptor = base::clone_descriptor(&base::Descriptor(shm.as_raw_descriptor()))
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
                // Waiting for the accept IS the point: returning before the RM has accepted would
                // hand the guest an address that reads as zeros instead of faulting.
                vm_accept: hypervisor::VmAccept::Sync,
            })
            .context("failed to send RegisterMemory")?;

        match self.vm_memory.recv::<VmMemoryResponse>() {
            Ok(VmMemoryResponse::RegisterMemory { .. }) => Ok(()),
            Ok(VmMemoryResponse::Err(e)) => Err(anyhow::Error::new(e)),
            Ok(other) => Err(anyhow!("RegisterMemory refused: {:?}", other)),
            Err(e) => Err(anyhow!("RegisterMemory response: {}", e)),
        }
    }

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
        let gpa = base.offset() + offset;

        if let Err(e) = self.grant(gpa, len) {
            error!("gunyah-pool: grant at {:#x}+{:#x} failed: {:#}", gpa, len, e);
            // RegisterMemory may have reached the VM handler before its response was lost. Do
            // not punch this range here: without an explicit "mapping was never established"
            // result, reclaiming it could race a live guest/host mapping. The prepared pages are
            // deliberately retained as the conservative recovery state.
            return -e
                .downcast_ref::<base::Error>()
                .map_or(libc::ENOMEM, |errno| errno.errno());
        }
        if let Err(e) = self.mem.pool_mark_granted(pool_id, offset, len, 0) {
            // The share succeeded but the table does not know: the guest would be told no while
            // holding memory it can use, and the range would never be released. Say so loudly.
            error!(
                "gunyah-pool: grant at {:#x} succeeded but bookkeeping failed ({:?}); \
                 that range is stranded until the VM exits",
                gpa, e
            );
            return -libc::EIO;
        }
        0
    }

    fn unshare(&mut self, pool_id: u32, offset: u64, len: u64) -> i32 {
        // Must name a grant exactly -- see PoolGrants: the RM reclaims a parcel whole.
        // Reserve before unregistering. The reservation makes the reference check and the
        // unregister effectively one operation from the resource-create path's perspective:
        // another thread cannot add a dma-buf reference while this request is in flight.
        if let Err(e) = self.mem.pool_begin_unshare(pool_id, offset, len) {
            warn!(
                "gunyah-pool: refusing UNSHARE pool={} offset={:#x} len={:#x}: {:?}",
                pool_id, offset, len, e
            );
            return -e.as_errno();
        }
        let Some(base) = self.mem.pool_base(pool_id) else {
            self.mem.pool_cancel_unshare(pool_id, offset, len);
            return -libc::ENODEV;
        };
        let gpa = base.offset() + offset;
        let req = VmMemoryRequest::UnregisterMemory(VmMemoryRegionId::from_guest_addr(
            GuestAddress(gpa),
        ));
        let r = self
            .vm_memory
            .send(&req)
            .map_err(|e| anyhow!("send: {}", e))
            .and_then(|()| match self.vm_memory.recv::<VmMemoryResponse>() {
                Ok(VmMemoryResponse::Ok) => Ok(()),
                Ok(VmMemoryResponse::Err(e)) => Err(anyhow::Error::new(e)),
                Ok(other) => Err(anyhow!("refused: {:?}", other)),
                Err(e) => Err(anyhow!("response: {}", e)),
            });
        match r {
            Ok(()) => {
                // Give the pages back for real.
                //
                // Unregistering only takes the range away from the GUEST; the pages are still in
                // the memfd's page cache, so without this the host has released nothing and the
                // reserve pool never sees them again -- a shrink that shrinks nothing. Punching
                // the hole is what returns them to the allocator, where the reserve module's
                // order-9 free hook picks them back up (its `served` falls and `avail` rises,
                // which is how a test can tell this actually happened).
                //
                // Must be after the unregister: while the guest still has the range accepted the
                // pages are pinned, and the punch would silently do nothing.
                //
                // Seals do not block it -- the guest memfd carries SHRINK|GROW|SEAL, and
                // shmem_fallocate only refuses PUNCH_HOLE for F_SEAL_WRITE / F_SEAL_FUTURE_WRITE.
                if let Err(e) = self.punch_range(gpa, len) {
                    error!(
                        "gunyah-pool: unshared {:#x}+{:#x} but could not punch it out ({:#}); \
                         trying to restore the mapping",
                        gpa, len, e
                    );

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
                }
                0
            }
            Err(e) => {
                // The guest mapping is still registered, so the reservation is safe to
                // release. Otherwise a transient tube/unregister failure permanently strands
                // this grant in the `releasing` state.
                self.mem.pool_cancel_unshare(pool_id, offset, len);
                error!("gunyah-pool: unshare at {:#x} failed: {:#}", gpa, e);
                -e.downcast_ref::<base::Error>()
                    .map_or(libc::EIO, |errno| errno.errno())
            }
        }
    }

    /// Return a released range's pages to the allocator.
    fn punch_range(&mut self, gpa: u64, len: u64) -> anyhow::Result<()> {
        let (shm, shm_offset) = self
            .mem
            .offset_from_base(GuestAddress(gpa))
            .map_err(|e| anyhow!("pool address has no shm offset: {}", e))?;
        // SAFETY: the descriptor belongs to a live GuestMemory region and the range was just
        // validated as a grant of this pool, so it lies inside that region's slice of the file.
        let rc = unsafe {
            libc::fallocate(
                shm.as_raw_descriptor(),
                libc::FALLOC_FL_PUNCH_HOLE | libc::FALLOC_FL_KEEP_SIZE,
                shm_offset as libc::off_t,
                len as libc::off_t,
            )
        };
        if rc != 0 {
            return Err(anyhow!(
                "fallocate(PUNCH_HOLE) at {:#x}+{:#x}: {}",
                shm_offset,
                len,
                std::io::Error::last_os_error()
            ));
        }
        Ok(())
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
