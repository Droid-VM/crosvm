// Copyright 2026 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! The VM-wide `media_host` buffer pool, the per-device leases on it, and the tube protocol
//! through which the VMM serves the pool to its media helper processes.
//!
//! Kept apart from the rest of `media.rs` because it is the one piece of the media device that
//! depends on nothing but `base`, `resources`, `vm_memory` and the virtio-media crate's allocator
//! trait -- which is what makes it testable on its own, and the lease sweep below is a path no
//! guest can drive (a `rmmod` cannot drop a device that still has open sessions, so the sweep only
//! runs on VM teardown, a hot-remove, or a helper dying; see `logs/vpu_wp/B2-acceptance.md`
//! finding 2).
//!
//! **Why the VMM is the only allocator.** A pool offset is what a device answers
//! `VIDIOC_QUERYBUF` with and the guest adds the `media_host` node's base to it, so every
//! allocator over the window must be the same allocator or offsets collide. The in-VMM devices
//! share [`MediaPool`] directly. A media *helper* is a separate process and cannot; it used to
//! rebuild a private allocator over the same window, which is exactly D49's band of noise -- the
//! encoder's coded-frame writes landing inside the camera's raw buffers
//! (`logs/vpu_wp/F12-encoder.md`) -- and the static slice carve that first fixed D49 traded the
//! aliasing for per-device ENOMEM walls (a 4K decode's CAPTURE set does not fit a 128 MiB
//! decoder slice). So the helper does not allocate at all: it holds one end of a [`base::Tube`]
//! (`--pool-fd`), asks the VMM to [`PoolRequest::Reserve`] and [`PoolRequest::Release`] offsets,
//! and builds its buffers from its own whole-pool mapping ([`MappedPool`]) at the offsets the
//! VMM's one allocator hands back. The VMM answers from a thread per helper
//! ([`MediaPool::spawn_server`]) that owns that helper's lease; the helper dying is an EOF on
//! the tube, and the lease's drop sweeps everything it still held back into the pool.

use std::collections::BTreeMap;
use std::fs::File;
use std::os::fd::OwnedFd;
use std::ptr::NonNull;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;

use anyhow::Context;
use base::error;
use base::info;
use base::AsRawDescriptor;
use base::MappedRegion;
use base::MemoryMapping;
use base::MemoryMappingBuilder;
use base::SafeDescriptor;
use base::Tube;
use base::TubeError;
use resources::address_allocator::AddressAllocator;
use resources::AddressRange;
use resources::Alloc;
use serde::Deserialize;
use serde::Serialize;
use sync::Mutex;
use virtio_media::HostBuffer;
use virtio_media::VirtioMediaBufferAllocator;
use vm_memory::GuestMemory;
use vm_memory::MediaPoolHandle;

/// Rebuild the `media_host` pool handle from a memory table that has lost its purposes: the
/// region starting at `gpa` (`VPU_DESIGN.md` §6.2, the `--pool-gpa` handoff).
///
/// A vhost-user helper's `GuestMemory` is what `SET_MEM_TABLE` describes -- every region of the
/// VMM's, memfd and offset included, but no `MemoryRegionPurpose`, so
/// `MediaPoolHandle::from_guest_memory` finds nothing there. The pool's guest-physical base is
/// enough to find it again, and everything else the handle needs is read off that region, so the
/// helper cannot disagree with the VMM about which bytes the pool is. `host_va` is the helper's
/// own mapping of the region; the handle's consumer maps the pool once more for itself
/// ([`MappedPool::new`]), which is what `MediaPoolHandle` says an out-of-process consumer must do.
pub fn pool_handle_at(mem: &GuestMemory, gpa: u64) -> anyhow::Result<MediaPoolHandle> {
    let region = mem
        .regions()
        .find(|region| region.guest_addr.offset() == gpa)
        .ok_or_else(|| {
            let starts: Vec<String> = mem
                .regions()
                .map(|r| format!("{:#x}+{:#x}", r.guest_addr.offset(), r.size))
                .collect();
            anyhow::anyhow!(
                "no memory region starts at the media_host pool base {:#x}; the memory table \
                 has [{}]",
                gpa,
                starts.join(", ")
            )
        })?;
    let fd = SafeDescriptor::try_from(region.shm as &dyn AsRawDescriptor)
        .with_context(|| format!("cannot dup the backing object of the region at {:#x}", gpa))?;
    Ok(MediaPoolHandle {
        fd,
        fd_offset: region.shm_offset,
        host_va: region.host_addr as u64,
        gpa,
        size: region.size as u64,
    })
}

/// What a helper asks the VMM's pool server ([`MediaPool::spawn_server`]). One request at a time,
/// synchronous, no ids: the helper's allocator blocks on the answer, and it is the only writer on
/// its tube.
#[derive(Debug, Serialize, Deserialize)]
pub enum PoolRequest {
    /// Carve `len` bytes (page-rounded by the VMM) out of the pool for this helper.
    Reserve { len: u64 },
    /// Give the reservation at `offset` back. An offset this helper does not own is logged by
    /// the VMM and ignored -- the space stays allocated to whoever owns it.
    Release { offset: u64 },
}

/// The VMM's answer to one [`PoolRequest`].
#[derive(Debug, Serialize, Deserialize)]
pub enum PoolResponse {
    /// `Reserve` succeeded: the buffer's offset in the pool's offset space.
    Reserved { offset: u64 },
    /// `Release` was processed (whether or not the offset was this helper's to free; a foreign
    /// offset is the VMM's error line, not the helper's problem to handle).
    Released,
    /// `Reserve` failed with this errno -- `ENOMEM` when the pool is full, `EINVAL` for a
    /// zero or absurd length.
    Errno(i32),
}

/// How long a helper waits for the VMM to answer one pool request. The VMM's server thread does
/// nothing but bookkeeping between recv and send, so the answer is normally immediate; a wait
/// this long means the VMM is gone or wedged, and the helper's allocation fails with `EIO`
/// rather than blocking a REQBUFS forever (the F5 rule: every wait bounded, by a named const).
pub const POOL_RPC_TIMEOUT: Duration = Duration::from_secs(5);

/// One process's own whole-pool mapping: the mapping half of the old allocator, without the
/// offset-space bookkeeping (which lives in the VMM's [`MediaPoolAllocator`] alone).
///
/// Both allocator shapes build their [`HostBuffer`]s through [`MappedPool::buffer_at`]: the
/// VMM's, at offsets its own allocator handed out, and a helper's [`RemotePoolAllocator`], at
/// offsets the VMM answered over the tube.
pub struct MappedPool {
    pool: MediaPoolHandle,
    /// Our own mapping of the whole pool, so buffers can be filled from this process.
    host_map: MemoryMapping,
}

impl MappedPool {
    /// Dup the pool descriptor and map the whole pool once for this process.
    pub fn new(pool: MediaPoolHandle) -> anyhow::Result<Self> {
        let file = File::from(
            pool.fd
                .try_clone()
                .context("cannot dup the media_host pool descriptor")?,
        );
        let host_map = MemoryMappingBuilder::new(pool.size as usize)
            .from_file(&file)
            .offset(pool.fd_offset)
            .build()
            .context("cannot map the media_host pool")?;
        Ok(MappedPool { pool, host_map })
    }

    fn size(&self) -> u64 {
        self.pool.size
    }

    /// Build the [`HostBuffer`] for the `len` bytes at `offset`: a dup of the pool descriptor at
    /// `fd_offset + offset`, a pointer into this process's mapping, and the pool offset the
    /// guest adds the `media_host` base to.
    ///
    /// The caller owns the offset: for the VMM that means its allocator reserved it, for a
    /// helper that the VMM answered `Reserved { offset }`. An offset that is not page-aligned
    /// and inside the pool is refused with `EIO` -- for a helper that means the two sides are
    /// out of step, and nothing this buffer could safely alias is known.
    ///
    /// The mapping must outlive the buffer; see the SAFETY comments.
    fn buffer_at(&self, offset: u64, len: u64) -> Result<HostBuffer, i32> {
        let page = base::pagesize() as u64;
        if len == 0
            || offset % page != 0
            || offset
                .checked_add(len)
                .map_or(true, |end| end > self.size())
        {
            error!(
                "virtio-media: pool offset {:#x}+{:#x} is not a page-aligned part of the \
                 {:#x}-byte media_host pool",
                offset,
                len,
                self.size()
            );
            return Err(libc::EIO);
        }
        let fd: OwnedFd = match self.pool.fd.try_clone() {
            Ok(fd) => fd.into(),
            Err(e) => {
                error!("virtio-media: cannot dup the pool descriptor: {}", e);
                return Err(libc::EIO);
            }
        };
        // SAFETY: `offset + len <= pool.size` (checked above), and `host_map` maps all
        // `pool.size` bytes for as long as this `MappedPool` exists. Every `MappedPool` outlives
        // the buffers built from it: the VMM's belongs to the VM-wide allocator, which lives for
        // the VM; a helper's is held (in an `Arc`) by the backend for the life of the process,
        // while its buffers live inside devices whose worker threads are joined before the
        // backend is dropped.
        let ptr = unsafe { self.host_map.as_ptr().add(offset as usize) };
        let ptr = match NonNull::new(ptr) {
            Some(ptr) => ptr,
            None => {
                error!("virtio-media: the pool mapping has no address for offset {offset:#x}");
                return Err(libc::EIO);
            }
        };
        // SAFETY: `ptr` maps the `len` bytes at `pool.fd_offset + offset` of `fd`, and stays
        // valid for the life of this `MappedPool` (see above).
        Ok(unsafe {
            HostBuffer::from_raw_parts(fd, self.pool.fd_offset + offset, len, ptr, Some(offset))
        })
    }
}

/// The VM-wide `media_host` pool: one offset space, shared by every media device.
///
/// It has to be VM-wide, and it has to be the only allocator over the window. A pool offset is
/// what the device answers `VIDIOC_QUERYBUF` with and the guest adds the `media_host` node's
/// base to it, so two allocators over the same window both hand out offset 0, 0x1000, ... and
/// the guest maps the same physical bytes for two unrelated buffers -- while one device's
/// `release` frees space the other is still filling (D49; `VPU_DESIGN.md` §2.2 sizes the one
/// 256 MiB pool for every media device in the VM). Helpers therefore do not get allocators of
/// their own: each is served by a thread of the VMM that holds this allocator's lock only for
/// the length of one reservation ([`MediaPool::spawn_server`]).
///
/// The descriptor is dup'ed and the whole pool mapped exactly once per process; a buffer's own
/// dup is what the crate's `HostBuffer` carries.
pub struct MediaPoolAllocator {
    mapped: MappedPool,
    /// The pool's offset space, `[0, pool.size)`, page aligned.
    allocator: AddressAllocator,
    /// Key for the next allocation; `AddressAllocator` wants every live allocation to have a
    /// distinct one.
    next_id: usize,
    /// Id for the next lease.
    next_owner: u64,
    /// `pool offset -> (owner, size)` for every reservation handed out. The owner is what lets
    /// a device that goes away -- a virtio reset drops its worker and every buffer with it, a
    /// helper that dies drops its server thread's lease -- give its space back to a pool the
    /// rest of the VM keeps using, and what refuses a release of somebody else's offset.
    live: BTreeMap<u64, (u64, u64)>,
    /// Bytes currently handed out, for the exhaustion log line.
    used: u64,
}

impl MediaPoolAllocator {
    fn new(pool: MediaPoolHandle) -> anyhow::Result<Self> {
        let mapped = MappedPool::new(pool)?;
        let page = base::pagesize() as u64;
        if mapped.size() == 0 || mapped.size() % page != 0 {
            anyhow::bail!(
                "the media_host pool's size {:#x} is not a whole number of pages",
                mapped.size()
            );
        }
        let allocator = AddressAllocator::new(
            AddressRange::from_start_and_end(0, mapped.size() - 1),
            Some(page),
            None,
        )?;
        Ok(MediaPoolAllocator {
            mapped,
            allocator,
            next_id: 0,
            next_owner: 0,
            live: BTreeMap::new(),
            used: 0,
        })
    }

    /// Reserve `len` bytes (page-rounded) of the offset space for `owner`. `card` only names the
    /// device in the exhaustion log -- with one pool for the whole VM, "the pool is full" is
    /// useless without knowing who asked.
    ///
    /// A `len` of zero, or one no pool could ever satisfy, is `EINVAL` without touching the
    /// allocator; a full pool is `ENOMEM`, logged.
    fn reserve(&mut self, len: u64, owner: u64, card: &str) -> Result<u64, i32> {
        if len == 0 {
            return Err(libc::EINVAL);
        }
        let page = base::pagesize() as u64;
        let size = len.checked_next_multiple_of(page).ok_or(libc::EINVAL)?;
        if size > self.mapped.size() {
            error!(
                "virtio-media: \"{}\" asked the media_host pool for {} bytes, more than the \
                 whole {}-byte pool",
                card,
                size,
                self.mapped.size()
            );
            return Err(libc::EINVAL);
        }
        let id = self.next_id;
        self.next_id += 1;
        let (used, pool_size) = (self.used, self.mapped.size());
        // Every allocation gets its own key; the offsets are what matter, the key is only a name.
        let offset = self
            .allocator
            .allocate(size, Alloc::Anon(id), "media buffer".into())
            .map_err(|e| {
                // The one place exhaustion surfaces: REQBUFS/CREATE_BUFS get ENOMEM, and the log
                // says which device asked and how full the pool was (VPU_DESIGN.md §4.1).
                error!(
                    "virtio-media: media_host pool exhausted for \"{}\": {} bytes requested with \
                     {} of {} in use ({:?})",
                    card, size, used, pool_size, e
                );
                libc::ENOMEM
            })?;
        self.used += size;
        self.live.insert(offset, (owner, size));
        Ok(offset)
    }

    /// Give one reservation back -- if it is `owner`'s to give. An offset owned by another
    /// device, or by nobody, is refused and left exactly as it was: with one offset space for
    /// the whole VM, honouring a stray release would hand one device's bytes to the next asker
    /// (the write-side of D49, on the free path).
    fn unreserve(&mut self, offset: u64, owner: u64) -> Result<(), i32> {
        match self.live.get(&offset) {
            Some((o, _)) if *o == owner => {
                self.free(offset);
                Ok(())
            }
            Some(_) => {
                error!(
                    "virtio-media: refusing to release pool offset {:#x}: it belongs to another \
                     device",
                    offset
                );
                Err(libc::EINVAL)
            }
            None => {
                error!(
                    "virtio-media: refusing to release pool offset {:#x}: nothing is allocated \
                     there",
                    offset
                );
                Err(libc::EINVAL)
            }
        }
    }

    /// Carve `len` bytes out of the pool for `owner` and build the buffer: [`Self::reserve`]
    /// plus [`MappedPool::buffer_at`]. The in-VMM path; a helper does the same two steps with
    /// the reserve on the far side of its tube.
    fn allocate(&mut self, len: u64, owner: u64, card: &str) -> Result<HostBuffer, i32> {
        let offset = self.reserve(len, owner, card)?;
        match self.mapped.buffer_at(offset, len) {
            Ok(buffer) => Ok(buffer),
            Err(errno) => {
                let _ = self.unreserve(offset, owner);
                Err(errno)
            }
        }
    }

    /// Drop one offset from the books. The caller has already checked the owner
    /// ([`Self::unreserve`]) or filtered by it ([`Self::release_owner`]).
    fn free(&mut self, offset: u64) {
        match self.live.remove(&offset) {
            Some((_, size)) => {
                self.used = self.used.saturating_sub(size);
                if let Err(e) = self.allocator.release_containing(offset) {
                    error!(
                        "virtio-media: releasing a buffer at pool offset {:#x} that is not \
                         allocated: {}",
                        offset, e
                    );
                }
            }
            None => error!(
                "virtio-media: releasing a buffer at pool offset {:#x} that is not allocated",
                offset
            ),
        }
    }

    fn release(&mut self, buf: HostBuffer, owner: u64) {
        if let Some(offset) = buf.pool_offset {
            let _ = self.unreserve(offset, owner);
        }
        // Dropping closes the descriptor dup; the pool mapping is ours, not the buffer's.
        drop(buf);
    }

    /// Take back everything a lease still holds. Its buffers are gone with the device (dropping
    /// a `HostBuffer` frees nothing in the pool by design), so only the offset space is at stake.
    ///
    /// Returns how many reservations were reclaimed -- the same number the log line carries, and
    /// the only way a test can tell the sweep apart from "there was nothing to sweep".
    fn release_owner(&mut self, owner: u64) -> usize {
        let offsets: Vec<u64> = self
            .live
            .iter()
            .filter(|(_, (o, _))| *o == owner)
            .map(|(offset, _)| *offset)
            .collect();
        if !offsets.is_empty() {
            info!(
                "virtio-media: reclaiming {} media_host buffers from a device that went away",
                offsets.len()
            );
        }
        let reclaimed = offsets.len();
        for offset in offsets {
            self.free(offset);
        }
        reclaimed
    }
}

/// Handle on the VM-wide [`MediaPoolAllocator`], created once per VM and cloned into every media
/// device.
#[derive(Clone)]
pub struct MediaPool {
    inner: Arc<Mutex<MediaPoolAllocator>>,
    /// The pool descriptor, for `keep_rds`. The allocator owns the only dup this side keeps.
    fd: base::RawDescriptor,
    /// The guest-physical window, `(base, size)`: what a helper is told so it can find the pool
    /// again in its own memory table.
    guest_range: (u64, u64),
}

impl MediaPool {
    /// Map the pool and build its allocator. One dup of the descriptor and one host mapping, for
    /// the whole VM.
    pub fn new(pool: MediaPoolHandle) -> anyhow::Result<Self> {
        let fd = pool.fd.as_raw_descriptor();
        let (gpa, size) = (pool.gpa, pool.size);
        let inner = MediaPoolAllocator::new(pool)?;
        info!(
            "virtio-media: serving MMAP buffers from the media_host pool (gpa {:#x}, {} MiB)",
            gpa,
            size >> 20
        );
        Ok(MediaPool {
            inner: Arc::new(Mutex::new(inner)),
            fd,
            guest_range: (gpa, size),
        })
    }

    pub fn as_raw_descriptor(&self) -> base::RawDescriptor {
        self.fd
    }

    /// The pool's guest-physical `(base, size)`, as the `media_host` device-tree node has it.
    pub fn guest_range(&self) -> (u64, u64) {
        self.guest_range
    }

    /// One device's lease on the pool. What the lease still holds when it is dropped goes back
    /// to the pool, so a device that is reset does not eat the VM's pool for good.
    pub(crate) fn lease(&self, card: String) -> PoolBufferAllocator {
        let owner = {
            let mut pool = self.inner.lock();
            pool.next_owner += 1;
            pool.next_owner
        };
        PoolBufferAllocator {
            pool: Arc::clone(&self.inner),
            card,
            owner,
        }
    }

    /// Serve the pool to one helper over `tube`, on a named thread of its own
    /// (`media pool <card>`).
    ///
    /// The thread owns the VMM's end of the helper's `--pool-fd` tube and one lease on the pool
    /// (owner = this helper), and answers one request at a time: `Reserve` is the lease's
    /// [`PoolBufferAllocator::reserve`], `Release` its [`PoolBufferAllocator::unreserve`]. It
    /// blocks on nothing but its own `recv` -- the lease takes the VM-wide pool lock only inside
    /// its own calls, never across a tube operation -- and its `send` is bounded by
    /// [`POOL_RPC_TIMEOUT`] (the one way it could block is a helper that never reads).
    ///
    /// The sweep -- the lease dropped, `release_owner`'s `reclaiming N media_host buffers from
    /// a device that went away` -- happens on EOF and **only** on EOF: EOF is the one event that
    /// proves the helper's process is gone and cannot write into its buffers any more. Every
    /// other tube failure (a malformed request, a transient recv errno, a send that timed out)
    /// keeps the lease alive: the thread logs, keeps serving while the errors look transient,
    /// and if the connection has to be abandoned it parks on a blocking recv until the EOF
    /// really arrives ([`park_until_eof`]). Sweeping on anything less re-opens D49 -- the
    /// still-running helper keeps writing into offsets the pool has already handed to a
    /// neighbour (review-m8 R8-1).
    pub fn spawn_server(
        &self,
        card: &str,
        tube: Tube,
    ) -> anyhow::Result<std::thread::JoinHandle<()>> {
        let mut lease = self.lease(card.to_string());
        let name = card.to_string();
        if let Err(e) = tube.set_send_timeout(Some(POOL_RPC_TIMEOUT)) {
            error!("virtio-media: cannot bound the pool server's send: {e}");
        }
        std::thread::Builder::new()
            .name(format!("media pool {card}"))
            .spawn(move || {
                serve_pool(&tube, &mut lease, &name);
                // `lease` drops here -- and `serve_pool` returns only on EOF, so the helper
                // process is gone and whatever it still held goes back to the pool.
            })
            .with_context(|| format!("cannot start the media pool server thread for \"{card}\""))
    }
}

/// How many consecutive unreadable requests the pool server tolerates before it stops serving
/// the connection and [`park_until_eof`]s. One bad packet is survived (the helper's request
/// times out and the client resyncs); an errno that repeats this often is not transient.
const POOL_SERVER_MAX_CONSECUTIVE_ERRORS: u32 = 8;

/// The pool server's loop: see [`MediaPool::spawn_server`].
///
/// Returns only when the tube has reached EOF, because returning is the sweep (the caller drops
/// the lease) and only EOF proves the helper cannot write into its buffers any more (R8-1).
fn serve_pool(tube: &Tube, lease: &mut PoolBufferAllocator, card: &str) {
    let mut errors_in_a_row = 0u32;
    loop {
        let request: PoolRequest = match tube.recv() {
            Ok(request) => {
                errors_in_a_row = 0;
                request
            }
            Err(TubeError::Disconnected) => {
                // The helper is gone -- it exited, or died. Dropping the lease (our caller's
                // job) is the sweep; the reclaim line, when there is anything to reclaim, is the
                // pool's.
                info!("virtio-media: the pool connection for \"{card}\" is closed");
                return;
            }
            Err(e) => {
                // A malformed request, or a transient read errno. The helper is still alive --
                // this is NOT EOF -- so its buffers stay reserved; it sees a timeout on the
                // answer that never comes and resyncs. Log the first of a run, keep serving.
                errors_in_a_row += 1;
                if errors_in_a_row == 1 {
                    error!(
                        "virtio-media: cannot read \"{card}\"'s pool request (the lease is \
                         kept; only EOF sweeps): {e}"
                    );
                }
                if errors_in_a_row >= POOL_SERVER_MAX_CONSECUTIVE_ERRORS {
                    error!(
                        "virtio-media: giving up on serving \"{card}\" after {errors_in_a_row} \
                         consecutive errors; holding its lease until EOF"
                    );
                    park_until_eof(tube, card);
                    return;
                }
                std::thread::sleep(Duration::from_millis(10));
                continue;
            }
        };
        let response = match request {
            PoolRequest::Reserve { len } => match lease.reserve(len) {
                Ok(offset) => PoolResponse::Reserved { offset },
                Err(errno) => PoolResponse::Errno(errno),
            },
            PoolRequest::Release { offset } => {
                // A foreign or unallocated offset was logged (and refused) by the pool; the
                // helper gets its ack either way, because there is nothing it could do
                // differently and the reservation books are already right.
                let _ = lease.unreserve(offset);
                PoolResponse::Released
            }
        };
        if let Err(e) = tube.send(&response) {
            // The helper is not reading its answers. Serving it is pointless, but it is still
            // alive and still writing into its buffers, so the lease must survive until EOF.
            error!(
                "virtio-media: cannot answer \"{card}\"'s pool request (nobody is reading; \
                 holding its lease until EOF): {e}"
            );
            park_until_eof(tube, card);
            return;
        }
    }
}

/// Block on the tube until it reaches EOF, discarding everything else. The connection is being
/// abandoned, but the helper on the far end is still a live process with live buffers: only its
/// EOF -- process exit -- makes dropping the lease (the caller's next step) sound. Requests read
/// here are deliberately not answered; the helper sees timeouts, `EIO`s its guest, and its
/// reservations stay on the books until the process really goes.
fn park_until_eof(tube: &Tube, card: &str) {
    loop {
        match tube.recv::<PoolRequest>() {
            Ok(_) => {}
            Err(TubeError::Disconnected) => {
                info!("virtio-media: the pool connection for \"{card}\" is closed");
                return;
            }
            // Still not EOF; don't spin on a repeating errno.
            Err(_) => std::thread::sleep(Duration::from_millis(100)),
        }
    }
}

/// One device's lease on the VM-wide pool: the in-VMM allocator shape, and what a pool server
/// thread holds for its helper.
pub struct PoolBufferAllocator {
    pool: Arc<Mutex<MediaPoolAllocator>>,
    /// The device's V4L2 card name, for the exhaustion log.
    card: String,
    owner: u64,
}

impl PoolBufferAllocator {
    /// Reserve `len` bytes (page-rounded) for this lease: the offset, without a buffer.
    pub(crate) fn reserve(&mut self, len: u64) -> Result<u64, i32> {
        let owner = self.owner;
        self.pool.lock().reserve(len, owner, &self.card)
    }

    /// Give the reservation at `offset` back, if it is this lease's to give; a foreign or
    /// unallocated offset is refused (`EINVAL`, logged) and nothing is freed.
    pub(crate) fn unreserve(&mut self, offset: u64) -> Result<(), i32> {
        let owner = self.owner;
        self.pool.lock().unreserve(offset, owner)
    }
}

impl VirtioMediaBufferAllocator for PoolBufferAllocator {
    fn allocate(&mut self, len: u64) -> Result<HostBuffer, i32> {
        let owner = self.owner;
        self.pool.lock().allocate(len, owner, &self.card)
    }

    fn release(&mut self, buf: HostBuffer) {
        let owner = self.owner;
        self.pool.lock().release(buf, owner)
    }
}

impl Drop for PoolBufferAllocator {
    fn drop(&mut self) {
        let _ = self.pool.lock().release_owner(self.owner);
    }
}

/// A helper's connection to the VMM's pool server: the tube, this process's whole-pool mapping,
/// and one flag that poisons the connection once it has missed an answer.
///
/// Shared (behind an `Arc`) between the backend -- which keeps it across device resets, the way
/// the in-VMM pool outlives its leases -- and the [`RemotePoolAllocator`] of the device
/// currently running.
pub struct RemotePool {
    tube: Tube,
    mapped: MappedPool,
    /// Set when a request timed out or the tube broke. The protocol is synchronous with no ids,
    /// so after a missed answer the two sides are out of step -- a late `Reserved` would be read
    /// as the answer to the *next* request -- and the only safe thing left is to fail
    /// everything. The VMM sweeps this helper's reservations when the tube closes, i.e. when
    /// this process exits.
    dead: AtomicBool,
}

impl RemotePool {
    /// Wrap the helper's `--pool-fd` tube and map the pool this process found at `pool_gpa`
    /// ([`pool_handle_at`]). Bounds every send and recv on the tube by [`POOL_RPC_TIMEOUT`].
    pub fn new(tube: Tube, pool: MediaPoolHandle) -> anyhow::Result<Arc<Self>> {
        Self::with_timeout(tube, pool, POOL_RPC_TIMEOUT)
    }

    /// [`RemotePool::new`] with the timeout injectable, for tests that want a dead VMM to be
    /// noticed in milliseconds rather than seconds.
    pub fn with_timeout(
        tube: Tube,
        pool: MediaPoolHandle,
        timeout: Duration,
    ) -> anyhow::Result<Arc<Self>> {
        tube.set_recv_timeout(Some(timeout))
            .context("cannot bound the pool tube's recv")?;
        tube.set_send_timeout(Some(timeout))
            .context("cannot bound the pool tube's send")?;
        let (gpa, size) = (pool.gpa, pool.size);
        let mapped = MappedPool::new(pool)?;
        info!(
            "virtio-media: serving MMAP buffers from the media_host pool (gpa {:#x}, {} MiB), \
             allocated by the VMM",
            gpa,
            size >> 20
        );
        Ok(Arc::new(RemotePool {
            tube,
            mapped,
            dead: AtomicBool::new(false),
        }))
    }

    /// One synchronous round trip to the VMM. Any failure -- a send or recv error, a timeout --
    /// marks the connection dead and is `EIO`: the VMM is gone or wedged, and no later answer
    /// could be matched to its request.
    fn request(&self, request: &PoolRequest) -> Result<PoolResponse, i32> {
        if self.dead.load(Ordering::Acquire) {
            return Err(libc::EIO);
        }
        if let Err(e) = self.tube.send(request) {
            error!("virtio-media: cannot reach the VMM's pool server: {e}");
            self.dead.store(true, Ordering::Release);
            return Err(libc::EIO);
        }
        match self.tube.recv() {
            Ok(response) => Ok(response),
            Err(e) => {
                error!(
                    "virtio-media: no answer from the VMM's pool server within {} s: {e}",
                    POOL_RPC_TIMEOUT.as_secs()
                );
                self.dead.store(true, Ordering::Release);
                Err(libc::EIO)
            }
        }
    }

    /// An answer that is not what the request asked for: the two sides are out of step (see
    /// [`RemotePool::dead`]), so the connection is poisoned.
    fn out_of_step(&self, response: &PoolResponse) -> i32 {
        error!(
            "virtio-media: the VMM's pool server is out of step (unexpected {response:?}); \
             giving up on the connection"
        );
        self.dead.store(true, Ordering::Release);
        libc::EIO
    }
}

/// A helper device's buffer allocator: every offset comes from the VMM's one allocator, over the
/// tube; the buffers are built locally from the helper's own whole-pool mapping.
///
/// Dropping it releases nothing (the buffers' offsets live in the VMM's books until they are
/// released one by one, or until this process's tube closes and the VMM's server sweeps the
/// lot) -- the same contract `HostBuffer` itself has.
pub struct RemotePoolAllocator {
    pool: Arc<RemotePool>,
    /// The device's V4L2 card name, for this side's error lines (the exhaustion log is the
    /// VMM's, where the allocator is).
    card: String,
}

impl RemotePoolAllocator {
    pub fn new(pool: Arc<RemotePool>, card: String) -> Self {
        RemotePoolAllocator { pool, card }
    }

    /// Return one reserved offset to the VMM. Logs and carries on whatever comes back: the
    /// guest's STREAMOFF/REQBUFS(0) must not fail over a bookkeeping line, and if the VMM is
    /// really gone this process is about to be swept whole anyway.
    fn give_back(&mut self, offset: u64) {
        match self.pool.request(&PoolRequest::Release { offset }) {
            Ok(PoolResponse::Released) => {}
            Ok(other) => {
                let _ = self.pool.out_of_step(&other);
            }
            Err(_) => {
                error!(
                    "virtio-media: \"{}\" could not return pool offset {:#x} to the VMM",
                    self.card, offset
                );
            }
        }
    }
}

impl VirtioMediaBufferAllocator for RemotePoolAllocator {
    fn allocate(&mut self, len: u64) -> Result<HostBuffer, i32> {
        match self.pool.request(&PoolRequest::Reserve { len })? {
            PoolResponse::Reserved { offset } => {
                match self.pool.mapped.buffer_at(offset, len) {
                    Ok(buffer) => Ok(buffer),
                    Err(errno) => {
                        // The VMM already booked the reservation, and this side is the only one
                        // that knows the buffer over it was never built: give the offset back,
                        // exactly as the in-VMM twin unreserves (R8-2) -- otherwise a helper
                        // that cannot dup another fd leaks pool space on every retried REQBUFS.
                        self.give_back(offset);
                        Err(errno)
                    }
                }
            }
            PoolResponse::Errno(errno) => Err(errno),
            other => Err(self.pool.out_of_step(&other)),
        }
    }

    fn release(&mut self, buf: HostBuffer) {
        let Some(offset) = buf.pool_offset else {
            error!(
                "virtio-media: \"{}\" is releasing a buffer that is not the pool's",
                self.card
            );
            return;
        };
        self.give_back(offset);
        drop(buf);
    }
}

#[cfg(test)]
mod tests {
    use std::time::Instant;

    use base::pagesize;
    use base::SafeDescriptor;
    use base::SharedMemory;
    use base::UnixSeqpacket;
    use vm_memory::GuestAddress;

    use super::*;

    const POOL_SIZE: u64 = 16 << 20;

    /// A pool over a plain shared-memory object, the same shape `MediaPoolHandle` describes for
    /// the `media_host` window carved out of guest memory.
    fn pool() -> MediaPool {
        MediaPool::new(handle()).unwrap()
    }

    fn handle() -> MediaPoolHandle {
        let shm = SharedMemory::new("media_pool_test", POOL_SIZE).unwrap();
        handle_over(&SafeDescriptor::from(shm))
    }

    /// A `MediaPoolHandle` over `fd` -- so one shared-memory object can be "the pool" for a
    /// server and several clients at once, as the one memfd window is for the VMM and its
    /// helpers.
    fn handle_over(fd: &SafeDescriptor) -> MediaPoolHandle {
        MediaPoolHandle {
            fd: fd.try_clone().unwrap(),
            fd_offset: 0,
            host_va: 0,
            gpa: 0x1_0000_0000,
            size: POOL_SIZE,
        }
    }

    fn used(pool: &MediaPool) -> u64 {
        pool.inner.lock().used
    }

    fn live(pool: &MediaPool) -> usize {
        pool.inner.lock().live.len()
    }

    /// The reservation half on its own: reserve hands out page-aligned offsets, unreserve gives
    /// them back -- but only to their owner. A foreign offset (another lease's, or nobody's) is
    /// refused and nothing is freed, because with one offset space for the whole VM a stray
    /// release would hand one device's bytes to the next asker.
    #[test]
    fn reserve_and_unreserve_round_trip_with_an_owner_check() {
        let pool = pool();
        let page = pagesize() as u64;
        let mut camera = pool.lease("camera".into());
        let mut encoder = pool.lease("encoder".into());

        let offset = camera.reserve(5 * page).unwrap();
        assert_eq!(offset % page, 0);
        assert_eq!(used(&pool), 5 * page);

        // A zero or absurd length is EINVAL before the allocator is touched.
        assert_eq!(camera.reserve(0), Err(libc::EINVAL));
        assert_eq!(camera.reserve(u64::MAX), Err(libc::EINVAL));
        assert_eq!(camera.reserve(POOL_SIZE + page), Err(libc::EINVAL));
        assert_eq!(used(&pool), 5 * page);

        // The encoder cannot free the camera's offset, and nobody can free a made-up one.
        assert_eq!(encoder.unreserve(offset), Err(libc::EINVAL));
        assert_eq!(camera.unreserve(offset + 100 * page), Err(libc::EINVAL));
        assert_eq!(used(&pool), 5 * page, "a refused release frees nothing");

        // The owner can; and only once.
        camera.unreserve(offset).unwrap();
        assert_eq!(used(&pool), 0);
        assert_eq!(camera.unreserve(offset), Err(libc::EINVAL));
    }

    /// Buffers a device still holds when it goes away come back to the pool.
    ///
    /// This is the `release_owner` sweep of `MediaPool`, which no guest can reach: `video_open`
    /// takes a reference on the driver module, so an open session (or a live mmap, which pins the
    /// file) makes `rmmod` fail, and with nothing open the buffers are already gone -- two
    /// acceptance runs on hardware never produced the log line (`B2-acceptance.md` finding 2).
    /// Dropping a lease with buffers outstanding is what a virtio reset, a VM teardown, or (via
    /// the pool server thread) a helper dying does.
    #[test]
    fn dropping_a_lease_gives_its_buffers_back_to_the_pool() {
        let pool = pool();
        let page = pagesize() as u64;
        let len = 4 * page;

        let mut lease = pool.lease("lb0".into());
        let buffers: Vec<HostBuffer> = (0..3).map(|_| lease.allocate(len).unwrap()).collect();
        assert_eq!(live(&pool), 3);
        assert_eq!(used(&pool), 3 * len);
        // Distinct, page-aligned offsets inside the pool.
        let mut offsets: Vec<u64> = buffers.iter().map(|b| b.pool_offset.unwrap()).collect();
        offsets.sort_unstable();
        offsets.dedup();
        assert_eq!(offsets.len(), 3);
        assert!(offsets.iter().all(|o| o % page == 0 && *o < POOL_SIZE));

        // The device goes away with its buffers still live: dropping a `HostBuffer` frees
        // nothing in the pool by design, so only the lease's sweep can give the space back.
        let owner = lease.owner;
        drop(buffers);
        assert_eq!(
            used(&pool),
            3 * len,
            "a dropped buffer keeps its reservation"
        );
        // The sweep is the branch that logs `reclaiming N media_host buffers ...`; `N` is what
        // it returns, so this is that log line, asserted.
        assert_eq!(pool.inner.lock().release_owner(owner), 3);
        drop(lease);
        assert_eq!(live(&pool), 0);
        assert_eq!(used(&pool), 0);
        // Nothing is swept twice.
        assert_eq!(pool.inner.lock().release_owner(owner), 0);

        // And the offset space really is free again: a fresh lease can take the whole pool.
        let mut next = pool.lease("lb0".into());
        let whole = next.allocate(POOL_SIZE).unwrap();
        assert_eq!(whole.pool_offset, Some(0));
        next.release(whole);
        assert_eq!(used(&pool), 0);
    }

    /// One lease's sweep leaves the other lease's buffers alone -- the pool is shared by every
    /// media device of the VM, so an owner id is what separates them.
    #[test]
    fn a_sweep_only_takes_the_buffers_of_its_own_lease() {
        let pool = pool();
        let len = 4 * pagesize() as u64;

        let mut first = pool.lease("lb0".into());
        let mut second = pool.lease("simple_device".into());
        let kept = second.allocate(len).unwrap();
        let dropped = first.allocate(len).unwrap();
        assert_ne!(kept.pool_offset, dropped.pool_offset);
        assert_eq!(used(&pool), 2 * len);

        drop(dropped);
        drop(first);
        assert_eq!(live(&pool), 1, "the other device keeps its buffer");
        assert_eq!(used(&pool), len);

        second.release(kept);
        assert_eq!(used(&pool), 0);
        drop(second);
    }

    /// Exhaustion is `ENOMEM`, and a failed allocation leaves nothing behind.
    /// A helper rebuilds the pool from the memory table it was sent, given only the base: the
    /// region that starts there, with its memfd and its offset inside it. The pool is the second
    /// region of the same backing object, the way the aarch64 layout appends it above guest RAM,
    /// so a handle that lost the offset would map the guest's RAM instead.
    #[test]
    fn the_pool_is_found_again_by_its_guest_base() {
        let ram_base = GuestAddress(0x8000_0000);
        let ram_size = 4 * pagesize() as u64;
        let pool_gpa = 0x1_0000_0000u64;
        let mem =
            GuestMemory::new(&[(ram_base, ram_size), (GuestAddress(pool_gpa), POOL_SIZE)]).unwrap();

        let handle = pool_handle_at(&mem, pool_gpa).unwrap();
        assert_eq!((handle.gpa, handle.size), (pool_gpa, POOL_SIZE));
        assert_eq!(
            handle.fd_offset, ram_size,
            "the pool sits behind guest RAM in the memfd"
        );
        assert_eq!(handle.guest_range(), (GuestAddress(pool_gpa), POOL_SIZE));

        // Bytes written through the helper's own mapping of the pool are the guest's bytes.
        let pool = MediaPool::new(handle).unwrap();
        assert_eq!(pool.guest_range(), (pool_gpa, POOL_SIZE));
        let mut lease = pool.lease("lb0".into());
        let mut buffer = lease.allocate(pagesize() as u64).unwrap();
        let offset = buffer.pool_offset.unwrap();
        // SAFETY: the buffer maps at least one page, and nothing else references it.
        unsafe { buffer.as_mut_ptr().write_bytes(0xa5, 16) };
        let mut seen = [0u8; 16];
        mem.read_exact_at_addr(&mut seen, GuestAddress(pool_gpa + offset))
            .unwrap();
        assert_eq!(seen, [0xa5; 16]);
        // ... and not RAM's, which is what forgetting `fd_offset` would have written.
        mem.read_exact_at_addr(&mut seen, GuestAddress(ram_base.offset() + offset))
            .unwrap();
        assert_eq!(seen, [0; 16]);
        lease.release(buffer);

        // A base that is not a region start, even one inside a region, is refused by name.
        let e = pool_handle_at(&mem, pool_gpa + pagesize() as u64)
            .err()
            .expect("a base inside a region is not that region's base");
        assert!(
            e.to_string().contains("no memory region starts at"),
            "{e:#}"
        );
        assert!(pool_handle_at(&mem, 0).is_err());
    }

    #[test]
    fn an_exhausted_pool_answers_enomem_and_stays_usable() {
        let pool = pool();
        let mut lease = pool.lease("lb0".into());
        let whole = lease.allocate(POOL_SIZE).unwrap();
        assert_eq!(lease.allocate(pagesize() as u64).err(), Some(libc::ENOMEM));
        assert_eq!(live(&pool), 1);
        lease.release(whole);
        assert_eq!(used(&pool), 0);
        let again = lease.allocate(POOL_SIZE).unwrap();
        assert_eq!(again.pool_offset, Some(0));
        lease.release(again);
    }

    /// A short RPC bound, so a test's dead VMM is noticed in milliseconds.
    const TEST_RPC_TIMEOUT: Duration = Duration::from_millis(200);

    /// A remote client over `fd`'s pool, its RPC bound shortened for the tests.
    fn remote_client(fd: &SafeDescriptor, tube: Tube, card: &str) -> RemotePoolAllocator {
        let pool = RemotePool::with_timeout(tube, handle_over(fd), TEST_RPC_TIMEOUT).unwrap();
        RemotePoolAllocator::new(pool, card.to_string())
    }

    /// The M8 shape end to end, in one process: the VMM's pool, two server threads, two remote
    /// clients over the same shared-memory window. Every offset comes from the one allocator, so
    /// two helpers never alias (what D49's slices used to guarantee statically); the space one
    /// frees is space the other can have (what the slices could not do); a foreign release is
    /// refused; and a client whose tube closes is swept whole.
    #[test]
    fn the_vmm_serves_the_pool_to_remote_allocators() {
        let page = pagesize() as u64;
        let shm = SafeDescriptor::from(SharedMemory::new("media_pool_test", POOL_SIZE).unwrap());
        let pool = MediaPool::new(handle_over(&shm)).unwrap();

        let (camera_vmm, camera_helper) = Tube::pair().unwrap();
        let (encoder_vmm, encoder_helper) = Tube::pair().unwrap();
        let camera_server = pool.spawn_server("camera", camera_vmm).unwrap();
        let encoder_server = pool.spawn_server("encoder", encoder_vmm).unwrap();
        let mut camera = remote_client(&shm, camera_helper, "camera");
        let mut encoder = remote_client(&shm, encoder_helper, "encoder");

        // Round trips through both clients: distinct, page-aligned offsets out of the one
        // allocator -- never both 0, which is what two private allocators answered (D49).
        let raw = camera.allocate(5 * page).unwrap();
        let coded = encoder.allocate(2 * page).unwrap();
        let (raw_offset, coded_offset) = (raw.pool_offset.unwrap(), coded.pool_offset.unwrap());
        assert_ne!(raw_offset, coded_offset);
        assert!(raw_offset % page == 0 && coded_offset % page == 0);
        assert_eq!(used(&pool), 7 * page);
        // A buffer's bytes really are the shared window's at its offset: what one side writes
        // through its own mapping, the other side of the memfd shows.
        let mut raw = raw;
        // SAFETY: the buffer maps 5 pages and nothing else references it.
        unsafe { raw.as_mut_ptr().write_bytes(0x5a, 16) };
        let check = MemoryMappingBuilder::new(POOL_SIZE as usize)
            .from_file(&File::from(shm.try_clone().unwrap()))
            .build()
            .unwrap();
        let mut seen = [0u8; 16];
        check.read_slice(&mut seen, raw_offset as usize).unwrap();
        assert_eq!(seen, [0x5a; 16]);

        // Exhaustion is the VMM's ENOMEM over the tube; space one device frees, the other can
        // have -- the dynamic sharing the static slices could not do.
        assert_eq!(camera.allocate(POOL_SIZE).err(), Some(libc::ENOMEM));
        assert_eq!(camera.allocate(0).err(), Some(libc::EINVAL));
        camera.release(raw);
        encoder.release(coded);
        assert_eq!(used(&pool), 0);
        let whole = encoder.allocate(POOL_SIZE).unwrap();
        assert_eq!(whole.pool_offset, Some(0));

        // A foreign-offset release is refused: the camera returning the encoder's offset changes
        // nothing (the VMM logs it), and the encoder's buffer stays reserved.
        let foreign = pool.inner.lock().mapped.buffer_at(0, page).unwrap();
        camera.release(foreign);
        assert_eq!(used(&pool), POOL_SIZE, "a foreign release frees nothing");

        // The encoder helper dies with its buffer outstanding: EOF on its tube is the server
        // thread's exit, and the lease it drops sweeps the reservation back.
        drop(whole);
        drop(encoder);
        encoder_server.join().unwrap();
        assert_eq!(
            used(&pool),
            0,
            "the sweep reclaimed the dead helper's bytes"
        );
        assert_eq!(live(&pool), 0);

        // The camera is untouched by its neighbour's death and can now have everything.
        let after = camera.allocate(POOL_SIZE).unwrap();
        assert_eq!(after.pool_offset, Some(0));
        camera.release(after);
        drop(camera);
        camera_server.join().unwrap();
        assert_eq!(used(&pool), 0);
    }

    /// A helper whose VMM stops answering gets `EIO` within the RPC bound -- never a hung
    /// REQBUFS -- and the connection stays failed once out of step.
    #[test]
    fn a_dead_vmm_is_eio_within_the_timeout() {
        let page = pagesize() as u64;
        let shm = SafeDescriptor::from(SharedMemory::new("media_pool_test", POOL_SIZE).unwrap());

        // Wedged: the far end is alive but nothing serves it.
        let (wedged_vmm, helper) = Tube::pair().unwrap();
        let mut client = remote_client(&shm, helper, "decoder");
        let asked = Instant::now();
        assert_eq!(client.allocate(page).err(), Some(libc::EIO));
        let waited = asked.elapsed();
        assert!(
            waited >= TEST_RPC_TIMEOUT && waited < TEST_RPC_TIMEOUT * 10,
            "the wait was bounded by the RPC timeout, not forever: {waited:?}"
        );
        // Out of step is for good: the next request fails at once, without waiting again.
        let again = Instant::now();
        assert_eq!(client.allocate(page).err(), Some(libc::EIO));
        assert!(again.elapsed() < TEST_RPC_TIMEOUT);
        drop(wedged_vmm);

        // Gone: the far end is closed, and a fresh client fails without waiting out the bound.
        let (dead_vmm, helper) = Tube::pair().unwrap();
        drop(dead_vmm);
        let mut client = remote_client(&shm, helper, "decoder");
        assert_eq!(client.allocate(page).err(), Some(libc::EIO));
        // Releasing into the void is logged and survived, not an error the device sees.
        let orphan = MappedPool::new(handle_over(&shm))
            .unwrap()
            .buffer_at(0, page)
            .unwrap();
        client.release(orphan);
    }

    /// A reservation whose buffer cannot be built is given straight back to the VMM: without
    /// that, every failed `buffer_at` (an `EMFILE` on the per-buffer dup, say) leaks its
    /// page-rounded reservation for the life of the helper, and retried REQBUFS drain the whole
    /// VM's pool (R8-2). Injected here by a client whose own mapping is smaller than the pool,
    /// so the third offset the VMM hands out is one its `buffer_at` must refuse.
    #[test]
    fn a_buffer_that_cannot_be_built_returns_its_reservation() {
        let page = pagesize() as u64;
        let shm = SafeDescriptor::from(SharedMemory::new("media_pool_test", POOL_SIZE).unwrap());
        let pool = MediaPool::new(handle_over(&shm)).unwrap();
        let (vmm, helper) = Tube::pair().unwrap();
        let server = pool.spawn_server("decoder", vmm).unwrap();

        let small = MediaPoolHandle {
            fd: shm.try_clone().unwrap(),
            fd_offset: 0,
            host_va: 0,
            gpa: 0x1_0000_0000,
            size: 2 * page,
        };
        let remote = RemotePool::with_timeout(helper, small, TEST_RPC_TIMEOUT).unwrap();
        let mut client = RemotePoolAllocator::new(remote, "decoder".to_string());

        let first = client.allocate(page).unwrap();
        let second = client.allocate(page).unwrap();
        assert_eq!(used(&pool), 2 * page);

        // The VMM reserves the third page; `buffer_at` fails past the small mapping; the failed
        // allocate must return the reservation rather than strand it.
        assert_eq!(client.allocate(page).err(), Some(libc::EIO));
        assert_eq!(
            used(&pool),
            2 * page,
            "the reservation behind the failed buffer build was given back"
        );

        client.release(first);
        client.release(second);
        assert_eq!(used(&pool), 0);
        drop(client);
        server.join().unwrap();
    }

    /// A tube error that is not EOF must not sweep the lease: the helper on the far end is
    /// still a live process, still writing into the buffers it holds (R8-1). One garbage packet
    /// is logged and survived -- the pool's books do not move and the server keeps serving --
    /// and a run of them makes the server abandon the connection but hold the lease, parked
    /// until the EOF that alone proves the writer is gone.
    #[test]
    fn a_non_eof_tube_error_keeps_the_lease_alive() {
        let page = pagesize() as u64;
        let shm = SafeDescriptor::from(SharedMemory::new("media_pool_test", POOL_SIZE).unwrap());
        let pool = MediaPool::new(handle_over(&shm)).unwrap();

        let (vmm_socket, helper_socket) = UnixSeqpacket::pair().unwrap();
        // A raw copy of the helper's end, to inject packets no `PoolRequest` deserializes from.
        let raw = helper_socket.try_clone().unwrap();
        let server = pool
            .spawn_server("camera", Tube::try_from(vmm_socket).unwrap())
            .unwrap();
        let mut camera = remote_client(&shm, Tube::try_from(helper_socket).unwrap(), "camera");

        let held = camera.allocate(5 * page).unwrap();
        assert_eq!(used(&pool), 5 * page);

        // One unreadable packet: a non-EOF error on the server's recv.
        raw.send(b"not a pool request").unwrap();
        // The next round trip proves the server is still serving (SOCK_SEQPACKET is FIFO, so
        // the garbage was processed first) and that the error swept nothing.
        let second = camera.allocate(page).unwrap();
        assert_eq!(used(&pool), 6 * page);
        camera.release(second);
        assert_eq!(
            used(&pool),
            5 * page,
            "an injected non-EOF error did not move the pool's books"
        );

        // A whole run of them: the server stops serving -- but the lease must survive, parked
        // until EOF.
        for _ in 0..POOL_SERVER_MAX_CONSECUTIVE_ERRORS {
            raw.send(b"junk").unwrap();
        }
        // The next request goes unanswered (the server is parked): a bounded EIO, and no sweep.
        assert_eq!(camera.allocate(page).err(), Some(libc::EIO));
        assert_eq!(
            used(&pool),
            5 * page,
            "the parked server still holds the lease for its live helper"
        );

        // Only EOF sweeps: every client-side copy of the fd closes, the parked server sees it.
        drop(raw);
        drop(camera);
        drop(held);
        server.join().unwrap();
        assert_eq!(used(&pool), 0);
        assert_eq!(live(&pool), 0);
    }
}
