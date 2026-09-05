// Copyright 2026 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! The VM-wide `media_host` buffer pool and the per-device leases on it.
//!
//! Kept apart from the rest of `media.rs` because it is the one piece of the media device that
//! depends on nothing but `base`, `resources`, `vm_memory` and the virtio-media crate's allocator
//! trait -- which is what makes it testable on its own, and the lease sweep below is a path no
//! guest can drive (a `rmmod` cannot drop a device that still has open sessions, so the sweep only
//! runs on VM teardown or a hot-remove; see `logs/vpu_wp/B2-acceptance.md` finding 2).

use std::collections::BTreeMap;
use std::fs::File;
use std::os::fd::OwnedFd;
use std::ptr::NonNull;
use std::sync::Arc;

use anyhow::Context;
use base::error;
use base::info;
use base::AsRawDescriptor;
use base::MappedRegion;
use base::MemoryMapping;
use base::MemoryMappingBuilder;
use base::SafeDescriptor;
use resources::address_allocator::AddressAllocator;
use resources::AddressRange;
use resources::Alloc;
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
/// ([`MediaPool::new`]), which is what `MediaPoolHandle` says an out-of-process consumer must do.
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

/// Split a pool of `size` bytes into one page-aligned `(offset, len)` slice per weight, in
/// order, covering the pool: `size * w / total` rounded down to a page, the remainder on the
/// last slice. The weights are the media devices of the VM in configuration order (the decoder
/// weighs twice the others: its CAPTURE queue holds a decoded-frame queue, not one frame in
/// flight), so the VMM and every helper agree on who owns which bytes without ever talking --
/// which is the whole point: helpers cannot share the VM-wide allocator, and two allocators
/// over one window alias each other's buffers (D49).
///
/// An empty `weights` returns no slices; a zero weight gets a zero-length slice (which
/// [`MediaPool::with_slice`] refuses -- do not pass one for a device that allocates).
pub fn carve_slices(size: u64, weights: &[u64]) -> Vec<(u64, u64)> {
    let page = base::pagesize() as u64;
    let total: u64 = weights.iter().sum();
    if total == 0 {
        return weights.iter().map(|_| (0, 0)).collect();
    }
    let mut slices = Vec::with_capacity(weights.len());
    let mut at = 0u64;
    for w in weights {
        let len = (size / total).saturating_mul(*w) / page * page;
        slices.push((at, len));
        at += len;
    }
    // The rounding's remainder goes to the last slice that allocates at all.
    if let Some(last) = slices
        .iter()
        .zip(weights)
        .rposition(|(_, w)| *w > 0)
        .map(|i| &mut slices[i].1)
    {
        *last += size - at;
    }
    slices
}

/// The VM-wide `media_host` pool: one offset space, shared by every media device.
///
/// It has to be VM-wide. A pool offset is what the device answers `VIDIOC_QUERYBUF` with and the
/// guest adds the `media_host` node's base to it, so two devices with private allocators over the
/// same window both hand out offset 0, 0x1000, ... and the guest maps the same physical bytes for
/// two unrelated buffers -- while one device's `release` frees space the other is still filling
/// (`VPU_DESIGN.md` §2.2 sizes the one 256 MiB pool for every media device in the VM).
///
/// A media *helper* is a separate process and cannot share this allocator, so it gets a `slice`
/// instead: a `(offset, len)` window of the offset space that is its alone, computed once by the
/// VMM over every media device of the VM ([`carve_slices`]) and carried in the helper's
/// parameters. Without that, three helpers over one pool each hand out offset 0 first, and the
/// encoder's coded-frame writes land inside the camera's raw buffers -- which is exactly D49's
/// band of noise (`logs/vpu_wp/F12-encoder.md`).
///
/// The descriptor is dup'ed and the whole pool mapped exactly once, here; a buffer's own dup is
/// what the crate's `HostBuffer` carries.
pub struct MediaPoolAllocator {
    pool: MediaPoolHandle,
    /// This allocator's part of the pool's offset space -- `[0, pool.size)` for the VM-wide
    /// allocator, the device's slice in a helper -- page aligned.
    allocator: AddressAllocator,
    /// Our own mapping of the pool, so that buffers can be filled from this process.
    host_map: MemoryMapping,
    /// Key for the next allocation; `AddressAllocator` wants every live allocation to have a
    /// distinct one.
    next_id: usize,
    /// Id for the next lease.
    next_owner: u64,
    /// `pool offset -> (owner, size)` for every slice handed out. The owner is what lets a
    /// device that goes away -- a virtio reset drops its worker and every buffer with it -- give
    /// its slices back to a pool the rest of the VM keeps using.
    live: BTreeMap<u64, (u64, u64)>,
    /// Bytes currently handed out, for the exhaustion log line.
    used: u64,
}

impl MediaPoolAllocator {
    fn new(pool: MediaPoolHandle, slice: Option<(u64, u64)>) -> anyhow::Result<Self> {
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
        let page = base::pagesize() as u64;
        let (start, len) = slice.unwrap_or((0, pool.size));
        if len == 0
            || start % page != 0
            || len % page != 0
            || start.checked_add(len).map_or(true, |end| end > pool.size)
        {
            anyhow::bail!(
                "media_host pool slice {:#x}+{:#x} is not a page-aligned part of the {:#x}-byte \
                 pool",
                start,
                len,
                pool.size
            );
        }
        let allocator = AddressAllocator::new(
            AddressRange::from_start_and_end(start, start + len - 1),
            Some(page),
            None,
        )?;
        Ok(MediaPoolAllocator {
            pool,
            allocator,
            host_map,
            next_id: 0,
            next_owner: 0,
            live: BTreeMap::new(),
            used: 0,
        })
    }

    /// Carve `len` bytes out of the pool for `owner`. `card` only names the device in the
    /// exhaustion log -- with one pool for the whole VM, "the pool is full" is useless without
    /// knowing who asked.
    fn allocate(&mut self, len: u64, owner: u64, card: &str) -> Result<HostBuffer, i32> {
        if len == 0 {
            return Err(libc::EINVAL);
        }
        let page = base::pagesize() as u64;
        let size = len.checked_next_multiple_of(page).ok_or(libc::EINVAL)?;
        let id = self.next_id;
        self.next_id += 1;
        let (used, pool_size) = (self.used, self.pool.size);
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

        let fd: OwnedFd = match self.pool.fd.try_clone() {
            Ok(fd) => fd.into(),
            Err(e) => {
                error!("virtio-media: cannot dup the pool descriptor: {}", e);
                let _ = self.allocator.release_containing(offset);
                return Err(libc::EIO);
            }
        };
        self.used += size;
        self.live.insert(offset, (owner, size));
        // SAFETY: `offset + size <= pool.size` (the allocator's range), and `host_map` maps all
        // `pool.size` bytes for as long as this allocator exists, which is longer than any buffer
        // it hands out (buffers come back through `release`, and a lease that dies takes its
        // buffers' space with it).
        let ptr = unsafe { self.host_map.as_ptr().add(offset as usize) };
        let ptr = match NonNull::new(ptr) {
            Some(ptr) => ptr,
            None => {
                self.free(offset);
                return Err(libc::EIO);
            }
        };
        // SAFETY: `ptr` maps the `len` bytes at `pool.fd_offset + offset` of `fd`, and stays
        // valid for the life of the allocator (see above).
        Ok(unsafe {
            HostBuffer::from_raw_parts(fd, self.pool.fd_offset + offset, len, ptr, Some(offset))
        })
    }

    /// Give one offset back to the offset space.
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

    fn release(&mut self, buf: HostBuffer) {
        if let Some(offset) = buf.pool_offset {
            self.free(offset);
        }
        // Dropping closes the descriptor dup; the pool mapping is ours, not the buffer's.
        drop(buf);
    }

    /// Take back everything a lease still holds. Its buffers are gone with the device (dropping
    /// a `HostBuffer` frees nothing in the pool by design), so only the offset space is at stake.
    ///
    /// Returns how many slices were reclaimed -- the same number the log line carries, and the
    /// only way a test can tell the sweep apart from "there was nothing to sweep".
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
        Self::with_slice(pool, None)
    }

    /// Map the pool but allocate from `slice` (`(offset, len)` in the pool's offset space) alone.
    /// This is how a helper process, which cannot share the VM-wide allocator, is kept off the
    /// other devices' buffers: the VMM carves one slice per media device ([`carve_slices`]) and
    /// each helper allocates only inside its own. `None` is the whole pool.
    pub fn with_slice(pool: MediaPoolHandle, slice: Option<(u64, u64)>) -> anyhow::Result<Self> {
        let fd = pool.fd.as_raw_descriptor();
        let (gpa, size) = (pool.gpa, pool.size);
        let inner = MediaPoolAllocator::new(pool, slice)?;
        match slice {
            None => info!(
                "virtio-media: serving MMAP buffers from the media_host pool (gpa {:#x}, {} MiB)",
                gpa,
                size >> 20
            ),
            Some((start, len)) => info!(
                "virtio-media: serving MMAP buffers from the media_host pool slice {:#x}+{:#x} \
                 (gpa {:#x}, {} of {} MiB)",
                start,
                len,
                gpa,
                len >> 20,
                size >> 20
            ),
        }
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
}

/// One device's lease on the VM-wide pool.
pub struct PoolBufferAllocator {
    pool: Arc<Mutex<MediaPoolAllocator>>,
    /// The device's V4L2 card name, for the exhaustion log.
    card: String,
    owner: u64,
}

impl VirtioMediaBufferAllocator for PoolBufferAllocator {
    fn allocate(&mut self, len: u64) -> Result<HostBuffer, i32> {
        self.pool.lock().allocate(len, self.owner, &self.card)
    }

    fn release(&mut self, buf: HostBuffer) {
        self.pool.lock().release(buf)
    }
}

impl Drop for PoolBufferAllocator {
    fn drop(&mut self) {
        let _ = self.pool.lock().release_owner(self.owner);
    }
}

#[cfg(test)]
mod tests {
    use base::pagesize;
    use base::SafeDescriptor;
    use base::SharedMemory;
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
        let fd = SafeDescriptor::from(shm);
        MediaPoolHandle {
            fd,
            fd_offset: 0,
            host_va: 0,
            gpa: 0x1_0000_0000,
            size: POOL_SIZE,
        }
    }

    /// The D49 regression: helper processes cannot share one allocator, so each gets a slice of
    /// the offset space, and two sliced pools over the same window never hand out the same
    /// bytes. Without slices both start at offset 0 and the encoder's coded-frame writes land
    /// inside the camera's raw buffers.
    #[test]
    fn sliced_pools_over_one_window_do_not_alias() {
        let page = pagesize() as u64;
        let slices = carve_slices(POOL_SIZE, &[1, 2, 1]);
        assert_eq!(slices.len(), 3);
        // Page-aligned, in order, disjoint, and covering the whole pool.
        let mut end = 0;
        for (start, len) in &slices {
            assert_eq!(*start % page, 0);
            assert_eq!(*len % page, 0);
            assert_eq!(*start, end, "slices are adjacent and disjoint");
            end = start + len;
        }
        assert_eq!(end, POOL_SIZE, "the slices cover the pool");
        assert_eq!(
            slices[1].1,
            2 * slices[0].1,
            "the decoder's weight doubles its slice"
        );

        // Two "helpers" over the same window, each on its own slice: every offset handed out
        // stays inside its slice, so none alias -- unlike two whole-window pools, which both
        // answer 0 first.
        let (camera, encoder) = (
            MediaPool::with_slice(handle(), Some(slices[0])).unwrap(),
            MediaPool::with_slice(handle(), Some(slices[2])).unwrap(),
        );
        let mut cam_lease = camera.lease("camera".into());
        let mut enc_lease = encoder.lease("encoder".into());
        for _ in 0..3 {
            let raw = cam_lease.allocate(5 * page).unwrap();
            let coded = enc_lease.allocate(2 * page).unwrap();
            let (r, c) = (raw.pool_offset.unwrap(), coded.pool_offset.unwrap());
            let (cs, ce) = (slices[0].0, slices[0].0 + slices[0].1);
            let (es, ee) = (slices[2].0, slices[2].0 + slices[2].1);
            assert!(r >= cs && r + 5 * page <= ce, "camera stays in its slice");
            assert!(c >= es && c + 2 * page <= ee, "encoder stays in its slice");
        }
        // A slice is also all a helper gets: exhaustion is ENOMEM, not a neighbour's bytes.
        let full = cam_lease.allocate(slices[0].1).map(|b| b.pool_offset);
        assert_eq!(full, Err(libc::ENOMEM));

        // The unsliced pool still answers offset 0 first, so the whole-pool path is unchanged.
        let mut whole = pool().lease("lb0".into());
        assert_eq!(whole.allocate(page).unwrap().pool_offset, Some(0));

        // A slice outside the pool, or an unaligned one, is refused when the pool is built.
        assert!(MediaPool::with_slice(handle(), Some((0, POOL_SIZE + page))).is_err());
        assert!(MediaPool::with_slice(handle(), Some((page / 2, page))).is_err());
        assert!(MediaPool::with_slice(handle(), Some((0, 0))).is_err());
    }

    /// `carve_slices` corner cases: one device takes everything, a zero total carves nothing,
    /// and the rounding remainder lands on the last slice that allocates.
    #[test]
    fn carve_slices_covers_the_pool() {
        let page = pagesize() as u64;
        assert_eq!(carve_slices(POOL_SIZE, &[1]), vec![(0, POOL_SIZE)]);
        assert_eq!(carve_slices(POOL_SIZE, &[0, 0]), vec![(0, 0), (0, 0)]);
        // Three equal weights do not divide 16 MiB of pages evenly: the remainder goes to the
        // last slice and the total still covers the pool.
        let thirds = carve_slices(POOL_SIZE, &[1, 1, 1]);
        let total: u64 = thirds.iter().map(|(_, len)| len).sum();
        assert_eq!(total, POOL_SIZE);
        assert!(thirds[2].1 >= thirds[0].1);
        // A trailing zero weight stays empty; the remainder finds the last allocating slice.
        let with_empty = carve_slices(POOL_SIZE, &[1, 1, 0]);
        assert_eq!(with_empty[2].1, 0);
        assert_eq!(with_empty[0].1 + with_empty[1].1, POOL_SIZE);
        assert!(with_empty
            .iter()
            .all(|(start, len)| (start + len) % page == 0));
    }

    fn used(pool: &MediaPool) -> u64 {
        pool.inner.lock().used
    }

    fn live(pool: &MediaPool) -> usize {
        pool.inner.lock().live.len()
    }

    /// Buffers a device still holds when it goes away come back to the pool.
    ///
    /// This is the `release_owner` sweep of `MediaPool`, which no guest can reach: `video_open`
    /// takes a reference on the driver module, so an open session (or a live mmap, which pins the
    /// file) makes `rmmod` fail, and with nothing open the buffers are already gone -- two
    /// acceptance runs on hardware never produced the log line (`B2-acceptance.md` finding 2).
    /// Dropping a lease with buffers outstanding is what a virtio reset or a VM teardown does.
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
        assert_eq!(used(&pool), 3 * len, "a dropped buffer keeps its slice");
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
}
