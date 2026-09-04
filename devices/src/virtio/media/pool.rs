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
use resources::address_allocator::AddressAllocator;
use resources::AddressRange;
use resources::Alloc;
use sync::Mutex;
use virtio_media::HostBuffer;
use virtio_media::VirtioMediaBufferAllocator;
use vm_memory::MediaPoolHandle;

/// The VM-wide `media_host` pool: one offset space, shared by every media device.
///
/// It has to be VM-wide. A pool offset is what the device answers `VIDIOC_QUERYBUF` with and the
/// guest adds the `media_host` node's base to it, so two devices with private allocators over the
/// same window both hand out offset 0, 0x1000, ... and the guest maps the same physical bytes for
/// two unrelated buffers -- while one device's `release` frees space the other is still filling
/// (`VPU_DESIGN.md` §2.2 sizes the one 256 MiB pool for every media device in the VM).
///
/// The descriptor is dup'ed and the whole pool mapped exactly once, here; a buffer's own dup is
/// what the crate's `HostBuffer` carries.
pub struct MediaPoolAllocator {
    pool: MediaPoolHandle,
    /// The pool's offset space, `[0, pool.size)`, page aligned.
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
    fn new(pool: MediaPoolHandle) -> anyhow::Result<Self> {
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
        let allocator = AddressAllocator::new(
            AddressRange::from_start_and_end(0, pool.size - 1),
            Some(base::pagesize() as u64),
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
            HostBuffer::from_raw_parts(
                fd,
                self.pool.fd_offset + offset,
                len,
                ptr,
                Some(offset),
            )
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
        })
    }

    pub fn as_raw_descriptor(&self) -> base::RawDescriptor {
        self.fd
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

    use super::*;

    const POOL_SIZE: u64 = 16 << 20;

    /// A pool over a plain shared-memory object, the same shape `MediaPoolHandle` describes for
    /// the `media_host` window carved out of guest memory.
    fn pool() -> MediaPool {
        let shm = SharedMemory::new("media_pool_test", POOL_SIZE).unwrap();
        let fd = SafeDescriptor::from(shm);
        MediaPool::new(MediaPoolHandle {
            fd,
            fd_offset: 0,
            host_va: 0,
            gpa: 0x1_0000_0000,
            size: POOL_SIZE,
        })
        .unwrap()
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
