// Copyright 2026 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! Handing the virtio-media pools to whoever consumes them.
//!
//! The other pools travel to their consumer as environment variables, because their consumer is a
//! renderer that crosvm forks and cannot otherwise be reached. The media device's consumer is not:
//! it is built from inside the VMM, and the vhost-user media helper is `exec`'d from the same
//! place with the pool descriptor already in hand. So this hands over the region itself --
//! `(fd, fd_offset, host_va, gpa, size)` -- rather than a set of strings that have to be parsed
//! back into it, and does it through one function both callers share, so an in-VMM device and a
//! helper cannot end up disagreeing about which region is the pool.
//!
//! See `plans/VPU_DESIGN.md` §2.2 and §3.1.

#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
use base::AsRawDescriptor;
use base::SafeDescriptor;

use crate::GuestAddress;
use crate::GuestMemory;
#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
use crate::MemoryRegionPurpose;

/// Everything a media device needs in order to serve host-owned buffers out of `media_host`.
///
/// The five numbers are all read off one `GuestMemory` region, so they cannot drift apart: the
/// descriptor and `fd_offset` are the slice of the guest memfd the region occupies (a pool is a
/// region of the shared guest memory object, not an object of its own, unless `isolate_backing`
/// asked otherwise), `host_va` is the mapping crosvm already made of exactly those pages, and
/// `(gpa, size)` is the window the guest was told about in the `media_host` device-tree node.
///
/// The descriptor is a **dup**, not a borrow: an in-VMM device keeps it for the life of the VM,
/// and a helper process needs something it can pass across an `exec`.
pub struct MediaPoolHandle {
    /// A dup of the region's backing object. Buffers are carved out of it as
    /// `(fd, fd_offset + pool_offset, len)`, which is what the media crate hands to
    /// `add_mapping` in place of a per-buffer memfd.
    pub fd: SafeDescriptor,
    /// Byte offset of the pool inside `fd`. Zero only by accident -- the pools sit above guest
    /// RAM in the same object -- so a consumer that forgets to add it reads the guest's RAM.
    pub fd_offset: u64,
    /// The host virtual address crosvm has already mapped the pool at. An in-process consumer can
    /// use it directly and save a second mapping of the same pages; a helper in another process
    /// must ignore it and map `fd` itself.
    pub host_va: u64,
    /// Guest-physical base, the same number the `media_host` reserved-memory node carries. The
    /// guest adds it to the pool-relative offset the host answers `VIDIOC_QUERYBUF` with.
    pub gpa: u64,
    /// Size of the window, in bytes.
    pub size: u64,
}

impl MediaPoolHandle {
    /// Find the `media_host` pool in a VM's memory, or `None` when the VM was not given one.
    ///
    /// `None` is a real answer, not a failure: a KVM VM, or a Gunyah VM started without
    /// `--pre-alloc media-host-mb`, has no pool and the media device falls back to the upstream
    /// per-buffer memfd + PCI shared-memory BAR path (see `VPU_DESIGN.md` §2.4). It is also the
    /// answer when the descriptor cannot be duplicated, which is logged rather than returned,
    /// because the only caller that could act on the difference would fall back anyway.
    pub fn from_guest_memory(mem: &GuestMemory) -> Option<MediaPoolHandle> {
        #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
        {
            for region in mem.regions() {
                if region.options.purpose != MemoryRegionPurpose::MediaPool {
                    continue;
                }
                let fd = match SafeDescriptor::try_from(region.shm as &dyn AsRawDescriptor) {
                    Ok(fd) => fd,
                    Err(e) => {
                        base::error!(
                            "MEDIA-POOL: cannot dup the media_host backing object at gpa {:#x}: \
                             {}",
                            region.guest_addr.offset(),
                            e,
                        );
                        return None;
                    }
                };
                return Some(MediaPoolHandle {
                    fd,
                    fd_offset: region.shm_offset,
                    host_va: region.host_addr as u64,
                    gpa: region.guest_addr.offset(),
                    size: region.size as u64,
                });
            }
        }
        // Referenced so the signature is identical on every architecture and a consumer needs no
        // `cfg` of its own: on x86 there is no such purpose and the answer is always `None`.
        let _ = mem;
        None
    }

    /// The guest-physical window, as the device tree describes it.
    pub fn guest_range(&self) -> (GuestAddress, u64) {
        (GuestAddress(self.gpa), self.size)
    }
}

/// The `media_guest` pool's guest-physical window, or `None` when the VM was not given one.
///
/// Only `(gpa, size)` on purpose. Nothing on the host allocates from this pool -- the guest driver
/// owns it and sends bare guest-physical scatter-gather lists -- so a host consumer wants exactly
/// one thing from it: whether an incoming address belongs to the pool, which it answers by
/// containment. Resolving such an address to host memory goes through `GuestMemory` like any
/// other guest address (`find_region` / `get_slice_at_addr`), gated by `check_host_access_range`.
pub fn media_guest_region(mem: &GuestMemory) -> Option<(u64, u64)> {
    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    {
        for region in mem.regions() {
            if region.options.purpose == MemoryRegionPurpose::MediaPoolGuest {
                return Some((region.guest_addr.offset(), region.size as u64));
            }
        }
    }
    let _ = mem;
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    use crate::MemoryRegionOptions;

    /// Both pools, side by side, in the same shape the aarch64 layout builds them: appended above
    /// guest RAM, fully pre-shared (`step_size == 0`).
    ///
    /// The point of the test is the pair of things that fail *silently* when a pool purpose is
    /// added to the enum and nowhere else (`VPU_DESIGN.md` §1.1): `check_host_access`'s catch-all
    /// arm would refuse the host every byte of both pools in a protected VM, with no compile
    /// error and no log line, and the media device would then read zeros or take an EFAULT that
    /// looks like a guest bug.
    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    #[test]
    fn media_pools_are_host_accessible_when_protected() {
        let ram = GuestAddress(0x0);
        let host_pool = GuestAddress(0x20_0000);
        let guest_pool = GuestAddress(0x40_0000);
        let mem = GuestMemory::new_with_options(&[
            (ram, 0x20_0000, MemoryRegionOptions::new()),
            (
                host_pool,
                0x20_0000,
                MemoryRegionOptions::new().purpose(MemoryRegionPurpose::MediaPool),
            ),
            (
                guest_pool,
                0x20_0000,
                MemoryRegionOptions::new().purpose(MemoryRegionPurpose::MediaPoolGuest),
            ),
        ])
        .unwrap();

        // Before protection everything is reachable, pool or not.
        assert!(mem.get_slice_at_addr(ram, 0x1000).is_ok());
        assert!(mem.get_slice_at_addr(host_pool, 0x1000).is_ok());

        mem.set_protected();

        // Plain guest RAM is lent to a protected guest: the host must not touch it.
        assert!(mem.get_slice_at_addr(ram, 0x1000).is_err());
        // Both pools are SHARE'd, so they stay reachable -- at the base, in the middle, and
        // across a range rather than a single byte.
        assert!(mem.get_slice_at_addr(host_pool, 0x1000).is_ok());
        assert!(mem
            .get_slice_at_addr(host_pool.unchecked_add(0x1_0000), 0x1000)
            .is_ok());
        assert!(mem.get_slice_at_addr(guest_pool, 0x1000).is_ok());
        assert!(mem
            .get_slice_at_addr(guest_pool.unchecked_add(0x1_0000), 0x1000)
            .is_ok());
    }

    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    #[test]
    fn handle_finds_the_host_pool_and_not_the_guest_one() {
        let ram = GuestAddress(0x0);
        let host_pool = GuestAddress(0x20_0000);
        let guest_pool = GuestAddress(0x40_0000);
        let mem = GuestMemory::new_with_options(&[
            (ram, 0x20_0000, MemoryRegionOptions::new()),
            (
                host_pool,
                0x20_0000,
                MemoryRegionOptions::new().purpose(MemoryRegionPurpose::MediaPool),
            ),
            (
                guest_pool,
                0x10_0000,
                MemoryRegionOptions::new().purpose(MemoryRegionPurpose::MediaPoolGuest),
            ),
        ])
        .unwrap();

        let handle = MediaPoolHandle::from_guest_memory(&mem).expect("media_host pool not found");
        assert_eq!(handle.gpa, 0x20_0000);
        assert_eq!(handle.size, 0x20_0000);
        assert_ne!(handle.host_va, 0);
        // The pool is a slice of the shared guest memory object, so its offset in that object is
        // not zero -- guest RAM is in front of it. A consumer that drops `fd_offset` would read
        // the guest's RAM instead of the pool.
        assert_eq!(handle.fd_offset, 0x20_0000);
        assert_eq!(handle.guest_range(), (host_pool, 0x20_0000));

        assert_eq!(media_guest_region(&mem), Some((0x40_0000, 0x10_0000)));
    }

    #[test]
    fn no_pools_is_not_an_error() {
        let mem = GuestMemory::new(&[(GuestAddress(0x0), 0x20_0000)]).unwrap();
        assert!(MediaPoolHandle::from_guest_memory(&mem).is_none());
        assert!(media_guest_region(&mem).is_none());
    }
}
