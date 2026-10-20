// Copyright 2026 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! Guest-owned media buffers as the host sees them.
//!
//! A guest driver that owns its buffers hands the device a scatter-gather list of guest-physical
//! ranges (`USERPTR` on the wire). This module turns such a list into something the host can use:
//! a linear host view of the bytes (an arena that re-maps the guest pages back to back, or for
//! small payloads a shadow copy written back on drop), and, on request, a dma-buf over the same
//! pages built with udmabuf, for a backend that can consume one.
//!
//! The address arithmetic is the one thing here that must be right and was not before
//! (`VPU_DESIGN.md` §1.4, §3.4): a guest-physical address is resolved through `find_region`, which
//! yields the region's mapping, the address's offset inside it, and the region's offset inside its
//! backing memfd; the memfd offset to map is the sum of the last two. Treating the address itself
//! as a memfd offset, or as an offset from the first region's base, is wrong on aarch64 where RAM
//! starts at `0x8000_0000`, and wrong for every region that is not the first. The host's right to
//! touch the bytes is checked first, before anything is mapped, so lent memory in a protected VM
//! answers `EFAULT` instead of a `SIGBUS` -- and so the four sources a guest-owned buffer can come
//! from (`media_guest` pool, restricted-dma-pool swiotlb, shared RAM, plain KVM RAM) all take the
//! same path. Who answers that question is a [`HostAccessPolicy`]: in the VMM the `GuestMemory`
//! itself, in a vhost-user helper the list of windows the VMM computed for it (`VPU_DESIGN.md`
//! §6.2).
//!
//! This module deliberately depends only on `base`, `vm_memory` and `resources` (for
//! `AddressRange`), not on the virtio-media crate, so it can be tested on its own.

use std::cell::OnceCell;

use base::pagesize;
use base::MappedRegion;
use base::MemoryMappingArena;
use base::Protection;
use base::SafeDescriptor;
use resources::AddressRange;
use thiserror::Error;
use vm_memory::udmabuf::UdmabufDriver;
use vm_memory::udmabuf::UdmabufDriverTrait;
use vm_memory::udmabuf::UdmabufError;
use vm_memory::GuestAddress;
use vm_memory::GuestMemory;
use vm_memory::GuestMemoryError;
#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
use vm_memory::MemoryRegionPurpose;

/// Below this many bytes a guest buffer is shadowed (copied in, copied back on drop) rather than
/// re-mapped: a mapping costs more than copying a control payload.
pub const MAPPING_THRESHOLD: usize = 0x400;

#[derive(Debug, Error)]
pub enum ImportError {
    #[error("empty scatter-gather list")]
    Empty,
    #[error("scatter-gather list cannot be mapped linearly: {0}")]
    NotLinear(&'static str),
    #[error("guest range {0:#x}+{1:#x} is not accessible to the host: {2}")]
    Inaccessible(u64, usize, GuestMemoryError),
    #[error("guest range {0:#x}+{1:#x} is outside every window the host may touch")]
    OutsideWindows(u64, usize),
    #[error("guest range {0:#x}+{1:#x} is not valid guest memory: {2}")]
    InvalidRange(u64, usize, GuestMemoryError),
    #[error("scatter-gather list length overflows")]
    Overflow,
    #[error("cannot build the host arena: {0}")]
    Arena(base::MmapError),
    #[error("cannot reference the pool pages: {0}")]
    Grant(String),
    #[error("udmabuf: {0}")]
    Udmabuf(UdmabufError),
}

impl ImportError {
    /// The errno the guest should see for this failure.
    pub fn errno(&self) -> i32 {
        match self {
            // Memory the host is not allowed to touch: the address is the guest's mistake.
            ImportError::Inaccessible(..) | ImportError::OutsideWindows(..) => libc::EFAULT,
            ImportError::Empty
            | ImportError::NotLinear(_)
            | ImportError::InvalidRange(..)
            | ImportError::Overflow => libc::EINVAL,
            ImportError::Arena(_) | ImportError::Grant(_) => libc::ENOMEM,
            ImportError::Udmabuf(_) => libc::EIO,
        }
    }
}

/// Distinguish "the host may not touch this" from "this is not guest memory at all".
fn access_error(addr: GuestAddress, len: usize, e: GuestMemoryError) -> ImportError {
    match e {
        GuestMemoryError::ProtectedMemoryAccess(..) => {
            ImportError::Inaccessible(addr.offset(), len, e)
        }
        other => ImportError::InvalidRange(addr.offset(), len, other),
    }
}

/// Who decides whether the host may touch a guest-physical range (`VPU_DESIGN.md` §6.2).
///
/// In the VMM the `GuestMemory` knows: its regions carry a purpose and it is marked protected
/// once the VM is, so `get_slice_at_addr` refuses lent memory by itself. In a vhost-user helper it
/// does not: the memory table the frontend sends carries no purpose and no protection flag, so
/// every address maps and a lent one would fault on first touch and take the helper -- and the VM
/// -- with it. The VMM therefore works out, from its own `GuestMemory`, which windows the host
/// may touch ([`host_accessible_windows`]), hands them to the helper, and the helper checks every
/// scatter-gather entry against them before mapping anything. Same question, two answerers.
pub enum HostAccessPolicy {
    /// The `GuestMemory`'s own gate: `check_host_access`, through `get_slice_at_addr`.
    GuestMemory,
    /// Only these guest-physical windows; an entry outside all of them is `EFAULT`.
    Windows(Vec<AddressRange>),
}

impl HostAccessPolicy {
    /// Refuse `addr..addr+len` unless the host may touch all of it.
    ///
    /// An entry has to lie inside one window, the way `get_slice_at_addr` needs it inside one
    /// region; the windows are the VMM's regions, so nothing legitimate straddles two. Whichever
    /// policy this is, an address that is not guest memory at all is `EINVAL`, and in the VMM the
    /// `GuestMemory` gate still runs after the window check -- it is the one that knows about a
    /// growable pool's grants.
    pub fn check(
        &self,
        mem: &GuestMemory,
        addr: GuestAddress,
        len: usize,
    ) -> Result<(), ImportError> {
        if let HostAccessPolicy::Windows(windows) = self {
            let range = AddressRange::from_start_and_size(addr.offset(), len as u64)
                .ok_or(ImportError::Overflow)?;
            if len != 0 && !windows.iter().any(|window| window.contains_range(range)) {
                return Err(ImportError::OutsideWindows(addr.offset(), len));
            }
        }
        mem.get_slice_at_addr(addr, len)
            .map(|_| ())
            .map_err(|e| access_error(addr, len, e))
    }
}

/// The guest-physical windows the host may touch in `mem`, for a helper that cannot tell.
///
/// The same rule `GuestMemory::check_host_access` applies once the VM is protected, written out
/// here because the VMM computes this before `set_protected` is called and the gate is not
/// public: every pool (SHARE'd, never lent), the static swiotlb region, the shared RAM window of
/// a pseudo-unprotected VM and the page its shim is told about, and the shared framebuffer. When
/// the VM's memory is not lent at all (`memory_is_lent == false`, i.e.
/// `!ProtectionType::isolates_memory()`), every region is the host's to touch.
///
/// A growable pool is listed whole: its ungranted pages are still the host's own memory, so
/// touching them cannot fault the helper; they are merely invisible to the guest, and a guest
/// cannot name them in a scatter-gather list it did not allocate. Every pool that exists today
/// is fully pre-shared anyway.
pub fn host_accessible_windows(mem: &GuestMemory, memory_is_lent: bool) -> Vec<AddressRange> {
    mem.regions()
        .filter(|region| {
            if !memory_is_lent {
                return true;
            }
            #[allow(clippy::match_like_matches_macro)]
            match region.options.purpose {
                #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
                MemoryRegionPurpose::GpuPool
                | MemoryRegionPurpose::GpuPoolGuest
                | MemoryRegionPurpose::Drm2KgslPool
                | MemoryRegionPurpose::VenusPool
                | MemoryRegionPurpose::MediaPool
                | MemoryRegionPurpose::MediaPoolGuest
                | MemoryRegionPurpose::DynamicTestPool
                | MemoryRegionPurpose::SharedGuestRam
                | MemoryRegionPurpose::ShimHandoff
                | MemoryRegionPurpose::SharedFramebuffer
                | MemoryRegionPurpose::StaticSwiotlbRegion => true,
                _ => false,
            }
        })
        .filter_map(|region| {
            AddressRange::from_start_and_size(region.guest_addr.offset(), region.size as u64)
        })
        .collect()
}

/// Direct linear mapping of sparse guest memory: an arena into which every SG entry's pages are
/// mapped back to back from the guest memory's backing memfd.
pub struct GuestArenaMapping {
    arena: MemoryMappingArena,
    /// Offset of the first byte inside the first page (the first entry need not be page aligned).
    start_offset: usize,
    len: usize,
}

impl GuestArenaMapping {
    pub fn new(
        mem: &GuestMemory,
        sgs: &[(GuestAddress, usize)],
        prot: Protection,
        policy: &HostAccessPolicy,
    ) -> Result<Self, ImportError> {
        let page_size = pagesize() as u64;
        let page_mask = page_size - 1;

        if sgs.is_empty() {
            return Err(ImportError::Empty);
        }

        // We can only map full pages and need to maintain a linear area. This means that the
        // following invariants must be withheld:
        //
        // - For all entries but the first, the start offset within the page must be 0.
        // - For all entries but the last, `start + len` must be a multiple of page size.
        for (addr, _) in sgs.iter().skip(1) {
            if addr.offset() & page_mask != 0 {
                return Err(ImportError::NotLinear(
                    "non-initial SG entry does not start on a page boundary",
                ));
            }
        }
        for (addr, len) in sgs.iter().take(sgs.len() - 1) {
            let end = addr
                .offset()
                .checked_add(*len as u64)
                .ok_or(ImportError::Overflow)?;
            if end & page_mask != 0 {
                return Err(ImportError::NotLinear(
                    "non-terminal SG entry does not end on a page boundary",
                ));
            }
        }

        // Compute the arena size.
        let mut arena_size: u64 = 0;
        let mut total_len: usize = 0;
        for (addr, len) in sgs {
            if *len == 0 {
                return Err(ImportError::Empty);
            }
            arena_size = arena_size
                .checked_add((addr.offset() & page_mask) + *len as u64)
                .ok_or(ImportError::Overflow)?;
            total_len = total_len.checked_add(*len).ok_or(ImportError::Overflow)?;
        }
        // Align to page size if the last entry did not cover a full page.
        let arena_size = arena_size.next_multiple_of(page_size);
        let mut arena = MemoryMappingArena::new(arena_size as usize).map_err(ImportError::Arena)?;

        // Map all SG entries.
        let mut pos = 0usize;
        for (addr, len) in sgs {
            // The host's right to these bytes, before anything is mapped: this is the gate that
            // refuses lent memory in a protected VM.
            policy.check(mem, *addr, *len)?;

            // Address of the first page of the region, and the whole pages to map.
            let first_page = GuestAddress(addr.offset() & !page_mask);
            let map_len = (addr.offset() - first_page.offset() + *len as u64)
                .next_multiple_of(page_size) as usize;

            // Where those pages are in the backing object: the region's offset inside it, plus
            // the address's offset inside the region.
            let (mapping, map_offset, memfd_offset) = mem
                .find_region(first_page)
                .map_err(|e| access_error(*addr, *len, e))?;
            if map_offset
                .checked_add(map_len)
                .map_or(true, |end| end > mapping.size())
            {
                return Err(ImportError::InvalidRange(
                    addr.offset(),
                    *len,
                    GuestMemoryError::InvalidGuestAddress(*addr),
                ));
            }
            let fd = mem
                .shm_region(first_page)
                .map_err(|e| access_error(*addr, *len, e))?;

            arena
                .add_fd_offset_protection(pos, map_len, fd, memfd_offset + map_offset as u64, prot)
                .map_err(ImportError::Arena)?;
            pos += map_len;
        }

        let start_offset = (sgs[0].0.offset() & page_mask) as usize;

        Ok(GuestArenaMapping {
            arena,
            start_offset,
            len: total_len,
        })
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    pub fn as_ptr(&self) -> *const u8 {
        // SAFETY: the arena has a valid pointer that covers `start_offset + len`.
        unsafe { self.arena.as_ptr().add(self.start_offset) }
    }

    pub fn as_mut_ptr(&mut self) -> *mut u8 {
        // SAFETY: the arena has a valid pointer that covers `start_offset + len`.
        unsafe { self.arena.as_ptr().add(self.start_offset) }
    }
}

/// Copy of sparse guest memory that is written back upon destruction.
///
/// Contrary to `GuestArenaMapping` which re-maps guest memory to make it appear linear to the
/// host, this copies the sparse guest memory into a linear vector that is copied back upon
/// destruction. Doing so can be faster than a costly mapping operation if the guest area is small
/// enough.
pub struct GuestShadowMapping {
    /// Sparse data copied from the guest.
    data: Vec<u8>,
    /// Guest memory to read from.
    mem: GuestMemory,
    /// SG entries describing the sparse guest area.
    sgs: Vec<(GuestAddress, usize)>,
    /// Whether the data has potentially been modified and requires to be written back to the
    /// guest.
    dirty: bool,
}

impl GuestShadowMapping {
    pub fn new(
        mem: &GuestMemory,
        sgs: Vec<(GuestAddress, usize)>,
        policy: &HostAccessPolicy,
    ) -> Result<Self, ImportError> {
        let mut total_size = 0usize;
        for (_, len) in &sgs {
            total_size = total_size.checked_add(*len).ok_or(ImportError::Overflow)?;
        }
        if total_size == 0 {
            return Err(ImportError::Empty);
        }
        let mut data = vec![0u8; total_size];
        let mut pos = 0;
        for (addr, len) in &sgs {
            // Same gate as the arena's, before the copy touches anything.
            policy.check(mem, *addr, *len)?;
            mem.read_exact_at_addr(&mut data[pos..pos + len], *addr)
                .map_err(|e| access_error(*addr, *len, e))?;
            pos += len;
        }

        Ok(Self {
            data,
            mem: mem.clone(),
            sgs,
            dirty: false,
        })
    }

    pub fn len(&self) -> usize {
        self.data.len()
    }

    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    pub fn as_ptr(&self) -> *const u8 {
        self.data.as_ptr()
    }

    pub fn as_mut_ptr(&mut self) -> *mut u8 {
        self.dirty = true;
        self.data.as_mut_ptr()
    }
}

/// Write the potentially modified shadow buffer back into the guest memory.
impl Drop for GuestShadowMapping {
    fn drop(&mut self) {
        // No need to copy back if no modification has been done.
        if !self.dirty {
            return;
        }

        let mut pos = 0;
        for (addr, len) in &self.sgs {
            if let Err(e) = self
                .mem
                .write_all_at_addr(&self.data[pos..pos + len], *addr)
            {
                base::error!("failed to write back guest memory shadow mapping: {:#}", e);
            }
            pos += len;
        }
    }
}

/// A chunk of guest memory which can be either directly mapped, or copied into a shadow buffer.
pub enum GuestMemoryChunk {
    Mapping(GuestArenaMapping),
    Shadow(GuestShadowMapping),
}

impl GuestMemoryChunk {
    pub fn as_ptr(&self) -> *const u8 {
        match self {
            GuestMemoryChunk::Mapping(m) => m.as_ptr(),
            GuestMemoryChunk::Shadow(s) => s.as_ptr(),
        }
    }

    pub fn as_mut_ptr(&mut self) -> *mut u8 {
        match self {
            GuestMemoryChunk::Mapping(m) => m.as_mut_ptr(),
            GuestMemoryChunk::Shadow(s) => s.as_mut_ptr(),
        }
    }

    pub fn len(&self) -> usize {
        match self {
            GuestMemoryChunk::Mapping(m) => m.len(),
            GuestMemoryChunk::Shadow(s) => s.len(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }
}

/// A guest-owned buffer imported into the host: its scatter-gather list, a linear host view of
/// the bytes, and, once asked for, a dma-buf over the same pages.
///
/// The dma-buf is built lazily because v1 has nobody to hand it to (`VPU_DESIGN.md` §3.4): the
/// CPU path must never pay for it, and must keep working when `/dev/udmabuf` cannot be opened.
/// While a dma-buf exists the pages are referenced in the pool grant table
/// (`GuestMemory::pool_ref_iovecs`) so a growable pool cannot reclaim them underneath it; the
/// reference is dropped with the import. For a fully pre-shared pool (`step_size == 0`) the
/// reference is a no-op.
pub struct GuestBufferImport {
    mem: GuestMemory,
    sgs: Vec<(GuestAddress, usize)>,
    mapping: GuestMemoryChunk,
    dmabuf: OnceCell<SafeDescriptor>,
}

impl GuestBufferImport {
    /// Import `sgs`, mapped with `prot` (the direction the device will access it in: read-only
    /// for an OUTPUT buffer, read-write for a CAPTURE one; a caller that cannot tell passes
    /// read-write), once `policy` has agreed that the host may touch every entry.
    pub fn new(
        mem: &GuestMemory,
        sgs: Vec<(GuestAddress, usize)>,
        prot: Protection,
        policy: &HostAccessPolicy,
    ) -> Result<Self, ImportError> {
        let mut total_size = 0usize;
        for (_, len) in &sgs {
            total_size = total_size.checked_add(*len).ok_or(ImportError::Overflow)?;
        }
        if total_size == 0 {
            return Err(ImportError::Empty);
        }

        let mapping = if total_size >= MAPPING_THRESHOLD {
            GuestMemoryChunk::Mapping(GuestArenaMapping::new(mem, &sgs, prot, policy)?)
        } else {
            GuestMemoryChunk::Shadow(GuestShadowMapping::new(mem, sgs.clone(), policy)?)
        };

        Ok(Self {
            mem: mem.clone(),
            sgs,
            mapping,
            dmabuf: OnceCell::new(),
        })
    }

    /// The guest ranges this buffer is made of.
    pub fn sgs(&self) -> &[(GuestAddress, usize)] {
        &self.sgs
    }

    pub fn len(&self) -> usize {
        self.mapping.len()
    }

    pub fn is_empty(&self) -> bool {
        self.mapping.is_empty()
    }

    pub fn as_ptr(&self) -> *const u8 {
        self.mapping.as_ptr()
    }

    pub fn as_mut_ptr(&mut self) -> *mut u8 {
        self.mapping.as_mut_ptr()
    }

    /// Whether the linear view is a shadow copy (written back to the guest on drop) rather than
    /// a mapping of the guest's own pages.
    pub fn is_shadowed(&self) -> bool {
        matches!(self.mapping, GuestMemoryChunk::Shadow(_))
    }

    /// A dma-buf over the buffer's pages, built on first use with `driver` and kept for the life
    /// of the import. Every entry must be page-aligned in address and length, which a driver-owned
    /// buffer's entries are.
    pub fn dmabuf(&self, driver: &UdmabufDriver) -> Result<&SafeDescriptor, ImportError> {
        if let Some(fd) = self.dmabuf.get() {
            return Ok(fd);
        }

        // Reference the pool pages first, so a growable pool cannot give them away between the
        // check and the import; let go again if the import fails.
        self.mem
            .pool_ref_iovecs(&self.sgs)
            .map_err(|e| ImportError::Grant(format!("{:?}", e)))?;
        let fd = match driver.create_udmabuf(&self.mem, &self.sgs) {
            Ok(fd) => fd,
            Err(e) => {
                self.mem.pool_unref_iovecs(&self.sgs);
                return Err(ImportError::Udmabuf(e));
            }
        };
        // `set` can only fail if something was set meanwhile, which `get` above rules out.
        let _ = self.dmabuf.set(fd);
        Ok(self.dmabuf.get().expect("dma-buf was just set"))
    }
}

impl Drop for GuestBufferImport {
    fn drop(&mut self) {
        // The dma-buf goes before its grant does.
        if let Some(fd) = self.dmabuf.take() {
            drop(fd);
            self.mem.pool_unref_iovecs(&self.sgs);
        }
    }
}

#[cfg(test)]
mod tests {
    use std::fs::File;

    use base::MemoryMappingBuilder;

    use super::*;

    /// Guest RAM the way the aarch64 layout has it: starting at `0x8000_0000`, not at 0, so an
    /// implementation that mistakes an address for an offset is caught. Memfd-backed, like the
    /// real thing, which is what udmabuf needs.
    const RAM_BASE: u64 = 0x8000_0000;

    /// The in-VMM policy: the `GuestMemory` answers for itself.
    const VMM: HostAccessPolicy = HostAccessPolicy::GuestMemory;

    fn guest_memory(pages: u64) -> GuestMemory {
        GuestMemory::new(&[(GuestAddress(RAM_BASE), pages * pagesize() as u64)]).unwrap()
    }

    fn pattern(len: usize, seed: usize) -> Vec<u8> {
        (0..len).map(|i| ((i + seed) * 13 % 251) as u8).collect()
    }

    #[test]
    fn arena_maps_sparse_pages_linearly_and_writes_through() {
        let page = pagesize();
        let mem = guest_memory(8);
        // Two separate runs of guest pages, filled with distinct patterns.
        let a = GuestAddress(RAM_BASE + page as u64);
        let b = GuestAddress(RAM_BASE + 5 * page as u64);
        mem.write_all_at_addr(&pattern(2 * page, 1), a).unwrap();
        mem.write_all_at_addr(&pattern(page, 2), b).unwrap();

        let sgs = vec![(a, 2 * page), (b, page)];
        let mut import = GuestBufferImport::new(&mem, sgs, Protection::read_write(), &VMM).unwrap();
        assert!(!import.is_shadowed());
        assert_eq!(import.len(), 3 * page);

        // SAFETY: the import maps `len` bytes.
        let view = unsafe { std::slice::from_raw_parts(import.as_ptr(), import.len()) };
        assert_eq!(&view[..2 * page], &pattern(2 * page, 1)[..]);
        assert_eq!(&view[2 * page..], &pattern(page, 2)[..]);

        // A write through the arena lands in the guest's pages, in the right place.
        // SAFETY: as above, mutably.
        let view = unsafe { std::slice::from_raw_parts_mut(import.as_mut_ptr(), import.len()) };
        view[2 * page + 7] = 0xee;
        let mut byte = [0u8; 1];
        mem.read_exact_at_addr(&mut byte, GuestAddress(b.offset() + 7))
            .unwrap();
        assert_eq!(byte[0], 0xee);
        // ... and not in the page before it.
        mem.read_exact_at_addr(&mut byte, GuestAddress(b.offset() - page as u64 + 7))
            .unwrap();
        assert_eq!(byte[0], 0);
    }

    #[test]
    fn unaligned_first_entry_keeps_its_offset() {
        let page = pagesize();
        let mem = guest_memory(4);
        let start = GuestAddress(RAM_BASE + page as u64 + 0x100);
        mem.write_all_at_addr(&pattern(0x800, 3), start).unwrap();

        // Large enough to be mapped, not shadowed, and ending mid-page (last entry may).
        let import =
            GuestBufferImport::new(&mem, vec![(start, 0x800)], Protection::read(), &VMM).unwrap();
        assert!(!import.is_shadowed());
        // SAFETY: the import maps `len` bytes.
        let view = unsafe { std::slice::from_raw_parts(import.as_ptr(), import.len()) };
        assert_eq!(view, &pattern(0x800, 3)[..]);
    }

    #[test]
    fn shadow_below_threshold_writes_back_on_drop() {
        let page = pagesize();
        let mem = guest_memory(4);
        let start = GuestAddress(RAM_BASE + 2 * page as u64 + 0x40);
        mem.write_all_at_addr(&pattern(0x100, 4), start).unwrap();

        let mut import =
            GuestBufferImport::new(&mem, vec![(start, 0x100)], Protection::read_write(), &VMM)
                .unwrap();
        assert!(import.is_shadowed());
        // SAFETY: the shadow holds `len` bytes.
        let view = unsafe { std::slice::from_raw_parts_mut(import.as_mut_ptr(), import.len()) };
        assert_eq!(view, &pattern(0x100, 4)[..]);
        view[0x80] = 0x77;
        // Not in the guest yet...
        let mut byte = [0u8; 1];
        mem.read_exact_at_addr(&mut byte, GuestAddress(start.offset() + 0x80))
            .unwrap();
        assert_ne!(byte[0], 0x77);
        // ... until the import goes away.
        drop(import);
        mem.read_exact_at_addr(&mut byte, GuestAddress(start.offset() + 0x80))
            .unwrap();
        assert_eq!(byte[0], 0x77);
    }

    #[test]
    fn bad_lists_are_einval() {
        let page = pagesize();
        let mem = guest_memory(4);
        let a = GuestAddress(RAM_BASE + page as u64);

        let e = GuestBufferImport::new(&mem, vec![], Protection::read(), &VMM)
            .err()
            .expect("import should have failed");
        assert_eq!(e.errno(), libc::EINVAL);

        // A non-initial entry that is not page aligned cannot be made linear.
        let e = GuestBufferImport::new(
            &mem,
            vec![
                (a, page),
                (GuestAddress(a.offset() + page as u64 + 8), page),
            ],
            Protection::read(),
            &VMM,
        )
        .err()
        .expect("import should have failed");
        assert_eq!(e.errno(), libc::EINVAL);

        // Outside guest memory.
        let e = GuestBufferImport::new(
            &mem,
            vec![(GuestAddress(RAM_BASE + 64 * page as u64), page)],
            Protection::read(),
            &VMM,
        )
        .err()
        .expect("import should have failed");
        assert_eq!(e.errno(), libc::EINVAL);
    }

    /// In a protected VM, plain guest RAM is lent, not shared: the host must not map it, and the
    /// guest gets `EFAULT` rather than crosvm getting a `SIGBUS`.
    #[test]
    fn lent_memory_is_efault() {
        let page = pagesize();
        let mem = guest_memory(4);
        let a = GuestAddress(RAM_BASE + page as u64);
        assert!(GuestBufferImport::new(&mem, vec![(a, page)], Protection::read(), &VMM).is_ok());
        // Small enough to shadow: the copy path must refuse too.
        assert!(GuestBufferImport::new(&mem, vec![(a, 0x100)], Protection::read(), &VMM).is_ok());

        mem.set_protected();
        let e = GuestBufferImport::new(&mem, vec![(a, page)], Protection::read(), &VMM)
            .err()
            .expect("import should have failed");
        assert_eq!(e.errno(), libc::EFAULT, "{e}");
        let e = GuestBufferImport::new(&mem, vec![(a, 0x100)], Protection::read(), &VMM)
            .err()
            .expect("import should have failed");
        assert_eq!(e.errno(), libc::EFAULT, "{e}");
    }

    /// The dma-buf udmabuf builds over the guest pages shows the same bytes as the arena.
    /// Skipped, loudly, where `/dev/udmabuf` cannot be opened.
    #[test]
    fn udmabuf_and_arena_see_the_same_bytes() {
        let driver = match UdmabufDriver::new() {
            Ok(driver) => driver,
            Err(e) => {
                eprintln!("skipping: cannot open /dev/udmabuf ({e:?})");
                return;
            }
        };
        let page = pagesize();
        let mem = guest_memory(8);
        let a = GuestAddress(RAM_BASE + page as u64);
        let b = GuestAddress(RAM_BASE + 6 * page as u64);
        mem.write_all_at_addr(&pattern(page, 5), a).unwrap();
        mem.write_all_at_addr(&pattern(page, 6), b).unwrap();

        let import = GuestBufferImport::new(
            &mem,
            vec![(a, page), (b, page)],
            Protection::read_write(),
            &VMM,
        )
        .unwrap();
        let fd = import.dmabuf(&driver).expect("udmabuf import failed");
        // Asking again is free and gives the same descriptor.
        let again = import.dmabuf(&driver).unwrap();
        assert_eq!(
            base::AsRawDescriptor::as_raw_descriptor(fd),
            base::AsRawDescriptor::as_raw_descriptor(again)
        );

        let file = File::from(fd.try_clone().unwrap());
        let mapping = MemoryMappingBuilder::new(2 * page)
            .from_file(&file)
            .build()
            .unwrap();
        // SAFETY: the mapping is `2 * page` bytes.
        let via_dmabuf = unsafe { std::slice::from_raw_parts(mapping.as_ptr(), 2 * page) };
        // SAFETY: the import maps `len` bytes.
        let via_arena = unsafe { std::slice::from_raw_parts(import.as_ptr(), import.len()) };
        assert_eq!(via_dmabuf, via_arena);
        assert_eq!(&via_dmabuf[..page], &pattern(page, 5)[..]);
        assert_eq!(&via_dmabuf[page..], &pattern(page, 6)[..]);
    }

    /// The helper's policy: a window list the VMM computed, consulted before anything is mapped.
    /// Inside a window the import works as in the VMM; outside, or straddling a window's end, it
    /// is `EFAULT` -- and nothing was mapped or copied to find that out.
    #[test]
    fn windows_policy_refuses_what_is_outside_them() {
        let page = pagesize();
        // Eight pages of guest memory, all of which are really mapped in this process (as they
        // are in a helper), of which the VMM allows only pages 2..=5.
        let mem = guest_memory(8);
        let allowed_start = RAM_BASE + 2 * page as u64;
        let policy = HostAccessPolicy::Windows(vec![AddressRange::from_start_and_size(
            allowed_start,
            4 * page as u64,
        )
        .unwrap()]);
        let inside = GuestAddress(allowed_start + page as u64);
        mem.write_all_at_addr(&pattern(page, 7), inside).unwrap();

        // Inside: both the mapped and the shadowed shape.
        let import =
            GuestBufferImport::new(&mem, vec![(inside, page)], Protection::read(), &policy)
                .unwrap();
        // SAFETY: the import maps `len` bytes.
        let view = unsafe { std::slice::from_raw_parts(import.as_ptr(), import.len()) };
        assert_eq!(view, &pattern(page, 7)[..]);
        assert!(
            GuestBufferImport::new(&mem, vec![(inside, 0x100)], Protection::read(), &policy)
                .unwrap()
                .is_shadowed()
        );

        // Outside: page 0 is guest memory the helper could map, and must not.
        let outside = GuestAddress(RAM_BASE);
        for len in [page, 0x100] {
            let e = GuestBufferImport::new(&mem, vec![(outside, len)], Protection::read(), &policy)
                .err()
                .expect("import outside the windows should have failed");
            assert_eq!(e.errno(), libc::EFAULT, "{e}");
        }

        // Straddling: starts on the last allowed page and runs one page past the window.
        let last = GuestAddress(allowed_start + 3 * page as u64);
        let e = GuestBufferImport::new(&mem, vec![(last, 2 * page)], Protection::read(), &policy)
            .err()
            .expect("import straddling the window end should have failed");
        assert_eq!(e.errno(), libc::EFAULT, "{e}");
        // Exactly up to the window's end is fine.
        assert!(
            GuestBufferImport::new(&mem, vec![(last, page)], Protection::read(), &policy).is_ok()
        );
        // One good entry and one bad one: the whole list is refused.
        let e = GuestBufferImport::new(
            &mem,
            vec![(inside, page), (outside, page)],
            Protection::read(),
            &policy,
        )
        .err()
        .expect("a list with an entry outside the windows should have failed");
        assert_eq!(e.errno(), libc::EFAULT, "{e}");

        // Outside every window, an address is EFAULT whether or not it is memory: the guest
        // named something the host may not touch, and that is the whole of the answer.
        let nowhere = GuestAddress(RAM_BASE + 64 * page as u64);
        let e = GuestBufferImport::new(&mem, vec![(nowhere, page)], Protection::read(), &policy)
            .err()
            .expect("import of non-memory should have failed");
        assert_eq!(e.errno(), libc::EFAULT, "{e}");
        // Inside a window that covers no memory -- a window list is what the VMM says, not a
        // promise that pages exist -- it is not memory, and stays EINVAL as in the VMM.
        let generous = HostAccessPolicy::Windows(vec![AddressRange::from_start_and_size(
            RAM_BASE,
            128 * page as u64,
        )
        .unwrap()]);
        for policy in [&generous, &VMM] {
            let e = GuestBufferImport::new(&mem, vec![(nowhere, page)], Protection::read(), policy)
                .err()
                .expect("import of non-memory should have failed");
            assert_eq!(e.errno(), libc::EINVAL, "{e}");
        }
    }

    /// What the VMM hands a helper: every region when the guest's memory is not lent, and only
    /// the SHARE'd purposes when it is. Plain guest RAM is lent in a protected VM, so it drops
    /// out; on aarch64 the pools and the swiotlb region stay.
    #[test]
    fn accessible_windows_follow_the_protected_vm_rule() {
        use vm_memory::MemoryRegionOptions;
        use vm_memory::MemoryRegionPurpose;

        let page = pagesize() as u64;
        let ram = (GuestAddress(RAM_BASE), 4 * page, MemoryRegionOptions::new());
        #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
        let mem = GuestMemory::new_with_options(&[
            ram,
            (
                GuestAddress(RAM_BASE + 4 * page),
                2 * page,
                MemoryRegionOptions::new().purpose(MemoryRegionPurpose::StaticSwiotlbRegion),
            ),
            (
                GuestAddress(RAM_BASE + 8 * page),
                2 * page,
                MemoryRegionOptions::new().purpose(MemoryRegionPurpose::MediaPool),
            ),
        ])
        .unwrap();
        #[cfg(not(any(target_arch = "arm", target_arch = "aarch64")))]
        let mem = GuestMemory::new_with_options(&[
            ram,
            (
                GuestAddress(RAM_BASE + 4 * page),
                2 * page,
                MemoryRegionOptions::new().purpose(MemoryRegionPurpose::ReservedMemory),
            ),
        ])
        .unwrap();

        let everything: Vec<(u64, u64)> = host_accessible_windows(&mem, false)
            .into_iter()
            .map(|w| (w.start, w.end))
            .collect();
        let all_regions: Vec<(u64, u64)> = mem
            .regions()
            .map(|r| {
                (
                    r.guest_addr.offset(),
                    r.guest_addr.offset() + r.size as u64 - 1,
                )
            })
            .collect();
        assert_eq!(everything, all_regions);

        let lent: Vec<(u64, u64)> = host_accessible_windows(&mem, true)
            .into_iter()
            .map(|w| (w.start, w.end))
            .collect();
        #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
        assert_eq!(
            lent,
            vec![
                (RAM_BASE + 4 * page, RAM_BASE + 6 * page - 1),
                (RAM_BASE + 8 * page, RAM_BASE + 10 * page - 1),
            ]
        );
        #[cfg(not(any(target_arch = "arm", target_arch = "aarch64")))]
        assert!(lent.is_empty(), "{lent:?}");
    }
}
