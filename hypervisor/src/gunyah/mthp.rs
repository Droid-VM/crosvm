// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Derived from QEMU (GPL-2.0-or-later): gunyah_add_mem() in gunyah-all.c.
// The additional permissions do not extend to QEMU copyright in this file.
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

//! Multi-size Transparent Huge Page (mTHP) preparation and LEND chunking
//! for Gunyah memory regions.
//!
//! Ported from QEMU gunyah-all.c `gunyah_add_mem()` which addresses several
//! Gunyah hypervisor memory defects:
//!
//! 1. Without THPs, an 8 GB guest needs ~2M page-table entries and exhausts
//!    the hypervisor's fixed-size page-table pool → ENOMEM crash.
//! 2. The kernel's `gunyah_gup_share_parcel()` calls `kcalloc()` for the
//!    entire region; for 8 GB that is 16 MB contiguous kernel memory which
//!    always fails on phones.  Splitting into 256 MB chunks keeps each
//!    `kcalloc` at ~512 KB.
//! 3. Demand-paging after LEND only pins one page at a time, missing THP.

use std::fs;
use std::io::Write;

use base::info;
use base::warn;

// ── constants ────────────────────────────────────────────────────────────────

const THP_SIZE: u64 = 2 * 1024 * 1024; // 2 MB
const MAP_UNIT: u64 = 64 * 1024; // 64 KB – smallest collapse unit
const BATCH_SIZE: u64 = 64 * 1024 * 1024; // 64 MB populate batch
const COMPACT_INTERVAL: usize = 4; // compact every N batches (256 MB)

/// Maximum size of a single LEND ioctl – keeps kcalloc at ~512 KB.
pub const LEND_CHUNK_SIZE: u64 = 256 * 1024 * 1024; // 256 MB

#[allow(dead_code)]
const MADV_POPULATE_WRITE: i32 = 23;
#[allow(dead_code)]
const MADV_COLLAPSE: i32 = 25;
const MADV_POPULATE_READ: i32 = 22;

// ── mTHP sizes to enable ────────────────────────────────────────────────────

static MTHP_SIZES: &[&str] = &[
    "16kB", "32kB", "64kB", "128kB", "256kB", "512kB", "1024kB",
];

// ── collapse cascade table ──────────────────────────────────────────────────

struct CollapseLevel {
    size: u64,
    order: u8,
    name: &'static str,
}

static COLLAPSE_LEVELS: &[CollapseLevel] = &[
    CollapseLevel { size: 2 * 1024 * 1024, order: 9, name: "2MB" },
    CollapseLevel { size: 1024 * 1024, order: 8, name: "1MB" },
    CollapseLevel { size: 512 * 1024, order: 7, name: "512KB" },
    CollapseLevel { size: 256 * 1024, order: 6, name: "256KB" },
    CollapseLevel { size: 128 * 1024, order: 5, name: "128KB" },
    CollapseLevel { size: 64 * 1024, order: 4, name: "64KB" },
];

// ── helpers ─────────────────────────────────────────────────────────────────

fn write_file(path: &str, value: &str) -> bool {
    match fs::OpenOptions::new().write(true).open(path) {
        Ok(mut f) => {
            let _ = f.write_all(value.as_bytes());
            true
        }
        Err(_) => false,
    }
}

fn trigger_compact() {
    write_file("/proc/sys/vm/compact_memory", "1\n");
}

// ── public API ──────────────────────────────────────────────────────────────

/// Result of [`prepare_lend_region`]: carries a per-2MB bitmap indicating
/// which chunks are fully backed by 2 MB THPs (false = needs mTHP/4KB treatment).
pub struct LendPrepResult {
    /// Per-2MB-chunk: `true` means NOT a 2MB THP (i.e. needs small-page LEND).
    pub need_small: Vec<bool>,
    /// Total bytes that were successfully collapsed to ≥ 64 KB folios.
    pub large_page_bytes: u64,
    /// Hard verification that every page in the region is genuinely resolvable
    /// (`MADV_POPULATE_READ` over the whole range succeeded). Phase 2's
    /// MADV_POPULATE_WRITE / manual-touch fallback can both report success while a
    /// custom reserve-pool fault hook silently hands back an unbacked mapping (e.g.
    /// CMA reservoir exhausted); this is the only phase that treats that as fatal so
    /// the caller can fail VM creation instead of the guest SIGBUS-ing minutes later
    /// deep inside its GPU stack (host-alloc must fail loudly, not silently).
    pub populated: bool,
    /// Whether Phase 4's `mlock` succeeded. An unpinned page inside a SHARE'd pool can still be
    /// migrated or reclaimed by the host kernel while the RM's stage-2 mapping keeps pointing at
    /// the page it was blessed with, and neither side notices -- the guest simply reads and
    /// writes memory the GPU no longer shares. Reported rather than acted on here so the caller
    /// can be fatal for the pool purposes and lenient elsewhere.
    pub mlocked: bool,
}

/// Prepare a lend region for Gunyah by maximising large-page backing.
///
/// Implements the four-phase strategy from QEMU's `gunyah_add_mem`:
///   Phase 1 – drop caches, compact, enable mTHP intermediate sizes
///   Phase 2 – populate in 64 MB batches (MADV_POPULATE_WRITE)
///   Phase 3 – cascading MADV_COLLAPSE (2 MB → 64 KB)
///   Phase 4 – mlock
///
/// # Safety
/// `host_addr` must point to a valid memory mapping of at least `size` bytes.
pub unsafe fn prepare_lend_region(host_addr: *mut u8, size: u64) -> LendPrepResult {
    info!(
        "GH: preparing LEND region: hva={:#x} size={:#x} ({} MB)",
        host_addr as u64,
        size,
        size >> 20
    );

    // ── Phase 1: free page-cache, compact, enable mTHP ──────────────

    info!("GH: Phase 1: dropping caches + compacting ...");
    write_file("/proc/sys/vm/drop_caches", "3\n");
    trigger_compact();
    std::thread::sleep(std::time::Duration::from_millis(500));

    // Enable intermediate mTHP sizes
    {
        let mut enabled = 0u32;
        for sz in MTHP_SIZES {
            let path = format!(
                "/sys/kernel/mm/transparent_hugepage/hugepages-{}/enabled",
                sz
            );
            if write_file(&path, "always\n") {
                enabled += 1;
            }
        }
        if enabled > 0 {
            info!(
                "GH: Phase 1: enabled mTHP at {} intermediate sizes (16kB-1024kB)",
                enabled
            );
        } else {
            info!("GH: Phase 1: mTHP not available");
        }
    }

    // Request THPs for the whole region
    let ret = libc::madvise(host_addr as *mut libc::c_void, size as usize, libc::MADV_HUGEPAGE);
    info!(
        "GH: MADV_HUGEPAGE: {}",
        if ret == 0 { "OK" } else { "FAILED" }
    );

    // ── Phase 2: populate in 64 MB batches ──────────────────────────

    {
        let num_batches = (size + BATCH_SIZE - 1) / BATCH_SIZE;
        info!(
            "GH: Phase 2: populating {} MB in {} x {} MB batches ...",
            size >> 20,
            num_batches,
            BATCH_SIZE >> 20
        );

        let mut batch_idx: usize = 0;
        let mut offset: u64 = 0;
        while offset < size {
            let len = std::cmp::min(size - offset, BATCH_SIZE) as usize;
            let ptr = host_addr.add(offset as usize);

            let ret = libc::madvise(ptr as *mut libc::c_void, len, MADV_POPULATE_WRITE);
            if ret != 0 {
                // Fallback: touch each page manually
                let npages = len / 4096;
                for i in 0..npages {
                    let p = ptr.add(i * 4096);
                    std::ptr::write_volatile(p, std::ptr::read_volatile(p));
                }
            }

            batch_idx += 1;
            if batch_idx % COMPACT_INTERVAL == 0 && offset + BATCH_SIZE < size {
                trigger_compact();
                std::thread::sleep(std::time::Duration::from_millis(200));
            }
            offset += BATCH_SIZE;
        }
        info!("GH: Phase 2: population complete");
    }

    // ── Phase 3: cascading MADV_COLLAPSE (2 MB → 64 KB) ────────────

    let map_count = (size / MAP_UNIT) as usize;
    let mut order_map = vec![0u8; map_count];
    let mut large_page_bytes: u64 = 0;

    {
        info!("GH: Phase 3: cascading MADV_COLLAPSE (2MB -> 64KB) ...");

        for level in COLLAPSE_LEVELS {
            let csize = level.size;
            let corder = level.order;
            let units_per_chunk = (csize / MAP_UNIT) as usize;
            let num_chunks = (size / csize) as usize;
            let mut collapsed: u64 = 0;
            let mut skipped: u64 = 0;
            let mut failed: u64 = 0;
            let mut last_err: i32 = 0;

            for ci in 0..num_chunks {
                let map_base = ci * units_per_chunk;

                // Skip if any sub-unit already collapsed
                let all_free = (0..units_per_chunk).all(|u| order_map[map_base + u] == 0);
                if !all_free {
                    skipped += 1;
                    continue;
                }

                let ptr = host_addr.add((ci as u64 * csize) as usize);
                let ret = libc::madvise(ptr as *mut libc::c_void, csize as usize, MADV_COLLAPSE);
                if ret == 0 {
                    for u in 0..units_per_chunk {
                        order_map[map_base + u] = corder;
                    }
                    collapsed += 1;
                } else {
                    failed += 1;
                    last_err = std::io::Error::last_os_error().raw_os_error().unwrap_or(0);
                }
            }

            info!(
                "GH:   {} pass 0: {} OK, {} skipped, {} failed (err={})",
                level.name, collapsed, skipped, failed, last_err
            );
        }

        // Summary
        let mut order_total = [0u64; 10];
        let mut uncollapsed: u64 = 0;
        for &o in &order_map {
            if o > 0 && (o as usize) <= 9 {
                order_total[o as usize] += 1;
            } else {
                uncollapsed += 1;
            }
        }

        info!("GH: Phase 3 done — collapse summary (per 64KB unit):");
        for o in (4..=9).rev() {
            if order_total[o] > 0 {
                let size_kb = 4u64 << o;
                let mb = (order_total[o] * 64) / 1024;
                info!("GH:   {}KB: {} units = {} MB", size_kb, order_total[o], mb);
            }
        }
        info!(
            "GH:   uncollapsed (4KB): {} units = {} MB",
            uncollapsed,
            (uncollapsed * 64) / 1024
        );

        large_page_bytes = (map_count as u64 - uncollapsed) * MAP_UNIT;

        // Re-populate any regions left unpopulated by retry passes
        for mi in 0..map_count {
            if order_map[mi] == 0 {
                let off = mi as u64 * MAP_UNIT;
                let ptr = host_addr.add(off as usize);
                let ret =
                    libc::madvise(ptr as *mut libc::c_void, MAP_UNIT as usize, MADV_POPULATE_WRITE);
                if ret != 0 {
                    let npages = MAP_UNIT / 4096;
                    for pg in 0..npages {
                        let p = ptr.add((pg * 4096) as usize);
                        std::ptr::write_volatile(p, std::ptr::read_volatile(p));
                    }
                }
            }
        }
    }

    info!(
        "GH: === large-page coverage: {} / {} MB ({:.1}%) ===",
        large_page_bytes >> 20,
        size >> 20,
        large_page_bytes as f64 * 100.0 / size as f64
    );

    // ── Phase 4: mlock ──────────────────────────────────────────────

    let ret = libc::mlock(host_addr as *const libc::c_void, size as usize);
    let mlocked = ret == 0;
    if mlocked {
        info!("GH: mlock: OK");
    } else {
        warn!(
            "GH: mlock FAILED: errno={} -- pages in this region can still be migrated or \
             reclaimed while the RM's stage-2 keeps the old ones (check RLIMIT_MEMLOCK)",
            std::io::Error::last_os_error().raw_os_error().unwrap_or(0)
        );
    }

    // ── Build need_small bitmap from order_map ──────────────────────

    let total_chunks = (size / THP_SIZE) as usize;
    let units_per_thp = (THP_SIZE / MAP_UNIT) as usize;
    let mut need_small = vec![false; total_chunks];
    for ci in 0..total_chunks {
        let map_base = ci * units_per_thp;
        // Mark as THP only if ALL 64KB units are order >= 9 (2MB THP)
        let is_thp = (0..units_per_thp).all(|u| order_map[map_base + u] >= 9);
        need_small[ci] = !is_thp;
    }

    // ── Hard verification: every page must genuinely be resolvable ──
    //
    // MADV_POPULATE_READ forces the kernel to resolve (not just request) every page
    // in the range; a custom fault hook (e.g. the reserve-pool supply hook) that
    // cannot serve a page returns an error here instead of silently leaving a hole
    // that only surfaces as a SIGBUS on first guest/host touch. Safe (no crash risk)
    // unlike a manual read/write touch of memory that might not be backed.
    let verify_ret =
        libc::madvise(host_addr as *mut libc::c_void, size as usize, MADV_POPULATE_READ);
    let populated = verify_ret == 0;
    if populated {
        info!("GH: verify (MADV_POPULATE_READ): OK, region fully backed");
    } else {
        warn!(
            "GH: verify (MADV_POPULATE_READ) FAILED: errno={} -- region is NOT fully backed \
             (reserve pool likely exhausted); caller must treat this as fatal",
            std::io::Error::last_os_error().raw_os_error().unwrap_or(0)
        );
    }

    // ── Phase 5: clean the region out of the caches ─────────────────
    //
    // Phases 2-3 wrote every page through the normal cacheable mapping, so those lines are
    // sitting dirty in this CPU's caches. The GPU then reaches the same physical pages either
    // non-coherently or through a write-combining guest mapping, so a line that has not reached
    // the point of coherency can still be written back later, on top of what the guest wrote --
    // or read back as the zeros we populated with. virglrenderer's drm2kgsl backend attributes a
    // hard glmark2 DEVICE LOST to exactly this (the CP executing stale zero lines that alias
    // WC guest writes, giving a type-0 write to register 0 and an AHB error).
    clean_dcache_to_poc(host_addr, size as usize);

    LendPrepResult {
        need_small,
        large_page_bytes,
        populated,
        mlocked,
    }
}

/// Clean+invalidate `size` bytes from `host_addr` to the point of coherency.
#[cfg(target_arch = "aarch64")]
fn clean_dcache_to_poc(host_addr: *mut u8, size: usize) {
    if size == 0 {
        return;
    }
    let ctr: u64;
    // SAFETY: reading CTR_EL0 and cleaning cache lines by VA are permitted at EL0 while Linux
    // sets SCTLR_EL1.UCI, which it always does on arm64. `host_addr..host_addr+size` is a live
    // writable mapping, and `dc civac` neither reads nor writes through the pointer.
    unsafe {
        std::arch::asm!("mrs {ctr}, ctr_el0", ctr = out(reg) ctr);
        // CTR_EL0.DminLine is log2 of the line size in words.
        let line_size = 4usize << ((ctr >> 16) & 0xf);
        let mut addr = (host_addr as usize) & !(line_size - 1);
        let end = host_addr as usize + size;
        while addr < end {
            std::arch::asm!("dc civac, {addr}", addr = in(reg) addr, options(nostack));
            addr += line_size;
        }
        std::arch::asm!("dsb sy", options(nostack));
    }
    info!("GH: dcache clean to PoC: {} MB", size >> 20);
}

#[cfg(not(target_arch = "aarch64"))]
fn clean_dcache_to_poc(_host_addr: *mut u8, _size: usize) {}

/// Fold `[offset, offset+len)` of an already-sized fd into 2 MiB folios, leaving the rest of the
/// file untouched.
///
/// The alignment dance is the same and for the same reason: MADV_COLLAPSE on a file mapping only
/// forms a PMD when the virtual address and the file offset are congruent mod 2 MiB, so the fd is
/// mapped at a 2 MiB-aligned VA with the window's own offset, rather than wherever mmap happens to
/// land it. `offset` and `len` must both be 2 MiB multiples; anything else is rejected rather than
/// silently degraded, because a degraded grant is a parcel with 512x the mem_entries and that
/// failure shows up much later as an order-5 kcalloc failing under fragmentation.
///
/// Returns Ok even when individual collapses fail -- the caller gets 4 KiB backing, which works
/// but is expensive. How much of the range really came back as 2 MiB folios is in the returned
/// [`FolioCoverage`], which a caller who cannot live with a degraded range checks and a caller who
/// only wanted the cheap shape ignores.
///
/// # Safety
/// `fd` must be a live shmem descriptor at least `offset + len` bytes long.
pub unsafe fn folio_back_range(fd: i32, offset: u64, len: u64) -> std::io::Result<FolioCoverage> {
    let err = || std::io::Error::last_os_error();
    if offset % THP_SIZE != 0 || len % THP_SIZE != 0 || len == 0 {
        return Err(std::io::Error::from_raw_os_error(libc::EINVAL));
    }
    // NO fallocate here, deliberately. Punching the range in first allocates it as ordinary 4 KiB
    // shmem, and on these phones the free memory a 4 KiB allocation can reach is whatever the
    // gh_hugepage reserve pool did not park (measured on 8gen3: MemAvailable ~1 GiB against a
    // 5.5 GiB reservoir). So a 1 GiB grant failed with ENOMEM inside fallocate while 5.5 GiB of
    // 2 MiB folios sat unused, and the collapse below would then have had to allocate the folio
    // *and* migrate into it -- twice the peak for a worse result.
    //
    // MADV_POPULATE_WRITE through the mapping, with MADV_HUGEPAGE already set, faults the file
    // pages in as order-9 folios straight from the reserve pool's supply hook: the same path the
    // multi-GiB LEND of guest RAM has always taken (prepare_lend_region).
    let l = len as usize;
    let thp = THP_SIZE as usize;
    let base = libc::mmap(
        std::ptr::null_mut(),
        l + thp,
        libc::PROT_NONE,
        libc::MAP_PRIVATE | libc::MAP_ANONYMOUS | libc::MAP_NORESERVE,
        -1,
        0,
    );
    if base == libc::MAP_FAILED {
        return Err(err());
    }
    let base_u = base as usize;
    let aligned = (base_u + thp - 1) & !(thp - 1);
    // The file offset is a 2 MiB multiple and the VA is 2 MiB-aligned, so they are congruent.
    let mapped = libc::mmap(
        aligned as *mut libc::c_void,
        l,
        libc::PROT_READ | libc::PROT_WRITE,
        libc::MAP_SHARED | libc::MAP_FIXED,
        fd,
        offset as libc::off_t,
    );
    if mapped == libc::MAP_FAILED {
        let e = err();
        libc::munmap(base, l + thp);
        return Err(e);
    }
    if aligned > base_u {
        libc::munmap(base, aligned - base_u);
    }
    let tail_start = aligned + l;
    let tail_end = base_u + l + thp;
    if tail_end > tail_start {
        libc::munmap(tail_start as *mut libc::c_void, tail_end - tail_start);
    }

    let coverage = fold_mapped_range(mapped as *mut u8, len);
    libc::munmap(mapped, l);
    coverage
}

/// Fold an already-mapped shmem range into 2 MiB folios, one window at a time.
///
/// This is the actual work [`folio_back_range`] does; it is separate because the mapping does not
/// always have to be made. A caller that already holds a mapping whose virtual address is congruent
/// with its file offset mod 2 MiB -- guest RAM's own region mapping, and the framebuffer's -- can
/// fold it in place, and gets something the fd-window version cannot give: the range is left
/// POPULATED IN THAT MAPPING. Through a temporary mapping the folios end up in the page cache and
/// the caller's own page tables stay empty, so anything that reads the caller's address space
/// afterwards -- the pin probe, most of all -- sees an absent page and says so.
///
/// `MADV_HUGEPAGE` first, because `shmem_enabled` is `advise` on these phones: without it the fault
/// below allocates 4 KiB pages and there is nothing for the reserve's order-9 hook to intercept.
///
/// # Safety
/// `base` must point at a live shared shmem mapping of at least `len` bytes, 2 MiB-aligned and
/// congruent with its file offset mod 2 MiB.
pub unsafe fn fold_mapped_range(base: *mut u8, len: u64) -> std::io::Result<FolioCoverage> {
    let err = || std::io::Error::last_os_error();
    let _ = libc::madvise(base as *mut libc::c_void, len as usize, libc::MADV_HUGEPAGE);
    let windows = len / THP_SIZE;
    let mut collapsed = 0u64;
    for w in 0..windows {
        let ptr = base.add((w * THP_SIZE) as usize) as *mut libc::c_void;
        if libc::madvise(ptr, THP_SIZE as usize, MADV_POPULATE_WRITE) != 0 {
            let e = err();
            match e.raw_os_error() {
                // Older kernels lack MADV_POPULATE_WRITE; fault each page in by hand so the
                // collapse has something to work with.
                Some(libc::EINVAL) | Some(libc::ENOSYS) => {
                    let npages = THP_SIZE / 4096;
                    for pg in 0..npages {
                        let p = (ptr as *mut u8).add((pg * 4096) as usize);
                        std::ptr::write_volatile(p, std::ptr::read_volatile(p));
                    }
                }
                // Out of memory is the caller's business, and it must not be answered by the
                // hand-fault loop: a write fault that cannot allocate raises SIGBUS, which kills
                // the VMM instead of failing one grant.
                _ => return Err(e),
            }
        }
        if libc::madvise(ptr, THP_SIZE as usize, MADV_COLLAPSE) == 0 {
            collapsed += 1;
        }
    }
    Ok(FolioCoverage { windows, collapsed })
}

/// How much of a range [`folio_back_range`] was asked about came back as 2 MiB folios.
///
/// `collapsed` counts the windows whose `MADV_COLLAPSE` returned success, which includes the ones
/// the populate above had already faulted in as a huge folio -- collapsing an already-huge range
/// succeeds trivially, and on a healthy reserve that is what every window does.
///
/// This says nothing about WHERE those folios came from. `MADV_COLLAPSE` asks for an order-9
/// allocation; the gh_hugepage reserve hook intercepts order-9 and serves it from the pool, but
/// when the pool is empty the buddy allocator answers instead and the folio can be movable or in
/// CMA. So complete coverage is necessary and not sufficient for "the host can leave these pages
/// alone", and a caller who needs that has to ask the pin probe as well.
pub struct FolioCoverage {
    /// 2 MiB windows in the range.
    pub windows: u64,
    /// How many of them are 2 MiB folios.
    pub collapsed: u64,
}

impl FolioCoverage {
    /// Whether every 2 MiB of the range is a 2 MiB folio. An empty range is not complete: there is
    /// nothing there to have succeeded.
    pub fn is_complete(&self) -> bool {
        self.windows > 0 && self.collapsed == self.windows
    }

    /// Bytes that came back as 2 MiB folios.
    pub fn covered_bytes(&self) -> u64 {
        self.collapsed * THP_SIZE
    }

    /// Bytes asked about.
    pub fn total_bytes(&self) -> u64 {
        self.windows * THP_SIZE
    }

    /// Coverage as the percentage the mTHP phases print.
    pub fn pct(&self) -> f64 {
        if self.windows == 0 {
            return 0.0;
        }
        self.collapsed as f64 * 100.0 / self.windows as f64
    }
}

/// An individual chunk to LEND, produced by [`compute_lend_chunks`].
pub struct LendChunk {
    /// Offset from the base of the region.
    pub offset: u64,
    /// Size of this chunk in bytes.
    pub size: u64,
}

/// Split a large LEND region into chunks for the ioctl.
///
/// If `prep` is `Some`, uses the THP-aware bitmap to group contiguous runs
/// of same-backing type and sub-split at 256 MB boundaries.
/// Otherwise falls back to fixed 256 MB chunks.
///
/// Returns an empty vec when the region is small enough for a single slot.
pub fn compute_lend_chunks(total_size: u64, prep: Option<&LendPrepResult>) -> Vec<LendChunk> {
    if total_size <= LEND_CHUNK_SIZE {
        return Vec::new();
    }

    let mut chunks = Vec::new();

    if let Some(prep) = prep {
        // THP-aware splitting
        let total_thp_chunks = (total_size / THP_SIZE) as usize;
        let mut c: usize = 0;

        let thp_ok = prep.need_small.iter().filter(|&&s| !s).count();
        let thp_fail = prep.need_small.iter().filter(|&&s| s).count();
        info!(
            "GH: THP-aware LEND split: {} MB total, {} THP(2MB) chunks, {} mTHP/4KB chunks",
            total_size >> 20,
            thp_ok,
            thp_fail
        );

        while c < total_thp_chunks {
            let is_small = prep.need_small[c];
            let run_start = c;

            // Find contiguous run of same backing type
            while c < total_thp_chunks && prep.need_small[c] == is_small {
                c += 1;
            }

            let run_offset = run_start as u64 * THP_SIZE;
            let run_size = (c - run_start) as u64 * THP_SIZE;

            // Sub-split at 256 MB boundaries
            let mut sub_off: u64 = 0;
            while sub_off < run_size {
                let sub_sz = std::cmp::min(run_size - sub_off, LEND_CHUNK_SIZE);
                chunks.push(LendChunk {
                    offset: run_offset + sub_off,
                    size: sub_sz,
                });
                sub_off += sub_sz;
            }
        }

        info!(
            "GH: THP-aware split done: {} LEND slots",
            chunks.len()
        );
    } else {
        // Fallback: fixed 256 MB chunks
        let num = (total_size + LEND_CHUNK_SIZE - 1) / LEND_CHUNK_SIZE;
        info!(
            "GH: splitting {} MB LEND into {} x {} MB chunks",
            total_size >> 20,
            num,
            LEND_CHUNK_SIZE >> 20
        );

        let mut offset: u64 = 0;
        while offset < total_size {
            let chunk_sz = std::cmp::min(total_size - offset, LEND_CHUNK_SIZE);
            chunks.push(LendChunk {
                offset,
                size: chunk_sz,
            });
            offset += chunk_sz;
        }
    }

    chunks
}
