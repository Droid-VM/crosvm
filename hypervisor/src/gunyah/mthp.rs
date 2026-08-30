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
    if ret == 0 {
        info!("GH: mlock: OK");
    } else {
        warn!("GH: mlock FAILED: errno={}", std::io::Error::last_os_error().raw_os_error().unwrap_or(0));
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

    LendPrepResult {
        need_small,
        large_page_bytes,
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
