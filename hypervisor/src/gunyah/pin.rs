// Copyright 2026 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! Long-term pinning probe for memory that is about to be handed to Gunyah.
//!
//! Every gunyah memory transfer -- the boot-time LEND of guest RAM, the boot SHARE of the GPU
//! pools, and each runtime SHARE of a host-visible blob -- ends in the kernel doing
//! `pin_user_pages_fast(FOLL_LONGTERM)` over the range. A page that sits in a CMA pageblock
//! cannot be pinned that way: the kernel must first migrate it to a non-movable pageblock, and
//! when there is nothing to migrate into, that step fails.
//!
//! On these phones shmem/memfd pages routinely land in CMA -- measured on all three test devices,
//! including the one whose reserve module has its CMA redirect hook turned off, because plain
//! `__GFP_MOVABLE` allocations may use CMA pageblocks. As long as the gh_hugepage reserve pool
//! serves the whole region the question never arises (its folios are already pinnable, and a
//! 3456 MB LEND takes ~60 ms). The moment the pool is short, the shortfall comes from ordinary
//! movable memory, part of it CMA, and the failure surfaces *inside* the gunyah ioctl -- where
//! the observed outcomes were a 2.5-minute whole-host stall ending in the kernel OOM-killing
//! crosvm, and a `qcom_scm: Assign memory protection call failed -22` that reset the phone.
//!
//! The cheap question is not "pin it" but "would pinning have to migrate anything?", and that
//! is a property of the pages: `/dev/gh_pinprobe` (gh_unmovable.ko) samples one page per 2MB and
//! reports how many sit in a CMA / isolate / ZONE_MOVABLE pageblock. It takes no long-term
//! reference and migrates nothing, so asking costs microseconds and cannot itself push a tight
//! system over. A pool-served region answers zero and goes straight to the ioctl.
//!
//! Only when the probe says migration WOULD be needed is there a decision to make, and the
//! default is to refuse -- that region came from outside the reserve pool, which is exactly the
//! condition that produced the two failures above. `GUNYAH_PIN_POLICY=fix` instead does the
//! migration deliberately, by taking the pin ourselves first, where the error is ours to handle:
//!
//!   * it forces exactly the same migration, at a cost measured at 0.1-0.85 s per 512 MB when
//!     pages really are in CMA and ~10 ms when they are not;
//!   * it fails as a plain `-ENOMEM` we can turn into "VM start refused" or "this blob request
//!     failed", instead of a hypervisor call we cannot take back;
//!   * and it leaves the pages migrated out of CMA, so the gunyah pin that follows needs no
//!     migration at all.
//!
//! The pin is released as soon as gunyah owns the memory (the LEND/SHARE ioctl has returned, and
//! for runtime blobs the guest has accepted or failed to accept). Migration is not undone by
//! unpinning -- the pages stay where they were moved to -- so the pin is a lever, not a lease.
//! Holding it any longer would only add a second owner to the release path, and a slow release
//! is what this whole mechanism exists to avoid.
//!
//! The pinning is done through io_uring's `IORING_REGISTER_BUFFERS`, which is
//! `pin_user_pages(FOLL_WRITE | FOLL_LONGTERM)` and nothing else; `IORING_UNREGISTER_BUFFERS`
//! (and closing the ring) is `unpin_user_pages`. That keeps this entirely in userspace: no new
//! kernel module, no new ioctl, and identical behaviour on the 6.1 / 6.6 / 6.12 KMIs (verified
//! present and permitted on all three test devices).
//!
//! Environment overrides, for bringing this up and for testing the refusal paths:
//!   * `GUNYAH_PIN=0`            -- skip all of this (pre-probe behaviour).
//!   * `GUNYAH_PIN_POLICY=fix`   -- when the probe reports unpinnable pages, migrate them (by
//!                                  pinning) instead of refusing.
//!   * `GUNYAH_PIN_FAIL=preboot` -- pretend every pre-boot probe fails.
//!   * `GUNYAH_PIN_FAIL=share`   -- pretend every runtime-share probe fails.
//!   * `GUNYAH_PIN_FAIL=all`     -- both.

use std::fs::File;
use std::io::Error as IoError;
use std::os::fd::FromRawFd;
use std::os::fd::RawFd;
use std::time::Instant;

use base::error;
use base::info;
use base::warn;
use base::AsRawDescriptor;

use base::Error;
use base::Result;

// aarch64 syscall numbers; io_uring is not in libc's aarch64-android bindings.
const SYS_IO_URING_SETUP: libc::c_long = 425;
const SYS_IO_URING_REGISTER: libc::c_long = 427;

const IORING_REGISTER_BUFFERS: libc::c_uint = 0;
const IORING_UNREGISTER_BUFFERS: libc::c_uint = 1;

/// One iovec per chunk. A single registered buffer costs the kernel one `page*` array
/// (8 bytes per 4 KiB), so 3456 MB in one go would be a 7 MB kvmalloc right when memory is
/// tight; 512 MB chunks keep each allocation at 1 MB. The register call takes them all at once,
/// so this costs nothing in round trips.
const CHUNK_BYTES: u64 = 512 << 20;

/// How many times a pre-boot region is re-probed while the reserve catches up, and how long to
/// wait between looks. The reserve returns a stopped VM's pages in about two seconds, so five
/// looks a second apart covers the relaunch case with room to spare without making a genuinely
/// short pool take noticeably longer to refuse.
const PREBOOT_PROBE_ATTEMPTS: u32 = 5;
const PREBOOT_PROBE_WAIT: std::time::Duration = std::time::Duration::from_secs(1);

/// Which call site a probe belongs to. Only used for logging and `GUNYAH_PIN_FAIL`.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum PinSite {
    /// Boot-time regions: guest RAM (LEND) and the GPU pools (SHARE). A failure here refuses
    /// VM start.
    PreBoot,
    /// A runtime host-visible blob (SHARE). A failure here fails that one request.
    Share,
}

impl PinSite {
    fn as_str(self) -> &'static str {
        match self {
            PinSite::PreBoot => "preboot",
            PinSite::Share => "share",
        }
    }
}

#[repr(C)]
#[derive(Default)]
struct IoSqringOffsets {
    head: u32,
    tail: u32,
    ring_mask: u32,
    ring_entries: u32,
    flags: u32,
    dropped: u32,
    array: u32,
    resv1: u32,
    user_addr: u64,
}

#[repr(C)]
#[derive(Default)]
struct IoCqringOffsets {
    head: u32,
    tail: u32,
    ring_mask: u32,
    ring_entries: u32,
    overflow: u32,
    cqes: u32,
    flags: u32,
    resv1: u32,
    user_addr: u64,
}

#[repr(C)]
#[derive(Default)]
struct IoUringParams {
    sq_entries: u32,
    cq_entries: u32,
    flags: u32,
    sq_thread_cpu: u32,
    sq_thread_idle: u32,
    features: u32,
    wq_fd: u32,
    resv: [u32; 3],
    sq_off: IoSqringOffsets,
    cq_off: IoCqringOffsets,
}

/// `struct gh_pinprobe_range` from gh_unmovable.ko. Layout must match byte for byte.
#[repr(C)]
#[derive(Default, Debug)]
struct GhPinprobeRange {
    addr: u64,
    len: u64,
    samples: u64,
    samples_cma: u64,
    samples_isolate: u64,
    samples_movable: u64,
    samples_absent: u64,
    first_bad_offset: u64,
    sample_bytes: u32,
    flags: u32,
}

base::ioctl_iowr_nr!(GH_PINPROBE_RANGE, 'P' as u32, 1, GhPinprobeRange);

/// What the read-only probe found. `bad` is the number of 2MB samples that would have to be
/// migrated before a `FOLL_LONGTERM` pin could succeed.
struct ProbeVerdict {
    bad: u64,
    samples: u64,
    cma: u64,
    isolate: u64,
    movable: u64,
    absent: u64,
    first_bad_offset: u64,
}

/// Asks `/dev/gh_pinprobe`. `Ok(None)` means the node is not there (older module): the caller
/// falls back to proving pinnability the expensive way.
fn probe_range(host_addr: u64, size: u64) -> Option<ProbeVerdict> {
    let dev = File::open("/dev/gh_pinprobe").ok()?;
    let mut req = GhPinprobeRange {
        addr: host_addr,
        len: size,
        ..Default::default()
    };
    // SAFETY: the ioctl reads the request and writes the counters back into `req`.
    let ret = unsafe { base::ioctl_with_mut_ref(&dev, GH_PINPROBE_RANGE, &mut req) };
    if ret != 0 {
        warn!(
            "GH-PIN: /dev/gh_pinprobe ioctl failed ({}), falling back to pinning",
            IoError::last_os_error()
        );
        return None;
    }
    Some(ProbeVerdict {
        bad: req.samples_cma + req.samples_isolate + req.samples_movable,
        samples: req.samples,
        cma: req.samples_cma,
        isolate: req.samples_isolate,
        movable: req.samples_movable,
        absent: req.samples_absent,
        first_bad_offset: req.first_bad_offset,
    })
}

/// What the read-only probe can say about a range that is about to be handed to something which
/// will hold ordinary page references on it (today: the framebuffer's udmabuf).
///
/// This is the same question `ensure_pinnable` asks, minus the decision. `ensure_pinnable` is
/// allowed to answer an untidy range by migrating it; a caller who is not going to pin, and whose
/// consumer will instead make the pages *unmigratable* by referencing them, has no such move
/// available. All it can do is find out, and either proceed or stay off that path.
pub(crate) enum Settled {
    /// Every sample is present and outside CMA / isolate / ZONE_MOVABLE. Nothing has to move for
    /// a later `FOLL_LONGTERM` pin to succeed, so a reference taken now cannot block one.
    Yes,
    /// At least one sample is absent, or sits where a pin would have to migrate it. The prose is
    /// the counters, for the line that explains the refusal.
    No(String),
    /// `/dev/gh_pinprobe` is not there (older gh_unmovable.ko), so nothing can be said either way.
    Unknown,
}

/// Ask the probe whether `size` bytes at `host_addr` are settled where the host can leave them.
///
/// Reads one page per 2 MB through the module's own walk: it takes no reference and migrates
/// nothing, so asking is free and cannot itself disturb what it is measuring.
pub(crate) fn probe_settled(host_addr: u64, size: u64) -> Settled {
    let Some(v) = probe_range(host_addr, size) else {
        return Settled::Unknown;
    };
    if v.samples == 0 {
        return Settled::No("the probe sampled nothing".to_string());
    }
    if v.bad == 0 && v.absent == 0 {
        return Settled::Yes;
    }
    Settled::No(format!(
        "{}/{} 2MB samples would have to move (cma={} isolate={} movable={}) and {} had no page \
         present, first at +{:#x}; CmaFree {} kB",
        v.bad, v.samples, v.cma, v.isolate, v.movable, v.absent, v.first_bad_offset, cma_free_kb(),
    ))
}

/// Share of sampled 2 MB folios that may be off-pool before migration stops being the cheap
/// answer. A handful is the ordinary case -- populate can fall back to 4 KB faults under
/// fragmentation and those pages can land in CMA, which the collapse pass then mostly repairs;
/// measured on device, the one failure in a 96-boot sweep was 151 of 1725 samples, 8.8%. A large
/// share means something else is wrong (the reserve is genuinely empty), and migrating a
/// gigabyte to find that out is worse than saying so.
const AUTO_FIX_MAX_BAD_PCT: u64 = 10;

/// What to do when the probe finds pages that would have to be migrated.
#[derive(PartialEq)]
enum PinPolicy {
    /// Migrate a small share, refuse a large one. The default.
    Auto,
    /// Always migrate, however much there is.
    Fix,
    /// Never migrate; refuse as soon as the probe finds anything. The pre-threshold behaviour,
    /// kept for bisecting.
    Refuse,
}

/// Share of sampled folios the probe could not pin, as a percentage.
fn bad_pct(v: &ProbeVerdict) -> u64 {
    if v.samples == 0 {
        return 0;
    }
    v.bad.saturating_mul(100) / v.samples
}

/// Whether to migrate rather than refuse, given what the probe found.
fn migrate_wanted(v: &ProbeVerdict) -> bool {
    match policy() {
        PinPolicy::Fix => true,
        PinPolicy::Refuse => false,
        PinPolicy::Auto => bad_pct(v) <= AUTO_FIX_MAX_BAD_PCT,
    }
}

/// What the collapse pass managed, in a form that can be appended to a one-line message.
///
/// This is the number that tells the two failure modes apart, and it is already in hand: the
/// preparation that ran immediately before this counted how much of the region it could get into
/// 2 MB folios. Coverage well under 100% means populate fell back to 4 KB faults and the collapse
/// could not repair them, which is where off-pool pages come from; coverage at 100% with off-pool
/// pages means the reserve did not serve the allocation at all, which is a different problem.
fn collapse_note(prep: Option<&crate::gunyah::mthp::LendPrepResult>, size: u64) -> String {
    match prep {
        Some(p) if size > 0 => format!(
            ", collapse covered {} of {} MB ({:.1}%)",
            p.large_page_bytes >> 20,
            size >> 20,
            p.large_page_bytes as f64 * 100.0 / size as f64
        ),
        _ => String::new(),
    }
}

fn policy() -> PinPolicy {
    match std::env::var("GUNYAH_PIN_POLICY").as_deref() {
        Ok("fix") => PinPolicy::Fix,
        Ok("refuse") => PinPolicy::Refuse,
        _ => PinPolicy::Auto,
    }
}

fn pin_enabled() -> bool {
    !matches!(std::env::var("GUNYAH_PIN").as_deref(), Ok("0"))
}

fn fail_injected(site: PinSite) -> bool {
    match std::env::var("GUNYAH_PIN_FAIL").as_deref() {
        Ok("all") => true,
        Ok(s) => s == site.as_str(),
        Err(_) => false,
    }
}

/// `CmaFree` in kB, or -1 if it cannot be read. Reported around every probe that had work to do:
/// it is the one number that shows whether the range was in CMA, and it moved by exactly the
/// probe's size in the bring-up measurements.
fn cma_free_kb() -> i64 {
    let Ok(meminfo) = std::fs::read_to_string("/proc/meminfo") else {
        return -1;
    };
    for line in meminfo.lines() {
        if let Some(rest) = line.strip_prefix("CmaFree:") {
            return rest
                .trim()
                .trim_end_matches(" kB")
                .trim()
                .parse::<i64>()
                .unwrap_or(-1);
        }
    }
    -1
}

/// A held `FOLL_LONGTERM` pin over one or more host ranges. Dropping it unpins.
pub struct LongtermPin {
    ring: Option<File>,
    site: PinSite,
    mb: u64,
}

impl LongtermPin {
    /// Makes sure `size` bytes at `host_addr` can take the `FOLL_LONGTERM` pin the hypervisor is
    /// about to attempt, or returns the error to refuse with.
    ///
    /// Asks `/dev/gh_pinprobe` first, which touches nothing. A clean answer means there is
    /// nothing to do and nothing is held (`Ok(None)`). Unpinnable pages mean the region did not
    /// come from the reserve pool: refuse, unless `GUNYAH_PIN_POLICY=fix` asks us to migrate them
    /// by pinning. If the probe node is missing we cannot tell, so we pin -- the safe, slower
    /// answer.
    pub fn ensure_pinnable(
        host_addr: u64,
        size: u64,
        site: PinSite,
        prep: Option<&crate::gunyah::mthp::LendPrepResult>,
    ) -> Result<Option<LongtermPin>> {
        if size == 0 || !pin_enabled() {
            return Ok(None);
        }
        if fail_injected(site) {
            warn!(
                "GH-PIN[{}]: injected failure (GUNYAH_PIN_FAIL) for {} MB at {:#x}",
                site.as_str(),
                size >> 20,
                host_addr
            );
            return Err(Error::new(libc::ENOMEM));
        }

        // A pre-boot region that is not pool-served is usually a VM starting into the couple of
        // seconds the reserve takes to reclaim the previous one's pages -- a guest reboot relaunches
        // crosvm about two seconds after the old one exited, which lands exactly there. Refusing on
        // the first look turns that into "reboot kills the VM", so wait for the reserve before
        // giving up. Runtime blobs never wait: they are on the allocation hot path and a caller
        // that gets an error simply allocates elsewhere.
        let attempts = if site == PinSite::PreBoot { PREBOOT_PROBE_ATTEMPTS } else { 1 };
        for attempt in 1..attempts {
            match probe_range(host_addr, size) {
                Some(v) if v.bad > 0 => {
                    info!(
                        "GH-PIN[{}]: {}/{} samples not pool-served, waiting {:?} for the reserve \
                         (attempt {}/{})",
                        site.as_str(),
                        v.bad,
                        v.samples,
                        PREBOOT_PROBE_WAIT,
                        attempt,
                        attempts
                    );
                    std::thread::sleep(PREBOOT_PROBE_WAIT);
                }
                _ => break,
            }
        }

        match probe_range(host_addr, size) {
            Some(v) if v.bad == 0 => {
                if v.absent > 0 {
                    warn!(
                        "GH-PIN[{}]: {} of {} samples had no page present at {:#x} \
                         (region not fully populated?)",
                        site.as_str(),
                        v.absent,
                        v.samples,
                        host_addr
                    );
                }
                Ok(None)
            }
            Some(v) if migrate_wanted(&v) => {
                info!(
                    "GH-PIN[{}]: {} MB at {:#x}: {}/{} samples ({}%) need migrating \
                     (cma={} isolate={} movable={}){} -- migrating",
                    site.as_str(),
                    size >> 20,
                    host_addr,
                    v.bad,
                    v.samples,
                    bad_pct(&v),
                    v.cma,
                    v.isolate,
                    v.movable,
                    collapse_note(prep, size),
                );
                Self::migrate_and_hold(host_addr, size, site, prep)
            }
            Some(v) => {
                error!(
                    "GH-PIN[{}]: refusing {} MB at {:#x}: {}/{} 2MB samples ({}%) cannot be \
                     long-term pinned (cma={} isolate={} movable={}, first at +{:#x}){} -- this \
                     memory did not come from the reserve pool. Handing it to the hypervisor \
                     stalls the host or resets the phone; GUNYAH_PIN_POLICY=fix migrates it \
                     instead of refusing, at any size. CmaFree {} kB.",
                    site.as_str(),
                    size >> 20,
                    host_addr,
                    v.bad,
                    v.samples,
                    bad_pct(&v),
                    v.cma,
                    v.isolate,
                    v.movable,
                    v.first_bad_offset,
                    collapse_note(prep, size),
                    cma_free_kb(),
                );
                Err(Error::new(libc::ENOMEM))
            }
            None => Self::pin(host_addr, size, site),
        }
    }

    /// Takes the `FOLL_LONGTERM` pin for real, which migrates whatever needs migrating.
    /// Migrate the off-pool pages by pinning them, and keep the pin.
    ///
    /// Migration is not something this process performs; it is what the kernel does inside a
    /// `FOLL_LONGTERM` pin, so taking the pin ourselves is how it is asked for -- and the pin is
    /// also the answer: if the migration could not find anywhere to move a page to, the pin
    /// fails, and that error is this function's error.
    ///
    /// The pin is then HELD, all the way through the hypervisor call the caller is about to make.
    /// That is not just convenience. Measured on device: unpinning first, collapsing the region
    /// again to repair any folio the migration had split, and re-probing, left 57 of 1728 samples
    /// back in CMA when the reserve was empty -- the repair pass allocated fresh pages, the
    /// reserve had none, and the buddy allocator handed back exactly the memory that had just
    /// been migrated away from. Holding the pin makes that impossible: a pinned page cannot be
    /// migrated, by us or by anyone else.
    ///
    /// What is therefore not verified is the folio order after migration. The kernel allocates a
    /// same-order target where it can and falls back to 4 KB pages where it cannot, so a region
    /// can in principle come back more fragmented than the collapse pass left it, and the parcel
    /// shape the caller computes is from the pre-migration map. Nothing observed has needed that
    /// yet -- the coverage logged below is what to check first if an RM call ever fails after a
    /// migration.
    fn migrate_and_hold(
        host_addr: u64,
        size: u64,
        site: PinSite,
        prep: Option<&crate::gunyah::mthp::LendPrepResult>,
    ) -> Result<Option<LongtermPin>> {
        let pin = Self::pin(host_addr, size, site)?;
        info!(
            "GH-PIN[{}]: migrated {} MB at {:#x} and holding the pin through the hypervisor \
             call{}",
            site.as_str(),
            size >> 20,
            host_addr,
            collapse_note(prep, size),
        );
        Ok(pin)
    }

    fn pin(host_addr: u64, size: u64, site: PinSite) -> Result<Option<LongtermPin>> {
        let mut params = IoUringParams::default();
        // SAFETY: passing a properly sized, zeroed params struct; the kernel writes it back.
        let ring_fd = unsafe {
            libc::syscall(
                SYS_IO_URING_SETUP,
                1 as libc::c_long,
                &mut params as *mut IoUringParams,
            )
        };
        if ring_fd < 0 {
            let err = IoError::last_os_error();
            // No io_uring on this kernel (or policy forbids it): that is not a reason to refuse
            // the VM. Carry on unprobed -- the gunyah ioctl will pin as it always did.
            warn!(
                "GH-PIN[{}]: io_uring unavailable ({}), continuing without the pin probe",
                site.as_str(),
                err
            );
            return Ok(None);
        }
        // SAFETY: io_uring_setup returned this fd and we are its only owner.
        let ring = unsafe { File::from_raw_fd(ring_fd as RawFd) };

        let mut iovecs: Vec<libc::iovec> = Vec::new();
        let mut offset = 0u64;
        while offset < size {
            let len = std::cmp::min(CHUNK_BYTES, size - offset);
            iovecs.push(libc::iovec {
                iov_base: (host_addr + offset) as *mut libc::c_void,
                iov_len: len as usize,
            });
            offset += len;
        }

        let before = cma_free_kb();
        let start = Instant::now();
        // SAFETY: the iovecs describe a live mapping owned by this process; the kernel only
        // reads the array.
        let ret = unsafe {
            libc::syscall(
                SYS_IO_URING_REGISTER,
                ring.as_raw_descriptor() as libc::c_long,
                IORING_REGISTER_BUFFERS as libc::c_long,
                iovecs.as_ptr(),
                iovecs.len() as libc::c_long,
            )
        };
        let elapsed = start.elapsed();
        if ret < 0 {
            let err = IoError::last_os_error();
            let errno = err.raw_os_error().unwrap_or(libc::ENOMEM);
            error!(
                "GH-PIN[{}]: FOLL_LONGTERM pin of {} MB at {:#x} failed after {:?}: {} \
                 -- the pages cannot be pinned (CMA with nothing to migrate into). \
                 CmaFree {} kB. Refusing to hand this memory to the hypervisor.",
                site.as_str(),
                size >> 20,
                host_addr,
                elapsed,
                err,
                before,
            );
            return Err(Error::new(errno));
        }

        let after = cma_free_kb();
        // Quiet on the fast path: a pool-served region needs no migration and finishes in a few
        // milliseconds, and this runs per blob on the runtime-share path.
        if elapsed.as_millis() >= 50 || after.saturating_sub(before) > 0 {
            info!(
                "GH-PIN[{}]: pinned {} MB at {:#x} in {:?} (CmaFree {} -> {} kB)",
                site.as_str(),
                size >> 20,
                host_addr,
                elapsed,
                before,
                after,
            );
        }

        Ok(Some(LongtermPin {
            ring: Some(ring),
            site,
            mb: size >> 20,
        }))
    }
}

impl Drop for LongtermPin {
    fn drop(&mut self) {
        let Some(ring) = self.ring.take() else {
            return;
        };
        // SAFETY: unregistering takes no user pointer; the fd is ours.
        let ret = unsafe {
            libc::syscall(
                SYS_IO_URING_REGISTER,
                ring.as_raw_descriptor() as libc::c_long,
                IORING_UNREGISTER_BUFFERS as libc::c_long,
                std::ptr::null::<libc::iovec>(),
                0 as libc::c_long,
            )
        };
        if ret < 0 {
            // Closing the ring below unpins anyway; this only means we could not do it early.
            warn!(
                "GH-PIN[{}]: unregister of {} MB failed: {}",
                self.site.as_str(),
                self.mb,
                IoError::last_os_error()
            );
        }
        drop(ring);
    }
}
