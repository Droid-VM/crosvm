// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

use std::thread;
use std::time::Duration;
use std::time::Instant;

use anyhow::Context;
use anyhow::Result;
use base::debug;
use base::error;
use base::info;
use base::warn;
use base::AsRawDescriptor;
use base::SafeDescriptor;
use base::SendTube;
use base::VmEventType;
use base::VolatileSlice;
use base::WaitContext;
use gpu_display::Damage;
use gpu_display::DisplayExternalResourceImport;
use gpu_display::GpuDisplay;
use gpu_display::GpuDisplayError;
use gpu_display::GpuDisplayExt;
use gpu_display::PresentOutcome;
use gpu_display::ScanoutFrame;
use gpu_display::SurfaceType;
use gpu_display::VncBindingInput;
use vm_control::gpu::DisplayMode;
use vm_control::gpu::DisplayParameters;
use vm_memory::udmabuf::UdmabufDriver;
use vm_memory::udmabuf::UdmabufDriverTrait;
use vm_memory::FramebufferPrep;
use vm_memory::GuestAddress;
use vm_memory::GuestMemory;

use crate::crosvm::config::TransportCap;

pub struct SimplefbDisplayParams {
    pub addr: u64,
    pub width: u32,
    pub height: u32,
    pub stride: u32,
    pub bpp: u32,
    pub size: u64,
    /// DRM fourcc of the framebuffer, from the same DT `format` string the guest is handed.
    pub fourcc: u32,
    /// How many times a second to look at the framebuffer. Nothing in the guest announces a frame
    /// here, so this rate is the entirety of what decides when a picture exists.
    pub poll_hz: u32,
}

const fn drm_fourcc(a: u8, b: u8, c: u8, d: u8) -> u32 {
    (a as u32) | ((b as u32) << 8) | ((c as u32) << 16) | ((d as u32) << 24)
}

/// The DRM fourcc named by a simplefb device-tree `format` string.
///
/// The DT string is the only statement anyone makes about this framebuffer's byte order, and the
/// host has been acting on it implicitly: the default `a8r8g8b8` is ARGB8888, which in memory is
/// B,G,R,A. Naming it lets the CPU edge compare the source with each sink's real layout, and lets
/// the GPU path pick its VkFormat from the same fact instead of assuming one byte order.
///
/// The fallback matches the one the bpp lookup uses on an unrecognised string, for the same reason:
/// `a8r8g8b8` is what the device tree defaults to when nobody says otherwise.
pub fn simplefb_format_fourcc(format: &str) -> u32 {
    match format {
        "x8r8g8b8" => drm_fourcc(b'X', b'R', b'2', b'4'),
        "a8b8g8r8" => drm_fourcc(b'A', b'B', b'2', b'4'),
        "r8g8b8" => drm_fourcc(b'R', b'G', b'2', b'4'),
        "r5g6b5" => drm_fourcc(b'R', b'G', b'1', b'6'),
        _ => drm_fourcc(b'A', b'R', b'2', b'4'),
    }
}

/// Where simplefb frames go. Everything past opening the display is backend-agnostic: the bridge
/// asks the `GpuDisplay` what it can take and uses whichever half it has -- import + `flip_to`, or
/// `framebuffer()` + `flip()` -- so any backend works and none of them are named below this point.
pub enum SimplefbDisplayTarget {
    Vnc {
        addr: String,
        password: Option<String>,
        /// Whether this binding may run the hardware H.264 encoder. Carried here for the same
        /// reason `transport_cap` is: the ceiling belongs to the binding, and this screen's binding
        /// is a different one from the GPU screen's. There is no port with it -- the stream rides
        /// `addr`, the RFB port this server already listens on.
        hw_encode: bool,
        /// This screen's own absolute pointer and keyboard, for the sink to inject RFB events into.
        /// Empty on a `view-only=true` binding, which is how RFB input comes to be dropped: there
        /// is nowhere for it to go.
        ///
        /// Carried on the target rather than passed beside it because it is a property of THIS
        /// binding, and the other target has no input of its own at all.
        input: VncBindingInput,
    },
    /// The Android Surface the app hands over through the display service binder. Input does
    /// NOT come through the display here -- it arrives on the `--input` evdev sockets, same as
    /// the virtio-gpu native-display path.
    Android { service_name: String },
}

/// Turns the configured poll rate into the interval between ticks. The rate is validated at parse
/// time; the clamp is here so that this arithmetic cannot be the thing that decides what a bad
/// value means.
fn tick_duration(poll_hz: u32) -> Duration {
    Duration::from_nanos(1_000_000_000 / poll_hz.max(1) as u64)
}

/// Spawns the bridge.
///
/// `vm_evt_wrtube` is how the thread reports that the exporter it was given could not be opened at
/// all. It is needed because the display is opened on the thread and not before it: `GpuDisplay`
/// holds `Rc`s and cannot be moved across one, so the failure cannot be handed back to the caller
/// as a `Result` the way the rest of VM setup reports things. Without it the thread's only option
/// was to log and return, which left the VM running with the screen the user configured silently
/// unexported -- the same failure this whole change is about, one layer up.
pub fn start_simplefb_display_thread(
    guest_mem: GuestMemory,
    params: SimplefbDisplayParams,
    target: SimplefbDisplayTarget,
    transport_cap: TransportCap,
    vm_evt_wrtube: SendTube,
) -> Result<thread::JoinHandle<()>> {
    thread::Builder::new()
        .name("simplefb_display".into())
        .spawn(move || {
            // Whether handing this exporter a frame it did not ask for is nearly free. It decides
            // the default for `CROSVM_SIMPLEFB_GPU_ALWAYS_FLIP`, and it is a property of the
            // exporter rather than of the transport -- which is the correction the measurements
            // forced. See the switch's own note for the four numbers.
            //
            // Asked here because `target` is consumed below and this is the last place that knows
            // which one it was. The honest home for it is a `GpuDisplay` capability, next to
            // `is_dmabuf_import_supported`, so a sink states its own cost instead of being
            // classified from outside; that is a trait change and this is not.
            let unconditional_present_is_cheap =
                matches!(target, SimplefbDisplayTarget::Android { .. });

            // `target` is consumed rather than borrowed: the VNC arm's input devices are moved into
            // the sink, and a device cannot be handed over through a `&`.
            let display_result = match target {
                SimplefbDisplayTarget::Vnc {
                    addr,
                    password,
                    hw_encode,
                    input,
                } => GpuDisplay::open_vnc_tcp(
                    &addr,
                    params.width,
                    params.height,
                    password,
                    hw_encode,
                    input.tablet,
                    input.keyboard,
                ),
                SimplefbDisplayTarget::Android { service_name } => {
                    GpuDisplay::open_android(&service_name)
                }
            };
            let mut display = match display_result {
                Ok(d) => d,
                // An address that could not be listened on is a VM that is not what was asked for,
                // so it stops rather than runs on: the bridge has already logged which family
                // failed and why. Every other way a display can fail to open keeps the old
                // behaviour -- the bridge gives up and the rest of the VM carries on.
                Err(e @ GpuDisplayError::Listen(_)) => {
                    error!("simplefb: {}; stopping the VM", e);
                    if let Err(e) = vm_evt_wrtube.send::<VmEventType>(&VmEventType::Exit) {
                        error!("simplefb: failed to request VM exit: {}", e);
                    }
                    return;
                }
                Err(e) => {
                    error!("simplefb: failed to open display: {:?}", e);
                    return;
                }
            };

            // The ceiling this binding was configured with, applied to the display before anything
            // asks it a question. Doing it here rather than at the import attempt is what makes the
            // cap a property of the display for its whole life: nothing downstream has to remember
            // to consult it, and the probe never even reaches the backend.
            if !transport_cap.allows_gpu_copy() {
                display.cap_transport_to_cpu();
            }

            // No `import_event_device` here any more. The VNC sink was handed its devices above and
            // delivers to them itself; importing them would have put them in this display's
            // owner-scoped map, which is precisely the map that was empty on this path whenever a
            // GPU device existed -- the device present, the road absent. The Android backend never
            // had input of its own (the app drives the `--input` evdev sockets instead).

            if let Err(e) = simplefb_display_loop(
                guest_mem,
                &params,
                &mut display,
                transport_cap,
                unconditional_present_is_cheap,
            ) {
                error!("simplefb display thread exited with error: {:?}", e);
            }
        })
        .context("failed to spawn simplefb display thread")
}

/// The only modifier a udmabuf window over guest pages can honestly claim: the bytes are exactly
/// as the guest laid them out, row after row.
const DRM_FORMAT_MOD_LINEAR: u64 = 0;

/// How long the bridge waits for the sink to finish reading the framebuffer before flipping onto
/// it again.
///
/// This is NOT the virtio-gpu flush path's fence, despite riding the same API. There the fence
/// gates the *guest*'s reuse of a buffer it rendered into, and skipping the wait corrupts a frame.
/// Here the source is guest memory the guest writes continuously with no synchronisation of any
/// kind, so there is nothing to hold back and nothing to corrupt that was not already racy: the
/// only thing this wait buys is that our timer loop does not queue a second blit while the first
/// is still reading. The guest never waits on it -- it is a pacing device for the producer, and
/// the timeout exists so that a sink which stops signalling slows the bridge down instead of
/// stopping it. Three 120Hz vsyncs, the same figure the flush path settled on.
const FLIP_FENCE_TIMEOUT: Duration = Duration::from_millis(25);

/// What the bridge does with a frame once it has decided one exists.
///
/// The two arms differ in who reads the framebuffer, not in what is on screen. On `Cpu` the bridge
/// copies `read_buf` into a buffer the sink hands out. On `Gpu` the sink reads the guest pages
/// itself, through a dmabuf window created once at startup, and the copy the CPU path pays per
/// frame becomes a Vulkan blit the sink was going to do anyway.
///
/// The watcher is untouched by this choice: it still reads the changed bands out of guest memory
/// into `read_buf` on both. That read is redundant on the GPU path -- nothing presents `read_buf`
/// there -- and it is kept deliberately, because it is also what makes the fallback below a
/// single assignment: the moment a blit fails, the frame the sink was going to be shown is already
/// sitting in host memory in the layout the CPU path wants. Removing it means the watcher has to
/// learn which transport is live, which is the coupling §4.2 is trying to avoid; the measurement
/// that would justify paying that price has not been taken.
enum Transport {
    Gpu(GpuTransport),
    /// Always reachable. Every failure on the GPU path -- at startup or mid-run -- ends here and
    /// stays here for the life of the bridge, because a udmabuf that could not be created or an
    /// import a sink refused will not start working on the next tick, and retrying every 33ms
    /// would be a log line and an ioctl per frame forever.
    Cpu,
}

/// A dmabuf window over the framebuffer and the sink's import of it.
struct GpuTransport {
    /// The import the sink blits from. Created once and reused for the life of the bridge: unlike
    /// virtio-gpu, whose compositor cycles through swapchain buffers, this framebuffer is one
    /// fixed region at one fixed address, so there is nothing to re-import per frame.
    import_id: u32,
    /// The dmabuf the import was made from. Held for as long as the import is alive: the sink
    /// takes what it needs from the fd at import time, but nothing here should depend on that
    /// having been a full transfer of ownership.
    _dmabuf: SafeDescriptor,
    /// The completion fence of the previous `flip_to`, when the backend produced one. Waited on
    /// (bounded) before the next flip; see `FLIP_FENCE_TIMEOUT`.
    pending_fence: Option<SafeDescriptor>,
    /// Whether a fence timeout has already been reported. A sink that stops signalling stops on
    /// every frame, and a line per frame is not observability.
    fence_timeout_logged: bool,
}

impl GpuTransport {
    /// Waits, with a bound, for the previous flip's completion fence.
    fn await_previous_flip(&mut self) {
        let Some(fence) = self.pending_fence.take() else {
            return;
        };
        let mut pfd = libc::pollfd {
            fd: fence.as_raw_descriptor(),
            events: libc::POLLIN,
            revents: 0,
        };
        // SAFETY: `pfd` points at one valid pollfd for the duration of the call, and the fd is
        // owned by `fence`, which outlives it.
        let ret = unsafe {
            libc::poll(
                &mut pfd,
                1,
                FLIP_FENCE_TIMEOUT.as_millis() as libc::c_int,
            )
        };
        if ret == 0 && !self.fence_timeout_logged {
            self.fence_timeout_logged = true;
            warn!(
                "simplefb: display flip fence still unsignaled after {:?}; flipping anyway \
                 (further timeouts silent)",
                FLIP_FENCE_TIMEOUT
            );
        }
    }
}

/// Builds the GPU transport for this framebuffer, or says why there is none.
///
/// Everything here runs once, immediately after the surface is created and therefore BEFORE the
/// first tick -- which means before any consumer can exist. That ordering is the point: whether
/// this framebuffer can be handed to a sink as a dmabuf is a property of the region and the sink,
/// not of whether somebody happens to be watching, so the answer is reached, and logged, on every
/// run rather than only on runs where an app attached.
fn build_gpu_transport(
    guest_mem: &GuestMemory,
    params: &SimplefbDisplayParams,
    display: &mut GpuDisplay,
    surface_id: u32,
    transport_cap: TransportCap,
) -> std::result::Result<GpuTransport, String> {
    // The cap is already in force on the display, so the probe below would refuse anyway. Asked
    // separately only so the startup line names the ceiling instead of blaming the sink: "the sink
    // imports no dmabufs" would be a false statement about a perfectly capable sink, and the whole
    // value of that line is that somebody reading it can tell why they are on the slow path.
    if !transport_cap.allows_gpu_copy() {
        return Err("capped by transport-cap=cpu".to_string());
    }
    if !display.is_dmabuf_import_supported() {
        return Err("the bound sink imports no dmabufs".to_string());
    }

    // Whether the pages under this region are somewhere the host can leave them, as established by
    // the backend that laid the region out (see `vm_memory::FramebufferPrep`). This is the one
    // question that has to be asked before the udmabuf below and not after: the udmabuf takes a
    // page reference on every page of the region, a referenced page cannot be migrated, and if any
    // of those pages is in CMA the guest's first touch of the framebuffer kills the vcpu --
    // `page fault at <gpa>, attempt: -12`, because gunyah's pin has nowhere to move it to. There is
    // no recovery from that and no way to notice it from here; the failure lands on the guest.
    //
    // Anything short of a positive answer means the CPU path, including "nobody said". A partial
    // answer would be the same lottery with better odds, and the floor -- copying the bytes -- costs
    // a memcpy per frame and cannot fail this way at all.
    match guest_mem.framebuffer_prep() {
        FramebufferPrep::PoolBacked => {}
        FramebufferPrep::NotPoolBacked(why) => {
            return Err(format!("framebuffer pages not pool-backed: {why}"));
        }
        FramebufferPrep::Unclaimed => {
            return Err("framebuffer pages not pool-backed: no hypervisor backend prepared the \
                        region, so nothing can say the guest's first fault will find a page it \
                        can pin"
                .to_string());
        }
    }

    // udmabuf windows are made of whole pages, and it rejects anything else with `NotPageAligned`
    // -- checking here is only so the refusal names which end was wrong.
    let pagesize = base::pagesize();
    if params.addr % pagesize as u64 != 0 || params.size % pagesize as u64 != 0 {
        return Err(format!(
            "framebuffer region {:#x}+{:#x} is not page aligned",
            params.addr, params.size
        ));
    }
    let fb_bytes = (params.stride as usize)
        .checked_mul(params.height as usize)
        .ok_or_else(|| "framebuffer geometry overflows a usize".to_string())?;
    if fb_bytes as u64 > params.size {
        return Err(format!(
            "{fb_bytes}-byte framebuffer does not fit the {}-byte region",
            params.size
        ));
    }
    // The window is the WHOLE reserved region, not just the bytes the picture occupies, and the
    // reason is not the obvious one. An importer's own layout for the same geometry can need more
    // memory than `stride x height`: turnip asks for two rows beyond the last one, and refuses an
    // fd smaller than its `VkMemoryRequirements` ("requirements exceed DMA-BUF allocation",
    // measured on 5567). Nothing here can predict that number -- it belongs to whichever driver
    // the sink loaded -- so give the importer everything the region has instead of computing a
    // bound that is only right for the sinks we happened to test. It costs nothing: this region
    // exists for this framebuffer and holds nothing else, the extra pages are never read (the
    // image is described at its real geometry), and the guest cannot see the difference.
    let window = params.size as usize;

    let driver = UdmabufDriver::new().map_err(|e| format!("udmabuf unavailable: {e}"))?;
    // The region is a slice of the guest's own memfd (see the comment where these params are
    // built), so this is a window over exactly the pages the guest writes -- no copy, no second
    // allocation. The driver fd is not kept: the ioctl is what needed it, and the dma_buf it
    // returns holds its own reference to the memfd pages.
    let dmabuf = driver
        .create_udmabuf(guest_mem, &[(GuestAddress(params.addr), window)])
        .map_err(|e| format!("udmabuf create failed: {e}"))?;

    // THE CROSSING (plan §4.4). Both transports now carry this source fourcc explicitly: the CPU
    // edge compares it with the sink framebuffer, while the GPU sink picks a VkFormat from it
    // (`vkFormatFromDrmFourcc`). The device tree's default `a8r8g8b8` is AR24, so declaring it lands
    // on B8G8R8A8_UNORM. Declare it wrong and every
    // pixel comes out with red and blue exchanged, on both paths' behalf, with no error anywhere
    // and nothing in the logs -- the failure looks like a picture. `params.fourcc` is the DT
    // string resolved once at config time (`simplefb_format_fourcc`) precisely so that this is one
    // declaration rather than an assumption made twice.
    //
    // `linear_layout_verified` means what the pool-scanout path means by it: this single-plane
    // buffer really is a linear image with the stride given. Here it holds by construction rather
    // than by inspection -- the window starts at byte zero of the framebuffer, rows are `stride`
    // apart because that is how simplefb is defined, and no tiling exists anywhere on this route.
    let import_id = display
        .import_resource(
            surface_id,
            DisplayExternalResourceImport::Dmabuf {
                descriptor: &dmabuf,
                offset: 0,
                stride: params.stride,
                modifiers: DRM_FORMAT_MOD_LINEAR,
                linear_layout_verified: true,
                width: params.width,
                height: params.height,
                fourcc: params.fourcc,
            },
        )
        .map_err(|e| format!("the sink refused the import: {e:#}"))?;

    Ok(GpuTransport {
        import_id,
        _dmabuf: dmabuf,
        pending_fence: None,
        fence_timeout_logged: false,
    })
}

/// Rows hashed as one unit.
///
/// The only consumer of this granularity is the decision to skip a tick, so the shape that matters
/// is the one that is cheapest to compute: a band is a run of whole rows, which is contiguous
/// memory when the stride is packed and a fixed walk when it is not. Tiles would buy a narrower
/// answer that nothing here can spend -- a frame goes out whole (see `present_frame` below) -- and
/// would pay for it with a strided read per tile. 32 rows is also the VNC sink's own band height,
/// so on that route the two layers agree about what a unit of change is.
const WATCH_BAND_ROWS: usize = 32;

/// How often the watcher looks at the framebuffer while nothing is watching the screen.
///
/// Bookkeeping, not a setting: what it buys is that `last_changed_at` is roughly right at the
/// moment somebody asks which screen is alive, and that moment is precisely when no sink is
/// attached yet. Stopping altogether would make the answer oldest exactly when it is needed.
const LIVENESS_INTERVAL: Duration = Duration::from_secs(1);

/// Floor on how often the change log line may repeat. Content that is genuinely moving changes on
/// every tick, and a line per tick is not observability.
const CHANGE_LOG_INTERVAL: Duration = Duration::from_secs(5);

/// Whether `len` bytes at two addresses differ.
///
/// One of the two is guest memory the guest writes with no synchronisation of any kind, so this is
/// a racing read by construction -- as every read of this framebuffer is, including the copy beside
/// it. What makes that safe to build on is not the read but where its answer is recorded: a band is
/// only ever declared "already on screen" against the bytes that were actually copied, so a
/// comparison that caught a half-written frame is corrected by the next tick rather than believed.
///
/// `memcmp` rather than a hash for a measured reason. The FNV-1a this replaced is a serial
/// dependency chain -- one xor and one multiply per byte, neither vectorisable nor overlappable --
/// and cost 10.3 ms for a 1080p frame against 0.45 ms for the same bytes through `memcmp`. That
/// difference is the whole per-tick budget: a hash is worth paying for when the fingerprint has to
/// be stored, and here the previous frame is already sitting in `read_buf`.
///
/// # Safety
///
/// Both pointers must be readable for `len` bytes.
unsafe fn bytes_differ(a: *const u8, b: *const u8, len: usize) -> bool {
    len != 0 && libc::memcmp(a as *const libc::c_void, b as *const libc::c_void, len) != 0
}

/// Debug switches for the bridge, read from the environment once when the loop starts.
///
/// They exist to separate three explanations of a region that stays stale on screen until the
/// guest happens to repaint it, which the normal logs cannot tell apart: the watcher skipped a
/// band that really had changed, the sink was handed a correct frame and did not show it, or the
/// guest's framebuffer holds the stale content itself. Off unless set; the only per-tick cost when
/// `dump_prefix` is set is one `stat`.
struct SimplefbDebug {
    /// `CROSVM_SIMPLEFB_NO_SKIP=1`: copy and present every band on every tick, whatever the hashes
    /// say. A stale region that survives this is not being produced by the band skip.
    no_skip: bool,
    /// `CROSVM_SIMPLEFB_DUMP=<prefix>`: when `<prefix>.trigger` appears, write what the sink was
    /// last given and what guest memory holds at that same moment, and log a per-band comparison
    /// of the two. The trigger file is removed as it is consumed, so one `touch` is one dump.
    dump_prefix: Option<String>,
    /// On the GPU transport, present every tick without looking at the framebuffer at all.
    ///
    /// **Defaults to whether an unconditional present is cheap for this exporter**, which is a
    /// property of the exporter and not of the transport. That correction came from measurement,
    /// so the measurement is here -- 1500x1060 at 120 Hz, static desktop, bridge thread only:
    ///
    /// | exporter | always flip | present only what changed |
    /// |---|---|---|
    /// | native display | **4.1%** | ~20% |
    /// | VNC | 47.3%, hw encoder fed 118 fps | **21.5%**, encoder fed 0 fps |
    ///
    /// The native display's sink does a GPU blit and posts a buffer -- what a display does every
    /// vsync anyway -- so not asking whether the frame is new is both cheaper and safer than
    /// asking. The VNC sink reads the frame back, memcmps it whole against `last_clean` and feeds
    /// a hardware encoder, per offered frame: unconditional presents there run the AVC encoder at
    /// full rate over a screen that is not moving, which costs power and bitrate on top of the CPU.
    ///
    /// What it buys where it is on: this is the only configuration in which the host CPU never
    /// reads this region. The guest maps it write-combining and the host maps the same pages
    /// cacheable, so a cacheable read can return bytes the guest overwrote -- and the flip decision
    /// is a cacheable read. Not making the decision is a stronger fix than making it correctly,
    /// where the decision was not worth anything.
    gpu_always_flip: bool,
    /// **On unless `CROSVM_SIMPLEFB_INVALIDATE=0`**: drop the framebuffer's cache lines before
    /// reading it.
    ///
    /// Not a switch in the sense the others are. Without it every read of this region -- the
    /// comparison that decides whether to present, and the copy that produces the frame -- can
    /// return bytes the guest overwrote and the host's cache never heard about, and the screen
    /// keeps a stale 64-byte line until something evicts it. That is a correctness failure with no
    /// symptom in any log, so it is on by default and the environment variable exists to turn it
    /// OFF, for measuring what it costs or for confirming that it is still what is holding the
    /// picture together.
    invalidate: bool,
}

/// Length of a data cache line, from `CTR_EL0.DminLine` -- the granule `dc civac` operates on.
///
/// Read rather than assumed. 64 bytes is what every core this runs on reports and what the observed
/// artefacts measured, but a hard-coded 64 on a machine with 128-byte lines would invalidate half
/// the region and leave the rest exactly as wrong as before, silently.
#[cfg(target_arch = "aarch64")]
fn dcache_line_size() -> usize {
    let ctr: u64;
    // SAFETY: a read of a system register with no side effects. `CTR_EL0` is readable from EL0 on
    // every arm64 Linux (trapped and emulated where the hardware does not allow it).
    unsafe {
        std::arch::asm!("mrs {}, ctr_el0", out(reg) ctr, options(nomem, nostack, preserves_flags));
    }
    // DminLine is log2 of the line length in WORDS, not bytes.
    4usize << ((ctr >> 16) & 0xf)
}

/// Drops the host's cached copy of `[ptr, ptr + len)` so the next read of it comes from memory.
///
/// `dc civac` is clean-and-invalidate, and clean is what makes it safe here: this mapping is only
/// ever read, so its lines are always clean and the "clean" half writes nothing back over the
/// guest. The invalidate-only form (`dc ivac`) would say what is meant, but it is EL1-only --
/// `civac` is one of the four operations `SCTLR_EL1.UCI` opens to EL0, which arm64 Linux sets.
///
/// Cost is one instruction per line: ~57k for a 720p framebuffer, well under a millisecond, and
/// cheaper than the full-frame hash it sits next to.
#[cfg(target_arch = "aarch64")]
fn invalidate_dcache_range(ptr: *const u8, len: usize) {
    if len == 0 {
        return;
    }
    let line = dcache_line_size();
    let mut addr = (ptr as usize) & !(line - 1);
    let end = (ptr as usize).saturating_add(len);
    while addr < end {
        // SAFETY: `dc civac` takes a virtual address and affects only cache state. The range walked
        // is the caller's mapping, rounded down to a line boundary at the start.
        unsafe {
            std::arch::asm!("dc civac, {}", in(reg) addr, options(nostack, preserves_flags));
        }
        addr += line;
    }
    // SAFETY: a barrier; no memory operands.
    unsafe {
        std::arch::asm!("dsb ish", options(nostack, preserves_flags));
    }
}

#[cfg(not(target_arch = "aarch64"))]
fn invalidate_dcache_range(_ptr: *const u8, _len: usize) {}

impl SimplefbDebug {
    fn from_env(unconditional_present_is_cheap: bool) -> Self {
        // Every switch here is a tri-state: unset means the default, and both `1` and `0` are
        // answers. A debug switch that could only be turned on would leave the one thing with a
        // non-false default unable to be turned off.
        let flag = |name: &str, default: bool| {
            std::env::var(name)
                .map(|v| v == "1" || v == "true" || v == "on")
                .unwrap_or(default)
        };
        let no_skip = flag("CROSVM_SIMPLEFB_NO_SKIP", false);
        let gpu_always_flip = flag(
            "CROSVM_SIMPLEFB_GPU_ALWAYS_FLIP",
            unconditional_present_is_cheap,
        );
        // Default on, so the question the environment answers is "turn it off", not "turn it on".
        let invalidate = flag("CROSVM_SIMPLEFB_INVALIDATE", true);
        let dump_prefix = std::env::var("CROSVM_SIMPLEFB_DUMP")
            .ok()
            .filter(|v| !v.is_empty());
        if no_skip {
            info!("simplefb: CROSVM_SIMPLEFB_NO_SKIP=1, every tick presents every band");
        }
        // Conditional on purpose: which transport this screen ends up on is decided later, and on
        // the CPU one this setting does nothing at all.
        info!(
            "simplefb: on the gpu transport this screen will {}",
            if gpu_always_flip {
                "present every tick without reading the framebuffer"
            } else {
                "present only what changed"
            }
        );
        if invalidate {
            #[cfg(target_arch = "aarch64")]
            info!(
                "simplefb: dropping {}-byte cache lines before each read",
                dcache_line_size()
            );
            #[cfg(not(target_arch = "aarch64"))]
            warn!("simplefb: cache invalidation before reads is unimplemented on this architecture");
        } else {
            warn!(
                "simplefb: CROSVM_SIMPLEFB_INVALIDATE=0, reading the framebuffer through the \
                 host's caches -- stale 64-byte lines on screen are expected"
            );
        }
        if let Some(prefix) = &dump_prefix {
            info!("simplefb: dump armed; touch {prefix}.trigger to take one");
        }
        SimplefbDebug {
            no_skip,
            dump_prefix,
            gpu_always_flip,
            invalidate,
        }
    }

    /// True once per appearance of the trigger file, which is removed here so that a dump is a
    /// deliberate act rather than a state the VM can get stuck in.
    fn dump_requested(&self) -> Option<&str> {
        let prefix = self.dump_prefix.as_deref()?;
        let trigger = format!("{prefix}.trigger");
        if std::fs::metadata(&trigger).is_err() {
            return None;
        }
        if let Err(e) = std::fs::remove_file(&trigger) {
            // Left in place, the next tick would dump again and the one after that -- so a trigger
            // that cannot be removed disarms the dump instead of repeating it.
            error!("simplefb: cannot remove {trigger} ({e}); dump not taken");
            return None;
        }
        Some(prefix)
    }
}

/// Writes, for one moment, both halves of the question "is what the sink was given still what the
/// guest holds", and logs which bands disagree.
///
/// What this can and cannot see is worth stating, because an earlier version of it claimed more.
/// It compares `read_buf` -- the bytes the sink was actually handed -- against a full-frame copy of
/// guest memory taken here, per band. A band that differs is an update the watcher has not carried
/// across yet, which on a settled screen is a missed one.
///
/// It CANNOT see a read that lies. Every read it makes reaches guest memory through the same
/// cacheable mapping the watcher uses, so a stale cache line is stale for the dump too and all of
/// its answers agree with each other. That is not hypothetical: it is how the mismatched-attribute
/// staleness this bridge carries was missed three times. What found that was comparing the dumped
/// pixels with a known-good earlier frame and measuring the geometry of what differed -- which is
/// why the two `.bin` files matter more than the counters.
fn dump_watcher_state(
    prefix: &str,
    watcher: &FramebufferWatcher,
    params: &SimplefbDisplayParams,
    read_buf: &[u8],
    fb: VolatileSlice,
) {
    let mut snap = vec![0u8; read_buf.len()];
    fb.copy_to_volatile_slice(VolatileSlice::new(&mut snap));

    let mut disagreeing = 0usize;
    let mut reported = 0usize;
    for band in 0..watcher.bands() {
        let (off, len) = watcher.band_range(band);
        let (Some(a), Some(b)) = (read_buf.get(off..off + len), snap.get(off..off + len)) else {
            continue;
        };
        if a == b {
            continue;
        }
        disagreeing += 1;
        if reported < 24 {
            reported += 1;
            let (first, last) = watcher.band_rows(band);
            let at = a.iter().zip(b).position(|(x, y)| x != y).unwrap_or(0);
            info!(
                "simplefb dump: band {band} rows {first}..{last} differs from guest memory, \
                 first at byte {} of the frame",
                off + at,
            );
        }
    }

    let info_path = format!("{prefix}.info.txt");
    let readbuf_path = format!("{prefix}.readbuf.bin");
    let guest_path = format!("{prefix}.guest.bin");
    let summary = format!(
        "{}x{} stride={} bpp={} fourcc={:#x} bands={} disagreeing={} read_buf_valid={} \
         pending_present={}\n",
        params.width,
        params.height,
        params.stride,
        params.bpp,
        params.fourcc,
        watcher.bands(),
        disagreeing,
        watcher.read_buf_valid,
        watcher.pending_present,
    );
    for (path, bytes) in [
        (&info_path, summary.as_bytes()),
        (&readbuf_path, read_buf),
        (&guest_path, &snap[..]),
    ] {
        if let Err(e) = std::fs::write(path, bytes) {
            error!("simplefb dump: failed to write {path}: {e}");
        }
    }
    info!(
        "simplefb dump: {}/{} band(s) differ from guest memory; wrote {readbuf_path} and \
         {guest_path}",
        disagreeing,
        watcher.bands(),
    );
}

/// What the simplefb bridge remembers between ticks so that an unchanged framebuffer costs nothing
/// downstream.
///
/// There is no signal from this device: the guest maps the region write-combining, the region is
/// shared rather than lent, and Gunyah has no dirty log, so comparing content is the only way to
/// learn that a frame exists.
///
/// The thing compared against is `read_buf` itself -- the bytes the sink was last handed -- so
/// there is no separate record to keep in step with it, and no way for the two to part company.
/// The band hashes this replaced were exactly such a record, and they cost a full-frame FNV-1a per
/// tick to maintain for the privilege.
struct FramebufferWatcher {
    /// Bytes from the start of one row to the start of the next, as the guest lays them out.
    stride: usize,
    /// Bytes of each row that are actually displayed. Row padding is copied along with its band --
    /// `read_buf` mirrors the guest's layout -- but it is never compared: no sink shows those
    /// bytes, so a guest that scribbles in them has not changed the picture.
    visible_bytes: usize,
    height: usize,
    bands: usize,
    /// False when `read_buf` cannot be trusted to describe what the sink is showing, which forces
    /// the next pass to copy and present every band. Set false on every path that can break the
    /// relationship: the first tick, a consumer arriving, and any failure mid-pass.
    ///
    /// The direction of this flag is the whole safety argument. A stale "unchanged" verdict does
    /// not report itself -- it is a region of screen that stays wrong forever with nothing
    /// transmitted to say so, which is exactly the black-client bug the VNC sink's `last_clean`
    /// already produced once (see `vnc_server_resize`). So every uncertainty resolves to "re-read
    /// everything", never to "assume it is still on screen".
    read_buf_valid: bool,
    /// `read_buf` holds something no sink has accepted yet. A frame the sink refused is not lost
    /// here the way a guest flush would be -- this producer is a clock and offers it again on the
    /// next tick -- but only if the skip logic remembers that it never landed.
    pending_present: bool,
    /// Whether a consumer was attached on the previous tick, so that false -> true can be seen.
    had_consumer: bool,
    /// The sink's consumer generation on the previous tick. Separate from `had_consumer` because
    /// it answers a question the bool cannot: a second KIND of client arriving while the first is
    /// still attached leaves the flag true throughout, and that client needs the same forced full
    /// pass as one that arrives to an empty sink. See `DisplayT::consumer_generation`.
    consumer_generation: u64,
    /// When the content last compared differently. Nothing reads it yet; it becomes the screen
    /// definition's liveness field, which is how a user is meant to tell a frozen screen from a
    /// live one without staring at it.
    last_changed_at: Instant,
    last_change_log_at: Instant,
    last_liveness_at: Instant,
}

impl FramebufferWatcher {
    fn new(params: &SimplefbDisplayParams) -> Self {
        let height = params.height as usize;
        let now = Instant::now();
        FramebufferWatcher {
            stride: params.stride as usize,
            visible_bytes: (params.width as usize)
                .saturating_mul(params.bpp as usize)
                .min(params.stride as usize),
            height,
            bands: height.div_ceil(WATCH_BAND_ROWS),
            read_buf_valid: false,
            pending_present: false,
            had_consumer: false,
            consumer_generation: 0,
            last_changed_at: now,
            last_change_log_at: now,
            last_liveness_at: now,
        }
    }

    /// Forget everything believed about what is on screen. Every caller of this is a place where
    /// the sink's buffers and `read_buf` may have parted company.
    fn invalidate(&mut self) {
        self.read_buf_valid = false;
    }

    fn bands(&self) -> usize {
        self.bands
    }

    /// First and last-plus-one row of a band.
    fn band_rows(&self, band: usize) -> (usize, usize) {
        let first = band * WATCH_BAND_ROWS;
        (first, (first + WATCH_BAND_ROWS).min(self.height))
    }

    /// Byte offset and length of a band within a frame laid out at `stride`.
    fn band_range(&self, band: usize) -> (usize, usize) {
        let (first, last) = self.band_rows(band);
        (first * self.stride, (last - first) * self.stride)
    }

    /// Whether a band's visible bytes differ between guest memory and what the sink was last
    /// handed. `None` means the band could not be read, which the caller must treat as changed
    /// rather than as unchanged.
    ///
    /// Row by row rather than band at a time, because a band is only contiguous when the stride is
    /// packed -- and it is not, whenever the row pitch has been padded for the GPU transport. Row
    /// padding is skipped for the same reason it always was: no sink shows those bytes.
    fn band_differs(&self, fb: VolatileSlice, band: usize, read_buf: &[u8]) -> Option<bool> {
        let (first, last) = self.band_rows(band);
        for row in first..last {
            let off = row * self.stride;
            let src = fb.sub_slice(off, self.visible_bytes).ok()?;
            let dst = read_buf.get(off..off + self.visible_bytes)?;
            // SAFETY: `src` is a live mapping of `visible_bytes` and `dst` a slice of that length.
            if unsafe { bytes_differ(src.as_ptr(), dst.as_ptr(), self.visible_bytes) } {
                return Some(true);
            }
        }
        Some(false)
    }

    fn copy_band(&self, fb: VolatileSlice, band: usize, read_buf: &mut [u8]) -> bool {
        let (off, len) = self.band_range(band);
        let Ok(src) = fb.sub_slice(off, len) else {
            return false;
        };
        let Some(dst) = read_buf.get_mut(off..off + len) else {
            return false;
        };
        src.copy_to_volatile_slice(VolatileSlice::new(dst));
        true
    }

    /// The band loop both passes share. Returns how many bands were copied and how many of those
    /// the comparison had called changed -- which on a forced pass is none of them, because a
    /// forced pass does not ask.
    ///
    /// A band that differs is copied immediately, and what lands in `read_buf` is what the next
    /// tick compares against. That is the self-correcting part: the guest writes this region with
    /// no synchronisation of any kind, so a comparison can catch a half-written frame and the copy
    /// can land a different half -- and the tick after it sees the difference and copies again.
    fn pass(&mut self, fb: VolatileSlice, read_buf: &mut [u8], force: bool) -> (usize, usize) {
        let mut copied = 0usize;
        let mut changed = 0usize;
        let mut complete = true;

        for band in 0..self.bands {
            let differs = if force {
                None
            } else {
                self.band_differs(fb, band, read_buf)
            };
            if differs == Some(false) {
                continue;
            }
            if !self.copy_band(fb, band, read_buf) {
                complete = false;
                continue;
            }
            if differs == Some(true) {
                changed += 1;
            }
            copied += 1;
        }

        self.read_buf_valid = complete;
        (copied, changed)
    }

    /// Brings `read_buf` up to date with guest memory and says whether a sink needs to see it.
    ///
    /// `force_all` is the debug switch (`CROSVM_SIMPLEFB_NO_SKIP`) and nothing else sets it: it
    /// takes the band skip out of the picture entirely so that a stale region which survives can
    /// be attributed somewhere other than here.
    fn sync(&mut self, fb: VolatileSlice, read_buf: &mut [u8], force_all: bool) -> bool {
        let force = force_all || !self.read_buf_valid;
        let (copied, changed) = self.pass(fb, read_buf, force);
        if changed > 0 {
            self.note_change(changed, true);
        }
        copied > 0
    }

    /// The tick that runs while nobody is watching: notice that the guest is still painting,
    /// without producing a frame for it.
    ///
    /// It maintains `read_buf` exactly as the watching pass does, and only the present is left out.
    /// The hash-based version could not: its record described guest memory rather than the buffer,
    /// so every path back to a consumer had to go through `invalidate` to undo it. That is still
    /// done on the arrival edge -- a returning viewer is owed a frame whether or not the content
    /// moved -- but it is now a deliberate re-supply rather than a repair.
    fn liveness_pass(&mut self, fb: VolatileSlice, read_buf: &mut [u8]) {
        let force = !self.read_buf_valid;
        let (_copied, changed) = self.pass(fb, read_buf, force);
        self.last_liveness_at = Instant::now();
        if changed > 0 {
            self.note_change(changed, false);
        }
    }

    fn liveness_due(&self) -> bool {
        self.last_liveness_at.elapsed() >= LIVENESS_INTERVAL
    }

    fn note_change(&mut self, bands: usize, watching: bool) {
        let now = Instant::now();
        let quiet = now.saturating_duration_since(self.last_changed_at);
        self.last_changed_at = now;
        if now.saturating_duration_since(self.last_change_log_at) >= CHANGE_LOG_INTERVAL {
            self.last_change_log_at = now;
            debug!(
                "simplefb: {}/{} band(s) changed after {:.1}s still, watching={}",
                bands,
                self.bands(),
                quiet.as_secs_f32(),
                watching,
            );
        }
    }
}

fn simplefb_display_loop(
    guest_mem: GuestMemory,
    params: &SimplefbDisplayParams,
    display: &mut GpuDisplay,
    transport_cap: TransportCap,
    unconditional_present_is_cheap: bool,
) -> Result<()> {
    let display_params = DisplayParameters::default_with_mode(DisplayMode::Windowed(
        params.width,
        params.height,
    ));

    // One surface for the life of the bridge, and one geometry: simplefb's is fixed by the device
    // tree, which is why nothing below recreates or resizes it. Should that ever stop being true,
    // `watcher.invalidate()` has to be called on the same path -- a recreated surface hands out
    // buffers with none of our content in them, and hashes that still say "already on screen"
    // would then skip exactly the bands the guest is not repainting. That is not a subtle
    // degradation, it is a permanently wrong screen with nothing sent to report it; the VNC sink
    // hit the identical failure through `last_clean` across a resize (see `vnc_server_resize`).
    let surface_id = display
        .create_surface(None, None, &display_params, SurfaceType::Scanout)
        .context("failed to create display surface")?;

    info!(
        "simplefb display bridge: {}x{} stride={} bpp={} addr={:#x} @ {}fps",
        params.width, params.height, params.stride, params.bpp, params.addr, params.poll_hz,
    );

    // Decided here, before the loop and before any consumer can exist, so that the outcome is on
    // record for every run. A failure is permanent by design: see `Transport::Cpu`.
    let mut transport = match build_gpu_transport(
        &guest_mem,
        params,
        display,
        surface_id,
        transport_cap,
    ) {
        Ok(gpu) => {
            info!(
                "simplefb: transport=gpu-blit {}x{} stride={} fourcc={:#x} import={}",
                params.width, params.height, params.stride, params.fourcc, gpu.import_id,
            );
            Transport::Gpu(gpu)
        }
        Err(reason) => {
            info!("simplefb: transport=cpu ({reason})");
            Transport::Cpu
        }
    };

    let frame_duration = tick_duration(params.poll_hz);
    let guest_addr = GuestAddress(params.addr);
    let fb_size = (params.stride as usize) * (params.height as usize);
    // Persists across ticks and therefore always holds the last full frame handed to the sink,
    // which is what lets a tick copy only the bands that moved and still present a whole picture.
    let mut read_buf = vec![0u8; fb_size];
    let mut watcher = FramebufferWatcher::new(params);
    let mut no_framebuffer: u64 = 0;
    let debug = SimplefbDebug::from_env(unconditional_present_is_cheap);

    loop {
        let frame_start = Instant::now();

        // Process any pending VNC input events and route to EventDevices.
        if let Err(e) = display.dispatch_events() {
            match e {
                gpu_display::GpuDisplayError::ConnectionBroken => {
                    info!("simplefb: display connection closed, exiting");
                    break;
                }
                gpu_display::GpuDisplayError::IoError(ref ioe)
                    if ioe.kind() == std::io::ErrorKind::WouldBlock =>
                {
                    // Nonblocking input/event sockets may transiently return EAGAIN.
                    // This is not fatal; just retry on the next frame.
                }
                _ => {
                    error!("simplefb: dispatch_events error: {:?}", e);
                    break;
                }
            }
        }

        // Asked every tick, whether or not anything is going to be hashed: it is a mutex and a
        // pointer read, and the answer is what decides between the two regimes below. The
        // dispatch_events above is what notices a VNC client arriving, so both directions of this
        // are picked up within one tick.
        let has_consumer = display.has_consumer();
        let consumer_generation = display.consumer_generation();
        if (has_consumer && !watcher.had_consumer)
            || (has_consumer && consumer_generation != watcher.consumer_generation)
        {
            // A consumer arriving is not a change in content, and that is exactly why it needs
            // saying. Content that sat still while nobody watched hashes as unchanged, so without
            // this the returning viewer is shown whatever its buffers happened to hold until the
            // guest next paints something -- which on a Windows desktop that has not moved since
            // firmware handover may be never. Forcing a full pass here is also what puts the very
            // first frame on screen.
            watcher.invalidate();
        }
        watcher.had_consumer = has_consumer;
        watcher.consumer_generation = consumer_generation;

        // The host mapping of the region, not a copy of it. What replaced the unconditional
        // `read_exact_at_addr` is this plus the band pass: a tick where nothing moved reads each
        // band once to hash it and writes nothing anywhere.
        let fb = match guest_mem.get_slice_at_addr(guest_addr, fb_size) {
            Ok(fb) => fb,
            Err(_) => {
                info!("simplefb: guest memory no longer readable, exiting");
                break;
            }
        };

        // Whether the GPU transport is about to present without anybody looking at the pixels on
        // this side. Asked once, because two things below need the same answer: the branch itself,
        // and the cache invalidation, which is only worth its 64-byte-per-line walk on a tick where
        // a host CPU read is going to act on what it finds.
        let gpu_presents_blind =
            debug.gpu_always_flip && has_consumer && matches!(transport, Transport::Gpu(_));

        // Consumed before the invalidation decision rather than after it: a dump reads the
        // framebuffer, so a tick that takes one is a tick that reads whatever the branch below
        // would have done.
        let dump_prefix = debug.dump_requested();

        // Before anything reads a byte of it, so that every read below -- the comparison, the copy,
        // and the dump's own snapshot -- sees memory rather than a line some earlier tick left
        // behind. Skipped entirely when nothing on this side is going to read: the walk is ~99k
        // `dc civac` for a 1500x1060 framebuffer, and paying it 120 times a second to make a read
        // honest that never happens is the most expensive way to do nothing.
        if debug.invalidate && (!gpu_presents_blind || dump_prefix.is_some()) {
            invalidate_dcache_range(fb.as_ptr(), fb_size);
        }

        // Before `sync`, deliberately: here `read_buf` still holds exactly what the sink was last
        // handed, which is exactly what the comparison below is about to be made against, so
        // the dump describes the decision as it stands rather than after it has been remade. It
        // also runs on the ticks that end early, which is where a missed update would be hiding.
        if let Some(prefix) = dump_prefix {
            dump_watcher_state(prefix, &watcher, params, &read_buf, fb);
        }

        // The whole watcher, skipped. The sink is handed the same pages every tick and works out
        // for itself what moved -- which on the GPU transport it was going to read anyway, and
        // which the VNC ingest already does a second time on host memory (`collect_damaged_bands`),
        // so nothing downstream sends more than it did before.
        //
        // `has_consumer` still gates it: a flip with nobody attached is a Vulkan blit and a buffer
        // post for no one, and the sink's own arrival edge cannot be seen from here.
        if gpu_presents_blind {
            // Same shape as the normal present below: the failure is recorded while the transport
            // is borrowed and acted on after, because replacing `transport` is what the fallback
            // does and that cannot happen through a borrow of it.
            let mut blit_failed: Option<u32> = None;
            if let Transport::Gpu(gpu) = &mut transport {
                gpu.await_previous_flip();
                match display.flip_to(surface_id, gpu.import_id, None, None, None) {
                    Ok(_waitable) => {
                        gpu.pending_fence = display.take_flip_completion_fence(surface_id);
                    }
                    Err(e) => {
                        warn!(
                            "simplefb: gpu blit failed, falling back to cpu copy for the rest of \
                             this VM: {e:#}"
                        );
                        blit_failed = Some(gpu.import_id);
                    }
                }
            }
            if let Some(import_id) = blit_failed {
                display.release_import(import_id, surface_id);
                transport = Transport::Cpu;
                // Nothing has maintained `read_buf` or the hashes while this path was running, so
                // both describe a frame from before the switch -- or no frame at all. The CPU
                // path's very first pass has to copy and present everything.
                watcher.invalidate();
                watcher.pending_present = false;
            }
            let elapsed = frame_start.elapsed();
            if elapsed < frame_duration {
                thread::sleep(frame_duration - elapsed);
            }
            continue;
        }

        if !has_consumer {
            // Nothing downstream is positioned to see a frame -- VNC with no client, or the app
            // having left the display view. Producing one is then work done for nobody, and this
            // producer is a clock, so a skipped frame is re-offered 33 ms later rather than lost.
            //
            // Not stopped altogether, though: dropped to the liveness rate, where the pass only
            // hashes. `last_changed_at` has to keep moving because the moment somebody wants to
            // know which screen is alive is the moment before they attach to one.
            if watcher.liveness_due() {
                watcher.liveness_pass(fb, &mut read_buf);
            }
            let elapsed = frame_start.elapsed();
            if elapsed < frame_duration {
                thread::sleep(frame_duration - elapsed);
            }
            continue;
        }

        if watcher.sync(fb, &mut read_buf, debug.no_skip) {
            watcher.pending_present = true;
        }
        if !watcher.pending_present {
            // Nothing moved. No read of guest memory beyond the hashes, no write to read_buf, and
            // nothing handed to the sink -- which is the part that matters, because a sink that is
            // not called does not wake a compositor, encode anything or touch a GPU.
            let elapsed = frame_start.elapsed();
            if elapsed < frame_duration {
                thread::sleep(frame_duration - elapsed);
            }
            continue;
        }

        // Which transport carries it is settled; what counts as a frame is not affected by it. Both
        // arms clear `pending_present` only on a present that landed, so the attach edge's forced
        // full pass and the `NoFramebuffer` re-offer behave identically either way.
        let mut blit_failed: Option<u32> = None;
        match &mut transport {
            Transport::Gpu(gpu) => {
                // Pace against the previous blit before handing the sink the same pages again.
                gpu.await_previous_flip();
                match display.flip_to(surface_id, gpu.import_id, None, None, None) {
                    Ok(_waitable) => {
                        watcher.pending_present = false;
                        no_framebuffer = 0;
                        // A backend whose blit is asynchronous hands back a sync_file here; one
                        // whose flip is synchronous hands back nothing and the wait above is a
                        // no-op. Collect it either way -- an uncollected fence is closed on the
                        // sink's next flip, which is a leak of exactly one fd per frame.
                        gpu.pending_fence = display.take_flip_completion_fence(surface_id);
                    }
                    Err(e) => {
                        // The blit is the sink's, and a sink that cannot do it now will not be able
                        // to on the next tick either (a lost Vulkan device, a torn-down surface).
                        // Say so once and spend the rest of this VM on the CPU path, which needs no
                        // cooperation from anything: `read_buf` already holds this exact frame, and
                        // `pending_present` is still set, so the very next tick presents it.
                        warn!(
                            "simplefb: gpu blit failed, falling back to cpu copy for the rest of \
                             this VM: {e:#}"
                        );
                        blit_failed = Some(gpu.import_id);
                    }
                }
            }
            Transport::Cpu => {
                // The whole buffer, with full damage, however few bands were copied into it. The
                // narrower present that suggests itself here -- copy only the changed rows into the
                // sink -- is a trap: the destination rotates between buffers and nothing tracks
                // their age, so the rows this frame does not write are whatever some earlier frame
                // left in that particular buffer, not what is currently on screen. `read_buf` is
                // what makes the skip safe, and the skip is where the saving is; narrowing the copy
                // as well would give back a memcpy of host memory in exchange for a class of
                // stale-region bug that reports nothing when it happens.
                let frame = ScanoutFrame {
                    bytes: &read_buf,
                    stride: params.stride,
                    width: params.width,
                    height: params.height,
                    fourcc: params.fourcc,
                    damage: Damage::Full,
                };
                match display.present_frame(surface_id, &frame) {
                    PresentOutcome::Flipped => {
                        watcher.pending_present = false;
                        no_framebuffer = 0;
                    }
                    PresentOutcome::NoFramebuffer => {
                        // No framebuffer to write into: the sink never gave us one (an Android
                        // display whose service lost the name race hands out a surface with no
                        // window behind it) or it is transiently locked. Silence here cost a whole
                        // debugging session -- the bridge looked perfectly healthy while presenting
                        // nothing -- so say it, backing off so a permanent condition does not fill
                        // the log.
                        //
                        // `pending_present` stays set. It has to: the content is in read_buf and
                        // its hash is recorded, so nothing later would call this frame new, and a
                        // dropped frame that never comes back is the failure this producer is
                        // supposed to be immune to.
                        no_framebuffer += 1;
                        if no_framebuffer == 1 || no_framebuffer % 300 == 0 {
                            warn!(
                                "simplefb: no framebuffer from the display sink ({} frame(s) \
                                 dropped)",
                                no_framebuffer
                            );
                        }
                        // The flip on this path is not presenting anything -- it is what releases a
                        // sink that handed back nothing, and it has been here since before the copy
                        // moved out.
                        display.flip(surface_id);
                    }
                }
            }
        }
        if let Some(import_id) = blit_failed {
            display.release_import(import_id, surface_id);
            transport = Transport::Cpu;
        }

        let elapsed = frame_start.elapsed();
        if elapsed < frame_duration {
            thread::sleep(frame_duration - elapsed);
        }
    }

    if let Transport::Gpu(gpu) = &transport {
        display.release_import(gpu.import_id, surface_id);
    }
    display.release_surface(surface_id);
    Ok(())
}
