// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

use std::sync::Arc;
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
use base::VolatileSlice;
use base::WaitContext;
use gpu_display::Damage;
use gpu_display::EventDevice;
use gpu_display::GpuDisplay;
use gpu_display::GpuDisplayExt;
use gpu_display::PresentOutcome;
use gpu_display::ScanoutFrame;
use gpu_display::SurfaceType;
use vm_control::gpu::DisplayMode;
use vm_control::gpu::DisplayParameters;
use vm_memory::GuestAddress;
use vm_memory::GuestMemory;

use devices::virtio::ExternalScanout;

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
/// B,G,R,A -- the CPU pipeline's canonical order -- which is precisely why the bridge's plain copy
/// has always produced the right colours without a swizzle anywhere on this route. Naming it turns
/// that from a coincidence nobody wrote down into a field a sink can read, which is what the GPU
/// path will need when it picks a VkFormat from the fourcc instead of assuming BGRX.
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

/// Where simplefb frames go. Everything past opening the display is
/// backend-agnostic: the bridge only uses framebuffer()/flip(), so any
/// GpuDisplay backend works.
pub enum SimplefbDisplayTarget {
    Vnc {
        addr: String,
        password: Option<String>,
        /// Same `--vnc-server input=` interpretation as the virtio-gpu display path (see
        /// vnc_touch_input): true = legacy multi-touch, false = absolute-mouse tablet.
        touch_input: bool,
    },
    /// The Android Surface the app hands over through the display service binder. Input does
    /// NOT come through the display here -- it arrives on the `--input` evdev sockets, same as
    /// the virtio-gpu native-display path.
    Android { service_name: String },
    /// The virtio-gpu device's own display. Used whenever this VM has a GPU: there is one
    /// Surface, so there must be one writer, and the device is it -- we hand frames over and it
    /// shows them while the guest is not displaying through virtio-gpu itself. See
    /// `devices::virtio::ExternalScanout`.
    GpuDevice { scanout: Arc<ExternalScanout> },
}

/// Turns the configured poll rate into the interval between ticks. The rate is validated at parse
/// time; the clamp is here so that this arithmetic cannot be the thing that decides what a bad
/// value means.
fn tick_duration(poll_hz: u32) -> Duration {
    Duration::from_nanos(1_000_000_000 / poll_hz.max(1) as u64)
}

/// Feeds guest framebuffer frames to the GPU device, which owns the one display.
///
/// Skips the copy entirely while the guest is displaying through virtio-gpu (the device tells us
/// so), and only submits when the framebuffer actually changed -- a frozen firmware framebuffer
/// behind a live guest desktop must not keep waking the worker, and an unchanged frame is not a
/// reason to take the display away from anyone.
fn simplefb_feed_loop(
    guest_mem: GuestMemory,
    params: &SimplefbDisplayParams,
    scanout: Arc<ExternalScanout>,
) -> Result<()> {
    let frame_duration = tick_duration(params.poll_hz);
    let guest_addr = GuestAddress(params.addr);
    let fb_size = (params.stride as usize) * (params.height as usize);
    let mut read_buf = vec![0u8; fb_size];
    let mut last_buf: Vec<u8> = Vec::new();

    info!(
        "simplefb: feeding the gpu display: {}x{} stride={} addr={:#x} @ {}fps",
        params.width, params.height, params.stride, params.addr, params.poll_hz,
    );

    let mut idle_pokes: u32 = 0;
    loop {
        let frame_start = Instant::now();
        if scanout.guest_owns() {
            // Nothing to send while the guest is driving the display -- but ownership can also
            // lapse on a clock (a guest that bound a scanout and stopped presenting), and that is
            // only ever re-evaluated on the worker's side of this event. Poke it about once a
            // second so a lapse is noticed without copying a frame nobody will look at.
            idle_pokes += 1;
            if idle_pokes >= params.poll_hz.max(1) {
                idle_pokes = 0;
                scanout.poke();
            }
        } else {
            if idle_pokes != 0 {
                // Just took the display back. The framebuffer may not have changed a byte since
                // we last looked at it (a Windows guest that has been sitting on the same picture
                // since the firmware handed over), and "unchanged" must not mean "not shown" on
                // the frame where the display became ours.
                idle_pokes = 0;
                last_buf.clear();
            }
            if guest_mem
                .read_exact_at_addr(&mut read_buf, guest_addr)
                .is_err()
            {
                info!("simplefb: guest memory no longer readable, exiting");
                break;
            }
            if last_buf != read_buf {
                last_buf.clear();
                last_buf.extend_from_slice(&read_buf);
                scanout.submit(&read_buf);
            }
        }
        let elapsed = frame_start.elapsed();
        if elapsed < frame_duration {
            thread::sleep(frame_duration - elapsed);
        }
    }
    Ok(())
}

pub fn start_simplefb_display_thread(
    guest_mem: GuestMemory,
    params: SimplefbDisplayParams,
    target: SimplefbDisplayTarget,
    event_devices: Vec<EventDevice>,
) -> Result<thread::JoinHandle<()>> {
    thread::Builder::new()
        .name("simplefb_display".into())
        .spawn(move || {
            // Handing frames to the GPU device needs no display of our own -- it owns the one
            // Surface, so this thread is a producer, not a presenter.
            if let SimplefbDisplayTarget::GpuDevice { scanout } = &target {
                if let Err(e) = simplefb_feed_loop(guest_mem, &params, scanout.clone()) {
                    error!("simplefb feed thread exited with error: {:?}", e);
                }
                return;
            }

            let display_result = match &target {
                SimplefbDisplayTarget::Vnc {
                    addr,
                    password,
                    touch_input,
                } => GpuDisplay::open_vnc_tcp(
                    addr,
                    params.width,
                    params.height,
                    password.clone(),
                    *touch_input,
                ),
                SimplefbDisplayTarget::Android { service_name } => {
                    GpuDisplay::open_android(service_name)
                }
                // Handled above; it never opens a display.
                SimplefbDisplayTarget::GpuDevice { .. } => unreachable!(),
            };
            let mut display = match display_result {
                Ok(d) => d,
                Err(e) => {
                    error!("simplefb: failed to open display: {:?}", e);
                    return;
                }
            };

            // Routes VNC input to the guest. The Android backend has no input of its own (the
            // app drives the `--input` evdev sockets instead), so this is a no-op there.
            for ed in event_devices {
                if let Err(e) = display.import_event_device(ed) {
                    error!("simplefb: failed to import event device: {:?}", e);
                }
            }

            if let Err(e) = simplefb_display_loop(guest_mem, &params, &mut display) {
                error!("simplefb display thread exited with error: {:?}", e);
            }
        })
        .context("failed to spawn simplefb display thread")
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

/// How much of a band is pulled out of guest memory at a time to be hashed.
///
/// The hash needs the bytes in a normal slice and guest memory is only reachable through volatile
/// accesses, so they come across in chunks that stay in L1 rather than in one buffer the size of a
/// band. This is not the copy that the watcher exists to avoid: nothing downstream sees it, it does
/// not touch `read_buf`, and it is gone by the end of the band.
const HASH_CHUNK_BYTES: usize = 4096;

/// How often the watcher looks at the framebuffer while nothing is watching the screen.
///
/// Bookkeeping, not a setting: what it buys is that `last_changed_at` is roughly right at the
/// moment somebody asks which screen is alive, and that moment is precisely when no sink is
/// attached yet. Stopping altogether would make the answer oldest exactly when it is needed.
const LIVENESS_INTERVAL: Duration = Duration::from_secs(1);

/// Floor on how often the change log line may repeat. Content that is genuinely moving changes on
/// every tick, and a line per tick is not observability.
const CHANGE_LOG_INTERVAL: Duration = Duration::from_secs(5);

const FNV1A64_OFFSET_BASIS: u64 = 0xcbf2_9ce4_8422_2325;
const FNV1A64_PRIME: u64 = 0x0000_0100_0000_01b3;

/// FNV-1a 64, the same hash both sinks use for their frame diagnostics. Nothing compares a watcher
/// hash with a sink hash -- these never leave this file -- but there is no reason for a tree to
/// carry two answers to "how do we hash a frame here".
fn fnv1a64(mut hash: u64, bytes: &[u8]) -> u64 {
    for b in bytes {
        hash = (hash ^ *b as u64).wrapping_mul(FNV1A64_PRIME);
    }
    hash
}

/// What the simplefb bridge remembers between ticks so that an unchanged framebuffer costs nothing
/// downstream.
///
/// There is no signal from this device: the guest maps the region write-combining, the region is
/// shared rather than lent, and Gunyah has no dirty log, so comparing content is the only way to
/// learn that a frame exists. Storing a hash per band rather than the previous frame is what makes
/// that affordable -- a few hundred bytes instead of a second full-size buffer, and no write at all
/// on a tick where nothing moved.
///
/// The hashes describe `read_buf`, i.e. what the sink was last given, not what guest memory last
/// held. The one exception is the liveness pass, which has no `read_buf` to describe and is always
/// followed by a forced full pass before anything is presented again.
struct FramebufferWatcher {
    /// Bytes from the start of one row to the start of the next, as the guest lays them out.
    stride: usize,
    /// Bytes of each row that are actually displayed. Row padding is copied along with its band --
    /// `read_buf` mirrors the guest's layout -- but it is never hashed: no sink shows those bytes,
    /// so a guest that scribbles in them has not changed the picture.
    visible_bytes: usize,
    height: usize,
    /// FNV-1a of each band's visible bytes.
    band_hashes: Vec<u64>,
    /// False when `band_hashes` cannot be trusted to describe `read_buf`, which forces the next
    /// pass to copy and present every band. Set false on every path that can invalidate the
    /// relationship: the first tick, a consumer arriving, and any failure mid-pass.
    ///
    /// The direction of this flag is the whole safety argument. A stale "unchanged" verdict does
    /// not report itself -- it is a region of screen that stays wrong forever with nothing
    /// transmitted to say so, which is exactly the black-client bug the VNC sink's `last_clean`
    /// already produced once (see `vnc_server_resize`). So every uncertainty resolves to "re-read
    /// everything", never to "assume it is still on screen".
    hashes_valid: bool,
    /// `read_buf` holds something no sink has accepted yet. A frame the sink refused is not lost
    /// here the way a guest flush would be -- this producer is a clock and offers it again on the
    /// next tick -- but only if the skip logic remembers that it never landed.
    pending_present: bool,
    /// Whether a consumer was attached on the previous tick, so that false -> true can be seen.
    had_consumer: bool,
    /// When the content last hashed differently. Nothing reads it yet; it becomes the screen
    /// definition's liveness field, which is how a user is meant to tell a frozen screen from a
    /// live one without staring at it.
    last_changed_at: Instant,
    last_change_log_at: Instant,
    last_liveness_at: Instant,
}

impl FramebufferWatcher {
    fn new(params: &SimplefbDisplayParams) -> Self {
        let height = params.height as usize;
        let bands = height.div_ceil(WATCH_BAND_ROWS);
        let now = Instant::now();
        FramebufferWatcher {
            stride: params.stride as usize,
            visible_bytes: (params.width as usize)
                .saturating_mul(params.bpp as usize)
                .min(params.stride as usize),
            height,
            band_hashes: vec![0; bands],
            hashes_valid: false,
            pending_present: false,
            had_consumer: false,
            last_changed_at: now,
            last_change_log_at: now,
            last_liveness_at: now,
        }
    }

    /// Forget everything believed about what is on screen. Every caller of this is a place where
    /// the sink's buffers and our hashes may have parted company.
    fn invalidate(&mut self) {
        self.hashes_valid = false;
    }

    fn bands(&self) -> usize {
        self.band_hashes.len()
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

    /// Hashes a band as it stands in guest memory. `None` means the band could not be read, which
    /// the caller must treat as changed rather than as unchanged.
    fn hash_band_in_guest(
        &self,
        fb: VolatileSlice,
        band: usize,
        scratch: &mut [u8; HASH_CHUNK_BYTES],
    ) -> Option<u64> {
        let (first, last) = self.band_rows(band);
        let mut hash = FNV1A64_OFFSET_BASIS;
        for row in first..last {
            let mut done = 0;
            while done < self.visible_bytes {
                let len = (self.visible_bytes - done).min(HASH_CHUNK_BYTES);
                let src = fb.sub_slice(row * self.stride + done, len).ok()?;
                src.copy_to_volatile_slice(VolatileSlice::new(&mut scratch[..len]));
                hash = fnv1a64(hash, &scratch[..len]);
                done += len;
            }
        }
        Some(hash)
    }

    /// Hashes a band as it stands in `read_buf`, i.e. in what a sink would be shown.
    fn hash_band_in_buf(&self, band: usize, read_buf: &[u8]) -> u64 {
        let (first, last) = self.band_rows(band);
        let mut hash = FNV1A64_OFFSET_BASIS;
        for row in first..last {
            let off = row * self.stride;
            if let Some(bytes) = read_buf.get(off..off + self.visible_bytes) {
                hash = fnv1a64(hash, bytes);
            }
        }
        hash
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

    /// Brings `read_buf` up to date with guest memory and says whether a sink needs to see it.
    ///
    /// Per band: hash it where it lies, and if the hash matches the stored one, do nothing at all
    /// -- no write to `read_buf`, no second read of guest memory. A band that differs is copied
    /// immediately, while it is still in cache, and the hash then stored is the hash of the bytes
    /// that landed in `read_buf`, not the one just computed from guest memory. The guest writes
    /// this region with no synchronisation of any kind, so the two can disagree; storing the
    /// pre-copy hash would leave `read_buf` holding bytes whose hash is not recorded anywhere, and
    /// the same band would be re-copied on every tick for as long as the content kept moving.
    /// Storing what was copied is self-correcting: whatever ended up in `read_buf` is what the next
    /// tick compares against, and a torn frame is fixed by the tick after it.
    fn sync(
        &mut self,
        fb: VolatileSlice,
        read_buf: &mut [u8],
        scratch: &mut [u8; HASH_CHUNK_BYTES],
    ) -> bool {
        let force = !self.hashes_valid;
        let mut copied = 0usize;
        let mut changed = 0usize;
        let mut complete = true;

        for band in 0..self.bands() {
            if !force {
                if let Some(hash) = self.hash_band_in_guest(fb, band, scratch) {
                    if hash == self.band_hashes[band] {
                        continue;
                    }
                }
            }
            if !self.copy_band(fb, band, read_buf) {
                complete = false;
                continue;
            }
            let hash = self.hash_band_in_buf(band, read_buf);
            if hash != self.band_hashes[band] {
                changed += 1;
                self.band_hashes[band] = hash;
            }
            copied += 1;
        }

        self.hashes_valid = complete;
        if changed > 0 {
            self.note_change(changed, true);
        }
        copied > 0
    }

    /// The tick that runs while nobody is watching: notice that the guest is still painting,
    /// without producing a frame for it.
    ///
    /// This updates the hashes and nothing else -- no copy into `read_buf`, no present -- which is
    /// why every path back to a consumer goes through `invalidate` first. The hashes left here
    /// describe guest memory, and the frame the returning viewer must be sent is a fresh one.
    fn liveness_pass(&mut self, fb: VolatileSlice, scratch: &mut [u8; HASH_CHUNK_BYTES]) {
        let known = self.hashes_valid;
        let mut changed = 0usize;
        let mut complete = true;

        for band in 0..self.bands() {
            match self.hash_band_in_guest(fb, band, scratch) {
                Some(hash) => {
                    if known && hash != self.band_hashes[band] {
                        changed += 1;
                    }
                    self.band_hashes[band] = hash;
                }
                None => complete = false,
            }
        }

        self.hashes_valid = complete;
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

    let frame_duration = tick_duration(params.poll_hz);
    let guest_addr = GuestAddress(params.addr);
    let fb_size = (params.stride as usize) * (params.height as usize);
    // Persists across ticks and therefore always holds the last full frame handed to the sink,
    // which is what lets a tick copy only the bands that moved and still present a whole picture.
    let mut read_buf = vec![0u8; fb_size];
    let mut scratch = [0u8; HASH_CHUNK_BYTES];
    let mut watcher = FramebufferWatcher::new(params);
    let mut no_framebuffer: u64 = 0;

    info!(
        "simplefb display bridge: {}x{} stride={} bpp={} addr={:#x} @ {}fps",
        params.width, params.height, params.stride, params.bpp, params.addr, params.poll_hz,
    );

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
        if has_consumer && !watcher.had_consumer {
            // A consumer arriving is not a change in content, and that is exactly why it needs
            // saying. Content that sat still while nobody watched hashes as unchanged, so without
            // this the returning viewer is shown whatever its buffers happened to hold until the
            // guest next paints something -- which on a Windows desktop that has not moved since
            // firmware handover may be never. Forcing a full pass here is also what puts the very
            // first frame on screen.
            watcher.invalidate();
        }
        watcher.had_consumer = has_consumer;

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

        if !has_consumer {
            // Nothing downstream is positioned to see a frame -- VNC with no client, or the app
            // having left the display view. Producing one is then work done for nobody, and this
            // producer is a clock, so a skipped frame is re-offered 33 ms later rather than lost.
            //
            // Not stopped altogether, though: dropped to the liveness rate, where the pass only
            // hashes. `last_changed_at` has to keep moving because the moment somebody wants to
            // know which screen is alive is the moment before they attach to one.
            if watcher.liveness_due() {
                watcher.liveness_pass(fb, &mut scratch);
            }
            let elapsed = frame_start.elapsed();
            if elapsed < frame_duration {
                thread::sleep(frame_duration - elapsed);
            }
            continue;
        }

        if watcher.sync(fb, &mut read_buf, &mut scratch) {
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

        // The whole buffer, with full damage, however few bands were copied into it. The narrower
        // present that suggests itself here -- copy only the changed rows into the sink -- is a
        // trap: the destination rotates between buffers and nothing tracks their age, so the rows
        // this frame does not write are whatever some earlier frame left in that particular
        // buffer, not what is currently on screen. `read_buf` is what makes the skip safe, and the
        // skip is where the saving is; narrowing the copy as well would give back a memcpy of host
        // memory in exchange for a class of stale-region bug that reports nothing when it happens.
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
                // No framebuffer to write into: the sink never gave us one (an Android display
                // whose service lost the name race hands out a surface with no window behind it)
                // or it is transiently locked. Silence here cost a whole debugging session -- the
                // bridge looked perfectly healthy while presenting nothing -- so say it, backing
                // off so a permanent condition does not fill the log.
                //
                // `pending_present` stays set. It has to: the content is in read_buf and its hash
                // is recorded, so nothing later would call this frame new, and a dropped frame
                // that never comes back is the failure this producer is supposed to be immune to.
                no_framebuffer += 1;
                if no_framebuffer == 1 || no_framebuffer % 300 == 0 {
                    warn!(
                        "simplefb: no framebuffer from the display sink ({} frame(s) dropped)",
                        no_framebuffer
                    );
                }
                // The flip on this path is not presenting anything -- it is what releases a sink
                // that handed back nothing, and it has been here since before the copy moved out.
                display.flip(surface_id);
            }
        }

        let elapsed = frame_start.elapsed();
        if elapsed < frame_duration {
            thread::sleep(frame_duration - elapsed);
        }
    }

    display.release_surface(surface_id);
    Ok(())
}
