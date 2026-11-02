// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

//! A display sink that serves one screen to browsers over WebRTC.
//!
//! The frame path is the VNC sink's, minus the readback: a guest dmabuf is imported once, in the
//! byte order the encoder reads, and each flip blits it straight into a MediaCodec input buffer
//! (`EncoderPipeline::feed` with a GPU source and no pixels). Nothing on the CPU touches the
//! picture on that route. A producer with no dmabuf (`flip`) has its pixels uploaded instead.
//!
//! Input arrives on each viewer's `input` DataChannel and goes into this binding's own tablet and
//! keyboard -- the same per-binding devices, scaling and button logic the VNC sink uses.
//!
//! The network half (peers, signalling, the stream's audience) is webrtc_hub.rs; win3d/docs/webrtc
//! has the plan and the measurements behind it.

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::collections::VecDeque;
use std::rc::Rc;
use std::sync::atomic::Ordering;
use std::sync::Arc;

use anyhow::anyhow;
use base::AsRawDescriptor;
use base::Event;
use base::RawDescriptor;
use base::VolatileSlice;
use linux_input_sys::virtio_input_event;

use crate::gpu_display_vnc::pointer_report;
use crate::gpu_display_vnc::VNC_ABS_MAX;
use crate::video_encoder::bitrate_for;
use crate::video_encoder::CursorOverlay;
use crate::video_encoder::EncoderConfig;
use crate::video_encoder::EncoderPipeline;
use crate::video_encoder::FeedFrame;
use crate::video_encoder::VideoCodec;
use crate::vnc_blit::VncBlitContext;
use crate::webrtc_hub::HubConfig;
use crate::webrtc_hub::WebrtcAudience;
use crate::webrtc_hub::WebrtcHub;
use crate::webrtc_hub::WebrtcInput;
use crate::DisplayT;
use crate::EventDevice;
use crate::GpuDisplayError;
use crate::GpuDisplayEvents;
use crate::GpuDisplayFramebuffer;
use crate::GpuDisplayResult;
use crate::GpuDisplaySurface;
use crate::SemaphoreTimepoint;
use crate::SurfaceType;
use crate::SysDisplayT;

/// MediaCodecInfo.CodecProfileLevel.AVCProfileConstrainedBaseline: the profile the hub offers
/// (`42e01f`), and the one every browser decodes -- Firefox on Linux offers nothing else (P0-1).
const AVC_PROFILE_CONSTRAINED_BASELINE: i32 = 0x10000;

/// MediaCodecInfo.CodecProfileLevel.HEVCProfileMain.
const HEVC_PROFILE_MAIN: i32 = 1;

/// The encoder for a codec the hub settled on. No B-frames either way (WebRTC has no reordering
/// to spare); H.264 is Constrained Baseline, which is what the offer promises.
fn encoder_config(codec: VideoCodec, bitrate_bps: Option<u32>, frame_rate: i32) -> EncoderConfig {
    EncoderConfig {
        codec,
        profile: match codec {
            VideoCodec::H264 => AVC_PROFILE_CONSTRAINED_BASELINE,
            VideoCodec::H265 => HEVC_PROFILE_MAIN,
        },
        max_bframes: 0,
        // A backstop only: a viewer that loses its place sends a PLI and is answered within
        // tens of milliseconds, and every IDR is a burst the link has to absorb.
        iframe_interval_secs: WEBRTC_IFRAME_INTERVAL_SECS,
        bitrate_bps: bitrate_bps.unwrap_or(0) as i32,
        // Every frame held in the codec is latency the viewer sees; measured 23 ms on average
        // without this (c2.qti.hevc.encoder, 1080p, ~110 fps offered).
        low_latency: true,
        // The screen's own rate, so clocks and the per-frame bit budget match what is offered.
        frame_rate,
        // Without it the Qualcomm core clock is planned for frame-rate, and a 1080p frame takes
        // most of a 46 fps frame interval to encode.
        max_operating_rate: true,
    }
}

/// Seconds between unrequested IDRs; see `EncoderConfig::iframe_interval_secs`.
const WEBRTC_IFRAME_INTERVAL_SECS: i32 = 10;

/// The lowest bitrate a REMB can push the goal to: below this a 1080p desktop's text smears.
const WEBRTC_MIN_BITRATE: u32 = 500_000;

/// The lowest the encoder itself is set to. Lower than the goal's floor: the setting is the goal
/// corrected for a codec that overshoots it severalfold at high frame rates.
const WEBRTC_MIN_SETTING: u32 = 100_000;

/// A guest dmabuf imported for the encoder's blit.
#[derive(Clone)]
struct WebrtcImport {
    ctx: Arc<VncBlitContext>,
    handle: i64,
    width: u32,
    height: u32,
}

/// The picture last fed, so it can be fed again: a viewer that joins, or asks for a sync frame,
/// while the guest's screen is still would otherwise wait for a frame that is not coming, and the
/// encoder will not repeat one on its own for long enough to cover it (P0-3).
enum LastFrame {
    None,
    Gpu { import_id: u32 },
    Cpu { pixels: Vec<u8>, width: u32, height: u32 },
}

/// What the surfaces and the display share: the pipeline, the imports, and the last frame. Lives
/// on the display thread only (`GpuDisplay` is `!Send`), hence `Rc`/`RefCell`.
struct Feeder {
    pipeline: Arc<EncoderPipeline<WebrtcAudience>>,
    /// For the statistics: every picture offered, and by which route.
    hub: Arc<WebrtcHub>,
    imports: RefCell<BTreeMap<u32, WebrtcImport>>,
    last: RefCell<LastFrame>,
}

impl Feeder {
    /// The GPU route. An `Err` sends the producer to the CPU route for this frame, which is what
    /// has to happen once the device has shown that the blit into a codec buffer does not work.
    fn feed_gpu(&self, import_id: u32, width: u32, height: u32) -> anyhow::Result<()> {
        let import = self
            .imports
            .borrow()
            .get(&import_id)
            .cloned()
            .ok_or_else(|| anyhow!("invalid WebRTC display import id {}", import_id))?;
        if import.width != width || import.height != height {
            return Err(anyhow!(
                "import is {}x{} but this WebRTC screen is {}x{}",
                import.width,
                import.height,
                width,
                height
            ));
        }
        if self.pipeline.gpu_route_failed() {
            return Err(anyhow!("the codec cannot be fed by blit on this device"));
        }
        self.hub.note_fed(true);
        self.pipeline.feed(&FeedFrame {
            pixels: std::ptr::null(),
            size: 0,
            width,
            height,
            gpu_blit_ctx: import.ctx.as_native_ptr(),
            gpu_import_id: import.handle,
            cursor: &CursorOverlay::none(),
        });
        if self.pipeline.gpu_route_failed() {
            return Err(anyhow!("the codec cannot be fed by blit on this device"));
        }
        *self.last.borrow_mut() = LastFrame::Gpu { import_id };
        Ok(())
    }

    /// The CPU route: `pixels` is XRGB8888, packed, `width` x `height`.
    fn feed_cpu(&self, pixels: &[u8], width: u32, height: u32) {
        self.hub.note_fed(false);
        self.pipeline.feed(&FeedFrame {
            pixels: pixels.as_ptr(),
            size: pixels.len() as u32,
            width,
            height,
            gpu_blit_ctx: std::ptr::null_mut(),
            gpu_import_id: 0,
            cursor: &CursorOverlay::none(),
        });
        let mut last = self.last.borrow_mut();
        match &mut *last {
            LastFrame::Cpu {
                pixels: kept,
                width: w,
                height: h,
            } => {
                kept.clear();
                kept.extend_from_slice(pixels);
                *w = width;
                *h = height;
            }
            _ => {
                *last = LastFrame::Cpu {
                    pixels: pixels.to_vec(),
                    width,
                    height,
                }
            }
        }
    }

    /// Offers the last picture again, as what the sync frame the hub's new join generation asks
    /// for is made of (`answer_joins`: requested here or already by the drain thread, never both).
    fn refeed(&self) {
        let gpu = match &*self.last.borrow() {
            LastFrame::None => return,
            LastFrame::Gpu { import_id } => Some(*import_id),
            LastFrame::Cpu {
                pixels,
                width,
                height,
            } => {
                self.pipeline.answer_joins();
                self.pipeline.feed(&FeedFrame {
                    pixels: pixels.as_ptr(),
                    size: pixels.len() as u32,
                    width: *width,
                    height: *height,
                    gpu_blit_ctx: std::ptr::null_mut(),
                    gpu_import_id: 0,
                    cursor: &CursorOverlay::none(),
                });
                None
            }
        };
        if let Some(import_id) = gpu {
            let Some(import) = self.imports.borrow().get(&import_id).cloned() else {
                // Released since; the guest's next flush will carry a newer picture anyway.
                return;
            };
            self.pipeline.answer_joins();
            let _ = self.feed_gpu(import_id, import.width, import.height);
        }
    }
}

struct WebrtcSurface {
    width: u32,
    height: u32,
    buffer: Vec<u8>,
    feeder: Rc<Feeder>,
}

impl GpuDisplaySurface for WebrtcSurface {
    fn framebuffer(&mut self) -> Option<GpuDisplayFramebuffer> {
        Some(GpuDisplayFramebuffer::new(
            VolatileSlice::new(self.buffer.as_mut_slice()),
            self.width * 4,
            4,
            crate::DRM_FORMAT_XRGB8888,
        ))
    }

    fn flip(&mut self) {
        self.feeder.feed_cpu(&self.buffer, self.width, self.height);
    }

    fn flip_to(
        &mut self,
        import_id: u32,
        _acquire_timepoint: Option<SemaphoreTimepoint>,
        _release_timepoint: Option<SemaphoreTimepoint>,
        _extra_info: Option<crate::FlipToExtraInfo>,
    ) -> anyhow::Result<sync::Waitable> {
        self.feeder.feed_gpu(import_id, self.width, self.height)?;
        Ok(sync::Waitable::signaled())
    }

    /// Always `None`, for the VNC sink's reason (gpu_display_vnc.rs): a release fence would put
    /// the network in the guest's vblank loop. The blit into the codec buffer has completed by the
    /// time `flip_to` returns, so the guest's buffer is free.
    fn take_flip_completion_fence(&mut self) -> Option<base::SafeDescriptor> {
        None
    }
}

/// virtio-gpu's cursor plane, drawn by the viewer page rather than into the stream.
///
/// Not composited, unlike VNC's: a page with input draws it as its own pointer (a CSS cursor),
/// which follows the hand with no network in between, and a view-only page draws it where the
/// guest says it is. Either way a pointer move costs a few bytes on the DataChannel instead of an
/// encoded frame, and the picture the encoder sees never has a cursor to smear.
struct WebrtcCursorSurface {
    hub: Arc<WebrtcHub>,
    hot_x: u32,
    hot_y: u32,
    pixels: Vec<u8>,
}

/// virtio-gpu's cursor plane is fixed at this size (see `DisplayVnc::create_surface`).
const CURSOR_SIZE: u32 = 64;

impl GpuDisplaySurface for WebrtcCursorSurface {
    fn framebuffer(&mut self) -> Option<GpuDisplayFramebuffer> {
        Some(GpuDisplayFramebuffer::new(
            VolatileSlice::new(self.pixels.as_mut_slice()),
            CURSOR_SIZE * 4,
            4,
            crate::DRM_FORMAT_ARGB8888,
        ))
    }

    fn flip(&mut self) {
        self.hub.set_cursor_shape(
            CURSOR_SIZE as u16,
            CURSOR_SIZE as u16,
            self.hot_x as u16,
            self.hot_y as u16,
            &self.pixels,
        );
        self.hub.set_cursor_visible(true);
    }

    fn set_cursor_hotspot(&mut self, hot_x: u32, hot_y: u32) {
        self.hot_x = hot_x.min(CURSOR_SIZE - 1);
        self.hot_y = hot_y.min(CURSOR_SIZE - 1);
    }

    /// `(x, y)` is the image's top-left corner; the page is told the pointer, so the hotspot goes
    /// back on here (as VNC does for LibVNCServer).
    fn set_position(&mut self, x: i32, y: i32) {
        self.hub
            .set_cursor_pos(x + self.hot_x as i32, y + self.hot_y as i32);
    }

    fn set_cursor_visible(&mut self, visible: bool) {
        self.hub.set_cursor_visible(visible);
    }
}

pub struct DisplayWebrtc {
    event: Event,
    width: u32,
    height: u32,
    hub: Arc<WebrtcHub>,
    feeder: Rc<Feeder>,
    input_queue: RefCell<VecDeque<WebrtcInput>>,
    prev_button_mask: u8,
    /// This binding's own devices, exactly as the VNC sink has them (see `DisplayVnc::tablet`).
    /// Both `None` on a `view-only=true` binding.
    tablet: Option<EventDevice>,
    keyboard: Option<EventDevice>,
    blit: Option<Arc<VncBlitContext>>,
    blit_probed: bool,
    /// `bitrate=`: the encoder's starting rate and the REMB ceiling, instead of the derived one.
    bitrate_bps: Option<u32>,
    /// `adapt=`: whether the viewers' REMB may lower the rate; `false` holds it at the ceiling.
    adapt: bool,
    /// Frames per second the encoder is told: virtio-gpu refresh-rate or simplefb poll-hz.
    frame_rate: i32,
}

impl DisplayWebrtc {
    /// `addr` is where the signalling endpoint and the viewer page listen. A port that cannot be
    /// bound is `GpuDisplayError::Listen`, which stops the VM: the user asked for this endpoint.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        addr: &str,
        width: u32,
        height: u32,
        token: Option<String>,
        stun: Vec<String>,
        allow_h265: bool,
        bitrate_bps: Option<u32>,
        adapt: bool,
        frame_rate: u32,
        tablet: Option<EventDevice>,
        keyboard: Option<EventDevice>,
    ) -> GpuDisplayResult<DisplayWebrtc> {
        let frame_rate = frame_rate.min(i32::MAX as u32) as i32;
        let event = Event::new().map_err(|_| GpuDisplayError::CreateEvent)?;
        let wake = event.try_clone().map_err(|_| GpuDisplayError::CreateEvent)?;
        let input_enabled = tablet.is_some() || keyboard.is_some();
        let config = HubConfig {
            stun,
            token,
            allow_h265,
            input_enabled,
        };
        let hub = WebrtcHub::start(addr, config, wake).map_err(|e| {
            base::error!("WebRTC: cannot listen on {}: {}", addr, e);
            GpuDisplayError::Listen(addr.to_string())
        })?;
        hub.set_screen_size(width, height);
        let config = encoder_config(VideoCodec::H264, bitrate_bps, frame_rate);
        let pipeline = EncoderPipeline::start("WebRTC h264", config, WebrtcAudience(hub.clone()))
            .map_err(|e| {
                base::error!("WebRTC: cannot start the encoder thread: {}", e);
                hub.shutdown();
                GpuDisplayError::Allocate
            })?;
        hub.set_epoch(pipeline.epoch());
        let feeder_hub = hub.clone();
        base::info!(
            "WebRTC: screen {}x{}, input -> {}",
            width,
            height,
            match (tablet.is_some(), keyboard.is_some()) {
                (true, true) => "this binding's own tablet + keyboard",
                (true, false) => "this binding's own tablet (no keyboard)",
                (false, true) => "this binding's own keyboard (no tablet)",
                (false, false) => "dropped (view-only)",
            }
        );
        Ok(DisplayWebrtc {
            event,
            width,
            height,
            hub,
            feeder: Rc::new(Feeder {
                pipeline,
                hub: feeder_hub,
                imports: RefCell::new(BTreeMap::new()),
                last: RefCell::new(LastFrame::None),
            }),
            input_queue: RefCell::new(VecDeque::new()),
            prev_button_mask: 0,
            tablet,
            keyboard,
            blit: None,
            blit_probed: false,
            bitrate_bps,
            adapt,
            frame_rate,
        })
    }

    /// Retargets the encoder to the viewers' estimate, between a floor that keeps text legible and
    /// the configured rate (`bitrate=`), else the default for the size (more than that buys
    /// nothing on a desktop; see `bitrate_for`).
    /// `None` is "no estimate", which is the default. Run on the display thread, which owns the
    /// pipeline handle; a refusal (no live encoder yet) is retried on the next REMB.
    fn apply_bitrate(&self, target: Option<u32>) {
        let max = self
            .bitrate_bps
            .unwrap_or_else(|| bitrate_for(self.width, self.height) as u32);
        let goal = target.map(|t| t.clamp(WEBRTC_MIN_BITRATE, max)).unwrap_or(max);
        if self.hub.bitrate_goal.swap(goal, Ordering::Relaxed) != goal {
            base::info!(
                "WebRTC: bitrate goal {} kbit/s (estimate {:?}, ceiling {})",
                goal / 1000,
                target.map(|t| t / 1000),
                max / 1000
            );
        }
        // What the codec is told: the goal, scaled by the correction `WebrtcHub::correct_bitrate`
        // learnt from what it actually produced.
        let factor = self.hub.rate_factor.load(Ordering::Relaxed) as u64;
        let setting = ((goal as u64 * factor / 1000) as u32).max(WEBRTC_MIN_SETTING);
        let applied = &self.hub.bitrate_applied;
        let previous = applied.load(Ordering::Relaxed);
        if setting == previous || (previous == 0 && setting == max) {
            return;
        }
        if self.feeder.pipeline.set_bitrate(setting as i32) {
            applied.store(setting, Ordering::Relaxed);
            base::debug!("WebRTC: encoder set to {} kbit/s (correction {})", setting / 1000, factor);
        }
    }

    /// The Vulkan half, brought up once on first use; see `DisplayVnc::blit_context`.
    fn blit_context(&mut self) -> Option<&Arc<VncBlitContext>> {
        if !self.blit_probed {
            self.blit_probed = true;
            self.blit = VncBlitContext::open(self.width, self.height);
            match &self.blit {
                Some(_) => base::info!("WebRTC: GPU transport available"),
                None => base::info!("WebRTC: no GPU transport; frames will be uploaded by the CPU"),
            }
        }
        self.blit.as_ref()
    }

    fn inject_next_event(&mut self) {
        let Some(ev) = self.input_queue.borrow_mut().pop_front() else {
            return;
        };
        match ev {
            WebrtcInput::Key { code, down } => {
                let Some(keyboard) = &mut self.keyboard else {
                    return;
                };
                if let Err(e) =
                    keyboard.send_report([virtio_input_event::key(code, down, false)].into_iter())
                {
                    base::error!("WebRTC: keyboard event dropped: {}", e);
                }
            }
            WebrtcInput::Pointer { buttons, x, y } => {
                let scale = |v: u16| (v as i64 * VNC_ABS_MAX as i64 / 65535) as i32;
                // Tracked even when view-only, for the reason `DisplayVnc::inject_next_event`
                // gives: a button held across input coming back must not release unpressed.
                let events =
                    pointer_report(&mut self.prev_button_mask, buttons & 0x07, scale(x), scale(y));
                let Some(tablet) = &mut self.tablet else {
                    return;
                };
                if let Err(e) = tablet.send_report(events.into_iter()) {
                    base::error!("WebRTC: pointer event dropped: {}", e);
                }
            }
            WebrtcInput::Wheel { steps } => {
                let Some(tablet) = &mut self.tablet else {
                    return;
                };
                if let Err(e) =
                    tablet.send_report([virtio_input_event::wheel(steps as i32)].into_iter())
                {
                    base::error!("WebRTC: wheel event dropped: {}", e);
                }
            }
        }
    }
}

impl DisplayT for DisplayWebrtc {
    fn is_dmabuf_import_supported(&mut self) -> bool {
        self.blit_context().is_some()
    }

    /// At least one viewer whose track is open. With none, a frame offered now reaches nobody.
    fn has_consumer(&self) -> bool {
        self.feeder.pipeline.wants_frames()
    }

    /// Moves when a viewer starts streaming (and on a PLI), so a timer-driven producer re-supplies
    /// a frame for it.
    fn consumer_generation(&self) -> u64 {
        self.feeder.pipeline.connect_generation()
    }

    /// Runs first on every wakeup of this display's event: collects viewer input, and offers the
    /// last picture again if a viewer needs a sync frame.
    fn flush(&self) {
        let _ = self.event.wait_timeout(std::time::Duration::ZERO);
        self.hub.take_input(&mut self.input_queue.borrow_mut());
        // Before the refeed: a codec decision comes with a sync request, and the picture offered
        // for it has to reach the encoder built for the new codec.
        if let Some(codec) = self.hub.take_codec_change() {
            self.feeder
                .pipeline
                .set_config(encoder_config(codec, self.bitrate_bps, self.frame_rate));
            base::info!("WebRTC: encoder will be rebuilt for {:?}", codec);
        }
        if self.hub.take_refeed() {
            self.feeder.refeed();
        }
        // Still taken with `adapt=false`: it also carries the rate correction moving.
        if let Some(target) = self.hub.bitrate_target() {
            self.apply_bitrate(target.filter(|_| self.adapt));
        }
    }

    fn pending_events(&self) -> bool {
        !self.input_queue.borrow().is_empty()
    }

    fn next_event(&mut self) -> GpuDisplayResult<u64> {
        Ok(0)
    }

    fn handle_next_event(
        &mut self,
        _surface: &mut Box<dyn GpuDisplaySurface>,
    ) -> Option<GpuDisplayEvents> {
        self.inject_next_event();
        None
    }

    fn handle_next_event_without_surface(&mut self) -> Option<GpuDisplayEvents> {
        self.inject_next_event();
        None
    }

    fn create_surface(
        &mut self,
        parent_surface_id: Option<u32>,
        _surface_id: u32,
        _scanout_id: Option<u32>,
        display_params: &crate::DisplayParameters,
        surf_type: SurfaceType,
    ) -> GpuDisplayResult<Box<dyn GpuDisplaySurface>> {
        if parent_surface_id.is_some() {
            if !matches!(surf_type, SurfaceType::Cursor) {
                return Err(GpuDisplayError::Unsupported);
            }
            return Ok(Box::new(WebrtcCursorSurface {
                hub: self.hub.clone(),
                hot_x: 0,
                hot_y: 0,
                pixels: vec![0u8; (CURSOR_SIZE * CURSOR_SIZE * 4) as usize],
            }));
        }
        let (req_w, req_h) = display_params.get_virtual_display_size();
        if req_w != 0 && req_h != 0 {
            self.width = req_w;
            self.height = req_h;
        }
        base::info!("WebRTC: created surface {}x{}", self.width, self.height);
        self.hub.set_screen_size(self.width, self.height);
        Ok(Box::new(WebrtcSurface {
            width: self.width,
            height: self.height,
            buffer: vec![0u8; (self.width as usize) * (self.height as usize) * 4],
            feeder: self.feeder.clone(),
        }))
    }

    /// One import per dmabuf, in the byte order the encoder reads (`exchange_red_blue=false`);
    /// there is no readback here, so the VNC sink's second, RFB-ordered view has no reader.
    fn import_resource(
        &mut self,
        import_id: u32,
        _surface_id: u32,
        external_display_resource: crate::DisplayExternalResourceImport,
    ) -> anyhow::Result<()> {
        let crate::DisplayExternalResourceImport::Dmabuf {
            descriptor,
            offset,
            stride,
            modifiers,
            linear_layout_verified,
            width,
            height,
            fourcc,
        } = external_display_resource
        else {
            return Err(anyhow!("the WebRTC sink only imports DMA-BUFs"));
        };
        let ctx = self
            .blit_context()
            .ok_or_else(|| anyhow!("this WebRTC display has no GPU half"))?
            .clone();
        let handle = ctx
            .import_dmabuf(
                descriptor.as_raw_descriptor(),
                offset,
                stride,
                modifiers,
                linear_layout_verified,
                width,
                height,
                fourcc,
                /* exchange_red_blue= */ false,
            )
            .ok_or_else(|| anyhow!("Turnip DMA-BUF import failed"))?;
        self.feeder.imports.borrow_mut().insert(
            import_id,
            WebrtcImport {
                ctx,
                handle,
                width,
                height,
            },
        );
        Ok(())
    }

    fn release_import(&mut self, import_id: u32, _surface_id: u32) {
        if let Some(import) = self.feeder.imports.borrow_mut().remove(&import_id) {
            import.ctx.release_import(import.handle);
        }
    }
}

impl Drop for DisplayWebrtc {
    fn drop(&mut self) {
        // Peers first: their callbacks point at the hub, and the encoder thread feeds them.
        self.hub.shutdown();
        self.feeder.pipeline.shutdown();
    }
}

impl SysDisplayT for DisplayWebrtc {}

impl AsRawDescriptor for DisplayWebrtc {
    fn as_raw_descriptor(&self) -> RawDescriptor {
        self.event.as_raw_descriptor()
    }
}
