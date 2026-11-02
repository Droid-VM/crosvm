// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

//! A hardware video encoder fed from a display sink, and the thread that drains it.
//!
//! This is the codec half of what used to be the VNC sink's H.264 rung (vnc_h264.rs), taken out so
//! that a second kind of audience -- WebRTC peers -- can be served by the same machinery. What it
//! does not know is who is watching: that is an `EncodedAudience`, which says whether anybody wants
//! frames, when somebody new has joined, and what to do with each compressed unit. The RFB-50
//! broadcaster is one; a WebRTC session table is another.
//!
//! The encoder itself is MediaCodec, reached through `libcrosvm_android_display_client`
//! (crosvm_android_display_client.cpp). H.264 and H.265 come out the same way -- Annex-B, the
//! parameter sets once in a CODEC_CONFIG unit, sync frames flagged by the codec -- so nothing here
//! parses a NAL unit.
//!
//! Parts of the API -- H.265, profiles, bitrate retargeting, explicit sync requests, the cached
//! parameter sets -- have no caller until the WebRTC backend lands; they are marked as such rather
//! than left for a later change to rediscover.

use std::ffi::c_char;
use std::ffi::c_void;
use std::ffi::CStr;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::AtomicU64;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::sync::Mutex;
use std::thread;
use std::time::Duration;
use std::time::Instant;

/// Frames per second declared to the encoder, and the rate the bitrate is scaled against.
///
/// Not a cap on anything: the producer's offer rate is what it is (the guest's flush rate, or the
/// simplefb timer), and this number only tells rate control what to expect. Declaring 30 while
/// receiving 60 makes the encoder spend more bits than asked; declaring 30 while receiving 5 makes
/// it spend fewer. Both are recoverable, and neither is worth a measurement loop here.
const ENCODER_FRAME_RATE: i32 = 30;

/// Seconds between the encoder's own IDR frames.
///
/// The interval only has to cover "a decoder that lost its place"; a viewer that joins is given an
/// explicit sync frame the moment it does, so this is a backstop rather than the join mechanism.
const ENCODER_IFRAME_INTERVAL_SECS: i32 = 2;

/// Bits per pixel per frame the bitrate is derived from: 0.1, at the declared frame rate.
///
/// 2.8 Mbit/s for 1280x720, 4.4 for 1400x1050. A desktop is mostly flat colour and text, so this
/// is generous for the content and modest for a LAN.
pub(crate) fn bitrate_for(width: u32, height: u32) -> i32 {
    let bits = (width as u64) * (height as u64) * (ENCODER_FRAME_RATE as u64) / 10;
    bits.clamp(1_000_000, 40_000_000) as i32
}

/// How big a compressed frame the drain thread is ready for before it has to grow.
const OUTPUT_BUFFER_INITIAL: usize = 512 * 1024;

/// Above this, a compressed frame gets a complaint as well as a buffer. It is not a refusal: the
/// codec owns the frame until it is drained, so declining to take it would wedge the encoder, and
/// a wedged encoder is a worse failure than a large allocation.
const OUTPUT_BUFFER_LOUD: usize = 16 * 1024 * 1024;

/// Longest a drain iteration parks in the codec waiting for output. Also the shutdown latency, and
/// the resolution of the audience's `tick`: it is called once per iteration.
const OUTPUT_POLL_TIMEOUT_US: i64 = 100_000;

/// The codec's output-buffer flags, as `poll_output` hands them over verbatim from
/// `AMediaCodecBufferInfo::flags` (crosvm_android_display_client.cpp: `*outFlags = info.flags`).
///
/// `CODEC_CONFIG` is `AMEDIACODEC_BUFFER_FLAG_CODEC_CONFIG` from the NDK's NdkMediaCodec.h, which
/// is where the C++ side reads it. `SYNC_FRAME` is `MediaCodec.BUFFER_FLAG_KEY_FRAME`, spelled out
/// here rather than taken from that header because the NDK only named it at API 34 and this tree
/// builds against 33 -- the value is the same one the Java constant has always had.
pub(crate) const BUFFER_FLAG_SYNC_FRAME: u32 = 1;
pub(crate) const BUFFER_FLAG_CODEC_CONFIG: u32 = 2;

/// Which compression the encoder produces.
#[allow(dead_code)] // WebRTC backend
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum VideoCodec {
    H264,
    H265,
}

impl VideoCodec {
    fn mime(self) -> &'static CStr {
        match self {
            VideoCodec::H264 => c"video/avc",
            VideoCodec::H265 => c"video/hevc",
        }
    }
}

/// What an encoder is built with beyond its geometry.
///
/// `EncoderConfig::default()` is the VNC rung's encoder exactly: AVC, built through the original
/// `android_h264_enc_create`, every other key left to the component.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct EncoderConfig {
    pub codec: VideoCodec,
    /// A MediaCodecInfo.CodecProfileLevel value, or 0 for the component's default.
    pub profile: i32,
    /// -1 for the component's default, 0 for a stream without B-frames.
    pub max_bframes: i32,
    /// Seconds between the encoder's own IDRs (counted in frames at the declared rate, so with a
    /// faster producer they come sooner). A sink that answers PLIs can make this long.
    pub iframe_interval_secs: i32,
    /// Bits per second to start at; 0 derives it from the size (`bitrate_for`).
    pub bitrate_bps: i32,
    /// Ask the codec to hold no frames back (realtime priority, latency 1, vendor low-latency).
    pub low_latency: bool,
    /// Frames per second declared to the codec: the screen's own rate (virtio-gpu refresh-rate,
    /// simplefb poll-hz). 0 declares `ENCODER_FRAME_RATE`.
    pub frame_rate: i32,
    /// Ask for the codec's top operating point (`operating-rate`) rather than clocks planned for
    /// the declared rate.
    pub max_operating_rate: bool,
}

impl EncoderConfig {
    /// What an encoder for this size starts at: the configured rate, else the derived one.
    pub fn start_bitrate(&self, width: u32, height: u32) -> i32 {
        if self.bitrate_bps > 0 {
            self.bitrate_bps
        } else {
            bitrate_for(width, height)
        }
    }

    fn declared_frame_rate(&self) -> i32 {
        if self.frame_rate > 0 {
            self.frame_rate
        } else {
            ENCODER_FRAME_RATE
        }
    }
}

impl Default for EncoderConfig {
    fn default() -> Self {
        EncoderConfig {
            codec: VideoCodec::H264,
            profile: 0,
            max_bframes: -1,
            iframe_interval_secs: ENCODER_IFRAME_INTERVAL_SECS,
            bitrate_bps: 0,
            low_latency: false,
            frame_rate: 0,
            max_operating_rate: false,
        }
    }
}

// -------------------------------------------------------------------------------------------
// The encoder, on the other side of the FFI.
// -------------------------------------------------------------------------------------------

#[cfg(any(feature = "android_display", feature = "android_display_stub"))]
mod ffi {
    use super::*;

    extern "C" {
        pub fn android_h264_enc_create(
            width: u32,
            height: u32,
            bitrate_bps: i32,
            frame_rate: i32,
            iframe_interval_secs: i32,
        ) -> *mut c_void;
        #[allow(clippy::too_many_arguments)]
        pub fn android_video_enc_create(
            mime: *const c_char,
            width: u32,
            height: u32,
            bitrate_bps: i32,
            frame_rate: i32,
            iframe_interval_secs: i32,
            profile: i32,
            repeat_previous_frame_after_us: i64,
            max_bframes: i32,
            low_latency: bool,
            max_operating_rate: bool,
        ) -> *mut c_void;
        pub fn android_video_enc_set_bitrate(ctx: *mut c_void, bitrate_bps: i32) -> bool;
        pub fn android_video_enc_get_name(ctx: *mut c_void, buf: *mut c_char, size: usize) -> usize;
        pub fn android_h264_enc_destroy(ctx: *mut c_void);
        pub fn android_h264_enc_request_sync_frame(ctx: *mut c_void);
        #[allow(clippy::too_many_arguments)]
        pub fn android_h264_enc_encode_frame(
            ctx: *mut c_void,
            blit_ctx: *mut c_void,
            import_id: i64,
            pixels: *const u8,
            pixels_size: u32,
            width: u32,
            height: u32,
            cursor_bgra: *const u8,
            cursor_w: i32,
            cursor_h: i32,
            cursor_x: i32,
            cursor_y: i32,
            cursor_visible: bool,
            pts_us: i64,
            out_error: *mut c_char,
            error_cap: u32,
        ) -> bool;
        pub fn android_h264_enc_poll_output(
            ctx: *mut c_void,
            out: *mut u8,
            cap: u32,
            out_size: *mut u32,
            out_flags: *mut u32,
            out_pts_us: *mut i64,
            timeout_us: i64,
        ) -> i32;
        pub fn android_h264_enc_codec_config(ctx: *mut c_void, out: *mut u8, cap: u32) -> u32;
        pub fn android_h264_enc_frame_counts(
            ctx: *mut c_void,
            out_queued: *mut u64,
            out_dropped: *mut u64,
        );
    }
}

/// A build with no Android display backend links no encoder, for the same reason it links no blit:
/// both live in `libcrosvm_android_display_client`. "There is no encoder" is a run-time answer
/// this module already has to handle, so this configuration takes that path rather than a second
/// one.
#[cfg(not(any(feature = "android_display", feature = "android_display_stub")))]
#[allow(clippy::too_many_arguments)]
mod ffi {
    use super::*;

    pub unsafe fn android_h264_enc_create(
        _width: u32,
        _height: u32,
        _bitrate_bps: i32,
        _frame_rate: i32,
        _iframe_interval_secs: i32,
    ) -> *mut c_void {
        std::ptr::null_mut()
    }
    pub unsafe fn android_video_enc_create(
        _mime: *const c_char,
        _width: u32,
        _height: u32,
        _bitrate_bps: i32,
        _frame_rate: i32,
        _iframe_interval_secs: i32,
        _profile: i32,
        _repeat_previous_frame_after_us: i64,
        _max_bframes: i32,
        _low_latency: bool,
        _max_operating_rate: bool,
    ) -> *mut c_void {
        std::ptr::null_mut()
    }
    pub unsafe fn android_video_enc_set_bitrate(_ctx: *mut c_void, _bitrate_bps: i32) -> bool {
        false
    }
    pub unsafe fn android_video_enc_get_name(
        _ctx: *mut c_void,
        _buf: *mut c_char,
        _size: usize,
    ) -> usize {
        0
    }
    pub unsafe fn android_h264_enc_destroy(_ctx: *mut c_void) {}
    pub unsafe fn android_h264_enc_request_sync_frame(_ctx: *mut c_void) {}
    pub unsafe fn android_h264_enc_encode_frame(
        _ctx: *mut c_void,
        _blit_ctx: *mut c_void,
        _import_id: i64,
        _pixels: *const u8,
        _pixels_size: u32,
        _width: u32,
        _height: u32,
        _cursor_bgra: *const u8,
        _cursor_w: i32,
        _cursor_h: i32,
        _cursor_x: i32,
        _cursor_y: i32,
        _cursor_visible: bool,
        _pts_us: i64,
        _out_error: *mut c_char,
        _error_cap: u32,
    ) -> bool {
        false
    }
    pub unsafe fn android_h264_enc_poll_output(
        _ctx: *mut c_void,
        _out: *mut u8,
        _cap: u32,
        _out_size: *mut u32,
        _out_flags: *mut u32,
        _out_pts_us: *mut i64,
        _timeout_us: i64,
    ) -> i32 {
        -2
    }
    pub unsafe fn android_h264_enc_codec_config(
        _ctx: *mut c_void,
        _out: *mut u8,
        _cap: u32,
    ) -> u32 {
        0
    }
    pub unsafe fn android_h264_enc_frame_counts(
        _ctx: *mut c_void,
        _out_queued: *mut u64,
        _out_dropped: *mut u64,
    ) {
    }
}

/// One configured encoder.
///
/// Held behind an `Arc` so that a geometry change can swap in a replacement while the drain thread
/// is still inside the old one's poll: the swap only drops the producer's reference, and the codec
/// is destroyed by whichever thread lets go of it last.
pub(crate) struct Encoder {
    ptr: *mut c_void,
    pub width: u32,
    pub height: u32,
    /// What it was built with, so a pipeline whose config has moved on knows to rebuild it.
    pub config: EncoderConfig,
}

// SAFETY: every entry point behind these calls is one of AMediaCodec's, which are documented to be
// callable from several threads at once, and this module only ever does two things concurrently:
// feed from the producer thread and drain from its own. The input Surface -- the part that is NOT
// thread-safe -- is touched from the producer thread alone.
unsafe impl Send for Encoder {}
unsafe impl Sync for Encoder {}

impl Encoder {
    fn open(config: &EncoderConfig, width: u32, height: u32) -> Option<Encoder> {
        let bitrate = config.start_bitrate(width, height);
        let ptr = if *config == EncoderConfig::default() {
            // The original constructor, so the VNC rung's encoder is built by the very same call
            // it always was.
            // SAFETY: no arguments are borrowed; the returned pointer is ours from here.
            unsafe {
                ffi::android_h264_enc_create(
                    width,
                    height,
                    bitrate,
                    ENCODER_FRAME_RATE,
                    ENCODER_IFRAME_INTERVAL_SECS,
                )
            }
        } else {
            // SAFETY: `mime` is a static NUL-terminated string read during the call only.
            unsafe {
                ffi::android_video_enc_create(
                    config.codec.mime().as_ptr(),
                    width,
                    height,
                    bitrate,
                    config.declared_frame_rate(),
                    config.iframe_interval_secs,
                    config.profile,
                    /* repeat_previous_frame_after_us= */ 0,
                    config.max_bframes,
                    config.low_latency,
                    config.max_operating_rate,
                )
            }
        };
        if ptr.is_null() {
            return None;
        }
        Some(Encoder {
            ptr,
            width,
            height,
            config: *config,
        })
    }

    /// The codec component's name, or "?" if the platform could not say.
    pub fn name(&self) -> String {
        let mut buf = [0u8; 128];
        // SAFETY: `ptr` is live; `buf` is writable for the size passed and NUL-terminated by it.
        let n = unsafe {
            ffi::android_video_enc_get_name(self.ptr, buf.as_mut_ptr() as *mut c_char, buf.len())
        };
        let end = buf.iter().position(|b| *b == 0).unwrap_or(buf.len());
        if n == 0 || end == 0 {
            return "?".to_string();
        }
        String::from_utf8_lossy(&buf[..end]).into_owned()
    }

    pub fn request_sync_frame(&self) {
        // SAFETY: `ptr` came from an encoder constructor and is live for `self`.
        unsafe { ffi::android_h264_enc_request_sync_frame(self.ptr) }
    }

    #[allow(dead_code)] // WebRTC backend
    /// Retargets the running encoder. `false` if the component refused the request.
    pub fn set_bitrate(&self, bitrate_bps: i32) -> bool {
        // SAFETY: as above.
        unsafe { ffi::android_video_enc_set_bitrate(self.ptr, bitrate_bps) }
    }

    /// Feeds one picture. `Ok(())` means it reached the encoder, or was deliberately dropped
    /// because the codec had no free input buffer; `Err` carries the native message verbatim.
    #[allow(clippy::too_many_arguments)]
    fn encode(
        &self,
        blit_ctx: *mut c_void,
        import_id: i64,
        pixels: *const u8,
        pixels_size: u32,
        width: u32,
        height: u32,
        cursor: &CursorOverlay,
        pts_us: i64,
    ) -> Result<(), String> {
        let mut message = [0u8; 256];
        // SAFETY: `pixels` and the cursor image are borrowed by the callee for the duration of the
        // call only, and both outlive it -- they belong to the frame the caller is holding.
        // `message` is a live local of `error_cap` bytes.
        let ok = unsafe {
            ffi::android_h264_enc_encode_frame(
                self.ptr,
                blit_ctx,
                import_id,
                pixels,
                pixels_size,
                width,
                height,
                cursor.pixels,
                cursor.width,
                cursor.height,
                cursor.x,
                cursor.y,
                cursor.visible,
                pts_us,
                message.as_mut_ptr() as *mut c_char,
                message.len() as u32,
            )
        };
        if ok {
            return Ok(());
        }
        let end = message.iter().position(|b| *b == 0).unwrap_or(message.len());
        Err(String::from_utf8_lossy(&message[..end]).into_owned())
    }

    /// Drains one compressed frame into `buf`, blocking for at most `timeout_us`.
    fn poll_output(&self, buf: &mut Vec<u8>, timeout_us: i64) -> PollOutput {
        let mut size = 0u32;
        let mut flags = 0u32;
        let mut pts = 0i64;
        // SAFETY: `buf` has `capacity` writable bytes at its pointer and nothing else aliases it;
        // the callee writes at most that many and reports how many.
        let ret = unsafe {
            ffi::android_h264_enc_poll_output(
                self.ptr,
                buf.as_mut_ptr(),
                buf.capacity() as u32,
                &mut size,
                &mut flags,
                &mut pts,
                timeout_us,
            )
        };
        match ret {
            n if n > 0 => {
                // SAFETY: the callee wrote exactly `n` initialised bytes, and `n <= capacity` or it
                // would have answered `TooSmall` instead.
                unsafe { buf.set_len(n as usize) };
                PollOutput::Frame { flags, pts_us: pts }
            }
            0 => PollOutput::Idle,
            -1 => PollOutput::TooSmall(size as usize),
            _ => PollOutput::Failed,
        }
    }

    #[allow(dead_code)] // WebRTC backend
    /// The cached parameter sets (SPS+PPS, or VPS+SPS+PPS), or `None` if the encoder has not
    /// produced them yet.
    pub fn codec_config(&self) -> Option<Vec<u8>> {
        let mut buf = vec![0u8; 4096];
        // SAFETY: `buf` is 4096 writable bytes.
        let size = unsafe {
            ffi::android_h264_enc_codec_config(self.ptr, buf.as_mut_ptr(), buf.len() as u32)
        } as usize;
        if size == 0 || size > buf.len() {
            return None;
        }
        buf.truncate(size);
        Some(buf)
    }

    fn frame_counts(&self) -> (u64, u64) {
        let (mut queued, mut dropped) = (0u64, 0u64);
        // SAFETY: both out params are live locals.
        unsafe { ffi::android_h264_enc_frame_counts(self.ptr, &mut queued, &mut dropped) };
        (queued, dropped)
    }
}

impl Drop for Encoder {
    fn drop(&mut self) {
        // SAFETY: the pointer came from an encoder constructor and is destroyed once.
        unsafe { ffi::android_h264_enc_destroy(self.ptr) }
    }
}

enum PollOutput {
    /// A compressed frame is in the buffer.
    Frame { flags: u32, pts_us: i64 },
    /// Nothing was ready before the timeout.
    Idle,
    /// The frame is bigger than the buffer; it is still queued, and this is how big it is.
    TooSmall(usize),
    Failed,
}

/// The cursor, flattened so the encode call does not take six more arguments.
pub(crate) struct CursorOverlay {
    pub pixels: *const u8,
    pub width: i32,
    pub height: i32,
    pub x: i32,
    pub y: i32,
    pub visible: bool,
}

impl CursorOverlay {
    #[allow(dead_code)] // WebRTC backend
    /// No cursor composited into the picture.
    pub fn none() -> CursorOverlay {
        CursorOverlay {
            pixels: std::ptr::null(),
            width: 0,
            height: 0,
            x: 0,
            y: 0,
            visible: false,
        }
    }
}

/// One picture offered to the pipeline, borrowed for the length of `EncoderPipeline::feed`.
///
/// `gpu_blit_ctx` + `gpu_import_id` are the same picture while it is still a GPU object, for the
/// encoder to blit straight into its input buffer; `pixels` is the CPU copy the encoder uploads
/// when there is no GPU source or the GPU route has failed. Either may be absent, not both.
pub(crate) struct FeedFrame<'a> {
    pub pixels: *const u8,
    pub size: u32,
    pub width: u32,
    pub height: u32,
    pub gpu_blit_ctx: *mut c_void,
    pub gpu_import_id: i64,
    pub cursor: &'a CursorOverlay,
}

// -------------------------------------------------------------------------------------------
// The audience.
// -------------------------------------------------------------------------------------------

/// What the drain thread knows about the encoder, for an audience that tells its viewers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum EncoderState {
    /// Not asked for yet, or between builds.
    Warming,
    /// An encoder exists.
    Available,
    /// No encoder could be created on this device, and none will be.
    Unavailable,
}

/// Who the compressed stream is for.
///
/// Every method is called from one of two threads -- the producer's (`wants_frames`,
/// `encoder_changed`, `encoder_failed`) or the drain thread (the rest) -- and none of them may
/// block: the first is the guest's own flush path, the second is the only thing emptying the codec.
pub(crate) trait EncodedAudience: Send + Sync + 'static {
    /// Whether anybody is waiting for frames now. While this is false the pipeline feeds nothing,
    /// and so never even brings the encoder up.
    fn wants_frames(&self) -> bool;
    /// A counter that moves whenever a viewer joins (or has to restart). Each new value earns one
    /// sync frame.
    fn join_generation(&self) -> u64;
    /// Once per drain iteration, about ten times a second, whether or not there is output.
    fn tick(&self, _state: EncoderState) {}
    /// A new encoder now produces frames of this size; whatever was joined to the old one has to
    /// start again on the next sync frame.
    fn encoder_changed(&self, _width: u32, _height: u32) {}
    /// No encoder could be created, permanently.
    fn encoder_failed(&self) {}
    /// A new encoder is up: the codec component's name (e.g. "c2.qti.avc.encoder") and what it was
    /// built with, for an audience that reports them.
    fn encoder_built(&self, _name: &str, _config: &EncoderConfig, _bitrate_bps: i32) {}
    /// One compressed unit. `width`/`height`/`codec` are the encoder's, so a unit still draining
    /// out of a replaced encoder can be told apart from one the viewers were prepared for.
    fn on_unit(
        &self,
        data: &[u8],
        flags: u32,
        pts_us: i64,
        width: u32,
        height: u32,
        codec: VideoCodec,
    );
}

// -------------------------------------------------------------------------------------------
// The pipeline.
// -------------------------------------------------------------------------------------------

/// Producer-side feeding, the drain thread, and the encoder they share.
pub(crate) struct EncoderPipeline<A: EncodedAudience> {
    /// Prefix for every log line, e.g. "VNC h264".
    tag: &'static str,
    /// What the next encoder is built with. Replaceable (`set_config`): a sink can change codec
    /// while running, which takes effect as a rebuild at the next offered frame.
    config: Mutex<EncoderConfig>,
    audience: A,
    /// The audience's join generation as of the last sync frame asked for on its behalf.
    joins_answered: AtomicU64,
    /// The live encoder. `None` until the first frame is offered with something waiting for it,
    /// and replaced wholesale when the screen changes size.
    ///
    /// The lock is held only long enough to clone or replace the `Arc`, never across a codec call
    /// -- the drain thread parks in the codec for up to a tenth of a second, and the producer
    /// thread must not queue behind that.
    encoder: Mutex<Option<Arc<Encoder>>>,
    /// Set once a Vulkan blit into a codec input buffer has failed. From then on every frame takes
    /// the CPU upload route, because the answer will not change: it is a statement about what
    /// gralloc and the codec agreed to allocate, not about this frame.
    gpu_route_failed: AtomicBool,
    /// Set once bringing an encoder up has failed: "this device has no encoder we can use" is a
    /// fact about the device, so asking again on the next offer would be thirty failed codec
    /// creations a second, each one a binder round trip to mediaserver.
    encoder_failed: AtomicBool,
    /// Whether the last offer found somebody waiting, so the transition is logged once.
    was_feeding: AtomicBool,
    paused_offers: AtomicU64,
    encode_errors: AtomicU64,
    /// Time base for presentation timestamps. Wall time, not frame counting: the producer's rate
    /// is whatever the guest is doing.
    epoch: Instant,
    stop: AtomicBool,
}

impl<A: EncodedAudience> EncoderPipeline<A> {
    /// Builds the pipeline and starts its drain thread. Does NOT bring the encoder up -- that waits
    /// until something is waiting for frames, so a screen nobody watches never loads the media
    /// stack, and the encoder is built against the geometry of a real frame.
    pub fn start(
        tag: &'static str,
        config: EncoderConfig,
        audience: A,
    ) -> std::io::Result<Arc<EncoderPipeline<A>>> {
        let pipeline = Arc::new(EncoderPipeline {
            tag,
            config: Mutex::new(config),
            audience,
            joins_answered: AtomicU64::new(0),
            encoder: Mutex::new(None),
            gpu_route_failed: AtomicBool::new(false),
            encoder_failed: AtomicBool::new(false),
            was_feeding: AtomicBool::new(false),
            paused_offers: AtomicU64::new(0),
            encode_errors: AtomicU64::new(0),
            epoch: Instant::now(),
            stop: AtomicBool::new(false),
        });
        let drain_side = pipeline.clone();
        thread::Builder::new()
            .name("video_enc_drain".into())
            .spawn(move || drain_side.drain_loop())?;
        Ok(pipeline)
    }

    pub fn audience(&self) -> &A {
        &self.audience
    }

    /// Whether the stream needs frames to keep arriving. Part of the sink's `has_consumer`: the
    /// producer does not build a frame when nothing is reported waiting, and the encoder is only
    /// built out of an offered frame, so leaving this out would be a deadlock.
    pub fn wants_frames(&self) -> bool {
        self.audience.wants_frames()
    }

    /// See `DisplayT::consumer_generation`.
    pub fn connect_generation(&self) -> u64 {
        self.audience.join_generation()
    }

    /// Whether feeding by Vulkan blit has failed on this device, after which only pixels are
    /// accepted. A sink with no pixels of its own to offer has to send the producer elsewhere.
    #[allow(dead_code)] // WebRTC backend
    pub fn gpu_route_failed(&self) -> bool {
        self.gpu_route_failed.load(Ordering::Relaxed)
    }

    #[allow(dead_code)] // WebRTC backend
    /// The time base of every `pts_us` this pipeline hands its audience.
    pub fn epoch(&self) -> Instant {
        self.epoch
    }

    #[allow(dead_code)] // WebRTC backend
    /// Replaces what the next encoder is built with. The live one is kept until the next offered
    /// frame finds it out of date and rebuilds; the caller offers one if the screen is still.
    pub fn set_config(&self, config: EncoderConfig) {
        if let Ok(mut current) = self.config.lock() {
            *current = config;
        }
    }

    #[allow(dead_code)] // WebRTC backend
    /// Answers the audience's current join generation now, from the producer's side, for a
    /// producer about to offer the picture the sync frame should be made of. Whichever of this and
    /// the drain thread sees a generation first asks the codec; the other finds it answered. Two
    /// requests would make two IDRs: the second lands on the guest's next picture.
    pub fn answer_joins(&self) {
        if let Some(encoder) = self.current_encoder() {
            self.sync_frame_for_joins(&encoder);
        }
    }

    #[allow(dead_code)] // WebRTC backend
    /// Retargets the live encoder's bitrate. `false` if there is no encoder or it refused.
    pub fn set_bitrate(&self, bitrate_bps: i32) -> bool {
        self.current_encoder()
            .map(|e| e.set_bitrate(bitrate_bps))
            .unwrap_or(false)
    }

    #[allow(dead_code)] // WebRTC backend
    /// The live encoder's parameter sets, for a viewer that joins after they were emitted.
    pub fn codec_config(&self) -> Option<Vec<u8>> {
        self.current_encoder().and_then(|e| e.codec_config())
    }

    /// Moves compressed frames from the codec to the audience, and nowhere else.
    ///
    /// It keeps draining with nobody watching, which is deliberate: feeding stops when the last
    /// viewer leaves, so what is left inside the codec is a handful of frames that have to come out
    /// before it can be reused. Draining them into nothing is how it gets back to idle.
    ///
    /// `tick` is called BEFORE the encoder is looked for, so viewers are kept alive across a codec
    /// that is being rebuilt as much as across a screen that is not moving.
    fn drain_loop(self: Arc<Self>) {
        let mut buf: Vec<u8> = Vec::with_capacity(OUTPUT_BUFFER_INITIAL);
        let mut announced = 0u32;
        while !self.stop.load(Ordering::Relaxed) {
            let held = self.current_encoder();
            self.audience.tick(if self.encoder_failed.load(Ordering::Relaxed) {
                EncoderState::Unavailable
            } else if held.is_some() {
                EncoderState::Available
            } else {
                EncoderState::Warming
            });
            let Some(encoder) = held else {
                thread::sleep(Duration::from_millis(100));
                continue;
            };
            self.sync_frame_for_joins(&encoder);
            buf.clear();
            match encoder.poll_output(&mut buf, OUTPUT_POLL_TIMEOUT_US) {
                PollOutput::Frame { flags, pts_us } => {
                    // One line for the first few units of a stream, because "the encoder is
                    // producing parameter sets and IDRs" is the single fact that separates "it is
                    // running" from "a viewer could actually start decoding".
                    if announced < 3 {
                        announced += 1;
                        base::info!(
                            "{}: unit {} is {} bytes, flags {:#x}",
                            self.tag,
                            announced,
                            buf.len(),
                            flags
                        );
                    }
                    self.audience
                        .on_unit(
                            &buf,
                            flags,
                            pts_us,
                            encoder.width,
                            encoder.height,
                            encoder.config.codec,
                        );
                }
                PollOutput::Idle => {}
                PollOutput::TooSmall(needed) => {
                    if needed >= OUTPUT_BUFFER_LOUD {
                        base::error!(
                            "{}: a compressed frame is {} bytes; taking it anyway",
                            self.tag,
                            needed
                        );
                    } else {
                        base::info!("{}: growing the output buffer to {} bytes", self.tag, needed);
                    }
                    // `buf` was cleared at the top of the loop, so its length is zero and this
                    // asks for `needed` bytes of capacity rather than `needed` more than it has.
                    buf.reserve(needed);
                }
                PollOutput::Failed => {
                    // The codec is kept: a drain failure is not the same statement as "this device
                    // has no encoder", and the next poll may well succeed.
                    base::error!("{}: the encoder failed while draining", self.tag);
                    thread::sleep(Duration::from_millis(500));
                }
            }
        }
    }

    fn current_encoder(&self) -> Option<Arc<Encoder>> {
        self.encoder.lock().ok().and_then(|g| g.clone())
    }

    /// Answers a viewer that has joined -- or been thrown out of -- the stream with a sync frame,
    /// once per generation.
    ///
    /// On the drain thread rather than where the producer polls the generation, because the
    /// producer must never make a codec call -- and because this thread already wakes ten times a
    /// second holding the encoder.
    fn sync_frame_for_joins(&self, encoder: &Encoder) {
        let joins = self.audience.join_generation();
        if self.joins_answered.swap(joins, Ordering::Relaxed) == joins {
            return;
        }
        encoder.request_sync_frame();
        base::info!(
            "{}: a viewer is waiting to start the stream (join {}); sync frame requested",
            self.tag,
            joins
        );
    }

    /// The encoder for a frame of this size, building or rebuilding it if that is what it takes.
    ///
    /// Runs on the producer's thread; the audience hooks it calls must not block.
    fn encoder_for(&self, width: u32, height: u32) -> Option<Arc<Encoder>> {
        {
            let guard = self.encoder.lock().ok()?;
            if let Some(encoder) = guard.as_ref() {
                let config = *self.config.lock().ok()?;
                if encoder.width == width && encoder.height == height && encoder.config == config {
                    return Some(encoder.clone());
                }
            }
        }
        if self.encoder_failed.load(Ordering::Relaxed) {
            return None;
        }
        // Built outside the lock: bringing a codec up talks to mediaserver over binder and takes
        // long enough that the drain thread should not be waiting behind it.
        let config = *self.config.lock().ok()?;
        let built = Encoder::open(&config, width, height).map(Arc::new);
        if built.is_none() {
            // The reason is already in the log, from the side of the FFI that knows it.
            self.encoder_failed.store(true, Ordering::Relaxed);
            base::error!(
                "{}: no encoder for a {}x{} screen; this host serves pixels only",
                self.tag,
                width,
                height
            );
            self.audience.encoder_failed();
            return None;
        }
        {
            let mut guard = self.encoder.lock().ok()?;
            *guard = built.clone();
        }
        if let Some(encoder) = built.as_ref() {
            self.audience
                .encoder_built(&encoder.name(), &config, config.start_bitrate(width, height));
        }
        self.audience.encoder_changed(width, height);
        built
    }

    /// One offered picture, on the producer's thread.
    pub fn feed(&self, frame: &FeedFrame) {
        if self.stop.load(Ordering::Relaxed) {
            return;
        }
        if !self.audience.wants_frames() {
            let count = self.paused_offers.fetch_add(1, Ordering::Relaxed);
            if self.was_feeding.swap(false, Ordering::Relaxed) {
                base::info!(
                    "{}: nothing wants the stream; encoder paused ({} offers skipped so far)",
                    self.tag,
                    count
                );
            }
            return;
        }
        if !self.was_feeding.swap(true, Ordering::Relaxed) {
            base::info!("{}: a viewer is on the stream; feeding the encoder", self.tag);
        }

        let has_gpu_source = frame.gpu_import_id != 0 && !frame.gpu_blit_ctx.is_null();
        if (frame.pixels.is_null() && !has_gpu_source) || frame.width == 0 || frame.height == 0 {
            return;
        }
        let Some(encoder) = self.encoder_for(frame.width, frame.height) else {
            return;
        };
        let pts_us = self.epoch.elapsed().as_micros() as i64;

        let use_gpu = has_gpu_source && !self.gpu_route_failed.load(Ordering::Relaxed);
        let attempt = if use_gpu {
            encoder.encode(
                frame.gpu_blit_ctx,
                frame.gpu_import_id,
                frame.pixels,
                frame.size,
                frame.width,
                frame.height,
                frame.cursor,
                pts_us,
            )
        } else {
            encoder.encode(
                std::ptr::null_mut(),
                0,
                frame.pixels,
                frame.size,
                frame.width,
                frame.height,
                frame.cursor,
                pts_us,
            )
        };

        let Err(reason) = attempt else {
            return;
        };
        if use_gpu {
            // "A MediaCodec input Surface can be fed by Vulkan" answered in the negative, for this
            // device. Loud and once: everything after this frame is on the CPU route.
            self.gpu_route_failed.store(true, Ordering::Relaxed);
            base::error!(
                "{}: feeding the codec by Vulkan blit failed ({}); every frame from here is \
                 uploaded by the CPU instead",
                self.tag,
                reason
            );
            if frame.pixels.is_null() {
                return;
            }
            if let Err(reason) = encoder.encode(
                std::ptr::null_mut(),
                0,
                frame.pixels,
                frame.size,
                frame.width,
                frame.height,
                frame.cursor,
                pts_us,
            ) {
                self.report_encode_error(&reason);
            }
            return;
        }
        self.report_encode_error(&reason);
    }

    fn report_encode_error(&self, reason: &str) {
        let count = self.encode_errors.fetch_add(1, Ordering::Relaxed);
        if count == 0 || count.is_power_of_two() {
            base::error!(
                "{}: frame not encoded ({} so far): {}",
                self.tag,
                count + 1,
                reason
            );
        }
    }

    /// Stops the drain thread and says what the run did.
    pub fn shutdown(&self) {
        self.stop.store(true, Ordering::Relaxed);
        match self.current_encoder().map(|e| e.frame_counts()) {
            Some((queued, dropped)) => base::info!(
                "{}: {} frames queued, {} dropped for want of an input buffer, {} offers \
                 skipped with nothing listening",
                self.tag,
                queued,
                dropped,
                self.paused_offers.load(Ordering::Relaxed)
            ),
            None => base::info!(
                "{}: no encoder was ever built; {} offers went past with nothing listening",
                self.tag,
                self.paused_offers.load(Ordering::Relaxed)
            ),
        }
    }
}
