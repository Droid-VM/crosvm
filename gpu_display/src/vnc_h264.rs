// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

//! The VNC sink's hardware-encode rung: a second consumer on the frame bus, and the codec behind
//! it.
//!
//! Plan §6 step 13, and then plans/H264_SINGLE_PORT.md. The frame the sink is already holding --
//! the same offer, at the same instant, that the LibVNCServer consumer is turning into RFB
//! rectangles -- is also handed to a MediaCodec H.264 encoder. Nothing about RFB changes: a legacy
//! client connected to the same server keeps being served the same way it always was, from the
//! same offer, which is the entire reason step 12 split ingest from consumers.
//!
//! **One door, on the RFB port.** The compressed stream leaves by exactly one route:
//! `vnc_h264_rfb.c`, which serves it inside ordinary FramebufferUpdate messages to clients that
//! asked for the Open H.264 encoding (50) -- TigerVNC, noVNC, and the DroidVM app, which asks for
//! 50 and for the private pseudo-encoding 0x44564831 as well.
//!
//! It used to leave by two. The DVH2 side channel -- its own TCP listener on `RFB port + 100`, its
//! own framing, one client at a time -- is gone, and this file is what is left of it: the codec,
//! the drain thread, and the two facts the side channel's framing used to carry, which have moved
//! onto the RFB wire as 0x44564831 rects. What the deletion buys is one port per screen, which is
//! one thing to open, one thing to forward, and one thing that can be wrong.
//!
//! Two of its mechanisms survive by name in the C file, because they were never about sockets:
//!
//! * The **heartbeat**, and its three-second unit. A still screen and a dead stream are
//!   indistinguishable to a receiver that is only ever written to. DVH2 answered that with a
//!   zero-length frame every three seconds; the broker answers it with a heartbeat rect on the
//!   same cadence, to clients that speak the pseudo-encoding.
//! * The **refusal**, which was a token on the wire and is now a caps value: 1 where DVH2 said
//!   `no-encoder`, which is the one answer that licenses a client to stop asking.
//!
//! **Double encoding is real and bounded.** While a LEGACY RFB client and an h264 one are both
//! connected the frame is encoded twice, once by LibVNCServer and once by the codec. That is the
//! honest cost of serving two client kinds at once and it is what the bus is for; it is not paid
//! when only one kind is connected, because this consumer feeds nothing with nothing waiting for
//! it and LibVNCServer marks nothing with no RFB client. An RFB client being served encoding 50
//! does not pay it either: its pixel path is suppressed for as long as it is on the stream (see
//! vnc_h264_rfb.c), so it costs the codec nothing extra and LibVNCServer nothing at all.
//!
//! **SPS/PPS: cached, not attached to every IDR.** The encoder emits its codec-specific data
//! exactly once, in its first output buffer, which serves whoever was already connected and nobody
//! after that. So the parameter sets are cached by the broker when they appear and put in front of
//! the IDR each later client joins on. The alternative -- prepend them to every IDR -- costs bytes
//! every couple of seconds forever to solve a problem that only exists at join time.

use std::ffi::c_char;
use std::ffi::c_int;
use std::ffi::c_void;
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
/// The interval only has to cover "a decoder that lost its place"; a client that connects through
/// this module is given an explicit sync frame the moment it does, so this is a backstop rather
/// than the join mechanism.
const ENCODER_IFRAME_INTERVAL_SECS: i32 = 2;

/// Bits per pixel per frame the bitrate is derived from: 0.1, at the declared frame rate.
///
/// 2.8 Mbit/s for 1280x720, 4.4 for 1400x1050. A desktop is mostly flat colour and text, so this
/// is generous for the content and modest for a LAN -- which is the trade this rung exists to
/// make, the classic path already covering the case where bandwidth is free.
fn bitrate_for(width: u32, height: u32) -> i32 {
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
/// the resolution of the broker's heartbeat clock: `vnc_h264_rfb_tick` is called once per
/// iteration, so a beat lands within a tenth of a second of when it is due.
const OUTPUT_POLL_TIMEOUT_US: i64 = 100_000;

/// What this host can currently do about H.264, as the caps rect's `value` byte says it.
/// plans/H264_SINGLE_PORT.md §1; the numbers are the wire and are mirrored in vnc_h264_rfb.h.
const CAPS_AVAILABLE: c_int = 0;
const CAPS_UNAVAILABLE: c_int = 1;
const CAPS_WARMING: c_int = 2;

/// The caps value for a host in this state, derived rather than remembered.
///
/// `encoder_failed` is the permanent answer and outranks everything: it is a fact about the device
/// -- no codec could be created -- and the one value that licenses a client to stop waiting. An
/// encoder that exists is `available`; one that has not been asked for yet, or is between builds,
/// is `warming`, which is what a broker starts at.
///
/// A free function so that the mapping can be asserted without a device, a server or a codec.
fn caps_for(encoder_failed: bool, have_encoder: bool) -> c_int {
    if encoder_failed {
        CAPS_UNAVAILABLE
    } else if have_encoder {
        CAPS_AVAILABLE
    } else {
        CAPS_WARMING
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

/// A build with the VNC sink but no Android display backend links no encoder, for the same reason
/// it links no blit: both live in `libcrosvm_android_display_client`. "There is no encoder" is a
/// run-time answer this module already has to handle, so this configuration takes that path rather
/// than a second one.
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
struct Encoder {
    ptr: *mut c_void,
    width: u32,
    height: u32,
}

// SAFETY: every entry point behind these calls is one of AMediaCodec's, which are documented to be
// callable from several threads at once, and this module only ever does two things concurrently:
// feed from the producer thread and drain from its own. The input Surface -- the part that is NOT
// thread-safe -- is touched from the producer thread alone.
unsafe impl Send for Encoder {}
unsafe impl Sync for Encoder {}

impl Encoder {
    fn open(width: u32, height: u32) -> Option<Encoder> {
        let bitrate = bitrate_for(width, height);
        // SAFETY: no arguments are borrowed; the returned pointer is ours from here.
        let ptr = unsafe {
            ffi::android_h264_enc_create(
                width,
                height,
                bitrate,
                ENCODER_FRAME_RATE,
                ENCODER_IFRAME_INTERVAL_SECS,
            )
        };
        if ptr.is_null() {
            return None;
        }
        Some(Encoder { ptr, width, height })
    }

    fn request_sync_frame(&self) {
        // SAFETY: `ptr` came from `android_h264_enc_create` and is live for `self`.
        unsafe { ffi::android_h264_enc_request_sync_frame(self.ptr) }
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
        // call only, and both outlive it -- they belong to the offer, which outlives the consumer
        // callback this is reached from. `message` is a live local of `error_cap` bytes.
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
                PollOutput::Frame { flags }
            }
            0 => PollOutput::Idle,
            -1 => PollOutput::TooSmall(size as usize),
            _ => PollOutput::Failed,
        }
    }

    /// The cached SPS+PPS, or `None` if the encoder has not produced them yet.
    fn codec_config(&self) -> Option<Vec<u8>> {
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
        // SAFETY: the pointer came from `android_h264_enc_create` and is destroyed once.
        unsafe { ffi::android_h264_enc_destroy(self.ptr) }
    }
}

enum PollOutput {
    /// A compressed frame is in the buffer.
    Frame { flags: u32 },
    /// Nothing was ready before the timeout.
    Idle,
    /// The frame is bigger than the buffer; it is still queued, and this is how big it is.
    TooSmall(usize),
    Failed,
}

/// The cursor, flattened out of the offer so the encode call does not take six more arguments.
struct CursorOverlay {
    pixels: *const u8,
    width: i32,
    height: i32,
    x: i32,
    y: i32,
    visible: bool,
}

// -------------------------------------------------------------------------------------------
// The consumer.
// -------------------------------------------------------------------------------------------

/// Everything the frame callback and the drain thread share.
///
/// One object rather than two because the two are one mechanism: a client arriving is what makes
/// the consumer feed the codec, and what the codec produces is what the client is sent.
pub(crate) struct H264Consumer {
    /// The RFB-50 broadcaster, or `None` on a build or a device where it could not be created.
    ///
    /// It is the only audience there is. `None` therefore means this consumer will never serve
    /// anybody -- it stays registered on the bus, answers "nothing wants frames" to every offer,
    /// and costs a pointer.
    rfb: Option<RfbBroker>,
    /// The broadcaster's join generation as of the last sync frame asked for on its behalf. A join
    /// is served nothing until an IDR arrives, so somebody has to ask for one, and the drain thread
    /// is the thread that already wakes ten times a second with the encoder in its hand.
    rfb_joins_answered: AtomicU64,
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
    /// Set once bringing an encoder up has failed. Same shape and the same reason as the sink's
    /// `blit_probed`: "this device has no H.264 encoder we can use" is a fact about the device, so
    /// asking again on the next offer would be thirty failed codec creations a second, each one
    /// talking to mediaserver over binder and logging what it found.
    encoder_failed: AtomicBool,
    /// Whether the last offer found somebody waiting, so the transition can be logged once instead
    /// of the state being logged forever.
    was_feeding: AtomicBool,
    paused_offers: AtomicU64,
    encode_errors: AtomicU64,
    /// Time base for presentation timestamps. Wall time, not frame counting: the producer's rate
    /// is whatever the guest is doing, so numbering frames would tell the decoder the wrong story
    /// about how long each one was on screen.
    epoch: Instant,
    stop: AtomicBool,
}

/// Mirror of `struct vnc_frame_offer` (vnc_frame_consumer.h). Field for field, in order: the two
/// definitions are one ABI and the C one is the original.
#[repr(C)]
struct VncFrameOffer {
    pixels: *const u8,
    size: u32,
    width: c_int,
    height: c_int,
    full: c_int,
    bands: *const c_void,
    band_count: c_int,
    frame_replaced: c_int,
    gpu_blit_ctx: *mut c_void,
    gpu_import_id: i64,
    cursor_argb: *const u8,
    cursor_w: c_int,
    cursor_h: c_int,
    cursor_x: c_int,
    cursor_y: c_int,
    cursor_visible: c_int,
}

/// Mirror of `struct vnc_frame_consumer` (vnc_frame_consumer.h).
#[repr(C)]
struct VncFrameConsumer {
    name: *const c_char,
    ctx: *mut c_void,
    on_frame: Option<extern "C" fn(*mut c_void, *mut c_void, *const VncFrameOffer)>,
}

extern "C" {
    fn vnc_server_attach_consumer(server: *mut c_void, consumer: *const VncFrameConsumer) -> c_int;

    fn vnc_h264_rfb_create(server: *mut c_void) -> *mut c_void;
    fn vnc_h264_rfb_destroy(broker: *mut c_void);
    fn vnc_h264_rfb_client_count(broker: *mut c_void) -> c_int;
    fn vnc_h264_rfb_join_generation(broker: *mut c_void) -> u64;
    fn vnc_h264_rfb_reset(broker: *mut c_void, width: c_int, height: c_int);
    fn vnc_h264_rfb_set_caps(broker: *mut c_void, value: c_int);
    fn vnc_h264_rfb_tick(broker: *mut c_void);
    #[allow(clippy::too_many_arguments)]
    fn vnc_h264_rfb_submit(
        broker: *mut c_void,
        data: *const u8,
        len: u32,
        is_config: c_int,
        is_idr: c_int,
        width: c_int,
        height: c_int,
    );
}

/// The codec's output-buffer flags, as `poll_output` hands them over verbatim from
/// `AMediaCodecBufferInfo::flags` (crosvm_android_display_client.cpp: `*outFlags = info.flags`).
///
/// `CODEC_CONFIG` is `AMEDIACODEC_BUFFER_FLAG_CODEC_CONFIG` from the NDK's NdkMediaCodec.h, which
/// is where the C++ side reads it. `SYNC_FRAME` is `MediaCodec.BUFFER_FLAG_KEY_FRAME`, spelled out
/// here rather than taken from that header because the NDK only named it at API 34 and this tree
/// builds against 33 -- the value is the same one the Java constant has always had.
const BUFFER_FLAG_SYNC_FRAME: u32 = 1;
const BUFFER_FLAG_CODEC_CONFIG: u32 = 2;

/// The RFB-50 broadcaster (vnc_h264_rfb.c), which serves this same stream to ordinary VNC clients
/// on the RFB port.
///
/// Owned here rather than by the C server, and the direction is deliberate: the drain thread holds
/// an `Arc<H264Consumer>` and therefore outlives the display that destroys the server, so a broker
/// belonging to the server could be freed while a frame was on its way into it. Belonging to the
/// consumer, it cannot be: `vnc_server_destroy` detaches it instead, after LibVNCServer has joined
/// every client thread, and a submit that arrives afterwards finds a broker with no clients.
struct RfbBroker {
    ptr: *mut c_void,
}

// SAFETY: every entry point behind this pointer takes the broker's own mutex before touching
// anything shared, and the pointer itself is fixed for the object's life. Sharing it across the
// drain thread and the producer's is the whole point of it.
unsafe impl Send for RfbBroker {}
unsafe impl Sync for RfbBroker {}

impl RfbBroker {
    /// Builds the broadcaster and arms LibVNCServer's protocol extension. `None` if it cannot be
    /// built, which leaves every client on the pixel path exactly as before.
    fn create(server: *mut c_void) -> Option<RfbBroker> {
        // SAFETY: `server` is the live server handle, and the call only stores a pointer in it.
        let ptr = unsafe { vnc_h264_rfb_create(server) };
        if ptr.is_null() {
            return None;
        }
        Some(RfbBroker { ptr })
    }

    fn client_count(&self) -> i32 {
        // SAFETY: `ptr` came from `vnc_h264_rfb_create` and is live for `self`.
        unsafe { vnc_h264_rfb_client_count(self.ptr) }
    }

    fn join_generation(&self) -> u64 {
        // SAFETY: as above.
        unsafe { vnc_h264_rfb_join_generation(self.ptr) }
    }

    fn reset(&self, width: u32, height: u32) {
        // SAFETY: as above.
        unsafe { vnc_h264_rfb_reset(self.ptr, width as c_int, height as c_int) }
    }

    /// Declares what this host can do about H.264, so the broker can tell the clients that speak
    /// the pseudo-encoding (plans/H264_SINGLE_PORT.md §1). A value equal to the current one does
    /// nothing, so this is cheap to call from a path that only sometimes changes the answer.
    fn set_caps(&self, value: c_int) {
        // SAFETY: as above.
        unsafe { vnc_h264_rfb_set_caps(self.ptr, value) }
    }

    /// One turn of the broker's heartbeat clock. See `drain_loop`.
    fn tick(&self) {
        // SAFETY: as above.
        unsafe { vnc_h264_rfb_tick(self.ptr) }
    }

    /// Hands over one compressed unit. The geometry is the encoder's, not the screen's, so that a
    /// frame still draining out of a codec that has been replaced can be told apart from one the
    /// clients have been prepared for.
    fn submit(&self, payload: &[u8], flags: u32, width: u32, height: u32) {
        // SAFETY: the payload is borrowed for the duration of the call and copied by the callee
        // into whatever per-client queues it decides to; nothing of it is retained.
        unsafe {
            vnc_h264_rfb_submit(
                self.ptr,
                payload.as_ptr(),
                payload.len() as u32,
                ((flags & BUFFER_FLAG_CODEC_CONFIG) != 0) as c_int,
                ((flags & BUFFER_FLAG_SYNC_FRAME) != 0) as c_int,
                width as c_int,
                height as c_int,
            )
        }
    }
}

impl Drop for RfbBroker {
    fn drop(&mut self) {
        // SAFETY: the pointer came from `vnc_h264_rfb_create` and is destroyed once. Reached only
        // after every thread that could submit has dropped its `Arc<H264Consumer>`.
        unsafe { vnc_h264_rfb_destroy(self.ptr) }
    }
}

/// The name the bus knows this consumer by. NUL-terminated here because the bus keeps the pointer.
const CONSUMER_NAME: &[u8] = b"h264-rfb\0";

impl H264Consumer {
    /// Brings up the hardware-encode rung: builds the RFB broadcaster, starts the drain thread, and
    /// registers on the frame bus.
    ///
    /// Returns `None` if the bus has no room, and says why. It does NOT bring the encoder up --
    /// that waits until something is waiting for frames, so a VM nobody watches never loads the
    /// media stack, and the encoder is built against the geometry of a real frame rather than the
    /// one the command line guessed.
    ///
    /// Nothing here binds anything. The stream's only door is the RFB port the server has already
    /// been given, so there is no listener to fail, no port to collide, and no address for a
    /// caller to have to be told.
    pub fn start(server: *mut c_void) -> Option<Arc<H264Consumer>> {
        // Before the thread, because the drain thread reads this field, and before the frame bus
        // registration for the same reason the thread is: a failure after it would leave the
        // server pointing at a broker this function dropped on its way out. Dropping it is safe
        // even so -- `vnc_h264_rfb_destroy` takes the pointer back out of the server -- but the
        // server has not been started yet either, so no client can have reached it.
        let rfb = RfbBroker::create(server);
        if rfb.is_none() {
            base::error!("VNC h264: no RFB h264 broadcaster; this screen will serve pixels only");
        }

        let consumer = Arc::new(H264Consumer {
            rfb,
            rfb_joins_answered: AtomicU64::new(0),
            encoder: Mutex::new(None),
            gpu_route_failed: AtomicBool::new(false),
            encoder_failed: AtomicBool::new(false),
            was_feeding: AtomicBool::new(false),
            paused_offers: AtomicU64::new(0),
            encode_errors: AtomicU64::new(0),
            epoch: Instant::now(),
            stop: AtomicBool::new(false),
        });

        // Thread first, registration last, and the order is load-bearing: the bus keeps the `ctx`
        // pointer forever, so registering and THEN failing would leave it pointing at an `Arc`
        // this function dropped on its way out -- a dangling callback that fires on the next
        // frame. Nothing offers frames until the server is started, so there is no window in
        // which the thread exists and the registration does not.
        let drain_side = consumer.clone();
        if let Err(e) = thread::Builder::new()
            .name("vnc_h264_drain".into())
            .spawn(move || drain_side.drain_loop())
        {
            base::error!("VNC h264: cannot start the drain thread: {}", e);
            consumer.stop.store(true, Ordering::Relaxed);
            return None;
        }

        // The bus copies the descriptor by value but keeps `ctx`, so that pointer has to outlive
        // every offer. It does: `DisplayVnc` holds this `Arc` and is dropped after the server it
        // is registered with, and destroying the server is what stops offers.
        let descriptor = VncFrameConsumer {
            name: CONSUMER_NAME.as_ptr() as *const c_char,
            ctx: Arc::as_ptr(&consumer) as *mut c_void,
            on_frame: Some(h264_on_frame),
        };
        // SAFETY: `server` is the live server handle, and the descriptor is read and copied during
        // the call and not retained.
        if unsafe { vnc_server_attach_consumer(server, &descriptor) } == 0 {
            base::error!("VNC h264: the frame bus has no room for another consumer");
            consumer.stop.store(true, Ordering::Relaxed);
            return None;
        }

        base::info!("VNC h264: serving the stream on the RFB port as encoding 50");
        Some(consumer)
    }

    /// Moves compressed frames from the codec into the broker, and nowhere else.
    ///
    /// It keeps draining with no client connected, which is deliberate: the consumer stops FEEDING
    /// when the last client leaves, so what is left inside the codec is a handful of frames that
    /// have to come out before it can be reused. Draining them into nothing is how it gets back to
    /// idle.
    ///
    /// **This is also the thread that turns the broker's heartbeat clock**, and it belongs here
    /// rather than on either of the other two. The producer's thread is out of the question: it is
    /// the guest's flush path, and must not be given a job that has to happen on time. A client's
    /// own output thread cannot do it either -- it is asleep in `clientOutput` waiting for exactly
    /// the event that is not coming, which is what a heartbeat exists to say. This thread wakes ten
    /// times a second whether or not there is an encoder, because that is how long it parks in
    /// `poll_output`, so the tick is free and lands within a tenth of a second of when it is due.
    ///
    /// The tick and the caps push are both BEFORE the encoder is looked for, so a client is kept
    /// alive across a codec that is being rebuilt as much as across a screen that is not moving.
    fn drain_loop(self: Arc<Self>) {
        let mut buf: Vec<u8> = Vec::with_capacity(OUTPUT_BUFFER_INITIAL);
        let mut announced = 0u32;
        while !self.stop.load(Ordering::Relaxed) {
            let held = self.current_encoder();
            if let Some(rfb) = self.rfb.as_ref() {
                // Derived here, not remembered. `encoder_for` pushes each transition at the instant
                // it decides it, which is what makes the news prompt; this is the same answer read
                // off the state itself, so a transition that somehow went unreported cannot leave a
                // client waiting for ever on a stale value. `set_caps` does nothing when the two
                // agree, which is every tick but the one after a change.
                rfb.set_caps(caps_for(
                    self.encoder_failed.load(Ordering::Relaxed),
                    held.is_some(),
                ));
                rfb.tick();
            }
            let Some(encoder) = held else {
                thread::sleep(Duration::from_millis(100));
                continue;
            };
            self.sync_frame_for_rfb_joins(&encoder);
            buf.clear();
            match encoder.poll_output(&mut buf, OUTPUT_POLL_TIMEOUT_US) {
                PollOutput::Frame { flags } => {
                    // One line for the first few units of a stream, because "the encoder is
                    // producing parameter sets and IDRs" is the single fact that separates "it is
                    // running" from "it is running and a client could actually start decoding".
                    if announced < 3 {
                        announced += 1;
                        base::info!(
                            "VNC h264: unit {} is {} bytes, flags {:#x}",
                            announced,
                            buf.len(),
                            flags
                        );
                    }
                    // It queues and returns: the RFB clients' own output threads do the writing,
                    // so this call cannot be delayed by one of them that has stopped reading --
                    // which is the whole reason the broadcaster is built the way it is.
                    if let Some(rfb) = self.rfb.as_ref() {
                        rfb.submit(&buf, flags, encoder.width, encoder.height);
                    }
                }
                PollOutput::Idle => {}
                PollOutput::TooSmall(needed) => {
                    if needed >= OUTPUT_BUFFER_LOUD {
                        base::error!(
                            "VNC h264: a compressed frame is {} bytes; taking it anyway",
                            needed
                        );
                    } else {
                        base::info!("VNC h264: growing the output buffer to {} bytes", needed);
                    }
                    // `buf` was cleared at the top of the loop, so its length is zero and this
                    // asks for `needed` bytes of capacity rather than `needed` more than it has.
                    buf.reserve(needed);
                }
                PollOutput::Failed => {
                    // The codec is kept: a drain failure is not the same statement as "this device
                    // has no encoder", and the next poll may well succeed. The clients keep their
                    // connection and their heartbeats and see a still picture, which is what they
                    // would see anyway -- the one thing they must not be told is the permanent
                    // answer, which only a codec that could not be CREATED justifies.
                    base::error!("VNC h264: the encoder failed while draining");
                    thread::sleep(Duration::from_millis(500));
                }
            }
        }
    }

    fn current_encoder(&self) -> Option<Arc<Encoder>> {
        self.encoder.lock().ok().and_then(|g| g.clone())
    }

    /// Answers a client that has joined -- or been thrown out of -- the RFB stream with a sync
    /// frame, once per generation.
    ///
    /// A joining receiver is served nothing at all until an IDR arrives, because the alternative
    /// is showing it the middle of a stream. It runs on the drain thread rather than where the
    /// sink polls the generation, because the sink's poll happens on the producer's thread and the
    /// producer must never make a codec call -- and because this thread already wakes ten times a
    /// second holding the encoder, which is everything the request needs.
    fn sync_frame_for_rfb_joins(&self, encoder: &Encoder) {
        let Some(rfb) = self.rfb.as_ref() else {
            return;
        };
        let joins = rfb.join_generation();
        if self.rfb_joins_answered.swap(joins, Ordering::Relaxed) == joins {
            return;
        }
        encoder.request_sync_frame();
        base::info!(
            "VNC h264: an RFB client is waiting to start the stream (join {}); sync frame requested",
            joins
        );
    }

    /// The encoder for a frame of this size, building or rebuilding it if that is what it takes.
    ///
    /// **The two caps transitions §1 names are decided here**, because this is the only place that
    /// learns either of them: a codec that came up, and a codec that could not be created. They are
    /// pushed straight into the broker rather than left for the drain thread to notice, so that
    /// "there will never be a stream on this host" reaches a client in the same instant the host
    /// found it out, and not one poll later. `set_caps` does nothing when the value has not moved,
    /// which is what makes it safe to call from a path that runs once per encoder.
    ///
    /// This runs on the producer's thread, and that is the one thing worth checking rather than
    /// assuming: `set_caps` takes the broker lock and, under it, `cl->updateMutex` -- the same pair
    /// in the same order as `submit`, and LibVNCServer holds that mutex for region arithmetic only
    /// (rfbserver.c:3280-3374, released before the encode and before the first byte goes out). A
    /// client that has stopped reading cannot hold it, so it cannot hold the guest's flush path
    /// either.
    fn encoder_for(&self, width: u32, height: u32) -> Option<Arc<Encoder>> {
        {
            let guard = self.encoder.lock().ok()?;
            if let Some(encoder) = guard.as_ref() {
                if encoder.width == width && encoder.height == height {
                    return Some(encoder.clone());
                }
            }
        }
        if self.encoder_failed.load(Ordering::Relaxed) {
            return None;
        }
        // Built outside the lock: bringing a codec up talks to mediaserver over binder and takes
        // long enough that the drain thread should not be waiting behind it.
        let built = Encoder::open(width, height).map(Arc::new);
        if built.is_none() {
            // The reason is already in the log, from the side of the FFI that knows it. What is
            // recorded here is only that it was asked and answered.
            self.encoder_failed.store(true, Ordering::Relaxed);
            base::error!(
                "VNC h264: no encoder for a {}x{} screen; this host serves pixels only",
                width,
                height
            );
            // The permanent answer, and the only one that licenses a client to stop waiting.
            if let Some(rfb) = self.rfb.as_ref() {
                rfb.set_caps(CAPS_UNAVAILABLE);
            }
            return None;
        }
        {
            let mut guard = self.encoder.lock().ok()?;
            *guard = built.clone();
        }
        // The RFB clients are not disconnected: their protocol has a way to say "the desktop is
        // this size now" and DVH2's did not, so they are put back to joining instead and restarted
        // on the next sync frame at the new geometry. Declaring it here rather than from the sink's
        // resize path is what keeps it in step with the codec that will actually produce those
        // frames.
        if let Some(rfb) = self.rfb.as_ref() {
            rfb.reset(width, height);
            rfb.set_caps(CAPS_AVAILABLE);
        }
        built
    }

    /// One offered frame, from the bus.
    fn on_frame(&self, offer: &VncFrameOffer) {
        if self.stop.load(Ordering::Relaxed) {
            return;
        }
        if !self.wants_frames() {
            // Paused. The classic consumer is still being served out of this very offer, so
            // pausing here costs an RFB client nothing -- which is the property the bus exists to
            // give.
            let count = self.paused_offers.fetch_add(1, Ordering::Relaxed);
            if self.was_feeding.swap(false, Ordering::Relaxed) {
                base::info!(
                    "VNC h264: nothing wants the stream; encoder paused ({} offers skipped so far)",
                    count
                );
            }
            return;
        }
        if !self.was_feeding.swap(true, Ordering::Relaxed) {
            base::info!("VNC h264: an RFB client is on the stream; feeding the encoder");
        }

        if offer.pixels.is_null() || offer.width <= 0 || offer.height <= 0 {
            return;
        }
        let width = offer.width as u32;
        let height = offer.height as u32;
        let Some(encoder) = self.encoder_for(width, height) else {
            return;
        };

        let cursor = CursorOverlay {
            pixels: offer.cursor_argb,
            width: offer.cursor_w,
            height: offer.cursor_h,
            x: offer.cursor_x,
            y: offer.cursor_y,
            visible: offer.cursor_visible != 0 && !offer.cursor_argb.is_null(),
        };
        let pts_us = self.epoch.elapsed().as_micros() as i64;

        // The GPU route needs a source that is still a GPU object, which a cursor-only offer does
        // not have -- see vnc_frame_consumer.h. Those fall to the upload of `pixels`, which for a
        // cursor move is the frame the last blit already read back, so nothing is lost.
        let use_gpu = !self.gpu_route_failed.load(Ordering::Relaxed)
            && offer.gpu_import_id != 0
            && !offer.gpu_blit_ctx.is_null();
        let attempt = if use_gpu {
            encoder.encode(
                offer.gpu_blit_ctx,
                offer.gpu_import_id,
                offer.pixels,
                offer.size,
                width,
                height,
                &cursor,
                pts_us,
            )
        } else {
            encoder.encode(
                std::ptr::null_mut(),
                0,
                offer.pixels,
                offer.size,
                width,
                height,
                &cursor,
                pts_us,
            )
        };

        let Err(reason) = attempt else {
            return;
        };
        if use_gpu {
            // The plan's §7 premise -- "a MediaCodec input Surface can be fed by Vulkan" --
            // answered in the negative, for this device. Loud and once: everything after this
            // frame is on the CPU route, so a second copy of the message would only repeat the
            // same statement about the same decision.
            self.gpu_route_failed.store(true, Ordering::Relaxed);
            base::error!(
                "VNC h264: feeding the codec by Vulkan blit failed ({}); every frame from here is \
                 uploaded by the CPU instead",
                reason
            );
            if let Err(reason) = encoder.encode(
                std::ptr::null_mut(),
                0,
                offer.pixels,
                offer.size,
                width,
                height,
                &cursor,
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
                "VNC h264: frame not encoded ({} so far): {}",
                count + 1,
                reason
            );
        }
    }

    /// Whether the stream needs frames to keep arriving: the broker's client count, and nothing
    /// else, now that the broker is the only audience.
    ///
    /// This is the sink's `has_consumer` answer as far as this consumer is concerned, and it has to
    /// be part of it, because otherwise it is a deadlock and not a refinement: the simplefb
    /// producer does not build a frame at all when the sink reports no consumer, the encoder is
    /// only built out of an offered frame, and a screen watched over RFB h264 alone would wait for
    /// a stream that is waiting for it.
    ///
    /// A client that advertised only the DroidVM pseudo-encoding is deliberately NOT counted: it
    /// asked what this host can do, not for a picture, and building a codec for it would be a
    /// mediaserver round trip nobody is watching. See `vnc_h264_rfb_client_count`.
    pub fn wants_frames(&self) -> bool {
        self.rfb.as_ref().map(|r| r.client_count()).unwrap_or(0) > 0
    }

    /// How many clients have joined the stream. See `DisplayT::consumer_generation`.
    ///
    /// A counter and not a flag, because the case that matters is a second client arriving while
    /// the first is still there -- the moment when every boolean in sight is already true, and the
    /// producer would otherwise re-supply nothing and leave the new stream showing a screen that
    /// had stopped moving before it joined. Only the fact that the number moved is ever read.
    pub fn connect_generation(&self) -> u64 {
        self.rfb.as_ref().map(|r| r.join_generation()).unwrap_or(0)
    }

    /// Stops the drain thread and says what the run did.
    pub fn shutdown(&self) {
        self.stop.store(true, Ordering::Relaxed);
        match self.current_encoder().map(|e| e.frame_counts()) {
            Some((queued, dropped)) => base::info!(
                "VNC h264: {} frames queued, {} dropped for want of an input buffer, {} offers \
                 skipped with nothing listening",
                queued,
                dropped,
                self.paused_offers.load(Ordering::Relaxed)
            ),
            None => base::info!(
                "VNC h264: no encoder was ever built; {} offers went past with nothing listening",
                self.paused_offers.load(Ordering::Relaxed)
            ),
        }
    }
}

/// The bus callback. Everything it does is `H264Consumer::on_frame`; it exists to put the `unsafe`
/// in one place with the reasoning beside it.
extern "C" fn h264_on_frame(_server: *mut c_void, ctx: *mut c_void, offer: *const VncFrameOffer) {
    if ctx.is_null() || offer.is_null() {
        return;
    }
    // SAFETY: `ctx` is the `Arc<H264Consumer>` pointer registered in `start`, kept alive by the
    // `DisplayVnc` that owns it for longer than the server that offers frames; `offer` is the
    // bridge's own stack object, valid for the length of this call.
    let consumer = unsafe { &*(ctx as *const H264Consumer) };
    let offer = unsafe { &*offer };
    consumer.on_frame(offer);
}

/// What the wire is entitled to assume, asserted here so that changing it breaks a test rather
/// than a client.
///
/// Almost nothing of the seam is reachable from a host test any more, and that is the point of the
/// single-port change rather than a gap in it: the bytes are written by vnc_h264_rfb.c and the
/// encoder is a device's MediaCodec. What is still decided on this side of the FFI is which of §1's
/// three caps values a given host state is, and a client that stops waiting for ever does so
/// because of it.
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn caps_says_permanent_only_when_a_codec_could_not_be_built() {
        // Nothing has asked for an encoder yet: warming, not available. Sending 0 here would
        // promise a stream before anything had tried to make one, and would leave §1's second caps
        // rect with no transition to report.
        assert_eq!(caps_for(false, false), CAPS_WARMING);
        assert_eq!(caps_for(false, true), CAPS_AVAILABLE);

        // Permanent outranks everything, including an encoder still held from before the failure:
        // it is the only value that licenses a client to stop asking, so it must not be reachable
        // by accident -- and must not be masked by one that is.
        assert_eq!(caps_for(true, false), CAPS_UNAVAILABLE);
        assert_eq!(caps_for(true, true), CAPS_UNAVAILABLE);
    }
}
