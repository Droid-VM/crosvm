// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

//! The VNC sink's hardware-encode rung: a second consumer on the frame bus, and the socket its
//! output leaves by.
//!
//! Plan §6 step 13. The frame the sink is already holding -- the same offer, at the same instant,
//! that the LibVNCServer consumer is turning into RFB rectangles -- is also handed to a MediaCodec
//! H.264 encoder, and the compressed result is served on a TCP port of its own. Nothing about RFB
//! changes: a legacy client connected to the same server keeps being served the same way it always
//! was, from the same offer, which is the entire reason step 12 split ingest from consumers.
//!
//! **Why a side channel and not an RFB encoding.** RFB has no H.264 pseudo-encoding that every
//! client understands, and inventing one would make this sink stop being a VNC server for anybody
//! who does not know about it. A separate port keeps the two audiences apart: an ordinary viewer
//! connects to 5900 and sees a picture, the DroidVM app connects to both -- pixels from the side
//! channel, input over RFB -- and neither has to know the other exists. It also means the failure
//! mode of the new thing is "nobody connects to the new port", not "VNC broke".
//!
//! **Double encoding is real and bounded.** While an RFB client and a side-channel client are both
//! connected the frame is encoded twice, once by LibVNCServer and once by the codec. That is the
//! honest cost of serving two client kinds at once and it is what the bus is for; it is not paid
//! when only one kind is connected, because this consumer feeds nothing with no side-channel
//! client and LibVNCServer marks nothing with no RFB client.
//!
//! **The wire format**, in full, because a receiver has to be written against it:
//!
//! ```text
//! on connect:    "DVH2"          4 bytes, magic
//!                width           u16 little-endian
//!                height          u16 little-endian
//! then, forever: length          u32 little-endian
//!                payload         `length` bytes of Annex-B NAL units (start codes included)
//! ```
//!
//! A refused connection gets the magic `"DVHX"` followed by a NUL-terminated reason and is closed.
//! A client that does not read `"DVH2"` must give up rather than guess. The geometry in the header
//! describes every payload that follows it: if the screen changes size the connection is ended, so
//! that the new size is stated by a new header rather than inferred from the stream.
//!
//! **SPS/PPS: sent on connect, not attached to every IDR.** The encoder emits its codec-specific
//! data exactly once, in its first output buffer, which serves the client that was already
//! connected and nobody after it. So the parameter sets are cached when they appear and written as
//! the first payload frame of every later connection, immediately before the sync frame that
//! connection asks for. The alternative -- prepend them to every IDR -- costs bytes every couple
//! of seconds forever to solve a problem that only exists at connect time. Because payloads are
//! concatenated by the receiver, the bytes that arrive are `SPS PPS IDR ...` either way, which is
//! what an Annex-B decoder wants to see.

use std::ffi::c_char;
use std::ffi::c_int;
use std::ffi::c_void;
use std::io::Write;
use std::net::SocketAddr;
use std::net::TcpListener;
use std::net::TcpStream;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::AtomicU64;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::sync::Mutex;
use std::thread;
use std::time::Duration;
use std::time::Instant;

/// The side channel's port, relative to the RFB port this sink already listens on.
///
/// A fixed offset rather than "the next free port": a client has to be able to work out where the
/// stream is without being told, and it already knows the VNC port. 100 rather than 1 so the
/// pairing survives the common habit of running several VMs on consecutive VNC ports -- 5900/5901
/// would collide with the neighbour's RFB port, 6000/6001 do not collide with anything.
/// `h264-port=` overrides it for the case where something else already holds the address.
pub const H264_PORT_OFFSET: u16 = 100;

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

/// Longest a drain iteration parks in the codec waiting for output. Also the shutdown latency.
const OUTPUT_POLL_TIMEOUT_US: i64 = 100_000;

/// How long a socket write is given before the client is considered gone.
///
/// There is a reason to be strict here that is not politeness: this socket is written from the
/// drain thread under the lock the producer takes when the screen resizes, so an unbounded write
/// to a stalled peer could stall the guest's flush path. Bounded, the worst case is this.
const CLIENT_WRITE_TIMEOUT: Duration = Duration::from_secs(2);

/// How long a connecting client waits for the first encoder to exist before being turned away.
///
/// It has to wait for one, because the header states the stream's geometry and there is no honest
/// answer until a frame has arrived to be encoded. Ten seconds is long enough for a guest that is
/// merely between frames and short enough that a client attached to a VM producing nothing at all
/// finds out.
const ADMIT_ENCODER_WAIT: Duration = Duration::from_secs(10);

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
// The socket.
// -------------------------------------------------------------------------------------------

const MAGIC_STREAM: &[u8; 4] = b"DVH2";
const MAGIC_REFUSED: &[u8; 4] = b"DVHX";

/// Where the one client is in its life.
///
/// `Pending` exists because of an ordering that cannot be avoided: the header states the stream's
/// geometry, the geometry is a property of the first frame, and the first frame is only encoded
/// because somebody is waiting for it. So a connection is accepted, parked, and promoted by the
/// thread that accepted it once the producer has built an encoder.
#[derive(Default)]
struct ClientSlot {
    /// Connected, header sent, receiving payloads.
    live: Option<TcpStream>,
    /// Accepted, waiting for the first encoder.
    pending: Option<TcpStream>,
}

/// The one client, and the flags everything else reads instead of locking to ask.
struct Channel {
    slot: Mutex<ClientSlot>,
    /// `slot.live.is_some()`. Read by the drain thread before every write.
    connected: AtomicBool,
    /// `slot.live.is_some() || slot.pending.is_some()`. Read once per offered frame on the
    /// producer's thread, so it is an atomic rather than a lock: the producer must never be able
    /// to wait on a socket write.
    wanted: AtomicBool,
    written: AtomicU64,
}

impl Channel {
    fn new() -> Channel {
        Channel {
            slot: Mutex::new(ClientSlot::default()),
            connected: AtomicBool::new(false),
            wanted: AtomicBool::new(false),
            written: AtomicU64::new(0),
        }
    }

    fn has_client(&self) -> bool {
        self.connected.load(Ordering::Relaxed)
    }

    /// Whether anything is waiting for frames, connected or not. This is what makes the producer
    /// feed the encoder, and it goes true one connection earlier than `has_client`.
    fn is_wanted(&self) -> bool {
        self.wanted.load(Ordering::Relaxed)
    }

    fn refresh_flags(&self, slot: &ClientSlot) {
        self.connected.store(slot.live.is_some(), Ordering::Relaxed);
        self.wanted
            .store(slot.live.is_some() || slot.pending.is_some(), Ordering::Relaxed);
    }

    /// Writes one length-prefixed payload. A failed write drops the client rather than retrying:
    /// the stream is a sequence of NAL units, so a partial one is not a frame lost, it is the
    /// stream desynchronised for good.
    fn send_frame(&self, payload: &[u8]) {
        let Ok(mut slot) = self.slot.lock() else {
            return;
        };
        let Some(stream) = slot.live.as_mut() else {
            return;
        };
        let header = (payload.len() as u32).to_le_bytes();
        let result = stream
            .write_all(&header)
            .and_then(|_| stream.write_all(payload));
        match result {
            Ok(()) => {
                self.written.fetch_add(1, Ordering::Relaxed);
            }
            Err(e) => {
                base::info!("VNC h264: side-channel client left mid-frame: {}", e);
                slot.live = None;
                self.refresh_flags(&slot);
            }
        }
    }

    /// Notices a client that has gone away without us having written to it.
    ///
    /// Needed because this socket only ever carries traffic in one direction, so the ordinary way
    /// a dead peer is discovered -- a failed write -- only happens when there is a frame to send.
    /// On a screen that has stopped moving there is no next frame, so a client that closed stays
    /// "connected" indefinitely: the port refuses the reconnect, and the producer goes on encoding
    /// for nobody. Measured, not feared -- a client that left during a static EDK2 screen held the
    /// slot for seventeen seconds, until a keystroke produced the frame whose write failed.
    ///
    /// A peek of zero bytes is EOF and nothing else: the protocol is write-only, so a well-behaved
    /// client never sends anything, and a `WouldBlock` is the healthy answer.
    fn reap_if_closed(&self) {
        let Ok(mut slot) = self.slot.lock() else {
            return;
        };
        let Some(stream) = slot.live.as_ref() else {
            return;
        };
        if stream.set_nonblocking(true).is_err() {
            return;
        }
        let mut probe = [0u8; 1];
        let closed = matches!(stream.peek(&mut probe), Ok(0));
        // Restored whatever the answer was: `send_frame` blocks on purpose, bounded by the write
        // timeout, and a socket left non-blocking would turn a large frame into a short write.
        let _ = stream.set_nonblocking(false);
        if closed {
            slot.live = None;
            self.refresh_flags(&slot);
            base::info!("VNC h264: the side-channel client closed the connection");
        }
    }

    /// Ends whatever connection there is. Used when the geometry changes underneath a client whose
    /// header said something else, and at shutdown.
    fn disconnect(&self, why: &str) {
        let Ok(mut slot) = self.slot.lock() else {
            return;
        };
        let had = slot.live.is_some() || slot.pending.is_some();
        slot.live = None;
        slot.pending = None;
        self.refresh_flags(&slot);
        if had {
            base::info!("VNC h264: dropped the side-channel client: {}", why);
        }
    }
}

// -------------------------------------------------------------------------------------------
// The consumer.
// -------------------------------------------------------------------------------------------

/// Everything the frame callback, the listener and the drain thread share.
///
/// One object rather than three because the three are one mechanism: a client arriving is what
/// makes the consumer feed the codec, and what the codec produces is what the client is sent.
pub(crate) struct H264Consumer {
    channel: Channel,
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
    /// Bumped every time a client is accepted. It is the sink's answer to "did the consumer set
    /// change" (`DisplayT::consumer_generation`), and it has to be a counter rather than a flag
    /// because the interesting case is a side-channel client arriving while an RFB one is already
    /// connected -- the moment when every boolean in sight is already true.
    connect_generation: AtomicU64,
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
}

/// The name the bus knows this consumer by. NUL-terminated here because the bus keeps the pointer.
const CONSUMER_NAME: &[u8] = b"h264-side-channel\0";

impl H264Consumer {
    /// Brings up the side channel: binds the listener, starts the two service threads, and
    /// registers on the frame bus.
    ///
    /// Returns `None` if the port cannot be bound or the bus has no room, and says why. It does
    /// NOT bring the encoder up -- that waits until something is waiting for frames, so a VM
    /// nobody watches never loads the media stack, and the encoder is built against the geometry
    /// of a real frame rather than the one the command line guessed.
    pub fn start(server: *mut c_void, port: u16) -> Option<Arc<H264Consumer>> {
        let listener = match TcpListener::bind(("0.0.0.0", port)) {
            Ok(l) => l,
            Err(e) => {
                base::error!("VNC h264: cannot listen on port {}: {}", port, e);
                return None;
            }
        };
        // Polled rather than blocked in, so `stop` is noticed without having to connect to
        // ourselves to wake the accept up.
        if let Err(e) = listener.set_nonblocking(true) {
            base::error!("VNC h264: cannot poll the listener on port {}: {}", port, e);
            return None;
        }

        let consumer = Arc::new(H264Consumer {
            channel: Channel::new(),
            encoder: Mutex::new(None),
            gpu_route_failed: AtomicBool::new(false),
            encoder_failed: AtomicBool::new(false),
            was_feeding: AtomicBool::new(false),
            paused_offers: AtomicU64::new(0),
            encode_errors: AtomicU64::new(0),
            connect_generation: AtomicU64::new(0),
            epoch: Instant::now(),
            stop: AtomicBool::new(false),
        });

        // Threads first, registration last, and the order is load-bearing: the bus keeps the `ctx`
        // pointer forever, so registering and THEN failing would leave it pointing at an `Arc`
        // this function dropped on its way out -- a dangling callback that fires on the next
        // frame. Nothing offers frames until the server is started, so there is no window in
        // which the threads exist and the registration does not.
        let accept_side = consumer.clone();
        if let Err(e) = thread::Builder::new()
            .name("vnc_h264_accept".into())
            .spawn(move || accept_side.accept_loop(listener))
        {
            base::error!("VNC h264: cannot start the listener thread: {}", e);
            consumer.stop.store(true, Ordering::Relaxed);
            return None;
        }
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

        base::info!(
            "VNC h264: side channel listening on port {} (length-prefixed Annex-B)",
            port
        );
        Some(consumer)
    }

    /// Accepts one client at a time and turns the rest away with a reason.
    ///
    /// One is a limitation, not a design: a second stream would need a second encoder, or a shared
    /// one with per-client sync frames, and neither is worth building before anything asks. Being
    /// turned away with `"DVHX"` and a sentence is at least something a client can report.
    fn accept_loop(self: Arc<Self>, listener: TcpListener) {
        while !self.stop.load(Ordering::Relaxed) {
            match listener.accept() {
                Ok((stream, peer)) => self.admit(stream, peer),
                Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    thread::sleep(Duration::from_millis(200));
                }
                Err(e) => {
                    base::error!("VNC h264: accept failed: {}", e);
                    thread::sleep(Duration::from_millis(500));
                }
            }
        }
    }

    /// Takes a connection, parks it until there is an encoder, then sends the header and promotes
    /// it. All of it on this thread, so that no part of a socket write is ever on the producer's.
    fn admit(&self, mut stream: TcpStream, peer: SocketAddr) {
        // Nagle would hold a small NAL back waiting for company, which on a stream whose whole
        // point is latency is the wrong trade.
        let _ = stream.set_nodelay(true);
        let _ = stream.set_write_timeout(Some(CLIENT_WRITE_TIMEOUT));

        // Before deciding the slot is taken: the client holding it may have closed while the
        // screen was still, in which case nothing has tried to write to it since.
        self.channel.reap_if_closed();

        {
            let Ok(mut slot) = self.channel.slot.lock() else {
                return;
            };
            if slot.live.is_some() || slot.pending.is_some() {
                base::info!("VNC h264: refused {} -- one client at a time", peer);
                let _ = stream.write_all(MAGIC_REFUSED);
                let _ = stream.write_all(b"another client already has the stream\0");
                return;
            }
            // Parked here rather than held as a local, so a second connection arriving while this
            // one waits is refused rather than allowed to race it -- and so the producer starts
            // feeding, which is the only thing that will ever produce the encoder waited for below.
            slot.pending = Some(stream);
            self.channel.refresh_flags(&slot);
            // Bumped under the same lock that made the connection visible, so a producer that
            // reads the generation after seeing `wants_frames` cannot see the old value.
            self.connect_generation.fetch_add(1, Ordering::Relaxed);
        }

        let deadline = Instant::now() + ADMIT_ENCODER_WAIT;
        let encoder = loop {
            if self.stop.load(Ordering::Relaxed) {
                self.channel.disconnect("the display is going away");
                return;
            }
            if let Some(encoder) = self.current_encoder() {
                break encoder;
            }
            if self.encoder_failed.load(Ordering::Relaxed) || Instant::now() >= deadline {
                base::info!(
                    "VNC h264: refused {} -- {}",
                    peer,
                    if self.encoder_failed.load(Ordering::Relaxed) {
                        "this device has no encoder we can use"
                    } else {
                        "no frame arrived to encode"
                    }
                );
                let Ok(mut slot) = self.channel.slot.lock() else {
                    return;
                };
                if let Some(mut waiting) = slot.pending.take() {
                    let _ = waiting.write_all(MAGIC_REFUSED);
                    let _ = waiting.write_all(
                        b"no encoded frame is available; the screen may be idle or unencodable\0",
                    );
                }
                self.channel.refresh_flags(&slot);
                return;
            }
            thread::sleep(Duration::from_millis(50));
        };

        let mut header = Vec::with_capacity(8);
        header.extend_from_slice(MAGIC_STREAM);
        header.extend_from_slice(&(encoder.width as u16).to_le_bytes());
        header.extend_from_slice(&(encoder.height as u16).to_le_bytes());
        // The parameter sets go out before the sync frame is even asked for: a decoder handed an
        // IDR with no SPS in front of it has nothing to decode it against. If the encoder has not
        // emitted them yet -- which is the case for the very first client, whose arrival is what
        // started the encoder -- then it has not emitted a coded picture either, and its own first
        // output buffer will carry them down this same socket.
        let config = encoder.codec_config();

        let Ok(mut slot) = self.channel.slot.lock() else {
            return;
        };
        let Some(mut waiting) = slot.pending.take() else {
            // Dropped while waiting: a resize, or shutdown.
            self.channel.refresh_flags(&slot);
            return;
        };
        let sent = waiting.write_all(&header).and_then(|_| match &config {
            Some(config) => waiting
                .write_all(&(config.len() as u32).to_le_bytes())
                .and_then(|_| waiting.write_all(config)),
            None => Ok(()),
        });
        if let Err(e) = sent {
            base::info!("VNC h264: {} left before the header: {}", peer, e);
            self.channel.refresh_flags(&slot);
            return;
        }
        slot.live = Some(waiting);
        self.channel.refresh_flags(&slot);
        drop(slot);

        encoder.request_sync_frame();
        base::info!(
            "VNC h264: {} attached to the {}x{} stream; {} bytes of parameter sets, sync frame \
             requested",
            peer,
            encoder.width,
            encoder.height,
            config.map(|c| c.len()).unwrap_or(0)
        );
    }

    /// Moves compressed frames from the codec to the socket, and nowhere else.
    ///
    /// It keeps draining with no client connected, which is deliberate: the consumer stops FEEDING
    /// when the last client leaves, so what is left inside the codec is a handful of frames that
    /// have to come out before it can be reused. Draining them into nothing is how it gets back to
    /// idle.
    fn drain_loop(self: Arc<Self>) {
        let mut buf: Vec<u8> = Vec::with_capacity(OUTPUT_BUFFER_INITIAL);
        let mut announced = 0u32;
        while !self.stop.load(Ordering::Relaxed) {
            let Some(encoder) = self.current_encoder() else {
                self.channel.reap_if_closed();
                thread::sleep(Duration::from_millis(100));
                continue;
            };
            // Ten times a second, which is what makes "the last client left" a fact the producer
            // learns from the pause rather than from the next failed write.
            self.channel.reap_if_closed();
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
                    if self.channel.has_client() {
                        self.channel.send_frame(&buf);
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
                    base::error!("VNC h264: the encoder failed while draining; stream ends");
                    self.channel.disconnect("the encoder failed");
                    thread::sleep(Duration::from_millis(500));
                }
            }
        }
    }

    fn current_encoder(&self) -> Option<Arc<Encoder>> {
        self.encoder.lock().ok().and_then(|g| g.clone())
    }

    /// The encoder for a frame of this size, building or rebuilding it if that is what it takes.
    ///
    /// A geometry change ends the current connection. The header a client was given states the
    /// size of everything that follows it, so sending a different size down the same connection
    /// would be a lie the receiver has no way to detect; making it reconnect restates the
    /// contract instead.
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
                "VNC h264: no encoder for a {}x{} screen; the side channel will serve nobody",
                width,
                height
            );
            return None;
        }
        let replaced = {
            let mut guard = self.encoder.lock().ok()?;
            let replaced = guard.is_some();
            *guard = built.clone();
            replaced
        };
        if replaced {
            self.channel
                .disconnect("the screen changed size; reconnect for the new geometry");
        }
        built
    }

    /// One offered frame, from the bus.
    fn on_frame(&self, offer: &VncFrameOffer) {
        if self.stop.load(Ordering::Relaxed) {
            return;
        }
        if !self.channel.is_wanted() {
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
            base::info!("VNC h264: a side-channel client is waiting; feeding the encoder");
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

    /// Whether the side channel needs frames to keep arriving.
    ///
    /// This is the sink's `has_consumer` answer as far as this consumer is concerned, and it has
    /// to be part of it: the simplefb producer does not build a frame at all when the sink reports
    /// no consumer, so a VM watched over the side channel and nothing else would be a stream of
    /// nothing -- the encoder waiting for offers that the producer is not making because the
    /// encoder is the only thing that wants them.
    ///
    /// True from the moment a connection is accepted, not from the moment it is promoted, because
    /// the promotion needs an encoder and the encoder needs a frame.
    pub fn wants_frames(&self) -> bool {
        self.channel.is_wanted()
    }

    /// How many clients this channel has accepted. See `DisplayT::consumer_generation`.
    pub fn connect_generation(&self) -> u64 {
        self.connect_generation.load(Ordering::Relaxed)
    }

    /// Stops the service threads and says what the run did.
    pub fn shutdown(&self) {
        self.stop.store(true, Ordering::Relaxed);
        self.channel.disconnect("the display is going away");
        match self.current_encoder().map(|e| e.frame_counts()) {
            Some((queued, dropped)) => base::info!(
                "VNC h264: {} frames queued, {} dropped for want of an input buffer, {} written to \
                 the socket, {} offers skipped with nothing listening",
                queued,
                dropped,
                self.channel.written.load(Ordering::Relaxed),
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
