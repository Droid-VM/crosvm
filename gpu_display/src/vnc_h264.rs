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
//! **Two audiences, one encoder.** The compressed stream leaves by two doors. This file owns the
//! side channel: a TCP port of its own, the DVH2 framing below, one client at a time, and it is
//! what the DroidVM app uses -- pixels from here, input over RFB, neither half having to know
//! about the other. The second door is `vnc_h264_rfb.c`, which serves the same stream inside
//! ordinary FramebufferUpdate messages on the RFB port itself, to third-party clients that ask for
//! the Open H.264 encoding (50). Both are fed from the drain thread below, out of one codec.
//!
//! The side channel came first and is not made redundant by the RFB one. It states the geometry in
//! a header instead of a rectangle, it is length-prefixed rather than request-driven, and it is
//! independent of whatever an RFB client is doing to the pixel path -- so it stays the thing the
//! app depends on, and the failure mode of the newer door is "a viewer sees no picture", not "the
//! app lost its stream".
//!
//! **Double encoding is real and bounded.** While a LEGACY RFB client and a side-channel client
//! are both connected the frame is encoded twice, once by LibVNCServer and once by the codec. That
//! is the honest cost of serving two client kinds at once and it is what the bus is for; it is not
//! paid when only one kind is connected, because this consumer feeds nothing with nothing waiting
//! for it and LibVNCServer marks nothing with no RFB client. An RFB client being served encoding
//! 50 does not pay it either: its pixel path is suppressed for as long as it is on the stream (see
//! vnc_h264_rfb.c), so it costs the codec nothing extra and LibVNCServer nothing at all.
//!
//! **The wire format**, in full, because a receiver has to be written against it:
//!
//! ```text
//! on connect:    "DVH2"          4 bytes, magic
//!                width           u16 little-endian
//!                height          u16 little-endian
//! then, forever: length          u32 little-endian
//!                payload         `length` bytes of Annex-B NAL units (start codes included)
//!                                -- or nothing at all, when `length` is zero: a heartbeat
//! ```
//!
//! A refused connection gets the magic `"DVHX"` followed by a NUL-terminated reason and is closed.
//! A client that does not read `"DVH2"` must give up rather than guess. The geometry in the header
//! describes every payload that follows it: if the screen changes size the connection is ended, so
//! that the new size is stated by a new header rather than inferred from the stream.
//!
//! **Liveness, in both directions, out of one mechanism.** The stream carries no traffic at all
//! while the screen is still, and a TCP connection with no traffic on it tells neither end anything
//! about the other. So a live connection that has had nothing to say for three seconds is sent a
//! frame of length zero and no payload. It is not a picture and it is not counted as one; it exists
//! so that both ends have something to time out against.
//!
//! * For the client, it means silence longer than the heartbeat interval is a fact about the host,
//!   not about the screen: a receiver sets its own read timeout and falls back when the beats stop.
//! * For the host, it means there is always a write to fail. A peer that is gone -- process dead,
//!   connection reset, cable pulled -- is discovered by the next beat, within three seconds,
//!   whatever the screen is doing. Before the heartbeat there was nothing to fail on: a client that
//!   left during a static EDK2 screen held the stream for seventeen measured seconds, because the
//!   write that would have failed only happened when a keystroke finally changed the picture.
//!
//! A peer that is still *there* and has merely stopped reading is a slower thing to see, and the
//! honest bound is worth stating: the write only blocks once the bytes it is not draining have
//! filled its receive buffer and then this end's send buffer (capped, see `CLIENT_SEND_BUFFER`),
//! and only then does the ten-second write timeout start. How long that takes is the buffering
//! divided by the bitrate -- seconds on a moving screen, longer on an idle one, where the only
//! thing not being read is a four-byte beat every three seconds and nothing is being missed.
//!
//! Heartbeats begin after the header and only on a promoted connection; a client that is still
//! waiting for the first encoder gets nothing until it is admitted or refused.
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
use std::os::fd::AsRawFd;
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
/// It is a bound on how long a stalled reader can occupy the stream, and nothing else has to be
/// traded against it: the write happens on the drain thread with no lock held, so a peer that has
/// stopped reading costs this module a drain thread and the codec a few buffers, never the
/// producer -- the guest's flush path takes the slot lock only to say the screen resized, and it
/// finds it free.
///
/// Ten seconds is generous for a LAN and deliberately so: the thing being caught here is a client
/// that is gone or wedged, not one that is briefly slow, and the heartbeat below is what turns
/// "gone" into a write that can time out at all.
const CLIENT_WRITE_TIMEOUT: Duration = Duration::from_secs(10);

/// How large a backlog the kernel may hold for a client before a write has to wait for it.
///
/// Measured, because the write timeout above turns out not to mean what it looks like: `write`
/// blocks only when the socket's send buffer is full, and Linux grows that buffer on its own --
/// 6.4 MB on the test device. A client that stopped reading a 40 KB/s stream absorbed four
/// megabytes over two minutes without a single write ever blocking, so the ten-second timeout had
/// nothing to count against and the slot stayed held. Capping the buffer converts the timeout from
/// "ten seconds after the kernel gives up growing" into "ten seconds after a quarter of a megabyte
/// is outstanding", which is a wall-clock bound as soon as the stream has a bitrate at all.
///
/// 256 KiB is about two thirds of a second of a 720p stream, and at any round trip this is used
/// over -- loopback, USB, a LAN -- it still allows tens of megabytes a second, which is more than
/// an order of magnitude above what the stream asks for.
///
/// It bounds this end only, and that is worth being honest about: the peer's receive buffer is not
/// ours to size, and a receiver that stops reading keeps acknowledging into its own buffer until
/// that is full too. The host cannot see a stalled reader any sooner than the reader's own buffer
/// allows -- which is the one thing a write-only protocol cannot fix from this side.
const CLIENT_SEND_BUFFER: c_int = 256 * 1024;

/// How long a promoted client may hear nothing before the host sends it a zero-length frame.
///
/// Three seconds is the whole liveness contract's unit: the receiver's read timeout is set against
/// it (comfortably longer, so a late beat is not a death), and the worst case for noticing a peer
/// that stopped reading is one of these plus one `CLIENT_WRITE_TIMEOUT`.
///
/// It is a floor on silence, not a schedule. A frame that goes out resets it, so a moving screen
/// sends no heartbeats at all -- there is nothing to prove when frames are arriving.
const HEARTBEAT_INTERVAL: Duration = Duration::from_secs(3);

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

// -------------------------------------------------------------------------------------------
// Why a connection was refused, as the wire says it.
//
// **The first token is machine-readable and stable.** A reason is `token: sentence`: everything
// before the first colon is a fixed word a receiver may dispatch on and this file may never
// reword, and everything after it is for a human reading a log and may be reworded at will. A
// receiver that matches on the sentence is matching on the wrong half.
//
// The tokens, and what each one licenses the client to do:
//
// * `no-encoder` -- this device cannot build an H.264 encoder at all. **Permanent**: the answer
//   comes from a codec that could not be created, which is a fact about the device and not about
//   this moment, so a client should stop asking and tell its user the stream is unavailable.
// * `busy` -- another client holds the stream. **Transient**: one at a time is a limitation of this
//   implementation, and the slot frees the moment that client leaves, so back off and retry.
// * `no-frame` -- nothing has been encoded yet; the screen may simply be idle. **Transient**, and
//   the token a client that only knows the two above should treat like any other unknown one: back
//   off and retry. New tokens may be added; the two named above will not change meaning.
//
// The set only grows, and no token ever changes what it means. A client that sees a token it does
// not know should retry rather than give up, because giving up is the one answer that only
// `no-encoder` justifies.
// -------------------------------------------------------------------------------------------

const REFUSE_NO_ENCODER: &str = "no-encoder: this device has no H.264 encoder we can use";
const REFUSE_BUSY: &str = "busy: another client already has the stream";
const REFUSE_NO_FRAME: &str = "no-frame: no encoded frame is available; the screen may be idle";

/// Caps how much the kernel will queue for this client. See `CLIENT_SEND_BUFFER`.
///
/// A failure is worth a line and nothing more: the stream still works, it is only the promptness of
/// noticing a stalled reader that is lost.
fn cap_send_buffer(stream: &TcpStream) {
    let size = CLIENT_SEND_BUFFER;
    // SAFETY: the fd belongs to `stream` and outlives the call; `size` is a live `c_int` and the
    // length passed is its own.
    let ret = unsafe {
        libc::setsockopt(
            stream.as_raw_fd(),
            libc::SOL_SOCKET,
            libc::SO_SNDBUF,
            &size as *const c_int as *const c_void,
            std::mem::size_of::<c_int>() as libc::socklen_t,
        )
    };
    if ret != 0 {
        base::info!(
            "VNC h264: could not cap the client's send buffer: {}",
            std::io::Error::last_os_error()
        );
    }
}

/// Turns a connection away with `"DVHX"` and a NUL-terminated reason, and drops it.
///
/// Errors are ignored on purpose: this is the last thing said to a socket nobody will read again,
/// and a client that has already left is not a problem worth a log line.
fn refuse(stream: &mut TcpStream, reason: &str) {
    let _ = stream.write_all(MAGIC_REFUSED);
    let _ = stream.write_all(reason.as_bytes());
    let _ = stream.write_all(&[0u8]);
}

/// Where the one client is in its life.
///
/// `Pending` exists because of an ordering that cannot be avoided: the header states the stream's
/// geometry, the geometry is a property of the first frame, and the first frame is only encoded
/// because somebody is waiting for it. So a connection is accepted, parked, and promoted by the
/// thread that accepted it once the producer has built an encoder.
#[derive(Default)]
struct ClientSlot {
    /// Connected, header sent, receiving payloads.
    ///
    /// Shared rather than owned because writing to it must not hold this lock: the drain thread
    /// takes a reference out, releases the lock, and writes for as long as the write timeout
    /// allows, while everything else -- a resize, a reconnect, the EOF peek -- goes on taking the
    /// lock and finding it free. The socket outlives its removal from the slot by exactly one
    /// in-flight write, which is what `shutdown` is for.
    live: Option<Arc<TcpStream>>,
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
    /// Payload frames written. Heartbeats are not payload and are counted separately, so that
    /// "how much did this client actually get" keeps meaning what it says on an idle screen.
    written: AtomicU64,
    heartbeats: AtomicU64,
    /// Milliseconds since `epoch` at the last byte successfully written to the live client, of
    /// either kind.
    ///
    /// One timestamp rather than two because the heartbeat rule reduces to one question -- has
    /// anything at all gone down this socket in the last interval -- and a heartbeat is an answer
    /// to it as much as a frame is. Reset when a client is promoted, so the first beat is due one
    /// interval after the header rather than immediately.
    last_write_ms: AtomicU64,
    epoch: Instant,
}

impl Channel {
    fn new() -> Channel {
        Channel {
            slot: Mutex::new(ClientSlot::default()),
            connected: AtomicBool::new(false),
            wanted: AtomicBool::new(false),
            written: AtomicU64::new(0),
            heartbeats: AtomicU64::new(0),
            last_write_ms: AtomicU64::new(0),
            epoch: Instant::now(),
        }
    }

    fn now_ms(&self) -> u64 {
        self.epoch.elapsed().as_millis() as u64
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

    /// Writes one length-prefixed unit to the live client, if there is one.
    ///
    /// The single place bytes reach a promoted connection, so that the framing exists once and
    /// heartbeats cannot be interleaved into the middle of a frame: both callers are the drain
    /// thread, and the header is written by the accepting thread before the connection is live.
    ///
    /// The lock is taken to find the socket and released before the write, so a peer that has
    /// stopped reading blocks this thread for the write timeout and blocks nobody else at all. A
    /// failed write drops the client rather than retrying: the stream is a sequence of NAL units,
    /// so a partial one is not a frame lost, it is the stream desynchronised for good.
    fn write_unit(&self, payload: &[u8], what: &str) -> bool {
        let stream = {
            let Ok(slot) = self.slot.lock() else {
                return false;
            };
            match slot.live.as_ref() {
                Some(stream) => stream.clone(),
                None => return false,
            }
        };
        let header = (payload.len() as u32).to_le_bytes();
        // `&TcpStream` writes, because the socket is shared: `write_all` needs `&mut Write`, and
        // that is the reference this gets without owning the stream.
        let result = (&*stream).write_all(&header).and_then(|_| {
            if payload.is_empty() {
                Ok(())
            } else {
                (&*stream).write_all(payload)
            }
        });
        match result {
            Ok(()) => {
                self.last_write_ms.store(self.now_ms(), Ordering::Relaxed);
                true
            }
            Err(e) => {
                // One line, whichever way it died, because the two are the same event to whoever
                // reads the log: the client is not taking bytes any more.
                let stalled = matches!(
                    e.kind(),
                    std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                );
                let why = if stalled {
                    let seconds = CLIENT_WRITE_TIMEOUT.as_secs();
                    format!("stopped reading; the {what} write timed out after {seconds}s")
                } else {
                    format!("left mid-{what}: {e}")
                };
                self.drop_live(&stream, &why);
                false
            }
        }
    }

    /// Writes one length-prefixed payload.
    fn send_frame(&self, payload: &[u8]) {
        if self.write_unit(payload, "frame") {
            self.written.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// Sends the zero-length frame if the connection has been silent for an interval.
    ///
    /// `now_ms` is passed in rather than read here so the rule can be tested without waiting three
    /// seconds for it. Called from the drain thread ten times a second, which is what makes the
    /// beat land within a tenth of a second of when it is due.
    fn heartbeat_if_due(&self, now_ms: u64) {
        if !self.has_client() {
            // Never before the header, and never to a connection still waiting for its first
            // encoder: `connected` is set at promotion and at no other time.
            return;
        }
        let idle_ms = now_ms.saturating_sub(self.last_write_ms.load(Ordering::Relaxed));
        if idle_ms < HEARTBEAT_INTERVAL.as_millis() as u64 {
            return;
        }
        if self.write_unit(&[], "heartbeat") {
            self.heartbeats.fetch_add(1, Ordering::Relaxed);
        }
    }

    /// Takes the live client away, if it is still the one this write was for.
    ///
    /// The identity check is what makes an out-of-lock write safe: by the time a stalled write
    /// gives up, the slot may hold a different client entirely -- the screen resized, the old
    /// socket was shut down, somebody else was admitted -- and the answer to "my write failed" is
    /// then "of course it did", not "drop whoever is there now".
    fn drop_live(&self, stream: &Arc<TcpStream>, why: &str) {
        let Ok(mut slot) = self.slot.lock() else {
            return;
        };
        let same = matches!(slot.live.as_ref(), Some(live) if Arc::ptr_eq(live, stream));
        if !same {
            return;
        }
        let _ = stream.shutdown(std::net::Shutdown::Both);
        slot.live = None;
        self.refresh_flags(&slot);
        base::info!("VNC h264: dropped the side-channel client: {}", why);
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
    /// client never sends anything, and an `EAGAIN` is the healthy answer.
    ///
    /// The peek asks for non-blocking by flag rather than by putting the socket into non-blocking
    /// mode, because the mode is a property of the socket and the drain thread may be inside a
    /// write on it: flipping it under a write in flight would turn one large frame into a short
    /// one and cost a healthy client its connection. `MSG_DONTWAIT` is per-call and cannot.
    fn reap_if_closed(&self) {
        let Ok(mut slot) = self.slot.lock() else {
            return;
        };
        let Some(stream) = slot.live.as_ref() else {
            return;
        };
        let mut probe = 0u8;
        // SAFETY: the fd is owned by the `TcpStream` this borrows and outlives the call; `recv`
        // writes at most the one byte offered and reports how many.
        let peeked = unsafe {
            libc::recv(
                stream.as_raw_fd(),
                &mut probe as *mut u8 as *mut c_void,
                1,
                libc::MSG_PEEK | libc::MSG_DONTWAIT,
            )
        };
        if peeked == 0 {
            let _ = stream.shutdown(std::net::Shutdown::Both);
            slot.live = None;
            self.refresh_flags(&slot);
            base::info!("VNC h264: the side-channel client closed the connection");
        }
    }

    /// Ends whatever connection there is. Used when the geometry changes underneath a client whose
    /// header said something else, and at shutdown.
    ///
    /// The shutdown is not a formality: a write may be in flight on the live socket, and dropping
    /// the slot's reference would leave it to finish or time out on its own. Shutting the socket
    /// down ends both at once -- the client sees the stream close now, and the in-flight write
    /// fails now instead of ten seconds from now.
    fn disconnect(&self, why: &str) {
        let Ok(mut slot) = self.slot.lock() else {
            return;
        };
        let had = slot.live.is_some() || slot.pending.is_some();
        if let Some(stream) = slot.live.take() {
            let _ = stream.shutdown(std::net::Shutdown::Both);
        }
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
    /// The RFB-50 broadcaster, or `None` on a build or a device where it could not be created. The
    /// side channel does not depend on it: the two are audiences for one encoder, not layers.
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

    fn vnc_h264_rfb_create(server: *mut c_void) -> *mut c_void;
    fn vnc_h264_rfb_destroy(broker: *mut c_void);
    fn vnc_h264_rfb_client_count(broker: *mut c_void) -> c_int;
    fn vnc_h264_rfb_join_generation(broker: *mut c_void) -> u64;
    fn vnc_h264_rfb_reset(broker: *mut c_void, width: c_int, height: c_int);
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
    /// built, which costs the side channel nothing.
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

        // Before the threads, because the drain thread reads this field, and before the frame bus
        // registration for the same reason the threads are: a failure after it would leave the
        // server pointing at a broker this function dropped on its way out. Dropping it is safe
        // even so -- `vnc_h264_rfb_destroy` takes the pointer back out of the server -- but the
        // server has not been started yet either, so no client can have reached it.
        let rfb = RfbBroker::create(server);
        if rfb.is_none() {
            base::error!("VNC h264: no RFB h264 broadcaster; only the side channel will be served");
        }

        let consumer = Arc::new(H264Consumer {
            channel: Channel::new(),
            rfb,
            rfb_joins_answered: AtomicU64::new(0),
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
        cap_send_buffer(&stream);

        // Before deciding the slot is taken: the client holding it may have closed while the
        // screen was still, in which case nothing has tried to write to it since.
        self.channel.reap_if_closed();

        {
            let Ok(mut slot) = self.channel.slot.lock() else {
                return;
            };
            if slot.live.is_some() || slot.pending.is_some() {
                base::info!("VNC h264: refused {} -- {}", peer, REFUSE_BUSY);
                refuse(&mut stream, REFUSE_BUSY);
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
                // The two are told apart on the wire because they mean opposite things to a
                // client: one says never ask again, the other says ask again later.
                let reason = if self.encoder_failed.load(Ordering::Relaxed) {
                    REFUSE_NO_ENCODER
                } else {
                    REFUSE_NO_FRAME
                };
                base::info!("VNC h264: refused {} -- {}", peer, reason);
                let Ok(mut slot) = self.channel.slot.lock() else {
                    return;
                };
                if let Some(mut waiting) = slot.pending.take() {
                    refuse(&mut waiting, reason);
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
        slot.live = Some(Arc::new(waiting));
        // Promotion is the first thing this connection has heard, so the heartbeat clock starts
        // here: no beat until it has been silent for an interval, and none at all before now --
        // the drain thread writes only to `live`, which is what this line makes it.
        self.channel
            .last_write_ms
            .store(self.channel.now_ms(), Ordering::Relaxed);
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
    ///
    /// **This is also where the heartbeat's three-second clock lives**, and it belongs here rather
    /// than on either of the other two threads. The producer is out of the question: it must never
    /// touch a socket, which is the property the whole `wanted` flag exists to protect. The
    /// listener could hold a timer -- it wakes every 200ms between accepts -- but it is not the
    /// thread that writes frames, so it would need a shared "did anything go out recently" answer
    /// AND it can be parked inside `admit` for as long as ten seconds waiting for a first encoder,
    /// which is three missed beats. The drain thread is the one that writes payloads, it wakes ten
    /// times a second whether or not there is an encoder, and "nothing has gone out for three
    /// seconds" is a fact it already has in its hands.
    fn drain_loop(self: Arc<Self>) {
        let mut buf: Vec<u8> = Vec::with_capacity(OUTPUT_BUFFER_INITIAL);
        let mut announced = 0u32;
        while !self.stop.load(Ordering::Relaxed) {
            // Before the encoder is looked for, so that a client is kept alive across a codec
            // that is being rebuilt as much as across a screen that is not moving.
            self.channel.heartbeat_if_due(self.channel.now_ms());
            let Some(encoder) = self.current_encoder() else {
                self.channel.reap_if_closed();
                thread::sleep(Duration::from_millis(100));
                continue;
            };
            // Ten times a second, which is what makes "the last client left" a fact the producer
            // learns from the pause rather than from the next failed write.
            self.channel.reap_if_closed();
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
                    if self.channel.has_client() {
                        self.channel.send_frame(&buf);
                    }
                    // The same bytes, to the other audience. It queues and returns: the RFB
                    // clients' own output threads do the writing, so this call cannot be delayed
                    // by one of them that has stopped reading -- which is the whole reason the
                    // broadcaster is built the way it is.
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

    /// Answers a client that has joined -- or been thrown out of -- the RFB stream with a sync
    /// frame, once per generation.
    ///
    /// The same mechanism as the side channel's, and deliberately not a second one: a joining
    /// receiver is served nothing at all until an IDR arrives, because the alternative is showing
    /// it the middle of a stream. It runs on the drain thread rather than where the sink polls the
    /// generation, because the sink's poll happens on the producer's thread and the producer must
    /// never make a codec call -- and because this thread already wakes ten times a second holding
    /// the encoder, which is everything the request needs.
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
        // The RFB clients are not disconnected the way the side-channel one is: their protocol has
        // a way to say "the desktop is this size now" and this one does not, so they are put back
        // to joining instead and restarted on the next sync frame at the new geometry. Declaring
        // it here rather than from the sink's resize path is what keeps it in step with the codec
        // that will actually produce those frames.
        if let Some(rfb) = self.rfb.as_ref() {
            rfb.reset(width, height);
        }
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
    ///
    /// An RFB client that asked for encoding 50 counts for exactly the same reason a side-channel
    /// one does, and it is the same deadlock if it does not: the encoder is only built out of an
    /// offered frame, and the simplefb producer only builds a frame when something says it wants
    /// one. A screen watched over RFB h264 alone would otherwise wait for a stream that is waiting
    /// for it.
    pub fn wants_frames(&self) -> bool {
        self.channel.is_wanted() || self.rfb.as_ref().map(|r| r.client_count()).unwrap_or(0) > 0
    }

    /// How many clients have joined either audience. See `DisplayT::consumer_generation`.
    ///
    /// Both halves are counters and they are packed rather than added, so that a side-channel
    /// client arriving in the same instant an RFB one leaves cannot come out as no change at all.
    /// Only the fact that the number moved is ever read.
    pub fn connect_generation(&self) -> u64 {
        let joins = self.rfb.as_ref().map(|r| r.join_generation()).unwrap_or(0);
        (self.connect_generation.load(Ordering::Relaxed) << 32) | (joins & 0xffff_ffff)
    }

    /// Stops the service threads and says what the run did.
    pub fn shutdown(&self) {
        self.stop.store(true, Ordering::Relaxed);
        self.channel.disconnect("the display is going away");
        match self.current_encoder().map(|e| e.frame_counts()) {
            Some((queued, dropped)) => base::info!(
                "VNC h264: {} frames queued, {} dropped for want of an input buffer, {} written to \
                 the socket, {} heartbeats, {} offers skipped with nothing listening",
                queued,
                dropped,
                self.channel.written.load(Ordering::Relaxed),
                self.channel.heartbeats.load(Ordering::Relaxed),
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

/// What a receiver is entitled to assume, asserted here so that changing it breaks a test rather
/// than a client.
///
/// The encoder is not reachable from a host test -- it is a device's MediaCodec -- but the wire is:
/// framing, the heartbeat rule and the refusal tokens are all decided on this side of the FFI and
/// all of them are what another codebase is written against. Each test drives a `Channel` over a
/// real loopback socket, so what is asserted is the bytes that arrive, not the intent.
#[cfg(test)]
mod tests {
    use std::io::ErrorKind;
    use std::io::Read;

    use super::*;

    /// A connected pair: the end the `Channel` writes to, and the end a client reads from.
    fn connected_pair() -> (TcpStream, TcpStream) {
        let listener = TcpListener::bind(("127.0.0.1", 0)).expect("bind loopback");
        let addr = listener.local_addr().expect("local addr");
        let client = TcpStream::connect(addr).expect("connect");
        let (server, _) = listener.accept().expect("accept");
        client
            .set_read_timeout(Some(Duration::from_secs(5)))
            .expect("read timeout");
        (server, client)
    }

    /// A channel with a promoted client, as `admit` would leave it.
    fn live_channel() -> (Channel, TcpStream) {
        let (server, client) = connected_pair();
        let channel = Channel::new();
        {
            let mut slot = channel.slot.lock().expect("slot");
            slot.live = Some(Arc::new(server));
            channel.refresh_flags(&slot);
        }
        (channel, client)
    }

    fn read_frame(client: &mut TcpStream) -> Vec<u8> {
        let mut length = [0u8; 4];
        client.read_exact(&mut length).expect("length prefix");
        let mut payload = vec![0u8; u32::from_le_bytes(length) as usize];
        if !payload.is_empty() {
            client.read_exact(&mut payload).expect("payload");
        }
        payload
    }

    /// Whether anything at all is waiting to be read, without waiting long for it.
    fn quiet(client: &mut TcpStream) -> bool {
        client
            .set_read_timeout(Some(Duration::from_millis(200)))
            .expect("read timeout");
        let mut byte = [0u8; 1];
        let answer = matches!(
            client.read(&mut byte),
            Err(ref e) if e.kind() == ErrorKind::WouldBlock || e.kind() == ErrorKind::TimedOut
        );
        client
            .set_read_timeout(Some(Duration::from_secs(5)))
            .expect("read timeout");
        answer
    }

    #[test]
    fn a_heartbeat_is_a_length_of_zero_and_nothing_else() {
        let (channel, mut client) = live_channel();
        let interval = HEARTBEAT_INTERVAL.as_millis() as u64;

        // Silence shorter than the interval is not yet worth a beat.
        channel.heartbeat_if_due(0);
        channel.heartbeat_if_due(interval - 1);
        assert_eq!(channel.heartbeats.load(Ordering::Relaxed), 0);
        assert!(quiet(&mut client));

        channel.heartbeat_if_due(interval);
        assert_eq!(channel.heartbeats.load(Ordering::Relaxed), 1);
        assert!(read_frame(&mut client).is_empty());
        // The four bytes of length were the whole of it: no payload followed.
        assert!(quiet(&mut client));

        // A beat resets the clock exactly as a frame does, so the next one is an interval away.
        let sent_at = channel.last_write_ms.load(Ordering::Relaxed);
        channel.heartbeat_if_due(sent_at + interval - 1);
        assert_eq!(channel.heartbeats.load(Ordering::Relaxed), 1);
        channel.heartbeat_if_due(sent_at + interval);
        assert_eq!(channel.heartbeats.load(Ordering::Relaxed), 2);
    }

    #[test]
    fn a_heartbeat_is_not_a_frame_in_any_of_the_accounting() {
        let (channel, mut client) = live_channel();
        channel.heartbeat_if_due(HEARTBEAT_INTERVAL.as_millis() as u64);
        assert!(read_frame(&mut client).is_empty());

        // Payload counters do not move for a beat, and neither does demand: the producer must not
        // start feeding an encoder because the host said "still here".
        assert_eq!(channel.written.load(Ordering::Relaxed), 0);
        assert_eq!(channel.heartbeats.load(Ordering::Relaxed), 1);
        assert!(channel.has_client());
        assert!(channel.is_wanted());
    }

    #[test]
    fn a_payload_frame_is_length_prefixed_and_defers_the_next_beat() {
        let (channel, mut client) = live_channel();
        let interval = HEARTBEAT_INTERVAL.as_millis() as u64;
        let unit = b"\x00\x00\x00\x01\x65 a coded picture".to_vec();

        channel.send_frame(&unit);
        assert_eq!(read_frame(&mut client), unit);
        assert_eq!(channel.written.load(Ordering::Relaxed), 1);

        let sent_at = channel.last_write_ms.load(Ordering::Relaxed);
        channel.heartbeat_if_due(sent_at + interval - 1);
        assert_eq!(channel.heartbeats.load(Ordering::Relaxed), 0);
        assert!(quiet(&mut client));

        channel.heartbeat_if_due(sent_at + interval);
        assert_eq!(channel.heartbeats.load(Ordering::Relaxed), 1);
        assert!(read_frame(&mut client).is_empty());
    }

    #[test]
    fn a_connection_waiting_for_its_first_encoder_is_never_written_to() {
        let (server, mut client) = connected_pair();
        let channel = Channel::new();
        {
            let mut slot = channel.slot.lock().expect("slot");
            slot.pending = Some(server);
            channel.refresh_flags(&slot);
        }

        // Wanted -- that is what makes the producer feed the encoder that will promote it -- but
        // not connected, and the heartbeat follows the second flag, not the first.
        assert!(channel.is_wanted());
        assert!(!channel.has_client());
        channel.heartbeat_if_due(u64::MAX);
        assert_eq!(channel.heartbeats.load(Ordering::Relaxed), 0);
        assert!(quiet(&mut client));
    }

    #[test]
    fn a_closed_client_is_reaped_without_a_frame_having_to_flow() {
        let (channel, client) = live_channel();
        drop(client);

        // The FIN has to arrive first; on loopback that is immediate, but "immediate" is not a
        // guarantee, so this waits for it rather than assuming it.
        for _ in 0..40 {
            channel.reap_if_closed();
            if !channel.has_client() {
                break;
            }
            thread::sleep(Duration::from_millis(50));
        }
        assert!(!channel.has_client());
        assert!(!channel.is_wanted());
    }

    #[test]
    fn a_failed_write_takes_the_client_away() {
        let (channel, client) = live_channel();
        drop(client);

        // The first write after the peer is gone often succeeds -- it lands in a buffer whose
        // reset is still in flight -- so what is asserted is that writing keeps not being a
        // no-op, not that the very first one fails.
        for _ in 0..40 {
            channel.send_frame(b"\x00\x00\x00\x01\x65 nobody is listening");
            if !channel.has_client() {
                break;
            }
            thread::sleep(Duration::from_millis(50));
        }
        assert!(!channel.has_client());
        assert!(!channel.is_wanted());
    }

    #[test]
    fn every_refusal_is_a_stable_token_and_a_human_tail() {
        for (token, reason) in [
            ("no-encoder", REFUSE_NO_ENCODER),
            ("busy", REFUSE_BUSY),
            ("no-frame", REFUSE_NO_FRAME),
        ] {
            // The token is what a receiver dispatches on, and the two ways a receiver is likely to
            // cut it out -- up to the colon, or the first word with the colon trimmed -- have to
            // agree, because the wire cannot say which one the other codebase chose.
            assert!(reason.starts_with(&format!("{token}: ")), "{}", reason);
            assert_eq!(reason.split(':').next(), Some(token));
            assert_eq!(
                reason
                    .split_whitespace()
                    .next()
                    .map(|word| word.trim_end_matches(':')),
                Some(token)
            );
            assert!(!token.contains(char::is_whitespace));
            assert!(!reason.contains('\0'));
            // A human tail, and a real one.
            assert!(reason[token.len() + 2..].len() > 8, "{}", reason);
        }
    }

    #[test]
    fn a_refusal_is_the_magic_the_reason_and_a_nul() {
        let (mut server, mut client) = connected_pair();
        refuse(&mut server, REFUSE_BUSY);
        drop(server);

        let mut said = Vec::new();
        client.read_to_end(&mut said).expect("read the refusal");
        let mut expected = MAGIC_REFUSED.to_vec();
        expected.extend_from_slice(REFUSE_BUSY.as_bytes());
        expected.push(0);
        assert_eq!(said, expected);
    }
}
