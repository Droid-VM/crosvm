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
//!
//! **The codec lives in video_encoder.rs.** What is left here is the RFB-50 audience: the broker,
//! the caps values it advertises, and the frame-bus callback that hands each offer to the shared
//! `EncoderPipeline`.

use std::ffi::c_char;
use std::ffi::c_int;
use std::ffi::c_void;
use std::sync::Arc;

use crate::video_encoder::CursorOverlay;
use crate::video_encoder::EncodedAudience;
use crate::video_encoder::EncoderConfig;
use crate::video_encoder::EncoderPipeline;
use crate::video_encoder::EncoderState;
use crate::video_encoder::FeedFrame;
use crate::video_encoder::VideoCodec;
use crate::video_encoder::BUFFER_FLAG_CODEC_CONFIG;
use crate::video_encoder::BUFFER_FLAG_SYNC_FRAME;

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
// The consumer.
// -------------------------------------------------------------------------------------------

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

/// The RFB-50 broadcaster as the pipeline's audience.
///
/// It is the only audience there is on a VNC screen. `None` therefore means this consumer will
/// never serve anybody -- it stays registered on the bus, answers "nothing wants frames" to every
/// offer, and costs a pointer.
pub(crate) struct RfbAudience {
    rfb: Option<RfbBroker>,
}

impl EncodedAudience for RfbAudience {
    /// The broker's client count, and nothing else.
    ///
    /// A client that advertised only the DroidVM pseudo-encoding is deliberately NOT counted: it
    /// asked what this host can do, not for a picture, and building a codec for it would be a
    /// mediaserver round trip nobody is watching. See `vnc_h264_rfb_client_count`.
    fn wants_frames(&self) -> bool {
        self.rfb.as_ref().map(|r| r.client_count()).unwrap_or(0) > 0
    }

    /// A counter and not a flag, because the case that matters is a second client arriving while
    /// the first is still there -- the moment when every boolean in sight is already true.
    fn join_generation(&self) -> u64 {
        self.rfb.as_ref().map(|r| r.join_generation()).unwrap_or(0)
    }

    /// The caps value derived from the state rather than remembered, so a transition that somehow
    /// went unreported cannot leave a client waiting for ever on a stale value; and the turn of the
    /// heartbeat clock, which belongs on the drain thread because the producer's thread is the
    /// guest's flush path and a client's output thread is asleep waiting for exactly the event that
    /// is not coming. `set_caps` does nothing when the value has not moved.
    fn tick(&self, state: EncoderState) {
        if let Some(rfb) = self.rfb.as_ref() {
            rfb.set_caps(caps_for(
                state == EncoderState::Unavailable,
                state == EncoderState::Available,
            ));
            rfb.tick();
        }
    }

    /// The RFB clients are not disconnected: their protocol has a way to say "the desktop is this
    /// size now", so they are put back to joining instead and restarted on the next sync frame at
    /// the new geometry.
    ///
    /// On the producer's thread: `reset` and `set_caps` take the broker lock and, under it,
    /// `cl->updateMutex` -- the same pair in the same order as `submit`, and LibVNCServer holds that
    /// mutex for region arithmetic only (rfbserver.c:3280-3374). A client that has stopped reading
    /// cannot hold it, so it cannot hold the guest's flush path either.
    fn encoder_changed(&self, width: u32, height: u32) {
        if let Some(rfb) = self.rfb.as_ref() {
            rfb.reset(width, height);
            rfb.set_caps(CAPS_AVAILABLE);
        }
    }

    /// The permanent answer, pushed at the instant it is known rather than one tick later: it is
    /// the only one that licenses a client to stop waiting.
    fn encoder_failed(&self) {
        if let Some(rfb) = self.rfb.as_ref() {
            rfb.set_caps(CAPS_UNAVAILABLE);
        }
    }

    /// It queues and returns: the RFB clients' own output threads do the writing, so this call
    /// cannot be delayed by one of them that has stopped reading.
    fn on_unit(
        &self,
        data: &[u8],
        flags: u32,
        _pts_us: i64,
        width: u32,
        height: u32,
        _codec: VideoCodec,
    ) {
        if let Some(rfb) = self.rfb.as_ref() {
            rfb.submit(data, flags, width, height);
        }
    }
}

/// The VNC sink's hardware-encode rung: the shared pipeline, with the RFB-50 broker as audience.
pub(crate) type H264Consumer = EncoderPipeline<RfbAudience>;

/// The name the bus knows this consumer by. NUL-terminated here because the bus keeps the pointer.
const CONSUMER_NAME: &[u8] = b"h264-rfb\0";

impl EncoderPipeline<RfbAudience> {
    /// Brings up the hardware-encode rung: builds the RFB broadcaster, starts the pipeline, and
    /// registers on the frame bus.
    ///
    /// Returns `None` if the bus has no room, and says why. It does NOT bring the encoder up --
    /// that waits until something is waiting for frames.
    ///
    /// Nothing here binds anything. The stream's only door is the RFB port the server has already
    /// been given, so there is no listener to fail, no port to collide, and no address for a
    /// caller to have to be told.
    pub fn start_vnc(server: *mut c_void) -> Option<Arc<H264Consumer>> {
        // Before the pipeline, and before the frame bus registration: a failure after it would
        // leave the server pointing at a broker this function dropped on its way out. Dropping it
        // is safe even so -- `vnc_h264_rfb_destroy` takes the pointer back out of the server --
        // but the server has not been started yet either, so no client can have reached it.
        let rfb = RfbBroker::create(server);
        if rfb.is_none() {
            base::error!("VNC h264: no RFB h264 broadcaster; this screen will serve pixels only");
        }

        // Pipeline (and its thread) first, registration last, and the order is load-bearing: the
        // bus keeps the `ctx` pointer forever, so registering and THEN failing would leave it
        // pointing at an `Arc` this function dropped on its way out.
        let consumer =
            match EncoderPipeline::start("VNC h264", EncoderConfig::default(), RfbAudience { rfb })
            {
                Ok(consumer) => consumer,
                Err(e) => {
                    base::error!("VNC h264: cannot start the drain thread: {}", e);
                    return None;
                }
            };

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
            consumer.shutdown();
            return None;
        }

        base::info!("VNC h264: serving the stream on the RFB port as encoding 50");
        Some(consumer)
    }
}

/// The bus callback: one offer, turned into the pipeline's frame.
///
/// The GPU route needs a source that is still a GPU object, which a cursor-only offer does not
/// have -- see vnc_frame_consumer.h. Those carry no import and fall to the upload of `pixels`,
/// which for a cursor move is the frame the last blit already read back, so nothing is lost.
extern "C" fn h264_on_frame(_server: *mut c_void, ctx: *mut c_void, offer: *const VncFrameOffer) {
    if ctx.is_null() || offer.is_null() {
        return;
    }
    // SAFETY: `ctx` is the `Arc<H264Consumer>` pointer registered in `start_vnc`, kept alive by
    // the `DisplayVnc` that owns it for longer than the server that offers frames; `offer` is the
    // bridge's own stack object, valid for the length of this call.
    let consumer = unsafe { &*(ctx as *const H264Consumer) };
    let offer = unsafe { &*offer };
    let cursor = CursorOverlay {
        pixels: offer.cursor_argb,
        width: offer.cursor_w,
        height: offer.cursor_h,
        x: offer.cursor_x,
        y: offer.cursor_y,
        visible: offer.cursor_visible != 0 && !offer.cursor_argb.is_null(),
    };
    // An offer with no pixels is not fed even when it names an import: that is the bus's contract
    // with this consumer (and the pipeline still counts it as a paused offer if nobody is waiting).
    let has_pixels = !offer.pixels.is_null();
    consumer.feed(&FeedFrame {
        pixels: offer.pixels,
        size: offer.size,
        width: offer.width.max(0) as u32,
        height: offer.height.max(0) as u32,
        gpu_blit_ctx: if has_pixels { offer.gpu_blit_ctx } else { std::ptr::null_mut() },
        gpu_import_id: if has_pixels { offer.gpu_import_id } else { 0 },
        cursor: &cursor,
    });
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
