// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Binds LibVNCServer (GPL-2.0-or-later); no LibVNCServer code is included here.
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

#ifndef VNC_H264_RFB_H
#define VNC_H264_RFB_H

#include <stdint.h>

#include "vnc_server_bridge.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The Open H.264 encoding (RFB 50) broadcaster, and the DroidVM pseudo-encoding beside it: the
 * hardware stream, served to ordinary VNC clients on the ordinary RFB port. It is the only door
 * the stream leaves by -- the DVH2 side channel that used to sit on `RFB port + 100` is gone. See
 * vnc_h264_rfb.c for the wire format and for why the machinery is shaped the way it is.
 *
 * One broker per server (per screen). The protocol extension it registers with LibVNCServer is
 * process-global, so everything per-client routes back through `cl->screen` to find the broker
 * that owns that client. */
struct _rfbClientRec;

typedef struct vnc_h264_rfb vnc_h264_rfb_t;

/* Creates the broker for `server` and arms the protocol extension (once per process).
 *
 * Called by the Rust side between `vnc_server_create` and `vnc_server_start`, from the same place
 * and for the same reason the H.264 frame consumer is registered there: whether this server has a
 * hardware stream at all is a property of the binding's transport ceiling, which the bridge has no
 * way to learn. Returns NULL if it cannot be built, which leaves every client on the classic
 * pixel path exactly as before. */
vnc_h264_rfb_t* vnc_h264_rfb_create(vnc_server_t* server);

/* Frees the broker. The handle is owned by the caller (the Rust consumer), NOT by the server: the
 * consumer's drain thread submits frames through it and outlives the server at shutdown. */
void vnc_h264_rfb_destroy(vnc_h264_rfb_t* broker);

/* Severs the broker from its server. Called by `vnc_server_destroy` once every client thread has
 * been joined, so that a frame submitted afterwards reaches nothing rather than a freed screen. */
void vnc_h264_rfb_detach(vnc_h264_rfb_t* broker);

/* How many clients are being served the H.264 stream. THE sink's "does anything want frames"
 * answer as far as hardware encoding is concerned: the encoder is not built, and the producer does
 * not build frames, until something is waiting for them.
 *
 * Counts clients that asked for encoding 50, not clients this broker has a slot for: a client that
 * advertised only the DroidVM pseudo-encoding gets its capabilities answer and stays on the pixel
 * path, and must not be the reason a codec is brought up. */
int vnc_h264_rfb_client_count(vnc_h264_rfb_t* broker);

/* Bumped every time a client joins the stream or has to rejoin it. The Rust side polls this on its
 * drain thread and answers a change by asking the encoder for a sync frame -- a joining client is
 * served nothing until an IDR arrives. It is also the sink's whole "did the consumer set change"
 * answer, which is why it is a counter and not a flag. */
uint64_t vnc_h264_rfb_join_generation(vnc_h264_rfb_t* broker);

/* Declares the geometry of the stream. Called when an encoder is built or rebuilt.
 *
 * A geometry the broker has not seen before puts every client back into `joining` and drops the
 * cached parameter sets: the rectangle a client's decoder context is keyed by is about to change,
 * and nothing encoded against the old size can be decoded against the new one. Until this is
 * called at least once there is no stream, and clients that asked for encoding 50 keep being
 * served pixels -- which is what a device with no usable encoder ends up doing forever. */
void vnc_h264_rfb_reset(vnc_h264_rfb_t* broker, int width, int height);

/* One compressed unit, straight off the encoder's output queue.
 *
 * `is_config` marks parameter sets (SPS/PPS) and `is_idr` a sync frame; both come from the codec's
 * own buffer flags. `width`/`height` are the geometry of the encoder that produced it, so that
 * frames still draining out of a codec that has just been replaced are discarded rather than sent
 * to a decoder that has been told to expect the new size. */
void vnc_h264_rfb_submit(vnc_h264_rfb_t* broker, const uint8_t* data, uint32_t len,
                         int is_config, int is_idr, int width, int height);

/* The `value` byte of a capabilities rect, as H264_SINGLE_PORT.md §1 pins it. The numbers are the
 * wire, not an internal enum: a client dispatches on them. */
#define VNC_H264_RFB_CAPS_AVAILABLE   0 /* encoder up, or expected */
#define VNC_H264_RFB_CAPS_UNAVAILABLE 1 /* no encoder on this host, permanent -- stop waiting */
#define VNC_H264_RFB_CAPS_WARMING     2 /* asked for, not producing yet */

/* Declares what this host can currently do about H.264, and tells the clients that care.
 *
 * A broker starts at `WARMING`, because it is built before anything has tried to bring an encoder
 * up. Called from the Rust side wherever that answer is decided -- an encoder built, or one that
 * could not be. A value equal to the current one does nothing at all, so this is cheap to call on
 * a path that only sometimes changes the answer.
 *
 * Every DVH-aware client that has not been told this value is queued a capabilities rect and woken
 * to collect it (§1: the rect is re-sent whenever the value changes). */
void vnc_h264_rfb_set_caps(vnc_h264_rfb_t* broker, int value);

/* Periodic tick from the Rust drain thread, which already wakes ten times a second.
 *
 * The heartbeat's clock (§1: a request held with an empty stream queue for three seconds is
 * answered with a heartbeat rect) cannot live on a client's own output thread, because that thread
 * is asleep in `clientOutput` waiting for exactly the event that is not coming. So somebody has to
 * come and wake it, and this is that somebody: it looks for held requests that are due and pokes
 * those clients into calling the display hook, which is where the rect is actually written.
 *
 * Takes only the broker lock and, under it, `cl->updateMutex` -- the same pair, in the same order,
 * that `vnc_h264_rfb_submit` takes from the same thread. */
void vnc_h264_rfb_tick(vnc_h264_rfb_t* broker);

/* Runs from the bridge's `displayHook`, on the client's own output thread, with `cl->sendMutex`
 * held -- that is, at the top of the update LibVNCServer was about to compose. Returns 1 if this
 * file answered the client's outstanding request itself (an H.264 rect, or a DroidVM caps or
 * heartbeat rect), in which case the pixel path for this update has been suppressed. */
int vnc_h264_rfb_display_hook(vnc_h264_rfb_t* broker, struct _rfbClientRec* cl);

#ifdef __cplusplus
}
#endif

#endif
