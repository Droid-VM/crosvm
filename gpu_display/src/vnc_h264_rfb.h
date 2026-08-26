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

/* The Open H.264 encoding (RFB 50) broadcaster: the same compressed stream the DVH2 side channel
 * carries, served to ordinary VNC clients on the ordinary RFB port. See vnc_h264_rfb.c for the
 * wire format and for why the machinery is shaped the way it is.
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

/* How many clients are being served the H.264 stream. Part of the sink's "does anything want
 * frames" answer: the encoder is not built, and the producer does not build frames, until
 * something is waiting for them. */
int vnc_h264_rfb_client_count(vnc_h264_rfb_t* broker);

/* Bumped every time a client joins the stream or has to rejoin it. The Rust side polls this
 * exactly where it polls the side channel's own connect generation, and answers a change by asking
 * the encoder for a sync frame -- a joining client is served nothing until an IDR arrives. */
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

/* Runs from the bridge's `displayHook`, on the client's own output thread, with `cl->sendMutex`
 * held -- that is, at the top of the update LibVNCServer was about to compose. Returns 1 if `cl`
 * is being served H.264, in which case the pixel path for this update has been suppressed. */
int vnc_h264_rfb_display_hook(vnc_h264_rfb_t* broker, struct _rfbClientRec* cl);

#ifdef __cplusplus
}
#endif

#endif
