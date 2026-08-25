// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Binds LibVNCServer (GPL-2.0-or-later); no LibVNCServer code is included here.
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

#ifndef VNC_FRAME_CONSUMER_H
#define VNC_FRAME_CONSUMER_H

#include <stddef.h>
#include <stdint.h>

#include "vnc_server_bridge.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The seam between "a frame was offered" and "somebody does something with it".
 *
 * Above the seam is ingest, and it runs once per offered frame no matter how many consumers
 * there are: clip the producer's buffer to the screen, compare it against the frame before it,
 * hand down the answer. Below the seam is one consumer at a time.
 *
 * The split exists because the two consumers this sink is growing want opposite things out of
 * the same frame. LibVNCServer wants to touch as little as possible -- the whole point of the
 * band damage is that a static desktop costs nothing -- while a hardware H.264 encoder wants the
 * entire picture every time, because that is what a codec swallows. An offer therefore carries
 * both: the whole frame, AND what part of it is new. Neither consumer has to know the other
 * exists, and neither of them repeats the comparison.
 *
 * The LibVNCServer consumer is registered internally when the server is created and is not
 * configurable. The H.264 one is registered by the Rust sink between `vnc_server_create` and
 * `vnc_server_start`, because whether it exists at all is a property of the binding (its transport
 * ceiling and its `h264-port=`) that this file has no way to learn.
 */

/* A run of consecutive full-width rows that differ from the previously offered frame. */
struct vnc_damage_band {
    int    y;     /* first row */
    int    rows;  /* how many rows; the last band of a screen is short unless the height divides */
    size_t off;   /* where the band starts in the frame, in bytes */
    size_t len;   /* how many bytes of it there are -- short if the producer's buffer ends first */
};

/* One frame, or one cursor-only update, as every consumer sees it. */
struct vnc_frame_offer {
    /* The guest scanout with NO cursor composited into it, packed to `width` pixels of 4 bytes
     * each in BGRX order. Borrowed for the duration of the call and not beyond it. `size` is how
     * much of it is readable, already clipped to the screen: a producer's buffer is not required
     * to be screen-sized, and was measured not to be during a resize. */
    const uint8_t* pixels;
    uint32_t       size;
    int            width;
    int            height;

    /* 1 when the guest produced a new frame, 0 when only the pointer moved. A consumer that
     * encodes pictures wants the first kind and can ignore the second; `bands` is always empty
     * for a cursor-only update, because not one guest pixel changed. */
    int full;

    /* What is new since the previous offer, top to bottom. Empty means the frame is byte for
     * byte the one before it. A consumer is free to ignore this entirely and take `pixels`
     * whole -- that is the difference the seam exists to allow. */
    const struct vnc_damage_band* bands;
    int                           band_count;

    /* Set when ingest had no buffer to compare against, so the single band above is the whole
     * frame. Not the same statement as "the diff happened to mark every band": it also says
     * nothing of the previous frame is left underneath, so a consumer that draws its own overlay
     * on top of the picture has nothing to restore. Only reachable when the comparison buffer
     * cannot be allocated. */
    int frame_replaced;

    /* The same picture as `pixels`, still where the GPU can reach it: the guest's own dmabuf, as
     * the producer imported it, ready to be blitted somewhere a consumer chooses. Both zero when
     * the frame came up the CPU transport, and both zero on a cursor-only update -- there the
     * previous frame's import may already have been released, and `pixels` is the honest answer
     * anyway because it is what the last blit left behind.
     *
     * The handle is only good for the duration of the call, like `pixels`. A consumer that wants
     * the frame later has to have copied it, which for the H.264 consumer means handing it to a
     * codec before returning.
     *
     * `gpu_import_id` is the source declared in the byte order a video encoder reads (R,G,B,A),
     * not the one LibVNCServer is served in -- the two orders are two imports of one dmabuf, and
     * this is the consumer that wants the other one. See blitSourceFourcc, C++ side. */
    void*   gpu_blit_ctx;
    int64_t gpu_import_id;

    /* The guest's hardware cursor as last reported, in the same BGRX order as `pixels` with a
     * meaningful alpha byte. `visible` is the whole answer: it already folds in "the guest hid
     * it" and "there is no image to draw". (cursor_x, cursor_y) is the image's top-left corner
     * with the hotspot already applied by the guest, and goes negative against the top and left
     * edges. */
    const uint8_t* cursor_argb;
    int cursor_w, cursor_h;
    int cursor_x, cursor_y;
    int cursor_visible;
};

/* A consumer of offered frames. Copied by value on registration, so the descriptor does not have
 * to outlive the call. */
struct vnc_frame_consumer {
    const char* name;
    /* Consumer-private state, handed back on every offer. NULL for the LibVNCServer path, whose
     * state is the server itself. */
    void* ctx;
    void (*on_frame)(vnc_server_t* server, void* ctx, const struct vnc_frame_offer* offer);
};

/* Registers a consumer, returning 0 if there is no room for it. Call it while the server is
 * being built: offers run on the producer's thread and the list is read without a lock, so it is
 * fixed by the time frames start arriving. */
int vnc_server_attach_consumer(vnc_server_t* server, const struct vnc_frame_consumer* consumer);

#ifdef __cplusplus
}
#endif

#endif
