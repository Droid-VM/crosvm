// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Binds LibVNCServer (GPL-2.0-or-later); no LibVNCServer code is included here.
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

#ifndef VNC_SERVER_BRIDGE_H
#define VNC_SERVER_BRIDGE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct vnc_server vnc_server_t;

#define VNC_INPUT_NONE      0
#define VNC_INPUT_KEY       1
#define VNC_INPUT_POINTER   2

struct vnc_input_event {
    uint8_t  type;
    uint8_t  down;
    uint16_t linux_keycode;
    int32_t  x;
    int32_t  y;
    uint8_t  button_mask;
};

vnc_server_t* vnc_server_create(int width, int height, int port, const char* password);
void vnc_server_start(vnc_server_t* server);
int vnc_server_has_input_events(vnc_server_t* server);
int vnc_server_resize(vnc_server_t* server, int width, int height);
void vnc_server_update_framebuffer(vnc_server_t* server, const uint8_t* data, uint32_t size);
void vnc_server_destroy(vnc_server_t* server);

/* Whether any RFB client is connected. Non-zero means a frame pushed now will actually be
 * encoded and sent; zero means every copy on the way there is wasted. Cheap enough to ask per
 * frame -- it is a NULL check on the client list head. */
int vnc_server_has_clients(vnc_server_t* server);

/* Publish the guest's hardware cursor.
 *
 * `argb` is width*height pixels of the guest's cursor resource in the same BGRX byte order as the
 * framebuffer, with a meaningful alpha byte. LibVNCServer then serves it two ways from one call:
 * clients that speak the Cursor pseudo-encoding draw the pointer themselves (no framebuffer
 * traffic at all when the mouse moves), and clients that do not get it composited into the
 * outgoing framebuffer at the position rfbDefaultPtrAddEvent has been tracking.
 *
 * width == 0 hides the cursor, which is how the guest disables it (UPDATE_CURSOR with
 * resource_id 0). */
void vnc_server_set_cursor(vnc_server_t* server, const uint8_t* argb,
                           int width, int height, int hot_x, int hot_y);

/* Where the guest thinks its pointer is.
 *
 * Needed because the pointer is not necessarily driven by the VNC client: the DroidVM app feeds
 * input over its own channel, so a passive viewer sends no PointerEvents and LibVNCServer's idea
 * of the cursor position would stay at (0,0) forever. Clients that draw the cursor themselves
 * ignore this; clients that do not get it composited here. */
void vnc_server_set_cursor_pos(vnc_server_t* server, int x, int y);

/* Offer the guest frame and the guest's cursor to whoever is consuming frames.
 *
 * This is the sink's ingest point. It works out what changed since the last offer and then hands
 * the frame to each registered consumer; compositing the cursor into an outgoing framebuffer is
 * one consumer's job, not this function's. See vnc_frame_consumer.h.
 *
 * `gpu_blit_ctx`/`gpu_import_id` describe the same picture as `clean` while it is still a GPU
 * object -- the guest dmabuf the producer imported -- for a consumer that would rather blit it
 * than read it. NULL/0 says there is no such thing for this frame, which is the answer on the CPU
 * transport and on every cursor-only update. They are arguments rather than state on the server
 * because the handle is only valid for the length of this call.
 *
 * `clean` is the guest scanout with NO cursor in it, and stays that way -- keeping a pristine
 * copy is what removes the need for LibVNCServer's save-under-cursor bookkeeping entirely: the
 * area the cursor used to cover is restored by copying from `clean`, never by remembering what
 * was underneath.
 *
 * full=1 for a new guest frame. full=0 for a cursor move, where not one guest pixel changed and
 * only the pointer's old and new rectangles are in play -- that is what lets the pointer keep
 * moving over a completely static desktop without pushing a whole frame. */
void vnc_server_offer_frame(vnc_server_t* server, const uint8_t* clean, uint32_t clean_size,
                            const uint8_t* cursor_argb, int cw, int ch,
                            int cx, int cy, int visible, int full,
                            void* gpu_blit_ctx, int64_t gpu_import_id);
void vnc_server_set_input_event_fd(vnc_server_t* server, int fd);
int vnc_server_poll_input_event(vnc_server_t* server, struct vnc_input_event* out);

#ifdef __cplusplus
}
#endif

#endif
