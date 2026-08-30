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
void vnc_server_set_input_event_fd(vnc_server_t* server, int fd);
int vnc_server_poll_input_event(vnc_server_t* server, struct vnc_input_event* out);

#ifdef __cplusplus
}
#endif

#endif
