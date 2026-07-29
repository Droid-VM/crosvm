// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright DroidVM contributors
// Binds LibVNCServer (GPL-2.0-or-later); no LibVNCServer code is included here.
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

#define _GNU_SOURCE
#include "vnc_server_bridge.h"

#include <rfb/rfb.h>
#include <rfb/keysym.h>
#include <linux/input-event-codes.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define INPUT_RING_SIZE 256
#define INPUT_RING_MASK (INPUT_RING_SIZE - 1)

struct input_ring {
    struct vnc_input_event buf[INPUT_RING_SIZE];
    volatile unsigned head;
    volatile unsigned tail;
};

struct vnc_server {
    rfbScreenInfoPtr screen;
    char* passwords[2];
    struct input_ring ring;
    pthread_mutex_t ring_lock;
    int input_event_fd;
};

struct keysym_entry {
    uint32_t keysym;
    uint16_t linux_keycode;
};

static const struct keysym_entry keysym_map[] = {
    { XK_Escape,      KEY_ESC },
    { XK_Return,      KEY_ENTER },
    { XK_BackSpace,   KEY_BACKSPACE },
    { XK_Tab,         KEY_TAB },
    { XK_space,       KEY_SPACE },
    { XK_Delete,      KEY_DELETE },
    { XK_Insert,      KEY_INSERT },
    { XK_Home,        KEY_HOME },
    { XK_End,         KEY_END },
    { XK_Page_Up,     KEY_PAGEUP },
    { XK_Page_Down,   KEY_PAGEDOWN },
    { XK_Left,        KEY_LEFT },
    { XK_Up,          KEY_UP },
    { XK_Right,       KEY_RIGHT },
    { XK_Down,        KEY_DOWN },
    { XK_Print,       KEY_SYSRQ },
    { XK_Scroll_Lock, KEY_SCROLLLOCK },
    { XK_Pause,       KEY_PAUSE },
    { XK_Num_Lock,    KEY_NUMLOCK },
    { XK_Menu,        KEY_COMPOSE },
    { XK_F1,  KEY_F1 },  { XK_F2,  KEY_F2 },  { XK_F3,  KEY_F3 },
    { XK_F4,  KEY_F4 },  { XK_F5,  KEY_F5 },  { XK_F6,  KEY_F6 },
    { XK_F7,  KEY_F7 },  { XK_F8,  KEY_F8 },  { XK_F9,  KEY_F9 },
    { XK_F10, KEY_F10 }, { XK_F11, KEY_F11 }, { XK_F12, KEY_F12 },
    { XK_Shift_L,   KEY_LEFTSHIFT },  { XK_Shift_R,   KEY_RIGHTSHIFT },
    { XK_Control_L, KEY_LEFTCTRL },   { XK_Control_R, KEY_RIGHTCTRL },
    { XK_Alt_L,     KEY_LEFTALT },    { XK_Alt_R,     KEY_RIGHTALT },
    { XK_Super_L,   KEY_LEFTMETA },   { XK_Super_R,   KEY_RIGHTMETA },
    { XK_Caps_Lock, KEY_CAPSLOCK },
    { XK_0, KEY_0 }, { XK_1, KEY_1 }, { XK_2, KEY_2 }, { XK_3, KEY_3 },
    { XK_4, KEY_4 }, { XK_5, KEY_5 }, { XK_6, KEY_6 }, { XK_7, KEY_7 },
    { XK_8, KEY_8 }, { XK_9, KEY_9 },
    { XK_a, KEY_A }, { XK_b, KEY_B }, { XK_c, KEY_C }, { XK_d, KEY_D },
    { XK_e, KEY_E }, { XK_f, KEY_F }, { XK_g, KEY_G }, { XK_h, KEY_H },
    { XK_i, KEY_I }, { XK_j, KEY_J }, { XK_k, KEY_K }, { XK_l, KEY_L },
    { XK_m, KEY_M }, { XK_n, KEY_N }, { XK_o, KEY_O }, { XK_p, KEY_P },
    { XK_q, KEY_Q }, { XK_r, KEY_R }, { XK_s, KEY_S }, { XK_t, KEY_T },
    { XK_u, KEY_U }, { XK_v, KEY_V }, { XK_w, KEY_W }, { XK_x, KEY_X },
    { XK_y, KEY_Y }, { XK_z, KEY_Z },
    { XK_A, KEY_A }, { XK_B, KEY_B }, { XK_C, KEY_C }, { XK_D, KEY_D },
    { XK_E, KEY_E }, { XK_F, KEY_F }, { XK_G, KEY_G }, { XK_H, KEY_H },
    { XK_I, KEY_I }, { XK_J, KEY_J }, { XK_K, KEY_K }, { XK_L, KEY_L },
    { XK_M, KEY_M }, { XK_N, KEY_N }, { XK_O, KEY_O }, { XK_P, KEY_P },
    { XK_Q, KEY_Q }, { XK_R, KEY_R }, { XK_S, KEY_S }, { XK_T, KEY_T },
    { XK_U, KEY_U }, { XK_V, KEY_V }, { XK_W, KEY_W }, { XK_X, KEY_X },
    { XK_Y, KEY_Y }, { XK_Z, KEY_Z },
    { XK_minus,        KEY_MINUS },
    { XK_equal,        KEY_EQUAL },
    { XK_bracketleft,  KEY_LEFTBRACE },
    { XK_bracketright, KEY_RIGHTBRACE },
    { XK_backslash,    KEY_BACKSLASH },
    { XK_semicolon,    KEY_SEMICOLON },
    { XK_apostrophe,   KEY_APOSTROPHE },
    { XK_grave,        KEY_GRAVE },
    { XK_comma,        KEY_COMMA },
    { XK_period,       KEY_DOT },
    { XK_slash,        KEY_SLASH },
    { XK_exclam,       KEY_1 },
    { XK_at,           KEY_2 },
    { XK_numbersign,   KEY_3 },
    { XK_dollar,       KEY_4 },
    { XK_percent,      KEY_5 },
    { XK_asciicircum,  KEY_6 },
    { XK_ampersand,    KEY_7 },
    { XK_asterisk,     KEY_8 },
    { XK_parenleft,    KEY_9 },
    { XK_parenright,   KEY_0 },
    { XK_underscore,   KEY_MINUS },
    { XK_plus,         KEY_EQUAL },
    { XK_braceleft,    KEY_LEFTBRACE },
    { XK_braceright,   KEY_RIGHTBRACE },
    { XK_bar,          KEY_BACKSLASH },
    { XK_colon,        KEY_SEMICOLON },
    { XK_quotedbl,     KEY_APOSTROPHE },
    { XK_asciitilde,   KEY_GRAVE },
    { XK_less,         KEY_COMMA },
    { XK_greater,      KEY_DOT },
    { XK_question,     KEY_SLASH },
    { XK_KP_Enter,     KEY_KPENTER },
    { XK_KP_Multiply,  KEY_KPASTERISK },
    { XK_KP_Add,       KEY_KPPLUS },
    { XK_KP_Subtract,  KEY_KPMINUS },
    { XK_KP_Decimal,   KEY_KPDOT },
    { XK_KP_Divide,    KEY_KPSLASH },
    { XK_KP_0, KEY_KP0 }, { XK_KP_1, KEY_KP1 }, { XK_KP_2, KEY_KP2 },
    { XK_KP_3, KEY_KP3 }, { XK_KP_4, KEY_KP4 }, { XK_KP_5, KEY_KP5 },
    { XK_KP_6, KEY_KP6 }, { XK_KP_7, KEY_KP7 }, { XK_KP_8, KEY_KP8 },
    { XK_KP_9, KEY_KP9 },
};
#define KEYSYM_MAP_SIZE (sizeof(keysym_map) / sizeof(keysym_map[0]))

static uint16_t keysym_to_linux(uint32_t keysym) {
    for (unsigned i = 0; i < KEYSYM_MAP_SIZE; i++)
        if (keysym_map[i].keysym == keysym)
            return keysym_map[i].linux_keycode;
    return 0;
}

static void ring_push(struct vnc_server* s, const struct vnc_input_event* ev) {
    pthread_mutex_lock(&s->ring_lock);
    unsigned next = (s->ring.head + 1) & INPUT_RING_MASK;
    if (next != s->ring.tail) {
        s->ring.buf[s->ring.head] = *ev;
        s->ring.head = next;
    }
    if (s->input_event_fd >= 0) {
        uint64_t val = 1;
        (void)write(s->input_event_fd, &val, sizeof(val));
    }
    pthread_mutex_unlock(&s->ring_lock);
}

static void vnc_kbd_callback(rfbBool down, rfbKeySym keySym, rfbClientPtr cl) {
    struct vnc_server* s = (struct vnc_server*)cl->screen->screenData;
    if (!s) return;
    uint16_t lkc = keysym_to_linux(keySym);
    if (lkc == 0) {
        fprintf(stderr, "VNC: unmapped keysym 0x%x\n", keySym);
        return;
    }
    struct vnc_input_event ev = {
        .type = VNC_INPUT_KEY,
        .down = down ? 1 : 0,
        .linux_keycode = lkc,
    };
    ring_push(s, &ev);
}

static void vnc_ptr_callback(int buttonMask, int x, int y, rfbClientPtr cl) {
    struct vnc_server* s = (struct vnc_server*)cl->screen->screenData;
    if (!s) return;
    struct vnc_input_event ev = {
        .type = VNC_INPUT_POINTER,
        .down = (buttonMask != 0) ? 1 : 0,
        .x = x,
        .y = y,
        .button_mask = (uint8_t)buttonMask,
    };
    ring_push(s, &ev);
    rfbDefaultPtrAddEvent(buttonMask, x, y, cl);
}

static void vnc_server_set_bgrx_format(rfbScreenInfoPtr screen) {
    screen->serverFormat.bitsPerPixel = 32;
    screen->serverFormat.depth = 24;
    screen->serverFormat.trueColour = TRUE;
    screen->serverFormat.bigEndian = 0;
    screen->serverFormat.redShift   = 16;
    screen->serverFormat.greenShift = 8;
    screen->serverFormat.blueShift  = 0;
    screen->serverFormat.redMax     = 0xFF;
    screen->serverFormat.greenMax   = 0xFF;
    screen->serverFormat.blueMax    = 0xFF;
}

vnc_server_t* vnc_server_create(int width, int height, int port, const char* password) {
    vnc_server_t* server = calloc(1, sizeof(vnc_server_t));
    if (!server)
        return NULL;
    pthread_mutex_init(&server->ring_lock, NULL);
    server->input_event_fd = -1;
    server->screen = rfbGetScreen(NULL, NULL, width, height, 8, 3, 4);
    if (!server->screen) {
        free(server);
        return NULL;
    }
    server->screen->frameBuffer = calloc(width * height, 4);
    if (!server->screen->frameBuffer) {
        rfbScreenCleanup(server->screen);
        free(server);
        return NULL;
    }
    server->screen->desktopName = "crosvm";
    server->screen->port = port;
    server->screen->ipv6port = port;
    server->screen->alwaysShared = TRUE;
    server->screen->screenData = server;
    server->screen->kbdAddEvent = vnc_kbd_callback;
    server->screen->ptrAddEvent = vnc_ptr_callback;
    if (password && password[0]) {
        server->passwords[0] = strdup(password);
        server->passwords[1] = NULL;
        server->screen->authPasswdData = (void*)server->passwords;
        server->screen->passwordCheck = rfbCheckPasswordByList;
    }
    vnc_server_set_bgrx_format(server->screen);
    rfbInitServer(server->screen);
    return server;
}

void vnc_server_start(vnc_server_t* server) {
    if (!server || !server->screen)
        return;
    rfbRunEventLoop(server->screen, -1, TRUE);
}

int vnc_server_has_input_events(vnc_server_t* server) {
    if (!server) return 0;
    return server->ring.head != server->ring.tail;
}

void vnc_server_set_input_event_fd(vnc_server_t* server, int fd) {
    if (!server) return;
    server->input_event_fd = fd;
}

int vnc_server_poll_input_event(vnc_server_t* server, struct vnc_input_event* out) {
    if (!server || !out) return VNC_INPUT_NONE;
    pthread_mutex_lock(&server->ring_lock);
    if (server->ring.head == server->ring.tail) {
        pthread_mutex_unlock(&server->ring_lock);
        return VNC_INPUT_NONE;
    }
    *out = server->ring.buf[server->ring.tail];
    server->ring.tail = (server->ring.tail + 1) & INPUT_RING_MASK;
    pthread_mutex_unlock(&server->ring_lock);
    return out->type;
}

int vnc_server_resize(vnc_server_t* server, int width, int height) {
    if (!server || !server->screen)
        return -1;
    if (server->screen->width == width && server->screen->height == height)
        return 0;
    char* new_fb = calloc(width * height, 4);
    if (!new_fb)
        return -1;
    char* old_fb = server->screen->frameBuffer;
    rfbNewFramebuffer(server->screen, new_fb, width, height, 8, 3, 4);
    vnc_server_set_bgrx_format(server->screen);
    free(old_fb);
    return 0;
}

void vnc_server_update_framebuffer(vnc_server_t* server, const uint8_t* data, uint32_t size) {
    if (!server || !server->screen || !server->screen->frameBuffer || !data)
        return;
    uint32_t fb_size = server->screen->width * server->screen->height * 4;
    if (size > fb_size)
        size = fb_size;
    memcpy(server->screen->frameBuffer, data, size);
    rfbMarkRectAsModified(server->screen, 0, 0, server->screen->width, server->screen->height);
}

void vnc_server_destroy(vnc_server_t* server) {
    if (!server)
        return;
    if (server->screen) {
        rfbShutdownServer(server->screen, TRUE);
        free(server->screen->frameBuffer);
        server->screen->frameBuffer = NULL;
        rfbScreenCleanup(server->screen);
    }
    pthread_mutex_destroy(&server->ring_lock);
    free(server->passwords[0]);
    free(server);
}
