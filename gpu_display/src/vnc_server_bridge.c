// SPDX-License-Identifier: GPL-2.0-or-later
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

#include "vnc_frame_consumer.h"
#include "vnc_h264_rfb.h"

#define INPUT_RING_SIZE 256
#define INPUT_RING_MASK (INPUT_RING_SIZE - 1)

/* Rows compared as one unit. Small enough that a pointer-sized change marks a small rectangle,
 * large enough that the per-band bookkeeping stays negligible against the memcmp. */
#define DAMAGE_BAND_ROWS 32

/* How many consumers a server can carry. Fixed rather than grown: this is a handful of entries
 * for the lifetime of the process, and a list that can fail to allocate would put an error path
 * on the frame route in exchange for nothing. */
#define VNC_MAX_FRAME_CONSUMERS 4

struct input_ring {
    struct vnc_input_event buf[INPUT_RING_SIZE];
    volatile unsigned head;
    volatile unsigned tail;
};

/* What the ingest half needs to say what changed. Shared by every consumer and owned by none of
 * them: the comparison happens once per offered frame however many are listening. */
struct vnc_ingest {
    /* The last cursor-free frame we were offered, kept so the next one can be compared against
     * it. screen->frameBuffer cannot serve: it has the cursor blended in, so the pointer's
     * rectangle would read as changed on every frame no matter what the guest drew. `valid` is
     * separate from the pointer because a freshly allocated buffer says nothing about what is on
     * screen -- an all-black guest frame would compare equal to a zeroed buffer and nothing
     * would be sent. */
    uint8_t* last_clean;
    uint32_t last_clean_size;
    int last_clean_valid;
    /* The band list handed down with each offer. Held here rather than built on the stack
     * because it is sized by the screen, and freed with the server. */
    struct vnc_damage_band* bands;
    int bands_cap;
};

struct vnc_server {
    rfbScreenInfoPtr screen;
    char* passwords[2];
    struct input_ring ring;
    pthread_mutex_t ring_lock;
    int input_event_fd;
    /* Rectangle the composited cursor currently occupies, so a move knows what to restore from
     * the clean frame. w==0 means nothing is drawn.
     *
     * This is the LibVNCServer consumer's own state, and it lives here rather than behind that
     * consumer's `ctx` because its canvas is the server's own screen -- there is nothing else
     * for it to hang off. A consumer that is not the bridge brings its own. */
    int drawn_x, drawn_y, drawn_w, drawn_h;
    struct vnc_ingest ingest;
    struct vnc_frame_consumer consumers[VNC_MAX_FRAME_CONSUMERS];
    int consumer_count;
    /* The Open H.264 broadcaster serving this screen's clients, or NULL when this binding has no
     * hardware stream. Borrowed, not owned -- see vnc_server_set_h264_rfb in the header. */
    struct vnc_h264_rfb* h264_rfb;
};

static void attach_frame_consumers(vnc_server_t* server);

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

/* Stop LibVNCServer compositing the cursor into the shared framebuffer.
 *
 * rfbSendFramebufferUpdate draws the cursor into screen->frameBuffer whenever the client it is
 * serving has enableCursorShapeUpdates == FALSE (rfbserver.c:3376), and erases it afterwards
 * (rfbserver.c:3628). Three things make that unusable here:
 *
 *   - The decision is PER-CLIENT but the canvas is PER-SCREEN. With alwaysShared, one viewer that
 *     never negotiated a cursor encoding paints into the same framebuffer another viewer's thread
 *     is encoding, so a client that draws its own pointer sees a second one baked into the pixels.
 *   - rfbHideCursor restores the pixels but never marks that rectangle modified, so the leftover
 *     stays on the client until some unrelated full-frame update washes it away. That is exactly
 *     the "stale content around the pointer" and "cursor only appears once it stops" behaviour.
 *   - SetEncodings clears the flag at rfbserver.c:2370 and only sets it back after blocking reads
 *     of the encoding list, so the window reopens every time a client renegotiates.
 *
 * displayHook runs at the top of rfbSendFramebufferUpdate (rfbserver.c:3180), BEFORE that gate, so
 * forcing the flag here closes it for good. Doing it in newClientHook instead would not survive
 * the reset at 2370.
 *
 * The flag's value on entry is also the only reliable answer to "did this client ask for cursor
 * updates?" -- SetEncodings resets every cursor flag, so none of them is a durable marker. A
 * client that asked already has it TRUE and is left alone; for one that did not, cursorWasChanged
 * is cleared as well so forcing the flag does not make us send a shape it never requested.
 */
static void vnc_display_hook(rfbClientPtr cl) {
    if (!cl->enableCursorShapeUpdates) {
        cl->enableCursorShapeUpdates = TRUE;
        cl->cursorWasChanged = FALSE;
    }

    /* AFTER the flag above, never before: the H.264 broadcaster suppresses cursor rectangles for
     * the clients it serves, and doing that first would destroy the one reliable answer to "did
     * this client ask for cursor updates?" that the block above depends on. */
    {
        struct vnc_server* s = (struct vnc_server*)cl->screen->screenData;
        if (s && s->h264_rfb)
            (void)vnc_h264_rfb_display_hook(s->h264_rfb, cl);
    }
}

void vnc_server_set_h264_rfb(vnc_server_t* server, struct vnc_h264_rfb* broker) {
    if (!server)
        return;
    server->h264_rfb = broker;
}

struct vnc_h264_rfb* vnc_server_h264_rfb_for_client(struct _rfbClientRec* client) {
    rfbClientPtr cl = (rfbClientPtr)client;
    struct vnc_server* server;
    if (!cl || !cl->screen)
        return NULL;
    /* Every screen in this process is one of ours, so screenData is a vnc_server_t: the bridge is
     * the only thing here that calls rfbGetScreen. */
    server = (struct vnc_server*)cl->screen->screenData;
    return server ? server->h264_rfb : NULL;
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
    server->screen->displayHook = vnc_display_hook;
    if (password && password[0]) {
        server->passwords[0] = strdup(password);
        server->passwords[1] = NULL;
        server->screen->authPasswdData = (void*)server->passwords;
        server->screen->passwordCheck = rfbCheckPasswordByList;
    }
    vnc_server_set_bgrx_format(server->screen);
    attach_frame_consumers(server);
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

    /* The framebuffer handed over above is freshly zeroed, which makes every "this band is already
     * on screen" verdict recorded in last_clean false. collect_damaged_bands would then skip
     * exactly the bands the guest is not repainting and leave them black -- marking nothing, so not
     * one byte goes out to report it. It shows up as a permanently black client at 0 bytes/s, and
     * only sometimes: ensure_ingest_buffers already reallocates (and invalidates) when the pixel
     * count changes, so the hole is a resize that lands back on a size seen before with no offer in
     * between. The display does exactly that while booting -- 1400x1050 -> 640x480 -> 1280x800 ->
     * 1400x1050 inside 200 ms, against a 33 ms producer.
     *
     * drawn_* is dropped for the same reason: it locates the cursor in the OLD geometry, and there
     * is nothing of the old frame left underneath it to restore. */
    server->ingest.last_clean_valid = 0;
    server->drawn_x = 0; server->drawn_y = 0;
    server->drawn_w = 0; server->drawn_h = 0;
    return 0;
}

void vnc_server_update_framebuffer(vnc_server_t* server, const uint8_t* data, uint32_t size) {
    if (!server || !server->screen || !server->screen->frameBuffer || !data)
        return;
    uint32_t fb_size = server->screen->width * server->screen->height * 4;
    if (size > fb_size)
        size = fb_size;
    memcpy(server->screen->frameBuffer, data, size);
    /* This path writes the server framebuffer without going through last_clean, so the band
     * comparison in collect_damaged_bands would afterwards skip every band whose *source* did not
     * change -- leaving those regions showing whatever was written here, permanently, with
     * nothing reporting a fault. Invalidating forces the next offer to refresh everything.
     *
     * There is no caller today (the Rust side declares this symbol and never uses it), so this
     * is not a live bug. It is one line because the function existing at all is an invitation:
     * whoever wires it up will not know a caching assumption depends on them. */
    server->ingest.last_clean_valid = 0;
    rfbMarkRectAsModified(server->screen, 0, 0, server->screen->width, server->screen->height);
}

void vnc_server_set_cursor(vnc_server_t* server, const uint8_t* argb,
                           int width, int height, int hot_x, int hot_y) {
    if (!server || !server->screen)
        return;

    /* width 0 (or no pixels) means "no cursor": the guest disables it with UPDATE_CURSOR
     * resource_id 0. rfbSetCursor(NULL) frees the previous one and stops advertising it. */
    if (!argb || width <= 0 || height <= 0) {
        rfbSetCursor(server->screen, NULL);
        return;
    }

    rfbCursorPtr c = calloc(1, sizeof(rfbCursor));
    if (!c)
        return;
    c->width  = width;
    c->height = height;
    c->xhot   = hot_x < 0 ? 0 : (hot_x >= width  ? width  - 1 : hot_x);
    c->yhot   = hot_y < 0 ? 0 : (hot_y >= height ? height - 1 : hot_y);

    /* richSource is in the SERVER pixel format, which vnc_server_set_bgrx_format pinned to the
     * same BGRX byte order the guest's cursor resource already uses -- so the colour bytes copy
     * straight across and only the alpha has to be split out into its own plane. */
    size_t px = (size_t)width * (size_t)height;
    c->richSource  = malloc(px * 4);
    c->alphaSource = malloc(px);
    if (!c->richSource || !c->alphaSource) {
        free(c->richSource);
        free(c->alphaSource);
        free(c);
        return;
    }
    memcpy(c->richSource, argb, px * 4);
    for (size_t i = 0; i < px; i++)
        c->alphaSource[i] = argb[i * 4 + 3];

    /* A mask is MANDATORY even for a rich cursor. rfbSendCursorShape dereferences
     * pCursor->mask[0] unconditionally, before it has decided which encoding to use:
     *
     *     if ( pCursor && pCursor->width == 1 && ... && pCursor->mask[0] == 0 )
     *
     * With only richSource set, the first client to negotiate RichCursor works fine and the
     * SECOND client -- which may negotiate plain XCursor -- takes crosvm down with a SIGSEGV at
     * a null fault address. rfbMakeMaskFromAlphaSource builds it from the alpha we already have.
     *
     * `source` is left NULL deliberately: cursor.c fills it in on demand with
     * rfbMakeXCursorFromRichCursor for clients that need the 1-bit form, and doing it lazily
     * avoids paying for a conversion no client may ever ask for. */
    c->mask = (unsigned char*)rfbMakeMaskFromAlphaSource(width, height, c->alphaSource);
    if (!c->mask) {
        free(c->richSource);
        free(c->alphaSource);
        free(c);
        return;
    }

    /* Tell LibVNCServer it owns these buffers: rfbSetCursor frees the OLD cursor using exactly
     * these flags, and a cursor moves several times a second, so getting them wrong is a leak
     * that grows for as long as the VM runs rather than a one-off. cleanupSource is TRUE even
     * though we pass no source, because cursor.c may allocate one later and that allocation has
     * to be freed by the same flags. */
    c->cleanup           = TRUE;
    c->cleanupRichSource = TRUE;
    c->cleanupSource     = TRUE;
    c->cleanupMask       = TRUE;

    /* The guest's cursor comes off a DRM cursor plane in DRM_FORMAT_HOST_ARGB8888, and both
     * compositors we care about (KWin, mutter) render those PREMULTIPLIED -- that is the Wayland
     * and DRM convention. Getting this wrong does not hide the cursor, it fringes it: straight
     * alpha declared premultiplied comes out too bright at the edges, and the reverse too dark.
     * Worth an eyeball at first light rather than trusting the convention blindly. */
    c->alphaPreMultiplied = TRUE;

    rfbSetCursor(server->screen, c);
}

void vnc_server_set_cursor_pos(vnc_server_t* server, int x, int y) {
    if (!server || !server->screen)
        return;
    /* Assignment is enough: LibVNCServer's per-client update check compares cl->cursorX against
     * screen->cursorX, so changing it here is what makes the next update carry the new position
     * (as a cursor-position message, or as a redraw for clients without cursor encodings). */
    server->screen->cursorX = x;
    server->screen->cursorY = y;
}

/* Alpha-blend one cursor image into the outgoing framebuffer at (cx,cy), clipped to the screen.
 * Both are BGRX in the server pixel format (vnc_server_set_bgrx_format pins that), so only the
 * alpha byte needs interpreting. Returns the rectangle actually touched via the out params. */
static void blend_cursor(rfbScreenInfoPtr screen, const uint8_t* cur, int cw, int ch,
                         int cx, int cy, int* out_x, int* out_y, int* out_w, int* out_h) {
    int sw = screen->width, sh = screen->height;
    int x0 = cx < 0 ? 0 : cx;
    int y0 = cy < 0 ? 0 : cy;
    int x1 = cx + cw > sw ? sw : cx + cw;
    int y1 = cy + ch > sh ? sh : cy + ch;
    *out_x = x0; *out_y = y0;
    *out_w = x1 > x0 ? x1 - x0 : 0;
    *out_h = y1 > y0 ? y1 - y0 : 0;
    if (*out_w == 0 || *out_h == 0)
        return;

    for (int y = y0; y < y1; y++) {
        const uint8_t* src = cur + (((y - cy) * cw) + (x0 - cx)) * 4;
        uint8_t* dst = (uint8_t*)screen->frameBuffer + ((size_t)y * sw + x0) * 4;
        for (int x = x0; x < x1; x++, src += 4, dst += 4) {
            uint32_t a = src[3];
            if (a == 0)
                continue;          /* fully transparent: the common case, most of a cursor image */
            if (a == 255) {
                dst[0] = src[0]; dst[1] = src[1]; dst[2] = src[2];
                continue;
            }
            /* Straight (non-premultiplied) alpha: the guest's cursor plane is ARGB8888 and both
             * KWin and mutter hand it over premultiplied, but treating it as premultiplied here
             * and being wrong darkens the edges, while this form is correct either way for the
             * fully-opaque and fully-transparent pixels that dominate a pointer. */
            for (int c = 0; c < 3; c++)
                dst[c] = (uint8_t)((src[c] * a + dst[c] * (255 - a)) / 255);
        }
    }
}

/* Copy one rectangle of the clean guest frame back over the outgoing framebuffer. */
/* The rectangle is clamped to the CURRENT screen, because it does not necessarily describe the
 * current screen: drawn_* is written by the previous frame, and vnc_server_resize can replace the
 * framebuffer with a smaller one in between. Unclamped, a rectangle left over from a taller screen
 * indexes past the end of the new buffer -- a heap write out of bounds, not a wrong pixel. That was
 * survivable while only the cursor-move path came here; the full-frame path calls it every frame
 * now. clean_size bounds the source for the same reason: `clean` is the producer's buffer and is
 * not required to be as large as the screen. */
static void restore_rect(rfbScreenInfoPtr screen, const uint8_t* clean, uint32_t clean_size,
                         int x, int y, int w, int h) {
    int sw = screen->width, sh = screen->height;
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x >= sw || y >= sh)
        return;
    if (x + w > sw) w = sw - x;
    if (y + h > sh) h = sh - y;
    if (w <= 0 || h <= 0)
        return;
    for (int r = 0; r < h; r++) {
        size_t off = ((size_t)(y + r) * sw + x) * 4;
        size_t len = (size_t)w * 4;
        if (off >= clean_size)
            break;
        if (off + len > clean_size)
            len = clean_size - off;
        memcpy((uint8_t*)screen->frameBuffer + off, clean + off, len);
    }
}

/* THE CLASSIC CONSUMER: the LibVNCServer path. blend_cursor and restore_rect above are its alone
 * -- nothing on the ingest side of the seam calls them, and the H.264 consumer that joined it does
 * its own blending into its own canvas.
 *
 * Copies the bands ingest says are new into the outgoing framebuffer, marks them, and puts the
 * cursor back on top -- erase where it was, draw where it is, mark both -- because banded copying
 * does not refresh the whole frame. A band that did not change is not rewritten, so the pointer's
 * old pixels would survive there unless restore_rect takes them out.
 *
 * A cursor-only offer arrives with no bands at all, so the loop does nothing and only the two
 * pointer rectangles move: that is what keeps the pointer travelling at input rate over a static
 * desktop without pushing a whole frame for every step. */
static void libvncserver_on_frame(vnc_server_t* server, void* ctx,
                                  const struct vnc_frame_offer* offer) {
    (void)ctx;
    rfbScreenInfoPtr screen = server->screen;
    int ox = server->drawn_x, oy = server->drawn_y;
    int ow = server->drawn_w, oh = server->drawn_h;
    int nx = 0, ny = 0, nw = 0, nh = 0;

    for (int i = 0; i < offer->band_count; i++) {
        const struct vnc_damage_band* band = &offer->bands[i];
        memcpy(screen->frameBuffer + band->off, offer->pixels + band->off, band->len);
        rfbMarkRectAsModified(screen, 0, band->y, offer->width, band->y + band->rows);
    }

    /* The whole frame was just rewritten in one piece, so nothing of the previous cursor is left
     * to restore -- and the rectangle it used to occupy is inside what was already marked. */
    if (offer->frame_replaced)
        ox = oy = ow = oh = 0;

    restore_rect(screen, offer->pixels, offer->size, ox, oy, ow, oh);
    if (offer->cursor_visible && offer->cursor_argb && offer->cursor_w > 0 && offer->cursor_h > 0)
        blend_cursor(screen, offer->cursor_argb, offer->cursor_w, offer->cursor_h, offer->cursor_x,
                     offer->cursor_y, &nx, &ny, &nw, &nh);
    server->drawn_x = nx; server->drawn_y = ny;
    server->drawn_w = nw; server->drawn_h = nh;

    if (ow > 0 && oh > 0)
        rfbMarkRectAsModified(screen, ox, oy, ox + ow, oy + oh);
    if (nw > 0 && nh > 0)
        rfbMarkRectAsModified(screen, nx, ny, nx + nw, ny + nh);
}

/* Where the compiled-in consumers are named. Unconditional -- there is no configuration surface
 * here, because whether LibVNCServer should be served is not a question this file gets to ask.
 *
 * The H.264 consumer is not in this list and could not be: whether it exists depends on the
 * binding's transport ceiling and on an encoder coming up, neither of which is known here. It
 * registers itself through vnc_server_attach_consumer before the server starts. */
static void attach_frame_consumers(vnc_server_t* server) {
    static const struct vnc_frame_consumer libvncserver = {
        .name = "libvncserver",
        .ctx = NULL,
        .on_frame = libvncserver_on_frame,
    };
    vnc_server_attach_consumer(server, &libvncserver);
}

int vnc_server_attach_consumer(vnc_server_t* server, const struct vnc_frame_consumer* consumer) {
    if (!server || !consumer || !consumer->on_frame)
        return 0;
    if (server->consumer_count >= VNC_MAX_FRAME_CONSUMERS)
        return 0;
    server->consumers[server->consumer_count++] = *consumer;
    return 1;
}

/* Makes the ingest buffers usable for a frame of `size` bytes on a screen `height` rows tall.
 * Returns 0 if it cannot, in which case the caller offers the whole frame untracked.
 *
 * The band list is grown first and never shrunk: it is four ints per 32 rows, so a screen's worth
 * is a rounding error next to the comparison buffer beside it. */
static int ensure_ingest_buffers(vnc_server_t* server, uint32_t size, int height) {
    int want = (height + DAMAGE_BAND_ROWS - 1) / DAMAGE_BAND_ROWS;
    if (want < 1)
        want = 1;
    if (server->ingest.bands_cap < want) {
        struct vnc_damage_band* grown = (struct vnc_damage_band*)realloc(
            server->ingest.bands, (size_t)want * sizeof(*grown));
        if (!grown)
            return 0;
        server->ingest.bands = grown;
        server->ingest.bands_cap = want;
    }
    if (server->ingest.last_clean && server->ingest.last_clean_size == size)
        return 1;
    free(server->ingest.last_clean);
    server->ingest.last_clean = (uint8_t*)malloc(size);
    if (!server->ingest.last_clean) {
        server->ingest.last_clean_size = 0;
        server->ingest.last_clean_valid = 0;
        return 0;
    }
    server->ingest.last_clean_size = size;
    server->ingest.last_clean_valid = 0;  /* contents unknown: the next pass must refresh all */
    return 1;
}

/* INGEST. Records which horizontal bands of `clean` differ from the previous frame and brings
 * last_clean into step with it. Returns how many bands are in server->ingest.bands.
 *
 * Nothing here touches a consumer, and it runs once however many are listening -- that is the
 * whole reason the comparison sits on this side of the seam.
 *
 * Without this the bridge marked the whole screen on every frame, so LibVNCServer re-encoded and
 * re-sent 1400x1050 whether or not a single pixel had moved: measured at 0.4-0.6 MB/s to a
 * connected client watching a completely static desktop, plus the encode behind it. A memcmp of
 * the frame is a fraction of that and skips both.
 *
 * Bands rather than a single whole-frame compare because the answer has to be a rectangle to mark,
 * and rather than tiles because the encoder's own unit is a row range -- a taller, full-width
 * rectangle costs it nothing extra. */
static int collect_damaged_bands(vnc_server_t* server, const uint8_t* clean, uint32_t clean_size,
                                 int width, int height) {
    size_t row_bytes = (size_t)width * 4;
    int force = !server->ingest.last_clean_valid;
    int count = 0;
    for (int y0 = 0; y0 < height; y0 += DAMAGE_BAND_ROWS) {
        int rows = (y0 + DAMAGE_BAND_ROWS <= height) ? DAMAGE_BAND_ROWS : (height - y0);
        size_t off = (size_t)y0 * row_bytes;
        if (off >= clean_size)
            break;
        size_t len = (size_t)rows * row_bytes;
        if (off + len > clean_size)
            len = clean_size - off;
        if (!force && memcmp(server->ingest.last_clean + off, clean + off, len) == 0)
            continue;
        memcpy(server->ingest.last_clean + off, clean + off, len);
        server->ingest.bands[count].y = y0;
        server->ingest.bands[count].rows = rows;
        server->ingest.bands[count].off = off;
        server->ingest.bands[count].len = len;
        count++;
    }
    server->ingest.last_clean_valid = 1;
    return count;
}

/* The acceptance instrument for a byte-identical refactor of the display pipeline (see the plan's
 * §6 step 4 and §9): the three CPU copy sites that feed this sink are being consolidated behind one
 * function, and "the sink receives the same bytes" is not an acceptance condition until something
 * measures it. It sits deliberately below everything that refactor touches -- and is the same hash,
 * over the same thing, in the same line shape as the native sink's -- so the two frame sequences
 * can simply be compared, to each other and across binaries.
 *
 * It is logged from the ingest point, one line per offered frame, which is what keeps it meaning
 * the same thing as consumers come and go: it describes what the sink was handed, not what any
 * one consumer did with it.
 *
 * `clean` is packed to screen->width, but the row loop is written against the stride anyway: the
 * number must describe the visible pixels and nothing else, or a padded producer would make two
 * identical frames disagree. Off unless CROSVM_DISPLAY_HASH_FRAMES=1, read once. */
static int frame_hash_enabled(void) {
    static int cached = -1;
    if (cached < 0) {
        const char* value = getenv("CROSVM_DISPLAY_HASH_FRAMES");
        cached = (value && (strcmp(value, "1") == 0 || strcmp(value, "true") == 0 ||
                            strcmp(value, "on") == 0))
                ? 1
                : 0;
    }
    return cached;
}

static void log_frame_hash(const char* kind, const uint8_t* clean, uint32_t clean_size, int width,
                           int height) {
    uint64_t hash = 0xcbf29ce484222325ULL;
    size_t row_bytes = (size_t)width * 4;
    size_t stride = row_bytes;
    for (int y = 0; y < height; y++) {
        size_t off = (size_t)y * stride;
        if (off + row_bytes > clean_size)
            break;
        for (size_t i = 0; i < row_bytes; i++)
            hash = (hash ^ clean[off + i]) * 0x100000001b3ULL;
    }
    fprintf(stderr, "VNC: FRAMEHASH surface=%s %dx%d fnv1a64=0x%016llx\n", kind, width, height,
            (unsigned long long)hash);
}

int vnc_server_has_clients(vnc_server_t* server) {
    if (!server || !server->screen)
        return 0;
    return server->screen->clientHead != NULL;
}

void vnc_server_offer_frame(vnc_server_t* server, const uint8_t* clean, uint32_t clean_size,
                            const uint8_t* cursor_argb, int cw, int ch,
                            int cx, int cy, int visible, int full,
                            void* gpu_blit_ctx, int64_t gpu_import_id) {
    if (!server || !server->screen || !server->screen->frameBuffer || !clean)
        return;
    rfbScreenInfoPtr screen = server->screen;

    if (frame_hash_enabled()) {
        uint32_t hashable = (uint32_t)screen->width * screen->height * 4;
        if (clean_size < hashable)
            hashable = clean_size;
        log_frame_hash(full ? "scanout" : "scanout-cursoronly", clean, hashable, screen->width,
                       screen->height);
    }

    /* No early return for "there are no clients" here, though everything below is work done for
     * nobody when the client list is empty. This is only reached from the producer's flip, and on
     * the virtio-gpu route that producer is the guest's own flush: a frame skipped here is never
     * offered again, so a client connecting to a guest that has since gone idle is served whatever
     * the framebuffer held when the last consumer left. That was measured as a permanently black
     * screen after a guest resolution change -- the resize zeroes the framebuffer, so the stale
     * content was black rather than merely old -- and it went away with a client held across the
     * same change. The skip belongs to the timer-driven producer, which picks a new client up on
     * its next tick; see simplefb_display_loop. */
    uint32_t fb_size = (uint32_t)screen->width * screen->height * 4;
    if (clean_size > fb_size)
        clean_size = fb_size;

    /* Lives here rather than in the offer because it is only ever one band and only ever on the
     * path where there is no band list to put it in. */
    struct vnc_damage_band whole;
    struct vnc_frame_offer offer;
    memset(&offer, 0, sizeof(offer));
    offer.pixels = clean;
    offer.size = clean_size;
    offer.width = screen->width;
    offer.height = screen->height;
    offer.full = full;
    offer.cursor_argb = cursor_argb;
    offer.cursor_w = cw;
    offer.cursor_h = ch;
    /* (cx,cy) is the cursor image's top-left corner, already hotspot-compensated by the guest --
     * measured, not assumed: a pointer driven to (700,400) with hot=(22,21) arrives here as
     * (678,379). Subtracting the hotspot a second time was drawing the pointer up and left of
     * the truth by exactly the hotspot, invisible on an arrow and 22px on a resize arrow.
     * blend_cursor clips negative origins, so a pointer against the left edge is simply cut off. */
    offer.cursor_x = cx;
    offer.cursor_y = cy;
    offer.cursor_visible = visible;
    offer.gpu_blit_ctx = gpu_blit_ctx;
    offer.gpu_import_id = gpu_import_id;

    if (full) {
        if (ensure_ingest_buffers(server, clean_size, screen->height)) {
            offer.bands = server->ingest.bands;
            offer.band_count =
                collect_damaged_bands(server, clean, clean_size, screen->width, screen->height);
        } else {
            /* No memory for the comparison buffer: nothing can be told apart from anything, so
             * the frame is offered as one band covering all of it. Correct, only wasteful. */
            whole.y = 0;
            whole.rows = screen->height;
            whole.off = 0;
            whole.len = clean_size;
            offer.bands = &whole;
            offer.band_count = 1;
            offer.frame_replaced = 1;
        }
    }

    for (int i = 0; i < server->consumer_count; i++)
        server->consumers[i].on_frame(server, server->consumers[i].ctx, &offer);
}

void vnc_server_destroy(vnc_server_t* server) {
    if (!server)
        return;
    free(server->ingest.last_clean);
    server->ingest.last_clean = NULL;
    server->ingest.last_clean_size = 0;
    server->ingest.last_clean_valid = 0;
    free(server->ingest.bands);
    server->ingest.bands = NULL;
    server->ingest.bands_cap = 0;
    if (server->screen) {
        rfbShutdownServer(server->screen, TRUE);
        /* After the shutdown and not before: rfbShutdownServer joins every client thread
         * (main.c:1245-1260), so by here no client can be inside the broker and every enrolled
         * client has already removed itself through clientGoneHook. Detaching rather than
         * destroying, because the broker belongs to the H.264 consumer whose drain thread may
         * still be holding a frame -- what this ends is its ability to reach a screen that is
         * about to be freed. */
        if (server->h264_rfb) {
            vnc_h264_rfb_detach(server->h264_rfb);
            server->h264_rfb = NULL;
        }
        free(server->screen->frameBuffer);
        server->screen->frameBuffer = NULL;
        rfbScreenCleanup(server->screen);
    }
    pthread_mutex_destroy(&server->ring_lock);
    free(server->passwords[0]);
    free(server);
}
