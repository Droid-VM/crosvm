// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Binds LibVNCServer (GPL-2.0-or-later); no LibVNCServer code is included here.
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

/* THE OPEN H.264 ENCODING (RFB 50), served on the ordinary RFB port, and the one private
 * pseudo-encoding beside it.
 *
 * The hardware stream this broadcasts is the one vnc_h264.rs produces: one encoder, one Annex-B
 * stream, N receivers, and -- since the DVH2 side channel was retired -- exactly one door out of
 * the host, the RFB port the client is already connected to. The audience is TigerVNC >= 1.13 and
 * noVNC, which ask for encoding 50 in SetEncodings and expect the answer inside ordinary
 * FramebufferUpdate messages, and the DroidVM app, which asks for 50 and for 0x44564831 as well.
 *
 * Nothing about a client that never mentions 50 changes: this file is only ever reached for a
 * client that asked, on a screen whose transport ceiling allowed an encoder to exist at all.
 *
 * -------------------------------------------------------------------------------------------
 * THE WIRE, and where every byte of it comes from. The format is the clients', not ours.
 * -------------------------------------------------------------------------------------------
 *
 * One emission is one complete FramebufferUpdate carrying exactly one rectangle:
 *
 *     u8  0                       message type: FramebufferUpdate
 *     u8  pad
 *     u16 1                       number of rectangles
 *     u16 x = 0, u16 y = 0        the rectangle's origin
 *     u16 w, u16 h                the encoder's geometry
 *     s32 50                      encoding: Open H.264
 *     u32 length                  bytes of Annex-B that follow
 *     u32 flags                   ResetContext / ResetAllContexts
 *     u8[length]                  H.264 Annex-B, start codes included
 *
 * All of it big-endian, as everything in RFB is. `length` and `flags` are read with the same
 * big-endian primitives as the rest of the message on both clients: TigerVNC H264Decoder.cxx:81-82
 * (`is->readU32()`) and noVNC core/decoders/h264.js:295-296 (`sock.rQshift32()`).
 *
 * The vendored LibVNCServer's `rfbEncodingH264 0x48323634` (rfbproto.h) is an older, abandoned
 * proposal and is NOT this. 50 is defined here, locally, because the vendored header does not
 * know it.
 *
 * -------------------------------------------------------------------------------------------
 * THE DROIDVM PSEUDO-ENCODING 0x44564831, and the two things RFB cannot say without it.
 * -------------------------------------------------------------------------------------------
 *
 * Encoding 50 answers "here is a picture". It has no way to say "there will never be one on this
 * host, stop waiting" and no way to say "the screen is still, but I am alive" -- and a viewer that
 * cannot tell a still screen from a dead stream either hangs on a frozen picture or reconnects
 * through a perfectly good one. The DVH2 side channel said both, in its header and in its
 * zero-length heartbeat. With that channel gone, the two statements move here.
 *
 * The format is pinned in plans/H264_SINGLE_PORT.md §1 and is implemented from there byte for
 * byte, by this file and by the app, neither side renegotiating it. One rectangle, always at
 * x=0 y=0 w=0 h=0, encoding 0x44564831 big-endian, then four payload bytes:
 *
 *     u8 version   = 1
 *     u8 kind      0 = capabilities, 1 = heartbeat
 *     u8 value     kind 0: 0 = stream available (encoder up or expected)
 *                          1 = no encoder on this host, permanent -- stop waiting
 *                          2 = warming: asked for, not producing yet
 *                  kind 1: 0
 *     u8 reserved  = 0
 *
 * 0x44564831 is "DVH1" in ASCII: a vendor-style positive number in unassigned space, the same
 * pattern as VMware's 0x574d56xx block. It is NEVER sent to a client that did not advertise it,
 * which is what keeps TigerVNC and noVNC seeing exactly what they saw before.
 *
 * WHEN. A caps rect goes out as the first answer after the client advertises the pseudo-encoding,
 * and again whenever the value changes (`vnc_h264_rfb_set_caps`). A heartbeat rect goes out to a
 * DVH-aware client that is on the stream with an encoder running, when its request has been held
 * here with an empty queue for three seconds -- the cadence and the meaning DVH2's own heartbeat
 * had. Each of them is a complete single-rect FramebufferUpdate and consumes the one outstanding
 * request, which both reference clients re-issue immediately, so it costs one round trip.
 *
 * WHO WAKES IT. The client's output thread is asleep in clientOutput waiting for a modified
 * region that a still screen will never produce, so the three-second clock cannot live on it.
 * `vnc_h264_rfb_tick` is called by the Rust drain thread -- the thread that already wakes ten
 * times a second, and the thread that already reaches in here on every frame -- and it does
 * nothing but `wake_client` the ones whose held request is due. The rect itself is still written
 * from the hook, on the client's own thread, like every other byte this file sends.
 *
 * A client may advertise 0x44564831 without advertising 50. It gets its caps rect and stays on the
 * pixel path: it is not on the stream, it is not counted by `vnc_h264_rfb_client_count`, and it is
 * therefore not a reason to bring a codec up.
 *
 * -------------------------------------------------------------------------------------------
 * THE FOUR QUESTIONS THE CLIENTS HAD TO ANSWER, and their answers, since they decide the framing.
 * -------------------------------------------------------------------------------------------
 *
 * 1. FLAG BITS. `ResetContext = 0x1`, `ResetAllContexts = 0x2`, everything else unused and to be
 *    ignored by the client. Spec rfbproto.rst "Open H.264 Encoding" (bit table); TigerVNC
 *    H264Decoder.cxx:40-43 `enum rectFlags { resetContext = 0x1, resetAllContexts = 0x2 }`; noVNC
 *    h264.js:288-289 `const resetContextFlag = 1; const resetAllContextsFlag = 2;`. Three
 *    independent statements of the same two bits.
 *
 *    Does a server ever NEED ResetAllContexts? Only to abandon a geometry. Both clients key the
 *    decoder context by the rectangle -- TigerVNC by `Rect` equality (H264Decoder.cxx:61-67,
 *    `isEqualRect`), noVNC by the string `x,y,width,height` (h264.js:247-249) -- so a resized
 *    stream lands on a NEW context either way, and ResetContext alone would leave the old one
 *    behind (bounded at 64, then evicted by age). This file therefore sends ResetContext for a
 *    join or a resync, and ResetAllContexts once after a geometry change, which is the only case
 *    where a context this server will never address again exists.
 *
 * 2. ONE RECT PER UPDATE, AND ITS SIZE. Rectangle kinds may be mixed freely -- the client reads
 *    nRects headers and dispatches each on its own encoding number (TigerVNC CMsgReader.cxx:216-286,
 *    noVNC rfb.js:2655-2673) -- so a lone h264 rect in its own FramebufferUpdate is ordinary. It
 *    need not equal the framebuffer, but it MUST fit inside it: TigerVNC CMsgReader.cxx:551-559
 *    throws `protocol_error("Invalid rectangle received")` and drops the connection when
 *    `r.br.x > server.width() || r.br.y > server.height()`. The picture must also be at least as
 *    large as the rectangle -- the spec says the client crops an oversized frame to the rect, and
 *    TigerVNC's decoder silently drops the frame when it is smaller
 *    (H264LibavDecoderContext.cxx:186-188). The rect is therefore exactly the encoder's geometry,
 *    and an emission is skipped whenever that does not fit the client's current framebuffer.
 *
 * 3. RESIZE ON THE WIRE. DesktopSize first, THEN the reset-flagged rectangle -- forced by the
 *    bounds check in (2): a client learns its new framebuffer size only from the
 *    DesktopSize/ExtendedDesktopSize pseudo-rect, and an h264 rect in the new geometry that
 *    arrives first is out of bounds by definition. LibVNCServer sends that pseudo-rect from the
 *    top of rfbSendFramebufferUpdate (rfbserver.c:3187-3210), immediately AFTER displayHook, so
 *    the hook below declines to emit while `newFBSizePending` is set and lets it go out alone.
 *
 * 4. CONFIG-ONLY RECTS. Tolerated by both clients, but not used, because concatenation is what
 *    both are actually built for. A payload of nothing but SPS/PPS decodes to zero pictures on
 *    TigerVNC (H264LibavDecoderContext.cxx:174-176: `if (!frames_received) return;`) and makes
 *    noVNC log "Missing key frame" and keep waiting (h264.js:204-208) -- neither breaks. But
 *    noVNC's parser configures its decoder from the SPS it finds in the SAME payload it is about
 *    to decode (h264.js:198-215), and the spec says the data field is "one or more H.264 frames
 *    glued together in a row" to be "parsed as a regular H.264 stream". So a joining client is
 *    sent `SPS PPS IDR` as ONE rect, which is what an Annex-B decoder wants to see.
 *
 * -------------------------------------------------------------------------------------------
 * WHERE THE BYTES ARE WRITTEN FROM, AND WHY IT IS NOT THE DRAIN THREAD.
 * -------------------------------------------------------------------------------------------
 *
 * The isolation invariant: a stalled RFB h264 client may lose ITSELF and nothing else -- never the
 * drain thread, never another RFB client, never the producer. The socket belongs to LibVNCServer,
 * and LibVNCServer has already solved this: every client has its own output thread (main.c:459
 * clientOutput), and every write goes through rfbWriteExact, which is non-blocking plus poll()
 * with a 20-second ceiling (sockets.c:856-960, `rfbMaxClientWait`).
 *
 * So the frames are queued by the drain thread and WRITTEN BY THE CLIENT'S OWN OUTPUT THREAD, from
 * `displayHook` -- which runs at the top of rfbSendFramebufferUpdate (rfbserver.c:3180) with
 * `cl->sendMutex` held, i.e. exactly where and under exactly the lock LibVNCServer's own update
 * for that client would have been composed. A client that stops reading blocks that one thread and
 * is dropped by rfbWriteExact's own timeout. The drain thread never touches a socket at all: it
 * appends to a bounded per-client queue under this file's mutex and returns.
 *
 * That also settles the update-request question for free. RFB is request-driven, and neither
 * reference client keeps more than one request outstanding: TigerVNC re-requests from
 * framebufferUpdateEnd (CConnection.cxx:585-599 -> requestNewUpdate, :1066-1072) and noVNC does
 * the same after each completed update (rfb.js:2585-2588). Neither negotiates continuous updates
 * here, because this LibVNCServer does not implement them (no rfbEncodingContinuousUpdates
 * anywhere in src/). clientOutput will not call the hook at all unless the client has an
 * outstanding request (main.c:478-486, "always require a FB Update Request"), so the hook can only
 * ever run when the client is owed exactly one update -- and it emits exactly one.
 *
 * -------------------------------------------------------------------------------------------
 * SUPPRESSING THE PIXEL PATH, AND WHY IT MEANS TAKING THE REQUEST INTO CUSTODY.
 * -------------------------------------------------------------------------------------------
 *
 * An h264 client must not also be sent Tight/ZRLE rectangles: that is the same picture encoded
 * twice and sent twice. Nor cursor rectangles -- the encoded stream already has the pointer
 * composited into it (CursorOverlay, vnc_h264.rs).
 *
 * Emptying `cl->modifiedRegion` in the hook is not enough on its own. rfbSendFramebufferUpdate is
 * handed a COPY of that region, taken before the hook ran (main.c:507-513), and what it sends is
 * that copy intersected with `cl->requestedRegion` (rfbserver.c:3318-3330). The one lever inside
 * the hook that empties the intersection is therefore the requested region, and emptying it is
 * also what makes the whole update collapse into the "nothing to send" early return at
 * rfbserver.c:3319 -- which returns without sending a byte and without clearing anything else.
 *
 * But the client's request is a fact, not a nuisance: it is still owed an update. So it is taken
 * INTO CUSTODY rather than thrown away (`pending_request`), and put back the moment there is
 * something to answer it with (`wake_client`). Over the connection's life the arithmetic is
 * unchanged -- one FramebufferUpdate leaves for every FramebufferUpdateRequest that arrives -- and
 * in between, the fact that the client is waiting lives in this file instead of in a region.
 *
 * The wake is needed for the same reason the heartbeat is: the moment worth delivering is often
 * the LAST frame before the screen goes still, and by then nothing else will ever mark that
 * client's region again. `wake_client` supplies both halves of clientOutput's condition (a
 * modified region and a requested region) and signals `updateCond`. It is also the whole of what
 * the periodic tick does -- a held request that is due a heartbeat is a request whose client is
 * asleep for exactly this reason.
 *
 * -------------------------------------------------------------------------------------------
 * LOCK ORDER, once, so that nothing has to work it out twice.
 * -------------------------------------------------------------------------------------------
 *
 *     cl->sendMutex  ->  broker->lock  ->  cl->updateMutex
 *
 * The hook is entered with sendMutex held and takes the other two in that order. Every entry point
 * from the Rust side -- submit, tick, set_caps, reset -- enters at the broker lock and takes
 * updateMutex under it, from the drain thread or the producer's; neither of them holds anything
 * else. LibVNCServer itself only ever takes sendMutex before updateMutex (rfbserver.c:3280 under
 * main.c:512), which the chain above contains. Nothing in this file takes the broker lock while
 * holding updateMutex, and nothing holds the broker lock across a socket write -- the queue is
 * swapped out under it and written without it, which is what keeps a stalled client from reaching
 * anybody else.
 *
 * Waiting for updateMutex is bounded by arithmetic and not by a peer: LibVNCServer holds it for
 * region math only and releases it at rfbserver.c:3374, before the encode and before the first
 * byte goes out. That is what makes it safe for the producer's own thread to reach in here at all.
 *
 * Holding the broker lock is also what makes an `rfbClientPtr` safe to touch from the drain
 * thread: an entry is removed only by `client_gone`, which runs on the client's own thread and
 * has to take the same lock, and LibVNCServer frees the client struct only afterwards
 * (rfbserver.c:652 `cl->clientGoneHook(cl)`, then :690 `free(cl)`).
 */

#define _GNU_SOURCE
#include "vnc_h264_rfb.h"

#include <rfb/rfb.h>
#include <rfb/rfbregion.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* RFB encoding number for the Open H.264 encoding (rfbproto.rst, "Encodings" table: 50). Not in
 * the vendored rfbproto.h, which only knows the abandoned 0x48323634 proposal. */
#define VNC_RFB_ENCODING_H264 50

/* The DroidVM pseudo-encoding, "DVH1" in ASCII. plans/H264_SINGLE_PORT.md §1. */
#define VNC_RFB_ENCODING_DVH1 0x44564831

/* The four payload bytes of a DVH1 rect, §1. `version` is the first of them and is 1 for every
 * rect this file has ever sent. */
#define VNC_RFB_DVH1_VERSION        1
#define VNC_RFB_DVH1_KIND_CAPS      0
#define VNC_RFB_DVH1_KIND_HEARTBEAT 1

/* Bytes of a DVH1 emission: 4 of FramebufferUpdate header, 12 of rectangle header, 4 of payload.
 * Fixed for the life of version 1, which is why it is a constant and not a computation. */
#define VNC_RFB_DVH1_BYTES 20

/* How long a client's request may sit here, with nothing queued for it, before it is answered with
 * a heartbeat rect.
 *
 * Three seconds, and the number is inherited rather than chosen: it is DVH2's HEARTBEAT_INTERVAL,
 * and the reasoning carries over verbatim. A still screen and a dead stream are indistinguishable
 * to a receiver that is only ever written to, so silence has to be given a meaning, and the meaning
 * is only worth anything if both ends agree on the unit. The app's own read timeout (10s, §1) is
 * set against this one. */
#define VNC_RFB_H264_HEARTBEAT_MS 3000u

/* Flags in the rect payload. See question 1 at the top of this file for the three references that
 * agree on these two bits. */
#define VNC_RFB_H264_RESET_CONTEXT      0x1u
#define VNC_RFB_H264_RESET_ALL_CONTEXTS 0x2u

/* Bytes of message before the payload: 4 of FramebufferUpdate header, 12 of rectangle header, 8 of
 * the encoding's own length and flags. */
#define VNC_RFB_H264_HEADER_BYTES 24

/* How many clients one screen will serve the stream to.
 *
 * Fixed rather than grown for the same reason the frame bus's consumer list is: it is a handful of
 * entries for the lifetime of a server, and a client that does not fit is not turned away -- it is
 * simply never enrolled, and goes on being served pixels like any other viewer. */
#define VNC_RFB_H264_MAX_CLIENTS 8

/* How many bytes of undelivered stream one client may accumulate before its queue is thrown away.
 *
 * 2 MiB is several seconds of a 720p desktop stream at the bitrate vnc_h264.rs asks for (0.1 bits
 * per pixel per frame, ~350 KB/s) and about half a second at the 40 Mbit/s ceiling. The number
 * only has to be large enough that an ordinary slow moment does not cost a resync and small enough
 * that a client which has stopped reading cannot make the host hold a stream nobody is watching:
 * the queue is what fills while that client's output thread sits inside rfbWriteExact, and
 * rfbWriteExact gives it 20 seconds before dropping it. */
#define VNC_RFB_H264_QUEUE_CAP (2u * 1024u * 1024u)

/* First allocation of a client's queue. Grows geometrically to the cap and is never shrunk. */
#define VNC_RFB_H264_QUEUE_INITIAL (64u * 1024u)

/* One client being served the stream.
 *
 * `queue` is written by the drain thread and swapped into `outbound` under the broker lock by the
 * client's own thread, which then writes `outbound` to the socket with no lock held. Two buffers
 * rather than one copy, so that a large frame is never memcpy'd on the way out. */
struct h264_rfb_client {
    rfbClientPtr cl;
    /* Whatever the client's clientGoneHook was before this file chained itself in front of it.
     * Never NULL in practice -- LibVNCServer installs rfbDoNothingWithClient (rfbserver.c:378) --
     * but called through a NULL check anyway, because the cost of being wrong is a jump to zero
     * during teardown. */
    ClientGoneHookPtr prev_gone;

    /* This client asked for encoding 50 and is being served the stream. A slot exists for a client
     * that asked only for 0x44564831 too, and that one is NOT on the stream: it gets its caps rect
     * and goes on being served pixels. Everything below this line except the DVH1 fields is only
     * ever touched for a client with this set. */
    int on_stream;
    /* This client advertised 0x44564831 and may therefore be sent caps and heartbeat rects. A
     * server must never send one to a client that did not ask (§1), so this is the only gate. */
    int dvh_aware;
    /* A caps rect is owed: the client has just advertised the pseudo-encoding, or the value has
     * changed since `caps_told` was written. */
    int pending_caps;
    /* The value this client was last told, so that a caps change only reaches the clients it is
     * news to. */
    uint8_t caps_told;
    /* When `pending_request` last went from 0 to 1, on the monotonic clock.
     *
     * The EDGE, not every suppression: the tick's wake-up puts the request back and runs the hook
     * again, so restarting the clock on each pass would push the heartbeat one tick further away
     * every time it came due, forever. */
    uint64_t custody_since_ms;

    /* Waiting for the parameter sets and an IDR. A joining client is served NOTHING, not even a
     * delta it could not decode: the spec is explicit that a new context must start at an I-frame
     * ("The server must start sending data for the new context from I-frame"). */
    int joining;
    /* The next emission carries ResetContext: this client is starting a context, or restarting one
     * whose stream it lost the middle of. */
    int reset_context;
    /* The next emission carries ResetAllContexts INSTEAD, because the geometry changed and the
     * context this client holds for the old rectangle will never be addressed again -- the one
     * case where a server has anything to gain from the stronger flag. See question 1. */
    int reset_all;
    /* This client has asked for an update and has not been given one. The request itself has been
     * taken out of cl->requestedRegion; see the custody note at the top of the file. */
    int pending_request;
    /* How many times its queue has been thrown away for not being collected. Reported, never acted
     * on -- see the long note at the overflow itself for why this is not grounds for a close. */
    unsigned overflows;

    uint8_t* queue;
    size_t queue_len;
    size_t queue_cap;
    uint8_t* outbound;
    size_t outbound_len;
    size_t outbound_cap;

    uint64_t updates_sent;
    uint64_t bytes_sent;
    uint64_t frames_dropped;
    /* Caps and heartbeat rects together. Counted apart from `updates_sent` because they carry no
     * picture: "how much stream did this client actually get" has to keep meaning what it says on
     * a screen that has been still for an hour. */
    uint64_t dvh_rects;
};

struct vnc_h264_rfb {
    pthread_mutex_t lock;
    /* The server this belongs to, or NULL once it has gone. Only ever compared and logged: the
     * clients carry their own screen pointer. */
    vnc_server_t* server;
    int detached;

    struct h264_rfb_client clients[VNC_RFB_H264_MAX_CLIENTS];
    /* Slots in use, whatever the client asked for. Bounds the scans; not the answer to "does
     * anything want frames". */
    int slot_count;
    /* Slots whose client asked for encoding 50. This is `vnc_h264_rfb_client_count`, and through
     * it the sink's whole answer to whether the encoder should exist. */
    int stream_count;

    /* What a caps rect currently says, one of the VNC_H264_RFB_CAPS_* values.
     *
     * Starts at WARMING and not at AVAILABLE: a broker is built before anything has asked an
     * encoder to exist, so at enrolment time "not producing yet" is the true statement and
     * "available" is a guess. It is also what makes §1's second caps rect mean something -- there
     * is a transition to report when the encoder does come up. */
    uint8_t caps_value;

    /* Geometry of the stream, and 0x0 while there is not one. Every rectangle is emitted at this
     * size, and a submission that disagrees with it is discarded -- that is how a frame still
     * draining out of a codec that has just been replaced is kept away from a decoder that has
     * been told to expect the new size. */
    int width;
    int height;

    /* SPS/PPS as the encoder last emitted them, prepended to the IDR a joining client starts on.
     * The encoder produces them exactly once, in its first output buffer, which serves whoever was
     * already connected and nobody after that. */
    uint8_t* config;
    size_t config_len;
    size_t config_cap;

    uint64_t join_generation;
    uint64_t geometry_mismatches;
};

/* ------------------------------------------------------------------------------------------- */

static void put_u16(uint8_t* p, unsigned v) {
    p[0] = (uint8_t)((v >> 8) & 0xff);
    p[1] = (uint8_t)(v & 0xff);
}

static void put_u32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)((v >> 24) & 0xff);
    p[1] = (uint8_t)((v >> 16) & 0xff);
    p[2] = (uint8_t)((v >> 8) & 0xff);
    p[3] = (uint8_t)(v & 0xff);
}

/* Milliseconds on a clock that does not move when the wall clock is set. The heartbeat is a
 * duration, never a time of day, so CLOCK_MONOTONIC is the only correct source: a client mid-call
 * must not be given a three-second beat because NTP stepped the host. */
static uint64_t now_ms(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0)
        return 0;
    return (uint64_t)ts.tv_sec * 1000u + (uint64_t)ts.tv_nsec / 1000000u;
}

/* Appends to a growable buffer, refusing rather than growing past `limit`. Returns 0 if the bytes
 * did not fit or could not be allocated, in which case the buffer is left exactly as it was. */
static int buffer_append(uint8_t** buf, size_t* len, size_t* cap, const uint8_t* data, size_t n,
                         size_t limit) {
    if (n > limit || *len + n > limit)
        return 0;
    if (*len + n > *cap) {
        size_t want = *cap ? *cap : VNC_RFB_H264_QUEUE_INITIAL;
        uint8_t* grown;
        while (want < *len + n)
            want *= 2;
        if (want > limit)
            want = limit;
        grown = (uint8_t*)realloc(*buf, want);
        if (!grown)
            return 0;
        *buf = grown;
        *cap = want;
    }
    memcpy(*buf + *len, data, n);
    *len += n;
    return 1;
}

/* Whether an Annex-B run contains an IDR picture (NAL unit type 5).
 *
 * The codec's own sync-frame flag is the primary answer; this is the corroborating one, and it is
 * the same test noVNC's parser makes on the bytes it is handed (h264.js:53-59, `unitType == 5` ->
 * `{ slice: true, key: true }`). It exists because "this client may not be sent a delta yet" is a
 * correctness statement about the stream and the flag is a statement about the encoder, and only
 * one of those two can be checked here. Scanned only when some client is waiting for an IDR. */
static int annexb_has_idr(const uint8_t* data, uint32_t len) {
    uint32_t i;
    for (i = 0; i + 3 < len; i++) {
        uint32_t nal;
        if (data[i] != 0 || data[i + 1] != 0)
            continue;
        if (data[i + 2] == 1)
            nal = i + 3;
        else if (data[i + 2] == 0 && data[i + 3] == 1)
            nal = i + 4;
        else
            continue;
        if (nal < len && (data[nal] & 0x1f) == 5)
            return 1;
    }
    return 0;
}

/* The entry for `cl`, or NULL. Called with the lock held. A linear scan over at most eight slots:
 * a list would be a pointer to keep valid across a teardown that already has enough of those. */
static struct h264_rfb_client* find_client(vnc_h264_rfb_t* broker, rfbClientPtr cl) {
    int i;
    for (i = 0; i < VNC_RFB_H264_MAX_CLIENTS; i++)
        if (broker->clients[i].cl == cl)
            return &broker->clients[i];
    return NULL;
}

static void free_client_slot(struct h264_rfb_client* entry) {
    free(entry->queue);
    free(entry->outbound);
    memset(entry, 0, sizeof(*entry));
}

/* Makes the client's output thread come and collect what is queued for it.
 *
 * Called with the broker lock held, which is what makes `entry->cl` safe to dereference here.
 *
 * clientOutput waits until the client BOTH has an outstanding request and has something modified
 * (main.c:478-491), so both halves have to be supplied. The modified region is a single pixel --
 * it is a wake-up, not a description of anything, and the hook empties it again before
 * LibVNCServer can encode it. The requested region is the one this file took into custody, put
 * back exactly as wide as the client asked: it is being restored, not invented, because that
 * client really is waiting for an update it has not been given. */
static void wake_client(struct h264_rfb_client* entry) {
    rfbClientPtr cl = entry->cl;
    sraRegionPtr poke;

    LOCK(cl->updateMutex);
    if (entry->pending_request) {
        sraRegionPtr whole = sraRgnCreateRect(0, 0, cl->scaledScreen->width,
                                              cl->scaledScreen->height);
        if (whole) {
            sraRgnOr(cl->requestedRegion, whole);
            sraRgnDestroy(whole);
        }
    }
    poke = sraRgnCreateRect(0, 0, 1, 1);
    if (poke) {
        sraRgnOr(cl->modifiedRegion, poke);
        sraRgnDestroy(poke);
    }
    TSIGNAL(cl->updateCond);
    UNLOCK(cl->updateMutex);
}

/* Whether there is a stream to serve at all: an encoder has been built and has declared its
 * geometry. Called with the lock held. Also this file's answer to "is the encoder running", which
 * is one of §1's conditions on a heartbeat -- the broker learns of an encoder exactly once, when
 * `vnc_h264_rfb_reset` states its size. */
static int have_stream(const vnc_h264_rfb_t* broker) {
    return broker->width > 0 && broker->height > 0;
}

/* Whether this client's held request is due a heartbeat rect. Called with the lock held.
 *
 * §1, in full: DVH-aware, enrolled on the stream, an encoder running, and the request has been in
 * custody with an empty queue for three seconds. One predicate rather than two so that the tick
 * that WAKES a client and the hook that WRITES to it cannot disagree about who was due -- a
 * disagreement there is a wake that emits nothing, which is silent and looks like a still screen.
 */
static int heartbeat_due(const vnc_h264_rfb_t* broker, const struct h264_rfb_client* entry,
                         uint64_t now) {
    if (!entry->dvh_aware || !entry->on_stream || !entry->pending_request)
        return 0;
    if (entry->queue_len > 0 || !have_stream(broker))
        return 0;
    return now - entry->custody_since_ms >= VNC_RFB_H264_HEARTBEAT_MS;
}

/* ------------------------------------------------------------------------------------------- */
/* The protocol extension. Process-global, because LibVNCServer's extension list is
 * (main.c:75 rfbRegisterProtocolExtension), while brokers are per-screen -- so every callback
 * starts by routing from the client back to the broker that owns its screen.               */
/* ------------------------------------------------------------------------------------------- */

static void client_gone(rfbClientPtr cl);

/* The slot for `cl`, taking a free one if this client does not have one yet. Called with the lock
 * held; NULL when every slot is taken.
 *
 * A slot is claimed by whichever of the two encodings the client names first, and the client sends
 * both in one SetEncodings message -- 0x44564831 before 50, which is the order the app writes them
 * in -- so the second one finds the slot the first one made. */
static struct h264_rfb_client* claim_slot(vnc_h264_rfb_t* broker, rfbClientPtr cl) {
    struct h264_rfb_client* entry = find_client(broker, cl);
    int i;

    if (entry)
        return entry;
    for (i = 0; i < VNC_RFB_H264_MAX_CLIENTS; i++) {
        if (broker->clients[i].cl != NULL)
            continue;
        entry = &broker->clients[i];
        memset(entry, 0, sizeof(*entry));
        entry->cl = cl;
        /* Teardown is learned from clientGoneHook and not from the extension's own `close`
         * callback, because the vendored LibVNCServer never calls the latter: `close` appears in
         * the extension struct (rfb.h:181) and nowhere in src/. rfbClientConnectionGone does call
         * clientGoneHook (rfbserver.c:652), before it frees the client, and it is per-client so
         * chaining is enough. */
        entry->prev_gone = cl->clientGoneHook;
        cl->clientGoneHook = client_gone;
        broker->slot_count++;
        return entry;
    }
    return NULL;
}

/* Enrols a client that has just asked for encoding 50. Returns nothing: a client that cannot be
 * enrolled keeps the pixel path, which is a worse picture and not a broken one. */
static void enroll_stream(vnc_h264_rfb_t* broker, rfbClientPtr cl) {
    struct h264_rfb_client* entry;

    pthread_mutex_lock(&broker->lock);
    if (broker->detached) {
        pthread_mutex_unlock(&broker->lock);
        return;
    }
    entry = claim_slot(broker, cl);
    if (!entry) {
        pthread_mutex_unlock(&broker->lock);
        rfbLog("VNC h264-rfb: %s asked for encoding 50 but the stream is already serving %d "
               "clients; it stays on the pixel path\n",
               cl->host ? cl->host : "?", VNC_RFB_H264_MAX_CLIENTS);
        return;
    }
    if (entry->on_stream) {
        /* Already on the stream: SetEncodings is re-sent by both reference clients whenever
         * quality or compression settings change, and re-arming `joining` there would spend a full
         * IDR on a stream that is already running fine. */
        pthread_mutex_unlock(&broker->lock);
        return;
    }
    entry->on_stream = 1;
    entry->joining = 1;
    broker->stream_count++;
    /* A join is a demand for an IDR, and the Rust side is what can ask the encoder for one. */
    broker->join_generation++;
    pthread_mutex_unlock(&broker->lock);

    rfbLog("VNC h264-rfb: %s asked for encoding 50; waiting for a sync frame to start it on\n",
           cl->host ? cl->host : "?");
}

/* Records that a client speaks the DroidVM pseudo-encoding, and queues it the caps rect §1 owes it
 * as the first answer after the advertisement.
 *
 * Nothing is woken here: the client has just sent SetEncodings and its first
 * FramebufferUpdateRequest is right behind it, so the hook will run on its own. Waking a client
 * that has not asked for an update yet would be poking a region for no reason. */
static void enroll_dvh(vnc_h264_rfb_t* broker, rfbClientPtr cl) {
    struct h264_rfb_client* entry;
    uint8_t value;

    pthread_mutex_lock(&broker->lock);
    if (broker->detached) {
        pthread_mutex_unlock(&broker->lock);
        return;
    }
    entry = claim_slot(broker, cl);
    if (!entry) {
        pthread_mutex_unlock(&broker->lock);
        rfbLog("VNC h264-rfb: %s speaks the DroidVM pseudo-encoding but all %d slots are taken; it "
               "is told nothing and stays on the pixel path\n",
               cl->host ? cl->host : "?", VNC_RFB_H264_MAX_CLIENTS);
        return;
    }
    if (entry->dvh_aware) {
        pthread_mutex_unlock(&broker->lock);
        return;
    }
    entry->dvh_aware = 1;
    entry->pending_caps = 1;
    value = broker->caps_value;
    pthread_mutex_unlock(&broker->lock);

    rfbLog("VNC h264-rfb: %s speaks the DroidVM pseudo-encoding; capabilities %u on its next "
           "update\n",
           cl->host ? cl->host : "?", (unsigned)value);
}

static void unenroll(rfbClientPtr cl) {
    vnc_h264_rfb_t* broker = vnc_server_h264_rfb_for_client(cl);
    struct h264_rfb_client* entry;
    uint64_t updates = 0, bytes = 0, dropped = 0, dvh = 0;
    int was_enrolled;

    if (!broker)
        return;
    pthread_mutex_lock(&broker->lock);
    entry = find_client(broker, cl);
    was_enrolled = entry != NULL && entry->on_stream;
    if (entry) {
        updates = entry->updates_sent;
        bytes = entry->bytes_sent;
        dropped = entry->frames_dropped;
        dvh = entry->dvh_rects;
        if (entry->on_stream)
            broker->stream_count--;
        free_client_slot(entry);
        broker->slot_count--;
    }
    pthread_mutex_unlock(&broker->lock);

    if (was_enrolled)
        rfbLog("VNC h264-rfb: %s left the stream after %llu updates, %llu bytes, %llu frames "
               "dropped, %llu DroidVM rects\n",
               cl->host ? cl->host : "?", (unsigned long long)updates, (unsigned long long)bytes,
               (unsigned long long)dropped, (unsigned long long)dvh);
}

static void client_gone(rfbClientPtr cl) {
    ClientGoneHookPtr next = NULL;
    vnc_h264_rfb_t* broker = vnc_server_h264_rfb_for_client(cl);

    if (broker) {
        struct h264_rfb_client* entry;
        pthread_mutex_lock(&broker->lock);
        entry = find_client(broker, cl);
        next = entry ? entry->prev_gone : NULL;
        pthread_mutex_unlock(&broker->lock);
    }
    unenroll(cl);
    if (next)
        next(cl);
}

/* LibVNCServer offers this to every extension for every encoding number it does not handle itself
 * (rfbserver.c:2562-2600), including -- once this extension is enabled for a client -- numbers
 * that have nothing to do with it. */
static rfbBool enable_pseudo_encoding(rfbClientPtr cl, void** data, int encodingNumber) {
    vnc_h264_rfb_t* broker;

    (void)data; /* No per-client extension data: the broker owns that, keyed by `cl`. */
    if (encodingNumber != VNC_RFB_ENCODING_H264 && encodingNumber != VNC_RFB_ENCODING_DVH1)
        return FALSE;

    broker = vnc_server_h264_rfb_for_client(cl);
    if (broker) {
        if (encodingNumber == VNC_RFB_ENCODING_H264)
            enroll_stream(broker, cl);
        else
            enroll_dvh(broker, cl);
    }
    /* TRUE even with no broker -- a screen whose transport ceiling forbids an encoder. The client
     * is then served Tight/ZRLE exactly as before, which is what "the server ignored encoding 50"
     * looks like on the wire; answering FALSE would produce the same behaviour plus a log line
     * accusing this extension of pretending (rfbserver.c:2585). A DVH-aware client on such a
     * screen is told nothing at all, which §3.4 of the plan is written for: no caps rect within a
     * few seconds means the same thing as caps value 1. */
    return TRUE;
}

/* Never called by the vendored LibVNCServer -- see the note in `claim_slot`. Provided so that a
 * future version which does call it finds the teardown already written, and idempotent against the
 * clientGoneHook that does the work today. */
static void extension_close(rfbClientPtr cl, void* data) {
    (void)data;
    unenroll(cl);
}

/* 0-terminated, which is LibVNCServer's own convention for this list (rfbserver.c:2578,
 * `while(encs && *encs!=0)`). Encoding 0 is Raw and is handled natively, so it never reaches an
 * extension and cannot be confused with the terminator. */
static int h264_pseudo_encodings[] = { VNC_RFB_ENCODING_H264, VNC_RFB_ENCODING_DVH1, 0 };

/* Non-const and static for its whole life: rfbRegisterProtocolExtension links the object itself
 * into a global list through its own `next` field (main.c:75-90). */
static rfbProtocolExtension h264_extension = {
    NULL,                    /* newClient: nothing to do until a client names one of the two */
    NULL,                    /* init */
    h264_pseudo_encodings,
    enable_pseudo_encoding,
    NULL,                    /* handleMessage: this encoding has no client-to-server message */
    extension_close,
    NULL,                    /* usage */
    NULL,                    /* processArgument */
    NULL,                    /* next: owned by LibVNCServer once registered */
};

static pthread_once_t h264_extension_once = PTHREAD_ONCE_INIT;

static void register_extension(void) {
    rfbRegisterProtocolExtension(&h264_extension);
}

/* ------------------------------------------------------------------------------------------- */
/* The hook: everything this file sends leaves from here.                                        */
/* ------------------------------------------------------------------------------------------- */

/* One DroidVM rectangle, written from the client's own output thread with `cl->sendMutex` held --
 * the same place, and for the same reasons, as the H.264 ones.
 *
 * The bytes are plans/H264_SINGLE_PORT.md §1 and nothing else: a FramebufferUpdate carrying one
 * rectangle at x=0 y=0 w=0 h=0, encoding 0x44564831 big-endian, then the four payload bytes
 * version / kind / value / reserved. Called with no lock held. */
static void send_dvh1(rfbClientPtr cl, uint8_t kind, uint8_t value) {
    uint8_t msg[VNC_RFB_DVH1_BYTES];

    msg[0] = rfbFramebufferUpdate;
    msg[1] = 0;
    put_u16(msg + 2, 1); /* one rectangle */
    put_u16(msg + 4, 0); /* x */
    put_u16(msg + 6, 0); /* y */
    put_u16(msg + 8, 0); /* w */
    put_u16(msg + 10, 0); /* h */
    put_u32(msg + 12, (uint32_t)VNC_RFB_ENCODING_DVH1);
    msg[16] = VNC_RFB_DVH1_VERSION;
    msg[17] = kind;
    msg[18] = value;
    msg[19] = 0; /* reserved */

    /* Anything LibVNCServer had buffered for this client goes first, or these bytes would jump the
     * queue. In practice ublen is zero here -- every sender flushes before releasing sendMutex. */
    if (cl->ublen > 0 && !rfbSendUpdateBuf(cl))
        return;
    if (rfbWriteExact(cl, (char*)msg, (int)sizeof(msg)) < 0) {
        rfbLogPerror("VNC h264-rfb: DroidVM rect write");
        rfbCloseClient(cl);
    }
}

int vnc_h264_rfb_display_hook(vnc_h264_rfb_t* broker, struct _rfbClientRec* client) {
    rfbClientPtr cl = (rfbClientPtr)client;
    struct h264_rfb_client* entry;
    uint8_t header[VNC_RFB_H264_HEADER_BYTES];
    const uint8_t* payload;
    size_t payload_len;
    uint32_t flags;
    int width, height;
    int suppress;
    uint64_t now;

    if (!broker || !cl)
        return 0;

    pthread_mutex_lock(&broker->lock);
    entry = find_client(broker, cl);
    if (!entry || broker->detached) {
        pthread_mutex_unlock(&broker->lock);
        return 0;
    }
    /* Taking the pixel path away is only right while there is a stream to put in its place. A
     * client that asked for 50 on a device whose encoder never came up lands here on every update
     * for the life of the connection and is served pixels -- the one fallback worth having,
     * because the alternative is a viewer showing nothing at all -- and so does a client that
     * advertised the DroidVM pseudo-encoding and never asked for the stream at all. Its damage has
     * to survive the caps rect that tells it so, which is why this is a separate question from
     * "does this file owe it anything". */
    suppress = entry->on_stream && have_stream(broker);
    if (!suppress && !entry->pending_caps) {
        pthread_mutex_unlock(&broker->lock);
        return 0;
    }

    /* The size change has to reach the client first; see question 3. Nothing is suppressed on this
     * path either: rfbSendFramebufferUpdate sends the DesktopSize rect and returns without looking
     * at the regions (rfbserver.c:3187-3210). A caps rect waits its turn behind it, because the
     * request it would have consumed is being spent on the resize. */
    if (cl->useNewFBSize && cl->newFBSizePending) {
        pthread_mutex_unlock(&broker->lock);
        return suppress;
    }

    /* SUPPRESSION AND CUSTODY. Everything LibVNCServer would have composed for this client is
     * emptied here, and the client's outstanding request is taken into custody so that the update
     * collapses into the early return at rfbserver.c:3319 rather than being answered with pixels.
     *
     * cl->enableCursorShapeUpdates is deliberately NOT touched: the bridge's own displayHook has
     * just forced it TRUE, and that is what stops LibVNCServer compositing a pointer into the
     * shared framebuffer (rfbserver.c:3376) that another client's thread is encoding. What is
     * cleared instead is every reason it might have to SEND a cursor rectangle -- the stream this
     * client is being served already has the pointer drawn into it.
     *
     * The custody timestamp is written on the 0 -> 1 edge alone. The tick that answers an idle
     * client wakes it, which runs this block again, so restarting the clock here would push the
     * heartbeat one tick further into the future every time it came due -- forever. */
    now = now_ms();
    LOCK(cl->updateMutex);
    if (!sraRgnEmpty(cl->requestedRegion)) {
        if (!entry->pending_request) {
            entry->pending_request = 1;
            entry->custody_since_ms = now;
        }
        sraRgnMakeEmpty(cl->requestedRegion);
    }
    if (suppress) {
        sraRgnMakeEmpty(cl->modifiedRegion);
        sraRgnMakeEmpty(cl->copyRegion);
        cl->copyDX = 0;
        cl->copyDY = 0;
    }
    UNLOCK(cl->updateMutex);
    if (suppress) {
        cl->cursorWasChanged = FALSE;
        cl->cursorWasMoved = FALSE;
        cl->enableCursorPosUpdates = FALSE;
    }

    if (!entry->pending_request) {
        pthread_mutex_unlock(&broker->lock);
        return suppress;
    }

    /* CAPS FIRST, §1: the rect is the first answer after the client advertises the pseudo-encoding,
     * and the answer again whenever the value changes. It goes ahead of a queued picture because a
     * client that has just learned there will never be one has nothing to do with the picture, and
     * one that has just learned there will be loses a single frame's latency for the news. */
    if (entry->pending_caps) {
        uint8_t value = broker->caps_value;
        entry->pending_caps = 0;
        entry->caps_told = value;
        entry->pending_request = 0;
        entry->dvh_rects++;
        pthread_mutex_unlock(&broker->lock);
        send_dvh1(cl, VNC_RFB_DVH1_KIND_CAPS, value);
        return 1;
    }

    if (entry->queue_len == 0) {
        /* Nothing queued. If this client's request has been held here long enough, the silence is
         * itself the thing worth saying -- §1's heartbeat, on DVH2's three-second unit. */
        if (heartbeat_due(broker, entry, now)) {
            entry->pending_request = 0;
            entry->dvh_rects++;
            pthread_mutex_unlock(&broker->lock);
            send_dvh1(cl, VNC_RFB_DVH1_KIND_HEARTBEAT, 0);
            return 1;
        }
        pthread_mutex_unlock(&broker->lock);
        return suppress;
    }
    /* Out of bounds for this client's framebuffer, so unsendable: TigerVNC drops the connection
     * over it (CMsgReader.cxx:551-559). Reachable when a client that cannot be told about a resize
     * -- no DesktopSize in its encoding list -- is connected across one. Scaled screens are
     * declined for the same reason: the rectangle would have to describe the scaled geometry and
     * the picture inside it does not. */
    if (cl->screen != cl->scaledScreen || broker->width > cl->scaledScreen->width ||
        broker->height > cl->scaledScreen->height) {
        pthread_mutex_unlock(&broker->lock);
        return 1;
    }

    {
        uint8_t* swap_buf = entry->outbound;
        size_t swap_cap = entry->outbound_cap;
        entry->outbound = entry->queue;
        entry->outbound_len = entry->queue_len;
        entry->outbound_cap = entry->queue_cap;
        entry->queue = swap_buf;
        entry->queue_cap = swap_cap;
        entry->queue_len = 0;
    }
    /* Never both: ResetAllContexts already deletes the context this rectangle would name, and
     * TigerVNC reads the two as alternatives -- ResetAllContexts is handled first and clears the
     * other bit before it can be looked at (H264Decoder.cxx:106-120). */
    flags = entry->reset_all ? VNC_RFB_H264_RESET_ALL_CONTEXTS
                             : (entry->reset_context ? VNC_RFB_H264_RESET_CONTEXT : 0);
    entry->reset_all = 0;
    entry->reset_context = 0;
    entry->pending_request = 0;
    entry->overflows = 0;
    payload = entry->outbound;
    payload_len = entry->outbound_len;
    width = broker->width;
    height = broker->height;
    entry->updates_sent++;
    entry->bytes_sent += payload_len;
    pthread_mutex_unlock(&broker->lock);

    header[0] = rfbFramebufferUpdate;
    header[1] = 0;
    put_u16(header + 2, 1);
    put_u16(header + 4, 0);
    put_u16(header + 6, 0);
    put_u16(header + 8, (unsigned)width);
    put_u16(header + 10, (unsigned)height);
    put_u32(header + 12, (uint32_t)VNC_RFB_ENCODING_H264);
    put_u32(header + 16, (uint32_t)payload_len);
    put_u32(header + 20, flags);

    /* Anything LibVNCServer had buffered for this client goes first, or these bytes would jump the
     * queue. In practice ublen is zero here -- every sender flushes before releasing sendMutex --
     * and the check costs a load. */
    if (cl->ublen > 0 && !rfbSendUpdateBuf(cl))
        return 1;

    if (rfbWriteExact(cl, (char*)header, (int)sizeof(header)) < 0 ||
        (payload_len > 0 && rfbWriteExact(cl, (const char*)payload, (int)payload_len) < 0)) {
        /* Either the client stopped reading for twenty seconds (sockets.c:951-956, ETIMEDOUT) or
         * it is gone. Same answer as LibVNCServer's own update path takes on a failed write, from
         * this same thread: drop it. Half a rectangle has been written, so there is nothing to
         * retry -- the message framing is broken from here on. */
        rfbLogPerror("VNC h264-rfb: write");
        rfbCloseClient(cl);
        return 1;
    }
    return 1;
}

/* ------------------------------------------------------------------------------------------- */
/* Entry points from the Rust side.                                                              */
/* ------------------------------------------------------------------------------------------- */

void vnc_h264_rfb_submit(vnc_h264_rfb_t* broker, const uint8_t* data, uint32_t len, int is_config,
                         int is_idr, int width, int height) {
    int i;
    int idr;
    int anyone_joining = 0;

    if (!broker || !data || len == 0)
        return;

    pthread_mutex_lock(&broker->lock);
    if (broker->detached) {
        pthread_mutex_unlock(&broker->lock);
        return;
    }
    if (width != broker->width || height != broker->height) {
        /* A frame from an encoder that has already been replaced, still draining out of the old
         * codec. Sending it would put a picture of the old size behind a rectangle header of the
         * new one, which is exactly what the client's bounds check and its context key exist to
         * catch. The geometry travels with the frame for this one reason. */
        broker->geometry_mismatches++;
        pthread_mutex_unlock(&broker->lock);
        return;
    }

    if (is_config) {
        /* Replaced rather than appended: these are the parameter sets of the stream as it is now,
         * and the only reason to keep them is to put them in front of the IDR the next client
         * joins on. `len` as the limit, because this buffer holds exactly one copy. */
        broker->config_len = 0;
        if (!buffer_append(&broker->config, &broker->config_len, &broker->config_cap, data, len,
                           len))
            broker->config_len = 0;
        /* A codec-config buffer carries no coded picture, so there is nothing here for a client
         * that is already decoding -- it was given these bytes when it joined. */
        if (!is_idr) {
            pthread_mutex_unlock(&broker->lock);
            return;
        }
    }

    if (broker->stream_count == 0) {
        pthread_mutex_unlock(&broker->lock);
        return;
    }

    for (i = 0; i < VNC_RFB_H264_MAX_CLIENTS; i++)
        if (broker->clients[i].cl && broker->clients[i].on_stream && broker->clients[i].joining)
            anyone_joining = 1;
    idr = is_idr || (anyone_joining && annexb_has_idr(data, len));

    for (i = 0; i < VNC_RFB_H264_MAX_CLIENTS; i++) {
        struct h264_rfb_client* entry = &broker->clients[i];
        /* A slot claimed by the pseudo-encoding alone holds a client that never asked for a
         * picture; queueing one for it would be bytes nobody can read. */
        if (!entry->cl || !entry->on_stream)
            continue;

        if (entry->joining) {
            if (!idr) {
                /* Not a place a decoder can start. Dropped rather than queued: a client that is
                 * shown the middle of a stream shows garbage, and the sync frame this join has
                 * already asked for is on its way. */
                entry->frames_dropped++;
                continue;
            }
            entry->queue_len = 0;
            if (broker->config_len > 0 &&
                !buffer_append(&entry->queue, &entry->queue_len, &entry->queue_cap, broker->config,
                               broker->config_len, VNC_RFB_H264_QUEUE_CAP)) {
                entry->frames_dropped++;
                continue;
            }
            if (!buffer_append(&entry->queue, &entry->queue_len, &entry->queue_cap, data, len,
                               VNC_RFB_H264_QUEUE_CAP)) {
                entry->queue_len = 0;
                entry->frames_dropped++;
                continue;
            }
            /* `SPS PPS IDR` in one rectangle, which is question 4's answer and what the side
             * channel hands its own receiver. */
            entry->joining = 0;
            if (!entry->reset_all)
                entry->reset_context = 1;
        } else if (!buffer_append(&entry->queue, &entry->queue_len, &entry->queue_cap, data, len,
                                  VNC_RFB_H264_QUEUE_CAP)) {
            /* This client is not collecting what has been queued for it. The whole queue goes, not
             * just this frame: a stream with a hole in it is not a stream, and the honest recovery
             * is to put the client back to joining and start it again on an IDR.
             *
             * NOT a reason to close it, and not a reason to ask for a sync frame either. Both were
             * tried and both are wrong. There are two ways to reach this line -- a client whose
             * socket will not take bytes, and a client that has simply stopped sending
             * FramebufferUpdateRequests -- and from here they look identical, because a client
             * blocked inside a write has no request outstanding either. The second kind is not
             * broken: RFB lets a client ask for nothing for as long as it likes, and closing it
             * would be a bug. The first kind does not need us -- rfbWriteExact gives it twenty
             * seconds and then drops it (sockets.c:951-956) -- and its own output thread is where
             * that happens. Asking for a sync frame here would let either kind spend a full IDR of
             * everybody's bandwidth every few seconds; the encoder's own IDR interval restarts it
             * within a couple of seconds anyway, which is what that backstop is for. */
            entry->queue_len = 0;
            entry->joining = 1;
            entry->reset_context = 1;
            entry->frames_dropped++;
            entry->overflows++;
            rfbLog("VNC h264-rfb: %s is not collecting the stream (overflow %u); it restarts on "
                   "the next sync frame if it comes back\n",
                   entry->cl->host ? entry->cl->host : "?", entry->overflows);
            continue;
        }

        wake_client(entry);
    }
    pthread_mutex_unlock(&broker->lock);
}

void vnc_h264_rfb_reset(vnc_h264_rfb_t* broker, int width, int height) {
    int i;
    int had_stream;
    int affected = 0;

    if (!broker || width <= 0 || height <= 0)
        return;

    pthread_mutex_lock(&broker->lock);
    if (broker->detached || (broker->width == width && broker->height == height)) {
        pthread_mutex_unlock(&broker->lock);
        return;
    }
    had_stream = broker->width > 0 && broker->height > 0;
    broker->width = width;
    broker->height = height;
    /* The parameter sets describe the stream that has just ended. Keeping them would hand the next
     * joining client an SPS for the wrong geometry, in front of an IDR for the right one. */
    broker->config_len = 0;

    for (i = 0; i < VNC_RFB_H264_MAX_CLIENTS; i++) {
        struct h264_rfb_client* entry = &broker->clients[i];
        if (!entry->cl || !entry->on_stream)
            continue;
        entry->queue_len = 0;
        entry->joining = 1;
        entry->overflows = 0;
        if (had_stream)
            entry->reset_all = 1;
        else
            entry->reset_context = 1;
        affected++;
    }
    if (affected > 0)
        broker->join_generation++;
    pthread_mutex_unlock(&broker->lock);

    rfbLog("VNC h264-rfb: the stream is %dx%d; %d client(s) restart on the next sync frame\n",
           width, height, affected);
}

int vnc_h264_rfb_client_count(vnc_h264_rfb_t* broker) {
    int count;
    if (!broker)
        return 0;
    pthread_mutex_lock(&broker->lock);
    /* Clients on the stream, not slots. A client that advertised only the pseudo-encoding is not a
     * reason to build a codec: it asked what this host can do, not for a picture. */
    count = broker->stream_count;
    pthread_mutex_unlock(&broker->lock);
    return count;
}

void vnc_h264_rfb_set_caps(vnc_h264_rfb_t* broker, int value) {
    int i;
    int told = 0;
    uint8_t v;

    if (!broker || value < 0 || value > 0xff)
        return;
    v = (uint8_t)value;

    pthread_mutex_lock(&broker->lock);
    if (broker->detached || broker->caps_value == v) {
        pthread_mutex_unlock(&broker->lock);
        return;
    }
    broker->caps_value = v;
    for (i = 0; i < VNC_RFB_H264_MAX_CLIENTS; i++) {
        struct h264_rfb_client* entry = &broker->clients[i];
        if (!entry->cl || !entry->dvh_aware)
            continue;
        if (!entry->pending_caps && entry->caps_told == v)
            continue;
        entry->pending_caps = 1;
        told++;
        /* Woken unconditionally, and not only when the request is already in custody: the client
         * whose news this most is -- one waiting on a screen that has stopped moving -- is asleep
         * in clientOutput with a request LibVNCServer can see and this file has not taken, and the
         * poke at its modified region is the only thing that will call the hook again. */
        wake_client(entry);
    }
    pthread_mutex_unlock(&broker->lock);

    rfbLog("VNC h264-rfb: h264 capabilities are now %u; %d client(s) to be told\n", (unsigned)v,
           told);
}

void vnc_h264_rfb_tick(vnc_h264_rfb_t* broker) {
    int i;
    uint64_t now;

    if (!broker)
        return;
    pthread_mutex_lock(&broker->lock);
    if (broker->detached || broker->stream_count == 0) {
        pthread_mutex_unlock(&broker->lock);
        return;
    }
    now = now_ms();
    for (i = 0; i < VNC_RFB_H264_MAX_CLIENTS; i++) {
        struct h264_rfb_client* entry = &broker->clients[i];
        if (!entry->cl || !entry->dvh_aware || !entry->on_stream)
            continue;

        /* Before the custody rule below, and not folded into it: `set_caps` wakes a client and
         * clientOutput then sleeps `deferUpdateTime` before it reaches the hook. A tick landing in
         * that window would empty the requested region the wake had just supplied, and clientOutput
         * would find it empty and go back to sleep with the caps rect still owed -- delivered on
         * the next heartbeat instead of now. One more wake costs nothing and closes it. */
        if (entry->pending_caps) {
            wake_client(entry);
            continue;
        }

        if (entry->queue_len > 0 || !have_stream(broker))
            continue;

        if (!entry->pending_request) {
            /* Custody is taken HERE for an idle client, and that is the whole reason this entry
             * point exists rather than a timestamp read from the hook.
             *
             * After the last frame goes out, the client re-requests; clientOutput then finds a
             * requested region and an empty modified one and goes back to sleep WITHOUT calling
             * the hook (main.c:478-491). So on a screen that has stopped moving the hook never
             * runs again, nothing is ever in custody, and a clock kept there would never start.
             * Emptying the requested region here is the same operation the hook performs, under
             * the same mutex, on a thread that is asleep. */
            rfbClientPtr cl = entry->cl;
            LOCK(cl->updateMutex);
            if (!sraRgnEmpty(cl->requestedRegion)) {
                entry->pending_request = 1;
                entry->custody_since_ms = now;
                sraRgnMakeEmpty(cl->requestedRegion);
            }
            UNLOCK(cl->updateMutex);
            continue;
        }
        /* Due. Nothing is written from this thread: the wake supplies both halves of
         * clientOutput's condition, and the rect leaves from the hook on the client's own thread
         * under its own sendMutex, like every other byte this file sends. */
        if (heartbeat_due(broker, entry, now))
            wake_client(entry);
    }
    pthread_mutex_unlock(&broker->lock);
}

uint64_t vnc_h264_rfb_join_generation(vnc_h264_rfb_t* broker) {
    uint64_t generation;
    if (!broker)
        return 0;
    pthread_mutex_lock(&broker->lock);
    generation = broker->join_generation;
    pthread_mutex_unlock(&broker->lock);
    return generation;
}

void vnc_h264_rfb_detach(vnc_h264_rfb_t* broker) {
    if (!broker)
        return;
    pthread_mutex_lock(&broker->lock);
    broker->detached = 1;
    broker->server = NULL;
    pthread_mutex_unlock(&broker->lock);
}

vnc_h264_rfb_t* vnc_h264_rfb_create(vnc_server_t* server) {
    vnc_h264_rfb_t* broker;

    if (!server)
        return NULL;
    broker = (vnc_h264_rfb_t*)calloc(1, sizeof(*broker));
    if (!broker)
        return NULL;
    if (pthread_mutex_init(&broker->lock, NULL) != 0) {
        free(broker);
        return NULL;
    }
    broker->server = server;
    /* Not the zero `calloc` left behind: zero is "the stream is available" on the wire (§1) and
     * nothing has asked for an encoder yet. Warming is the true statement here, and it is also
     * what leaves a transition for `vnc_h264_rfb_set_caps` to report when one does come up. */
    broker->caps_value = VNC_H264_RFB_CAPS_WARMING;
    pthread_once(&h264_extension_once, register_extension);
    vnc_server_set_h264_rfb(server, broker);
    return broker;
}

void vnc_h264_rfb_destroy(vnc_h264_rfb_t* broker) {
    int i;
    if (!broker)
        return;
    /* A server that has not been destroyed yet is still holding this pointer -- the case where the
     * consumer failed to start after the broker was built, and the case where a display is torn
     * down without its server going first. Taking it back is what keeps the field's promise that
     * it is either NULL or live. */
    if (broker->server)
        vnc_server_set_h264_rfb(broker->server, NULL);
    for (i = 0; i < VNC_RFB_H264_MAX_CLIENTS; i++)
        free_client_slot(&broker->clients[i]);
    free(broker->config);
    pthread_mutex_destroy(&broker->lock);
    free(broker);
}
