// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

//! The WebRTC sink's network half: the peers, the HTTP endpoint that signals them, and the
//! compressed stream's audience.
//!
//! **Signalling.** One plain-HTTP listener per screen serves the viewer page and two endpoints:
//! `GET /offer` creates a peer connection and answers with its SDP offer (the server offers, the
//! browser answers -- the direction P0 proved against a browser), `POST /answer?peer=N` completes
//! it. An optional token gates both. There is nothing else on the port: no WebSocket, no trickle
//! ICE -- the offer goes out after gathering has finished, which on a LAN is immediate.
//!
//! **Media.** Each peer gets one send-only H.264 track behind libdatachannel's packetizer, SR
//! reporter, NACK responder and PLI handler, and a DataChannel `input` the page writes pointer and
//! key events into. Units come from the shared `EncoderPipeline`; a peer is given nothing until a
//! sync frame, and every sync frame goes out with the cached parameter sets in front of it,
//! because the encoder emits those exactly once and a peer that joins later would otherwise never
//! be able to decode (win3d/docs/webrtc, P0-2).
//!
//! **Threads.** libdatachannel calls back on its own threads; the drain thread calls `on_unit`;
//! the HTTP thread creates and reaps peers. Nothing here ever blocks the producer: the guest's
//! flush path only reaches `wants_frames`, which takes the peer lock for a scan.

use std::collections::BTreeMap;
use std::collections::VecDeque;
use std::ffi::c_char;
use std::ffi::c_int;
use std::ffi::c_uint;
use std::ffi::c_void;
use std::ffi::CStr;
use std::ffi::CString;
use std::io::Read;
use std::io::Write;
use std::net::TcpListener;
use std::net::TcpStream;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::AtomicU32;
use std::sync::atomic::AtomicU64;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::sync::Condvar;
use std::sync::Mutex;
use std::sync::Once;
use std::thread;
use std::time::Duration;
use std::time::Instant;

use base::Event;

use crate::video_encoder::EncodedAudience;
use crate::video_encoder::EncoderConfig;
use crate::video_encoder::VideoCodec;
use crate::video_encoder::BUFFER_FLAG_CODEC_CONFIG;
use crate::video_encoder::BUFFER_FLAG_SYNC_FRAME;
use crate::webrtc_rtc::*;

/// How many viewers one screen serves at once. Each one is a full DTLS/SRTP session and a NACK
/// history, and the encoder output is shared, so this is about the phone's uplink, not its CPU.
const MAX_PEERS: usize = 4;

/// How long `GET /offer` waits for ICE gathering before answering with what it has.
const GATHER_TIMEOUT: Duration = Duration::from_secs(5);

/// The fewest milliseconds between two sync frames asked for on behalf of PLIs. A browser that
/// cannot decode sends one every few tens of milliseconds, and each answer is a full IDR.
const PLI_MIN_INTERVAL: Duration = Duration::from_millis(500);

/// RTP payload types offered. H.264 is Constrained Baseline, which every browser decodes; H.265
/// (Main) is offered first when the binding allows it, and taken by a browser that can decode it in
/// hardware (Chrome 136+, Safari 18+; Firefox has none). The encoder is configured to match either
/// (see `DisplayWebrtc::encoder_config`).
const H264_PAYLOAD_TYPE: u8 = 96;
const H265_PAYLOAD_TYPE: u8 = 97;

/// The playout-delay RTP header extension (webrtc.org experiments/rtp-hdrext/playout-delay), offered
/// at this id. Sent as min = max = 0, it tells the browser to show each frame as soon as it is
/// decoded instead of holding it in a jitter buffer (measured 25 ms in Chrome on a LAN): a remote
/// screen wants the least delay, and pays for it with less smoothing when the network jitters.
const PLAYOUT_DELAY_EXT_ID: u8 = 5;
const PLAYOUT_DELAY_URI: &str = "http://www.webrtc.org/experiments/rtp-hdrext/playout-delay";

/// The absolute send time RTP header extension, offered at this id. Without it (and without
/// transport-cc) Chrome's REMB estimator reads delay off the RTP timestamps, which here are capture
/// times: the encoder's 19–31 ms of varying latency then looks like a queue building on the network,
/// and the estimate ratchets down to a few hundred kbit/s on a clean LAN. Stamped as packets leave,
/// it lets the estimator see only the network.
const ABS_SEND_TIME_EXT_ID: u8 = 3;
const ABS_SEND_TIME_URI: &str = "http://www.webrtc.org/experiments/rtp-hdrext/abs-send-time";

/// The header extension ids an answer kept; `None` for one the viewer declined.
#[derive(Clone, Copy, Debug, Default)]
struct HeaderExtensions {
    playout_delay: Option<u8>,
    abs_send_time: Option<u8>,
}

/// What a viewer asked for on `/offer?codec=`: `auto` (the default) offers every codec the binding
/// allows; `h264` or `h265` offers that one alone. Asking for H.265 where the binding allows only
/// H.264 gets H.264. One encoder serves the screen, so a viewer that insists on H.265 while another
/// needs H.264 is the one dropped (`reconcile_codec`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum CodecChoice {
    Auto,
    H264,
    H265,
}

impl CodecChoice {
    fn parse(v: Option<&str>) -> CodecChoice {
        match v.map(str::to_ascii_lowercase).as_deref() {
            Some("h264") => CodecChoice::H264,
            Some("h265") | Some("hevc") => CodecChoice::H265,
            _ => CodecChoice::Auto,
        }
    }
}

/// The levels offered, chosen from the screen size at offer time: the encoder picks its level from
/// the size (a 1080p stream measured H.264 4.0 / H.265 level-id 120), and advertising less than it
/// produces would be a promise the SPS breaks. Up to 1080p: H.264 4.2, H.265 4.1 (level-id 123);
/// above: 5.1 for both.
fn codec_fmtp(codec: VideoCodec, width: u32, height: u32) -> &'static str {
    let large = (width as u64) * (height as u64) > 1920 * 1088;
    match (codec, large) {
        (VideoCodec::H264, false) => {
            "profile-level-id=42e02a;packetization-mode=1;level-asymmetry-allowed=1"
        }
        (VideoCodec::H264, true) => {
            "profile-level-id=42e033;packetization-mode=1;level-asymmetry-allowed=1"
        }
        (VideoCodec::H265, false) => "profile-id=1;tier-flag=0;level-id=123;tx-mode=SRST",
        (VideoCodec::H265, true) => "profile-id=1;tier-flag=0;level-id=153;tx-mode=SRST",
    }
}

/// Largest NAL fragment per RTP packet, either codec. libdatachannel's default (1220) assumes a 1280-byte path
/// with no header extensions; over a 1280-MTU tunnel (Tailscale) the MID extension and the GCM tag
/// push packets past it and the kernel refuses them (EMSGSIZE, "datagram is too large").
const MAX_FRAGMENT: u16 = 1100;

/// The viewer page, served at `/`.
const PAGE: &str = include_str!("webrtc_page.html");

/// One event from a viewer's `input` DataChannel, decoded. Coordinates are 0..=65535 across the
/// video picture as the viewer sees it; the sink scales them to the tablet's range.
#[derive(Clone, Copy, Debug)]
pub(crate) enum WebrtcInput {
    Key { code: u16, down: bool },
    Pointer { buttons: u8, x: u16, y: u16 },
    Wheel { steps: i16 },
}

impl WebrtcInput {
    /// The wire format is the page's (webrtc_page.html): a type byte, then little-endian fields.
    fn parse(m: &[u8]) -> Option<WebrtcInput> {
        match *m.first()? {
            1 if m.len() >= 4 => Some(WebrtcInput::Key {
                down: m[1] != 0,
                code: u16::from_le_bytes([m[2], m[3]]),
            }),
            2 if m.len() >= 6 => Some(WebrtcInput::Pointer {
                buttons: m[1],
                x: u16::from_le_bytes([m[2], m[3]]),
                y: u16::from_le_bytes([m[4], m[5]]),
            }),
            3 if m.len() >= 3 => Some(WebrtcInput::Wheel {
                steps: i16::from_le_bytes([m[1], m[2]]),
            }),
            _ => None,
        }
    }
}

/// What the configuration says about this screen's endpoint.
pub(crate) struct HubConfig {
    /// `stun:host:port` URLs handed to ICE as-is. Empty is host candidates only, which is a LAN.
    pub stun: Vec<String>,
    /// Required as `?token=` on the signalling endpoints when set.
    pub token: Option<String>,
    /// Whether H.265 is offered (`codec=auto`). H.264 always is.
    pub allow_h265: bool,
    /// Whether viewer input goes anywhere (`view-only=false` and devices exist). Told to every
    /// page, which decides from it where to draw the guest's cursor.
    pub input_enabled: bool,
}

/// Messages from the sink to a viewer's page on the `input` DataChannel. The first byte is the
/// type; fields are little-endian. Input travels the other way with types 1..=3, so these start
/// at 0x10 to keep one byte meaning one thing in both directions.
const MSG_HELLO: u8 = 0x10; // [0x10, input_enabled]
const MSG_CURSOR_SHAPE: u8 = 0x11; // [0x11, w u16, h u16, hot_x u16, hot_y u16, B,G,R,A * w*h]
const MSG_CURSOR_POS: u8 = 0x12; // [0x12, x i32, y i32] -- the pointer (hotspot), screen pixels
const MSG_CURSOR_VISIBLE: u8 = 0x13; // [0x13, visible]

/// Page -> sink messages on the `input` channel that are not input (those are 1..=3).
const MSG_STATS_ENABLE: u8 = 4; // [4, on]
const MSG_PING: u8 = 5; // [5, t f64] -- answered at once with {"type":"pong","c":t,"s":unix_ms}

/// How often a page with statistics on gets a report.
const STATS_INTERVAL: Duration = Duration::from_secs(1);

/// What the encoder side of the statistics report is built from. Counters only: rates are the
/// difference between two reports.
#[derive(Default)]
struct StatsCounters {
    /// Compressed units sent on (parameter sets excluded), their bytes, and how many were IDRs.
    units: AtomicU64,
    bytes: AtomicU64,
    idrs: AtomicU64,
    /// Offer to compressed unit, per unit: the encoder's own latency, queueing included.
    enc_lat_sum_us: AtomicU64,
    enc_lat_n: AtomicU64,
    enc_lat_max_us: AtomicU64,
    /// Pictures offered to the encoder (the guest's flushes, plus refeeds), and the route the last
    /// one took.
    fed: AtomicU64,
    route_gpu: AtomicBool,
}

/// The encoder as last built.
struct EncoderInfo {
    name: String,
    codec: VideoCodec,
    profile: i32,
    width: u32,
    height: u32,
    default_bitrate: i32,
}

/// The output counters at the previous rate correction, and the smoothed rate since.
struct RateSample {
    at: Instant,
    units: u64,
    bytes: u64,
    ema_bps: f64,
}

/// The correction factor's range, in thousandths. Above 1000 because the codec is told the screen's
/// rate: a guest drawing below it gets less than the goal per second and has to be corrected up.
/// Kept low because simple content also undershoots and walks the factor to the cap, which the
/// next burst of motion then overshoots by.
const RATE_FACTOR_MAX: u32 = 1500;
const RATE_FACTOR_MIN: u32 = 50;
/// Where the factor starts: the goal as given.
const RATE_FACTOR_START: u32 = 1000;

/// The counters at the previous report, for rates.
struct StatsSnapshot {
    at: Instant,
    units: u64,
    bytes: u64,
    fed: u64,
}

/// The guest's hardware cursor as last reported, already in wire form, so a page that connects
/// later is brought up to date with the same bytes everyone else got. Also the filter for repeats:
/// virtio-gpu re-sends the whole cursor (image and all) on most pointer moves, and a message equal
/// to the cached one is not sent again.
#[derive(Default)]
struct CursorCache {
    shape: Option<Vec<u8>>,
    pos: Option<Vec<u8>>,
    visible: Option<Vec<u8>>,
}

struct Peer {
    track: c_int,
    ssrc: u32,
    /// The payload types the answer kept, per codec; `None` for a codec the viewer declined.
    /// Both `None` until the answer is in.
    h264_pt: Option<u8>,
    h265_pt: Option<u8>,
    /// The header extension ids the answer kept.
    extensions: HeaderExtensions,
    /// What the track is packetized for, which is what it may be sent; `None` until the answer.
    codec: Option<VideoCodec>,
    input: c_int,
    /// The input channel is open: messages to the page can be sent.
    input_open: bool,
    /// The page asked for statistics reports.
    stats_on: bool,
    gathered: bool,
    /// The track is open: DTLS is up and RTP can flow.
    streaming: bool,
    /// Nothing has been sent yet that a decoder could start from.
    waiting_sync: bool,
    /// The receiver's last REMB estimate, bits per second; `None` until one arrives.
    remb: Option<u32>,
    /// Connection gone; reaped by the HTTP thread, which is the one that may call delete.
    dead: bool,
}

pub(crate) struct WebrtcHub {
    config: HubConfig,
    peers: Mutex<BTreeMap<c_int, Peer>>,
    gathered: Condvar,
    /// Moves on every viewer that starts streaming and on every PLI the throttle lets through.
    joins: AtomicU64,
    last_pli: Mutex<Option<Instant>>,
    /// The encoder's parameter sets as last emitted.
    codec_config: Mutex<Vec<u8>>,
    /// Input decoded on libdatachannel's threads, for the display thread to inject.
    input: Mutex<VecDeque<WebrtcInput>>,
    cursor: Mutex<CursorCache>,
    /// The codec every viewer is being served (one encoder per screen, so one codec), and whether
    /// the display thread still has to rebuild the encoder for it (`take_codec_change`).
    stream_codec: Mutex<VideoCodec>,
    codec_dirty: AtomicBool,
    /// The screen's size as the display last created it, for the levels in the next offer.
    screen: AtomicU64,
    stats: StatsCounters,
    encoder_info: Mutex<Option<EncoderInfo>>,
    /// The pipeline's time base, and when that was in Unix milliseconds: `pts_us` (and so the RTP
    /// timestamp) counts from it, which is how a page maps a frame back to when it was offered.
    epoch: Mutex<Option<(Instant, f64)>>,
    stats_prev: Mutex<Option<StatsSnapshot>>,
    /// The target bitrate may have changed: a REMB moved, a peer came or went, or the encoder was
    /// rebuilt at its default. The display thread re-derives and applies it (`bitrate_target`).
    bitrate_dirty: AtomicBool,
    /// What the display thread last applied to the live encoder; 0 after a rebuild.
    pub(crate) bitrate_applied: AtomicU32,
    /// The rate the stream should average (the viewers' estimate under the ceiling), as the
    /// display thread last derived it; 0 until it has.
    pub(crate) bitrate_goal: AtomicU32,
    /// The encoder is set to `goal * rate_factor / 1000`. The codec budgets per frame at the
    /// frame rate it was told (the screen's rate), so a guest offering more or fewer frames misses
    /// the goal;
    /// `correct_bitrate` measures what actually goes out and moves this until it matches.
    pub(crate) rate_factor: AtomicU32,
    rate_prev: Mutex<Option<RateSample>>,
    /// The display thread is asked to offer its last picture again (a viewer needs a sync frame
    /// and the screen may be still, so nothing else would produce one -- P0-3).
    refeed: AtomicBool,
    /// The display's own event: signalled for input and for `refeed`.
    wake: Event,
    stop: AtomicBool,
}

impl WebrtcHub {
    /// Binds the signalling port and starts serving it. A port that cannot be bound is the caller's
    /// `GpuDisplayError::Listen`: a VM configured with an endpoint it cannot have does not start.
    pub fn start(
        addr: &str,
        config: HubConfig,
        wake: Event,
    ) -> std::io::Result<Arc<WebrtcHub>> {
        static LOGGER: Once = Once::new();
        LOGGER.call_once(|| {
            // SAFETY: the callback is a plain function that only reads the message it is given.
            unsafe { rtcInitLogger(RTC_LOG_WARNING, Some(log_callback)) }
        });

        let listener = TcpListener::bind(addr)?;
        listener.set_nonblocking(true)?;
        let hub = Arc::new(WebrtcHub {
            config,
            peers: Mutex::new(BTreeMap::new()),
            gathered: Condvar::new(),
            joins: AtomicU64::new(0),
            last_pli: Mutex::new(None),
            codec_config: Mutex::new(Vec::new()),
            input: Mutex::new(VecDeque::new()),
            cursor: Mutex::new(CursorCache::default()),
            stream_codec: Mutex::new(VideoCodec::H264),
            codec_dirty: AtomicBool::new(false),
            screen: AtomicU64::new(0),
            stats: StatsCounters::default(),
            encoder_info: Mutex::new(None),
            epoch: Mutex::new(None),
            stats_prev: Mutex::new(None),
            bitrate_dirty: AtomicBool::new(false),
            bitrate_applied: AtomicU32::new(0),
            bitrate_goal: AtomicU32::new(0),
            rate_factor: AtomicU32::new(RATE_FACTOR_START),
            rate_prev: Mutex::new(None),
            refeed: AtomicBool::new(false),
            wake,
            stop: AtomicBool::new(false),
        });
        let serving = hub.clone();
        thread::Builder::new()
            .name("webrtc_http".into())
            .spawn(move || serving.serve(listener))?;
        base::info!("WebRTC: viewer page on http://{}/", addr);
        Ok(hub)
    }

    /// Closes every peer and stops the HTTP thread. Called before the display goes away, so no
    /// libdatachannel callback can arrive after: deleting a peer connection joins its threads.
    pub fn shutdown(&self) {
        self.stop.store(true, Ordering::Relaxed);
        let pcs: Vec<c_int> = self.peers.lock().unwrap().keys().copied().collect();
        for pc in pcs {
            // SAFETY: `pc` came from rtcCreatePeerConnection and is deleted once, here.
            unsafe { rtcDeletePeerConnection(pc) };
        }
        self.peers.lock().unwrap().clear();
    }

    /// Whether the display should offer its last picture again; clears the request.
    pub fn take_refeed(&self) -> bool {
        self.refeed.swap(false, Ordering::Relaxed)
    }

    /// Everything the viewers sent since the last call.
    pub fn take_input(&self, out: &mut VecDeque<WebrtcInput>) {
        out.append(&mut self.input.lock().unwrap());
    }

    /// The guest's cursor image (ARGB8888, `w * h * 4` bytes) and its hotspot.
    pub fn set_cursor_shape(&self, w: u16, h: u16, hot_x: u16, hot_y: u16, argb: &[u8]) {
        let mut m = Vec::with_capacity(9 + argb.len());
        m.push(MSG_CURSOR_SHAPE);
        for v in [w, h, hot_x, hot_y] {
            m.extend_from_slice(&v.to_le_bytes());
        }
        m.extend_from_slice(argb);
        let mut cursor = self.cursor.lock().unwrap();
        if cursor.shape.as_deref() == Some(m.as_slice()) {
            return;
        }
        self.broadcast(&m);
        cursor.shape = Some(m);
    }

    /// Where the guest's pointer is (the hotspot, not the image origin), in screen pixels.
    pub fn set_cursor_pos(&self, x: i32, y: i32) {
        let mut m = Vec::with_capacity(9);
        m.push(MSG_CURSOR_POS);
        m.extend_from_slice(&x.to_le_bytes());
        m.extend_from_slice(&y.to_le_bytes());
        let mut cursor = self.cursor.lock().unwrap();
        if cursor.pos.as_deref() == Some(m.as_slice()) {
            return;
        }
        self.broadcast(&m);
        cursor.pos = Some(m);
    }

    pub fn set_cursor_visible(&self, visible: bool) {
        let m = vec![MSG_CURSOR_VISIBLE, visible as u8];
        let mut cursor = self.cursor.lock().unwrap();
        if cursor.visible.as_deref() == Some(m.as_slice()) {
            return;
        }
        self.broadcast(&m);
        cursor.visible = Some(m);
    }

    /// To every page whose input channel is open. Called with `cursor` held, so a broadcast and
    /// the cache update after it are one step against `on_input_open`: a page either finds the
    /// message in the cache or is already open when it goes out. Lock order is cursor, then peers.
    fn broadcast(&self, m: &[u8]) {
        let targets: Vec<c_int> = self
            .peers
            .lock()
            .unwrap()
            .values()
            .filter(|p| p.input_open)
            .map(|p| p.input)
            .collect();
        for dc in targets {
            send_binary(dc, m);
        }
    }

    /// `Some` when the target bitrate has to be looked at again: the lowest REMB among the
    /// streaming peers that sent one (`None` inside: nobody has, so the encoder's default).
    /// Lowest, because one stream serves all of them and the slowest link sets what it can carry.
    pub fn bitrate_target(&self) -> Option<Option<u32>> {
        if !self.bitrate_dirty.swap(false, Ordering::Relaxed) {
            return None;
        }
        let peers = self.peers.lock().unwrap();
        Some(peers.values().filter(|p| p.streaming).filter_map(|p| p.remb).min())
    }

    /// The codec the encoder has to be rebuilt for, once per decision.
    pub fn take_codec_change(&self) -> Option<VideoCodec> {
        if !self.codec_dirty.swap(false, Ordering::Relaxed) {
            return None;
        }
        Some(*self.stream_codec.lock().unwrap())
    }

    /// The encoder pipeline's time base; see `epoch`.
    pub fn set_epoch(&self, epoch: Instant) {
        let unix_ms = unix_ms() - epoch.elapsed().as_secs_f64() * 1000.0;
        *self.epoch.lock().unwrap() = Some((epoch, unix_ms));
    }

    /// One picture offered to the encoder, by the GPU route or the CPU one.
    pub fn note_fed(&self, gpu: bool) {
        self.stats.fed.fetch_add(1, Ordering::Relaxed);
        self.stats.route_gpu.store(gpu, Ordering::Relaxed);
    }

    /// The report for pages that asked for one, as JSON (built by hand: one flat object).
    fn stats_json(&self) -> String {
        let now = Instant::now();
        let c = &self.stats;
        let (units, bytes, fed) = (
            c.units.load(Ordering::Relaxed),
            c.bytes.load(Ordering::Relaxed),
            c.fed.load(Ordering::Relaxed),
        );
        let (fps, kbps, fed_fps) = {
            let mut prev = self.stats_prev.lock().unwrap();
            let rates = match prev.as_ref() {
                Some(p) => {
                    let dt = now.duration_since(p.at).as_secs_f64().max(0.001);
                    (
                        (units - p.units) as f64 / dt,
                        (bytes - p.bytes) as f64 * 8.0 / 1000.0 / dt,
                        (fed - p.fed) as f64 / dt,
                    )
                }
                None => (0.0, 0.0, 0.0),
            };
            *prev = Some(StatsSnapshot {
                at: now,
                units,
                bytes,
                fed,
            });
            rates
        };
        let n = c.enc_lat_n.swap(0, Ordering::Relaxed);
        let sum = c.enc_lat_sum_us.swap(0, Ordering::Relaxed);
        let max = c.enc_lat_max_us.swap(0, Ordering::Relaxed);
        let enc_avg = if n > 0 { sum as f64 / n as f64 / 1000.0 } else { -1.0 };
        let (remb, viewers) = {
            let peers = self.peers.lock().unwrap();
            let streaming = peers.values().filter(|p| p.streaming);
            (
                streaming.clone().filter_map(|p| p.remb).min(),
                streaming.count(),
            )
        };
        let applied = self.bitrate_applied.load(Ordering::Relaxed);
        let goal = self.bitrate_goal.load(Ordering::Relaxed);
        let info = self.encoder_info.lock().unwrap();
        let (name, codec, profile, w, h, target, set) = match info.as_ref() {
            Some(i) => (
                i.name.as_str(),
                format!("{:?}", i.codec),
                i.profile,
                i.width,
                i.height,
                if goal > 0 { goal as i64 } else { i.default_bitrate as i64 },
                if applied > 0 { applied as i64 } else { i.default_bitrate as i64 },
            ),
            None => ("", String::new(), 0, 0, 0, 0, 0),
        };
        let epoch_unix = self.epoch.lock().unwrap().map(|e| e.1).unwrap_or(0.0);
        format!(
            concat!(
                "{{\"type\":\"stats\",\"encoder\":\"{}\",\"codec\":\"{}\",\"profile\":{},",
                "\"width\":{},\"height\":{},\"fps\":{:.1},\"kbps\":{:.0},",
                "\"target_kbps\":{},\"set_kbps\":{},\"remb_kbps\":{},\"idr\":{},\"enc_ms\":{:.2},",
                "\"enc_ms_max\":{:.2},\"fed_fps\":{:.1},\"route\":\"{}\",\"viewers\":{},",
                "\"epoch_unix_ms\":{:.3},\"server_unix_ms\":{:.3}}}"
            ),
            name.replace(['"', '\\'], ""),
            codec,
            profile,
            w,
            h,
            fps,
            kbps,
            target / 1000,
            set / 1000,
            remb.map(|r| (r / 1000).to_string()).unwrap_or_else(|| "null".into()),
            c.idrs.load(Ordering::Relaxed),
            enc_avg,
            max as f64 / 1000.0,
            fed_fps,
            if c.route_gpu.load(Ordering::Relaxed) { "gpu" } else { "cpu" },
            viewers,
            epoch_unix,
            unix_ms(),
        )
    }

    /// Once a second, from the HTTP thread: holds what goes out to the goal by moving
    /// `rate_factor`, and asks the display thread to re-apply when it moved.
    ///
    /// Learns only while the picture moves (10+ units in the second): a still screen sends next to
    /// nothing, and unlearning on it would hand the next burst of motion the full overshoot again.
    /// The rate is smoothed over about two seconds, so one IDR is not read as a trend.
    fn correct_bitrate(&self) {
        let now = Instant::now();
        let units = self.stats.units.load(Ordering::Relaxed);
        let bytes = self.stats.bytes.load(Ordering::Relaxed);
        let mut prev = self.rate_prev.lock().unwrap();
        let Some(p) = prev.as_mut() else {
            *prev = Some(RateSample {
                at: now,
                units,
                bytes,
                ema_bps: 0.0,
            });
            return;
        };
        let dt = now.duration_since(p.at).as_secs_f64().max(0.001);
        let moving = units - p.units >= 10;
        let bps = (bytes - p.bytes) as f64 * 8.0 / dt;
        p.at = now;
        p.units = units;
        p.bytes = bytes;
        if !moving {
            return;
        }
        p.ema_bps = if p.ema_bps == 0.0 { bps } else { 0.5 * p.ema_bps + 0.5 * bps };
        let goal = match self.bitrate_goal.load(Ordering::Relaxed) {
            0 => match self.encoder_info.lock().unwrap().as_ref() {
                Some(info) => info.default_bitrate as f64,
                None => return,
            },
            g => g as f64,
        };
        let factor = self.rate_factor.load(Ordering::Relaxed);
        let next = if p.ema_bps > goal * 1.10 {
            (factor as f64 * (goal / p.ema_bps).clamp(0.5, 0.95)) as u32
        } else if p.ema_bps < goal * 0.80 {
            (factor as f64 * 1.15) as u32
        } else {
            factor
        }
        .clamp(RATE_FACTOR_MIN, RATE_FACTOR_MAX);
        if next != factor {
            base::debug!(
                "WebRTC: output {:.0} kbit/s against {:.0}; correction {} -> {}",
                p.ema_bps / 1000.0,
                goal / 1000.0,
                factor,
                next
            );
            self.rate_factor.store(next, Ordering::Relaxed);
            drop(prev);
            self.bitrate_changed();
        }
    }

    /// Once per `STATS_INTERVAL`, from the HTTP thread: a report to every page that asked.
    fn send_stats(&self) {
        let targets: Vec<c_int> = self
            .peers
            .lock()
            .unwrap()
            .values()
            .filter(|p| p.stats_on && p.input_open)
            .map(|p| p.input)
            .collect();
        if targets.is_empty() {
            return;
        }
        let json = self.stats_json();
        for dc in targets {
            send_text(dc, &json);
        }
    }

    pub fn set_screen_size(&self, width: u32, height: u32) {
        self.screen
            .store(((width as u64) << 32) | height as u64, Ordering::Relaxed);
    }

    /// Settles which codec the screen is served in and packetizes every answered track for it.
    ///
    /// H.265 when it is allowed and every answered viewer took it, H.264 otherwise: one encoder
    /// serves them all, so the least capable viewer decides. A change re-packetizes the tracks
    /// already streaming (each then waits for the new encoder's first IDR) and asks the display
    /// thread to rebuild the encoder. Run on the HTTP thread only -- after an answer and after a
    /// reap -- because a packetizer is not to be swapped from inside a libdatachannel callback.
    fn reconcile_codec(&self) {
        let mut todo: Vec<(c_int, c_int, u32, u8, VideoCodec, HeaderExtensions)> = Vec::new();
        let mut unservable: Vec<c_int> = Vec::new();
        let desired = {
            let mut peers = self.peers.lock().unwrap();
            let answered: Vec<&Peer> = peers
                .values()
                .filter(|p| !p.dead && (p.h264_pt.is_some() || p.h265_pt.is_some()))
                .collect();
            let desired = if self.config.allow_h265
                && !answered.is_empty()
                && answered.iter().all(|p| p.h265_pt.is_some())
            {
                VideoCodec::H265
            } else {
                VideoCodec::H264
            };
            for (pc, p) in peers.iter_mut() {
                if p.dead || (p.h264_pt.is_none() && p.h265_pt.is_none()) {
                    continue;
                }
                if p.codec == Some(desired) {
                    continue;
                }
                let pt = match desired {
                    VideoCodec::H264 => p.h264_pt,
                    VideoCodec::H265 => p.h265_pt,
                };
                match pt {
                    Some(pt) => {
                        p.codec = Some(desired);
                        p.waiting_sync = true;
                        todo.push((*pc, p.track, p.ssrc, pt, desired, p.extensions));
                    }
                    // Took H.265 only, while somebody else needs H.264.
                    None => unservable.push(*pc),
                }
            }
            desired
        };
        for pc in unservable {
            base::warn!("WebRTC: peer {} declined H.264, which another viewer needs; dropped", pc);
            self.mark_dead(pc);
        }
        let mut switched = false;
        for (pc, track, ssrc, pt, codec, extensions) in todo {
            if !packetize(track, codec, pt, ssrc, extensions) {
                base::error!("WebRTC: peer {}: could not set up the {:?} packetizer", pc, codec);
                self.mark_dead(pc);
                continue;
            }
            base::info!("WebRTC: peer {} served {:?} (payload type {})", pc, codec, pt);
            switched = true;
        }
        let mut current = self.stream_codec.lock().unwrap();
        if *current != desired {
            base::info!("WebRTC: stream codec {:?} -> {:?}", *current, desired);
            *current = desired;
            self.codec_dirty.store(true, Ordering::Relaxed);
            drop(current);
            self.ask_for_sync();
        } else if switched {
            let _ = self.wake.signal();
        }
    }

    fn bitrate_changed(&self) {
        self.bitrate_dirty.store(true, Ordering::Relaxed);
        let _ = self.wake.signal();
    }

    fn ask_for_sync(&self) {
        self.joins.fetch_add(1, Ordering::Relaxed);
        self.refeed.store(true, Ordering::Relaxed);
        let _ = self.wake.signal();
    }

    fn ptr(self: &Arc<Self>) -> *mut c_void {
        Arc::as_ptr(self) as *mut c_void
    }

    // ---------------------------------------------------------------------------------------
    // Signalling.
    // ---------------------------------------------------------------------------------------

    fn serve(self: Arc<Self>, listener: TcpListener) {
        let mut last_stats = Instant::now();
        while !self.stop.load(Ordering::Relaxed) {
            self.reap();
            if last_stats.elapsed() >= STATS_INTERVAL {
                last_stats = Instant::now();
                self.correct_bitrate();
                self.send_stats();
            }
            match listener.accept() {
                Ok((stream, _)) => {
                    if let Err(e) = self.handle(stream) {
                        base::warn!("WebRTC: request failed: {}", e);
                    }
                }
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                    thread::sleep(Duration::from_millis(100));
                }
                Err(e) => {
                    base::error!("WebRTC: accept failed: {}", e);
                    thread::sleep(Duration::from_millis(500));
                }
            }
        }
    }

    /// Deletes the peers whose connection ended. Here rather than in the state callback, because
    /// libdatachannel must not be asked to delete a connection from inside its own callback.
    fn reap(&self) {
        let dead: Vec<c_int> = {
            let mut peers = self.peers.lock().unwrap();
            let dead: Vec<c_int> = peers.iter().filter(|(_, p)| p.dead).map(|(k, _)| *k).collect();
            for pc in &dead {
                peers.remove(pc);
            }
            dead
        };
        let any = !dead.is_empty();
        for pc in dead {
            // SAFETY: removed from the table above, so this is its only deletion.
            unsafe { rtcDeletePeerConnection(pc) };
            base::info!("WebRTC: peer {} closed", pc);
        }
        if any {
            // The viewer that held the screen to H.264 may be the one that left.
            self.reconcile_codec();
        }
    }

    fn handle(self: &Arc<Self>, mut stream: TcpStream) -> std::io::Result<()> {
        stream.set_nonblocking(false)?;
        stream.set_read_timeout(Some(Duration::from_secs(5)))?;
        let mut req = Vec::new();
        let mut buf = [0u8; 8192];
        let (head_end, body_len) = loop {
            let n = stream.read(&mut buf)?;
            if n == 0 {
                return Ok(());
            }
            req.extend_from_slice(&buf[..n]);
            if req.len() > 256 * 1024 {
                return respond(&mut stream, "413 Payload Too Large", "text/plain", b"");
            }
            if let Some(end) = find(&req, b"\r\n\r\n") {
                let head = String::from_utf8_lossy(&req[..end]).to_ascii_lowercase();
                let len = head
                    .lines()
                    .find_map(|l| l.strip_prefix("content-length:"))
                    .and_then(|v| v.trim().parse::<usize>().ok())
                    .unwrap_or(0);
                break (end + 4, len);
            }
        };
        while req.len() < head_end + body_len {
            let n = stream.read(&mut buf)?;
            if n == 0 {
                break;
            }
            req.extend_from_slice(&buf[..n]);
        }
        let head = String::from_utf8_lossy(&req[..head_end]).into_owned();
        let body = &req[head_end..(head_end + body_len).min(req.len())];
        let mut words = head.split_whitespace();
        let method = words.next().unwrap_or("");
        let target = words.next().unwrap_or("/");
        let (path, query) = target.split_once('?').unwrap_or((target, ""));
        let param = |name: &str| {
            query
                .split('&')
                .find_map(|kv| kv.strip_prefix(name).and_then(|v| v.strip_prefix('=')))
        };

        match (method, path) {
            ("GET", "/") | ("GET", "/index.html") => {
                respond(&mut stream, "200 OK", "text/html; charset=utf-8", PAGE.as_bytes())
            }
            ("GET", "/offer") | ("POST", "/answer") => {
                if let Some(token) = &self.config.token {
                    if param("token") != Some(token.as_str()) {
                        return respond(&mut stream, "403 Forbidden", "text/plain", b"token");
                    }
                }
                if method == "GET" {
                    match self.create_peer(CodecChoice::parse(param("codec"))) {
                        Ok((pc, sdp)) => respond_with(
                            &mut stream,
                            "200 OK",
                            "application/sdp",
                            &[("X-Peer", pc.to_string())],
                            sdp.as_bytes(),
                        ),
                        Err(e) => {
                            base::warn!("WebRTC: cannot create a peer: {}", e);
                            respond(&mut stream, "503 Service Unavailable", "text/plain", e.as_bytes())
                        }
                    }
                } else {
                    let Some(pc) = param("peer").and_then(|v| v.parse::<c_int>().ok()) else {
                        return respond(&mut stream, "400 Bad Request", "text/plain", b"peer");
                    };
                    match self.accept_answer(pc, body) {
                        Ok(()) => respond(&mut stream, "200 OK", "text/plain", b"ok"),
                        Err(e) => {
                            respond(&mut stream, "400 Bad Request", "text/plain", e.as_bytes())
                        }
                    }
                }
            }
            _ => respond(&mut stream, "404 Not Found", "text/plain", b""),
        }
    }

    fn create_peer(self: &Arc<Self>, choice: CodecChoice) -> Result<(c_int, String), String> {
        if self.peers.lock().unwrap().values().filter(|p| !p.dead).count() >= MAX_PEERS {
            return Err(format!("this screen already serves {} viewers", MAX_PEERS));
        }
        let stun: Vec<CString> = self
            .config
            .stun
            .iter()
            .filter_map(|s| CString::new(s.as_str()).ok())
            .collect();
        let stun_ptrs: Vec<*const c_char> = stun.iter().map(|s| s.as_ptr()).collect();
        let config = rtcConfiguration {
            ice_servers: if stun_ptrs.is_empty() { std::ptr::null() } else { stun_ptrs.as_ptr() },
            ice_servers_count: stun_ptrs.len() as c_int,
            proxy_server: std::ptr::null(),
            bind_address: std::ptr::null(),
            certificate_type: 0,
            ice_transport_policy: 0,
            enable_ice_tcp: false,
            enable_ice_udp_mux: false,
            disable_auto_negotiation: true,
            force_media_transport: false,
            port_range_begin: 0,
            port_range_end: 0,
            mtu: 0,
            max_message_size: 0,
        };
        // SAFETY: `config` and the strings it points at are live for the call.
        let pc = unsafe { rtcCreatePeerConnection(&config) };
        if pc < 0 {
            return Err(format!("rtcCreatePeerConnection: {}", pc));
        }
        let ptr = self.ptr();
        // SAFETY: `pc` is live; the user pointer is the hub, which outlives every peer
        // (`shutdown` deletes them all before the hub can go).
        unsafe {
            rtcSetUserPointer(pc, ptr);
            rtcSetStateChangeCallback(pc, Some(on_state));
            rtcSetGatheringStateChangeCallback(pc, Some(on_gathering));
        }

        let ssrc: u32 = rand_u32();
        let media = CString::new(self.video_media(ssrc, choice)).map_err(|e| e.to_string())?;
        // The m-line is written out rather than built by rtcAddTrackEx, which offers one codec.
        // The packetizer comes after the answer (`reconcile_codec`), once it is known which codec
        // this viewer took; until then the track has none and is sent nothing.
        // SAFETY: `media` is live for the call.
        let track = unsafe { rtcAddTrack(pc, media.as_ptr()) };
        if track < 0 {
            // SAFETY: created above and in no table yet.
            unsafe { rtcDeletePeerConnection(pc) };
            return Err(format!("rtcAddTrack: {}", track));
        }
        // SAFETY: `track` is live.
        let chained = unsafe {
            rtcSetUserPointer(track, ptr);
            rtcSetOpenCallback(track, Some(on_track_open)) >= 0
                && rtcSetClosedCallback(track, Some(on_track_closed)) >= 0
        };
        // SAFETY: `pc` is live.
        let input = unsafe { rtcCreateDataChannel(pc, c"input".as_ptr()) };
        if !chained || input < 0 {
            // SAFETY: created above and in no table yet.
            unsafe { rtcDeletePeerConnection(pc) };
            return Err("could not set up the track or the input channel".into());
        }
        // SAFETY: `input` is live.
        unsafe {
            rtcSetUserPointer(input, ptr);
            rtcSetMessageCallback(input, Some(on_input));
            rtcSetOpenCallback(input, Some(on_input_open));
            rtcSetClosedCallback(input, Some(on_input_closed));
        }

        self.peers.lock().unwrap().insert(
            pc,
            Peer {
                track,
                ssrc,
                h264_pt: None,
                h265_pt: None,
                extensions: HeaderExtensions::default(),
                codec: None,
                input,
                input_open: false,
                stats_on: false,
                gathered: false,
                streaming: false,
                waiting_sync: true,
                remb: None,
                dead: false,
            },
        );
        // SAFETY: `pc` is live.
        if unsafe { rtcSetLocalDescription(pc, c"offer".as_ptr()) } < 0 {
            self.mark_dead(pc);
            return Err("rtcSetLocalDescription failed".into());
        }
        {
            let peers = self.peers.lock().unwrap();
            let _ = self
                .gathered
                .wait_timeout_while(peers, GATHER_TIMEOUT, |p| {
                    p.get(&pc).map(|p| !p.gathered).unwrap_or(false)
                })
                .unwrap();
        }
        let mut sdp = vec![0u8; 64 * 1024];
        // SAFETY: `sdp` is a writable buffer of the size passed.
        let n = unsafe { rtcGetLocalDescription(pc, sdp.as_mut_ptr() as *mut c_char, sdp.len() as c_int) };
        if n <= 0 {
            self.mark_dead(pc);
            return Err(format!("rtcGetLocalDescription: {}", n));
        }
        let end = sdp.iter().position(|b| *b == 0).unwrap_or(n as usize);
        sdp.truncate(end);
        base::info!("WebRTC: peer {} offered", pc);
        Ok((pc, String::from_utf8_lossy(&sdp).into_owned()))
    }

    /// The video m-section of an offer: H.265 first when allowed, then H.264, each with the
    /// feedback the sender uses (NACK, PLI, REMB), and the one SSRC both share.
    fn video_media(&self, ssrc: u32, choice: CodecChoice) -> String {
        let screen = self.screen.load(Ordering::Relaxed);
        let (w, h) = ((screen >> 32) as u32, screen as u32);
        let mut codecs = Vec::new();
        let h265 = self.config.allow_h265 && choice != CodecChoice::H264;
        if h265 {
            codecs.push((H265_PAYLOAD_TYPE, "H265", VideoCodec::H265));
        }
        if !h265 || choice != CodecChoice::H265 {
            codecs.push((H264_PAYLOAD_TYPE, "H264", VideoCodec::H264));
        }
        let pts: Vec<String> = codecs.iter().map(|c| c.0.to_string()).collect();
        let mut m = format!(
            "m=video 9 UDP/TLS/RTP/SAVPF {}\r\na=mid:video\r\na=sendonly\r\na=msid:droidvm video\r\n",
            pts.join(" ")
        );
        for (pt, name, codec) in codecs {
            m += &format!("a=rtpmap:{pt} {name}/90000\r\na=fmtp:{pt} {}\r\n", codec_fmtp(codec, w, h));
            for fb in ["nack", "nack pli", "goog-remb"] {
                m += &format!("a=rtcp-fb:{pt} {fb}\r\n");
            }
        }
        m += &format!("a=extmap:{ABS_SEND_TIME_EXT_ID} {ABS_SEND_TIME_URI}\r\n");
        m += &format!("a=extmap:{PLAYOUT_DELAY_EXT_ID} {PLAYOUT_DELAY_URI}\r\n");
        m += &format!("a=ssrc:{ssrc} cname:droidvm\r\na=ssrc:{ssrc} msid:droidvm video\r\n");
        m
    }

    fn accept_answer(&self, pc: c_int, sdp: &[u8]) -> Result<(), String> {
        if !self.peers.lock().unwrap().contains_key(&pc) {
            return Err(format!("no peer {}", pc));
        }
        let answer = String::from_utf8_lossy(sdp);
        let (h264_pt, h265_pt) = answered_payload_types(&answer);
        let extensions = HeaderExtensions {
            playout_delay: answered_extension_id(&answer, PLAYOUT_DELAY_URI),
            abs_send_time: answered_extension_id(&answer, ABS_SEND_TIME_URI),
        };
        if h264_pt.is_none() && h265_pt.is_none() {
            self.mark_dead(pc);
            return Err("the answer takes neither H.264 nor H.265".into());
        }
        let sdp = CString::new(sdp).map_err(|_| "the answer has a NUL in it".to_string())?;
        // SAFETY: `pc` is in the table, so live; the strings are live for the call.
        if unsafe { rtcSetRemoteDescription(pc, sdp.as_ptr(), c"answer".as_ptr()) } < 0 {
            self.mark_dead(pc);
            return Err("the answer was refused".into());
        }
        if let Some(peer) = self.peers.lock().unwrap().get_mut(&pc) {
            peer.h264_pt = h264_pt;
            peer.h265_pt = h265_pt;
            peer.extensions = extensions;
        }
        base::info!(
            "WebRTC: peer {} answered: H.264 {:?}, H.265 {:?}, {:?}",
            pc,
            h264_pt,
            h265_pt,
            extensions
        );
        self.reconcile_codec();
        Ok(())
    }

    fn mark_dead(&self, pc: c_int) {
        if let Some(peer) = self.peers.lock().unwrap().get_mut(&pc) {
            peer.dead = true;
            peer.streaming = false;
        }
        self.bitrate_changed();
    }
}

/// The drain thread's side: compressed units out to every peer that can use them.
pub(crate) struct WebrtcAudience(pub Arc<WebrtcHub>);

impl EncodedAudience for WebrtcAudience {
    fn wants_frames(&self) -> bool {
        self.0.peers.lock().unwrap().values().any(|p| p.streaming)
    }

    fn join_generation(&self) -> u64 {
        self.0.joins.load(Ordering::Relaxed)
    }

    /// A new encoder starts a new stream: every peer waits for its first sync frame.
    fn encoder_changed(&self, width: u32, height: u32) {
        base::info!("WebRTC: stream is now {}x{}", width, height);
        for peer in self.0.peers.lock().unwrap().values_mut() {
            peer.waiting_sync = true;
        }
        // The new encoder runs at its starting rate; whatever was adapted, and the correction
        // learnt for the old one, has to be applied again.
        self.0.bitrate_applied.store(0, Ordering::Relaxed);
        self.0.bitrate_changed();
    }

    fn encoder_built(&self, name: &str, config: &EncoderConfig, bitrate_bps: i32) {
        base::info!("WebRTC: encoder {} ({:?})", name, config.codec);
        let screen = self.0.screen.load(Ordering::Relaxed);
        *self.0.encoder_info.lock().unwrap() = Some(EncoderInfo {
            name: name.to_string(),
            codec: config.codec,
            profile: config.profile,
            width: (screen >> 32) as u32,
            height: screen as u32,
            default_bitrate: bitrate_bps,
        });
    }

    fn on_unit(
        &self,
        data: &[u8],
        flags: u32,
        pts_us: i64,
        width: u32,
        height: u32,
        codec: VideoCodec,
    ) {
        if flags & BUFFER_FLAG_CODEC_CONFIG != 0 {
            *self.0.codec_config.lock().unwrap() = data.to_vec();
            return;
        }
        let sync = flags & BUFFER_FLAG_SYNC_FRAME != 0;
        {
            let c = &self.0.stats;
            c.units.fetch_add(1, Ordering::Relaxed);
            c.bytes.fetch_add(data.len() as u64, Ordering::Relaxed);
            if sync {
                c.idrs.fetch_add(1, Ordering::Relaxed);
            }
            if let Some((epoch, _)) = *self.0.epoch.lock().unwrap() {
                let lat = (epoch.elapsed().as_micros() as i64 - pts_us).max(0) as u64;
                c.enc_lat_sum_us.fetch_add(lat, Ordering::Relaxed);
                c.enc_lat_n.fetch_add(1, Ordering::Relaxed);
                c.enc_lat_max_us.fetch_max(lat, Ordering::Relaxed);
            }
            if let Some(info) = self.0.encoder_info.lock().unwrap().as_mut() {
                info.width = width;
                info.height = height;
            }
        }
        // Who gets this unit, decided under the lock and sent outside it: libdatachannel calls
        // back into the peer table from its own threads, and a send must not wait behind one.
        let targets: Vec<c_int> = {
            let mut peers = self.0.peers.lock().unwrap();
            peers
                .values_mut()
                // A track is sent only what it is packetized for: units still draining out of an
                // encoder that a codec switch replaced go to nobody.
                .filter(|p| p.streaming && p.codec == Some(codec))
                .filter_map(|p| {
                    if p.waiting_sync {
                        if !sync {
                            return None;
                        }
                        p.waiting_sync = false;
                    }
                    Some(p.track)
                })
                .collect()
        };
        if targets.is_empty() {
            return;
        }
        let framed;
        let payload: &[u8] = if sync {
            let config = self.0.codec_config.lock().unwrap();
            framed = [config.as_slice(), data].concat();
            &framed
        } else {
            data
        };
        // 90 kHz, from the encoder's own presentation time, so the browser paces playout by when
        // the guest actually produced each picture rather than by a nominal frame rate.
        let rtp_ts = ((pts_us.max(0) as u64) * 9 / 100) as u32;
        for track in targets {
            // SAFETY: `track` belongs to a peer that was live under the lock a moment ago; a peer
            // deleted since makes these calls fail, which is all the handling a dropped viewer
            // needs. `payload` is live for the call.
            unsafe {
                rtcSetTrackRtpTimestamp(track, rtp_ts);
                rtcSendMessage(track, payload.as_ptr() as *const c_char, payload.len() as c_int);
            }
        }
    }
}

// -------------------------------------------------------------------------------------------
// libdatachannel callbacks. `ptr` is always the hub (`WebrtcHub::ptr`).
// -------------------------------------------------------------------------------------------

/// SAFETY (all of them): `ptr` was set to the hub's `Arc` pointer for this id, and the hub outlives
/// every id because `shutdown` deletes them all first.
fn hub<'a>(ptr: *mut c_void) -> &'a WebrtcHub {
    unsafe { &*(ptr as *const WebrtcHub) }
}

extern "C" fn log_callback(level: rtcLogLevel, message: *const c_char) {
    if message.is_null() {
        return;
    }
    // SAFETY: libdatachannel passes a NUL-terminated string valid for the call.
    let message = unsafe { CStr::from_ptr(message) }.to_string_lossy();
    if level <= 2 {
        base::error!("libdatachannel: {}", message);
    } else {
        base::warn!("libdatachannel: {}", message);
    }
}

extern "C" fn on_state(pc: c_int, state: rtcState, ptr: *mut c_void) {
    let hub = hub(ptr);
    match state {
        RTC_CONNECTED => base::info!("WebRTC: peer {} connected", pc),
        RTC_DISCONNECTED | RTC_FAILED | RTC_CLOSED => {
            base::info!("WebRTC: peer {} ended (state {})", pc, state);
            hub.mark_dead(pc);
        }
        _ => {}
    }
}

extern "C" fn on_gathering(pc: c_int, state: rtcGatheringState, ptr: *mut c_void) {
    if state != RTC_GATHERING_COMPLETE {
        return;
    }
    let hub = hub(ptr);
    if let Some(peer) = hub.peers.lock().unwrap().get_mut(&pc) {
        peer.gathered = true;
    }
    hub.gathered.notify_all();
}

extern "C" fn on_track_open(tr: c_int, ptr: *mut c_void) {
    let hub = hub(ptr);
    let opened = {
        let mut peers = hub.peers.lock().unwrap();
        match peers.values_mut().find(|p| p.track == tr) {
            Some(peer) => {
                peer.streaming = true;
                peer.waiting_sync = true;
                true
            }
            None => false,
        }
    };
    if opened {
        base::info!("WebRTC: track {} open; asking for a sync frame", tr);
        hub.ask_for_sync();
    }
}

extern "C" fn on_track_closed(tr: c_int, ptr: *mut c_void) {
    let hub = hub(ptr);
    if let Some(peer) = hub.peers.lock().unwrap().values_mut().find(|p| p.track == tr) {
        peer.streaming = false;
    }
    hub.bitrate_changed();
}

/// A receiver's bandwidth estimate (REMB, which the offer asks for with `goog-remb`). Arrives
/// about once a second per peer; only a move of more than 5% is worth waking the display for.
extern "C" fn on_remb(tr: c_int, bitrate: c_uint, ptr: *mut c_void) {
    let hub = hub(ptr);
    let moved = {
        let mut peers = hub.peers.lock().unwrap();
        match peers.values_mut().find(|p| p.track == tr) {
            Some(peer) => {
                let moved = peer
                    .remb
                    .map(|old| (bitrate as i64 - old as i64).unsigned_abs() * 20 > old as u64)
                    .unwrap_or(true);
                peer.remb = Some(bitrate);
                moved
            }
            None => false,
        }
    };
    if moved {
        hub.bitrate_changed();
    }
}

extern "C" fn on_pli(tr: c_int, ptr: *mut c_void) {
    let hub = hub(ptr);
    // A receiver asks for a keyframe as soon as it has none, which for a viewer that just joined
    // is while the sync frame its join already asked for is on its way: answering that too made
    // every connection start with two IDRs ~100 ms apart.
    let pending = hub
        .peers
        .lock()
        .unwrap()
        .values()
        .any(|p| p.track == tr && p.waiting_sync);
    if pending {
        return;
    }
    {
        let mut last = hub.last_pli.lock().unwrap();
        if last.map(|t| t.elapsed() < PLI_MIN_INTERVAL).unwrap_or(false) {
            return;
        }
        *last = Some(Instant::now());
    }
    base::info!("WebRTC: PLI on track {}", tr);
    hub.ask_for_sync();
}

/// A page's input channel came up: say whether its input is used, then replay the cursor.
extern "C" fn on_input_open(dc: c_int, ptr: *mut c_void) {
    let hub = hub(ptr);
    send_binary(dc, &[MSG_HELLO, hub.config.input_enabled as u8]);
    // Under `cursor` (see `broadcast`): nothing can be sent between the replay and the open mark.
    let cursor = hub.cursor.lock().unwrap();
    for m in [&cursor.shape, &cursor.visible, &cursor.pos].into_iter().flatten() {
        send_binary(dc, m);
    }
    if let Some(peer) = hub.peers.lock().unwrap().values_mut().find(|p| p.input == dc) {
        peer.input_open = true;
    }
}

/// The page went away: it never closes its input channel otherwise, and the SCTP shutdown that
/// closes it arrives at once, where the connection itself is only declared gone by ICE ~15 s later.
/// That wait mattered: a viewer that held the screen to H.264 kept it there until then. Marked dead
/// here and deleted by the HTTP thread's reap, which also re-decides the codec.
extern "C" fn on_input_closed(dc: c_int, ptr: *mut c_void) {
    let hub = hub(ptr);
    let gone = {
        let mut peers = hub.peers.lock().unwrap();
        match peers.iter_mut().find(|(_, p)| p.input == dc) {
            Some((pc, peer)) => {
                peer.input_open = false;
                (!peer.dead).then_some(*pc)
            }
            None => None,
        }
    };
    if let Some(pc) = gone {
        base::info!("WebRTC: peer {} closed its input channel; viewer gone", pc);
        hub.mark_dead(pc);
    }
}

extern "C" fn on_input(dc: c_int, message: *const c_char, size: c_int, ptr: *mut c_void) {
    // Binary messages only: libdatachannel reports a string message with a negative size.
    if message.is_null() || size <= 0 {
        return;
    }
    // SAFETY: `size` bytes at `message` are valid for the call.
    let bytes = unsafe { std::slice::from_raw_parts(message as *const u8, size as usize) };
    match bytes[0] {
        MSG_STATS_ENABLE if bytes.len() >= 2 => {
            let hub = hub(ptr);
            if let Some(peer) = hub.peers.lock().unwrap().values_mut().find(|p| p.input == dc) {
                peer.stats_on = bytes[1] != 0;
            }
            return;
        }
        // Answered here, on libdatachannel's thread, so the reply's delay is the network's.
        MSG_PING if bytes.len() >= 9 => {
            let mut t = [0u8; 8];
            t.copy_from_slice(&bytes[1..9]);
            let client = f64::from_le_bytes(t);
            send_text(
                dc,
                &format!(
                    "{{\"type\":\"pong\",\"c\":{:.3},\"s\":{:.3}}}",
                    client,
                    unix_ms()
                ),
            );
            return;
        }
        _ => {}
    }
    let Some(event) = WebrtcInput::parse(bytes) else {
        return;
    };
    let hub = hub(ptr);
    hub.input.lock().unwrap().push_back(event);
    let _ = hub.wake.signal();
}

// -------------------------------------------------------------------------------------------
// Small helpers.
// -------------------------------------------------------------------------------------------

/// Installs the packetizer for `codec` on a track, and the RTCP handlers chained behind it (sender
/// reports, NACK retransmission, PLI and REMB in). Setting a packetizer replaces the whole chain,
/// so this is also how a track changes codec.
fn packetize(
    track: c_int,
    codec: VideoCodec,
    pt: u8,
    ssrc: u32,
    extensions: HeaderExtensions,
) -> bool {
    let mut packetizer = rtcPacketizerInit::zeroed();
    packetizer.ssrc = ssrc;
    packetizer.cname = c"droidvm".as_ptr();
    packetizer.payload_type = pt;
    packetizer.clock_rate = 90_000;
    packetizer.nal_separator = RTC_NAL_SEPARATOR_START_SEQUENCE;
    packetizer.max_fragment_size = MAX_FRAGMENT;
    if let Some(id) = extensions.playout_delay {
        packetizer.playout_delay_id = id;
        packetizer.playout_delay_min = 0;
        packetizer.playout_delay_max = 0;
    }
    if let Some(id) = extensions.abs_send_time {
        packetizer.abs_send_time_id = id;
    }
    // SAFETY: `track` is live (a stale one makes these fail) and `packetizer` is read during the
    // call only.
    unsafe {
        let set = match codec {
            VideoCodec::H264 => rtcSetH264Packetizer(track, &packetizer),
            VideoCodec::H265 => rtcSetH265Packetizer(track, &packetizer),
        };
        set >= 0
            && rtcChainRtcpSrReporter(track) >= 0
            && rtcChainRtcpNackResponder(track, 512) >= 0
            && rtcChainPliHandler(track, Some(on_pli)) >= 0
            && rtcChainRembHandler(track, Some(on_remb)) >= 0
    }
}

/// The payload types an answer kept for each codec: the first of each named in the video
/// m-line's format list. The answer only ever repeats ones from our offer.
fn answered_payload_types(sdp: &str) -> (Option<u8>, Option<u8>) {
    let mut formats: Vec<u8> = Vec::new();
    let mut in_video = false;
    let (mut h264, mut h265) = (None, None);
    for line in sdp.lines().map(str::trim) {
        if let Some(m) = line.strip_prefix("m=") {
            in_video = m.starts_with("video ");
            if in_video {
                formats = m.split_whitespace().skip(3).filter_map(|f| f.parse().ok()).collect();
            }
            continue;
        }
        let Some(map) = line.strip_prefix("a=rtpmap:").filter(|_| in_video) else {
            continue;
        };
        let mut parts = map.split_whitespace();
        let (Some(pt), Some(name)) = (parts.next().and_then(|p| p.parse::<u8>().ok()), parts.next())
        else {
            continue;
        };
        if !formats.contains(&pt) {
            continue;
        }
        let name = name.split('/').next().unwrap_or("").to_ascii_uppercase();
        match name.as_str() {
            "H264" if h264.is_none() => h264 = Some(pt),
            "H265" if h265.is_none() => h265 = Some(pt),
            _ => {}
        }
    }
    (h264, h265)
}

/// The id an answer kept for the RTP header extension `uri` in its video section, if it kept it.
fn answered_extension_id(sdp: &str, uri: &str) -> Option<u8> {
    let mut in_video = false;
    for line in sdp.lines().map(str::trim) {
        if let Some(m) = line.strip_prefix("m=") {
            in_video = m.starts_with("video ");
            continue;
        }
        let Some(ext) = line.strip_prefix("a=extmap:").filter(|_| in_video) else {
            continue;
        };
        let mut parts = ext.split_whitespace();
        let (Some(id), Some(u)) = (parts.next(), parts.next()) else {
            continue;
        };
        if u == uri {
            // "5" or "5/sendonly"
            return id.split('/').next()?.parse().ok().filter(|id| (1..=14).contains(id));
        }
    }
    None
}

/// One text message (the page receives a string), for the JSON reports.
fn send_text(dc: c_int, text: &str) {
    let Ok(text) = CString::new(text) else {
        return;
    };
    // SAFETY: a negative size is libdatachannel's "NUL-terminated string"; `text` is live for the
    // call.
    unsafe { rtcSendMessage(dc, text.as_ptr(), -1) };
}

/// Wall-clock milliseconds since the Unix epoch, the clock the page converts its own into.
fn unix_ms() -> f64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs_f64() * 1000.0)
        .unwrap_or(0.0)
}

/// One binary message on a DataChannel. A channel that has closed since makes this fail, which is
/// all the handling a departed viewer needs.
fn send_binary(dc: c_int, m: &[u8]) {
    // SAFETY: `m` is live for the call; a stale `dc` is rejected by libdatachannel.
    unsafe { rtcSendMessage(dc, m.as_ptr() as *const c_char, m.len() as c_int) };
}

fn find(haystack: &[u8], needle: &[u8]) -> Option<usize> {
    haystack.windows(needle.len()).position(|w| w == needle)
}

fn respond(stream: &mut TcpStream, status: &str, kind: &str, body: &[u8]) -> std::io::Result<()> {
    respond_with(stream, status, kind, &[], body)
}

fn respond_with(
    stream: &mut TcpStream,
    status: &str,
    kind: &str,
    headers: &[(&str, String)],
    body: &[u8],
) -> std::io::Result<()> {
    let mut head = format!(
        "HTTP/1.1 {}\r\nContent-Type: {}\r\nContent-Length: {}\r\nCache-Control: no-store\r\n\
         Connection: close\r\n",
        status,
        kind,
        body.len()
    );
    for (name, value) in headers {
        head.push_str(&format!("{}: {}\r\n", name, value));
    }
    head.push_str("\r\n");
    stream.write_all(head.as_bytes())?;
    stream.write_all(body)
}

fn rand_u32() -> u32 {
    let mut v = [0u8; 4];
    // SAFETY: getrandom writes at most `len` bytes into the buffer.
    let n = unsafe { libc::getrandom(v.as_mut_ptr() as *mut c_void, v.len(), 0) };
    if n != 4 {
        return Instant::now().elapsed().subsec_nanos() ^ 0x5eed_1234;
    }
    u32::from_le_bytes(v)
}
