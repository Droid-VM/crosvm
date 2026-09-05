// Copyright 2026 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! The MediaCodec NDK behind the virtio-media video decoder device (`VPU_DESIGN.md` §7.2).
//!
//! [`MediaCodecDecoderBackend`] is the platform's list of hardware video decoders, read once from
//! `AMediaCodecStore` through the `android_codec` crate and turned into the
//! `DecoderCapabilities` the device answers `ENUM_FMT` / `ENUM_FRAMESIZES` from: nothing about
//! what the phone can decode is written down here. [`MediaCodecDecoderSession`] is one decode: an
//! `AMediaCodec` in asynchronous mode, created at `STREAMON(OUTPUT)`, fed the guest's bitstream
//! buffers and emptied into the guest's `CAPTURE` buffers.
//!
//! # Threads
//!
//! ```text
//!  device worker thread                                     NDK callback thread
//!  ────────────────────                                     ───────────────────
//!  decode(InputBuffer)     copy into getInputBuffer +       onAsyncInputAvailable  ─┐ push onto the
//!                          queueInputBuffer, else FIFO      onAsyncOutputAvailable  │ Codec's queue,
//!  use_as_capture(buf)     FIFO of lent CAPTURE buffers     onAsyncFormatChanged    │ then bump the
//!  take_events()           drain the Codec's queue: feed    onAsyncError           ─┘ session eventfd
//!                          inputs, copy outputs into lent                             (the wake hook)
//!                          buffers, releaseOutputBuffer
//!  start / flush / stop    on a short-lived thread, waited for at most a named bound
//! ```
//!
//! Everything that touches a guest or pool buffer happens on the worker thread, inside the trait
//! calls: the four NDK callbacks only queue an event and bump the device session's eventfd
//! (`android_codec`'s wake hook), so there is no capture thread and nothing to join for §2.5.
//! Once [`VideoDecoderBackendSession::flush`],
//! [`VideoDecoderBackendSession::clear_capture_buffers`] or [`VideoDecoderBackendSession::stop`]
//! has emptied the session's own FIFOs, no code path can write a lent buffer any more.
//!
//! What can block is the NDK itself: `createCodecByName` + `configure` + `start`, `flush` +
//! `start`, and `stop` + `delete` are synchronous calls into the codec service and the component
//! with no timeout of their own. Each runs on a thread of its own and the worker waits at most
//! [`CODEC_START_TIMEOUT`], [`CODEC_FLUSH_TIMEOUT`] or [`CODEC_STOP_TIMEOUT`] for it -- the rule
//! `logs/vpu_wp/F5-crosvm.md` §2 set for the camera; past the bound the codec is abandoned to
//! that thread, which deletes it when the call returns, and the session is over.
//! `queueInputBuffer`, `getOutputBuffer`, `getBufferFormat` and `releaseOutputBuffer` are looper
//! round-trips that do not wait on the hardware, and are called in place.
//!
//! # Buffers
//!
//! * A bitstream buffer is **copied** into the codec's input buffer as soon as an input slot is
//!   free, and `InputBufferDone` follows at once, so the guest gets its buffer back right after the
//!   copy (design §7.2); with no slot free it waits in a FIFO, still lent. An access unit larger
//!   than the codec's input buffer goes in pieces with `BUFFER_FLAG_PARTIAL_FRAME`. Parameter sets
//!   alone (H.264 / HEVC, told by `android_codec::bitstream` after the copy, on the codec's own
//!   memory) go as `BUFFER_FLAG_CODEC_CONFIG`.
//! * A decoded output is copied into the next lent `CAPTURE` buffer as tightly packed NV12 of the
//!   announced coded size (`tight_nv12_rows` over the buffer's `MediaImage2`, written through the
//!   raw pointer -- never a slice over guest-visible memory), then released to the codec. With no
//!   `CAPTURE` buffer lent, or none large enough, the output index is held, not released, until one
//!   arrives.
//! * Seek ([`VideoDecoderBackendSession::flush`]) is `AMediaCodec_flush` then `start` (the async
//!   rule); every pending input and held output is dropped. Drain is an empty `END_OF_STREAM`
//!   input; the output that carries the flag becomes the guest's `LAST` buffer. After an EOS the
//!   codec accepts no input until it is flushed, which the next `decode` does.

use std::collections::VecDeque;
use std::sync::mpsc;
use std::sync::mpsc::RecvTimeoutError;
use std::thread;
use std::time::Duration;
use std::time::Instant;

use android_codec::bitstream::annexb_access_units;
use android_codec::bitstream::NalCodec;
use android_codec::color_format_name;
use android_codec::image::nv12_size;
use android_codec::keys;
use android_codec::list_codecs;
use android_codec::media_status_name;
use android_codec::tight_nv12_rows;
use android_codec::BufferInfo;
use android_codec::Codec;
use android_codec::CodecError;
use android_codec::CodecEvent;
use android_codec::CodecInfo;
use android_codec::CodecKind;
use android_codec::CodecType;
use android_codec::ListOptions;
use android_codec::MediaFormat;
use android_codec::MediaImage2;
use android_codec::BUFFER_FLAG_CODEC_CONFIG;
use android_codec::BUFFER_FLAG_END_OF_STREAM;
use android_codec::BUFFER_FLAG_PARTIAL_FRAME;
use android_codec::COLOR_FORMAT_YUV420_FLEXIBLE;
use anyhow::Context;
use base::debug;
use base::error;
use base::info;
use base::warn;
use virtio_media::devices::video_decoder::CodedFormat;
use virtio_media::devices::video_decoder::DecoderCapabilities;
use virtio_media::devices::video_decoder::DecoderEvent;
use virtio_media::devices::video_decoder::DecoderSink;
use virtio_media::devices::video_decoder::InputBuffer;
use virtio_media::devices::video_decoder::OutputBuffer;
use virtio_media::devices::video_decoder::SizeRange;
use virtio_media::devices::video_decoder::VideoDecoderBackend;
use virtio_media::devices::video_decoder::VideoDecoderBackendSession;
use virtio_media::ioctl::IoctlResult;
use virtio_media::v4l2r::bindings;
use virtio_media::v4l2r::PixelFormat;
use virtio_media::v4l2r::Rect;

/// How long `STREAMON(OUTPUT)` waits for `createCodecByName` + `configure` + `start`. Creating a
/// codec is a round of binder into `media.codec` and the vendor HAL, normally well under a
/// second; past this the guest's ioctl answers `ETIMEDOUT` rather than the worker staying parked.
pub(super) const CODEC_START_TIMEOUT: Duration = Duration::from_secs(10);
/// How long a seek (`STREAMOFF(OUTPUT)`) waits for `AMediaCodec_flush` + `start`: a flush waits
/// for the component to give every buffer back, normally milliseconds.
pub(super) const CODEC_FLUSH_TIMEOUT: Duration = Duration::from_secs(5);
/// How long a session teardown waits for `AMediaCodec_stop` + `delete` before detaching the
/// thread that runs them. Nothing of the guest's is at stake by then (the FIFOs are already
/// empty); the bound only keeps `REQBUFS(0)` / close from waiting on a wedged component.
pub(super) const CODEC_STOP_TIMEOUT: Duration = Duration::from_secs(5);
/// Floor of `KEY_MAX_INPUT_SIZE`, the codec's input buffer capacity. The device's own floor for
/// an `OUTPUT` buffer is the same 1 MiB, so an ordinary access unit goes in whole.
const MIN_INPUT_BUFFER_SIZE: usize = 1 << 20;
/// `V4L2_CID_MIN_BUFFERS_FOR_CAPTURE`, announced with every format change. Outputs are copied
/// out and released to the codec at once, so the codec's own reference frames never pin a
/// `CAPTURE` buffer: a small number keeps the pipeline moving (GStreamer adds its own extras,
/// ffmpeg allocates 20 regardless). Design §7.2 says "4 + a conservative margin"; the margin is
/// 0 for the reason above.
const MIN_CAPTURE_BUFFERS: u32 = 4;
/// The size range offered for a codec whose `VideoCapabilities` the platform does not publish
/// (the no-Store fallback of `android_codec::list_codecs`). Logged when used.
const FALLBACK_SIZE_RANGE: SizeRange = SizeRange::new(16, 4096, 2);
/// How many refused input / output indices are logged one by one after a seek; the rest are
/// counted. A callback posted before `AMediaCodec_flush` can still be delivered after it, with
/// an index that no longer means anything (`logs/vpu_wp/M6-probe.md` §9 item 5): such an index
/// answers `getInputBuffer` / `getOutputBuffer` with null, which is skipped at debug level and
/// never counted against this limit (`logs/vpu_wp/B5-acceptance.md` D23 -- the HEVC decoder
/// delivers one after most seeks). What is counted here is a *refusal*: an NDK status from
/// `queueInputBuffer` / `releaseOutputBuffer` on an index the codec did hand out.
pub(super) const STALE_LOG_LIMIT: u32 = 3;
/// More refused indices than this since the last seek is not staleness but a codec that refuses
/// everything; the session ends rather than spinning.
pub(super) const MAX_REFUSED_INDICES: u32 = 64;

/// The coded formats a decoder device can offer, in `ENUM_FMT(OUTPUT)` order (and an encoder
/// device in `ENUM_FMT(CAPTURE)` order), and the MediaCodec mime each stands for (design §7.2,
/// §7.3). Whether one is offered is decided by the codec store.
pub(super) const CODED_FORMATS: [(&[u8; 4], &str); 5] = [
    (b"H264", "video/avc"),
    (b"HEVC", "video/hevc"),
    (b"VP80", "video/x-vnd.on2.vp8"),
    (b"VP90", "video/x-vnd.on2.vp9"),
    (b"AV10", "video/av01"),
];

/// The `FEATURE_*` strings that matter for the choice (`NdkMediaCodecInfo.h`,
/// `AMediaCodecInfo_FEATURE_SecurePlayback` / `_LowLatency`).
pub(super) const FEATURE_SECURE_PLAYBACK: &str = "secure-playback";
const FEATURE_LOW_LATENCY: &str = "low-latency";

/// One codec the backend picked for a coded format: what `STREAMON(OUTPUT)` creates.
#[derive(Clone, Debug)]
struct ChosenCodec {
    fourcc: PixelFormat,
    mime: String,
    /// The canonical name, the one `AMediaCodec_createCodecByName` takes.
    name: String,
    hardware: bool,
    vendor: bool,
    /// The component supports `FEATURE_LowLatency`, so `KEY_LOW_LATENCY` is set at configure.
    low_latency: bool,
}

/// The platform's hardware decoders as `DecoderCapabilities`, enumerated once. Cheap to clone:
/// the device factory builds a device from it on the worker thread at every start.
#[derive(Clone, Debug)]
pub struct MediaCodecDecoderBackend {
    caps: DecoderCapabilities,
    /// Parallel to `caps.coded_formats`.
    codecs: Vec<ChosenCodec>,
}

impl MediaCodecDecoderBackend {
    /// Enumerate the decoders and choose one per coded format. Loads the NDK, which starts this
    /// process's binder thread pool, and walks the codec store's lazily built tables, which have
    /// no lock: call it once, from one thread, after the uid drop (design §7.2).
    ///
    /// Software decoders are left out unless `allow_sw`; a format with no eligible decoder is
    /// simply not offered. A platform with no eligible decoder at all still gets its device,
    /// with no coded format (`ENUM_FMT` lists nothing and the device refuses every session with
    /// `ENODEV`): the alternative, a helper that exits, is a VM that does not boot, which the
    /// app cannot foresee because it has no way to ask the codec store from the daemon's uid
    /// (`logs/vpu_wp/A4.md` §10 item 2).
    pub fn new(allow_sw: bool) -> anyhow::Result<Self> {
        let list = list_codecs(&ListOptions {
            include_non_video: false,
            // Profiles and per-size rates are M5's; a first STREAMON must not wait for them.
            probe_profiles: false,
            sizes: Vec::new(),
        })
        .context("cannot enumerate the platform's codecs")?;
        if !list.missing_symbols.is_empty() {
            warn!(
                "decoder: {} optional NDK symbol(s) missing; codecs listed by {}",
                list.missing_symbols.len(),
                list.source
            );
        }
        let decoders = list
            .codecs
            .iter()
            .filter(|c| c.kind == CodecKind::Decoder)
            .count();

        let mut caps = DecoderCapabilities::default();
        let mut codecs = Vec::new();
        for (fourcc, mime) in CODED_FORMATS {
            let fourcc = PixelFormat::from_fourcc(fourcc);
            match choose(&list.codecs, CodecKind::Decoder, mime, allow_sw) {
                Some((chosen, passed_over)) => {
                    let (width, height) = size_ranges(chosen, fourcc);
                    info!(
                        "decoder: {} -> {} ({}, {}{}), {}..{} x {}..{} step {}x{}, {} fps; \
                         passed over: {}",
                        fourcc,
                        chosen.name,
                        mime,
                        if is_hardware(chosen) {
                            "hardware"
                        } else {
                            "software"
                        },
                        if chosen.is_vendor { ", vendor" } else { "" },
                        width.min,
                        width.max,
                        height.min,
                        height.max,
                        width.step,
                        height.step,
                        chosen
                            .video
                            .as_ref()
                            .map(|v| format!("{}..{}", v.frame_rates.0, v.frame_rates.1))
                            .unwrap_or_else(|| "?".to_string()),
                        if passed_over.is_empty() {
                            "none".to_string()
                        } else {
                            passed_over.join(", ")
                        }
                    );
                    caps.coded_formats.push(CodedFormat {
                        fourcc,
                        width,
                        height,
                        // Every one of these codecs carries its resolution in the bitstream.
                        dynamic_resolution: true,
                    });
                    codecs.push(ChosenCodec {
                        fourcc,
                        mime: mime.to_string(),
                        name: chosen.name.clone(),
                        hardware: is_hardware(chosen),
                        vendor: chosen.is_vendor,
                        low_latency: has_feature(chosen, FEATURE_LOW_LATENCY),
                    });
                }
                None => {
                    let software: Vec<&str> = list
                        .codecs
                        .iter()
                        .filter(|c| c.kind == CodecKind::Decoder && c.mime == mime)
                        .map(|c| c.name.as_str())
                        .collect();
                    if software.is_empty() {
                        info!("decoder: {} has no decoder on this platform", fourcc);
                    } else {
                        info!(
                            "decoder: {} has no usable hardware decoder (only {}; \
                             allow_sw=true would include software)",
                            fourcc,
                            software.join(", ")
                        );
                    }
                }
            }
        }
        if caps.coded_formats.is_empty() {
            warn!(
                "decoder: no usable video decoder: the codec store ({}) lists {} decoder(s), \
                 none of them hardware for {} (allow_sw={}); the device is served with no coded \
                 format, and every session on it is refused with ENODEV",
                list.source,
                decoders,
                CODED_FORMATS
                    .iter()
                    .map(|(_, mime)| *mime)
                    .collect::<Vec<_>>()
                    .join(", "),
                allow_sw
            );
            return Ok(Self { caps, codecs });
        }
        info!(
            "decoder: {} coded format(s) for the guest from {} ({} decoder(s) listed, \
             allow_sw={}): {}",
            caps.coded_formats.len(),
            list.source,
            decoders,
            allow_sw,
            codecs
                .iter()
                .map(|c| format!("{} {}", c.fourcc, c.name))
                .collect::<Vec<_>>()
                .join(", ")
        );
        Ok(Self { caps, codecs })
    }
}

pub(super) fn has_feature(info: &CodecInfo, feature: &str) -> bool {
    info.features
        .iter()
        .any(|f| f.feature == feature && f.supported)
}

pub(super) fn is_hardware(info: &CodecInfo) -> bool {
    match info.codec_type {
        CodecType::HardwareAccelerated => true,
        CodecType::SoftwareOnly | CodecType::SoftwareWithDeviceAccess | CodecType::Invalid => false,
        // No codec store on this platform: judge by the name, as the crate's fallback does for
        // `is_vendor`.
        CodecType::Unknown => {
            !(info.name.starts_with("c2.android.") || info.name.starts_with("OMX.google."))
        }
    }
}

/// How much a codec of `kind` is wanted for a format: vendor hardware first, then any hardware,
/// then -- only with `allow_sw` -- software. `None` is "not eligible". The encoder backend
/// applies the same rule (design §7.3: "the same preference rule as the decoder").
fn rank(info: &CodecInfo, kind: CodecKind, mime: &str, allow_sw: bool) -> Option<u8> {
    if info.kind != kind || info.mime != mime {
        return None;
    }
    // A decoder that *requires* secure playback decodes DRM content into buffers nobody may
    // read (and a secure encoder reads from buffers nobody may write): never usable here. The
    // name check covers a platform without the feature strings.
    if info
        .features
        .iter()
        .any(|f| f.required && f.feature == FEATURE_SECURE_PLAYBACK)
        || info.name.ends_with(".secure")
    {
        return None;
    }
    match (is_hardware(info), info.is_vendor) {
        (true, true) => Some(3),
        (true, false) => Some(2),
        (false, _) if allow_sw => Some(1),
        _ => None,
    }
}

/// The codec of `kind` for `mime`, and the names of the eligible ones it was preferred to.
/// Among equal ranks the shorter canonical name wins (the base component before its
/// `.low_latency`, `.cq` and other variants), then the store's own order, which lists the
/// platform's preferred codec first.
pub(super) fn choose<'a>(
    codecs: &'a [CodecInfo],
    kind: CodecKind,
    mime: &str,
    allow_sw: bool,
) -> Option<(&'a CodecInfo, Vec<String>)> {
    let mut eligible: Vec<(u8, usize, &CodecInfo)> = codecs
        .iter()
        .enumerate()
        .filter_map(|(order, c)| rank(c, kind, mime, allow_sw).map(|r| (r, order, c)))
        .collect();
    eligible.sort_by(|a, b| {
        b.0.cmp(&a.0)
            .then(a.2.name.len().cmp(&b.2.name.len()))
            .then(a.1.cmp(&b.1))
    });
    let mut it = eligible.into_iter();
    let (_, _, chosen) = it.next()?;
    let passed_over = it
        .map(|(_, _, c)| {
            if is_hardware(c) {
                c.name.clone()
            } else {
                format!("{} (software)", c.name)
            }
        })
        .collect();
    Some((chosen, passed_over))
}

/// `ENUM_FRAMESIZES` for a codec: the store's width / height ranges with the alignment as the
/// step, or a documented fallback when the platform publishes none.
fn size_ranges(info: &CodecInfo, fourcc: PixelFormat) -> (SizeRange, SizeRange) {
    let range = |lo: i32, hi: i32, step: i32| {
        let lo = lo.max(1) as u32;
        let hi = (hi.max(lo as i32)) as u32;
        SizeRange::new(lo, hi, step.max(1) as u32)
    };
    match &info.video {
        Some(v) => (
            range(v.widths.0, v.widths.1, v.width_alignment),
            range(v.heights.0, v.heights.1, v.height_alignment),
        ),
        None => {
            warn!(
                "decoder: {} ({}) publishes no video capabilities; offering {}..{} step {} \
                 in both dimensions",
                fourcc,
                info.name,
                FALLBACK_SIZE_RANGE.min,
                FALLBACK_SIZE_RANGE.max,
                FALLBACK_SIZE_RANGE.step
            );
            (FALLBACK_SIZE_RANGE, FALLBACK_SIZE_RANGE)
        }
    }
}

impl VideoDecoderBackend for MediaCodecDecoderBackend {
    type Session = MediaCodecDecoderSession;

    fn capabilities(&self) -> &DecoderCapabilities {
        &self.caps
    }

    fn new_session(&mut self, id: u32, sink: DecoderSink) -> IoctlResult<MediaCodecDecoderSession> {
        Ok(MediaCodecDecoderSession::new(id, sink, self.codecs.clone()))
    }

    fn close_session(&mut self, mut session: MediaCodecDecoderSession) {
        session.stop();
    }
}

// ---------------------------------------------------------------------------------------------
// The session
// ---------------------------------------------------------------------------------------------

/// What the codec delivered and the session has not passed on yet, in delivery order: an output
/// buffer, or a format change. A format change is announced only once every output delivered
/// before it has gone to the guest -- a mid-stream resolution change arrives between the last
/// old-size frame and the first new-size one, and an old frame must not be laid out for the
/// new size.
enum Held {
    Output { index: i32, info: BufferInfo },
    Format(Option<MediaFormat>),
}

/// A bitstream buffer waiting for a codec input slot, or the end of the stream.
enum PendingInput {
    Bitstream {
        buffer: InputBuffer,
        /// Bytes already handed to the codec, when the access unit goes in pieces.
        offset: usize,
    },
    Eos,
}

/// What the codec announced, kept for the copy and for the log.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Announced {
    coded: (u32, u32),
    visible: (i32, i32, u32, u32),
}

/// The errno a guest's ioctl gets for a codec call that failed.
pub(super) fn errno_for(e: &CodecError) -> i32 {
    match e {
        CodecError::LibraryLoad(..) | CodecError::MissingSymbol(_) => libc::ENOSYS,
        // createCodecByName gave nothing: the name the store gave does not resolve, or the
        // codec service is gone.
        CodecError::Null("AMediaCodec_createCodecByName") => libc::ENODEV,
        // AMEDIACODEC_ERROR_INSUFFICIENT_RESOURCE, AMEDIACODEC_ERROR_RECLAIMED
        CodecError::Ndk(_, 1100 | 1101, _) => libc::EBUSY,
        // AMEDIA_ERROR_UNSUPPORTED, AMEDIA_ERROR_INVALID_PARAMETER
        CodecError::Ndk(_, -10002 | -10004, _) => libc::EINVAL,
        _ => libc::EIO,
    }
}

/// A V4L2 timestamp as the microsecond presentation time MediaCodec carries through a frame.
pub(super) fn pts_from(ts: bindings::timeval) -> u64 {
    (ts.tv_sec as i64)
        .wrapping_mul(1_000_000)
        .wrapping_add(ts.tv_usec as i64) as u64
}

/// The way back: a presentation time as a V4L2 timestamp (`V4L2_BUF_FLAG_TIMESTAMP_COPY`).
pub(super) fn timeval_from(pts_us: i64) -> bindings::timeval {
    bindings::timeval {
        tv_sec: pts_us.div_euclid(1_000_000) as _,
        tv_usec: pts_us.rem_euclid(1_000_000) as _,
    }
}

/// `KEY_MAX_INPUT_SIZE` for a coded size: ffmpeg's own sizing of its bitstream buffers
/// (`v4l2_get_framesize_compressed`, `(w*h*3/2)/2 + 128`) with the device's 1 MiB floor, so a
/// buffer the guest was allowed to queue fits the codec's in one piece.
fn max_input_size(coded: (u32, u32)) -> usize {
    ((coded.0 as usize * coded.1 as usize * 3 / 2) / 2 + 128).max(MIN_INPUT_BUFFER_SIZE)
}

/// Run `op` on a thread of its own and wait at most `timeout` for what it returns. `None` is a
/// timeout (or a thread that could not be started): the thread is then detached, and whatever
/// `op` owns -- the codec -- is dropped by that thread when the call finally returns, which is
/// the only way the platform gets the codec back. The value `op` produces after a timeout is
/// dropped there too. `who` names the caller in the log (`decoder` / `encoder`).
pub(super) fn bounded<T: Send + 'static>(
    who: &'static str,
    session: u32,
    what: &'static str,
    timeout: Duration,
    op: impl FnOnce() -> T + Send + 'static,
) -> Option<T> {
    let (tx, rx) = mpsc::sync_channel::<T>(1);
    let thread = match thread::Builder::new()
        .name(format!("v_codec_{session}"))
        .spawn(move || {
            // A receiver that gave up drops what is sent here, on this thread.
            let _ = tx.send(op());
        }) {
        Ok(thread) => thread,
        Err(e) => {
            error!(
                "{} session {}: cannot start a thread for {}: {}",
                who, session, what, e
            );
            return None;
        }
    };
    match rx.recv_timeout(timeout) {
        Ok(value) => {
            // It has sent, so it is finishing: this join is bounded.
            if thread.join().is_err() {
                error!("{} session {}: the {} thread panicked", who, session, what);
            }
            Some(value)
        }
        Err(RecvTimeoutError::Timeout) => {
            error!(
                "{} session {}: {} did not return within {:?}; the codec is abandoned to its \
                 thread and the session ends",
                who, session, what, timeout
            );
            None
        }
        Err(RecvTimeoutError::Disconnected) => {
            error!(
                "{} session {}: the {} thread ended without an answer",
                who, session, what
            );
            None
        }
    }
}

/// One decode: see the module documentation.
pub struct MediaCodecDecoderSession {
    id: u32,
    sink: DecoderSink,
    codecs: Vec<ChosenCodec>,
    /// The codec, from `start` to `stop`.
    codec: Option<Codec>,
    chosen: Option<ChosenCodec>,
    /// Set for H.264 / HEVC: how to tell a parameter-set-only buffer.
    nal: Option<NalCodec>,
    /// The coded size the codec announced (`FormatChanged`), which is what every `CAPTURE`
    /// buffer is laid out for; `None` before the stream is parsed.
    announced: Option<Announced>,
    format_changes: u32,
    /// Input indices the codec offered and nothing has used yet.
    free_inputs: VecDeque<i32>,
    /// Bitstream buffers waiting for an input index, oldest first.
    pending: VecDeque<PendingInput>,
    /// `CAPTURE` buffers lent by the device, oldest first.
    captures: VecDeque<OutputBuffer>,
    /// Outputs the codec delivered that no `CAPTURE` buffer has taken yet, and the format
    /// changes in between them.
    held_outputs: VecDeque<Held>,
    /// For the device, in order.
    events: Vec<DecoderEvent>,
    /// An `END_OF_STREAM` input is queued: the codec takes no more input until a flush.
    eos_queued: bool,
    /// The `END_OF_STREAM` output was delivered as `LAST`.
    eos_seen: bool,
    /// The codec is gone (error, timeout); the device has been or is being told.
    dead: bool,
    input_capacity: Option<usize>,
    layout_logged: bool,
    /// Indices refused by the codec since the last seek, stale indices (a null buffer for an
    /// index a flush made void, D23) since the last seek, and seeks so far.
    refused: u32,
    stale: u32,
    /// `Codec::stale_events` as of the last seek: what the crate's generation stamp dropped.
    stale_events_seen: u64,
    seeks: u32,
    inputs: u64,
    frames: u64,
    started_at: Option<Instant>,
}

impl MediaCodecDecoderSession {
    fn new(id: u32, sink: DecoderSink, codecs: Vec<ChosenCodec>) -> Self {
        Self {
            id,
            sink,
            codecs,
            codec: None,
            chosen: None,
            nal: None,
            announced: None,
            format_changes: 0,
            free_inputs: VecDeque::new(),
            pending: VecDeque::new(),
            captures: VecDeque::new(),
            held_outputs: VecDeque::new(),
            events: Vec::new(),
            eos_queued: false,
            eos_seen: false,
            dead: false,
            input_capacity: None,
            layout_logged: false,
            refused: 0,
            stale: 0,
            stale_events_seen: 0,
            seeks: 0,
            inputs: 0,
            frames: 0,
            started_at: None,
        }
    }

    fn codec_name(&self) -> &str {
        self.chosen.as_ref().map(|c| c.name.as_str()).unwrap_or("-")
    }

    /// The session is over: the device hears `Error` (and ends it), nothing is lent any more.
    fn fail(&mut self, why: String) {
        if self.dead {
            return;
        }
        error!("decoder session {}: {}", self.id, why);
        self.dead = true;
        self.pending.clear();
        self.captures.clear();
        self.held_outputs.clear();
        self.free_inputs.clear();
        self.events.push(DecoderEvent::Error(why));
        self.sink.signal();
    }

    /// A refused input or output index. A null buffer (`CodecError::Null` from `getInputBuffer`
    /// / `getOutputBuffer`) is a stale index -- a callback the NDK looper had queued before a
    /// flush and delivered after it (D23) -- and is skipped: counted for the seek line, a debug
    /// line, never a session error. Anything else is the codec refusing an index it handed out,
    /// which is tolerated up to [`MAX_REFUSED_INDICES`] per seek.
    fn refused_index(&mut self, what: &str, index: i32, e: &CodecError) {
        if matches!(e, CodecError::Null(_)) {
            self.stale += 1;
            debug!(
                "decoder session {}: stale {} index {} after a flush ignored ({})",
                self.id, what, index, e
            );
            return;
        }
        self.refused += 1;
        if self.refused <= STALE_LOG_LIMIT {
            warn!(
                "decoder session {}: {} index {} refused: {}",
                self.id, what, index, e
            );
        }
        if self.refused > MAX_REFUSED_INDICES {
            self.fail(format!(
                "{} refused {} indices since the last seek; the codec is not taking buffers",
                self.codec_name(),
                self.refused
            ));
        }
    }

    /// Feed the codec from the pending FIFO while it offers input slots.
    fn pump_input(&mut self) {
        while !self.dead && !self.eos_queued {
            let (Some(&index), Some(head)) = (self.free_inputs.front(), self.pending.front_mut())
            else {
                break;
            };
            let Some(codec) = self.codec.as_ref() else {
                break;
            };
            match head {
                PendingInput::Eos => match codec.queue_eos(index, 0) {
                    Ok(()) => {
                        self.free_inputs.pop_front();
                        self.pending.pop_front();
                        self.eos_queued = true;
                        info!(
                            "decoder session {}: drain: EOS queued after {} bitstream buffers",
                            self.id, self.inputs
                        );
                    }
                    Err(e) => {
                        self.free_inputs.pop_front();
                        self.refused_index("input", index, &e);
                    }
                },
                PendingInput::Bitstream { buffer, offset } => {
                    let buffer = *buffer;
                    let start = *offset;
                    let remaining = buffer.len - start;
                    let nal = self.nal;
                    let mut copied = 0usize;
                    let result =
                        codec.queue_input_with_flags(index, pts_from(buffer.timestamp), |dst| {
                            let n = remaining.min(dst.len());
                            // SAFETY: the device lends `len` readable bytes at `ptr` until
                            // `InputBufferDone`, which is only reported below; `dst` is the codec's
                            // own input buffer, `n` bytes of which exist; the two never overlap.
                            unsafe {
                                std::ptr::copy_nonoverlapping(
                                    buffer.ptr.as_ptr().add(start),
                                    dst.as_mut_ptr(),
                                    n,
                                )
                            };
                            copied = n;
                            let mut flags = 0;
                            if n < remaining {
                                // The rest follows in the next input buffer(s).
                                flags |= BUFFER_FLAG_PARTIAL_FRAME;
                            }
                            // Parameter sets on their own (a `codec_data` buffer, or SPS/PPS a
                            // client sends ahead of the first picture) are config, not a frame;
                            // decided on the codec's copy, never on the guest's memory.
                            if start == 0 && n == remaining {
                                if let Some(nal) = nal {
                                    if let Ok(units) = annexb_access_units(&dst[..n], nal) {
                                        if !units.is_empty()
                                            && units.iter().all(|u| !u.has_picture)
                                            && units.iter().any(|u| u.has_parameter_sets)
                                        {
                                            flags |= BUFFER_FLAG_CODEC_CONFIG;
                                        }
                                    }
                                }
                            }
                            Ok((n, flags))
                        });
                    match result {
                        Ok(()) => {
                            self.free_inputs.pop_front();
                            if let Some(PendingInput::Bitstream { offset, .. }) =
                                self.pending.front_mut()
                            {
                                *offset += copied;
                            }
                            if start + copied >= buffer.len {
                                self.pending.pop_front();
                                self.inputs += 1;
                                // The bitstream is in the codec's buffer: the guest's goes back.
                                self.events
                                    .push(DecoderEvent::InputBufferDone(buffer.index));
                            }
                        }
                        Err(e) => {
                            self.free_inputs.pop_front();
                            self.refused_index("input", index, &e);
                        }
                    }
                }
            }
        }
    }

    /// Copy every held output into a lent `CAPTURE` buffer that can take it.
    fn pump_output(&mut self) {
        // The codec is taken out of `self` for the duration: `output_buffer` borrows it, and the
        // helpers below need `self`. It goes back at the end, dead or not, so `stop` can stop it.
        let Some(mut codec) = self.codec.take() else {
            return;
        };
        while !self.dead {
            let (index, info) = match self.held_outputs.front() {
                Some(Held::Output { index, info }) => (*index, *info),
                Some(Held::Format(_)) => {
                    // Every output before it is out: the new size can be announced.
                    let Some(Held::Format(format)) = self.held_outputs.pop_front() else {
                        unreachable!("checked above");
                    };
                    self.handle_format(format);
                    continue;
                }
                None => break,
            };
            let is_eos = info.flags & BUFFER_FLAG_END_OF_STREAM != 0;
            let is_config = info.flags & BUFFER_FLAG_CODEC_CONFIG != 0;
            if info.size <= 0 || is_config {
                // Nothing to copy. An empty EOS is the guest's empty LAST buffer, which needs a
                // CAPTURE buffer to ride on; an empty non-EOS output (or a config output, which
                // a decoder does not produce) is just given back.
                if !is_eos {
                    if let Err(e) = codec.release_output(index) {
                        self.refused_index("output", index, &e);
                    }
                    self.held_outputs.pop_front();
                    continue;
                }
                let Some(capture) = self.captures.pop_front() else {
                    break;
                };
                if let Err(e) = codec.release_output(index) {
                    self.refused_index("output", index, &e);
                }
                self.held_outputs.pop_front();
                self.finish_frame(capture.index, 0, info, true);
                continue;
            }

            // The frame. Its layout comes with the buffer; the canvas it is written into is the
            // announced coded size.
            let src = match codec.output_buffer(index) {
                Ok(src) => src,
                Err(e) => {
                    self.held_outputs.pop_front();
                    self.refused_index("output", index, &e);
                    continue;
                }
            };
            let format = match codec.buffer_format(index) {
                Ok(format) => format,
                Err(e) => {
                    self.held_outputs.pop_front();
                    self.refused_index("output", index, &e);
                    continue;
                }
            };
            let image = match self.image_for(&format, src.len()) {
                Ok(image) => image,
                Err(why) => {
                    self.fail(why);
                    break;
                }
            };
            let canvas = match self.canvas_for(&image) {
                Ok(canvas) => canvas,
                Err(why) => {
                    self.fail(why);
                    break;
                }
            };
            let need = nv12_size(canvas.0 as usize, canvas.1 as usize);
            // The first lent buffer that holds a frame of the announced size. A smaller one --
            // lent for the placeholder size before the SOURCE_CHANGE -- stays lent and unfilled
            // until the guest takes it back with STREAMOFF(CAPTURE) and reallocates; the frame
            // waits with it.
            let Some(at) = self.captures.iter().position(|c| c.len >= need) else {
                break;
            };
            let capture = self.captures.remove(at).expect("position was found above");
            let out = capture.ptr.as_ptr();
            let (cw, w, h) = (
                canvas.0 as usize,
                image.width as usize,
                image.height as usize,
            );
            let chroma_base = cw * canvas.1 as usize;
            let mut row = 0usize;
            let copied = tight_nv12_rows(&image, src, |bytes| {
                let (at, width) = if row < h {
                    (row * cw, w)
                } else {
                    (chroma_base + (row - h) * cw, cw)
                };
                let n = bytes.len().min(width);
                // SAFETY: `at + n <= need <= capture.len`, and the device lends `len` writable
                // bytes at `ptr` until `FrameDecoded`; `bytes` is a row of the codec's buffer or
                // a scratch row; the two never overlap.
                unsafe { std::ptr::copy_nonoverlapping(bytes.as_ptr(), out.add(at), n) };
                row += 1;
            });
            if let Err(e) = copied {
                self.fail(format!(
                    "cannot repack output {} ({}x{}) into NV12: {}",
                    index, image.width, image.height, e
                ));
                break;
            }
            // The codec gets its buffer back before the guest hears about the frame.
            if let Err(e) = codec.release_output(index) {
                self.refused_index("output", index, &e);
            }
            self.held_outputs.pop_front();
            self.frames += 1;
            self.finish_frame(capture.index, need as u32, info, is_eos);
        }
        self.codec = Some(codec);
    }

    fn finish_frame(
        &mut self,
        capture_index: u32,
        bytesused: u32,
        info: BufferInfo,
        is_last: bool,
    ) {
        self.events.push(DecoderEvent::FrameDecoded {
            index: capture_index,
            bytesused,
            timestamp: timeval_from(info.presentation_time_us),
            is_last,
        });
        if is_last {
            self.eos_seen = true;
            info!(
                "decoder session {}: EOS reached after {} frames ({} bytes in the LAST buffer)",
                self.id, self.frames, bytesused
            );
        }
    }

    /// The `MediaImage2` of one output buffer, from its `image-data` or -- when a component
    /// publishes none -- the stride / slice-height idiom of its format; the `mPlane[0].mOffset`
    /// question (`M6-probe` §9 item 1) is answered by what fits the buffer. Logged once.
    fn image_for(&mut self, format: &MediaFormat, buf_len: usize) -> Result<MediaImage2, String> {
        let width = format.get_i32(keys::WIDTH).unwrap_or(0).max(0) as u32;
        let height = format.get_i32(keys::HEIGHT).unwrap_or(0).max(0) as u32;
        let (image, verdict) = match format.image_data() {
            Some(Ok(raw)) => {
                if raw.planes[0].offset == 0 {
                    (raw, "offset 0: offsets as written")
                } else if raw.required_size() <= buf_len {
                    (raw, "both fit the buffer: offsets as written")
                } else if raw.rebased().required_size() <= buf_len {
                    (
                        raw.rebased(),
                        "only the rebased layout fits: the pointer already includes it",
                    )
                } else {
                    return Err(format!(
                        "image-data describes {} bytes (rebased {}) but the output buffer holds \
                         {}",
                        raw.required_size(),
                        raw.rebased().required_size(),
                        buf_len
                    ));
                }
            }
            Some(Err(e)) => return Err(format!("image-data does not parse: {e}")),
            None => {
                let stride = format.get_i32(keys::STRIDE).unwrap_or(width as i32).max(0) as u32;
                let slice = format
                    .get_i32(keys::SLICE_HEIGHT)
                    .unwrap_or(height as i32)
                    .max(0) as u32;
                (
                    MediaImage2::semiplanar(width, height, stride, slice),
                    "no image-data: NV12 assumed from stride and slice-height",
                )
            }
        };
        if !self.layout_logged {
            self.layout_logged = true;
            let p = |i: usize| {
                let p = &image.planes[i];
                format!("off {} col {} row {}", p.offset, p.col_inc, p.row_inc)
            };
            info!(
                "decoder session {}: output layout: {}x{} color-format {} stride {:?} \
                 slice-height {:?} crop {:?}; MediaImage2 {}x{} Y({}) U({}) V({}) chroma {:?}; \
                 mPlane[0].mOffset {}: {}; buffer {} bytes, needs {}",
                self.id,
                width,
                height,
                format
                    .get_i32(keys::COLOR_FORMAT)
                    .map(color_format_name)
                    .unwrap_or_else(|| "-".to_string()),
                format.get_i32(keys::STRIDE),
                format.get_i32(keys::SLICE_HEIGHT),
                format.crop(),
                image.width,
                image.height,
                p(0),
                p(1),
                p(2),
                image.chroma_layout(),
                image.planes[0].offset,
                verdict,
                buf_len,
                image.required_size()
            );
        }
        Ok(image)
    }

    /// The coded size the frame is laid out for: what was announced, unless the picture turns
    /// out larger, in which case the larger size is announced now (a `SOURCE_CHANGE` the guest
    /// handles like any other) and the frame waits for buffers of that size.
    fn canvas_for(&mut self, image: &MediaImage2) -> Result<(u32, u32), String> {
        let picture = (image.width, image.height);
        match self.announced {
            Some(a) if a.coded.0 >= picture.0 && a.coded.1 >= picture.1 => Ok(a.coded),
            _ => {
                warn!(
                    "decoder session {}: a {}x{} picture exceeds the announced coded size {:?}; \
                     announcing it",
                    self.id, picture.0, picture.1, self.announced
                );
                self.announce(picture, (0, 0, picture.0, picture.1), "picture");
                Ok(picture)
            }
        }
    }

    fn announce(&mut self, coded: (u32, u32), visible: (i32, i32, u32, u32), from: &str) {
        let announced = Announced { coded, visible };
        if self.announced == Some(announced) {
            return;
        }
        self.announced = Some(announced);
        self.format_changes += 1;
        info!(
            "decoder session {}: format change #{}: coded {}x{}, visible {}x{} at ({},{}) from \
             the {} -> SOURCE_CHANGE, min {} CAPTURE buffers",
            self.id,
            self.format_changes,
            coded.0,
            coded.1,
            visible.2,
            visible.3,
            visible.0,
            visible.1,
            from,
            MIN_CAPTURE_BUFFERS
        );
        self.events.push(DecoderEvent::FormatChanged {
            coded_size: coded,
            visible_rect: Rect::new(visible.0, visible.1, visible.2, visible.3),
            min_capture_buffers: MIN_CAPTURE_BUFFERS,
        });
    }

    /// `onAsyncFormatChanged`: the stream's size, from the output format.
    fn handle_format(&mut self, format: Option<MediaFormat>) {
        let Some(format) = format else {
            warn!(
                "decoder session {}: a format change with no format",
                self.id
            );
            return;
        };
        let width = format.get_i32(keys::WIDTH).unwrap_or(0).max(0) as u32;
        let height = format.get_i32(keys::HEIGHT).unwrap_or(0).max(0) as u32;
        if width == 0 || height == 0 {
            warn!(
                "decoder session {}: a format change without a size: {}",
                self.id, format
            );
            return;
        }
        // The visible rectangle. The picture is always written at the origin of the CAPTURE
        // buffer (`MediaImage2` describes the cropped picture), so a crop that does not start
        // at (0, 0) is reported by its size alone.
        let visible = match format.crop() {
            Some((l, t, r, b)) if r >= l && b >= t => {
                if (l, t) != (0, 0) {
                    warn!(
                        "decoder session {}: crop starts at ({}, {}); the picture is written at \
                         the origin",
                        self.id, l, t
                    );
                }
                (
                    0,
                    0,
                    ((r - l + 1) as u32).min(width),
                    ((b - t + 1) as u32).min(height),
                )
            }
            _ => (0, 0, width, height),
        };
        info!("decoder session {}: output format: {}", self.id, format);
        self.announce((width, height), visible, "codec");
    }

    /// After an EOS the codec takes no input until it is flushed (the async rule): the next
    /// bitstream buffer after a drain restarts it.
    fn restart_after_eos(&mut self) -> IoctlResult<()> {
        if !self.eos_queued {
            return Ok(());
        }
        info!(
            "decoder session {}: input after EOS: flushing to restart",
            self.id
        );
        self.flush_codec("restart after EOS")
    }

    /// `AMediaCodec_flush` + `start`, bounded; drops every codec-side index this session holds.
    fn flush_codec(&mut self, what: &'static str) -> IoctlResult<()> {
        let Some(codec) = self.codec.take() else {
            return Ok(());
        };
        // Whatever the codec delivered is void after the flush: not released, just forgotten. A
        // format change held back in there goes with it; if the size really changed, the first
        // picture after the restart announces it (`canvas_for`).
        self.held_outputs.clear();
        self.free_inputs.clear();
        self.eos_queued = false;
        self.eos_seen = false;
        self.refused = 0;
        self.stale = 0;
        let id = self.id;
        match bounded("decoder", id, what, CODEC_FLUSH_TIMEOUT, move || {
            let mut codec = codec;
            let result = codec.flush_and_restart();
            (codec, result)
        }) {
            Some((codec, Ok(()))) => {
                self.codec = Some(codec);
                Ok(())
            }
            Some((codec, Err(e))) => {
                // The codec is back but did not flush: it is stopped like any dead codec.
                let errno = errno_for(&e);
                self.stop_codec(codec);
                self.fail(format!("{} failed: {}", what, e));
                Err(errno)
            }
            None => {
                self.fail(format!("{} timed out", what));
                Err(libc::ETIMEDOUT)
            }
        }
    }

    /// `AMediaCodec_stop` + `delete` on a thread, waited for at most [`CODEC_STOP_TIMEOUT`].
    fn stop_codec(&mut self, codec: Codec) {
        let id = self.id;
        let name = codec.name().to_string();
        if bounded("decoder", id, "stop", CODEC_STOP_TIMEOUT, move || {
            let mut codec = codec;
            if let Err(e) = codec.stop() {
                warn!("decoder session {}: AMediaCodec_stop: {}", id, e);
            }
            drop(codec);
        })
        .is_none()
        {
            warn!(
                "decoder session {}: {} is still stopping on a detached thread",
                id, name
            );
        }
    }
}

impl VideoDecoderBackendSession for MediaCodecDecoderSession {
    fn start(&mut self, coded_format: PixelFormat, coded_size: (u32, u32)) -> IoctlResult<()> {
        if self.codec.is_some() {
            return Ok(());
        }
        if self.dead {
            return Err(libc::ENODEV);
        }
        let chosen = self
            .codecs
            .iter()
            .find(|c| c.fourcc == coded_format)
            .cloned()
            .ok_or(libc::EINVAL)?;
        let mut format = MediaFormat::new().map_err(|e| errno_for(&e))?;
        format
            .set_str(keys::MIME, &chosen.mime)
            .map_err(|e| errno_for(&e))?;
        format.set_i32(keys::WIDTH, coded_size.0.min(i32::MAX as u32) as i32);
        format.set_i32(keys::HEIGHT, coded_size.1.min(i32::MAX as u32) as i32);
        format.set_i32(keys::COLOR_FORMAT, COLOR_FORMAT_YUV420_FLEXIBLE);
        let input_size = max_input_size(coded_size);
        format.set_i32(
            keys::MAX_INPUT_SIZE,
            input_size.min(i32::MAX as usize) as i32,
        );
        if chosen.low_latency {
            format.set_i32(keys::LOW_LATENCY, 1);
        }
        info!(
            "decoder session {}: creating {} for {} at {}x{}: {}",
            self.id, chosen.name, coded_format, coded_size.0, coded_size.1, format
        );

        let sink = self.sink.clone();
        let name = chosen.name.clone();
        let id = self.id;
        let created = bounded("decoder", id, "start", CODEC_START_TIMEOUT, move || {
            let mut codec = Codec::create_by_name(&name)?;
            // Every callback bumps the device session's eventfd once it has queued its event:
            // that is the whole wake-up path, no thread in between.
            codec.set_wake_hook(Box::new(move || sink.signal()));
            codec.configure(&format, false)?;
            codec.start()?;
            Ok::<Codec, CodecError>(codec)
        });
        match created {
            Some(Ok(codec)) => {
                info!(
                    "decoder session {}: {} ({}{}) started for {} at {}x{}: max-input-size {}, \
                     low-latency {}",
                    self.id,
                    codec.name(),
                    if chosen.hardware {
                        "hardware"
                    } else {
                        "software"
                    },
                    if chosen.vendor { ", vendor" } else { "" },
                    coded_format,
                    coded_size.0,
                    coded_size.1,
                    input_size,
                    if chosen.low_latency { "on" } else { "off" }
                );
                self.nal = NalCodec::for_mime(&chosen.mime);
                self.chosen = Some(chosen);
                self.codec = Some(codec);
                self.started_at = Some(Instant::now());
                Ok(())
            }
            Some(Err(e)) => {
                error!(
                    "decoder session {}: cannot start {} for {}: {}",
                    self.id, chosen.name, coded_format, e
                );
                Err(errno_for(&e))
            }
            None => Err(libc::ETIMEDOUT),
        }
    }

    fn decode(&mut self, buffer: InputBuffer) -> IoctlResult<()> {
        if self.dead {
            return Err(libc::ENODEV);
        }
        if self.codec.is_none() {
            return Err(libc::EINVAL);
        }
        if self.eos_seen {
            // A new stream after a drain. Input queued while a drain is still in flight waits
            // in the FIFO, as the kernel's decoder interface says it must, and restarts the
            // codec once the LAST buffer is out (`take_events`).
            self.restart_after_eos()?;
        }
        self.pending
            .push_back(PendingInput::Bitstream { buffer, offset: 0 });
        let before = self.events.len();
        self.pump_input();
        if self.events.len() > before {
            self.sink.signal();
        }
        Ok(())
    }

    fn use_as_capture(&mut self, buffer: OutputBuffer) -> IoctlResult<()> {
        if self.dead {
            return Err(libc::ENODEV);
        }
        self.captures.push_back(buffer);
        let before = self.events.len();
        self.pump_output();
        if self.events.len() > before {
            self.sink.signal();
        }
        Ok(())
    }

    fn clear_capture_buffers(&mut self) -> IoctlResult<()> {
        // STREAMOFF(CAPTURE): every lent buffer goes back unfilled, and so must every frame
        // report that has not reached the device yet, or a buffer lent again after this call
        // could be returned as decoded with nothing in it. The frames the codec still holds are
        // the client's discarded ones (a reallocation is under way): released, not kept.
        let lent = self.captures.len();
        self.captures.clear();
        self.events
            .retain(|e| !matches!(e, DecoderEvent::FrameDecoded { .. }));
        let mut held = 0;
        let mut formats = Vec::new();
        for entry in std::mem::take(&mut self.held_outputs) {
            match entry {
                Held::Output { index, .. } => {
                    held += 1;
                    if let Some(codec) = self.codec.as_mut() {
                        if let Err(e) = codec.release_output(index) {
                            warn!(
                                "decoder session {}: releasing output {} on STREAMOFF(CAPTURE): {}",
                                self.id, index, e
                            );
                        }
                    }
                }
                // A format change the discarded frames were holding back is announced now.
                Held::Format(format) => formats.push(format),
            }
        }
        for format in formats {
            self.handle_format(format);
        }
        info!(
            "decoder session {}: CAPTURE cleared: {} lent buffer(s) returned, {} held output(s) \
             released",
            self.id, lent, held
        );
        Ok(())
    }

    fn flush(&mut self) -> IoctlResult<()> {
        // STREAMOFF(OUTPUT), a seek. The device takes every OUTPUT buffer back itself when this
        // returns, so the pending ones are dropped, not reported: an `InputBufferDone` delivered
        // later would land on a buffer the guest may have queued again by then.
        let pending = self.pending.len();
        self.pending.clear();
        self.events
            .retain(|e| !matches!(e, DecoderEvent::InputBufferDone(_)));
        let held = self.held_outputs.len();
        let refused = self.refused;
        let stale = self.stale;
        self.seeks += 1;
        let result = self.flush_codec("flush");
        // What the crate's generation stamp dropped at this flush (D23): events queued before
        // the flush that nobody had taken yet.
        let stale_events = self
            .codec
            .as_ref()
            .map(|codec| codec.stale_events())
            .unwrap_or(self.stale_events_seen);
        let dropped_by_flush = stale_events.saturating_sub(self.stale_events_seen);
        self.stale_events_seen = stale_events;
        info!(
            "decoder session {}: seek #{}: {} pending input(s) and {} held output(s) dropped, \
             {} stale index(es) ignored and {} refused since the last seek, {} queued event(s) \
             dropped by this flush{}",
            self.id,
            self.seeks,
            pending,
            held,
            stale,
            refused,
            dropped_by_flush,
            match result {
                Ok(()) => "".to_string(),
                Err(errno) => format!("; the flush failed with errno {errno}"),
            }
        );
        result
    }

    fn drain(&mut self) -> IoctlResult<()> {
        if self.dead {
            return Err(libc::ENODEV);
        }
        if self.eos_queued || self.pending.iter().any(|p| matches!(p, PendingInput::Eos)) {
            return Ok(());
        }
        self.pending.push_back(PendingInput::Eos);
        let before = self.events.len();
        self.pump_input();
        if self.events.len() > before {
            self.sink.signal();
        }
        Ok(())
    }

    fn stop(&mut self) {
        self.pending.clear();
        self.captures.clear();
        self.held_outputs.clear();
        self.free_inputs.clear();
        self.events.clear();
        if let Some(codec) = self.codec.take() {
            let name = codec.name().to_string();
            self.stop_codec(codec);
            info!(
                "decoder session {}: {} stopped after {:?}: {} bitstream buffers in, {} frames \
                 out, {} seek(s), {} format change(s)",
                self.id,
                name,
                self.started_at
                    .map(|t| t.elapsed())
                    .unwrap_or(Duration::ZERO),
                self.inputs,
                self.frames,
                self.seeks,
                self.format_changes
            );
        }
    }

    fn take_events(&mut self) -> Vec<DecoderEvent> {
        let codec_events = self
            .codec
            .as_ref()
            .map(|codec| codec.take_events())
            .unwrap_or_default();
        for event in codec_events {
            match event {
                CodecEvent::InputAvailable(index) => {
                    if self.input_capacity.is_none() {
                        if let Some(Ok(capacity)) =
                            self.codec.as_ref().map(|codec| codec.input_capacity(index))
                        {
                            self.input_capacity = Some(capacity);
                            info!(
                                "decoder session {}: input buffers hold {} bytes",
                                self.id, capacity
                            );
                        }
                    }
                    self.free_inputs.push_back(index);
                }
                CodecEvent::OutputAvailable { index, info } => {
                    self.held_outputs.push_back(Held::Output { index, info });
                }
                CodecEvent::FormatChanged(format) => {
                    if self.held_outputs.is_empty() {
                        self.handle_format(format);
                    } else {
                        self.held_outputs.push_back(Held::Format(format));
                    }
                }
                CodecEvent::Error {
                    status,
                    action_code,
                    detail,
                } => {
                    self.fail(format!(
                        "{} reported error {} ({}), action {}: {}",
                        self.codec_name(),
                        status,
                        media_status_name(status),
                        action_code,
                        detail
                    ));
                    break;
                }
            }
        }
        if !self.dead {
            // Input first: a drain that was waiting for a slot goes in before its EOS output is
            // looked for.
            self.pump_input();
            self.pump_output();
            if self.eos_seen && self.pending.iter().any(|p| !matches!(p, PendingInput::Eos)) {
                // Bitstream that arrived during the drain: the codec restarts now that the LAST
                // buffer is out.
                if self.restart_after_eos().is_ok() {
                    self.pump_input();
                }
            }
        }
        std::mem::take(&mut self.events)
    }
}

impl Drop for MediaCodecDecoderSession {
    fn drop(&mut self) {
        // `stop` is the normal path (the device calls it before `close_session`); a session
        // dropped without it still must not run `AMediaCodec_delete` unbounded on the worker.
        self.stop();
    }
}
