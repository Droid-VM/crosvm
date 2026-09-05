// Copyright 2026 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! The MediaCodec NDK behind the virtio-media video encoder device (`VPU_DESIGN.md` §7.3).
//!
//! [`MediaCodecEncoderBackend`] is the platform's list of hardware video encoders, read once
//! from `AMediaCodecStore` through the `android_codec` crate and turned into the
//! `EncoderCapabilities` the device answers `ENUM_FMT` / `ENUM_FRAMESIZES` /
//! `ENUM_FRAMEINTERVALS` and the `V4L2_CID_MPEG_VIDEO_*` control ranges and menus from: nothing
//! about what the phone can encode is written down here. [`MediaCodecEncoderSession`] is one
//! encode: an `AMediaCodec` in asynchronous mode, created when both queues stream (the device
//! decides when, `logs/vpu_wp/M7-crate.md` §1), fed the guest's raw NV12 frames and emptied
//! into the guest's `CAPTURE` (bitstream) buffers.
//!
//! # Threads
//!
//! The decoder's shape (`android.rs`), mirrored:
//!
//! ```text
//!  device worker thread                                     NDK callback thread
//!  ────────────────────                                     ───────────────────
//!  encode(InputBuffer)     pack into getInputBuffer +       onAsyncInputAvailable  ─┐ push onto the
//!                          queueInputBuffer, else FIFO      onAsyncOutputAvailable  │ Codec's queue,
//!  use_as_capture(buf)     FIFO of lent CAPTURE buffers     onAsyncFormatChanged    │ then bump the
//!  take_events()           drain the Codec's queue: feed    onAsyncError           ─┘ session eventfd
//!                          frames, copy coded frames into                              (the wake hook)
//!                          lent buffers, releaseOutputBuffer
//!  start / flush / stop    on a short-lived thread, waited for at most a named bound
//! ```
//!
//! Everything that touches a guest or pool buffer happens on the worker thread, inside the trait
//! calls; the four NDK callbacks only queue an event and bump the device session's eventfd
//! (`android_codec`'s wake hook). There is no encoder thread and nothing to join for §2.5: once
//! [`VideoEncoderBackendSession::flush`] or [`VideoEncoderBackendSession::stop`] has emptied the
//! session's own FIFOs, no code path can touch a lent buffer any more. The NDK calls that can
//! block -- `createCodecByName` + `configure` + `start`, `flush` + `start`, `stop` + `delete` --
//! run on a thread of their own, waited for at most `CODEC_START_TIMEOUT`, `CODEC_FLUSH_TIMEOUT`
//! or `CODEC_STOP_TIMEOUT` (`logs/vpu_wp/F5-crosvm.md` §2's rule; the bounds are the decoder's).
//!
//! # Buffers
//!
//! * A raw frame is **copied** into the codec's input buffer as soon as an input slot is free --
//!   the visible rectangle of the guest's tightly packed NV12, padded to the `stride` and
//!   `slice-height` the codec published after `configure` (design §7.3; chroma at `stride *
//!   slice-height`) -- and `InputBufferDone` follows at once, so the guest gets its buffer back
//!   right after the copy. With no slot free it waits in a FIFO, still lent.
//! * A coded frame is copied out of the codec's output buffer the moment it is delivered and the
//!   buffer released (tens of kilobytes against the raw copy; the codec never waits on the guest
//!   for an output slot), then written into the next lent `CAPTURE` buffer that holds it;
//!   `bytesused`, the frame's timestamp (the input's `presentationTimeUs`, carried through by the
//!   codec) and `KEYFRAME` from `BUFFER_FLAG_KEY_FRAME` go to the device. With no `CAPTURE` buffer
//!   lent the frame is held, up to a limit past which no raw frame is fed.
//! * The stream headers (`BUFFER_FLAG_CODEC_CONFIG`: SPS/PPS, VPS/SPS/PPS) follow the device's
//!   `V4L2_CID_MPEG_VIDEO_HEADER_MODE`: `SEPARATE` puts them in a `CAPTURE` buffer of their own
//!   (`FrameKind::Headers`, the first frame's timestamp), `JOINED_WITH_1ST_FRAME` in front of the
//!   next frame (unless that frame already starts with them, which `PREPEND_SPSPPS_TO_IDR` makes
//!   the codec do).
//! * Drain is an empty `END_OF_STREAM` input; the output that carries the flag -- usually empty --
//!   becomes the guest's `LAST` buffer. `flush` (`STREAMOFF(OUTPUT)`, and `V4L2_ENC_CMD_START`
//!   after a drain) is `AMediaCodec_flush` then `start`, the async rule; `stop`
//!   (`STREAMOFF(CAPTURE)`, close) deletes the codec, and the next `start` creates a new one.
//! * A flush voids every index the codec handed out. `android_codec` stamps events with a
//!   generation and drops the old ones; the callback the NDK looper had already queued arrives
//!   anyway, with a dead index, and a null `getInputBuffer` / `getOutputBuffer` on it is skipped at
//!   debug level, never a session error (`logs/vpu_wp/B5-acceptance.md` D23).
//!
//! # Controls
//!
//! The two that change while the codec runs are forwarded as `setParameters`:
//! `V4L2_CID_MPEG_VIDEO_BITRATE` as `video-bitrate`, `V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME` as
//! `request-sync` (applied right before the next frame is queued, so it lands on the frame the
//! guest meant when frames are waiting for an input slot). Everything else goes into `configure`
//! when the codec is created, from the `EncoderConfig` the device gathers.

use std::collections::VecDeque;
use std::time::Duration;
use std::time::Instant;

use android_codec::color_format_name;
use android_codec::keys;
use android_codec::list_codecs;
use android_codec::media_status_name;
use android_codec::BufferInfo;
use android_codec::Codec;
use android_codec::CodecError;
use android_codec::CodecEvent;
use android_codec::CodecInfo;
use android_codec::CodecKind;
use android_codec::ListOptions;
use android_codec::MediaFormat;
use android_codec::BITRATE_MODE_CBR;
use android_codec::BITRATE_MODE_CQ;
use android_codec::BITRATE_MODE_VBR;
use android_codec::BUFFER_FLAG_CODEC_CONFIG;
use android_codec::BUFFER_FLAG_END_OF_STREAM;
use android_codec::BUFFER_FLAG_KEY_FRAME;
use android_codec::COLOR_FORMAT_YUV420_FLEXIBLE;
use android_codec::COLOR_FORMAT_YUV420_PACKED_PLANAR;
use android_codec::COLOR_FORMAT_YUV420_PACKED_SEMI_PLANAR;
use android_codec::COLOR_FORMAT_YUV420_PLANAR;
use android_codec::COLOR_FORMAT_YUV420_SEMI_PLANAR;
use android_codec::COLOR_QCOM_FORMAT_YUV420_SEMI_PLANAR;
use android_codec::COLOR_RANGE_FULL;
use android_codec::COLOR_RANGE_LIMITED;
use android_codec::COLOR_STANDARD_BT2020;
use android_codec::COLOR_STANDARD_BT601_NTSC;
use android_codec::COLOR_STANDARD_BT601_PAL;
use android_codec::COLOR_STANDARD_BT709;
use android_codec::COLOR_TRANSFER_LINEAR;
use android_codec::COLOR_TRANSFER_SDR_VIDEO;
use android_codec::COLOR_TRANSFER_ST2084;
use anyhow::Context;
use base::debug;
use base::error;
use base::info;
use base::warn;
use virtio_media::devices::video_encoder::CodedFormat;
use virtio_media::devices::video_encoder::Colorimetry;
use virtio_media::devices::video_encoder::ControlRange;
use virtio_media::devices::video_encoder::EncoderCapabilities;
use virtio_media::devices::video_encoder::EncoderConfig;
use virtio_media::devices::video_encoder::EncoderEvent;
use virtio_media::devices::video_encoder::EncoderSink;
use virtio_media::devices::video_encoder::FrameKind;
use virtio_media::devices::video_encoder::FrameRateRange;
use virtio_media::devices::video_encoder::InputBuffer;
use virtio_media::devices::video_encoder::OutputBuffer;
use virtio_media::devices::video_encoder::QpRange;
use virtio_media::devices::video_encoder::SizeRange;
use virtio_media::devices::video_encoder::VideoEncoderBackend;
use virtio_media::devices::video_encoder::VideoEncoderBackendSession;
use virtio_media::ioctl::IoctlResult;
use virtio_media::v4l2r::bindings;
use virtio_media::v4l2r::controls::codec::VideoBitrateMode;
use virtio_media::v4l2r::controls::codec::VideoH264Level;
use virtio_media::v4l2r::controls::codec::VideoH264Profile;
use virtio_media::v4l2r::controls::codec::VideoHEVCLevel;
use virtio_media::v4l2r::controls::codec::VideoHEVCProfile;
use virtio_media::v4l2r::controls::codec::VideoHeaderMode;
use virtio_media::v4l2r::PixelFormat;
use virtio_media::v4l2r::Rect;

use super::android::bounded;
use super::android::choose;
use super::android::errno_for;
use super::android::has_feature;
use super::android::is_hardware;
use super::android::pts_from;
use super::android::timeval_from;
use super::android::CODEC_FLUSH_TIMEOUT;
use super::android::CODEC_START_TIMEOUT;
use super::android::CODEC_STOP_TIMEOUT;
use super::android::CODED_FORMATS;
use super::android::MAX_REFUSED_INDICES;
use super::android::STALE_LOG_LIMIT;

/// `V4L2_CID_MIN_BUFFERS_FOR_OUTPUT`: raw frames are copied into the codec's own input buffer as
/// soon as a slot is free and returned at once, so the encoder never pins a guest frame; a few
/// keep the copy pipelined. GStreamer sizes its upstream pool from this, ffmpeg allocates 16
/// regardless. Same number as the decoder's `MIN_CAPTURE_BUFFERS`.
const MIN_OUTPUT_BUFFERS: u32 = 4;
/// The bitrate a session starts with when the guest sets none (`V4L2_CID_MPEG_VIDEO_BITRATE`'s
/// default), clamped into the codec's range. ffmpeg always sets one; GStreamer's `v4l2h264enc`
/// only with its `extra-controls`.
const DEFAULT_BITRATE: i32 = 2_000_000;
/// `V4L2_CID_MPEG_VIDEO_GOP_SIZE`'s range: MediaCodec takes the keyframe interval in seconds
/// (`KEY_I_FRAME_INTERVAL`, converted with the frame rate at `configure`; 0 = every frame a
/// keyframe), so the range is a driver's choice -- venus's `0..65535`, default 30 (one keyframe
/// per second at the default 30 fps).
const GOP_RANGE: ControlRange = ControlRange::new(0, 65_535, 30);
/// The quantiser range an 8-bit H.264 / HEVC codec accepts; offered only when the codec reports
/// `FEATURE_QpBounds`, and passed to it only when the guest narrows it.
const QP_RANGE: QpRange = QpRange { min: 0, max: 51 };
/// Offered for a codec whose `VideoCapabilities` the platform does not publish (the no-Store
/// fallback of `android_codec::list_codecs`). Logged when used.
const FALLBACK_SIZE_RANGE: SizeRange = SizeRange::new(16, 4096, 2);
const FALLBACK_FRAME_RATE: FrameRateRange = FrameRateRange { min: 1, max: 240 };
const FALLBACK_BITRATE: (i32, i32) = (1, 100_000_000);

/// `AMediaCodecInfo_FEATURE_QpBounds`: the codec honours `KEY_VIDEO_QP_MIN` / `_MAX`.
const FEATURE_QP_BOUNDS: &str = "qp-bounds";

/// The AV1 fourcc: offered only with `allow_sw` (`logs/vpu_wp/M7-crate.md` §8 item 1:
/// `v4l2-compliance` does not know AV1 as a stateful-encoder format and would fail two tests).
const AV10: PixelFormat = PixelFormat::from_fourcc(b"AV10");

// ---------------------------------------------------------------------------------------------
// MediaCodec <-> V4L2 value maps
// ---------------------------------------------------------------------------------------------

/// `(MediaCodec profile constant, V4L2 profile menu value)` for a mime: the
/// `MediaCodecConstants.h` values `android_codec::profile_table` probes with, against the
/// kernel's `v4l2_mpeg_video_*_profile` enums (through v4l2r's). Profiles the kernel has no
/// value for (HDR variants) are not listed and so never offered.
fn profile_pairs(mime: &str) -> &'static [(i32, i32)] {
    match mime {
        "video/avc" => &[
            (0x01, VideoH264Profile::Baseline as i32),
            (0x02, VideoH264Profile::Main as i32),
            (0x04, VideoH264Profile::Extended as i32),
            (0x08, VideoH264Profile::High as i32),
            (0x10, VideoH264Profile::High10 as i32),
            (0x20, VideoH264Profile::High422 as i32),
            (0x40, VideoH264Profile::High444Predictive as i32),
            (0x10000, VideoH264Profile::ConstrainedBaseline as i32),
            (0x80000, VideoH264Profile::ConstrainedHigh as i32),
        ],
        "video/hevc" => &[
            (0x01, VideoHEVCProfile::Main as i32),
            (0x02, VideoHEVCProfile::Main10 as i32),
            (0x04, VideoHEVCProfile::MainStill as i32),
        ],
        // `VP8ProfileMain` is the one VP8 profile; V4L2 numbers them 0..3.
        "video/x-vnd.on2.vp8" => &[(0x01, 0)],
        "video/x-vnd.on2.vp9" => &[(0x01, 0), (0x02, 1), (0x04, 2), (0x08, 3)],
        _ => &[],
    }
}

/// The profile a session starts with when the guest sets none: the one the codec picks for
/// itself (`c2.qti.avc.encoder` answers High, `c2.qti.hevc.encoder` Main,
/// `logs/vpu_wp/scratch-b5/codec/b5codec/enc_*.txt`), as a V4L2 value.
fn preferred_profile(mime: &str) -> Option<i32> {
    match mime {
        "video/avc" => Some(VideoH264Profile::High as i32),
        "video/hevc" => Some(VideoHEVCProfile::Main as i32),
        "video/x-vnd.on2.vp8" | "video/x-vnd.on2.vp9" => Some(0),
        _ => None,
    }
}

/// The 20 H.264 levels in the order both `MediaCodecConstants.h` (`AVCLevel1 = 1 <<
/// index`) and the kernel (`V4L2_MPEG_VIDEO_H264_LEVEL_1_0 = index`) list them.
const H264_LEVELS: [VideoH264Level; 20] = [
    VideoH264Level::L1_0,
    VideoH264Level::L1B,
    VideoH264Level::L1_1,
    VideoH264Level::L1_2,
    VideoH264Level::L1_3,
    VideoH264Level::L2_0,
    VideoH264Level::L2_1,
    VideoH264Level::L2_2,
    VideoH264Level::L3_0,
    VideoH264Level::L3_1,
    VideoH264Level::L3_2,
    VideoH264Level::L4_0,
    VideoH264Level::L4_1,
    VideoH264Level::L4_2,
    VideoH264Level::L5_0,
    VideoH264Level::L5_1,
    VideoH264Level::L5_2,
    VideoH264Level::L6_0,
    VideoH264Level::L6_1,
    VideoH264Level::L6_2,
];

/// The 13 HEVC levels in the kernel's order; `MediaCodecConstants.h` lists each twice
/// (`HEVCMainTierLevelN = 1 << (2 * index)`, `HEVCHighTierLevelN = 1 << (2 * index + 1)`) and
/// the kernel keeps the tier in a control of its own, so both map to the same value.
const HEVC_LEVELS: [VideoHEVCLevel; 13] = [
    VideoHEVCLevel::L1_0,
    VideoHEVCLevel::L2_0,
    VideoHEVCLevel::L2_1,
    VideoHEVCLevel::L3_0,
    VideoHEVCLevel::L3_1,
    VideoHEVCLevel::L4_0,
    VideoHEVCLevel::L4_1,
    VideoHEVCLevel::L5_0,
    VideoHEVCLevel::L5_1,
    VideoHEVCLevel::L5_2,
    VideoHEVCLevel::L6_0,
    VideoHEVCLevel::L6_1,
    VideoHEVCLevel::L6_2,
];

/// A MediaCodec level constant as the V4L2 level menu value, for the codecs the device has a
/// level control for (H.264 and HEVC).
fn level_to_v4l2(mime: &str, mc: i32) -> Option<i32> {
    if mc <= 0 || mc.count_ones() != 1 {
        return None;
    }
    let bit = mc.trailing_zeros() as usize;
    match mime {
        "video/avc" => H264_LEVELS.get(bit).map(|l| *l as i32),
        "video/hevc" => HEVC_LEVELS.get(bit / 2).map(|l| *l as i32),
        _ => None,
    }
}

/// The way back: a V4L2 level menu value as the MediaCodec constant `KEY_LEVEL` takes (the main
/// tier for HEVC).
fn level_to_mc(mime: &str, v4l2: i32) -> Option<i32> {
    match mime {
        "video/avc" => H264_LEVELS
            .iter()
            .position(|l| *l as i32 == v4l2)
            .map(|i| 1 << i),
        "video/hevc" => HEVC_LEVELS
            .iter()
            .position(|l| *l as i32 == v4l2)
            .map(|i| 1 << (2 * i)),
        _ => None,
    }
}

/// `V4L2_CID_MPEG_VIDEO_BITRATE_MODE` as `KEY_BITRATE_MODE` (`ABitrateMode`, written straight
/// into `"bitrate-mode"`).
fn bitrate_mode_to_mc(mode: VideoBitrateMode) -> i32 {
    match mode {
        VideoBitrateMode::VariableBitrate => BITRATE_MODE_VBR,
        VideoBitrateMode::ConstantBitrate => BITRATE_MODE_CBR,
        VideoBitrateMode::ConstantQuality => BITRATE_MODE_CQ,
    }
}

/// The V4L2 colorimetry of the raw frames as MediaCodec's `(color-standard, color-range,
/// color-transfer)`, each `None` where V4L2 has no value the codec could be told. `DEFAULT`
/// follows the kernel's own defaulting rules for a YUV format: the transfer function of the
/// colorspace (`V4L2_MAP_XFER_FUNC_DEFAULT`: SDR for every SDR colorspace) and limited range
/// (`V4L2_MAP_QUANTIZATION_DEFAULT`).
fn color_aspects(c: &Colorimetry) -> (Option<i32>, Option<i32>, Option<i32>) {
    let sdr = matches!(
        c.colorspace,
        bindings::v4l2_colorspace_V4L2_COLORSPACE_REC709
            | bindings::v4l2_colorspace_V4L2_COLORSPACE_SMPTE170M
            | bindings::v4l2_colorspace_V4L2_COLORSPACE_470_SYSTEM_M
            | bindings::v4l2_colorspace_V4L2_COLORSPACE_470_SYSTEM_BG
            | bindings::v4l2_colorspace_V4L2_COLORSPACE_BT2020
    );
    let standard = match c.colorspace {
        bindings::v4l2_colorspace_V4L2_COLORSPACE_REC709 => Some(COLOR_STANDARD_BT709),
        bindings::v4l2_colorspace_V4L2_COLORSPACE_SMPTE170M
        | bindings::v4l2_colorspace_V4L2_COLORSPACE_470_SYSTEM_M => Some(COLOR_STANDARD_BT601_NTSC),
        bindings::v4l2_colorspace_V4L2_COLORSPACE_470_SYSTEM_BG => Some(COLOR_STANDARD_BT601_PAL),
        bindings::v4l2_colorspace_V4L2_COLORSPACE_BT2020 => Some(COLOR_STANDARD_BT2020),
        _ => None,
    };
    let range = match c.quantization {
        bindings::v4l2_quantization_V4L2_QUANTIZATION_FULL_RANGE => Some(COLOR_RANGE_FULL),
        bindings::v4l2_quantization_V4L2_QUANTIZATION_LIM_RANGE => Some(COLOR_RANGE_LIMITED),
        bindings::v4l2_quantization_V4L2_QUANTIZATION_DEFAULT if sdr => Some(COLOR_RANGE_LIMITED),
        _ => None,
    };
    let transfer = match c.xfer_func {
        bindings::v4l2_xfer_func_V4L2_XFER_FUNC_709 => Some(COLOR_TRANSFER_SDR_VIDEO),
        bindings::v4l2_xfer_func_V4L2_XFER_FUNC_SMPTE2084 => Some(COLOR_TRANSFER_ST2084),
        bindings::v4l2_xfer_func_V4L2_XFER_FUNC_NONE => Some(COLOR_TRANSFER_LINEAR),
        bindings::v4l2_xfer_func_V4L2_XFER_FUNC_DEFAULT if sdr => Some(COLOR_TRANSFER_SDR_VIDEO),
        _ => None,
    };
    (standard, range, transfer)
}

/// `KEY_I_FRAME_INTERVAL` for a GOP: seconds between keyframes at the frame rate, which
/// `CCodecConfig` turns back into frames as `interval * rate + 0.5` (`CCodecConfig.cpp:1743`),
/// so a fractional interval round-trips to the GOP the guest set. `0` = every frame a keyframe,
/// on both sides.
fn i_frame_interval(gop_size: u32, fps: f32) -> f32 {
    if gop_size == 0 || fps <= 0.0 {
        0.0
    } else {
        gop_size as f32 / fps
    }
}

/// Write `value` under `key` as the int the Java world writes when it is whole, as a float
/// otherwise (`KEY_FRAME_RATE` and `KEY_I_FRAME_INTERVAL` are "int or float").
fn set_number(format: &mut MediaFormat, key: &std::ffi::CStr, value: f32) {
    if value.fract() == 0.0 && value.abs() < i32::MAX as f32 {
        format.set_i32(key, value as i32);
    } else {
        format.set_f32(key, value);
    }
}

// ---------------------------------------------------------------------------------------------
// The input layout and the copy into it
// ---------------------------------------------------------------------------------------------

/// How the codec wants its input buffer laid out, read back from `AMediaCodec_getInputFormat`
/// after `configure` (`CCodec.cpp:2004-2053`): rows padded to `stride`, the chroma plane
/// `slice_height` rows down, interleaved (NV12, `COLOR_FormatYUV420SemiPlanar`) or as two planes
/// of `stride / 2` bytes per row (`COLOR_FormatYUV420Planar`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct InputLayout {
    color_format: i32,
    stride: usize,
    slice_height: usize,
    semiplanar: bool,
}

impl InputLayout {
    /// Bytes the padded frame occupies: what `queueInputBuffer` is given as the size.
    fn frame_size(&self) -> usize {
        let luma = self.stride * self.slice_height;
        luma + luma / 2
    }
}

/// Bytes of one tightly packed NV12 frame of `width` x `height` (`bytesperline = width`; odd
/// dimensions round the chroma plane up), the device's `OUTPUT` frame contract.
fn nv12_size(width: usize, height: usize) -> usize {
    width * height + 2 * width.div_ceil(2) * height.div_ceil(2)
}

/// Copy `visible` of the tightly packed NV12 frame at `src` (`coded.0` bytes per luma row,
/// chroma at `coded.0 * coded.1`, `src_len` bytes in all) into `dst` as `layout` says, and
/// return the bytes used. Bytes in the padding are left as they were.
///
/// The crop's origin is held to even coordinates, so its chroma samples exist as a whole; the
/// codec was configured with the visible size, so what it gets is the visible picture alone.
///
/// # Safety
///
/// `src` must point at `src_len` readable bytes that stay mapped for the duration of the call.
unsafe fn pack_frame(
    src: *const u8,
    src_len: usize,
    coded: (u32, u32),
    visible: Rect,
    layout: &InputLayout,
    dst: &mut [u8],
) -> Result<usize, String> {
    let (cw, ch) = (coded.0 as usize, coded.1 as usize);
    let left = (visible.left.max(0) as usize & !1).min(cw);
    let top = (visible.top.max(0) as usize & !1).min(ch);
    let w = (visible.width as usize).min(cw - left);
    let h = (visible.height as usize).min(ch - top);
    if w == 0 || h == 0 {
        return Err(format!(
            "an empty visible rectangle ({:?} of {}x{})",
            visible, cw, ch
        ));
    }
    if src_len < nv12_size(cw, ch) {
        return Err(format!(
            "a {}-byte OUTPUT buffer holds no {}x{} NV12 frame ({} bytes)",
            src_len,
            cw,
            ch,
            nv12_size(cw, ch)
        ));
    }
    if layout.stride < w || layout.slice_height < h {
        return Err(format!(
            "the codec's input layout (stride {}, slice-height {}) does not cover a {}x{} picture",
            layout.stride, layout.slice_height, w, h
        ));
    }
    let needed = layout.frame_size();
    if dst.len() < needed {
        return Err(format!(
            "the codec's input buffer holds {} bytes, the padded {}x{} frame needs {}",
            dst.len(),
            layout.stride,
            layout.slice_height,
            needed
        ));
    }
    let chroma_src = cw * ch;
    let chroma_row_src = 2 * cw.div_ceil(2);
    let chroma_rows = h.div_ceil(2);
    let chroma_w = 2 * w.div_ceil(2);
    let chroma_dst = layout.stride * layout.slice_height;

    for row in 0..h {
        // SAFETY: `src` holds `nv12_size(cw, ch)` bytes (checked above); this row of the luma
        // plane lies inside it; `dst` is a slice we hold exclusively; the two never overlap.
        unsafe {
            std::ptr::copy_nonoverlapping(
                src.add((top + row) * cw + left),
                dst.as_mut_ptr().add(row * layout.stride),
                w,
            )
        };
    }
    if layout.semiplanar {
        for row in 0..chroma_rows {
            // SAFETY: as above, for a row of the interleaved chroma plane.
            unsafe {
                std::ptr::copy_nonoverlapping(
                    src.add(chroma_src + (top / 2 + row) * chroma_row_src + left),
                    dst.as_mut_ptr().add(chroma_dst + row * layout.stride),
                    chroma_w,
                )
            };
        }
    } else {
        let half = layout.stride / 2;
        let v_plane = chroma_dst + half * (layout.slice_height / 2);
        let mut row_buf = vec![0u8; chroma_w];
        for row in 0..chroma_rows {
            // SAFETY: as above; the row goes through a scratch buffer to be de-interleaved.
            unsafe {
                std::ptr::copy_nonoverlapping(
                    src.add(chroma_src + (top / 2 + row) * chroma_row_src + left),
                    row_buf.as_mut_ptr(),
                    chroma_w,
                )
            };
            for (col, pair) in row_buf.chunks_exact(2).enumerate() {
                dst[chroma_dst + row * half + col] = pair[0];
                dst[v_plane + row * half + col] = pair[1];
            }
        }
    }
    Ok(needed)
}

// ---------------------------------------------------------------------------------------------
// The backend: enumeration
// ---------------------------------------------------------------------------------------------

/// One codec the backend picked for a coded format: what the session creates.
#[derive(Clone, Debug)]
struct ChosenEncoder {
    fourcc: PixelFormat,
    mime: String,
    /// The canonical name, the one `AMediaCodec_createCodecByName` takes.
    name: String,
    hardware: bool,
    vendor: bool,
    /// The codec honours `KEY_VIDEO_QP_MIN` / `_MAX`.
    qp_bounds: bool,
    /// The level the device's control starts at (the highest the codec supports). A session at
    /// that value does not pass `KEY_LEVEL`, so the codec picks the level its size and rate
    /// need -- a stream stamped with the maximum level would be refused by players that check
    /// levels; a guest that sets another level gets exactly that one.
    default_level: Option<i32>,
}

/// The platform's hardware encoders as `EncoderCapabilities`, enumerated once. Cheap to clone:
/// the device factory builds a device from it on the worker thread at every start.
#[derive(Clone, Debug)]
pub struct MediaCodecEncoderBackend {
    caps: EncoderCapabilities,
    /// Parallel to `caps.coded_formats`.
    codecs: Vec<ChosenEncoder>,
}

impl MediaCodecEncoderBackend {
    /// Enumerate the encoders and choose one per coded format, with the profiles and levels
    /// each supports. Loads the NDK, which starts this process's binder thread pool, and walks
    /// the codec store's lazily built tables, which have no lock: call it once, from one thread,
    /// after the uid drop (design §7.2, §7.3).
    ///
    /// Software encoders are left out unless `allow_sw`, and so is AV1 (`AV10`) altogether; a
    /// format with no eligible encoder is simply not offered. A platform with no eligible
    /// encoder at all still gets its device, with no coded format (`ENUM_FMT` lists nothing and
    /// the device refuses every session with `ENODEV`): a helper that exits is a VM that does
    /// not boot, which the app cannot foresee (`logs/vpu_wp/A4.md` §10 item 2).
    pub fn new(allow_sw: bool) -> anyhow::Result<Self> {
        let list = list_codecs(&ListOptions {
            include_non_video: false,
            // The profile and level menus need the probe (`isFormatSupported` per candidate,
            // `logs/vpu_wp/M6-probe.md` §3): in-process checks on the store's tables, not binder
            // round trips, so a helper start can afford them.
            probe_profiles: true,
            sizes: Vec::new(),
        })
        .context("cannot enumerate the platform's codecs")?;
        if !list.missing_symbols.is_empty() {
            warn!(
                "encoder: {} optional NDK symbol(s) missing; codecs listed by {}",
                list.missing_symbols.len(),
                list.source
            );
        }
        let encoders = list
            .codecs
            .iter()
            .filter(|c| c.kind == CodecKind::Encoder)
            .count();

        let mut caps = EncoderCapabilities {
            coded_formats: Vec::new(),
            min_output_buffers: MIN_OUTPUT_BUFFERS,
        };
        let mut codecs = Vec::new();
        for (fourcc, mime) in CODED_FORMATS {
            let fourcc = PixelFormat::from_fourcc(fourcc);
            if fourcc == AV10 && !allow_sw {
                info!(
                    "encoder: {} is offered only with allow_sw=true (v4l2-compliance does not \
                     know AV1 as a stateful-encoder format)",
                    fourcc
                );
                continue;
            }
            match choose(&list.codecs, CodecKind::Encoder, mime, allow_sw) {
                Some((chosen, passed_over)) => {
                    let (format, codec) = describe(chosen, fourcc, mime);
                    info!(
                        "encoder: {} -> {} ({}, {}{}), {}..{} x {}..{} step {}x{}, {}..{} fps, \
                         {}..{} bit/s, modes {}, profiles {:?}, levels {:?}, qp {}; passed over: {}",
                        fourcc,
                        chosen.name,
                        mime,
                        if codec.hardware {
                            "hardware"
                        } else {
                            "software"
                        },
                        if codec.vendor { ", vendor" } else { "" },
                        format.width.min,
                        format.width.max,
                        format.height.min,
                        format.height.max,
                        format.width.step,
                        format.height.step,
                        format.frame_rate.min,
                        format.frame_rate.max,
                        format.bitrate.min,
                        format.bitrate.max,
                        format
                            .bitrate_modes
                            .iter()
                            .map(|m| format!("{:?}", m))
                            .collect::<Vec<_>>()
                            .join("/"),
                        format.profiles,
                        format.levels,
                        match format.qp {
                            Some(qp) => format!("{}..{}", qp.min, qp.max),
                            None => "none".to_string(),
                        },
                        if passed_over.is_empty() {
                            "none".to_string()
                        } else {
                            passed_over.join(", ")
                        }
                    );
                    caps.coded_formats.push(format);
                    codecs.push(codec);
                }
                None => {
                    let software: Vec<&str> = list
                        .codecs
                        .iter()
                        .filter(|c| c.kind == CodecKind::Encoder && c.mime == mime)
                        .map(|c| c.name.as_str())
                        .collect();
                    if software.is_empty() {
                        info!("encoder: {} has no encoder on this platform", fourcc);
                    } else {
                        info!(
                            "encoder: {} has no usable hardware encoder (only {}; \
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
                "encoder: no usable video encoder: the codec store ({}) lists {} encoder(s), \
                 none of them hardware for {} (allow_sw={}); the device is served with no coded \
                 format, and every session on it is refused with ENODEV",
                list.source,
                encoders,
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
            "encoder: {} coded format(s) for the guest from {} ({} encoder(s) listed, \
             allow_sw={}): {}",
            caps.coded_formats.len(),
            list.source,
            encoders,
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

/// A chosen codec as the `CodedFormat` the device advertises and the `ChosenEncoder` the
/// session creates: every range, menu and default from what the store said, with a logged
/// fallback where it said nothing.
fn describe(info: &CodecInfo, fourcc: PixelFormat, mime: &str) -> (CodedFormat, ChosenEncoder) {
    let (width, height, frame_rate, bitrate) = match &info.video {
        Some(v) => {
            let size = |lo: i32, hi: i32, step: i32| {
                let lo = lo.max(1) as u32;
                SizeRange::new(lo, hi.max(lo as i32) as u32, step.max(1) as u32)
            };
            let frame_rate = if v.frame_rates.1 <= 0 {
                warn!(
                    "encoder: {} ({}) publishes no frame-rate range; offering {}..{} fps",
                    fourcc, info.name, FALLBACK_FRAME_RATE.min, FALLBACK_FRAME_RATE.max
                );
                FALLBACK_FRAME_RATE
            } else {
                let min = v.frame_rates.0.max(1) as u32;
                FrameRateRange {
                    min,
                    max: (v.frame_rates.1 as u32).max(min),
                }
            };
            let bitrate = if v.bitrates.1 <= 0 {
                warn!(
                    "encoder: {} ({}) publishes no bitrate range; offering {}..{} bit/s",
                    fourcc, info.name, FALLBACK_BITRATE.0, FALLBACK_BITRATE.1
                );
                FALLBACK_BITRATE
            } else {
                (v.bitrates.0.max(1), v.bitrates.1.max(v.bitrates.0.max(1)))
            };
            (
                size(v.widths.0, v.widths.1, v.width_alignment),
                size(v.heights.0, v.heights.1, v.height_alignment),
                frame_rate,
                bitrate,
            )
        }
        None => {
            warn!(
                "encoder: {} ({}) publishes no video capabilities; offering {}..{} step {} in \
                 both dimensions, {}..{} fps, {}..{} bit/s",
                fourcc,
                info.name,
                FALLBACK_SIZE_RANGE.min,
                FALLBACK_SIZE_RANGE.max,
                FALLBACK_SIZE_RANGE.step,
                FALLBACK_FRAME_RATE.min,
                FALLBACK_FRAME_RATE.max,
                FALLBACK_BITRATE.0,
                FALLBACK_BITRATE.1
            );
            (
                FALLBACK_SIZE_RANGE,
                FALLBACK_SIZE_RANGE,
                FALLBACK_FRAME_RATE,
                FALLBACK_BITRATE,
            )
        }
    };

    // The bitrate modes the store says the codec takes, VBR (the default) before CBR. Constant
    // quality is not offered: MediaCodec's CQ needs `KEY_QUALITY`, which the device has no
    // control for (`V4L2_CID_MPEG_VIDEO_CONSTANT_QUALITY` is not in its table).
    let bitrate_modes = match &info.encoder {
        Some(caps) => {
            let supported = |name: &str| caps.bitrate_modes.iter().any(|(n, s)| n == name && *s);
            let mut modes = Vec::new();
            if supported("VBR") {
                modes.push(VideoBitrateMode::VariableBitrate);
            }
            if supported("CBR") {
                modes.push(VideoBitrateMode::ConstantBitrate);
            }
            if modes.is_empty() {
                warn!(
                    "encoder: {} ({}) supports neither VBR nor CBR ({:?}); offering VBR",
                    fourcc, info.name, caps.bitrate_modes
                );
                modes.push(VideoBitrateMode::VariableBitrate);
            }
            modes
        }
        None => {
            warn!(
                "encoder: {} ({}) publishes no encoder capabilities; offering VBR and CBR",
                fourcc, info.name
            );
            vec![
                VideoBitrateMode::VariableBitrate,
                VideoBitrateMode::ConstantBitrate,
            ]
        }
    };

    // Profiles as probed, the codec's own default first; levels as the union over the profiles,
    // highest first (the default, see `ChosenEncoder::default_level`).
    let pairs = profile_pairs(mime);
    let mut profiles: Vec<i32> = Vec::new();
    let mut levels: Vec<i32> = Vec::new();
    for p in &info.profiles {
        if let Some((_, v4l2)) = pairs.iter().find(|(mc, _)| *mc == p.profile) {
            if !profiles.contains(v4l2) {
                profiles.push(*v4l2);
            }
        }
        for l in &p.levels {
            if let Some(v4l2) = level_to_v4l2(mime, l.level) {
                if !levels.contains(&v4l2) {
                    levels.push(v4l2);
                }
            }
        }
    }
    if let Some(preferred) = preferred_profile(mime) {
        if let Some(at) = profiles.iter().position(|p| *p == preferred) {
            profiles.remove(at);
            profiles.insert(0, preferred);
        }
    }
    levels.sort_unstable_by(|a, b| b.cmp(a));
    let qp = has_feature(info, FEATURE_QP_BOUNDS).then_some(QP_RANGE);

    let format = CodedFormat {
        fourcc,
        width,
        height,
        frame_rate,
        bitrate: ControlRange::new(
            bitrate.0,
            bitrate.1,
            DEFAULT_BITRATE.clamp(bitrate.0, bitrate.1),
        ),
        gop_size: GOP_RANGE,
        bitrate_modes,
        profiles,
        levels: levels.clone(),
        qp,
    };
    let codec = ChosenEncoder {
        fourcc,
        mime: mime.to_string(),
        name: info.name.clone(),
        hardware: is_hardware(info),
        vendor: info.is_vendor,
        qp_bounds: qp.is_some(),
        default_level: levels.first().copied(),
    };
    (format, codec)
}

impl VideoEncoderBackend for MediaCodecEncoderBackend {
    type Session = MediaCodecEncoderSession;

    fn capabilities(&self) -> &EncoderCapabilities {
        &self.caps
    }

    fn new_session(&mut self, id: u32, sink: EncoderSink) -> IoctlResult<MediaCodecEncoderSession> {
        Ok(MediaCodecEncoderSession::new(id, sink, self.codecs.clone()))
    }

    fn close_session(&mut self, mut session: MediaCodecEncoderSession) {
        session.stop();
    }
}

// ---------------------------------------------------------------------------------------------
// The session
// ---------------------------------------------------------------------------------------------

/// A raw frame waiting for a codec input slot, or the end of the stream.
enum PendingInput {
    Frame(InputBuffer),
    Eos,
}

/// What the codec delivered and no `CAPTURE` buffer has taken yet, in delivery order. Every
/// coded output is copied out of the codec's buffer as it arrives and the buffer released at
/// once (a coded frame is tens of kilobytes against the megabyte raw copy, and the codec never
/// waits on the guest for an output slot), so what is held here is the session's own and
/// survives a flush.
enum Held {
    /// The stream headers alone (`SEPARATE` mode): a `CAPTURE` buffer of their own.
    Headers { bytes: Vec<u8>, pts_us: i64 },
    /// A coded frame; `is_last` when it carried the codec's `END_OF_STREAM`.
    Coded {
        bytes: Vec<u8>,
        pts_us: i64,
        key: bool,
        is_last: bool,
    },
    /// An empty `LAST` buffer owed to the guest: the `END_OF_STREAM` output was empty.
    EmptyLast { pts_us: i64 },
}

/// Coded frames held back because no `CAPTURE` buffer is lent, beyond which no raw frame is
/// fed: a guest that streams `OUTPUT` without ever queueing a `CAPTURE` buffer would otherwise
/// have the session hold a growing bitstream for it. Its raw frames stay lent, which is the
/// backpressure V4L2 has.
const HELD_OUTPUT_LIMIT: usize = 32;

/// What `start`'s bounded thread hands back.
struct Started {
    codec: Codec,
    input_format: MediaFormat,
    /// The `KEY_COLOR_FORMAT` the codec accepted.
    color_format: i32,
}

/// The input layout the codec published after `configure`, or why the codec is refused. The
/// stride falls back to the width as the framework's own client does
/// (`MediaCodec_sanity_test.cpp:357`); the slice height does not: its absence means the chroma
/// offset is not a whole number of strides, so the `Y[stride * slice_height]` packing would be
/// wrong (design §7.3, `android_codec::InputLayout`).
fn input_layout(
    format: &MediaFormat,
    requested: i32,
    width: u32,
    height: u32,
) -> Result<InputLayout, String> {
    let color_format = format.get_i32(keys::COLOR_FORMAT).unwrap_or(requested);
    let semiplanar = match color_format {
        COLOR_FORMAT_YUV420_SEMI_PLANAR
        | COLOR_FORMAT_YUV420_PACKED_SEMI_PLANAR
        | COLOR_QCOM_FORMAT_YUV420_SEMI_PLANAR => true,
        COLOR_FORMAT_YUV420_PLANAR | COLOR_FORMAT_YUV420_PACKED_PLANAR => false,
        // Read back as flexible: the component takes the semi-planar layout it was asked for
        // (`logs/vpu_wp/B5-acceptance.md` §5.4: byte-identical bitstreams from the two on
        // `c2.qti.avc.encoder`).
        COLOR_FORMAT_YUV420_FLEXIBLE => true,
        other => {
            return Err(format!(
                "wants input color-format {}, which is neither semi-planar nor planar 4:2:0 \
                 (input format: {})",
                color_format_name(other),
                format
            ))
        }
    };
    let stride = format.get_i32(keys::STRIDE).unwrap_or(width as i32);
    let Some(slice_height) = format.get_i32(keys::SLICE_HEIGHT) else {
        return Err(format!(
            "publishes no slice-height for {}x{} (input format: {}), so the chroma offset \
             cannot be derived (design 7.3)",
            width, height, format
        ));
    };
    if stride < width as i32 || slice_height < height as i32 {
        return Err(format!(
            "publishes stride {} and slice-height {} for a {}x{} picture (input format: {})",
            stride, slice_height, width, height, format
        ));
    }
    Ok(InputLayout {
        color_format,
        stride: stride as usize,
        slice_height: slice_height as usize,
        semiplanar,
    })
}

/// `setParameters(request-sync)`: the next frame the component processes is a keyframe -- the
/// same moment a kernel driver's `V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME` takes effect.
fn request_sync(id: u32, codec: &Codec) {
    let result = MediaFormat::new().and_then(|mut params| {
        params.set_i32(keys::REQUEST_SYNC_FRAME, 0);
        codec.set_parameters(&params)
    });
    match result {
        Ok(()) => info!("encoder session {}: keyframe requested", id),
        Err(e) => warn!("encoder session {}: request-sync: {}", id, e),
    }
}

/// One encode: see the module documentation.
pub struct MediaCodecEncoderSession {
    id: u32,
    sink: EncoderSink,
    codecs: Vec<ChosenEncoder>,
    /// The codec, from `start` to `stop`.
    codec: Option<Codec>,
    chosen: Option<ChosenEncoder>,
    /// How the codec wants its input buffers filled, read back after `configure`.
    layout: Option<InputLayout>,
    coded_size: (u32, u32),
    visible: Rect,
    header_mode: VideoHeaderMode,
    /// Input indices the codec offered and nothing has used yet.
    free_inputs: VecDeque<i32>,
    /// Raw frames waiting for an input index, oldest first.
    pending: VecDeque<PendingInput>,
    /// `CAPTURE` buffers lent by the device, oldest first.
    captures: VecDeque<OutputBuffer>,
    /// Coded outputs the codec delivered that no `CAPTURE` buffer has taken yet.
    held_outputs: VecDeque<Held>,
    /// The stream headers in `JOINED_WITH_1ST_FRAME` mode, waiting for the next frame.
    joined_headers: Option<Vec<u8>>,
    /// For the device, in order.
    events: Vec<EncoderEvent>,
    /// An `END_OF_STREAM` input is queued: the codec takes no more input until a flush.
    eos_queued: bool,
    /// The `END_OF_STREAM` output was delivered as `LAST`.
    eos_seen: bool,
    /// The codec is gone (error, timeout); the device has been or is being told.
    dead: bool,
    input_capacity: Option<usize>,
    /// The timestamp of the first frame queued since the codec started: what a headers-only
    /// buffer carries (`V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE`).
    first_pts: Option<i64>,
    /// Since the last flush: indices the codec refused, and stale indices ignored (D23).
    refused: u32,
    stale: u32,
    /// `Codec::stale_events` as of the last flush: what the crate's generation stamp dropped.
    stale_events_seen: u64,
    flushes: u32,
    inputs: u64,
    frames: u64,
    keyframes: u64,
    bytes: u64,
    started_at: Option<Instant>,
}

impl MediaCodecEncoderSession {
    fn new(id: u32, sink: EncoderSink, codecs: Vec<ChosenEncoder>) -> Self {
        Self {
            id,
            sink,
            codecs,
            codec: None,
            chosen: None,
            layout: None,
            coded_size: (0, 0),
            visible: Rect::new(0, 0, 0, 0),
            header_mode: VideoHeaderMode::JoinedWith1stFrame,
            free_inputs: VecDeque::new(),
            pending: VecDeque::new(),
            captures: VecDeque::new(),
            held_outputs: VecDeque::new(),
            joined_headers: None,
            events: Vec::new(),
            eos_queued: false,
            eos_seen: false,
            dead: false,
            input_capacity: None,
            first_pts: None,
            refused: 0,
            stale: 0,
            stale_events_seen: 0,
            flushes: 0,
            inputs: 0,
            frames: 0,
            keyframes: 0,
            bytes: 0,
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
        error!("encoder session {}: {}", self.id, why);
        self.dead = true;
        self.pending.clear();
        self.captures.clear();
        self.held_outputs.clear();
        self.free_inputs.clear();
        self.joined_headers = None;
        self.events.push(EncoderEvent::Error(why));
        self.sink.signal();
    }

    /// A refused input or output index. A null buffer (`CodecError::Null` from `getInputBuffer`
    /// / `getOutputBuffer`) is a stale index -- a callback the NDK looper had queued before a
    /// flush and delivered after it (D23) -- and is skipped: counted for the flush line, a debug
    /// line, never a session error. Anything else is the codec refusing an index it handed out,
    /// tolerated up to [`MAX_REFUSED_INDICES`] per flush.
    fn refused_index(&mut self, what: &str, index: i32, e: &CodecError) {
        if matches!(e, CodecError::Null(_)) {
            self.stale += 1;
            debug!(
                "encoder session {}: stale {} index {} after a flush ignored ({})",
                self.id, what, index, e
            );
            return;
        }
        self.refused += 1;
        if self.refused <= STALE_LOG_LIMIT {
            warn!(
                "encoder session {}: {} index {} refused: {}",
                self.id, what, index, e
            );
        }
        if self.refused > MAX_REFUSED_INDICES {
            self.fail(format!(
                "{} refused {} indices since the last flush; the codec is not taking buffers",
                self.codec_name(),
                self.refused
            ));
        }
    }

    /// Feed the codec from the pending FIFO while it offers input slots and the guest takes
    /// what comes out.
    fn pump_input(&mut self) {
        // The codec is taken out of `self` for the duration: the helpers below need `self`. It
        // goes back at the end, dead or not, so `stop` can stop it.
        let Some(codec) = self.codec.take() else {
            return;
        };
        while !self.dead && !self.eos_queued && self.held_outputs.len() < HELD_OUTPUT_LIMIT {
            let (Some(&index), Some(head)) = (self.free_inputs.front(), self.pending.front())
            else {
                break;
            };
            let Some(layout) = self.layout else {
                break;
            };
            match *head {
                PendingInput::Eos => match codec.queue_eos(index, 0) {
                    Ok(()) => {
                        self.free_inputs.pop_front();
                        self.pending.pop_front();
                        self.eos_queued = true;
                        info!(
                            "encoder session {}: drain: EOS queued after {} frames",
                            self.id, self.inputs
                        );
                    }
                    Err(e) => {
                        self.free_inputs.pop_front();
                        self.refused_index("input", index, &e);
                    }
                },
                PendingInput::Frame(buffer) => {
                    let pts = pts_from(buffer.timestamp);
                    let (coded, visible) = (self.coded_size, self.visible);
                    let result = codec.queue_input_with(index, pts, 0, |dst| {
                        // SAFETY: the device lends `len` readable bytes at `ptr` until
                        // `InputBufferDone`, which is only reported below; `dst` is the codec's
                        // own input buffer; the two never overlap.
                        unsafe {
                            pack_frame(
                                buffer.ptr.as_ptr(),
                                buffer.len,
                                coded,
                                visible,
                                &layout,
                                dst,
                            )
                        }
                    });
                    match result {
                        Ok(()) => {
                            self.free_inputs.pop_front();
                            self.pending.pop_front();
                            self.inputs += 1;
                            if self.first_pts.is_none() {
                                self.first_pts = Some(pts as i64);
                            }
                            // The frame is in the codec's buffer: the guest's goes back.
                            self.events
                                .push(EncoderEvent::InputBufferDone(buffer.index));
                        }
                        // The frame does not fit the codec's layout, or the codec's buffer: a
                        // configuration the session cannot recover from, not a stale index.
                        Err(e @ (CodecError::Fill(_) | CodecError::BufferTooSmall(..))) => {
                            self.fail(format!(
                                "cannot pack OUTPUT buffer {} into codec input {}: {}",
                                buffer.index, index, e
                            ));
                        }
                        Err(e) => {
                            self.free_inputs.pop_front();
                            self.refused_index("input", index, &e);
                        }
                    }
                }
            }
        }
        self.codec = Some(codec);
    }

    /// `onAsyncOutputAvailable`: copy the output out of the codec and give the buffer back,
    /// then hold what it was for a `CAPTURE` buffer.
    fn take_output(&mut self, codec: &mut Codec, index: i32, info: BufferInfo) {
        let is_eos = info.flags & BUFFER_FLAG_END_OF_STREAM != 0;
        let is_config = info.flags & BUFFER_FLAG_CODEC_CONFIG != 0;
        let key = info.flags & BUFFER_FLAG_KEY_FRAME != 0;
        let size = info.size.max(0) as usize;
        let bytes = if size > 0 {
            match codec.output_buffer(index) {
                Ok(src) => src[..size.min(src.len())].to_vec(),
                Err(e) => {
                    self.refused_index("output", index, &e);
                    return;
                }
            }
        } else {
            Vec::new()
        };
        if let Err(e) = codec.release_output(index) {
            self.refused_index("output", index, &e);
        }
        let pts_us = info.presentation_time_us;
        if is_config {
            self.take_headers(bytes, pts_us);
        } else if !bytes.is_empty() {
            self.held_outputs.push_back(Held::Coded {
                bytes,
                pts_us,
                key,
                is_last: is_eos,
            });
            return;
        }
        if is_eos {
            self.held_outputs.push_back(Held::EmptyLast { pts_us });
        }
    }

    /// The stream headers (`BUFFER_FLAG_CODEC_CONFIG`: SPS/PPS, VPS/SPS/PPS), placed as the
    /// device's `V4L2_CID_MPEG_VIDEO_HEADER_MODE` says.
    fn take_headers(&mut self, bytes: Vec<u8>, pts_us: i64) {
        let pts_us = self.first_pts.unwrap_or(pts_us);
        info!(
            "encoder session {}: stream headers: {} bytes, {}",
            self.id,
            bytes.len(),
            match self.header_mode {
                VideoHeaderMode::Separate => "in a CAPTURE buffer of their own",
                VideoHeaderMode::JoinedWith1stFrame => "in front of the next frame",
            }
        );
        match self.header_mode {
            VideoHeaderMode::Separate => {
                self.held_outputs.push_back(Held::Headers { bytes, pts_us })
            }
            VideoHeaderMode::JoinedWith1stFrame => self.joined_headers = Some(bytes),
        }
    }

    /// Write every held output into a lent `CAPTURE` buffer that can take it.
    fn pump_output(&mut self) {
        while !self.dead {
            // Whether the headers go in front of this frame: only in `JOINED` mode, and not
            // when the frame already starts with them (`PREPEND_SPSPPS_TO_IDR` makes the codec
            // do that itself).
            let prefix = match self.held_outputs.front() {
                None => break,
                Some(Held::Coded { bytes, .. }) => match &self.joined_headers {
                    Some(headers) if !bytes.starts_with(headers) => headers.len(),
                    _ => 0,
                },
                Some(Held::Headers { .. } | Held::EmptyLast { .. }) => 0,
            };
            let need = prefix
                + match self.held_outputs.front() {
                    Some(Held::Coded { bytes, .. } | Held::Headers { bytes, .. }) => bytes.len(),
                    _ => 0,
                };
            // The first lent buffer that holds it.
            let Some(at) = self.captures.iter().position(|c| c.len >= need) else {
                if !self.captures.is_empty() {
                    // The kernel would return the buffer with V4L2_BUF_FLAG_ERROR and go on;
                    // the trait has no error flag (M7-crate §9 item 2), and a truncated frame
                    // would be a corrupt stream: the session ends, saying why.
                    let largest = self.captures.iter().map(|c| c.len).max().unwrap_or(0);
                    self.fail(format!(
                        "a {}-byte coded frame fits none of the {} lent CAPTURE buffer(s) (the \
                         largest holds {} bytes): the guest's S_FMT(CAPTURE) sizeimage is too \
                         small for this stream",
                        need,
                        self.captures.len(),
                        largest
                    ));
                }
                break;
            };
            let capture = self.captures.remove(at).expect("position was found above");
            let held = self.held_outputs.pop_front().expect("checked above");
            let out = capture.ptr.as_ptr();
            let (bytesused, pts_us, kind, is_last) = match held {
                Held::Headers { bytes, pts_us } => {
                    // SAFETY: `bytes.len() == need <= capture.len`, and the device lends `len`
                    // writable bytes at `ptr` until `FrameEncoded`; the source is ours.
                    unsafe { std::ptr::copy_nonoverlapping(bytes.as_ptr(), out, bytes.len()) };
                    (bytes.len(), pts_us, FrameKind::Headers, false)
                }
                Held::Coded {
                    bytes,
                    pts_us,
                    key,
                    is_last,
                } => {
                    if prefix > 0 {
                        let headers = self.joined_headers.take().unwrap_or_default();
                        // SAFETY: as above, `prefix + bytes.len() == need <= capture.len`.
                        unsafe {
                            std::ptr::copy_nonoverlapping(headers.as_ptr(), out, headers.len())
                        };
                    } else {
                        // Delivered inside the frame, or not wanted any more.
                        self.joined_headers = None;
                    }
                    // SAFETY: as above.
                    unsafe {
                        std::ptr::copy_nonoverlapping(bytes.as_ptr(), out.add(prefix), bytes.len())
                    };
                    self.frames += 1;
                    self.bytes += (prefix + bytes.len()) as u64;
                    if key {
                        self.keyframes += 1;
                    }
                    (
                        prefix + bytes.len(),
                        pts_us,
                        if key {
                            FrameKind::Key
                        } else {
                            FrameKind::Inter
                        },
                        is_last,
                    )
                }
                Held::EmptyLast { pts_us } => (0, pts_us, FrameKind::Inter, true),
            };
            self.events.push(EncoderEvent::FrameEncoded {
                index: capture.index,
                bytesused: bytesused as u32,
                timestamp: timeval_from(pts_us),
                kind,
                is_last,
            });
            if is_last {
                self.eos_seen = true;
                info!(
                    "encoder session {}: EOS reached after {} coded frames ({} bytes in the LAST \
                     buffer)",
                    self.id, self.frames, bytesused
                );
            }
        }
    }

    /// `AMediaCodec_flush` + `start`, bounded; drops every codec-side index this session holds.
    fn flush_codec(&mut self, what: &'static str) -> IoctlResult<()> {
        let Some(codec) = self.codec.take() else {
            return Ok(());
        };
        self.free_inputs.clear();
        self.eos_queued = false;
        self.eos_seen = false;
        self.refused = 0;
        self.stale = 0;
        let id = self.id;
        match bounded("encoder", id, what, CODEC_FLUSH_TIMEOUT, move || {
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
        if bounded("encoder", id, "stop", CODEC_STOP_TIMEOUT, move || {
            let mut codec = codec;
            if let Err(e) = codec.stop() {
                warn!("encoder session {}: AMediaCodec_stop: {}", id, e);
            }
            drop(codec);
        })
        .is_none()
        {
            warn!(
                "encoder session {}: {} is still stopping on a detached thread",
                id, name
            );
        }
    }
}

impl VideoEncoderBackendSession for MediaCodecEncoderSession {
    fn start(&mut self, config: &EncoderConfig) -> IoctlResult<()> {
        if self.codec.is_some() {
            return Ok(());
        }
        if self.dead {
            return Err(libc::ENODEV);
        }
        let chosen = self
            .codecs
            .iter()
            .find(|c| c.fourcc == config.coded_format)
            .cloned()
            .ok_or(libc::EINVAL)?;
        // The codec is created for the visible picture: that is what it is given (the copy
        // crops), and what the stream says.
        let width = config.visible_rect.width.clamp(1, i32::MAX as u32);
        let height = config.visible_rect.height.clamp(1, i32::MAX as u32);
        let fps = config.frame_rate.num as f32 / config.frame_rate.den.max(1) as f32;
        let mut format = MediaFormat::new().map_err(|e| errno_for(&e))?;
        format
            .set_str(keys::MIME, &chosen.mime)
            .map_err(|e| errno_for(&e))?;
        format.set_i32(keys::WIDTH, width as i32);
        format.set_i32(keys::HEIGHT, height as i32);
        format.set_i32(keys::COLOR_FORMAT, COLOR_FORMAT_YUV420_SEMI_PLANAR);
        format.set_i32(keys::BIT_RATE, config.bitrate.min(i32::MAX as u32) as i32);
        format.set_i32(keys::BITRATE_MODE, bitrate_mode_to_mc(config.bitrate_mode));
        set_number(&mut format, keys::FRAME_RATE, fps);
        set_number(
            &mut format,
            keys::I_FRAME_INTERVAL,
            i_frame_interval(config.gop_size, fps),
        );
        let profile = config.profile.and_then(|p| {
            profile_pairs(&chosen.mime)
                .iter()
                .find(|(_, v4l2)| *v4l2 == p)
                .map(|(mc, _)| *mc)
        });
        if let Some(p) = profile {
            format.set_i32(keys::PROFILE, p);
        }
        let level = config
            .level
            .filter(|l| Some(*l) != chosen.default_level)
            .and_then(|l| level_to_mc(&chosen.mime, l));
        if let Some(l) = level {
            format.set_i32(keys::LEVEL, l);
        }
        if config.prepend_sps_pps_to_idr {
            format.set_i32(keys::PREPEND_HEADER_TO_SYNC_FRAMES, 1);
        }
        if chosen.qp_bounds {
            if let Some((lo, hi)) = config.qp {
                if lo > QP_RANGE.min {
                    format.set_i32(keys::VIDEO_QP_MIN, lo);
                }
                if hi < QP_RANGE.max {
                    format.set_i32(keys::VIDEO_QP_MAX, hi);
                }
            }
        }
        let (standard, range, transfer) = color_aspects(&config.colorimetry);
        if let Some(s) = standard {
            format.set_i32(keys::COLOR_STANDARD, s);
        }
        if let Some(r) = range {
            format.set_i32(keys::COLOR_RANGE, r);
        }
        if let Some(t) = transfer {
            format.set_i32(keys::COLOR_TRANSFER, t);
        }
        info!(
            "encoder session {}: creating {} for {} at {}x{} ({:?} of {}x{}): {}",
            self.id,
            chosen.name,
            config.coded_format,
            width,
            height,
            config.visible_rect,
            config.coded_size.0,
            config.coded_size.1,
            format
        );

        let sink = self.sink.clone();
        let name = chosen.name.clone();
        let id = self.id;
        let created = bounded("encoder", id, "start", CODEC_START_TIMEOUT, move || {
            let mut format = format;
            let mut codec = Codec::create_by_name(&name)?;
            // Every callback bumps the device session's eventfd once it has queued its event:
            // that is the whole wake-up path, no thread in between.
            let hook = sink.clone();
            codec.set_wake_hook(Box::new(move || hook.signal()));
            let color_format = match codec.configure(&format, true) {
                Ok(()) => COLOR_FORMAT_YUV420_SEMI_PLANAR,
                Err(first) => {
                    // A failed configure resets the codec to INITIALIZED
                    // (`MediaCodec::configure`, "configure failed ... resetting"), but whether
                    // the async callback survives that reset is nowhere written down: a fresh
                    // codec is certain, and this path is once per session.
                    info!(
                        "encoder session {}: {} does not take COLOR_FormatYUV420SemiPlanar \
                         ({}); trying COLOR_FormatYUV420Flexible on a fresh codec",
                        id, name, first
                    );
                    drop(codec);
                    codec = Codec::create_by_name(&name)?;
                    codec.set_wake_hook(Box::new(move || sink.signal()));
                    format.set_i32(keys::COLOR_FORMAT, COLOR_FORMAT_YUV420_FLEXIBLE);
                    codec.configure(&format, true)?;
                    COLOR_FORMAT_YUV420_FLEXIBLE
                }
            };
            let input_format = codec.input_format()?;
            codec.start()?;
            Ok::<Started, CodecError>(Started {
                codec,
                input_format,
                color_format,
            })
        });
        match created {
            Some(Ok(started)) => {
                let layout = match input_layout(
                    &started.input_format,
                    started.color_format,
                    width,
                    height,
                ) {
                    Ok(layout) => layout,
                    Err(why) => {
                        error!(
                            "encoder session {}: {} for {} {}; the codec is refused",
                            self.id, chosen.name, config.coded_format, why
                        );
                        self.stop_codec(started.codec);
                        return Err(libc::EINVAL);
                    }
                };
                info!(
                    "encoder session {}: {} ({}{}) started for {} at {}x{}: input {} stride {} \
                     slice-height {} ({}, {} bytes per frame), {} bit/s {:?}, {} fps, gop {}, \
                     headers {}{}{}{}",
                    self.id,
                    started.codec.name(),
                    if chosen.hardware {
                        "hardware"
                    } else {
                        "software"
                    },
                    if chosen.vendor { ", vendor" } else { "" },
                    config.coded_format,
                    width,
                    height,
                    color_format_name(layout.color_format),
                    layout.stride,
                    layout.slice_height,
                    if layout.semiplanar {
                        "semi-planar"
                    } else {
                        "planar"
                    },
                    layout.frame_size(),
                    config.bitrate,
                    config.bitrate_mode,
                    fps,
                    config.gop_size,
                    match config.header_mode {
                        VideoHeaderMode::Separate => "separate",
                        VideoHeaderMode::JoinedWith1stFrame => "joined",
                    },
                    if config.prepend_sps_pps_to_idr {
                        ", prepended to every IDR"
                    } else {
                        ""
                    },
                    profile
                        .map(|p| format!(", profile {:#x}", p))
                        .unwrap_or_default(),
                    level
                        .map(|l| format!(", level {:#x}", l))
                        .unwrap_or_default()
                );
                self.chosen = Some(chosen);
                self.codec = Some(started.codec);
                self.layout = Some(layout);
                self.coded_size = config.coded_size;
                self.visible = config.visible_rect;
                self.header_mode = config.header_mode;
                self.free_inputs.clear();
                self.held_outputs.clear();
                self.joined_headers = None;
                self.eos_queued = false;
                self.eos_seen = false;
                self.input_capacity = None;
                self.first_pts = None;
                self.refused = 0;
                self.stale = 0;
                self.stale_events_seen = 0;
                self.flushes = 0;
                self.inputs = 0;
                self.frames = 0;
                self.keyframes = 0;
                self.bytes = 0;
                self.started_at = Some(Instant::now());
                Ok(())
            }
            Some(Err(e)) => {
                error!(
                    "encoder session {}: cannot start {} for {}: {}",
                    self.id, chosen.name, config.coded_format, e
                );
                Err(errno_for(&e))
            }
            None => Err(libc::ETIMEDOUT),
        }
    }

    fn encode(&mut self, buffer: InputBuffer) -> IoctlResult<()> {
        if self.dead {
            return Err(libc::ENODEV);
        }
        if self.codec.is_none() {
            return Err(libc::EINVAL);
        }
        // A frame queued while a drain is in flight waits here, as the kernel's encoder
        // interface says it must; the device holds the ones after the LAST buffer itself.
        self.pending.push_back(PendingInput::Frame(buffer));
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
        // A CAPTURE buffer may have been what held the input back.
        self.pump_input();
        if self.events.len() > before {
            self.sink.signal();
        }
        Ok(())
    }

    fn set_bitrate(&mut self, bitrate: u32) -> IoctlResult<()> {
        if self.dead {
            return Err(libc::ENODEV);
        }
        // Before the codec exists the device keeps the value for `start`.
        let Some(codec) = self.codec.as_ref() else {
            return Ok(());
        };
        let result = MediaFormat::new().and_then(|mut params| {
            params.set_i32(keys::VIDEO_BITRATE, bitrate.min(i32::MAX as u32) as i32);
            codec.set_parameters(&params)
        });
        match result {
            Ok(()) => {
                info!("encoder session {}: bitrate -> {} bit/s", self.id, bitrate);
                Ok(())
            }
            Err(e) => {
                warn!(
                    "encoder session {}: video-bitrate {}: {}",
                    self.id, bitrate, e
                );
                Err(errno_for(&e))
            }
        }
    }

    fn force_keyframe(&mut self) -> IoctlResult<()> {
        if self.dead {
            return Err(libc::ENODEV);
        }
        // Before the codec exists the first frame is a keyframe anyway.
        if let Some(codec) = self.codec.as_ref() {
            request_sync(self.id, codec);
        }
        Ok(())
    }

    fn flush(&mut self) -> IoctlResult<()> {
        // STREAMOFF(OUTPUT), or V4L2_ENC_CMD_START after a finished drain. The frames still
        // waiting for an input slot are dropped, not reported: on STREAMOFF the device takes
        // every OUTPUT buffer back itself right after this returns, and an `InputBufferDone`
        // delivered later would land on a buffer the guest may have queued again by then (the
        // decoder's rule, for the same reason). The coded frames already copied out stay: the
        // CAPTURE queue keeps streaming.
        let pending = self.pending.len();
        self.pending.clear();
        self.events
            .retain(|e| !matches!(e, EncoderEvent::InputBufferDone(_)));
        let held = self.held_outputs.len();
        let refused = self.refused;
        let stale = self.stale;
        self.flushes += 1;
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
            "encoder session {}: flush #{}: {} pending frame(s) dropped, {} held coded frame(s) \
             kept, {} stale index(es) ignored and {} refused since the last flush, {} queued \
             event(s) dropped by this flush{}",
            self.id,
            self.flushes,
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
        self.joined_headers = None;
        // The device reads `InputBufferDone` after this call to return the raw frames the codec
        // finished (its `stop_codec`), so those stay; a `FrameEncoded` would land on a CAPTURE
        // buffer the device is taking back, so those go.
        self.events
            .retain(|e| matches!(e, EncoderEvent::InputBufferDone(_) | EncoderEvent::Error(_)));
        self.layout = None;
        self.eos_queued = false;
        self.eos_seen = false;
        self.input_capacity = None;
        self.first_pts = None;
        if let Some(codec) = self.codec.take() {
            let name = codec.name().to_string();
            self.stop_codec(codec);
            info!(
                "encoder session {}: {} stopped after {:?}: {} frames in, {} coded frames out \
                 ({} keyframes, {} bytes), {} flush(es)",
                self.id,
                name,
                self.started_at
                    .map(|t| t.elapsed())
                    .unwrap_or(Duration::ZERO),
                self.inputs,
                self.frames,
                self.keyframes,
                self.bytes,
                self.flushes
            );
        }
    }

    fn take_events(&mut self) -> Vec<EncoderEvent> {
        if let Some(mut codec) = self.codec.take() {
            for event in codec.take_events() {
                match event {
                    CodecEvent::InputAvailable(index) => {
                        if self.input_capacity.is_none() {
                            if let Ok(capacity) = codec.input_capacity(index) {
                                self.input_capacity = Some(capacity);
                                info!(
                                    "encoder session {}: input buffers hold {} bytes",
                                    self.id, capacity
                                );
                                if let Some(layout) = self.layout {
                                    if capacity < layout.frame_size() {
                                        self.fail(format!(
                                            "{} offers {}-byte input buffers for a padded frame \
                                             of {} bytes (stride {}, slice-height {})",
                                            codec.name(),
                                            capacity,
                                            layout.frame_size(),
                                            layout.stride,
                                            layout.slice_height
                                        ));
                                        break;
                                    }
                                }
                            }
                        }
                        self.free_inputs.push_back(index);
                    }
                    CodecEvent::OutputAvailable { index, info } => {
                        self.take_output(&mut codec, index, info);
                    }
                    CodecEvent::FormatChanged(format) => {
                        info!(
                            "encoder session {}: output format: {}",
                            self.id,
                            format
                                .map(|f| f.to_string())
                                .unwrap_or_else(|| "-".to_string())
                        );
                    }
                    CodecEvent::Error {
                        status,
                        action_code,
                        detail,
                    } => {
                        self.fail(format!(
                            "{} reported error {} ({}), action {}: {}",
                            codec.name(),
                            status,
                            media_status_name(status),
                            action_code,
                            detail
                        ));
                        break;
                    }
                }
            }
            self.codec = Some(codec);
        }
        if !self.dead {
            // Output first: a coded frame delivered frees the input the limit was holding.
            self.pump_output();
            self.pump_input();
        }
        std::mem::take(&mut self.events)
    }
}

impl Drop for MediaCodecEncoderSession {
    fn drop(&mut self) {
        // `stop` is the normal path (the device calls it before `close_session`); a session
        // dropped without it still must not run `AMediaCodec_delete` unbounded on the worker.
        self.stop();
    }
}
