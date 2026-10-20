// Copyright 2026 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! What `MediaCodecDecoderBackend` and `MediaCodecEncoderBackend` are on a host without the
//! MediaCodec NDK: constructors that say so. The types exist so the two codec devices
//! instantiate on every target and the helper's `kind=decoder` / `kind=encoder` arms are the
//! same code everywhere; nothing here is ever reached past `new`.

use anyhow::bail;
use virtio_media::devices::video_decoder;
use virtio_media::devices::video_decoder::DecoderCapabilities;
use virtio_media::devices::video_decoder::DecoderEvent;
use virtio_media::devices::video_decoder::DecoderSink;
use virtio_media::devices::video_decoder::VideoDecoderBackend;
use virtio_media::devices::video_decoder::VideoDecoderBackendSession;
use virtio_media::devices::video_encoder;
use virtio_media::devices::video_encoder::EncoderCapabilities;
use virtio_media::devices::video_encoder::EncoderConfig;
use virtio_media::devices::video_encoder::EncoderEvent;
use virtio_media::devices::video_encoder::EncoderSink;
use virtio_media::devices::video_encoder::VideoEncoderBackend;
use virtio_media::devices::video_encoder::VideoEncoderBackendSession;
use virtio_media::ioctl::IoctlResult;
use virtio_media::v4l2r::PixelFormat;

/// A decoder this build of crosvm cannot reach.
#[derive(Clone, Debug)]
pub struct MediaCodecDecoderBackend {
    caps: DecoderCapabilities,
}

impl MediaCodecDecoderBackend {
    /// Always fails: the MediaCodec NDK exists on Android only.
    pub fn new(allow_sw: bool) -> anyhow::Result<Self> {
        bail!(
            "the decoder device (allow_sw={}) needs the Android MediaCodec NDK, which this build \
             of crosvm has no access to",
            allow_sw
        )
    }
}

/// Never constructed.
pub struct MediaCodecDecoderSession {
    _never: std::convert::Infallible,
}

impl VideoDecoderBackendSession for MediaCodecDecoderSession {
    fn start(&mut self, _coded_format: PixelFormat, _coded_size: (u32, u32)) -> IoctlResult<()> {
        match self._never {}
    }

    fn decode(&mut self, _buffer: video_decoder::InputBuffer) -> IoctlResult<()> {
        match self._never {}
    }

    fn use_as_capture(&mut self, _buffer: video_decoder::OutputBuffer) -> IoctlResult<()> {
        match self._never {}
    }

    fn clear_capture_buffers(&mut self) -> IoctlResult<()> {
        match self._never {}
    }

    fn flush(&mut self) -> IoctlResult<()> {
        match self._never {}
    }

    fn drain(&mut self) -> IoctlResult<()> {
        match self._never {}
    }

    fn stop(&mut self) {
        match self._never {}
    }

    fn take_events(&mut self) -> Vec<DecoderEvent> {
        match self._never {}
    }
}

impl VideoDecoderBackend for MediaCodecDecoderBackend {
    type Session = MediaCodecDecoderSession;

    fn capabilities(&self) -> &DecoderCapabilities {
        &self.caps
    }

    fn new_session(&mut self, _id: u32, _sink: DecoderSink) -> IoctlResult<Self::Session> {
        Err(libc::ENODEV)
    }

    fn close_session(&mut self, session: Self::Session) {
        match session._never {}
    }
}

/// An encoder this build of crosvm cannot reach.
#[derive(Clone, Debug)]
pub struct MediaCodecEncoderBackend {
    caps: EncoderCapabilities,
}

impl MediaCodecEncoderBackend {
    /// Always fails: the MediaCodec NDK exists on Android only.
    pub fn new(allow_sw: bool) -> anyhow::Result<Self> {
        bail!(
            "the encoder device (allow_sw={}) needs the Android MediaCodec NDK, which this build \
             of crosvm has no access to",
            allow_sw
        )
    }
}

/// Never constructed.
pub struct MediaCodecEncoderSession {
    _never: std::convert::Infallible,
}

impl VideoEncoderBackendSession for MediaCodecEncoderSession {
    fn start(&mut self, _config: &EncoderConfig) -> IoctlResult<()> {
        match self._never {}
    }

    fn encode(&mut self, _buffer: video_encoder::InputBuffer) -> IoctlResult<()> {
        match self._never {}
    }

    fn use_as_capture(&mut self, _buffer: video_encoder::OutputBuffer) -> IoctlResult<()> {
        match self._never {}
    }

    fn set_bitrate(&mut self, _bitrate: u32) -> IoctlResult<()> {
        match self._never {}
    }

    fn force_keyframe(&mut self) -> IoctlResult<()> {
        match self._never {}
    }

    fn flush(&mut self) -> IoctlResult<()> {
        match self._never {}
    }

    fn drain(&mut self) -> IoctlResult<()> {
        match self._never {}
    }

    fn stop(&mut self) {
        match self._never {}
    }

    fn take_events(&mut self) -> Vec<EncoderEvent> {
        match self._never {}
    }
}

impl VideoEncoderBackend for MediaCodecEncoderBackend {
    type Session = MediaCodecEncoderSession;

    fn capabilities(&self) -> &EncoderCapabilities {
        &self.caps
    }

    fn new_session(&mut self, _id: u32, _sink: EncoderSink) -> IoctlResult<Self::Session> {
        Err(libc::ENODEV)
    }

    fn close_session(&mut self, session: Self::Session) {
        match session._never {}
    }
}
