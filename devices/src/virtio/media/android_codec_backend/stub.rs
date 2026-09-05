// Copyright 2026 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! What `MediaCodecDecoderBackend` is on a host without the MediaCodec NDK: a constructor that
//! says so. The types exist so the decoder device instantiates on every target and the helper's
//! `kind=decoder` arm is the same code everywhere; nothing here is ever reached past `new`.

use anyhow::bail;
use virtio_media::devices::video_decoder::DecoderCapabilities;
use virtio_media::devices::video_decoder::DecoderEvent;
use virtio_media::devices::video_decoder::DecoderSink;
use virtio_media::devices::video_decoder::InputBuffer;
use virtio_media::devices::video_decoder::OutputBuffer;
use virtio_media::devices::video_decoder::VideoDecoderBackend;
use virtio_media::devices::video_decoder::VideoDecoderBackendSession;
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

    fn decode(&mut self, _buffer: InputBuffer) -> IoctlResult<()> {
        match self._never {}
    }

    fn use_as_capture(&mut self, _buffer: OutputBuffer) -> IoctlResult<()> {
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
