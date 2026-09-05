// Copyright 2026 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! What `AndroidCameraBackend` is on a host without the Camera2 NDK: a constructor that says
//! so. The types exist so the camera device instantiates on every target and the helper's
//! `kind=camera` arm is the same code everywhere; nothing here is ever reached past `new`.

use anyhow::bail;
use virtio_media::devices::camera::CameraBackend;
use virtio_media::devices::camera::CameraControl;
use virtio_media::devices::camera::CameraEvent;
use virtio_media::devices::camera::CameraInfo;
use virtio_media::devices::camera::CameraStream;
use virtio_media::devices::camera::CaptureSink;
use virtio_media::devices::camera::EmptyBuffer;
use virtio_media::devices::camera::FilledBuffer;
use virtio_media::devices::camera::StreamRequest;

/// A camera this build of crosvm cannot reach.
#[derive(Clone, Debug)]
pub struct AndroidCameraBackend {
    info: CameraInfo,
}

impl AndroidCameraBackend {
    /// Always fails: the Camera2 NDK exists on Android only.
    pub fn new(camera_id: Option<&str>) -> anyhow::Result<Self> {
        bail!(
            "the camera device (camera_id={:?}) needs the Android camera NDK, which this build \
             of crosvm has no access to",
            camera_id
        )
    }
}

/// Never constructed.
pub struct AndroidCameraStream {
    _never: std::convert::Infallible,
}

impl CameraStream for AndroidCameraStream {
    fn give_empty(&mut self, _buffer: EmptyBuffer) -> Result<(), i32> {
        match self._never {}
    }

    fn take_filled(&mut self) -> Vec<FilledBuffer> {
        match self._never {}
    }

    fn take_events(&mut self) -> Vec<CameraEvent> {
        match self._never {}
    }

    fn set_controls(&mut self, _controls: &[CameraControl]) -> Result<(), i32> {
        match self._never {}
    }

    fn close(self) {
        match self._never {}
    }
}

impl CameraBackend for AndroidCameraBackend {
    type Stream = AndroidCameraStream;

    fn info(&self) -> &CameraInfo {
        &self.info
    }

    fn open_stream(
        &mut self,
        _request: StreamRequest,
        _sink: CaptureSink,
    ) -> Result<AndroidCameraStream, i32> {
        Err(libc::ENODEV)
    }
}
