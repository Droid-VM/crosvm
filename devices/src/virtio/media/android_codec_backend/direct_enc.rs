// Copyright 2026 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! Who encodes for `--virtio-media kind=encoder` (`backend=`, the decoder's key and values): the
//! platform's codec service (MediaCodec, [`super::android_encoder`]), or a *direct driver* -- the
//! SoC's own V4L2 encoder, opened by this process ([`super::vidc_enc`] for Qualcomm's msm_vidc).
//!
//! The guest sees the same stateful V4L2 encoder whichever it is: tightly packed NV12 in, Annex B
//! out, the device's controls. So unlike the decoder's, the encoder's `auto` needs nothing of the
//! guest: it is the direct driver this host has, for every session whose settings the driver
//! takes ([`DirectEncoderDriver::takes`]), and MediaCodec for the rest and on a host without one.
//! A named driver (`msm-vidc`) is the same, except that it is tried without asking whether the
//! host has it. Either way a session the driver refuses at `STREAMON` is MediaCodec's.
//!
//! What a direct driver brings: the controls reach the hardware as V4L2 controls (the bitrate the
//! guest asks is the one the stream gets, changed mid-stream with the same control), the
//! encoder's latency is one hardware pass instead of a trip through the codec service, and the
//! capabilities stay MediaCodec's -- the same hardware, as the platform describes it.

use base::info;
use base::warn;
use virtio_media::devices::video_encoder::EncoderCapabilities;
use virtio_media::devices::video_encoder::EncoderConfig;
use virtio_media::devices::video_encoder::EncoderEvent;
use virtio_media::devices::video_encoder::EncoderSink;
use virtio_media::devices::video_encoder::InputBuffer;
use virtio_media::devices::video_encoder::OutputBuffer;
use virtio_media::devices::video_encoder::VideoEncoderBackend;
use virtio_media::devices::video_encoder::VideoEncoderBackendSession;
use virtio_media::ioctl::IoctlResult;
use virtio_media::v4l2r::PixelFormat;

use super::android_encoder::MediaCodecEncoderBackend;
use super::android_encoder::MediaCodecEncoderSession;
use crate::virtio::media::DecoderBackendKind;

/// A SoC's own V4L2 encoder driver.
pub trait DirectEncoderDriver: Sync {
    /// The `backend=` name ([`DecoderBackendKind::direct_driver`]).
    fn name(&self) -> &'static str;

    /// Whether this host has the driver. Asked once, for `backend=auto`.
    fn present(&self) -> bool;

    /// Whether the driver takes a session of `config` (its coded format, its bitrate mode); the
    /// reason when it does not.
    fn takes(&self, config: &EncoderConfig) -> Result<(), String>;

    /// Open the driver for one session of `config`.
    fn start(
        &self,
        id: u32,
        sink: EncoderSink,
        config: &EncoderConfig,
    ) -> Result<Box<dyn VideoEncoderBackendSession + Send>, String>;
}

/// Every direct encoder there is, in the order `backend=auto` looks for one.
pub static ENCODERS: &[&dyn DirectEncoderDriver] = &[&super::vidc_enc::MsmVidcEncoder];

/// The MediaCodec encoder backend, with a direct driver in front of it where one was chosen.
#[derive(Clone)]
pub struct HwEncoderBackend {
    codec: MediaCodecEncoderBackend,
    direct: Option<&'static dyn DirectEncoderDriver>,
}

impl HwEncoderBackend {
    pub fn new(codec: MediaCodecEncoderBackend, kind: DecoderBackendKind) -> Self {
        let direct = match kind {
            DecoderBackendKind::Mediacodec => None,
            DecoderBackendKind::Auto => {
                let found = ENCODERS.iter().copied().find(|driver| driver.present());
                match found {
                    Some(driver) => info!(
                        "encoder: backend=auto: {} for the sessions it takes, MediaCodec for the rest",
                        driver.name()
                    ),
                    None => info!("encoder: backend=auto: no direct driver on this host; MediaCodec"),
                }
                found
            }
            named => {
                let found = ENCODERS
                    .iter()
                    .copied()
                    .find(|driver| Some(driver.name()) == named.direct_driver());
                match found {
                    Some(driver) => {
                        info!("encoder: {} first, MediaCodec where it cannot", driver.name())
                    }
                    None => warn!(
                        "encoder: backend={} names no driver this build has; MediaCodec",
                        named.as_str()
                    ),
                }
                found
            }
        };
        Self { codec, direct }
    }
}

pub struct HwEncoderSession {
    id: u32,
    sink: EncoderSink,
    driver: Option<&'static dyn DirectEncoderDriver>,
    direct: Option<Box<dyn VideoEncoderBackendSession + Send>>,
    codec: MediaCodecEncoderSession,
    /// What a stopped direct session still had for the device: the raw frames it finished
    /// (`InputBufferDone`) and its errors, which the device reads after `stop`, as it does
    /// MediaCodec's.
    leftover: Vec<EncoderEvent>,
}

impl VideoEncoderBackend for HwEncoderBackend {
    type Session = HwEncoderSession;

    fn capabilities(&self) -> &EncoderCapabilities {
        self.codec.capabilities()
    }

    fn new_session(&mut self, id: u32, sink: EncoderSink) -> IoctlResult<HwEncoderSession> {
        Ok(HwEncoderSession {
            id,
            sink: sink.clone(),
            driver: self.direct,
            direct: None,
            codec: self.codec.new_session(id, sink)?,
            leftover: Vec::new(),
        })
    }

    fn close_session(&mut self, mut session: HwEncoderSession) {
        if let Some(mut direct) = session.direct.take() {
            direct.stop();
        }
        self.codec.close_session(session.codec);
    }

    fn min_capture_buffers(&self, fourcc: PixelFormat) -> Option<u32> {
        self.codec.min_capture_buffers(fourcc)
    }
}

impl HwEncoderSession {
    fn stop_direct(&mut self) {
        if let Some(mut direct) = self.direct.take() {
            direct.stop();
            self.leftover.extend(
                direct
                    .take_events()
                    .into_iter()
                    .filter(|e| matches!(e, EncoderEvent::InputBufferDone(_) | EncoderEvent::Error(_))),
            );
        }
    }

    fn backend(&mut self) -> &mut dyn VideoEncoderBackendSession {
        match &mut self.direct {
            Some(direct) => direct.as_mut(),
            None => &mut self.codec,
        }
    }
}

impl VideoEncoderBackendSession for HwEncoderSession {
    fn start(&mut self, config: &EncoderConfig) -> IoctlResult<()> {
        self.stop_direct();
        if let Some(driver) = self.driver {
            match driver.takes(config).and_then(|()| driver.start(self.id, self.sink.clone(), config)) {
                Ok(direct) => {
                    self.direct = Some(direct);
                    return Ok(());
                }
                Err(e) => warn!(
                    "encoder session {}: {} refused ({}); MediaCodec instead",
                    self.id,
                    driver.name(),
                    e
                ),
            }
        }
        self.codec.start(config)
    }

    fn encode(&mut self, buffer: InputBuffer) -> IoctlResult<()> {
        self.backend().encode(buffer)
    }

    fn use_as_capture(&mut self, buffer: OutputBuffer) -> IoctlResult<()> {
        self.backend().use_as_capture(buffer)
    }

    fn set_bitrate(&mut self, bitrate: u32) -> IoctlResult<()> {
        // Before a session starts the device keeps the value for `start`; MediaCodec's session
        // takes it then the same way.
        self.backend().set_bitrate(bitrate)
    }

    fn force_keyframe(&mut self) -> IoctlResult<()> {
        self.backend().force_keyframe()
    }

    fn drain_for_streamoff(&mut self) {
        self.backend().drain_for_streamoff()
    }

    fn flush(&mut self) -> IoctlResult<()> {
        self.backend().flush()
    }

    fn drain(&mut self) -> IoctlResult<()> {
        self.backend().drain()
    }

    fn stop(&mut self) {
        self.stop_direct();
        self.codec.stop();
    }

    fn take_events(&mut self) -> Vec<EncoderEvent> {
        // A direct session's events, and whatever MediaCodec's last session still had.
        let mut events = std::mem::take(&mut self.leftover);
        events.extend(self.codec.take_events());
        if let Some(direct) = &mut self.direct {
            events.extend(direct.take_events());
        }
        events
    }
}
