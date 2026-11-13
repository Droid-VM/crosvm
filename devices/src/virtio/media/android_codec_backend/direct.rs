// Copyright 2026 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! Who decodes for `--virtio-media kind=decoder` (`backend=`): the platform's codec service
//! (MediaCodec, [`super::android`]), or a *direct driver* -- the SoC's own V4L2 codec driver,
//! opened by this process, which writes each frame straight into the guest's buffer instead of
//! into a block of the codec component's that is then converted and copied.
//!
//! A direct driver is one implementation of [`DirectDriver`] and one line in [`DRIVERS`];
//! Qualcomm's msm_vidc ([`super::vidc`]) is the first. What a driver has to bring is small, because
//! the session logic here is the same for all of them: how its device node is found, which coded
//! formats it takes, and a [`DirectSession`] that feeds it bitstream and guest buffers.
//!
//! [`HwDecoderBackend`] is what the decoder device is given, whatever was chosen. It always holds
//! the MediaCodec backend -- the device's capabilities are MediaCodec's, and every session a
//! driver does not take is MediaCodec's -- and, by [`DecoderBackendKind`]:
//! - `mediacodec`: nothing else; the device is exactly the MediaCodec one.
//! - a named driver (`msm-vidc`): that driver for every session whose coded format it takes and
//!   that it opens; the rest fall back to MediaCodec.
//! - `auto`: the first driver of [`DRIVERS`] this host has, for the sessions that ask for what
//!   only a direct driver gives -- frames in decode order, which a guest asks for with the
//!   display-delay controls -- and MediaCodec for every other session. A guest that does not know
//!   the direct path is then served exactly as with `mediacodec`: a direct driver needs the guest
//!   to lend `CAPTURE` buffers of its own memory ([`OutputBuffer::dmabuf`]) and to read the
//!   driver's layout ([`DecoderEvent::CaptureLayout`]), which a guest that sets those controls
//!   does.

use std::os::fd::FromRawFd;
use std::os::fd::OwnedFd;

use base::info;
use base::warn;
use virtio_media::devices::video_decoder::DecoderCapabilities;
use virtio_media::devices::video_decoder::DecoderEvent;
use virtio_media::devices::video_decoder::DecoderSink;
use virtio_media::devices::video_decoder::InputBuffer;
use virtio_media::devices::video_decoder::OutputBuffer;
use virtio_media::devices::video_decoder::VideoDecoderBackend;
use virtio_media::devices::video_decoder::VideoDecoderBackendSession;
use virtio_media::ioctl::IoctlResult;
use virtio_media::v4l2r::bindings;
use virtio_media::v4l2r::PixelFormat;

use super::android::MediaCodecDecoderBackend;
use super::android::MediaCodecDecoderSession;
use crate::virtio::media::DecoderBackendKind;

/// A SoC's own V4L2 decoder driver.
pub trait DirectDriver: Sync {
    /// The `backend=` name ([`DecoderBackendKind::direct_driver`]).
    fn name(&self) -> &'static str;

    /// Whether this host has the driver: its device node opens and answers as that driver. Asked
    /// once, for `backend=auto`; a driver named outright is simply tried at each session.
    fn present(&self) -> bool;

    /// The coded formats the driver decodes.
    fn takes(&self, coded: PixelFormat) -> bool;

    /// Open the driver for one session of `coded` at `size`. `display_delay` is what the guest
    /// asked with the display-delay controls ([`VideoDecoderBackendSession::set_display_delay`]);
    /// a driver that cannot keep it must fail here.
    fn start(
        &self,
        id: u32,
        sink: DecoderSink,
        coded: PixelFormat,
        size: (u32, u32),
        display_delay: Option<u32>,
    ) -> Result<Box<dyn DirectSession>, String>;
}

/// One decoding session on a [`DirectDriver`].
pub trait DirectSession: Send {
    /// An access unit. The bytes are a copy: the guest's buffer has already gone back.
    fn decode(&mut self, bitstream: Vec<u8>, timestamp: bindings::timeval) -> IoctlResult<()>;

    /// A guest `CAPTURE` buffer for the driver to write: `dmabuf` is over the guest's own pages
    /// and stays the session's until the buffer comes back.
    fn use_as_capture(&mut self, index: u32, dmabuf: OwnedFd, len: u32) -> IoctlResult<()>;

    fn clear_capture_buffers(&mut self) -> IoctlResult<()>;
    fn flush(&mut self) -> IoctlResult<()>;
    fn drain(&mut self) -> IoctlResult<()>;
    fn resume(&mut self);
    fn stop(&mut self);

    /// The events for the device since the last call, in order.
    fn take_events(&mut self) -> Vec<DecoderEvent>;

    /// An event of the wrapper's own (an input buffer given back), in line with the session's.
    fn push_event(&mut self, event: DecoderEvent);
}

/// Every direct driver there is, in the order `backend=auto` looks for one.
pub static DRIVERS: &[&dyn DirectDriver] = &[&super::vidc::MsmVidc];

/// Which sessions go to the direct driver.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DirectWhen {
    /// Every session whose coded format the driver takes (a driver named outright).
    Always,
    /// Only the sessions that asked for decode order (`backend=auto`).
    DecodeOrder,
}

/// The MediaCodec decoder backend, with a direct driver in front of it where one was chosen.
#[derive(Clone)]
pub struct HwDecoderBackend {
    codec: MediaCodecDecoderBackend,
    direct: Option<(&'static dyn DirectDriver, DirectWhen)>,
}

impl HwDecoderBackend {
    pub fn new(codec: MediaCodecDecoderBackend, kind: DecoderBackendKind) -> Self {
        let direct = match kind {
            DecoderBackendKind::Mediacodec => None,
            DecoderBackendKind::Auto => {
                let found = DRIVERS.iter().copied().find(|driver| driver.present());
                match found {
                    Some(driver) => info!(
                        "decoder: backend=auto: {} for the sessions that ask for decode order, \
                         MediaCodec for the rest",
                        driver.name()
                    ),
                    None => info!("decoder: backend=auto: no direct driver on this host; MediaCodec"),
                }
                found.map(|driver| (driver, DirectWhen::DecodeOrder))
            }
            named => {
                let found = DRIVERS
                    .iter()
                    .copied()
                    .find(|driver| Some(driver.name()) == named.direct_driver());
                match found {
                    Some(driver) => {
                        info!("decoder: {} first, MediaCodec where it cannot", driver.name())
                    }
                    None => warn!(
                        "decoder: backend={} names no driver this build has; MediaCodec",
                        named.as_str()
                    ),
                }
                found.map(|driver| (driver, DirectWhen::Always))
            }
        };
        Self { codec, direct }
    }
}

pub struct HwDecoderSession {
    id: u32,
    sink: DecoderSink,
    driver: Option<(&'static dyn DirectDriver, DirectWhen)>,
    /// The display delay the guest asked for (`set_display_delay`); `Some`, the session is a
    /// direct driver's or nothing: MediaCodec puts frames out in display order only.
    display_delay: Option<u32>,
    direct: Option<Box<dyn DirectSession>>,
    codec: MediaCodecDecoderSession,
}

impl VideoDecoderBackend for HwDecoderBackend {
    type Session = HwDecoderSession;

    fn capabilities(&self) -> &DecoderCapabilities {
        self.codec.capabilities()
    }

    fn new_session(&mut self, id: u32, sink: DecoderSink) -> IoctlResult<HwDecoderSession> {
        Ok(HwDecoderSession {
            id,
            sink: sink.clone(),
            driver: self.direct,
            display_delay: None,
            direct: None,
            codec: self.codec.new_session(id, sink)?,
        })
    }

    fn close_session(&mut self, mut session: HwDecoderSession) {
        if let Some(mut direct) = session.direct.take() {
            direct.stop();
        }
        self.codec.close_session(session.codec);
    }

    fn min_capture_buffers(&self, fourcc: PixelFormat) -> Option<u32> {
        self.codec.min_capture_buffers(fourcc)
    }

    fn decode_order(&self) -> bool {
        self.direct.is_some()
    }
}

impl VideoDecoderBackendSession for HwDecoderSession {
    fn start(&mut self, coded_format: PixelFormat, coded_size: (u32, u32)) -> IoctlResult<()> {
        let wanted = self.driver.filter(|(driver, when)| {
            driver.takes(coded_format)
                && (*when == DirectWhen::Always || self.display_delay.is_some())
        });
        if let Some((driver, _)) = wanted {
            match driver.start(
                self.id,
                self.sink.clone(),
                coded_format,
                coded_size,
                self.display_delay,
            ) {
                Ok(direct) => {
                    self.direct = Some(direct);
                    return Ok(());
                }
                Err(e) if self.display_delay.is_some() => {
                    warn!(
                        "decoder session {}: {} refused ({}), and decode order was asked for",
                        self.id,
                        driver.name(),
                        e
                    );
                    return Err(libc::EIO);
                }
                Err(e) => warn!(
                    "decoder session {}: {} refused ({}); MediaCodec instead",
                    self.id,
                    driver.name(),
                    e
                ),
            }
        }
        if self.display_delay.is_some() {
            warn!(
                "decoder session {}: decode order asked for, only a direct driver gives it",
                self.id
            );
            return Err(libc::EINVAL);
        }
        self.codec.start(coded_format, coded_size)
    }

    fn set_display_delay(&mut self, delay: Option<u32>) -> IoctlResult<()> {
        if self.driver.is_none() {
            return Err(libc::EINVAL);
        }
        if self.direct.is_some() {
            return Err(libc::EBUSY);
        }
        self.display_delay = delay;
        Ok(())
    }

    fn decode(&mut self, buffer: InputBuffer) -> IoctlResult<()> {
        let Some(direct) = &mut self.direct else {
            return self.codec.decode(buffer);
        };
        // The bitstream is copied out at once (an access unit is small) and its buffer given
        // straight back, so nothing of the guest's is held across a seek.
        // SAFETY: the device lends `len` readable bytes at `ptr` until `InputBufferDone`.
        let bytes = unsafe { std::slice::from_raw_parts(buffer.ptr.as_ptr(), buffer.len) }.to_vec();
        direct.decode(bytes, buffer.timestamp)?;
        direct.push_event(DecoderEvent::InputBufferDone(buffer.index));
        self.sink.signal();
        Ok(())
    }

    fn use_as_capture(&mut self, buffer: OutputBuffer) -> IoctlResult<()> {
        let Some(direct) = &mut self.direct else {
            return self.codec.use_as_capture(buffer);
        };
        let Some(raw) = buffer.dmabuf else {
            warn!(
                "decoder session {}: CAPTURE buffer {} has no dma-buf (it is not the guest's own \
                 memory); a direct driver cannot write it",
                self.id, buffer.index
            );
            return Err(libc::EINVAL);
        };
        // SAFETY: `raw` is open while the buffer is lent; the copy is ours to close.
        let fd = unsafe { libc::fcntl(raw, libc::F_DUPFD_CLOEXEC, 0) };
        if fd < 0 {
            return Err(std::io::Error::last_os_error()
                .raw_os_error()
                .unwrap_or(libc::EIO));
        }
        // SAFETY: just duplicated, owned by no one else.
        let fd = unsafe { OwnedFd::from_raw_fd(fd) };
        direct.use_as_capture(
            buffer.index,
            fd,
            buffer.len.min(u32::MAX as usize) as u32,
        )
    }

    fn clear_capture_buffers(&mut self) -> IoctlResult<()> {
        match &mut self.direct {
            Some(direct) => direct.clear_capture_buffers(),
            None => self.codec.clear_capture_buffers(),
        }
    }

    fn flush(&mut self) -> IoctlResult<()> {
        match &mut self.direct {
            Some(direct) => direct.flush(),
            None => self.codec.flush(),
        }
    }

    fn reinit(&mut self) -> IoctlResult<()> {
        match &self.direct {
            Some(_) => Ok(()),
            None => self.codec.reinit(),
        }
    }

    fn drain(&mut self) -> IoctlResult<()> {
        match &mut self.direct {
            Some(direct) => direct.drain(),
            None => self.codec.drain(),
        }
    }

    fn resume(&mut self) {
        match &mut self.direct {
            Some(direct) => direct.resume(),
            None => self.codec.resume(),
        }
    }

    fn stop(&mut self) {
        if let Some(mut direct) = self.direct.take() {
            direct.stop();
        }
        self.codec.stop();
    }

    fn take_events(&mut self) -> Vec<DecoderEvent> {
        match &mut self.direct {
            Some(direct) => direct.take_events(),
            None => self.codec.take_events(),
        }
    }

    fn wants_dmabuf(&self) -> bool {
        self.direct.is_some()
    }
}
