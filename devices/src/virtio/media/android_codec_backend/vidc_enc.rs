// Copyright 2026 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! The encoder straight through Qualcomm's msm_vidc V4L2 driver (`/dev/video33`), for
//! `--virtio-media kind=encoder,backend=msm-vidc|auto` ([`super::direct_enc`]).
//!
//! What the driver wants, as measured on the phone (SM8850, UniXNU-droidvm tools/rig/vencprobe.c):
//! - The kernel's stateful encoder order: `S_FMT(CAPTURE)` (the codec), `S_FMT(OUTPUT)` (NV12 of
//!   the size; the coded size follows, and with it the bitstream `sizeimage`, so `CAPTURE` is read
//!   again after it), `S_PARM(OUTPUT)`, the controls, both queues' buffers, `STREAMON` both.
//! - Neither queue has `MMAP` (`mmap` of a `CAPTURE` buffer fails `ENODEV`): raw frames and
//!   bitstream both go in dma-heap buffers of ours, sized at least the `sizeimage` each queue
//!   reports, or `QBUF` fails `EINVAL`.
//! - The linear NV12 it takes has rows of `bytesperline` (the width aligned to 128) and the
//!   chroma after `sizeimage / bytesperline * 2 / 3` rows (the height aligned to 32); the visible
//!   size is the `S_FMT` size, the driver crops to it by itself.
//! - The parameter sets always come in front of the first key frame, whatever
//!   `HEADER_MODE` says; the guest's `SEPARATE` is made here, by cutting them off into a
//!   buffer of their own.
//! - Right after `STREAMON` it hands back every `CAPTURE` buffer once, empty and unflagged; those
//!   go straight back in.
//! - `V4L2_ENC_CMD_STOP` drains, the last buffer flagged `LAST` (empty), and `CAPTURE` answers
//!   `EPIPE` until `V4L2_ENC_CMD_START`.
//! - The rate control runs off the timestamps' deltas (`TIME_DELTA_BASED_RC`, on by default), so the
//!   driver is given timestamps at the session's frame rate, one per frame, and each coded frame's
//!   is mapped back to the guest's (a guest may stamp frame numbers).
//!
//! The guest's frames and bitstream buffers are its own memory, but not in the driver's layout
//! (tightly packed NV12) or not dma-bufs (the device lends `CAPTURE` buffers as mappings): each
//! raw frame is copied once into the driver's layout and each coded frame once out -- what
//! MediaCodec's path does too, without its trip through the codec service. Nothing of the guest's
//! reaches the driver, so nothing of the guest's can be kept by it (the decoder's udmabuf trap).

use std::collections::HashMap;
use std::collections::VecDeque;
use std::fs::File;
use std::fs::OpenOptions;
use std::os::fd::AsRawFd;
use std::os::fd::FromRawFd;
use std::os::fd::OwnedFd;
use std::os::fd::RawFd;
use std::os::unix::fs::OpenOptionsExt;
use std::sync::mpsc;
use std::sync::Arc;
use std::sync::Mutex;
use std::thread::JoinHandle;
use std::time::Duration;
use std::time::Instant;

use base::info;
use base::warn;
use virtio_media::devices::video_encoder::EncoderConfig;
use virtio_media::devices::video_encoder::EncoderEvent;
use virtio_media::devices::video_encoder::EncoderSink;
use virtio_media::devices::video_encoder::FrameKind;
use virtio_media::devices::video_encoder::InputBuffer;
use virtio_media::devices::video_encoder::OutputBuffer;
use virtio_media::devices::video_encoder::VideoEncoderBackendSession;
use virtio_media::ioctl::IoctlResult;
use virtio_media::v4l2r::bindings;
use virtio_media::v4l2r::controls::codec::VideoBitrateMode;
use virtio_media::v4l2r::controls::codec::VideoHeaderMode;

use super::direct_enc::DirectEncoderDriver;
use super::vidc::errno;
use super::vidc::ioc;
use super::vidc::vidioc;
use super::vidc::xioctl;
use super::vidc::zeroed;
use super::vidc::BitstreamBuffer as HeapBuffer;
use super::vidc::CAPTURE_MPLANE;
use super::vidc::DMA_HEAP;
use super::vidc::IOC_READ;
use super::vidc::IOC_WRITE;
use super::vidc::MEMORY_DMABUF;
use super::vidc::OUTPUT_MPLANE;
use super::vidc::VIDIOC_DQBUF;
use super::vidc::VIDIOC_G_FMT;
use super::vidc::VIDIOC_QBUF;
use super::vidc::VIDIOC_QUERYCAP;
use super::vidc::VIDIOC_REQBUFS;
use super::vidc::VIDIOC_STREAMOFF;
use super::vidc::VIDIOC_STREAMON;
use super::vidc::VIDIOC_S_FMT;

const DEVICE: &str = "/dev/video33";
/// Raw frames with the driver at once.
const RAW_BUFFERS: u32 = 8;
/// Bitstream buffers with the driver at once.
const BIT_BUFFERS: u32 = 8;
/// Raw frames copied and acknowledged but not yet with the driver; past it a frame stays lent
/// (unacknowledged) until there is room, which is the back-pressure on a fast guest.
const STAGE_MAX: usize = 4;
/// Coded frames waiting for a guest `CAPTURE` buffer; past it no raw frame is fed.
const CODED_MAX: usize = 32;
/// How long the session waits for the worker to answer a command it must finish first.
const ANSWER_TIMEOUT: Duration = Duration::from_secs(3);
/// How long a `STREAMOFF(OUTPUT)` waits for the frames already acknowledged to come out.
const STREAMOFF_DRAIN: Duration = Duration::from_millis(1500);

const NV12: u32 = u32::from_le_bytes(*b"NV12");

const VIDIOC_S_PARM: u32 = vidioc::<bindings::v4l2_streamparm>(IOC_READ | IOC_WRITE, 22);
const VIDIOC_S_EXT_CTRLS: u32 = vidioc::<bindings::v4l2_ext_controls>(IOC_READ | IOC_WRITE, 72);
const VIDIOC_ENCODER_CMD: u32 = vidioc::<bindings::v4l2_encoder_cmd>(IOC_READ | IOC_WRITE, 77);

/// `struct dma_buf_sync` (`linux/dma-buf.h`) and its flags.
#[repr(C)]
struct DmaBufSync {
    flags: u64,
}
const DMA_BUF_IOCTL_SYNC: u32 = ioc(IOC_WRITE, b'b', 0, std::mem::size_of::<DmaBufSync>());
const DMA_BUF_SYNC_READ: u64 = 1;
const DMA_BUF_SYNC_WRITE: u64 = 2;
const DMA_BUF_SYNC_END: u64 = 4;

/// Bracket CPU access to a dma-buf (`DMA_BUF_IOCTL_SYNC`), best effort.
fn cpu_access(fd: RawFd, flags: u64) {
    let mut sync = DmaBufSync { flags };
    let _ = xioctl(fd, DMA_BUF_IOCTL_SYNC, &mut sync);
}

/// The coded formats this backend encodes.
fn vidc_enc_takes(fourcc: u32) -> bool {
    matches!(&fourcc.to_le_bytes(), b"H264" | b"HEVC")
}

fn is_hevc(fourcc: u32) -> bool {
    &fourcc.to_le_bytes() == b"HEVC"
}

/// Where the first NAL unit that is not a parameter set (or a delimiter, or SEI) starts, in an
/// Annex B access unit: the end of the stream headers in front of a key frame.
fn headers_end(bytes: &[u8], hevc: bool) -> Option<usize> {
    let mut i = 0;
    while i + 3 < bytes.len() {
        if bytes[i] == 0 && bytes[i + 1] == 0 && bytes[i + 2] == 1 {
            let start = if i > 0 && bytes[i - 1] == 0 { i - 1 } else { i };
            let header = if hevc {
                matches!((bytes[i + 3] >> 1) & 63, 32 | 33 | 34 | 35 | 39)
            } else {
                matches!(bytes[i + 3] & 31, 6 | 7 | 8 | 9)
            };
            if !header {
                return Some(start);
            }
            i += 3;
        } else {
            i += 1;
        }
    }
    None
}

/// What the session asks of its worker.
enum Command {
    /// A raw frame, already in the visible size, tightly packed.
    Frame(Vec<u8>, bindings::timeval),
    Capture(OutputBuffer),
    Control(u32, i32),
    /// Encode what is staged and give it to the `CAPTURE` buffers lent, bounded; then answer.
    FinishStaged(mpsc::Sender<()>),
    Flush(mpsc::Sender<()>),
    Drain,
    Stop,
}

/// What the worker and the session share: the events for the device, and how many raw frames
/// the worker has been given that are not yet with the driver.
#[derive(Default)]
struct Shared {
    events: Vec<EncoderEvent>,
    staged: usize,
}

struct Coded {
    bytes: Vec<u8>,
    timestamp: bindings::timeval,
    kind: FrameKind,
    is_last: bool,
}

/// The worker's state: the driver, its buffers, and what waits to go into them or out to the
/// guest.
struct Encoder {
    id: u32,
    dev: File,
    wake: OwnedFd,
    shared: Arc<Mutex<Shared>>,
    sink: EncoderSink,
    raw: Vec<HeapBuffer>,
    /// The driver's bitstream buffers; `free` here means with us, not with the driver.
    bits: Vec<HeapBuffer>,
    width: usize,
    height: usize,
    stride: usize,
    scanlines: usize,
    raw_size: usize,
    staged: VecDeque<(Vec<u8>, bindings::timeval)>,
    guest_capture: VecDeque<OutputBuffer>,
    coded: VecDeque<Coded>,
    timestamps: HashMap<i64, bindings::timeval>,
    next_frame: i64,
    step_us: i64,
    hevc: bool,
    separate_headers: bool,
    headers_out: bool,
    drain_wanted: bool,
    drain_sent: bool,
    /// The driver has given its `LAST` buffer; it wants `ENC_CMD_START` before it encodes again.
    drained: bool,
    finish: Option<(mpsc::Sender<()>, Instant)>,
    frames_in: u64,
    frames_out: u64,
    bytes_out: u64,
    keyframes: u64,
    started: Instant,
}

/// Both queues stopped and emptied before the node closes, as the decoder does (`vidc::Stream`):
/// the buffers are ours either way, but msm_vidc frees its session state on `REQBUFS(0)`.
impl Drop for Encoder {
    fn drop(&mut self) {
        let fd = self.fd();
        for type_ in [CAPTURE_MPLANE, OUTPUT_MPLANE] {
            let mut kind = type_ as libc::c_int;
            let _ = xioctl(fd, VIDIOC_STREAMOFF, &mut kind);
            let mut request: bindings::v4l2_requestbuffers = zeroed();
            request.count = 0;
            request.type_ = type_;
            request.memory = MEMORY_DMABUF;
            if let Err(e) = xioctl(fd, VIDIOC_REQBUFS, &mut request) {
                warn!("encoder session {}: msm_vidc REQBUFS(0) of queue {}: errno {}", self.id, type_, e);
            }
        }
        info!(
            "encoder session {}: msm_vidc closed after {:?}: {} frames in, {} coded out ({} key, {} bytes)",
            self.id,
            self.started.elapsed(),
            self.frames_in,
            self.frames_out,
            self.keyframes,
            self.bytes_out
        );
    }
}

fn set_control(fd: RawFd, id: u32, value: i32) -> Result<(), i32> {
    let mut control: bindings::v4l2_ext_control = zeroed();
    control.id = id;
    control.__bindgen_anon_1.value = value;
    let mut controls: bindings::v4l2_ext_controls = zeroed();
    controls.__bindgen_anon_1.which = bindings::V4L2_CTRL_WHICH_CUR_VAL;
    controls.count = 1;
    controls.controls = &mut control;
    xioctl(fd, VIDIOC_S_EXT_CTRLS, &mut controls)
}

impl Encoder {
    fn open(
        id: u32,
        config: &EncoderConfig,
        wake: OwnedFd,
        shared: Arc<Mutex<Shared>>,
        sink: EncoderSink,
    ) -> Result<Self, String> {
        let coded = config.coded_format.to_u32();
        let (width, height) = (config.visible_rect.width, config.visible_rect.height);
        let dev = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_NONBLOCK | libc::O_CLOEXEC)
            .open(DEVICE)
            .map_err(|e| format!("cannot open {}: {}", DEVICE, e))?;
        let fd = dev.as_raw_fd();

        let mut capture: bindings::v4l2_format = zeroed();
        capture.type_ = CAPTURE_MPLANE;
        // SAFETY (this and the other `pix_mp` accesses): multi-planar formats, the union member
        // for these types.
        unsafe {
            capture.fmt.pix_mp.pixelformat = coded;
            capture.fmt.pix_mp.width = width;
            capture.fmt.pix_mp.height = height;
            capture.fmt.pix_mp.num_planes = 1;
        }
        xioctl(fd, VIDIOC_S_FMT, &mut capture).map_err(|e| format!("S_FMT(CAPTURE): errno {}", e))?;

        let mut raw: bindings::v4l2_format = zeroed();
        raw.type_ = OUTPUT_MPLANE;
        let c = config.colorimetry;
        unsafe {
            raw.fmt.pix_mp.pixelformat = NV12;
            raw.fmt.pix_mp.width = width;
            raw.fmt.pix_mp.height = height;
            raw.fmt.pix_mp.num_planes = 1;
            raw.fmt.pix_mp.colorspace = c.colorspace;
            raw.fmt.pix_mp.__bindgen_anon_1.ycbcr_enc = c.ycbcr_enc as u8;
            raw.fmt.pix_mp.quantization = c.quantization as u8;
            raw.fmt.pix_mp.xfer_func = c.xfer_func as u8;
        }
        xioctl(fd, VIDIOC_S_FMT, &mut raw).map_err(|e| format!("S_FMT(OUTPUT): errno {}", e))?;
        xioctl(fd, VIDIOC_G_FMT, &mut raw).map_err(|e| format!("G_FMT(OUTPUT): errno {}", e))?;
        // Copied out: the struct is packed.
        let pix = unsafe { raw.fmt.pix_mp };
        let (raw_fourcc, stride, raw_size) =
            (pix.pixelformat, pix.plane_fmt[0].bytesperline as usize, pix.plane_fmt[0].sizeimage as usize);
        if raw_fourcc != NV12 || stride < width as usize {
            return Err(format!(
                "OUTPUT is {:08x} with rows of {} for {}x{}",
                raw_fourcc, stride, width, height
            ));
        }
        let scanlines = raw_size * 2 / 3 / stride;
        if scanlines < height as usize {
            return Err(format!(
                "OUTPUT sizeimage {} holds {} rows of {}, the frame has {}",
                raw_size, scanlines, stride, height
            ));
        }
        xioctl(fd, VIDIOC_G_FMT, &mut capture).map_err(|e| format!("G_FMT(CAPTURE): errno {}", e))?;
        let bit_size = (unsafe { capture.fmt.pix_mp.plane_fmt[0].sizeimage } as usize).max(64 << 10);

        let rate = config.frame_rate;
        let (num, den) = (rate.num.max(1), rate.den.max(1));
        let mut parm: bindings::v4l2_streamparm = zeroed();
        parm.type_ = OUTPUT_MPLANE;
        // SAFETY: the output member, for an OUTPUT type.
        unsafe {
            parm.parm.output.timeperframe.numerator = den;
            parm.parm.output.timeperframe.denominator = num;
        }
        if let Err(e) = xioctl(fd, VIDIOC_S_PARM, &mut parm) {
            warn!("encoder session {}: msm_vidc S_PARM {}/{} fps: errno {}", id, num, den, e);
        }
        let step_us = (1_000_000i64 * den as i64 / num as i64).max(1);

        let mode = match config.bitrate_mode {
            VideoBitrateMode::VariableBitrate => bindings::v4l2_mpeg_video_bitrate_mode_V4L2_MPEG_VIDEO_BITRATE_MODE_VBR,
            VideoBitrateMode::ConstantBitrate => bindings::v4l2_mpeg_video_bitrate_mode_V4L2_MPEG_VIDEO_BITRATE_MODE_CBR,
            VideoBitrateMode::ConstantQuality => return Err("constant quality".into()),
        };
        // The ones the stream is made of: refused, the session is MediaCodec's.
        for (cid, value, what) in [
            (bindings::V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE, 1, "rate control"),
            (bindings::V4L2_CID_MPEG_VIDEO_BITRATE_MODE, mode as i32, "bitrate mode"),
            (
                bindings::V4L2_CID_MPEG_VIDEO_BITRATE,
                config.bitrate.clamp(1, i32::MAX as u32) as i32,
                "bitrate",
            ),
            (bindings::V4L2_CID_MPEG_VIDEO_B_FRAMES, 0, "no B frames"),
            (
                bindings::V4L2_CID_MPEG_VIDEO_HEADER_MODE,
                VideoHeaderMode::JoinedWith1stFrame as i32,
                "header mode",
            ),
        ] {
            set_control(fd, cid, value).map_err(|e| format!("{} {}: errno {}", what, value, e))?;
        }
        // The ones that shape it: refused, the driver's default stands.
        let (profile_cid, level_cid, min_qp_cid, max_qp_cid) = if is_hevc(coded) {
            (
                bindings::V4L2_CID_MPEG_VIDEO_HEVC_PROFILE,
                bindings::V4L2_CID_MPEG_VIDEO_HEVC_LEVEL,
                bindings::V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP,
                bindings::V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP,
            )
        } else {
            (
                bindings::V4L2_CID_MPEG_VIDEO_H264_PROFILE,
                bindings::V4L2_CID_MPEG_VIDEO_H264_LEVEL,
                bindings::V4L2_CID_MPEG_VIDEO_H264_MIN_QP,
                bindings::V4L2_CID_MPEG_VIDEO_H264_MAX_QP,
            )
        };
        let mut shaping = vec![(
            bindings::V4L2_CID_MPEG_VIDEO_PREPEND_SPSPPS_TO_IDR,
            config.prepend_sps_pps_to_idr as i32,
            "headers before every key frame",
        )];
        if config.gop_size > 0 {
            shaping.push((
                bindings::V4L2_CID_MPEG_VIDEO_GOP_SIZE,
                config.gop_size.min(i32::MAX as u32) as i32,
                "GOP size",
            ));
        }
        if let Some(profile) = config.profile {
            shaping.push((profile_cid, profile, "profile"));
        }
        if let Some(level) = config.level {
            shaping.push((level_cid, level, "level"));
        }
        if let Some((min, max)) = config.qp {
            shaping.push((min_qp_cid, min, "minimum QP"));
            shaping.push((max_qp_cid, max, "maximum QP"));
        }
        for (cid, value, what) in shaping {
            if let Err(e) = set_control(fd, cid, value) {
                warn!("encoder session {}: msm_vidc {} {}: errno {}; its default", id, what, value, e);
            }
        }

        let heap = File::options()
            .read(true)
            .open(DMA_HEAP)
            .map_err(|e| format!("cannot open {}: {}", DMA_HEAP, e))?;
        let page = |n: usize| (n + 4095) & !4095;
        let mut buffers = |type_: u32, count: u32, len: usize, what: &str| -> Result<Vec<HeapBuffer>, String> {
            let mut request: bindings::v4l2_requestbuffers = zeroed();
            request.count = count;
            request.type_ = type_;
            request.memory = MEMORY_DMABUF;
            xioctl(fd, VIDIOC_REQBUFS, &mut request)
                .map_err(|e| format!("REQBUFS({}): errno {}", what, e))?;
            (0..request.count.min(count))
                .map(|_| HeapBuffer::new(&heap, page(len)))
                .collect()
        };
        let raw_buffers = buffers(OUTPUT_MPLANE, RAW_BUFFERS, raw_size, "OUTPUT")?;
        let mut bits = buffers(CAPTURE_MPLANE, BIT_BUFFERS, bit_size, "CAPTURE")?;
        if raw_buffers.is_empty() || bits.is_empty() {
            return Err("the driver gave no buffers".into());
        }
        for (index, buffer) in bits.iter_mut().enumerate() {
            queue_bitstream(fd, index as u32, buffer)?;
        }
        for (type_, what) in [(CAPTURE_MPLANE, "CAPTURE"), (OUTPUT_MPLANE, "OUTPUT")] {
            let mut kind = type_ as libc::c_int;
            xioctl(fd, VIDIOC_STREAMON, &mut kind)
                .map_err(|e| format!("STREAMON({}): errno {}", what, e))?;
        }
        info!(
            "encoder session {}: msm_vidc {} {}x{} {}/{} fps, {} bit/s {:?}, GOP {}, profile {:?} \
             level {:?}; NV12 rows of {} chroma at row {}, {} raw buffers of {} and {} bitstream \
             buffers of {}",
            id,
            String::from_utf8_lossy(&coded.to_le_bytes()),
            width,
            height,
            num,
            den,
            config.bitrate,
            config.bitrate_mode,
            config.gop_size,
            config.profile,
            config.level,
            stride,
            scanlines,
            raw_buffers.len(),
            raw_size,
            bits.len(),
            bit_size
        );
        Ok(Self {
            id,
            dev,
            wake,
            shared,
            sink,
            raw: raw_buffers,
            bits,
            width: width as usize,
            height: height as usize,
            stride,
            scanlines,
            raw_size,
            staged: VecDeque::new(),
            guest_capture: VecDeque::new(),
            coded: VecDeque::new(),
            timestamps: HashMap::new(),
            next_frame: 0,
            step_us,
            hevc: is_hevc(coded),
            separate_headers: config.header_mode == VideoHeaderMode::Separate,
            headers_out: false,
            drain_wanted: false,
            drain_sent: false,
            drained: false,
            finish: None,
            frames_in: 0,
            frames_out: 0,
            bytes_out: 0,
            keyframes: 0,
            started: Instant::now(),
        })
    }

    fn fd(&self) -> RawFd {
        self.dev.as_raw_fd()
    }

    fn emit(&self, event: EncoderEvent) {
        self.shared.lock().unwrap().events.push(event);
        self.sink.signal();
    }

    /// Run until `Stop`, the commands' sender going away, or the driver failing.
    fn run(mut self, commands: mpsc::Receiver<Command>) {
        loop {
            let mut fds = [
                libc::pollfd {
                    fd: self.fd(),
                    events: libc::POLLIN | libc::POLLOUT | libc::POLLPRI,
                    revents: 0,
                },
                libc::pollfd {
                    fd: self.wake.as_raw_fd(),
                    events: libc::POLLIN,
                    revents: 0,
                },
            ];
            // SAFETY: two live pollfds.
            unsafe { libc::poll(fds.as_mut_ptr(), 2, 20) };
            let mut count = 0u64;
            // SAFETY: draining 8 bytes from our non-blocking eventfd.
            unsafe { libc::read(self.wake.as_raw_fd(), &mut count as *mut u64 as *mut _, 8) };
            loop {
                match commands.try_recv() {
                    Ok(Command::Stop) | Err(mpsc::TryRecvError::Disconnected) => return,
                    Ok(command) => {
                        if let Err(e) = self.command(command) {
                            self.emit(EncoderEvent::Error(format!("msm_vidc: {}", e)));
                            return;
                        }
                    }
                    Err(mpsc::TryRecvError::Empty) => break,
                }
            }
            if let Err(e) = self.service() {
                self.emit(EncoderEvent::Error(format!("msm_vidc: {}", e)));
                return;
            }
            self.answer_finish();
        }
    }

    fn command(&mut self, command: Command) -> Result<(), String> {
        match command {
            Command::Frame(bytes, timestamp) => self.staged.push_back((bytes, timestamp)),
            Command::Capture(buffer) => self.guest_capture.push_back(buffer),
            Command::Control(cid, value) => {
                if let Err(e) = set_control(self.fd(), cid, value) {
                    warn!("encoder session {}: msm_vidc control {:#x} = {}: errno {}", self.id, cid, value, e);
                }
            }
            Command::FinishStaged(done) => self.finish = Some((done, Instant::now() + STREAMOFF_DRAIN)),
            Command::Flush(done) => {
                let fd = self.fd();
                let mut kind = OUTPUT_MPLANE as libc::c_int;
                xioctl(fd, VIDIOC_STREAMOFF, &mut kind)
                    .map_err(|e| format!("STREAMOFF(OUTPUT): errno {}", e))?;
                for buffer in &mut self.raw {
                    buffer.free = true;
                }
                self.staged.clear();
                self.shared.lock().unwrap().staged = 0;
                self.drain_wanted = false;
                self.drain_sent = false;
                xioctl(fd, VIDIOC_STREAMON, &mut kind)
                    .map_err(|e| format!("STREAMON(OUTPUT): errno {}", e))?;
                if self.drained {
                    let mut cmd: bindings::v4l2_encoder_cmd = zeroed();
                    cmd.cmd = bindings::V4L2_ENC_CMD_START;
                    xioctl(fd, VIDIOC_ENCODER_CMD, &mut cmd)
                        .map_err(|e| format!("ENC_CMD_START: errno {}", e))?;
                    self.drained = false;
                    // The bitstream buffers kept back since LAST.
                    for (index, slot) in self.bits.iter_mut().enumerate() {
                        if slot.free {
                            queue_bitstream(fd, index as u32, slot)?;
                        }
                    }
                }
                let _ = done.send(());
            }
            Command::Drain => self.drain_wanted = true,
            Command::Stop => {}
        }
        Ok(())
    }

    /// One pass over everything the driver may have for us, and everything we may give it.
    fn service(&mut self) -> Result<(), String> {
        self.take_raw_back()?;
        self.feed()?;
        if self.drain_wanted && !self.drain_sent && self.staged.is_empty() {
            let mut cmd: bindings::v4l2_encoder_cmd = zeroed();
            cmd.cmd = bindings::V4L2_ENC_CMD_STOP;
            xioctl(self.fd(), VIDIOC_ENCODER_CMD, &mut cmd)
                .map_err(|e| format!("ENC_CMD_STOP: errno {}", e))?;
            self.drain_sent = true;
        }
        self.take_coded()?;
        self.deliver();
        Ok(())
    }

    fn take_raw_back(&mut self) -> Result<(), String> {
        loop {
            let mut plane: bindings::v4l2_plane = zeroed();
            let mut buffer: bindings::v4l2_buffer = zeroed();
            buffer.type_ = OUTPUT_MPLANE;
            buffer.memory = MEMORY_DMABUF;
            buffer.m.planes = &mut plane;
            buffer.length = 1;
            match xioctl(self.fd(), VIDIOC_DQBUF, &mut buffer) {
                Ok(()) => {
                    if let Some(slot) = self.raw.get_mut(buffer.index as usize) {
                        slot.free = true;
                    }
                }
                Err(libc::EAGAIN) | Err(libc::EPIPE) => return Ok(()),
                Err(e) => return Err(format!("DQBUF(OUTPUT): errno {}", e)),
            }
        }
    }

    /// Every staged frame a free raw buffer can take, while the coded frames have somewhere to go.
    fn feed(&mut self) -> Result<(), String> {
        let mut fed = false;
        while !self.staged.is_empty() && self.coded.len() < CODED_MAX {
            let Some(index) = self.raw.iter().position(|b| b.free) else {
                break;
            };
            let (frame, timestamp) = self.staged.pop_front().unwrap();
            let (w, h, stride) = (self.width, self.height, self.stride);
            let slot = &mut self.raw[index];
            cpu_access(slot.fd.as_raw_fd(), DMA_BUF_SYNC_WRITE);
            // SAFETY: the mapping is `slot.len` >= `raw_size` bytes, which holds `scanlines` rows
            // of `stride` luma and half as many of chroma; the source is `w * h * 3 / 2` bytes.
            unsafe {
                for y in 0..h {
                    std::ptr::copy_nonoverlapping(frame.as_ptr().add(y * w), slot.map.add(y * stride), w);
                }
                let chroma = slot.map.add(self.scanlines * stride);
                for y in 0..h / 2 {
                    std::ptr::copy_nonoverlapping(frame.as_ptr().add(w * h + y * w), chroma.add(y * stride), w);
                }
            }
            cpu_access(slot.fd.as_raw_fd(), DMA_BUF_SYNC_WRITE | DMA_BUF_SYNC_END);
            let us = self.next_frame * self.step_us;
            self.next_frame += 1;
            self.timestamps.insert(us, timestamp);
            if self.timestamps.len() > 1024 {
                let floor = us - 512 * self.step_us;
                self.timestamps.retain(|&k, _| k >= floor);
            }
            let mut plane: bindings::v4l2_plane = zeroed();
            plane.bytesused = self.raw_size as u32;
            plane.length = slot.len as u32;
            plane.m.fd = slot.fd.as_raw_fd();
            let mut buffer: bindings::v4l2_buffer = zeroed();
            buffer.index = index as u32;
            buffer.type_ = OUTPUT_MPLANE;
            buffer.memory = MEMORY_DMABUF;
            buffer.m.planes = &mut plane;
            buffer.length = 1;
            buffer.timestamp.tv_sec = (us / 1_000_000) as _;
            buffer.timestamp.tv_usec = (us % 1_000_000) as _;
            xioctl(self.dev.as_raw_fd(), VIDIOC_QBUF, &mut buffer)
                .map_err(|e| format!("QBUF(OUTPUT) frame {}: errno {}", self.frames_in, e))?;
            slot.free = false;
            self.frames_in += 1;
            fed = true;
            let mut shared = self.shared.lock().unwrap();
            shared.staged = shared.staged.saturating_sub(1);
        }
        if fed {
            // Room in the stage: the session may take the frames it holds lent.
            self.sink.signal();
        }
        Ok(())
    }

    fn take_coded(&mut self) -> Result<(), String> {
        loop {
            let mut plane: bindings::v4l2_plane = zeroed();
            let mut buffer: bindings::v4l2_buffer = zeroed();
            buffer.type_ = CAPTURE_MPLANE;
            buffer.memory = MEMORY_DMABUF;
            buffer.m.planes = &mut plane;
            buffer.length = 1;
            match xioctl(self.fd(), VIDIOC_DQBUF, &mut buffer) {
                Ok(()) => {}
                Err(libc::EAGAIN) => return Ok(()),
                // After the LAST buffer of a drain, until ENC_CMD_START.
                Err(libc::EPIPE) => return Ok(()),
                Err(e) => return Err(format!("DQBUF(CAPTURE): errno {}", e)),
            }
            let index = buffer.index as usize;
            let Some(slot) = self.bits.get(index) else {
                return Err(format!("DQBUF(CAPTURE) of buffer {} past the {}", index, self.bits.len()));
            };
            let flags = buffer.flags;
            let is_last = flags & bindings::V4L2_BUF_FLAG_LAST != 0;
            let error = flags & bindings::V4L2_BUF_FLAG_ERROR != 0;
            let start = (plane.data_offset as usize).min(slot.len);
            let end = (plane.bytesused as usize).min(slot.len).max(start);
            if error {
                warn!("encoder session {}: msm_vidc coded buffer {} came back with ERROR", self.id, index);
            }
            if !error && end > start {
                cpu_access(slot.fd.as_raw_fd(), DMA_BUF_SYNC_READ);
                // SAFETY: `start..end` lies inside the mapping.
                let bytes =
                    unsafe { std::slice::from_raw_parts(slot.map.add(start), end - start) }.to_vec();
                cpu_access(slot.fd.as_raw_fd(), DMA_BUF_SYNC_READ | DMA_BUF_SYNC_END);
                let us = buffer.timestamp.tv_sec as i64 * 1_000_000 + buffer.timestamp.tv_usec as i64;
                let timestamp = self.timestamps.remove(&us).unwrap_or(buffer.timestamp);
                let kind = if flags & bindings::V4L2_BUF_FLAG_KEYFRAME != 0 {
                    FrameKind::Key
                } else if flags & bindings::V4L2_BUF_FLAG_BFRAME != 0 {
                    FrameKind::Bidirectional
                } else {
                    FrameKind::Inter
                };
                self.frames_out += 1;
                self.bytes_out += bytes.len() as u64;
                if kind == FrameKind::Key {
                    self.keyframes += 1;
                }
                self.push_coded(bytes, timestamp, kind, is_last);
            } else if is_last {
                self.coded.push_back(Coded {
                    bytes: Vec::new(),
                    timestamp: buffer.timestamp,
                    kind: FrameKind::Inter,
                    is_last: true,
                });
            }
            if is_last {
                self.drained = true;
                self.drain_wanted = false;
                self.drain_sent = false;
            }
            // Straight back to the driver (the empty buffers it returns at STREAMON too); after
            // LAST, not until ENC_CMD_START, which flush sends -- it queues them all again then.
            let fd = self.fd();
            let slot = &mut self.bits[index];
            if !is_last {
                queue_bitstream(fd, index as u32, slot)?;
            } else {
                slot.free = true;
            }
        }
    }

    /// A coded frame for the guest; the stream headers cut off into a buffer of their own first if
    /// the guest asked for them separate.
    fn push_coded(&mut self, mut bytes: Vec<u8>, timestamp: bindings::timeval, kind: FrameKind, is_last: bool) {
        if self.separate_headers && !self.headers_out && kind == FrameKind::Key {
            self.headers_out = true;
            if let Some(end) = headers_end(&bytes, self.hevc).filter(|&end| end > 0) {
                let rest = bytes.split_off(end);
                self.coded.push_back(Coded {
                    bytes,
                    timestamp,
                    kind: FrameKind::Headers,
                    is_last: false,
                });
                bytes = rest;
            }
        }
        self.coded.push_back(Coded {
            bytes,
            timestamp,
            kind,
            is_last,
        });
    }

    /// Coded frames into the guest's `CAPTURE` buffers, as many as are lent.
    fn deliver(&mut self) {
        while !self.coded.is_empty() && !self.guest_capture.is_empty() {
            let coded = self.coded.pop_front().unwrap();
            let buffer = self.guest_capture.pop_front().unwrap();
            let len = coded.bytes.len().min(buffer.len);
            if len < coded.bytes.len() {
                warn!(
                    "encoder session {}: a coded frame of {} bytes cut to the {} of CAPTURE buffer {}",
                    self.id,
                    coded.bytes.len(),
                    buffer.len,
                    buffer.index
                );
            }
            // SAFETY: the device lends `buffer.len` writable bytes at `ptr` until `FrameEncoded`.
            unsafe { std::ptr::copy_nonoverlapping(coded.bytes.as_ptr(), buffer.ptr.as_ptr(), len) };
            self.emit(EncoderEvent::FrameEncoded {
                index: buffer.index,
                bytesused: len as u32,
                timestamp: coded.timestamp,
                kind: coded.kind,
                is_last: coded.is_last,
            });
        }
    }

    /// `FinishStaged`'s answer, once every staged frame is with the driver and every frame the
    /// driver took came out, or at its deadline.
    fn answer_finish(&mut self) {
        let Some((_, deadline)) = &self.finish else {
            return;
        };
        let done = self.staged.is_empty() && self.raw.iter().all(|b| b.free) && self.coded.is_empty();
        if done || Instant::now() >= *deadline {
            if !done {
                warn!(
                    "encoder session {}: STREAMOFF(OUTPUT): {} staged and {} coded frame(s) left at \
                     the deadline",
                    self.id,
                    self.staged.len(),
                    self.coded.len()
                );
            }
            let (done, _) = self.finish.take().unwrap();
            let _ = done.send(());
        }
    }
}

fn queue_bitstream(fd: RawFd, index: u32, slot: &mut HeapBuffer) -> Result<(), String> {
    let mut plane: bindings::v4l2_plane = zeroed();
    plane.length = slot.len as u32;
    plane.m.fd = slot.fd.as_raw_fd();
    let mut buffer: bindings::v4l2_buffer = zeroed();
    buffer.index = index;
    buffer.type_ = CAPTURE_MPLANE;
    buffer.memory = MEMORY_DMABUF;
    buffer.m.planes = &mut plane;
    buffer.length = 1;
    xioctl(fd, VIDIOC_QBUF, &mut buffer).map_err(|e| format!("QBUF(CAPTURE {}): errno {}", index, e))?;
    slot.free = false;
    Ok(())
}

/// A session on msm_vidc's encoder: the worker thread that owns the driver, the line to it, and
/// the raw frames held lent while the stage is full.
struct VidcEncSession {
    id: u32,
    commands: mpsc::Sender<Command>,
    wake: OwnedFd,
    shared: Arc<Mutex<Shared>>,
    sink: EncoderSink,
    thread: Option<JoinHandle<()>>,
    held: VecDeque<InputBuffer>,
    /// A drain asked for while frames were still held: sent once they are staged.
    drain_deferred: bool,
    coded_size: (usize, usize),
    visible: (usize, usize, usize, usize),
}

impl VidcEncSession {
    fn start(id: u32, sink: EncoderSink, config: &EncoderConfig) -> Result<Self, String> {
        let (cw, ch) = (config.coded_size.0 as usize, config.coded_size.1 as usize);
        let r = config.visible_rect;
        let (left, top) = (r.left.max(0) as usize & !1, r.top.max(0) as usize & !1);
        let (w, h) = (r.width as usize, r.height as usize);
        if w == 0 || h == 0 || w & 1 != 0 || h & 1 != 0 || left + w > cw || top + h > ch {
            return Err(format!("visible {}x{} at {},{} of {}x{}", w, h, left, top, cw, ch));
        }
        // SAFETY: eventfd takes no pointers; checked before it is owned.
        let raw = unsafe { libc::eventfd(0, libc::EFD_CLOEXEC | libc::EFD_NONBLOCK) };
        if raw < 0 {
            return Err(format!("eventfd: errno {}", errno()));
        }
        // SAFETY: just created, owned by no one else.
        let wake = unsafe { OwnedFd::from_raw_fd(raw) };
        let worker_wake = wake.try_clone().map_err(|e| format!("eventfd dup: {}", e))?;
        let shared = Arc::new(Mutex::new(Shared::default()));
        let worker_shared = Arc::clone(&shared);
        let worker_sink = sink.clone();
        let worker_config = config.clone();
        let (commands, receiver) = mpsc::channel();
        let (opened, open_result) = mpsc::channel();
        let thread = std::thread::Builder::new()
            .name(format!("vidc_enc_{}", id))
            .spawn(move || {
                match Encoder::open(id, &worker_config, worker_wake, worker_shared, worker_sink) {
                    Ok(encoder) => {
                        let _ = opened.send(Ok(()));
                        encoder.run(receiver);
                    }
                    Err(e) => {
                        let _ = opened.send(Err(e));
                    }
                }
            })
            .map_err(|e| format!("cannot start the vidc thread: {}", e))?;
        match open_result.recv_timeout(ANSWER_TIMEOUT) {
            Ok(Ok(())) => Ok(Self {
                id,
                commands,
                wake,
                shared,
                sink,
                thread: Some(thread),
                held: VecDeque::new(),
                drain_deferred: false,
                coded_size: (cw, ch),
                visible: (left, top, w, h),
            }),
            Ok(Err(e)) => {
                let _ = thread.join();
                Err(e)
            }
            Err(_) => Err("msm_vidc did not open in time".into()),
        }
    }

    fn send(&self, command: Command) -> IoctlResult<()> {
        self.commands.send(command).map_err(|_| libc::EIO)?;
        let one: u64 = 1;
        // SAFETY: writing 8 bytes to our eventfd.
        unsafe { libc::write(self.wake.as_raw_fd(), &one as *const u64 as *const _, 8) };
        Ok(())
    }

    fn send_and_wait(
        &self,
        make: impl FnOnce(mpsc::Sender<()>) -> Command,
        timeout: Duration,
    ) -> IoctlResult<()> {
        let (done, wait) = mpsc::channel();
        self.send(make(done))?;
        wait.recv_timeout(timeout).map_err(|_| libc::EIO)
    }

    /// Copy a lent frame's visible part out, tightly packed, hand it to the worker and give the
    /// guest its buffer back.
    fn stage(&mut self, buffer: InputBuffer) -> IoctlResult<()> {
        let (cw, ch) = self.coded_size;
        let (left, top, w, h) = self.visible;
        if buffer.len < cw * ch * 3 / 2 {
            warn!(
                "encoder session {}: raw frame {} of {} bytes, {}x{} NV12 is {}",
                self.id,
                buffer.index,
                buffer.len,
                cw,
                ch,
                cw * ch * 3 / 2
            );
            return Err(libc::EINVAL);
        }
        // SAFETY: the device lends `len` readable bytes at `ptr` until `InputBufferDone`, which
        // is queued only after this copy.
        let src = unsafe { std::slice::from_raw_parts(buffer.ptr.as_ptr() as *const u8, buffer.len) };
        let mut frame = Vec::with_capacity(w * h * 3 / 2);
        for y in 0..h {
            let at = (top + y) * cw + left;
            frame.extend_from_slice(&src[at..at + w]);
        }
        for y in 0..h / 2 {
            let at = cw * ch + (top / 2 + y) * cw + left;
            frame.extend_from_slice(&src[at..at + w]);
        }
        {
            let mut shared = self.shared.lock().unwrap();
            shared.staged += 1;
            shared.events.push(EncoderEvent::InputBufferDone(buffer.index));
        }
        self.send(Command::Frame(frame, buffer.timestamp))?;
        self.sink.signal();
        Ok(())
    }

    /// The held frames the stage has room for; the deferred drain once none is left.
    fn unhold(&mut self) -> IoctlResult<()> {
        while !self.held.is_empty() && self.shared.lock().unwrap().staged < STAGE_MAX {
            let buffer = self.held.pop_front().unwrap();
            self.stage(buffer)?;
        }
        if self.drain_deferred && self.held.is_empty() {
            self.drain_deferred = false;
            self.send(Command::Drain)?;
        }
        Ok(())
    }

    fn stop(&mut self) {
        self.held.clear();
        let _ = self.send(Command::Stop);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

impl Drop for VidcEncSession {
    fn drop(&mut self) {
        self.stop();
    }
}

impl VideoEncoderBackendSession for VidcEncSession {
    fn start(&mut self, _config: &EncoderConfig) -> IoctlResult<()> {
        // A session is started once, by `MsmVidcEncoder::start`; a restart is a new session.
        Err(libc::EBUSY)
    }

    fn encode(&mut self, buffer: InputBuffer) -> IoctlResult<()> {
        if self.held.is_empty() && self.shared.lock().unwrap().staged < STAGE_MAX {
            self.stage(buffer)
        } else {
            self.held.push_back(buffer);
            Ok(())
        }
    }

    fn use_as_capture(&mut self, buffer: OutputBuffer) -> IoctlResult<()> {
        self.send(Command::Capture(buffer))
    }

    fn set_bitrate(&mut self, bitrate: u32) -> IoctlResult<()> {
        info!("encoder session {}: msm_vidc bitrate -> {} bit/s", self.id, bitrate);
        self.send(Command::Control(
            bindings::V4L2_CID_MPEG_VIDEO_BITRATE,
            bitrate.clamp(1, i32::MAX as u32) as i32,
        ))
    }

    fn force_keyframe(&mut self) -> IoctlResult<()> {
        self.send(Command::Control(bindings::V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME, 1))
    }

    fn drain_for_streamoff(&mut self) {
        // The held frames were never acknowledged: the device gives them back as they are. The
        // staged ones were, and are owed their encoding.
        self.held.clear();
        self.drain_deferred = false;
        let _ = self.send_and_wait(Command::FinishStaged, STREAMOFF_DRAIN + Duration::from_millis(500));
    }

    fn flush(&mut self) -> IoctlResult<()> {
        self.held.clear();
        self.drain_deferred = false;
        let result = self.send_and_wait(Command::Flush, ANSWER_TIMEOUT);
        // Nothing more about the frames given back: an ack the device has not read yet would land
        // on a buffer the guest may queue again.
        self.shared
            .lock()
            .unwrap()
            .events
            .retain(|e| !matches!(e, EncoderEvent::InputBufferDone(_)));
        result
    }

    fn drain(&mut self) -> IoctlResult<()> {
        if self.held.is_empty() {
            self.send(Command::Drain)
        } else {
            self.drain_deferred = true;
            Ok(())
        }
    }

    fn stop(&mut self) {
        VidcEncSession::stop(self)
    }

    fn take_events(&mut self) -> Vec<EncoderEvent> {
        if let Err(e) = self.unhold() {
            self.shared
                .lock()
                .unwrap()
                .events
                .push(EncoderEvent::Error(format!("msm_vidc: a held frame: errno {}", e)));
        }
        std::mem::take(&mut self.shared.lock().unwrap().events)
    }
}

/// Qualcomm's msm_vidc encoder as a direct driver: `backend=msm-vidc`, and what `backend=auto`
/// finds on a Snapdragon phone.
pub struct MsmVidcEncoder;

impl DirectEncoderDriver for MsmVidcEncoder {
    fn name(&self) -> &'static str {
        "msm-vidc"
    }

    fn present(&self) -> bool {
        let Ok(dev) = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_NONBLOCK | libc::O_CLOEXEC)
            .open(DEVICE)
        else {
            info!("encoder: cannot open {}", DEVICE);
            return false;
        };
        let mut capability: bindings::v4l2_capability = zeroed();
        if xioctl(dev.as_raw_fd(), VIDIOC_QUERYCAP, &mut capability).is_err() {
            return false;
        }
        let text = |bytes: &[u8]| {
            let end = bytes.iter().position(|b| *b == 0).unwrap_or(bytes.len());
            String::from_utf8_lossy(&bytes[..end]).into_owned()
        };
        let (driver, card) = (text(&capability.driver), text(&capability.card));
        // The node is a number: msm_vidc's encoder only if the driver says so (its card is
        // "msm_vidc_encoder", the decoder's "msm_vidc_decoder").
        let ours = driver.contains("msm_vidc") && card.contains("encoder");
        info!(
            "encoder: {} is driver \"{}\", card \"{}\": {}",
            DEVICE,
            driver,
            card,
            if ours { "msm_vidc's encoder" } else { "not msm_vidc's encoder" }
        );
        ours
    }

    fn takes(&self, config: &EncoderConfig) -> Result<(), String> {
        let fourcc = config.coded_format.to_u32();
        if !vidc_enc_takes(fourcc) {
            return Err(format!("{} is not a format it encodes", String::from_utf8_lossy(&fourcc.to_le_bytes())));
        }
        if config.bitrate_mode == VideoBitrateMode::ConstantQuality {
            return Err("it has no constant quality mode".into());
        }
        Ok(())
    }

    fn start(
        &self,
        id: u32,
        sink: EncoderSink,
        config: &EncoderConfig,
    ) -> Result<Box<dyn VideoEncoderBackendSession + Send>, String> {
        VidcEncSession::start(id, sink, config)
            .map(|session| Box::new(session) as Box<dyn VideoEncoderBackendSession + Send>)
    }
}
