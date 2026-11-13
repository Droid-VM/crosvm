// Copyright 2026 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! The decoder straight through Qualcomm's msm_vidc V4L2 driver (`/dev/video32`), for
//! `--virtio-media kind=decoder,vidc=true` (macOS `docs/k27-vt-zero-copy-plan.md`, Z2): the
//! guest's `CAPTURE` buffers go to the driver as udmabufs over their own pages
//! ([`OutputBuffer::dmabuf`]), so the hardware writes every frame into guest memory and nothing
//! copies it -- against MediaCodec's path, which has the component convert into a linear block of
//! its own and this process copy that into the guest (4096x2160 Main10 measured 40 fps that way,
//! 178-209 fps this way, `research/vidc-probe`).
//!
//! What the driver wants, as measured on the phone (P0):
//! - `OUTPUT` has no `MMAP` (`mmap` fails `ENODEV`): the bitstream goes in dma-heap buffers of
//!   ours, each at least the `sizeimage` `G_FMT(OUTPUT)` reports, or `QBUF` fails `EINVAL`.
//! - The order Rungic's decoder keeps (kevinzhow/Rungic#12): `OUTPUT` streaming, an access
//!   unit, `SOURCE_CHANGE`, then the `CAPTURE` queue.
//! - After the `SOURCE_CHANGE` of a 10-bit stream the only `CAPTURE` formats are UBWC (`Q10C`)
//!   and linear `P010`; an 8-bit one offers linear `NV12`. Linear is what a guest can read, so
//!   that is what is chosen, and announced with [`DecoderEvent::CaptureLayout`] (one plane:
//!   `bytesperline` rows of luma, `height` of them, then the chroma).
//! - `V4L2_DEC_CMD_STOP` gives back the frames the decoder still holds.
//!
//! The guest's timestamps are its own (macOS's VideoToolbox host sends frame numbers), and the
//! driver sets its clocks from the rate it reads in them; so the driver is given timestamps of a
//! 240 fps stream, one per access unit, and each frame's is mapped back to the guest's.
//!
//! [`HwDecoderBackend`] wraps the MediaCodec backend: a session whose coded format msm_vidc
//! takes and whose driver opens decodes here, every other one goes to MediaCodec as before.

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
use virtio_media::v4l2r::Rect;

use super::android::MediaCodecDecoderBackend;
use super::android::MediaCodecDecoderSession;

const DEVICE: &str = "/dev/video32";
const DMA_HEAP: &str = "/dev/dma_heap/system";
/// Bitstream buffers in flight: an access unit each.
const OUTPUT_BUFFERS: u32 = 8;
/// `CAPTURE` slots asked of the driver; the guest's buffer indices map onto them one to one.
const CAPTURE_SLOTS: u32 = 32;
/// The timestamp step the driver is fed: a 240 fps stream, so it clocks for the fastest.
const DRIVER_STEP_US: i64 = 4166;
/// How long the session waits for the worker to answer a command it must finish first.
const ANSWER_TIMEOUT: Duration = Duration::from_secs(3);

const fn fourcc(code: &[u8; 4]) -> u32 {
    u32::from_le_bytes(*code)
}
const NV12: u32 = fourcc(b"NV12");
const P010: u32 = fourcc(b"P010");

// The ioctls used, as the kernel's `_IOC` makes them (`'V'` and the type's size).
const IOC_WRITE: u32 = 1;
const IOC_READ: u32 = 2;
const fn ioc(dir: u32, kind: u8, nr: u32, size: usize) -> u32 {
    (dir << 30) | ((size as u32) << 16) | ((kind as u32) << 8) | nr
}
const fn vidioc<T>(dir: u32, nr: u32) -> u32 {
    ioc(dir, b'V', nr, std::mem::size_of::<T>())
}
const VIDIOC_ENUM_FMT: u32 = vidioc::<bindings::v4l2_fmtdesc>(IOC_READ | IOC_WRITE, 2);
const VIDIOC_G_FMT: u32 = vidioc::<bindings::v4l2_format>(IOC_READ | IOC_WRITE, 4);
const VIDIOC_S_FMT: u32 = vidioc::<bindings::v4l2_format>(IOC_READ | IOC_WRITE, 5);
const VIDIOC_REQBUFS: u32 = vidioc::<bindings::v4l2_requestbuffers>(IOC_READ | IOC_WRITE, 8);
const VIDIOC_QBUF: u32 = vidioc::<bindings::v4l2_buffer>(IOC_READ | IOC_WRITE, 15);
const VIDIOC_DQBUF: u32 = vidioc::<bindings::v4l2_buffer>(IOC_READ | IOC_WRITE, 17);
/// CAPTURE buffers queued before STREAMON(CAPTURE), at most (the driver's minimum if lower).
const CAPTURE_STREAMON_AT: u32 = 4;
const VIDIOC_STREAMON: u32 = vidioc::<libc::c_int>(IOC_WRITE, 18);
const VIDIOC_STREAMOFF: u32 = vidioc::<libc::c_int>(IOC_WRITE, 19);
const VIDIOC_G_CTRL: u32 = vidioc::<bindings::v4l2_control>(IOC_READ | IOC_WRITE, 27);
const VIDIOC_S_CTRL: u32 = vidioc::<bindings::v4l2_control>(IOC_READ | IOC_WRITE, 28);
const VIDIOC_DQEVENT: u32 = vidioc::<bindings::v4l2_event>(IOC_READ, 89);
const VIDIOC_SUBSCRIBE_EVENT: u32 = vidioc::<bindings::v4l2_event_subscription>(IOC_WRITE, 90);
const VIDIOC_G_SELECTION: u32 = vidioc::<bindings::v4l2_selection>(IOC_READ | IOC_WRITE, 94);
const VIDIOC_DECODER_CMD: u32 = vidioc::<bindings::v4l2_decoder_cmd>(IOC_READ | IOC_WRITE, 96);

/// `struct dma_heap_allocation_data` (`linux/dma-heap.h`).
#[repr(C)]
struct DmaHeapAllocation {
    len: u64,
    fd: u32,
    fd_flags: u32,
    heap_flags: u64,
}
const DMA_HEAP_IOCTL_ALLOC: u32 =
    ioc(IOC_READ | IOC_WRITE, b'H', 0, std::mem::size_of::<DmaHeapAllocation>());

const OUTPUT_MPLANE: u32 = bindings::v4l2_buf_type_V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
const CAPTURE_MPLANE: u32 = bindings::v4l2_buf_type_V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
const MEMORY_DMABUF: u32 = bindings::v4l2_memory_V4L2_MEMORY_DMABUF;

fn errno() -> i32 {
    std::io::Error::last_os_error().raw_os_error().unwrap_or(libc::EIO)
}

/// One ioctl on `fd`, retried on `EINTR`; the errno otherwise.
fn xioctl<T>(fd: RawFd, request: u32, arg: &mut T) -> Result<(), i32> {
    loop {
        // SAFETY: `arg` is the live, correctly sized structure `request` names.
        if unsafe { libc::ioctl(fd, request as _, arg as *mut T) } >= 0 {
            return Ok(());
        }
        match errno() {
            libc::EINTR => continue,
            e => return Err(e),
        }
    }
}

fn zeroed<T>() -> T {
    // SAFETY: only used for the kernel's plain-data V4L2 structures, for which all zeroes is valid.
    unsafe { std::mem::zeroed() }
}

/// The msm_vidc coded formats this backend takes.
fn vidc_takes(format: PixelFormat) -> bool {
    matches!(&format.to_u32().to_le_bytes(), b"HEVC" | b"H264" | b"VP90")
}

/// A bitstream buffer of ours: a dma-heap buffer, mapped.
struct BitstreamBuffer {
    fd: OwnedFd,
    map: *mut u8,
    len: usize,
    free: bool,
}

impl Drop for BitstreamBuffer {
    fn drop(&mut self) {
        // SAFETY: `map` is the mapping of `len` bytes made in `new` and nothing else holds it.
        unsafe { libc::munmap(self.map as *mut libc::c_void, self.len) };
    }
}

impl BitstreamBuffer {
    fn new(heap: &File, len: usize) -> Result<Self, String> {
        let mut alloc = DmaHeapAllocation {
            len: len as u64,
            fd: 0,
            fd_flags: (libc::O_RDWR | libc::O_CLOEXEC) as u32,
            heap_flags: 0,
        };
        xioctl(heap.as_raw_fd(), DMA_HEAP_IOCTL_ALLOC, &mut alloc)
            .map_err(|e| format!("dma-heap allocation of {} bytes: errno {}", len, e))?;
        // SAFETY: the heap just returned this descriptor, owned by no one else.
        let fd = unsafe { OwnedFd::from_raw_fd(alloc.fd as RawFd) };
        // SAFETY: a fresh shared mapping of the buffer, checked below.
        let map = unsafe {
            libc::mmap(
                std::ptr::null_mut(),
                len,
                libc::PROT_READ | libc::PROT_WRITE,
                libc::MAP_SHARED,
                fd.as_raw_fd(),
                0,
            )
        };
        if map == libc::MAP_FAILED {
            return Err(format!("mmap of a bitstream buffer: errno {}", errno()));
        }
        Ok(Self {
            fd,
            map: map as *mut u8,
            len,
            free: true,
        })
    }
}

/// What the session asks of its worker.
enum Command {
    Bitstream(Vec<u8>, bindings::timeval),
    Capture { index: u32, fd: OwnedFd, len: u32 },
    ClearCapture(mpsc::Sender<()>),
    Flush(mpsc::Sender<()>),
    Drain,
    Resume,
    Stop,
}

/// The worker's state: the driver, its queues, and what is waiting to go into them.
struct Stream {
    id: u32,
    dev: File,
    wake: OwnedFd,
    events: Arc<Mutex<Vec<DecoderEvent>>>,
    sink: DecoderSink,
    bitstream: Vec<BitstreamBuffer>,
    pending_bitstream: VecDeque<(Vec<u8>, bindings::timeval)>,
    /// Guest `CAPTURE` buffers lent before the queue is set up, or not yet queued.
    pending_capture: VecDeque<(u32, OwnedFd, u32)>,
    /// Guest `CAPTURE` buffers with the driver, by index; the dma-buf stays open while they are.
    queued_capture: HashMap<u32, OwnedFd>,
    capture_ready: bool,
    capture_streaming: bool,
    format_announced: bool,
    min_capture: u32,
    /// The guest's timestamp of each access unit, by the driver timestamp it was given.
    timestamps: HashMap<i64, bindings::timeval>,
    next_driver_us: i64,
    drain_wanted: bool,
    drain_sent: bool,
}

fn driver_us(tv: &bindings::timeval) -> i64 {
    tv.tv_sec as i64 * 1_000_000 + tv.tv_usec as i64
}

impl Stream {
    /// Open the driver for `coded` at `size`, set the bitstream queue up and start it.
    fn open(
        id: u32,
        coded: PixelFormat,
        size: (u32, u32),
        display_delay: Option<u32>,
        wake: OwnedFd,
        events: Arc<Mutex<Vec<DecoderEvent>>>,
        sink: DecoderSink,
    ) -> Result<Self, String> {
        let dev = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_NONBLOCK | libc::O_CLOEXEC)
            .open(DEVICE)
            .map_err(|e| format!("cannot open {}: {}", DEVICE, e))?;
        let fd = dev.as_raw_fd();
        let mut format: bindings::v4l2_format = zeroed();
        format.type_ = OUTPUT_MPLANE;
        // SAFETY: multi-planar format, the union member for this type.
        unsafe {
            format.fmt.pix_mp.pixelformat = coded.to_u32();
            format.fmt.pix_mp.width = size.0.max(16);
            format.fmt.pix_mp.height = size.1.max(16);
            format.fmt.pix_mp.num_planes = 1;
        }
        xioctl(fd, VIDIOC_S_FMT, &mut format).map_err(|e| format!("S_FMT(OUTPUT): errno {}", e))?;
        xioctl(fd, VIDIOC_G_FMT, &mut format).map_err(|e| format!("G_FMT(OUTPUT): errno {}", e))?;
        if let Some(delay) = display_delay {
            // Decode order, a frame out per access unit as soon as it is decoded: msm_vidc takes the
            // kernel's standard pair (research/vidc-probe vidc-replay --decode-order --sync).
            for (id, value) in [
                (bindings::V4L2_CID_MPEG_VIDEO_DEC_DISPLAY_DELAY_ENABLE, 1),
                (bindings::V4L2_CID_MPEG_VIDEO_DEC_DISPLAY_DELAY, delay as i32),
            ] {
                let mut control: bindings::v4l2_control = zeroed();
                control.id = id;
                control.value = value;
                xioctl(fd, VIDIOC_S_CTRL, &mut control)
                    .map_err(|e| format!("S_CTRL(display delay {:#x}): errno {}", id, e))?;
            }
        }
        // SAFETY: as above.
        let sizeimage = unsafe { format.fmt.pix_mp.plane_fmt[0].sizeimage } as usize;
        let bitstream_len = (sizeimage.max(1 << 20) + 4095) & !4095;
        for event in [bindings::V4L2_EVENT_SOURCE_CHANGE, bindings::V4L2_EVENT_EOS] {
            let mut sub: bindings::v4l2_event_subscription = zeroed();
            sub.type_ = event;
            xioctl(fd, VIDIOC_SUBSCRIBE_EVENT, &mut sub)
                .map_err(|e| format!("SUBSCRIBE_EVENT {}: errno {}", event, e))?;
        }
        let mut request: bindings::v4l2_requestbuffers = zeroed();
        request.count = OUTPUT_BUFFERS;
        request.type_ = OUTPUT_MPLANE;
        request.memory = MEMORY_DMABUF;
        xioctl(fd, VIDIOC_REQBUFS, &mut request)
            .map_err(|e| format!("REQBUFS(OUTPUT): errno {}", e))?;
        let heap = File::options()
            .read(true)
            .open(DMA_HEAP)
            .map_err(|e| format!("cannot open {}: {}", DMA_HEAP, e))?;
        let bitstream = (0..request.count.min(OUTPUT_BUFFERS))
            .map(|_| BitstreamBuffer::new(&heap, bitstream_len))
            .collect::<Result<Vec<_>, _>>()?;
        let mut kind = OUTPUT_MPLANE as libc::c_int;
        xioctl(fd, VIDIOC_STREAMON, &mut kind).map_err(|e| format!("STREAMON(OUTPUT): errno {}", e))?;
        info!(
            "decoder session {}: msm_vidc {} at {}x{}: {} bitstream buffers of {} bytes",
            id,
            String::from_utf8_lossy(&coded.to_u32().to_le_bytes()),
            size.0,
            size.1,
            bitstream.len(),
            bitstream_len
        );
        Ok(Self {
            id,
            dev,
            wake,
            events,
            sink,
            bitstream,
            pending_bitstream: VecDeque::new(),
            pending_capture: VecDeque::new(),
            queued_capture: HashMap::new(),
            capture_ready: false,
            capture_streaming: false,
            format_announced: false,
            min_capture: 1,
            timestamps: HashMap::new(),
            next_driver_us: 0,
            drain_wanted: false,
            drain_sent: false,
        })
    }

    fn emit(&self, event: DecoderEvent) {
        self.events.lock().unwrap().push(event);
        self.sink.signal();
    }

    fn fd(&self) -> RawFd {
        self.dev.as_raw_fd()
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
            unsafe { libc::poll(fds.as_mut_ptr(), 2, 100) };
            let mut count = 0u64;
            // SAFETY: draining 8 bytes from our non-blocking eventfd.
            unsafe { libc::read(self.wake.as_raw_fd(), &mut count as *mut u64 as *mut _, 8) };
            loop {
                match commands.try_recv() {
                    Ok(Command::Stop) | Err(mpsc::TryRecvError::Disconnected) => {
                        info!("decoder session {}: msm_vidc closed", self.id);
                        return;
                    }
                    Ok(command) => {
                        if let Err(e) = self.command(command) {
                            self.emit(DecoderEvent::Error(e));
                            return;
                        }
                    }
                    Err(mpsc::TryRecvError::Empty) => break,
                }
            }
            if let Err(e) = self.service() {
                self.emit(DecoderEvent::Error(format!("msm_vidc: {}", e)));
                return;
            }
        }
    }

    fn command(&mut self, command: Command) -> Result<(), String> {
        match command {
            Command::Bitstream(bytes, timestamp) => self.pending_bitstream.push_back((bytes, timestamp)),
            Command::Capture { index, fd, len } => self.pending_capture.push_back((index, fd, len)),
            Command::ClearCapture(done) => {
                if self.capture_streaming {
                    let mut kind = CAPTURE_MPLANE as libc::c_int;
                    xioctl(self.fd(), VIDIOC_STREAMOFF, &mut kind)
                        .map_err(|e| format!("STREAMOFF(CAPTURE): errno {}", e))?;
                    self.capture_streaming = false;
                }
                self.queued_capture.clear();
                self.pending_capture.clear();
                let _ = done.send(());
            }
            Command::Flush(done) => {
                let mut kind = OUTPUT_MPLANE as libc::c_int;
                xioctl(self.fd(), VIDIOC_STREAMOFF, &mut kind)
                    .map_err(|e| format!("STREAMOFF(OUTPUT): errno {}", e))?;
                for buffer in &mut self.bitstream {
                    buffer.free = true;
                }
                self.pending_bitstream.clear();
                self.drain_wanted = false;
                self.drain_sent = false;
                xioctl(self.fd(), VIDIOC_STREAMON, &mut kind)
                    .map_err(|e| format!("STREAMON(OUTPUT): errno {}", e))?;
                let _ = done.send(());
            }
            Command::Drain => self.drain_wanted = true,
            Command::Resume => {
                let mut cmd: bindings::v4l2_decoder_cmd = zeroed();
                cmd.cmd = bindings::V4L2_DEC_CMD_START;
                xioctl(self.fd(), VIDIOC_DECODER_CMD, &mut cmd)
                    .map_err(|e| format!("DECODER_CMD START: errno {}", e))?;
            }
            Command::Stop => {}
        }
        Ok(())
    }

    /// One pass over everything the driver may have for us, and everything we may give it.
    fn service(&mut self) -> Result<(), String> {
        self.take_events()?;
        self.take_bitstream_back()?;
        self.feed()?;
        if self.drain_wanted && !self.drain_sent && self.pending_bitstream.is_empty() {
            let mut cmd: bindings::v4l2_decoder_cmd = zeroed();
            cmd.cmd = bindings::V4L2_DEC_CMD_STOP;
            xioctl(self.fd(), VIDIOC_DECODER_CMD, &mut cmd)
                .map_err(|e| format!("DECODER_CMD STOP: errno {}", e))?;
            self.drain_sent = true;
        }
        self.queue_capture()?;
        self.take_frames()
    }

    fn take_events(&mut self) -> Result<(), String> {
        loop {
            let mut event: bindings::v4l2_event = zeroed();
            if xioctl(self.fd(), VIDIOC_DQEVENT, &mut event).is_err() {
                return Ok(());
            }
            if event.type_ == bindings::V4L2_EVENT_SOURCE_CHANGE {
                self.set_capture_up()?;
            }
        }
    }

    /// The `SOURCE_CHANGE`: a linear `CAPTURE` format, its layout, the slots; then the
    /// announcement.
    fn set_capture_up(&mut self) -> Result<(), String> {
        if self.format_announced {
            return Err("a mid-stream resolution change, which this backend does not follow".into());
        }
        let fd = self.fd();
        let mut offered = Vec::new();
        for index in 0.. {
            let mut desc: bindings::v4l2_fmtdesc = zeroed();
            desc.index = index;
            desc.type_ = CAPTURE_MPLANE;
            if xioctl(fd, VIDIOC_ENUM_FMT, &mut desc).is_err() {
                break;
            }
            offered.push(desc.pixelformat);
        }
        let linear = [NV12, P010]
            .into_iter()
            .find(|f| offered.contains(f))
            .ok_or_else(|| format!("no linear CAPTURE format among {:08x?}", offered))?;
        let mut format: bindings::v4l2_format = zeroed();
        format.type_ = CAPTURE_MPLANE;
        xioctl(fd, VIDIOC_G_FMT, &mut format).map_err(|e| format!("G_FMT(CAPTURE): errno {}", e))?;
        // SAFETY: multi-planar format.
        unsafe { format.fmt.pix_mp.pixelformat = linear };
        xioctl(fd, VIDIOC_S_FMT, &mut format).map_err(|e| format!("S_FMT(CAPTURE): errno {}", e))?;
        xioctl(fd, VIDIOC_G_FMT, &mut format).map_err(|e| format!("G_FMT(CAPTURE): errno {}", e))?;
        // SAFETY: multi-planar format. The fields are copied out: the struct is packed.
        let pix = unsafe { format.fmt.pix_mp };
        let (width, height) = (pix.width, pix.height);
        let (bytesperline, sizeimage) = (pix.plane_fmt[0].bytesperline, pix.plane_fmt[0].sizeimage);
        let mut selection: bindings::v4l2_selection = zeroed();
        selection.type_ = bindings::v4l2_buf_type_V4L2_BUF_TYPE_VIDEO_CAPTURE;
        selection.target = bindings::V4L2_SEL_TGT_COMPOSE;
        let visible = match xioctl(fd, VIDIOC_G_SELECTION, &mut selection) {
            Ok(()) => Rect::new(
                selection.r.left,
                selection.r.top,
                selection.r.width,
                selection.r.height,
            ),
            Err(_) => Rect::new(0, 0, width, height),
        };
        let mut control: bindings::v4l2_control = zeroed();
        control.id = bindings::V4L2_CID_MIN_BUFFERS_FOR_CAPTURE;
        self.min_capture = match xioctl(fd, VIDIOC_G_CTRL, &mut control) {
            Ok(()) => (control.value.max(1) as u32).min(CAPTURE_SLOTS),
            Err(_) => 4,
        };
        let mut request: bindings::v4l2_requestbuffers = zeroed();
        request.count = CAPTURE_SLOTS;
        request.type_ = CAPTURE_MPLANE;
        request.memory = MEMORY_DMABUF;
        xioctl(fd, VIDIOC_REQBUFS, &mut request)
            .map_err(|e| format!("REQBUFS(CAPTURE): errno {}", e))?;
        self.capture_ready = true;
        self.format_announced = true;
        info!(
            "decoder session {}: msm_vidc CAPTURE {} {}x{} (visible {}x{}) bytesperline {} \
             sizeimage {}, min {} buffers, {} slots",
            self.id,
            String::from_utf8_lossy(&linear.to_le_bytes()),
            width,
            height,
            visible.width,
            visible.height,
            bytesperline,
            sizeimage,
            self.min_capture,
            request.count
        );
        self.emit(DecoderEvent::CaptureLayout {
            pixelformat: PixelFormat::from_u32(linear),
            bytesperline,
            sizeimage,
        });
        self.emit(DecoderEvent::FormatChanged {
            coded_size: (width, height),
            visible_rect: visible,
            min_capture_buffers: self.min_capture,
        });
        Ok(())
    }

    fn take_bitstream_back(&mut self) -> Result<(), String> {
        loop {
            let mut plane: bindings::v4l2_plane = zeroed();
            let mut buffer: bindings::v4l2_buffer = zeroed();
            buffer.type_ = OUTPUT_MPLANE;
            buffer.memory = MEMORY_DMABUF;
            buffer.m.planes = &mut plane;
            buffer.length = 1;
            match xioctl(self.fd(), VIDIOC_DQBUF, &mut buffer) {
                Ok(()) => {
                    if let Some(slot) = self.bitstream.get_mut(buffer.index as usize) {
                        slot.free = true;
                    }
                }
                Err(libc::EAGAIN) | Err(libc::EPIPE) => return Ok(()),
                Err(e) => return Err(format!("DQBUF(OUTPUT): errno {}", e)),
            }
        }
    }

    /// Every waiting access unit that a free bitstream buffer can take.
    fn feed(&mut self) -> Result<(), String> {
        while !self.pending_bitstream.is_empty() {
            let Some(index) = self.bitstream.iter().position(|b| b.free) else {
                return Ok(());
            };
            let (bytes, guest_timestamp) = self.pending_bitstream.pop_front().unwrap();
            let dev = self.dev.as_raw_fd();
            let slot = &mut self.bitstream[index];
            let len = bytes.len().min(slot.len);
            if len < bytes.len() {
                warn!(
                    "decoder session {}: an access unit of {} bytes cut to {}",
                    self.id,
                    bytes.len(),
                    len
                );
            }
            // SAFETY: `len` bytes fit the mapping, which nothing else writes.
            unsafe { std::ptr::copy_nonoverlapping(bytes.as_ptr(), slot.map, len) };
            let us = self.next_driver_us;
            self.next_driver_us += DRIVER_STEP_US;
            self.timestamps.insert(us, guest_timestamp);
            // Access units that never make a frame (parameter sets alone, frames the decoder
            // drops) would leave their entry behind: keep only the most recent few hundred.
            if self.timestamps.len() > 1024 {
                let floor = us - 512 * DRIVER_STEP_US;
                self.timestamps.retain(|&k, _| k >= floor);
            }
            let mut plane: bindings::v4l2_plane = zeroed();
            plane.bytesused = len as u32;
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
            xioctl(dev, VIDIOC_QBUF, &mut buffer)
                .map_err(|e| format!("QBUF(OUTPUT) of {} bytes: errno {}", len, e))?;
            slot.free = false;
        }
        Ok(())
    }

    /// The guest buffers lent since, once the queue exists; streaming from the minimum on.
    fn queue_capture(&mut self) -> Result<(), String> {
        if !self.capture_ready {
            return Ok(());
        }
        while let Some((index, fd, len)) = self.pending_capture.pop_front() {
            if index >= CAPTURE_SLOTS {
                return Err(format!("CAPTURE buffer {} past the {} slots", index, CAPTURE_SLOTS));
            }
            let mut plane: bindings::v4l2_plane = zeroed();
            plane.length = len;
            plane.m.fd = fd.as_raw_fd();
            let mut buffer: bindings::v4l2_buffer = zeroed();
            buffer.index = index;
            buffer.type_ = CAPTURE_MPLANE;
            buffer.memory = MEMORY_DMABUF;
            buffer.m.planes = &mut plane;
            buffer.length = 1;
            xioctl(self.fd(), VIDIOC_QBUF, &mut buffer)
                .map_err(|e| format!("QBUF(CAPTURE {}, {} bytes): errno {}", index, len, e))?;
            self.queued_capture.insert(index, fd);
        }
        // msm_vidc streams and decodes with fewer than its MIN_BUFFERS_FOR_CAPTURE queued (6 for 4K HEVC Main10, research/
        // vidc-probe vidc-replay --queue): a guest that queues only what it needs in flight (the macOS kext lends each a 26.7 MB
        // surface of the player's) waited here for ever, not one frame out.
        if !self.capture_streaming
            && self.queued_capture.len() as u32 >= self.min_capture.min(CAPTURE_STREAMON_AT)
        {
            let mut kind = CAPTURE_MPLANE as libc::c_int;
            xioctl(self.fd(), VIDIOC_STREAMON, &mut kind)
                .map_err(|e| format!("STREAMON(CAPTURE): errno {}", e))?;
            self.capture_streaming = true;
        }
        Ok(())
    }

    fn take_frames(&mut self) -> Result<(), String> {
        if !self.capture_streaming {
            return Ok(());
        }
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
                // After the LAST buffer of a drain, until DEC_CMD_START.
                Err(libc::EPIPE) => return Ok(()),
                Err(e) => return Err(format!("DQBUF(CAPTURE): errno {}", e)),
            }
            self.queued_capture.remove(&buffer.index);
            let is_last = buffer.flags & bindings::V4L2_BUF_FLAG_LAST != 0;
            if is_last {
                self.drain_wanted = false;
                self.drain_sent = false;
            }
            let error = buffer.flags & bindings::V4L2_BUF_FLAG_ERROR != 0;
            let timestamp = self
                .timestamps
                .remove(&driver_us(&buffer.timestamp))
                .unwrap_or(buffer.timestamp);
            self.emit(DecoderEvent::FrameDecoded {
                index: buffer.index,
                bytesused: if error { 0 } else { plane.bytesused },
                timestamp,
                is_last,
            });
        }
    }
}

/// A session on msm_vidc: the worker thread that owns the driver, and the line to it.
struct VidcSession {
    commands: mpsc::Sender<Command>,
    wake: OwnedFd,
    events: Arc<Mutex<Vec<DecoderEvent>>>,
    thread: Option<JoinHandle<()>>,
}

impl VidcSession {
    fn start(
        id: u32,
        sink: DecoderSink,
        coded: PixelFormat,
        size: (u32, u32),
        display_delay: Option<u32>,
    ) -> Result<Self, String> {
        // SAFETY: eventfd takes no pointers; checked before it is owned.
        let raw = unsafe { libc::eventfd(0, libc::EFD_CLOEXEC | libc::EFD_NONBLOCK) };
        if raw < 0 {
            return Err(format!("eventfd: errno {}", errno()));
        }
        // SAFETY: just created, owned by no one else.
        let wake = unsafe { OwnedFd::from_raw_fd(raw) };
        let worker_wake = wake.try_clone().map_err(|e| format!("eventfd dup: {}", e))?;
        let events = Arc::new(Mutex::new(Vec::new()));
        let worker_events = Arc::clone(&events);
        let (commands, receiver) = mpsc::channel();
        let (opened, open_result) = mpsc::channel();
        let thread = std::thread::Builder::new()
            .name(format!("vidc_dec_{}", id))
            .spawn(move || {
                match Stream::open(id, coded, size, display_delay, worker_wake, worker_events, sink) {
                    Ok(stream) => {
                        let _ = opened.send(Ok(()));
                        stream.run(receiver);
                    }
                    Err(e) => {
                        let _ = opened.send(Err(e));
                    }
                }
            })
            .map_err(|e| format!("cannot start the vidc thread: {}", e))?;
        match open_result.recv_timeout(ANSWER_TIMEOUT) {
            Ok(Ok(())) => Ok(Self {
                commands,
                wake,
                events,
                thread: Some(thread),
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

    /// Send a command the worker must finish before the caller goes on.
    fn send_and_wait(&self, make: impl FnOnce(mpsc::Sender<()>) -> Command) -> IoctlResult<()> {
        let (done, wait) = mpsc::channel();
        self.send(make(done))?;
        wait.recv_timeout(ANSWER_TIMEOUT).map_err(|_| libc::EIO)
    }

    fn stop(&mut self) {
        let _ = self.send(Command::Stop);
        if let Some(thread) = self.thread.take() {
            let _ = thread.join();
        }
    }
}

impl Drop for VidcSession {
    fn drop(&mut self) {
        self.stop();
    }
}

/// The MediaCodec decoder backend, with msm_vidc in front of it where `vidc` is set.
#[derive(Clone)]
pub struct HwDecoderBackend {
    codec: MediaCodecDecoderBackend,
    vidc: bool,
}

impl HwDecoderBackend {
    pub fn new(codec: MediaCodecDecoderBackend, vidc: bool) -> Self {
        if vidc {
            info!("decoder: msm_vidc ({}) first, MediaCodec where it cannot", DEVICE);
        }
        Self { codec, vidc }
    }
}

pub struct HwDecoderSession {
    id: u32,
    sink: DecoderSink,
    vidc_allowed: bool,
    /// The display delay the guest asked for (`set_display_delay`); `Some`, the session is msm_vidc's
    /// or nothing: MediaCodec puts frames out in display order only.
    display_delay: Option<u32>,
    vidc: Option<VidcSession>,
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
            vidc_allowed: self.vidc,
            display_delay: None,
            vidc: None,
            codec: self.codec.new_session(id, sink)?,
        })
    }

    fn close_session(&mut self, mut session: HwDecoderSession) {
        if let Some(mut vidc) = session.vidc.take() {
            vidc.stop();
        }
        self.codec.close_session(session.codec);
    }

    fn min_capture_buffers(&self, fourcc: PixelFormat) -> Option<u32> {
        self.codec.min_capture_buffers(fourcc)
    }

    fn decode_order(&self) -> bool {
        self.vidc
    }
}

impl VideoDecoderBackendSession for HwDecoderSession {
    fn start(&mut self, coded_format: PixelFormat, coded_size: (u32, u32)) -> IoctlResult<()> {
        if self.vidc_allowed && vidc_takes(coded_format) {
            match VidcSession::start(self.id, self.sink.clone(), coded_format, coded_size, self.display_delay) {
                Ok(vidc) => {
                    self.vidc = Some(vidc);
                    return Ok(());
                }
                Err(e) if self.display_delay.is_some() => {
                    warn!("decoder session {}: msm_vidc refused ({}), and decode order was asked for", self.id, e);
                    return Err(libc::EIO);
                }
                Err(e) => warn!(
                    "decoder session {}: msm_vidc refused ({}); MediaCodec instead",
                    self.id, e
                ),
            }
        }
        if self.display_delay.is_some() {
            warn!("decoder session {}: decode order asked for, only msm_vidc gives it", self.id);
            return Err(libc::EINVAL);
        }
        self.codec.start(coded_format, coded_size)
    }

    fn set_display_delay(&mut self, delay: Option<u32>) -> IoctlResult<()> {
        if !self.vidc_allowed {
            return Err(libc::EINVAL);
        }
        if self.vidc.is_some() {
            return Err(libc::EBUSY);
        }
        self.display_delay = delay;
        Ok(())
    }

    fn decode(&mut self, buffer: InputBuffer) -> IoctlResult<()> {
        let Some(vidc) = &self.vidc else {
            return self.codec.decode(buffer);
        };
        // The bitstream is copied out at once (an access unit is small) and its buffer given
        // straight back, so nothing of the guest's is held across a seek.
        // SAFETY: the device lends `len` readable bytes at `ptr` until `InputBufferDone`.
        let bytes = unsafe { std::slice::from_raw_parts(buffer.ptr.as_ptr(), buffer.len) }.to_vec();
        vidc.send(Command::Bitstream(bytes, buffer.timestamp))?;
        vidc.events.lock().unwrap().push(DecoderEvent::InputBufferDone(buffer.index));
        self.sink.signal();
        Ok(())
    }

    fn use_as_capture(&mut self, buffer: OutputBuffer) -> IoctlResult<()> {
        let Some(vidc) = &self.vidc else {
            return self.codec.use_as_capture(buffer);
        };
        let Some(raw) = buffer.dmabuf else {
            warn!(
                "decoder session {}: CAPTURE buffer {} has no dma-buf; msm_vidc cannot write it",
                self.id, buffer.index
            );
            return Err(libc::EINVAL);
        };
        // SAFETY: `raw` is open while the buffer is lent; the copy is ours to close.
        let fd = unsafe { libc::fcntl(raw, libc::F_DUPFD_CLOEXEC, 0) };
        if fd < 0 {
            return Err(errno());
        }
        // SAFETY: just duplicated, owned by no one else.
        let fd = unsafe { OwnedFd::from_raw_fd(fd) };
        vidc.send(Command::Capture {
            index: buffer.index,
            fd,
            len: buffer.len.min(u32::MAX as usize) as u32,
        })
    }

    fn clear_capture_buffers(&mut self) -> IoctlResult<()> {
        match &self.vidc {
            Some(vidc) => vidc.send_and_wait(Command::ClearCapture),
            None => self.codec.clear_capture_buffers(),
        }
    }

    fn flush(&mut self) -> IoctlResult<()> {
        match &self.vidc {
            Some(vidc) => vidc.send_and_wait(Command::Flush),
            None => self.codec.flush(),
        }
    }

    fn reinit(&mut self) -> IoctlResult<()> {
        match &self.vidc {
            Some(_) => Ok(()),
            None => self.codec.reinit(),
        }
    }

    fn drain(&mut self) -> IoctlResult<()> {
        match &self.vidc {
            Some(vidc) => vidc.send(Command::Drain),
            None => self.codec.drain(),
        }
    }

    fn resume(&mut self) {
        match &self.vidc {
            Some(vidc) => {
                let _ = vidc.send(Command::Resume);
            }
            None => self.codec.resume(),
        }
    }

    fn stop(&mut self) {
        if let Some(mut vidc) = self.vidc.take() {
            vidc.stop();
        }
        self.codec.stop();
    }

    fn take_events(&mut self) -> Vec<DecoderEvent> {
        match &self.vidc {
            Some(vidc) => std::mem::take(&mut *vidc.events.lock().unwrap()),
            None => self.codec.take_events(),
        }
    }

    fn wants_dmabuf(&self) -> bool {
        self.vidc.is_some()
    }
}
