// Copyright 2026 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! The Camera2 NDK behind the virtio-media camera device (`VPU_DESIGN.md` §7.1).
//!
//! [`AndroidCameraBackend`] is one host camera as `android_camera` describes it; its
//! [`AndroidCameraStream`] is a *capture thread* that owns the open [`Camera`] -- the NDK
//! handles are raw pointers and must be created, used and dropped on one thread -- and copies
//! every frame straight into the buffer the device lent it, once, before the buffer goes back.
//!
//! # Threads and channels
//!
//! ```text
//!  device worker thread                      capture thread (one per open stream)
//!  ────────────────────                      ────────────────────────────────────
//!  give_empty(EmptyBuffer) ──commands──▶     Camera::open_with()      (STREAMON waits for this)
//!  set_controls(..)        ──commands──▶     loop { next_frame_latest(); copy; }
//!  close()                 ──commands──▶     Stop -> drop(Camera) -> exit  (close() joins)
//!  take_filled()           ◀──filled────     FilledBuffer per frame, then sink.signal()
//!  take_events()           ◀──events────     Disconnected / Error, then sink.signal()
//!                                            ▲
//!                          binder thread ────┘ device-state callback (StateListener)
//! ```
//!
//! Nothing here blocks the worker for long: `give_empty` and `set_controls` post to a channel
//! and `take_*` drain one. Two waits remain, and both are the worker's:
//!
//! * `open_stream` waits for the camera to open on the new thread -- that is `STREAMON`, which is
//!   synchronous for the guest anyway and is when the camera is taken from the host. It is bounded
//!   by [`OPEN_TIMEOUT`], because `ACameraManager_openCamera` and
//!   `ACameraDevice_createCaptureSession` are synchronous binder calls into `cameraserver` with no
//!   timeout of their own and a wedged one must not park the worker (`review-m4` R3). On a timeout
//!   the capture thread is *detached*, never joined, and `STREAMON` answers `ETIMEDOUT`.
//! * `close` (and `Drop`) joins the capture thread, which is the §2.5 barrier -- no host write
//!   into a buffer after the stream is gone -- and cannot be given up. The loop itself answers
//!   within one `FRAME_WAIT`, but the join then waits for `Camera::drop`, i.e.
//!   `ACameraDevice_close`, which "will stop all repeating captures ... and block until all
//!   capture requests ... [are] complete" (`NdkCameraDevice.h:181-184`): **unbounded by the NDK's
//!   own contract**, so this join is bounded only by the platform's goodwill.
//!
//! The worker's `KillSignal` (`crate::virtio::media::kill`) covers neither: it is armed for
//! `EventQueue::send_event` alone and does not reach into a device call. `logs/vpu_wp/F5-crosvm.md`
//! §2.3 says what making the open wait interruptible by it would take.
//!
//! # The copy
//!
//! The device's contract is tightly packed NV12 (`virtio_media::devices::camera`). A
//! `YUV_420_888` frame from the HAL is one of three layouts, read per frame (`Frame::layout`):
//! NV12 rows are copied, NV21 rows are copied with every Cb/Cr pair swapped, I420's two chroma
//! planes are interleaved; row padding (`row_stride > width`) is dropped. For the interleaved
//! layouts the chroma region is taken from the first of the two plane pointers to the farther
//! of their two ends, because each plane's own length stops one byte short of the region -- the
//! last sample's other half belongs to the other plane (`camera_probe`'s `dump_frame`). A frame
//! in any other arrangement, or of the wrong size, ends the stream with an error the guest sees.
//! The timestamp is the frame's (`AImage_getTimestamp`, the sensor's monotonic clock).

use std::collections::VecDeque;
use std::sync::mpsc;
use std::sync::mpsc::RecvTimeoutError;
use std::sync::mpsc::TryRecvError;
use std::thread;
use std::time::Duration;

use android_camera::Camera;
use android_camera::CameraError;
use android_camera::DeviceState;
use android_camera::Frame;
use android_camera::LensFacing;
use android_camera::StateListener;
use android_camera::YuvLayout;
use anyhow::bail;
use anyhow::Context;
use base::error;
use base::info;
use base::warn;
use virtio_media::devices::camera::CameraBackend;
use virtio_media::devices::camera::CameraControl;
use virtio_media::devices::camera::CameraEvent;
use virtio_media::devices::camera::CameraInfo;
use virtio_media::devices::camera::CameraStream;
use virtio_media::devices::camera::CaptureSink;
use virtio_media::devices::camera::EmptyBuffer;
use virtio_media::devices::camera::FilledBuffer;
use virtio_media::devices::camera::FrameSize;
use virtio_media::devices::camera::StreamRequest;

/// Fewest `AImage`s the reader may hold: `AImageReader_acquireLatestImage` needs two free slots
/// besides the one being copied to discard anything (`NdkImageReader.h:239-245` in NDK r29 --
/// "calling ... with less than two images of margin, that is (maxImages - currentAcquiredImages
/// < 2) will not discard as expected"). The thread holds one image, so 2 would already do; 3 is
/// the conservative choice.
const MIN_READER_DEPTH: u32 = 3;
/// Most: the capture thread holds one image at a time and copies it out, so depth beyond a few
/// only buys the HAL slack, at a gralloc buffer of a frame each.
const MAX_READER_DEPTH: u32 = 8;
/// How long one wait for a frame is; also how late a `Stop` is noticed at most.
const FRAME_WAIT: Duration = Duration::from_millis(50);
/// The camera is running but no frame has come for this long: say so, once per interval.
const STALL_REPORT: Duration = Duration::from_secs(2);
/// How long `STREAMON` waits for the capture thread to report that the camera opened. Generous:
/// a cold `cameraserver` on a loaded phone takes hundreds of milliseconds, and the guest's own
/// ioctl is what is being held. Past it the wait is given up rather than the worker parked.
const OPEN_TIMEOUT: Duration = Duration::from_secs(10);

/// One host camera, described once at construction (`list_cameras`, no permission needed) and
/// opened per stream. Cheap to clone: the device factory builds a device from it on the worker
/// thread at every start.
#[derive(Clone, Debug)]
pub struct AndroidCameraBackend {
    info: CameraInfo,
}

impl AndroidCameraBackend {
    /// Describe camera `camera_id`, or, without one, the first back-facing camera the platform
    /// lists (the first of any facing if there is none). Loads the NDK, which starts this
    /// process's binder thread pool: call it after the uid drop.
    pub fn new(camera_id: Option<&str>) -> anyhow::Result<Self> {
        let cameras = android_camera::list_cameras().context("cannot enumerate the cameras")?;
        let ids: Vec<&str> = cameras.iter().map(|c| c.id.as_str()).collect();
        let chosen = match camera_id {
            Some(id) => cameras.iter().find(|c| c.id == id).with_context(|| {
                format!(
                    "camera {id:?} is not one this uid can see (it lists [{}])",
                    ids.join(", ")
                )
            })?,
            None => cameras
                .iter()
                .find(|c| c.facing == LensFacing::Back)
                .or_else(|| cameras.first())
                .context("this uid can see no camera at all")?,
        };
        let info = describe(chosen);
        if info.sizes.is_empty() {
            bail!("camera {} offers no YUV_420_888 output size", info.id);
        }
        info!(
            "camera {}: {:?}, {} sizes ({}x{} .. {}x{}), fps ranges {:?}",
            info.id,
            chosen.facing,
            info.sizes.len(),
            info.sizes[0].width,
            info.sizes[0].height,
            info.sizes[info.sizes.len() - 1].width,
            info.sizes[info.sizes.len() - 1].height,
            info.fps_ranges
        );
        Ok(Self { info })
    }
}

/// The crate's view of a camera from the NDK's.
fn describe(camera: &android_camera::CameraInfo) -> CameraInfo {
    let sizes = camera
        .yuv_sizes
        .iter()
        .filter(|&&(w, h)| w > 0 && h > 0)
        .map(|&(w, h)| FrameSize {
            width: w as u32,
            height: h as u32,
            min_frame_duration_ns: camera
                .yuv_min_frame_durations
                .iter()
                .find(|&&((dw, dh), _)| dw == w && dh == h)
                .map(|&(_, ns)| ns)
                .filter(|&ns| ns > 0)
                .map(|ns| ns as u64),
        })
        .collect();
    let fps_ranges = camera
        .fps_ranges
        .iter()
        .filter(|&&(min, max)| min > 0 && max >= min)
        .map(|&(min, max)| (min as u32, max as u32))
        .collect();
    CameraInfo {
        id: camera.id.clone(),
        name: format!("Camera {} ({:?})", camera.id, camera.facing),
        sizes,
        fps_ranges,
    }
}

/// The errno a guest's `STREAMON` gets for a camera that would not open.
fn errno_for(e: &CameraError) -> i32 {
    match e {
        // ERROR_CAMERA_IN_USE, ERROR_MAX_CAMERA_IN_USE
        CameraError::Ndk(_, -10010 | -10011, _) => libc::EBUSY,
        // ERROR_CAMERA_DISABLED (what a uid that is not foreground, or fails AppOps, gets) and
        // ERROR_PERMISSION_DENIED
        CameraError::Ndk(_, -10012 | -10013, _) => libc::EACCES,
        // ERROR_CAMERA_DISCONNECTED
        CameraError::Ndk(_, -10002, _) => libc::ENODEV,
        CameraError::NoSuchCamera(_) | CameraError::BadCameraId => libc::ENODEV,
        CameraError::UnsupportedSize(..) => libc::EINVAL,
        CameraError::LibraryLoad(..) | CameraError::MissingSymbol(_) => libc::ENOSYS,
        _ => libc::EIO,
    }
}

/// What the device asks of the capture thread.
enum Command {
    Lend(EmptyBuffer),
    Controls(Vec<CameraControl>),
    Stop,
}

/// An open stream: the capture thread and the channels to it. See the module documentation.
pub struct AndroidCameraStream {
    commands: mpsc::Sender<Command>,
    filled: mpsc::Receiver<FilledBuffer>,
    events: mpsc::Receiver<CameraEvent>,
    thread: Option<thread::JoinHandle<()>>,
    id: String,
}

impl CameraBackend for AndroidCameraBackend {
    type Stream = AndroidCameraStream;

    fn info(&self) -> &CameraInfo {
        &self.info
    }

    fn open_stream(
        &mut self,
        request: StreamRequest,
        sink: CaptureSink,
    ) -> Result<AndroidCameraStream, i32> {
        let (commands, command_rx) = mpsc::channel();
        let (filled_tx, filled) = mpsc::channel();
        let (events_tx, events) = mpsc::channel();
        // The thread reports once whether the camera opened; `STREAMON` waits for that.
        let (opened_tx, opened_rx) = mpsc::sync_channel::<Result<(), i32>>(1);
        let id = self.info.id.clone();
        let depth = request.buffers.clamp(MIN_READER_DEPTH, MAX_READER_DEPTH) as i32;

        let thread_id = id.clone();
        let thread = thread::Builder::new()
            .name(format!("v_camera_{id}"))
            .spawn(move || {
                let id = thread_id;
                // The platform's disconnect / error callbacks come on a binder thread; they go
                // onto the event channel and wake the worker like a frame would.
                let listener: StateListener = {
                    let events = events_tx.clone();
                    let sink = sink.clone();
                    Box::new(move |state| {
                        let event = match state {
                            DeviceState::Disconnected => CameraEvent::Disconnected,
                            DeviceState::Error(code) => {
                                CameraEvent::Error(format!("camera device error {code}"))
                            }
                        };
                        let _ = events.send(event);
                        sink.signal();
                    })
                };
                let mut camera = match Camera::open_with(
                    &id,
                    request.width as i32,
                    request.height as i32,
                    depth,
                    Some(listener),
                ) {
                    Ok(camera) => camera,
                    Err(e) => {
                        error!(
                            "camera {}: cannot open {}x{} with {} reader slots: {}",
                            id, request.width, request.height, depth, e
                        );
                        let _ = opened_tx.send(Err(errno_for(&e)));
                        return;
                    }
                };
                if let Err(e) = camera.set_fps_range(request.fps.0 as i32, request.fps.1 as i32) {
                    // The range came from the camera's own list, so this should not happen; the
                    // stream runs at the template's default rate rather than not at all.
                    warn!(
                        "camera {}: fps range {:?} refused, keeping the default: {}",
                        id, request.fps, e
                    );
                }
                info!(
                    "camera {}: streaming {}x{} at {:?} fps, {} reader slots, latest-frame {}",
                    id,
                    request.width,
                    request.height,
                    request.fps,
                    depth,
                    if camera.can_acquire_latest() {
                        "on"
                    } else {
                        "unavailable"
                    }
                );
                let _ = opened_tx.send(Ok(()));
                capture_loop(
                    &id,
                    &mut camera,
                    &command_rx,
                    &filled_tx,
                    &events_tx,
                    &sink,
                    request.width,
                    request.height,
                );
                // `camera` drops here, on the thread that opened it, which gives it back to the
                // platform.
            })
            .map_err(|e| {
                error!("camera {}: cannot start the capture thread: {}", id, e);
                libc::EAGAIN
            })?;

        // The stream value is built on the success path only: an `AndroidCameraStream` that was
        // never returned would run `Drop` -- a `Stop` and a join -- on a thread this path has
        // just decided not to wait for.
        match opened_rx.recv_timeout(OPEN_TIMEOUT) {
            Ok(Ok(())) => Ok(AndroidCameraStream {
                commands,
                filled,
                events,
                thread: Some(thread),
                id,
            }),
            Ok(Err(errno)) => {
                // It failed to open, so it is on its way out and the join is bounded.
                join_capture_thread(&id, thread);
                Err(errno)
            }
            // The thread died before it could say (a panic would have aborted the process; this
            // is the channel closing without a message).
            Err(RecvTimeoutError::Disconnected) => {
                join_capture_thread(&id, thread);
                Err(libc::EIO)
            }
            Err(RecvTimeoutError::Timeout) => {
                error!(
                    "camera {}: did not open within {:?}; STREAMON fails with ETIMEDOUT and the \
                     capture thread is detached",
                    id, OPEN_TIMEOUT
                );
                // Detached, not joined: whatever `Camera::open_with` is blocked in has no bound,
                // so joining here would park the worker exactly as the unbounded wait did. The
                // command sender is dropped instead, which disconnects the channel; whenever the
                // open does return, the loop's first `try_recv` sees `Disconnected` and the
                // thread exits, dropping its `Camera` -- the platform's only handback -- on the
                // thread that opened it. It holds no lent buffer: nothing is lent until
                // `STREAMON` has succeeded, and this one has not.
                drop(commands);
                Err(libc::ETIMEDOUT)
            }
        }
    }
}

/// Wait for a capture thread that is already finishing, reporting a panic in it as an error
/// rather than propagating it into the worker.
fn join_capture_thread(id: &str, thread: thread::JoinHandle<()>) {
    if thread.join().is_err() {
        error!("camera {}: the capture thread panicked", id);
    }
}

impl AndroidCameraStream {
    fn join(&mut self) {
        if let Some(thread) = self.thread.take() {
            if thread.join().is_err() {
                error!("camera {}: the capture thread panicked", self.id);
            }
        }
    }
}

impl CameraStream for AndroidCameraStream {
    fn give_empty(&mut self, buffer: EmptyBuffer) -> Result<(), i32> {
        self.commands
            .send(Command::Lend(buffer))
            .map_err(|_| libc::EIO)
    }

    fn take_filled(&mut self) -> Vec<FilledBuffer> {
        self.filled.try_iter().collect()
    }

    fn take_events(&mut self) -> Vec<CameraEvent> {
        self.events.try_iter().collect()
    }

    fn set_controls(&mut self, controls: &[CameraControl]) -> Result<(), i32> {
        self.commands
            .send(Command::Controls(controls.to_vec()))
            .map_err(|_| libc::EIO)
    }

    /// Stop the capture thread and wait for it: after this no lent buffer is written to, and the
    /// camera has been handed back to the platform.
    fn close(mut self) {
        let _ = self.commands.send(Command::Stop);
        self.join();
    }
}

impl Drop for AndroidCameraStream {
    fn drop(&mut self) {
        // `close` is the normal path; a dropped stream still must not leave a thread writing into
        // buffers that are about to be freed.
        let _ = self.commands.send(Command::Stop);
        self.join();
    }
}

/// Apply `controls` to the running camera. A refused control is logged, not fatal: the stream
/// goes on as it was.
fn apply_controls(id: &str, camera: &mut Camera, controls: &[CameraControl]) {
    for control in controls {
        match *control {
            CameraControl::FpsRange(min, max) => {
                if let Err(e) = camera.set_fps_range(min as i32, max as i32) {
                    warn!("camera {}: fps range {}-{} refused: {}", id, min, max, e);
                }
            }
        }
    }
}

/// The capture thread's loop: frames into lent buffers until told to stop or the camera fails.
#[allow(clippy::too_many_arguments)]
fn capture_loop(
    id: &str,
    camera: &mut Camera,
    commands: &mpsc::Receiver<Command>,
    filled: &mpsc::Sender<FilledBuffer>,
    events: &mpsc::Sender<CameraEvent>,
    sink: &CaptureSink,
    width: u32,
    height: u32,
) {
    let mut empties: VecDeque<EmptyBuffer> = VecDeque::new();
    let mut sequence = 0u32;
    let mut dropped = 0u64;
    // Discarded since the last frame was handed over; see where `sequence` is set.
    let mut dropped_since_delivery = 0u32;
    let mut stalled = Duration::ZERO;

    let fail = |why: String| {
        error!("camera {}: {}", id, why);
        let _ = events.send(CameraEvent::Error(why));
        sink.signal();
    };

    loop {
        // Everything the device has posted since the last frame, without waiting.
        loop {
            match commands.try_recv() {
                Ok(Command::Lend(buffer)) => empties.push_back(buffer),
                Ok(Command::Controls(controls)) => apply_controls(id, camera, &controls),
                Ok(Command::Stop) | Err(TryRecvError::Disconnected) => return,
                Err(TryRecvError::Empty) => break,
            }
        }

        if empties.is_empty() {
            // Nothing to fill. A frame that arrives now would be stale by the time a buffer
            // comes, so let it go (and keep the reader from backing up), then wait on the
            // commands rather than on the camera so a buffer or a Stop is seen at once.
            match camera.next_frame_latest(Duration::ZERO) {
                Ok(Some(_frame)) => {
                    dropped += 1;
                    dropped_since_delivery = dropped_since_delivery.saturating_add(1);
                }
                Ok(None) => (),
                Err(e) => return fail(format!("acquiring a frame failed: {e}")),
            }
            match commands.recv_timeout(FRAME_WAIT) {
                Ok(Command::Lend(buffer)) => empties.push_back(buffer),
                Ok(Command::Controls(controls)) => apply_controls(id, camera, &controls),
                Ok(Command::Stop) | Err(RecvTimeoutError::Disconnected) => return,
                Err(RecvTimeoutError::Timeout) => (),
            }
            continue;
        }

        let frame = match camera.next_frame_latest(FRAME_WAIT) {
            Ok(Some(frame)) => frame,
            Ok(None) => {
                stalled += FRAME_WAIT;
                if stalled >= STALL_REPORT {
                    warn!(
                        "camera {}: no frame for {:?} (sequence {}, {} dropped)",
                        id, stalled, sequence, dropped
                    );
                    stalled = Duration::ZERO;
                }
                continue;
            }
            Err(e) => return fail(format!("acquiring a frame failed: {e}")),
        };
        stalled = Duration::ZERO;

        let buffer = empties.pop_front().expect("checked non-empty above");
        let bytesused = match copy_frame(&frame, &buffer, width, height) {
            Ok(bytes) => bytes,
            Err(why) => return fail(why),
        };
        // V4L2 counts `sequence` from the start of streaming, gaps included: a guest detects
        // frame loss by the jump, and `ffmpeg -f v4l2` and `v4l2src` both look for it. So the
        // numbers of the frames discarded above are skipped rather than reused (review-m4 R8).
        // Frames `AImageReader_acquireLatestImage` itself dropped inside `next_frame_latest` are
        // not counted -- the NDK does not say how many there were.
        sequence = sequence.wrapping_add(dropped_since_delivery);
        dropped_since_delivery = 0;
        let _ = filled.send(FilledBuffer {
            index: buffer.index,
            bytesused,
            timestamp_ns: frame.timestamp_ns,
            sequence,
        });
        sequence = sequence.wrapping_add(1);
        // The image goes back to the reader before the worker is woken: it is copied out.
        drop(frame);
        sink.signal();
    }
}

/// Copy `frame` into `dst` as tightly packed NV12; returns the bytes written. See the module
/// documentation for the layouts handled.
fn copy_frame(
    frame: &Frame<'_>,
    dst: &EmptyBuffer,
    width: u32,
    height: u32,
) -> Result<u32, String> {
    if frame.width != width as i32 || frame.height != height as i32 {
        return Err(format!(
            "the camera delivered a {}x{} frame to a {}x{} stream",
            frame.width, frame.height, width, height
        ));
    }
    let (w, h) = (width as usize, height as usize);
    let stride = dst.stride as usize;
    if stride < w {
        return Err(format!(
            "destination stride {stride} is narrower than the width {w}"
        ));
    }
    let chroma_rows = h.div_ceil(2);
    let chroma_pairs = w.div_ceil(2);
    let need = stride * h + stride * chroma_rows;
    if dst.len < need {
        return Err(format!(
            "a {}x{} NV12 frame needs {} bytes; the buffer holds {}",
            w, h, need, dst.len
        ));
    }
    if frame.planes.len() != 3 {
        return Err(format!(
            "YUV_420_888 frame with {} planes instead of 3",
            frame.planes.len()
        ));
    }
    let out = dst.ptr.as_ptr();

    // Luma: `h` rows of `w` bytes, whatever the source row padding.
    let y = &frame.planes[0];
    let y_stride = y.row_stride.max(0) as usize;
    if y.pixel_stride != 1 || y_stride < w || y.len() < (h - 1) * y_stride + w {
        return Err(format!(
            "luma plane does not hold {}x{}: row_stride {}, pixel_stride {}, len {}",
            w,
            h,
            y.row_stride,
            y.pixel_stride,
            y.len()
        ));
    }
    for row in 0..h {
        // SAFETY: the source row lies within the plane (checked above); the destination row
        // lies within the buffer (`need <= dst.len`); the two never overlap, one being a
        // camera image and the other a media buffer.
        unsafe {
            std::ptr::copy_nonoverlapping(y.as_ptr().add(row * y_stride), out.add(row * stride), w)
        };
    }

    let (u, v) = (&frame.planes[1], &frame.planes[2]);
    match frame.layout() {
        layout @ (YuvLayout::Nv12 | YuvLayout::Nv21) => {
            // Interleaved chroma is one region; `first` is the plane at its start. Each
            // plane's own length ends a byte early (its last sample's other half is the other
            // plane's), so the region runs to the farther of the two ends.
            let first = if layout == YuvLayout::Nv12 { u } else { v };
            let uv_stride = first.row_stride.max(0) as usize;
            if uv_stride < w {
                return Err(format!(
                    "chroma row_stride {} is narrower than the width {}",
                    first.row_stride, w
                ));
            }
            // The union of the two plane ranges is one region only because each covers at
            // least one byte: with an empty plane the first byte would be outside both and
            // `available` would describe memory this code may not read (review-m4 R10).
            if u.is_empty() || v.is_empty() {
                return Err(format!(
                    "interleaved chroma with an empty plane (u {} bytes, v {} bytes)",
                    u.len(),
                    v.len()
                ));
            }
            let start = first.as_ptr() as usize;
            let end = (u.as_ptr() as usize + u.len()).max(v.as_ptr() as usize + v.len());
            let available = end.saturating_sub(start);
            let src = first.as_ptr();
            for row in 0..chroma_rows {
                let offset = row * uv_stride;
                // Bytes of this row that exist in the image; a short last row is padded.
                let have = w.min(available.saturating_sub(offset));
                let dst_row = (h + row) * stride;
                // SAFETY: `offset + have <= available`, so the source bytes are inside the
                // region the two planes span, and the one byte past them the NV21 tail may read
                // is taken only when it is inside it too; the destination row is inside the
                // buffer.
                unsafe {
                    let src_row = src.add(offset);
                    if layout == YuvLayout::Nv12 {
                        std::ptr::copy_nonoverlapping(src_row, out.add(dst_row), have);
                    } else {
                        // NV21 is Cr,Cb per pair; NV12 wants Cb,Cr.
                        let pairs = have / 2;
                        for pair in 0..pairs {
                            *out.add(dst_row + 2 * pair) = *src_row.add(2 * pair + 1);
                            *out.add(dst_row + 2 * pair + 1) = *src_row.add(2 * pair);
                        }
                        // An odd row ends on a Cb slot in NV12 order (`have - 1` is even), and
                        // its sample sits one byte further into the source than the pairs
                        // above, NV21 being Cr,Cb. Take it when the region really holds it --
                        // it does for every row but, possibly, the last -- and pad only when it
                        // does not (review-m4 R9).
                        if have % 2 == 1 {
                            let cb = offset + have;
                            *out.add(dst_row + have - 1) =
                                if cb < available { *src.add(cb) } else { 0x80 };
                        }
                    }
                    if have < w {
                        std::ptr::write_bytes(out.add(dst_row + have), 0x80, w - have);
                    }
                }
            }
        }
        YuvLayout::I420 => {
            let (u_stride, v_stride) = (u.row_stride.max(0) as usize, v.row_stride.max(0) as usize);
            let plane_ok = |p: &android_camera::Plane, s: usize| {
                s >= chroma_pairs && p.len() >= (chroma_rows - 1) * s + chroma_pairs
            };
            if !plane_ok(u, u_stride) || !plane_ok(v, v_stride) {
                return Err(format!(
                    "I420 chroma planes do not hold {}x{}: u len {} stride {}, v len {} stride {}",
                    w,
                    h,
                    u.len(),
                    u.row_stride,
                    v.len(),
                    v.row_stride
                ));
            }
            for row in 0..chroma_rows {
                let dst_row = (h + row) * stride;
                // SAFETY: each source sample is inside its plane (checked above), each
                // destination byte inside the buffer.
                unsafe {
                    let u_row = u.as_ptr().add(row * u_stride);
                    let v_row = v.as_ptr().add(row * v_stride);
                    for x in 0..chroma_pairs {
                        *out.add(dst_row + 2 * x) = *u_row.add(x);
                        if 2 * x + 1 < w {
                            *out.add(dst_row + 2 * x + 1) = *v_row.add(x);
                        }
                    }
                }
            }
        }
        YuvLayout::Unknown => {
            return Err(format!(
                "YUV_420_888 layout is neither NV12, NV21 nor I420 (chroma pixel strides {}, {})",
                u.pixel_stride, v.pixel_stride
            ));
        }
    }

    Ok(need as u32)
}
