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
//!  (backend) set_controls  ──commands──▶       Controls -> one Camera::apply (+ AF trigger)
//!  close()                 ──commands──▶     Stop -> drop(Camera) -> exit  (close() joins)
//!  take_filled()           ◀──filled────     FilledBuffer per frame, then sink.signal()
//!  take_events()           ◀──events────     Disconnected / Error / Control, then sink.signal()
//!                                            ▲                       ▲
//!                          binder thread ────┘ device-state callback  │ capture-result callback
//!                                              (StateListener)        (ResultListener)
//! ```
//!
//! # Controls
//!
//! A V4L2 control set reaches the capture thread as one `Command::Controls` -- from the stream's
//! `set_controls` (the frame rate, `S_PARM`) or the backend's (everything else, whichever session
//! set it) -- and becomes one `Camera::apply`, i.e. one `setRepeatingRequest`, plus a one-shot
//! capture for an AF trigger. The stream is opened with every control at its current value,
//! written into the request before its first submission. The mapping from V4L2 to Camera2,
//! with the units, is in [`build_update`]; the auto-exposure state machine that decides
//! `AE_MODE` from the two V4L2 manual switches and the flash control is in [`Applied::ae`].
//!
//! # Capture results
//!
//! Every completed capture's metadata is read on the framework's callback thread
//! ([`result_listener`]): the autofocus state, the auto-exposure state and the active physical
//! lens are compared with what was last reported and, only when different, sent down the events
//! channel as `CameraEvent::Control` with the sink bumped -- so thirty results a second become a
//! handful of `V4L2_EVENT_CTRL`s. The applied zoom, exposure time and sensitivity are read too
//! and kept for the log; in auto exposure they change every frame and are no control's value.
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

use std::sync::Arc;
use std::sync::Mutex;

use android_camera::AntibandingMode;
use android_camera::Area;
use android_camera::AwbMode;
use android_camera::Camera;
use android_camera::CameraError;
use android_camera::CaptureResult;
use android_camera::ControlMode;
use android_camera::DeviceState;
use android_camera::EffectMode;
use android_camera::Frame;
use android_camera::LensFacing;
use android_camera::Rect;
use android_camera::RequestUpdate;
use android_camera::ResultListener;
use android_camera::StateListener;
use android_camera::VideoStabilizationMode;
use android_camera::YuvLayout;
use anyhow::bail;
use anyhow::Context;
use base::error;
use base::info;
use base::warn;
use virtio_media::devices::camera::AeMode;
use virtio_media::devices::camera::AeState;
use virtio_media::devices::camera::AfMode;
use virtio_media::devices::camera::AfTrigger;
use virtio_media::devices::camera::CameraBackend;
use virtio_media::devices::camera::CameraControl;
use virtio_media::devices::camera::CameraEvent;
use virtio_media::devices::camera::CameraInfo;
use virtio_media::devices::camera::CameraStream;
use virtio_media::devices::camera::CaptureSink;
use virtio_media::devices::camera::ColorEffect;
use virtio_media::devices::camera::EmptyBuffer;
use virtio_media::devices::camera::ExposureBias;
use virtio_media::devices::camera::ExposureMode;
use virtio_media::devices::camera::FilledBuffer;
use virtio_media::devices::camera::FlashLed;
use virtio_media::devices::camera::FrameSize;
use virtio_media::devices::camera::IsoMode;
use virtio_media::devices::camera::MaxRegions;
use virtio_media::devices::camera::PowerLine;
use virtio_media::devices::camera::Region;
use virtio_media::devices::camera::SceneMode;
use virtio_media::devices::camera::StreamRequest;
use virtio_media::devices::camera::WhiteBalance;
use virtio_media::devices::camera::AF_STATUS_BUSY;
use virtio_media::devices::camera::AF_STATUS_FAILED;
use virtio_media::devices::camera::AF_STATUS_IDLE;
use virtio_media::devices::camera::AF_STATUS_REACHED;
use virtio_media::devices::camera::REGION_SCALE;

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
/// The floor of the stall interval: the camera is running and no frame has come for at least
/// this long. The interval in force is [`stall_report`], which scales it by what the guest asked
/// for -- a 54-second exposure delivers one frame a minute, and that is not a stall (D38).
const STALL_REPORT: Duration = Duration::from_secs(2);
/// How many capture results to wait for before deciding whether the camera echoed a mode the
/// guest asked for: the request pipeline is about three frames deep (B7-controls 17.4), so an
/// immediate comparison would always read the previous value.
const ECHO_AFTER: u32 = 6;
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
    /// The Camera2-side facts the capture thread needs beyond `info`.
    facts: Facts,
    /// The way to the stream opened last, for `set_controls`: `None` before the first stream;
    /// after one closes the send fails, which is "no stream" and not an error.
    stream_commands: Option<mpsc::Sender<Command>>,
}

/// What the Camera2 side knows that the crate's `CameraInfo` does not say.
#[derive(Clone, Debug)]
struct Facts {
    /// The sensor's active array: the coordinate system of the regions.
    active_array: Option<Rect>,
    /// The AF mode `FOCUS_AUTO` set selects, and the one it cleared selects.
    af_continuous: android_camera::AfMode,
    af_single: android_camera::AfMode,
    /// The V4L2 white-balance presets the camera has, with the Camera2 mode each stands for.
    awb: Vec<(WhiteBalance, AwbMode)>,
    /// The V4L2 colour effects the camera has, with the Camera2 effect each stands for.
    effects: Vec<(ColorEffect, EffectMode)>,
    /// The V4L2 scene modes the camera has, with the Camera2 scene each stands for.
    scenes: Vec<(SceneMode, android_camera::SceneMode)>,
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
        let (info, facts) = describe(chosen);
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
        info!(
            "camera {}: zoom {:?}, af {:?}, flash {}, ae {:?}, exposure {:?} ns, iso {:?}, \
             bias {:?}, awb {:?}, antibanding {:?}, effects {:?}, scenes {:?}, stabilization {}, \
             regions {:?}, active array {:?}, lenses {:?}",
            info.id,
            info.zoom_range,
            info.af_modes,
            info.flash,
            info.ae_modes,
            info.exposure_range_ns,
            info.iso_range,
            info.exposure_bias,
            info.awb_modes,
            info.antibanding,
            info.effects,
            info.scenes,
            info.stabilization,
            info.max_regions,
            facts.active_array,
            info.physical_ids
        );
        Ok(Self {
            info,
            facts,
            stream_commands: None,
        })
    }
}

/// Camera2's white-balance modes as V4L2 presets, where one exists (`Off` is manual white
/// balance, which V4L2 does through separate gain controls this device does not offer).
const AWB_MODES: [(AwbMode, WhiteBalance); 8] = [
    (AwbMode::Auto, WhiteBalance::Auto),
    (AwbMode::Incandescent, WhiteBalance::Incandescent),
    (AwbMode::Fluorescent, WhiteBalance::Fluorescent),
    (AwbMode::WarmFluorescent, WhiteBalance::FluorescentH),
    (AwbMode::Daylight, WhiteBalance::Daylight),
    (AwbMode::CloudyDaylight, WhiteBalance::Cloudy),
    (AwbMode::Twilight, WhiteBalance::Horizon),
    (AwbMode::Shade, WhiteBalance::Shade),
];

/// Camera2's colour effects as V4L2 ones, where one exists (posterize, whiteboard and
/// blackboard have none).
const EFFECT_MODES: [(EffectMode, ColorEffect); 6] = [
    (EffectMode::Off, ColorEffect::None),
    (EffectMode::Mono, ColorEffect::BlackWhite),
    (EffectMode::Negative, ColorEffect::Negative),
    (EffectMode::Solarize, ColorEffect::Solarization),
    (EffectMode::Sepia, ColorEffect::Sepia),
    (EffectMode::Aqua, ColorEffect::Aqua),
];

/// Camera2's scene modes as V4L2 ones, where one exists. Two Camera2 scenes share a V4L2 one
/// twice (beach and snow; action and sports): the first listed here wins when both are
/// offered.
const SCENE_MODES: [(android_camera::SceneMode, SceneMode); 13] = [
    (android_camera::SceneMode::Disabled, SceneMode::None),
    (android_camera::SceneMode::Sports, SceneMode::Sports),
    (android_camera::SceneMode::Action, SceneMode::Sports),
    (android_camera::SceneMode::Portrait, SceneMode::Portrait),
    (android_camera::SceneMode::Landscape, SceneMode::Landscape),
    (android_camera::SceneMode::Night, SceneMode::Night),
    (android_camera::SceneMode::Beach, SceneMode::BeachSnow),
    (android_camera::SceneMode::Snow, SceneMode::BeachSnow),
    (android_camera::SceneMode::Sunset, SceneMode::Sunset),
    (android_camera::SceneMode::Fireworks, SceneMode::Fireworks),
    (android_camera::SceneMode::Party, SceneMode::PartyIndoor),
    (
        android_camera::SceneMode::Candlelight,
        SceneMode::CandleLight,
    ),
    (android_camera::SceneMode::Barcode, SceneMode::Text),
];

/// The crate's view of a camera from the NDK's, and the Camera2 facts kept beside it.
fn describe(camera: &android_camera::CameraInfo) -> (CameraInfo, Facts) {
    let af_modes: Vec<android_camera::AfMode> = camera
        .af_modes
        .iter()
        .filter_map(|&m| android_camera::AfMode::from_u8(m))
        .collect();
    let has = |mode: android_camera::AfMode| af_modes.contains(&mode);
    // What `FOCUS_AUTO` selects: continuous video first (the RECORD template's own), then
    // continuous picture; cleared, the single-shot mode a trigger scans in, else off.
    let af_continuous = if has(android_camera::AfMode::ContinuousVideo) {
        android_camera::AfMode::ContinuousVideo
    } else {
        android_camera::AfMode::ContinuousPicture
    };
    let af_single = if has(android_camera::AfMode::Auto) {
        android_camera::AfMode::Auto
    } else if has(android_camera::AfMode::Macro) {
        android_camera::AfMode::Macro
    } else {
        android_camera::AfMode::Off
    };
    let awb: Vec<(WhiteBalance, AwbMode)> = AWB_MODES
        .iter()
        .filter(|(mode, _)| camera.awb_modes.contains(&(*mode as u8)))
        .map(|&(mode, preset)| (preset, mode))
        .collect();
    let effects: Vec<(ColorEffect, EffectMode)> = EFFECT_MODES
        .iter()
        .filter(|(mode, _)| camera.effects.contains(&(*mode as u8)))
        .map(|&(mode, effect)| (effect, mode))
        .collect();
    let mut scenes: Vec<(SceneMode, android_camera::SceneMode)> = Vec::new();
    for &(mode, scene) in SCENE_MODES.iter() {
        if camera.scene_modes.contains(&(mode as u8)) && !scenes.iter().any(|(s, _)| *s == scene) {
            scenes.push((scene, mode));
        }
    }
    let regions = |n: i32| n.max(0) as u32;
    let facts = Facts {
        active_array: camera.active_array.filter(|r| r.width > 0 && r.height > 0),
        af_continuous,
        af_single,
        awb,
        effects,
        scenes,
    };
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
    let info = CameraInfo {
        id: camera.id.clone(),
        name: format!("Camera {} ({:?})", camera.id, camera.facing),
        sizes,
        fps_ranges,
        // Hundredths, rounded inward so every value offered is one the camera takes.
        zoom_range: camera
            .zoom_ratio_range
            .map(|(lo, hi)| ((lo * 100.0).ceil() as u32, (hi * 100.0).floor() as u32))
            .filter(|&(lo, hi)| lo > 0 && lo <= hi),
        af_modes: af_modes
            .iter()
            .map(|m| match m {
                android_camera::AfMode::Off => AfMode::Off,
                android_camera::AfMode::Auto => AfMode::Auto,
                android_camera::AfMode::Macro => AfMode::Macro,
                android_camera::AfMode::ContinuousVideo => AfMode::ContinuousVideo,
                android_camera::AfMode::ContinuousPicture => AfMode::ContinuousPicture,
                android_camera::AfMode::Edof => AfMode::Edof,
            })
            .collect(),
        flash: camera.flash_available,
        ae_modes: camera
            .ae_modes
            .iter()
            .filter_map(|&m| android_camera::AeMode::from_u8(m))
            .filter_map(|m| match m {
                android_camera::AeMode::Off => Some(AeMode::Off),
                android_camera::AeMode::On => Some(AeMode::On),
                android_camera::AeMode::OnAutoFlash => Some(AeMode::OnAutoFlash),
                android_camera::AeMode::OnAlwaysFlash => Some(AeMode::OnAlwaysFlash),
                android_camera::AeMode::OnAutoFlashRedeye => Some(AeMode::OnAutoFlashRedeye),
                android_camera::AeMode::OnExternalFlash => Some(AeMode::OnExternalFlash),
                android_camera::AeMode::OnLowLightBoostBrightnessPriority => None,
            })
            .collect(),
        exposure_range_ns: camera
            .exposure_time_range_ns
            .filter(|&(lo, hi)| lo > 0 && lo <= hi)
            .map(|(lo, hi)| (lo as u64, hi as u64)),
        iso_range: camera
            .sensitivity_range
            .filter(|&(lo, hi)| lo > 0 && lo <= hi)
            .map(|(lo, hi)| (lo as u32, hi as u32)),
        exposure_bias: match (camera.ae_compensation_range, camera.ae_compensation_step) {
            (Some((min, max)), Some((num, den))) if max > min && den > 0 && num != 0 => {
                Some(ExposureBias {
                    min,
                    max,
                    step_num: num,
                    step_den: den,
                })
            }
            _ => None,
        },
        awb_modes: facts.awb.iter().map(|&(preset, _)| preset).collect(),
        antibanding: camera
            .antibanding_modes
            .iter()
            .filter_map(|&m| AntibandingMode::from_u8(m))
            .map(|m| match m {
                AntibandingMode::Off => PowerLine::Disabled,
                AntibandingMode::Hz50 => PowerLine::Hz50,
                AntibandingMode::Hz60 => PowerLine::Hz60,
                AntibandingMode::Auto => PowerLine::Auto,
            })
            .collect(),
        effects: facts.effects.iter().map(|&(effect, _)| effect).collect(),
        scenes: facts.scenes.iter().map(|&(scene, _)| scene).collect(),
        stabilization: camera
            .video_stabilization_modes
            .contains(&(VideoStabilizationMode::On as u8)),
        // Regions need the coordinate system they are converted into.
        max_regions: match facts.active_array {
            Some(_) => MaxRegions {
                ae: regions(camera.max_regions.0),
                awb: regions(camera.max_regions.1),
                af: regions(camera.max_regions.2),
            },
            None => MaxRegions::default(),
        },
        physical_ids: camera.physical_ids.clone(),
    };
    (info, facts)
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
#[derive(Debug)]
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
        let facts = self.facts.clone();
        self.stream_commands = Some(commands.clone());

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
                // Shared with the capture thread: what the last set asked of the three modes
                // the HAL may ignore, so the result listener can say what became of them (D36).
                let echo = Arc::new(Mutex::new(ModeEcho::default()));
                let results = result_listener(
                    id.clone(),
                    events_tx.clone(),
                    sink.clone(),
                    Arc::clone(&echo),
                );
                // Every control at its current value, and the frame rate, go into the request
                // before it is first submitted: the first frame is taken with them.
                let geometry = Geometry {
                    active_array: facts.active_array,
                    width: request.width,
                    height: request.height,
                };
                let mut applied = Applied::default();
                let (mut initial, triggers, asked) =
                    build_update(&id, &facts, &geometry, &mut applied, &request.controls);
                if !asked.is_empty() {
                    *echo.lock().unwrap_or_else(|e| e.into_inner()) = asked;
                }
                initial = initial.fps_range(request.fps.0 as i32, request.fps.1 as i32);
                applied.fps_min = request.fps.0;
                let mut camera = match Camera::open_with(
                    &id,
                    request.width as i32,
                    request.height as i32,
                    depth,
                    Some(listener),
                    Some(results),
                    &initial,
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
                fire_triggers(&id, &mut camera, &triggers);
                info!(
                    "camera {}: streaming {}x{} at {:?} fps, {} reader slots, latest-frame {}, \
                     {} controls applied, ae {:?}",
                    id,
                    request.width,
                    request.height,
                    request.fps,
                    depth,
                    if camera.can_acquire_latest() {
                        "on"
                    } else {
                        "unavailable"
                    },
                    request.controls.len(),
                    applied.ae()
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
                    &facts,
                    &geometry,
                    &mut applied,
                    &echo,
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
                self.stream_commands = None;
                join_capture_thread(&id, thread);
                Err(errno)
            }
            // The thread died before it could say (a panic would have aborted the process; this
            // is the channel closing without a message).
            Err(RecvTimeoutError::Disconnected) => {
                self.stream_commands = None;
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
                // so joining here would park the worker exactly as the unbounded wait did. Every
                // command sender is dropped instead -- this one and the backend's copy for
                // `set_controls` -- which disconnects the channel; whenever the open does return,
                // the loop's first `try_recv` sees `Disconnected` and the thread exits, dropping
                // its `Camera` -- the platform's only handback -- on the thread that opened it.
                // It holds no lent buffer: nothing is lent until `STREAMON` has succeeded, and
                // this one has not.
                self.stream_commands = None;
                drop(commands);
                Err(libc::ETIMEDOUT)
            }
        }
    }

    /// The V4L2 controls, from whichever session set them, to the stream opened last. A closed
    /// stream's thread has dropped its receiver and the send fails, which is "no stream" -- the
    /// values will come back in the next `StreamRequest` -- not an error.
    fn set_controls(&mut self, controls: &[CameraControl]) -> Result<(), i32> {
        if let Some(commands) = &self.stream_commands {
            let _ = commands.send(Command::Controls(controls.to_vec()));
        }
        Ok(())
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

/// The stream's geometry, for converting a guest's region into sensor coordinates.
struct Geometry {
    active_array: Option<Rect>,
    width: u32,
    height: u32,
}

impl Geometry {
    /// `region` (stream-relative, in `REGION_SCALE`ths of the frame) as a Camera2 area in the
    /// active array's coordinates; `None` for a region that is not set, or with no active array
    /// to convert into.
    ///
    /// The frame is the active array cropped to the frame's aspect ratio about its centre --
    /// the "additional crop resulted from the aspect ratio differences between the preview
    /// stream and `SCALER_CROP_REGION`" the header describes (`NdkCameraMetadataTags.h`,
    /// `ACAMERA_CONTROL_AE_REGIONS`, :655-662) -- since this device never sets
    /// `SCALER_CROP_REGION` and it stays the whole array. Zoom needs no arithmetic here:
    /// "Starting from API level 30, the coordinate system of activeArraySize ... is used to
    /// represent post-zoomRatio field of view" (:663-670), so with `CONTROL_ZOOM_RATIO` the same
    /// coordinates always mean the same place *in the frame*, which is what the guest tapped.
    fn area_of(&self, region: &Region) -> Option<Area> {
        if !region.is_set() {
            return None;
        }
        let active = self.active_array?;
        let (aw, ah) = (active.width as i64, active.height as i64);
        let (sw, sh) = (self.width as i64, self.height as i64);
        if aw <= 0 || ah <= 0 || sw <= 0 || sh <= 0 {
            return None;
        }
        // The array cropped to the frame's aspect ratio, centred.
        let (cw, ch) = if aw * sh > ah * sw {
            (ah * sw / sh, ah)
        } else {
            (aw, aw * sh / sw)
        };
        let cx = active.left as i64 + (aw - cw) / 2;
        let cy = active.top as i64 + (ah - ch) / 2;
        let scale = REGION_SCALE as i64;
        let x0 = region.x.min(REGION_SCALE) as i64;
        let y0 = region.y.min(REGION_SCALE) as i64;
        let x1 = (region.x as i64 + region.width as i64).min(scale);
        let y1 = (region.y as i64 + region.height as i64).min(scale);
        let xmin = cx + x0 * cw / scale;
        let ymin = cy + y0 * ch / scale;
        // Exclusive maxima, at least one pixel past the minima.
        let xmax = (cx + x1 * cw / scale).max(xmin + 1);
        let ymax = (cy + y1 * ch / scale).max(ymin + 1);
        let clamp = |v: i64| v.clamp(i32::MIN as i64, i32::MAX as i64) as i32;
        Some(Area {
            xmin: clamp(xmin),
            ymin: clamp(ymin),
            xmax: clamp(xmax),
            ymax: clamp(ymax),
            weight: region.weight as i32,
        })
    }
}

/// What the capture thread last applied of the controls that share a Camera2 entry: the
/// auto-exposure state machine's inputs (design §7.1, plan §3.1).
#[derive(Debug, Clone, Copy)]
struct Applied {
    exposure: ExposureMode,
    iso: IsoMode,
    /// 100 µs units.
    exposure_time: u32,
    iso_value: u32,
    flash: FlashLed,
    /// The bottom of the frame-rate range in force, frames per second: the slowest the camera
    /// may deliver under automatic exposure. Zero until a range has been set.
    fps_min: u32,
}

impl Default for Applied {
    fn default() -> Self {
        Self {
            exposure: ExposureMode::Auto,
            iso: IsoMode::Auto,
            exposure_time: 333,
            iso_value: 100,
            flash: FlashLed::Off,
            fps_min: 0,
        }
    }
}

impl Applied {
    /// Whether the camera's auto-exposure is off: Camera2 has one switch, V4L2 two, and either
    /// manual switch turns it off (and then both manual values apply).
    fn manual(&self) -> bool {
        self.exposure == ExposureMode::Manual || self.iso == IsoMode::Manual
    }

    /// How long one frame should take, given what the guest asked for: its own exposure time
    /// when the exposure is manual -- a frame cannot arrive faster than the light it is made of
    /// -- and the period of the frame-rate floor otherwise. Zero when nothing is known.
    fn frame_duration(&self) -> Duration {
        let exposure = if self.manual() {
            // 100 µs units.
            Duration::from_micros(self.exposure_time as u64 * 100)
        } else {
            Duration::ZERO
        };
        let interval = match self.fps_min {
            0 => Duration::ZERO,
            fps => Duration::from_secs(1) / fps,
        };
        std::cmp::max(exposure, interval)
    }

    /// The `AE_MODE` / `FLASH_MODE` pair for the current inputs -- the state machine:
    ///
    /// | `EXPOSURE_AUTO` | `ISO_SENSITIVITY_AUTO` | `FLASH_LED_MODE` | `AE_MODE` | `FLASH_MODE` | exposure time, sensitivity |
    /// |---|---|---|---|---|---|
    /// | Auto | Auto | Off | `ON` | `OFF` | the camera's |
    /// | Auto | Auto | Torch | `ON` | `TORCH` | the camera's |
    /// | Manual | any | Off / Torch | `OFF` | `OFF` / `TORCH` | written |
    /// | any | Manual | Off / Torch | `OFF` | `OFF` / `TORCH` | written |
    ///
    /// `AE_MODE` is never one of the `ON_*_FLASH` modes: under those the 3A routine owns the
    /// LED and overrides `FLASH_MODE` (plan §3.1), so auto-flash is not offered. `Flash` (fire
    /// on strobe) is not a menu item the guest can select.
    fn ae(&self) -> (android_camera::AeMode, android_camera::FlashMode) {
        let ae = if self.manual() {
            android_camera::AeMode::Off
        } else {
            android_camera::AeMode::On
        };
        let flash = match self.flash {
            FlashLed::Torch => android_camera::FlashMode::Torch,
            FlashLed::Off | FlashLed::Flash => android_camera::FlashMode::Off,
        };
        (ae, flash)
    }
}

/// The request entries for `controls`, in the units Camera2 takes, plus the AF triggers to
/// fire after they are applied. `applied` is updated as the state machine's inputs change.
///
/// A control this camera has no mapping for (a preset `describe` did not list) is logged and
/// skipped, never refused: the guest's set is already committed by the time it gets here.
fn build_update(
    id: &str,
    facts: &Facts,
    geometry: &Geometry,
    applied: &mut Applied,
    controls: &[CameraControl],
) -> (RequestUpdate, Vec<AfTrigger>, ModeEcho) {
    let mut update = RequestUpdate::new();
    let mut triggers = Vec::new();
    let mut echo = ModeEcho::default();
    let mut ae_changed = false;
    let areas = |regions: &[Region]| -> Vec<Area> {
        regions.iter().filter_map(|r| geometry.area_of(r)).collect()
    };
    for control in controls {
        match control {
            CameraControl::FpsRange(min, max) => {
                applied.fps_min = *min;
                update = update.fps_range(*min as i32, *max as i32);
            }
            // Hundredths to the ratio.
            CameraControl::Zoom(v) => update = update.zoom_ratio(*v as f32 / 100.0),
            CameraControl::ExposureMode(mode) => {
                applied.exposure = *mode;
                ae_changed = true;
            }
            CameraControl::ExposureTime(v) => {
                applied.exposure_time = *v;
                ae_changed = true;
            }
            CameraControl::IsoMode(mode) => {
                applied.iso = *mode;
                ae_changed = true;
            }
            CameraControl::Iso(v) => {
                applied.iso_value = *v;
                ae_changed = true;
            }
            CameraControl::FlashLed(mode) => {
                applied.flash = *mode;
                ae_changed = true;
            }
            // Already in the camera's own steps.
            CameraControl::ExposureBias(steps) => {
                update = update.ae_compensation(*steps);
            }
            CameraControl::WhiteBalance(preset) => {
                match facts.awb.iter().find(|(p, _)| p == preset) {
                    Some(&(_, mode)) => {
                        echo.awb = Some(mode);
                        update = update.awb_mode(mode);
                    }
                    None => warn!("camera {}: no Camera2 mode for {:?}", id, preset),
                }
            }
            CameraControl::PowerLine(mode) => {
                update = update.antibanding_mode(match mode {
                    PowerLine::Disabled => AntibandingMode::Off,
                    PowerLine::Hz50 => AntibandingMode::Hz50,
                    PowerLine::Hz60 => AntibandingMode::Hz60,
                    PowerLine::Auto => AntibandingMode::Auto,
                });
            }
            CameraControl::ColorEffect(effect) => {
                match facts.effects.iter().find(|(e, _)| e == effect) {
                    Some(&(_, mode)) => {
                        echo.effect = Some(mode);
                        update = update.effect_mode(mode);
                    }
                    None => warn!("camera {}: no Camera2 effect for {:?}", id, effect),
                }
            }
            // A scene applies only under CONTROL_MODE USE_SCENE_MODE; `None` returns the
            // control mode to AUTO, where the individual 3A modes rule.
            CameraControl::SceneMode(scene) => {
                match facts.scenes.iter().find(|(s, _)| s == scene) {
                    Some(&(SceneMode::None, mode)) => {
                        echo.scene = Some((mode, ControlMode::Auto));
                        update = update.control_mode(ControlMode::Auto).scene_mode(mode);
                    }
                    Some(&(_, mode)) => {
                        echo.scene = Some((mode, ControlMode::UseSceneMode));
                        update = update
                            .control_mode(ControlMode::UseSceneMode)
                            .scene_mode(mode);
                    }
                    None => warn!("camera {}: no Camera2 scene for {:?}", id, scene),
                }
            }
            CameraControl::Stabilization(on) => {
                update = update.video_stabilization(if *on {
                    VideoStabilizationMode::On
                } else {
                    VideoStabilizationMode::Off
                });
            }
            CameraControl::FocusAuto(continuous) => {
                update = update.af_mode(if *continuous {
                    facts.af_continuous
                } else {
                    facts.af_single
                });
            }
            CameraControl::AfTrigger(trigger) => triggers.push(*trigger),
            CameraControl::AeRegions(regions) => update = update.ae_regions(&areas(regions)),
            CameraControl::AfRegions(regions) => update = update.af_regions(&areas(regions)),
            CameraControl::AwbRegions(regions) => update = update.awb_regions(&areas(regions)),
            // Reported by this backend, never set through it.
            CameraControl::AfStatus(_)
            | CameraControl::AeState(_)
            | CameraControl::ActivePhysicalId(_) => {}
        }
    }
    if ae_changed {
        let (ae, flash) = applied.ae();
        update = update.ae_mode(ae).flash_mode(flash);
        if applied.manual() {
            // 100 µs to ns; the ISO number as is.
            update = update
                .exposure_time_ns(applied.exposure_time as i64 * 100_000)
                .sensitivity(applied.iso_value as i32);
        }
    }
    (update, triggers, echo)
}

/// The three request entries a HAL may take or silently drop, as one submission asked for
/// them, and how many capture results have come back since (D36).
///
/// `AUTO_N_PRESET_WHITE_BALANCE`, `COLORFX` and `SCENE_MODE` are accepted by the device, mapped
/// to a Camera2 mode and submitted without an error, and on 5566 they change nothing in the
/// pixels (B7-controls 10.2, 12.2). A capture result carries the values the camera *used*, so
/// comparing the two says which half is at fault: an echoed value that changes nothing is the
/// HAL's business, a value the result does not carry is ours. The verdict is logged once per
/// set rather than per result.
#[derive(Default)]
struct ModeEcho {
    awb: Option<AwbMode>,
    effect: Option<EffectMode>,
    /// The scene, and the `CONTROL_MODE` it needs to have any effect at all.
    scene: Option<(android_camera::SceneMode, ControlMode)>,
    /// Results seen since the set; the comparison waits [`ECHO_AFTER`] of them.
    results: u32,
}

impl ModeEcho {
    fn is_empty(&self) -> bool {
        self.awb.is_none() && self.effect.is_none() && self.scene.is_none()
    }

    /// What the result says against what was asked for, one clause per mode, or `None` when
    /// nothing was asked for.
    fn verdict(&self, result: &CaptureResult<'_>) -> Option<String> {
        if self.is_empty() {
            return None;
        }
        let mut out = Vec::new();
        if let Some(want) = self.awb {
            let got = result.awb_mode();
            out.push(format!(
                "AWB_MODE {:?} -> {:?} ({})",
                want,
                got,
                if got == Some(want) {
                    "echoed"
                } else {
                    "dropped"
                }
            ));
        }
        if let Some(want) = self.effect {
            let got = result.effect_mode();
            out.push(format!(
                "EFFECT_MODE {:?} -> {:?} ({})",
                want,
                got,
                if got == Some(want) {
                    "echoed"
                } else {
                    "dropped"
                }
            ));
        }
        if let Some((want, mode)) = self.scene {
            let got = result.scene_mode();
            let got_mode = result.control_mode();
            out.push(format!(
                "SCENE_MODE {:?} -> {:?}, CONTROL_MODE {:?} -> {:?} ({})",
                want,
                got,
                mode,
                got_mode,
                if got == Some(want) && got_mode == Some(mode) {
                    "echoed"
                } else {
                    "dropped"
                }
            ));
        }
        Some(out.join("; "))
    }
}

/// Apply `controls` to the running camera in one submission, then fire any AF trigger. A
/// refused set is logged, not fatal: the stream goes on as it was.
fn apply_controls(
    id: &str,
    camera: &mut Camera,
    facts: &Facts,
    geometry: &Geometry,
    applied: &mut Applied,
    echo: &Arc<Mutex<ModeEcho>>,
    controls: &[CameraControl],
) {
    let (update, triggers, asked) = build_update(id, facts, geometry, applied, controls);
    if let Err(e) = camera.apply(&update) {
        warn!(
            "camera {}: {} control(s) refused, the stream goes on as it was: {}",
            id,
            controls.len(),
            e
        );
    }
    if !asked.is_empty() {
        *echo.lock().unwrap_or_else(|e| e.into_inner()) = asked;
    }
    fire_triggers(id, camera, &triggers);
}

/// The AF triggers of a set, each a one-shot capture.
fn fire_triggers(id: &str, camera: &mut Camera, triggers: &[AfTrigger]) {
    for trigger in triggers {
        let result = camera.trigger_af(match trigger {
            AfTrigger::Start => android_camera::AfTrigger::Start,
            AfTrigger::Cancel => android_camera::AfTrigger::Cancel,
        });
        if let Err(e) = result {
            warn!("camera {}: AF trigger {:?} refused: {}", id, trigger, e);
        }
    }
}

/// `CONTROL_AF_STATE` as the `V4L2_CID_AUTO_FOCUS_STATUS` mask.
fn af_status_of(state: android_camera::AfState) -> u32 {
    use android_camera::AfState;
    match state {
        AfState::Inactive => AF_STATUS_IDLE,
        AfState::PassiveScan | AfState::ActiveScan => AF_STATUS_BUSY,
        AfState::PassiveFocused | AfState::FocusedLocked => AF_STATUS_REACHED,
        AfState::NotFocusedLocked | AfState::PassiveUnfocused => AF_STATUS_FAILED,
    }
}

fn ae_state_of(state: android_camera::AeState) -> AeState {
    match state {
        android_camera::AeState::Inactive => AeState::Inactive,
        android_camera::AeState::Searching => AeState::Searching,
        android_camera::AeState::Converged => AeState::Converged,
        android_camera::AeState::Locked => AeState::Locked,
        android_camera::AeState::FlashRequired => AeState::FlashRequired,
        android_camera::AeState::Precapture => AeState::Precapture,
    }
}

/// What the result listener last reported, so it reports only changes.
#[derive(Default)]
struct LastReported {
    af: Option<u32>,
    ae: Option<AeState>,
    lens: Option<String>,
    zoom: Option<u32>,
}

/// The capture-result listener: runs on the framework's callback thread once per completed
/// capture, reads the three states the guest can subscribe to, and sends each as a
/// `CameraEvent::Control` only when it differs from the last one sent. The applied zoom is
/// reported the same way, but only when it is not what the guest asked for (a clamp by the
/// HAL): in that case the value the guest reads should be the camera's. Nothing here blocks:
/// one uncontended lock, channel sends and an eventfd write.
fn result_listener(
    id: String,
    events: mpsc::Sender<CameraEvent>,
    sink: CaptureSink,
    echo: Arc<Mutex<ModeEcho>>,
) -> ResultListener {
    let last = Mutex::new(LastReported::default());
    Box::new(move |result: &CaptureResult<'_>| {
        // What the camera did with the last set of the three modes it may ignore (D36): read
        // out of the result a few frames after the set, said once, and then forgotten.
        {
            let mut echo = echo.lock().unwrap_or_else(|e| e.into_inner());
            if !echo.is_empty() {
                echo.results += 1;
                if echo.results >= ECHO_AFTER {
                    if let Some(verdict) = echo.verdict(result) {
                        info!(
                            "camera {}: mode echo after {} results: {}",
                            id, ECHO_AFTER, verdict
                        );
                    }
                    *echo = ModeEcho::default();
                }
            }
        }
        let mut changed: Vec<CameraControl> = Vec::new();
        {
            let mut last = last.lock().unwrap_or_else(|e| e.into_inner());
            if let Some(status) = result.af_state().map(af_status_of) {
                if last.af != Some(status) {
                    last.af = Some(status);
                    changed.push(CameraControl::AfStatus(status));
                }
            }
            if let Some(state) = result.ae_state().map(ae_state_of) {
                if last.ae != Some(state) {
                    last.ae = Some(state);
                    changed.push(CameraControl::AeState(state));
                }
            }
            if let Some(lens) = result.active_physical_id() {
                if last.lens.as_deref() != Some(lens.as_str()) {
                    info!("camera {}: physical lens {} is active", id, lens);
                    last.lens = Some(lens.clone());
                    changed.push(CameraControl::ActivePhysicalId(lens));
                }
            }
            if let Some(ratio) = result.zoom_ratio().filter(|r| r.is_finite() && *r > 0.0) {
                let hundredths = (ratio * 100.0).round() as u32;
                if last.zoom != Some(hundredths) {
                    last.zoom = Some(hundredths);
                    changed.push(CameraControl::Zoom(hundredths));
                }
            }
        }
        if !changed.is_empty() {
            for control in changed {
                let _ = events.send(CameraEvent::Control(control));
            }
            sink.signal();
        }
    })
}

/// How long a stall must last before it is worth a line: [`STALL_REPORT`], or three times the
/// frame duration the guest's own settings imply, whichever is longer -- and then doubled for
/// every line already said, up to sixteen times, so a camera that never comes back says so a
/// handful of times rather than every two seconds.
///
/// A guest is entitled to ask for a 54-second exposure, and 5566 answers it with one frame a
/// minute; that produced 74 identical "no frame for 2s" warnings in one 150-second capture
/// (D38), which is how a real stall gets lost.
fn stall_report(applied: &Applied, already_said: u32) -> Duration {
    let base = std::cmp::max(STALL_REPORT, applied.frame_duration() * 3);
    base * (1 << already_said.min(4))
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
    facts: &Facts,
    geometry: &Geometry,
    applied: &mut Applied,
    echo: &Arc<Mutex<ModeEcho>>,
) {
    let mut empties: VecDeque<EmptyBuffer> = VecDeque::new();
    let mut sequence = 0u32;
    let mut dropped = 0u64;
    // Discarded since the last frame was handed over; see where `sequence` is set.
    let mut dropped_since_delivery = 0u32;
    let mut stalled = Duration::ZERO;
    let mut stall_reports = 0u32;

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
                Ok(Command::Controls(controls)) => {
                    apply_controls(id, camera, facts, geometry, applied, echo, &controls)
                }
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
                Ok(Command::Controls(controls)) => {
                    apply_controls(id, camera, facts, geometry, applied, echo, &controls)
                }
                Ok(Command::Stop) | Err(RecvTimeoutError::Disconnected) => return,
                Err(RecvTimeoutError::Timeout) => (),
            }
            continue;
        }

        let frame = match camera.next_frame_latest(FRAME_WAIT) {
            Ok(Some(frame)) => frame,
            Ok(None) => {
                stalled += FRAME_WAIT;
                if stalled >= stall_report(applied, stall_reports) {
                    warn!(
                        "camera {}: no frame for {:?} (sequence {}, {} dropped, one frame should \
                         take {:?})",
                        id,
                        stalled,
                        sequence,
                        dropped,
                        applied.frame_duration()
                    );
                    stalled = Duration::ZERO;
                    stall_reports = stall_reports.saturating_add(1);
                }
                continue;
            }
            Err(e) => return fail(format!("acquiring a frame failed: {e}")),
        };
        stalled = Duration::ZERO;
        stall_reports = 0;

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
