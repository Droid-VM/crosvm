// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

//! Exercises `android_camera` on a device, so the Rust-to-NDK path can be proven before a
//! virtio-media capture device is built on top of it.
//!
//! It answers, in one run: does the NDK link and load, does `cameraserver` accept us at this uid,
//! what does the platform say each camera can do, do frames actually arrive, what layout are they
//! in, are the pixels real rather than a black or frozen buffer, and do the controls that have
//! V4L2 equivalents take effect.
//!
//! ```text
//! camera_probe list  [--uid N]
//! camera_probe controls --id 0 [--uid N]
//! camera_probe capture --id 0 --size 1280x720 --frames 90 [--uid N] [--zoom R]
//!                      [--flash off|single|torch] [--af off|auto|continuous-video|continuous-picture]
//!                      [--fps MIN:MAX] [--exposure NS --iso N] [--af-trigger] [--watch]
//!                      [--dump PATH]
//! ```
//!
//! `controls` prints the ranges and menus the V4L2 camera device builds its controls from
//! (`VPU_DESIGN.md` §7.1): what the acceptance records. `capture` also reports what the
//! capture-result callbacks saw -- every autofocus and auto-exposure state transition, the lens
//! in use -- which is the pipeline the device's `V4L2_EVENT_CTRL` comes from; `--af-trigger`
//! fires one AF scan a second in, `--exposure`/`--iso` take the exposure manual.
//!
//! `--watch` asks for each of `CONTROL_AWB_MODE`, `CONTROL_EFFECT_MODE` and
//! `CONTROL_SCENE_MODE` in turn and prints what the capture result says the camera *used*. That
//! is the D36 decider: those three set and read back through V4L2 without moving a pixel on
//! 5566, and only the result says whether the camera took the request entry and rendered
//! nothing (the HAL's business) or never saw it at all (ours).
//!
//! `--uid` drops to that uid before touching the camera, the way `snd_helper` does for AAudio:
//! `cameraserver` resolves the client package from the real uid, and uid 0 resolves to none.

use std::env;
use std::fs::File;
use std::io::Write;
use std::process::ExitCode;
use std::time::Duration;
use std::time::Instant;

use std::sync::Arc;
use std::sync::Mutex;

use android_camera::AeMode;
use android_camera::AfMode;
use android_camera::AfTrigger;
use android_camera::AntibandingMode;
use android_camera::AwbMode;
use android_camera::Camera;
use android_camera::CaptureResult;
use android_camera::EffectMode;
use android_camera::FlashMode;
use android_camera::RequestUpdate;
use android_camera::ResultListener;
use android_camera::SceneMode;
use android_camera::VideoStabilizationMode;
use android_camera::YuvLayout;

struct Args {
    command: String,
    uid: Option<u32>,
    id: String,
    width: i32,
    height: i32,
    frames: u32,
    zoom: Option<f32>,
    flash: Option<FlashMode>,
    af: Option<AfMode>,
    fps: Option<(i32, i32)>,
    dump: Option<String>,
    /// Manual exposure: `SENSOR_EXPOSURE_TIME` in ns and `SENSOR_SENSITIVITY`, under
    /// `AE_MODE_OFF`.
    exposure_ns: Option<i64>,
    iso: Option<i32>,
    af_trigger: bool,
    /// Report what the capture result says of the three modes of D36.
    watch: bool,
}

fn parse_args() -> Result<Args, String> {
    let mut argv = env::args().skip(1);
    let command = argv.next().unwrap_or_else(|| "list".to_owned());
    let mut args = Args {
        command,
        uid: None,
        id: "0".to_owned(),
        width: 1280,
        height: 720,
        frames: 90,
        zoom: None,
        flash: None,
        af: None,
        fps: None,
        dump: None,
        exposure_ns: None,
        iso: None,
        af_trigger: false,
        watch: false,
    };
    while let Some(flag) = argv.next() {
        let mut value = || {
            argv.next()
                .ok_or_else(|| format!("{} needs a value", flag))
        };
        match flag.as_str() {
            "--uid" => args.uid = Some(value()?.parse().map_err(|e| format!("--uid: {}", e))?),
            "--id" => args.id = value()?,
            "--size" => {
                let v = value()?;
                let (w, h) = v.split_once('x').ok_or("--size wants WxH")?;
                args.width = w.parse().map_err(|e| format!("--size width: {}", e))?;
                args.height = h.parse().map_err(|e| format!("--size height: {}", e))?;
            }
            "--frames" => {
                args.frames = value()?.parse().map_err(|e| format!("--frames: {}", e))?
            }
            "--zoom" => args.zoom = Some(value()?.parse().map_err(|e| format!("--zoom: {}", e))?),
            "--flash" => {
                args.flash = Some(match value()?.as_str() {
                    "off" => FlashMode::Off,
                    "single" => FlashMode::Single,
                    "torch" => FlashMode::Torch,
                    other => return Err(format!("--flash: unknown mode {:?}", other)),
                })
            }
            "--af" => {
                args.af = Some(match value()?.as_str() {
                    "off" => AfMode::Off,
                    "auto" => AfMode::Auto,
                    "macro" => AfMode::Macro,
                    "continuous-video" => AfMode::ContinuousVideo,
                    "continuous-picture" => AfMode::ContinuousPicture,
                    other => return Err(format!("--af: unknown mode {:?}", other)),
                })
            }
            "--fps" => {
                let v = value()?;
                let (lo, hi) = v.split_once(':').ok_or("--fps wants MIN:MAX")?;
                args.fps = Some((
                    lo.parse().map_err(|e| format!("--fps min: {}", e))?,
                    hi.parse().map_err(|e| format!("--fps max: {}", e))?,
                ));
            }
            "--dump" => args.dump = Some(value()?),
            "--exposure" => {
                args.exposure_ns = Some(value()?.parse().map_err(|e| format!("--exposure: {}", e))?)
            }
            "--iso" => args.iso = Some(value()?.parse().map_err(|e| format!("--iso: {}", e))?),
            "--af-trigger" => args.af_trigger = true,
            "--watch" => args.watch = true,
            other => return Err(format!("unknown flag {:?}", other)),
        }
    }
    Ok(args)
}

/// Drop to `uid` before the first NDK call. Nothing here needs the capabilities we give up, and
/// `cameraserver` reads the real uid, not the effective one.
fn drop_to_uid(uid: u32) -> Result<(), String> {
    // setresuid, not setuid: setuid()'s set_user() -- which updates cred->user, the field
    // commit_creds() gates the per-uid RLIMIT_NPROC charge on -- is cap-gated, so a drop that
    // reaches the saved-uid path updates cred->ucounts but not cred->user, leaving the app
    // uid's NPROC counter un-incremented here yet decremented at exit; it drifts until the app
    // can no longer fork ("won't open" until reboot). setresuid() calls set_user()
    // unconditionally on a real-uid change. All three ids go to uid -- the same real-uid drop
    // cameraserver needs.
    // SAFETY: setresuid on the current process with no borrowed state; the result is checked.
    if unsafe { libc::setresuid(uid, uid, uid) } != 0 {
        return Err(format!(
            "setresuid({}) failed: {}",
            uid,
            std::io::Error::last_os_error()
        ));
    }
    // SAFETY: getuid cannot fail and takes no arguments.
    let now = unsafe { libc::getuid() };
    if now != uid {
        return Err(format!("setresuid({}) left uid at {}", uid, now));
    }
    Ok(())
}

/// `SCALER_AVAILABLE_STREAM_USE_CASES` values, named so the list reads as capabilities rather
/// than as numbers.
fn use_case_name(value: i64) -> String {
    match value {
        0 => "DEFAULT".to_owned(),
        1 => "PREVIEW".to_owned(),
        2 => "STILL_CAPTURE".to_owned(),
        3 => "VIDEO_RECORD".to_owned(),
        4 => "PREVIEW_VIDEO_STILL".to_owned(),
        5 => "VIDEO_CALL".to_owned(),
        6 => "CROPPED_RAW".to_owned(),
        other => format!("vendor(0x{:x})", other),
    }
}

/// A byte list as names, one per value, with the numbers this crate cannot name kept as such.
fn names<T: std::fmt::Debug>(values: &[u8], name: impl Fn(u8) -> Option<T>) -> String {
    if values.is_empty() {
        return "(read returned nothing)".to_owned();
    }
    values
        .iter()
        .map(|&v| match name(v) {
            Some(n) => format!("{:?}({})", n, v),
            None => format!("?({})", v),
        })
        .collect::<Vec<_>>()
        .join(", ")
}

/// The characteristics the V4L2 camera device derives its controls from, for one camera.
fn cmd_controls(args: &Args) -> Result<(), String> {
    let cameras = android_camera::list_cameras().map_err(|e| e.to_string())?;
    let c = cameras
        .iter()
        .find(|c| c.id == args.id)
        .ok_or_else(|| format!("camera {:?} is not one this uid can see", args.id))?;
    println!("camera {} ({:?})", c.id, c.facing);
    println!(
        "  zoom ratio range         {}",
        match c.zoom_ratio_range {
            Some((lo, hi)) => format!(
                "{:.2}..{:.2}  -> ZOOM_ABSOLUTE {}..{} (x100)",
                lo,
                hi,
                (lo * 100.0).ceil() as i64,
                (hi * 100.0).floor() as i64
            ),
            None => "(none: no ZOOM_ABSOLUTE)".to_owned(),
        }
    );
    println!(
        "  af available modes       {}",
        names(&c.af_modes, AfMode::from_u8)
    );
    println!("  flash available          {}", c.flash_available);
    println!(
        "  ae available modes       {}  (Off and On -> EXPOSURE_AUTO / ISO_SENSITIVITY_AUTO)",
        names(&c.ae_modes, AeMode::from_u8)
    );
    println!(
        "  exposure time range      {}",
        match c.exposure_time_range_ns {
            Some((lo, hi)) => format!(
                "{}..{} ns  -> EXPOSURE_ABSOLUTE {}..{} (100 us)",
                lo,
                hi,
                (lo as u64).div_ceil(100_000).max(1),
                (hi as u64) / 100_000
            ),
            None => "(none)".to_owned(),
        }
    );
    println!(
        "  sensitivity range        {}",
        match c.sensitivity_range {
            Some((lo, hi)) => format!("ISO {}..{}  -> ISO_SENSITIVITY menu", lo, hi),
            None => "(none)".to_owned(),
        }
    );
    println!(
        "  ae compensation          {}",
        match (c.ae_compensation_range, c.ae_compensation_step) {
            (Some((lo, hi)), Some((num, den))) => format!(
                "{}..{} steps of {}/{} EV  -> AUTO_EXPOSURE_BIAS menu of {} items",
                lo,
                hi,
                num,
                den,
                (hi as i64 - lo as i64 + 1).max(0)
            ),
            (range, step) => format!("range {:?}, step {:?}", range, step),
        }
    );
    println!(
        "  awb available modes      {}",
        names(&c.awb_modes, AwbMode::from_u8)
    );
    println!(
        "  antibanding modes        {}",
        names(&c.antibanding_modes, AntibandingMode::from_u8)
    );
    println!(
        "  available effects        {}",
        names(&c.effects, EffectMode::from_u8)
    );
    println!(
        "  available scene modes    {}",
        names(&c.scene_modes, SceneMode::from_u8)
    );
    let stabilization = names(
        &c.video_stabilization_modes,
        VideoStabilizationMode::from_u8,
    );
    println!("  video stabilization      {}", stabilization);
    println!(
        "  max regions AE/AWB/AF    {:?}  (non-zero -> the private regions controls)",
        c.max_regions
    );
    println!(
        "  active array             {}",
        match c.active_array {
            Some(r) => format!(
                "left {} top {} width {} height {}  (the regions' coordinate system)",
                r.left, r.top, r.width, r.height
            ),
            None => "(none: no regions controls)".to_owned(),
        }
    );
    println!(
        "  physical lenses          {}",
        if c.physical_ids.is_empty() {
            "(none)".to_owned()
        } else {
            format!(
                "{}  (-> the private active-physical-id menu)",
                c.physical_ids.join(", ")
            )
        }
    );
    Ok(())
}

fn cmd_list() -> Result<(), String> {
    let cameras = android_camera::list_cameras().map_err(|e| e.to_string())?;
    println!("cameras: {}", cameras.len());
    for c in &cameras {
        println!();
        println!("  id {}", c.id);
        println!("    facing              {:?}", c.facing);
        println!("    sensor orientation  {} deg", c.orientation);
        println!("    hardware level      {}", c.hardware_level);
        println!(
            "    zoom ratio range    {}",
            match c.zoom_ratio_range {
                Some((lo, hi)) => format!("{:.2}..{:.2}", lo, hi),
                None => "unsupported (pre-API-30 crop-region zoom only)".to_owned(),
            }
        );
        println!(
            "    max digital zoom    {}",
            match c.max_digital_zoom {
                Some(z) => format!("{:.2}", z),
                None => "-".to_owned(),
            }
        );
        println!("    flash available     {}", c.flash_available);
        println!(
            "    max regions AE/AWB/AF {:?}  (non-zero has no V4L2 equivalent)",
            c.max_regions
        );
        println!(
            "    capabilities        {}",
            if c.capabilities.is_empty() {
                "(read returned nothing -- not the same as none)".to_owned()
            } else {
                c.capabilities.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(",")
            }
        );
        println!(
            "    logical multi-cam   {} (capability 11 {})",
            c.is_logical_multi_camera(),
            if c.capabilities.is_empty() { "unknown" } else { "checked" }
        );
        println!(
            "    physical lenses     {}",
            if c.physical_ids.is_empty() {
                format!("(none; PHYSICAL_IDS returned {} bytes)", c.physical_ids_raw_len)
            } else {
                c.physical_ids.join(", ")
            }
        );
        println!(
            "    stream use cases    {}",
            if c.stream_use_cases.is_empty() {
                "(unsupported)".to_owned()
            } else {
                c.stream_use_cases
                    .iter()
                    .map(|u| use_case_name(*u))
                    .collect::<Vec<_>>()
                    .join(", ")
            }
        );
        // Every size, each with the minimum frame duration reported for it and the frame rate
        // that duration allows. These two lists are what the V4L2 camera device answers
        // `ENUM_FRAMESIZES` and `ENUM_FRAMEINTERVALS` from, and M4's acceptance is asked to
        // record both, so all of it is printed: this used to stop after 8 sizes and 4 durations
        // and the rest could only be inferred from the guest (defect D20).
        let duration_of = |size: &(i32, i32)| {
            c.yuv_min_frame_durations
                .iter()
                .find(|(s, _)| s == size)
                .map(|&(_, ns)| ns)
        };
        println!("    YUV_420_888 sizes   {}", c.yuv_sizes.len());
        for size in &c.yuv_sizes {
            let (w, h) = *size;
            match duration_of(size) {
                Some(ns) if ns > 0 => {
                    let fps = 1e9 / ns as f64;
                    println!("      {}x{}: {} ns ({:.1} fps max)", w, h, ns, fps)
                }
                // A duration of zero or less is not a rate; say so rather than divide by it.
                Some(ns) => println!("      {}x{}: {} ns (not a rate)", w, h, ns),
                None => println!("      {}x{}: (no min frame duration reported)", w, h),
            }
        }
        println!(
            "    fps ranges          {}",
            if c.fps_ranges.is_empty() {
                "(read returned nothing)".to_owned()
            } else {
                c.fps_ranges
                    .iter()
                    .map(|(lo, hi)| format!("{}-{}", lo, hi))
                    .collect::<Vec<_>>()
                    .join(", ")
            }
        );
        // The V4L2 device caps ENUM_FRAMEINTERVALS at each size by these. They are printed with
        // the sizes above; the two lists come from different metadata keys, so a duration for a
        // size that is not an output size is worth seeing on its own.
        let orphans: Vec<_> = c
            .yuv_min_frame_durations
            .iter()
            .filter(|(size, _)| !c.yuv_sizes.contains(size))
            .collect();
        println!(
            "    min frame durations {} for YUV_420_888{}",
            c.yuv_min_frame_durations.len(),
            if orphans.is_empty() {
                " (all shown with the sizes above)"
            } else {
                ""
            }
        );
        for ((w, h), ns) in orphans {
            println!("      {}x{}: {} ns -- no such output size", w, h, ns);
        }
    }
    Ok(())
}

/// FNV-1a over a subsample of the luma plane. Cheap enough to run on every frame, and its job is
/// only to tell frames apart -- a stream that returns the same checksum every time is a frozen
/// buffer, which looks exactly like success in a frame counter.
fn luma_digest(y: &[u8], width: i32, height: i32, row_stride: i32) -> (u64, f64) {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut sum: u64 = 0;
    let mut count: u64 = 0;
    for row in (0..height).step_by(4) {
        let start = (row as usize) * (row_stride as usize);
        let end = start + width as usize;
        if end > y.len() {
            break;
        }
        for &px in y[start..end].iter().step_by(4) {
            hash ^= px as u64;
            hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
            sum += px as u64;
            count += 1;
        }
    }
    (hash, if count == 0 { 0.0 } else { sum as f64 / count as f64 })
}

/// Write the frame with its row padding removed, so the file is exactly W*H*3/2 and any ordinary
/// YUV viewer can open it.
fn dump_frame(path: &str, frame: &android_camera::Frame<'_>) -> Result<(), String> {
    let mut file = File::create(path).map_err(|e| format!("{}: {}", path, e))?;
    let (w, h) = (frame.width as usize, frame.height as usize);

    let y = frame.plane_data(0);
    let y_stride = frame.planes[0].row_stride as usize;
    for row in 0..h {
        let start = row * y_stride;
        let end = start + w;
        if end > y.len() {
            return Err(format!("luma plane short: {} < {}", y.len(), end));
        }
        file.write_all(&y[start..end]).map_err(|e| e.to_string())?;
    }

    match frame.layout() {
        YuvLayout::Nv12 | YuvLayout::Nv21 => {
            // Interleaved chroma: one plane of w bytes by h/2 rows, taken from whichever of the
            // two plane pointers comes first. That plane's reported length is one byte short of
            // the region -- the last sample's second half belongs to the other plane's range --
            // so the final row is padded to keep the file exactly w*h*3/2 and openable.
            let first = if frame.layout() == YuvLayout::Nv12 { 1 } else { 2 };
            let uv = frame.plane_data(first);
            let uv_stride = frame.planes[first].row_stride as usize;
            for row in 0..h / 2 {
                let start = row * uv_stride;
                let end = (start + w).min(uv.len());
                let written = end.saturating_sub(start);
                if written > 0 {
                    file.write_all(&uv[start..end]).map_err(|e| e.to_string())?;
                }
                if written < w {
                    file.write_all(&vec![0x80u8; w - written])
                        .map_err(|e| e.to_string())?;
                }
            }
        }
        _ => {
            for plane in 1..3 {
                let c = frame.plane_data(plane);
                let stride = frame.planes[plane].row_stride as usize;
                for row in 0..h / 2 {
                    let start = row * stride;
                    let end = (start + w / 2).min(c.len());
                    if start >= c.len() {
                        break;
                    }
                    file.write_all(&c[start..end]).map_err(|e| e.to_string())?;
                }
            }
        }
    }
    Ok(())
}

/// What the capture-result callbacks reported: every transition of the three states the V4L2
/// device turns into `V4L2_EVENT_CTRL`, with the frame it happened at.
#[derive(Default)]
struct Transitions {
    results: u64,
    af: Option<android_camera::AfState>,
    ae: Option<android_camera::AeState>,
    lens: Option<String>,
    log: Vec<String>,
    /// The three modes of D36 as the *result* carries them -- the values the camera says it
    /// used -- kept at their latest rather than only on change, so `--watch` can read them back
    /// after a set. `CONTROL_MODE` comes with the scene: a scene applies only under
    /// `USE_SCENE_MODE`.
    awb: Option<AwbMode>,
    effect: Option<EffectMode>,
    scene: Option<SceneMode>,
    control_mode: Option<android_camera::ControlMode>,
}

fn result_listener(seen: Arc<Mutex<Transitions>>) -> ResultListener {
    Box::new(move |result: &CaptureResult<'_>| {
        let mut seen = seen.lock().unwrap_or_else(|e| e.into_inner());
        seen.results += 1;
        let n = seen.results;
        if let Some(af) = result.af_state() {
            if seen.af != Some(af) {
                seen.af = Some(af);
                seen.log.push(format!("result {}: AF_STATE {:?}", n, af));
            }
        }
        if let Some(ae) = result.ae_state() {
            if seen.ae != Some(ae) {
                seen.ae = Some(ae);
                seen.log.push(format!(
                    "result {}: AE_STATE {:?} (exposure {:?} ns, iso {:?}, zoom {:?})",
                    n,
                    ae,
                    result.exposure_time_ns(),
                    result.sensitivity(),
                    result.zoom_ratio()
                ));
            }
        }
        if let Some(lens) = result.active_physical_id() {
            if seen.lens.as_deref() != Some(lens.as_str()) {
                seen.log
                    .push(format!("result {}: ACTIVE_PHYSICAL_ID {}", n, lens));
                seen.lens = Some(lens);
            }
        }
        // What the camera says it used for the three modes of D36. Logged on change, kept at
        // their latest for `--watch`.
        let (awb, effect) = (result.awb_mode(), result.effect_mode());
        let (scene, control_mode) = (result.scene_mode(), result.control_mode());
        if awb.is_some() && seen.awb != awb {
            seen.log.push(format!("result {}: AWB_MODE {:?}", n, awb));
        }
        if effect.is_some() && seen.effect != effect {
            seen.log
                .push(format!("result {}: EFFECT_MODE {:?}", n, effect));
        }
        if (scene.is_some() || control_mode.is_some())
            && (seen.scene != scene || seen.control_mode != control_mode)
        {
            seen.log.push(format!(
                "result {}: SCENE_MODE {:?} under CONTROL_MODE {:?}",
                n, scene, control_mode
            ));
        }
        if awb.is_some() {
            seen.awb = awb;
        }
        if effect.is_some() {
            seen.effect = effect;
        }
        if scene.is_some() {
            seen.scene = scene;
        }
        if control_mode.is_some() {
            seen.control_mode = control_mode;
        }
    })
}

/// The D36 decider: ask for each of the three modes a guest can set and this camera may ignore,
/// let the request pipeline drain, and print what the capture result says the camera used.
///
/// `echoed` means the camera took the request entry, so a picture that does not change is the
/// HAL's rendering and the honest answer is to document it; `dropped` means the entry never
/// reached the camera, which would be a defect on our side of the NDK.
fn watch_modes(
    camera: &mut Camera,
    seen: &Arc<Mutex<Transitions>>,
    info: &android_camera::CameraInfo,
) -> Result<(), String> {
    // Two values of each, the first ones the camera offers that are not its neutral setting.
    let awbs: Vec<AwbMode> = info
        .awb_modes
        .iter()
        .filter_map(|&v| AwbMode::from_u8(v))
        .filter(|m| !matches!(m, AwbMode::Auto | AwbMode::Off))
        .take(2)
        .collect();
    let effects: Vec<EffectMode> = info
        .effects
        .iter()
        .filter_map(|&v| EffectMode::from_u8(v))
        .filter(|m| !matches!(m, EffectMode::Off))
        .take(2)
        .collect();
    let scenes: Vec<SceneMode> = info
        .scene_modes
        .iter()
        .filter_map(|&v| SceneMode::from_u8(v))
        .filter(|m| !matches!(m, SceneMode::Disabled))
        .take(2)
        .collect();

    println!();
    println!("D36 echo test: what the capture result says the camera used");
    for mode in &awbs {
        let update = RequestUpdate::new().awb_mode(*mode);
        camera.apply(&update).map_err(|e| e.to_string())?;
        let got = drain(camera, seen)?.0;
        println!(
            "  CONTROL_AWB_MODE     requested {:<16?} result {:<16?} {}",
            mode,
            got,
            verdict(got == Some(*mode))
        );
    }
    for mode in &effects {
        let update = RequestUpdate::new().effect_mode(*mode);
        camera.apply(&update).map_err(|e| e.to_string())?;
        let got = drain(camera, seen)?.1;
        println!(
            "  CONTROL_EFFECT_MODE  requested {:<16?} result {:<16?} {}",
            mode,
            got,
            verdict(got == Some(*mode))
        );
    }
    for mode in &scenes {
        // A scene has no effect at all unless the control mode says to use it.
        let update = RequestUpdate::new()
            .control_mode(android_camera::ControlMode::UseSceneMode)
            .scene_mode(*mode);
        camera.apply(&update).map_err(|e| e.to_string())?;
        let (_, _, got, control_mode) = drain(camera, seen)?;
        println!(
            "  CONTROL_SCENE_MODE   requested {:<16?} result {:<16?} {}  (CONTROL_MODE {:?})",
            mode,
            got,
            verdict(got == Some(*mode)),
            control_mode
        );
    }
    // Back to the neutral settings, so the rest of the run is not measured under a scene.
    let restore = RequestUpdate::new()
        .awb_mode(AwbMode::Auto)
        .effect_mode(EffectMode::Off)
        .control_mode(android_camera::ControlMode::Auto)
        .scene_mode(SceneMode::Disabled);
    camera.apply(&restore).map_err(|e| e.to_string())?;
    drain(camera, seen)?;
    Ok(())
}

fn verdict(echoed: bool) -> &'static str {
    if echoed {
        "ECHOED"
    } else {
        "DROPPED"
    }
}

/// Consume frames until the request pipeline has certainly turned over -- twelve, four times its
/// depth -- and answer with the three modes the last result carried.
#[allow(clippy::type_complexity)]
fn drain(
    camera: &mut Camera,
    seen: &Arc<Mutex<Transitions>>,
) -> Result<
    (
        Option<AwbMode>,
        Option<EffectMode>,
        Option<SceneMode>,
        Option<android_camera::ControlMode>,
    ),
    String,
> {
    for _ in 0..12 {
        if camera
            .next_frame(Duration::from_millis(2000))
            .map_err(|e| e.to_string())?
            .is_none()
        {
            return Err("no frame within 2s while draining the request pipeline".to_owned());
        }
    }
    let seen = seen.lock().unwrap_or_else(|e| e.into_inner());
    Ok((seen.awb, seen.effect, seen.scene, seen.control_mode))
}

fn cmd_capture(args: &Args) -> Result<(), String> {
    // Depth 4: enough that acquiring one frame does not stall the camera, small enough that a leak
    // shows up immediately as a stall rather than as growing memory.
    let opened = Instant::now();
    let seen = Arc::new(Mutex::new(Transitions::default()));
    let mut camera = Camera::open_with(
        &args.id,
        args.width,
        args.height,
        4,
        None,
        Some(result_listener(Arc::clone(&seen))),
        &RequestUpdate::new(),
    )
    .map_err(|e| e.to_string())?;
    println!(
        "opened camera {} at {}x{} in {:?}",
        args.id, args.width, args.height, opened.elapsed()
    );

    if let Some((lo, hi)) = args.fps {
        camera.set_fps_range(lo, hi).map_err(|e| e.to_string())?;
        println!("fps range set to {}:{}", lo, hi);
    }
    if args.exposure_ns.is_some() || args.iso.is_some() {
        // Manual exposure is AE off with both values written, in one submission.
        let mut update = RequestUpdate::new().ae_mode(AeMode::Off);
        if let Some(ns) = args.exposure_ns {
            update = update.exposure_time_ns(ns);
        }
        if let Some(iso) = args.iso {
            update = update.sensitivity(iso);
        }
        camera.apply(&update).map_err(|e| e.to_string())?;
        println!(
            "manual exposure: {:?} ns, iso {:?} (AE_MODE_OFF)",
            args.exposure_ns, args.iso
        );
    }
    if let Some(mode) = args.af {
        camera.set_af_mode(mode).map_err(|e| e.to_string())?;
        println!("af mode set to {:?}", mode);
    }
    if let Some(ratio) = args.zoom {
        camera.set_zoom_ratio(ratio).map_err(|e| e.to_string())?;
        println!("zoom ratio set to {:.2}", ratio);
    }
    if let Some(mode) = args.flash {
        camera.set_flash_mode(mode).map_err(|e| e.to_string())?;
        println!("flash mode set to {:?}", mode);
    }

    if args.watch {
        let cameras = android_camera::list_cameras().map_err(|e| e.to_string())?;
        let info = cameras
            .iter()
            .find(|c| c.id == args.id)
            .ok_or_else(|| format!("camera {:?} is not one this uid can see", args.id))?;
        watch_modes(&mut camera, &seen, info)?;
    }

    let start = Instant::now();
    let mut first_frame: Option<Duration> = None;
    let mut digests = std::collections::HashSet::new();
    let mut received = 0u32;
    let mut stalls = 0u32;
    let mut first_ts = 0i64;
    let mut last_ts = 0i64;
    let mut luma_min = f64::MAX;
    let mut luma_max = f64::MIN;

    let mut af_fired = false;
    while received < args.frames {
        // One AF scan, a second in, once the stream has settled. Before the frame is acquired:
        // a held frame borrows the camera.
        if args.af_trigger && received >= 30 && !af_fired {
            camera
                .trigger_af(AfTrigger::Start)
                .map_err(|e| e.to_string())?;
            println!("  AF trigger START fired at frame {}", received);
            af_fired = true;
        }
        let frame = match camera
            .next_frame(Duration::from_millis(2000))
            .map_err(|e| e.to_string())?
        {
            Some(frame) => frame,
            None => {
                stalls += 1;
                println!("no frame within 2s (stall {})", stalls);
                if stalls >= 3 {
                    return Err("camera produced no frames".to_owned());
                }
                continue;
            }
        };

        if first_frame.is_none() {
            first_frame = Some(start.elapsed());
            first_ts = frame.timestamp_ns;
            println!();
            println!("first frame after {:?}", first_frame.unwrap());
            println!("  {}x{} format 0x{:x}", frame.width, frame.height, frame.format);
            println!("  layout {:?}", frame.layout());
            for (i, plane) in frame.planes.iter().enumerate() {
                println!(
                    "  plane {}: row_stride {} pixel_stride {} len {}",
                    i,
                    plane.row_stride,
                    plane.pixel_stride,
                    frame.plane_data(i).len()
                );
            }
            let padding = frame.planes[0].row_stride - frame.width;
            println!(
                "  luma row padding {} byte(s) -- a V4L2 capture device would advertise \
                 bytesperline {} for width {}",
                padding, frame.planes[0].row_stride, frame.width
            );
            println!();
        }

        last_ts = frame.timestamp_ns;
        let (digest, mean) = luma_digest(
            frame.plane_data(0),
            frame.width,
            frame.height,
            frame.planes[0].row_stride,
        );
        digests.insert(digest);
        luma_min = luma_min.min(mean);
        luma_max = luma_max.max(mean);
        received += 1;
        if received % 30 == 0 {
            println!(
                "  {} frames, luma mean {:.1}, {} distinct",
                received, mean, digests.len()
            );
        }
    }

    let wall = start.elapsed();
    let sensor_span_ns = (last_ts - first_ts) as f64;
    println!();
    println!("frames                {}", received);
    println!("wall clock            {:?}", wall);
    println!("wall fps              {:.2}", received as f64 / wall.as_secs_f64());
    if sensor_span_ns > 0.0 && received > 1 {
        println!(
            "sensor timestamp fps  {:.2}",
            (received - 1) as f64 / (sensor_span_ns / 1e9)
        );
    }
    println!("listener callbacks    {}", camera.frames_signalled());
    println!("distinct luma digests {} of {}", digests.len(), received);
    println!("luma mean range       {:.1}..{:.1}", luma_min, luma_max);
    let stats = camera.results();
    println!(
        "capture results       {} completed, {} failed, {} buffers lost",
        stats.completed, stats.failed, stats.buffers_lost
    );
    {
        let seen = seen.lock().unwrap_or_else(|e| e.into_inner());
        println!(
            "result transitions    {} (from {} results read in the callback)",
            seen.log.len(),
            seen.results
        );
        for line in &seen.log {
            println!("  {}", line);
        }
    }

    // The two ways this can pass without the camera working: every frame identical (a frozen or
    // never-filled buffer), or every pixel zero (a black frame that still has the right shape).
    if digests.len() <= 1 {
        return Err("every frame was byte-identical: not a live stream".to_owned());
    }
    if luma_max <= 1.0 {
        return Err("every frame was black: pixels never arrived".to_owned());
    }
    if camera.frames_signalled() == 0 {
        println!();
        println!("WARNING: the image listener never fired; frames came from the polling fallback");
    }

    if let Some(path) = &args.dump {
        let frame = camera
            .next_frame(Duration::from_millis(2000))
            .map_err(|e| e.to_string())?
            .ok_or("no frame to dump")?;
        dump_frame(path, &frame)?;
        println!();
        println!(
            "dumped one {}x{} frame to {} ({:?}, {} bytes)",
            frame.width,
            frame.height,
            path,
            frame.layout(),
            frame.width as usize * frame.height as usize * 3 / 2
        );
    }
    Ok(())
}

fn run() -> Result<(), String> {
    let args = parse_args()?;
    if let Some(uid) = args.uid {
        drop_to_uid(uid)?;
    }
    // SAFETY: getuid/geteuid take no arguments and cannot fail.
    println!("running as uid {}", unsafe { libc::getuid() });
    match args.command.as_str() {
        "list" => cmd_list(),
        "controls" => cmd_controls(&args),
        "capture" => cmd_capture(&args),
        other => Err(format!(
            "unknown command {:?}; want list, controls or capture",
            other
        )),
    }
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("camera_probe: {}", e);
            ExitCode::FAILURE
        }
    }
}
