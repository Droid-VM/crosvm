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
//! camera_probe capture --id 0 --size 1280x720 --frames 90 [--uid N] [--zoom R]
//!                      [--flash off|single|torch] [--af off|auto|continuous-video|continuous-picture]
//!                      [--fps MIN:MAX] [--dump PATH]
//! ```
//!
//! `--uid` drops to that uid before touching the camera, the way `snd_helper` does for AAudio:
//! `cameraserver` resolves the client package from the real uid, and uid 0 resolves to none.

use std::env;
use std::fs::File;
use std::io::Write;
use std::process::ExitCode;
use std::time::Duration;
use std::time::Instant;

use android_camera::AfMode;
use android_camera::Camera;
use android_camera::FlashMode;
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
            other => return Err(format!("unknown flag {:?}", other)),
        }
    }
    Ok(args)
}

/// Drop to `uid` before the first NDK call. Nothing here needs the capabilities we give up, and
/// `cameraserver` reads the real uid, not the effective one.
fn drop_to_uid(uid: u32) -> Result<(), String> {
    // SAFETY: setuid on the current process with no borrowed state; the result is checked.
    if unsafe { libc::setuid(uid) } != 0 {
        return Err(format!(
            "setuid({}) failed: {}",
            uid,
            std::io::Error::last_os_error()
        ));
    }
    // SAFETY: getuid cannot fail and takes no arguments.
    let now = unsafe { libc::getuid() };
    if now != uid {
        return Err(format!("setuid({}) left uid at {}", uid, now));
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
        println!("    YUV_420_888 sizes   {}", c.yuv_sizes.len());
        for (w, h) in c.yuv_sizes.iter().take(8) {
            println!("      {}x{}", w, h);
        }
        if c.yuv_sizes.len() > 8 {
            println!("      ... and {} more", c.yuv_sizes.len() - 8);
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

fn cmd_capture(args: &Args) -> Result<(), String> {
    // Depth 4: enough that acquiring one frame does not stall the camera, small enough that a leak
    // shows up immediately as a stall rather than as growing memory.
    let opened = Instant::now();
    let mut camera =
        Camera::open(&args.id, args.width, args.height, 4).map_err(|e| e.to_string())?;
    println!(
        "opened camera {} at {}x{} in {:?}",
        args.id, args.width, args.height, opened.elapsed()
    );

    if let Some((lo, hi)) = args.fps {
        camera.set_fps_range(lo, hi).map_err(|e| e.to_string())?;
        println!("fps range set to {}:{}", lo, hi);
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

    let start = Instant::now();
    let mut first_frame: Option<Duration> = None;
    let mut digests = std::collections::HashSet::new();
    let mut received = 0u32;
    let mut stalls = 0u32;
    let mut first_ts = 0i64;
    let mut last_ts = 0i64;
    let mut luma_min = f64::MAX;
    let mut luma_max = f64::MIN;

    while received < args.frames {
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
        "capture" => cmd_capture(&args),
        other => Err(format!("unknown command {:?}; want list or capture", other)),
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
