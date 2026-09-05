// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

//! Exercises `android_codec` on a device, so the Rust-to-NDK path and the phone's real codec
//! capabilities are known before a virtio-media decoder or encoder device is built on top.
//!
//! It answers: does `libmediandk` load and which API 36 symbols are missing, what does the codec
//! store say each codec can do (sizes, alignment, rates, bitrates, profiles, features), does a
//! decoder deliver frames through the async callbacks and in what layout (`image-data`), does
//! flush-then-start resume, does an encoder accept padded NV12 and what does it emit, and does
//! encode -> decode reproduce the input (luma PSNR).
//!
//! ```text
//! codec_probe list      [--uid N] [--json] [--all] [--no-profiles]
//! codec_probe decode    --mime M --input FILE [--uid N] [--codec NAME] [--size WxH] [--fps F]
//!                       [--frames N] [--dump out.nv12] [--no-image-data] [--csd config|format|inband]
//!                       [--no-seek] [--timeout S]
//! codec_probe encode    --mime M --size WxH --frames N [--uid N] [--codec NAME] [--bitrate B]
//!                       [--fps F] [--gop S] [--color-format flexible|semiplanar|planar|INT]
//!                       [--bitrate-mode cq|vbr|cbr|cbr-fd] [--profile P] [--level L]
//!                       [--dump out.h264|out.h265|out.ivf] [--timeout S]
//! codec_probe roundtrip --mime M --size WxH --frames N [--uid N] [--codec ENCODER]
//!                       [--decoder NAME] [--bitrate B] [--fps F] [--min-psnr DB] [--csd ...]
//! ```
//!
//! `--uid` drops to that uid before the first NDK call, the way `camera_probe` does: the codec
//! devices run in the app-uid helper (design 7.4), and this is the run that must match.

use std::collections::HashMap;
use std::collections::HashSet;
use std::env;
use std::fs;
use std::io::Write;
use std::process::ExitCode;
use std::time::Duration;
use std::time::Instant;

use android_codec::bitstream;
use android_codec::bitstream::NalCodec;
use android_codec::color_format_name;
use android_codec::decoder_format;
use android_codec::synth;
use android_codec::synth::PaddedLayout;
use android_codec::BufferInfo;
use android_codec::Codec;
use android_codec::CodecEvent;
use android_codec::CodecInfo;
use android_codec::EncoderConfig;
use android_codec::InputLayout;
use android_codec::ListOptions;
use android_codec::MediaImage2;
use android_codec::OutputLayout;
use android_codec::BUFFER_FLAG_CODEC_CONFIG;
use android_codec::BUFFER_FLAG_END_OF_STREAM;
use android_codec::BUFFER_FLAG_KEY_FRAME;
use android_codec::COLOR_FORMAT_YUV420_FLEXIBLE;
use android_codec::COLOR_FORMAT_YUV420_PACKED_PLANAR;
use android_codec::COLOR_FORMAT_YUV420_PLANAR;
use android_codec::COLOR_FORMAT_YUV420_SEMI_PLANAR;

struct Args {
    command: String,
    values: HashMap<String, String>,
    switches: HashSet<String>,
}

const SWITCHES: &[&str] = &[
    "--json",
    "--all",
    "--no-profiles",
    "--no-image-data",
    "--no-seek",
];

impl Args {
    fn parse() -> Result<Args, String> {
        let mut argv = env::args().skip(1);
        let command = argv.next().unwrap_or_else(|| "list".to_owned());
        let mut args = Args {
            command,
            values: HashMap::new(),
            switches: HashSet::new(),
        };
        while let Some(flag) = argv.next() {
            if !flag.starts_with("--") {
                return Err(format!("unexpected argument {flag:?}"));
            }
            if SWITCHES.contains(&flag.as_str()) {
                args.switches.insert(flag);
                continue;
            }
            let value = argv.next().ok_or_else(|| format!("{flag} needs a value"))?;
            args.values.insert(flag, value);
        }
        Ok(args)
    }

    fn has(&self, switch: &str) -> bool {
        self.switches.contains(switch)
    }

    fn get(&self, flag: &str) -> Option<&str> {
        self.values.get(flag).map(String::as_str)
    }

    fn required(&self, flag: &str) -> Result<&str, String> {
        self.get(flag).ok_or_else(|| format!("{flag} is required"))
    }

    fn parse_or<T: std::str::FromStr>(&self, flag: &str, default: T) -> Result<T, String>
    where
        T::Err: std::fmt::Display,
    {
        match self.get(flag) {
            Some(v) => v.parse().map_err(|e| format!("{flag}: {e}")),
            None => Ok(default),
        }
    }

    fn parse_opt<T: std::str::FromStr>(&self, flag: &str) -> Result<Option<T>, String>
    where
        T::Err: std::fmt::Display,
    {
        match self.get(flag) {
            Some(v) => v.parse().map(Some).map_err(|e| format!("{flag}: {e}")),
            None => Ok(None),
        }
    }

    fn size(&self, default: (i32, i32)) -> Result<(i32, i32), String> {
        match self.get("--size") {
            Some(v) => {
                let (w, h) = v.split_once('x').ok_or("--size wants WxH")?;
                Ok((
                    w.parse().map_err(|e| format!("--size width: {e}"))?,
                    h.parse().map_err(|e| format!("--size height: {e}"))?,
                ))
            }
            None => Ok(default),
        }
    }

    fn timeout(&self) -> Result<Duration, String> {
        Ok(Duration::from_secs_f64(self.parse_or("--timeout", 10.0)?))
    }
}

/// Drop to `uid` before the first NDK call. `setresuid`, not `setuid`, for the reason
/// `camera_probe` gives: a real-uid change through `setresuid` updates the per-uid NPROC
/// accounting; the `setuid` path does not, and the app then drifts until it can no longer fork.
fn drop_to_uid(uid: u32) -> Result<(), String> {
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
        return Err(format!("setresuid({uid}) left uid at {now}"));
    }
    Ok(())
}

fn range<T: std::fmt::Display>(r: (T, T)) -> String {
    format!("{}..{}", r.0, r.1)
}

fn print_codec(c: &CodecInfo) {
    println!();
    println!(
        "  {}  {}  {:?}  {:?}{}{}",
        c.name,
        c.mime,
        c.kind,
        c.codec_type,
        if c.is_vendor { "  vendor" } else { "" },
        match c.max_instances {
            Some(n) => format!("  max instances {n}"),
            None => String::new(),
        }
    );
    if let Some(v) = &c.video {
        println!(
            "    size      {} x {}  align {}x{}",
            range(v.widths),
            range(v.heights),
            v.width_alignment,
            v.height_alignment
        );
        if v.widths_at_max_height.is_some() || v.heights_at_max_width.is_some() {
            println!(
                "              widths at h={}: {}   heights at w={}: {}",
                v.heights.1,
                v.widths_at_max_height
                    .map(range)
                    .unwrap_or_else(|| "-".into()),
                v.widths.1,
                v.heights_at_max_width
                    .map(range)
                    .unwrap_or_else(|| "-".into()),
            );
        }
        println!(
            "    fps       {}  bitrate {}  performance points {}",
            range(v.frame_rates),
            range(v.bitrates),
            v.performance_points
        );
        for s in &v.sizes {
            let fmt_f = |r: Option<(f64, f64)>| match r {
                Some((lo, hi)) => format!("{lo:.0}..{hi:.0}"),
                None => "-".to_owned(),
            };
            let fmt_i = |r: Option<i32>| match r {
                Some(v) => v.to_string(),
                None => "-".to_owned(),
            };
            println!(
                "    {:>9}  {}  fps {}  achievable {}  rate<= {}  perf-point<= {}",
                format!("{}x{}", s.width, s.height),
                if s.supported { "ok " } else { "no " },
                fmt_f(s.frame_rates),
                fmt_f(s.achievable),
                fmt_i(s.max_rate_supported),
                fmt_i(s.max_rate_covered),
            );
        }
    } else {
        println!("    (no video capabilities)");
    }
    if !c.features.is_empty() {
        let on: Vec<String> = c
            .features
            .iter()
            .filter(|f| f.supported || f.required)
            .map(|f| {
                if f.required {
                    format!("{}(required)", f.feature)
                } else {
                    f.feature.clone()
                }
            })
            .collect();
        println!(
            "    features  {}",
            if on.is_empty() {
                "(none)".to_owned()
            } else {
                on.join(", ")
            }
        );
    }
    if !c.profiles.is_empty() {
        let mut parts = Vec::new();
        for p in &c.profiles {
            let levels = if p.levels.is_empty() {
                "(no level)".to_owned()
            } else if p.levels.len() > 3 {
                format!(
                    "{}..{}",
                    p.levels[0].level_name,
                    p.levels[p.levels.len() - 1].level_name
                )
            } else {
                p.levels
                    .iter()
                    .map(|l| l.level_name.clone())
                    .collect::<Vec<_>>()
                    .join("/")
            };
            parts.push(format!("{} {}", p.profile_name, levels));
        }
        println!("    profiles  {}", parts.join("; "));
    }
    if let Some(e) = &c.encoder {
        let modes: Vec<&str> = e
            .bitrate_modes
            .iter()
            .filter(|(_, ok)| *ok)
            .map(|(n, _)| n.as_str())
            .collect();
        println!(
            "    encoder   quality {}  complexity {}  bitrate modes {}",
            e.quality.map(range).unwrap_or_else(|| "-".into()),
            e.complexity.map(range).unwrap_or_else(|| "-".into()),
            if modes.is_empty() {
                "(none reported)".to_owned()
            } else {
                modes.join(",")
            }
        );
    }
}

fn cmd_list(args: &Args) -> Result<(), String> {
    let opts = ListOptions {
        include_non_video: args.has("--all"),
        probe_profiles: !args.has("--no-profiles"),
        ..ListOptions::default()
    };
    let started = Instant::now();
    let list = android_codec::list_codecs(&opts).map_err(|e| e.to_string())?;
    let took = started.elapsed();
    if args.has("--json") {
        println!(
            "{}",
            serde_json::to_string_pretty(&list).map_err(|e| e.to_string())?
        );
        return Ok(());
    }
    println!(
        "source: {}  ({} codecs in {:?})",
        list.source,
        list.codecs.len(),
        took
    );
    println!(
        "missing optional symbols: {}",
        if list.missing_symbols.is_empty() {
            "none".to_owned()
        } else {
            list.missing_symbols.join(", ")
        }
    );
    println!("media types: {}", list.media_types.len());
    for t in &list.media_types {
        if opts.include_non_video || t.mime.starts_with("video/") {
            println!(
                "  {:<24} {}{}",
                t.mime,
                if t.decoder { "decoder " } else { "" },
                if t.encoder { "encoder" } else { "" }
            );
        }
    }
    for c in &list.codecs {
        print_codec(c);
    }
    Ok(())
}

/// One input buffer's worth of bitstream.
struct Au {
    data: Vec<u8>,
    pts_us: u64,
    keyframe: bool,
}

/// Where the parameter sets go.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CsdMode {
    /// A `BUFFER_FLAG_CODEC_CONFIG` input buffer of their own, ahead of the first picture.
    Config,
    /// `csd-0` / `csd-1` in the configure format.
    Format,
    /// Left in the first access unit, in front of the IDR.
    Inband,
}

impl CsdMode {
    fn parse(s: Option<&str>) -> Result<CsdMode, String> {
        Ok(match s.unwrap_or("config") {
            "config" => CsdMode::Config,
            "format" => CsdMode::Format,
            "inband" => CsdMode::Inband,
            other => return Err(format!("--csd: {other:?} is not config, format or inband")),
        })
    }
}

/// A parsed input stream: access units, and the parameter sets taken out of them (empty when
/// `mode` leaves them in-band or the container has none).
struct Stream {
    aus: Vec<Au>,
    csd0: Vec<u8>,
    csd1: Vec<u8>,
    container: &'static str,
    size_hint: Option<(i32, i32)>,
}

fn load_stream(data: &[u8], mime: &str, fps: f64, mode: CsdMode) -> Result<Stream, String> {
    if data.starts_with(b"DKIF") {
        let (header, frames) = bitstream::read_ivf(data).map_err(|e| e.to_string())?;
        if let Some(m) = header.mime() {
            if m != mime {
                return Err(format!("IVF fourcc says {m}, --mime says {mime}"));
            }
        }
        let aus = frames
            .iter()
            .map(|f| Au {
                data: data[f.range.clone()].to_vec(),
                pts_us: header.timestamp_us(f.timestamp),
                keyframe: bitstream::ivf_keyframe(&header.fourcc, &data[f.range.clone()])
                    .unwrap_or(false),
            })
            .collect();
        return Ok(Stream {
            aus,
            csd0: Vec::new(),
            csd1: Vec::new(),
            container: "IVF",
            size_hint: Some((header.width as i32, header.height as i32)),
        });
    }
    let codec = NalCodec::for_mime(mime)
        .ok_or_else(|| format!("{mime}: not an IVF file and not an Annex-B mime"))?;
    let units = bitstream::annexb_access_units(data, codec).map_err(|e| e.to_string())?;
    let mut aus: Vec<Au> = Vec::new();
    let mut csd0 = Vec::new();
    let mut csd1 = Vec::new();
    let mut carry: Vec<u8> = Vec::new();
    let frame_us = 1_000_000.0 / fps;
    for u in &units {
        let bytes = &data[u.range.clone()];
        // Only the first parameter sets are lifted out (to the config buffer or the format);
        // later repeats stay in-band, which is what a guest's stream looks like.
        let rest = if mode == CsdMode::Inband || !csd0.is_empty() {
            bytes.to_vec()
        } else {
            let (unit_csd0, unit_csd1, rest) = bitstream::split_parameter_sets(bytes, codec);
            csd0 = unit_csd0;
            csd1 = unit_csd1;
            rest
        };
        if !u.has_picture {
            // Parameter sets or SEI alone: they belong with the next picture.
            carry.extend_from_slice(&rest);
            continue;
        }
        let mut au = std::mem::take(&mut carry);
        au.extend_from_slice(&rest);
        let k = aus.len() as f64;
        aus.push(Au {
            data: au,
            pts_us: (k * frame_us) as u64,
            keyframe: u.is_keyframe,
        });
    }
    Ok(Stream {
        aus,
        csd0,
        csd1,
        container: "Annex-B",
        size_hint: None,
    })
}

/// What one decoded output looked like.
struct FrameOut<'a> {
    nth: u32,
    info: BufferInfo,
    /// Tight NV12, when `image-data` was read and repacked.
    nv12: Option<&'a [u8]>,
    image: Option<&'a MediaImage2>,
    width: usize,
    height: usize,
}

struct DecodeOutcome {
    outputs: u32,
    first_output: Option<Duration>,
    wall: Duration,
    eos: bool,
    format_changes: u32,
    layout: Option<OutputLayout>,
    seek: Option<SeekOutcome>,
    callbacks: u64,
}

struct SeekOutcome {
    at_output: u32,
    resumed_after: Option<Duration>,
    outputs_after: u32,
}

struct DecodeOpts {
    read_image_data: bool,
    max_frames: Option<u32>,
    seek_at: Option<u32>,
    csd_mode: CsdMode,
    timeout: Duration,
    verbose: bool,
}

/// Feed `stream` to `codec` and hand every output to `on_frame`. The whole async contract is in
/// here: inputs only on `InputAvailable`, outputs only on `OutputAvailable`, EOS as an empty
/// buffer after the last access unit, and (once) a flush-then-start in the middle.
fn run_decode(
    codec: &mut Codec,
    stream: &Stream,
    opts: &DecodeOpts,
    on_frame: &mut dyn FnMut(&FrameOut<'_>) -> Result<(), String>,
) -> Result<DecodeOutcome, String> {
    let config_buffer: Vec<u8> = if opts.csd_mode == CsdMode::Config {
        let mut v = stream.csd0.clone();
        v.extend_from_slice(&stream.csd1);
        v
    } else {
        Vec::new()
    };
    let started = Instant::now();
    let mut outcome = DecodeOutcome {
        outputs: 0,
        first_output: None,
        wall: Duration::ZERO,
        eos: false,
        format_changes: 0,
        layout: None,
        seek: None,
        callbacks: 0,
    };
    let mut next_au = 0usize;
    let mut config_sent = config_buffer.is_empty();
    let mut eos_sent = false;
    let mut nv12 = Vec::new();
    let mut offset_verdict_printed = false;
    let mut last_progress = Instant::now();
    let mut seek_started: Option<Instant> = None;

    codec.start().map_err(|e| e.to_string())?;

    'outer: loop {
        if last_progress.elapsed() > opts.timeout {
            return Err(format!(
                "decoder stalled: no callback for {:?} ({} outputs, {} of {} access units queued)",
                opts.timeout,
                outcome.outputs,
                next_au,
                stream.aus.len()
            ));
        }
        let events = codec
            .wait_events(Duration::from_millis(500))
            .map_err(|e| e.to_string())?;
        if !events.is_empty() {
            last_progress = Instant::now();
        }
        for ev in events {
            match ev {
                CodecEvent::InputAvailable(index) => {
                    if !config_sent {
                        match codec.queue_input(index, &config_buffer, 0, BUFFER_FLAG_CODEC_CONFIG) {
                            Ok(()) => config_sent = true,
                            Err(e) => println!("  config buffer on index {index} refused: {e} (stale index after flush?)"),
                        }
                    } else if next_au < stream.aus.len() {
                        let au = &stream.aus[next_au];
                        match codec.queue_input(index, &au.data, au.pts_us, 0) {
                            Ok(()) => next_au += 1,
                            Err(e) => println!(
                                "  input index {index} refused: {e} (stale index after flush?)"
                            ),
                        }
                    } else if !eos_sent {
                        codec.queue_eos(index, 0).map_err(|e| e.to_string())?;
                        eos_sent = true;
                    }
                }
                CodecEvent::OutputAvailable { index, info } => {
                    let is_eos = info.flags & BUFFER_FLAG_END_OF_STREAM != 0;
                    let is_config = info.flags & BUFFER_FLAG_CODEC_CONFIG != 0;
                    if info.size > 0 && !is_config {
                        let now = started.elapsed();
                        if outcome.first_output.is_none() {
                            outcome.first_output = Some(now);
                        }
                        if let (Some(s), Some(seek)) = (seek_started, outcome.seek.as_mut()) {
                            if seek.resumed_after.is_none() {
                                seek.resumed_after = Some(s.elapsed());
                            }
                            seek.outputs_after += 1;
                        }
                        let buf = codec.output_buffer(index).map_err(|e| e.to_string())?;
                        let used = &buf[..(info.size as usize).min(buf.len())];
                        let mut frame = FrameOut {
                            nth: outcome.outputs,
                            info,
                            nv12: None,
                            image: None,
                            width: 0,
                            height: 0,
                        };
                        let mut layout: Option<OutputLayout> = None;
                        let mut chosen: Option<MediaImage2> = None;
                        if opts.read_image_data {
                            let f = codec.buffer_format(index).map_err(|e| e.to_string())?;
                            let l = OutputLayout::from_format(&f);
                            if outcome.layout.as_ref() != Some(&l) {
                                if opts.verbose {
                                    print_output_layout(&l, buf.len(), &f.to_string());
                                }
                                outcome.layout = Some(l.clone());
                            }
                            if let Some(image) = l.image {
                                let (use_image, verdict) = pick_image(&image, buf.len());
                                if !offset_verdict_printed && opts.verbose {
                                    println!(
                                        "  mPlane[0].mOffset = {}: {}",
                                        image.planes[0].offset, verdict
                                    );
                                    offset_verdict_printed = true;
                                }
                                android_codec::tight_nv12(&use_image, buf, &mut nv12)
                                    .map_err(|e| format!("output {}: {e}", outcome.outputs))?;
                                frame.width = use_image.width as usize;
                                frame.height = use_image.height as usize;
                                chosen = Some(use_image);
                            } else if let Some(err) = &l.image_error {
                                return Err(format!("image-data: {err}"));
                            } else {
                                // No image-data: the stride/slice-height idiom, assuming NV12.
                                let w = l.width.unwrap_or(0).max(0) as usize;
                                let h = l.height.unwrap_or(0).max(0) as usize;
                                let stride =
                                    l.effective_stride().unwrap_or(w as i32).max(0) as usize;
                                let slice =
                                    l.effective_slice_height().unwrap_or(h as i32).max(0) as usize;
                                let assumed = MediaImage2::semiplanar(
                                    w as u32,
                                    h as u32,
                                    stride as u32,
                                    slice as u32,
                                );
                                android_codec::tight_nv12(&assumed, used, &mut nv12)
                                    .map_err(|e| format!("output {} (no image-data, assumed NV12 {stride}x{slice}): {e}", outcome.outputs))?;
                                frame.width = w;
                                frame.height = h;
                                chosen = Some(assumed);
                            }
                            layout = Some(l);
                        }
                        frame.nv12 = chosen.as_ref().map(|_| nv12.as_slice());
                        frame.image = chosen.as_ref();
                        let _ = layout;
                        on_frame(&frame)?;
                        outcome.outputs += 1;
                    }
                    codec.release_output(index).map_err(|e| e.to_string())?;
                    if is_eos {
                        outcome.eos = true;
                        break 'outer;
                    }
                    if let Some(max) = opts.max_frames {
                        if outcome.outputs >= max {
                            break 'outer;
                        }
                    }
                    if let Some(at) = opts.seek_at {
                        if outcome.seek.is_none() && outcome.outputs >= at {
                            if opts.verbose {
                                println!();
                                println!(
                                    "seek: flush + start after {} outputs ({} access units queued), restarting from access unit 0",
                                    outcome.outputs, next_au
                                );
                            }
                            codec.flush_and_restart().map_err(|e| e.to_string())?;
                            next_au = 0;
                            config_sent = config_buffer.is_empty();
                            eos_sent = false;
                            seek_started = Some(Instant::now());
                            outcome.seek = Some(SeekOutcome {
                                at_output: outcome.outputs,
                                resumed_after: None,
                                outputs_after: 0,
                            });
                        }
                    }
                }
                CodecEvent::FormatChanged(format) => {
                    outcome.format_changes += 1;
                    if opts.verbose {
                        println!();
                        println!(
                            "format changed ({}): {}",
                            outcome.format_changes,
                            format
                                .as_ref()
                                .map(|f| f.to_string())
                                .unwrap_or_else(|| "(null)".into())
                        );
                        if let Some(f) = &format {
                            print_output_layout(&OutputLayout::from_format(f), 0, "");
                        }
                    }
                }
                CodecEvent::Error {
                    status,
                    action_code,
                    detail,
                } => {
                    return Err(format!(
                        "codec error {} ({}), action {}: {}",
                        status,
                        android_codec::media_status_name(status),
                        action_code,
                        detail
                    ));
                }
            }
        }
    }
    outcome.wall = started.elapsed();
    outcome.callbacks = codec.callbacks_fired();
    codec.stop().map_err(|e| e.to_string())?;
    Ok(outcome)
}

/// Survey open question 1: does `getOutputBuffer`'s pointer already include `mPlane[0].mOffset`?
/// Decide from what fits in the buffer, and say which way the evidence points.
fn pick_image(image: &MediaImage2, buf_len: usize) -> (MediaImage2, &'static str) {
    if image.planes[0].offset == 0 {
        return (
            *image,
            "zero, so the pointer question does not arise on this stream",
        );
    }
    let raw_fits = image.required_size() <= buf_len;
    let rebased = image.rebased();
    let rebased_fits = rebased.required_size() <= buf_len;
    match (raw_fits, rebased_fits) {
        (false, true) => (
            rebased,
            "raw offsets overrun the buffer, rebased ones fit: the pointer ALREADY includes mPlane[0].mOffset -- index by (mOffset - mPlane[0].mOffset)",
        ),
        (true, _) => (
            *image,
            "raw offsets fit (and so do rebased ones): undetermined by bounds alone -- compare the two luma digests against the dump",
        ),
        (false, false) => (
            *image,
            "neither interpretation fits the buffer: the layout is not what image-data says",
        ),
    }
}

fn print_output_layout(l: &OutputLayout, buf_len: usize, raw: &str) {
    println!();
    println!(
        "output format: {}x{}  color-format {}  stride {}  slice-height {}  crop {:?}  display {:?}  rotation {:?}",
        l.width.unwrap_or(-1),
        l.height.unwrap_or(-1),
        l.color_format.map(color_format_name).unwrap_or_else(|| "-".into()),
        l.stride.map(|v| v.to_string()).unwrap_or_else(|| "-(=width)".into()),
        l.slice_height.map(|v| v.to_string()).unwrap_or_else(|| "-(=height)".into()),
        l.crop,
        l.display,
        l.rotation,
    );
    println!(
        "  color range/standard/transfer {:?}   image-data {}",
        l.color,
        match l.image_data_len {
            Some(n) => format!("{n} bytes"),
            None => "absent".to_owned(),
        }
    );
    if let Some(img) = &l.image {
        println!(
            "  MediaImage2: type {} planes {} {}x{} bit depth {}/{}",
            img.image_type,
            img.num_planes,
            img.width,
            img.height,
            img.bit_depth,
            img.bit_depth_allocated
        );
        for (i, name) in ["Y", "U", "V", "A"].iter().enumerate() {
            if i < img.num_planes as usize {
                let p = &img.planes[i];
                println!(
                    "    {name}: offset {:>8} colInc {:>2} rowInc {:>6} subsampling {}x{}",
                    p.offset, p.col_inc, p.row_inc, p.horiz_subsampling, p.vert_subsampling
                );
            }
        }
        println!(
            "    chroma layout {:?}; required {} bytes (rebased {}); buffer {} bytes",
            img.chroma_layout(),
            img.required_size(),
            img.rebased().required_size(),
            buf_len
        );
    }
    if let Some(e) = &l.image_error {
        println!("  image-data did not parse: {e}");
    }
    if !raw.is_empty() {
        println!("  raw: {raw}");
    }
}

fn cmd_decode(args: &Args) -> Result<(), String> {
    let mime = args.required("--mime")?;
    let input = args.required("--input")?;
    let fps: f64 = args.parse_or("--fps", 30.0)?;
    let csd_mode = CsdMode::parse(args.get("--csd"))?;
    let data = fs::read(input).map_err(|e| format!("{input}: {e}"))?;
    let stream = load_stream(&data, mime, fps, csd_mode)?;
    let (w, h) = args.size(stream.size_hint.unwrap_or((1280, 720)))?;
    let total = stream.aus.len();
    let max_frames: Option<u32> = args.parse_opt("--frames")?;
    println!(
        "input {input}: {} bytes, {} {} access units, {} keyframes, csd-0 {} bytes csd-1 {} bytes -> {:?}; size hint {}x{}{}",
        data.len(),
        total,
        stream.container,
        stream.aus.iter().filter(|a| a.keyframe).count(),
        stream.csd0.len(),
        stream.csd1.len(),
        csd_mode,
        w,
        h,
        if stream.size_hint.is_none() && args.get("--size").is_none() {
            " (default; pass --size, the output format reports the real one)"
        } else {
            ""
        }
    );
    if total == 0 {
        return Err("no access units with pictures".to_owned());
    }

    let mut codec = match args.get("--codec") {
        Some(name) => Codec::create_by_name(name),
        None => Codec::create_decoder(mime),
    }
    .map_err(|e| e.to_string())?;
    println!("decoder {} ({mime})", codec.name());
    let (csd0, csd1) = if csd_mode == CsdMode::Format {
        (stream.csd0.as_slice(), stream.csd1.as_slice())
    } else {
        (&[][..], &[][..])
    };
    let format = decoder_format(mime, w, h, csd0, csd1).map_err(|e| e.to_string())?;
    codec.configure(&format, false).map_err(|e| e.to_string())?;
    println!("configured: {}", format);
    if let Ok(f) = codec.output_format() {
        println!("output format before start: {f}");
    }

    let mut dump = match args.get("--dump") {
        Some(path) => Some(fs::File::create(path).map_err(|e| format!("{path}: {e}"))?),
        None => None,
    };
    // The seek happens halfway through the outputs asked for (or halfway through the stream);
    // the restart plays from access unit 0 again, and --frames caps the outputs in total.
    let seek_at = if args.has("--no-seek") || total < 4 {
        None
    } else {
        Some(max_frames.unwrap_or(total as u32).min(total as u32) / 2)
    };
    let opts = DecodeOpts {
        read_image_data: !args.has("--no-image-data"),
        max_frames,
        seek_at,
        csd_mode,
        timeout: args.timeout()?,
        verbose: true,
    };
    let mut digests = Vec::new();
    let mut first_ts = Instant::now();
    let mut on_frame = |f: &FrameOut<'_>| -> Result<(), String> {
        if f.nth == 0 {
            first_ts = Instant::now();
        }
        if let Some(nv12) = f.nv12 {
            let (digest, mean) = synth::luma_digest(&nv12[..f.width * f.height], f.width, f.height);
            digests.push(digest);
            if f.nth < 5 {
                println!(
                    "frame {:>3}  pts {:>9} us  {} bytes  flags 0x{:x}  {}x{}  luma digest {:016x} mean {:.1}",
                    f.nth, f.info.presentation_time_us, f.info.size, f.info.flags, f.width, f.height, digest, mean
                );
            }
            if let Some(file) = dump.as_mut() {
                file.write_all(nv12).map_err(|e| e.to_string())?;
            }
        } else if f.nth < 5 {
            println!(
                "frame {:>3}  pts {:>9} us  {} bytes  flags 0x{:x}",
                f.nth, f.info.presentation_time_us, f.info.size, f.info.flags
            );
        }
        Ok(())
    };
    let outcome = run_decode(&mut codec, &stream, &opts, &mut on_frame)?;
    let _ = first_ts;

    println!();
    println!("outputs               {}", outcome.outputs);
    println!(
        "first output after    {}",
        outcome
            .first_output
            .map(|d| format!("{d:?}"))
            .unwrap_or_else(|| "never".into())
    );
    println!("wall clock            {:?}", outcome.wall);
    if let Some(first) = outcome.first_output {
        let span = outcome.wall.saturating_sub(first).as_secs_f64();
        if span > 0.0 && outcome.outputs > 1 {
            println!(
                "decode fps            {:.1} (outputs after the first / time since it)",
                (outcome.outputs - 1) as f64 / span
            );
        }
    }
    println!("format changes        {}", outcome.format_changes);
    println!("EOS seen              {}", outcome.eos);
    println!("callbacks fired       {}", outcome.callbacks);
    let distinct: HashSet<u64> = digests.iter().copied().collect();
    if !digests.is_empty() {
        println!(
            "distinct luma digests {} of {}",
            distinct.len(),
            digests.len()
        );
    }
    if let Some(s) = &outcome.seek {
        println!(
            "seek                  at output {}: resumed after {}, {} outputs after",
            s.at_output,
            s.resumed_after
                .map(|d| format!("{d:?}"))
                .unwrap_or_else(|| "NEVER".into()),
            s.outputs_after
        );
        if s.outputs_after == 0 {
            return Err("no output after flush + start".to_owned());
        }
    }
    if let Some(path) = args.get("--dump") {
        println!(
            "dumped {} tight NV12 frames to {path} (ffplay -f rawvideo -pixel_format nv12 -video_size {}x{} {path})",
            digests.len(),
            outcome.layout.as_ref().and_then(|l| l.image.map(|i| i.width)).unwrap_or(w as u32),
            outcome.layout.as_ref().and_then(|l| l.image.map(|i| i.height)).unwrap_or(h as u32),
        );
    }
    if outcome.outputs == 0 {
        return Err("decoder produced no output".to_owned());
    }
    if digests.len() > 1 && distinct.len() <= 1 {
        return Err("every frame had the same luma digest: frozen output".to_owned());
    }
    Ok(())
}

struct EncodedFrame {
    data: Vec<u8>,
    pts_us: u64,
    flags: u32,
}

struct EncodeOutcome {
    csd: Vec<u8>,
    frames: Vec<EncodedFrame>,
    first_output: Option<Duration>,
    wall: Duration,
    layout: PaddedLayout,
    input: InputLayout,
    output_format: Option<String>,
    callbacks: u64,
}

fn parse_color_format(s: Option<&str>) -> Result<i32, String> {
    Ok(match s.unwrap_or("flexible") {
        "flexible" => COLOR_FORMAT_YUV420_FLEXIBLE,
        "semiplanar" | "nv12" => COLOR_FORMAT_YUV420_SEMI_PLANAR,
        "planar" | "i420" => COLOR_FORMAT_YUV420_PLANAR,
        other => {
            let v = other.trim_start_matches("0x");
            i32::from_str_radix(v, 16)
                .or_else(|_| other.parse())
                .map_err(|_| format!("--color-format: {other:?}"))?
        }
    })
}

fn encoder_config(args: &Args, mime: &str, w: i32, h: i32) -> Result<EncoderConfig, String> {
    let bitrate_mode = match args.get("--bitrate-mode") {
        None => None,
        Some("cq") => Some(android_codec::BITRATE_MODE_CQ),
        Some("vbr") => Some(android_codec::BITRATE_MODE_VBR),
        Some("cbr") => Some(android_codec::BITRATE_MODE_CBR),
        Some("cbr-fd") => Some(android_codec::BITRATE_MODE_CBR_FD),
        Some(other) => return Err(format!("--bitrate-mode: {other:?}")),
    };
    Ok(EncoderConfig {
        mime: mime.to_owned(),
        width: w,
        height: h,
        color_format: parse_color_format(args.get("--color-format"))?,
        bitrate: args.parse_or("--bitrate", 4_000_000)?,
        frame_rate: args.parse_or("--fps", 30.0f32)?,
        i_frame_interval_s: args.parse_or("--gop", 1)?,
        bitrate_mode,
        profile: args.parse_opt("--profile")?,
        level: args.parse_opt("--level")?,
    })
}

fn run_encode(
    codec: &mut Codec,
    cfg: &EncoderConfig,
    frames: u32,
    timeout: Duration,
    verbose: bool,
) -> Result<EncodeOutcome, String> {
    let format = cfg.to_format().map_err(|e| e.to_string())?;
    codec.configure(&format, true).map_err(|e| e.to_string())?;
    if verbose {
        println!("configured: {format}");
    }
    let input_format = codec.input_format().map_err(|e| e.to_string())?;
    let input = InputLayout::from_format(&input_format, cfg.width);
    if verbose {
        println!("input format: {input_format}");
        println!(
            "input layout: color-format {}  stride {}{}  slice-height {}",
            input
                .color_format
                .map(color_format_name)
                .unwrap_or_else(|| "-".into()),
            input.stride,
            if input.stride_reported {
                ""
            } else {
                " (absent: =width)"
            },
            input
                .slice_height
                .map(|v| v.to_string())
                .unwrap_or_else(|| {
                    "ABSENT (chroma offset is not a stride multiple; design 7.3 rejects this codec)"
                        .into()
                }),
        );
    }
    let semiplanar = !matches!(
        input.color_format,
        Some(COLOR_FORMAT_YUV420_PLANAR) | Some(COLOR_FORMAT_YUV420_PACKED_PLANAR)
    );
    let layout = PaddedLayout {
        stride: input.stride.max(cfg.width) as usize,
        slice_height: input.slice_height.unwrap_or(cfg.height).max(cfg.height) as usize,
        semiplanar,
    };
    if verbose {
        println!(
            "filling {} frames as {} {}x{} padded ({} bytes each)",
            frames,
            if semiplanar { "NV12" } else { "I420" },
            layout.stride,
            layout.slice_height,
            layout.frame_size()
        );
    }

    let started = Instant::now();
    let mut outcome = EncodeOutcome {
        csd: Vec::new(),
        frames: Vec::new(),
        first_output: None,
        wall: Duration::ZERO,
        layout,
        input,
        output_format: None,
        callbacks: 0,
    };
    let frame_us = 1_000_000.0 / cfg.frame_rate as f64;
    let mut next = 0u32;
    let mut eos_sent = false;
    let mut last_progress = Instant::now();
    codec.start().map_err(|e| e.to_string())?;
    'outer: loop {
        if last_progress.elapsed() > timeout {
            return Err(format!(
                "encoder stalled: no callback for {timeout:?} ({} of {frames} frames queued, {} outputs)",
                next,
                outcome.frames.len()
            ));
        }
        let events = codec
            .wait_events(Duration::from_millis(500))
            .map_err(|e| e.to_string())?;
        if !events.is_empty() {
            last_progress = Instant::now();
        }
        for ev in events {
            match ev {
                CodecEvent::InputAvailable(index) => {
                    if next < frames {
                        let picture =
                            synth::synth_frame(next, cfg.width as usize, cfg.height as usize);
                        let pts = (next as f64 * frame_us) as u64;
                        codec
                            .queue_input_with(index, pts, 0, |buf| {
                                synth::pack_frame(&picture, &layout, buf)
                            })
                            .map_err(|e| format!("frame {next}: {e}"))?;
                        next += 1;
                    } else if !eos_sent {
                        codec
                            .queue_eos(index, (next as f64 * frame_us) as u64)
                            .map_err(|e| e.to_string())?;
                        eos_sent = true;
                    }
                }
                CodecEvent::OutputAvailable { index, info } => {
                    let buf = codec.output_buffer(index).map_err(|e| e.to_string())?;
                    let used = &buf[..(info.size as usize).min(buf.len())];
                    if info.flags & BUFFER_FLAG_CODEC_CONFIG != 0 {
                        outcome.csd.extend_from_slice(used);
                        if verbose {
                            println!(
                                "codec config: {} bytes  flags 0x{:x}",
                                used.len(),
                                info.flags
                            );
                        }
                    } else if !used.is_empty() {
                        if outcome.first_output.is_none() {
                            outcome.first_output = Some(started.elapsed());
                        }
                        if verbose && outcome.frames.len() < 5 {
                            println!(
                                "output {:>3}  pts {:>9} us  {:>7} bytes  flags 0x{:x}{}",
                                outcome.frames.len(),
                                info.presentation_time_us,
                                used.len(),
                                info.flags,
                                if info.flags & BUFFER_FLAG_KEY_FRAME != 0 {
                                    " KEY"
                                } else {
                                    ""
                                }
                            );
                        }
                        outcome.frames.push(EncodedFrame {
                            data: used.to_vec(),
                            pts_us: info.presentation_time_us.max(0) as u64,
                            flags: info.flags,
                        });
                    }
                    codec.release_output(index).map_err(|e| e.to_string())?;
                    if info.flags & BUFFER_FLAG_END_OF_STREAM != 0 {
                        break 'outer;
                    }
                }
                CodecEvent::FormatChanged(f) => {
                    let s = f.map(|f| f.to_string()).unwrap_or_else(|| "(null)".into());
                    if verbose {
                        println!("output format: {s}");
                    }
                    outcome.output_format = Some(s);
                }
                CodecEvent::Error {
                    status,
                    action_code,
                    detail,
                } => {
                    return Err(format!(
                        "codec error {} ({}), action {}: {}",
                        status,
                        android_codec::media_status_name(status),
                        action_code,
                        detail
                    ));
                }
            }
        }
    }
    outcome.wall = started.elapsed();
    outcome.callbacks = codec.callbacks_fired();
    codec.stop().map_err(|e| e.to_string())?;
    Ok(outcome)
}

fn print_encode_summary(o: &EncodeOutcome, frames: u32) {
    let bytes: usize = o.frames.iter().map(|f| f.data.len()).sum();
    let keys = o
        .frames
        .iter()
        .filter(|f| f.flags & BUFFER_FLAG_KEY_FRAME != 0)
        .count();
    println!();
    println!(
        "input layout used     {} stride {} slice-height {} ({} bytes/frame; codec said stride {} slice-height {})",
        if o.layout.semiplanar { "NV12" } else { "I420" },
        o.layout.stride,
        o.layout.slice_height,
        o.layout.frame_size(),
        if o.input.stride_reported {
            o.input.stride.to_string()
        } else {
            "absent".to_owned()
        },
        o.input
            .slice_height
            .map(|v| v.to_string())
            .unwrap_or_else(|| "absent".to_owned()),
    );
    println!("frames in / out       {} / {}", frames, o.frames.len());
    println!("codec config          {} bytes", o.csd.len());
    println!("bitstream             {} bytes, {} key frames", bytes, keys);
    println!(
        "first output after    {}",
        o.first_output
            .map(|d| format!("{d:?}"))
            .unwrap_or_else(|| "never".into())
    );
    println!("wall clock            {:?}", o.wall);
    if o.wall.as_secs_f64() > 0.0 {
        println!(
            "encode fps            {:.1}",
            o.frames.len() as f64 / o.wall.as_secs_f64()
        );
    }
    println!("callbacks fired       {}", o.callbacks);
}

fn cmd_encode(args: &Args) -> Result<(), String> {
    let mime = args.required("--mime")?;
    let (w, h) = args.size((1280, 720))?;
    let frames: u32 = args.parse_or("--frames", 60)?;
    let cfg = encoder_config(args, mime, w, h)?;
    let mut codec = match args.get("--codec") {
        Some(name) => Codec::create_by_name(name),
        None => Codec::create_encoder(mime),
    }
    .map_err(|e| e.to_string())?;
    println!(
        "encoder {} ({mime}) {}x{} @ {} fps, {} bps, gop {} s, color-format {}",
        codec.name(),
        w,
        h,
        cfg.frame_rate,
        cfg.bitrate,
        cfg.i_frame_interval_s,
        color_format_name(cfg.color_format)
    );
    let outcome = run_encode(&mut codec, &cfg, frames, args.timeout()?, true)?;
    print_encode_summary(&outcome, frames);
    if outcome.frames.is_empty() {
        return Err("encoder produced no frames".to_owned());
    }
    if let Some(path) = args.get("--dump") {
        let bytes = match bitstream::fourcc_for_mime(mime) {
            Some(fourcc) => {
                let header = bitstream::IvfHeader {
                    fourcc,
                    width: w as u16,
                    height: h as u16,
                    timebase_den: 1_000_000,
                    timebase_num: 1,
                    frame_count: 0,
                };
                let list: Vec<(u64, &[u8])> = outcome
                    .frames
                    .iter()
                    .map(|f| (f.pts_us, f.data.as_slice()))
                    .collect();
                bitstream::write_ivf(&header, &list)
            }
            None => {
                let mut out = outcome.csd.clone();
                for f in &outcome.frames {
                    out.extend_from_slice(&f.data);
                }
                out
            }
        };
        fs::write(path, &bytes).map_err(|e| format!("{path}: {e}"))?;
        println!("dumped {} bytes to {path}", bytes.len());
    }
    Ok(())
}

fn cmd_roundtrip(args: &Args) -> Result<(), String> {
    let mime = args.required("--mime")?;
    let (w, h) = args.size((1280, 720))?;
    let frames: u32 = args.parse_or("--frames", 60)?;
    let min_psnr: f64 = args.parse_or("--min-psnr", 30.0)?;
    let csd_mode = CsdMode::parse(args.get("--csd"))?;
    let cfg = encoder_config(args, mime, w, h)?;
    let timeout = args.timeout()?;

    let mut encoder = match args.get("--codec") {
        Some(name) => Codec::create_by_name(name),
        None => Codec::create_encoder(mime),
    }
    .map_err(|e| e.to_string())?;
    println!(
        "encoder {} ({mime}) {}x{} @ {} fps, {} bps",
        encoder.name(),
        w,
        h,
        cfg.frame_rate,
        cfg.bitrate
    );
    let encoded = run_encode(&mut encoder, &cfg, frames, timeout, true)?;
    print_encode_summary(&encoded, frames);
    drop(encoder);
    if encoded.frames.is_empty() {
        return Err("encoder produced no frames".to_owned());
    }

    // The encoder's output, as a stream: Annex-B codecs get their parameter sets the way
    // --csd says; the IVF codecs carry everything in-band.
    let stream = match NalCodec::for_mime(mime) {
        Some(nal) => {
            let (csd0, csd1, _) = bitstream::split_parameter_sets(&encoded.csd, nal);
            let mut aus: Vec<Au> = encoded
                .frames
                .iter()
                .map(|f| Au {
                    data: f.data.clone(),
                    pts_us: f.pts_us,
                    keyframe: f.flags & BUFFER_FLAG_KEY_FRAME != 0,
                })
                .collect();
            if csd_mode == CsdMode::Inband {
                if let Some(first) = aus.first_mut() {
                    let mut d = encoded.csd.clone();
                    d.extend_from_slice(&first.data);
                    first.data = d;
                }
            }
            Stream {
                aus,
                csd0: if csd_mode == CsdMode::Inband {
                    Vec::new()
                } else {
                    csd0
                },
                csd1: if csd_mode == CsdMode::Inband {
                    Vec::new()
                } else {
                    csd1
                },
                container: "encoder output",
                size_hint: Some((w, h)),
            }
        }
        None => Stream {
            aus: encoded
                .frames
                .iter()
                .map(|f| Au {
                    data: f.data.clone(),
                    pts_us: f.pts_us,
                    keyframe: f.flags & BUFFER_FLAG_KEY_FRAME != 0,
                })
                .collect(),
            csd0: Vec::new(),
            csd1: Vec::new(),
            container: "encoder output",
            size_hint: Some((w, h)),
        },
    };

    let mut decoder = match args.get("--decoder") {
        Some(name) => Codec::create_by_name(name),
        None => Codec::create_decoder(mime),
    }
    .map_err(|e| e.to_string())?;
    println!();
    println!(
        "decoder {} ({mime}), {} access units, csd {:?}",
        decoder.name(),
        stream.aus.len(),
        csd_mode
    );
    let (csd0, csd1) = if csd_mode == CsdMode::Format {
        (stream.csd0.as_slice(), stream.csd1.as_slice())
    } else {
        (&[][..], &[][..])
    };
    let format = decoder_format(mime, w, h, csd0, csd1).map_err(|e| e.to_string())?;
    decoder
        .configure(&format, false)
        .map_err(|e| e.to_string())?;

    let opts = DecodeOpts {
        read_image_data: true,
        max_frames: None,
        seek_at: None,
        csd_mode,
        timeout,
        verbose: true,
    };
    let frame_us = 1_000_000.0 / cfg.frame_rate as f64;
    let mut psnrs: Vec<(u32, f64)> = Vec::new();
    let mut mismatched_size = false;
    let mut on_frame = |f: &FrameOut<'_>| -> Result<(), String> {
        let nv12 = f.nv12.ok_or("no pixels to compare")?;
        let k = (f.info.presentation_time_us as f64 / frame_us)
            .round()
            .max(0.0) as u32;
        let reference = synth::synth_frame(k, w as usize, h as usize);
        let cw = f.width.min(w as usize);
        let ch = f.height.min(h as usize);
        if (f.width, f.height) != (w as usize, h as usize) {
            mismatched_size = true;
        }
        let mut a = Vec::with_capacity(cw * ch);
        let mut b = Vec::with_capacity(cw * ch);
        for row in 0..ch {
            a.extend_from_slice(&nv12[row * f.width..row * f.width + cw]);
            b.extend_from_slice(&reference.y[row * w as usize..row * w as usize + cw]);
        }
        let db = synth::psnr_luma(&a, &b);
        if f.nth < 5 || db < min_psnr {
            println!(
                "frame {:>3}  pts {:>9} us -> input frame {:>3}  luma PSNR {:.2} dB{}",
                f.nth,
                f.info.presentation_time_us,
                k,
                db,
                if db < min_psnr { "  LOW" } else { "" }
            );
        }
        psnrs.push((k, db));
        Ok(())
    };
    let outcome = run_decode(&mut decoder, &stream, &opts, &mut on_frame)?;

    println!();
    println!(
        "decoded               {} of {} encoded frames (EOS {})",
        outcome.outputs,
        encoded.frames.len(),
        outcome.eos
    );
    if mismatched_size {
        println!("NOTE: decoded size differs from {w}x{h}; PSNR over the overlap");
    }
    if psnrs.is_empty() {
        return Err("nothing decoded".to_owned());
    }
    let finite: Vec<f64> = psnrs.iter().map(|(_, d)| d.min(99.0)).collect();
    let min = finite.iter().cloned().fold(f64::INFINITY, f64::min);
    let mean = finite.iter().sum::<f64>() / finite.len() as f64;
    let mut seen: Vec<u32> = psnrs.iter().map(|(k, _)| *k).collect();
    seen.sort_unstable();
    seen.dedup();
    println!("luma PSNR             min {:.2} dB  mean {:.2} dB  over {} frames ({} distinct input frames matched)", min, mean, finite.len(), seen.len());
    if min < min_psnr {
        return Err(format!(
            "round trip PSNR {min:.2} dB is below {min_psnr} dB"
        ));
    }
    if seen.len() < psnrs.len() / 2 {
        return Err("output timestamps did not map back to distinct input frames".to_owned());
    }
    println!("round trip OK (>= {min_psnr} dB)");
    Ok(())
}

fn run() -> Result<(), String> {
    let args = Args::parse()?;
    if let Some(uid) = args.parse_opt::<u32>("--uid")? {
        drop_to_uid(uid)?;
    }
    // SAFETY: getuid takes no arguments and cannot fail.
    println!("running as uid {}", unsafe { libc::getuid() });
    android_codec::ensure_loaded().map_err(|e| e.to_string())?;
    let missing = android_codec::missing_symbols().map_err(|e| e.to_string())?;
    println!(
        "libmediandk loaded; {} optional (API 36) symbols missing{}",
        missing.len(),
        if missing.is_empty() {
            String::new()
        } else {
            format!(": {}", missing.join(", "))
        }
    );
    match args.command.as_str() {
        "list" => cmd_list(&args),
        "decode" => cmd_decode(&args),
        "encode" => cmd_encode(&args),
        "roundtrip" => cmd_roundtrip(&args),
        other => Err(format!(
            "unknown command {other:?}; want list, decode, encode or roundtrip"
        )),
    }
}

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(e) => {
            eprintln!("codec_probe: {e}");
            ExitCode::FAILURE
        }
    }
}
