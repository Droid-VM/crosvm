// Copyright 2024 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

pub mod device_table;

#[cfg(feature = "libaaudio_stub")]
mod libaaudio_stub;

use std::os::raw::c_void;
use std::path::PathBuf;
use std::time::Duration;
use std::time::Instant;

use async_trait::async_trait;
use audio_streams::capture::AsyncCaptureBuffer;
use audio_streams::capture::AsyncCaptureBufferStream;
use audio_streams::capture::CaptureBuffer;
use audio_streams::capture::CaptureBufferStream;
use audio_streams::AsyncBufferCommit;
use audio_streams::AsyncPlaybackBuffer;
use audio_streams::AsyncPlaybackBufferStream;
use audio_streams::AudioStreamsExecutor;
use audio_streams::BoxError;
use audio_streams::BufferCommit;
use audio_streams::NoopStreamControl;
use audio_streams::PlaybackBuffer;
use audio_streams::PlaybackBufferStream;
use audio_streams::SampleFormat;
use audio_streams::StreamControl;
use audio_streams::StreamEffect;
use audio_streams::StreamSource;
use audio_streams::StreamSourceGenerator;
use std::path::Path;

use base::error;
use base::warn;
use thiserror::Error;

#[derive(Clone, Copy)]
enum AndroidAudioStreamDirection {
    Input = 1,
    Output = 0,
}

#[derive(Error, Debug)]
pub enum AAudioError {
    #[error("Failed to create stream builder")]
    StreamBuilderCreation,
    #[error("Failed to open stream")]
    StreamOpen,
    #[error("Failed to start stream")]
    StreamStart,
    #[error("Failed to delete stream builder")]
    StreamBuilderDelete,
}

// Opaque blob
#[repr(C)]
struct AAudioStream {
    _data: [u8; 0],
    _marker: core::marker::PhantomData<(*mut u8, core::marker::PhantomPinned)>,
}

// Opaque blob
#[repr(C)]
struct AAudioStreamBuilder {
    _data: [u8; 0],
    _marker: core::marker::PhantomData<(*mut u8, core::marker::PhantomPinned)>,
}

type AaudioFormatT = i32;
type AaudioResultT = i32;
const AAUDIO_OK: AaudioResultT = 0;

// aaudio_format_t, as the NDK header numbers them. Written out rather than derived from the
// SampleFormat discriminant: those two happen to agree on 16-bit and on nothing else, so casting
// one to the other described S32 as 24-bit-packed and S24 as float -- a frame size the stream
// does not have, which is silence or noise rather than an error.
const AAUDIO_FORMAT_UNSPECIFIED: AaudioFormatT = 0;
/// `AAUDIO_UNSPECIFIED`, which is what the rate and channel-count setters take to mean "you
/// choose". Distinct from the format constant of the same name only in type.
const AAUDIO_UNSPECIFIED_VALUE: i32 = 0;
const AAUDIO_FORMAT_PCM_I16: AaudioFormatT = 1;
const AAUDIO_FORMAT_PCM_FLOAT: AaudioFormatT = 2;
const AAUDIO_FORMAT_PCM_I32: AaudioFormatT = 4;

/// The AAudio format that carries this one byte for byte, or `None` when there is none.
///
/// U8 has no AAudio equivalent at all. Neither does virtio's S24, which is 24 significant bits in
/// a four-byte container: AAudio's only 24-bit format packs them into three bytes, so the two
/// describe different frame sizes and cannot be substituted for one another.
fn aaudio_format(format: SampleFormat) -> Option<AaudioFormatT> {
    match format {
        SampleFormat::S16LE => Some(AAUDIO_FORMAT_PCM_I16),
        SampleFormat::S32LE => Some(AAUDIO_FORMAT_PCM_I32),
        SampleFormat::F32LE => Some(AAUDIO_FORMAT_PCM_FLOAT),
        SampleFormat::U8 | SampleFormat::S24LE => None,
    }
}
const NANOS_PER_SEC: i64 = 1_000_000_000;
/// `AAUDIO_UNSPECIFIED`: let the platform pick the device (the AAudio default).
pub const AAUDIO_DEVICE_UNSPECIFIED: i32 = 0;

extern "C" {
    fn AAudio_createStreamBuilder(builder: *mut *mut AAudioStreamBuilder) -> AaudioResultT;
    fn AAudioStreamBuilder_delete(builder: *mut AAudioStreamBuilder) -> AaudioResultT;
    fn AAudioStreamBuilder_setBufferCapacityInFrames(
        builder: *mut AAudioStreamBuilder,
        num_frames: i32,
    );
    fn AAudioStreamBuilder_setDirection(builder: *mut AAudioStreamBuilder, direction: u32);
    fn AAudioStreamBuilder_setFormat(builder: *mut AAudioStreamBuilder, format: AaudioFormatT);
    fn AAudioStreamBuilder_setSampleRate(builder: *mut AAudioStreamBuilder, sample_rate: i32);
    fn AAudioStreamBuilder_setChannelCount(builder: *mut AAudioStreamBuilder, channel_count: i32);
    fn AAudioStreamBuilder_setDeviceId(builder: *mut AAudioStreamBuilder, device_id: i32);
    /// What an output stream is for. Decides routing when no device is named, and the volume
    /// stream and ducking rules either way.
    fn AAudioStreamBuilder_setUsage(builder: *mut AAudioStreamBuilder, usage: i32);
    /// What an output stream carries. The companion to usage: one says why, the other what.
    fn AAudioStreamBuilder_setContentType(builder: *mut AAudioStreamBuilder, content_type: i32);
    /// What an input stream is for. Decides which microphone the platform picks when none is
    /// named, and which processing -- echo cancellation, noise suppression -- is applied either
    /// way.
    fn AAudioStreamBuilder_setInputPreset(builder: *mut AAudioStreamBuilder, input_preset: i32);
    /// Which endpoint the platform actually gave us, which need not be the one asked for.
    fn AAudioStream_getDeviceId(stream: *mut AAudioStream) -> i32;
    fn AAudioStream_getFormat(stream: *mut AAudioStream) -> AaudioFormatT;
    fn AAudioStream_getSampleRate(stream: *mut AAudioStream) -> i32;
    fn AAudioStream_getChannelCount(stream: *mut AAudioStream) -> i32;
    fn AAudioStreamBuilder_openStream(
        builder: *mut AAudioStreamBuilder,
        stream: *mut *mut AAudioStream,
    ) -> AaudioResultT;
    fn AAudioStream_getBufferSizeInFrames(stream: *mut AAudioStream) -> i32;
    fn AAudioStream_requestStart(stream: *mut AAudioStream) -> AaudioResultT;
    fn AAudioStream_read(
        stream: *mut AAudioStream,
        buffer: *mut c_void,
        num_frames: i32,
        timeout_nanoseconds: i64,
    ) -> AaudioResultT;
    fn AAudioStream_write(
        stream: *mut AAudioStream,
        buffer: *const c_void,
        num_frames: i32,
        timeout_nanoseconds: i64,
    ) -> AaudioResultT;
    fn AAudioStream_close(stream: *mut AAudioStream) -> AaudioResultT;
}

/// How long to wait before trying to open a departed endpoint again.
///
/// Short enough that plugging a headset back in is not a noticeable wait, long enough that an
/// endpoint which is simply gone costs almost nothing to keep asking about.
const RECONNECT_INTERVAL: Duration = Duration::from_millis(500);

/// Everything needed to open the stream, kept so it can be opened again.
#[derive(Clone)]
struct OpenParams {
    num_channels: usize,
    format: SampleFormat,
    frame_rate: u32,
    buffer_size: usize,
    direction: AndroidAudioStreamDirection,
    /// The endpoint by name; see [`device_table`]. Empty means `device_id` is used instead.
    host_key: String,
    table_path: PathBuf,
    device_id: i32,
}

/// The open AAudio stream, and what it takes to open it again.
///
/// A host endpoint can go away underneath a running stream -- a Bluetooth headset drops, a wired
/// one is unplugged -- and AAudio's answer is to fail every subsequent call on it. There is
/// nothing to tell the guest: virtio-snd has no message for "the thing behind this device is
/// gone", and a guest that was told would tear the endpoint down and take the application's
/// stream with it, which is a worse outcome than a gap in the audio.
///
/// So the guest's stream stays up and this side follows the hardware: play into nothing, record
/// silence, and keep trying to open the endpoint again by the name it was configured with. The
/// number that name resolves to changes across a reconnection, which is exactly why the name is
/// what was kept.
struct AAudioStreamPtr {
    // TODO: Use callback function to avoid possible thread preemption and glitches cause by
    // using AAudio APIs in different threads.
    /// Null while the endpoint is not there.
    stream_ptr: *mut AAudioStream,
    open: OpenParams,
    /// When it is next worth asking the platform for the endpoint again.
    retry_at: Instant,
    /// So the log says it once per outage rather than once per period.
    reported_lost: bool,
}

impl AAudioStreamPtr {
    fn connected(&self) -> bool {
        !self.stream_ptr.is_null()
    }

    /// Closes the stream the platform has already taken away, so the next attempt starts clean.
    fn mark_lost(&mut self, reason: &str) {
        if !self.reported_lost {
            warn!(
                "host audio endpoint {} lost ({}); continuing in silence",
                if self.open.host_key.is_empty() {
                    "(platform routing)"
                } else {
                    &self.open.host_key
                },
                reason
            );
            self.reported_lost = true;
        }
        if !self.stream_ptr.is_null() {
            // SAFETY: the pointer came from AAudioStreamBuilder_openStream and is closed once.
            unsafe {
                AAudioStream_close(self.stream_ptr);
            }
            self.stream_ptr = std::ptr::null_mut();
        }
        self.retry_at = Instant::now() + RECONNECT_INTERVAL;
    }

    /// Opens the endpoint again if it is time to try. Cheap to call on every period.
    fn try_reconnect(&mut self) {
        if self.connected() || Instant::now() < self.retry_at {
            return;
        }
        self.retry_at = Instant::now() + RECONNECT_INTERVAL;
        match open_aaudio_stream(&self.open) {
            Ok(ptr) => {
                self.stream_ptr = ptr;
                self.reported_lost = false;
                warn!(
                    "host audio endpoint {} is back",
                    if self.open.host_key.is_empty() {
                        "(platform routing)"
                    } else {
                        &self.open.host_key
                    }
                );
            }
            Err(_) => {
                // Expected while the endpoint is absent; mark_lost already said so once.
            }
        }
    }
}

/// `aaudio_usage_t`, as named in the configuration. Only the ones an ordinary application may
/// ask for: the rest exist but are refused without system privileges, and offering a choice that
/// cannot be honoured is worse than not offering it.
fn usage_from_name(name: &str) -> Option<i32> {
    Some(match name {
        "media" => 1,
        "voice_communication" => 2,
        "voice_communication_signalling" => 3,
        "alarm" => 4,
        "notification" => 5,
        "notification_ringtone" => 6,
        "notification_event" => 10,
        "assistance_accessibility" => 11,
        "assistance_navigation_guidance" => 12,
        "assistance_sonification" => 13,
        "game" => 14,
        "assistant" => 16,
        _ => return None,
    })
}

/// `aaudio_content_type_t`.
fn content_type_from_name(name: &str) -> Option<i32> {
    Some(match name {
        "speech" => 1,
        "music" => 2,
        "movie" => 3,
        "sonification" => 4,
        _ => return None,
    })
}

/// `aaudio_input_preset_t`. SYSTEM_HOTWORD and SYSTEM_ECHO_REFERENCE are deliberately absent:
/// they need system privileges, so naming them would only produce a stream that fails to open.
fn input_preset_from_name(name: &str) -> Option<i32> {
    Some(match name {
        "generic" => 1,
        "camcorder" => 5,
        "voice_recognition" => 6,
        "voice_communication" => 7,
        "unprocessed" => 9,
        "voice_performance" => 10,
        _ => return None,
    })
}

/// Applies one named attribute, saying so when the name is not one we know rather than quietly
/// leaving the platform's default in place: a setting that was typed and ignored is worse than
/// one that was never offered.
fn apply_attr(attrs: &str, name: &str, lookup: fn(&str) -> Option<i32>, set: impl FnOnce(i32)) {
    let Some(text) = device_table::attr(attrs, name) else {
        return;
    };
    match lookup(text) {
        Some(value) => set(value),
        None => warn!("host audio: {}={} is not a value this driver knows", name, text),
    }
}

/// Opens one AAudio stream for `open`, resolving the endpoint by name when it has one.
/// What an endpoint natively runs at, as opposed to what it will accept.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct NativeConfig {
    pub frame_rate: u32,
    pub num_channels: u8,
    pub format: SampleFormat,
}

/// Asks the endpoint what it runs at, by opening a stream that specifies nothing and reading
/// back what the platform chose.
///
/// This is the only authoritative answer available: `AudioDeviceInfo` reports the rates and
/// channel counts an endpoint will *accept*, which on a real device is a list, and a list does
/// not say which entry costs no conversion.
///
/// The stream is opened but never started, which is what keeps this from being a recording: an
/// open input stream raises no event in the platform's recording activity and no track in the
/// mixer, where starting one does both. Measured, because the difference decides whether probing
/// a microphone lights the indicator on the user's phone.
pub fn probe_native(table_path: &Path, host_key: &str, input: bool) -> Option<NativeConfig> {
    let device_id = device_table::resolve(table_path, host_key, input)
        .unwrap_or(AAUDIO_DEVICE_UNSPECIFIED);

    let mut builder: *mut AAudioStreamBuilder = std::ptr::null_mut();
    let mut stream_ptr: *mut AAudioStream = std::ptr::null_mut();
    // SAFETY: interfacing with the AAudio C API; the pointers are ours and checked.
    unsafe {
        if AAudio_createStreamBuilder(&mut builder) != AAUDIO_OK {
            return None;
        }
        AAudioStreamBuilder_setDirection(
            builder,
            if input {
                AndroidAudioStreamDirection::Input as u32
            } else {
                AndroidAudioStreamDirection::Output as u32
            },
        );
        AAudioStreamBuilder_setDeviceId(builder, device_id);
        // Everything else left unspecified on purpose: that is what makes the platform answer
        // with the endpoint's own configuration instead of converting to a request.
        AAudioStreamBuilder_setFormat(builder, AAUDIO_FORMAT_UNSPECIFIED);
        AAudioStreamBuilder_setSampleRate(builder, AAUDIO_UNSPECIFIED_VALUE);
        AAudioStreamBuilder_setChannelCount(builder, AAUDIO_UNSPECIFIED_VALUE);
        let opened = AAudioStreamBuilder_openStream(builder, &mut stream_ptr);
        AAudioStreamBuilder_delete(builder);
        if opened != AAUDIO_OK {
            warn!(
                "host audio endpoint {}: cannot be opened to ask what it runs at",
                host_key
            );
            return None;
        }

        let rate = AAudioStream_getSampleRate(stream_ptr);
        let channels = AAudioStream_getChannelCount(stream_ptr);
        let format = AAudioStream_getFormat(stream_ptr);
        AAudioStream_close(stream_ptr);

        let format = match format {
            AAUDIO_FORMAT_PCM_I16 => SampleFormat::S16LE,
            AAUDIO_FORMAT_PCM_I32 => SampleFormat::S32LE,
            AAUDIO_FORMAT_PCM_FLOAT => SampleFormat::F32LE,
            // A layout with no virtio equivalent. Saying nothing is better than naming a format
            // the guest would then be encouraged to pick.
            _ => return None,
        };
        if rate <= 0 || channels <= 0 || channels > u8::MAX as i32 {
            return None;
        }
        Some(NativeConfig {
            frame_rate: rate as u32,
            num_channels: channels as u8,
            format,
        })
    }
}

fn open_aaudio_stream(open: &OpenParams) -> Result<*mut AAudioStream, BoxError> {
    // Refused here rather than at the builder: a format with no AAudio equivalent would
    // otherwise be sent as whichever one happened to share its number.
    let format = match aaudio_format(open.format) {
        Some(f) => f,
        None => {
            error!(
                "host audio endpoint {}: {} has no AAudio equivalent",
                open.host_key, open.format
            );
            return Err(Box::new(AAudioError::StreamOpen));
        }
    };

    // Resolved per attempt, not once: the number an endpoint has changes every time it is
    // reconnected, and following it is the whole reason the name is carried.
    let device_id = if open.host_key.is_empty() {
        open.device_id
    } else {
        device_table::resolve(&open.table_path, &open.host_key, matches!(open.direction, AndroidAudioStreamDirection::Input))
            .unwrap_or(AAUDIO_DEVICE_UNSPECIFIED)
    };

    let mut stream_ptr: *mut AAudioStream = std::ptr::null_mut();
    let mut builder: *mut AAudioStreamBuilder = std::ptr::null_mut();
    // SAFETY:
    // Interfacing with the AAudio C API. Assumes correct linking
    // and `builder` and `stream_ptr` pointers are valid and properly initialized.
    unsafe {
        if AAudio_createStreamBuilder(&mut builder) != AAUDIO_OK {
            return Err(Box::new(AAudioError::StreamBuilderCreation));
        }
        AAudioStreamBuilder_setDirection(builder, open.direction as u32);
        AAudioStreamBuilder_setBufferCapacityInFrames(builder, open.buffer_size as i32 * 2);
        AAudioStreamBuilder_setFormat(builder, format);
        AAudioStreamBuilder_setSampleRate(builder, open.frame_rate as i32);
        AAudioStreamBuilder_setChannelCount(builder, open.num_channels as i32);
        // AAUDIO_UNSPECIFIED (0) keeps the platform's own routing; anything else pins the
        // stream to one host endpoint (a specific speaker, headset, mic, ...).
        AAudioStreamBuilder_setDeviceId(builder, device_id);
        // What the stream is for, which is a separate question from which endpoint it is on.
        // For input it decides the processing the platform applies -- echo cancellation, noise
        // suppression -- and for output the volume stream it belongs to; for either, it is what
        // chooses the endpoint when none was named.
        let (_, attrs) = device_table::split_key(&open.host_key);
        match open.direction {
            AndroidAudioStreamDirection::Input => {
                apply_attr(attrs, "preset", input_preset_from_name, |v| {
                    AAudioStreamBuilder_setInputPreset(builder, v)
                });
            }
            AndroidAudioStreamDirection::Output => {
                apply_attr(attrs, "usage", usage_from_name, |v| {
                    AAudioStreamBuilder_setUsage(builder, v)
                });
                apply_attr(attrs, "content", content_type_from_name, |v| {
                    AAudioStreamBuilder_setContentType(builder, v)
                });
            }
        }
        if AAudioStreamBuilder_openStream(builder, &mut stream_ptr) != AAUDIO_OK {
            AAudioStreamBuilder_delete(builder);
            return Err(Box::new(AAudioError::StreamOpen));
        }
        if AAudioStreamBuilder_delete(builder) != AAUDIO_OK {
            return Err(Box::new(AAudioError::StreamBuilderDelete));
        }
        if AAudioStream_requestStart(stream_ptr) != AAUDIO_OK {
            AAudioStream_close(stream_ptr);
            return Err(Box::new(AAudioError::StreamStart));
        }
    }

    // Every frame written from here on is laid out the way the guest declared in SET_PARAMS, so a
    // stream that came back with a different layout would be fed frames of the wrong size --
    // audio at the wrong speed or noise, with nothing anywhere saying why.
    // SAFETY: the stream was opened just above.
    let (got_format, got_rate, got_channels) = unsafe {
        (
            AAudioStream_getFormat(stream_ptr),
            AAudioStream_getSampleRate(stream_ptr),
            AAudioStream_getChannelCount(stream_ptr),
        )
    };
    if got_format != format
        || got_rate != open.frame_rate as i32
        || got_channels != open.num_channels as i32
    {
        error!(
            "host audio endpoint {}: asked for {}Hz/{}ch/format {}, the platform gave \
             {}Hz/{}ch/format {}; refusing rather than writing frames of the wrong size",
            open.host_key, open.frame_rate, open.num_channels, format,
            got_rate, got_channels, got_format
        );
        // SAFETY: the stream was opened just above and is closed once.
        unsafe { AAudioStream_close(stream_ptr) };
        return Err(Box::new(AAudioError::StreamOpen));
    }

    // Asking for an endpoint and getting one are different things: the platform routes by the
    // stream's purpose as well as by the id, and a device id it will not honour is refused
    // silently, by handing back a stream on something else. Nothing downstream can tell -- the
    // audio simply comes from somewhere the user did not choose -- so say it here.
    // SAFETY: the stream was opened and started just above.
    let actual = unsafe { AAudioStream_getDeviceId(stream_ptr) };
    if device_id != AAUDIO_DEVICE_UNSPECIFIED && actual != device_id {
        warn!(
            "host audio endpoint {}: asked for device {}, the platform gave {}",
            if open.host_key.is_empty() { "(by id)" } else { &open.host_key },
            device_id,
            actual
        );
    } else {
        base::info!(
            "host audio endpoint {} opened on device {}",
            if open.host_key.is_empty() { "(by id)" } else { &open.host_key },
            actual
        );
    }
    Ok(stream_ptr)
}

// SAFETY:
// AudioStream.drop.buffer_ptr: *const u8 points to AudioStream.buffer, which would be alive
// whenever AudioStream.drop.buffer_ptr is alive.
unsafe impl Send for AndroidAudioStreamCommit {}

struct AudioStream {
    buffer: Box<[u8]>,
    frame_size: usize,
    frame_rate: u32,
    next_frame: Instant,
    start_time: Option<Instant>,
    total_frames: i32,
    buffer_drop: AndroidAudioStreamCommit,
    read_count: i32,
    aaudio_buffer_size: usize,
}

struct AndroidAudioStreamCommit {
    buffer_ptr: *const u8,
    stream: AAudioStreamPtr,
    direction: AndroidAudioStreamDirection,
    /// Needed to size the write timeout in terms of the audio the buffer holds.
    frame_size: usize,
    frame_rate: u32,
}

impl BufferCommit for AndroidAudioStreamCommit {
    fn commit(&mut self, _nwritten: usize) {
        // This traits function is never called.
        unimplemented!();
    }
}

#[async_trait(?Send)]
impl AsyncBufferCommit for AndroidAudioStreamCommit {
    async fn commit(&mut self, nwritten: usize) {
        match self.direction {
            AndroidAudioStreamDirection::Input => {}
            AndroidAudioStreamDirection::Output => {
                // Upstream passed a zero timeout and threw away whatever AAudio would not
                // take, which is an audible click every time the device buffer happens to be
                // full. Wait instead, then finish the remainder: the period is only worth a
                // few milliseconds of audio, and pacing to the device is what this path is
                // for. The wait is bounded at twice the audio the buffer holds so a wedged
                // stream cannot stall the virtio-snd worker.
                self.stream.try_reconnect();
                if !self.stream.connected() {
                    // Nowhere to put it. The guest's stream is still running, so this period is
                    // simply not heard -- which is what a gap in the audio sounds like, and is
                    // the point: the alternative is telling the guest its device vanished and
                    // taking the application's stream down with it.
                    return;
                }
                let mut written = 0usize;
                while written < nwritten {
                    let remaining = nwritten - written;
                    let timeout_ns = if self.frame_rate > 0 {
                        (remaining as i64)
                            .saturating_mul(2 * NANOS_PER_SEC)
                            / i64::from(self.frame_rate)
                    } else {
                        0
                    };
                    // SAFETY: see above; buffer_ptr stays valid and `written` never exceeds
                    // nwritten, which is within the buffer.
                    let frames_written: i32 = unsafe {
                        AAudioStream_write(
                            self.stream.stream_ptr,
                            self.buffer_ptr.add(written * self.frame_size) as *const c_void,
                            remaining as i32,
                            timeout_ns,
                        )
                    };
                    if frames_written < 0 {
                        self.stream.mark_lost("write failed");
                        break;
                    }
                    if frames_written == 0 {
                        // Timed out with no progress: dropping the rest is all that is left,
                        // but say so -- silently discarding audio is what made this hard to
                        // find in the first place.
                        warn!("Android Audio Stream: dropping {} frames after timeout", remaining);
                        break;
                    }
                    written += frames_written as usize;
                }
            }
        }
    }
}

impl AudioStream {
    pub fn new(
        num_channels: usize,
        format: SampleFormat,
        frame_rate: u32,
        buffer_size: usize,
        direction: AndroidAudioStreamDirection,
        open: OpenParams,
    ) -> Result<Self, BoxError> {
        let frame_size = format.sample_bytes() * num_channels;

        let stream_ptr = open_aaudio_stream(&open)?;
        // SAFETY:
        // Interfacing with the AAudio C API. Assumes correct linking
        // and `stream_ptr` pointers are valid and properly initialized.
        let aaudio_buffer_size = unsafe { AAudioStream_getBufferSizeInFrames(stream_ptr) } as usize;
        let buffer = vec![0; buffer_size * frame_size].into_boxed_slice();
        let stream = AAudioStreamPtr {
            stream_ptr,
            open,
            retry_at: Instant::now(),
            reported_lost: false,
        };
        let buffer_drop = AndroidAudioStreamCommit {
            stream,
            buffer_ptr: buffer.as_ptr(),
            direction,
            frame_size,
            frame_rate,
        };
        Ok(AudioStream {
            buffer,
            frame_size,
            frame_rate,
            next_frame: Instant::now(),
            start_time: None,
            total_frames: 0,
            buffer_drop,
            read_count: 0,
            aaudio_buffer_size,
        })
    }
}

impl PlaybackBufferStream for AudioStream {
    fn next_playback_buffer<'b, 's: 'b>(&'s mut self) -> Result<PlaybackBuffer<'b>, BoxError> {
        // This traits function is never called.
        unimplemented!();
    }
}

impl AudioStream {
    /// Waits until this period is due, and returns the anchor the schedule is measured from.
    ///
    /// The schedule is absolute -- period `n` is due `n` periods after the anchor -- so that the
    /// stream neither drifts nor accumulates rounding. The part that matters is what happens when
    /// a period comes late, which on a phone it eventually will: the naive reading is that the
    /// stream owes the difference and should deliver the backlog as fast as it can. It must not.
    /// Delivering faster than real time hands the guest completions for audio that has not been
    /// heard yet, and a guest writing into a cyclic buffer against those completions overruns the
    /// part the audio engine is still filling. What that sounds like is a tear at a period
    /// boundary -- measured, at eight per second, exactly on the boundaries.
    ///
    /// So a period that is merely a little late is absorbed by the schedule, and one that is late
    /// by more than a whole period re-anchors it: the backlog is abandoned rather than chased.
    async fn pace(
        &mut self,
        ex: &dyn AudioStreamsExecutor,
        buffer_size: usize,
    ) -> Result<Instant, BoxError> {
        let now = Instant::now();
        let anchor = match self.start_time {
            None => now,
            Some(anchor) => {
                if let Some(wait) = self.next_frame.checked_duration_since(now) {
                    ex.delay(wait).await?;
                    anchor
                } else if now.duration_since(self.next_frame) >= self.period_duration(buffer_size) {
                    // More than a period behind. Start counting again from here; the audio that
                    // should have been delivered in the gap is gone either way, and racing to
                    // deliver it now would only put the guest ahead of the endpoint.
                    self.total_frames = buffer_size as i32;
                    now
                } else {
                    anchor
                }
            }
        };
        self.start_time = Some(anchor);
        Ok(anchor)
    }

    fn period_duration(&self, buffer_size: usize) -> Duration {
        if self.frame_rate == 0 {
            return Duration::ZERO;
        }
        Duration::from_nanos(buffer_size as u64 * NANOS_PER_SEC as u64 / self.frame_rate as u64)
    }
}

#[async_trait(?Send)]
impl AsyncPlaybackBufferStream for AudioStream {
    async fn next_playback_buffer<'a>(
        &'a mut self,
        ex: &dyn AudioStreamsExecutor,
    ) -> Result<AsyncPlaybackBuffer<'a>, BoxError> {
        let buffer_size = self.buffer.len() / self.frame_size;
        self.total_frames += buffer_size as i32;
        let start_time = self.pace(ex, buffer_size).await?;
        self.next_frame = start_time
            + Duration::from_millis(self.total_frames as u64 * 1000 / self.frame_rate as u64);
        Ok(
            AsyncPlaybackBuffer::new(self.frame_size, self.buffer.as_mut(), &mut self.buffer_drop)
                .map_err(Box::new)?,
        )
    }
}

#[async_trait(?Send)]
impl CaptureBufferStream for AudioStream {
    fn next_capture_buffer<'b, 's: 'b>(&'s mut self) -> Result<CaptureBuffer<'b>, BoxError> {
        // This traits function is never called.
        unimplemented!()
    }
}

#[async_trait(?Send)]
impl AsyncCaptureBufferStream for AudioStream {
    async fn next_capture_buffer<'a>(
        &'a mut self,
        ex: &dyn AudioStreamsExecutor,
    ) -> Result<AsyncCaptureBuffer<'a>, BoxError> {
        let buffer_size = self.buffer.len() / self.frame_size;
        self.read_count += 1;
        self.total_frames += buffer_size as i32;
        let start_time = self.pace(ex, buffer_size).await?;
        self.next_frame = start_time
            + Duration::from_millis(self.total_frames as u64 * 1000 / self.frame_rate as u64);

        // Skip for at least (1.5x aaudio buffer size - buffer_size) to ensure there is always a
        // aaudio buffer available for read.
        if self.read_count < (self.aaudio_buffer_size * 3 / 2 / buffer_size) as i32 + 1 {
            self.buffer.fill(0);
            return Ok(AsyncCaptureBuffer::new(
                buffer_size,
                self.buffer.as_mut(),
                &mut self.buffer_drop,
            )
            .map_err(Box::new)?);
        }

        self.buffer_drop.stream.try_reconnect();
        if !self.buffer_drop.stream.connected() {
            // Nothing is listening. Hand the guest silence rather than an error: its stream keeps
            // running and the recording has a gap, which is recoverable, where a failed read
            // would end the application's capture.
            self.buffer.fill(0);
            return Ok(AsyncCaptureBuffer::new(
                buffer_size,
                self.buffer.as_mut(),
                &mut self.buffer_drop,
            )
            .map_err(Box::new)?);
        }

        // SAFETY:
        // The AAudioStream_read writes buffer for buffer.len() / frame_size * frame_size bytes
        let frames_read = unsafe {
            AAudioStream_read(
                self.buffer_drop.stream.stream_ptr,
                self.buffer.as_mut_ptr() as *mut c_void,
                (buffer_size) as i32,
                0,
            )
        };

        if frames_read < 0 {
            self.buffer_drop.stream.mark_lost("read failed");
            self.buffer.fill(0);
        } else if (frames_read as usize) < buffer_size {
            warn!(
                "AAudio stream read data not enough. frames read: {frames_read}, buffer size: {buffer_size}",
            );
            self.buffer[frames_read as usize * self.frame_size..].fill(0);
        }

        Ok(
            AsyncCaptureBuffer::new(buffer_size, self.buffer.as_mut(), &mut self.buffer_drop)
                .map_err(Box::new)?,
        )
    }
}

impl Drop for AAudioStreamPtr {
    fn drop(&mut self) {
        // SAFETY:
        // Interfacing with the AAudio C API. Assumes correct linking
        // and `stream_ptr` are valid and properly initialized.
        if unsafe { AAudioStream_close(self.stream_ptr) } != AAUDIO_OK {
            warn!("AAudio stream close failed.");
        }
    }
}

#[derive(Default)]
struct AndroidAudioStreamSource {
    /// The endpoint by a name that survives it being unplugged; see
    /// [`device_table`]. Empty, or [`device_table::SYSTEM_DEFAULT_KEY`], to follow the
    /// platform's own routing.
    host_key: String,
    /// Where the platform's current device list is published.
    table_path: PathBuf,
    /// Used when no name was given, so a caller that already knows the number still works.
    device_id: i32,
}

impl AndroidAudioStreamSource {
    /// Today's id for the configured endpoint.
    ///
    /// Resolved per stream rather than once, because the number changes every time the device is
    /// reconnected while the name does not -- that is the whole point of having the name. An
    /// endpoint that is not currently present resolves to
    /// [`AAUDIO_DEVICE_UNSPECIFIED`]: the stream opens on whatever the platform routes to rather
    /// than failing, and is reopened on the right endpoint when it comes back.
    fn open_params(
        &self,
        num_channels: usize,
        format: SampleFormat,
        frame_rate: u32,
        buffer_size: usize,
        direction: AndroidAudioStreamDirection,
    ) -> OpenParams {
        OpenParams {
            num_channels,
            format,
            frame_rate,
            buffer_size,
            direction,
            host_key: self.host_key.clone(),
            table_path: self.table_path.clone(),
            device_id: self.device_id,
        }
    }
}

impl StreamSource for AndroidAudioStreamSource {
    #[allow(clippy::type_complexity)]
    fn new_playback_stream(
        &mut self,
        _num_channels: usize,
        _format: SampleFormat,
        _frame_rate: u32,
        _buffer_size: usize,
    ) -> Result<(Box<dyn StreamControl>, Box<dyn PlaybackBufferStream>), BoxError> {
        // This traits function is never called.
        unimplemented!();
    }

    #[allow(clippy::type_complexity)]
    fn new_async_playback_stream(
        &mut self,
        num_channels: usize,
        format: SampleFormat,
        frame_rate: u32,
        buffer_size: usize,
        _ex: &dyn AudioStreamsExecutor,
    ) -> Result<(Box<dyn StreamControl>, Box<dyn AsyncPlaybackBufferStream>), BoxError> {
        let audio_stream = AudioStream::new(
            num_channels,
            format,
            frame_rate,
            buffer_size,
            AndroidAudioStreamDirection::Output,
            self.open_params(
                num_channels,
                format,
                frame_rate,
                buffer_size,
                AndroidAudioStreamDirection::Output,
            ),
        )?;
        Ok((Box::new(NoopStreamControl::new()), Box::new(audio_stream)))
    }

    #[allow(clippy::type_complexity)]
    fn new_capture_stream(
        &mut self,
        _num_channels: usize,
        _format: SampleFormat,
        _frame_rate: u32,
        _buffer_size: usize,
        _effects: &[StreamEffect],
    ) -> std::result::Result<(Box<dyn StreamControl>, Box<dyn CaptureBufferStream>), BoxError> {
        // This traits function is never called.
        unimplemented!();
    }

    #[allow(clippy::type_complexity)]
    fn new_async_capture_stream(
        &mut self,
        num_channels: usize,
        format: SampleFormat,
        frame_rate: u32,
        buffer_size: usize,
        _effects: &[StreamEffect],
        _ex: &dyn AudioStreamsExecutor,
    ) -> std::result::Result<(Box<dyn StreamControl>, Box<dyn AsyncCaptureBufferStream>), BoxError>
    {
        let audio_stream = AudioStream::new(
            num_channels,
            format,
            frame_rate,
            buffer_size,
            AndroidAudioStreamDirection::Input,
            self.open_params(
                num_channels,
                format,
                frame_rate,
                buffer_size,
                AndroidAudioStreamDirection::Input,
            ),
        )?;
        Ok((Box::new(NoopStreamControl::new()), Box::new(audio_stream)))
    }
}

#[derive(Default)]
pub struct AndroidAudioStreamSourceGenerator {
    host_key: String,
    table_path: PathBuf,
    device_id: i32,
}

impl AndroidAudioStreamSourceGenerator {
    /// `device_id` is an `AAudioDeviceInfo` id as reported by Android's `AudioManager`, or
    /// [`AAUDIO_DEVICE_UNSPECIFIED`] to follow whatever the platform would route to anyway.
    pub fn new(device_id: i32) -> Self {
        AndroidAudioStreamSourceGenerator {
            host_key: String::new(),
            table_path: PathBuf::new(),
            device_id,
        }
    }

    /// Names the endpoint instead of numbering it. `host_key` is `TYPE|address` as Android's
    /// `AudioDeviceInfo` reports them, and `table_path` is where the current device list is
    /// published; both are needed, because the number the name resolves to changes every time
    /// the device is reconnected.
    pub fn with_host_key(host_key: String, table_path: PathBuf) -> Self {
        AndroidAudioStreamSourceGenerator {
            host_key,
            table_path,
            device_id: AAUDIO_DEVICE_UNSPECIFIED,
        }
    }
}

/// `AndroidAudioStreamSourceGenerator` is a struct that implements [`StreamSourceGenerator`]
/// for `AndroidAudioStreamSource`.
impl StreamSourceGenerator for AndroidAudioStreamSourceGenerator {
    fn generate(&self) -> Result<Box<dyn StreamSource>, BoxError> {
        Ok(Box::new(AndroidAudioStreamSource {
            host_key: self.host_key.clone(),
            table_path: self.table_path.clone(),
            device_id: self.device_id,
        }))
    }
}
