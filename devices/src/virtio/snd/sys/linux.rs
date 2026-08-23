// Copyright 2022 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#[cfg(feature = "audio_aaudio")]
use android_audio::AndroidAudioStreamSourceGenerator;
#[cfg(feature = "audio_aaudio")]
use android_audio::AAUDIO_DEVICE_UNSPECIFIED;
use async_trait::async_trait;
use audio_streams::capture::AsyncCaptureBuffer;
use audio_streams::capture::AsyncCaptureBufferStream;
use audio_streams::AsyncPlaybackBufferStream;
use audio_streams::BoxError;
use audio_streams::StreamSource;
use audio_streams::StreamSourceGenerator;
#[cfg(feature = "audio_cras")]
use base::error;
use base::set_rt_prio_limit;
use base::set_rt_round_robin;
use base::warn;
use cros_async::Executor;
use futures::channel::mpsc::UnboundedSender;
#[cfg(feature = "audio_cras")]
use libcras::CrasStreamSourceGenerator;
#[cfg(feature = "audio_cras")]
use libcras::CrasStreamType;
use serde::Deserialize;
use serde::Serialize;

use crate::virtio::snd::constants::VIRTIO_SND_D_INPUT;
use crate::virtio::snd::common_backend::async_funcs::CaptureBufferReader;
use crate::virtio::snd::common_backend::async_funcs::PlaybackBufferWriter;
use crate::virtio::snd::common_backend::stream_info::StreamInfo;
use crate::virtio::snd::common_backend::underrun::UnderrunConcealer;
use crate::virtio::snd::parameters::UnderrunMode;
use crate::virtio::snd::common_backend::DirectionalStream;
use crate::virtio::snd::common_backend::Error;
use crate::virtio::snd::common_backend::PcmResponse;
use crate::virtio::snd::common_backend::SndData;
use crate::virtio::snd::parameters::Error as ParametersError;
use crate::virtio::snd::parameters::Parameters;

const AUDIO_THREAD_RTPRIO: u16 = 10; // Matches other cros audio clients.

pub(crate) type SysAudioStreamSourceGenerator = Box<dyn StreamSourceGenerator>;
pub(crate) type SysAudioStreamSource = Box<dyn StreamSource>;
pub(crate) type SysBufferReader = UnixBufferReader;

pub struct SysDirectionOutput {
    pub async_playback_buffer_stream: Box<dyn audio_streams::AsyncPlaybackBufferStream>,
    pub buffer_writer: Box<dyn PlaybackBufferWriter>,
    /// Present only when this stream conceals underruns; see [`UnderrunConcealer`].
    pub concealer: Option<UnderrunConcealer>,
}

pub(crate) struct SysAsyncStreamObjects {
    pub(crate) stream: DirectionalStream,
    pub(crate) pcm_sender: UnboundedSender<PcmResponse>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Serialize)]
pub enum StreamSourceBackend {
    #[cfg(feature = "audio_aaudio")]
    AAUDIO,
    #[cfg(feature = "audio_cras")]
    CRAS,
}

// Implemented to make backend serialization possible, since we deserialize from str.
impl From<StreamSourceBackend> for String {
    fn from(backend: StreamSourceBackend) -> Self {
        match backend {
            #[cfg(feature = "audio_aaudio")]
            StreamSourceBackend::AAUDIO => "aaudio".to_owned(),
            #[cfg(feature = "audio_cras")]
            StreamSourceBackend::CRAS => "cras".to_owned(),
        }
    }
}

impl TryFrom<&str> for StreamSourceBackend {
    type Error = ParametersError;

    fn try_from(s: &str) -> Result<Self, Self::Error> {
        match s {
            #[cfg(feature = "audio_aaudio")]
            "aaudio" => Ok(StreamSourceBackend::AAUDIO),
            #[cfg(feature = "audio_cras")]
            "cras" => Ok(StreamSourceBackend::CRAS),
            _ => Err(ParametersError::InvalidBackend),
        }
    }
}

#[cfg(feature = "audio_aaudio")]
pub(crate) fn create_aaudio_stream_source_generators(
    params: &Parameters,
    snd_data: &SndData,
) -> Vec<SysAudioStreamSourceGenerator> {
    let mut generators: Vec<Box<dyn StreamSourceGenerator>> =
        Vec::with_capacity(snd_data.pcm_info_len());
    for pcm_info in snd_data.pcm_info_iter() {
        assert_eq!(pcm_info.features, 0); // Should be 0. Android audio backend does not support any features.
        // Per-PCM-device host routing: output_device_config[i]/input_device_config[i] may pin
        // device i to one AAudio endpoint. Anything unset stays on the platform's own routing.
        //
        // Say so when the config is shorter than the device count. Falling back is right -- a
        // device with no entry should follow the platform -- but doing it silently makes a
        // miscounted config space look exactly like a deliberate one, and the symptom is audio
        // coming out of the wrong endpoint with nothing in the log to explain it.
        let device_params = match params.get_device_params(pcm_info) {
            Ok(p) => p,
            Err(e) => {
                warn!(
                    "virtio-snd: no host device config for {} device {} ({}); \
                     following the platform's routing",
                    if pcm_info.direction == VIRTIO_SND_D_INPUT {
                        "input"
                    } else {
                        "output"
                    },
                    u32::from(pcm_info.hdr.hda_fn_nid),
                    e
                );
                Default::default()
            }
        };
        // Prefer the name over the number: the number is only what the endpoint happens to be
        // called today, and a reconnected device gets a new one.
        let generator = match (&device_params.host_device, params.device_table.as_str()) {
            (Some(key), table) if !key.is_empty() && !table.is_empty() => {
                AndroidAudioStreamSourceGenerator::with_host_key(key.clone(), table.into())
            }
            _ => AndroidAudioStreamSourceGenerator::new(
                device_params.device_id.unwrap_or(AAUDIO_DEVICE_UNSPECIFIED),
            ),
        };
        generators.push(Box::new(generator));
    }
    generators
}

#[cfg(feature = "audio_cras")]
pub(crate) fn create_cras_stream_source_generators(
    params: &Parameters,
    snd_data: &SndData,
) -> Vec<Box<dyn StreamSourceGenerator>> {
    let mut generators: Vec<Box<dyn StreamSourceGenerator>> =
        Vec::with_capacity(snd_data.pcm_info_len());
    for pcm_info in snd_data.pcm_info_iter() {
        let device_params = params.get_device_params(pcm_info).unwrap_or_else(|err| {
            error!("Create cras stream source generator error: {}", err);
            Default::default()
        });
        generators.push(Box::new(CrasStreamSourceGenerator::with_stream_type(
            params.capture,
            device_params.client_type.unwrap_or(params.client_type),
            params.socket_type,
            device_params
                .stream_type
                .unwrap_or(CrasStreamType::CRAS_STREAM_TYPE_DEFAULT),
        )));
    }
    generators
}

#[allow(unused_variables)]
pub(crate) fn create_stream_source_generators(
    backend: StreamSourceBackend,
    params: &Parameters,
    snd_data: &SndData,
) -> Vec<Box<dyn StreamSourceGenerator>> {
    match backend {
        #[cfg(feature = "audio_aaudio")]
        StreamSourceBackend::AAUDIO => create_aaudio_stream_source_generators(params, snd_data),
        #[cfg(feature = "audio_cras")]
        StreamSourceBackend::CRAS => create_cras_stream_source_generators(params, snd_data),
    }
}

pub(crate) fn set_audio_thread_priority() -> Result<(), base::Error> {
    set_rt_prio_limit(u64::from(AUDIO_THREAD_RTPRIO))
        .and_then(|_| set_rt_round_robin(i32::from(AUDIO_THREAD_RTPRIO)))
}

impl StreamInfo {
    /// (*)
    /// `buffer_size` in `audio_streams` API indicates the buffer size in bytes that the stream
    /// consumes (or transmits) each time (next_playback/capture_buffer).
    /// `period_bytes` in virtio-snd device (or ALSA) indicates the device transmits (or
    /// consumes) for each PCM message.
    /// Therefore, `buffer_size` in `audio_streams` == `period_bytes` in virtio-snd.
    async fn set_up_async_playback_stream(
        &mut self,
        frame_size: usize,
        ex: &Executor,
    ) -> Result<Box<dyn AsyncPlaybackBufferStream>, Error> {
        Ok(self
            .stream_source
            .as_mut()
            .ok_or(Error::EmptyStreamSource)?
            .async_new_async_playback_stream(
                self.channels as usize,
                self.format,
                self.frame_rate,
                // See (*)
                self.period_bytes / frame_size,
                ex,
            )
            .await
            .map_err(Error::CreateStream)?
            .1)
    }

    pub(crate) async fn set_up_async_capture_stream(
        &mut self,
        frame_size: usize,
        ex: &Executor,
    ) -> Result<SysBufferReader, Error> {
        let async_capture_buffer_stream = self
            .stream_source
            .as_mut()
            .ok_or(Error::EmptyStreamSource)?
            .async_new_async_capture_stream(
                self.channels as usize,
                self.format,
                self.frame_rate,
                self.period_bytes / frame_size,
                &self.effects,
                ex,
            )
            .await
            .map_err(Error::CreateStream)?
            .1;
        Ok(SysBufferReader::new(async_capture_buffer_stream))
    }

    pub(crate) async fn create_directionstream_output(
        &mut self,
        frame_size: usize,
        ex: &Executor,
    ) -> Result<DirectionalStream, Error> {
        let async_playback_buffer_stream =
            self.set_up_async_playback_stream(frame_size, ex).await?;

        let buffer_writer = UnixBufferWriter::new(self.period_bytes);

        // Built here because this is where the stream's geometry is known. None when the mode is
        // Silence, or when the format is one the concealer will not guess at.
        let concealer = match self.underrun {
            UnderrunMode::Silence => None,
            UnderrunMode::Repeat => UnderrunConcealer::new(
                self.channels as usize,
                self.frame_rate,
                self.period_bytes,
                self.format.sample_bytes(),
            ),
        };

        Ok(DirectionalStream::Output(SysDirectionOutput {
            async_playback_buffer_stream,
            buffer_writer: Box::new(buffer_writer),
            concealer,
        }))
    }
}

pub(crate) struct UnixBufferReader {
    async_stream: Box<dyn AsyncCaptureBufferStream>,
}

impl UnixBufferReader {
    fn new(async_stream: Box<dyn AsyncCaptureBufferStream>) -> Self
    where
        Self: Sized,
    {
        UnixBufferReader { async_stream }
    }
}
#[async_trait(?Send)]
impl CaptureBufferReader for UnixBufferReader {
    async fn get_next_capture_period(
        &mut self,
        ex: &Executor,
    ) -> Result<AsyncCaptureBuffer, BoxError> {
        Ok(self
            .async_stream
            .next_capture_buffer(ex)
            .await
            .map_err(Error::FetchBuffer)?)
    }
}

pub(crate) struct UnixBufferWriter {
    guest_period_bytes: usize,
}

#[async_trait(?Send)]
impl PlaybackBufferWriter for UnixBufferWriter {
    fn new(guest_period_bytes: usize) -> Self {
        UnixBufferWriter { guest_period_bytes }
    }
    fn endpoint_period_bytes(&self) -> usize {
        self.guest_period_bytes
    }
}
