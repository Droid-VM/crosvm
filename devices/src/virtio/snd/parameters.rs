// Copyright 2022 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use std::num::ParseIntError;
use std::str::ParseBoolError;

use audio_streams::StreamEffect;
#[cfg(all(unix, feature = "audio_cras"))]
use libcras::CrasClientType;
#[cfg(all(unix, feature = "audio_cras"))]
use libcras::CrasSocketType;
#[cfg(all(unix, feature = "audio_cras"))]
use libcras::CrasStreamType;
use serde::Deserialize;
use serde::Serialize;
use serde_keyvalue::FromKeyValues;
use thiserror::Error as ThisError;

use crate::virtio::snd::constants::*;
use crate::virtio::snd::layout::*;
use crate::virtio::snd::sys::StreamSourceBackend as SysStreamSourceBackend;

#[derive(ThisError, Debug, PartialEq, Eq)]
pub enum Error {
    /// Unknown snd parameter value.
    #[error("Invalid snd parameter value ({0}): {1}")]
    InvalidParameterValue(String, String),
    /// Failed to parse bool value.
    #[error("Invalid bool value: {0}")]
    InvalidBoolValue(ParseBoolError),
    /// Failed to parse int value.
    #[error("Invalid int value: {0}")]
    InvalidIntValue(ParseIntError),
    // Invalid backend.
    #[error("Backend is not implemented")]
    InvalidBackend,
    /// Failed to parse parameters.
    #[error("Invalid snd parameter: {0}")]
    UnknownParameter(String),
    /// Invalid PCM device config index. Happens when the length of PCM device config is less than
    /// the number of PCM devices.
    #[error("Invalid PCM device config index: {0}")]
    InvalidPCMDeviceConfigIndex(usize),
    /// Invalid PCM info direction (VIRTIO_SND_D_OUTPUT = 0, VIRTIO_SND_D_INPUT = 1)
    #[error("Invalid PCM Info direction: {0}")]
    InvalidPCMInfoDirection(u8),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Serialize)]
#[serde(into = "String", try_from = "&str")]
pub enum StreamSourceBackend {
    NULL,
    FILE,
    Sys(SysStreamSourceBackend),
}

// Implemented to make backend serialization possible, since we deserialize from str.
impl From<StreamSourceBackend> for String {
    fn from(backend: StreamSourceBackend) -> Self {
        match backend {
            StreamSourceBackend::NULL => "null".to_owned(),
            StreamSourceBackend::FILE => "file".to_owned(),
            StreamSourceBackend::Sys(sys_backend) => sys_backend.into(),
        }
    }
}

impl TryFrom<&str> for StreamSourceBackend {
    type Error = Error;

    fn try_from(s: &str) -> Result<Self, Self::Error> {
        match s {
            "null" => Ok(StreamSourceBackend::NULL),
            "file" => Ok(StreamSourceBackend::FILE),
            _ => SysStreamSourceBackend::try_from(s).map(StreamSourceBackend::Sys),
        }
    }
}

/// What the device plays when the guest has not queued a period in time.
///
/// An underrun is a hole in the stream, and the hole is what the listener hears as a click.
/// Filling it with silence is honest but maximally audible; holding the previous audio hides
/// short holes, which is the trade a low-latency configuration wants to make -- fewer periods
/// in flight means more underruns by construction.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Serialize, Default)]
#[serde(rename_all = "snake_case", try_from = "&str", into = "String")]
pub enum UnderrunMode {
    /// Write a period of zeroes. The upstream behaviour, and the default.
    #[default]
    Silence,
    /// Repeat the tail of the last good period, pitch-aligned and fading out.
    Repeat,
}

impl From<UnderrunMode> for String {
    fn from(mode: UnderrunMode) -> Self {
        match mode {
            UnderrunMode::Silence => "silence".to_owned(),
            UnderrunMode::Repeat => "repeat".to_owned(),
        }
    }
}

impl TryFrom<&str> for UnderrunMode {
    type Error = Error;

    fn try_from(s: &str) -> Result<Self, Self::Error> {
        match s {
            "silence" => Ok(UnderrunMode::Silence),
            "repeat" => Ok(UnderrunMode::Repeat),
            _ => Err(Error::InvalidParameterValue(
                s.to_owned(),
                "expected silence or repeat".to_owned(),
            )),
        }
    }
}

/// Holds the parameters for each PCM device
#[derive(Debug, Clone, Default, Deserialize, Serialize, FromKeyValues, PartialEq, Eq)]
#[serde(deny_unknown_fields, default)]
pub struct PCMDeviceParameters {
    #[cfg(all(unix, feature = "audio_cras"))]
    pub client_type: Option<CrasClientType>,
    #[cfg(all(unix, feature = "audio_cras"))]
    pub stream_type: Option<CrasStreamType>,
    pub effects: Option<Vec<StreamEffect>>,
    /// Host endpoint this PCM device plays to / records from, as an Android `AudioDeviceInfo`
    /// id. Only the aaudio backend honours it; `None` (or 0) leaves the routing to the
    /// platform. The ids are assigned per boot, so whoever writes this has to resolve a stable
    /// descriptor to a live id at VM start.
    pub device_id: Option<i32>,
    /// What this PCM device's host endpoint runs at natively, in Hz. Published to the guest as
    /// a hint so its mixer can land on that rate instead of one the host would have to convert.
    /// `None` (or 0) says nothing and the guest keeps its own default.
    pub preferred_rate: Option<u32>,
    /// The same for the endpoint's channel count.
    pub preferred_channels: Option<u8>,
    /// What kind of thing the host endpoint is: 1 speaker, 2 headphones, 3 headset, 4 line out,
    /// 5 digital, 6 microphone, 7 telephony. 0 or unset says nothing.
    pub endpoint_kind: Option<u32>,
    /// The host endpoint by a name that survives it being unplugged: `TYPE|address`, as
    /// Android's `AudioDeviceInfo` reports them.
    ///
    /// Preferred over `device_id`, which is only today's number for it -- reconnect a headset
    /// and the number is different while the name is not. Requires `device_table`.
    pub host_device: Option<String>,
}

/// Holds the parameters for a cras sound device
#[derive(Debug, Clone, Deserialize, Serialize, FromKeyValues)]
#[serde(deny_unknown_fields, default)]
pub struct Parameters {
    pub capture: bool,
    pub num_output_devices: u32,
    pub num_input_devices: u32,
    pub backend: StreamSourceBackend,
    pub num_output_streams: u32,
    pub num_input_streams: u32,
    pub playback_path: String,
    pub playback_size: usize,
    #[cfg(all(unix, feature = "audio_cras"))]
    #[serde(deserialize_with = "libcras::deserialize_cras_client_type")]
    pub client_type: CrasClientType,
    #[cfg(all(unix, feature = "audio_cras"))]
    pub socket_type: CrasSocketType,
    pub output_device_config: Vec<PCMDeviceParameters>,
    pub input_device_config: Vec<PCMDeviceParameters>,
    pub card_index: usize,
    /// What to play on underrun. See [`UnderrunMode`].
    /// Where the platform's current audio device list is published, for resolving
    /// `host_device`. Empty when nobody publishes one, and then only `device_id` works.
    pub device_table: String,
    pub underrun: UnderrunMode,
    /// Hint published to the guest (see the device's vendor config block): how many periods it
    /// should try to keep in flight. 0 leaves the decision to the guest driver.
    ///
    /// This is the latency knob, and it can only be a hint -- the device cannot make a driver
    /// queue more than it wants to, and the driver must never send a period the OS has not
    /// written yet. A driver that ignores the block keeps its own default.
    pub guest_outstanding_packets: u32,
    /// Hint published to the guest: preferred period size in bytes. 0 means no preference.
    pub guest_period_bytes: u32,
    /// Run this device's backend in its own process under this uid.
    ///
    /// Named to match `--shared-dir` and `--pmem-ext2`, which already take a `uid`/`gid` for the
    /// process a device runs as. The meaning here is the same -- the identity the device's own
    /// process has -- with one difference worth knowing: theirs is applied through the sandbox
    /// and so does nothing at all when sandboxing is off, while this is a real host uid and
    /// always applies. Android decides whether an audio stream is audible from the real uid, so
    /// a namespace-relative one would not do.
    ///
    /// `None` keeps the device in the VMM's own process, which on Android means silence: the VMM
    /// is root and the platform mutes uid 0 in both directions.
    pub uid: Option<u32>,
    /// Group for that process. Defaults to `uid` when unset.
    pub gid: Option<u32>,
    /// Supplementary groups for that process.
    ///
    /// Empty drops all of them, which is the safe default: the VMM's own groups are root's, and
    /// carrying any of them into an unprivileged process would be a privilege leak. Anything the
    /// backend genuinely needs has to be named here by whoever knows -- the VMM cannot work it
    /// out, and guessing would defeat the point of dropping them.
    #[serde(default)]
    pub supp_gids: Vec<u32>,
}

impl Default for Parameters {
    fn default() -> Self {
        Parameters {
            capture: false,
            num_output_devices: 1,
            num_input_devices: 1,
            backend: StreamSourceBackend::NULL,
            num_output_streams: 1,
            num_input_streams: 1,
            playback_path: "".to_string(),
            playback_size: 0,
            #[cfg(all(unix, feature = "audio_cras"))]
            client_type: CrasClientType::CRAS_CLIENT_TYPE_CROSVM,
            #[cfg(all(unix, feature = "audio_cras"))]
            socket_type: CrasSocketType::Unified,
            output_device_config: vec![],
            input_device_config: vec![],
            card_index: 0,
            device_table: String::new(),
            underrun: UnderrunMode::Silence,
            guest_outstanding_packets: 0,
            guest_period_bytes: 0,
            uid: None,
            gid: None,
            supp_gids: Vec::new(),
        }
    }
}

impl Parameters {
    pub(crate) fn get_total_output_streams(&self) -> u32 {
        self.num_output_devices * self.num_output_streams
    }

    pub(crate) fn get_total_input_streams(&self) -> u32 {
        self.num_input_devices * self.num_input_streams
    }

    pub(crate) fn get_total_streams(&self) -> u32 {
        self.get_total_output_streams() + self.get_total_input_streams()
    }

    #[allow(dead_code)]
    pub(crate) fn get_device_params(
        &self,
        pcm_info: &virtio_snd_pcm_info,
    ) -> Result<PCMDeviceParameters, Error> {
        let device_config = match pcm_info.direction {
            VIRTIO_SND_D_OUTPUT => &self.output_device_config,
            VIRTIO_SND_D_INPUT => &self.input_device_config,
            _ => return Err(Error::InvalidPCMInfoDirection(pcm_info.direction)),
        };
        let device_idx = u32::from(pcm_info.hdr.hda_fn_nid) as usize;
        device_config
            .get(device_idx)
            .cloned()
            .ok_or(Error::InvalidPCMDeviceConfigIndex(device_idx))
    }
}

#[cfg(test)]
#[allow(clippy::needless_update)]
mod tests {
    use super::*;

    fn check_failure(s: &str) {
        serde_keyvalue::from_key_values::<Parameters>(s).expect_err("parse should have failed");
    }

    fn check_success(
        s: &str,
        capture: bool,
        backend: StreamSourceBackend,
        num_output_devices: u32,
        num_input_devices: u32,
        num_output_streams: u32,
        num_input_streams: u32,
        output_device_config: Vec<PCMDeviceParameters>,
        input_device_config: Vec<PCMDeviceParameters>,
    ) {
        let params: Parameters =
            serde_keyvalue::from_key_values(s).expect("parse should have succeded");
        assert_eq!(params.capture, capture);
        assert_eq!(params.backend, backend);
        assert_eq!(params.num_output_devices, num_output_devices);
        assert_eq!(params.num_input_devices, num_input_devices);
        assert_eq!(params.num_output_streams, num_output_streams);
        assert_eq!(params.num_input_streams, num_input_streams);
        assert_eq!(params.output_device_config, output_device_config);
        assert_eq!(params.input_device_config, input_device_config);
    }

    #[test]
    fn parameters_fromstr() {
        check_failure("capture=none");
        check_success(
            "capture=false",
            false,
            StreamSourceBackend::NULL,
            1,
            1,
            1,
            1,
            vec![],
            vec![],
        );
        check_success(
            "capture=true,num_output_streams=2,num_input_streams=3",
            true,
            StreamSourceBackend::NULL,
            1,
            1,
            2,
            3,
            vec![],
            vec![],
        );
        check_success(
            "capture=true,num_output_devices=3,num_input_devices=2",
            true,
            StreamSourceBackend::NULL,
            3,
            2,
            1,
            1,
            vec![],
            vec![],
        );
        check_success(
            "capture=true,num_output_devices=2,num_input_devices=3,\
            num_output_streams=3,num_input_streams=2",
            true,
            StreamSourceBackend::NULL,
            2,
            3,
            3,
            2,
            vec![],
            vec![],
        );
        check_success(
            "capture=true,backend=null,num_output_devices=2,num_input_devices=3,\
            num_output_streams=3,num_input_streams=2",
            true,
            StreamSourceBackend::NULL,
            2,
            3,
            3,
            2,
            vec![],
            vec![],
        );
        check_success(
            "output_device_config=[[effects=[aec]],[]]",
            false,
            StreamSourceBackend::NULL,
            1,
            1,
            1,
            1,
            vec![
                PCMDeviceParameters {
                    effects: Some(vec![StreamEffect::EchoCancellation]),
                    ..Default::default()
                },
                Default::default(),
            ],
            vec![],
        );
        check_success(
            "input_device_config=[[effects=[aec]],[]]",
            false,
            StreamSourceBackend::NULL,
            1,
            1,
            1,
            1,
            vec![],
            vec![
                PCMDeviceParameters {
                    effects: Some(vec![StreamEffect::EchoCancellation]),
                    ..Default::default()
                },
                Default::default(),
            ],
        );

        // Invalid effect in device config
        check_failure("output_device_config=[[effects=[none]]]");
    }

    #[test]
    #[cfg(all(unix, feature = "audio_cras"))]
    fn cras_parameters_fromstr() {
        fn cras_check_success(
            s: &str,
            backend: StreamSourceBackend,
            client_type: CrasClientType,
            socket_type: CrasSocketType,
            output_device_config: Vec<PCMDeviceParameters>,
            input_device_config: Vec<PCMDeviceParameters>,
        ) {
            let params: Parameters =
                serde_keyvalue::from_key_values(s).expect("parse should have succeded");
            assert_eq!(params.backend, backend);
            assert_eq!(params.client_type, client_type);
            assert_eq!(params.socket_type, socket_type);
            assert_eq!(params.output_device_config, output_device_config);
            assert_eq!(params.input_device_config, input_device_config);
        }

        cras_check_success(
            "backend=cras",
            StreamSourceBackend::Sys(SysStreamSourceBackend::CRAS),
            CrasClientType::CRAS_CLIENT_TYPE_CROSVM,
            CrasSocketType::Unified,
            vec![],
            vec![],
        );
        cras_check_success(
            "backend=cras,client_type=crosvm",
            StreamSourceBackend::Sys(SysStreamSourceBackend::CRAS),
            CrasClientType::CRAS_CLIENT_TYPE_CROSVM,
            CrasSocketType::Unified,
            vec![],
            vec![],
        );
        cras_check_success(
            "backend=cras,client_type=arcvm",
            StreamSourceBackend::Sys(SysStreamSourceBackend::CRAS),
            CrasClientType::CRAS_CLIENT_TYPE_ARCVM,
            CrasSocketType::Unified,
            vec![],
            vec![],
        );
        check_failure("backend=cras,client_type=none");
        cras_check_success(
            "backend=cras,socket_type=legacy",
            StreamSourceBackend::Sys(SysStreamSourceBackend::CRAS),
            CrasClientType::CRAS_CLIENT_TYPE_CROSVM,
            CrasSocketType::Legacy,
            vec![],
            vec![],
        );
        cras_check_success(
            "backend=cras,socket_type=unified",
            StreamSourceBackend::Sys(SysStreamSourceBackend::CRAS),
            CrasClientType::CRAS_CLIENT_TYPE_CROSVM,
            CrasSocketType::Unified,
            vec![],
            vec![],
        );
        cras_check_success(
            "output_device_config=[[client_type=crosvm],[client_type=arcvm,stream_type=pro_audio],[]]",
            StreamSourceBackend::NULL,
            CrasClientType::CRAS_CLIENT_TYPE_CROSVM,
            CrasSocketType::Unified,
            vec![
                PCMDeviceParameters{
                    client_type: Some(CrasClientType::CRAS_CLIENT_TYPE_CROSVM),
                    stream_type: None,
                    effects: None,
                    device_id: None,
                },
                PCMDeviceParameters{
                    client_type: Some(CrasClientType::CRAS_CLIENT_TYPE_ARCVM),
                    stream_type: Some(CrasStreamType::CRAS_STREAM_TYPE_PRO_AUDIO),
                    effects: None,
                    device_id: None,
                },
                Default::default(),
                ],
            vec![],
        );
        cras_check_success(
            "input_device_config=[[client_type=crosvm],[client_type=arcvm,effects=[aec],stream_type=pro_audio],[effects=[EchoCancellation]],[]]",
            StreamSourceBackend::NULL,
            CrasClientType::CRAS_CLIENT_TYPE_CROSVM,
            CrasSocketType::Unified,
            vec![],
            vec![
                PCMDeviceParameters{
                    client_type: Some(CrasClientType::CRAS_CLIENT_TYPE_CROSVM),
                    stream_type: None,
                    effects: None,
                    device_id: None,
                },
                PCMDeviceParameters{
                    client_type: Some(CrasClientType::CRAS_CLIENT_TYPE_ARCVM),
                    stream_type: Some(CrasStreamType::CRAS_STREAM_TYPE_PRO_AUDIO),
                    effects: Some(vec![StreamEffect::EchoCancellation]),
                    device_id: None,
                },
                PCMDeviceParameters{
                    client_type: None,
                    stream_type: None,
                    effects: Some(vec![StreamEffect::EchoCancellation]),
                    device_id: None,
                },
                Default::default(),
                ],
        );

        // Invalid client_type in device config
        check_failure("output_device_config=[[client_type=none]]");

        // Invalid stream type in device config
        check_failure("output_device_config=[[stream_type=none]]");
    }

    #[test]
    fn get_device_params_output() {
        let params = Parameters {
            output_device_config: vec![
                PCMDeviceParameters {
                    effects: Some(vec![StreamEffect::EchoCancellation]),
                    ..Default::default()
                },
                PCMDeviceParameters {
                    effects: Some(vec![
                        StreamEffect::EchoCancellation,
                        StreamEffect::EchoCancellation,
                    ]),
                    ..Default::default()
                },
            ],
            ..Default::default()
        };

        let default_pcm_info = virtio_snd_pcm_info {
            hdr: virtio_snd_info {
                hda_fn_nid: 0.into(),
            },
            features: 0.into(),
            formats: 0.into(),
            rates: 0.into(),
            direction: VIRTIO_SND_D_OUTPUT, // Direction is OUTPUT
            channels_min: 1,
            channels_max: 6,
            padding: [0; 5],
        };

        let mut pcm_info = default_pcm_info;
        pcm_info.hdr.hda_fn_nid = 0.into();
        assert_eq!(
            params.get_device_params(&pcm_info),
            Ok(params.output_device_config[0].clone())
        );

        let mut pcm_info = default_pcm_info;
        pcm_info.hdr.hda_fn_nid = 1.into();
        assert_eq!(
            params.get_device_params(&pcm_info),
            Ok(params.output_device_config[1].clone())
        );

        let mut pcm_info = default_pcm_info;
        pcm_info.hdr.hda_fn_nid = 2.into();
        assert_eq!(
            params.get_device_params(&pcm_info),
            Err(Error::InvalidPCMDeviceConfigIndex(2))
        );
    }

    #[test]
    fn get_device_params_input() {
        let params = Parameters {
            input_device_config: vec![
                PCMDeviceParameters {
                    effects: Some(vec![
                        StreamEffect::EchoCancellation,
                        StreamEffect::EchoCancellation,
                    ]),
                    ..Default::default()
                },
                PCMDeviceParameters {
                    effects: Some(vec![StreamEffect::EchoCancellation]),
                    ..Default::default()
                },
            ],
            ..Default::default()
        };

        let default_pcm_info = virtio_snd_pcm_info {
            hdr: virtio_snd_info {
                hda_fn_nid: 0.into(),
            },
            features: 0.into(),
            formats: 0.into(),
            rates: 0.into(),
            direction: VIRTIO_SND_D_INPUT, // Direction is INPUT
            channels_min: 1,
            channels_max: 6,
            padding: [0; 5],
        };

        let mut pcm_info = default_pcm_info;
        pcm_info.hdr.hda_fn_nid = 0.into();
        assert_eq!(
            params.get_device_params(&pcm_info),
            Ok(params.input_device_config[0].clone())
        );

        let mut pcm_info = default_pcm_info;
        pcm_info.hdr.hda_fn_nid = 1.into();
        assert_eq!(
            params.get_device_params(&pcm_info),
            Ok(params.input_device_config[1].clone())
        );

        let mut pcm_info = default_pcm_info;
        pcm_info.hdr.hda_fn_nid = 2.into();
        assert_eq!(
            params.get_device_params(&pcm_info),
            Err(Error::InvalidPCMDeviceConfigIndex(2))
        );
    }

    #[test]
    fn get_device_params_invalid_direction() {
        let params = Parameters::default();

        let pcm_info = virtio_snd_pcm_info {
            hdr: virtio_snd_info {
                hda_fn_nid: 0.into(),
            },
            features: 0.into(),
            formats: 0.into(),
            rates: 0.into(),
            direction: 2, // Invalid direction
            channels_min: 1,
            channels_max: 6,
            padding: [0; 5],
        };

        assert_eq!(
            params.get_device_params(&pcm_info),
            Err(Error::InvalidPCMInfoDirection(2))
        );
    }
}
