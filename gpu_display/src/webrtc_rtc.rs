// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

//! The slice of libdatachannel's C API (third_party/libdatachannel/include/rtc/rtc.h) the WebRTC
//! sink uses. Mirrors, field for field and in order: rtc.h is the original, and the library is
//! built with RTC_ENABLE_MEDIA=1, which is what makes the media half of it exist.

#![allow(non_camel_case_types, dead_code)]

use std::ffi::c_char;
use std::ffi::c_int;
use std::ffi::c_uint;
use std::ffi::c_void;

pub type rtcState = c_int;
pub const RTC_CONNECTED: rtcState = 2;
pub const RTC_DISCONNECTED: rtcState = 3;
pub const RTC_FAILED: rtcState = 4;
pub const RTC_CLOSED: rtcState = 5;

pub type rtcGatheringState = c_int;
pub const RTC_GATHERING_COMPLETE: rtcGatheringState = 2;

pub type rtcLogLevel = c_int;
pub const RTC_LOG_WARNING: rtcLogLevel = 3;

pub type rtcCodec = c_int;
pub const RTC_CODEC_H264: rtcCodec = 0;
pub const RTC_CODEC_H265: rtcCodec = 3;

pub type rtcDirection = c_int;
pub const RTC_DIRECTION_SENDONLY: rtcDirection = 1;

pub type rtcNalUnitSeparator = c_int;
pub const RTC_NAL_SEPARATOR_START_SEQUENCE: rtcNalUnitSeparator = 3;

#[repr(C)]
pub struct rtcConfiguration {
    pub ice_servers: *const *const c_char,
    pub ice_servers_count: c_int,
    pub proxy_server: *const c_char,
    pub bind_address: *const c_char,
    pub certificate_type: c_int,
    pub ice_transport_policy: c_int,
    pub enable_ice_tcp: bool,
    pub enable_ice_udp_mux: bool,
    pub disable_auto_negotiation: bool,
    pub force_media_transport: bool,
    pub port_range_begin: u16,
    pub port_range_end: u16,
    pub mtu: c_int,
    pub max_message_size: c_int,
}

#[repr(C)]
pub struct rtcTrackInit {
    pub direction: rtcDirection,
    pub codec: rtcCodec,
    pub payload_type: c_int,
    pub ssrc: u32,
    pub mid: *const c_char,
    pub name: *const c_char,
    pub msid: *const c_char,
    pub track_id: *const c_char,
    pub profile: *const c_char,
}

#[repr(C)]
pub struct rtcPacketizerInit {
    pub ssrc: u32,
    pub cname: *const c_char,
    pub payload_type: u8,
    pub clock_rate: u32,
    pub sequence_number: u16,
    pub timestamp: u32,
    pub max_fragment_size: u16,
    pub nal_separator: rtcNalUnitSeparator,
    pub obu_packetization: c_int,
    pub playout_delay_id: u8,
    pub playout_delay_min: u16,
    pub playout_delay_max: u16,
    pub color_space_id: u8,
    pub color_chroma_siting_horz: u8,
    pub color_chroma_siting_vert: u8,
    pub color_range: u8,
    pub color_primaries: u8,
    pub color_transfer: u8,
    pub color_matrix: u8,
    pub abs_send_time_id: u8,
}

impl rtcPacketizerInit {
    /// Every field at its "not set" value, which rtc.h defines as zero throughout (null cname,
    /// default fragment size, no playout-delay or colour-space extension).
    pub fn zeroed() -> rtcPacketizerInit {
        // SAFETY: plain integers and one nullable pointer; all-zero is a valid value of each.
        unsafe { std::mem::zeroed() }
    }
}

pub type rtcLogCallbackFunc = extern "C" fn(level: rtcLogLevel, message: *const c_char);
pub type rtcStateChangeCallbackFunc = extern "C" fn(pc: c_int, state: rtcState, ptr: *mut c_void);
pub type rtcGatheringStateCallbackFunc =
    extern "C" fn(pc: c_int, state: rtcGatheringState, ptr: *mut c_void);
pub type rtcOpenCallbackFunc = extern "C" fn(id: c_int, ptr: *mut c_void);
pub type rtcClosedCallbackFunc = extern "C" fn(id: c_int, ptr: *mut c_void);
pub type rtcMessageCallbackFunc =
    extern "C" fn(id: c_int, message: *const c_char, size: c_int, ptr: *mut c_void);
pub type rtcPliHandlerCallbackFunc = extern "C" fn(tr: c_int, ptr: *mut c_void);
pub type rtcRembHandlerCallbackFunc = extern "C" fn(tr: c_int, bitrate: c_uint, ptr: *mut c_void);

extern "C" {
    pub fn rtcInitLogger(level: rtcLogLevel, cb: Option<rtcLogCallbackFunc>);
    pub fn rtcSetUserPointer(id: c_int, ptr: *mut c_void);

    pub fn rtcCreatePeerConnection(config: *const rtcConfiguration) -> c_int;
    pub fn rtcDeletePeerConnection(pc: c_int) -> c_int;
    pub fn rtcSetStateChangeCallback(pc: c_int, cb: Option<rtcStateChangeCallbackFunc>) -> c_int;
    pub fn rtcSetGatheringStateChangeCallback(
        pc: c_int,
        cb: Option<rtcGatheringStateCallbackFunc>,
    ) -> c_int;
    pub fn rtcSetLocalDescription(pc: c_int, type_: *const c_char) -> c_int;
    pub fn rtcSetRemoteDescription(pc: c_int, sdp: *const c_char, type_: *const c_char) -> c_int;
    pub fn rtcGetLocalDescription(pc: c_int, buffer: *mut c_char, size: c_int) -> c_int;

    pub fn rtcSetOpenCallback(id: c_int, cb: Option<rtcOpenCallbackFunc>) -> c_int;
    pub fn rtcSetClosedCallback(id: c_int, cb: Option<rtcClosedCallbackFunc>) -> c_int;
    pub fn rtcSetMessageCallback(id: c_int, cb: Option<rtcMessageCallbackFunc>) -> c_int;
    pub fn rtcSendMessage(id: c_int, data: *const c_char, size: c_int) -> c_int;
    pub fn rtcIsOpen(id: c_int) -> bool;

    pub fn rtcCreateDataChannel(pc: c_int, label: *const c_char) -> c_int;

    pub fn rtcAddTrack(pc: c_int, media_description_sdp: *const c_char) -> c_int;
    pub fn rtcAddTrackEx(pc: c_int, init: *const rtcTrackInit) -> c_int;
    pub fn rtcSetH264Packetizer(tr: c_int, init: *const rtcPacketizerInit) -> c_int;
    pub fn rtcSetH265Packetizer(tr: c_int, init: *const rtcPacketizerInit) -> c_int;
    pub fn rtcChainRtcpSrReporter(tr: c_int) -> c_int;
    pub fn rtcChainRtcpNackResponder(tr: c_int, max_stored_packets_count: c_uint) -> c_int;
    pub fn rtcChainPliHandler(tr: c_int, cb: Option<rtcPliHandlerCallbackFunc>) -> c_int;
    pub fn rtcChainRembHandler(tr: c_int, cb: Option<rtcRembHandlerCallbackFunc>) -> c_int;
    pub fn rtcSetTrackRtpTimestamp(id: c_int, timestamp: u32) -> c_int;
}
