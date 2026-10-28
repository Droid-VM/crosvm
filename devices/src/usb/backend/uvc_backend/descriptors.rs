// Copyright 2026 The crosvm Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! Descriptors of the virtual USB Video Class (UVC 1.1) camera backend.
//!
//! The device is a high speed, bulk only MJPEG camera. Interface 0 is the
//! VideoControl interface (camera terminal, processing unit and streaming
//! output terminal), interface 1 alternate setting 1 is the VideoStreaming
//! interface carrying a single bulk IN endpoint.

/// Size of the UVC 1.1 probe/commit control structure in bytes.
pub const PROBE_LEN: usize = 34;

pub const VENDOR_ID: u16 = 0x0525;
pub const PRODUCT_ID: u16 = 0xa4a2;

/// Configuration value of the single configuration.
pub const CONFIG_VALUE: u8 = 1;

/// Native capture size, and the largest frame advertised to the guest. 1080p
/// frames from the camera HAL are around one megabyte each, which both the HAL
/// encoder and the emulated xHCI path struggle to sustain, so the largest size
/// stays at 720p until the host can follow the negotiated size.
pub const FRAME_WIDTH: u16 = 1920;
pub const FRAME_HEIGHT: u16 = 1080;
/// Advertised frame sizes, smallest first; the last one is the default.
pub const FRAMES: [(u16, u16); 3] = [(640, 480), (1280, 720), (FRAME_WIDTH, FRAME_HEIGHT)];
/// 60 frames per second, in 100ns units.
pub const FRAME_INTERVAL_60: u32 = 166_666;
/// 30 frames per second, in 100ns units.
pub const FRAME_INTERVAL_30: u32 = 333_333;
pub const FRAME_INTERVALS: [u32; 2] = [FRAME_INTERVAL_60, FRAME_INTERVAL_30];
/// Default frame interval, used until the host negotiates something else.
pub const FRAME_INTERVAL: u32 = FRAME_INTERVAL_60;
/// Maximum number of payload bytes carried by a single bulk transfer. The
/// endpoint itself keeps a 512 byte packet size (USB 2.0 high speed limit), but
/// the host may chain several packets into one bulk URB, so a larger value here
/// reduces the number of round trips needed for one frame.
pub const MAX_PAYLOAD_TRANSFER_SIZE: u32 = 262144;
/// Upper bound of the compressed frame size, as reported to the guest.
pub const MAX_VIDEO_FRAME_SIZE: u32 = FRAME_WIDTH as u32 * FRAME_HEIGHT as u32 * 2;
pub const MIN_BIT_RATE: u32 = 800_000;
pub const MAX_BIT_RATE: u32 = 147_456_000;

pub const CONTROL_INTERFACE: u8 = 0;
pub const STREAM_INTERFACE: u8 = 1;
pub const BULK_IN_ENDPOINT: u8 = 0x81;
/// When true the bulk endpoint is declared in streaming alternate setting 0
/// instead of a separate alternate setting 1. Windows then never has to switch
/// the interface setting before streaming.
pub const ENDPOINT_IN_ALT_ZERO: bool = true;
/// Status interrupt endpoint of the VideoControl interface.
pub const STATUS_ENDPOINT: u8 = 0x82;
pub const CAMERA_TERMINAL_ID: u8 = 1;
pub const PROCESSING_UNIT_ID: u8 = 2;
pub const OUTPUT_TERMINAL_ID: u8 = 3;

/// UVC class specific bRequest values.
pub mod request {
    pub const SET_CUR: u8 = 0x01;
    pub const GET_CUR: u8 = 0x81;
    pub const GET_MIN: u8 = 0x82;
    pub const GET_MAX: u8 = 0x83;
    pub const GET_RES: u8 = 0x84;
    pub const GET_LEN: u8 = 0x85;
    pub const GET_INFO: u8 = 0x86;
    pub const GET_DEF: u8 = 0x87;
}

/// UVC control selectors, in the high byte of wValue.
pub mod selector {
    pub const PROBE: u8 = 0x01;
    pub const COMMIT: u8 = 0x02;
}

/// The JPEG test pattern sent as video payload while the camera is streaming.
pub const TEST_PATTERN: &[u8] = include_bytes!("test_pattern.jpg");

/// USB device descriptor. All string indexes are zero: the device carries no
/// strings, which keeps enumeration simple.
///
/// The bcdDevice minor version carries the camera slot, so a VM that exposes two
/// cameras yields two devices the guest can tell apart instead of two bit
/// identical ones.
pub const fn device_descriptor(slot: usize) -> [u8; 18] {
    [
        18,   // bLength
        0x01, // bDescriptorType = DEVICE
        0x00, 0x02, // bcdUSB = 2.00
        0xef, // bDeviceClass = Miscellaneous
        0x02, // bDeviceSubClass = Common Class
        0x01, // bDeviceProtocol = Interface Association Descriptor
        64,   // bMaxPacketSize0
        (VENDOR_ID & 0xff) as u8,
        (VENDOR_ID >> 8) as u8,
        (PRODUCT_ID & 0xff) as u8,
        (PRODUCT_ID >> 8) as u8,
        (slot & 0x3f) as u8, 0x01, // bcdDevice = 1.<slot>
        0,    // iManufacturer
        0,    // iProduct
        0,    // iSerialNumber
        1,    // bNumConfigurations
    ]
}

/// Builds the UVC 1.1 probe/commit structure with the current defaults.
pub fn default_probe() -> [u8; PROBE_LEN] {
    let mut probe = [0u8; PROBE_LEN];
    probe[2] = 1; // bFormatIndex
    probe[3] = 1; // bFrameIndex
    probe[4..8].copy_from_slice(&FRAME_INTERVAL.to_le_bytes());
    probe[18..22].copy_from_slice(&MAX_VIDEO_FRAME_SIZE.to_le_bytes());
    probe[22..26].copy_from_slice(&MAX_PAYLOAD_TRANSFER_SIZE.to_le_bytes());
    // dwClockFrequency stays zero: the camera is not locked to a clock.
    probe[30] = 0; // bmFramingInfo
    probe[31] = 0x10; // bPreferredVersion
    probe[32] = 0x10; // bMinVersion
    probe[33] = 0x10; // bMaxVersion
    probe
}

fn u16le(value: u16) -> [u8; 2] {
    value.to_le_bytes()
}

fn u32le(value: u32) -> [u8; 4] {
    value.to_le_bytes()
}

/// Builds the configuration descriptor set, computing every length field and
/// the class specific headers' wTotalLength values from the descriptors that
/// actually follow them.
pub fn config_descriptor() -> Vec<u8> {
    let mut config = Vec::new();

    // Configuration descriptor. wTotalLength is patched at the end.
    config.extend_from_slice(&[
        9,          // bLength
        0x02,       // bDescriptorType = CONFIGURATION
        0x00, 0x00, // wTotalLength, patched below
        2,          // bNumInterfaces
        CONFIG_VALUE, // bConfigurationValue
        0,          // iConfiguration
        0x80,       // bmAttributes = bus powered
        0x32,       // bMaxPower = 100mA
    ]);

    // Interface Association Descriptor tying the two video interfaces into one
    // function.
    config.extend_from_slice(&[
        8,    // bLength
        0x0b, // bDescriptorType = INTERFACE_ASSOCIATION
        CONTROL_INTERFACE,
        2,    // bInterfaceCount
        0x0e, // bFunctionClass = Video
        0x03, // bFunctionSubClass = Video Interface Collection
        0x00, // bFunctionProtocol
        0,    // iFunction
    ]);

    // VideoControl interface, alternate setting 0, status interrupt endpoint.
    config.extend_from_slice(&[
        9, // bLength
        0x04, CONTROL_INTERFACE, 0, 1, // bInterfaceNumber/alt/bNumEndpoints
        0x0e, 0x01, 0x00, // Video, VideoControl, no protocol
        0,    // iInterface
    ]);

    let mut control_specific = Vec::new();

    // VC Interface Header, 13 bytes, wTotalLength patched below.
    control_specific.extend_from_slice(&[
        13,   // bLength
        0x24, // bDescriptorType = CS_INTERFACE
        0x01, // bDescriptorSubtype = VC_HEADER
        0x10, 0x01, // bcdUVC = 1.10
        0x00, 0x00, // wTotalLength, patched below
        0x00, 0x6c, 0xdc, 0x02, // dwClockFrequency = 48 MHz
        1, // bInCollection
        STREAM_INTERFACE, // baInterfaceNr[0]
    ]);

    // Input Terminal: the camera sensor itself.
    control_specific.extend_from_slice(&[
        18,   // bLength
        0x24, // CS_INTERFACE
        0x02, // VC_INPUT_TERMINAL
        CAMERA_TERMINAL_ID, // bTerminalID
        0x01, 0x02, // wTerminalType = ITT_CAMERA
        0,    // bAssocTerminal
        0,    // iTerminal
        0x00, 0x00, // wObjectiveFocalLengthMin
        0x00, 0x00, // wObjectiveFocalLengthMax
        0x00, 0x00, // wOcularFocalLength
        3,    // bControlSize
        0, 0, 0, // bmControls
    ]);

    // Processing Unit.
    control_specific.extend_from_slice(&[
        13,   // bLength
        0x24, // CS_INTERFACE
        0x05, // VC_PROCESSING_UNIT
        PROCESSING_UNIT_ID, // bUnitID
        CAMERA_TERMINAL_ID, // bSourceID
        0x00, 0x00, // wMaxMultiplier
        3,    // bControlSize
        0, 0, 0, // bmControls
        0,    // iProcessing
        0,    // bmVideoStandards
    ]);

    // Output Terminal: the USB streaming pipe.
    control_specific.extend_from_slice(&[
        9,    // bLength
        0x24, // CS_INTERFACE
        0x03, // VC_OUTPUT_TERMINAL
        OUTPUT_TERMINAL_ID, // bTerminalID
        0x01, 0x01, // wTerminalType = TT_STREAMING
        0,    // bAssocTerminal
        PROCESSING_UNIT_ID, // bSourceID
        0,    // iTerminal
    ]);

    // Patch the VC header's wTotalLength with the real size of the block.
    let vc_total = control_specific.len() as u16;
    control_specific[5..7].copy_from_slice(&u16le(vc_total));
    config.extend_from_slice(&control_specific);

    // Status interrupt endpoint of the VideoControl interface, carrying class
    // specific event notifications (UVC 1.1 sections 3.8 and 3.9).
    config.extend_from_slice(&[
        7,    // bLength
        0x05, // bDescriptorType = ENDPOINT
        STATUS_ENDPOINT,
        0x03, // bmAttributes = interrupt
        0x10, 0x00, // wMaxPacketSize = 16
        8,    // bInterval = 8 frames
    ]);

    // VideoStreaming interface, alternate setting 0.
    config.extend_from_slice(&[
        9, // bLength
        0x04,
        STREAM_INTERFACE,
        0,
        if ENDPOINT_IN_ALT_ZERO { 1 } else { 0 }, // bNumEndpoints
        0x0e, 0x02, 0x00, // Video, VideoStreaming, no protocol
        0,    // iInterface
    ]);

    let mut stream_specific = Vec::new();

    // VS Input Header, 14 bytes, wTotalLength patched below.
    stream_specific.extend_from_slice(&[
        14,   // bLength
        0x24, // CS_INTERFACE
        0x01, // VS_INPUT_HEADER
        1,    // bNumFormats
        0x00, 0x00, // wTotalLength, patched below
        BULK_IN_ENDPOINT, // bEndpointAddress
        0,      // bmInfo
        OUTPUT_TERMINAL_ID, // bTerminalLink
        0,      // bStillCaptureMethod
        0,      // bTriggerSupport
        0,      // bTriggerUsage
        1,      // bControlSize
        0,      // bmaControls[0]
    ]);

    // MJPEG format descriptor.
    stream_specific.extend_from_slice(&[
        11,   // bLength
        0x24, // CS_INTERFACE
        0x06, // VS_FORMAT_MJPEG
        1,    // bFormatIndex
        FRAMES.len() as u8, // bNumFrameDescriptors
        0,    // bmFlags = not fixed size
        FRAMES.len() as u8, // bDefaultFrameIndex (largest size)
        0,    // bAspectRatioX
        0,    // bAspectRatioY
        0,    // bmInterlaceFlags
        0,    // bCopyProtect
    ]);

    // MJPEG frame descriptors, one per advertised size, two discrete intervals
    // each (60 fps and 30 fps).
    for (index, (width, height)) in FRAMES.iter().enumerate() {
        let mut frame = Vec::new();
        frame.push((26 + 4 * FRAME_INTERVALS.len()) as u8); // bLength
        frame.extend_from_slice(&[0x24, 0x07]); // CS_INTERFACE, VS_FRAME_MJPEG
        frame.push(index as u8 + 1); // bFrameIndex
        frame.push(0); // bmCapabilities
        frame.extend_from_slice(&u16le(*width));
        frame.extend_from_slice(&u16le(*height));
        frame.extend_from_slice(&u32le(MIN_BIT_RATE));
        frame.extend_from_slice(&u32le(MAX_BIT_RATE));
        frame.extend_from_slice(&u32le(*width as u32 * *height as u32 * 2));
        frame.extend_from_slice(&u32le(FRAME_INTERVAL_60)); // dwDefaultFrameInterval
        frame.push(FRAME_INTERVALS.len() as u8); // bFrameIntervalType = discrete
        for interval in FRAME_INTERVALS {
            frame.extend_from_slice(&u32le(interval));
        }
        stream_specific.extend_from_slice(&frame);
    }

    // The VS input header stores bNumFormats before wTotalLength, so the length
    // lives at bytes 4..6 (the VC header has bcdUVC in between, at 5..7).
    let vs_total = stream_specific.len() as u16;
    stream_specific[4..6].copy_from_slice(&u16le(vs_total));
    config.extend_from_slice(&stream_specific);

    if ENDPOINT_IN_ALT_ZERO {
        // The bulk endpoint lives in alternate setting 0, so the host does not
        // have to switch settings before streaming. Empty and bulk only cameras
        // that are known to work with Windows (and the QEMU UVC prototype) use
        // this layout.
        config.extend_from_slice(&[
            7,    // bLength
            0x05, // bDescriptorType = ENDPOINT
            BULK_IN_ENDPOINT,
            0x02, // bmAttributes = bulk
            0x00, 0x02, // wMaxPacketSize = 512
            0,    // bInterval
        ]);
    } else {
        // VideoStreaming interface, alternate setting 1: the bulk IN endpoint.
        config.extend_from_slice(&[
            9, // bLength
            0x04, STREAM_INTERFACE, 1, 1, // bInterfaceNumber/alt/bNumEndpoints
            0x0e, 0x02, 0x00, // Video, VideoStreaming, no protocol
            0,    // iInterface
        ]);

        // Bulk IN endpoint, 512 byte packets, high speed.
        config.extend_from_slice(&[
            7,    // bLength
            0x05, // bDescriptorType = ENDPOINT
            BULK_IN_ENDPOINT,
            0x02, // bmAttributes = bulk
            0x00, 0x02, // wMaxPacketSize = 512
            0,    // bInterval
        ]);
    }

    let total = config.len() as u16;
    config[2..4].copy_from_slice(&u16le(total));
    config
}

/// Collects the class specific (CS_INTERFACE, type 0x24) descriptors that
/// follow the given interface descriptor inside a configuration descriptor set.
pub fn class_specific_descriptors(config: &[u8], interface: u8) -> Vec<u8> {
    let mut block = Vec::new();
    let mut offset = 0;
    let mut inside = false;
    while offset + 2 <= config.len() {
        let length = config[offset] as usize;
        if length < 2 || offset + length > config.len() {
            break;
        }
        match config[offset + 1] {
            // INTERFACE: the class specific descriptors of an interface
            // immediately follow it.
            0x04 => inside = config[offset + 2] == interface,
            0x24 if inside => block.extend_from_slice(&config[offset..offset + length]),
            _ => {}
        }
        offset += length;
    }
    block
}
