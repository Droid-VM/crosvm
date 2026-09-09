// Copyright 2019 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use std::collections::BTreeMap;

use base::warn;
use linux_input_sys::constants::*;

use super::virtio_input_absinfo;
use super::virtio_input_bitmap;
use super::virtio_input_device_ids;
use super::VirtioInputConfig;

/// Instantiates a VirtioInputConfig object with the default configuration for a trackpad. It
/// supports touch, left button and right button events, as well as X and Y axis.
pub fn new_trackpad_config(
    idx: u32,
    width: u32,
    height: u32,
    name: Option<&str>,
) -> VirtioInputConfig {
    let name = name
        .map(str::to_owned)
        .unwrap_or(format!("Crosvm Virtio Trackpad {idx}"));
    VirtioInputConfig::new(
        virtio_input_device_ids::new(0, 0, 0, 0),
        name,
        format!("virtio-trackpad-{idx}"),
        virtio_input_bitmap::new([0u8; 128]),
        default_trackpad_events(),
        default_trackpad_absinfo(width, height),
    )
}

pub fn new_multitouch_trackpad_config(
    idx: u32,
    width: u32,
    height: u32,
    name: Option<&str>,
) -> VirtioInputConfig {
    let name = name
        .map(str::to_owned)
        .unwrap_or(format!("Crosvm Virtio Multi-touch Trackpad {idx}"));
    VirtioInputConfig::new(
        virtio_input_device_ids::new(0, 0, 0, 0),
        name,
        format!("virtio-multi-touch-trackpad-{idx}"),
        virtio_input_bitmap::from_bits(&[INPUT_PROP_POINTER, INPUT_PROP_BUTTONPAD]),
        default_multitouchpad_events(),
        default_multitouchpad_absinfo(width, height, 10, 65536),
    )
}

/// Instantiates a VirtioInputConfig object with the default configuration for a mouse.
/// It supports left, right and middle buttons, as wel as X, Y and wheel relative axes.
pub fn new_mouse_config(idx: u32, name: Option<&str>) -> VirtioInputConfig {
    let name = name
        .map(str::to_owned)
        .unwrap_or(format!("Crosvm Virtio Mouse {idx}"));
    VirtioInputConfig::new(
        virtio_input_device_ids::new(0, 0, 0, 0),
        name,
        format!("virtio-mouse-{idx}"),
        virtio_input_bitmap::new([0u8; 128]),
        default_mouse_events(),
        BTreeMap::new(),
    )
}

/// Instantiates a VirtioInputConfig object for an absolute-pointing mouse (the qemu usb-tablet
/// profile): ABS_X/ABS_Y positioning plus mouse buttons and a scroll wheel. Remote displays
/// (VNC) need absolute coordinates so the client cursor maps 1:1 onto the guest with no drift,
/// while the button/wheel set keeps full mouse semantics (hover, right-click, scroll).
///
/// `name` overrides the generated device name, exactly as it does for the touchscreen configs.
/// An absolute pointer's coordinates only mean anything against one output's geometry, and evdev
/// has no field saying which output that is -- every guest maps such a device to an output by its
/// name, so a caller creating one per output has to be able to say which one this is.
pub fn new_absolute_mouse_config(
    idx: u32,
    width: u32,
    height: u32,
    name: Option<&str>,
) -> VirtioInputConfig {
    let name = name
        .map(str::to_owned)
        .unwrap_or(format!("Crosvm Virtio Absolute Mouse {idx}"));
    VirtioInputConfig::new(
        virtio_input_device_ids::new(0, 0, 0, 0),
        name,
        format!("virtio-abs-mouse-{idx}"),
        virtio_input_bitmap::new([0u8; 128]),
        default_absolute_mouse_events(),
        default_trackpad_absinfo(width, height),
    )
}

/// Instantiates a VirtioInputConfig object with the default configuration for a keyboard.
/// It supports every key code Linux defines for a keyboard (see `default_keyboard_events`) and
/// the CAPSLOCK, NUMLOCK and SCROLLLOCK leds.
pub fn new_keyboard_config(idx: u32, name: Option<&str>) -> VirtioInputConfig {
    let name = name
        .map(str::to_owned)
        .unwrap_or(format!("Crosvm Virtio Keyboard {idx}"));
    VirtioInputConfig::new(
        virtio_input_device_ids::new(0, 0, 0, 0),
        name,
        format!("virtio-keyboard-{idx}"),
        virtio_input_bitmap::new([0u8; 128]),
        default_keyboard_events(),
        BTreeMap::new(),
    )
}

/// Instantiates a VirtioInputConfig object with the default configuration for a collection of
/// switches.
pub fn new_switches_config(idx: u32) -> VirtioInputConfig {
    VirtioInputConfig::new(
        virtio_input_device_ids::new(0, 0, 0, 0),
        format!("Crosvm Virtio Switches {idx}"),
        format!("virtio-switches-{idx}"),
        virtio_input_bitmap::new([0u8; 128]),
        default_switch_events(),
        BTreeMap::new(),
    )
}

/// Instantiates a VirtioInputConfig object with the default configuration for a collection of
/// rotary.
pub fn new_rotary_config(idx: u32) -> VirtioInputConfig {
    VirtioInputConfig::new(
        virtio_input_device_ids::new(0, 0, 0, 0),
        format!("Crosvm Virtio Rotary {idx}"),
        format!("virtio-rotary-{idx}"),
        virtio_input_bitmap::new([0u8; 128]),
        default_rotary_events(),
        BTreeMap::new(),
    )
}

/// Instantiates a VirtioInputConfig object with the default configuration for a touchscreen (no
/// multitouch support).
pub fn new_single_touch_config(
    idx: u32,
    width: u32,
    height: u32,
    name: Option<&str>,
) -> VirtioInputConfig {
    let name = name
        .map(str::to_owned)
        .unwrap_or(format!("Crosvm Virtio Touchscreen {idx}"));
    VirtioInputConfig::new(
        virtio_input_device_ids::new(0, 0, 0, 0),
        name,
        format!("virtio-touchscreen-{idx}"),
        virtio_input_bitmap::from_bits(&[INPUT_PROP_DIRECT]),
        default_touchscreen_events(),
        default_touchscreen_absinfo(width, height),
    )
}

/// Instantiates a VirtioInputConfig object with the default configuration for a multitouch
/// touchscreen.
pub fn new_multi_touch_config(
    idx: u32,
    width: u32,
    height: u32,
    name: Option<&str>,
) -> VirtioInputConfig {
    let name = name
        .map(str::to_owned)
        .unwrap_or(format!("Crosvm Virtio Multitouch Touchscreen {idx}"));
    VirtioInputConfig::new(
        virtio_input_device_ids::new(0, 0, 0, 0),
        name,
        format!("virtio-touchscreen-{idx}"),
        virtio_input_bitmap::from_bits(&[INPUT_PROP_DIRECT]),
        default_multitouchscreen_events(),
        default_multitouchscreen_absinfo(width, height, 10, 10),
    )
}

/// Initializes a VirtioInputConfig object for a custom virtio-input device.
///
/// # Arguments
///
/// * `idx` - input device index
/// * `name` - input device name
/// * `serial_name` - input device serial name
/// * `properties` - input device properties
/// * `supported_events` - Event configuration provided by a configuration file
/// * `axis_info` - Device axis configuration
pub fn new_custom_config(
    idx: u32,
    name: &str,
    serial_name: &str,
    properties: virtio_input_bitmap,
    supported_events: BTreeMap<u16, virtio_input_bitmap>,
    axis_info: BTreeMap<u16, virtio_input_absinfo>,
) -> VirtioInputConfig {
    let name: String = format!("{name} {idx}");
    let serial_name = format!("{serial_name}-{idx}");
    if name.len() > 128 {
        warn!("name: {name} exceeds 128 bytes, will be truncated.");
    }
    if serial_name.len() > 128 {
        warn!("serial_name: {serial_name} exceeds 128 bytes, will be truncated.");
    }

    VirtioInputConfig::new(
        virtio_input_device_ids::new(0, 0, 0, 0),
        name,
        serial_name,
        properties,
        supported_events,
        axis_info,
    )
}

fn default_touchscreen_absinfo(width: u32, height: u32) -> BTreeMap<u16, virtio_input_absinfo> {
    let mut absinfo: BTreeMap<u16, virtio_input_absinfo> = BTreeMap::new();
    absinfo.insert(ABS_X, virtio_input_absinfo::new(0, width, 0, 0));
    absinfo.insert(ABS_Y, virtio_input_absinfo::new(0, height, 0, 0));
    absinfo
}

fn default_touchscreen_events() -> BTreeMap<u16, virtio_input_bitmap> {
    let mut supported_events: BTreeMap<u16, virtio_input_bitmap> = BTreeMap::new();
    supported_events.insert(EV_KEY, virtio_input_bitmap::from_bits(&[BTN_TOUCH]));
    supported_events.insert(EV_ABS, virtio_input_bitmap::from_bits(&[ABS_X, ABS_Y]));
    supported_events
}

fn default_multitouchscreen_absinfo(
    width: u32,
    height: u32,
    slot: u32,
    id: u32,
) -> BTreeMap<u16, virtio_input_absinfo> {
    let mut absinfo: BTreeMap<u16, virtio_input_absinfo> = BTreeMap::new();
    absinfo.insert(ABS_MT_SLOT, virtio_input_absinfo::new(0, slot, 0, 0));
    absinfo.insert(ABS_MT_TRACKING_ID, virtio_input_absinfo::new(0, id, 0, 0));
    absinfo.insert(ABS_X, virtio_input_absinfo::new(0, width, 0, 0));
    absinfo.insert(ABS_Y, virtio_input_absinfo::new(0, height, 0, 0));
    absinfo.insert(ABS_MT_POSITION_X, virtio_input_absinfo::new(0, width, 0, 0));
    absinfo.insert(
        ABS_MT_POSITION_Y,
        virtio_input_absinfo::new(0, height, 0, 0),
    );
    absinfo
}

fn default_multitouchscreen_events() -> BTreeMap<u16, virtio_input_bitmap> {
    let mut supported_events: BTreeMap<u16, virtio_input_bitmap> = BTreeMap::new();
    supported_events.insert(EV_KEY, virtio_input_bitmap::from_bits(&[BTN_TOUCH]));
    supported_events.insert(
        EV_ABS,
        virtio_input_bitmap::from_bits(&[
            ABS_MT_SLOT,
            ABS_MT_TRACKING_ID,
            ABS_MT_POSITION_X,
            ABS_MT_POSITION_Y,
            ABS_X,
            ABS_Y,
        ]),
    );
    supported_events
}

fn default_multitouchpad_absinfo(
    width: u32,
    height: u32,
    slot: u32,
    id: u32,
) -> BTreeMap<u16, virtio_input_absinfo> {
    let mut absinfo: BTreeMap<u16, virtio_input_absinfo> = BTreeMap::new();
    absinfo.insert(ABS_MT_SLOT, virtio_input_absinfo::new(0, slot, 0, 0));
    absinfo.insert(ABS_MT_TRACKING_ID, virtio_input_absinfo::new(0, id, 0, 0));
    // TODO(b/347253952): make them configurable if necessary
    absinfo.insert(ABS_MT_PRESSURE, virtio_input_absinfo::new(0, 255, 0, 0));
    absinfo.insert(ABS_PRESSURE, virtio_input_absinfo::new(0, 255, 0, 0));
    absinfo.insert(ABS_MT_TOUCH_MAJOR, virtio_input_absinfo::new(0, 4095, 0, 0));
    absinfo.insert(ABS_MT_TOUCH_MINOR, virtio_input_absinfo::new(0, 4095, 0, 0));
    absinfo.insert(ABS_X, virtio_input_absinfo::new(0, width, 0, 0));
    absinfo.insert(ABS_Y, virtio_input_absinfo::new(0, height, 0, 0));
    absinfo.insert(ABS_MT_POSITION_X, virtio_input_absinfo::new(0, width, 0, 0));
    absinfo.insert(ABS_MT_TOOL_TYPE, virtio_input_absinfo::new(0, 2, 0, 0));
    absinfo.insert(
        ABS_MT_POSITION_Y,
        virtio_input_absinfo::new(0, height, 0, 0),
    );
    absinfo
}

fn default_multitouchpad_events() -> BTreeMap<u16, virtio_input_bitmap> {
    let mut supported_events: BTreeMap<u16, virtio_input_bitmap> = BTreeMap::new();
    supported_events.insert(
        EV_KEY,
        virtio_input_bitmap::from_bits(&[
            BTN_TOUCH,
            BTN_TOOL_FINGER,
            BTN_TOOL_DOUBLETAP,
            BTN_TOOL_TRIPLETAP,
            BTN_TOOL_QUADTAP,
            BTN_LEFT,
        ]),
    );
    supported_events.insert(
        EV_ABS,
        virtio_input_bitmap::from_bits(&[
            ABS_MT_SLOT,
            ABS_MT_TRACKING_ID,
            ABS_MT_POSITION_X,
            ABS_MT_POSITION_Y,
            ABS_MT_TOOL_TYPE,
            ABS_MT_PRESSURE,
            ABS_X,
            ABS_Y,
            ABS_PRESSURE,
            ABS_MT_TOUCH_MAJOR,
            ABS_MT_TOUCH_MINOR,
            ABS_PRESSURE,
        ]),
    );
    supported_events
}

fn default_trackpad_absinfo(width: u32, height: u32) -> BTreeMap<u16, virtio_input_absinfo> {
    let mut absinfo: BTreeMap<u16, virtio_input_absinfo> = BTreeMap::new();
    absinfo.insert(ABS_X, virtio_input_absinfo::new(0, width, 0, 0));
    absinfo.insert(ABS_Y, virtio_input_absinfo::new(0, height, 0, 0));
    absinfo
}

fn default_trackpad_events() -> BTreeMap<u16, virtio_input_bitmap> {
    let mut supported_events: BTreeMap<u16, virtio_input_bitmap> = BTreeMap::new();
    supported_events.insert(
        EV_KEY,
        virtio_input_bitmap::from_bits(&[BTN_TOOL_FINGER, BTN_TOUCH, BTN_LEFT, BTN_RIGHT]),
    );
    supported_events.insert(EV_ABS, virtio_input_bitmap::from_bits(&[ABS_X, ABS_Y]));
    supported_events
}

fn default_absolute_mouse_events() -> BTreeMap<u16, virtio_input_bitmap> {
    let mut supported_events: BTreeMap<u16, virtio_input_bitmap> = BTreeMap::new();
    supported_events.insert(
        EV_KEY,
        virtio_input_bitmap::from_bits(&[BTN_LEFT, BTN_RIGHT, BTN_MIDDLE]),
    );
    supported_events.insert(EV_ABS, virtio_input_bitmap::from_bits(&[ABS_X, ABS_Y]));
    // Advertise the horizontal wheel too so the guest keeps our REL_HWHEEL events (2D scrolling).
    supported_events.insert(
        EV_REL,
        virtio_input_bitmap::from_bits(&[REL_WHEEL, REL_HWHEEL]),
    );
    supported_events
}

fn default_mouse_events() -> BTreeMap<u16, virtio_input_bitmap> {
    let mut supported_events: BTreeMap<u16, virtio_input_bitmap> = BTreeMap::new();
    supported_events.insert(
        EV_KEY,
        virtio_input_bitmap::from_bits(&[BTN_LEFT, BTN_RIGHT, BTN_MIDDLE]),
    );
    // REL_HWHEEL: without it the guest drops horizontal-scroll events, so 2D/side scrolling
    // (two-finger pan sideways, tilt wheel) never reaches apps.
    supported_events.insert(
        EV_REL,
        virtio_input_bitmap::from_bits(&[REL_X, REL_Y, REL_WHEEL, REL_HWHEEL]),
    );
    supported_events
}

fn default_keyboard_events() -> BTreeMap<u16, virtio_input_bitmap> {
    let mut supported_events: BTreeMap<u16, virtio_input_bitmap> = BTreeMap::new();
    // Every key code Linux gives a keyboard, KEY_ESC(1) through KEY_MICMUTE(248), rather than an
    // en-us subset. The bitmap is not decoration: a guest drops an event whose code the device
    // never advertised, so a key left out here cannot reach the guest by any route. The subset
    // this used to be had no KEY_LEFTMETA, which is the Super/Windows key, so Win- and Super-
    // shortcuts died in the guest's input core even when the host had forwarded them faithfully;
    // KEY_102ND, the F13-F24 block, the Japanese and Korean keys and the media keys went the same
    // way. A real USB keyboard advertises its whole range for the same reason, and the host is
    // free to send only what it has. It stops at 248 because 0x100 upwards is the BTN_* block --
    // mouse and gamepad buttons, which belong to the pointer devices, not here.
    let keys: Vec<u16> = (KEY_ESC..=KEY_MICMUTE).collect();
    supported_events.insert(EV_KEY, virtio_input_bitmap::from_bits(&keys));
    supported_events.insert(
        EV_REP,
        virtio_input_bitmap::from_bits(&[REP_DELAY, REP_PERIOD]),
    );
    supported_events.insert(
        EV_LED,
        virtio_input_bitmap::from_bits(&[LED_CAPSL, LED_NUML, LED_SCROLLL]),
    );
    supported_events
}

fn default_switch_events() -> BTreeMap<u16, virtio_input_bitmap> {
    let mut supported_events: BTreeMap<u16, virtio_input_bitmap> = BTreeMap::new();
    supported_events.insert(
        EV_SW,
        virtio_input_bitmap::from_bits(&[
            SW_LID,
            SW_TABLET_MODE,
            SW_HEADPHONE_INSERT,
            SW_RFKILL_ALL,
            SW_MICROPHONE_INSERT,
            SW_DOCK,
            SW_LINEOUT_INSERT,
            SW_JACK_PHYSICAL_INSERT,
            SW_VIDEOOUT_INSERT,
            SW_CAMERA_LENS_COVER,
            SW_KEYPAD_SLIDE,
            SW_FRONT_PROXIMITY,
            SW_ROTATE_LOCK,
            SW_LINEIN_INSERT,
            SW_MUTE_DEVICE,
            SW_PEN_INSERTED,
            SW_MACHINE_COVER,
        ]),
    );
    supported_events
}

fn default_rotary_events() -> BTreeMap<u16, virtio_input_bitmap> {
    let mut supported_events: BTreeMap<u16, virtio_input_bitmap> = BTreeMap::new();
    supported_events.insert(EV_REL, virtio_input_bitmap::from_bits(&[REL_WHEEL]));
    supported_events
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_switches_config() {
        let config = new_switches_config(0);
        assert_eq!(config.serial_name, "virtio-switches-0");

        let events = config.supported_events;
        assert_eq!(events.len(), 1);
        assert_eq!(events.contains_key(&EV_SW), true);

        // The bitmap should contain SW_CNT=0x10+1=17 ones,
        // where each one is packed into the u8 bitmap.
        let mut expected_bitmap = [0_u8; 128];
        expected_bitmap[0] = 0b11111111u8;
        expected_bitmap[1] = 0b11111111u8;
        expected_bitmap[2] = 0b1u8;
        assert_eq!(events[&EV_SW].bitmap, expected_bitmap);
    }
}
