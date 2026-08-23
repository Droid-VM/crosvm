// Copyright 2026 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! Resolving a host audio endpoint from a stable name.
//!
//! AAudio identifies a device by an integer that the platform hands out per connection: unplug a
//! headset and plug it back in and the number is different, even though it is plainly the same
//! headset. Anything that wants to reopen a stream on "the device the user chose" therefore needs
//! a name that outlives the connection, and a way to turn that name back into today's number.
//!
//! The name is the one Android's own `AudioDeviceInfo` supports: its type and its address, as
//! `TYPE|address` -- a Bluetooth MAC, a USB card/device string, or nothing at all for the built-in
//! endpoints, of which there is one each. Moving a USB device to a different port changes its
//! address and so counts as a different endpoint; that is a deliberate limit, not an oversight.
//!
//! Native code cannot enumerate devices -- AAudio has no API for it and `AudioManager` is Java --
//! so the table is published by whoever launched the VM and re-read here. Doing the matching on
//! this side rather than being handed a number means a reconnection can be picked up without
//! asking anyone: the file changes, and the name is still the same name.

use std::fs;
use std::path::Path;
use std::time::SystemTime;

/// The endpoint that means "whatever the platform would route to".
///
/// Distinct from an endpoint that merely happens to be the default today: that one is named, and
/// stays named when the default moves elsewhere. It is published in the table like any other
/// endpoint, against id 0 -- `AAUDIO_DEVICE_UNSPECIFIED` -- so it resolves by the ordinary path;
/// the check below only covers a table that is missing or has not been written yet.
pub const SYSTEM_DEFAULT_KEY: &str = "DEFAULT|system default";

/// Separates the endpoint's name from what the stream is *for*.
///
/// An address already contains most punctuation worth choosing -- a Bluetooth MAC has colons, a
/// USB card address has both semicolons and equals signs -- so the separator has to be something
/// none of them use.
pub const ATTR_SEPARATOR: char = '#';

/// Splits `TYPE|address#attr=val,...` into the part that names the endpoint and the part that
/// says what the stream is for. Only the first is matched against the table; the second decides
/// how the stream is opened, and an endpoint means the same device whatever it is opened for.
pub fn split_key(key: &str) -> (&str, &str) {
    match key.split_once(ATTR_SEPARATOR) {
        Some((device, attrs)) => (device, attrs),
        None => (key, ""),
    }
}

/// Looks up one `attr=value` in the attribute part. Absent, or empty, means "not specified" --
/// which is distinct from any value, and is what leaves the platform's own default in place.
pub fn attr<'a>(attrs: &'a str, name: &str) -> Option<&'a str> {
    for pair in attrs.split(',') {
        if let Some((k, v)) = pair.split_once('=') {
            if k.trim() == name && !v.trim().is_empty() {
                return Some(v.trim());
            }
        }
    }
    None
}

/// Reads the table and returns the current AAudio device id for `key`, or `None` when the
/// endpoint is not present. `Some(0)` is never returned for a real match: 0 is
/// `AAUDIO_DEVICE_UNSPECIFIED` and means the caller should not pin at all.
///
/// Lines are `<id>\t<in|out>\t<TYPE|address>`; anything else is skipped, so a partially written
/// file costs at most a failed lookup and never a wrong one.
pub fn resolve(table_path: &Path, key: &str, input: bool) -> Option<i32> {
    let (key, _) = split_key(key);
    if key.is_empty() || key == SYSTEM_DEFAULT_KEY {
        return None;
    }
    let text = fs::read_to_string(table_path).ok()?;
    for line in text.lines() {
        let mut fields = line.split('\t');
        let id = fields.next()?.trim().parse::<i32>().ok();
        let dir = fields.next();
        let entry = fields.next();
        let (Some(id), Some(dir), Some(entry)) = (id, dir, entry) else {
            continue;
        };
        if (dir == "in") != input {
            continue;
        }
        if entry.trim() == key {
            return Some(id);
        }
    }
    None
}

/// When the table last changed, for noticing that a device may have come back without reading
/// and parsing the whole file on every poll.
pub fn generation(table_path: &Path) -> Option<SystemTime> {
    fs::metadata(table_path).ok()?.modified().ok()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn write(dir: &tempfile::TempDir, body: &str) -> std::path::PathBuf {
        let path = dir.path().join("devices");
        fs::write(&path, body).unwrap();
        path
    }

    #[test]
    fn matches_direction_as_well_as_name() {
        let dir = tempfile::TempDir::new().unwrap();
        // The same key can exist in both directions -- a headset is a speaker and a microphone --
        // and they are different endpoints with different ids.
        let path = write(
            &dir,
            "7\tout\tBLUETOOTH_A2DP|AA:BB\n9\tin\tBLUETOOTH_A2DP|AA:BB\n",
        );
        assert_eq!(resolve(&path, "BLUETOOTH_A2DP|AA:BB", false), Some(7));
        assert_eq!(resolve(&path, "BLUETOOTH_A2DP|AA:BB", true), Some(9));
    }

    #[test]
    fn absent_endpoint_does_not_resolve() {
        let dir = tempfile::TempDir::new().unwrap();
        let path = write(&dir, "2\tout\tBUILTIN_SPEAKER|\n");
        assert_eq!(resolve(&path, "WIRED_HEADPHONES|", false), None);
    }

    #[test]
    fn attributes_do_not_change_which_endpoint_is_meant() {
        let dir = tempfile::TempDir::new().unwrap();
        let path = write(&dir, "21\tin\tBUILTIN_MIC|bottom\n");
        // The same microphone, opened for two different purposes.
        assert_eq!(resolve(&path, "BUILTIN_MIC|bottom", true), Some(21));
        assert_eq!(
            resolve(&path, "BUILTIN_MIC|bottom#preset=voice_communication", true),
            Some(21)
        );
    }

    #[test]
    fn an_address_may_contain_the_punctuation_attributes_use() {
        let dir = tempfile::TempDir::new().unwrap();
        // A USB address has both a semicolon and an equals sign in it.
        let path = write(&dir, "31\tout\tUSB_DEVICE|card=1;device=0\n");
        let (device, attrs) = split_key("USB_DEVICE|card=1;device=0#usage=media");
        assert_eq!(device, "USB_DEVICE|card=1;device=0");
        assert_eq!(attr(attrs, "usage"), Some("media"));
        assert_eq!(resolve(&path, "USB_DEVICE|card=1;device=0#usage=media", false), Some(31));
    }

    #[test]
    fn an_absent_or_empty_attribute_is_not_a_value() {
        assert_eq!(attr("usage=media", "content"), None);
        assert_eq!(attr("usage=", "usage"), None);
        assert_eq!(attr("", "usage"), None);
    }

    #[test]
    fn system_default_is_not_an_endpoint() {
        let dir = tempfile::TempDir::new().unwrap();
        let path = write(&dir, "2\tout\tBUILTIN_SPEAKER|\n");
        assert_eq!(resolve(&path, SYSTEM_DEFAULT_KEY, false), None);
        assert_eq!(resolve(&path, "", false), None);
    }

    #[test]
    fn a_half_written_line_is_skipped_not_guessed() {
        let dir = tempfile::TempDir::new().unwrap();
        let path = write(&dir, "2\tout\tBUILTIN_SPEAKER|\nnot-a-number\tout\tX|\n5\tout");
        assert_eq!(resolve(&path, "BUILTIN_SPEAKER|", false), Some(2));
        assert_eq!(resolve(&path, "X|", false), None);
    }
}
