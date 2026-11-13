// Copyright 2026 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// Windows vioinput uses 16-bit HID position fields. Keep oversized evdev
/// positions and their advertised ranges consistent instead of truncating events.
#[derive(Clone, Copy)]
pub(super) struct AxisScale {
    min: i32,
    max: i32,
}

impl AxisScale {
    pub const MAX: u32 = 32767;

    pub fn new(min: i32, max: i32) -> Option<Self> {
        (min >= 0 && max > 65535 && max > min).then_some(Self { min, max })
    }

    pub fn position(self, value: i32) -> i32 {
        let offset = i64::from(value.clamp(self.min, self.max)) - i64::from(self.min);
        let span = i64::from(self.max) - i64::from(self.min);
        ((offset * i64::from(Self::MAX) + span / 2) / span) as i32
    }

    pub fn distance(self, value: u32) -> u32 {
        let span = (i64::from(self.max) - i64::from(self.min)) as u64;
        ((u64::from(value) * u64::from(Self::MAX) + span / 2) / span)
            .min(u64::from(Self::MAX)) as u32
    }
}

#[cfg(test)]
mod tests {
    use super::AxisScale;

    #[test]
    fn goodix_positions_do_not_wrap_at_16_bits() {
        for max in [127999, 277199] {
            let scale = AxisScale::new(0, max).unwrap();
            assert_eq!(scale.position(0), 0);
            assert_eq!(scale.position(max), 32767);
            assert!((scale.position(max / 2) - 16383).abs() <= 1);
            assert!(scale.position(65536) >= scale.position(65535));
            assert!(scale.position(65536) > 7000);
            assert_eq!(scale.position(-1), 0);
            assert_eq!(scale.position(i32::MAX), 32767);
            assert_eq!(scale.distance(0), 0);
        }
    }

    #[test]
    fn ordinary_and_signed_axes_remain_unchanged() {
        assert!(AxisScale::new(0, 65535).is_none());
        assert!(AxisScale::new(-100000, 100000).is_none());
        assert!(AxisScale::new(100000, 100000).is_none());
        let scale = AxisScale::new(1000, 101000).unwrap();
        assert_eq!(scale.position(1000), 0);
        assert_eq!(scale.position(51000), 16384);
        assert_eq!(scale.position(101000), 32767);
    }
}
