// Copyright 2026 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! DroidVM device-config extension for a fixed, boot-shared DRM blob pool.
//! Feature bit 7 gates bytes 16..48; the upstream 16-byte config is unchanged.

pub const FEATURE: u32 = 7;
pub const CONFIG_SIZE: usize = 32;

pub fn descriptor(base: u64, size: u64) -> Option<[u8; CONFIG_SIZE]> {
    if base == 0 || size == 0 || (base | size) & 4095 != 0
        || base.checked_add(size).is_none() || size > (1u64 << 32)
    {
        return None;
    }
    let mut bytes = [0; CONFIG_SIZE];
    bytes[0..8].copy_from_slice(b"DVMPOOL1");
    bytes[8..12].copy_from_slice(&1u32.to_le_bytes()); // ABI version
    bytes[12..16].copy_from_slice(&1u32.to_le_bytes()); // kind: drm2kgsl
    bytes[16..24].copy_from_slice(&base.to_le_bytes());
    bytes[24..32].copy_from_slice(&size.to_le_bytes());
    Some(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wire_layout_and_invalid_ranges() {
        let d = descriptor(0x17c000000, 0x4000000).unwrap();
        assert_eq!(&d[..8], b"DVMPOOL1");
        assert_eq!(u32::from_le_bytes(d[8..12].try_into().unwrap()), 1);
        assert_eq!(u32::from_le_bytes(d[12..16].try_into().unwrap()), 1);
        assert_eq!(u64::from_le_bytes(d[16..24].try_into().unwrap()), 0x17c000000);
        assert_eq!(u64::from_le_bytes(d[24..32].try_into().unwrap()), 0x4000000);
        for (base, size) in [(0, 4096), (4096, 0), (1, 4096), (4096, 1),
                             (u64::MAX - 4095, 4096), (4096, (1u64 << 32) + 4096)] {
            assert!(descriptor(base, size).is_none());
        }
    }
}
