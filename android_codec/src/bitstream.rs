// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

//! The two elementary-stream containers the probe reads and writes: Annex-B byte streams for
//! H.264 and H.265, and IVF for VP8, VP9 and AV1.
//!
//! A decoder wants one access unit (one picture's worth of NAL units, or one IVF frame) per
//! input buffer, with its parameter sets either in the format (`csd-0`/`csd-1`), in a
//! `BUFFER_FLAG_CODEC_CONFIG` buffer of their own, or in-band ahead of the first slice. This
//! module only cuts the stream; which of the three the codec is fed is the caller's choice.

use std::ops::Range;

use thiserror::Error;

#[derive(Error, Debug, Clone, PartialEq, Eq)]
pub enum BitstreamError {
    #[error("not an IVF file (no DKIF signature)")]
    NotIvf,
    #[error("IVF header claims {0} bytes, file has {1}")]
    IvfHeaderShort(usize, usize),
    #[error("IVF frame at byte {0} runs past the end of the file")]
    IvfFrameTruncated(usize),
    #[error("no start code in the first {0} bytes: not an Annex-B stream")]
    NotAnnexB(usize),
}

/// Which NAL unit syntax an Annex-B stream carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NalCodec {
    H264,
    H265,
}

impl NalCodec {
    pub fn for_mime(mime: &str) -> Option<NalCodec> {
        match mime {
            "video/avc" => Some(NalCodec::H264),
            "video/hevc" => Some(NalCodec::H265),
            _ => None,
        }
    }

    /// The `nal_unit_type` of a NAL unit given its bytes after the start code.
    pub fn nal_type(self, nal: &[u8]) -> Option<u8> {
        let first = *nal.first()?;
        Some(match self {
            NalCodec::H264 => first & 0x1f,
            NalCodec::H265 => (first >> 1) & 0x3f,
        })
    }

    fn header_len(self) -> usize {
        match self {
            NalCodec::H264 => 1,
            NalCodec::H265 => 2,
        }
    }

    fn is_vcl(self, t: u8) -> bool {
        match self {
            NalCodec::H264 => (1..=5).contains(&t),
            NalCodec::H265 => t <= 31,
        }
    }

    /// A non-VCL NAL unit that, by the spec's access-unit rules, can only be the first of a new
    /// access unit when it follows a VCL NAL unit: AUD, parameter sets, prefix SEI.
    fn starts_access_unit(self, t: u8) -> bool {
        match self {
            // 6 SEI, 7 SPS, 8 PPS, 9 AUD, 14-18 prefix / subset SPS / depth / reserved.
            NalCodec::H264 => t == 6 || (7..=9).contains(&t) || (14..=18).contains(&t),
            // 32 VPS, 33 SPS, 34 PPS, 35 AUD, 39 prefix SEI, 41-44 and 48-55 reserved.
            NalCodec::H265 => {
                (32..=35).contains(&t)
                    || t == 39
                    || (41..=44).contains(&t)
                    || (48..=55).contains(&t)
            }
        }
    }

    /// Whether this is a parameter set (what `csd-0`/`csd-1` carry).
    pub fn is_parameter_set(self, t: u8) -> bool {
        match self {
            NalCodec::H264 => t == 7 || t == 8,
            NalCodec::H265 => (32..=34).contains(&t),
        }
    }

    /// Whether a VCL NAL unit of this type begins a picture a decoder can start from.
    pub fn is_keyframe_type(self, t: u8) -> bool {
        match self {
            NalCodec::H264 => t == 5,
            // BLA_W_LP .. RSV_IRAP_VCL23: the IRAP range.
            NalCodec::H265 => (16..=23).contains(&t),
        }
    }

    /// Whether a VCL NAL unit is the first slice of its picture: `first_mb_in_slice` (H.264)
    /// or `first_slice_segment_in_pic_flag` (H.265) is the first syntax element after the NAL
    /// header, and both are "zero / set" exactly when the first payload bit is 1.
    fn first_slice_of_picture(self, nal: &[u8]) -> bool {
        nal.get(self.header_len())
            .map(|b| b & 0x80 != 0)
            .unwrap_or(false)
    }
}

/// Byte ranges of every NAL unit in `data`, each range starting at its start code (three or
/// four zero-prefixed bytes) and ending where the next start code begins.
pub fn nal_ranges(data: &[u8]) -> Vec<Range<usize>> {
    let mut starts = Vec::new();
    let mut i = 0;
    while i + 2 < data.len() {
        if data[i] == 0 && data[i + 1] == 0 && data[i + 2] == 1 {
            // A four-byte start code is a three-byte one with a leading zero; include it.
            let begin = if i > 0 && data[i - 1] == 0 { i - 1 } else { i };
            starts.push(begin);
            i += 3;
        } else {
            i += 1;
        }
    }
    let mut ranges = Vec::with_capacity(starts.len());
    for (n, &begin) in starts.iter().enumerate() {
        let end = starts.get(n + 1).copied().unwrap_or(data.len());
        ranges.push(begin..end);
    }
    ranges
}

/// The NAL unit's payload (after the start code) within one of [`nal_ranges`]'s slices.
pub fn nal_payload(nal: &[u8]) -> &[u8] {
    let skip = if nal.starts_with(&[0, 0, 0, 1]) {
        4
    } else if nal.starts_with(&[0, 0, 1]) {
        3
    } else {
        0
    };
    &nal[skip..]
}

/// One access unit of an Annex-B stream.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AccessUnit {
    /// Byte range in the source stream, start codes included.
    pub range: Range<usize>,
    /// Contains a VCL NAL unit (a picture). An access unit without one is parameter sets or SEI
    /// alone, which happens at the head of a stream.
    pub has_picture: bool,
    pub is_keyframe: bool,
    pub has_parameter_sets: bool,
}

/// Cut an Annex-B stream into access units. The rule is the spec's: a new access unit begins
/// at an AUD, a parameter set or a prefix SEI that follows a VCL NAL unit, or at a VCL NAL unit
/// that is the first slice of a picture when the current unit already holds a picture.
pub fn annexb_access_units(
    data: &[u8],
    codec: NalCodec,
) -> Result<Vec<AccessUnit>, BitstreamError> {
    let nals = nal_ranges(data);
    if nals.is_empty() {
        return Err(BitstreamError::NotAnnexB(data.len().min(64)));
    }
    let mut units: Vec<AccessUnit> = Vec::new();
    let mut current: Option<AccessUnit> = None;
    for range in nals {
        let payload = nal_payload(&data[range.clone()]);
        let t = codec.nal_type(payload).unwrap_or(0);
        let vcl = codec.is_vcl(t);
        let closes = match &current {
            Some(au) if au.has_picture => {
                codec.starts_access_unit(t) || (vcl && codec.first_slice_of_picture(payload))
            }
            _ => false,
        };
        if closes {
            units.extend(current.take());
        }
        let au = current.get_or_insert_with(|| AccessUnit {
            range: range.start..range.start,
            has_picture: false,
            is_keyframe: false,
            has_parameter_sets: false,
        });
        au.range.end = range.end;
        if vcl {
            if !au.has_picture {
                au.is_keyframe = codec.is_keyframe_type(t);
            }
            au.has_picture = true;
        }
        if codec.is_parameter_set(t) {
            au.has_parameter_sets = true;
        }
    }
    units.extend(current);
    Ok(units)
}

/// Split one access unit into its parameter-set NAL units and the rest, both as Annex-B bytes.
/// For H.264 the parameter sets come back as `(SPS.., PPS..)`, the two `csd` buffers; for H.265
/// everything (VPS, SPS, PPS) is in the first and the second is empty.
pub fn split_parameter_sets(au: &[u8], codec: NalCodec) -> (Vec<u8>, Vec<u8>, Vec<u8>) {
    let mut csd0 = Vec::new();
    let mut csd1 = Vec::new();
    let mut rest = Vec::new();
    for range in nal_ranges(au) {
        let nal = &au[range];
        let t = codec.nal_type(nal_payload(nal)).unwrap_or(0);
        let sink = match (codec, t) {
            (NalCodec::H264, 7) => &mut csd0,
            (NalCodec::H264, 8) => &mut csd1,
            (NalCodec::H265, 32..=34) => &mut csd0,
            _ => &mut rest,
        };
        sink.extend_from_slice(nal);
    }
    (csd0, csd1, rest)
}

/// The fixed IVF file header.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IvfHeader {
    pub fourcc: [u8; 4],
    pub width: u16,
    pub height: u16,
    /// Timestamps are in units of `timebase_num / timebase_den` seconds.
    pub timebase_den: u32,
    pub timebase_num: u32,
    pub frame_count: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IvfFrame {
    pub timestamp: u64,
    pub range: Range<usize>,
}

impl IvfHeader {
    pub const SIZE: usize = 32;

    /// The MediaCodec mime for this container's codec, if it is one we know.
    pub fn mime(&self) -> Option<&'static str> {
        mime_for_fourcc(&self.fourcc)
    }

    /// A frame timestamp in microseconds.
    pub fn timestamp_us(&self, timestamp: u64) -> u64 {
        if self.timebase_den == 0 {
            return timestamp;
        }
        (timestamp as u128 * self.timebase_num.max(1) as u128 * 1_000_000
            / self.timebase_den as u128) as u64
    }

    pub fn to_bytes(&self) -> [u8; Self::SIZE] {
        let mut out = [0u8; Self::SIZE];
        out[0..4].copy_from_slice(b"DKIF");
        out[4..6].copy_from_slice(&0u16.to_le_bytes());
        out[6..8].copy_from_slice(&(Self::SIZE as u16).to_le_bytes());
        out[8..12].copy_from_slice(&self.fourcc);
        out[12..14].copy_from_slice(&self.width.to_le_bytes());
        out[14..16].copy_from_slice(&self.height.to_le_bytes());
        out[16..20].copy_from_slice(&self.timebase_den.to_le_bytes());
        out[20..24].copy_from_slice(&self.timebase_num.to_le_bytes());
        out[24..28].copy_from_slice(&self.frame_count.to_le_bytes());
        out
    }
}

pub fn mime_for_fourcc(fourcc: &[u8; 4]) -> Option<&'static str> {
    match fourcc {
        b"VP80" => Some("video/x-vnd.on2.vp8"),
        b"VP90" => Some("video/x-vnd.on2.vp9"),
        b"AV01" => Some("video/av01"),
        _ => None,
    }
}

pub fn fourcc_for_mime(mime: &str) -> Option<[u8; 4]> {
    match mime {
        "video/x-vnd.on2.vp8" => Some(*b"VP80"),
        "video/x-vnd.on2.vp9" => Some(*b"VP90"),
        "video/av01" => Some(*b"AV01"),
        _ => None,
    }
}

/// Whether an IVF frame is a key frame, from the first byte of its uncompressed header: VP8's
/// frame tag has `key_frame` in bit 0 (0 = key); VP9's has `frame_marker` in the top two bits,
/// then the profile bits, `show_existing_frame` and `frame_type` (0 = key) -- profile 3 carries
/// an extra reserved bit before those two. AV1 would need OBU parsing; `None`.
pub fn ivf_keyframe(fourcc: &[u8; 4], frame: &[u8]) -> Option<bool> {
    let b0 = *frame.first()?;
    match fourcc {
        b"VP80" => Some(b0 & 0x01 == 0),
        b"VP90" => {
            let profile = ((b0 >> 5) & 1) | ((b0 >> 3) & 2);
            let (show_existing, frame_type) = if profile == 3 {
                ((b0 >> 2) & 1, (b0 >> 1) & 1)
            } else {
                ((b0 >> 3) & 1, (b0 >> 2) & 1)
            };
            Some(show_existing == 0 && frame_type == 0)
        }
        _ => None,
    }
}

fn u16_at(d: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([d[at], d[at + 1]])
}

fn u32_at(d: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([d[at], d[at + 1], d[at + 2], d[at + 3]])
}

/// Parse an IVF file into its header and frame table. Frame data stays in `data`.
pub fn read_ivf(data: &[u8]) -> Result<(IvfHeader, Vec<IvfFrame>), BitstreamError> {
    if data.len() < 8 || &data[0..4] != b"DKIF" {
        return Err(BitstreamError::NotIvf);
    }
    let header_size = u16_at(data, 6) as usize;
    if header_size < IvfHeader::SIZE || data.len() < header_size {
        return Err(BitstreamError::IvfHeaderShort(header_size, data.len()));
    }
    let header = IvfHeader {
        fourcc: [data[8], data[9], data[10], data[11]],
        width: u16_at(data, 12),
        height: u16_at(data, 14),
        timebase_den: u32_at(data, 16),
        timebase_num: u32_at(data, 20),
        frame_count: u32_at(data, 24),
    };
    let mut frames = Vec::new();
    let mut at = header_size;
    while at < data.len() {
        if at + 12 > data.len() {
            return Err(BitstreamError::IvfFrameTruncated(at));
        }
        let size = u32_at(data, at) as usize;
        let timestamp = u32_at(data, at + 4) as u64 | ((u32_at(data, at + 8) as u64) << 32);
        let start = at + 12;
        let end = start + size;
        if end > data.len() {
            return Err(BitstreamError::IvfFrameTruncated(at));
        }
        frames.push(IvfFrame {
            timestamp,
            range: start..end,
        });
        at = end;
    }
    Ok((header, frames))
}

/// Serialise frames as an IVF file. `frames` are `(timestamp, bytes)` in the header's timebase.
pub fn write_ivf(header: &IvfHeader, frames: &[(u64, &[u8])]) -> Vec<u8> {
    let mut header = *header;
    header.frame_count = frames.len() as u32;
    let mut out = Vec::with_capacity(
        IvfHeader::SIZE + frames.iter().map(|(_, f)| f.len() + 12).sum::<usize>(),
    );
    out.extend_from_slice(&header.to_bytes());
    for (ts, bytes) in frames {
        out.extend_from_slice(&(bytes.len() as u32).to_le_bytes());
        out.extend_from_slice(&ts.to_le_bytes());
        out.extend_from_slice(bytes);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn nal(codec: NalCodec, t: u8, first_slice: bool, body: &[u8]) -> Vec<u8> {
        let mut out = vec![0, 0, 0, 1];
        match codec {
            NalCodec::H264 => out.push(0x60 | t),
            NalCodec::H265 => out.extend_from_slice(&[t << 1, 0x01]),
        }
        out.push(if first_slice { 0x80 } else { 0x40 } | 0x0a);
        out.extend_from_slice(body);
        out
    }

    #[test]
    fn nal_ranges_handles_both_start_code_lengths() {
        let data = [
            0, 0, 0, 1, 0x67, 0xAA, 0, 0, 1, 0x68, 0xBB, 0xCC, 0, 0, 0, 1, 0x65, 0x88,
        ];
        let ranges = nal_ranges(&data);
        assert_eq!(ranges, vec![0..6, 6..12, 12..18]);
        assert_eq!(nal_payload(&data[ranges[0].clone()]), &[0x67, 0xAA]);
        assert_eq!(nal_payload(&data[ranges[1].clone()]), &[0x68, 0xBB, 0xCC]);
        assert_eq!(nal_payload(&data[ranges[2].clone()]), &[0x65, 0x88]);
    }

    #[test]
    fn h264_access_units_split_on_first_slice_and_parameter_sets() {
        let c = NalCodec::H264;
        let mut stream = Vec::new();
        let sps = nal(c, 7, true, &[1, 2]);
        let pps = nal(c, 8, true, &[3]);
        let idr_a = nal(c, 5, true, &[9, 9]);
        let idr_b = nal(c, 5, false, &[9, 9]); // second slice of the same picture
        let p1 = nal(c, 1, true, &[7]);
        let sei = nal(c, 6, true, &[5]);
        let p2 = nal(c, 1, true, &[8]);
        let aud = nal(c, 9, true, &[]);
        let idr2 = nal(c, 5, true, &[1]);
        for part in [&sps, &pps, &idr_a, &idr_b, &p1, &sei, &p2, &aud, &idr2] {
            stream.extend_from_slice(part);
        }
        let units = annexb_access_units(&stream, c).unwrap();
        assert_eq!(units.len(), 4, "{units:?}");
        // AU 0: SPS PPS IDR(2 slices)
        assert_eq!(
            units[0].range,
            0..sps.len() + pps.len() + idr_a.len() + idr_b.len()
        );
        assert!(units[0].has_picture && units[0].is_keyframe && units[0].has_parameter_sets);
        // AU 1: P
        assert_eq!(units[1].range.len(), p1.len());
        assert!(units[1].has_picture && !units[1].is_keyframe && !units[1].has_parameter_sets);
        // AU 2: SEI P  (the SEI after a VCL closes the previous unit and opens this one)
        assert_eq!(units[2].range.len(), sei.len() + p2.len());
        assert!(!units[2].is_keyframe);
        // AU 3: AUD IDR
        assert_eq!(units[3].range.len(), aud.len() + idr2.len());
        assert!(units[3].is_keyframe);
        assert_eq!(units[3].range.end, stream.len());
    }

    #[test]
    fn h265_access_units_and_irap() {
        let c = NalCodec::H265;
        let mut stream = Vec::new();
        let vps = nal(c, 32, true, &[]);
        let sps = nal(c, 33, true, &[]);
        let pps = nal(c, 34, true, &[]);
        let idr = nal(c, 19, true, &[1]); // IDR_W_RADL
        let trail_a = nal(c, 1, true, &[2]); // TRAIL_R, first slice segment
        let trail_b = nal(c, 1, false, &[2]); // dependent slice segment
        let cra = nal(c, 21, true, &[3]);
        for part in [&vps, &sps, &pps, &idr, &trail_a, &trail_b, &cra] {
            stream.extend_from_slice(part);
        }
        let units = annexb_access_units(&stream, c).unwrap();
        assert_eq!(units.len(), 3, "{units:?}");
        assert!(units[0].is_keyframe && units[0].has_parameter_sets);
        assert_eq!(units[1].range.len(), trail_a.len() + trail_b.len());
        assert!(!units[1].is_keyframe);
        assert!(units[2].is_keyframe);

        let (csd0, csd1, rest) = split_parameter_sets(&stream[units[0].range.clone()], c);
        assert_eq!(csd0.len(), vps.len() + sps.len() + pps.len());
        assert!(csd1.is_empty());
        assert_eq!(rest, idr);
    }

    #[test]
    fn h264_parameter_sets_split_into_two_csd_buffers() {
        let c = NalCodec::H264;
        let sps = nal(c, 7, true, &[1]);
        let pps = nal(c, 8, true, &[2]);
        let idr = nal(c, 5, true, &[3]);
        let mut au = sps.clone();
        au.extend_from_slice(&pps);
        au.extend_from_slice(&idr);
        let (csd0, csd1, rest) = split_parameter_sets(&au, c);
        assert_eq!(csd0, sps);
        assert_eq!(csd1, pps);
        assert_eq!(rest, idr);
    }

    #[test]
    fn parameter_sets_alone_form_a_pictureless_unit() {
        let c = NalCodec::H264;
        let mut stream = nal(c, 7, true, &[1]);
        stream.extend_from_slice(&nal(c, 8, true, &[2]));
        let units = annexb_access_units(&stream, c).unwrap();
        assert_eq!(units.len(), 1);
        assert!(!units[0].has_picture && units[0].has_parameter_sets);
        assert_eq!(
            annexb_access_units(b"nothing here", c),
            Err(BitstreamError::NotAnnexB(12))
        );
    }

    #[test]
    fn ivf_round_trip() {
        let header = IvfHeader {
            fourcc: *b"VP90",
            width: 640,
            height: 360,
            timebase_den: 30,
            timebase_num: 1,
            frame_count: 0,
        };
        let f0 = [0x82u8, 0x49, 0x83, 0x42];
        let f1 = [0x86u8, 0x00];
        let bytes = write_ivf(&header, &[(0, &f0), (1, &f1)]);
        assert_eq!(bytes.len(), 32 + 12 + 4 + 12 + 2);
        let (h, frames) = read_ivf(&bytes).unwrap();
        assert_eq!(h.fourcc, *b"VP90");
        assert_eq!(h.mime(), Some("video/x-vnd.on2.vp9"));
        assert_eq!((h.width, h.height), (640, 360));
        assert_eq!(h.frame_count, 2);
        assert_eq!(frames.len(), 2);
        assert_eq!(&bytes[frames[0].range.clone()], &f0);
        assert_eq!(&bytes[frames[1].range.clone()], &f1);
        assert_eq!(frames[1].timestamp, 1);
        assert_eq!(h.timestamp_us(1), 33_333);
        assert_eq!(h.timestamp_us(30), 1_000_000);
    }

    #[test]
    fn ivf_rejects_bad_files() {
        assert_eq!(read_ivf(b"RIFF....").unwrap_err(), BitstreamError::NotIvf);
        let header = IvfHeader {
            fourcc: *b"VP80",
            width: 16,
            height: 16,
            timebase_den: 1,
            timebase_num: 1,
            frame_count: 1,
        };
        let mut bytes = write_ivf(&header, &[(0, &[1, 2, 3, 4])]);
        bytes.truncate(bytes.len() - 1);
        assert_eq!(
            read_ivf(&bytes).unwrap_err(),
            BitstreamError::IvfFrameTruncated(32)
        );
        assert!(matches!(
            read_ivf(&bytes[..20]).unwrap_err(),
            BitstreamError::IvfHeaderShort(32, 20)
        ));
    }

    #[test]
    fn ivf_keyframe_bits() {
        // VP8: bit 0 of the frame tag is 0 for a key frame.
        assert_eq!(ivf_keyframe(b"VP80", &[0x50, 0x42, 0x00]), Some(true));
        assert_eq!(ivf_keyframe(b"VP80", &[0x51, 0x42, 0x00]), Some(false));
        // VP9 profile 0: 10 0 0 | show_existing 0 | frame_type 0 -> 0x82 key, 0x86 inter.
        assert_eq!(ivf_keyframe(b"VP90", &[0x82, 0x49, 0x83]), Some(true));
        assert_eq!(ivf_keyframe(b"VP90", &[0x86, 0x00]), Some(false));
        assert_eq!(
            ivf_keyframe(b"VP90", &[0x8a, 0x00]),
            Some(false),
            "show_existing_frame"
        );
        // VP9 profile 3 (both profile bits set, reserved bit, then the two flags).
        assert_eq!(ivf_keyframe(b"VP90", &[0xb0, 0x00]), Some(true));
        assert_eq!(ivf_keyframe(b"VP90", &[0xb2, 0x00]), Some(false));
        assert_eq!(ivf_keyframe(b"AV01", &[0x12, 0x00]), None);
        assert_eq!(ivf_keyframe(b"VP80", &[]), None);
    }

    #[test]
    fn fourcc_mime_table_is_symmetric() {
        for mime in ["video/x-vnd.on2.vp8", "video/x-vnd.on2.vp9", "video/av01"] {
            let fourcc = fourcc_for_mime(mime).unwrap();
            assert_eq!(mime_for_fourcc(&fourcc), Some(mime));
        }
        assert_eq!(fourcc_for_mime("video/avc"), None);
        assert_eq!(NalCodec::for_mime("video/hevc"), Some(NalCodec::H265));
    }
}
