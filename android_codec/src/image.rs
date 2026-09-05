// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

//! The `MediaImage2` plane description a decoder attaches to each output buffer, and the repack
//! from whatever layout it describes into tight NV12.
//!
//! There is no `AMediaCodec_getOutputImage` in the NDK. The plane layout of a `YUV420Flexible`
//! output buffer reaches a client as a 104-byte blob under the key `"image-data"` in the
//! per-buffer format (`AMediaCodec_getBufferFormat` + `AMediaFormat_getBuffer`), and that blob is
//! `struct MediaImage2` from `frameworks/native/headers/media_plugin/media/hardware/VideoAPI.h`:
//!
//! ```text
//! struct __attribute__((__packed__)) MediaImage2 {
//!     uint32_t mType;               // 0 unknown, 1 YUV, 2 YUVA, 3 RGB, 4 RGBA, 5 Y
//!     uint32_t mNumPlanes;
//!     uint32_t mWidth;              // unpadded
//!     uint32_t mHeight;             // unpadded
//!     uint32_t mBitDepth;
//!     uint32_t mBitDepthAllocated;  // 8 or 16
//!     struct __attribute__((__packed__)) PlaneInfo {
//!         uint32_t mOffset;         // first pixel of the plane, in bytes from the buffer start
//!         int32_t  mColInc;         // column increment in bytes
//!         int32_t  mRowInc;         // row increment in bytes
//!         uint32_t mHorizSubsampling;
//!         uint32_t mVertSubsampling;
//!     } mPlane[4];                  // Y U V (A); always four, so the struct is always 104 bytes
//! };
//! ```
//!
//! It is parsed field by field rather than transmuted: the blob is packed, so a `repr(C, packed)`
//! mirror would only trade an unaligned read for the same twenty little-endian loads.

use serde::Serialize;
use thiserror::Error;

/// `sizeof(MediaImage2)`, asserted in `VideoAPI.h`.
pub const MEDIA_IMAGE2_SIZE: usize = 104;

/// `MediaImage2::Type` values.
pub const MEDIA_IMAGE_TYPE_UNKNOWN: u32 = 0;
pub const MEDIA_IMAGE_TYPE_YUV: u32 = 1;
pub const MEDIA_IMAGE_TYPE_YUVA: u32 = 2;
pub const MEDIA_IMAGE_TYPE_RGB: u32 = 3;
pub const MEDIA_IMAGE_TYPE_RGBA: u32 = 4;
pub const MEDIA_IMAGE_TYPE_Y: u32 = 5;

#[derive(Error, Debug, Clone, PartialEq, Eq)]
pub enum ImageError {
    #[error("image-data is {0} bytes, MediaImage2 is {MEDIA_IMAGE2_SIZE}")]
    WrongSize(usize),
    #[error("not an 8-bit three-plane YUV image (type {image_type}, {num_planes} planes, {bit_depth_allocated} bits allocated)")]
    NotYuv420 {
        image_type: u32,
        num_planes: u32,
        bit_depth_allocated: u32,
    },
    #[error("chroma subsampling {0:?} is not 4:2:0")]
    NotSubsampled420((u32, u32, u32, u32)),
    #[error("plane {plane} reaches byte {needed} of a {available} byte buffer")]
    OutOfBounds {
        plane: usize,
        needed: usize,
        available: usize,
    },
    #[error("image has no pixels ({0}x{1})")]
    Empty(u32, u32),
}

/// One plane of a [`MediaImage2`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize)]
pub struct PlaneInfo {
    pub offset: u32,
    pub col_inc: i32,
    pub row_inc: i32,
    pub horiz_subsampling: u32,
    pub vert_subsampling: u32,
}

/// The parsed `"image-data"` blob.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize)]
pub struct MediaImage2 {
    pub image_type: u32,
    pub num_planes: u32,
    pub width: u32,
    pub height: u32,
    pub bit_depth: u32,
    pub bit_depth_allocated: u32,
    pub planes: [PlaneInfo; 4],
}

/// What the three chroma plane descriptors amount to, for the copy loop and for the report.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum ChromaLayout {
    /// Interleaved, Cb first: one memcpy per chroma row.
    Nv12,
    /// Interleaved, Cr first: byte swap per pair.
    Nv21,
    /// Two separate planes with a column increment of one (I420 or YV12).
    Planar,
    /// Anything else: the generic per-pixel walk.
    Other,
}

fn u32_at(bytes: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([bytes[at], bytes[at + 1], bytes[at + 2], bytes[at + 3]])
}

impl MediaImage2 {
    /// Parse the 104-byte blob. A longer buffer is accepted (only the first 104 bytes are read);
    /// a shorter one is not.
    pub fn parse(bytes: &[u8]) -> Result<MediaImage2, ImageError> {
        if bytes.len() < MEDIA_IMAGE2_SIZE {
            return Err(ImageError::WrongSize(bytes.len()));
        }
        let mut planes = [PlaneInfo::default(); 4];
        for (i, plane) in planes.iter_mut().enumerate() {
            let base = 24 + i * 20;
            *plane = PlaneInfo {
                offset: u32_at(bytes, base),
                col_inc: u32_at(bytes, base + 4) as i32,
                row_inc: u32_at(bytes, base + 8) as i32,
                horiz_subsampling: u32_at(bytes, base + 12),
                vert_subsampling: u32_at(bytes, base + 16),
            };
        }
        Ok(MediaImage2 {
            image_type: u32_at(bytes, 0),
            num_planes: u32_at(bytes, 4),
            width: u32_at(bytes, 8),
            height: u32_at(bytes, 12),
            bit_depth: u32_at(bytes, 16),
            bit_depth_allocated: u32_at(bytes, 20),
            planes,
        })
    }

    /// Width and height in samples of plane `i`, after subsampling (rounded up, so an odd width
    /// still has a chroma sample for its last column).
    pub fn plane_dims(&self, i: usize) -> (usize, usize) {
        let p = &self.planes[i];
        let hs = p.horiz_subsampling.max(1) as usize;
        let vs = p.vert_subsampling.max(1) as usize;
        (
            (self.width as usize).div_ceil(hs),
            (self.height as usize).div_ceil(vs),
        )
    }

    /// The first byte past the last sample of any plane: the buffer must be at least this long
    /// for the layout to be readable. This is the bound `MediaCodec_sanity_test.cpp` checks for
    /// the flexible layout.
    pub fn required_size(&self) -> usize {
        let mut needed = 0usize;
        for i in 0..(self.num_planes.min(4) as usize) {
            needed = needed.max(self.plane_end(i));
        }
        needed
    }

    /// The first byte past plane `i`'s last sample.
    pub fn plane_end(&self, i: usize) -> usize {
        let (w, h) = self.plane_dims(i);
        if w == 0 || h == 0 {
            return 0;
        }
        let p = &self.planes[i];
        // Either increment may be negative (bottom-up rows), in which case that axis extends
        // below the offset and the highest byte is reached along the other axis alone.
        let rows = ((h as i64 - 1) * p.row_inc as i64).max(0);
        let cols = ((w as i64 - 1) * p.col_inc as i64).max(0);
        (p.offset as i64 + rows + cols + 1) as usize
    }

    /// The same layout with every plane offset reduced by plane 0's, for the case where
    /// `AMediaCodec_getOutputBuffer` has already advanced the pointer past `mPlane[0].mOffset`
    /// (the framework's `CCodecBuffers::handleImageData` calls `setRange(mPlane[0].mOffset, ..)`,
    /// and the NDK returns `data()`, which is base + offset). Which interpretation is right is
    /// what the probe measures.
    pub fn rebased(&self) -> MediaImage2 {
        let base = self.planes[0].offset;
        let mut out = *self;
        for p in out.planes.iter_mut() {
            p.offset = p.offset.saturating_sub(base);
        }
        out
    }

    /// Classify the chroma planes.
    pub fn chroma_layout(&self) -> ChromaLayout {
        if self.num_planes < 3 {
            return ChromaLayout::Other;
        }
        let u = &self.planes[1];
        let v = &self.planes[2];
        if u.col_inc == 2 && v.col_inc == 2 && u.row_inc == v.row_inc {
            if v.offset == u.offset.wrapping_add(1) {
                return ChromaLayout::Nv12;
            }
            if u.offset == v.offset.wrapping_add(1) {
                return ChromaLayout::Nv21;
            }
        }
        if u.col_inc == 1 && v.col_inc == 1 {
            return ChromaLayout::Planar;
        }
        ChromaLayout::Other
    }

    fn check_yuv420(&self) -> Result<(), ImageError> {
        if self.image_type != MEDIA_IMAGE_TYPE_YUV
            || self.num_planes < 3
            || self.bit_depth_allocated != 8
        {
            return Err(ImageError::NotYuv420 {
                image_type: self.image_type,
                num_planes: self.num_planes,
                bit_depth_allocated: self.bit_depth_allocated,
            });
        }
        if self.width == 0 || self.height == 0 {
            return Err(ImageError::Empty(self.width, self.height));
        }
        let (u, v) = (&self.planes[1], &self.planes[2]);
        let sub = (
            u.horiz_subsampling,
            u.vert_subsampling,
            v.horiz_subsampling,
            v.vert_subsampling,
        );
        if sub != (2, 2, 2, 2) {
            return Err(ImageError::NotSubsampled420(sub));
        }
        Ok(())
    }

    fn check_bounds(&self, available: usize) -> Result<(), ImageError> {
        for plane in 0..3 {
            let needed = self.plane_end(plane);
            if needed > available {
                return Err(ImageError::OutOfBounds {
                    plane,
                    needed,
                    available,
                });
            }
        }
        Ok(())
    }
}

/// Size of a tight NV12 frame of `width` x `height`: the luma plane followed by one interleaved
/// chroma plane of `ceil(w/2) * 2` bytes per `ceil(h/2)` rows.
pub fn nv12_size(width: usize, height: usize) -> usize {
    width * height + 2 * width.div_ceil(2) * height.div_ceil(2)
}

/// Repack the buffer `src`, laid out as `image` says, into `dst` as tight NV12
/// (`bytesperline == width`, chroma immediately after luma, Cb before Cr). `dst` is cleared
/// first and ends up exactly [`nv12_size`] bytes long.
///
/// This is the one copy the decoder device makes per frame (design 7.2, CAPTURE format policy):
/// a memcpy per row when the source is already NV12, a byte swap per pair for NV21, an
/// interleave for planar sources, and the generic `offset + x*colInc + y*rowInc` walk for
/// anything else.
pub fn tight_nv12(image: &MediaImage2, src: &[u8], dst: &mut Vec<u8>) -> Result<(), ImageError> {
    image.check_yuv420()?;
    image.check_bounds(src.len())?;

    let w = image.width as usize;
    let h = image.height as usize;
    let (cw, ch) = image.plane_dims(1);
    dst.clear();
    dst.reserve(nv12_size(w, h));

    let y = &image.planes[0];
    for row in 0..h {
        let start = (y.offset as i64 + row as i64 * y.row_inc as i64) as usize;
        if y.col_inc == 1 {
            dst.extend_from_slice(&src[start..start + w]);
        } else {
            for col in 0..w {
                dst.push(src[(start as i64 + col as i64 * y.col_inc as i64) as usize]);
            }
        }
    }

    let u = &image.planes[1];
    let v = &image.planes[2];
    match image.chroma_layout() {
        ChromaLayout::Nv12 => {
            for row in 0..ch {
                let start = (u.offset as i64 + row as i64 * u.row_inc as i64) as usize;
                dst.extend_from_slice(&src[start..start + 2 * cw]);
            }
        }
        ChromaLayout::Nv21 => {
            for row in 0..ch {
                let start = (v.offset as i64 + row as i64 * v.row_inc as i64) as usize;
                for pair in src[start..start + 2 * cw].chunks_exact(2) {
                    dst.push(pair[1]);
                    dst.push(pair[0]);
                }
            }
        }
        ChromaLayout::Planar => {
            for row in 0..ch {
                let us = (u.offset as i64 + row as i64 * u.row_inc as i64) as usize;
                let vs = (v.offset as i64 + row as i64 * v.row_inc as i64) as usize;
                for col in 0..cw {
                    dst.push(src[us + col]);
                    dst.push(src[vs + col]);
                }
            }
        }
        ChromaLayout::Other => {
            for row in 0..ch {
                for col in 0..cw {
                    let ui = u.offset as i64
                        + row as i64 * u.row_inc as i64
                        + col as i64 * u.col_inc as i64;
                    let vi = v.offset as i64
                        + row as i64 * v.row_inc as i64
                        + col as i64 * v.col_inc as i64;
                    dst.push(src[ui as usize]);
                    dst.push(src[vi as usize]);
                }
            }
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn put_u32(out: &mut Vec<u8>, v: u32) {
        out.extend_from_slice(&v.to_le_bytes());
    }

    /// Build the blob the way the framework lays it out in memory: six header words, then four
    /// packed planes of five words each.
    fn blob(
        image_type: u32,
        num_planes: u32,
        w: u32,
        h: u32,
        planes: &[(u32, i32, i32, u32, u32)],
    ) -> Vec<u8> {
        let mut out = Vec::new();
        put_u32(&mut out, image_type);
        put_u32(&mut out, num_planes);
        put_u32(&mut out, w);
        put_u32(&mut out, h);
        put_u32(&mut out, 8);
        put_u32(&mut out, 8);
        for i in 0..4 {
            let (off, ci, ri, hs, vs) = planes.get(i).copied().unwrap_or((0, 0, 0, 0, 0));
            put_u32(&mut out, off);
            put_u32(&mut out, ci as u32);
            put_u32(&mut out, ri as u32);
            put_u32(&mut out, hs);
            put_u32(&mut out, vs);
        }
        assert_eq!(out.len(), MEDIA_IMAGE2_SIZE);
        out
    }

    /// A 6x4 NV12 frame padded to stride 8 and slice height 6: Y at 0, interleaved chroma at
    /// 48 (= stride * slice height), Cr one byte after Cb.
    fn nv12_6x4_padded() -> (MediaImage2, Vec<u8>) {
        let image = MediaImage2::parse(&blob(
            MEDIA_IMAGE_TYPE_YUV,
            3,
            6,
            4,
            &[(0, 1, 8, 1, 1), (48, 2, 8, 2, 2), (49, 2, 8, 2, 2)],
        ))
        .unwrap();
        let mut src = vec![0xEEu8; 48 + 8 * 2];
        for row in 0..4 {
            for col in 0..6 {
                src[row * 8 + col] = (row * 16 + col) as u8;
            }
        }
        for row in 0..2 {
            for col in 0..3 {
                src[48 + row * 8 + col * 2] = 0xA0 + (row * 4 + col) as u8; // Cb
                src[48 + row * 8 + col * 2 + 1] = 0xB0 + (row * 4 + col) as u8; // Cr
            }
        }
        (image, src)
    }

    fn expected_6x4() -> Vec<u8> {
        let mut want = Vec::new();
        for row in 0..4 {
            for col in 0..6 {
                want.push((row * 16 + col) as u8);
            }
        }
        for row in 0..2 {
            for col in 0..3 {
                want.push(0xA0 + (row * 4 + col) as u8);
                want.push(0xB0 + (row * 4 + col) as u8);
            }
        }
        assert_eq!(want.len(), nv12_size(6, 4));
        want
    }

    #[test]
    fn parses_the_104_byte_blob() {
        let bytes = blob(
            MEDIA_IMAGE_TYPE_YUV,
            3,
            1280,
            720,
            &[
                (0, 1, 1280, 1, 1),
                (1280 * 736, 2, 1280, 2, 2),
                (1280 * 736 + 1, 2, 1280, 2, 2),
            ],
        );
        let image = MediaImage2::parse(&bytes).unwrap();
        assert_eq!(image.image_type, MEDIA_IMAGE_TYPE_YUV);
        assert_eq!(image.num_planes, 3);
        assert_eq!((image.width, image.height), (1280, 720));
        assert_eq!((image.bit_depth, image.bit_depth_allocated), (8, 8));
        assert_eq!(
            image.planes[0],
            PlaneInfo {
                offset: 0,
                col_inc: 1,
                row_inc: 1280,
                horiz_subsampling: 1,
                vert_subsampling: 1
            }
        );
        assert_eq!(image.planes[1].offset, 1280 * 736);
        assert_eq!(image.planes[2].offset, 1280 * 736 + 1);
        assert_eq!(image.planes[2].col_inc, 2);
        assert_eq!(image.planes[3], PlaneInfo::default());
        assert_eq!(image.chroma_layout(), ChromaLayout::Nv12);
        assert_eq!(image.plane_dims(1), (640, 360));
        // Last Cr sample: offset + 359 rows + 639 cols * 2, plus one.
        assert_eq!(
            image.required_size(),
            1280 * 736 + 1 + 359 * 1280 + 639 * 2 + 1
        );
        // A signed row increment survives the round trip.
        let mut flipped = bytes.clone();
        flipped[24 + 8..24 + 12].copy_from_slice(&(-1280i32).to_le_bytes());
        assert_eq!(
            MediaImage2::parse(&flipped).unwrap().planes[0].row_inc,
            -1280
        );
    }

    #[test]
    fn rejects_a_short_blob() {
        assert_eq!(
            MediaImage2::parse(&[0u8; 100]),
            Err(ImageError::WrongSize(100))
        );
        assert!(MediaImage2::parse(&[0u8; 120]).is_ok());
    }

    #[test]
    fn rebased_subtracts_plane_zero_offset() {
        let image = MediaImage2::parse(&blob(
            MEDIA_IMAGE_TYPE_YUV,
            3,
            4,
            2,
            &[(64, 1, 4, 1, 1), (72, 2, 4, 2, 2), (73, 2, 4, 2, 2)],
        ))
        .unwrap();
        let r = image.rebased();
        assert_eq!(r.planes[0].offset, 0);
        assert_eq!(r.planes[1].offset, 8);
        assert_eq!(r.planes[2].offset, 9);
        // Chroma is 2x1 samples here: the Cr plane ends one column increment past its offset.
        assert_eq!(image.required_size(), 73 + 2 + 1);
        assert_eq!(r.required_size(), 9 + 2 + 1);
    }

    #[test]
    fn tight_nv12_from_padded_nv12() {
        let (image, src) = nv12_6x4_padded();
        let mut dst = Vec::new();
        tight_nv12(&image, &src, &mut dst).unwrap();
        assert_eq!(dst, expected_6x4());
    }

    #[test]
    fn tight_nv12_from_nv21_swaps_chroma() {
        let (mut image, mut src) = nv12_6x4_padded();
        // Same bytes, but now the descriptor says Cr comes first: swap the descriptor and the
        // data so the tight output must still be Cb-first.
        image.planes[1].offset = 49;
        image.planes[2].offset = 48;
        for row in 0..2 {
            for col in 0..3 {
                src.swap(48 + row * 8 + col * 2, 48 + row * 8 + col * 2 + 1);
            }
        }
        assert_eq!(image.chroma_layout(), ChromaLayout::Nv21);
        let mut dst = Vec::new();
        tight_nv12(&image, &src, &mut dst).unwrap();
        assert_eq!(dst, expected_6x4());
    }

    #[test]
    fn tight_nv12_from_i420_interleaves() {
        // Planar 6x4 with stride 8: Y at 0 (4 rows), U at 32 (2 rows of stride 4), V at 40.
        let image = MediaImage2::parse(&blob(
            MEDIA_IMAGE_TYPE_YUV,
            3,
            6,
            4,
            &[(0, 1, 8, 1, 1), (32, 1, 4, 2, 2), (40, 1, 4, 2, 2)],
        ))
        .unwrap();
        assert_eq!(image.chroma_layout(), ChromaLayout::Planar);
        let mut src = vec![0xEEu8; 48];
        for row in 0..4 {
            for col in 0..6 {
                src[row * 8 + col] = (row * 16 + col) as u8;
            }
        }
        for row in 0..2 {
            for col in 0..3 {
                src[32 + row * 4 + col] = 0xA0 + (row * 4 + col) as u8;
                src[40 + row * 4 + col] = 0xB0 + (row * 4 + col) as u8;
            }
        }
        let mut dst = Vec::new();
        tight_nv12(&image, &src, &mut dst).unwrap();
        assert_eq!(dst, expected_6x4());
    }

    #[test]
    fn tight_nv12_generic_walk_matches_fast_path() {
        // Same NV12 data, but described with a column increment the fast paths do not
        // recognise (chroma rows given as separate planes with colInc 2 and different row
        // increments), so the generic walk is what runs.
        let (mut image, src) = nv12_6x4_padded();
        image.planes[1].row_inc = 8;
        image.planes[2].row_inc = 8;
        image.planes[2].offset = 49;
        image.planes[1].col_inc = 2;
        image.planes[2].col_inc = 2;
        // Break the "u.row_inc == v.row_inc" test without changing the data: a Cr plane read
        // with a negative row increment from the last row is the same bytes.
        image.planes[2].offset = 49 + 8;
        image.planes[2].row_inc = -8;
        assert_eq!(image.chroma_layout(), ChromaLayout::Other);
        let mut dst = Vec::new();
        tight_nv12(&image, &src, &mut dst).unwrap();
        let mut want = expected_6x4();
        // Cr rows come out reversed under the negative increment; fix the expectation.
        let luma = 24;
        for col in 0..3 {
            want.swap(luma + col * 2 + 1, luma + 6 + col * 2 + 1);
        }
        assert_eq!(dst, want);
    }

    #[test]
    fn tight_nv12_odd_size() {
        // 3x3: chroma is 2x2 samples, so tight NV12 is 9 + 8 bytes.
        let image = MediaImage2::parse(&blob(
            MEDIA_IMAGE_TYPE_YUV,
            3,
            3,
            3,
            &[(0, 1, 4, 1, 1), (16, 2, 4, 2, 2), (17, 2, 4, 2, 2)],
        ))
        .unwrap();
        let src: Vec<u8> = (0..24).collect();
        let mut dst = Vec::new();
        tight_nv12(&image, &src, &mut dst).unwrap();
        assert_eq!(dst.len(), nv12_size(3, 3));
        assert_eq!(&dst[..9], &[0, 1, 2, 4, 5, 6, 8, 9, 10]);
        assert_eq!(&dst[9..], &[16, 17, 18, 19, 20, 21, 22, 23]);
    }

    #[test]
    fn tight_nv12_rejects_short_buffers_and_wrong_types() {
        let (image, src) = nv12_6x4_padded();
        let mut dst = Vec::new();
        let err = tight_nv12(&image, &src[..50], &mut dst).unwrap_err();
        assert!(
            matches!(err, ImageError::OutOfBounds { plane: 1, .. }),
            "{err}"
        );
        let mut rgb = image;
        rgb.image_type = MEDIA_IMAGE_TYPE_RGB;
        assert!(matches!(
            tight_nv12(&rgb, &src, &mut dst),
            Err(ImageError::NotYuv420 { .. })
        ));
        let mut p010 = image;
        p010.bit_depth_allocated = 16;
        assert!(matches!(
            tight_nv12(&p010, &src, &mut dst),
            Err(ImageError::NotYuv420 { .. })
        ));
        let mut yuv422 = image;
        yuv422.planes[1].vert_subsampling = 1;
        assert!(matches!(
            tight_nv12(&yuv422, &src, &mut dst),
            Err(ImageError::NotSubsampled420(_))
        ));
    }
}
