// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

//! Synthetic test pictures for the encoder, the padded input layout they are written into, and
//! the two measurements the probe makes on pixels: a luma digest and a luma PSNR.
//!
//! The picture is a function of the frame number alone, so a round trip never has to keep the
//! input around: frame `k` can be regenerated to compare against whatever the decoder returns
//! for the timestamp that frame was queued with.

/// A tight NV12 picture: luma, then interleaved chroma.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SynthFrame {
    pub width: usize,
    pub height: usize,
    pub y: Vec<u8>,
    pub uv: Vec<u8>,
}

/// Frame `k` of the moving test pattern at `width` x `height`: a horizontal luma ramp that
/// scrolls three columns per frame over a faint vertical ramp, with a bright square that walks
/// diagonally, and chroma ramps that scroll the other way. Every frame differs from the last,
/// nothing in it is noise, and the wrap of the ramp gives the encoder one hard edge to keep.
pub fn synth_frame(k: u32, width: usize, height: usize) -> SynthFrame {
    let cw = width.div_ceil(2);
    let ch = height.div_ceil(2);
    let mut y = vec![0u8; width * height];
    let mut uv = vec![0u8; cw * ch * 2];
    let k = k as usize;
    let w = width.max(1);
    let h = height.max(1);
    // Ramp 0..171 plus vertical 0..47 on top of 16: everything stays in 16..=234, video range.
    for row in 0..height {
        let vert = row * 48 / h;
        for col in 0..width {
            let ramp = ((col + 3 * k) % w) * 172 / w;
            y[row * width + col] = (16 + ramp + vert) as u8;
        }
    }
    // The walking square: 1/8 of the smaller dimension, moving 5 and 3 pixels per frame.
    let side = (width.min(height) / 8).max(1);
    if width > side && height > side {
        let x0 = (5 * k) % (width - side);
        let y0 = (3 * k) % (height - side);
        for row in y0..y0 + side {
            for col in x0..x0 + side {
                y[row * width + col] = 235;
            }
        }
    }
    for row in 0..ch {
        for col in 0..cw {
            let cb = 64 + ((row + k) % ch.max(1)) * 128 / ch.max(1);
            let cr = 64 + ((col + 2 * k) % cw.max(1)) * 128 / cw.max(1);
            uv[(row * cw + col) * 2] = cb as u8;
            uv[(row * cw + col) * 2 + 1] = cr as u8;
        }
    }
    SynthFrame {
        width,
        height,
        y,
        uv,
    }
}

/// How an encoder wants its input buffer laid out, read back from `AMediaCodec_getInputFormat`
/// after `configure`: rows padded to `stride`, chroma starting at `stride * slice_height`,
/// either interleaved (NV12, `COLOR_FormatYUV420SemiPlanar`) or as two planes of `stride / 2`
/// bytes per row (I420, `COLOR_FormatYUV420Planar`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PaddedLayout {
    pub stride: usize,
    pub slice_height: usize,
    pub semiplanar: bool,
}

impl PaddedLayout {
    /// Bytes the whole padded frame occupies: what `AMediaCodec_queueInputBuffer` is given as
    /// the size.
    pub fn frame_size(&self) -> usize {
        let luma = self.stride * self.slice_height;
        luma + luma / 2
    }
}

/// Write `frame` into `dst` as `layout` says, and return the number of bytes used. Bytes in the
/// padding are left as they were. `dst` must be at least `layout.frame_size()` long and the
/// stride and slice height must cover the picture.
pub fn pack_frame(
    frame: &SynthFrame,
    layout: &PaddedLayout,
    dst: &mut [u8],
) -> Result<usize, String> {
    let (w, h) = (frame.width, frame.height);
    let cw = w.div_ceil(2);
    let ch = h.div_ceil(2);
    if layout.stride < w || layout.slice_height < h {
        return Err(format!(
            "layout {}x{} does not cover a {}x{} picture",
            layout.stride, layout.slice_height, w, h
        ));
    }
    let needed = layout.frame_size();
    if dst.len() < needed {
        return Err(format!(
            "input buffer is {} bytes, the padded frame needs {}",
            dst.len(),
            needed
        ));
    }
    for row in 0..h {
        dst[row * layout.stride..row * layout.stride + w]
            .copy_from_slice(&frame.y[row * w..(row + 1) * w]);
    }
    let chroma = layout.stride * layout.slice_height;
    if layout.semiplanar {
        for row in 0..ch {
            let at = chroma + row * layout.stride;
            dst[at..at + 2 * cw].copy_from_slice(&frame.uv[row * 2 * cw..(row + 1) * 2 * cw]);
        }
    } else {
        let half = layout.stride / 2;
        let v_plane = chroma + half * (layout.slice_height / 2);
        for row in 0..ch {
            for col in 0..cw {
                dst[chroma + row * half + col] = frame.uv[(row * cw + col) * 2];
                dst[v_plane + row * half + col] = frame.uv[(row * cw + col) * 2 + 1];
            }
        }
    }
    Ok(needed)
}

/// Peak signal-to-noise ratio of two luma planes of the same size, in dB.
/// `f64::INFINITY` when they are identical; a mismatch in length compares the common prefix.
pub fn psnr_luma(a: &[u8], b: &[u8]) -> f64 {
    let n = a.len().min(b.len());
    if n == 0 {
        return 0.0;
    }
    let mut sum = 0u64;
    for (x, y) in a[..n].iter().zip(&b[..n]) {
        let d = *x as i64 - *y as i64;
        sum += (d * d) as u64;
    }
    if sum == 0 {
        return f64::INFINITY;
    }
    let mse = sum as f64 / n as f64;
    10.0 * (255.0f64 * 255.0 / mse).log10()
}

/// FNV-1a over every fourth pixel of every fourth row of a tight luma plane, with the mean of
/// the sampled pixels. Its job is to tell frames apart: a stream that returns the same digest
/// every time is a frozen buffer, which a frame counter cannot see.
pub fn luma_digest(y: &[u8], width: usize, height: usize) -> (u64, f64) {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    let mut sum: u64 = 0;
    let mut count: u64 = 0;
    for row in (0..height).step_by(4) {
        let start = row * width;
        let end = start + width;
        if end > y.len() {
            break;
        }
        for &px in y[start..end].iter().step_by(4) {
            hash ^= px as u64;
            hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
            sum += px as u64;
            count += 1;
        }
    }
    (
        hash,
        if count == 0 {
            0.0
        } else {
            sum as f64 / count as f64
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn frames_have_the_right_size_and_differ() {
        let a = synth_frame(0, 64, 48);
        let b = synth_frame(1, 64, 48);
        assert_eq!(a.y.len(), 64 * 48);
        assert_eq!(a.uv.len(), 32 * 24 * 2);
        assert_ne!(a.y, b.y);
        assert_ne!(a.uv, b.uv);
        assert_eq!(a, synth_frame(0, 64, 48), "deterministic");
        // Odd sizes round chroma up.
        let odd = synth_frame(3, 33, 17);
        assert_eq!(odd.y.len(), 33 * 17);
        assert_eq!(odd.uv.len(), 17 * 9 * 2);
        // Luma stays in the video range the encoder expects and includes the bright square.
        assert!(a.y.iter().all(|&v| (16..=235).contains(&v)));
        assert!(a.y.iter().any(|&v| v == 235));
    }

    #[test]
    fn pack_semiplanar_respects_stride_and_slice_height() {
        let frame = synth_frame(2, 6, 4);
        let layout = PaddedLayout {
            stride: 8,
            slice_height: 6,
            semiplanar: true,
        };
        let mut dst = vec![0xEEu8; layout.frame_size() + 5];
        let used = pack_frame(&frame, &layout, &mut dst).unwrap();
        assert_eq!(used, 8 * 6 * 3 / 2);
        for row in 0..4 {
            assert_eq!(&dst[row * 8..row * 8 + 6], &frame.y[row * 6..row * 6 + 6]);
            assert_eq!(
                &dst[row * 8 + 6..row * 8 + 8],
                &[0xEE, 0xEE],
                "row padding untouched"
            );
        }
        assert!(
            dst[32..48].iter().all(|&b| b == 0xEE),
            "slice padding untouched"
        );
        for row in 0..2 {
            assert_eq!(
                &dst[48 + row * 8..48 + row * 8 + 6],
                &frame.uv[row * 6..row * 6 + 6]
            );
            assert_eq!(&dst[48 + row * 8 + 6..48 + row * 8 + 8], &[0xEE, 0xEE]);
        }
        assert!(dst[used..].iter().all(|&b| b == 0xEE));
    }

    #[test]
    fn pack_planar_puts_u_then_v() {
        let frame = synth_frame(1, 4, 2);
        let layout = PaddedLayout {
            stride: 8,
            slice_height: 4,
            semiplanar: false,
        };
        let mut dst = vec![0u8; layout.frame_size()];
        pack_frame(&frame, &layout, &mut dst).unwrap();
        // U plane at 32 with rows of 4, V plane at 32 + 4*2 = 40.
        assert_eq!(&dst[32..34], &[frame.uv[0], frame.uv[2]]);
        assert_eq!(&dst[40..42], &[frame.uv[1], frame.uv[3]]);
    }

    #[test]
    fn pack_rejects_layouts_that_do_not_fit() {
        let frame = synth_frame(0, 8, 8);
        let small = PaddedLayout {
            stride: 4,
            slice_height: 8,
            semiplanar: true,
        };
        let mut dst = vec![0u8; 1024];
        assert!(pack_frame(&frame, &small, &mut dst).is_err());
        let ok = PaddedLayout {
            stride: 8,
            slice_height: 8,
            semiplanar: true,
        };
        assert!(pack_frame(&frame, &ok, &mut dst[..90]).is_err());
        assert_eq!(pack_frame(&frame, &ok, &mut dst[..96]).unwrap(), 96);
    }

    #[test]
    fn psnr_is_infinite_for_identical_and_known_for_a_fixed_error() {
        let a = synth_frame(5, 32, 32).y;
        assert_eq!(psnr_luma(&a, &a), f64::INFINITY);
        // Every pixel off by 2: MSE 4 -> 10*log10(65025/4) = 42.11 dB.
        let b: Vec<u8> = a.iter().map(|&v| v + 2).collect();
        let db = psnr_luma(&a, &b);
        assert!((db - 42.11).abs() < 0.01, "{db}");
        // Off by 16: 24.05 dB, below the 30 dB round-trip bar.
        let c: Vec<u8> = a.iter().map(|&v| v.saturating_sub(16).max(16)).collect();
        assert!(psnr_luma(&a, &c) < 30.0);
        assert_eq!(psnr_luma(&[], &[]), 0.0);
    }

    #[test]
    fn digest_tells_frames_apart() {
        let a = synth_frame(0, 64, 64);
        let b = synth_frame(1, 64, 64);
        let (da, ma) = luma_digest(&a.y, 64, 64);
        let (db, _) = luma_digest(&b.y, 64, 64);
        assert_ne!(da, db);
        assert_eq!(da, luma_digest(&a.y, 64, 64).0);
        assert!(ma > 16.0 && ma < 235.0);
        assert_eq!(luma_digest(&[], 64, 64).1, 0.0);
    }
}
