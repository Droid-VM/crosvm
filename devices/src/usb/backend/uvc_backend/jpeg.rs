// Copyright 2026 The crosvm Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! A small baseline JPEG encoder for 4:2:0 YUV frames.
//!
//! The virtual UVC camera streams MJPEG, so frames coming from the Android
//! camera (which hands out `YUV_420_888`) have to be turned into JPEG. This
//! encoder is self contained on purpose: no C library, no extra crate, just
//! baseline sequential JPEG with optimal Huffman tables that are computed from
//! the frame itself.
//!
//! It is a straight port of the well known IJG structure: forward DCT per 8x8
//! block, quality scaled quantization tables, then two entropy passes (the
//! first one collects symbol statistics, the second one emits the data).

/// Luminance quantization table, in natural (row major) order.
const STD_LUMA_QUANT: [u8; 64] = [
    16, 11, 10, 16, 24, 40, 51, 61, 12, 12, 14, 19, 26, 58, 60, 55, 14, 13, 16, 24, 40, 57, 69, 56,
    14, 17, 22, 29, 51, 87, 80, 62, 18, 22, 37, 56, 68, 109, 103, 77, 24, 35, 55, 64, 81, 104, 113,
    92, 49, 64, 78, 87, 103, 121, 120, 101, 72, 92, 95, 98, 112, 100, 103, 99,
];

/// Chrominance quantization table, in natural (row major) order.
const STD_CHROMA_QUANT: [u8; 64] = [
    17, 18, 24, 47, 99, 99, 99, 99, 18, 21, 26, 66, 99, 99, 99, 99, 24, 26, 56, 99, 99, 99, 99, 99,
    47, 66, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
];

/// Maps a zigzag index to the natural (row major) coefficient index.
const ZIGZAG: [usize; 64] = [
    0, 1, 8, 16, 9, 2, 3, 10, 17, 24, 32, 25, 18, 11, 4, 5, 12, 19, 26, 33, 40, 48, 41, 34, 27, 20,
    13, 6, 7, 14, 21, 28, 35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51, 58, 59,
    52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63,
];

fn scaled_quant_table(base: &[u8; 64], quality: u8) -> [u8; 64] {
    let quality = quality.clamp(1, 100) as i32;
    let scale = if quality < 50 {
        5000 / quality
    } else {
        200 - quality * 2
    };
    let mut table = [0u8; 64];
    for (dst, src) in table.iter_mut().zip(base.iter()) {
        let value = ((i32::from(*src) * scale + 50) / 100).clamp(1, 255);
        *dst = value as u8;
    }
    table
}

/// Forward DCT cosine table: `COS[u][x]`.
fn cosine_table() -> [[f32; 8]; 8] {
    let mut table = [[0f32; 8]; 8];
    for (u, row) in table.iter_mut().enumerate() {
        for (x, value) in row.iter_mut().enumerate() {
            let angle = (2.0 * x as f32 + 1.0) * u as f32 * std::f32::consts::PI / 16.0;
            let norm = if u == 0 {
                std::f32::consts::FRAC_1_SQRT_2
            } else {
                1.0
            };
            *value = 0.5 * norm * angle.cos();
        }
    }
    table
}

/// Forward DCT of one 8x8 block, output in natural order.
fn fdct(block: &[f32; 64], cos: &[[f32; 8]; 8], out: &mut [f32; 64]) {
    let mut temp = [0f32; 64];
    for y in 0..8 {
        for u in 0..8 {
            let mut sum = 0f32;
            for x in 0..8 {
                sum += block[y * 8 + x] * cos[u][x];
            }
            temp[y * 8 + u] = sum;
        }
    }
    for x in 0..8 {
        for v in 0..8 {
            let mut sum = 0f32;
            for y in 0..8 {
                sum += temp[y * 8 + x] * cos[v][y];
            }
            out[v * 8 + x] = sum;
        }
    }
}

/// Number of bits needed for the magnitude of `value`.
fn category(value: i32) -> u8 {
    let mut magnitude = value.unsigned_abs();
    let mut bits = 0u8;
    while magnitude != 0 {
        magnitude >>= 1;
        bits += 1;
    }
    bits
}

/// A JPEG Huffman table with canonical codes.
struct HuffTable {
    bits: [u8; 17],
    values: Vec<u8>,
    codes: [(u16, u8); 256],
}

impl HuffTable {
    /// Builds a length limited Huffman table from symbol frequencies, the same
    /// way libjpeg's `jpeg_gen_optimal_table` does.
    fn from_frequencies(freq: &[i64; 257]) -> HuffTable {
        // Pseudo symbol 256 keeps the all-ones code free for padding.
        let original = *freq;
        let mut freq = original;
        freq[256] = 1;

        let mut codesize = [0usize; 257];
        let mut others = [-1i64; 257];
        loop {
            let mut c1 = usize::MAX;
            let mut v1 = i64::MAX;
            for i in 0..=256 {
                if freq[i] != 0 && freq[i] <= v1 {
                    v1 = freq[i];
                    c1 = i;
                }
            }
            let mut c2 = usize::MAX;
            let mut v2 = i64::MAX;
            for i in 0..=256 {
                if freq[i] != 0 && freq[i] <= v2 && i != c1 {
                    v2 = freq[i];
                    c2 = i;
                }
            }
            if c1 == usize::MAX || c2 == usize::MAX {
                break;
            }
            freq[c1] += freq[c2];
            freq[c2] = 0;

            codesize[c1] += 1;
            let mut node = c1;
            while others[node] >= 0 {
                node = others[node] as usize;
                codesize[node] += 1;
            }
            others[node] = c2 as i64;

            codesize[c2] += 1;
            let mut node = c2;
            while others[node] >= 0 {
                node = others[node] as usize;
                codesize[node] += 1;
            }
        }

        let mut bits = [0u32; 33];
        for size in codesize.iter() {
            bits[*size] += 1;
        }

        // Move symbols out of code lengths above 16, JPEG cannot represent them.
        for i in (17..=32).rev() {
            while bits[i] > 0 {
                let mut j = i - 2;
                while bits[j] == 0 {
                    j -= 1;
                }
                bits[i] -= 2;
                bits[i - 1] += 1;
                bits[j + 1] += 2;
                bits[j] -= 1;
            }
        }

        // Drop the pseudo symbol from the longest code length.
        let mut longest = 16;
        while longest > 0 && bits[longest] == 0 {
            longest -= 1;
        }
        bits[longest] -= 1;

        // Assign the final code lengths: shortest codes to the symbols that had
        // the shortest codes before the length adjustment.
        let mut symbols: Vec<u8> = (0..=255u8).filter(|s| original[*s as usize] != 0).collect();
        symbols.sort_by_key(|s| (codesize[*s as usize], *s));

        let mut values = Vec::with_capacity(symbols.len());
        let mut codes = [(0u16, 0u8); 256];
        let mut code: u32 = 0;
        let mut index = 0;
        for length in 1..=16usize {
            for _ in 0..bits[length] {
                if index >= symbols.len() {
                    break;
                }
                let symbol = symbols[index];
                codes[symbol as usize] = (code as u16, length as u8);
                values.push(symbol);
                code += 1;
                index += 1;
            }
            code <<= 1;
        }

        let mut table_bits = [0u8; 17];
        for length in 1..=16usize {
            table_bits[length] = bits[length].min(255) as u8;
        }

        HuffTable {
            bits: table_bits,
            values,
            codes,
        }
    }
}

/// Bit writer that performs JPEG byte stuffing.
struct BitWriter {
    out: Vec<u8>,
    accumulator: u32,
    count: u8,
}

impl BitWriter {
    fn new(capacity: usize) -> BitWriter {
        BitWriter {
            out: Vec::with_capacity(capacity),
            accumulator: 0,
            count: 0,
        }
    }

    fn write_bits(&mut self, value: u32, length: u8) {
        if length == 0 {
            return;
        }
        self.accumulator = (self.accumulator << length) | (value & ((1u32 << length) - 1));
        self.count += length;
        while self.count >= 8 {
            let byte = ((self.accumulator >> (self.count - 8)) & 0xff) as u8;
            self.out.push(byte);
            if byte == 0xff {
                self.out.push(0x00);
            }
            self.count -= 8;
        }
        if self.count == 0 {
            self.accumulator = 0;
        } else {
            self.accumulator &= (1u32 << self.count) - 1;
        }
    }

    fn write_symbol(&mut self, table: &HuffTable, symbol: u8) {
        let (code, length) = table.codes[symbol as usize];
        self.write_bits(u32::from(code), length);
    }

    fn write_value(&mut self, value: i32, category: u8) {
        if category == 0 {
            return;
        }
        let bits = if value < 0 {
            (value + (1 << category) - 1) as u32
        } else {
            value as u32
        };
        self.write_bits(bits, category);
    }

    fn flush(&mut self) {
        if self.count > 0 {
            let padding = 8 - self.count;
            self.write_bits((1 << padding) - 1, padding);
        }
    }
}

/// A block of quantized coefficients, in zigzag order.
type Coefficients = [i16; 64];

struct Plane {
    data: Vec<u8>,
    stride: usize,
    width: usize,
    height: usize,
}

impl Plane {
    fn pixel(&self, x: usize, y: usize) -> u8 {
        let x = x.min(self.width.saturating_sub(1));
        let y = y.min(self.height.saturating_sub(1));
        self.data[y * self.stride + x]
    }
}

fn extract_plane(
    data: &[u8],
    stride: usize,
    width: usize,
    height: usize,
    pad_width: usize,
    pad_height: usize,
) -> Plane {
    let source = Plane {
        data: data.to_vec(),
        stride,
        width,
        height,
    };
    let mut padded = vec![0u8; pad_width * pad_height];
    for y in 0..pad_height {
        for x in 0..pad_width {
            padded[y * pad_width + x] = source.pixel(x, y);
        }
    }
    Plane {
        data: padded,
        stride: pad_width,
        width: pad_width,
        height: pad_height,
    }
}

/// Reads one 8x8 block of an already subsampled chroma plane.
fn chroma_block(plane: &Plane, x0: usize, y0: usize) -> [f32; 64] {
    let mut block = [0f32; 64];
    for y in 0..8 {
        for x in 0..8 {
            block[y * 8 + x] = f32::from(plane.pixel(x0 + x, y0 + y)) - 128.0;
        }
    }
    block
}

fn luma_block(plane: &Plane, x0: usize, y0: usize) -> [f32; 64] {
    let mut block = [0f32; 64];
    for y in 0..8 {
        for x in 0..8 {
            block[y * 8 + x] = f32::from(plane.pixel(x0 + x, y0 + y)) - 128.0;
        }
    }
    block
}

fn quantize(
    block: &[f32; 64],
    cos: &[[f32; 8]; 8],
    quant: &[u8; 64],
    out: &mut Coefficients,
) {
    let mut dct = [0f32; 64];
    fdct(block, cos, &mut dct);
    for zigzag in 0..64 {
        let natural = ZIGZAG[zigzag];
        let value = dct[natural] / f32::from(quant[natural]);
        out[zigzag] = value.round().clamp(-1023.0, 1023.0) as i16;
    }
}


fn encode_frame(
    width: usize,
    height: usize,
    quality: u8,
    luma_data: &[u8],
    luma_stride: usize,
    u_data: &[u8],
    u_stride: usize,
    v_data: &[u8],
    v_stride: usize,
) -> Option<Vec<u8>> {
    if width == 0 || height == 0 || width % 2 != 0 || height % 2 != 0 {
        return None;
    }
    let pad_width = (width + 15) & !15;
    let pad_height = (height + 15) & !15;
    let luma = extract_plane(luma_data, luma_stride, width, height, pad_width, pad_height);
    let cb_plane = extract_plane(u_data, u_stride, width / 2, height / 2, pad_width / 2, pad_height / 2);
    let cr_plane = extract_plane(v_data, v_stride, width / 2, height / 2, pad_width / 2, pad_height / 2);

    let cos = cosine_table();
    let luma_quant = scaled_quant_table(&STD_LUMA_QUANT, quality);
    let chroma_quant = scaled_quant_table(&STD_CHROMA_QUANT, quality);

    let mcus_x = pad_width / 16;
    let mcus_y = pad_height / 16;

    let mut luma_blocks: Vec<Coefficients> = Vec::with_capacity(mcus_x * mcus_y * 4);
    let mut cb_blocks: Vec<Coefficients> = Vec::with_capacity(mcus_x * mcus_y);
    let mut cr_blocks: Vec<Coefficients> = Vec::with_capacity(mcus_x * mcus_y);

    for my in 0..mcus_y {
        for mx in 0..mcus_x {
            let x0 = mx * 16;
            let y0 = my * 16;
            for (dx, dy) in [(0, 0), (8, 0), (0, 8), (8, 8)] {
                let block = luma_block(&luma, x0 + dx, y0 + dy);
                let mut coefficients = [0i16; 64];
                quantize(&block, &cos, &luma_quant, &mut coefficients);
                luma_blocks.push(coefficients);
            }
            let block = chroma_block(&cb_plane, x0 / 2, y0 / 2);
            let mut coefficients = [0i16; 64];
            quantize(&block, &cos, &chroma_quant, &mut coefficients);
            cb_blocks.push(coefficients);

            let block = chroma_block(&cr_plane, x0 / 2, y0 / 2);
            let mut coefficients = [0i16; 64];
            quantize(&block, &cos, &chroma_quant, &mut coefficients);
            cr_blocks.push(coefficients);
        }
    }

    // Pass one: gather symbol statistics for the optimal Huffman tables.
    let mut dc_luma_freq = [0i64; 257];
    let mut ac_luma_freq = [0i64; 257];
    let mut dc_chroma_freq = [0i64; 257];
    let mut ac_chroma_freq = [0i64; 257];

    for (index, block) in luma_blocks.iter().enumerate() {
        let previous = if index == 0 {
            0
        } else {
            luma_blocks[index - 1][0] as i32
        };
        collect_block(
            block,
            previous,
            &mut dc_luma_freq,
            &mut ac_luma_freq,
        );
    }
    for (index, block) in cb_blocks.iter().enumerate() {
        let previous = if index == 0 { 0 } else { cb_blocks[index - 1][0] as i32 };
        collect_block(block, previous, &mut dc_chroma_freq, &mut ac_chroma_freq);
    }
    for (index, block) in cr_blocks.iter().enumerate() {
        let previous = if index == 0 { 0 } else { cr_blocks[index - 1][0] as i32 };
        collect_block(block, previous, &mut dc_chroma_freq, &mut ac_chroma_freq);
    }

    let dc_luma = HuffTable::from_frequencies(&dc_luma_freq);
    let ac_luma = HuffTable::from_frequencies(&ac_luma_freq);
    let dc_chroma = HuffTable::from_frequencies(&dc_chroma_freq);
    let ac_chroma = HuffTable::from_frequencies(&ac_chroma_freq);

    let mut out: Vec<u8> = Vec::with_capacity(width * height / 4);
    out.extend_from_slice(&[0xff, 0xd8]); // SOI

    // APP0 / JFIF header.
    out.extend_from_slice(&[0xff, 0xe0, 0x00, 0x10]);
    out.extend_from_slice(b"JFIF\0");
    out.extend_from_slice(&[0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00]);

    // Quantization tables.
    for (id, table) in [(0u8, &luma_quant), (1u8, &chroma_quant)] {
        out.extend_from_slice(&[0xff, 0xdb, 0x00, 0x43, id]);
        let mut zigzag_table = [0u8; 64];
        for zigzag in 0..64 {
            zigzag_table[zigzag] = table[ZIGZAG[zigzag]];
        }
        out.extend_from_slice(&zigzag_table);
    }

    // Frame header: baseline, 4:2:0, three components.
    out.extend_from_slice(&[0xff, 0xc0, 0x00, 0x11, 0x08]);
    out.extend_from_slice(&(height as u16).to_be_bytes());
    out.extend_from_slice(&(width as u16).to_be_bytes());
    out.push(3);
    out.extend_from_slice(&[1, 0x22, 0x00]);
    out.extend_from_slice(&[2, 0x11, 0x01]);
    out.extend_from_slice(&[3, 0x11, 0x01]);

    // Huffman tables: DC luma 0x00, AC luma 0x10, DC chroma 0x01, AC chroma 0x11.
    for (class_id, table) in [(0x00u8, &dc_luma), (0x10u8, &ac_luma), (0x01u8, &dc_chroma), (0x11u8, &ac_chroma)] {
        let mut payload: Vec<u8> = Vec::with_capacity(17 + table.values.len());
        payload.push(class_id);
        payload.extend_from_slice(&table.bits[1..17]);
        payload.extend_from_slice(&table.values);
        out.extend_from_slice(&[0xff, 0xc4]);
        out.extend_from_slice(&((payload.len() + 2) as u16).to_be_bytes());
        out.extend_from_slice(&payload);
    }

    // Scan header.
    out.extend_from_slice(&[0xff, 0xda, 0x00, 0x0c, 0x03]);
    out.extend_from_slice(&[1, 0x00]);
    out.extend_from_slice(&[2, 0x11]);
    out.extend_from_slice(&[3, 0x11]);
    out.extend_from_slice(&[0x00, 0x3f, 0x00]);

    let mut writer = BitWriter::new(width * height / 4);
    let mut previous_dc = [0i32; 3];
    for index in 0..luma_blocks.len() {
        let block = &luma_blocks[index];
        write_block(
            &mut writer,
            block,
            &mut previous_dc[0],
            &dc_luma,
            &ac_luma,
        );
        if index % 4 == 3 {
            let mcu = index / 4;
            write_block(&mut writer, &cb_blocks[mcu], &mut previous_dc[1], &dc_chroma, &ac_chroma);
            write_block(&mut writer, &cr_blocks[mcu], &mut previous_dc[2], &dc_chroma, &ac_chroma);
        }
    }
    writer.flush();
    out.extend_from_slice(&writer.out);
    out.extend_from_slice(&[0xff, 0xd9]); // EOI
    Some(out)
}

fn collect_block(
    block: &Coefficients,
    previous_dc: i32,
    dc_freq: &mut [i64; 257],
    ac_freq: &mut [i64; 257],
) {
    let difference = i32::from(block[0]) - previous_dc;
    dc_freq[category(difference) as usize] += 1;
    let mut run = 0usize;
    for zigzag in 1..64 {
        let value = i32::from(block[zigzag]);
        if value == 0 {
            run += 1;
            continue;
        }
        while run > 15 {
            ac_freq[0xf0] += 1;
            run -= 16;
        }
        ac_freq[(run << 4) | category(value) as usize] += 1;
        run = 0;
    }
    if run > 0 {
        ac_freq[0x00] += 1;
    }
}

fn write_block(
    writer: &mut BitWriter,
    block: &Coefficients,
    previous_dc: &mut i32,
    dc_table: &HuffTable,
    ac_table: &HuffTable,
) {
    let difference = i32::from(block[0]) - *previous_dc;
    *previous_dc = i32::from(block[0]);
    let dc_category = category(difference);
    writer.write_symbol(dc_table, dc_category);
    writer.write_value(difference, dc_category);

    let mut run = 0u8;
    for zigzag in 1..64 {
        let value = i32::from(block[zigzag]);
        if value == 0 {
            run += 1;
            continue;
        }
        while run > 15 {
            writer.write_symbol(ac_table, 0xf0);
            run -= 16;
        }
        let ac_category = category(value);
        writer.write_symbol(ac_table, (run << 4) | ac_category);
        writer.write_value(value, ac_category);
        run = 0;
    }
    if run > 0 {
        writer.write_symbol(ac_table, 0x00);
    }
}

/// Encodes a `YUV_420_888` frame (three planes with their own row strides) into
/// a baseline JPEG image.
pub fn encode_yuv420(
    width: usize,
    height: usize,
    quality: u8,
    y: &[u8],
    y_stride: usize,
    u: &[u8],
    u_stride: usize,
    v: &[u8],
    v_stride: usize,
) -> Option<Vec<u8>> {
    encode_frame(
        width, height, quality, y, y_stride, u, u_stride, v, v_stride,
    )
}
