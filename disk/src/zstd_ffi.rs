// Copyright 2026 The Droid-VM Authors
// SPDX-License-Identifier: BSD-3-Clause
//
// Minimal stand-in for the `zstd` crate over the platform C libzstd, exposing only the
// entry points this crate uses (`stream::read::Decoder`, `stream::encode_all`,
// `bulk::Decompressor`). The soong tree has no rust zstd binding, only the C library.

use std::ffi::c_void;
use std::io;
use std::marker::PhantomData;
use std::os::raw::c_char;
use std::os::raw::c_int;

#[repr(C)]
struct ZstdInBuffer {
    src: *const c_void,
    size: usize,
    pos: usize,
}

#[repr(C)]
struct ZstdOutBuffer {
    dst: *mut c_void,
    size: usize,
    pos: usize,
}

extern "C" {
    fn ZSTD_isError(code: usize) -> u32;
    fn ZSTD_getErrorName(code: usize) -> *const c_char;

    fn ZSTD_createDCtx() -> *mut c_void;
    fn ZSTD_freeDCtx(dctx: *mut c_void) -> usize;
    fn ZSTD_decompressDCtx(
        dctx: *mut c_void,
        dst: *mut c_void,
        dst_capacity: usize,
        src: *const c_void,
        src_size: usize,
    ) -> usize;

    fn ZSTD_createDStream() -> *mut c_void;
    fn ZSTD_freeDStream(zds: *mut c_void) -> usize;
    fn ZSTD_initDStream(zds: *mut c_void) -> usize;
    fn ZSTD_decompressStream(
        zds: *mut c_void,
        output: *mut ZstdOutBuffer,
        input: *mut ZstdInBuffer,
    ) -> usize;

    fn ZSTD_compressBound(src_size: usize) -> usize;
    fn ZSTD_compress(
        dst: *mut c_void,
        dst_capacity: usize,
        src: *const c_void,
        src_size: usize,
        level: c_int,
    ) -> usize;
}

fn check(code: usize) -> io::Result<usize> {
    // SAFETY: ZSTD_isError/ZSTD_getErrorName accept any return code; the name is a static string.
    unsafe {
        if ZSTD_isError(code) != 0 {
            let name = std::ffi::CStr::from_ptr(ZSTD_getErrorName(code));
            Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("zstd: {}", name.to_string_lossy()),
            ))
        } else {
            Ok(code)
        }
    }
}

pub mod bulk {
    use super::*;

    pub struct Decompressor<'a> {
        dctx: *mut c_void,
        _lifetime: PhantomData<&'a ()>,
    }

    impl<'a> Decompressor<'a> {
        pub fn new() -> io::Result<Self> {
            // SAFETY: allocates an opaque context; checked for NULL below.
            let dctx = unsafe { ZSTD_createDCtx() };
            if dctx.is_null() {
                return Err(io::Error::new(
                    io::ErrorKind::OutOfMemory,
                    "zstd: ZSTD_createDCtx failed",
                ));
            }
            Ok(Decompressor {
                dctx,
                _lifetime: PhantomData,
            })
        }

        // Decompresses into `destination`'s spare capacity (like the zstd crate's WriteBuf
        // impl for Vec: the capacity is the limit, the Vec is never grown).
        pub fn decompress_to_buffer(
            &mut self,
            source: &[u8],
            destination: &mut Vec<u8>,
        ) -> io::Result<usize> {
            destination.clear();
            // SAFETY: dst pointer/capacity and src pointer/len describe valid buffers for the
            // duration of the call; set_len is bounded by the decoded size zstd reports.
            let decoded = check(unsafe {
                ZSTD_decompressDCtx(
                    self.dctx,
                    destination.as_mut_ptr() as *mut c_void,
                    destination.capacity(),
                    source.as_ptr() as *const c_void,
                    source.len(),
                )
            })?;
            // SAFETY: zstd wrote exactly `decoded` bytes and decoded <= capacity.
            unsafe { destination.set_len(decoded) };
            Ok(decoded)
        }
    }

    impl Drop for Decompressor<'_> {
        fn drop(&mut self) {
            // SAFETY: dctx is a live context owned by self.
            unsafe { ZSTD_freeDCtx(self.dctx) };
        }
    }
}

pub mod stream {
    use super::*;

    pub fn encode_all(source: &[u8], level: i32) -> io::Result<Vec<u8>> {
        // SAFETY: compressBound is a pure size computation.
        let bound = unsafe { ZSTD_compressBound(source.len()) };
        let mut out = Vec::with_capacity(bound);
        // SAFETY: dst pointer/capacity and src pointer/len describe valid buffers; set_len is
        // bounded by the compressed size zstd reports.
        let written = check(unsafe {
            ZSTD_compress(
                out.as_mut_ptr() as *mut c_void,
                bound,
                source.as_ptr() as *const c_void,
                source.len(),
                level,
            )
        })?;
        // SAFETY: zstd wrote exactly `written` bytes and written <= bound.
        unsafe { out.set_len(written) };
        Ok(out)
    }

    pub mod read {
        use super::*;

        pub struct Decoder<'a> {
            zds: *mut c_void,
            src: &'a [u8],
            pos: usize,
            single_frame: bool,
            frame_done: bool,
        }

        impl<'a> Decoder<'a> {
            pub fn with_buffer(src: &'a [u8]) -> io::Result<Self> {
                // SAFETY: allocates an opaque stream; checked for NULL below.
                let zds = unsafe { ZSTD_createDStream() };
                if zds.is_null() {
                    return Err(io::Error::new(
                        io::ErrorKind::OutOfMemory,
                        "zstd: ZSTD_createDStream failed",
                    ));
                }
                // SAFETY: zds is a live stream.
                if let Err(e) = check(unsafe { ZSTD_initDStream(zds) }) {
                    // SAFETY: zds is a live stream not yet owned by a Decoder.
                    unsafe { ZSTD_freeDStream(zds) };
                    return Err(e);
                }
                Ok(Decoder {
                    zds,
                    src,
                    pos: 0,
                    single_frame: false,
                    frame_done: false,
                })
            }

            pub fn single_frame(mut self) -> Self {
                self.single_frame = true;
                self
            }
        }

        impl io::Read for Decoder<'_> {
            fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
                if buf.is_empty() || (self.single_frame && self.frame_done) {
                    return Ok(0);
                }
                loop {
                    if self.pos == self.src.len() {
                        // Input exhausted; a truncated frame reads as EOF (qemu semantics).
                        return Ok(0);
                    }
                    let mut output = ZstdOutBuffer {
                        dst: buf.as_mut_ptr() as *mut c_void,
                        size: buf.len(),
                        pos: 0,
                    };
                    let mut input = ZstdInBuffer {
                        src: self.src[self.pos..].as_ptr() as *const c_void,
                        size: self.src.len() - self.pos,
                        pos: 0,
                    };
                    // SAFETY: zds is a live stream; output/input describe valid buffers for the
                    // duration of the call.
                    let ret =
                        check(unsafe { ZSTD_decompressStream(self.zds, &mut output, &mut input) })?;
                    self.pos += input.pos;
                    if ret == 0 {
                        self.frame_done = true;
                    }
                    if output.pos > 0 {
                        return Ok(output.pos);
                    }
                    if self.single_frame && self.frame_done {
                        return Ok(0);
                    }
                }
            }
        }

        impl Drop for Decoder<'_> {
            fn drop(&mut self) {
                // SAFETY: zds is a live stream owned by self.
                unsafe { ZSTD_freeDStream(self.zds) };
            }
        }
    }
}
