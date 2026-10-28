// Copyright 2026 The crosvm Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! Hardware JPEG frame source for the emulated camera.
//!
//! The phone's camera pipeline can render straight into a MediaCodec JPEG
//! encoder surface, which is far faster than the software encoder in
//! [`super::jpeg`]: one 1080p frame costs about 150 ms on the CPU but only a few
//! milliseconds in hardware, which is what 1080p60 needs.
//!
//! The media stack is loaded with `dlopen`, exactly like the camera bindings, so
//! a device without a JPEG encoder just falls back to the software path.

use std::ffi::{c_char, c_int, c_void, CString};
use std::sync::OnceLock;

use base::info;

const RTLD_NOW: c_int = 2;

extern "C" {
    fn dlopen(filename: *const c_char, flag: c_int) -> *mut c_void;
    fn dlsym(handle: *mut c_void, symbol: *const c_char) -> *mut c_void;
}

/// A `dlopen`ed shared library.
struct Lib {
    handle: *mut c_void,
}

impl Lib {
    fn open(name: &str) -> Option<Lib> {
        let cname = CString::new(name).ok()?;
        // SAFETY: `cname` is a valid NUL terminated string.
        let handle = unsafe { dlopen(cname.as_ptr(), RTLD_NOW) };
        if handle.is_null() {
            None
        } else {
            Some(Lib { handle })
        }
    }

    fn sym(&self, name: &str) -> Option<*mut c_void> {
        let cname = CString::new(name).ok()?;
        // SAFETY: `self.handle` comes from a successful `dlopen`.
        let pointer = unsafe { dlsym(self.handle, cname.as_ptr()) };
        if pointer.is_null() {
            None
        } else {
            Some(pointer)
        }
    }
}

macro_rules! bind {
    ($lib:expr, $out:ident, $name:literal, $ty:ty) => {
        let $out: $ty = match $lib.sym($name) {
            // SAFETY: the symbol exists and its declared signature matches the
            // NDK media C API.
            Some(pointer) => unsafe { std::mem::transmute::<*mut c_void, $ty>(pointer) },
            None => return None,
        };
    };
}

/// `AMediaFormat` is an opaque handle.
type AMediaFormat = c_void;
/// `AMediaCodec` is an opaque handle.
type AMediaCodec = c_void;

#[repr(C)]
struct BufferInfo {
    offset: i32,
    size: i32,
    presentation_time_us: i64,
    flags: u32,
}

/// `COLOR_FormatSurface`: the input format of a surface fed encoder.
const COLOR_FORMAT_SURFACE: i32 = 0x7f00_0789;
const CONFIGURE_FLAG_ENCODE: u32 = 1;
const BUFFER_FLAG_CODEC_CONFIG: u32 = 2;
const INFO_TRY_AGAIN_LATER: isize = -1;
const INFO_OUTPUT_FORMAT_CHANGED: isize = -2;
const INFO_OUTPUT_BUFFERS_CHANGED: isize = -3;

struct Api {
    _lib: Lib,
    codec_create_encoder_by_type: unsafe extern "C" fn(*const c_char) -> *mut AMediaCodec,
    codec_configure: unsafe extern "C" fn(
        *mut AMediaCodec,
        *const AMediaFormat,
        *mut c_void,
        *mut c_void,
        u32,
    ) -> i32,
    codec_create_input_surface: unsafe extern "C" fn(*mut AMediaCodec, *mut *mut c_void) -> i32,
    codec_start: unsafe extern "C" fn(*mut AMediaCodec) -> i32,
    codec_stop: unsafe extern "C" fn(*mut AMediaCodec) -> i32,
    codec_dequeue_output: unsafe extern "C" fn(*mut AMediaCodec, *mut BufferInfo, i64) -> isize,
    codec_get_output_buffer: unsafe extern "C" fn(*mut AMediaCodec, usize, *mut usize) -> *mut u8,
    codec_release_output: unsafe extern "C" fn(*mut AMediaCodec, usize, bool) -> i32,
    format_new: unsafe extern "C" fn() -> *mut AMediaFormat,
    format_delete: unsafe extern "C" fn(*mut AMediaFormat),
    format_set_int32: unsafe extern "C" fn(*mut AMediaFormat, *const c_char, i32) -> i32,
    format_set_string: unsafe extern "C" fn(*mut AMediaFormat, *const c_char, *const c_char) -> i32,
}

// SAFETY: the resolved function pointers are immutable and the library stays
// loaded for the lifetime of the process.
unsafe impl Send for Api {}
unsafe impl Sync for Api {}

fn api() -> Option<&'static Api> {
    static API: OnceLock<Option<Api>> = OnceLock::new();
    API.get_or_init(|| {
        let lib = Lib::open("libmediandk.so")?;
        bind!(
            lib,
            codec_create_encoder_by_type,
            "AMediaCodec_createEncoderByType",
            unsafe extern "C" fn(*const c_char) -> *mut AMediaCodec
        );
        bind!(
            lib,
            codec_configure,
            "AMediaCodec_configure",
            unsafe extern "C" fn(
                *mut AMediaCodec,
                *const AMediaFormat,
                *mut c_void,
                *mut c_void,
                u32
            ) -> i32
        );
        bind!(
            lib,
            codec_create_input_surface,
            "AMediaCodec_createInputSurface",
            unsafe extern "C" fn(*mut AMediaCodec, *mut *mut c_void) -> i32
        );
        bind!(
            lib,
            codec_start,
            "AMediaCodec_start",
            unsafe extern "C" fn(*mut AMediaCodec) -> i32
        );
        bind!(
            lib,
            codec_stop,
            "AMediaCodec_stop",
            unsafe extern "C" fn(*mut AMediaCodec) -> i32
        );
        bind!(
            lib,
            codec_dequeue_output,
            "AMediaCodec_dequeueOutputBuffer",
            unsafe extern "C" fn(*mut AMediaCodec, *mut BufferInfo, i64) -> isize
        );
        bind!(
            lib,
            codec_get_output_buffer,
            "AMediaCodec_getOutputBuffer",
            unsafe extern "C" fn(*mut AMediaCodec, usize, *mut usize) -> *mut u8
        );
        bind!(
            lib,
            codec_release_output,
            "AMediaCodec_releaseOutputBuffer",
            unsafe extern "C" fn(*mut AMediaCodec, usize, bool) -> i32
        );
        bind!(
            lib,
            format_new,
            "AMediaFormat_new",
            unsafe extern "C" fn() -> *mut AMediaFormat
        );
        bind!(
            lib,
            format_delete,
            "AMediaFormat_delete",
            unsafe extern "C" fn(*mut AMediaFormat)
        );
        bind!(
            lib,
            format_set_int32,
            "AMediaFormat_setInt32",
            unsafe extern "C" fn(*mut AMediaFormat, *const c_char, i32) -> i32
        );
        bind!(
            lib,
            format_set_string,
            "AMediaFormat_setString",
            unsafe extern "C" fn(*mut AMediaFormat, *const c_char, *const c_char) -> i32
        );
        Some(Api {
            _lib: lib,
            codec_create_encoder_by_type,
            codec_configure,
            codec_create_input_surface,
            codec_start,
            codec_stop,
            codec_dequeue_output,
            codec_get_output_buffer,
            codec_release_output,
            format_new,
            format_delete,
            format_set_int32,
            format_set_string,
        })
    })
    .as_ref()
}

fn log(line: &str) {
    use std::io::Write;
    info!("uvc: {line}");
    if let Ok(mut file) = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open("/data/local/tmp/camprobe.log")
    {
        let _ = writeln!(file, "{line}");
    }
}

/// Hardware JPEG encoder fed by the camera.
pub struct JpegEncoder {
    codec: *mut AMediaCodec,
    surface: *mut c_void,
    info: BufferInfo,
    frames: u64,
}

// SAFETY: the handle is only touched from the capture thread.
unsafe impl Send for JpegEncoder {}

impl JpegEncoder {
    /// Builds a JPEG encoder whose input surface the camera can render into.
    pub fn create(width: i32, height: i32, quality: i32) -> Option<JpegEncoder> {
        let api = api()?;
        let mime = CString::new("image/jpeg").ok()?;
        // SAFETY: every call follows the NDK media contract; failures are
        // reported through status codes and mapped to `None`.
        unsafe {
            let codec = (api.codec_create_encoder_by_type)(mime.as_ptr());
            if codec.is_null() {
                log("encoder: no image/jpeg encoder");
                return None;
            }
            let format = (api.format_new)();
            if format.is_null() {
                log("encoder: AMediaFormat_new failed");
                return None;
            }
            let key_mime = CString::new("mime").ok()?;
            let key_width = CString::new("width").ok()?;
            let key_height = CString::new("height").ok()?;
            let key_color = CString::new("color-format").ok()?;
            let key_quality = CString::new("quality").ok()?;
            (api.format_set_string)(format, key_mime.as_ptr(), mime.as_ptr());
            (api.format_set_int32)(format, key_width.as_ptr(), width);
            (api.format_set_int32)(format, key_height.as_ptr(), height);
            (api.format_set_int32)(format, key_color.as_ptr(), COLOR_FORMAT_SURFACE);
            (api.format_set_int32)(format, key_quality.as_ptr(), quality);
            let status = (api.codec_configure)(
                codec,
                format,
                std::ptr::null_mut(),
                std::ptr::null_mut(),
                CONFIGURE_FLAG_ENCODE,
            );
            (api.format_delete)(format);
            if status != 0 {
                log(&format!("encoder: configure failed status={status}"));
                return None;
            }
            let mut surface: *mut c_void = std::ptr::null_mut();
            let status = (api.codec_create_input_surface)(codec, &mut surface);
            if status != 0 || surface.is_null() {
                log(&format!("encoder: createInputSurface failed status={status}"));
                return None;
            }
            let status = (api.codec_start)(codec);
            if status != 0 {
                log(&format!("encoder: start failed status={status}"));
                return None;
            }
            log(&format!(
                "encoder: jpeg {width}x{height} quality={quality} ready"
            ));
            Some(JpegEncoder {
                codec,
                surface,
                info: BufferInfo {
                    offset: 0,
                    size: 0,
                    presentation_time_us: 0,
                    flags: 0,
                },
                frames: 0,
            })
        }
    }

    /// Window the camera should render into.
    pub fn surface(&self) -> *mut c_void {
        self.surface
    }

    /// Number of JPEG frames produced so far.
    pub fn frames(&self) -> u64 {
        self.frames
    }

    /// Drains finished frames and returns the newest JPEG, if any.
    pub fn poll(&mut self) -> Option<Vec<u8>> {
        let api = api()?;
        loop {
            // SAFETY: `codec` is live and `info` is a valid out-parameter.
            let index = unsafe { (api.codec_dequeue_output)(self.codec, &mut self.info, 0) };
            if index >= 0 {
                let index = index as usize;
                let mut size = 0usize;
                // SAFETY: the buffer stays valid until it is released below.
                let data = unsafe { (api.codec_get_output_buffer)(self.codec, index, &mut size) };
                let skip = self.info.flags & BUFFER_FLAG_CODEC_CONFIG != 0;
                let jpeg = if !data.is_null() && size > 0 && !skip {
                    // SAFETY: `data` points at `size` readable bytes.
                    let slice = unsafe { std::slice::from_raw_parts(data, size) };
                    self.frames += 1;
                    Some(slice.to_vec())
                } else {
                    None
                };
                unsafe { (api.codec_release_output)(self.codec, index, false) };
                if jpeg.is_some() {
                    return jpeg;
                }
                continue;
            }
            if index == INFO_OUTPUT_FORMAT_CHANGED || index == INFO_OUTPUT_BUFFERS_CHANGED {
                continue;
            }
            if index == INFO_TRY_AGAIN_LATER {
                return None;
            }
            return None;
        }
    }
}

impl Drop for JpegEncoder {
    fn drop(&mut self) {
        if let Some(api) = api() {
            // SAFETY: the codec is owned by `self` and stopped once.
            unsafe {
                (api.codec_stop)(self.codec);
            }
        }
    }
}
