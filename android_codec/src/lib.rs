// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

//! Bindings for the Android MediaCodec NDK (`libmediandk`): enumerate the platform's codecs and
//! run a decoder or encoder session in asynchronous mode.
//!
//! This is the `android_camera` pattern applied to codecs. It exists to back the virtio-media
//! decoder and encoder devices (design 7.2 / 7.3), so its shape is theirs: a capability listing
//! that answers `ENUM_FMT` / `ENUM_FRAMESIZES` / `ENUM_FRAMEINTERVALS` from what the platform
//! reports rather than from a table, and a session whose four NDK callbacks do nothing but push
//! an event onto a queue and bump an eventfd, so a device worker can `poll` it alongside its
//! virtqueues.
//!
//! # Loading
//!
//! `libmediandk.so` and `libbinder_ndk.so` are opened with `dlopen`, not linked (the reason is
//! `libgui`, see `android_camera`). Everything at API 28 or below is required; the API 36
//! codec-introspection surface (`AMediaCodecStore_*`, `AMediaCodecInfo_*`,
//! `ACodec*Capabilities_*`) is optional, so the library still loads on an older Android and
//! [`list_codecs`] then falls back to probing by mime with `createDecoderByType`.
//!
//! # Binder
//!
//! `MediaCodec` registers a `BnResourceManagerClient` with `ResourceManagerService` and is
//! called back over binder (reclaim), and `MediaCodecList` is fetched from `media.player`; a bare
//! native process has no binder threads to receive any of that, so the thread pool is started
//! when the NDK is loaded, as `android_camera` does.

use std::collections::VecDeque;
use std::ffi::c_void;
use std::ffi::CStr;
use std::ffi::CString;
use std::marker::PhantomData;
use std::os::raw::c_char;
use std::ptr::null;
use std::ptr::null_mut;
use std::sync::atomic::AtomicU64;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::sync::Mutex;
use std::sync::OnceLock;
use std::time::Duration;

use base::Event;
use serde::Serialize;
use thiserror::Error;

pub mod bitstream;
pub mod image;
pub mod synth;

pub use image::tight_nv12;
pub use image::tight_nv12_rows;
pub use image::ChromaLayout;
pub use image::ImageError;
pub use image::MediaImage2;
pub use image::PlaneInfo;

/// Opaque NDK handles. Written as zero-sized `repr(C)` bodies so a `*mut` to one is a distinct
/// type rather than an interchangeable `*mut c_void`.
macro_rules! opaque_handle {
    ($($name:ident),* $(,)?) => {
        $(
            #[repr(C)]
            pub struct $name {
                _data: [u8; 0],
                _marker: PhantomData<(*mut u8, core::marker::PhantomPinned)>,
            }
        )*
    };
}

opaque_handle!(
    AMediaCodec,
    AMediaFormat,
    AMediaCrypto,
    ANativeWindow,
    AMediaCodecInfo,
    ACodecVideoCapabilities,
    ACodecEncoderCapabilities,
    ACodecPerformancePoint,
);

/// `AMediaCodecBufferInfo` (`NdkMediaCodec.h:57-63`), used as-is: it is the struct the output
/// callback receives and what a caller needs to consume the buffer.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq, Serialize)]
pub struct BufferInfo {
    /// Always 0 since API 35: `AMediaCodec_getOutputBuffer` already returns the offset pointer.
    pub offset: i32,
    pub size: i32,
    pub presentation_time_us: i64,
    pub flags: u32,
}

/// `AMediaCodecOnAsyncNotifyCallback` (`NdkMediaCodec.h:149-154`): four function pointers,
/// passed to `AMediaCodec_setAsyncNotifyCallback` **by value**.
#[repr(C)]
struct AMediaCodecOnAsyncNotifyCallback {
    on_async_input_available: Option<extern "C" fn(*mut AMediaCodec, *mut c_void, i32)>,
    on_async_output_available:
        Option<extern "C" fn(*mut AMediaCodec, *mut c_void, i32, *mut BufferInfo)>,
    on_async_format_changed:
        Option<extern "C" fn(*mut AMediaCodec, *mut c_void, *mut AMediaFormat)>,
    on_async_error: Option<extern "C" fn(*mut AMediaCodec, *mut c_void, i32, i32, *const c_char)>,
}

/// `AMediaCodecSupportedMediaType` (`NdkMediaCodecStore.h:51-61`): a C++-only struct with a
/// scoped enum inside; its layout is a pointer and a `u32`.
#[repr(C)]
struct AMediaCodecSupportedMediaType {
    media_type: *const c_char,
    mode: u32,
}

/// `AMediaCodecSupportedMediaType::Mode` bits.
const MEDIA_TYPE_FLAG_DECODER: u32 = 1 << 0;
const MEDIA_TYPE_FLAG_ENCODER: u32 = 1 << 1;

/// `AIntRange` / `ADoubleRange` (`NdkMediaCodecInfo.h:58-69`), caller-allocated.
#[repr(C)]
#[derive(Default, Clone, Copy)]
struct AIntRange {
    lower: i32,
    upper: i32,
}

#[repr(C)]
#[derive(Default, Clone, Copy)]
struct ADoubleRange {
    lower: f64,
    upper: f64,
}

/// Declares every NDK entry point we use, then resolves them all at run time.
///
/// Resolved with `dlopen` rather than linked: `libmediandk` reaches `libgui`, whose build needs
/// projects a crosvm-only AOSP checkout does not carry, and resolving late turns "this platform
/// has no media NDK" into an error a caller can report rather than a binary that will not load.
///
/// Each declaration in the first group expands into three things: a field in `NdkApi`, the
/// `dlsym` that fills it, and a free function of the same name, so call sites read exactly as
/// they would against a real `extern "C"` block. The `optional` group is for entry points a
/// platform may lack (the API 36 introspection surface): they become `Option` fields, `None`
/// when absent, reached through [`opt!`]. The `optional_strings` group is for the
/// `extern const char*` feature-name globals, which are data symbols: resolved as a pointer to
/// a pointer and dereferenced once at load.
macro_rules! ndk_api {
    (
        $( fn $name:ident ( $($arg:ident : $argty:ty),* $(,)? ) $(-> $ret:ty)?; )*
        optional {
            $( fn $oname:ident ( $($oarg:ident : $oargty:ty),* $(,)? ) $(-> $oret:ty)?; )*
        }
        optional_strings {
            $( $sname:ident, )*
        }
    ) => {
        #[allow(non_snake_case)]
        struct NdkApi {
            $( $name: unsafe extern "C" fn($($argty),*) $(-> $ret)?, )*
            $( $oname: Option<unsafe extern "C" fn($($oargty),*) $(-> $oret)?>, )*
            $( $sname: Option<&'static CStr>, )*
        }

        impl NdkApi {
            fn load() -> std::result::Result<NdkApi, CodecError> {
                let handles = [
                    open_library("libmediandk.so\0")?,
                    open_library("libbinder_ndk.so\0")?,
                ];
                let api = NdkApi {
                    // SAFETY: each symbol is looked up under the name of the field it fills, and
                    // the type written here is the one transcribed from the NDK header.
                    $( $name: unsafe {
                        symbol(&handles, concat!(stringify!($name), "\0"))?
                    }, )*
                    // SAFETY: as above; a missing symbol is simply not offered.
                    $( $oname: unsafe {
                        symbol(&handles, concat!(stringify!($oname), "\0")).ok()
                    }, )*
                    // SAFETY: the symbol is an `extern const char*` in the header; the string it
                    // points at is static data of a library that is never closed.
                    $( $sname: unsafe {
                        symbol_string(&handles, concat!(stringify!($sname), "\0"))
                    }, )*
                };
                // MediaCodec is a binder client and a binder server: it fetches the codec list
                // from media.player and registers a ResourceManagerClient that
                // ResourceManagerService calls back into (reclaim). A process with no binder
                // threads has nobody to receive those transactions. An app never has to do this
                // because the framework starts the pool at process start; a bare native process
                // does.
                //
                // SAFETY: both pointers were just resolved out of libbinder_ndk, and starting the
                // pool more than once is harmless.
                unsafe {
                    (api.ABinderProcess_setThreadPoolMaxThreadCount)(BINDER_THREAD_POOL_SIZE);
                    (api.ABinderProcess_startThreadPool)();
                }
                Ok(api)
            }

            /// The optional entry points this platform does not export.
            fn missing(&self) -> Vec<&'static str> {
                let mut out = Vec::new();
                $( if self.$oname.is_none() { out.push(stringify!($oname)); } )*
                $( if self.$sname.is_none() { out.push(stringify!($sname)); } )*
                out
            }

            /// The feature-name globals that resolved, as `(symbol, feature string)`.
            fn feature_names(&self) -> Vec<(&'static str, &'static CStr)> {
                let mut out = Vec::new();
                $( if let Some(s) = self.$sname { out.push((stringify!($sname), s)); } )*
                out
            }
        }

        $(
            #[allow(non_snake_case, dead_code)]
            unsafe fn $name($($arg: $argty),*) $(-> $ret)? {
                (ndk().$name)($($arg),*)
            }
        )*
    };
}

/// An optional entry point, or `MissingSymbol` if this platform lacks it.
macro_rules! opt {
    ($name:ident) => {
        ndk()
            .$name
            .ok_or(CodecError::MissingSymbol(stringify!($name)))?
    };
}

/// The libraries stay open for the life of the process: the resolved pointers outlive any scope
/// that could close them, and nothing here is unloadable anyway.
fn open_library(name: &'static str) -> std::result::Result<*mut c_void, CodecError> {
    // SAFETY: name is a NUL-terminated literal, and the result is checked before use.
    let handle = unsafe { libc::dlopen(name.as_ptr() as *const c_char, libc::RTLD_NOW) };
    if handle.is_null() {
        // SAFETY: dlerror returns either null or a NUL-terminated string owned by the linker.
        let reason = unsafe {
            let err = libc::dlerror();
            if err.is_null() {
                "unknown".to_owned()
            } else {
                CStr::from_ptr(err).to_string_lossy().into_owned()
            }
        };
        return Err(CodecError::LibraryLoad(name.trim_end_matches('\0'), reason));
    }
    Ok(handle)
}

/// # Safety
///
/// `T` must be the function pointer type actually exported under `name`.
unsafe fn symbol<T: Copy>(
    handles: &[*mut c_void],
    name: &'static str,
) -> std::result::Result<T, CodecError> {
    for &handle in handles {
        let ptr = libc::dlsym(handle, name.as_ptr() as *const c_char);
        if !ptr.is_null() {
            return Ok(std::mem::transmute_copy(&ptr));
        }
    }
    Err(CodecError::MissingSymbol(name.trim_end_matches('\0')))
}

/// # Safety
///
/// `name` must be exported as a `const char*` variable.
unsafe fn symbol_string(handles: &[*mut c_void], name: &'static str) -> Option<&'static CStr> {
    for &handle in handles {
        let ptr = libc::dlsym(handle, name.as_ptr() as *const c_char) as *const *const c_char;
        if !ptr.is_null() && !(*ptr).is_null() {
            return Some(CStr::from_ptr(*ptr));
        }
    }
    None
}

static NDK: OnceLock<std::result::Result<NdkApi, CodecError>> = OnceLock::new();

/// Load the NDK if it has not been loaded, and report why if it cannot be. Every public entry
/// point calls this before the first FFI call.
pub fn ensure_loaded() -> Result<()> {
    match NDK.get_or_init(NdkApi::load) {
        Ok(_) => Ok(()),
        Err(e) => Err(e.clone()),
    }
}

/// The resolved entry points. Unreachable before a successful [`ensure_loaded`], because the only
/// ways into this module go through one.
fn ndk() -> &'static NdkApi {
    NDK.get()
        .and_then(|loaded| loaded.as_ref().ok())
        .expect("android_codec: NDK used before ensure_loaded() succeeded")
}

/// The optional (API 36) entry points this platform lacks. Empty on the target phone.
pub fn missing_symbols() -> Result<Vec<&'static str>> {
    ensure_loaded()?;
    Ok(ndk().missing())
}

ndk_api! {
    // libmediandk: AMediaCodec, all API 21 unless noted (NdkMediaCodec.h)
    fn AMediaCodec_createCodecByName(name: *const c_char) -> *mut AMediaCodec;
    fn AMediaCodec_createDecoderByType(mime_type: *const c_char) -> *mut AMediaCodec;
    fn AMediaCodec_createEncoderByType(mime_type: *const c_char) -> *mut AMediaCodec;
    fn AMediaCodec_delete(codec: *mut AMediaCodec) -> i32;
    fn AMediaCodec_configure(
        codec: *mut AMediaCodec,
        format: *const AMediaFormat,
        surface: *mut ANativeWindow,
        crypto: *mut AMediaCrypto,
        flags: u32,
    ) -> i32;
    fn AMediaCodec_start(codec: *mut AMediaCodec) -> i32;
    fn AMediaCodec_stop(codec: *mut AMediaCodec) -> i32;
    fn AMediaCodec_flush(codec: *mut AMediaCodec) -> i32;
    fn AMediaCodec_getInputBuffer(codec: *mut AMediaCodec, idx: usize, out_size: *mut usize) -> *mut u8;
    fn AMediaCodec_getOutputBuffer(codec: *mut AMediaCodec, idx: usize, out_size: *mut usize) -> *mut u8;
    // `_off_t_compat` is `off_t` on LP64: i64 on every target this runs on.
    fn AMediaCodec_queueInputBuffer(
        codec: *mut AMediaCodec,
        idx: usize,
        offset: libc::off_t,
        size: usize,
        time: u64,
        flags: u32,
    ) -> i32;
    fn AMediaCodec_releaseOutputBuffer(codec: *mut AMediaCodec, idx: usize, render: bool) -> i32;
    fn AMediaCodec_getOutputFormat(codec: *mut AMediaCodec) -> *mut AMediaFormat;
    // API 26
    fn AMediaCodec_setParameters(codec: *mut AMediaCodec, params: *const AMediaFormat) -> i32;
    // API 28
    fn AMediaCodec_getInputFormat(codec: *mut AMediaCodec) -> *mut AMediaFormat;
    fn AMediaCodec_getBufferFormat(codec: *mut AMediaCodec, index: usize) -> *mut AMediaFormat;
    fn AMediaCodec_getName(codec: *mut AMediaCodec, out_name: *mut *mut c_char) -> i32;
    fn AMediaCodec_releaseName(codec: *mut AMediaCodec, name: *mut c_char);
    fn AMediaCodec_setAsyncNotifyCallback(
        codec: *mut AMediaCodec,
        callback: AMediaCodecOnAsyncNotifyCallback,
        userdata: *mut c_void,
    ) -> i32;

    // libmediandk: AMediaFormat (NdkMediaFormat.h), API 21 unless noted
    fn AMediaFormat_new() -> *mut AMediaFormat;
    fn AMediaFormat_delete(format: *mut AMediaFormat) -> i32;
    fn AMediaFormat_toString(format: *mut AMediaFormat) -> *const c_char;
    fn AMediaFormat_getInt32(format: *mut AMediaFormat, name: *const c_char, out: *mut i32) -> bool;
    fn AMediaFormat_getInt64(format: *mut AMediaFormat, name: *const c_char, out: *mut i64) -> bool;
    fn AMediaFormat_getFloat(format: *mut AMediaFormat, name: *const c_char, out: *mut f32) -> bool;
    fn AMediaFormat_getBuffer(
        format: *mut AMediaFormat,
        name: *const c_char,
        data: *mut *mut c_void,
        size: *mut usize,
    ) -> bool;
    fn AMediaFormat_getString(
        format: *mut AMediaFormat,
        name: *const c_char,
        out: *mut *const c_char,
    ) -> bool;
    fn AMediaFormat_setInt32(format: *mut AMediaFormat, name: *const c_char, value: i32);
    fn AMediaFormat_setInt64(format: *mut AMediaFormat, name: *const c_char, value: i64);
    fn AMediaFormat_setFloat(format: *mut AMediaFormat, name: *const c_char, value: f32);
    fn AMediaFormat_setString(format: *mut AMediaFormat, name: *const c_char, value: *const c_char);
    fn AMediaFormat_setBuffer(
        format: *mut AMediaFormat,
        name: *const c_char,
        data: *const c_void,
        size: usize,
    );
    // API 28
    fn AMediaFormat_getRect(
        format: *mut AMediaFormat,
        name: *const c_char,
        left: *mut i32,
        top: *mut i32,
        right: *mut i32,
        bottom: *mut i32,
    ) -> bool;

    // libbinder_ndk: only to get this process onto the binder bus, see NdkApi::load.
    fn ABinderProcess_setThreadPoolMaxThreadCount(num_threads: u32) -> bool;
    fn ABinderProcess_startThreadPool();

    optional {
        // API 36: the codec store (NdkMediaCodecStore.h)
        fn AMediaCodecStore_getSupportedMediaTypes(
            out_media_types: *mut *const AMediaCodecSupportedMediaType,
            out_count: *mut usize,
        ) -> i32;
        fn AMediaCodecStore_findNextDecoderForFormat(
            format: *const AMediaFormat,
            out_codec_info: *mut *const AMediaCodecInfo,
        ) -> i32;
        fn AMediaCodecStore_findNextEncoderForFormat(
            format: *const AMediaFormat,
            out_codec_info: *mut *const AMediaCodecInfo,
        ) -> i32;
        fn AMediaCodecStore_getCodecInfo(
            name: *const c_char,
            out_codec_info: *mut *const AMediaCodecInfo,
        ) -> i32;
        // API 36: codec info (NdkMediaCodecInfo.h)
        fn AMediaCodecInfo_getCanonicalName(info: *const AMediaCodecInfo) -> *const c_char;
        fn AMediaCodecInfo_getKind(info: *const AMediaCodecInfo) -> i32;
        fn AMediaCodecInfo_isVendor(info: *const AMediaCodecInfo) -> i32;
        fn AMediaCodecInfo_getMediaCodecInfoType(info: *const AMediaCodecInfo) -> i32;
        fn AMediaCodecInfo_getMediaType(info: *const AMediaCodecInfo) -> *const c_char;
        fn AMediaCodecInfo_getMaxSupportedInstances(info: *const AMediaCodecInfo) -> i32;
        fn AMediaCodecInfo_isFeatureSupported(
            info: *const AMediaCodecInfo,
            feature_name: *const c_char,
        ) -> i32;
        fn AMediaCodecInfo_isFeatureRequired(
            info: *const AMediaCodecInfo,
            feature_name: *const c_char,
        ) -> i32;
        fn AMediaCodecInfo_isFormatSupported(
            info: *const AMediaCodecInfo,
            format: *const AMediaFormat,
        ) -> i32;
        fn AMediaCodecInfo_getVideoCapabilities(
            info: *const AMediaCodecInfo,
            out_video_caps: *mut *const ACodecVideoCapabilities,
        ) -> i32;
        fn AMediaCodecInfo_getEncoderCapabilities(
            info: *const AMediaCodecInfo,
            out_encoder_caps: *mut *const ACodecEncoderCapabilities,
        ) -> i32;
        // API 36: video capabilities
        fn ACodecVideoCapabilities_getBitrateRange(
            caps: *const ACodecVideoCapabilities,
            out_range: *mut AIntRange,
        ) -> i32;
        fn ACodecVideoCapabilities_getSupportedWidths(
            caps: *const ACodecVideoCapabilities,
            out_range: *mut AIntRange,
        ) -> i32;
        fn ACodecVideoCapabilities_getSupportedHeights(
            caps: *const ACodecVideoCapabilities,
            out_range: *mut AIntRange,
        ) -> i32;
        fn ACodecVideoCapabilities_getWidthAlignment(caps: *const ACodecVideoCapabilities) -> i32;
        fn ACodecVideoCapabilities_getHeightAlignment(caps: *const ACodecVideoCapabilities) -> i32;
        fn ACodecVideoCapabilities_getSupportedFrameRates(
            caps: *const ACodecVideoCapabilities,
            out_range: *mut AIntRange,
        ) -> i32;
        fn ACodecVideoCapabilities_getSupportedWidthsFor(
            caps: *const ACodecVideoCapabilities,
            height: i32,
            out_range: *mut AIntRange,
        ) -> i32;
        fn ACodecVideoCapabilities_getSupportedHeightsFor(
            caps: *const ACodecVideoCapabilities,
            width: i32,
            out_range: *mut AIntRange,
        ) -> i32;
        fn ACodecVideoCapabilities_getSupportedFrameRatesFor(
            caps: *const ACodecVideoCapabilities,
            width: i32,
            height: i32,
            out_range: *mut ADoubleRange,
        ) -> i32;
        fn ACodecVideoCapabilities_getAchievableFrameRatesFor(
            caps: *const ACodecVideoCapabilities,
            width: i32,
            height: i32,
            out_range: *mut ADoubleRange,
        ) -> i32;
        fn ACodecVideoCapabilities_getNextSupportedPerformancePoint(
            caps: *const ACodecVideoCapabilities,
            out_performance_point: *mut *const ACodecPerformancePoint,
        ) -> i32;
        fn ACodecVideoCapabilities_areSizeAndRateSupported(
            caps: *const ACodecVideoCapabilities,
            width: i32,
            height: i32,
            frame_rate: f64,
        ) -> i32;
        fn ACodecVideoCapabilities_isSizeSupported(
            caps: *const ACodecVideoCapabilities,
            width: i32,
            height: i32,
        ) -> i32;
        // API 36: performance points
        fn ACodecPerformancePoint_create(
            width: i32,
            height: i32,
            frame_rate: i32,
        ) -> *mut ACodecPerformancePoint;
        fn ACodecPerformancePoint_destroy(point: *mut ACodecPerformancePoint);
        fn ACodecPerformancePoint_covers(
            one: *const ACodecPerformancePoint,
            another: *const ACodecPerformancePoint,
        ) -> i32;
        // API 36: encoder capabilities
        fn ACodecEncoderCapabilities_getQualityRange(
            caps: *const ACodecEncoderCapabilities,
            out_range: *mut AIntRange,
        ) -> i32;
        fn ACodecEncoderCapabilities_getComplexityRange(
            caps: *const ACodecEncoderCapabilities,
            out_range: *mut AIntRange,
        ) -> i32;
        fn ACodecEncoderCapabilities_isBitrateModeSupported(
            caps: *const ACodecEncoderCapabilities,
            mode: i32,
        ) -> i32;
    }

    optional_strings {
        // API 36: `extern const char*` feature names (NdkMediaCodecInfo.h:680-878)
        AMediaCodecInfo_FEATURE_AdaptivePlayback,
        AMediaCodecInfo_FEATURE_SecurePlayback,
        AMediaCodecInfo_FEATURE_TunneledPlayback,
        AMediaCodecInfo_FEATURE_DynamicTimestamp,
        AMediaCodecInfo_FEATURE_FrameParsing,
        AMediaCodecInfo_FEATURE_MultipleFrames,
        AMediaCodecInfo_FEATURE_PartialFrame,
        AMediaCodecInfo_FEATURE_IntraRefresh,
        AMediaCodecInfo_FEATURE_LowLatency,
        AMediaCodecInfo_FEATURE_QpBounds,
        AMediaCodecInfo_FEATURE_EncodingStatistics,
        AMediaCodecInfo_FEATURE_HdrEditing,
        AMediaCodecInfo_FEATURE_HlgEditing,
        AMediaCodecInfo_FEATURE_DynamicColorAspects,
        AMediaCodecInfo_FEATURE_Roi,
        AMediaCodecInfo_FEATURE_DetachedSurface,
    }
}

/// Enough threads for the codec's incoming binder traffic (reclaim, death notifications)
/// without making the pool a resource of its own.
const BINDER_THREAD_POOL_SIZE: u32 = 4;

pub const AMEDIA_OK: i32 = 0;
pub const AMEDIA_ERROR_UNSUPPORTED: i32 = -10002;
pub const AMEDIACODEC_ERROR_RECLAIMED: i32 = 1101;

/// `AMEDIACODEC_BUFFER_FLAG_*` (`NdkMediaCodec.h:73-98`). `KEY_FRAME` was only named at API 34;
/// the value is the one Java's `BUFFER_FLAG_SYNC_FRAME` has always had.
pub const BUFFER_FLAG_KEY_FRAME: u32 = 1;
pub const BUFFER_FLAG_CODEC_CONFIG: u32 = 2;
pub const BUFFER_FLAG_END_OF_STREAM: u32 = 4;
pub const BUFFER_FLAG_PARTIAL_FRAME: u32 = 8;
pub const BUFFER_FLAG_DECODE_ONLY: u32 = 32;

/// `AMEDIACODEC_CONFIGURE_FLAG_ENCODE` (`NdkMediaCodec.h:101`).
pub const CONFIGURE_FLAG_ENCODE: u32 = 1;

/// `COLOR_Format*` values (`MediaCodecConstants.h:856-911`).
pub const COLOR_FORMAT_YUV420_PLANAR: i32 = 19;
pub const COLOR_FORMAT_YUV420_PACKED_PLANAR: i32 = 20;
pub const COLOR_FORMAT_YUV420_SEMI_PLANAR: i32 = 21;
pub const COLOR_FORMAT_YUV420_PACKED_SEMI_PLANAR: i32 = 39;
pub const COLOR_FORMAT_YUV420_FLEXIBLE: i32 = 0x7F42_0888;
pub const COLOR_FORMAT_YUV_P010: i32 = 54;
pub const COLOR_FORMAT_SURFACE: i32 = 0x7F00_0789;
pub const COLOR_QCOM_FORMAT_YUV420_SEMI_PLANAR: i32 = 0x7FA3_0C00;

/// `COLOR_RANGE_*`, `COLOR_STANDARD_*` and `COLOR_TRANSFER_*` (`MediaCodecConstants.h:1036-1045`):
/// the colour aspects an encoder is told under `"color-range"`, `"color-standard"` and
/// `"color-transfer"`, which it writes into the stream's VUI.
pub const COLOR_RANGE_FULL: i32 = 1;
pub const COLOR_RANGE_LIMITED: i32 = 2;
pub const COLOR_STANDARD_BT709: i32 = 1;
pub const COLOR_STANDARD_BT601_PAL: i32 = 2;
pub const COLOR_STANDARD_BT601_NTSC: i32 = 4;
pub const COLOR_STANDARD_BT2020: i32 = 6;
pub const COLOR_TRANSFER_LINEAR: i32 = 1;
pub const COLOR_TRANSFER_SDR_VIDEO: i32 = 3;
pub const COLOR_TRANSFER_ST2084: i32 = 6;
pub const COLOR_TRANSFER_HLG: i32 = 7;

/// `ABitrateMode` (`NdkMediaCodecInfo.h:655-660`) == `BITRATE_MODE_*`, writable straight into
/// `"bitrate-mode"`.
pub const BITRATE_MODE_CQ: i32 = 0;
pub const BITRATE_MODE_VBR: i32 = 1;
pub const BITRATE_MODE_CBR: i32 = 2;
pub const BITRATE_MODE_CBR_FD: i32 = 3;

/// `AMediaCodecKind` (`NdkMediaCodecInfo.h:84-93`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum CodecKind {
    Invalid,
    Decoder,
    Encoder,
}

impl From<i32> for CodecKind {
    fn from(v: i32) -> Self {
        match v {
            1 => CodecKind::Decoder,
            2 => CodecKind::Encoder,
            _ => CodecKind::Invalid,
        }
    }
}

/// `AMediaCodecType` (`NdkMediaCodecInfo.h:112-140`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub enum CodecType {
    Invalid,
    SoftwareOnly,
    HardwareAccelerated,
    SoftwareWithDeviceAccess,
    /// The Store API is absent and the codec was found by `create*ByType`; the name is the only
    /// hint (`c2.qti.` / `OMX.qcom.` are hardware, `c2.android.` is software).
    Unknown,
}

impl From<i32> for CodecType {
    fn from(v: i32) -> Self {
        match v {
            1 => CodecType::SoftwareOnly,
            2 => CodecType::HardwareAccelerated,
            3 => CodecType::SoftwareWithDeviceAccess,
            _ => CodecType::Invalid,
        }
    }
}

/// Format key names (`NdkMediaCodec.cpp:379-514` for the literal strings; `"image-data"` has no
/// `AMEDIAFORMAT_KEY_*` symbol and is read by its literal name).
pub mod keys {
    use std::ffi::CStr;

    pub const MIME: &CStr = c"mime";
    pub const WIDTH: &CStr = c"width";
    pub const HEIGHT: &CStr = c"height";
    pub const COLOR_FORMAT: &CStr = c"color-format";
    pub const STRIDE: &CStr = c"stride";
    pub const SLICE_HEIGHT: &CStr = c"slice-height";
    pub const BIT_RATE: &CStr = c"bitrate";
    pub const BITRATE_MODE: &CStr = c"bitrate-mode";
    pub const FRAME_RATE: &CStr = c"frame-rate";
    pub const I_FRAME_INTERVAL: &CStr = c"i-frame-interval";
    pub const PROFILE: &CStr = c"profile";
    pub const LEVEL: &CStr = c"level";
    pub const MAX_INPUT_SIZE: &CStr = c"max-input-size";
    pub const LOW_LATENCY: &CStr = c"low-latency";
    /// Qualcomm's decoder-side "emit in picture (decode) order, do not hold pictures back for
    /// display reorder" switch, measured on `c2.qti.avc.decoder` in WP `D91-lowlat-probe`: it
    /// turns display-order output (`0 3 2 1 4 6 5 ...`) into decode-order output (`0 1 2 3 ...`)
    /// while every decoded frame stays bit-identical once aligned by PTS (300/300, §6). Two
    /// things about it are easy to get wrong. It only takes effect when it is already in the
    /// `MediaFormat` handed to `configure()`: an `AMediaCodec_setParameters` after `start()`
    /// reports success and changes nothing (§3). And the component does not echo it on the
    /// input format, it reflects it on the OUTPUT one, so
    /// [`output_format`](crate::Codec::output_format) is the only place it can be verified
    /// (§2.2 -- reading the input format is what made an earlier probe record this key as
    /// unsupported).
    pub const QTI_PICTURE_ORDER: &CStr = c"vendor.qti-ext-dec-picture-order.enable";
    pub const CSD_0: &CStr = c"csd-0";
    pub const CSD_1: &CStr = c"csd-1";
    pub const DISPLAY_CROP: &CStr = c"crop";
    pub const CROP_LEFT: &CStr = c"crop-left";
    pub const CROP_TOP: &CStr = c"crop-top";
    pub const CROP_RIGHT: &CStr = c"crop-right";
    pub const CROP_BOTTOM: &CStr = c"crop-bottom";
    pub const DISPLAY_WIDTH: &CStr = c"display-width";
    pub const DISPLAY_HEIGHT: &CStr = c"display-height";
    pub const ROTATION: &CStr = c"rotation-degrees";
    pub const IMAGE_DATA: &CStr = c"image-data";
    pub const COLOR_RANGE: &CStr = c"color-range";
    pub const COLOR_STANDARD: &CStr = c"color-standard";
    pub const COLOR_TRANSFER: &CStr = c"color-transfer";
    /// Encoder configure keys with no `AMEDIAFORMAT_KEY_*` symbol, by their literal names
    /// (`MediaCodecConstants.h:1123`, `:1143-1144`): headers again in front of every sync frame
    /// (`KEY_PREPEND_HEADER_TO_SYNC_FRAMES`), and the quantiser bounds (`KEY_VIDEO_QP_MIN` /
    /// `_MAX`, honoured by a codec with `FEATURE_QpBounds`).
    pub const PREPEND_HEADER_TO_SYNC_FRAMES: &CStr = c"prepend-sps-pps-to-idr-frames";
    pub const VIDEO_QP_MIN: &CStr = c"video-qp-min";
    pub const VIDEO_QP_MAX: &CStr = c"video-qp-max";
    /// `AMEDIACODEC_KEY_*` for `setParameters` (`NdkMediaCodec.cpp:1142-1148`).
    pub const REQUEST_SYNC_FRAME: &CStr = c"request-sync";
    pub const VIDEO_BITRATE: &CStr = c"video-bitrate";
}

#[derive(Error, Debug, Clone)]
pub enum CodecError {
    #[error("could not load {0}: {1}")]
    LibraryLoad(&'static str, String),
    #[error("{0} is missing from the media NDK")]
    MissingSymbol(&'static str),
    #[error("{0} failed: {1} ({2})")]
    Ndk(&'static str, i32, &'static str),
    #[error("{0} returned null")]
    Null(&'static str),
    #[error("{0} contained an interior NUL")]
    BadString(&'static str),
    #[error("input buffer {0} holds {1} bytes, {2} needed")]
    BufferTooSmall(i32, usize, usize),
    #[error("filling input buffer: {0}")]
    Fill(String),
    #[error("event fd: {0}")]
    Event(String),
}

type Result<T> = std::result::Result<T, CodecError>;

/// `media_status_t` (`NdkMediaError.h:46-120`), plus the one `ssize_t` info code that
/// `translate_error` also returns as a status.
pub fn media_status_name(status: i32) -> &'static str {
    match status {
        0 => "AMEDIA_OK",
        -1 => "AMEDIACODEC_INFO_TRY_AGAIN_LATER (EAGAIN)",
        1100 => "AMEDIACODEC_ERROR_INSUFFICIENT_RESOURCE",
        1101 => "AMEDIACODEC_ERROR_RECLAIMED",
        -10000 => "AMEDIA_ERROR_UNKNOWN",
        -10001 => "AMEDIA_ERROR_MALFORMED",
        -10002 => "AMEDIA_ERROR_UNSUPPORTED",
        -10003 => "AMEDIA_ERROR_INVALID_OBJECT",
        -10004 => "AMEDIA_ERROR_INVALID_PARAMETER",
        -10005 => "AMEDIA_ERROR_INVALID_OPERATION",
        -10006 => "AMEDIA_ERROR_END_OF_STREAM",
        -10007 => "AMEDIA_ERROR_IO",
        -10008 => "AMEDIA_ERROR_WOULD_BLOCK",
        -20000 => "AMEDIA_DRM_ERROR_BASE",
        -30000 => "AMEDIA_IMGREADER_ERROR_BASE",
        _ => "?",
    }
}

fn check(what: &'static str, status: i32) -> Result<()> {
    if status == AMEDIA_OK {
        Ok(())
    } else {
        Err(CodecError::Ndk(what, status, media_status_name(status)))
    }
}

fn cstring(what: &'static str, s: &str) -> Result<CString> {
    CString::new(s).map_err(|_| CodecError::BadString(what))
}

/// # Safety
///
/// `ptr` must be null or a NUL-terminated string.
unsafe fn string_from(ptr: *const c_char) -> Option<String> {
    if ptr.is_null() {
        None
    } else {
        Some(CStr::from_ptr(ptr).to_string_lossy().into_owned())
    }
}

/// A `COLOR_Format*` value, named where the value is one of the ones a decoder or encoder is
/// likely to report.
pub fn color_format_name(v: i32) -> String {
    let name = match v {
        COLOR_FORMAT_YUV420_PLANAR => "YUV420Planar",
        COLOR_FORMAT_YUV420_PACKED_PLANAR => "YUV420PackedPlanar",
        COLOR_FORMAT_YUV420_SEMI_PLANAR => "YUV420SemiPlanar",
        COLOR_FORMAT_YUV420_PACKED_SEMI_PLANAR => "YUV420PackedSemiPlanar",
        COLOR_FORMAT_YUV420_FLEXIBLE => "YUV420Flexible",
        COLOR_FORMAT_YUV_P010 => "YUVP010",
        COLOR_FORMAT_SURFACE => "Surface",
        COLOR_QCOM_FORMAT_YUV420_SEMI_PLANAR => "QCOM_YUV420SemiPlanar",
        0x7F42_2888 => "YUV422Flexible",
        0x7F44_4888 => "YUV444Flexible",
        0x7F00_0100 => "TI_YUV420PackedSemiPlanar",
        _ => "",
    };
    if name.is_empty() {
        format!("0x{v:x}")
    } else {
        format!("{name} (0x{v:x})")
    }
}

/// An owned `AMediaFormat`.
pub struct MediaFormat {
    ptr: *mut AMediaFormat,
}

// SAFETY: an AMediaFormat is a heap object with no thread affinity; every entry point takes the
// pointer and locks nothing thread-local.
unsafe impl Send for MediaFormat {}

impl MediaFormat {
    pub fn new() -> Result<MediaFormat> {
        ensure_loaded()?;
        // SAFETY: plain constructor; the result is checked.
        let ptr = unsafe { AMediaFormat_new() };
        if ptr.is_null() {
            return Err(CodecError::Null("AMediaFormat_new"));
        }
        Ok(MediaFormat { ptr })
    }

    /// Take ownership of a format the NDK handed out (`getOutputFormat`, `getInputFormat`,
    /// `getBufferFormat`, the format-changed callback). `None` for null.
    ///
    /// # Safety
    ///
    /// `ptr` must be null or an `AMediaFormat` nobody else will delete.
    unsafe fn from_raw(ptr: *mut AMediaFormat) -> Option<MediaFormat> {
        if ptr.is_null() {
            None
        } else {
            Some(MediaFormat { ptr })
        }
    }

    pub fn as_ptr(&self) -> *const AMediaFormat {
        self.ptr
    }

    pub fn set_i32(&mut self, key: impl AsRef<CStr>, value: i32) {
        // SAFETY: the format is live and the key NUL-terminated.
        unsafe { AMediaFormat_setInt32(self.ptr, key.as_ref().as_ptr(), value) }
    }

    pub fn set_i64(&mut self, key: impl AsRef<CStr>, value: i64) {
        // SAFETY: as above.
        unsafe { AMediaFormat_setInt64(self.ptr, key.as_ref().as_ptr(), value) }
    }

    pub fn set_f32(&mut self, key: impl AsRef<CStr>, value: f32) {
        // SAFETY: as above.
        unsafe { AMediaFormat_setFloat(self.ptr, key.as_ref().as_ptr(), value) }
    }

    pub fn set_str(&mut self, key: impl AsRef<CStr>, value: &str) -> Result<()> {
        let value = cstring("format string value", value)?;
        // SAFETY: as above; the string is copied into the format.
        unsafe { AMediaFormat_setString(self.ptr, key.as_ref().as_ptr(), value.as_ptr()) }
        Ok(())
    }

    pub fn set_buffer(&mut self, key: impl AsRef<CStr>, value: &[u8]) {
        // SAFETY: as above; the bytes are copied into the format.
        unsafe {
            AMediaFormat_setBuffer(
                self.ptr,
                key.as_ref().as_ptr(),
                value.as_ptr() as *const c_void,
                value.len(),
            )
        }
    }

    pub fn get_i32(&self, key: impl AsRef<CStr>) -> Option<i32> {
        let mut out = 0i32;
        // SAFETY: the format is live and `out` outlives the call.
        unsafe { AMediaFormat_getInt32(self.ptr, key.as_ref().as_ptr(), &mut out) }.then_some(out)
    }

    pub fn get_i64(&self, key: impl AsRef<CStr>) -> Option<i64> {
        let mut out = 0i64;
        // SAFETY: as above.
        unsafe { AMediaFormat_getInt64(self.ptr, key.as_ref().as_ptr(), &mut out) }.then_some(out)
    }

    pub fn get_f32(&self, key: impl AsRef<CStr>) -> Option<f32> {
        let mut out = 0f32;
        // SAFETY: as above.
        unsafe { AMediaFormat_getFloat(self.ptr, key.as_ref().as_ptr(), &mut out) }.then_some(out)
    }

    pub fn get_str(&self, key: impl AsRef<CStr>) -> Option<String> {
        let mut out: *const c_char = null();
        // SAFETY: as above; the string is owned by the format and copied out before any other
        // call on it.
        unsafe {
            AMediaFormat_getString(self.ptr, key.as_ref().as_ptr(), &mut out)
                .then(|| string_from(out))
                .flatten()
        }
    }

    pub fn get_buffer(&self, key: impl AsRef<CStr>) -> Option<Vec<u8>> {
        let mut data: *mut c_void = null_mut();
        let mut size = 0usize;
        // SAFETY: as above; the bytes are owned by the format and copied out at once.
        unsafe {
            if AMediaFormat_getBuffer(self.ptr, key.as_ref().as_ptr(), &mut data, &mut size)
                && !data.is_null()
            {
                Some(std::slice::from_raw_parts(data as *const u8, size).to_vec())
            } else {
                None
            }
        }
    }

    /// `(left, top, right, bottom)`, inclusive, as the framework stores `"crop"`.
    pub fn get_rect(&self, key: impl AsRef<CStr>) -> Option<(i32, i32, i32, i32)> {
        let (mut l, mut t, mut r, mut b) = (0i32, 0i32, 0i32, 0i32);
        // SAFETY: as above.
        unsafe {
            AMediaFormat_getRect(
                self.ptr,
                key.as_ref().as_ptr(),
                &mut l,
                &mut t,
                &mut r,
                &mut b,
            )
        }
        .then_some((l, t, r, b))
    }

    /// The `"crop"` rect, or the four `crop-*` integers Java exposes, as
    /// `(left, top, right, bottom)`.
    pub fn crop(&self) -> Option<(i32, i32, i32, i32)> {
        self.get_rect(keys::DISPLAY_CROP).or_else(|| {
            Some((
                self.get_i32(keys::CROP_LEFT)?,
                self.get_i32(keys::CROP_TOP)?,
                self.get_i32(keys::CROP_RIGHT)?,
                self.get_i32(keys::CROP_BOTTOM)?,
            ))
        })
    }

    /// The parsed `"image-data"` blob, when the format carries one.
    pub fn image_data(&self) -> Option<std::result::Result<MediaImage2, ImageError>> {
        self.get_buffer(keys::IMAGE_DATA)
            .map(|bytes| MediaImage2::parse(&bytes))
    }
}

impl std::fmt::Display for MediaFormat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // SAFETY: the string is owned by the format, valid until the next toString.
        let s = unsafe { string_from(AMediaFormat_toString(self.ptr)) };
        f.write_str(s.as_deref().unwrap_or("(null)"))
    }
}

impl std::fmt::Debug for MediaFormat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "MediaFormat({self})")
    }
}

impl Drop for MediaFormat {
    fn drop(&mut self) {
        // SAFETY: we own the format and nothing else refers to it.
        unsafe {
            AMediaFormat_delete(self.ptr);
        }
    }
}

/// What a decoder session is configured with.
pub fn decoder_format(
    mime: &str,
    width: i32,
    height: i32,
    csd0: &[u8],
    csd1: &[u8],
) -> Result<MediaFormat> {
    let mut f = MediaFormat::new()?;
    f.set_str(keys::MIME, mime)?;
    f.set_i32(keys::WIDTH, width);
    f.set_i32(keys::HEIGHT, height);
    f.set_i32(keys::COLOR_FORMAT, COLOR_FORMAT_YUV420_FLEXIBLE);
    if !csd0.is_empty() {
        f.set_buffer(keys::CSD_0, csd0);
    }
    if !csd1.is_empty() {
        f.set_buffer(keys::CSD_1, csd1);
    }
    Ok(f)
}

/// What an encoder session is configured with.
#[derive(Debug, Clone)]
pub struct EncoderConfig {
    pub mime: String,
    pub width: i32,
    pub height: i32,
    /// `COLOR_Format*`: `YUV420Flexible` lets the component pick, a concrete `YUV420SemiPlanar`
    /// is what the encoder device asks for (design 7.3).
    pub color_format: i32,
    pub bitrate: i32,
    pub frame_rate: f32,
    pub i_frame_interval_s: i32,
    pub bitrate_mode: Option<i32>,
    pub profile: Option<i32>,
    pub level: Option<i32>,
}

impl EncoderConfig {
    pub fn to_format(&self) -> Result<MediaFormat> {
        let mut f = MediaFormat::new()?;
        f.set_str(keys::MIME, &self.mime)?;
        f.set_i32(keys::WIDTH, self.width);
        f.set_i32(keys::HEIGHT, self.height);
        f.set_i32(keys::COLOR_FORMAT, self.color_format);
        f.set_i32(keys::BIT_RATE, self.bitrate);
        // "Int32 or Float": integral rates go as the int the Java world writes.
        if self.frame_rate.fract() == 0.0 {
            f.set_i32(keys::FRAME_RATE, self.frame_rate as i32);
        } else {
            f.set_f32(keys::FRAME_RATE, self.frame_rate);
        }
        f.set_i32(keys::I_FRAME_INTERVAL, self.i_frame_interval_s);
        if let Some(m) = self.bitrate_mode {
            f.set_i32(keys::BITRATE_MODE, m);
        }
        if let Some(p) = self.profile {
            f.set_i32(keys::PROFILE, p);
        }
        if let Some(l) = self.level {
            f.set_i32(keys::LEVEL, l);
        }
        Ok(f)
    }
}

/// The encoder's input layout, read back after `configure` (`CCodec.cpp:2004-2053`): `stride`
/// is published whenever byte-buffer input works at all; `slice-height` is omitted when the
/// chroma offset is not a whole number of strides, which is the signal that a
/// `Y[stride * sliceHeight]` packing would be wrong.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct InputLayout {
    pub color_format: Option<i32>,
    /// Falls back to the width when absent (`MediaCodec_sanity_test.cpp:357`).
    pub stride: i32,
    pub stride_reported: bool,
    pub slice_height: Option<i32>,
    pub width: Option<i32>,
    pub height: Option<i32>,
}

impl InputLayout {
    pub fn from_format(f: &MediaFormat, width: i32) -> InputLayout {
        let stride = f.get_i32(keys::STRIDE);
        InputLayout {
            color_format: f.get_i32(keys::COLOR_FORMAT),
            stride: stride.unwrap_or(f.get_i32(keys::WIDTH).unwrap_or(width)),
            stride_reported: stride.is_some(),
            slice_height: f.get_i32(keys::SLICE_HEIGHT),
            width: f.get_i32(keys::WIDTH),
            height: f.get_i32(keys::HEIGHT),
        }
    }
}

/// The decoder's output layout for one buffer (`getBufferFormat`) or the stream
/// (`getOutputFormat`).
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct OutputLayout {
    pub width: Option<i32>,
    pub height: Option<i32>,
    pub color_format: Option<i32>,
    pub stride: Option<i32>,
    pub slice_height: Option<i32>,
    /// `(left, top, right, bottom)`, inclusive.
    pub crop: Option<(i32, i32, i32, i32)>,
    pub display: (Option<i32>, Option<i32>),
    pub rotation: Option<i32>,
    pub color: (Option<i32>, Option<i32>, Option<i32>),
    pub image_data_len: Option<usize>,
    pub image: Option<MediaImage2>,
    pub image_error: Option<String>,
}

impl OutputLayout {
    pub fn from_format(f: &MediaFormat) -> OutputLayout {
        let raw = f.get_buffer(keys::IMAGE_DATA);
        let parsed = raw.as_deref().map(MediaImage2::parse);
        OutputLayout {
            width: f.get_i32(keys::WIDTH),
            height: f.get_i32(keys::HEIGHT),
            color_format: f.get_i32(keys::COLOR_FORMAT),
            stride: f.get_i32(keys::STRIDE),
            slice_height: f.get_i32(keys::SLICE_HEIGHT),
            crop: f.crop(),
            display: (
                f.get_i32(keys::DISPLAY_WIDTH),
                f.get_i32(keys::DISPLAY_HEIGHT),
            ),
            rotation: f.get_i32(keys::ROTATION),
            color: (
                f.get_i32(keys::COLOR_RANGE),
                f.get_i32(keys::COLOR_STANDARD),
                f.get_i32(keys::COLOR_TRANSFER),
            ),
            image_data_len: raw.as_ref().map(Vec::len),
            image: parsed.as_ref().and_then(|p| p.as_ref().ok().copied()),
            image_error: parsed.and_then(|p| p.err().map(|e| e.to_string())),
        }
    }

    /// Stride and slice height with the framework's fallbacks (`stride` -> `width`,
    /// `slice-height` -> `height`).
    pub fn effective_stride(&self) -> Option<i32> {
        self.stride.or(self.width)
    }

    pub fn effective_slice_height(&self) -> Option<i32> {
        self.slice_height.or(self.height)
    }
}

/// One codec as the platform describes it: what a V4L2 decoder or encoder device answers
/// `ENUM_FMT`, `ENUM_FRAMESIZES`, `ENUM_FRAMEINTERVALS` and the profile/level menus from.
#[derive(Debug, Clone, Serialize)]
pub struct CodecInfo {
    /// The component name: what `AMediaCodec_createCodecByName` wants. From
    /// `AMediaCodecInfo_getCanonicalName`, or `AMediaCodec_getName` in the fallback. (The
    /// Store's own name, with its `.video/avc` and `.1` suffixes, is not retrievable from an
    /// `AMediaCodecInfo`, and is not what `createCodecByName` takes.)
    pub name: String,
    pub mime: String,
    pub kind: CodecKind,
    pub codec_type: CodecType,
    pub is_vendor: bool,
    pub max_instances: Option<i32>,
    /// `(feature, supported, required)` for each `AMediaCodecInfo_FEATURE_*` the platform names.
    pub features: Vec<FeatureSupport>,
    pub video: Option<VideoCaps>,
    pub encoder: Option<EncoderCaps>,
    /// Profiles the codec supports, each with the levels it supports at that profile, found by
    /// asking `AMediaCodecInfo_isFormatSupported` about a candidate table: the NDK has no
    /// `getProfileLevels`.
    pub profiles: Vec<ProfileSupport>,
}

#[derive(Debug, Clone, Serialize)]
pub struct FeatureSupport {
    pub feature: String,
    pub supported: bool,
    pub required: bool,
}

#[derive(Debug, Clone, Serialize)]
pub struct ProfileSupport {
    pub profile: i32,
    pub profile_name: String,
    pub levels: Vec<LevelSupport>,
}

#[derive(Debug, Clone, Serialize)]
pub struct LevelSupport {
    pub level: i32,
    pub level_name: String,
}

#[derive(Debug, Clone, Serialize)]
pub struct VideoCaps {
    pub widths: (i32, i32),
    pub heights: (i32, i32),
    pub width_alignment: i32,
    pub height_alignment: i32,
    pub frame_rates: (i32, i32),
    pub bitrates: (i32, i32),
    /// `getSupportedWidthsFor(max height)` and `getSupportedHeightsFor(max width)`: whether the
    /// size range is a rectangle or a budget.
    pub widths_at_max_height: Option<(i32, i32)>,
    pub heights_at_max_width: Option<(i32, i32)>,
    /// Number of framework-published performance points.
    pub performance_points: usize,
    /// Point checks at the sizes a guest will ask about.
    pub sizes: Vec<SizeCheck>,
}

#[derive(Debug, Clone, Serialize)]
pub struct SizeCheck {
    pub width: i32,
    pub height: i32,
    pub supported: bool,
    /// `getSupportedFrameRatesFor`: the standard's limit at this size.
    pub frame_rates: Option<(f64, f64)>,
    /// `getAchievableFrameRatesFor`: the manufacturer-measured range, when published.
    pub achievable: Option<(f64, f64)>,
    /// The highest of 24/30/60/120/240 fps that `areSizeAndRateSupported` accepts here.
    pub max_rate_supported: Option<i32>,
    /// The highest of the same candidates a published performance point covers.
    pub max_rate_covered: Option<i32>,
}

#[derive(Debug, Clone, Serialize)]
pub struct EncoderCaps {
    pub quality: Option<(i32, i32)>,
    pub complexity: Option<(i32, i32)>,
    /// `(mode name, supported)` for CQ, VBR, CBR, CBR_FD.
    pub bitrate_modes: Vec<(String, bool)>,
}

/// One entry of `AMediaCodecStore_getSupportedMediaTypes`.
#[derive(Debug, Clone, Serialize)]
pub struct MediaType {
    pub mime: String,
    pub decoder: bool,
    pub encoder: bool,
}

/// The whole listing.
#[derive(Debug, Clone, Serialize)]
pub struct CodecList {
    /// `"AMediaCodecStore"` or `"createDecoderByType probe"`.
    pub source: &'static str,
    /// Optional entry points this platform lacks.
    pub missing_symbols: Vec<&'static str>,
    pub media_types: Vec<MediaType>,
    pub codecs: Vec<CodecInfo>,
}

/// What [`list_codecs`] should spend its time on.
#[derive(Debug, Clone)]
pub struct ListOptions {
    /// Include audio and image media types (the devices only want `video/*`).
    pub include_non_video: bool,
    /// Ask `isFormatSupported` about every candidate profile and level.
    pub probe_profiles: bool,
    /// Sizes for the per-size checks.
    pub sizes: Vec<(i32, i32)>,
}

impl Default for ListOptions {
    fn default() -> Self {
        ListOptions {
            include_non_video: false,
            probe_profiles: true,
            sizes: vec![
                (320, 240),
                (640, 480),
                (1280, 720),
                (1920, 1080),
                (3840, 2160),
                (4096, 2160),
                (7680, 4320),
            ],
        }
    }
}

/// The mimes the fallback probes, and the `ENUM_FMT` row order of the decoder device.
pub const VIDEO_MIMES: &[&str] = &[
    "video/avc",
    "video/hevc",
    "video/x-vnd.on2.vp8",
    "video/x-vnd.on2.vp9",
    "video/av01",
    "video/mp4v-es",
    "video/3gpp",
];

const RATE_CANDIDATES: [i32; 5] = [24, 30, 60, 120, 240];

/// Enumerate the platform's codecs. Uses `AMediaCodecStore` when the platform has it; otherwise
/// creates a decoder and an encoder by type for each of [`VIDEO_MIMES`] and reports their names.
///
/// The Store's lazily built tables have no lock (`NdkMediaCodecStore.cpp:163-165`): call this
/// once, from one thread, before any concurrent use.
pub fn list_codecs(opts: &ListOptions) -> Result<CodecList> {
    ensure_loaded()?;
    let api = ndk();
    let missing = api.missing();
    if api.AMediaCodecStore_getSupportedMediaTypes.is_none()
        || api.AMediaCodecStore_findNextDecoderForFormat.is_none()
        || api.AMediaCodecStore_findNextEncoderForFormat.is_none()
    {
        return probe_by_type(missing);
    }

    let mut types: *const AMediaCodecSupportedMediaType = null();
    let mut count = 0usize;
    // SAFETY: the array is framework-owned with infinite lifetime; only read here.
    let media_types = unsafe {
        check(
            "AMediaCodecStore_getSupportedMediaTypes",
            (opt!(AMediaCodecStore_getSupportedMediaTypes))(&mut types, &mut count),
        )?;
        let mut out = Vec::with_capacity(count);
        if !types.is_null() {
            for t in std::slice::from_raw_parts(types, count) {
                let mime = string_from(t.media_type).unwrap_or_default();
                out.push(MediaType {
                    mime,
                    decoder: t.mode & MEDIA_TYPE_FLAG_DECODER != 0,
                    encoder: t.mode & MEDIA_TYPE_FLAG_ENCODER != 0,
                });
            }
        }
        out
    };

    let mut codecs = Vec::new();
    for kind in [CodecKind::Decoder, CodecKind::Encoder] {
        let next = match kind {
            CodecKind::Decoder => opt!(AMediaCodecStore_findNextDecoderForFormat),
            _ => opt!(AMediaCodecStore_findNextEncoderForFormat),
        };
        let mut info: *const AMediaCodecInfo = null();
        // A null format iterates every codec of that kind (NdkMediaCodecStore.h:81-99).
        loop {
            // SAFETY: the iteration contract: seed null, keep the last pointer, stop at
            // UNSUPPORTED / null. Infos are framework-owned.
            let status = unsafe { next(null(), &mut info) };
            if status != AMEDIA_OK || info.is_null() {
                break;
            }
            // SAFETY: `info` is a live framework-owned object.
            let described = unsafe { describe(info, kind, opts)? };
            if opts.include_non_video || described.mime.starts_with("video/") {
                codecs.push(described);
            }
        }
    }
    Ok(CodecList {
        source: "AMediaCodecStore",
        missing_symbols: missing,
        media_types,
        codecs,
    })
}

/// # Safety
///
/// `info` must be a live `AMediaCodecInfo`.
unsafe fn describe(
    info: *const AMediaCodecInfo,
    kind: CodecKind,
    opts: &ListOptions,
) -> Result<CodecInfo> {
    let api = ndk();
    let name = api
        .AMediaCodecInfo_getCanonicalName
        .and_then(|f| string_from(f(info)))
        .unwrap_or_default();
    let mime = api
        .AMediaCodecInfo_getMediaType
        .and_then(|f| string_from(f(info)))
        .unwrap_or_default();
    let reported_kind = api
        .AMediaCodecInfo_getKind
        .map(|f| CodecKind::from(f(info)))
        .unwrap_or(kind);
    let codec_type = api
        .AMediaCodecInfo_getMediaCodecInfoType
        .map(|f| CodecType::from(f(info)))
        .unwrap_or(CodecType::Unknown);
    let is_vendor = api
        .AMediaCodecInfo_isVendor
        .map(|f| f(info) != 0)
        .unwrap_or(false);
    let max_instances = api
        .AMediaCodecInfo_getMaxSupportedInstances
        .map(|f| f(info));

    let mut features = Vec::new();
    if let (Some(supported), Some(required)) = (
        api.AMediaCodecInfo_isFeatureSupported,
        api.AMediaCodecInfo_isFeatureRequired,
    ) {
        for (_symbol, feature) in api.feature_names() {
            features.push(FeatureSupport {
                feature: feature.to_string_lossy().into_owned(),
                supported: supported(info, feature.as_ptr()) != 0,
                required: required(info, feature.as_ptr()) != 0,
            });
        }
    }

    let video = video_caps(info, opts);
    let encoder = if reported_kind == CodecKind::Encoder {
        encoder_caps(info)
    } else {
        None
    };
    let profiles = if opts.probe_profiles {
        probe_profiles(info, &mime)?
    } else {
        Vec::new()
    };

    Ok(CodecInfo {
        name,
        mime,
        kind: reported_kind,
        codec_type,
        is_vendor,
        max_instances,
        features,
        video,
        encoder,
        profiles,
    })
}

/// # Safety
///
/// `info` must be a live `AMediaCodecInfo`.
unsafe fn video_caps(info: *const AMediaCodecInfo, opts: &ListOptions) -> Option<VideoCaps> {
    let api = ndk();
    let mut caps: *const ACodecVideoCapabilities = null();
    if (api.AMediaCodecInfo_getVideoCapabilities?)(info, &mut caps) != AMEDIA_OK || caps.is_null() {
        return None;
    }
    let int_range =
        |f: Option<unsafe extern "C" fn(*const ACodecVideoCapabilities, *mut AIntRange) -> i32>| {
            let mut r = AIntRange::default();
            (f?(caps, &mut r) == AMEDIA_OK).then_some((r.lower, r.upper))
        };
    let widths = int_range(api.ACodecVideoCapabilities_getSupportedWidths)?;
    let heights = int_range(api.ACodecVideoCapabilities_getSupportedHeights)?;
    let frame_rates =
        int_range(api.ACodecVideoCapabilities_getSupportedFrameRates).unwrap_or((0, 0));
    let bitrates = int_range(api.ACodecVideoCapabilities_getBitrateRange).unwrap_or((0, 0));
    let widths_at_max_height = api
        .ACodecVideoCapabilities_getSupportedWidthsFor
        .and_then(|f| {
            let mut r = AIntRange::default();
            (f(caps, heights.1, &mut r) == AMEDIA_OK).then_some((r.lower, r.upper))
        });
    let heights_at_max_width = api
        .ACodecVideoCapabilities_getSupportedHeightsFor
        .and_then(|f| {
            let mut r = AIntRange::default();
            (f(caps, widths.1, &mut r) == AMEDIA_OK).then_some((r.lower, r.upper))
        });

    // Performance points: framework-owned, iterated like the store; there is no accessor for a
    // point's own size and rate, only `covers`, so they are counted and tested against the
    // candidates below.
    let mut points: Vec<*const ACodecPerformancePoint> = Vec::new();
    if let Some(next) = api.ACodecVideoCapabilities_getNextSupportedPerformancePoint {
        let mut p: *const ACodecPerformancePoint = null();
        while points.len() < 256 && next(caps, &mut p) == AMEDIA_OK && !p.is_null() {
            points.push(p);
        }
    }

    let mut sizes = Vec::new();
    for &(w, h) in &opts.sizes {
        let supported = api
            .ACodecVideoCapabilities_isSizeSupported
            .map(|f| f(caps, w, h) != 0)
            .unwrap_or(false);
        let double_range = |f: Option<
            unsafe extern "C" fn(
                *const ACodecVideoCapabilities,
                i32,
                i32,
                *mut ADoubleRange,
            ) -> i32,
        >| {
            let mut r = ADoubleRange::default();
            (f?(caps, w, h, &mut r) == AMEDIA_OK).then_some((r.lower, r.upper))
        };
        let (frame_rates_for, achievable) = if supported {
            (
                double_range(api.ACodecVideoCapabilities_getSupportedFrameRatesFor),
                double_range(api.ACodecVideoCapabilities_getAchievableFrameRatesFor),
            )
        } else {
            (None, None)
        };
        let max_rate_supported = api
            .ACodecVideoCapabilities_areSizeAndRateSupported
            .and_then(|f| {
                RATE_CANDIDATES
                    .iter()
                    .rev()
                    .copied()
                    .find(|&rate| f(caps, w, h, rate as f64) != 0)
            });
        let max_rate_covered = match (
            api.ACodecPerformancePoint_create,
            api.ACodecPerformancePoint_destroy,
            api.ACodecPerformancePoint_covers,
        ) {
            (Some(create), Some(destroy), Some(covers)) if !points.is_empty() => {
                RATE_CANDIDATES.iter().rev().copied().find(|&rate| {
                    let candidate = create(w, h, rate);
                    if candidate.is_null() {
                        return false;
                    }
                    let hit = points.iter().any(|&p| covers(p, candidate) != 0);
                    destroy(candidate);
                    hit
                })
            }
            _ => None,
        };
        sizes.push(SizeCheck {
            width: w,
            height: h,
            supported,
            frame_rates: frame_rates_for,
            achievable,
            max_rate_supported,
            max_rate_covered,
        });
    }

    Some(VideoCaps {
        widths,
        heights,
        width_alignment: api
            .ACodecVideoCapabilities_getWidthAlignment
            .map(|f| f(caps))
            .unwrap_or(0),
        height_alignment: api
            .ACodecVideoCapabilities_getHeightAlignment
            .map(|f| f(caps))
            .unwrap_or(0),
        frame_rates,
        bitrates,
        widths_at_max_height,
        heights_at_max_width,
        performance_points: points.len(),
        sizes,
    })
}

/// # Safety
///
/// `info` must be a live `AMediaCodecInfo`.
unsafe fn encoder_caps(info: *const AMediaCodecInfo) -> Option<EncoderCaps> {
    let api = ndk();
    let mut caps: *const ACodecEncoderCapabilities = null();
    if (api.AMediaCodecInfo_getEncoderCapabilities?)(info, &mut caps) != AMEDIA_OK || caps.is_null()
    {
        return None;
    }
    let int_range = |f: Option<
        unsafe extern "C" fn(*const ACodecEncoderCapabilities, *mut AIntRange) -> i32,
    >| {
        let mut r = AIntRange::default();
        (f?(caps, &mut r) == AMEDIA_OK).then_some((r.lower, r.upper))
    };
    let modes = [
        ("CQ", BITRATE_MODE_CQ),
        ("VBR", BITRATE_MODE_VBR),
        ("CBR", BITRATE_MODE_CBR),
        ("CBR_FD", BITRATE_MODE_CBR_FD),
    ];
    let bitrate_modes = match api.ACodecEncoderCapabilities_isBitrateModeSupported {
        Some(f) => modes
            .iter()
            .map(|(name, mode)| (name.to_string(), f(caps, *mode) != 0))
            .collect(),
        None => Vec::new(),
    };
    Some(EncoderCaps {
        quality: int_range(api.ACodecEncoderCapabilities_getQualityRange),
        complexity: int_range(api.ACodecEncoderCapabilities_getComplexityRange),
        bitrate_modes,
    })
}

/// Candidate profiles per mime (`MediaCodecConstants.h:26-34, 98-106, 146-161, 294, 303-310,
/// 361-364, 431-435`).
pub fn profile_table(mime: &str) -> &'static [(i32, &'static str)] {
    match mime {
        "video/avc" => &[
            (0x01, "Baseline"),
            (0x02, "Main"),
            (0x04, "Extended"),
            (0x08, "High"),
            (0x10, "High10"),
            (0x20, "High422"),
            (0x40, "High444"),
            (0x10000, "ConstrainedBaseline"),
            (0x80000, "ConstrainedHigh"),
        ],
        "video/hevc" => &[
            (0x01, "Main"),
            (0x02, "Main10"),
            (0x04, "MainStill"),
            (0x1000, "Main10HDR10"),
            (0x2000, "Main10HDR10Plus"),
        ],
        "video/x-vnd.on2.vp8" => &[(0x01, "Main")],
        "video/x-vnd.on2.vp9" => &[
            (0x01, "0"),
            (0x02, "1"),
            (0x04, "2"),
            (0x08, "3"),
            (0x1000, "2HDR"),
            (0x2000, "3HDR"),
            (0x4000, "2HDR10Plus"),
            (0x8000, "3HDR10Plus"),
        ],
        "video/av01" => &[
            (0x1, "Main8"),
            (0x2, "Main10"),
            (0x1000, "Main10HDR10"),
            (0x2000, "Main10HDR10Plus"),
        ],
        "video/mp4v-es" => &[
            (0x01, "Simple"),
            (0x02, "SimpleScalable"),
            (0x04, "Core"),
            (0x08, "Main"),
            (0x8000, "AdvancedSimple"),
        ],
        "video/3gpp" => &[
            (0x01, "Baseline"),
            (0x02, "H320Coding"),
            (0x04, "BackwardCompatible"),
            (0x08, "ISWV2"),
        ],
        _ => &[],
    }
}

/// Candidate levels per mime (`MediaCodecConstants.h:51-70, 279-282, 326-339, 376-399,
/// 448-473`).
pub fn level_table(mime: &str) -> &'static [(i32, &'static str)] {
    match mime {
        "video/avc" => &[
            (0x01, "1"),
            (0x02, "1b"),
            (0x04, "1.1"),
            (0x08, "1.2"),
            (0x10, "1.3"),
            (0x20, "2"),
            (0x40, "2.1"),
            (0x80, "2.2"),
            (0x100, "3"),
            (0x200, "3.1"),
            (0x400, "3.2"),
            (0x800, "4"),
            (0x1000, "4.1"),
            (0x2000, "4.2"),
            (0x4000, "5"),
            (0x8000, "5.1"),
            (0x10000, "5.2"),
            (0x20000, "6"),
            (0x40000, "6.1"),
            (0x80000, "6.2"),
        ],
        "video/hevc" => &[
            (0x1, "MainTier1"),
            (0x2, "HighTier1"),
            (0x4, "MainTier2"),
            (0x8, "HighTier2"),
            (0x10, "MainTier2.1"),
            (0x20, "HighTier2.1"),
            (0x40, "MainTier3"),
            (0x80, "HighTier3"),
            (0x100, "MainTier3.1"),
            (0x200, "HighTier3.1"),
            (0x400, "MainTier4"),
            (0x800, "HighTier4"),
            (0x1000, "MainTier4.1"),
            (0x2000, "HighTier4.1"),
            (0x4000, "MainTier5"),
            (0x8000, "HighTier5"),
            (0x10000, "MainTier5.1"),
            (0x20000, "HighTier5.1"),
            (0x40000, "MainTier5.2"),
            (0x80000, "HighTier5.2"),
            (0x100000, "MainTier6"),
            (0x200000, "HighTier6"),
            (0x400000, "MainTier6.1"),
            (0x800000, "HighTier6.1"),
            (0x1000000, "MainTier6.2"),
            (0x2000000, "HighTier6.2"),
        ],
        "video/x-vnd.on2.vp8" => &[(0x01, "V0"), (0x02, "V1"), (0x04, "V2"), (0x08, "V3")],
        "video/x-vnd.on2.vp9" => &[
            (0x1, "1"),
            (0x2, "1.1"),
            (0x4, "2"),
            (0x8, "2.1"),
            (0x10, "3"),
            (0x20, "3.1"),
            (0x40, "4"),
            (0x80, "4.1"),
            (0x100, "5"),
            (0x200, "5.1"),
            (0x400, "5.2"),
            (0x800, "6"),
            (0x1000, "6.1"),
            (0x2000, "6.2"),
        ],
        "video/av01" => &[
            (0x1, "2"),
            (0x2, "2.1"),
            (0x4, "2.2"),
            (0x8, "2.3"),
            (0x10, "3"),
            (0x20, "3.1"),
            (0x40, "3.2"),
            (0x80, "3.3"),
            (0x100, "4"),
            (0x200, "4.1"),
            (0x400, "4.2"),
            (0x800, "4.3"),
            (0x1000, "5"),
            (0x2000, "5.1"),
            (0x4000, "5.2"),
            (0x8000, "5.3"),
            (0x10000, "6"),
            (0x20000, "6.1"),
            (0x40000, "6.2"),
            (0x80000, "6.3"),
            (0x100000, "7"),
            (0x200000, "7.1"),
            (0x400000, "7.2"),
            (0x800000, "7.3"),
        ],
        _ => &[],
    }
}

/// Ask `isFormatSupported` about each candidate profile, then about each level at the profiles
/// that passed. `CodecCapabilities::isFormatSupported` checks mime, `feature-*` keys and
/// profile/level and nothing else (`CodecCapabilities.cpp:139-231`), so a format holding only
/// the mime and a profile is a pure profile query, and a missing level means "any"
/// (`supportsProfileLevel`, `:251`).
///
/// # Safety
///
/// `info` must be a live `AMediaCodecInfo`.
unsafe fn probe_profiles(info: *const AMediaCodecInfo, mime: &str) -> Result<Vec<ProfileSupport>> {
    let Some(is_supported) = ndk().AMediaCodecInfo_isFormatSupported else {
        return Ok(Vec::new());
    };
    let mut out = Vec::new();
    for &(profile, profile_name) in profile_table(mime) {
        let mut f = MediaFormat::new()?;
        f.set_str(keys::MIME, mime)?;
        f.set_i32(keys::PROFILE, profile);
        if is_supported(info, f.as_ptr()) == 0 {
            continue;
        }
        let mut levels = Vec::new();
        for &(level, level_name) in level_table(mime) {
            f.set_i32(keys::LEVEL, level);
            if is_supported(info, f.as_ptr()) != 0 {
                levels.push(LevelSupport {
                    level,
                    level_name: level_name.to_owned(),
                });
            }
        }
        out.push(ProfileSupport {
            profile,
            profile_name: profile_name.to_owned(),
            levels,
        });
    }
    Ok(out)
}

/// The pre-API-36 listing: one decoder and one encoder per known mime, named.
fn probe_by_type(missing: Vec<&'static str>) -> Result<CodecList> {
    let mut codecs = Vec::new();
    let mut media_types = Vec::new();
    for &mime in VIDEO_MIMES {
        let mut mt = MediaType {
            mime: mime.to_owned(),
            decoder: false,
            encoder: false,
        };
        if let Ok(c) = Codec::create_decoder(mime) {
            mt.decoder = true;
            codecs.push(CodecInfo::probed(c.name(), mime, CodecKind::Decoder));
        }
        if let Ok(c) = Codec::create_encoder(mime) {
            mt.encoder = true;
            codecs.push(CodecInfo::probed(c.name(), mime, CodecKind::Encoder));
        }
        if mt.decoder || mt.encoder {
            media_types.push(mt);
        }
    }
    Ok(CodecList {
        source: "createDecoderByType probe",
        missing_symbols: missing,
        media_types,
        codecs,
    })
}

impl CodecInfo {
    fn probed(name: &str, mime: &str, kind: CodecKind) -> CodecInfo {
        CodecInfo {
            name: name.to_owned(),
            mime: mime.to_owned(),
            kind,
            codec_type: CodecType::Unknown,
            is_vendor: !name.starts_with("c2.android.") && !name.starts_with("OMX.google."),
            max_instances: None,
            features: Vec::new(),
            video: None,
            encoder: None,
            profiles: Vec::new(),
        }
    }
}

/// One of the four async callbacks, as delivered to the consumer.
#[derive(Debug)]
pub enum CodecEvent {
    /// `onAsyncInputAvailable`: input buffer `index` may be filled and queued.
    InputAvailable(i32),
    /// `onAsyncOutputAvailable`: output buffer `index` holds `info.size` bytes at
    /// `info.presentation_time_us` with `info.flags`.
    OutputAvailable { index: i32, info: BufferInfo },
    /// `onAsyncFormatChanged`. The NDK builds a fresh `AMediaFormat` for the callback and does
    /// not delete it afterwards (`NdkMediaCodec.cpp:250-269`), so it is owned here.
    FormatChanged(Option<MediaFormat>),
    /// `onAsyncError`. `status` is a `media_status_t`; `AMEDIACODEC_ERROR_RECLAIMED` means the
    /// codec was taken by ResourceManagerService and is dead.
    Error {
        status: i32,
        action_code: i32,
        detail: String,
    },
}

/// The event queue and the generation it is at. A flush voids every buffer index the codec has
/// handed out, so [`Codec::flush_and_restart`] bumps the generation under this lock and
/// [`Shared::take`] hands out only events posted under the current one; an event a callback
/// posted before the flush and this lock saw afterwards is dropped, counted, and never reaches
/// the consumer (`logs/vpu_wp/B5-acceptance.md` D23). The stamp cannot catch a callback the NDK
/// looper had already queued before the flush and delivers after the bump -- that one arrives
/// with the new generation and a dead index, which is why a null `getInputBuffer` /
/// `getOutputBuffer` on it is something a consumer skips, never a session error.
struct EventQueue {
    generation: u64,
    events: VecDeque<(u64, CodecEvent)>,
}

/// What the callbacks write and the consumer reads. The callbacks run on the codec's own NDK
/// looper thread, one at a time under the NDK's lock; each pushes one event and signals the
/// eventfd, and nothing else ("no heavy duty task should be performed on callback thread",
/// `NdkMediaCodec.h:506`).
struct Shared {
    queue: Mutex<EventQueue>,
    /// +1 per event: a device worker polls this next to its virtqueue events.
    event: Event,
    callbacks: AtomicU64,
    /// Events dropped because a flush made their generation old (D23).
    stale: AtomicU64,
    /// Run after every push, besides the eventfd above: a consumer that polls a descriptor of
    /// its own (the virtio-media decoder device, whose session eventfd is the device's) installs
    /// one that bumps it, and needs no thread of its own to forward the wake-up.
    wake: OnceLock<WakeHook>,
}

/// What [`Codec::set_wake_hook`] installs: runs on the codec's callback thread, once per event,
/// after the event is queued. Must only wake something up -- a write to an eventfd -- never do
/// work (`NdkMediaCodec.h:506`).
pub type WakeHook = Box<dyn Fn() + Send + Sync>;

impl Shared {
    fn new() -> Result<Shared> {
        Ok(Shared {
            queue: Mutex::new(EventQueue {
                generation: 0,
                events: VecDeque::new(),
            }),
            event: Event::new().map_err(|e| CodecError::Event(e.to_string()))?,
            callbacks: AtomicU64::new(0),
            stale: AtomicU64::new(0),
            wake: OnceLock::new(),
        })
    }

    /// Queue `ev`, stamped with the generation current at this moment, and wake the consumer.
    fn push(&self, ev: CodecEvent) {
        {
            let mut queue = self.queue.lock().unwrap_or_else(|e| e.into_inner());
            let generation = queue.generation;
            queue.events.push_back((generation, ev));
        }
        self.callbacks.fetch_add(1, Ordering::Relaxed);
        // A signal that fails (fd closed under us) leaves the event in the queue for the next
        // take; nothing to report from a foreign thread.
        let _ = self.event.signal();
        if let Some(wake) = self.wake.get() {
            wake();
        }
    }

    /// Every event of the current generation, oldest first; older ones are dropped and counted.
    fn take(&self) -> Vec<CodecEvent> {
        let mut queue = self.queue.lock().unwrap_or_else(|e| e.into_inner());
        let generation = queue.generation;
        let mut stale = 0;
        let out = queue
            .events
            .drain(..)
            .filter_map(|(posted, ev)| {
                if posted == generation {
                    Some(ev)
                } else {
                    stale += 1;
                    None
                }
            })
            .collect();
        if stale > 0 {
            self.stale.fetch_add(stale, Ordering::Relaxed);
        }
        out
    }

    /// Start a new generation: everything queued so far is void (their indices are), and so is
    /// anything a callback posts before it observes the new generation, since it takes this lock
    /// to post at all.
    fn bump(&self) -> u64 {
        let mut queue = self.queue.lock().unwrap_or_else(|e| e.into_inner());
        queue.generation += 1;
        let dropped = queue.events.len() as u64;
        queue.events.clear();
        if dropped > 0 {
            self.stale.fetch_add(dropped, Ordering::Relaxed);
        }
        queue.generation
    }
}

/// # Safety
///
/// `userdata` is the `Arc<Shared>` raw pointer registered by `Codec::new`, alive until
/// `AMediaCodec_delete` has returned in `Codec::drop`.
unsafe fn shared_from(userdata: *mut c_void) -> &'static Shared {
    &*(userdata as *const Shared)
}

extern "C" fn on_async_input_available(
    _codec: *mut AMediaCodec,
    userdata: *mut c_void,
    index: i32,
) {
    // SAFETY: see shared_from.
    unsafe { shared_from(userdata) }.push(CodecEvent::InputAvailable(index));
}

extern "C" fn on_async_output_available(
    _codec: *mut AMediaCodec,
    userdata: *mut c_void,
    index: i32,
    info: *mut BufferInfo,
) {
    // SAFETY: see shared_from; `info` points at the NDK's stack copy for the duration of the
    // call.
    let info = unsafe { info.as_ref().copied().unwrap_or_default() };
    unsafe { shared_from(userdata) }.push(CodecEvent::OutputAvailable { index, info });
}

extern "C" fn on_async_format_changed(
    _codec: *mut AMediaCodec,
    userdata: *mut c_void,
    format: *mut AMediaFormat,
) {
    // SAFETY: see shared_from; the format is a fresh object the NDK does not delete.
    let format = unsafe { MediaFormat::from_raw(format) };
    unsafe { shared_from(userdata) }.push(CodecEvent::FormatChanged(format));
}

extern "C" fn on_async_error(
    _codec: *mut AMediaCodec,
    userdata: *mut c_void,
    status: i32,
    action_code: i32,
    detail: *const c_char,
) {
    // SAFETY: see shared_from; `detail` is a C string owned by the NDK for the call.
    let detail = unsafe { string_from(detail) }.unwrap_or_default();
    unsafe { shared_from(userdata) }.push(CodecEvent::Error {
        status,
        action_code,
        detail,
    });
}

/// One decoder or encoder session, in asynchronous mode from creation.
///
/// Lifecycle is the NDK's: create -> [`configure`](Codec::configure) -> [`start`](Codec::start)
/// -> buffers -> [`stop`](Codec::stop) -> drop (`AMediaCodec_delete`). There is no `reset`;
/// re-create instead. In async mode `dequeue*` are forbidden and a flush must be followed by
/// `start` ([`flush_and_restart`](Codec::flush_and_restart)).
pub struct Codec {
    ptr: *mut AMediaCodec,
    shared: Arc<Shared>,
    userdata: *const Shared,
    name: String,
}

// SAFETY: AMediaCodec's entry points are documented as callable from several threads at once
// (the in-tree vnc_h264 encoder feeds and drains from two), the queue is behind a mutex and the
// eventfd is a file descriptor; a session may be moved to the worker thread that will drive it.
unsafe impl Send for Codec {}

impl Codec {
    pub fn create_by_name(name: &str) -> Result<Codec> {
        ensure_loaded()?;
        let c = cstring("codec name", name)?;
        // SAFETY: NUL-terminated name; the result is checked.
        let ptr = unsafe { AMediaCodec_createCodecByName(c.as_ptr()) };
        Codec::new(ptr, "AMediaCodec_createCodecByName")
    }

    pub fn create_decoder(mime: &str) -> Result<Codec> {
        ensure_loaded()?;
        let c = cstring("mime", mime)?;
        // SAFETY: as above.
        let ptr = unsafe { AMediaCodec_createDecoderByType(c.as_ptr()) };
        Codec::new(ptr, "AMediaCodec_createDecoderByType")
    }

    pub fn create_encoder(mime: &str) -> Result<Codec> {
        ensure_loaded()?;
        let c = cstring("mime", mime)?;
        // SAFETY: as above.
        let ptr = unsafe { AMediaCodec_createEncoderByType(c.as_ptr()) };
        Codec::new(ptr, "AMediaCodec_createEncoderByType")
    }

    fn new(ptr: *mut AMediaCodec, what: &'static str) -> Result<Codec> {
        if ptr.is_null() {
            return Err(CodecError::Null(what));
        }
        let shared = Arc::new(Shared::new()?);
        let userdata = Arc::into_raw(shared.clone());
        let mut codec = Codec {
            ptr,
            shared,
            userdata,
            name: String::new(),
        };
        // SAFETY: `ptr` is a live codec; the name is released with the codec's own release call.
        unsafe {
            let mut name: *mut c_char = null_mut();
            check("AMediaCodec_getName", AMediaCodec_getName(ptr, &mut name))?;
            codec.name = string_from(name).unwrap_or_default();
            if !name.is_null() {
                AMediaCodec_releaseName(ptr, name);
            }
        }
        // The callback must be installed while the codec is INITIALIZED or CONFIGURED
        // (`MediaCodec.cpp:5414-5423`), so right after creation, before configure.
        let callbacks = AMediaCodecOnAsyncNotifyCallback {
            on_async_input_available: Some(on_async_input_available),
            on_async_output_available: Some(on_async_output_available),
            on_async_format_changed: Some(on_async_format_changed),
            on_async_error: Some(on_async_error),
        };
        // SAFETY: the struct is passed by value as the header declares; `userdata` stays valid
        // until Codec::drop has deleted the codec.
        unsafe {
            check(
                "AMediaCodec_setAsyncNotifyCallback",
                AMediaCodec_setAsyncNotifyCallback(ptr, callbacks, userdata as *mut c_void),
            )?;
        }
        Ok(codec)
    }

    /// The component name (`AMediaCodec_getName`).
    pub fn name(&self) -> &str {
        &self.name
    }

    /// `AMediaCodec_configure` with no surface and no crypto.
    pub fn configure(&mut self, format: &MediaFormat, encoder: bool) -> Result<()> {
        let flags = if encoder { CONFIGURE_FLAG_ENCODE } else { 0 };
        // SAFETY: live codec and format.
        check("AMediaCodec_configure", unsafe {
            AMediaCodec_configure(self.ptr, format.as_ptr(), null_mut(), null_mut(), flags)
        })
    }

    pub fn start(&mut self) -> Result<()> {
        // SAFETY: live codec.
        check("AMediaCodec_start", unsafe { AMediaCodec_start(self.ptr) })
    }

    /// `AMediaCodec_stop` blocks on the codec's looper (`NdkMediaCodec.cpp:679-688`): never
    /// call it from the callback thread.
    pub fn stop(&mut self) -> Result<()> {
        // SAFETY: live codec.
        check("AMediaCodec_stop", unsafe { AMediaCodec_stop(self.ptr) })
    }

    /// `AMediaCodec_flush` alone. In async mode the codec then produces nothing until `start`.
    pub fn flush(&mut self) -> Result<()> {
        // SAFETY: live codec.
        check("AMediaCodec_flush", unsafe { AMediaCodec_flush(self.ptr) })
    }

    /// The async-mode seek: flush, start a new event generation (every buffer index the codec
    /// handed out is void after a flush, so every event queued so far is dropped, and so is one a
    /// callback posts before it sees the new generation), then `start` again so input buffers
    /// are offered afresh (`NdkMediaCodec.h:485-490`). A callback the NDK looper had already
    /// queued before the flush can still be delivered after this returns, stamped with the new
    /// generation and carrying a dead index: `queue_input` / `output_buffer` on it return
    /// [`CodecError::Null`], which a caller must skip -- never treat as a session error
    /// (`logs/vpu_wp/B5-acceptance.md` D23).
    pub fn flush_and_restart(&mut self) -> Result<()> {
        self.flush()?;
        self.shared.bump();
        self.start()
    }

    /// The current event generation: bumped by every [`Self::flush_and_restart`].
    pub fn generation(&self) -> u64 {
        self.shared
            .queue
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .generation
    }

    /// How many events were dropped so far because a flush made their generation old (D23).
    pub fn stale_events(&self) -> u64 {
        self.shared.stale.load(Ordering::Relaxed)
    }

    /// Copy `data` into input buffer `index` and queue it.
    pub fn queue_input(&self, index: i32, data: &[u8], pts_us: u64, flags: u32) -> Result<()> {
        self.queue_input_with(index, pts_us, flags, |buf| {
            if buf.len() < data.len() {
                return Err(format!(
                    "{} bytes into a {} byte buffer",
                    data.len(),
                    buf.len()
                ));
            }
            buf[..data.len()].copy_from_slice(data);
            Ok(data.len())
        })
    }

    /// Let `fill` write input buffer `index` in place (it returns the bytes used), then queue
    /// it. This is how the encoder's padded frame goes in without an intermediate copy.
    pub fn queue_input_with(
        &self,
        index: i32,
        pts_us: u64,
        flags: u32,
        fill: impl FnOnce(&mut [u8]) -> std::result::Result<usize, String>,
    ) -> Result<()> {
        self.queue_input_with_flags(index, pts_us, |buf| fill(buf).map(|used| (used, flags)))
    }

    /// As [`Self::queue_input_with`], but `fill` also returns the buffer flags: for a caller that
    /// can only tell what it queued -- parameter sets alone, so `BUFFER_FLAG_CODEC_CONFIG` -- by
    /// looking at the bytes once they are in the codec's own buffer.
    pub fn queue_input_with_flags(
        &self,
        index: i32,
        pts_us: u64,
        fill: impl FnOnce(&mut [u8]) -> std::result::Result<(usize, u32), String>,
    ) -> Result<()> {
        let mut size = 0usize;
        // SAFETY: live codec; the buffer stays ours until queueInputBuffer.
        let ptr = unsafe { AMediaCodec_getInputBuffer(self.ptr, index as usize, &mut size) };
        if ptr.is_null() {
            return Err(CodecError::Null("AMediaCodec_getInputBuffer"));
        }
        // SAFETY: the NDK reported `size` writable bytes at `ptr`.
        let buf = unsafe { std::slice::from_raw_parts_mut(ptr, size) };
        let (used, flags) = fill(buf).map_err(CodecError::Fill)?;
        if used > size {
            return Err(CodecError::BufferTooSmall(index, size, used));
        }
        // SAFETY: live codec, index owned by us, `used` within the buffer.
        check("AMediaCodec_queueInputBuffer", unsafe {
            AMediaCodec_queueInputBuffer(self.ptr, index as usize, 0, used, pts_us, flags)
        })
    }

    /// Queue an empty buffer carrying `END_OF_STREAM`.
    pub fn queue_eos(&self, index: i32, pts_us: u64) -> Result<()> {
        // SAFETY: live codec, index owned by us.
        check("AMediaCodec_queueInputBuffer", unsafe {
            AMediaCodec_queueInputBuffer(
                self.ptr,
                index as usize,
                0,
                0,
                pts_us,
                BUFFER_FLAG_END_OF_STREAM,
            )
        })
    }

    /// The capacity of an input buffer we hold.
    pub fn input_capacity(&self, index: i32) -> Result<usize> {
        let mut size = 0usize;
        // SAFETY: live codec.
        let ptr = unsafe { AMediaCodec_getInputBuffer(self.ptr, index as usize, &mut size) };
        if ptr.is_null() {
            return Err(CodecError::Null("AMediaCodec_getInputBuffer"));
        }
        Ok(size)
    }

    /// Output buffer `index` as delivered by `OutputAvailable`: the whole readable range the NDK
    /// reports (`abuf->size()`), which since API 35 starts at the data. Valid until
    /// [`release_output`](Codec::release_output), which takes `&mut self` so that the borrow
    /// this returns provably ends before the release (review-m6 R6-13): a use after the release
    /// does not compile.
    pub fn output_buffer(&self, index: i32) -> Result<&[u8]> {
        let mut size = 0usize;
        // SAFETY: live codec; the buffer is ours until released.
        let ptr = unsafe { AMediaCodec_getOutputBuffer(self.ptr, index as usize, &mut size) };
        if ptr.is_null() {
            return Err(CodecError::Null("AMediaCodec_getOutputBuffer"));
        }
        // SAFETY: the NDK reported `size` readable bytes at `ptr`.
        Ok(unsafe { std::slice::from_raw_parts(ptr, size) })
    }

    /// Give output buffer `index` back to the codec. Ends every [`Self::output_buffer`] borrow.
    pub fn release_output(&mut self, index: i32) -> Result<()> {
        // SAFETY: live codec, index owned by us; render=false, there is no surface.
        check("AMediaCodec_releaseOutputBuffer", unsafe {
            AMediaCodec_releaseOutputBuffer(self.ptr, index as usize, false)
        })
    }

    pub fn output_format(&self) -> Result<MediaFormat> {
        // SAFETY: live codec; the returned format is ours to delete.
        unsafe { MediaFormat::from_raw(AMediaCodec_getOutputFormat(self.ptr)) }
            .ok_or(CodecError::Null("AMediaCodec_getOutputFormat"))
    }

    /// After `configure`: what the codec accepted, including the encoder's `stride` and
    /// `slice-height`.
    pub fn input_format(&self) -> Result<MediaFormat> {
        // SAFETY: as above.
        unsafe { MediaFormat::from_raw(AMediaCodec_getInputFormat(self.ptr)) }
            .ok_or(CodecError::Null("AMediaCodec_getInputFormat"))
    }

    /// The per-buffer output format: the one that carries `"image-data"`.
    pub fn buffer_format(&self, index: i32) -> Result<MediaFormat> {
        // SAFETY: as above.
        unsafe { MediaFormat::from_raw(AMediaCodec_getBufferFormat(self.ptr, index as usize)) }
            .ok_or(CodecError::Null("AMediaCodec_getBufferFormat"))
    }

    /// `AMediaCodec_setParameters`: `video-bitrate`, `request-sync`, ...
    pub fn set_parameters(&self, params: &MediaFormat) -> Result<()> {
        // SAFETY: live codec and format.
        check("AMediaCodec_setParameters", unsafe {
            AMediaCodec_setParameters(self.ptr, params.as_ptr())
        })
    }

    /// The eventfd the callbacks bump: +1 per event, readable to reset. A device worker adds it
    /// to its `WaitContext`.
    pub fn poll_event(&self) -> &Event {
        &self.shared.event
    }

    /// Install `hook`, run by every callback after it has queued its event (in addition to the
    /// eventfd). Once per codec: a second call is refused and returns `false`. For a consumer
    /// whose poll descriptor is not [`Self::poll_event`], so that no thread has to sit between
    /// the two.
    pub fn set_wake_hook(&self, hook: WakeHook) -> bool {
        self.shared.wake.set(hook).is_ok()
    }

    /// Everything queued so far under the current generation, without waiting; events an
    /// earlier generation posted are dropped and counted in [`Self::stale_events`].
    pub fn take_events(&self) -> Vec<CodecEvent> {
        self.shared.take()
    }

    /// Everything queued, waiting up to `timeout` for the first event. An empty result is a
    /// timeout.
    pub fn wait_events(&self, timeout: Duration) -> Result<Vec<CodecEvent>> {
        let events = self.take_events();
        if !events.is_empty() {
            return Ok(events);
        }
        self.shared
            .event
            .wait_timeout(timeout)
            .map_err(|e| CodecError::Event(e.to_string()))?;
        Ok(self.take_events())
    }

    /// How many callbacks have fired since creation: the probe's proof that the async path,
    /// not luck, delivered the buffers.
    pub fn callbacks_fired(&self) -> u64 {
        self.shared.callbacks.load(Ordering::Relaxed)
    }
}

impl Drop for Codec {
    fn drop(&mut self) {
        // SAFETY: `AMediaCodec_delete` releases the codec and stops its looper
        // (`NdkMediaCodec.cpp:535-552`); no callback runs after it returns, so the userdata
        // reference can go.
        unsafe {
            AMediaCodec_delete(self.ptr);
            drop(Arc::from_raw(self.userdata));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ffi_struct_layouts() {
        assert_eq!(std::mem::size_of::<BufferInfo>(), 24);
        assert_eq!(
            std::mem::size_of::<AMediaCodecOnAsyncNotifyCallback>(),
            4 * std::mem::size_of::<usize>()
        );
        assert_eq!(
            std::mem::size_of::<AMediaCodecSupportedMediaType>(),
            2 * std::mem::size_of::<usize>()
        );
        assert_eq!(std::mem::size_of::<AIntRange>(), 8);
        assert_eq!(std::mem::size_of::<ADoubleRange>(), 16);
    }

    #[test]
    fn profile_tables_cover_the_video_mimes() {
        for mime in VIDEO_MIMES {
            assert!(!profile_table(mime).is_empty(), "{mime}");
        }
        assert_eq!(profile_table("video/avc").len(), 9);
        assert_eq!(level_table("video/avc").len(), 20);
        assert_eq!(level_table("video/hevc").len(), 26);
        assert_eq!(level_table("video/av01").len(), 24);
        assert_eq!(profile_table("audio/mp4a-latm").len(), 0);
    }

    /// D23: an event queued before a flush must never reach the consumer afterwards, whichever
    /// side it came from, and one queued after it must. The queue is exercised directly: a
    /// `Codec` needs the NDK, the generation logic does not.
    #[test]
    fn events_from_an_older_generation_are_dropped() {
        let shared = Shared::new().unwrap();
        assert_eq!(shared.take().len(), 0);

        shared.push(CodecEvent::InputAvailable(1));
        shared.push(CodecEvent::OutputAvailable {
            index: 2,
            info: BufferInfo::default(),
        });
        assert_eq!(shared.callbacks.load(Ordering::Relaxed), 2);
        // The seek: both are void.
        assert_eq!(shared.bump(), 1);
        assert_eq!(shared.stale.load(Ordering::Relaxed), 2);
        assert!(shared.take().is_empty());

        // Posted under generation 1, taken under generation 1: delivered, in order.
        shared.push(CodecEvent::InputAvailable(3));
        shared.push(CodecEvent::InputAvailable(4));
        let taken = shared.take();
        assert!(
            matches!(
                taken[..],
                [CodecEvent::InputAvailable(3), CodecEvent::InputAvailable(4)]
            ),
            "{taken:?}"
        );

        // An event that was stamped under the old generation but is still in the queue when
        // the consumer looks after a bump is dropped by `take` too, and counted.
        shared.push(CodecEvent::InputAvailable(5));
        {
            let mut queue = shared.queue.lock().unwrap();
            queue.generation += 1;
        }
        shared.push(CodecEvent::InputAvailable(6));
        let taken = shared.take();
        assert!(
            matches!(taken[..], [CodecEvent::InputAvailable(6)]),
            "{taken:?}"
        );
        assert_eq!(shared.stale.load(Ordering::Relaxed), 3);
        assert_eq!(shared.queue.lock().unwrap().generation, 2);
    }

    #[test]
    fn names() {
        assert_eq!(media_status_name(1101), "AMEDIACODEC_ERROR_RECLAIMED");
        assert_eq!(media_status_name(-10002), "AMEDIA_ERROR_UNSUPPORTED");
        assert_eq!(color_format_name(21), "YUV420SemiPlanar (0x15)");
        assert_eq!(color_format_name(0x7F420888), "YUV420Flexible (0x7f420888)");
        assert_eq!(color_format_name(0x123), "0x123");
        assert_eq!(CodecKind::from(2), CodecKind::Encoder);
        assert_eq!(CodecType::from(2), CodecType::HardwareAccelerated);
        assert_eq!(CodecType::from(9), CodecType::Invalid);
    }
}
