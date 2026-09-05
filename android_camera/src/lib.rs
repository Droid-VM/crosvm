// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

//! Bindings for the Android NDK camera APIs: Camera2 (`libcamera2ndk`) for control, and the
//! `AImageReader` half of `libmediandk` for pixels.
//!
//! This is the vendor-neutral entry point -- the same code path on Qualcomm, MediaTek, Exynos and
//! Tensor, because the platform resolves the request to whichever camera HAL the device has. It
//! exists to back a virtio-media capture device, so the shape of the API here is the shape a V4L2
//! capture device needs: enumerate, open at one fixed size, pull frames with their plane strides,
//! and set the handful of controls that have standard `V4L2_CID_*` equivalents.
//!
//! Frames come out raw (`YUV_420_888`, in practice NV12 on the phones we target). Nothing here
//! encodes: a V4L2 capture node hands over pixels, and re-encoding would only cost latency and
//! quality on the way to a guest that would have to undo it.
//!
//! Controls are edits to the one repeating request ([`RequestUpdate`], applied by
//! [`Camera::apply`] in a single `setRepeatingRequest`), and what the camera did with them
//! comes back per frame through the capture-result callbacks ([`ResultListener`], handed a
//! [`CaptureResult`] on the callback thread). Every metadata tag is a number copied from
//! `NdkCameraMetadataTags.h`, with the constant's name and line next to it: the NDK checks a
//! tag's type and refuses a wrong one, but only on the phone.
//!
//! # Privileges
//!
//! `cameraserver` decides what a client may do from the *real uid* of the process that called it:
//! NDK calls carry no package name, so the service resolves one from the uid
//! (`AttributionAndPermissionUtils::resolveAttributionPackage`) and runs the CAMERA permission and
//! AppOps checks against it. uid 0 resolves to no package and is refused; root is also not in
//! `isTrustedCallingUid()`, so it cannot borrow another uid's identity either. Whatever process
//! calls into this module must therefore already be running as the app's uid -- the same
//! arrangement `--virtio-snd ...,uid=N` makes for AAudio, and for the same reason.

use std::ffi::c_void;
use std::ffi::CStr;
use std::ffi::CString;
use std::marker::PhantomData;
use std::os::raw::c_char;
use std::os::raw::c_int;
use std::ptr::null_mut;
use std::sync::atomic::AtomicU64;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::sync::Condvar;
use std::sync::Mutex;
use std::sync::OnceLock;
use std::time::Duration;
use std::time::Instant;

use thiserror::Error;

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
    ACameraManager,
    ACameraDevice,
    ACameraCaptureSession,
    ACaptureRequest,
    ACameraMetadata,
    ACaptureSessionOutput,
    ACaptureSessionOutputContainer,
    ACameraOutputTarget,
    AImageReader,
    AImage,
    ANativeWindow,
    ACameraCaptureFailure,
);

#[repr(C)]
struct ACameraIdList {
    num_cameras: i32,
    camera_ids: *const *const c_char,
}

#[repr(C)]
struct ACameraMetadataConstEntry {
    tag: u32,
    entry_type: u8,
    count: u32,
    data: *const c_void,
}

#[repr(C)]
struct ACameraDeviceStateCallbacks {
    context: *mut c_void,
    on_disconnected: Option<extern "C" fn(*mut c_void, *mut ACameraDevice)>,
    on_error: Option<extern "C" fn(*mut c_void, *mut ACameraDevice, i32)>,
    /// Added to the struct after the original three fields. Declared so our allocation is at
    /// least as large as what a current `libcamera2ndk` reads, and left null because we never
    /// open a camera in shared mode. An older library simply never looks here.
    on_client_shared_access_priority_changed: Option<extern "C" fn()>,
}

#[repr(C)]
struct ACameraCaptureSessionStateCallbacks {
    context: *mut c_void,
    on_closed: Option<extern "C" fn(*mut c_void, *mut ACameraCaptureSession)>,
    on_ready: Option<extern "C" fn(*mut c_void, *mut ACameraCaptureSession)>,
    on_active: Option<extern "C" fn(*mut c_void, *mut ACameraCaptureSession)>,
}

#[repr(C)]
struct AImageReaderImageListener {
    context: *mut c_void,
    on_image_available: Option<extern "C" fn(*mut c_void, *mut AImageReader)>,
}

/// `ACameraCaptureSession_captureCallbacks` (`NdkCameraCaptureSession.h:285-419`): eight
/// fields, `context` then seven callbacks, in the header's order. Handed to every
/// `setRepeatingRequest` and `capture`, and so kept for the life of the session.
#[repr(C)]
struct ACameraCaptureSessionCaptureCallbacks {
    context: *mut c_void,
    on_capture_started:
        Option<extern "C" fn(*mut c_void, *mut ACameraCaptureSession, *const ACaptureRequest, i64)>,
    on_capture_progressed: Option<
        extern "C" fn(
            *mut c_void,
            *mut ACameraCaptureSession,
            *mut ACaptureRequest,
            *const ACameraMetadata,
        ),
    >,
    on_capture_completed: Option<
        extern "C" fn(
            *mut c_void,
            *mut ACameraCaptureSession,
            *mut ACaptureRequest,
            *const ACameraMetadata,
        ),
    >,
    on_capture_failed: Option<
        extern "C" fn(
            *mut c_void,
            *mut ACameraCaptureSession,
            *mut ACaptureRequest,
            *mut ACameraCaptureFailure,
        ),
    >,
    on_capture_sequence_completed:
        Option<extern "C" fn(*mut c_void, *mut ACameraCaptureSession, c_int, i64)>,
    on_capture_sequence_aborted:
        Option<extern "C" fn(*mut c_void, *mut ACameraCaptureSession, c_int)>,
    on_capture_buffer_lost: Option<
        extern "C" fn(
            *mut c_void,
            *mut ACameraCaptureSession,
            *mut ACaptureRequest,
            *mut ANativeWindow,
            i64,
        ),
    >,
}

/// Declares every NDK entry point we use, then resolves them all at run time.
///
/// Resolved with `dlopen` rather than linked. `libcamera2ndk` and `libmediandk` both reach
/// `libgui`, and building `libgui` needs host tools and bionic pieces that a crosvm-only AOSP
/// checkout does not carry -- a link-time dependency would make the tree unbuildable for anyone
/// without them, for a device that is optional. Resolving late also turns "this platform has no
/// camera NDK" into an error a caller can report rather than a binary that will not load.
///
/// Each declaration expands into three things: a field in `NdkApi`, the `dlsym` that fills it, and
/// a free function of the same name, so call sites read exactly as they would against a real
/// `extern "C"` block. Entry points in the `optional` group may be missing from the platform's
/// libraries: they become `Option` fields, `None` when absent, and get no free function.
macro_rules! ndk_api {
    (
        $( fn $name:ident ( $($arg:ident : $argty:ty),* $(,)? ) $(-> $ret:ty)?; )*
        optional {
            $( fn $oname:ident ( $($oarg:ident : $oargty:ty),* $(,)? ) $(-> $oret:ty)?; )*
        }
    ) => {
        #[allow(non_snake_case)]
        struct NdkApi {
            $( $name: unsafe extern "C" fn($($argty),*) $(-> $ret)?, )*
            $( $oname: Option<unsafe extern "C" fn($($oargty),*) $(-> $oret)?>, )*
        }

        impl NdkApi {
            fn load() -> std::result::Result<NdkApi, CameraError> {
                let handles = [
                    open_library("libcamera2ndk.so\0")?,
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
                };
                // cameraserver drives a capture session by calling back into this process:
                // results, buffer-ready notifications, device state. Those are kernel binder
                // transactions, and a process with no binder threads has nobody to receive them,
                // so the session configures, the HAL opens, streaming ops start -- and then not a
                // single frame ever arrives. An app never has to do this because the framework
                // starts the pool at process start; a bare native process does.
                //
                // SAFETY: both pointers were just resolved out of libbinder_ndk, and starting the
                // pool more than once is harmless.
                unsafe {
                    (api.ABinderProcess_setThreadPoolMaxThreadCount)(BINDER_THREAD_POOL_SIZE);
                    (api.ABinderProcess_startThreadPool)();
                }
                Ok(api)
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

/// The libraries stay open for the life of the process: the resolved pointers outlive any scope
/// that could close them, and nothing here is unloadable anyway.
fn open_library(name: &'static str) -> std::result::Result<*mut c_void, CameraError> {
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
        return Err(CameraError::LibraryLoad(
            name.trim_end_matches('\0'),
            reason,
        ));
    }
    Ok(handle)
}

/// # Safety
///
/// `T` must be the function pointer type actually exported under `name`.
unsafe fn symbol<T: Copy>(
    handles: &[*mut c_void],
    name: &'static str,
) -> std::result::Result<T, CameraError> {
    for &handle in handles {
        let ptr = libc::dlsym(handle, name.as_ptr() as *const c_char);
        if !ptr.is_null() {
            return Ok(std::mem::transmute_copy(&ptr));
        }
    }
    Err(CameraError::MissingSymbol(name.trim_end_matches('\0')))
}

static NDK: OnceLock<std::result::Result<NdkApi, CameraError>> = OnceLock::new();

/// Load the NDK if it has not been loaded, and report why if it cannot be. Every public entry
/// point calls this before the first FFI call.
fn ensure_loaded() -> Result<()> {
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
        .expect("android_camera: NDK used before ensure_loaded() succeeded")
}

ndk_api! {
    fn ACameraManager_create() -> *mut ACameraManager;
    fn ACameraManager_delete(manager: *mut ACameraManager);
    fn ACameraManager_getCameraIdList(
        manager: *mut ACameraManager,
        list: *mut *mut ACameraIdList,
    ) -> i32;
    fn ACameraManager_deleteCameraIdList(list: *mut ACameraIdList);
    fn ACameraManager_getCameraCharacteristics(
        manager: *mut ACameraManager,
        camera_id: *const c_char,
        characteristics: *mut *mut ACameraMetadata,
    ) -> i32;
    fn ACameraManager_openCamera(
        manager: *mut ACameraManager,
        camera_id: *const c_char,
        callbacks: *mut ACameraDeviceStateCallbacks,
        device: *mut *mut ACameraDevice,
    ) -> i32;
    fn ACameraMetadata_getConstEntry(
        metadata: *const ACameraMetadata,
        tag: u32,
        entry: *mut ACameraMetadataConstEntry,
    ) -> i32;
    fn ACameraMetadata_free(metadata: *mut ACameraMetadata);
    fn ACameraDevice_close(device: *mut ACameraDevice) -> i32;
    fn ACameraDevice_createCaptureRequest(
        device: *const ACameraDevice,
        template_id: i32,
        request: *mut *mut ACaptureRequest,
    ) -> i32;
    fn ACameraDevice_createCaptureSession(
        device: *mut ACameraDevice,
        outputs: *const ACaptureSessionOutputContainer,
        callbacks: *const ACameraCaptureSessionStateCallbacks,
        session: *mut *mut ACameraCaptureSession,
    ) -> i32;
    fn ACaptureSessionOutput_create(
        window: *mut ANativeWindow,
        output: *mut *mut ACaptureSessionOutput,
    ) -> i32;
    fn ACaptureSessionOutput_free(output: *mut ACaptureSessionOutput);
    fn ACaptureSessionOutputContainer_create(
        container: *mut *mut ACaptureSessionOutputContainer,
    ) -> i32;
    fn ACaptureSessionOutputContainer_free(container: *mut ACaptureSessionOutputContainer);
    fn ACaptureSessionOutputContainer_add(
        container: *mut ACaptureSessionOutputContainer,
        output: *const ACaptureSessionOutput,
    ) -> i32;
    fn ACameraOutputTarget_create(
        window: *mut ANativeWindow,
        target: *mut *mut ACameraOutputTarget,
    ) -> i32;
    fn ACameraOutputTarget_free(target: *mut ACameraOutputTarget);
    fn ACaptureRequest_free(request: *mut ACaptureRequest);
    fn ACaptureRequest_getConstEntry(
        request: *const ACaptureRequest,
        tag: u32,
        entry: *mut ACameraMetadataConstEntry,
    ) -> i32;
    fn ACaptureRequest_addTarget(
        request: *mut ACaptureRequest,
        target: *const ACameraOutputTarget,
    ) -> i32;
    fn ACaptureRequest_setEntry_u8(
        request: *mut ACaptureRequest,
        tag: u32,
        count: u32,
        data: *const u8,
    ) -> i32;
    fn ACaptureRequest_setEntry_i32(
        request: *mut ACaptureRequest,
        tag: u32,
        count: u32,
        data: *const i32,
    ) -> i32;
    fn ACaptureRequest_setEntry_float(
        request: *mut ACaptureRequest,
        tag: u32,
        count: u32,
        data: *const f32,
    ) -> i32;
    fn ACaptureRequest_setEntry_i64(
        request: *mut ACaptureRequest,
        tag: u32,
        count: u32,
        data: *const i64,
    ) -> i32;
    fn ACameraCaptureSession_setRepeatingRequest(
        session: *mut ACameraCaptureSession,
        callbacks: *mut ACameraCaptureSessionCaptureCallbacks,
        num_requests: i32,
        requests: *mut *mut ACaptureRequest,
        capture_sequence_id: *mut i32,
    ) -> i32;
    fn ACameraCaptureSession_capture(
        session: *mut ACameraCaptureSession,
        callbacks: *mut ACameraCaptureSessionCaptureCallbacks,
        num_requests: i32,
        requests: *mut *mut ACaptureRequest,
        capture_sequence_id: *mut i32,
    ) -> i32;
    fn ACameraCaptureSession_stopRepeating(session: *mut ACameraCaptureSession) -> i32;
    fn ACameraCaptureSession_close(session: *mut ACameraCaptureSession);
    // libmediandk: AImageReader and AImage
    fn AImageReader_new(
        width: i32,
        height: i32,
        format: i32,
        max_images: i32,
        reader: *mut *mut AImageReader,
    ) -> i32;
    fn AImageReader_delete(reader: *mut AImageReader);
    fn AImageReader_getWindow(reader: *mut AImageReader, window: *mut *mut ANativeWindow) -> i32;
    fn AImageReader_setImageListener(
        reader: *mut AImageReader,
        listener: *mut AImageReaderImageListener,
    ) -> i32;
    fn AImageReader_acquireNextImage(reader: *mut AImageReader, image: *mut *mut AImage) -> i32;
    fn AImage_delete(image: *mut AImage);
    fn AImage_getWidth(image: *const AImage, width: *mut i32) -> i32;
    fn AImage_getHeight(image: *const AImage, height: *mut i32) -> i32;
    fn AImage_getFormat(image: *const AImage, format: *mut i32) -> i32;
    fn AImage_getTimestamp(image: *const AImage, timestamp_ns: *mut i64) -> i32;
    fn AImage_getNumberOfPlanes(image: *const AImage, num_planes: *mut i32) -> i32;
    fn AImage_getPlanePixelStride(image: *const AImage, plane: i32, stride: *mut i32) -> i32;
    fn AImage_getPlaneRowStride(image: *const AImage, plane: i32, stride: *mut i32) -> i32;
    fn AImage_getPlaneData(
        image: *const AImage,
        plane: i32,
        data: *mut *mut u8,
        len: *mut i32,
    ) -> i32;

    // libbinder_ndk: only to get this process onto the binder bus, see NdkApi::load.
    fn ABinderProcess_setThreadPoolMaxThreadCount(num_threads: u32) -> bool;
    fn ABinderProcess_startThreadPool();

    optional {
        // API 24 like the rest, but the one call a consumer can live without: a reader whose
        // platform lacks it falls back to acquiring in order (`Camera::next_frame_latest`).
        fn AImageReader_acquireLatestImage(reader: *mut AImageReader, image: *mut *mut AImage) -> i32;
        // API 28: what a one-shot capture (`Camera::trigger_af`) is built from. Without it the
        // trigger is refused rather than the whole NDK.
        fn ACaptureRequest_copy(src: *const ACaptureRequest) -> *mut ACaptureRequest;
    }
}

/// Enough threads for the camera callbacks (results, buffer notifications, device state) without
/// making the pool a resource of its own.
const BINDER_THREAD_POOL_SIZE: u32 = 4;

const ACAMERA_OK: i32 = 0;
const AMEDIA_OK: i32 = 0;
const AMEDIA_IMGREADER_NO_BUFFER_AVAILABLE: i32 = -30001;

/// `AIMAGE_FORMAT_YUV_420_888`: the flexible planar YUV every device supports. The concrete layout
/// (NV12, NV21 or I420) is not part of the format -- it is read back per frame from the plane
/// strides, which is what [`Frame::layout`] does.
pub const AIMAGE_FORMAT_YUV_420_888: i32 = 0x23;

/// Metadata tag sections are the section index shifted into the high half of the tag, and each tag
/// is an offset within its section. Spelled out rather than pasted as hex so each constant below
/// reads the same way it does in `NdkCameraMetadataTags.h`.
const SECTION_CONTROL: u32 = 1 << 16;
const SECTION_FLASH: u32 = 4 << 16;
const SECTION_FLASH_INFO: u32 = 5 << 16;
const SECTION_LENS: u32 = 8 << 16;
const SECTION_SCALER: u32 = 13 << 16;
const SECTION_SENSOR: u32 = 14 << 16;
const SECTION_INFO: u32 = 21 << 16;
const SECTION_REQUEST: u32 = 12 << 16;
const SECTION_LOGICAL_MULTI_CAMERA: u32 = 26 << 16;

// Every value below is copied from `NdkCameraMetadataTags.h` (the tree's
// `frameworks/av/camera/ndk/include/camera/`), never recalled: the header constant and its line
// are next to each one. The NDK validates a tag's type on the phone and answers a wrong one with
// `ERROR_INVALID_PARAMETER`, so a slip here is invisible until then.
const SECTION_SENSOR_INFO: u32 = 15 << 16;

// ACAMERA_CONTROL_AE_ANTIBANDING_MODE = ACAMERA_CONTROL_START (:457), byte
const TAG_CONTROL_AE_ANTIBANDING_MODE: u32 = SECTION_CONTROL;
// ACAMERA_CONTROL_AE_EXPOSURE_COMPENSATION = ACAMERA_CONTROL_START + 1 (:493), int32
const TAG_CONTROL_AE_EXPOSURE_COMPENSATION: u32 = SECTION_CONTROL + 1;
// ACAMERA_CONTROL_AE_MODE = ACAMERA_CONTROL_START + 3 (:603), byte
const TAG_CONTROL_AE_MODE: u32 = SECTION_CONTROL + 3;
// ACAMERA_CONTROL_AE_REGIONS = ACAMERA_CONTROL_START + 4 (:696), int32[5*area_count]
const TAG_CONTROL_AE_REGIONS: u32 = SECTION_CONTROL + 4;
// ACAMERA_CONTROL_AE_TARGET_FPS_RANGE = ACAMERA_CONTROL_START + 5 (:731), int32[2]
const TAG_CONTROL_AE_TARGET_FPS_RANGE: u32 = SECTION_CONTROL + 5;
// ACAMERA_CONTROL_AF_MODE = ACAMERA_CONTROL_START + 7 (:831), byte
const TAG_CONTROL_AF_MODE: u32 = SECTION_CONTROL + 7;
// ACAMERA_CONTROL_AF_REGIONS = ACAMERA_CONTROL_START + 8 (:925), int32[5*area_count]
const TAG_CONTROL_AF_REGIONS: u32 = SECTION_CONTROL + 8;
// ACAMERA_CONTROL_AF_TRIGGER = ACAMERA_CONTROL_START + 9 (:959), byte
const TAG_CONTROL_AF_TRIGGER: u32 = SECTION_CONTROL + 9;
// ACAMERA_CONTROL_AWB_MODE = ACAMERA_CONTROL_START + 11 (:1037), byte
const TAG_CONTROL_AWB_MODE: u32 = SECTION_CONTROL + 11;
// ACAMERA_CONTROL_AWB_REGIONS = ACAMERA_CONTROL_START + 12 (:1131), int32[5*area_count]
const TAG_CONTROL_AWB_REGIONS: u32 = SECTION_CONTROL + 12;
// ACAMERA_CONTROL_EFFECT_MODE = ACAMERA_CONTROL_START + 14 (:1182), byte
const TAG_CONTROL_EFFECT_MODE: u32 = SECTION_CONTROL + 14;
// ACAMERA_CONTROL_MODE = ACAMERA_CONTROL_START + 15 (:1214), byte
const TAG_CONTROL_MODE: u32 = SECTION_CONTROL + 15;
// ACAMERA_CONTROL_SCENE_MODE = ACAMERA_CONTROL_START + 16 (:1243), byte
const TAG_CONTROL_SCENE_MODE: u32 = SECTION_CONTROL + 16;
// ACAMERA_CONTROL_VIDEO_STABILIZATION_MODE = ACAMERA_CONTROL_START + 17 (:1300), byte
const TAG_CONTROL_VIDEO_STABILIZATION_MODE: u32 = SECTION_CONTROL + 17;
// ACAMERA_CONTROL_AE_AVAILABLE_ANTIBANDING_MODES = ACAMERA_CONTROL_START + 18 (:1323), byte[n]
const TAG_CONTROL_AE_AVAILABLE_ANTIBANDING_MODES: u32 = SECTION_CONTROL + 18;
// ACAMERA_CONTROL_AE_AVAILABLE_MODES = ACAMERA_CONTROL_START + 19 (:1353), byte[n]
const TAG_CONTROL_AE_AVAILABLE_MODES: u32 = SECTION_CONTROL + 19;
// ACAMERA_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES = ACAMERA_CONTROL_START + 20 (:1403), int32[2*n]
const TAG_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES: u32 = SECTION_CONTROL + 20;
// ACAMERA_CONTROL_AE_COMPENSATION_RANGE = ACAMERA_CONTROL_START + 21 (:1421), int32[2]
const TAG_CONTROL_AE_COMPENSATION_RANGE: u32 = SECTION_CONTROL + 21;
// ACAMERA_CONTROL_AE_COMPENSATION_STEP = ACAMERA_CONTROL_START + 22 (:1442), rational
const TAG_CONTROL_AE_COMPENSATION_STEP: u32 = SECTION_CONTROL + 22;
// ACAMERA_CONTROL_AF_AVAILABLE_MODES = ACAMERA_CONTROL_START + 23 (:1471), byte[n]
const TAG_CONTROL_AF_AVAILABLE_MODES: u32 = SECTION_CONTROL + 23;
// ACAMERA_CONTROL_AVAILABLE_EFFECTS = ACAMERA_CONTROL_START + 24 (:1498), byte[n]
const TAG_CONTROL_AVAILABLE_EFFECTS: u32 = SECTION_CONTROL + 24;
// ACAMERA_CONTROL_AVAILABLE_SCENE_MODES = ACAMERA_CONTROL_START + 25 (:1525), byte[n]
const TAG_CONTROL_AVAILABLE_SCENE_MODES: u32 = SECTION_CONTROL + 25;
// ACAMERA_CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES = ACAMERA_CONTROL_START + 26 (:1542), byte[n]
const TAG_CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES: u32 = SECTION_CONTROL + 26;
// ACAMERA_CONTROL_AWB_AVAILABLE_MODES = ACAMERA_CONTROL_START + 27 (:1571), byte[n]
const TAG_CONTROL_AWB_AVAILABLE_MODES: u32 = SECTION_CONTROL + 27;
// ACAMERA_CONTROL_MAX_REGIONS = ACAMERA_CONTROL_START + 28 (:1592), int32[3]
const TAG_CONTROL_MAX_REGIONS: u32 = SECTION_CONTROL + 28;
// ACAMERA_CONTROL_AE_STATE = ACAMERA_CONTROL_START + 31 (:1666), byte, result only
const TAG_CONTROL_AE_STATE: u32 = SECTION_CONTROL + 31;
// ACAMERA_CONTROL_AF_STATE = ACAMERA_CONTROL_START + 32 (:1768), byte, result only
const TAG_CONTROL_AF_STATE: u32 = SECTION_CONTROL + 32;
// ACAMERA_CONTROL_ZOOM_RATIO_RANGE = ACAMERA_CONTROL_START + 46 (:2109), float[2]
const TAG_CONTROL_ZOOM_RATIO_RANGE: u32 = SECTION_CONTROL + 46;
// ACAMERA_CONTROL_ZOOM_RATIO = ACAMERA_CONTROL_START + 47 (:2208), float
const TAG_CONTROL_ZOOM_RATIO: u32 = SECTION_CONTROL + 47;
// ACAMERA_FLASH_MODE = ACAMERA_FLASH_START + 2 (:2587), byte
const TAG_FLASH_MODE: u32 = SECTION_FLASH + 2;
// ACAMERA_FLASH_INFO_AVAILABLE = ACAMERA_FLASH_INFO_START (:2773), byte
const TAG_FLASH_INFO_AVAILABLE: u32 = SECTION_FLASH_INFO;
// ACAMERA_LENS_FACING = ACAMERA_LENS_START + 5, byte
const TAG_LENS_FACING: u32 = SECTION_LENS + 5;
// ACAMERA_SCALER_AVAILABLE_MAX_DIGITAL_ZOOM = ACAMERA_SCALER_START + 4, float
const TAG_SCALER_AVAILABLE_MAX_DIGITAL_ZOOM: u32 = SECTION_SCALER + 4;
// ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS = ACAMERA_SCALER_START + 10 (:4377), int32[n*4]
const TAG_SCALER_AVAILABLE_STREAM_CONFIGURATIONS: u32 = SECTION_SCALER + 10;
// ACAMERA_SCALER_AVAILABLE_MIN_FRAME_DURATIONS = ACAMERA_SCALER_START + 11 (:4402), int64[4*n]
const TAG_SCALER_AVAILABLE_MIN_FRAME_DURATIONS: u32 = SECTION_SCALER + 11;
// ACAMERA_SENSOR_EXPOSURE_TIME = ACAMERA_SENSOR_START (:5000), int64
const TAG_SENSOR_EXPOSURE_TIME: u32 = SECTION_SENSOR;
// ACAMERA_SENSOR_SENSITIVITY = ACAMERA_SENSOR_START + 2 (:5124), int32
const TAG_SENSOR_SENSITIVITY: u32 = SECTION_SENSOR + 2;
// ACAMERA_SENSOR_ORIENTATION = ACAMERA_SENSOR_START + 14, int32
const TAG_SENSOR_ORIENTATION: u32 = SECTION_SENSOR + 14;
// ACAMERA_SENSOR_INFO_ACTIVE_ARRAY_SIZE = ACAMERA_SENSOR_INFO_START (:5919), int32[4]
const TAG_SENSOR_INFO_ACTIVE_ARRAY_SIZE: u32 = SECTION_SENSOR_INFO;
// ACAMERA_SENSOR_INFO_SENSITIVITY_RANGE = ACAMERA_SENSOR_INFO_START + 1 (:5937), int32[2]
const TAG_SENSOR_INFO_SENSITIVITY_RANGE: u32 = SECTION_SENSOR_INFO + 1;
// ACAMERA_SENSOR_INFO_EXPOSURE_TIME_RANGE = ACAMERA_SENSOR_INFO_START + 3 (:5969), int64[2]
const TAG_SENSOR_INFO_EXPOSURE_TIME_RANGE: u32 = SECTION_SENSOR_INFO + 3;
// ACAMERA_INFO_SUPPORTED_HARDWARE_LEVEL = ACAMERA_INFO_START, byte
const TAG_INFO_SUPPORTED_HARDWARE_LEVEL: u32 = SECTION_INFO;
// ACAMERA_REQUEST_AVAILABLE_CAPABILITIES = ACAMERA_REQUEST_START + 12 (:3950), byte[n]
const TAG_REQUEST_AVAILABLE_CAPABILITIES: u32 = SECTION_REQUEST + 12;
// ACAMERA_SCALER_AVAILABLE_STREAM_USE_CASES = ACAMERA_SCALER_START + 26 (:4916), int64[n]
const TAG_SCALER_AVAILABLE_STREAM_USE_CASES: u32 = SECTION_SCALER + 26;
// ACAMERA_LOGICAL_MULTI_CAMERA_PHYSICAL_IDS = ACAMERA_LOGICAL_MULTI_CAMERA_START (:7756), byte[n]
const TAG_LOGICAL_MULTI_CAMERA_PHYSICAL_IDS: u32 = SECTION_LOGICAL_MULTI_CAMERA;
// ACAMERA_LOGICAL_MULTI_CAMERA_ACTIVE_PHYSICAL_ID = ACAMERA_LOGICAL_MULTI_CAMERA_START + 2
// (:7806), byte (a NUL-terminated string), result only
const TAG_LOGICAL_MULTI_CAMERA_ACTIVE_PHYSICAL_ID: u32 = SECTION_LOGICAL_MULTI_CAMERA + 2;

/// `ACAMERA_REQUEST_AVAILABLE_CAPABILITIES_LOGICAL_MULTI_CAMERA`: the camera is one lens group
/// presented as one device, with the platform choosing which sensor serves a given zoom ratio.
const CAPABILITY_LOGICAL_MULTI_CAMERA: u8 = 11;

const TEMPLATE_RECORD: i32 = 3;

const TYPE_BYTE: u8 = 0;
const TYPE_INT32: u8 = 1;
const TYPE_FLOAT: u8 = 2;
const TYPE_INT64: u8 = 3;
/// `ACAMERA_TYPE_RATIONAL` (`NdkCameraMetadata.h:76`): `ACameraMetadata_rational`, two `int32`.
const TYPE_RATIONAL: u8 = 5;

#[derive(Error, Debug, Clone)]
pub enum CameraError {
    #[error("could not load {0}: {1}")]
    LibraryLoad(&'static str, String),
    #[error("{0} is missing from the camera NDK")]
    MissingSymbol(&'static str),
    #[error("ACameraManager_create returned null")]
    ManagerCreate,
    #[error("{0} failed: {1} ({2})")]
    Ndk(&'static str, i32, &'static str),
    #[error("camera id {0:?} not found")]
    NoSuchCamera(String),
    #[error("camera {0:?} does not offer {1}x{2} in YUV_420_888")]
    UnsupportedSize(String, i32, i32),
    #[error("camera id contained an interior NUL")]
    BadCameraId,
    #[error("setting request entry {0:#x} failed: {1} ({2})")]
    Entry(u32, i32, &'static str),
}

type Result<T> = std::result::Result<T, CameraError>;

/// The camera status codes worth naming: everything else is reported by number. `PERMISSION_DENIED`
/// is the one that tells us the uid story above went wrong, so it must not be anonymous.
fn camera_status_name(status: i32) -> &'static str {
    match status {
        0 => "ACAMERA_OK",
        -10000 => "ERROR_UNKNOWN",
        -10001 => "ERROR_INVALID_PARAMETER",
        -10002 => "ERROR_CAMERA_DISCONNECTED",
        -10003 => "ERROR_NOT_ENOUGH_MEMORY",
        -10004 => "ERROR_METADATA_NOT_FOUND",
        -10005 => "ERROR_CAMERA_DEVICE",
        -10006 => "ERROR_CAMERA_SERVICE",
        -10007 => "ERROR_SESSION_CLOSED",
        -10008 => "ERROR_INVALID_OPERATION",
        -10009 => "ERROR_STREAM_CONFIGURE_FAIL",
        -10010 => "ERROR_CAMERA_IN_USE",
        -10011 => "ERROR_MAX_CAMERA_IN_USE",
        -10012 => "ERROR_CAMERA_DISABLED",
        -10013 => "ERROR_PERMISSION_DENIED",
        -10014 => "ERROR_UNSUPPORTED_OPERATION",
        _ => "?",
    }
}

fn check(what: &'static str, status: i32) -> Result<()> {
    if status == ACAMERA_OK {
        Ok(())
    } else {
        Err(CameraError::Ndk(what, status, camera_status_name(status)))
    }
}

fn check_media(what: &'static str, status: i32) -> Result<()> {
    if status == AMEDIA_OK {
        Ok(())
    } else {
        Err(CameraError::Ndk(what, status, "AMEDIA"))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LensFacing {
    Front,
    Back,
    External,
    Unknown(u8),
}

impl From<u8> for LensFacing {
    fn from(v: u8) -> Self {
        match v {
            0 => LensFacing::Front,
            1 => LensFacing::Back,
            2 => LensFacing::External,
            other => LensFacing::Unknown(other),
        }
    }
}

/// `ACAMERA_FLASH_MODE_*` (`NdkCameraMetadataTags.h:9887-9898`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashMode {
    Off = 0,
    Single = 1,
    Torch = 2,
}

/// `ACAMERA_CONTROL_AF_MODE_*` (`NdkCameraMetadataTags.h:8795-8878`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AfMode {
    Off = 0,
    Auto = 1,
    Macro = 2,
    ContinuousVideo = 3,
    ContinuousPicture = 4,
    Edof = 5,
}

impl AfMode {
    pub fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::Off,
            1 => Self::Auto,
            2 => Self::Macro,
            3 => Self::ContinuousVideo,
            4 => Self::ContinuousPicture,
            5 => Self::Edof,
            _ => return None,
        })
    }
}

/// `ACAMERA_CONTROL_AE_MODE_*` (`NdkCameraMetadataTags.h:8628-8758`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AeMode {
    Off = 0,
    On = 1,
    OnAutoFlash = 2,
    OnAlwaysFlash = 3,
    OnAutoFlashRedeye = 4,
    OnExternalFlash = 5,
    OnLowLightBoostBrightnessPriority = 6,
}

impl AeMode {
    pub fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::Off,
            1 => Self::On,
            2 => Self::OnAutoFlash,
            3 => Self::OnAlwaysFlash,
            4 => Self::OnAutoFlashRedeye,
            5 => Self::OnExternalFlash,
            6 => Self::OnLowLightBoostBrightnessPriority,
            _ => return None,
        })
    }
}

/// `ACAMERA_CONTROL_AF_TRIGGER_*` (`NdkCameraMetadataTags.h:8887-8898`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AfTrigger {
    Idle = 0,
    Start = 1,
    Cancel = 2,
}

/// `ACAMERA_CONTROL_AWB_MODE_*` (`NdkCameraMetadataTags.h:8932-9062`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AwbMode {
    Off = 0,
    Auto = 1,
    Incandescent = 2,
    Fluorescent = 3,
    WarmFluorescent = 4,
    Daylight = 5,
    CloudyDaylight = 6,
    Twilight = 7,
    Shade = 8,
}

impl AwbMode {
    pub fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::Off,
            1 => Self::Auto,
            2 => Self::Incandescent,
            3 => Self::Fluorescent,
            4 => Self::WarmFluorescent,
            5 => Self::Daylight,
            6 => Self::CloudyDaylight,
            7 => Self::Twilight,
            8 => Self::Shade,
            _ => return None,
        })
    }
}

/// `ACAMERA_CONTROL_AE_ANTIBANDING_MODE_*` (`NdkCameraMetadataTags.h:8552-8573`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AntibandingMode {
    Off = 0,
    Hz50 = 1,
    Hz60 = 2,
    Auto = 3,
}

impl AntibandingMode {
    pub fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::Off,
            1 => Self::Hz50,
            2 => Self::Hz60,
            3 => Self::Auto,
            _ => return None,
        })
    }
}

/// `ACAMERA_CONTROL_EFFECT_MODE_*` (`NdkCameraMetadataTags.h:9139-9189`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EffectMode {
    Off = 0,
    Mono = 1,
    Negative = 2,
    Solarize = 3,
    Sepia = 4,
    Posterize = 5,
    Whiteboard = 6,
    Blackboard = 7,
    Aqua = 8,
}

impl EffectMode {
    pub fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::Off,
            1 => Self::Mono,
            2 => Self::Negative,
            3 => Self::Solarize,
            4 => Self::Sepia,
            5 => Self::Posterize,
            6 => Self::Whiteboard,
            7 => Self::Blackboard,
            8 => Self::Aqua,
            _ => return None,
        })
    }
}

/// `ACAMERA_CONTROL_MODE_*` (`NdkCameraMetadataTags.h:9211-9258`): a scene mode applies only
/// under `UseSceneMode`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlMode {
    Off = 0,
    Auto = 1,
    UseSceneMode = 2,
    OffKeepState = 3,
    UseExtendedSceneMode = 4,
}

impl ControlMode {
    pub fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::Off,
            1 => Self::Auto,
            2 => Self::UseSceneMode,
            3 => Self::OffKeepState,
            4 => Self::UseExtendedSceneMode,
            _ => return None,
        })
    }
}

/// `ACAMERA_CONTROL_SCENE_MODE_*` (`NdkCameraMetadataTags.h:9267-9417`; 17 is unassigned).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SceneMode {
    Disabled = 0,
    FacePriority = 1,
    Action = 2,
    Portrait = 3,
    Landscape = 4,
    Night = 5,
    NightPortrait = 6,
    Theatre = 7,
    Beach = 8,
    Snow = 9,
    Sunset = 10,
    Steadyphoto = 11,
    Fireworks = 12,
    Sports = 13,
    Party = 14,
    Candlelight = 15,
    Barcode = 16,
    Hdr = 18,
}

impl SceneMode {
    pub fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::Disabled,
            1 => Self::FacePriority,
            2 => Self::Action,
            3 => Self::Portrait,
            4 => Self::Landscape,
            5 => Self::Night,
            6 => Self::NightPortrait,
            7 => Self::Theatre,
            8 => Self::Beach,
            9 => Self::Snow,
            10 => Self::Sunset,
            11 => Self::Steadyphoto,
            12 => Self::Fireworks,
            13 => Self::Sports,
            14 => Self::Party,
            15 => Self::Candlelight,
            16 => Self::Barcode,
            18 => Self::Hdr,
            _ => return None,
        })
    }
}

/// `ACAMERA_CONTROL_VIDEO_STABILIZATION_MODE_*` (`NdkCameraMetadataTags.h:9426-9442`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VideoStabilizationMode {
    Off = 0,
    On = 1,
    PreviewStabilization = 2,
}

impl VideoStabilizationMode {
    pub fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::Off,
            1 => Self::On,
            2 => Self::PreviewStabilization,
            _ => return None,
        })
    }
}

/// `ACAMERA_CONTROL_AE_STATE_*` (`NdkCameraMetadataTags.h:9454-9497`), a capture result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AeState {
    Inactive = 0,
    Searching = 1,
    Converged = 2,
    Locked = 3,
    FlashRequired = 4,
    Precapture = 5,
}

impl AeState {
    pub fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::Inactive,
            1 => Self::Searching,
            2 => Self::Converged,
            3 => Self::Locked,
            4 => Self::FlashRequired,
            5 => Self::Precapture,
            _ => return None,
        })
    }
}

/// `ACAMERA_CONTROL_AF_STATE_*` (`NdkCameraMetadataTags.h:9511-9574`), a capture result.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AfState {
    Inactive = 0,
    PassiveScan = 1,
    PassiveFocused = 2,
    ActiveScan = 3,
    FocusedLocked = 4,
    NotFocusedLocked = 5,
    PassiveUnfocused = 6,
}

impl AfState {
    pub fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::Inactive,
            1 => Self::PassiveScan,
            2 => Self::PassiveFocused,
            3 => Self::ActiveScan,
            4 => Self::FocusedLocked,
            5 => Self::NotFocusedLocked,
            6 => Self::PassiveUnfocused,
            _ => return None,
        })
    }
}

/// A rectangle in the sensor's active-array coordinates (`ACAMERA_SENSOR_INFO_ACTIVE_ARRAY_SIZE`
/// is one: `(left, top, width, height)`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Rect {
    pub left: i32,
    pub top: i32,
    pub width: i32,
    pub height: i32,
}

/// One metering / focus / white-balance region as Camera2 takes it: `(xmin, ymin, xmax, ymax,
/// weight)` in active-array coordinates, `xmin`/`ymin` inclusive, `xmax`/`ymax` exclusive,
/// weight `0..=1000` (`ACAMERA_CONTROL_AE_REGIONS`, `NdkCameraMetadataTags.h:640-697`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Area {
    pub xmin: i32,
    pub ymin: i32,
    pub xmax: i32,
    pub ymax: i32,
    pub weight: i32,
}

/// What a camera can do, in the terms a V4L2 capture device has to answer `ENUM_FMT`,
/// `ENUM_FRAMESIZES` and `QUERYCTRL` with.
#[derive(Debug, Clone)]
pub struct CameraInfo {
    pub id: String,
    pub facing: LensFacing,
    /// Degrees the sensor image must be rotated clockwise to be upright. A V4L2 client has no way
    /// to be told this, so a capture device either rotates on the host or ignores it.
    pub orientation: i32,
    pub hardware_level: u8,
    /// `CONTROL_ZOOM_RATIO_RANGE`, the modern zoom control. Absent below API 30, where zoom is
    /// `SCALER_CROP_REGION` and bounded by `max_digital_zoom` instead.
    pub zoom_ratio_range: Option<(f32, f32)>,
    pub max_digital_zoom: Option<f32>,
    pub flash_available: bool,
    /// `CONTROL_MAX_REGIONS` as (AE, AWB, AF). Non-zero entries are the tap-to-focus and
    /// tap-to-meter rectangles, which have no standard V4L2 control at all.
    pub max_regions: (i32, i32, i32),
    /// Output sizes for `YUV_420_888`, largest first.
    pub yuv_sizes: Vec<(i32, i32)>,
    /// `SCALER_AVAILABLE_MIN_FRAME_DURATIONS` for `YUV_420_888`, as `((width, height), ns)`: the
    /// shortest frame duration each size sustains, which caps the frame rate a V4L2
    /// `ENUM_FRAMEINTERVALS` may offer there. A size missing here made no promise.
    pub yuv_min_frame_durations: Vec<((i32, i32), i64)>,
    /// `CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES` as `(min, max)` pairs: what
    /// [`Camera::set_fps_range`] accepts. A V4L2 `S_PARM` carries one rate, which the capture
    /// device maps to the widest of these whose maximum is that rate.
    pub fps_ranges: Vec<(i32, i32)>,
    /// `REQUEST_AVAILABLE_CAPABILITIES`, raw. Kept as the list rather than as the one flag we
    /// care about so that "this camera is not a logical multi-camera" cannot be confused with
    /// "the capability list did not read", which is the same empty answer.
    pub capabilities: Vec<u8>,
    /// Bytes `PHYSICAL_IDS` actually returned, for the same reason.
    pub physical_ids_raw_len: usize,
    /// The lens ids behind a logical camera, as `LOGICAL_MULTI_CAMERA_PHYSICAL_IDS` reports them.
    pub physical_ids: Vec<String>,
    /// `SCALER_AVAILABLE_STREAM_USE_CASES`: which purposes a stream may declare, which is how a
    /// camera offers preview, recording and stills at once without each guessing the others.
    pub stream_use_cases: Vec<i64>,
    /// `CONTROL_AF_AVAILABLE_MODES`, raw ([`AfMode`] values).
    pub af_modes: Vec<u8>,
    /// `CONTROL_AE_AVAILABLE_MODES`, raw ([`AeMode`] values); `Off` is what makes manual
    /// exposure possible.
    pub ae_modes: Vec<u8>,
    /// `CONTROL_AWB_AVAILABLE_MODES`, raw ([`AwbMode`] values).
    pub awb_modes: Vec<u8>,
    /// `CONTROL_AE_AVAILABLE_ANTIBANDING_MODES`, raw ([`AntibandingMode`] values).
    pub antibanding_modes: Vec<u8>,
    /// `CONTROL_AVAILABLE_EFFECTS`, raw ([`EffectMode`] values).
    pub effects: Vec<u8>,
    /// `CONTROL_AVAILABLE_SCENE_MODES`, raw ([`SceneMode`] values).
    pub scene_modes: Vec<u8>,
    /// `CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES`, raw ([`VideoStabilizationMode`] values).
    pub video_stabilization_modes: Vec<u8>,
    /// `SENSOR_INFO_EXPOSURE_TIME_RANGE` in nanoseconds; what `SENSOR_EXPOSURE_TIME` accepts
    /// under `AeMode::Off`.
    pub exposure_time_range_ns: Option<(i64, i64)>,
    /// `SENSOR_INFO_SENSITIVITY_RANGE` (ISO); what `SENSOR_SENSITIVITY` accepts.
    pub sensitivity_range: Option<(i32, i32)>,
    /// `CONTROL_AE_COMPENSATION_RANGE`, in steps of [`Self::ae_compensation_step`].
    pub ae_compensation_range: Option<(i32, i32)>,
    /// `CONTROL_AE_COMPENSATION_STEP` as `(numerator, denominator)` EV.
    pub ae_compensation_step: Option<(i32, i32)>,
    /// `SENSOR_INFO_ACTIVE_ARRAY_SIZE`: the coordinate system of the regions.
    pub active_array: Option<Rect>,
}

/// `PHYSICAL_IDS` is a byte blob of NUL-terminated strings rather than a list, so it has to be
/// cut apart by hand.
fn split_nul_strings(bytes: &[u8]) -> Vec<String> {
    bytes
        .split(|b| *b == 0)
        .filter(|part| !part.is_empty())
        .map(|part| String::from_utf8_lossy(part).into_owned())
        .collect()
}

struct Manager(*mut ACameraManager);

impl Manager {
    fn new() -> Result<Manager> {
        // SAFETY: no arguments, and the result is checked for null before use.
        let ptr = unsafe { ACameraManager_create() };
        if ptr.is_null() {
            return Err(CameraError::ManagerCreate);
        }
        Ok(Manager(ptr))
    }

    fn characteristics(&self, id: &CStr) -> Result<Characteristics> {
        let mut ptr = null_mut();
        // SAFETY: self.0 is a live manager, id outlives the call, and ptr is written only on OK.
        check("ACameraManager_getCameraCharacteristics", unsafe {
            ACameraManager_getCameraCharacteristics(self.0, id.as_ptr(), &mut ptr)
        })?;
        Ok(Characteristics(ptr))
    }
}

impl Drop for Manager {
    fn drop(&mut self) {
        // SAFETY: self.0 came from ACameraManager_create and is deleted exactly once.
        unsafe { ACameraManager_delete(self.0) };
    }
}

struct Characteristics(*mut ACameraMetadata);

impl CameraInfo {
    /// One device presenting several lenses, with the platform picking which serves a zoom ratio.
    pub fn is_logical_multi_camera(&self) -> bool {
        self.capabilities.contains(&CAPABILITY_LOGICAL_MULTI_CAMERA)
    }
}

impl Characteristics {
    fn entry(&self, tag: u32) -> Option<ACameraMetadataConstEntry> {
        let mut entry = ACameraMetadataConstEntry {
            tag: 0,
            entry_type: 0,
            count: 0,
            data: std::ptr::null(),
        };
        // SAFETY: self.0 is live for the borrow, and entry is fully initialised above.
        let status = unsafe { ACameraMetadata_getConstEntry(self.0, tag, &mut entry) };
        (status == ACAMERA_OK && entry.count > 0).then_some(entry)
    }

    fn u8s(&self, tag: u32) -> Vec<u8> {
        match self.entry(tag) {
            Some(e) if e.entry_type == TYPE_BYTE => {
                // SAFETY: the entry reports type BYTE and count elements owned by the metadata,
                // which outlives the copy made here.
                unsafe { std::slice::from_raw_parts(e.data as *const u8, e.count as usize) }
                    .to_vec()
            }
            _ => Vec::new(),
        }
    }

    fn i32s(&self, tag: u32) -> Vec<i32> {
        match self.entry(tag) {
            Some(e) if e.entry_type == TYPE_INT32 => {
                // SAFETY: as above, with the type checked to be INT32.
                unsafe { std::slice::from_raw_parts(e.data as *const i32, e.count as usize) }
                    .to_vec()
            }
            _ => Vec::new(),
        }
    }

    fn i64s(&self, tag: u32) -> Vec<i64> {
        match self.entry(tag) {
            Some(e) if e.entry_type == TYPE_INT64 => {
                // SAFETY: as above, with the type checked to be INT64.
                unsafe { std::slice::from_raw_parts(e.data as *const i64, e.count as usize) }
                    .to_vec()
            }
            _ => Vec::new(),
        }
    }

    fn f32s(&self, tag: u32) -> Vec<f32> {
        match self.entry(tag) {
            Some(e) if e.entry_type == TYPE_FLOAT => {
                // SAFETY: as above, with the type checked to be FLOAT.
                unsafe { std::slice::from_raw_parts(e.data as *const f32, e.count as usize) }
                    .to_vec()
            }
            _ => Vec::new(),
        }
    }

    /// `(numerator, denominator)` pairs of a RATIONAL entry (`ACameraMetadata_rational`, two
    /// `int32_t`, `NdkCameraMetadata.h:84-87`).
    fn rationals(&self, tag: u32) -> Vec<(i32, i32)> {
        match self.entry(tag) {
            Some(e) if e.entry_type == TYPE_RATIONAL => {
                // SAFETY: as above, with the type checked to be RATIONAL, each element two i32.
                unsafe { std::slice::from_raw_parts(e.data as *const i32, e.count as usize * 2) }
                    .chunks_exact(2)
                    .map(|p| (p[0], p[1]))
                    .collect()
            }
            _ => Vec::new(),
        }
    }

    fn pair_i32(&self, tag: u32) -> Option<(i32, i32)> {
        match self.i32s(tag).as_slice() {
            [a, b, ..] => Some((*a, *b)),
            _ => None,
        }
    }

    fn pair_i64(&self, tag: u32) -> Option<(i64, i64)> {
        match self.i64s(tag).as_slice() {
            [a, b, ..] => Some((*a, *b)),
            _ => None,
        }
    }

    fn rect(&self, tag: u32) -> Option<Rect> {
        match self.i32s(tag).as_slice() {
            [left, top, width, height, ..] => Some(Rect {
                left: *left,
                top: *top,
                width: *width,
                height: *height,
            }),
            _ => None,
        }
    }

    /// `SCALER_AVAILABLE_MIN_FRAME_DURATIONS` is a flat int64 array of
    /// (format, width, height, duration_ns) quads.
    fn min_frame_durations(&self, format: i32) -> Vec<((i32, i32), i64)> {
        self.i64s(TAG_SCALER_AVAILABLE_MIN_FRAME_DURATIONS)
            .chunks_exact(4)
            .filter(|q| q[0] == format as i64)
            .map(|q| ((q[1] as i32, q[2] as i32), q[3]))
            .collect()
    }

    /// `CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES` is a flat int32 array of (min, max) pairs.
    fn fps_ranges(&self) -> Vec<(i32, i32)> {
        self.i32s(TAG_CONTROL_AE_AVAILABLE_TARGET_FPS_RANGES)
            .chunks_exact(2)
            .map(|p| (p[0], p[1]))
            .collect()
    }

    /// `SCALER_AVAILABLE_STREAM_CONFIGURATIONS` is a flat int32 array of
    /// (format, width, height, input) quads; input==1 entries are reprocessing inputs, not outputs.
    fn output_sizes(&self, format: i32) -> Vec<(i32, i32)> {
        let mut sizes: Vec<(i32, i32)> = self
            .i32s(TAG_SCALER_AVAILABLE_STREAM_CONFIGURATIONS)
            .chunks_exact(4)
            .filter(|q| q[0] == format && q[3] == 0)
            .map(|q| (q[1], q[2]))
            .collect();
        sizes.sort_unstable_by_key(|(w, h)| std::cmp::Reverse((*w as i64) * (*h as i64)));
        sizes
    }

    fn info(&self, id: String) -> CameraInfo {
        let zoom = self.f32s(TAG_CONTROL_ZOOM_RATIO_RANGE);
        let regions = self.i32s(TAG_CONTROL_MAX_REGIONS);
        CameraInfo {
            id,
            facing: self
                .u8s(TAG_LENS_FACING)
                .first()
                .copied()
                .unwrap_or(255)
                .into(),
            orientation: self
                .i32s(TAG_SENSOR_ORIENTATION)
                .first()
                .copied()
                .unwrap_or(0),
            hardware_level: self
                .u8s(TAG_INFO_SUPPORTED_HARDWARE_LEVEL)
                .first()
                .copied()
                .unwrap_or(255),
            zoom_ratio_range: (zoom.len() == 2).then(|| (zoom[0], zoom[1])),
            max_digital_zoom: self
                .f32s(TAG_SCALER_AVAILABLE_MAX_DIGITAL_ZOOM)
                .first()
                .copied(),
            flash_available: self
                .u8s(TAG_FLASH_INFO_AVAILABLE)
                .first()
                .copied()
                .unwrap_or(0)
                != 0,
            max_regions: match regions.len() {
                3 => (regions[0], regions[1], regions[2]),
                _ => (0, 0, 0),
            },
            yuv_sizes: self.output_sizes(AIMAGE_FORMAT_YUV_420_888),
            yuv_min_frame_durations: self.min_frame_durations(AIMAGE_FORMAT_YUV_420_888),
            fps_ranges: self.fps_ranges(),
            capabilities: self.u8s(TAG_REQUEST_AVAILABLE_CAPABILITIES),
            physical_ids_raw_len: self.u8s(TAG_LOGICAL_MULTI_CAMERA_PHYSICAL_IDS).len(),
            physical_ids: split_nul_strings(&self.u8s(TAG_LOGICAL_MULTI_CAMERA_PHYSICAL_IDS)),
            stream_use_cases: self.i64s(TAG_SCALER_AVAILABLE_STREAM_USE_CASES),
            af_modes: self.u8s(TAG_CONTROL_AF_AVAILABLE_MODES),
            ae_modes: self.u8s(TAG_CONTROL_AE_AVAILABLE_MODES),
            awb_modes: self.u8s(TAG_CONTROL_AWB_AVAILABLE_MODES),
            antibanding_modes: self.u8s(TAG_CONTROL_AE_AVAILABLE_ANTIBANDING_MODES),
            effects: self.u8s(TAG_CONTROL_AVAILABLE_EFFECTS),
            scene_modes: self.u8s(TAG_CONTROL_AVAILABLE_SCENE_MODES),
            video_stabilization_modes: self.u8s(TAG_CONTROL_AVAILABLE_VIDEO_STABILIZATION_MODES),
            exposure_time_range_ns: self.pair_i64(TAG_SENSOR_INFO_EXPOSURE_TIME_RANGE),
            sensitivity_range: self.pair_i32(TAG_SENSOR_INFO_SENSITIVITY_RANGE),
            ae_compensation_range: self.pair_i32(TAG_CONTROL_AE_COMPENSATION_RANGE),
            ae_compensation_step: self
                .rationals(TAG_CONTROL_AE_COMPENSATION_STEP)
                .first()
                .copied(),
            active_array: self.rect(TAG_SENSOR_INFO_ACTIVE_ARRAY_SIZE),
        }
    }
}

impl Drop for Characteristics {
    fn drop(&mut self) {
        // SAFETY: self.0 came from getCameraCharacteristics and is freed exactly once.
        unsafe { ACameraMetadata_free(self.0) };
    }
}

/// Enumerate every camera the platform will hand this uid, with the capabilities a virtio-media
/// capture device would have to advertise.
pub fn list_cameras() -> Result<Vec<CameraInfo>> {
    ensure_loaded()?;
    let manager = Manager::new()?;
    let mut list: *mut ACameraIdList = null_mut();
    // SAFETY: manager is live, and list is written only when the call succeeds.
    check("ACameraManager_getCameraIdList", unsafe {
        ACameraManager_getCameraIdList(manager.0, &mut list)
    })?;

    let mut out = Vec::new();
    // SAFETY: the list is non-null after an OK status, and its two fields describe the array.
    let ids =
        unsafe { std::slice::from_raw_parts((*list).camera_ids, (*list).num_cameras as usize) };
    for &id in ids {
        // SAFETY: the NDK guarantees each entry is a NUL-terminated string owned by the list.
        let cstr = unsafe { CStr::from_ptr(id) };
        match manager.characteristics(cstr) {
            Ok(c) => out.push(c.info(cstr.to_string_lossy().into_owned())),
            // A camera the platform lists but will not describe is not fatal to enumeration: it is
            // usually one this uid may not touch. Skip it rather than losing the whole list.
            Err(_) => continue,
        }
    }
    // SAFETY: the list came from getCameraIdList and is deleted exactly once, after the last read.
    unsafe { ACameraManager_deleteCameraIdList(list) };
    Ok(out)
}

/// Counts `onImageAvailable` callbacks and lets a waiter block until the next one.
///
/// The count is also the answer to "did the listener ever fire": a capture that only ever produces
/// frames through the polling path in [`Camera::next_frame`] is working by accident.
struct FrameSignal {
    delivered: AtomicU64,
    lock: Mutex<()>,
    cond: Condvar,
}

impl FrameSignal {
    fn signal(&self) {
        self.delivered.fetch_add(1, Ordering::Release);
        let _guard = self.lock.lock().unwrap_or_else(|e| e.into_inner());
        self.cond.notify_all();
    }

    fn wait(&self, timeout: Duration) {
        let guard = self.lock.lock().unwrap_or_else(|e| e.into_inner());
        let _unused = self.cond.wait_timeout(guard, timeout);
    }
}

extern "C" fn on_image_available(context: *mut c_void, _reader: *mut AImageReader) {
    // SAFETY: context is the pointer Arc::into_raw produced in Camera::open. The reader is the
    // only caller and is deleted before that Arc is reclaimed in Camera::drop, so the referent is
    // alive for the whole time this callback can run.
    let signal = unsafe { &*(context as *const FrameSignal) };
    signal.signal();
}

/// What the platform tells an open camera about itself, on a binder thread of its own.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeviceState {
    /// The camera was taken away: another client with higher priority opened it, or a policy
    /// closed it. Frames stop; the handle only remains to be closed.
    Disconnected,
    /// A fatal device error (`ERROR_CAMERA_DEVICE`, `ERROR_CAMERA_SERVICE`, ...): the camera is
    /// unusable until closed and reopened.
    Error(i32),
}

/// Called with every [`DeviceState`] the platform reports, from a binder thread, so it must be
/// `Send + Sync` and must not block; a capture loop typically pushes the state onto a channel and
/// wakes whoever waits for frames.
pub type StateListener = Box<dyn Fn(DeviceState) + Send + Sync>;

/// The context the device-state callbacks are given: the pointer the NDK hands back is to this.
struct StateContext {
    listener: Option<StateListener>,
}

impl StateContext {
    fn notify(&self, state: DeviceState) {
        if let Some(listener) = &self.listener {
            listener(state);
        }
    }
}

extern "C" fn on_device_disconnected(context: *mut c_void, _device: *mut ACameraDevice) {
    base::error!("android_camera: camera device disconnected");
    if !context.is_null() {
        // SAFETY: context is the `StateContext` `Camera::open_with` registered; the box that
        // holds it outlives the device (it is dropped after `ACameraDevice_close`).
        unsafe { &*(context as *const StateContext) }.notify(DeviceState::Disconnected);
    }
}

extern "C" fn on_device_error(context: *mut c_void, _device: *mut ACameraDevice, error: i32) {
    base::error!("android_camera: camera device error {}", error);
    if !context.is_null() {
        // SAFETY: as above.
        unsafe { &*(context as *const StateContext) }.notify(DeviceState::Error(error));
    }
}

extern "C" fn on_session_closed(_context: *mut c_void, _session: *mut ACameraCaptureSession) {}
extern "C" fn on_session_ready(_context: *mut c_void, _session: *mut ACameraCaptureSession) {}
extern "C" fn on_session_active(_context: *mut c_void, _session: *mut ACameraCaptureSession) {}

/// One entry of a capture request, typed as the NDK types it.
#[derive(Debug, Clone, PartialEq)]
enum Entry {
    U8(u32, Vec<u8>),
    I32(u32, Vec<i32>),
    I64(u32, Vec<i64>),
    F32(u32, Vec<f32>),
}

/// Edits to the repeating request, applied together by [`Camera::apply`] in one
/// `setRepeatingRequest` -- a V4L2 `S_EXT_CTRLS` is one of these, however many controls it
/// carries. Every setter is a tag from `NdkCameraMetadataTags.h`; the AE/flash/exposure
/// arithmetic (which mode makes which entry apply) is the caller's.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct RequestUpdate {
    entries: Vec<Entry>,
}

impl RequestUpdate {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// `CONTROL_ZOOM_RATIO`: on a logical multi-camera also what makes the platform switch
    /// between the ultra-wide, main and tele lenses.
    pub fn zoom_ratio(mut self, ratio: f32) -> Self {
        self.entries
            .push(Entry::F32(TAG_CONTROL_ZOOM_RATIO, vec![ratio]));
        self
    }

    /// `CONTROL_AE_MODE`. Under `Off` the exposure time and sensitivity below apply; under any
    /// `On*Flash` mode the 3A routine owns the LED and overrides `flash_mode`.
    pub fn ae_mode(mut self, mode: AeMode) -> Self {
        self.entries
            .push(Entry::U8(TAG_CONTROL_AE_MODE, vec![mode as u8]));
        self
    }

    /// `FLASH_MODE`.
    pub fn flash_mode(mut self, mode: FlashMode) -> Self {
        self.entries
            .push(Entry::U8(TAG_FLASH_MODE, vec![mode as u8]));
        self
    }

    /// `CONTROL_AF_MODE`.
    pub fn af_mode(mut self, mode: AfMode) -> Self {
        self.entries
            .push(Entry::U8(TAG_CONTROL_AF_MODE, vec![mode as u8]));
        self
    }

    /// `CONTROL_AE_TARGET_FPS_RANGE`.
    pub fn fps_range(mut self, min: i32, max: i32) -> Self {
        self.entries
            .push(Entry::I32(TAG_CONTROL_AE_TARGET_FPS_RANGE, vec![min, max]));
        self
    }

    /// `SENSOR_EXPOSURE_TIME`, nanoseconds; applies under `AeMode::Off`.
    pub fn exposure_time_ns(mut self, ns: i64) -> Self {
        self.entries
            .push(Entry::I64(TAG_SENSOR_EXPOSURE_TIME, vec![ns]));
        self
    }

    /// `SENSOR_SENSITIVITY` (ISO); applies under `AeMode::Off`.
    pub fn sensitivity(mut self, iso: i32) -> Self {
        self.entries
            .push(Entry::I32(TAG_SENSOR_SENSITIVITY, vec![iso]));
        self
    }

    /// `CONTROL_AE_EXPOSURE_COMPENSATION`, in steps of the camera's
    /// `AE_COMPENSATION_STEP`.
    pub fn ae_compensation(mut self, steps: i32) -> Self {
        self.entries.push(Entry::I32(
            TAG_CONTROL_AE_EXPOSURE_COMPENSATION,
            vec![steps],
        ));
        self
    }

    /// `CONTROL_AWB_MODE`.
    pub fn awb_mode(mut self, mode: AwbMode) -> Self {
        self.entries
            .push(Entry::U8(TAG_CONTROL_AWB_MODE, vec![mode as u8]));
        self
    }

    /// `CONTROL_AE_ANTIBANDING_MODE`.
    pub fn antibanding_mode(mut self, mode: AntibandingMode) -> Self {
        self.entries
            .push(Entry::U8(TAG_CONTROL_AE_ANTIBANDING_MODE, vec![mode as u8]));
        self
    }

    /// `CONTROL_EFFECT_MODE`.
    pub fn effect_mode(mut self, mode: EffectMode) -> Self {
        self.entries
            .push(Entry::U8(TAG_CONTROL_EFFECT_MODE, vec![mode as u8]));
        self
    }

    /// `CONTROL_MODE`: `UseSceneMode` for a scene mode to apply, `Auto` otherwise.
    pub fn control_mode(mut self, mode: ControlMode) -> Self {
        self.entries
            .push(Entry::U8(TAG_CONTROL_MODE, vec![mode as u8]));
        self
    }

    /// `CONTROL_SCENE_MODE`.
    pub fn scene_mode(mut self, mode: SceneMode) -> Self {
        self.entries
            .push(Entry::U8(TAG_CONTROL_SCENE_MODE, vec![mode as u8]));
        self
    }

    /// `CONTROL_VIDEO_STABILIZATION_MODE`.
    pub fn video_stabilization(mut self, mode: VideoStabilizationMode) -> Self {
        self.entries.push(Entry::U8(
            TAG_CONTROL_VIDEO_STABILIZATION_MODE,
            vec![mode as u8],
        ));
        self
    }

    fn regions(mut self, tag: u32, areas: &[Area]) -> Self {
        let words = areas
            .iter()
            .flat_map(|a| [a.xmin, a.ymin, a.xmax, a.ymax, a.weight])
            .collect();
        self.entries.push(Entry::I32(tag, words));
        self
    }

    /// `CONTROL_AE_REGIONS`; an empty slice removes the entry (the camera's own metering).
    pub fn ae_regions(self, areas: &[Area]) -> Self {
        self.regions(TAG_CONTROL_AE_REGIONS, areas)
    }

    /// `CONTROL_AF_REGIONS`; an empty slice removes the entry.
    pub fn af_regions(self, areas: &[Area]) -> Self {
        self.regions(TAG_CONTROL_AF_REGIONS, areas)
    }

    /// `CONTROL_AWB_REGIONS`; an empty slice removes the entry.
    pub fn awb_regions(self, areas: &[Area]) -> Self {
        self.regions(TAG_CONTROL_AWB_REGIONS, areas)
    }

    /// Write every entry into `request`. An empty vector removes the tag ("set count to 0 and
    /// data to NULL", `NdkCaptureRequest.h`).
    ///
    /// # Safety
    ///
    /// `request` must be a live `ACaptureRequest`.
    unsafe fn write_into(&self, request: *mut ACaptureRequest) -> Result<()> {
        fn ptr_or_null<T>(v: &[T]) -> *const T {
            if v.is_empty() {
                std::ptr::null()
            } else {
                v.as_ptr()
            }
        }
        for entry in &self.entries {
            // SAFETY: each pointer addresses `count` elements of the type the NDK expects for
            // the call, or is null with a count of 0.
            let (tag, status) = match entry {
                Entry::U8(tag, v) => (
                    *tag,
                    ACaptureRequest_setEntry_u8(request, *tag, v.len() as u32, ptr_or_null(v)),
                ),
                Entry::I32(tag, v) => (
                    *tag,
                    ACaptureRequest_setEntry_i32(request, *tag, v.len() as u32, ptr_or_null(v)),
                ),
                Entry::I64(tag, v) => (
                    *tag,
                    ACaptureRequest_setEntry_i64(request, *tag, v.len() as u32, ptr_or_null(v)),
                ),
                Entry::F32(tag, v) => (
                    *tag,
                    ACaptureRequest_setEntry_float(request, *tag, v.len() as u32, ptr_or_null(v)),
                ),
            };
            if status != ACAMERA_OK {
                return Err(CameraError::Entry(tag, status, camera_status_name(status)));
            }
        }
        Ok(())
    }
}

/// The metadata of one completed capture, readable only inside the callback that delivers it
/// ("Do not access this pointer after this callback returns", `NdkCameraCaptureSession.h:215`),
/// which is why a [`ResultListener`] is handed a borrow.
pub struct CaptureResult<'a> {
    metadata: *const ACameraMetadata,
    /// The request this result was taken with, as the framework handed it to the callback
    /// ("the capture request that generated this capture result",
    /// `NdkCameraCaptureSession.h:212`); null only if the framework passed none. See
    /// [`CaptureResult::requested_awb_mode`] for what it is for.
    request: *const ACaptureRequest,
    _callback: PhantomData<&'a ()>,
}

impl CaptureResult<'_> {
    fn entry(&self, tag: u32) -> Option<ACameraMetadataConstEntry> {
        let mut entry = ACameraMetadataConstEntry {
            tag: 0,
            entry_type: 0,
            count: 0,
            data: std::ptr::null(),
        };
        // SAFETY: the metadata is live for the callback this value is confined to.
        let status = unsafe { ACameraMetadata_getConstEntry(self.metadata, tag, &mut entry) };
        (status == ACAMERA_OK && entry.count > 0).then_some(entry)
    }

    /// The same tag, read out of the *request* this result came from rather than out of the
    /// result.
    fn request_entry(&self, tag: u32) -> Option<ACameraMetadataConstEntry> {
        if self.request.is_null() {
            return None;
        }
        let mut entry = ACameraMetadataConstEntry {
            tag: 0,
            entry_type: 0,
            count: 0,
            data: std::ptr::null(),
        };
        // SAFETY: the request is live for the callback this value is confined to, and the
        // entry it fills in is owned by the framework ("Do not attempt to free it",
        // `NdkCaptureRequest.h:143-144`).
        let status = unsafe { ACaptureRequest_getConstEntry(self.request, tag, &mut entry) };
        (status == ACAMERA_OK && entry.count > 0).then_some(entry)
    }

    fn requested_u8(&self, tag: u32) -> Option<u8> {
        let e = self
            .request_entry(tag)
            .filter(|e| e.entry_type == TYPE_BYTE)?;
        // SAFETY: type BYTE, count >= 1, owned by the request for the callback.
        Some(unsafe { *(e.data as *const u8) })
    }

    fn u8(&self, tag: u32) -> Option<u8> {
        let e = self.entry(tag).filter(|e| e.entry_type == TYPE_BYTE)?;
        // SAFETY: type BYTE, count >= 1, owned by the metadata for the callback.
        Some(unsafe { *(e.data as *const u8) })
    }

    fn i32(&self, tag: u32) -> Option<i32> {
        let e = self.entry(tag).filter(|e| e.entry_type == TYPE_INT32)?;
        // SAFETY: as above, type INT32.
        Some(unsafe { *(e.data as *const i32) })
    }

    fn i64(&self, tag: u32) -> Option<i64> {
        let e = self.entry(tag).filter(|e| e.entry_type == TYPE_INT64)?;
        // SAFETY: as above, type INT64.
        Some(unsafe { *(e.data as *const i64) })
    }

    fn f32(&self, tag: u32) -> Option<f32> {
        let e = self.entry(tag).filter(|e| e.entry_type == TYPE_FLOAT)?;
        // SAFETY: as above, type FLOAT.
        Some(unsafe { *(e.data as *const f32) })
    }

    /// `CONTROL_AF_STATE`; `None` when absent or a value this crate does not name.
    pub fn af_state(&self) -> Option<AfState> {
        AfState::from_u8(self.u8(TAG_CONTROL_AF_STATE)?)
    }

    /// `CONTROL_AE_STATE`.
    pub fn ae_state(&self) -> Option<AeState> {
        AeState::from_u8(self.u8(TAG_CONTROL_AE_STATE)?)
    }

    /// `LOGICAL_MULTI_CAMERA_ACTIVE_PHYSICAL_ID`: the lens this frame came from, as the id
    /// string `physical_ids` lists.
    pub fn active_physical_id(&self) -> Option<String> {
        let e = self
            .entry(TAG_LOGICAL_MULTI_CAMERA_ACTIVE_PHYSICAL_ID)
            .filter(|e| e.entry_type == TYPE_BYTE)?;
        // SAFETY: type BYTE, `count` bytes owned by the metadata for the callback.
        let bytes = unsafe { std::slice::from_raw_parts(e.data as *const u8, e.count as usize) };
        let end = bytes.iter().position(|&b| b == 0).unwrap_or(bytes.len());
        Some(String::from_utf8_lossy(&bytes[..end]).into_owned())
    }

    /// `CONTROL_ZOOM_RATIO` as applied.
    pub fn zoom_ratio(&self) -> Option<f32> {
        self.f32(TAG_CONTROL_ZOOM_RATIO)
    }

    /// `SENSOR_EXPOSURE_TIME` as applied, nanoseconds.
    pub fn exposure_time_ns(&self) -> Option<i64> {
        self.i64(TAG_SENSOR_EXPOSURE_TIME)
    }

    /// `SENSOR_SENSITIVITY` as applied.
    pub fn sensitivity(&self) -> Option<i32> {
        self.i32(TAG_SENSOR_SENSITIVITY)
    }

    /// `CONTROL_AWB_MODE` as applied. These four are request keys the camera device echoes in
    /// its result -- "the values used for this capture" -- so reading them back is how a caller
    /// tells a request entry the HAL *took* from one it silently dropped, which is the only way
    /// to settle D36 (`AUTO_N_PRESET_WHITE_BALANCE`, `COLORFX` and `SCENE_MODE` set and read
    /// back through V4L2 without changing a pixel).
    pub fn awb_mode(&self) -> Option<AwbMode> {
        AwbMode::from_u8(self.u8(TAG_CONTROL_AWB_MODE)?)
    }

    /// `CONTROL_EFFECT_MODE` as applied.
    pub fn effect_mode(&self) -> Option<EffectMode> {
        EffectMode::from_u8(self.u8(TAG_CONTROL_EFFECT_MODE)?)
    }

    /// `CONTROL_SCENE_MODE` as applied. It has no effect unless [`Self::control_mode`] is
    /// `UseSceneMode`, so both are worth reading together.
    pub fn scene_mode(&self) -> Option<SceneMode> {
        SceneMode::from_u8(self.u8(TAG_CONTROL_SCENE_MODE)?)
    }

    /// `CONTROL_MODE` as applied.
    pub fn control_mode(&self) -> Option<ControlMode> {
        ControlMode::from_u8(self.u8(TAG_CONTROL_MODE)?)
    }

    /// `CONTROL_AWB_MODE` as the *request this result was taken with* asked for it.
    ///
    /// A repeating request replaced mid-stream does not reach the sensor at once: the results
    /// of the requests already in the pipeline keep coming first, and on 5566 the new value
    /// appears somewhere between the second and the seventh result after the submission
    /// (B8 §6(a) transitions, `16_d36_probe.txt`). So "the result does not carry what was
    /// asked for" means one of two entirely different things, and only the request the result
    /// came with tells them apart:
    ///
    /// * the request does **not** carry the value -- this is an older capture, still in flight when
    ///   the new request was submitted, and nothing has been decided yet;
    /// * the request **does** carry it and the result does not -- the camera ran the entry and
    ///   substituted its own value, which is the HAL dropping it (D36/D47).
    ///
    /// Waiting a fixed number of results instead was D47: six of them is inside the window
    /// above, so a mode set mid-stream was reported "dropped" although the very next results
    /// echoed it.
    pub fn requested_awb_mode(&self) -> Option<AwbMode> {
        AwbMode::from_u8(self.requested_u8(TAG_CONTROL_AWB_MODE)?)
    }

    /// `CONTROL_EFFECT_MODE` as the request asked for it; see
    /// [`CaptureResult::requested_awb_mode`].
    pub fn requested_effect_mode(&self) -> Option<EffectMode> {
        EffectMode::from_u8(self.requested_u8(TAG_CONTROL_EFFECT_MODE)?)
    }

    /// `CONTROL_SCENE_MODE` as the request asked for it; see
    /// [`CaptureResult::requested_awb_mode`].
    pub fn requested_scene_mode(&self) -> Option<SceneMode> {
        SceneMode::from_u8(self.requested_u8(TAG_CONTROL_SCENE_MODE)?)
    }

    /// `CONTROL_MODE` as the request asked for it; see [`CaptureResult::requested_awb_mode`].
    /// A scene mode applies only under `UseSceneMode`, so the two are read together.
    pub fn requested_control_mode(&self) -> Option<ControlMode> {
        ControlMode::from_u8(self.requested_u8(TAG_CONTROL_MODE)?)
    }
}

/// Called with every completed capture's result, on the camera framework's callback thread:
/// it must be `Send + Sync`, must not block, and must read what it wants before returning.
pub type ResultListener = Box<dyn Fn(&CaptureResult<'_>) + Send + Sync>;

/// What the capture callbacks count, for a probe or a log.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ResultStats {
    pub completed: u64,
    pub failed: u64,
    pub buffers_lost: u64,
}

/// The context the capture callbacks are given.
struct ResultContext {
    listener: Option<ResultListener>,
    completed: AtomicU64,
    failed: AtomicU64,
    buffers_lost: AtomicU64,
}

impl ResultContext {
    fn stats(&self) -> ResultStats {
        ResultStats {
            completed: self.completed.load(Ordering::Relaxed),
            failed: self.failed.load(Ordering::Relaxed),
            buffers_lost: self.buffers_lost.load(Ordering::Relaxed),
        }
    }
}

extern "C" fn on_capture_started(
    _context: *mut c_void,
    _session: *mut ACameraCaptureSession,
    _request: *const ACaptureRequest,
    _timestamp: i64,
) {
}

extern "C" fn on_capture_progressed(
    _context: *mut c_void,
    _session: *mut ACameraCaptureSession,
    _request: *mut ACaptureRequest,
    _result: *const ACameraMetadata,
) {
}

extern "C" fn on_capture_completed(
    context: *mut c_void,
    _session: *mut ACameraCaptureSession,
    request: *mut ACaptureRequest,
    result: *const ACameraMetadata,
) {
    if context.is_null() {
        return;
    }
    // SAFETY: context is the `ResultContext` `Camera::open_with` registered; the box holding it
    // outlives the session (dropped after `ACameraCaptureSession_close`).
    let ctx = unsafe { &*(context as *const ResultContext) };
    ctx.completed.fetch_add(1, Ordering::Relaxed);
    if let (Some(listener), false) = (&ctx.listener, result.is_null()) {
        // The request is the framework's copy of the one this capture was taken with -- it
        // "will not match what application has submitted" (`NdkCameraCaptureSession.h:207`),
        // and that is the point: it says what the camera actually ran, so a result that has
        // not caught up with a mid-stream change can be told from one the HAL overrode.
        listener(&CaptureResult {
            metadata: result,
            request,
            _callback: PhantomData,
        });
    }
}

extern "C" fn on_capture_failed(
    context: *mut c_void,
    _session: *mut ACameraCaptureSession,
    _request: *mut ACaptureRequest,
    _failure: *mut ACameraCaptureFailure,
) {
    if !context.is_null() {
        // SAFETY: as in `on_capture_completed`.
        unsafe { &*(context as *const ResultContext) }
            .failed
            .fetch_add(1, Ordering::Relaxed);
    }
}

extern "C" fn on_capture_sequence_completed(
    _context: *mut c_void,
    _session: *mut ACameraCaptureSession,
    _sequence_id: c_int,
    _frame_number: i64,
) {
}

extern "C" fn on_capture_sequence_aborted(
    _context: *mut c_void,
    _session: *mut ACameraCaptureSession,
    _sequence_id: c_int,
) {
}

extern "C" fn on_capture_buffer_lost(
    context: *mut c_void,
    _session: *mut ACameraCaptureSession,
    _request: *mut ACaptureRequest,
    _window: *mut ANativeWindow,
    _frame_number: i64,
) {
    if !context.is_null() {
        // SAFETY: as in `on_capture_completed`.
        unsafe { &*(context as *const ResultContext) }
            .buffers_lost
            .fetch_add(1, Ordering::Relaxed);
    }
}

/// One open camera producing one stream, which is exactly the scope of one V4L2 capture node.
pub struct Camera {
    // Torn down in Drop in the reverse of the order built, so the raw handles are kept as fields
    // rather than in wrappers whose drop order would be the declaration order.
    /// Never read after `open_with`; kept so the manager outlives the device it opened.
    _manager: Manager,
    device: *mut ACameraDevice,
    session: *mut ACameraCaptureSession,
    request: *mut ACaptureRequest,
    target: *mut ACameraOutputTarget,
    output: *mut ACaptureSessionOutput,
    container: *mut ACaptureSessionOutputContainer,
    reader: *mut AImageReader,
    /// Boxed because the NDK keeps the pointer we hand it; the raw context inside points at
    /// `signal_raw`.
    _listener: Box<AImageReaderImageListener>,
    _device_callbacks: Box<ACameraDeviceStateCallbacks>,
    /// What `_device_callbacks.context` points at; dropped after the device is closed. That no
    /// state callback runs *after* `ACameraDevice_close` returns is **[unverified]**: the header
    /// promises only that the device is removed from memory and that touching it then crashes
    /// (`NdkCameraDevice.h:173-184`), so the guarantee this order relies on comes from AOSP's
    /// `CameraDevice` destructor stopping its looper, not from the NDK contract (review-m4 R6).
    _state: Box<StateContext>,
    _session_callbacks: Box<ACameraCaptureSessionStateCallbacks>,
    /// Handed to every `setRepeatingRequest` and `capture`; its `context` is `_results`. Both
    /// are dropped after the session is closed, on the same **[unverified]** assumption as
    /// `_state`: that no capture callback runs once `ACameraCaptureSession_close` has returned.
    capture_callbacks: Box<ACameraCaptureSessionCaptureCallbacks>,
    _results: Box<ResultContext>,
    signal: Arc<FrameSignal>,
    signal_raw: *const FrameSignal,
    pub width: i32,
    pub height: i32,
}

/// Everything `Camera::open_with` has created so far, so that a failure halfway releases it all
/// in the reverse order of construction -- the same order `Camera::drop` uses -- instead of
/// leaking an opened device per attempt. Every handle is null (or `None`) until created; once
/// the `Camera` exists it takes them over and the guard has nothing left to free.
struct OpenGuard {
    manager: Option<Manager>,
    signal_raw: *const FrameSignal,
    reader: *mut AImageReader,
    listener: Option<Box<AImageReaderImageListener>>,
    device_callbacks: Option<Box<ACameraDeviceStateCallbacks>>,
    state: Option<Box<StateContext>>,
    device: *mut ACameraDevice,
    output: *mut ACaptureSessionOutput,
    container: *mut ACaptureSessionOutputContainer,
    session_callbacks: Option<Box<ACameraCaptureSessionStateCallbacks>>,
    session: *mut ACameraCaptureSession,
    request: *mut ACaptureRequest,
    target: *mut ACameraOutputTarget,
    capture_callbacks: Option<Box<ACameraCaptureSessionCaptureCallbacks>>,
    results: Option<Box<ResultContext>>,
}

impl Drop for OpenGuard {
    fn drop(&mut self) {
        // SAFETY: every non-null handle was produced by the matching create call and is released
        // exactly once, here or in `Camera::drop`, never both: `Camera::open_with` nulls the
        // fields it takes over.
        unsafe {
            if !self.session.is_null() {
                ACameraCaptureSession_stopRepeating(self.session);
                ACameraCaptureSession_close(self.session);
            }
            if !self.device.is_null() {
                ACameraDevice_close(self.device);
            }
            if !self.request.is_null() {
                ACaptureRequest_free(self.request);
            }
            if !self.target.is_null() {
                ACameraOutputTarget_free(self.target);
            }
            if !self.container.is_null() {
                ACaptureSessionOutputContainer_free(self.container);
            }
            if !self.output.is_null() {
                ACaptureSessionOutput_free(self.output);
            }
            if !self.reader.is_null() {
                // "Set this to NULL if the application no longer needs to listen to new images"
                // (`NdkImageReader.h:311-312`): the reader's own callback thread is told to stop
                // before the box holding its context can go anywhere (the android-camera survey's
                // open item (d)).
                AImageReader_setImageListener(self.reader, null_mut());
                AImageReader_delete(self.reader);
            }
            if !self.signal_raw.is_null() {
                drop(Arc::from_raw(self.signal_raw));
            }
        }
        // The boxes (callback structs, state context) and the manager drop with the guard, after
        // the handles that referenced them are gone -- on every path, because each box is moved
        // into the guard before the call that registers it and could fail.
    }
}

impl Camera {
    /// Open `id` and start a repeating request delivering `width`x`height` `YUV_420_888` frames.
    ///
    /// `max_images` is how many frames may be held by the caller at once; the camera stalls when
    /// they are all outstanding, so it is the queue depth a V4L2 `REQBUFS` would ask for. Keep it
    /// at 3 or more if [`Camera::next_frame_latest`] is to discard anything
    /// (`NdkImageReader.h:239-245`, NDK r29: with fewer than two free slots it cannot).
    pub fn open(id: &str, width: i32, height: i32, max_images: i32) -> Result<Camera> {
        Self::open_with(
            id,
            width,
            height,
            max_images,
            None,
            None,
            &RequestUpdate::new(),
        )
    }

    /// As [`Camera::open`], with `on_state` told when the platform disconnects the camera or
    /// reports a device error -- the only way a caller learns that frames have stopped for good
    /// rather than stalled -- `on_result` handed every completed capture's metadata, and
    /// `initial` written into the request before it is first submitted, so the first frame is
    /// already taken with the caller's settings.
    ///
    /// Fails cleanly: whatever was created before the failing step is released again, so a
    /// caller may retry (a guest reopening after a transient `ERROR_CAMERA_IN_USE`) without
    /// leaking an opened device per attempt.
    pub fn open_with(
        id: &str,
        width: i32,
        height: i32,
        max_images: i32,
        on_state: Option<StateListener>,
        on_result: Option<ResultListener>,
        initial: &RequestUpdate,
    ) -> Result<Camera> {
        ensure_loaded()?;
        let manager = Manager::new()?;
        let c_id = CString::new(id).map_err(|_| CameraError::BadCameraId)?;

        // Fail on an unsupported size here rather than letting session configuration fail later
        // with a status that does not say which stream was wrong.
        let characteristics = manager.characteristics(&c_id)?;
        let sizes = characteristics.output_sizes(AIMAGE_FORMAT_YUV_420_888);
        if sizes.is_empty() {
            return Err(CameraError::NoSuchCamera(id.to_owned()));
        }
        if !sizes.contains(&(width, height)) {
            return Err(CameraError::UnsupportedSize(id.to_owned(), width, height));
        }
        drop(characteristics);

        let mut g = OpenGuard {
            manager: Some(manager),
            signal_raw: std::ptr::null(),
            reader: null_mut(),
            listener: None,
            device_callbacks: None,
            state: None,
            device: null_mut(),
            output: null_mut(),
            container: null_mut(),
            session_callbacks: None,
            session: null_mut(),
            request: null_mut(),
            target: null_mut(),
            capture_callbacks: None,
            results: None,
        };
        let manager_ptr = g.manager.as_ref().expect("just set").0;

        let signal = Arc::new(FrameSignal {
            delivered: AtomicU64::new(0),
            lock: Mutex::new(()),
            cond: Condvar::new(),
        });
        g.signal_raw = Arc::into_raw(Arc::clone(&signal));

        check_media("AImageReader_new", unsafe {
            // SAFETY: out-parameter written only on success.
            AImageReader_new(
                width,
                height,
                AIMAGE_FORMAT_YUV_420_888,
                max_images,
                &mut g.reader,
            )
        })?;

        // Into the guard *before* the call that registers it: a failing
        // `AImageReader_setImageListener` must not drop the box the reader may already point at
        // (review-m4 R6). Same rule for the two boxes below.
        g.listener = Some(Box::new(AImageReaderImageListener {
            context: g.signal_raw as *mut c_void,
            on_image_available: Some(on_image_available),
        }));
        let listener_ptr: *mut AImageReaderImageListener =
            g.listener.as_mut().expect("just set").as_mut();
        check_media("AImageReader_setImageListener", unsafe {
            // SAFETY: reader is live, and the listener box is the guard's, so it is freed only
            // after AImageReader_delete -- in Camera::drop or in OpenGuard::drop, on every path.
            AImageReader_setImageListener(g.reader, listener_ptr)
        })?;

        let mut window: *mut ANativeWindow = null_mut();
        // SAFETY: reader is live; the window it returns is owned by the reader.
        check_media("AImageReader_getWindow", unsafe {
            AImageReader_getWindow(g.reader, &mut window)
        })?;

        g.state = Some(Box::new(StateContext { listener: on_state }));
        let state_ptr = g.state.as_ref().expect("just set").as_ref() as *const StateContext;
        g.device_callbacks = Some(Box::new(ACameraDeviceStateCallbacks {
            context: state_ptr as *mut c_void,
            on_disconnected: Some(on_device_disconnected),
            on_error: Some(on_device_error),
            on_client_shared_access_priority_changed: None,
        }));
        let callbacks_ptr: *mut ACameraDeviceStateCallbacks =
            g.device_callbacks.as_mut().expect("just set").as_mut();
        // SAFETY: all four arguments are live for the call; device is written only on success.
        // This is the call that fails with ERROR_PERMISSION_DENIED when the real uid resolves to
        // no package or to one without CAMERA. Both boxes already belong to the guard, so a
        // failure here closes the device the NDK may have written into `g.device` *before* the
        // context its callbacks point at is freed, instead of after it (review-m4 R6).
        check("ACameraManager_openCamera", unsafe {
            ACameraManager_openCamera(manager_ptr, c_id.as_ptr(), callbacks_ptr, &mut g.device)
        })?;

        // SAFETY: window belongs to the live reader; output written only on success.
        check("ACaptureSessionOutput_create", unsafe {
            ACaptureSessionOutput_create(window, &mut g.output)
        })?;

        // SAFETY: out-parameter written only on success.
        check("ACaptureSessionOutputContainer_create", unsafe {
            ACaptureSessionOutputContainer_create(&mut g.container)
        })?;
        // SAFETY: both handles are live and the container only records the pointer.
        check("ACaptureSessionOutputContainer_add", unsafe {
            ACaptureSessionOutputContainer_add(g.container, g.output)
        })?;

        let mut session_callbacks = Box::new(ACameraCaptureSessionStateCallbacks {
            context: null_mut(),
            on_closed: Some(on_session_closed),
            on_ready: Some(on_session_ready),
            on_active: Some(on_session_active),
        });
        // SAFETY: device and container are live; the callbacks box outlives the session.
        check("ACameraDevice_createCaptureSession", unsafe {
            ACameraDevice_createCaptureSession(
                g.device,
                g.container,
                session_callbacks.as_mut() as *const _,
                &mut g.session,
            )
        })?;
        g.session_callbacks = Some(session_callbacks);

        // TEMPLATE_RECORD rather than TEMPLATE_PREVIEW: a virtio-media capture device is a
        // continuous stream, and RECORD is the template whose defaults hold the frame rate steady
        // instead of letting AE drop it in low light.
        // SAFETY: device is live; request written only on success.
        check("ACameraDevice_createCaptureRequest", unsafe {
            ACameraDevice_createCaptureRequest(g.device, TEMPLATE_RECORD, &mut g.request)
        })?;

        // SAFETY: window belongs to the live reader; target written only on success.
        check("ACameraOutputTarget_create", unsafe {
            ACameraOutputTarget_create(window, &mut g.target)
        })?;
        // SAFETY: both handles live; the request records the target.
        check("ACaptureRequest_addTarget", unsafe {
            ACaptureRequest_addTarget(g.request, g.target)
        })?;
        // The caller's settings, before the first submission.
        // SAFETY: the request is live.
        unsafe { initial.write_into(g.request) }?;

        // The capture callbacks: results, failures and lost buffers come back through them from
        // the first submission on. Into the guard first, like the other boxes.
        g.results = Some(Box::new(ResultContext {
            listener: on_result,
            completed: AtomicU64::new(0),
            failed: AtomicU64::new(0),
            buffers_lost: AtomicU64::new(0),
        }));
        let results_ptr = g.results.as_ref().expect("just set").as_ref() as *const ResultContext;
        g.capture_callbacks = Some(Box::new(ACameraCaptureSessionCaptureCallbacks {
            context: results_ptr as *mut c_void,
            on_capture_started: Some(on_capture_started),
            on_capture_progressed: Some(on_capture_progressed),
            on_capture_completed: Some(on_capture_completed),
            on_capture_failed: Some(on_capture_failed),
            on_capture_sequence_completed: Some(on_capture_sequence_completed),
            on_capture_sequence_aborted: Some(on_capture_sequence_aborted),
            on_capture_buffer_lost: Some(on_capture_buffer_lost),
        }));

        // Everything exists: the camera takes the handles over and the guard is left empty.
        let mut camera = Camera {
            _manager: g.manager.take().expect("set above"),
            device: std::mem::replace(&mut g.device, null_mut()),
            session: std::mem::replace(&mut g.session, null_mut()),
            request: std::mem::replace(&mut g.request, null_mut()),
            target: std::mem::replace(&mut g.target, null_mut()),
            output: std::mem::replace(&mut g.output, null_mut()),
            container: std::mem::replace(&mut g.container, null_mut()),
            reader: std::mem::replace(&mut g.reader, null_mut()),
            _listener: g.listener.take().expect("set above"),
            _device_callbacks: g.device_callbacks.take().expect("set above"),
            _state: g.state.take().expect("set above"),
            _session_callbacks: g.session_callbacks.take().expect("set above"),
            capture_callbacks: g.capture_callbacks.take().expect("set above"),
            _results: g.results.take().expect("set above"),
            signal,
            signal_raw: std::mem::replace(&mut g.signal_raw, std::ptr::null()),
            width,
            height,
        };
        drop(g);
        // A failure here is the camera's own to unwind, through `Camera::drop`.
        camera.submit()?;
        Ok(camera)
    }

    /// Push the current request to the camera. Every control change goes through here: Camera2
    /// settings live on the request, not on the device, so a changed request has to be resubmitted
    /// to take effect.
    fn submit(&mut self) -> Result<()> {
        let mut request = self.request;
        let callbacks: *mut ACameraCaptureSessionCaptureCallbacks = self.capture_callbacks.as_mut();
        // SAFETY: session and request are live, the array of one is valid for the call, and the
        // callbacks box lives as long as the session.
        check("ACameraCaptureSession_setRepeatingRequest", unsafe {
            ACameraCaptureSession_setRepeatingRequest(
                self.session,
                callbacks,
                1,
                &mut request,
                null_mut(),
            )
        })
    }

    /// Write `update` into the repeating request and resubmit it once: however many entries,
    /// one `setRepeatingRequest`. An empty update submits nothing.
    pub fn apply(&mut self, update: &RequestUpdate) -> Result<()> {
        if update.is_empty() {
            return Ok(());
        }
        // SAFETY: the request is live.
        unsafe { update.write_into(self.request) }?;
        self.submit()
    }

    /// `CONTROL_AF_TRIGGER`, as Camera2 means it: a one-shot capture carrying the trigger,
    /// slipped into the repeating stream, so the trigger fires exactly once. Putting it in the
    /// repeating request instead would either fire it every frame or -- if it were reset in a
    /// second submission straight after -- maybe never, since a replaced repeating request need
    /// not have produced a frame.
    pub fn trigger_af(&mut self, trigger: AfTrigger) -> Result<()> {
        let copy_request = ndk()
            .ACaptureRequest_copy
            .ok_or(CameraError::MissingSymbol("ACaptureRequest_copy"))?;
        // SAFETY: the request is live; the copy is ours until freed below.
        let copy = unsafe { copy_request(self.request) };
        if copy.is_null() {
            return Err(CameraError::Ndk("ACaptureRequest_copy", 0, "returned null"));
        }
        let trigger = trigger as u8;
        let mut requests = copy;
        let callbacks: *mut ACameraCaptureSessionCaptureCallbacks = self.capture_callbacks.as_mut();
        // SAFETY: the copy is a live request; the pointer addresses one u8 for a count of 1;
        // the session and callbacks are live. The framework copies the request on submission,
        // so freeing ours afterwards is fine (the header notes the request seen in callbacks
        // "will not match what application has submitted").
        let result = unsafe {
            check(
                "ACaptureRequest_setEntry_u8(AF_TRIGGER)",
                ACaptureRequest_setEntry_u8(copy, TAG_CONTROL_AF_TRIGGER, 1, &trigger),
            )
            .and_then(|()| {
                check(
                    "ACameraCaptureSession_capture",
                    ACameraCaptureSession_capture(
                        self.session,
                        callbacks,
                        1,
                        &mut requests,
                        null_mut(),
                    ),
                )
            })
        };
        // SAFETY: freed exactly once, after its only use.
        unsafe { ACaptureRequest_free(copy) };
        result
    }

    /// What the capture callbacks have counted since open.
    pub fn results(&self) -> ResultStats {
        self._results.stats()
    }

    /// `CONTROL_ZOOM_RATIO`, the V4L2_CID_ZOOM_ABSOLUTE equivalent. On a logical multi-camera this
    /// is also what makes the platform switch between the ultra-wide, main and tele lenses.
    pub fn set_zoom_ratio(&mut self, ratio: f32) -> Result<()> {
        self.apply(&RequestUpdate::new().zoom_ratio(ratio))
    }

    /// `FLASH_MODE`, the V4L2_CID_FLASH_LED_MODE equivalent.
    ///
    /// AE has to be pinned to plain `ON` first: under any of the `ON_*_FLASH` auto modes the 3A
    /// routine owns the LED and silently overrides this.
    pub fn set_flash_mode(&mut self, mode: FlashMode) -> Result<()> {
        self.apply(&RequestUpdate::new().ae_mode(AeMode::On).flash_mode(mode))
    }

    /// `CONTROL_AF_MODE`, the V4L2_CID_FOCUS_AUTO equivalent.
    pub fn set_af_mode(&mut self, mode: AfMode) -> Result<()> {
        self.apply(&RequestUpdate::new().af_mode(mode))
    }

    /// `CONTROL_AE_TARGET_FPS_RANGE`. V4L2 `S_PARM` carries a single interval, so a capture device
    /// would pin both ends to the rate the guest asked for.
    pub fn set_fps_range(&mut self, min: i32, max: i32) -> Result<()> {
        self.apply(&RequestUpdate::new().fps_range(min, max))
    }

    /// How many times the image listener has fired since open.
    pub fn frames_signalled(&self) -> u64 {
        self.signal.delivered.load(Ordering::Acquire)
    }

    /// Wait up to `timeout` for the next frame. `Ok(None)` means the deadline passed with the
    /// camera still running, which is a stall rather than an error.
    pub fn next_frame(&self, timeout: Duration) -> Result<Option<Frame<'_>>> {
        self.acquire(timeout, false)
    }

    /// As [`Camera::next_frame`], but the *newest* frame, discarding older ones the reader still
    /// holds (`AImageReader_acquireLatestImage`): what a consumer that has fallen behind wants,
    /// since a frame it skipped is stale by the time it would be copied. On a platform without
    /// that entry point this is [`Camera::next_frame`].
    pub fn next_frame_latest(&self, timeout: Duration) -> Result<Option<Frame<'_>>> {
        self.acquire(timeout, true)
    }

    /// Whether [`Camera::next_frame_latest`] really discards, or is the ordered fallback.
    pub fn can_acquire_latest(&self) -> bool {
        ndk().AImageReader_acquireLatestImage.is_some()
    }

    fn acquire(&self, timeout: Duration, latest: bool) -> Result<Option<Frame<'_>>> {
        let deadline = Instant::now() + timeout;
        loop {
            let mut image: *mut AImage = null_mut();
            // SAFETY: reader is live; image is written only when a buffer was available.
            let status = unsafe {
                match (latest, ndk().AImageReader_acquireLatestImage) {
                    (true, Some(acquire_latest)) => acquire_latest(self.reader, &mut image),
                    _ => AImageReader_acquireNextImage(self.reader, &mut image),
                }
            };
            match status {
                AMEDIA_OK => return Frame::new(image).map(Some),
                // Not an error: the queue is empty until the camera produces the next frame.
                AMEDIA_IMGREADER_NO_BUFFER_AVAILABLE => {}
                other => {
                    return Err(CameraError::Ndk(
                        "AImageReader_acquireNextImage",
                        other,
                        "AMEDIA",
                    ))
                }
            }
            let now = Instant::now();
            if now >= deadline {
                return Ok(None);
            }
            // Bounded so a listener that never fires degrades to polling rather than to a hang --
            // and frames_signalled() still records which of the two actually happened.
            self.signal
                .wait(std::cmp::min(deadline - now, Duration::from_millis(20)));
        }
    }
}

impl Drop for Camera {
    fn drop(&mut self) {
        // Reverse of construction. The reader has to outlive the session that writes into its
        // window, and the Arc backing the listener context has to outlive the reader.
        // SAFETY: every handle below was produced by the matching create call in open() and is
        // released exactly once here.
        unsafe {
            ACameraCaptureSession_stopRepeating(self.session);
            ACameraCaptureSession_close(self.session);
            ACameraDevice_close(self.device);
            ACaptureRequest_free(self.request);
            ACameraOutputTarget_free(self.target);
            ACaptureSessionOutputContainer_free(self.container);
            ACaptureSessionOutput_free(self.output);
            // As in `OpenGuard::drop`: unregister before delete (`NdkImageReader.h:311-312`).
            AImageReader_setImageListener(self.reader, null_mut());
            AImageReader_delete(self.reader);
            drop(Arc::from_raw(self.signal_raw));
        }
    }
}

/// The concrete arrangement of a `YUV_420_888` frame, which is what picks the V4L2 fourcc.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum YuvLayout {
    /// Y plane then interleaved Cb/Cr: `V4L2_PIX_FMT_NV12`.
    Nv12,
    /// Y plane then interleaved Cr/Cb: `V4L2_PIX_FMT_NV21`.
    Nv21,
    /// Three fully separate planes: `V4L2_PIX_FMT_YUV420`.
    I420,
    /// Chroma is neither adjacent nor separate; no single fourcc describes it.
    Unknown,
}

pub struct Plane {
    data: *const u8,
    len: usize,
    pub row_stride: i32,
    pub pixel_stride: i32,
}

impl Plane {
    /// The first byte of the plane, valid for [`Plane::len`] bytes while the frame lives.
    ///
    /// For interleaved chroma (NV12/NV21) the two chroma planes are one allocation, offset by a
    /// byte: `len` of each stops one byte short of the region's end -- the last sample's other
    /// half belongs to the other plane -- so a copy of the whole region must take the farther of
    /// the two ends, which is what `plane_data()`'s per-plane slices cannot express.
    pub fn as_ptr(&self) -> *const u8 {
        self.data
    }

    pub fn len(&self) -> usize {
        self.len
    }

    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
}

/// One acquired frame. Holds a buffer the camera cannot reuse until it is dropped, so a caller
/// must not keep more than the `max_images` passed to [`Camera::open`].
pub struct Frame<'a> {
    image: *mut AImage,
    pub width: i32,
    pub height: i32,
    pub format: i32,
    pub timestamp_ns: i64,
    pub planes: Vec<Plane>,
    _camera: PhantomData<&'a Camera>,
}

impl<'a> Frame<'a> {
    fn new(image: *mut AImage) -> Result<Frame<'a>> {
        let mut width = 0;
        let mut height = 0;
        let mut format = 0;
        let mut timestamp_ns = 0;
        let mut num_planes = 0;
        // SAFETY: image is a live acquired AImage and every out-parameter is a live local.
        unsafe {
            check_media("AImage_getWidth", AImage_getWidth(image, &mut width))?;
            check_media("AImage_getHeight", AImage_getHeight(image, &mut height))?;
            check_media("AImage_getFormat", AImage_getFormat(image, &mut format))?;
            check_media(
                "AImage_getTimestamp",
                AImage_getTimestamp(image, &mut timestamp_ns),
            )?;
            check_media(
                "AImage_getNumberOfPlanes",
                AImage_getNumberOfPlanes(image, &mut num_planes),
            )?;
        }

        let mut planes = Vec::with_capacity(num_planes as usize);
        for i in 0..num_planes {
            let mut data: *mut u8 = null_mut();
            let mut len = 0;
            let mut row_stride = 0;
            let mut pixel_stride = 0;
            // SAFETY: i is within the plane count the image just reported.
            unsafe {
                check_media(
                    "AImage_getPlaneData",
                    AImage_getPlaneData(image, i, &mut data, &mut len),
                )?;
                check_media(
                    "AImage_getPlaneRowStride",
                    AImage_getPlaneRowStride(image, i, &mut row_stride),
                )?;
                check_media(
                    "AImage_getPlanePixelStride",
                    AImage_getPlanePixelStride(image, i, &mut pixel_stride),
                )?;
            }
            planes.push(Plane {
                data,
                len: len.max(0) as usize,
                row_stride,
                pixel_stride,
            });
        }

        Ok(Frame {
            image,
            width,
            height,
            format,
            timestamp_ns,
            planes,
            _camera: PhantomData,
        })
    }

    pub fn plane_data(&self, index: usize) -> &[u8] {
        let plane = &self.planes[index];
        // SAFETY: the pointer and length came from AImage_getPlaneData for this image, which stays
        // mapped until AImage_delete in Drop, and the returned slice borrows self.
        unsafe { std::slice::from_raw_parts(plane.data, plane.len) }
    }

    /// Work out the fourcc-equivalent layout from the strides and from where the chroma planes sit
    /// relative to each other. `YUV_420_888` does not promise any particular one, so this has to be
    /// read back per device rather than assumed.
    pub fn layout(&self) -> YuvLayout {
        if self.planes.len() != 3 {
            return YuvLayout::Unknown;
        }
        let (u, v) = (&self.planes[1], &self.planes[2]);
        match (u.pixel_stride, v.pixel_stride) {
            (1, 1) => YuvLayout::I420,
            (2, 2) => {
                // SAFETY: both pointers came from the same image's plane data; comparing their
                // offset is defined because interleaved chroma lives in one allocation.
                let delta = (v.data as isize) - (u.data as isize);
                match delta {
                    1 => YuvLayout::Nv12,
                    -1 => YuvLayout::Nv21,
                    _ => YuvLayout::Unknown,
                }
            }
            _ => YuvLayout::Unknown,
        }
    }
}

impl Drop for Frame<'_> {
    fn drop(&mut self) {
        // SAFETY: image came from AImageReader_acquireNextImage and is deleted exactly once, which
        // is also what returns the buffer to the camera.
        unsafe { AImage_delete(self.image) };
    }
}
