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
/// `extern "C"` block.
macro_rules! ndk_api {
    ($( fn $name:ident ( $($arg:ident : $argty:ty),* $(,)? ) $(-> $ret:ty)?; )*) => {
        #[allow(non_snake_case)]
        struct NdkApi {
            $( $name: unsafe extern "C" fn($($argty),*) $(-> $ret)?, )*
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
        return Err(CameraError::LibraryLoad(name.trim_end_matches('\0'), reason));
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
    fn ACameraCaptureSession_setRepeatingRequest(
        session: *mut ACameraCaptureSession,
        callbacks: *mut c_void,
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

const TAG_CONTROL_AE_MODE: u32 = SECTION_CONTROL + 3;
const TAG_CONTROL_AE_TARGET_FPS_RANGE: u32 = SECTION_CONTROL + 5;
const TAG_CONTROL_AF_MODE: u32 = SECTION_CONTROL + 7;
const TAG_CONTROL_MAX_REGIONS: u32 = SECTION_CONTROL + 28;
const TAG_CONTROL_ZOOM_RATIO_RANGE: u32 = SECTION_CONTROL + 46;
const TAG_CONTROL_ZOOM_RATIO: u32 = SECTION_CONTROL + 47;
const TAG_FLASH_MODE: u32 = SECTION_FLASH + 2;
const TAG_FLASH_INFO_AVAILABLE: u32 = SECTION_FLASH_INFO;
const TAG_LENS_FACING: u32 = SECTION_LENS + 5;
const TAG_SCALER_AVAILABLE_MAX_DIGITAL_ZOOM: u32 = SECTION_SCALER + 4;
const TAG_SCALER_AVAILABLE_STREAM_CONFIGURATIONS: u32 = SECTION_SCALER + 10;
const TAG_SENSOR_ORIENTATION: u32 = SECTION_SENSOR + 14;
const TAG_INFO_SUPPORTED_HARDWARE_LEVEL: u32 = SECTION_INFO;
const TAG_REQUEST_AVAILABLE_CAPABILITIES: u32 = SECTION_REQUEST + 12;
const TAG_SCALER_AVAILABLE_STREAM_USE_CASES: u32 = SECTION_SCALER + 26;
const TAG_LOGICAL_MULTI_CAMERA_PHYSICAL_IDS: u32 = SECTION_LOGICAL_MULTI_CAMERA;

/// `ACAMERA_REQUEST_AVAILABLE_CAPABILITIES_LOGICAL_MULTI_CAMERA`: the camera is one lens group
/// presented as one device, with the platform choosing which sensor serves a given zoom ratio.
const CAPABILITY_LOGICAL_MULTI_CAMERA: u8 = 11;

const TEMPLATE_RECORD: i32 = 3;

const TYPE_BYTE: u8 = 0;
const TYPE_INT32: u8 = 1;
const TYPE_FLOAT: u8 = 2;
const TYPE_INT64: u8 = 3;

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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlashMode {
    Off = 0,
    Single = 1,
    Torch = 2,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AfMode {
    Off = 0,
    Auto = 1,
    Macro = 2,
    ContinuousVideo = 3,
    ContinuousPicture = 4,
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
                unsafe { std::slice::from_raw_parts(e.data as *const u8, e.count as usize) }.to_vec()
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
            facing: self.u8s(TAG_LENS_FACING).first().copied().unwrap_or(255).into(),
            orientation: self.i32s(TAG_SENSOR_ORIENTATION).first().copied().unwrap_or(0),
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
            capabilities: self.u8s(TAG_REQUEST_AVAILABLE_CAPABILITIES),
            physical_ids_raw_len: self.u8s(TAG_LOGICAL_MULTI_CAMERA_PHYSICAL_IDS).len(),
            physical_ids: split_nul_strings(&self.u8s(TAG_LOGICAL_MULTI_CAMERA_PHYSICAL_IDS)),
            stream_use_cases: self.i64s(TAG_SCALER_AVAILABLE_STREAM_USE_CASES),
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
    let ids = unsafe { std::slice::from_raw_parts((*list).camera_ids, (*list).num_cameras as usize) };
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

extern "C" fn on_device_disconnected(_context: *mut c_void, _device: *mut ACameraDevice) {
    base::error!("android_camera: camera device disconnected");
}

extern "C" fn on_device_error(_context: *mut c_void, _device: *mut ACameraDevice, error: i32) {
    base::error!("android_camera: camera device error {}", error);
}

extern "C" fn on_session_closed(_context: *mut c_void, _session: *mut ACameraCaptureSession) {}
extern "C" fn on_session_ready(_context: *mut c_void, _session: *mut ACameraCaptureSession) {}
extern "C" fn on_session_active(_context: *mut c_void, _session: *mut ACameraCaptureSession) {}

/// One open camera producing one stream, which is exactly the scope of one V4L2 capture node.
pub struct Camera {
    // Torn down in Drop in the reverse of the order built, so the raw handles are kept as fields
    // rather than in wrappers whose drop order would be the declaration order.
    manager: Manager,
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
    _session_callbacks: Box<ACameraCaptureSessionStateCallbacks>,
    signal: Arc<FrameSignal>,
    signal_raw: *const FrameSignal,
    pub width: i32,
    pub height: i32,
}

impl Camera {
    /// Open `id` and start a repeating request delivering `width`x`height` `YUV_420_888` frames.
    ///
    /// `max_images` is how many frames may be held by the caller at once; the camera stalls when
    /// they are all outstanding, so it is the queue depth a V4L2 `REQBUFS` would ask for.
    pub fn open(id: &str, width: i32, height: i32, max_images: i32) -> Result<Camera> {
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

        let signal = Arc::new(FrameSignal {
            delivered: AtomicU64::new(0),
            lock: Mutex::new(()),
            cond: Condvar::new(),
        });
        let signal_raw = Arc::into_raw(Arc::clone(&signal));

        let mut reader: *mut AImageReader = null_mut();
        check_media("AImageReader_new", unsafe {
            // SAFETY: out-parameter written only on success.
            AImageReader_new(
                width,
                height,
                AIMAGE_FORMAT_YUV_420_888,
                max_images,
                &mut reader,
            )
        })?;

        let mut listener = Box::new(AImageReaderImageListener {
            context: signal_raw as *mut c_void,
            on_image_available: Some(on_image_available),
        });
        check_media("AImageReader_setImageListener", unsafe {
            // SAFETY: reader is live, and the listener box outlives it (dropped after
            // AImageReader_delete in Camera::drop).
            AImageReader_setImageListener(reader, listener.as_mut() as *mut _)
        })?;

        let mut window: *mut ANativeWindow = null_mut();
        // SAFETY: reader is live; the window it returns is owned by the reader.
        check_media("AImageReader_getWindow", unsafe {
            AImageReader_getWindow(reader, &mut window)
        })?;

        let mut device_callbacks = Box::new(ACameraDeviceStateCallbacks {
            context: null_mut(),
            on_disconnected: Some(on_device_disconnected),
            on_error: Some(on_device_error),
            on_client_shared_access_priority_changed: None,
        });
        let mut device: *mut ACameraDevice = null_mut();
        // SAFETY: all four arguments are live for the call; device is written only on success.
        // This is the call that fails with ERROR_PERMISSION_DENIED when the real uid resolves to
        // no package or to one without CAMERA.
        check("ACameraManager_openCamera", unsafe {
            ACameraManager_openCamera(
                manager.0,
                c_id.as_ptr(),
                device_callbacks.as_mut() as *mut _,
                &mut device,
            )
        })?;

        let mut output: *mut ACaptureSessionOutput = null_mut();
        // SAFETY: window belongs to the live reader; output written only on success.
        check("ACaptureSessionOutput_create", unsafe {
            ACaptureSessionOutput_create(window, &mut output)
        })?;

        let mut container: *mut ACaptureSessionOutputContainer = null_mut();
        // SAFETY: out-parameter written only on success.
        check("ACaptureSessionOutputContainer_create", unsafe {
            ACaptureSessionOutputContainer_create(&mut container)
        })?;
        // SAFETY: both handles are live and the container only records the pointer.
        check("ACaptureSessionOutputContainer_add", unsafe {
            ACaptureSessionOutputContainer_add(container, output)
        })?;

        let mut session_callbacks = Box::new(ACameraCaptureSessionStateCallbacks {
            context: null_mut(),
            on_closed: Some(on_session_closed),
            on_ready: Some(on_session_ready),
            on_active: Some(on_session_active),
        });
        let mut session: *mut ACameraCaptureSession = null_mut();
        // SAFETY: device and container are live; the callbacks box outlives the session.
        check("ACameraDevice_createCaptureSession", unsafe {
            ACameraDevice_createCaptureSession(
                device,
                container,
                session_callbacks.as_mut() as *const _,
                &mut session,
            )
        })?;

        // TEMPLATE_RECORD rather than TEMPLATE_PREVIEW: a virtio-media capture device is a
        // continuous stream, and RECORD is the template whose defaults hold the frame rate steady
        // instead of letting AE drop it in low light.
        let mut request: *mut ACaptureRequest = null_mut();
        // SAFETY: device is live; request written only on success.
        check("ACameraDevice_createCaptureRequest", unsafe {
            ACameraDevice_createCaptureRequest(device, TEMPLATE_RECORD, &mut request)
        })?;

        let mut target: *mut ACameraOutputTarget = null_mut();
        // SAFETY: window belongs to the live reader; target written only on success.
        check("ACameraOutputTarget_create", unsafe {
            ACameraOutputTarget_create(window, &mut target)
        })?;
        // SAFETY: both handles live; the request records the target.
        check("ACaptureRequest_addTarget", unsafe {
            ACaptureRequest_addTarget(request, target)
        })?;

        let camera = Camera {
            manager,
            device,
            session,
            request,
            target,
            output,
            container,
            reader,
            _listener: listener,
            _device_callbacks: device_callbacks,
            _session_callbacks: session_callbacks,
            signal,
            signal_raw,
            width,
            height,
        };
        camera.submit()?;
        Ok(camera)
    }

    /// Push the current request to the camera. Every control change goes through here: Camera2
    /// settings live on the request, not on the device, so a changed request has to be resubmitted
    /// to take effect.
    fn submit(&self) -> Result<()> {
        let mut request = self.request;
        // SAFETY: session and request are live, and the array of one is valid for the call.
        check("ACameraCaptureSession_setRepeatingRequest", unsafe {
            ACameraCaptureSession_setRepeatingRequest(
                self.session,
                null_mut(),
                1,
                &mut request,
                null_mut(),
            )
        })
    }

    /// `CONTROL_ZOOM_RATIO`, the V4L2_CID_ZOOM_ABSOLUTE equivalent. On a logical multi-camera this
    /// is also what makes the platform switch between the ultra-wide, main and tele lenses.
    pub fn set_zoom_ratio(&mut self, ratio: f32) -> Result<()> {
        // SAFETY: request is live and the pointer addresses one f32, matching count 1.
        check("ACaptureRequest_setEntry_float(ZOOM_RATIO)", unsafe {
            ACaptureRequest_setEntry_float(self.request, TAG_CONTROL_ZOOM_RATIO, 1, &ratio)
        })?;
        self.submit()
    }

    /// `FLASH_MODE`, the V4L2_CID_FLASH_LED_MODE equivalent.
    ///
    /// AE has to be pinned to plain `ON` first: under any of the `ON_*_FLASH` auto modes the 3A
    /// routine owns the LED and silently overrides this.
    pub fn set_flash_mode(&mut self, mode: FlashMode) -> Result<()> {
        let ae_on: u8 = 1;
        // SAFETY: request is live; each pointer addresses one u8, matching count 1.
        check("ACaptureRequest_setEntry_u8(AE_MODE)", unsafe {
            ACaptureRequest_setEntry_u8(self.request, TAG_CONTROL_AE_MODE, 1, &ae_on)
        })?;
        let value = mode as u8;
        // SAFETY: as above.
        check("ACaptureRequest_setEntry_u8(FLASH_MODE)", unsafe {
            ACaptureRequest_setEntry_u8(self.request, TAG_FLASH_MODE, 1, &value)
        })?;
        self.submit()
    }

    /// `CONTROL_AF_MODE`, the V4L2_CID_FOCUS_AUTO equivalent.
    pub fn set_af_mode(&mut self, mode: AfMode) -> Result<()> {
        let value = mode as u8;
        // SAFETY: request is live; the pointer addresses one u8, matching count 1.
        check("ACaptureRequest_setEntry_u8(AF_MODE)", unsafe {
            ACaptureRequest_setEntry_u8(self.request, TAG_CONTROL_AF_MODE, 1, &value)
        })?;
        self.submit()
    }

    /// `CONTROL_AE_TARGET_FPS_RANGE`. V4L2 `S_PARM` carries a single interval, so a capture device
    /// would pin both ends to the rate the guest asked for.
    pub fn set_fps_range(&mut self, min: i32, max: i32) -> Result<()> {
        let range = [min, max];
        // SAFETY: request is live; the pointer addresses two i32, matching count 2.
        check("ACaptureRequest_setEntry_i32(AE_TARGET_FPS_RANGE)", unsafe {
            ACaptureRequest_setEntry_i32(
                self.request,
                TAG_CONTROL_AE_TARGET_FPS_RANGE,
                2,
                range.as_ptr(),
            )
        })?;
        self.submit()
    }

    /// How many times the image listener has fired since open.
    pub fn frames_signalled(&self) -> u64 {
        self.signal.delivered.load(Ordering::Acquire)
    }

    /// Wait up to `timeout` for the next frame. `Ok(None)` means the deadline passed with the
    /// camera still running, which is a stall rather than an error.
    pub fn next_frame(&self, timeout: Duration) -> Result<Option<Frame<'_>>> {
        let deadline = Instant::now() + timeout;
        loop {
            let mut image: *mut AImage = null_mut();
            // SAFETY: reader is live; image is written only when a buffer was available.
            let status = unsafe { AImageReader_acquireNextImage(self.reader, &mut image) };
            match status {
                AMEDIA_OK => return Frame::new(image).map(Some),
                // Not an error: the queue is empty until the camera produces the next frame.
                AMEDIA_IMGREADER_NO_BUFFER_AVAILABLE => {}
                other => return Err(CameraError::Ndk("AImageReader_acquireNextImage", other, "AMEDIA")),
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
