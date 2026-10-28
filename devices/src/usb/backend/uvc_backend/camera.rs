// Copyright 2026 The crosvm Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! Camera2 access for the virtual UVC camera.
//!
//! crosvm runs as root (KernelSU domain `u:r:ksu:s0`), which is allowed to talk
//! to the Android camera service, so the frames can come straight from the
//! phone's camera instead of a built in test picture.
//!
//! The NDK camera stack (`libcamera2ndk`, `libmediandk`, `libandroid`) is
//! loaded with `dlopen` instead of being linked, so that the crosvm build does
//! not need those modules in its `shared_libs` list. Everything here is a
//! best-effort probe for now: it reports how far it gets and writes the result
//! to `/data/local/tmp/camprobe.log`.

#![allow(non_camel_case_types)]

use std::ffi::CString;
use std::os::raw::c_char;
use std::os::raw::c_int;
use std::os::raw::c_void;

use base::info;

const RTLD_NOW: c_int = 2;
/// `AIMAGE_FORMAT_YUV_420_888`
const IMAGE_FORMAT_YUV_420_888: c_int = 0x23;
/// `AIMAGE_FORMAT_JPEG`
const IMAGE_FORMAT_JPEG: c_int = 0x100;
/// Template id of `ACAMERA_TEMPLATE_PREVIEW`
const TEMPLATE_PREVIEW: c_int = 1;
/// `ACAMERA_CONTROL_AE_MODE` tag
const ACAMERA_CONTROL_AE_MODE: u32 = 0x0100_0003;
/// `ACAMERA_CONTROL_AE_TARGET_FPS_RANGE` tag
const ACAMERA_CONTROL_AE_TARGET_FPS_RANGE: u32 = 0x0100_0004;
/// `ACAMERA_JPEG_THUMBNAIL_SIZE` tag
const ACAMERA_JPEG_THUMBNAIL_SIZE: u32 = 0x0500_0000;
/// `ACAMERA_JPEG_QUALITY` tag
const ACAMERA_JPEG_QUALITY: u32 = 0x0500_0002;
/// JPEG quality requested from the camera HAL: high quality frames cost the
/// guest a lot of CPU to decode and slow the bulk stream down.
const HW_JPEG_QUALITY: i32 = 65;

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
        let ptr = unsafe { dlsym(self.handle, cname.as_ptr()) };
        if ptr.is_null() {
            None
        } else {
            Some(ptr)
        }
    }
}

macro_rules! bind {
    ($lib:expr, $out:ident, $name:literal, $ty:ty) => {
        let $out: $ty = {
            let ptr = $lib
                .sym($name)
                .ok_or_else(|| format!("missing symbol {}", $name))?;
            // SAFETY: the symbol exists and its declared signature matches the
            // NDK C API.
            unsafe { std::mem::transmute::<*mut c_void, $ty>(ptr) }
        };
    };
}

// Opaque handles.
type ACameraManager = c_void;
type ACameraDevice = c_void;
type ACaptureSessionOutputContainer = c_void;
type ACaptureSessionOutput = c_void;
type ACameraOutputTarget = c_void;
type ACaptureRequest = c_void;
type ACameraCaptureSession = c_void;
type AImageReader = c_void;
type ANativeWindow = c_void;

#[repr(C)]
struct ACameraIdList {
    num_cameras: c_int,
    camera_ids: *mut *const c_char,
}

// The callback structs below are passed by reference only, so they are given a
// little extra room: the NDK structs only grow by appending fields.
#[repr(C)]
struct DeviceStateCallbacks {
    context: *mut c_void,
    on_disconnected: Option<extern "C" fn(*mut c_void, *mut ACameraDevice)>,
    on_error: Option<extern "C" fn(*mut c_void, *mut ACameraDevice, c_int)>,
}

#[repr(C)]
struct SessionStateCallbacks {
    context: *mut c_void,
    on_ready: Option<extern "C" fn(*mut c_void, *mut ACameraCaptureSession)>,
    on_active: Option<extern "C" fn(*mut c_void, *mut ACameraCaptureSession)>,
    on_closed: Option<extern "C" fn(*mut c_void, *mut ACameraCaptureSession)>,
}

#[repr(C)]
struct SessionCaptureCallbacks {
    context: *mut c_void,
    on_started: Option<extern "C" fn(*mut c_void, *mut ACameraCaptureSession, *mut ACaptureRequest, u64)>,
    on_progress: Option<extern "C" fn(*mut c_void, *mut ACameraCaptureSession, *mut ACaptureRequest, i32)>,
    on_sequence_completed: Option<extern "C" fn(*mut c_void, *mut ACameraCaptureSession, *mut ACaptureRequest)>,
    on_buffer_lost: Option<extern "C" fn(*mut c_void, *mut ACameraCaptureSession, *mut ACaptureRequest, *mut c_void, i32)>,
}

#[repr(C)]
struct ImageReaderListener {
    context: *mut c_void,
    on_image_available: Option<extern "C" fn(*mut c_void, *mut AImageReader)>,
}

#[repr(C)]
struct AImage {
    _private: [u8; 0],
}

#[allow(dead_code)]
extern "C" fn on_camera_disconnected(_context: *mut c_void, _device: *mut ACameraDevice) {
    info!("uvc: camera disconnected");
}

#[allow(dead_code)]
extern "C" fn on_camera_error(_context: *mut c_void, _device: *mut ACameraDevice, error: c_int) {
    info!("uvc: camera error {error}");
}

#[allow(dead_code)]
extern "C" fn on_session_ready(_context: *mut c_void, _session: *mut ACameraCaptureSession) {
    info!("uvc: capture session ready");
}

#[allow(dead_code)]
extern "C" fn on_image_available(_context: *mut c_void, _reader: *mut AImageReader) {
    info!("uvc: image available");
}

/// Everything the probe needs, resolved from the NDK camera stack.
struct CameraApi {
    _camera_lib: Lib,
    _media_lib: Lib,
    _android_lib: Lib,

    manager_create: unsafe extern "C" fn() -> *mut ACameraManager,
    manager_delete: unsafe extern "C" fn(*mut ACameraManager),
    get_camera_id_list:
        unsafe extern "C" fn(*mut ACameraManager, *mut *mut ACameraIdList) -> c_int,
    delete_camera_id_list: unsafe extern "C" fn(*mut ACameraIdList),
    open_camera: unsafe extern "C" fn(
        *mut ACameraManager,
        *const c_char,
        *const DeviceStateCallbacks,
        *mut *mut ACameraDevice,
    ) -> c_int,
    device_close: unsafe extern "C" fn(*mut ACameraDevice) -> c_int,
    create_capture_session: unsafe extern "C" fn(
        *mut ACameraDevice,
        *const ACaptureSessionOutputContainer,
        *const SessionStateCallbacks,
        *mut *mut ACameraCaptureSession,
    ) -> c_int,
    session_output_container_create:
        unsafe extern "C" fn(*mut *mut ACaptureSessionOutputContainer) -> c_int,
    session_output_container_add:
        unsafe extern "C" fn(*mut ACaptureSessionOutputContainer, *const ACaptureSessionOutput) -> c_int,
    session_output_create: unsafe extern "C" fn(
        *mut ANativeWindow,
        *mut *mut ACaptureSessionOutput,
    ) -> c_int,
    output_target_create:
        unsafe extern "C" fn(*mut ANativeWindow, *mut *mut ACameraOutputTarget) -> c_int,
    device_create_capture_request:
        unsafe extern "C" fn(*mut ACameraDevice, c_int, *mut *mut ACaptureRequest) -> c_int,
    capture_request_add_target:
        unsafe extern "C" fn(*mut ACaptureRequest, *const ACameraOutputTarget) -> c_int,
    capture_request_set_entry_u8:
        unsafe extern "C" fn(*mut ACaptureRequest, u32, c_int, *const u8) -> c_int,
    capture_request_set_entry_i32:
        unsafe extern "C" fn(*mut ACaptureRequest, u32, c_int, *const i32) -> c_int,
    set_repeating_request: unsafe extern "C" fn(
        *mut ACameraCaptureSession,
        *const SessionCaptureCallbacks,
        c_int,
        *mut *mut ACaptureRequest,
        *mut c_int,
    ) -> c_int,
    session_stop_repeating: unsafe extern "C" fn(*mut ACameraCaptureSession) -> c_int,
    session_close: unsafe extern "C" fn(*mut ACameraCaptureSession),
    session_output_container_free: unsafe extern "C" fn(*mut ACaptureSessionOutputContainer),
    session_output_free: unsafe extern "C" fn(*mut ACaptureSessionOutput),
    output_target_free: unsafe extern "C" fn(*mut ACameraOutputTarget),
    capture_request_free: unsafe extern "C" fn(*mut ACaptureRequest),
    image_reader_delete: unsafe extern "C" fn(*mut AImageReader),

    image_reader_new:
        unsafe extern "C" fn(c_int, c_int, c_int, c_int, *mut *mut AImageReader) -> c_int,
    image_reader_get_window:
        unsafe extern "C" fn(*mut AImageReader, *mut *mut ANativeWindow) -> c_int,
    image_reader_set_listener:
        unsafe extern "C" fn(*mut AImageReader, *const ImageReaderListener) -> c_int,
    image_reader_acquire_latest: unsafe extern "C" fn(*mut AImageReader, *mut *mut AImage) -> c_int,
    image_get_number_of_planes: unsafe extern "C" fn(*mut AImage, *mut c_int) -> c_int,
    image_get_plane_data:
        unsafe extern "C" fn(*mut AImage, c_int, *mut *mut u8, *mut c_int) -> c_int,
    image_get_plane_row_stride: unsafe extern "C" fn(*mut AImage, c_int, *mut c_int) -> c_int,
    image_get_plane_pixel_stride: unsafe extern "C" fn(*mut AImage, c_int, *mut c_int) -> c_int,
    image_get_width: unsafe extern "C" fn(*mut AImage, *mut c_int) -> c_int,
    image_get_height: unsafe extern "C" fn(*mut AImage, *mut c_int) -> c_int,
    image_delete: unsafe extern "C" fn(*mut AImage),
}

impl CameraApi {
    fn load() -> Result<CameraApi, String> {
        let camera_lib =
            Lib::open("libcamera2ndk.so").ok_or_else(|| "dlopen libcamera2ndk.so failed".to_string())?;
        let media_lib =
            Lib::open("libmediandk.so").ok_or_else(|| "dlopen libmediandk.so failed".to_string())?;
        let android_lib =
            Lib::open("libandroid.so").ok_or_else(|| "dlopen libandroid.so failed".to_string())?;

        bind!(camera_lib, manager_create, "ACameraManager_create", unsafe extern "C" fn() -> *mut ACameraManager);
        bind!(camera_lib, manager_delete, "ACameraManager_delete", unsafe extern "C" fn(*mut ACameraManager));
        bind!(camera_lib, get_camera_id_list, "ACameraManager_getCameraIdList", unsafe extern "C" fn(*mut ACameraManager, *mut *mut ACameraIdList) -> c_int);
        bind!(camera_lib, delete_camera_id_list, "ACameraManager_deleteCameraIdList", unsafe extern "C" fn(*mut ACameraIdList));
        bind!(camera_lib, open_camera, "ACameraManager_openCamera", unsafe extern "C" fn(*mut ACameraManager, *const c_char, *const DeviceStateCallbacks, *mut *mut ACameraDevice) -> c_int);
        bind!(camera_lib, device_close, "ACameraDevice_close", unsafe extern "C" fn(*mut ACameraDevice) -> c_int);
        bind!(camera_lib, create_capture_session, "ACameraDevice_createCaptureSession", unsafe extern "C" fn(*mut ACameraDevice, *const ACaptureSessionOutputContainer, *const SessionStateCallbacks, *mut *mut ACameraCaptureSession) -> c_int);
        bind!(camera_lib, session_output_container_create, "ACaptureSessionOutputContainer_create", unsafe extern "C" fn(*mut *mut ACaptureSessionOutputContainer) -> c_int);
        bind!(camera_lib, session_output_container_add, "ACaptureSessionOutputContainer_add", unsafe extern "C" fn(*mut ACaptureSessionOutputContainer, *const ACaptureSessionOutput) -> c_int);
        bind!(camera_lib, session_output_create, "ACaptureSessionOutput_create", unsafe extern "C" fn(*mut ANativeWindow, *mut *mut ACaptureSessionOutput) -> c_int);
        bind!(camera_lib, output_target_create, "ACameraOutputTarget_create", unsafe extern "C" fn(*mut ANativeWindow, *mut *mut ACameraOutputTarget) -> c_int);
        bind!(camera_lib, device_create_capture_request, "ACameraDevice_createCaptureRequest", unsafe extern "C" fn(*mut ACameraDevice, c_int, *mut *mut ACaptureRequest) -> c_int);
        bind!(camera_lib, capture_request_add_target, "ACaptureRequest_addTarget", unsafe extern "C" fn(*mut ACaptureRequest, *const ACameraOutputTarget) -> c_int);
        bind!(camera_lib, capture_request_set_entry_u8, "ACaptureRequest_setEntry_u8", unsafe extern "C" fn(*mut ACaptureRequest, u32, c_int, *const u8) -> c_int);
bind!(camera_lib, capture_request_set_entry_i32, "ACaptureRequest_setEntry_i32", unsafe extern "C" fn(*mut ACaptureRequest, u32, c_int, *const i32) -> c_int);
        bind!(camera_lib, set_repeating_request, "ACameraCaptureSession_setRepeatingRequest", unsafe extern "C" fn(*mut ACameraCaptureSession, *const SessionCaptureCallbacks, c_int, *mut *mut ACaptureRequest, *mut c_int) -> c_int);
        bind!(camera_lib, session_stop_repeating, "ACameraCaptureSession_stopRepeating", unsafe extern "C" fn(*mut ACameraCaptureSession) -> c_int);
        bind!(camera_lib, session_close, "ACameraCaptureSession_close", unsafe extern "C" fn(*mut ACameraCaptureSession));
        bind!(camera_lib, session_output_container_free, "ACaptureSessionOutputContainer_free", unsafe extern "C" fn(*mut ACaptureSessionOutputContainer));
        bind!(camera_lib, session_output_free, "ACaptureSessionOutput_free", unsafe extern "C" fn(*mut ACaptureSessionOutput));
        bind!(camera_lib, output_target_free, "ACameraOutputTarget_free", unsafe extern "C" fn(*mut ACameraOutputTarget));
        bind!(camera_lib, capture_request_free, "ACaptureRequest_free", unsafe extern "C" fn(*mut ACaptureRequest));
        bind!(media_lib, image_reader_delete, "AImageReader_delete", unsafe extern "C" fn(*mut AImageReader));

        bind!(media_lib, image_reader_new, "AImageReader_new", unsafe extern "C" fn(c_int, c_int, c_int, c_int, *mut *mut AImageReader) -> c_int);
        bind!(media_lib, image_reader_get_window, "AImageReader_getWindow", unsafe extern "C" fn(*mut AImageReader, *mut *mut ANativeWindow) -> c_int);
        bind!(media_lib, image_reader_set_listener, "AImageReader_setImageListener", unsafe extern "C" fn(*mut AImageReader, *const ImageReaderListener) -> c_int);
        bind!(media_lib, image_reader_acquire_latest, "AImageReader_acquireLatestImage", unsafe extern "C" fn(*mut AImageReader, *mut *mut AImage) -> c_int);
        bind!(media_lib, image_get_number_of_planes, "AImage_getNumberOfPlanes", unsafe extern "C" fn(*mut AImage, *mut c_int) -> c_int);
        bind!(media_lib, image_get_plane_data, "AImage_getPlaneData", unsafe extern "C" fn(*mut AImage, c_int, *mut *mut u8, *mut c_int) -> c_int);
        bind!(media_lib, image_get_plane_row_stride, "AImage_getPlaneRowStride", unsafe extern "C" fn(*mut AImage, c_int, *mut c_int) -> c_int);
        bind!(media_lib, image_get_plane_pixel_stride, "AImage_getPlanePixelStride", unsafe extern "C" fn(*mut AImage, c_int, *mut c_int) -> c_int);
        bind!(media_lib, image_get_width, "AImage_getWidth", unsafe extern "C" fn(*mut AImage, *mut c_int) -> c_int);
        bind!(media_lib, image_get_height, "AImage_getHeight", unsafe extern "C" fn(*mut AImage, *mut c_int) -> c_int);
        bind!(media_lib, image_delete, "AImage_delete", unsafe extern "C" fn(*mut AImage));

        // Keep the libraries alive for the lifetime of the API object.
        let _ = &android_lib;

        Ok(CameraApi {
            _camera_lib: camera_lib,
            _media_lib: media_lib,
            _android_lib: android_lib,
            manager_create,
            manager_delete,
            get_camera_id_list,
            delete_camera_id_list,
            open_camera,
            device_close,
            create_capture_session,
            session_output_container_create,
            session_output_container_add,
            session_output_create,
            output_target_create,
            device_create_capture_request,
            capture_request_add_target,
            capture_request_set_entry_u8,
            capture_request_set_entry_i32,
            set_repeating_request,
            session_stop_repeating,
            session_close,
            session_output_container_free,
            session_output_free,
            output_target_free,
            capture_request_free,
            image_reader_delete,
            image_reader_new,
            image_reader_get_window,
            image_reader_set_listener,
            image_reader_acquire_latest,
            image_get_number_of_planes,
            image_get_plane_data,
            image_get_plane_row_stride,
            image_get_plane_pixel_stride,
            image_get_width,
            image_get_height,
            image_delete,
        })
    }
}

/// Copies the planes of `image` to `/data/local/tmp/camprobe_frame.bin` and adds
/// the plane geometry to the report.
fn dump_image(api: &CameraApi, image: *mut AImage, report: &mut String) {
    // SAFETY: `image` is a live image that we own.
    unsafe {
        let mut planes: c_int = 0;
        let status = (api.image_get_number_of_planes)(image, &mut planes);
        report.push_str(&format!("planes: status={status} n={planes}\n"));
        let mut raw: Vec<u8> = Vec::new();
        for plane in 0..planes.max(0) {
            let mut data: *mut u8 = std::ptr::null_mut();
            let mut len: c_int = 0;
            let status = (api.image_get_plane_data)(image, plane, &mut data, &mut len);
            let mut row_stride: c_int = 0;
            let mut pixel_stride: c_int = 0;
            (api.image_get_plane_row_stride)(image, plane, &mut row_stride);
            (api.image_get_plane_pixel_stride)(image, plane, &mut pixel_stride);
            report.push_str(&format!(
                "plane{plane}: status={status} len={len} row_stride={row_stride} \
                 pixel_stride={pixel_stride}\n"
            ));
            if status == 0 && !data.is_null() && len > 0 {
                raw.extend_from_slice(std::slice::from_raw_parts(data, len as usize));
            }
        }
        if raw.is_empty() {
            report.push_str("frame: empty\n");
        } else {
            match std::fs::write("/data/local/tmp/camprobe_frame.bin", &raw) {
                Ok(()) => report.push_str(&format!("frame: {} bytes dumped\n", raw.len())),
                Err(e) => report.push_str(&format!("frame: write failed: {e}\n")),
            }
        }
    }
}

/// Runs the camera probe and returns a human readable report.
#[allow(dead_code)]
fn run_probe() -> String {
    let mut report = String::new();
    let api = match CameraApi::load() {
        Ok(api) => api,
        Err(e) => {
            report.push_str(&format!("load: {e}\n"));
            return report;
        }
    };
    report.push_str("load: ok\n");

    // SAFETY: all pointers below come from the NDK API and are used with the
    // matching functions.
    unsafe {
        let manager = (api.manager_create)();
        if manager.is_null() {
            report.push_str("manager: null\n");
            return report;
        }
        report.push_str("manager: ok\n");

        let mut id_list: *mut ACameraIdList = std::ptr::null_mut();
        let status = (api.get_camera_id_list)(manager, &mut id_list);
        if status != 0 || id_list.is_null() {
            report.push_str(&format!("getCameraIdList: status={status}\n"));
            (api.manager_delete)(manager);
            return report;
        }
        let num = (*id_list).num_cameras;
        let mut ids: Vec<String> = Vec::new();
        for i in 0..num.max(0) as usize {
            let ptr = *(*id_list).camera_ids.add(i);
            if ptr.is_null() {
                continue;
            }
            let id = std::ffi::CStr::from_ptr(ptr).to_string_lossy().into_owned();
            ids.push(id);
        }
        report.push_str(&format!("cameras: {num} {ids:?}\n"));
        (api.delete_camera_id_list)(id_list);

        if ids.is_empty() {
            (api.manager_delete)(manager);
            return report;
        }

        let first = CString::new(ids[0].clone()).unwrap();
        let callbacks = DeviceStateCallbacks {
            context: std::ptr::null_mut(),
            on_disconnected: None,
            on_error: None,
        };
        let mut device: *mut ACameraDevice = std::ptr::null_mut();
        let status = (api.open_camera)(manager, first.as_ptr(), &callbacks, &mut device);
        report.push_str(&format!("openCamera({}): status={status}\n", ids[0]));
        if status != 0 || device.is_null() {
            (api.manager_delete)(manager);
            return report;
        }

        // Build an AImageReader that the camera can write into.
        let mut reader: *mut AImageReader = std::ptr::null_mut();
        let status = (api.image_reader_new)(640, 480, IMAGE_FORMAT_YUV_420_888, 8, &mut reader);
        report.push_str(&format!("AImageReader_new: status={status}\n"));
        if status != 0 || reader.is_null() {
            (api.device_close)(device);
            (api.manager_delete)(manager);
            return report;
        }

        let mut window: *mut ANativeWindow = std::ptr::null_mut();
        let status = (api.image_reader_get_window)(reader, &mut window);
        report.push_str(&format!("AImageReader_getWindow: status={status}\n"));
        if status != 0 || window.is_null() {
            (api.device_close)(device);
            (api.manager_delete)(manager);
            return report;
        }

        let listener = ImageReaderListener {
            context: std::ptr::null_mut(),
            on_image_available: None,
        };
        let status = (api.image_reader_set_listener)(reader, &listener);
        report.push_str(&format!("setImageListener: status={status}\n"));

        // Register the reader's window as the capture output.
        let mut container: *mut ACaptureSessionOutputContainer = std::ptr::null_mut();
        let status = (api.session_output_container_create)(&mut container);
        report.push_str(&format!("OutputContainer_create: status={status}\n"));

        let mut session_output: *mut ACaptureSessionOutput = std::ptr::null_mut();
        let status = (api.session_output_create)(window, &mut session_output);
        report.push_str(&format!("SessionOutput_create: status={status}\n"));
        if status == 0 && !container.is_null() {
            let status = (api.session_output_container_add)(container, session_output);
            report.push_str(&format!("OutputContainer_add: status={status}\n"));
        }

        let session_callbacks = SessionStateCallbacks {
            context: std::ptr::null_mut(),
            on_ready: None,
            on_active: None,
            on_closed: None,
        };
        let mut session: *mut ACameraCaptureSession = std::ptr::null_mut();
        let status = (api.create_capture_session)(device, container, &session_callbacks, &mut session);
        report.push_str(&format!("createCaptureSession: status={status}\n"));

        if status == 0 && !session.is_null() {
            let mut target: *mut ACameraOutputTarget = std::ptr::null_mut();
            let status = (api.output_target_create)(window, &mut target);
            report.push_str(&format!("OutputTarget_create: status={status}\n"));

            let mut request: *mut ACaptureRequest = std::ptr::null_mut();
            let status = (api.device_create_capture_request)(device, TEMPLATE_PREVIEW, &mut request);
            report.push_str(&format!("CaptureRequest_create: status={status}\n"));

            if status == 0 && !request.is_null() {
                let status = (api.capture_request_add_target)(request, target);
                report.push_str(&format!("CaptureRequest_addTarget: status={status}\n"));
                let auto: u8 = 1;
                let status =
                    (api.capture_request_set_entry_u8)(request, ACAMERA_CONTROL_AE_MODE, 1, &auto);
                report.push_str(&format!("set AE_MODE: status={status}\n"));

                let capture_callbacks = SessionCaptureCallbacks {
                    context: std::ptr::null_mut(),
                    on_started: None,
                    on_progress: None,
                    on_sequence_completed: None,
                    on_buffer_lost: None,
                };
                let mut sequence_id: c_int = 0;
                let status = (api.set_repeating_request)(
                    session,
                    &capture_callbacks,
                    1,
                    &mut request,
                    &mut sequence_id,
                );
                report.push_str(&format!(
                    "setRepeatingRequest: status={status} seq={sequence_id}\n"
                ));

                // Give the pipeline a moment, then try to grab one frame.
                let mut image: *mut AImage = std::ptr::null_mut();
                let mut attempts = 0;
                let mut last_status = -1;
                while attempts < 40 {
                    std::thread::sleep(std::time::Duration::from_millis(100));
                    attempts += 1;
                    last_status = (api.image_reader_acquire_latest)(reader, &mut image);
                    if last_status == 0 && !image.is_null() {
                        break;
                    }
                }
                report.push_str(&format!(
                    "acquireLatestImage: last_status={last_status} attempts={attempts}\n"
                ));
                if !image.is_null() {
                    let mut width: c_int = 0;
                    let mut height: c_int = 0;
                    (api.image_get_width)(image, &mut width);
                    (api.image_get_height)(image, &mut height);
                    report.push_str(&format!("image: {width}x{height}\n"));
                    dump_image(&api, image, &mut report);
                    (api.image_delete)(image);
                }
            }
        }

        (api.device_close)(device);
        (api.manager_delete)(manager);
    }

    report
}

/// Reads a tuning knob from the environment, then from a one-line file.
///
/// The file fallback exists so the camera can be switched (front/back) or the
/// frame-rate cap changed on a running device without rebuilding the APK: the
/// app's crosvm process reads `/data/local/tmp/droidvm-uvc-*` at stream start.
fn setting(env_name: &str, path: &str) -> Option<String> {
    if let Ok(value) = std::env::var(env_name) {
        let value = value.trim().to_string();
        if !value.is_empty() {
            return Some(value);
        }
    }
    std::fs::read_to_string(path)
        .ok()
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
}

/// JPEG quality used for the MJPEG stream.
const JPEG_QUALITY: u8 = 75;
/// Preferred capture resolution, must match the UVC frame descriptor.
const CAPTURE_WIDTH: c_int = 1920;
const CAPTURE_HEIGHT: c_int = 1080;

/// How many host cameras one VM can expose as UVC devices.
pub const CAMERA_SLOTS: usize = 2;

/// Newest JPEG frame produced by each capture loop.
static LATEST_FRAME: [std::sync::Mutex<Option<Vec<u8>>>; CAMERA_SLOTS] =
    [const { std::sync::Mutex::new(None) }; CAMERA_SLOTS];
/// Counter that grows with every captured frame, so that the UVC device can tell
/// a new picture from the one it already handed to the guest.
static FRAME_SEQ: [std::sync::atomic::AtomicU64; CAMERA_SLOTS] =
    [const { std::sync::atomic::AtomicU64::new(0) }; CAMERA_SLOTS];

/// Clamps a slot index coming from outside this module.
fn camera_slot_of(camera_slot: usize) -> usize {
    camera_slot.min(CAMERA_SLOTS - 1)
}

/// Returns a copy of the newest frame of `camera_slot` with its sequence number.
pub fn take_latest_frame(camera_slot: usize) -> Option<(u64, Vec<u8>)> {
    let index = camera_slot_of(camera_slot);
    let mut slot = LATEST_FRAME[index].lock().ok()?;
    let frame = slot.clone()?;
    let seq = FRAME_SEQ[index].load(std::sync::atomic::Ordering::Relaxed);
    Some((seq, frame))
}

fn store_frame(camera_slot: usize, frame: Vec<u8>) {
    let index = camera_slot_of(camera_slot);
    if let Ok(mut slot) = LATEST_FRAME[index].lock() {
        *slot = Some(frame);
        FRAME_SEQ[index].fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    }
}

fn clear_frame(camera_slot: usize) {
    let index = camera_slot_of(camera_slot);
    if let Ok(mut slot) = LATEST_FRAME[index].lock() {
        *slot = None;
    }
}

/// Whether the guest asked for video (UVC commit control) and is still pulling
/// frames. The camera stays closed unless both are true, so the phone keeps the
/// camera to itself while Windows is not using it.
static STREAM_WANTED: [std::sync::atomic::AtomicBool; CAMERA_SLOTS] =
    [const { std::sync::atomic::AtomicBool::new(false) }; CAMERA_SLOTS];
/// Last time the guest requested a frame from the bulk endpoint.
static LAST_ACTIVITY: [std::sync::Mutex<Option<std::time::Instant>>; CAMERA_SLOTS] =
    [const { std::sync::Mutex::new(None) }; CAMERA_SLOTS];

/// Called by the UVC device when the guest starts or stops streaming.
pub fn set_streaming(camera_slot: usize, on: bool) {
    use std::sync::atomic::Ordering;
    let index = camera_slot_of(camera_slot);
    STREAM_WANTED[index].store(on, Ordering::Relaxed);
    if on {
        touch_activity(camera_slot);
    } else {
        append_log(&format!("capture: camera {camera_slot} stopped streaming"));
    }
}

/// Called for every bulk IN request so an idle host releases the camera.
pub fn touch_activity(camera_slot: usize) {
    let index = camera_slot_of(camera_slot);
    if let Ok(mut slot) = LAST_ACTIVITY[index].lock() {
        *slot = Some(std::time::Instant::now());
    }
}

fn streaming_wanted(camera_slot: usize) -> bool {
    use std::sync::atomic::Ordering;
    let index = camera_slot_of(camera_slot);
    if !STREAM_WANTED[index].load(Ordering::Relaxed) {
        return false;
    }
    match LAST_ACTIVITY[index].lock().ok().and_then(|slot| *slot) {
        Some(at) => at.elapsed() < std::time::Duration::from_secs(3),
        None => false,
    }
}

fn append_log(line: &str) {
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

/// Copies one `AImage` plane into a tightly packed buffer.
///
/// # Safety
///
/// `image` must be a live `AImage` and `api` must have been loaded successfully.
unsafe fn copy_plane(
    api: &CameraApi,
    image: *mut AImage,
    index: c_int,
    width: usize,
    height: usize,
) -> Option<Vec<u8>> {
    let mut data: *mut u8 = std::ptr::null_mut();
    let mut length: c_int = 0;
    if (api.image_get_plane_data)(image, index, &mut data, &mut length) != 0 || data.is_null() {
        return None;
    }
    let mut row_stride: c_int = 0;
    let mut pixel_stride: c_int = 0;
    (api.image_get_plane_row_stride)(image, index, &mut row_stride);
    (api.image_get_plane_pixel_stride)(image, index, &mut pixel_stride);
    if row_stride <= 0 || pixel_stride <= 0 || length <= 0 {
        return None;
    }
    let row_stride = row_stride as usize;
    let pixel_stride = pixel_stride as usize;
    let source = std::slice::from_raw_parts(data, length as usize);
    if pixel_stride == 1 && row_stride == width && source.len() >= width * height {
        return Some(source[..width * height].to_vec());
    }
    let mut packed = vec![0u8; width * height];
    for y in 0..height {
        for x in 0..width {
            let offset = y * row_stride + x * pixel_stride;
            if offset < source.len() {
                packed[y * width + x] = source[offset];
            }
        }
    }
    Some(packed)
}

/// Encodes one camera image as JPEG.
///
/// Copies the single JPEG plane of a hardware encoded image.
///
/// # Safety
///
/// `image` must be a live `AImage` and `api` must have been loaded successfully.
unsafe fn image_jpeg_bytes(api: &CameraApi, image: *mut AImage) -> Option<Vec<u8>> {
    let mut data: *mut u8 = std::ptr::null_mut();
    let mut length: c_int = 0;
    if (api.image_get_plane_data)(image, 0, &mut data, &mut length) != 0
        || data.is_null()
        || length <= 0
    {
        return None;
    }
    Some(std::slice::from_raw_parts(data, length as usize).to_vec())
}

/// Drops the metadata segments a camera HAL glues in front of the actual JPEG
/// data. This device fills roughly 94% of every frame with a huge EXIF blob
/// (nearly a megabyte at 720p), which is pointless to ship over USB.
fn slim_jpeg(data: &[u8]) -> Vec<u8> {
    if data.len() < 4 || data[0] != 0xFF || data[1] != 0xD8 {
        return data.to_vec();
    }
    let mut out = Vec::with_capacity(data.len());
    out.extend_from_slice(&data[0..2]);
    let mut index = 2;
    while index + 3 < data.len() {
        if data[index] != 0xFF {
            // Not a segment boundary: keep whatever is left untouched.
            out.extend_from_slice(&data[index..]);
            return out;
        }
        let marker = data[index + 1];
        if marker == 0xDA {
            // Start of scan: the rest is entropy coded image data.
            out.extend_from_slice(&data[index..]);
            return out;
        }
        let length = ((data[index + 2] as usize) << 8) | data[index + 3] as usize;
        if length < 2 || index + 2 + length > data.len() {
            return data.to_vec();
        }
        // APP0-APP15 and COM carry metadata only; everything else is needed by
        // the decoder.
        if !matches!(marker, 0xE0..=0xEF | 0xFE) {
            out.extend_from_slice(&data[index..index + 2 + length]);
        }
        index += 2 + length;
    }
    out
}

/// # Safety
///
/// `image` must be a live `AImage` and `api` must have been loaded successfully.
unsafe fn image_to_jpeg(api: &CameraApi, image: *mut AImage, log_stats: bool) -> Option<Vec<u8>> {
    let mut width: c_int = 0;
    let mut height: c_int = 0;
    (api.image_get_width)(image, &mut width);
    (api.image_get_height)(image, &mut height);
    if width <= 0 || height <= 0 {
        return None;
    }
    let width = width as usize;
    let height = height as usize;
    let y = copy_plane(api, image, 0, width, height)?;
    let u = copy_plane(api, image, 1, width / 2, height / 2)?;
    let v = copy_plane(api, image, 2, width / 2, height / 2)?;
    if log_stats {
        let sum: u64 = y.iter().step_by(101).map(|&value| value as u64).sum();
        let count = y.iter().step_by(101).count().max(1) as u64;
        let min = y.iter().copied().min().unwrap_or(0);
        let max = y.iter().copied().max().unwrap_or(0);
        let first_u = u.first().copied().unwrap_or(0);
        append_log(&format!(
            "capture: y mean={} min={min} max={max} u0={first_u} size={}x{}",
            sum / count,
            width,
            height
        ));
    }
    crate::usb::backend::uvc_backend::jpeg::encode_yuv420(
        width,
        height,
        JPEG_QUALITY,
        &y,
        width,
        &u,
        width / 2,
        &v,
        width / 2,
    )
}

/// Streams camera frames through the hardware JPEG encoder.
///
/// Falls back to the caller when the session cannot be created, which keeps the
/// software path available on devices without a usable encoder.
#[allow(dead_code)]
fn stream_with_encoder(
    camera_slot: usize,
    api: &CameraApi,
    device: *mut ACameraDevice,
    mut encoder: crate::usb::backend::uvc_backend::mediacodec::JpegEncoder,
) {
    let window = encoder.surface() as *mut ANativeWindow;
    if window.is_null() {
        append_log("encoder: surface is null");
        return;
    }

    // SAFETY: every NDK call below is made with live handles that this function
    // owns; failures are reported through status codes.
    unsafe {
    let mut container: *mut ACaptureSessionOutputContainer = std::ptr::null_mut();
    let status = (api.session_output_container_create)(&mut container);
    if status != 0 || container.is_null() {
        append_log(&format!("encoder: container create failed status={status}"));
        return;
    }
    let mut session_output: *mut ACaptureSessionOutput = std::ptr::null_mut();
    let status = (api.session_output_create)(window, &mut session_output);
    if status != 0 || session_output.is_null() {
        append_log(&format!("encoder: SessionOutput_create failed status={status}"));
        return;
    }
    (api.session_output_container_add)(container, session_output);

    let session_callbacks = SessionStateCallbacks {
        context: std::ptr::null_mut(),
        on_ready: None,
        on_active: None,
        on_closed: None,
    };
    let mut session: *mut ACameraCaptureSession = std::ptr::null_mut();
    let status = (api.create_capture_session)(device, container, &session_callbacks, &mut session);
    if status != 0 || session.is_null() {
        append_log(&format!("encoder: createCaptureSession failed status={status}"));
        return;
    }
    append_log("encoder: capture session created");

    let mut target: *mut ACameraOutputTarget = std::ptr::null_mut();
    let status = (api.output_target_create)(window, &mut target);
    if status != 0 || target.is_null() {
        append_log(&format!("encoder: OutputTarget_create failed status={status}"));
        return;
    }

    let mut request: *mut ACaptureRequest = std::ptr::null_mut();
    let status = (api.device_create_capture_request)(device, TEMPLATE_PREVIEW, &mut request);
    if status != 0 || request.is_null() {
        append_log(&format!("encoder: createCaptureRequest failed status={status}"));
        return;
    }
    (api.capture_request_add_target)(request, target);

    let capture_callbacks = SessionCaptureCallbacks {
        context: std::ptr::null_mut(),
        on_started: None,
        on_progress: None,
        on_sequence_completed: None,
        on_buffer_lost: None,
    };
    let mut sequence_id: c_int = 0;
    let status = (api.set_repeating_request)(session, &capture_callbacks, 1, &mut request, &mut sequence_id);
    if status != 0 {
        append_log(&format!("encoder: setRepeatingRequest failed status={status}"));
        return;
    }
    append_log("encoder: repeating request started");
    }

    let mut frames = 0u64;
    let mut idle_since = std::time::Instant::now();
    loop {
        match encoder.poll() {
            Some(jpeg) => {
                let length = jpeg.len();
                if frames == 0 {
                    append_log(&format!("encoder: first frame jpeg={length} bytes"));
                }
                store_frame(camera_slot, jpeg);
                frames += 1;
                idle_since = std::time::Instant::now();
                if frames % 300 == 0 {
                    append_log(&format!("encoder: frame {frames} jpeg={length} bytes"));
                }
            }
            None => {
                if frames == 0 && idle_since.elapsed().as_secs() >= 3 {
                    append_log("encoder: no frames after 3s, is the surface connected?");
                    idle_since = std::time::Instant::now();
                }
                std::thread::sleep(std::time::Duration::from_millis(2));
            }
        }
    }
}
/// Opens the first camera on demand and streams frames into [`LATEST_FRAME`].
///
/// The camera is only held open while the guest is streaming: as soon as the
/// guest stops pulling frames, everything is torn down so the phone gets its
/// camera back.
fn start_capture(camera_slot: usize, requested: String) {
    let api = match CameraApi::load() {
        Ok(api) => api,
        Err(e) => {
            append_log(&format!("capture: load failed: {e}"));
            return;
        }
    };

    // SAFETY: all handles below come from the NDK camera API.
    let manager = unsafe { (api.manager_create)() };
    if manager.is_null() {
        append_log("capture: manager null");
        return;
    }
    let camera_id = unsafe {
        let mut id_list: *mut ACameraIdList = std::ptr::null_mut();
        if (api.get_camera_id_list)(manager, &mut id_list) != 0 || id_list.is_null() {
            append_log("capture: no camera id list");
            return;
        }
        let mut ids: Vec<String> = Vec::new();
        for i in 0..(*id_list).num_cameras.max(0) as usize {
            let ptr = *(*id_list).camera_ids.add(i);
            if !ptr.is_null() {
                ids.push(std::ffi::CStr::from_ptr(ptr).to_string_lossy().into_owned());
            }
        }
        (api.delete_camera_id_list)(id_list);
        // Which camera to expose: the UVC device hands over the id the app
        // asked for ("0", "1", "front", "back" or a literal Android id). An id
        // this device does not have falls back to the slot position.
        let want = requested.trim().to_ascii_lowercase();
        let chosen = match want.as_str() {
            "" => ids.get(camera_slot).cloned(),
            "back" | "rear" => ids.first().cloned(),
            "front" | "selfie" => ids
                .iter()
                .find(|id| id.as_str() == "1")
                .cloned()
                .or_else(|| ids.get(1).cloned()),
            other => ids
                .iter()
                .find(|id| id.as_str() == other)
                .cloned()
                .or_else(|| other.parse::<usize>().ok().and_then(|i| ids.get(i).cloned()))
                .or_else(|| ids.get(camera_slot).cloned()),
        };
        append_log(&format!(
            "capture: camera {camera_slot} cameras={ids:?} requested={want} chosen={chosen:?}"
        ));
        match chosen {
            Some(id) => id,
            None => {
                append_log(&format!("capture: camera {camera_slot} has no camera to open"));
                return;
            }
        }
    };
    let camera_id = CString::new(camera_id).unwrap_or_default();
    append_log("capture: on-demand capture thread ready");

    loop {
        if !streaming_wanted(camera_slot) {
            std::thread::sleep(std::time::Duration::from_millis(200));
            continue;
        }
        // SAFETY: the manager outlives every session opened here.
        unsafe {
            stream_once(camera_slot, &api, manager, camera_id.as_ptr());
        }
        clear_frame(camera_slot);
        std::thread::sleep(std::time::Duration::from_millis(200));
    }
}

/// Runs one open/stream/close cycle, returning once the guest stops streaming or
/// the camera stops delivering frames.
///
/// # Safety
///
/// `manager` must be a live `ACameraManager`, and `camera_id` must be a valid
/// NUL terminated camera id.
unsafe fn stream_once(
    camera_slot: usize,
    api: &CameraApi,
    manager: *mut ACameraManager,
    camera_id: *const c_char,
) {
    let device_callbacks = DeviceStateCallbacks {
        context: std::ptr::null_mut(),
        on_disconnected: None,
        on_error: None,
    };
    let mut device: *mut ACameraDevice = std::ptr::null_mut();
    let mut status = -1;
    for _ in 0..10 {
        status = (api.open_camera)(manager, camera_id, &device_callbacks, &mut device);
        if status == 0 && !device.is_null() {
            break;
        }
        if !streaming_wanted(camera_slot) {
            return;
        }
        std::thread::sleep(std::time::Duration::from_millis(500));
    }
    if status != 0 || device.is_null() {
        append_log(&format!("capture: openCamera failed status={status}"));
        return;
    }
    append_log("capture: camera opened");

    // This device ships no MediaCodec JPEG encoder and the software encoder is
    // far too slow, so ask the camera HAL itself for hardware encoded JPEG
    // frames and only fall back to YUV when that fails.
    let mut hw_jpeg = true;
    let mut reader: *mut AImageReader = std::ptr::null_mut();
    let mut status = -1;
    for (width, height) in [(CAPTURE_WIDTH, CAPTURE_HEIGHT), (1280, 720), (640, 480)] {
        status = (api.image_reader_new)(width, height, IMAGE_FORMAT_JPEG, 8, &mut reader);
        if status == 0 && !reader.is_null() {
            append_log(&format!("capture: hw jpeg reader {width}x{height}"));
            break;
        }
        append_log(&format!(
            "capture: hw jpeg reader {width}x{height} failed status={status}"
        ));
    }
    if status != 0 || reader.is_null() {
        hw_jpeg = false;
        for (width, height) in [(CAPTURE_WIDTH, CAPTURE_HEIGHT), (1280, 720), (640, 480)] {
            status = (api.image_reader_new)(width, height, IMAGE_FORMAT_YUV_420_888, 8, &mut reader);
            if status == 0 && !reader.is_null() {
                append_log(&format!("capture: yuv reader {width}x{height}"));
                break;
            }
            append_log(&format!(
                "capture: yuv reader {width}x{height} failed status={status}"
            ));
        }
    }
    if status != 0 || reader.is_null() {
        append_log("capture: AImageReader_new failed for every format and size");
        (api.device_close)(device);
        return;
    }
    let mut window: *mut ANativeWindow = std::ptr::null_mut();
    let status = (api.image_reader_get_window)(reader, &mut window);
    if status != 0 || window.is_null() {
        append_log(&format!("capture: getWindow failed status={status}"));
        (api.image_reader_delete)(reader);
        (api.device_close)(device);
        return;
    }

    let mut container: *mut ACaptureSessionOutputContainer = std::ptr::null_mut();
    (api.session_output_container_create)(&mut container);
    let mut session_output: *mut ACaptureSessionOutput = std::ptr::null_mut();
    let status = (api.session_output_create)(window, &mut session_output);
    if status != 0 {
        append_log(&format!("capture: SessionOutput_create failed status={status}"));
        (api.image_reader_delete)(reader);
        (api.device_close)(device);
        return;
    }
    (api.session_output_container_add)(container, session_output);

    let session_callbacks = SessionStateCallbacks {
        context: std::ptr::null_mut(),
        on_ready: None,
        on_active: None,
        on_closed: None,
    };
    let mut session: *mut ACameraCaptureSession = std::ptr::null_mut();
    let status = (api.create_capture_session)(device, container, &session_callbacks, &mut session);
    if status != 0 || session.is_null() {
        append_log(&format!("capture: createCaptureSession failed status={status}"));
        (api.session_output_container_free)(container);
        (api.session_output_free)(session_output);
        (api.image_reader_delete)(reader);
        (api.device_close)(device);
        return;
    }

    let mut target: *mut ACameraOutputTarget = std::ptr::null_mut();
    let status = (api.output_target_create)(window, &mut target);
    if status != 0 || target.is_null() {
        append_log(&format!("capture: OutputTarget_create failed status={status}"));
        (api.session_close)(session);
        (api.session_output_container_free)(container);
        (api.session_output_free)(session_output);
        (api.image_reader_delete)(reader);
        (api.device_close)(device);
        return;
    }
    let mut request: *mut ACaptureRequest = std::ptr::null_mut();
    let status = (api.device_create_capture_request)(device, TEMPLATE_PREVIEW, &mut request);
    if status != 0 || request.is_null() {
        append_log(&format!("capture: createCaptureRequest failed status={status}"));
        (api.output_target_free)(target);
        (api.session_close)(session);
        (api.session_output_container_free)(container);
        (api.session_output_free)(session_output);
        (api.image_reader_delete)(reader);
        (api.device_close)(device);
        return;
    }
    (api.capture_request_add_target)(request, target);
    let auto: u8 = 1;
    (api.capture_request_set_entry_u8)(request, ACAMERA_CONTROL_AE_MODE, 1, &auto);
    // Ask for the fastest supported frame rate; the HAL clamps to what the
    // sensor and the JPEG pipeline can actually do. Without this the preview
    // template defaults to 15 fps on several vivo devices.
    let fps_hi: i32 = setting("DROIDVM_UVC_FPS", "/data/local/tmp/droidvm-uvc-fps")
        .and_then(|v| v.trim().parse::<i32>().ok())
        .filter(|v| *v >= 15 && *v <= 120)
        .unwrap_or(60);
    // Vendors accept only the ranges they advertise, so walk a candidate list
    // until one sticks instead of assuming the highest value works everywhere.
    let mut fps_applied: Option<[i32; 2]> = None;
    for candidate in [
        [fps_hi.min(30), fps_hi],
        [30, 30],
        [15, 30],
        [15, 15],
    ] {
        let status = (api.capture_request_set_entry_i32)(
            request,
            ACAMERA_CONTROL_AE_TARGET_FPS_RANGE,
            2,
            candidate.as_ptr(),
        );
        append_log(&format!(
            "capture: fps range {candidate:?} status={status}"
        ));
        if status == 0 {
            fps_applied = Some(candidate);
            break;
        }
    }
    let _ = fps_applied;
    if hw_jpeg {
        let mut quality_status = -1;
        for quality in [HW_JPEG_QUALITY, 85, 95] {
            quality_status =
                (api.capture_request_set_entry_i32)(request, ACAMERA_JPEG_QUALITY, 1, &quality);
            append_log(&format!(
                "capture: jpeg quality={quality} status={quality_status}"
            ));
            if quality_status == 0 {
                break;
            }
        }
        let thumbnail: [i32; 2] = [0, 0];
        let status2 = (api.capture_request_set_entry_i32)(
            request,
            ACAMERA_JPEG_THUMBNAIL_SIZE,
            2,
            thumbnail.as_ptr(),
        );
        append_log(&format!("capture: thumbnail_off status={status2}"));
    }

    let capture_callbacks = SessionCaptureCallbacks {
        context: std::ptr::null_mut(),
        on_started: None,
        on_progress: None,
        on_sequence_completed: None,
        on_buffer_lost: None,
    };
    let mut sequence_id: c_int = 0;
    let status = (api.set_repeating_request)(
        session,
        &capture_callbacks,
        1,
        &mut request,
        &mut sequence_id,
    );
    if status != 0 {
        append_log(&format!("capture: setRepeatingRequest failed status={status}"));
        (api.capture_request_free)(request);
        (api.output_target_free)(target);
        (api.session_close)(session);
        (api.session_output_container_free)(container);
        (api.session_output_free)(session_output);
        (api.image_reader_delete)(reader);
        (api.device_close)(device);
        return;
    }
    append_log("capture: repeating request started");

    let mut frames = 0u64;
    let mut last_frame = std::time::Instant::now();
    // Rolling statistics so a single log line shows whether the stream keeps up.
    let mut stats_frames = 0u64;
    let mut stats_bytes = 0u64;
    let mut stats_at = std::time::Instant::now();
    loop {
        if !streaming_wanted(camera_slot) {
            append_log("capture: releasing camera, guest is not streaming");
            break;
        }
        std::thread::sleep(std::time::Duration::from_millis(1));
        let mut image: *mut AImage = std::ptr::null_mut();
        let status = (api.image_reader_acquire_latest)(reader, &mut image);
        if status != 0 || image.is_null() {
            if last_frame.elapsed() > std::time::Duration::from_secs(5) {
                append_log("capture: no frames for 5s, reopening the camera");
                break;
            }
            continue;
        }
        let frame = if hw_jpeg {
            image_jpeg_bytes(api, image).map(|raw| {
                let slim = slim_jpeg(&raw);
                if frames == 0 {
                    append_log(&format!(
                        "capture: jpeg raw={} slim={} bytes",
                        raw.len(),
                        slim.len()
                    ));
                }
                slim
            })
        } else {
            image_to_jpeg(api, image, frames == 0)
        };
        if let Some(jpeg) = frame {
            frames += 1;
            let size = jpeg.len();
            last_frame = std::time::Instant::now();
            if frames == 1 {
                if let Err(e) = std::fs::write(
                    format!("/data/local/tmp/frame1-{camera_slot}.jpg"),
                    &jpeg,
                ) {
                    append_log(&format!("capture: cannot dump frame: {e}"));
                }
            }
            store_frame(camera_slot, jpeg);
            stats_frames += 1;
            stats_bytes += size as u64;
            if frames == 1 || frames % 300 == 0 {
                append_log(&format!("capture: frame {frames} jpeg={size} bytes"));
            }
            let elapsed = stats_at.elapsed().as_secs_f64();
            if elapsed >= 5.0 {
                append_log(&format!(
                    "capture: stats fps={:.1} avg_frame={} max_payload={}",
                    stats_frames as f64 / elapsed,
                    stats_bytes / stats_frames.max(1),
                    crate::usb::backend::uvc_backend::descriptors::MAX_PAYLOAD_TRANSFER_SIZE,
                ));
                stats_frames = 0;
                stats_bytes = 0;
                stats_at = std::time::Instant::now();
            }
        }
        (api.image_delete)(image);
    }

    (api.session_stop_repeating)(session);
    (api.session_close)(session);
    (api.capture_request_free)(request);
    (api.output_target_free)(target);
    (api.session_output_container_free)(container);
    (api.session_output_free)(session_output);
    (api.image_reader_delete)(reader);
    (api.device_close)(device);
    append_log("capture: camera released");
}

/// Spawns the background thread that streams frames into [`LATEST_FRAME`] while
/// the guest is streaming. Progress goes to `/data/local/tmp/camprobe.log`.
pub fn spawn_camera(camera_slot: usize, camera_id: String) {
    let _ = std::thread::Builder::new()
        .name(format!("uvc-camera-{camera_slot}"))
        .spawn(move || {
            if camera_slot == 0 {
                let _ = std::fs::write(
                    "/data/local/tmp/camprobe.log",
                    "capture: on-demand camera thread started\n",
                );
            }
            start_capture(camera_slot, camera_id);
        });
}
