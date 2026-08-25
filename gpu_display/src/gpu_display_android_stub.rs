// Copyright 2024 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! Stub implementation of the native interface of libcrosvm_android_display_client
//!
//! This implementation is used to enable the gpu display backend for Android to be compiled
//! without libcrosvm_android_display_client available. It is only used for testing purposes and
//! not functional at runtime.

use std::ffi::c_char;
use std::ffi::c_int;

use crate::gpu_display_android::ANativeWindow_Buffer;
use crate::gpu_display_android::AndroidDisplayContext;
use crate::gpu_display_android::AndroidDisplaySurface;
use crate::gpu_display_android::ErrorCallback;

#[no_mangle]
extern "C" fn create_android_display_context(
    _name: *const c_char,
    _error_callback: ErrorCallback,
) -> *mut AndroidDisplayContext {
    unimplemented!();
}

#[no_mangle]
extern "C" fn destroy_android_display_context(_ctx: *mut AndroidDisplayContext) {
    unimplemented!();
}

#[no_mangle]
extern "C" fn create_android_surface(
    _ctx: *mut AndroidDisplayContext,
    _width: u32,
    _height: u32,
    _for_cursor: bool,
) -> *mut AndroidDisplaySurface {
    unimplemented!();
}

#[no_mangle]
extern "C" fn destroy_android_surface(
    _ctx: *mut AndroidDisplayContext,
    _surface: *mut AndroidDisplaySurface,
) {
    unimplemented!();
}

#[no_mangle]
extern "C" fn set_android_surface_position(_ctx: *mut AndroidDisplayContext, _x: u32, _y: u32) {
    unimplemented!();
}

#[no_mangle]
extern "C" fn get_android_surface_buffer(
    _ctx: *mut AndroidDisplayContext,
    _surface: *mut AndroidDisplaySurface,
    _out_buffer: *mut ANativeWindow_Buffer,
) -> u32 {
    unimplemented!();
}

#[no_mangle]
extern "C" fn post_android_surface_buffer(
    _ctx: *mut AndroidDisplayContext,
    _surface: *mut AndroidDisplaySurface,
) {
    unimplemented!();
}

#[no_mangle]
extern "C" fn set_android_surface_buffer_format(
    _ctx: *mut AndroidDisplayContext,
    _surface: *mut AndroidDisplaySurface,
    _fourcc: u32,
) {
    unimplemented!();
}

#[no_mangle]
extern "C" fn android_display_import_dmabuf(
    _ctx: *mut AndroidDisplayContext,
    _surface: *mut AndroidDisplaySurface,
    _fd: base::RawDescriptor,
    _offset: u32,
    _stride: u32,
    _modifier: u64,
    _linear_layout_verified: bool,
    _width: u32,
    _height: u32,
    _fourcc: u32,
) -> i64 {
    0
}

#[no_mangle]
extern "C" fn android_display_release_import(_ctx: *mut AndroidDisplayContext, _raw_handle: i64) {}

#[no_mangle]
extern "C" fn android_display_is_vulkan_blit_available(_ctx: *mut AndroidDisplayContext) -> bool {
    false
}

// Nothing can be attached to a display that does not exist, so `false` is the honest answer here
// as well as the one that keeps this stub free of a panic on a per-frame path.
#[no_mangle]
extern "C" fn android_display_has_consumer(_ctx: *mut AndroidDisplayContext) -> bool {
    false
}

// The headless blit context (the VNC sink's GPU half) lives in the same native library, so its
// symbols need standing in for on the same builds. `create` returning null is the whole answer: it
// is what the real one returns on a machine with no Vulkan blit driver named, and the sink treats
// that as "no GPU half" rather than as an error, so nothing below is ever reached.
#[no_mangle]
extern "C" fn android_blit_ctx_create(_width: u32, _height: u32) -> *mut std::ffi::c_void {
    std::ptr::null_mut()
}

#[no_mangle]
extern "C" fn android_blit_ctx_destroy(_ctx: *mut std::ffi::c_void) {}

#[no_mangle]
extern "C" fn android_blit_ctx_import_dmabuf(
    _ctx: *mut std::ffi::c_void,
    _fd: base::RawDescriptor,
    _offset: u32,
    _stride: u32,
    _modifier: u64,
    _linear_layout_verified: bool,
    _width: u32,
    _height: u32,
    _fourcc: u32,
) -> i64 {
    0
}

#[no_mangle]
extern "C" fn android_blit_ctx_release_import(_ctx: *mut std::ffi::c_void, _import_id: i64) {}

#[no_mangle]
extern "C" fn android_blit_ctx_blit(
    _ctx: *mut std::ffi::c_void,
    _import_id: i64,
    _width: u32,
    _height: u32,
    _timeout_ms: c_int,
) -> bool {
    false
}

#[no_mangle]
extern "C" fn android_blit_ctx_map(
    _ctx: *mut std::ffi::c_void,
    _out_pixels: *mut *const u8,
    _out_stride_bytes: *mut u32,
    _out_width: *mut u32,
    _out_height: *mut u32,
    _out_size: *mut u32,
) -> bool {
    false
}

#[no_mangle]
extern "C" fn android_blit_ctx_unmap(_ctx: *mut std::ffi::c_void) {}

#[no_mangle]
extern "C" fn android_display_flip_to(
    _ctx: *mut AndroidDisplayContext,
    _surface: *mut AndroidDisplaySurface,
    _raw_handle: i64,
    out_completion_fence_fd: *mut c_int,
) -> bool {
    if !out_completion_fence_fd.is_null() {
        // SAFETY: checked non-null, and the caller passes a pointer to its own c_int.
        unsafe { *out_completion_fence_fd = -1 };
    }
    false
}
