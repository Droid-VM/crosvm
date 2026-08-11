// Copyright 2024 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::ffi::c_char;
use std::ffi::c_int;
use std::ffi::CStr;
use std::ffi::CString;
use std::panic::catch_unwind;
use std::process::abort;
use std::ptr::NonNull;
use std::rc::Rc;
use std::slice;

use base::error;
use base::AsRawDescriptor;
use base::Event;
use base::FromRawDescriptor;
use base::RawDescriptor;
use base::SafeDescriptor;
use base::VolatileSlice;
use vm_control::gpu::DisplayParameters;

use crate::DisplayT;
use crate::GpuDisplayError;
use crate::GpuDisplayFramebuffer;
use crate::GpuDisplayResult;
use crate::GpuDisplaySurface;
use crate::SemaphoreTimepoint;
use crate::SurfaceType;
use crate::SysDisplayT;

// Opaque blob
#[repr(C)]
pub(crate) struct AndroidDisplayContext {
    _data: [u8; 0],
    _marker: core::marker::PhantomData<(*mut u8, core::marker::PhantomPinned)>,
}

// Opaque blob
#[repr(C)]
pub(crate) struct AndroidDisplaySurface {
    _data: [u8; 0],
    _marker: core::marker::PhantomData<(*mut u8, core::marker::PhantomPinned)>,
}

// Should be the same as ANativeWindow_Buffer in android/native_window.h
// Note that this struct is part of NDK; guaranteed to be stable, so we use it directly across the
// FFI.
#[repr(C)]
pub(crate) struct ANativeWindow_Buffer {
    width: i32,
    height: i32,
    stride: i32, // in number of pixels, NOT bytes
    format: i32,
    bits: *mut u8,
    reserved: [u32; 6],
}

pub(crate) type ErrorCallback = unsafe extern "C" fn(message: *const c_char);

/// virtio-gpu's cursor plane is fixed at 64x64 (drivers/gpu/drm/virtio registers it with that
/// single size), and a smaller cursor image arrives in the top-left of it.
const CURSOR_PLANE_SIZE: u32 = 64;

/// Sentinel position meaning "the guest hid its pointer". A real cursor position is a framebuffer
/// coordinate, so u32::MAX can never collide with one.
const CURSOR_HIDDEN_POS: u32 = u32::MAX;

extern "C" {
    /// Constructs an AndroidDisplayContext for this backend. This awlays returns a valid (ex:
    /// non-null) handle to the context. The `name` parameter is from crosvm commandline and the
    /// client of crosvm will use it to locate and communicate to the AndroidDisplayContext. For
    /// example, this can be a path to UNIX domain socket where a RPC binder server listens on.
    /// `error_callback` is a function pointer to an error reporting function, and will be used by
    /// this and other functions below when something goes wrong. The returned context should be
    /// destroyed by calling `destroy_android_display_context` if this backend is no longer in use.
    fn create_android_display_context(
        name: *const c_char,
        error_callback: ErrorCallback,
    ) -> *mut AndroidDisplayContext;

    /// Destroys the AndroidDisplayContext created from `create_android_display_context`.
    fn destroy_android_display_context(self_: *mut AndroidDisplayContext);

    /// Creates an Android Surface (which is also called as Window) of given size. If the surface
    /// can't be created for whatever reason, null pointer is returned, in which case we shouldn't
    /// proceed further.
    fn create_android_surface(
        ctx: *mut AndroidDisplayContext,
        width: u32,
        height: u32,
        for_cursor: bool,
    ) -> *mut AndroidDisplaySurface;

    /// Destroys the Android surface created from `create_android_surface`.
    #[allow(dead_code)]
    fn destroy_android_surface(
        ctx: *mut AndroidDisplayContext,
        surface: *mut AndroidDisplaySurface,
    );

    /// Obtains one buffer from the given Android Surface. The information about the buffer (buffer
    /// address, size, stride, etc) is reported via the `ANativeWindow_Buffer` struct. It shouldn't
    /// be null. The size of the buffer is guaranteed to be bigger than (width * stride * 4) bytes.
    /// This function locks the buffer for the client, which means the caller has the exclusive
    /// access to the buffer until it is returned back to Android display stack (surfaceflinger) by
    /// calling `post_android_surface_buffer`. This function may fail (in which case false is
    /// returned), then the caller shouldn't try to read `out_buffer` or use the buffer in any way.
    fn get_android_surface_buffer(
        ctx: *mut AndroidDisplayContext,
        surface: *mut AndroidDisplaySurface,
        out_buffer: *mut ANativeWindow_Buffer,
    ) -> bool;

    fn set_android_surface_position(ctx: *mut AndroidDisplayContext, x: u32, y: u32);

    /// Posts the buffer obtained from `get_android_surface_buffer` to the Android display system
    /// so that it can be displayed on the screen. Once this is called, the caller shouldn't use
    /// the buffer any more.
    fn post_android_surface_buffer(
        ctx: *mut AndroidDisplayContext,
        surface: *mut AndroidDisplaySurface,
    );

    fn set_android_surface_buffer_format(
        ctx: *mut AndroidDisplayContext,
        surface: *mut AndroidDisplaySurface,
        fourcc: u32,
    );

    fn android_display_import_dmabuf(
        ctx: *mut AndroidDisplayContext,
        surface: *mut AndroidDisplaySurface,
        fd: RawDescriptor,
        offset: u32,
        stride: u32,
        modifier: u64,
        linear_layout_verified: bool,
        width: u32,
        height: u32,
        fourcc: u32,
    ) -> i64;

    fn android_display_release_import(ctx: *mut AndroidDisplayContext, raw_handle: i64);

    fn android_display_is_vulkan_blit_available(ctx: *mut AndroidDisplayContext) -> bool;

    /// `out_completion_fence_fd` is written by the callee and is mandatory: the C side rejects a
    /// null pointer outright, and on the async-blit path it hands back an owned sync_file fd that
    /// this side must close. Keep the arity in step with the definition in
    /// `crosvm_android_display_client.cpp` -- these symbols are not declared in any shared header,
    /// so this extern block is the only "declaration" the compiler ever sees and a mismatch links
    /// silently.
    fn android_display_flip_to(
        ctx: *mut AndroidDisplayContext,
        surface: *mut AndroidDisplaySurface,
        raw_handle: i64,
        out_completion_fence_fd: *mut c_int,
    ) -> bool;
}

unsafe extern "C" fn error_callback(message: *const c_char) {
    catch_unwind(|| {
        error!(
            "{}",
            // SAFETY: message is null terminated
            unsafe { CStr::from_ptr(message) }.to_string_lossy()
        )
    })
    .unwrap_or_else(|_| abort())
}

struct AndroidDisplayContextWrapper(NonNull<AndroidDisplayContext>);

impl Drop for AndroidDisplayContextWrapper {
    fn drop(&mut self) {
        // SAFETY: this object is constructed from create_android_display_context
        unsafe { destroy_android_display_context(self.0.as_ptr()) };
    }
}

impl Default for ANativeWindow_Buffer {
    fn default() -> Self {
        Self {
            width: 0,
            height: 0,
            stride: 0,
            format: 0,
            bits: std::ptr::null_mut(),
            reserved: [0u32; 6],
        }
    }
}

impl From<ANativeWindow_Buffer> for GpuDisplayFramebuffer<'_> {
    fn from(anb: ANativeWindow_Buffer) -> Self {
        // TODO: check anb.format to see if it's ARGB8888?
        // TODO: infer bpp from anb.format?
        const BYTES_PER_PIXEL: u32 = 4;
        let stride_bytes = BYTES_PER_PIXEL * u32::try_from(anb.stride).unwrap();
        let buffer_size = stride_bytes * u32::try_from(anb.height).unwrap();
        let buffer =
            // SAFETY: get_android_surface_buffer guarantees that bits points to a valid buffer and
            // the buffer remains available until post_android_surface_buffer is called.
            unsafe { slice::from_raw_parts_mut(anb.bits, buffer_size.try_into().unwrap()) };
        Self::new(VolatileSlice::new(buffer), stride_bytes, BYTES_PER_PIXEL)
    }
}

struct AndroidSurface {
    context: Rc<AndroidDisplayContextWrapper>,
    surface: NonNull<AndroidDisplaySurface>,
    imports: Rc<RefCell<BTreeMap<u32, i64>>>,
}

impl GpuDisplaySurface for AndroidSurface {
    fn framebuffer(&mut self) -> Option<GpuDisplayFramebuffer> {
        let mut anb = ANativeWindow_Buffer::default();
        // SAFETY: context and surface are opaque handles and buf is used as the out parameter to
        // hold the return values.
        let success = unsafe {
            get_android_surface_buffer(
                self.context.0.as_ptr(),
                self.surface.as_ptr(),
                &mut anb as *mut ANativeWindow_Buffer,
            )
        };
        if success {
            Some(anb.into())
        } else {
            None
        }
    }

    fn flip(&mut self) {
        // SAFETY: context and surface are opaque handles.
        unsafe { post_android_surface_buffer(self.context.0.as_ptr(), self.surface.as_ptr()) }
    }

    fn set_position(&mut self, x: i32, y: i32) {
        // The image origin goes negative when the pointer is within the hotspot of the left or top
        // edge, but the app owns this FFI and it takes u32 -- and u32::MAX is the hide sentinel,
        // which -1 would land on exactly, making the pointer vanish near the edge. Clamp instead:
        // the cursor stops up to a hotspot short of the corner rather than disappearing into it.
        // Drawing it properly clipped would mean re-rendering the plane, which is the app's side.
        // SAFETY: context is an opaque handle.
        unsafe {
            set_android_surface_position(self.context.0.as_ptr(), x.max(0) as u32, y.max(0) as u32)
        };
    }

    fn set_buffer_fourcc(&mut self, fourcc: u32) {
        // SAFETY: context and surface are live opaque handles owned by this surface.
        unsafe {
            set_android_surface_buffer_format(
                self.context.0.as_ptr(),
                self.surface.as_ptr(),
                fourcc,
            )
        };
    }

    /// Hiding rides the existing position pipe rather than a new FFI entry point: the native side
    /// forwards whatever it is given straight to the app, so a coordinate the guest can never
    /// produce carries the message with no change to the C bridge or the AIDL.
    fn set_cursor_visible(&mut self, visible: bool) {
        if visible {
            return; // the next real position makes it visible again
        }
        // SAFETY: context is an opaque handle.
        unsafe {
            set_android_surface_position(self.context.0.as_ptr(), CURSOR_HIDDEN_POS, CURSOR_HIDDEN_POS)
        };
    }

    fn flip_to(
        &mut self,
        import_id: u32,
        _acquire_timepoint: Option<SemaphoreTimepoint>,
        _release_timepoint: Option<SemaphoreTimepoint>,
        _extra_info: Option<crate::FlipToExtraInfo>,
    ) -> anyhow::Result<sync::Waitable> {
        let raw_handle = self
            .imports
            .borrow()
            .get(&import_id)
            .copied()
            .ok_or_else(|| anyhow::anyhow!("invalid Android display import id {}", import_id))?;
        // On the synchronous blit path the bridge waits for the Turnip queue before handing the AHB
        // to SurfaceControl and reports -1 here. The async path instead exports a sync_file for the
        // blit's completion and transfers it to us, so this fd must be owned on every success path
        // or the process leaks one per presented frame.
        let mut completion_fence_fd: c_int = -1;
        let success = unsafe {
            android_display_flip_to(
                self.context.0.as_ptr(),
                self.surface.as_ptr(),
                raw_handle,
                &mut completion_fence_fd,
            )
        };
        // Adopt the fd before the error check: `success` says whether the flip happened, not
        // whether an fd came back, and leaking it on the failure path would be the same bug.
        let completion_fence = if completion_fence_fd >= 0 {
            // SAFETY: the bridge transfers ownership of this fd to us, and it is only written when
            // the export succeeded.
            Some(unsafe { SafeDescriptor::from_raw_descriptor(completion_fence_fd) })
        } else {
            None
        };
        if !success {
            return Err(anyhow::anyhow!(
                "Android display Vulkan blit failed; switching to CPU fallback"
            ));
        }
        // Dropping the fence closes it without waiting, which is what every caller of flip_to
        // already gets: `sync::Waitable` is a condvar pair with no fd backing, and the only caller
        // (virtio_gpu.rs resource_flush) discards the Waitable and completes RESOURCE_FLUSH
        // immediately. Backpressure comes from the bridge instead -- blit() calls reclaimSlot() on
        // the slot it is about to reuse, which CPU-waits that slot's previous fence, so with
        // kAsyncInFlightSlotCount=3 a blit is at most 2 frames behind. That bounds it but does not
        // order the guest's reuse of the *source* dmabuf against the blit reading it; closing the
        // real barrier needs an fd-backed Waitable plus a caller that waits on it.
        drop(completion_fence);
        Ok(sync::Waitable::signaled())
    }
}

pub struct DisplayAndroid {
    context: Rc<AndroidDisplayContextWrapper>,
    imports: Rc<RefCell<BTreeMap<u32, i64>>>,
    surfaces: BTreeMap<u32, NonNull<AndroidDisplaySurface>>,
    /// This event is never triggered and is used solely to fulfill AsRawDescriptor.
    event: Event,
}

impl DisplayAndroid {
    pub fn new(name: &str) -> GpuDisplayResult<DisplayAndroid> {
        let name = CString::new(name).unwrap();
        let context = NonNull::new(
            // SAFETY: service_name is not leaked outside of this function
            unsafe { create_android_display_context(name.as_ptr(), error_callback) },
        )
        .ok_or(GpuDisplayError::Unsupported)?;
        let context = AndroidDisplayContextWrapper(context);
        let event = Event::new().map_err(|_| GpuDisplayError::CreateEvent)?;
        Ok(DisplayAndroid {
            context: context.into(),
            imports: Rc::new(RefCell::new(BTreeMap::new())),
            surfaces: BTreeMap::new(),
            event,
        })
    }
}

impl Drop for DisplayAndroid {
    fn drop(&mut self) {
        let context = self.context.0.as_ptr();
        let imports = std::mem::take(&mut *self.imports.borrow_mut());
        for raw_handle in imports.into_values() {
            // SAFETY: every handle was created by the matching native import call and is
            // released before the context is dropped.
            unsafe { android_display_release_import(context, raw_handle) };
        }
    }
}

impl DisplayT for DisplayAndroid {
    fn is_dmabuf_import_supported(&mut self) -> bool {
        // SAFETY: context is a live opaque handle owned by this DisplayAndroid.
        unsafe { android_display_is_vulkan_blit_available(self.context.0.as_ptr()) }
    }

    fn create_surface(
        &mut self,
        parent_surface_id: Option<u32>,
        surface_id: u32,
        _scanout_id: Option<u32>,
        display_params: &DisplayParameters,
        _surf_type: SurfaceType,
    ) -> GpuDisplayResult<Box<dyn GpuDisplaySurface>> {
        // A parented surface is virtio-gpu's cursor. Its scanout carries no display_params of its
        // own, so crosvm passes DisplayParameters::default() -- sizing the cursor surface from
        // that would configure it at the default DISPLAY resolution instead of the cursor plane's
        // 64x64, and the Android side would allocate a full-screen buffer per pointer image.
        let (requested_width, requested_height) = if parent_surface_id.is_some() {
            (CURSOR_PLANE_SIZE, CURSOR_PLANE_SIZE)
        } else {
            display_params.get_virtual_display_size()
        };
        // SAFETY: context is an opaque handle.
        let surface = NonNull::new(unsafe {
            create_android_surface(
                self.context.0.as_ptr(),
                requested_width,
                requested_height,
                parent_surface_id.is_some(),
            )
        })
        .ok_or(GpuDisplayError::CreateSurface)?;
        self.surfaces.insert(surface_id, surface);

        Ok(Box::new(AndroidSurface {
            context: self.context.clone(),
            surface,
            imports: self.imports.clone(),
        }))
    }

    fn import_resource(
        &mut self,
        import_id: u32,
        surface_id: u32,
        external_display_resource: crate::DisplayExternalResourceImport,
    ) -> anyhow::Result<()> {
        let crate::DisplayExternalResourceImport::Dmabuf {
            descriptor,
            offset,
            stride,
            modifiers,
            linear_layout_verified,
            width,
            height,
            fourcc,
        } = external_display_resource
        else {
            return Err(anyhow::anyhow!(
                "Android display only supports DMA-BUF imports"
            ));
        };

        let surface = self
            .surfaces
            .get(&surface_id)
            .copied()
            .ok_or_else(|| anyhow::anyhow!("invalid Android display surface {}", surface_id))?;
        let raw_handle = unsafe {
            android_display_import_dmabuf(
                self.context.0.as_ptr(),
                surface.as_ptr(),
                descriptor.as_raw_descriptor(),
                offset,
                stride,
                modifiers,
                linear_layout_verified,
                width,
                height,
                fourcc,
            )
        };
        if raw_handle == 0 {
            return Err(anyhow::anyhow!("Turnip DMA-BUF import failed"));
        }
        self.imports.borrow_mut().insert(import_id, raw_handle);
        Ok(())
    }

    fn release_import(&mut self, import_id: u32, _surface_id: u32) {
        // The common display wrapper owns the import id counter; remove the native
        // object by id and let the shared map keep surface flips coherent.
        if let Some((_, raw_handle)) = self.imports.borrow_mut().remove_entry(&import_id) {
            unsafe { android_display_release_import(self.context.0.as_ptr(), raw_handle) };
        }
    }
}

impl SysDisplayT for DisplayAndroid {}

impl AsRawDescriptor for DisplayAndroid {
    fn as_raw_descriptor(&self) -> RawDescriptor {
        self.event.as_raw_descriptor()
    }
}
