// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

//! The VNC sink's GPU half: a Vulkan blit into a buffer the CPU can read.
//!
//! The machinery is the native sink's, used headless. `crosvm_android_display_client.cpp` already
//! kept its Vulkan bridge free of anything screen-shaped -- it takes a dmabuf fd in and an
//! `AHardwareBuffer*` out -- so the whole of this is the same import, the same blit and the same
//! fence, against a target allocated with CPU-read usage instead of dequeued from an app's Surface.
//! There is no second Vulkan stack here and there must never be one: a colour or layout rule that
//! held on one of them and not the other would be invisible until somebody compared two screens.
//!
//! What the VNC sink gets out of it is not "no copy". It has to put pixels on a socket, so a
//! readback is structural. What it gets is that the copy happens on the GPU, straight out of the
//! guest's own pages, with the channel-order conversion folded into it -- and that what the CPU
//! then touches is ordinary cached host memory rather than a write-combining guest mapping.

use std::ffi::c_int;
use std::ffi::c_void;
use std::sync::atomic::AtomicU64;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::time::Duration;

use base::RawDescriptor;

/// How long a blit is given to finish before the frame is abandoned.
///
/// A bound, and a hard one: on the far side of this wait the CPU reads the target, and reading a
/// target the GPU is still writing produces a torn frame that reports nothing. That is the opposite
/// of the simplefb bridge's flip fence, which is a pacing device and may be overrun -- there the
/// source is guest memory nobody synchronises anyway, here it is our own buffer and the wait is the
/// only thing that makes its contents mean anything.
///
/// Three vsyncs of a 120 Hz panel, the same figure the other two fence waits in this tree settled
/// on. A full-screen transfer is orders of magnitude under it; a blit that is not done by then is
/// not slow, it is wrong, and the answer to that is the CPU path rather than a longer wait.
const BLIT_FENCE_TIMEOUT: Duration = Duration::from_millis(25);

#[cfg(any(feature = "android_display", feature = "android_display_stub"))]
mod ffi {
    use super::*;

    extern "C" {
        pub fn android_blit_ctx_create(width: u32, height: u32) -> *mut c_void;
        pub fn android_blit_ctx_destroy(ctx: *mut c_void);
        #[allow(clippy::too_many_arguments)]
        pub fn android_blit_ctx_import_dmabuf(
            ctx: *mut c_void,
            fd: RawDescriptor,
            offset: u32,
            stride: u32,
            modifier: u64,
            linear_layout_verified: bool,
            width: u32,
            height: u32,
            fourcc: u32,
        ) -> i64;
        pub fn android_blit_ctx_release_import(ctx: *mut c_void, import_id: i64);
        pub fn android_blit_ctx_blit(
            ctx: *mut c_void,
            import_id: i64,
            width: u32,
            height: u32,
            timeout_ms: c_int,
        ) -> bool;
        pub fn android_blit_ctx_map(
            ctx: *mut c_void,
            out_pixels: *mut *const u8,
            out_stride_bytes: *mut u32,
            out_width: *mut u32,
            out_height: *mut u32,
            out_size: *mut u32,
        ) -> bool;
    }
}

/// A build with the VNC sink but no Android display backend links none of the above, because the
/// blit lives in `libcrosvm_android_display_client`.
///
/// These are not test stubs. "There is no blit context" is a real runtime answer -- it is what a
/// phone with no `CROSVM_DISPLAY_VULKAN_LIBRARY` named gives, and what a `transport-cap=cpu`
/// binding never even asks for -- so the sink already has exactly one way to handle it, and this
/// configuration takes that way rather than a second one.
#[cfg(not(any(feature = "android_display", feature = "android_display_stub")))]
#[allow(clippy::too_many_arguments)]
mod ffi {
    use super::*;

    pub unsafe fn android_blit_ctx_create(_width: u32, _height: u32) -> *mut c_void {
        std::ptr::null_mut()
    }
    pub unsafe fn android_blit_ctx_destroy(_ctx: *mut c_void) {}
    pub unsafe fn android_blit_ctx_import_dmabuf(
        _ctx: *mut c_void,
        _fd: RawDescriptor,
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
    pub unsafe fn android_blit_ctx_release_import(_ctx: *mut c_void, _import_id: i64) {}
    pub unsafe fn android_blit_ctx_blit(
        _ctx: *mut c_void,
        _import_id: i64,
        _width: u32,
        _height: u32,
        _timeout_ms: c_int,
    ) -> bool {
        false
    }
    pub unsafe fn android_blit_ctx_map(
        _ctx: *mut c_void,
        _out_pixels: *mut *const u8,
        _out_stride_bytes: *mut u32,
        _out_width: *mut u32,
        _out_height: *mut u32,
        _out_size: *mut u32,
    ) -> bool {
        false
    }
}

/// Where the last blitted frame is, and how it is laid out.
///
/// `stride_bytes` is gralloc's, which is free to exceed `width * 4` and does. Nothing here packs it
/// for the caller: the row padding is real and the consumer has to be told, because a consumer that
/// assumes packed rows produces a sheared picture rather than an error.
#[derive(Clone, Copy)]
pub(crate) struct BlitMapping {
    pub pixels: *const u8,
    pub stride_bytes: u32,
    pub width: u32,
    pub height: u32,
    /// Bytes readable from `pixels`, i.e. `stride_bytes * height`.
    pub size: u32,
    /// Which blit this mapping describes. There is one target per context, so a later blit
    /// invalidates every mapping of it -- including one held by a surface that has since been
    /// replaced but not yet released, which is the only way two holders of the same context exist
    /// at once (a resize builds the new surface before the old one is dropped). Checked rather than
    /// reasoned about, because the failure is a read of unmapped memory.
    generation: u64,
}

/// The headless blit context: one Vulkan device, one CPU-readable target, and the imports made
/// against it.
///
/// Held behind an `Arc` because the mapping it hands out is borrowed memory that it owns, so
/// anything holding a `BlitMapping` has to hold this too. Send/Sync are asserted for the same
/// reason `VncServerHandle` asserts them -- this lives on the display thread and is never touched
/// from another one; the assertion is what lets it sit inside the sink's existing `Arc<Mutex<..>>`.
pub(crate) struct VncBlitContext {
    ptr: *mut c_void,
    /// Bumped by every blit, so a `BlitMapping` can say which one it belongs to.
    generation: AtomicU64,
}

// SAFETY: the context is used from the display thread only. The unsafe impls exist so it can be
// stored beside the server handle, which makes the same claim for the same reason.
unsafe impl Send for VncBlitContext {}
unsafe impl Sync for VncBlitContext {}

impl VncBlitContext {
    /// Brings up the blit half, or answers that there is none.
    ///
    /// `None` is not a failure to report loudly here -- the native sink's probe answers the same
    /// way on any machine whose launcher named no Vulkan driver, which is most of them -- so the
    /// caller decides what to say about it, once.
    pub fn open(width: u32, height: u32) -> Option<Arc<VncBlitContext>> {
        // SAFETY: no arguments are borrowed; the returned pointer is owned by us from here.
        let ptr = unsafe { ffi::android_blit_ctx_create(width, height) };
        if ptr.is_null() {
            return None;
        }
        Some(Arc::new(VncBlitContext {
            ptr,
            generation: AtomicU64::new(0),
        }))
    }

    /// Imports a guest dmabuf as a blit source, returning the native handle.
    ///
    /// `fourcc` is what the GUEST declared. The correction that makes the blit land in VNC's byte
    /// order is applied on the other side of this call, next to the target it has to agree with;
    /// see `blitSourceFourcc` in `crosvm_android_display_client.cpp`.
    #[allow(clippy::too_many_arguments)]
    pub fn import_dmabuf(
        &self,
        fd: RawDescriptor,
        offset: u32,
        stride: u32,
        modifier: u64,
        linear_layout_verified: bool,
        width: u32,
        height: u32,
        fourcc: u32,
    ) -> Option<i64> {
        // SAFETY: `fd` is borrowed for the duration of the call; the native side dups what it keeps.
        let handle = unsafe {
            ffi::android_blit_ctx_import_dmabuf(
                self.ptr,
                fd,
                offset,
                stride,
                modifier,
                linear_layout_verified,
                width,
                height,
                fourcc,
            )
        };
        (handle != 0).then_some(handle)
    }

    pub fn release_import(&self, import_id: i64) {
        // SAFETY: `import_id` came from `import_dmabuf` on this context.
        unsafe { ffi::android_blit_ctx_release_import(self.ptr, import_id) }
    }

    /// Blits an import into the target and waits, bounded, for the GPU to finish with it.
    ///
    /// Any CPU mapping of the target is dropped by this call, so a `BlitMapping` from a previous
    /// frame must not be used afterwards. That ordering is not bookkeeping: the lock is where the
    /// CPU's view of this memory is invalidated, so a mapping held across a blit would keep showing
    /// the frame before it.
    pub fn blit(&self, import_id: i64, width: u32, height: u32) -> bool {
        // Bumped whether or not the blit succeeds: the unmap on the native side happens first
        // either way, so every mapping is stale from here on.
        self.generation.fetch_add(1, Ordering::Relaxed);
        // SAFETY: `import_id` came from this context, and nothing is borrowed across the call.
        unsafe {
            ffi::android_blit_ctx_blit(
                self.ptr,
                import_id,
                width,
                height,
                BLIT_FENCE_TIMEOUT.as_millis() as c_int,
            )
        }
    }

    /// Whether a mapping still describes the target's current contents.
    pub fn mapping_is_current(&self, mapping: &BlitMapping) -> bool {
        mapping.generation == self.generation.load(Ordering::Relaxed)
    }

    /// Maps the target for CPU reading. Valid until the next `blit`, or until this context is
    /// dropped -- which is why the caller keeps an `Arc` of it beside the mapping.
    pub fn map(&self) -> Option<BlitMapping> {
        let mut pixels: *const u8 = std::ptr::null();
        let (mut stride_bytes, mut width, mut height, mut size) = (0u32, 0u32, 0u32, 0u32);
        // SAFETY: every out pointer refers to a live local for the duration of the call.
        let ok = unsafe {
            ffi::android_blit_ctx_map(
                self.ptr,
                &mut pixels,
                &mut stride_bytes,
                &mut width,
                &mut height,
                &mut size,
            )
        };
        if !ok || pixels.is_null() {
            return None;
        }
        Some(BlitMapping {
            pixels,
            stride_bytes,
            width,
            height,
            size,
            generation: self.generation.load(Ordering::Relaxed),
        })
    }
}

impl Drop for VncBlitContext {
    fn drop(&mut self) {
        // SAFETY: the pointer came from `android_blit_ctx_create` and is dropped once.
        unsafe { ffi::android_blit_ctx_destroy(self.ptr) }
    }
}
