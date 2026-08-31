// Copyright 2018 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! Crate for displaying simple surfaces and GPU buffers over a low-level display backend such as
//! Wayland or X.

use std::collections::BTreeMap;
use std::io::Error as IoError;
use std::time::Duration;

use anyhow::anyhow;
use anyhow::Context;
use base::AsRawDescriptor;
use base::Error as BaseError;
use base::EventToken;
use base::EventType;
use base::SafeDescriptor;
use base::VolatileSlice;
use base::WaitContext;
use remain::sorted;
use serde::Deserialize;
use serde::Serialize;
use sync::Waitable;
use thiserror::Error;
use vm_control::gpu::DisplayParameters;
use vm_control::gpu::MouseMode;
#[cfg(feature = "vulkan_display")]
use vulkano::VulkanLibrary;

mod event_device;
#[cfg(feature = "android_display")]
mod gpu_display_android;
#[cfg(feature = "android_display_stub")]
mod gpu_display_android_stub;
mod gpu_display_stub;
#[cfg(feature = "vnc")]
mod gpu_display_vnc;
#[cfg(feature = "vnc")]
pub use gpu_display_vnc::DisplayVnc;
#[cfg(windows)]
mod gpu_display_win;
#[cfg(any(target_os = "android", target_os = "linux"))]
mod gpu_display_wl;
#[cfg(feature = "x")]
mod gpu_display_x;
#[cfg(any(windows, feature = "x"))]
mod keycode_converter;
mod sys;
#[cfg(feature = "vnc")]
mod vnc_blit;
#[cfg(feature = "vnc")]
mod vnc_h264;
#[cfg(feature = "vulkan_display")]
pub mod vulkan;

pub use event_device::EventDevice;
pub use event_device::EventDeviceKind;
pub use event_device::VncBindingInput;
#[cfg(windows)]
pub use gpu_display_win::WindowProcedureThread;
#[cfg(windows)]
pub use gpu_display_win::WindowProcedureThreadBuilder;
use linux_input_sys::virtio_input_event;
use sys::SysDisplayT;
pub use sys::SysGpuDisplayExt;

// The number of bytes in a vulkan UUID.
#[cfg(feature = "vulkan_display")]
const VK_UUID_BYTES: usize = 16;

pub const fn drm_fourcc(a: u8, b: u8, c: u8, d: u8) -> u32 {
    (a as u32) | ((b as u32) << 8) | ((c as u32) << 16) | ((d as u32) << 24)
}

pub const DRM_FORMAT_XRGB8888: u32 = drm_fourcc(b'X', b'R', b'2', b'4');
pub const DRM_FORMAT_ARGB8888: u32 = drm_fourcc(b'A', b'R', b'2', b'4');
pub const DRM_FORMAT_XBGR8888: u32 = drm_fourcc(b'X', b'B', b'2', b'4');
pub const DRM_FORMAT_ABGR8888: u32 = drm_fourcc(b'A', b'B', b'2', b'4');
pub const DRM_FORMAT_RGB888: u32 = drm_fourcc(b'R', b'G', b'2', b'4');
pub const DRM_FORMAT_RGB565: u32 = drm_fourcc(b'R', b'G', b'1', b'6');

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum PixelLayout {
    /// B, G, R, X/A in memory on a little-endian host.
    BlueFirst8888,
    /// R, G, B, X/A in memory on a little-endian host.
    RedFirst8888,
    /// B, G, R in memory on a little-endian host.
    BlueFirst888,
    Rgb565,
}

impl PixelLayout {
    fn bytes_per_pixel(self) -> usize {
        match self {
            Self::BlueFirst8888 | Self::RedFirst8888 => 4,
            Self::BlueFirst888 => 3,
            Self::Rgb565 => 2,
        }
    }
}

fn pixel_layout(fourcc: u32) -> Option<PixelLayout> {
    match fourcc {
        DRM_FORMAT_XRGB8888 | DRM_FORMAT_ARGB8888 => Some(PixelLayout::BlueFirst8888),
        DRM_FORMAT_XBGR8888 | DRM_FORMAT_ABGR8888 => Some(PixelLayout::RedFirst8888),
        DRM_FORMAT_RGB888 => Some(PixelLayout::BlueFirst888),
        DRM_FORMAT_RGB565 => Some(PixelLayout::Rgb565),
        _ => None,
    }
}

fn convert_scanout_row(
    source_fourcc: u32,
    target_fourcc: u32,
    width: usize,
    source: &[u8],
    target: &mut [u8],
) -> bool {
    let Some(source_layout) = pixel_layout(source_fourcc) else {
        return false;
    };
    let Some(target_layout) = pixel_layout(target_fourcc) else {
        return false;
    };
    if !matches!(
        target_layout,
        PixelLayout::BlueFirst8888 | PixelLayout::RedFirst8888
    ) {
        return false;
    }

    let source_bpp = source_layout.bytes_per_pixel();
    if source.len() < width.saturating_mul(source_bpp)
        || target.len() < width.saturating_mul(4)
    {
        return false;
    }

    for pixel in 0..width {
        let source = &source[pixel * source_bpp..];
        let (r, g, b, a) = match source_layout {
            PixelLayout::BlueFirst8888 => (source[2], source[1], source[0], source[3]),
            PixelLayout::RedFirst8888 => (source[0], source[1], source[2], source[3]),
            PixelLayout::BlueFirst888 => (source[2], source[1], source[0], 0xff),
            PixelLayout::Rgb565 => {
                let packed = u16::from_le_bytes([source[0], source[1]]);
                let r = (((packed >> 11) & 0x1f) * 255 / 31) as u8;
                let g = (((packed >> 5) & 0x3f) * 255 / 63) as u8;
                let b = ((packed & 0x1f) * 255 / 31) as u8;
                (r, g, b, 0xff)
            }
        };
        let target = &mut target[pixel * 4..];
        match target_layout {
            PixelLayout::BlueFirst8888 => target[..4].copy_from_slice(&[b, g, r, a]),
            PixelLayout::RedFirst8888 => target[..4].copy_from_slice(&[r, g, b, a]),
            PixelLayout::BlueFirst888 | PixelLayout::Rgb565 => unreachable!(),
        }
    }
    true
}

#[derive(Clone)]
pub struct VulkanCreateParams {
    #[cfg(feature = "vulkan_display")]
    pub vulkan_library: std::sync::Arc<VulkanLibrary>,
    #[cfg(feature = "vulkan_display")]
    pub device_uuid: [u8; VK_UUID_BYTES],
    #[cfg(feature = "vulkan_display")]
    pub driver_uuid: [u8; VK_UUID_BYTES],
}

/// An error generated by `GpuDisplay`.
#[sorted]
#[derive(Error, Debug)]
pub enum GpuDisplayError {
    /// An internal allocation failed.
    #[error("internal allocation failed")]
    Allocate,
    /// A base error occurred.
    #[error("received a base error: {0}")]
    BaseError(BaseError),
    /// Connecting to the compositor failed.
    #[error("failed to connect to compositor")]
    Connect,
    /// Connection to compositor has been broken.
    #[error("connection to compositor has been broken")]
    ConnectionBroken,
    /// Creating event file descriptor failed.
    #[error("failed to create event file descriptor")]
    CreateEvent,
    /// Failed to create a surface on the compositor.
    #[error("failed to crate surface on the compositor")]
    CreateSurface,
    /// Failed to import an event device.
    #[error("failed to import an event device: {0}")]
    FailedEventDeviceImport(String),
    #[error("failed to register an event device to listen for guest events: {0}")]
    FailedEventDeviceListen(base::TubeError),
    /// Failed to import a buffer to the compositor.
    #[error("failed to import a buffer to the compositor")]
    FailedImport,
    /// Android display service name is invalid.
    #[error("invalid Android display service name: {0}")]
    InvalidAndroidDisplayServiceName(String),
    /// The import ID is invalid.
    #[error("invalid import ID")]
    InvalidImportId,
    /// The path is invalid.
    #[error("invalid path")]
    InvalidPath,
    /// The surface ID is invalid.
    #[error("invalid surface ID")]
    InvalidSurfaceId,
    /// An input/output error occured.
    #[error("an input/output error occur: {0}")]
    IoError(IoError),
    /// A required feature was missing.
    #[error("required feature was missing: {0}")]
    RequiredFeature(&'static str),
    /// The method is unsupported by the implementation.
    #[error("unsupported by the implementation")]
    Unsupported,
}

pub type GpuDisplayResult<T> = std::result::Result<T, GpuDisplayError>;

impl From<BaseError> for GpuDisplayError {
    fn from(e: BaseError) -> GpuDisplayError {
        GpuDisplayError::BaseError(e)
    }
}

impl From<IoError> for GpuDisplayError {
    fn from(e: IoError) -> GpuDisplayError {
        GpuDisplayError::IoError(e)
    }
}

/// A surface type
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SurfaceType {
    /// Scanout surface
    Scanout,
    /// Mouse cursor surface
    Cursor,
}

/// Event token for display instances
#[derive(EventToken, Debug)]
pub enum DisplayEventToken {
    Display,
    EventDevice { event_device_id: u32 },
}

#[derive(Clone)]
pub struct GpuDisplayFramebuffer<'a> {
    framebuffer: VolatileSlice<'a>,
    slice: VolatileSlice<'a>,
    stride: u32,
    bytes_per_pixel: u32,
    fourcc: u32,
}

impl<'a> GpuDisplayFramebuffer<'a> {
    fn new(
        framebuffer: VolatileSlice<'a>,
        stride: u32,
        bytes_per_pixel: u32,
        fourcc: u32,
    ) -> GpuDisplayFramebuffer<'a> {
        GpuDisplayFramebuffer {
            framebuffer,
            slice: framebuffer,
            stride,
            bytes_per_pixel,
            fourcc,
        }
    }

    fn sub_region(
        &self,
        x: u32,
        y: u32,
        width: u32,
        height: u32,
    ) -> Option<GpuDisplayFramebuffer<'a>> {
        if width == 0 || height == 0 {
            return None;
        }

        let x_byte_offset = x.checked_mul(self.bytes_per_pixel)?;
        let y_byte_offset = y.checked_mul(self.stride)?;
        let byte_offset = x_byte_offset.checked_add(y_byte_offset)?;

        let width_bytes = width.checked_mul(self.bytes_per_pixel)?;
        let count = height
            .checked_mul(self.stride)?
            .checked_sub(self.stride)?
            .checked_add(width_bytes)?;
        let slice = self
            .framebuffer
            .sub_slice(byte_offset as usize, count as usize)
            .ok()?;

        Some(GpuDisplayFramebuffer { slice, ..*self })
    }

    pub fn as_volatile_slice(&self) -> VolatileSlice<'a> {
        self.slice
    }

    pub fn stride(&self) -> u32 {
        self.stride
    }

    pub fn fourcc(&self) -> u32 {
        self.fourcc
    }

    /// Whether a producer can write this framebuffer without changing pixel layout.
    ///
    /// X-versus-alpha does not require a conversion: both variants put their fourth byte in the
    /// same place. Only the red/blue order matters for the 32-bit CPU fast path.
    pub fn can_copy_direct_from(&self, source_fourcc: u32) -> bool {
        self.bytes_per_pixel == 4
            && matches!(
                (pixel_layout(source_fourcc), pixel_layout(self.fourcc)),
                (Some(PixelLayout::BlueFirst8888), Some(PixelLayout::BlueFirst8888))
                    | (Some(PixelLayout::RedFirst8888), Some(PixelLayout::RedFirst8888))
            )
    }

    /// Copies one CPU-produced frame into this sink framebuffer.
    ///
    /// The frame and framebuffer each describe their real layout. Conversion, when needed, is an
    /// edge operation: producers do not normalize into a global byte order and sinks do not apply
    /// an unconditional post-process after receiving the frame.
    pub fn copy_from_frame(&self, frame: &ScanoutFrame) {
        let dst_stride = self.stride as usize;
        let dst_bpp = self.bytes_per_pixel as usize;
        let dst = self.as_volatile_slice();
        let dst_rows = if dst_stride == 0 || dst.size() == 0 {
            0
        } else {
            (dst.size() - 1) / dst_stride + 1
        };
        let src_stride = frame.stride as usize;
        let source_layout = pixel_layout(frame.fourcc);
        let target_layout = pixel_layout(self.fourcc);
        let source_bpp = source_layout
            .map(PixelLayout::bytes_per_pixel)
            .unwrap_or(dst_bpp);
        let convertible = source_layout.is_some()
            && matches!(
                target_layout,
                Some(PixelLayout::BlueFirst8888 | PixelLayout::RedFirst8888)
            )
            && dst_bpp == 4;
        let direct = self.can_copy_direct_from(frame.fourcc);
        let pixels = (frame.width as usize)
            .min(src_stride.checked_div(source_bpp).unwrap_or(0))
            .min(dst_stride.checked_div(dst_bpp).unwrap_or(0));
        let source_row_bytes = pixels.saturating_mul(source_bpp);
        let target_row_bytes = pixels.saturating_mul(dst_bpp);
        let rows = (frame.height as usize).min(dst_rows);
        let mut converted = if convertible && !direct {
            vec![0; target_row_bytes]
        } else {
            Vec::new()
        };

        let Damage::Full = frame.damage;
        for row in 0..rows {
            let source_offset = row.saturating_mul(src_stride);
            let Some(source_end) = source_offset.checked_add(source_row_bytes) else {
                break;
            };
            if source_end > frame.bytes.len() {
                break;
            }
            let source = &frame.bytes[source_offset..source_end];

            let (bytes, bytes_to_write) = if direct {
                (source, target_row_bytes)
            } else if convertible
                && convert_scanout_row(
                    frame.fourcc,
                    self.fourcc,
                    pixels,
                    source,
                    &mut converted,
                )
            {
                (converted.as_slice(), target_row_bytes)
            } else {
                // Preserve the historical raw-copy fallback for formats this CPU edge does not
                // understand yet. The declared format is still retained, so adding a converter
                // later does not require changing either producer or sink.
                (source, source_row_bytes.min(target_row_bytes))
            };
            match dst.sub_slice(row * dst_stride, bytes_to_write) {
                Ok(target) => target.copy_from(&bytes[..bytes_to_write]),
                Err(_) => break,
            }
        }
    }
}

/// How much of a `ScanoutFrame` is new since the sink last saw one.
///
/// `Full` is the only variant today, and that is what makes this step checkable: the acceptance
/// instrument compares the bytes each sink received, and a frame that always carries everything
/// leaves a damage bug nowhere to hide. Rectangles arrive with the content watcher, which is what
/// first has an answer narrower than "all of it".
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Damage {
    Full,
}

/// A produced frame offered to a display sink.
///
/// The single currency between the frame producers and the CPU-copy sink boundary. Before it there
/// were three call sites doing the same fetch-copy-flip inline, each with its own idea of how many
/// bytes a row is, and no one place to change when the pipeline grows a second kind of frame.
///
/// CPU-only today: the GPU import/`flip_to` path does not come through here, and this grows a
/// `Dmabuf` variant at the udmabuf step rather than pretending to cover it now.
pub struct ScanoutFrame<'a> {
    /// The pixels, top row first. `bytes[0]` is the frame's top-left pixel: a producer whose frame
    /// begins partway into a larger buffer passes the subslice rather than an offset, so nothing
    /// downstream has to carry an origin it cannot check.
    pub bytes: &'a [u8],
    /// Distance in bytes from the start of one row of `bytes` to the start of the next. May exceed
    /// the visible row.
    pub stride: u32,
    pub width: u32,
    pub height: u32,
    /// DRM fourcc of `bytes`. This is the producer's actual layout, not a pipeline-wide canonical
    /// format. The CPU edge compares it with the sink framebuffer's fourcc and performs at most one
    /// conversion while copying the frame.
    pub fourcc: u32,
    pub damage: Damage,
}

/// What `present_frame` did.
pub enum PresentOutcome {
    /// The frame was copied into the surface's framebuffer and the surface was flipped.
    Flipped,
    /// The sink handed out no framebuffer: nothing was copied and nothing was flipped. What that
    /// means is the caller's to decide, and it differs -- a frame dropped by a timer-driven
    /// producer is offered again on the next tick, one dropped by a guest flush never comes back.
    NoFramebuffer,
}

trait GpuDisplaySurface {
    /// Returns an unique ID associated with the surface.  This is typically generated by the
    /// compositor or cast of a raw pointer.
    fn surface_descriptor(&self) -> u64 {
        0
    }

    /// Returns the next framebuffer, allocating if necessary.
    fn framebuffer(&mut self) -> Option<GpuDisplayFramebuffer> {
        None
    }

    /// Returns true if the next buffer in the swapchain is already in use.
    fn next_buffer_in_use(&self) -> bool {
        false
    }

    /// Returns true if the surface should be closed.
    fn close_requested(&self) -> bool {
        false
    }

    /// Puts the next buffer on the screen, making it the current buffer.
    fn flip(&mut self) {
        // no-op
    }

    /// Puts the specified import_id on the screen.
    fn flip_to(
        &mut self,
        _import_id: u32,
        _acquire_timepoint: Option<SemaphoreTimepoint>,
        _release_timepoint: Option<SemaphoreTimepoint>,
        _extra_info: Option<FlipToExtraInfo>,
    ) -> anyhow::Result<Waitable> {
        // no-op
        Ok(Waitable::signaled())
    }

    /// Takes the completion fence of the most recent `flip_to`, when the backend has one.
    ///
    /// A `Some` fd is a sync_file that signals when the display is done *reading* the flipped
    /// buffer, i.e. when the guest may safely render into it again. Backends whose flip consumes
    /// the buffer synchronously return `None`, and the caller must then complete the flush
    /// synchronously exactly as before: the CPU-copy paths, which never reach `flip_to` at all;
    /// wayland, whose commits are decoupled by wl_buffer semantics; and the VNC sink, which does
    /// reach `flip_to` but has finished reading its source by the time it returns -- deliberately,
    /// because a `Some` there would put a network service in the guest's vblank loop.
    fn take_flip_completion_fence(&mut self) -> Option<SafeDescriptor> {
        None
    }

    /// Commits the surface to the compositor.
    fn commit(&mut self) -> GpuDisplayResult<()> {
        Ok(())
    }

    /// Sets the mouse mode used on this surface.
    fn set_mouse_mode(&mut self, _mouse_mode: MouseMode) {
        // no-op
    }

    /// Sets the position of the identified subsurface relative to its parent.
    ///
    /// For a cursor surface this is the TOP-LEFT CORNER of the cursor image, not where the pointer
    /// points: virtio-gpu carries the guest's `crtc_x/crtc_y`, and the guest has already subtracted
    /// the hotspot. A backend that treats it as the pointer position and subtracts the hotspot
    /// again draws the cursor up and left of the truth by exactly the hotspot -- barely visible on
    /// an arrow (5,5), a very visible 22px on a resize arrow.
    ///
    /// Signed because `crtc_x` is: a pointer within `hot_x` of the left edge puts the image origin
    /// off-screen, and the wire carries that as a negative number in a `__le32` field.
    fn set_position(&mut self, _x: i32, _y: i32) {
        // no-op
    }

    /// Sets the DRM FourCC describing bytes written through `framebuffer`.
    fn set_buffer_fourcc(&mut self, _fourcc: u32) {
        // no-op
    }

    /// Sets the cursor hotspot: where inside the cursor image the pointer actually points.
    ///
    /// virtio-gpu carries this on every UPDATE_CURSOR and it is not optional dressing -- get it
    /// wrong and an I-beam or a resize arrow clicks somewhere other than where it looks like it
    /// points. Only backends that draw a real cursor need it, so the default ignores it.
    fn set_cursor_hotspot(&mut self, _hot_x: u32, _hot_y: u32) {
        // no-op
    }

    /// Show or hide the cursor this surface carries.
    ///
    /// The guest hides its pointer with UPDATE_CURSOR resource_id=0 -- switching to a text console
    /// does exactly that. Without a signal the backend keeps presenting the last cursor image it
    /// was given, so the pointer lingers on a console that should have none.
    fn set_cursor_visible(&mut self, _visible: bool) {
        // no-op
    }

    /// Returns the type of the completed buffer.
    #[allow(dead_code)]
    fn buffer_completion_type(&self) -> u32 {
        0
    }

    /// Draws the current buffer on the screen.
    #[allow(dead_code)]
    fn draw_current_buffer(&mut self) {
        // no-op
    }

    /// Handles a compositor-specific client event.
    #[allow(dead_code)]
    fn on_client_message(&mut self, _client_data: u64) {
        // no-op
    }

    /// Handles a compositor-specific shared memory completion event.
    #[allow(dead_code)]
    fn on_shm_completion(&mut self, _shm_complete: u64) {
        // no-op
    }
}

struct GpuDisplayEvents {
    events: Vec<virtio_input_event>,
    device_type: EventDeviceKind,
}

trait DisplayT: AsRawDescriptor {
    /// Returns true if there are events that are on the queue.
    fn pending_events(&self) -> bool {
        false
    }

    /// Sends any pending commands to the compositor.
    fn flush(&self) {
        // no-op
    }

    /// Returns the surface descirptor associated with the current event
    fn next_event(&mut self) -> GpuDisplayResult<u64> {
        Ok(0)
    }

    /// Handles the event from the compositor, and returns an list of events
    fn handle_next_event(
        &mut self,
        _surface: &mut Box<dyn GpuDisplaySurface>,
    ) -> Option<GpuDisplayEvents> {
        None
    }

    /// Handles the next pending event without a surface context.  Used for input events that
    /// can be dispatched regardless of whether a display surface exists.
    fn handle_next_event_without_surface(&mut self) -> Option<GpuDisplayEvents> {
        None
    }

    /// Returns whether this backend can import DMA-BUF resources. Backends with a runtime
    /// capability probe should cache the result for the lifetime of the display.
    fn is_dmabuf_import_supported(&mut self) -> bool {
        true
    }

    /// Whether anything is currently positioned to see a frame pushed to this backend.
    ///
    /// A producer that has to build a frame before it can offer one -- the simplefb bridge copies
    /// a whole framebuffer out of guest memory on a timer, whether or not the far end exists --
    /// can ask this first and skip the work entirely. `false` must mean "a frame pushed now
    /// reaches nobody", never "probably idle": a producer is entitled to drop the frame outright
    /// on the strength of this answer.
    ///
    /// The default is `true`, which is the answer for any backend whose output always has a
    /// destination. Only a backend that can genuinely have none -- VNC with no client connected --
    /// should override it.
    fn has_consumer(&self) -> bool {
        true
    }

    /// A number that changes whenever the set of consumers behind this sink changes.
    ///
    /// `has_consumer` is a bool, and a bool cannot say "a DIFFERENT consumer arrived while another
    /// one was already there". That distinction did not exist while every sink fed one kind of
    /// client; the VNC sink now feeds two, a client on the pixel path and one on the H.264 stream,
    /// and they arrive independently.
    ///
    /// It matters because of what a producer does with a consumer arriving: it re-supplies a frame
    /// that content-wise did not change, because content that sat still while nobody watched
    /// hashes as unchanged and the arriving viewer would otherwise wait for the guest to paint
    /// something -- possibly forever. A second kind of client arriving needs exactly the same
    /// treatment, and on the bool alone it is invisible: the flag was already true.
    ///
    /// The default never changes, which is right for every backend that cannot tell its consumers
    /// apart: those are covered by the `has_consumer` edge alone, as they always were.
    fn consumer_generation(&self) -> u64 {
        0
    }

    /// Creates a surface with the given parameters.  The display backend is given a non-zero
    /// `surface_id` as a handle for subsequent operations.
    fn create_surface(
        &mut self,
        parent_surface_id: Option<u32>,
        surface_id: u32,
        scanout_id: Option<u32>,
        display_params: &DisplayParameters,
        surf_type: SurfaceType,
    ) -> GpuDisplayResult<Box<dyn GpuDisplaySurface>>;

    /// Imports a resource into the display backend.  The display backend is given a non-zero
    /// `import_id` as a handle for subsequent operations.
    fn import_resource(
        &mut self,
        _import_id: u32,
        _surface_id: u32,
        _external_display_resource: DisplayExternalResourceImport,
    ) -> anyhow::Result<()> {
        Err(anyhow!("import_resource is unsupported"))
    }

    /// Frees a previously imported resource.
    fn release_import(&mut self, _import_id: u32, _surface_id: u32) {}
}

pub trait GpuDisplayExt {
    /// Imports the given `event_device` into the display, returning an event device id on success.
    /// This device may be used to dispatch input events to the guest.
    fn import_event_device(&mut self, event_device: EventDevice) -> GpuDisplayResult<u32>;

    /// Called when an event device is readable.
    fn handle_event_device(&mut self, event_device_id: u32);
}

pub enum DisplayExternalResourceImport<'a> {
    Dmabuf {
        descriptor: &'a dyn AsRawDescriptor,
        offset: u32,
        stride: u32,
        modifiers: u64,
        /// True only when the producer has verified that this single-plane
        /// DMA-BUF contains a linear image with the supplied layout.
        linear_layout_verified: bool,
        width: u32,
        height: u32,
        fourcc: u32,
    },
    VulkanImage {
        descriptor: &'a dyn AsRawDescriptor,
        metadata: VulkanDisplayImageImportMetadata,
    },
    VulkanTimelineSemaphore {
        descriptor: &'a dyn AsRawDescriptor,
    },
}

pub struct VkExtent3D {
    pub width: u32,
    pub height: u32,
    pub depth: u32,
}

pub struct VulkanDisplayImageImportMetadata {
    // These fields go into a VkImageCreateInfo
    pub flags: u32,
    pub image_type: i32,
    pub format: i32,
    pub extent: VkExtent3D,
    pub mip_levels: u32,
    pub array_layers: u32,
    pub samples: u32,
    pub tiling: i32,
    pub usage: u32,
    pub sharing_mode: i32,
    pub queue_family_indices: Vec<u32>,
    pub initial_layout: i32,

    // These fields go into a VkMemoryAllocateInfo
    pub allocation_size: u64,
    pub memory_type_index: u32,

    // Additional information
    pub dedicated_allocation: bool,
}

pub struct SemaphoreTimepoint {
    pub import_id: u32,
    pub value: u64,
}

pub enum FlipToExtraInfo {
    #[cfg(feature = "vulkan_display")]
    Vulkan { old_layout: i32, new_layout: i32 },
}

/// A connection to the compositor and associated collection of state.
///
/// The user of `GpuDisplay` can use `AsRawDescriptor` to poll on the compositor connection's file
/// descriptor. When the connection is readable, `dispatch_events` can be called to process it.
pub struct GpuDisplay {
    next_id: u32,
    event_devices: BTreeMap<u32, EventDevice>,
    surfaces: BTreeMap<u32, Box<dyn GpuDisplaySurface>>,
    /// Whether this display has been capped to the CPU transport (`cap_transport_to_cpu`).
    ///
    /// Deliberately here and not in the backends. The cap is a property of one *binding* -- this
    /// exporter, on this screen -- while a backend type is a property of a kind of sink, and every
    /// backend would otherwise have to grow the same field and remember to consult it. Holding it
    /// on the wrapper puts it at the two places a producer can learn about or use dmabuf import,
    /// `is_dmabuf_import_supported` and `import_resource`, which is what makes it impossible to
    /// route around: a producer that skips the probe still cannot get an import id.
    dmabuf_import_capped: bool,
    wait_ctx: WaitContext<DisplayEventToken>,
    // `inner` must be after `surfaces` to ensure those objects are dropped before
    // the display context. The drop order for fields inside a struct is the order in which they
    // are declared [Rust RFC 1857].
    //
    // We also don't want to drop inner before wait_ctx because it contains references to the event
    // devices owned by inner.display_event_dispatcher.
    inner: Box<dyn SysDisplayT>,
}

impl GpuDisplay {
    /// Opens a connection to X server
    pub fn open_x(display_name: Option<&str>) -> GpuDisplayResult<GpuDisplay> {
        let _ = display_name;
        #[cfg(feature = "x")]
        {
            let display = gpu_display_x::DisplayX::open_display(display_name)?;

            let wait_ctx = WaitContext::new()?;
            wait_ctx.add(&display, DisplayEventToken::Display)?;

            Ok(GpuDisplay {
                inner: Box::new(display),
                next_id: 1,
                event_devices: Default::default(),
                surfaces: Default::default(),
                dmabuf_import_capped: false,
                wait_ctx,
            })
        }
        #[cfg(not(feature = "x"))]
        Err(GpuDisplayError::Unsupported)
    }

    pub fn open_android(service_name: &str) -> GpuDisplayResult<GpuDisplay> {
        let _ = service_name;
        #[cfg(feature = "android_display")]
        {
            let display = gpu_display_android::DisplayAndroid::new(service_name)?;

            let wait_ctx = WaitContext::new()?;
            wait_ctx.add(&display, DisplayEventToken::Display)?;

            Ok(GpuDisplay {
                inner: Box::new(display),
                next_id: 1,
                event_devices: Default::default(),
                surfaces: Default::default(),
                dmabuf_import_capped: false,
                wait_ctx,
            })
        }
        #[cfg(not(feature = "android_display"))]
        Err(GpuDisplayError::Unsupported)
    }

    pub fn open_stub() -> GpuDisplayResult<GpuDisplay> {
        let display = gpu_display_stub::DisplayStub::new()?;
        let wait_ctx = WaitContext::new()?;
        wait_ctx.add(&display, DisplayEventToken::Display)?;

        Ok(GpuDisplay {
            inner: Box::new(display),
            next_id: 1,
            event_devices: Default::default(),
            surfaces: Default::default(),
            dmabuf_import_capped: false,
            wait_ctx,
        })
    }

    /// `tablet`/`keyboard` are this binding's own input devices; see `DisplayVnc::new_tcp`. They are
    /// deliberately NOT imported as this display's event devices: the VNC backend delivers to them
    /// itself, because the event-device map is scoped to a display owner and these belong to a
    /// binding. Both `None` is a view-only binding.
    #[cfg(feature = "vnc")]
    pub fn open_vnc_tcp(
        addr: &str,
        width: u32,
        height: u32,
        password: Option<String>,
        hw_encode: bool,
        tablet: Option<EventDevice>,
        keyboard: Option<EventDevice>,
    ) -> GpuDisplayResult<GpuDisplay> {
        let display = gpu_display_vnc::DisplayVnc::new_tcp(
            addr,
            width,
            height,
            password,
            hw_encode,
            tablet,
            keyboard,
        )?;

        let wait_ctx = WaitContext::new()?;
        wait_ctx.add(&display, DisplayEventToken::Display)?;

        Ok(GpuDisplay {
            inner: Box::new(display),
            next_id: 1,
            event_devices: Default::default(),
            surfaces: Default::default(),
            dmabuf_import_capped: false,
            wait_ctx,
        })
    }

    // Leaves the `GpuDisplay` in a undefined state.
    //
    // TODO: Would be nice to change receiver from `&mut self` to `self`. Requires some refactoring
    // elsewhere.
    pub fn take_event_devices(&mut self) -> Vec<EventDevice> {
        std::mem::take(&mut self.event_devices)
            .into_values()
            .collect()
    }

    fn dispatch_display_events(&mut self) -> GpuDisplayResult<()> {
        self.inner.flush();
        while self.inner.pending_events() {
            let surface_descriptor = self.inner.next_event()?;

            let mut matched = false;
            for surface in self.surfaces.values_mut() {
                if surface_descriptor != surface.surface_descriptor() {
                    continue;
                }

                matched = true;
                if let Some(gpu_display_events) = self.inner.handle_next_event(surface) {
                    for event_device in self.event_devices.values_mut() {
                        if event_device.kind() != gpu_display_events.device_type {
                            continue;
                        }

                        event_device.send_report(gpu_display_events.events.iter().cloned())?;
                    }
                }
            }

            if !matched {
                if let Some(gpu_display_events) = self.inner.handle_next_event_without_surface() {
                    for event_device in self.event_devices.values_mut() {
                        if event_device.kind() != gpu_display_events.device_type {
                            continue;
                        }

                        event_device.send_report(gpu_display_events.events.iter().cloned())?;
                    }
                }
            }
        }

        Ok(())
    }

    /// Dispatches internal events that were received from the compositor since the last call to
    /// `dispatch_events`.
    pub fn dispatch_events(&mut self) -> GpuDisplayResult<()> {
        let wait_events = self.wait_ctx.wait_timeout(Duration::default())?;

        for wait_event in wait_events.iter().filter(|e| e.is_hungup) {
            match wait_event.token {
                DisplayEventToken::Display => {
                    base::error!(
                        "Display signaled with a hungup event for token {:?}",
                        wait_event.token
                    );
                    self.wait_ctx = WaitContext::new().unwrap();
                    return GpuDisplayResult::Err(GpuDisplayError::ConnectionBroken);
                }
                DisplayEventToken::EventDevice { event_device_id } => {
                    base::info!(
                        "Event device {} hungup, removing from display",
                        event_device_id
                    );
                    if let Some(event_device) = self.event_devices.remove(&event_device_id) {
                        let _ = self.wait_ctx.delete(&event_device);
                    }
                }
            }
        }

        for wait_event in wait_events.iter().filter(|e| e.is_writable) {
            if let DisplayEventToken::EventDevice { event_device_id } = wait_event.token {
                if let Some(event_device) = self.event_devices.get_mut(&event_device_id) {
                    if !event_device.flush_buffered_events()? {
                        continue;
                    }
                    self.wait_ctx.modify(
                        event_device,
                        EventType::Read,
                        DisplayEventToken::EventDevice { event_device_id },
                    )?;
                }
            }
        }

        for wait_event in wait_events.iter().filter(|e| e.is_readable) {
            match wait_event.token {
                DisplayEventToken::Display => self.dispatch_display_events()?,
                DisplayEventToken::EventDevice { event_device_id } => {
                    self.handle_event_device(event_device_id)
                }
            }
        }

        Ok(())
    }

    /// Creates a surface on the the compositor as either a top level window, or child of another
    /// surface, returning a handle to the new surface.
    pub fn create_surface(
        &mut self,
        parent_surface_id: Option<u32>,
        scanout_id: Option<u32>,
        display_params: &DisplayParameters,
        surf_type: SurfaceType,
    ) -> GpuDisplayResult<u32> {
        if let Some(parent_id) = parent_surface_id {
            if !self.surfaces.contains_key(&parent_id) {
                return Err(GpuDisplayError::InvalidSurfaceId);
            }
        }

        let new_surface_id = self.next_id;
        let new_surface = self.inner.create_surface(
            parent_surface_id,
            new_surface_id,
            scanout_id,
            display_params,
            surf_type,
        )?;

        self.next_id += 1;
        self.surfaces.insert(new_surface_id, new_surface);
        Ok(new_surface_id)
    }

    /// Releases a previously created surface identified by the given handle.
    pub fn release_surface(&mut self, surface_id: u32) {
        self.surfaces.remove(&surface_id);
    }

    /// Gets a reference to an unused framebuffer for the identified surface.
    pub fn framebuffer(&mut self, surface_id: u32) -> Option<GpuDisplayFramebuffer> {
        let surface = self.surfaces.get_mut(&surface_id)?;
        surface.framebuffer()
    }

    /// Gets a reference to an unused framebuffer for the identified surface.
    pub fn framebuffer_region(
        &mut self,
        surface_id: u32,
        x: u32,
        y: u32,
        width: u32,
        height: u32,
    ) -> Option<GpuDisplayFramebuffer> {
        let framebuffer = self.framebuffer(surface_id)?;
        framebuffer.sub_region(x, y, width, height)
    }

    /// Returns true if the next buffer in the buffer queue for the given surface is currently in
    /// use.
    ///
    /// If the next buffer is in use, the memory returned from `framebuffer_memory` should not be
    /// written to.
    pub fn next_buffer_in_use(&self, surface_id: u32) -> bool {
        self.surfaces
            .get(&surface_id)
            .map(|s| s.next_buffer_in_use())
            .unwrap_or(false)
    }

    /// Changes the visible contents of the identified surface to the contents of the framebuffer
    /// last returned by `framebuffer_memory` for this surface.
    pub fn flip(&mut self, surface_id: u32) {
        if let Some(surface) = self.surfaces.get_mut(&surface_id) {
            surface.flip()
        }
    }

    /// Copies a produced frame into the identified surface's framebuffer and flips it.
    ///
    /// The one place the CPU pipeline turns a produced frame into pixels a sink can post. It does
    /// fetches the surface's framebuffer, copies the rows in, and flips. The framebuffer declares
    /// the sink's real pixel layout; when it differs from `ScanoutFrame::fourcc`, the copy and
    /// conversion happen at this one boundary instead of as producer normalization plus a second
    /// sink-side full-frame pass.
    ///
    /// The copy honours both strides and stops at the smaller rectangle in each axis, row by row.
    /// That is what makes a gralloc-padded destination and a packed source line up instead of
    /// shearing, and it is why the row length is a minimum of three things rather than a memcpy of
    /// whichever buffer is shorter.
    ///
    /// Decisions that belong to a particular producer stay with it: whether a busy swapchain buffer
    /// means "drop this frame", whether a surface has to be created first, what a missing
    /// framebuffer means. None of those are visible from here.
    pub fn present_frame(&mut self, surface_id: u32, frame: &ScanoutFrame) -> PresentOutcome {
        {
            let fb = match self.framebuffer(surface_id) {
                Some(fb) => fb,
                None => return PresentOutcome::NoFramebuffer,
            };
            fb.copy_from_frame(frame);
        }
        self.flip(surface_id);
        PresentOutcome::Flipped
    }

    /// Returns true if the identified top level surface has been told to close by the compositor,
    /// and by extension the user.
    pub fn close_requested(&self, surface_id: u32) -> bool {
        self.surfaces
            .get(&surface_id)
            .map(|s| s.close_requested())
            .unwrap_or(true)
    }

    /// Refuses dmabuf import on this display for the rest of its life, whatever the backend can
    /// actually do.
    ///
    /// This is the enforcement point for a binding's `transport-cap=cpu`. Capping only ever
    /// removes an option from a negotiation whose floor is a CPU copy, so it cannot fail and there
    /// is no matching "uncap": a caller that wants the negotiated answer simply never calls this.
    ///
    /// One-way on purpose. The value it enforces is fixed when the exporter is configured, and a
    /// display that could be uncapped mid-run would mean a producer's cached verdict (both the
    /// virtio-gpu `CpuFallback` cache and the simplefb bridge's `Transport`) could disagree with
    /// the display about which transport is in force, with no event to reconcile them.
    pub fn cap_transport_to_cpu(&mut self) {
        self.dmabuf_import_capped = true;
    }

    /// Imports a resource to the display backend. This resource may be an image for the compositor
    /// or a synchronization object.
    pub fn import_resource(
        &mut self,
        surface_id: u32,
        external_display_resource: DisplayExternalResourceImport,
    ) -> anyhow::Result<u32> {
        // Checked here and not only in the probe below, so that the cap holds against a producer
        // that never probed. A refusal is what every caller of this already knows how to handle --
        // it is the same answer a backend without a GPU half gives -- so the cap needs no new path
        // anywhere upstream.
        if self.dmabuf_import_capped {
            return Err(anyhow!(
                "dmabuf import is capped off on this display (transport-cap=cpu)"
            ));
        }
        let import_id = self.next_id;

        self.inner
            .import_resource(import_id, surface_id, external_display_resource)?;

        self.next_id += 1;
        Ok(import_id)
    }

    /// Returns whether the display backend can import DMA-BUF resources.
    pub fn is_dmabuf_import_supported(&mut self) -> bool {
        // The cap answers first, and the backend is not asked at all: on the Android sink the
        // honest answer costs a Vulkan capability probe, and the point of the cap is that nothing
        // downstream is going to act on it.
        !self.dmabuf_import_capped && self.inner.is_dmabuf_import_supported()
    }

    /// Whether anything is currently positioned to see a frame. See `DisplayT::has_consumer`.
    pub fn has_consumer(&self) -> bool {
        self.inner.has_consumer()
    }

    /// See `DisplayT::consumer_generation`.
    pub fn consumer_generation(&self) -> u64 {
        self.inner.consumer_generation()
    }

    /// Releases a previously imported resource identified by the given handle.
    pub fn release_import(&mut self, import_id: u32, surface_id: u32) {
        self.inner.release_import(import_id, surface_id);
    }

    /// Commits any pending state for the identified surface.
    pub fn commit(&mut self, surface_id: u32) -> GpuDisplayResult<()> {
        let surface = self
            .surfaces
            .get_mut(&surface_id)
            .ok_or(GpuDisplayError::InvalidSurfaceId)?;

        surface.commit()
    }

    /// Changes the visible contents of the identified surface to that of the identified imported
    /// buffer.
    pub fn flip_to(
        &mut self,
        surface_id: u32,
        import_id: u32,
        acquire_timepoint: Option<SemaphoreTimepoint>,
        release_timepoint: Option<SemaphoreTimepoint>,
        extra_info: Option<FlipToExtraInfo>,
    ) -> anyhow::Result<Waitable> {
        let surface = self
            .surfaces
            .get_mut(&surface_id)
            .ok_or(GpuDisplayError::InvalidSurfaceId)?;

        surface
            .flip_to(import_id, acquire_timepoint, release_timepoint, extra_info)
            .context("failed in flip on GpuDisplaySurface")
    }

    /// Takes the completion fence of the identified surface's most recent flip, if the backend
    /// produced one (see `GpuDisplaySurface::take_flip_completion_fence`).
    pub fn take_flip_completion_fence(&mut self, surface_id: u32) -> Option<SafeDescriptor> {
        self.surfaces
            .get_mut(&surface_id)
            .and_then(|surface| surface.take_flip_completion_fence())
    }

    /// Sets the mouse mode used on this surface.
    pub fn set_mouse_mode(
        &mut self,
        surface_id: u32,
        mouse_mode: MouseMode,
    ) -> GpuDisplayResult<()> {
        let surface = self
            .surfaces
            .get_mut(&surface_id)
            .ok_or(GpuDisplayError::InvalidSurfaceId)?;

        surface.set_mouse_mode(mouse_mode);
        Ok(())
    }

    /// Sets the position of the identified subsurface relative to its parent.
    ///
    /// The change in position will not be visible until `commit` is called for the parent surface.
    pub fn set_position(&mut self, surface_id: u32, x: i32, y: i32) -> GpuDisplayResult<()> {
        let surface = self
            .surfaces
            .get_mut(&surface_id)
            .ok_or(GpuDisplayError::InvalidSurfaceId)?;

        surface.set_position(x, y);
        Ok(())
    }

    /// Sets the DRM FourCC describing CPU fallback framebuffer contents.
    pub fn set_buffer_fourcc(&mut self, surface_id: u32, fourcc: u32) -> GpuDisplayResult<()> {
        let surface = self
            .surfaces
            .get_mut(&surface_id)
            .ok_or(GpuDisplayError::InvalidSurfaceId)?;

        surface.set_buffer_fourcc(fourcc);
        Ok(())
    }

    /// Sets the cursor hotspot on the identified surface. See
    /// `GpuDisplaySurface::set_cursor_hotspot`.
    pub fn set_cursor_hotspot(
        &mut self,
        surface_id: u32,
        hot_x: u32,
        hot_y: u32,
    ) -> GpuDisplayResult<()> {
        let surface = self
            .surfaces
            .get_mut(&surface_id)
            .ok_or(GpuDisplayError::InvalidSurfaceId)?;

        surface.set_cursor_hotspot(hot_x, hot_y);
        Ok(())
    }

    /// See `GpuDisplaySurface::set_cursor_visible`.
    pub fn set_cursor_visible(&mut self, surface_id: u32, visible: bool) -> GpuDisplayResult<()> {
        let surface = self
            .surfaces
            .get_mut(&surface_id)
            .ok_or(GpuDisplayError::InvalidSurfaceId)?;

        surface.set_cursor_visible(visible);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn converts_between_red_first_and_blue_first_8888() {
        let rgba = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
        let mut bgra = [0; 8];
        assert!(convert_scanout_row(
            DRM_FORMAT_ABGR8888,
            DRM_FORMAT_XRGB8888,
            2,
            &rgba,
            &mut bgra,
        ));
        assert_eq!(bgra, [0x33, 0x22, 0x11, 0x44, 0x77, 0x66, 0x55, 0x88]);

        let mut round_trip = [0; 8];
        assert!(convert_scanout_row(
            DRM_FORMAT_ARGB8888,
            DRM_FORMAT_ABGR8888,
            2,
            &bgra,
            &mut round_trip,
        ));
        assert_eq!(round_trip, rgba);
    }

    #[test]
    fn preserves_rows_when_8888_orders_match() {
        let source = [0x33, 0x22, 0x11, 0x44];
        let mut target = [0; 4];
        assert!(convert_scanout_row(
            DRM_FORMAT_ARGB8888,
            DRM_FORMAT_XRGB8888,
            1,
            &source,
            &mut target,
        ));
        assert_eq!(target, source);
    }

    #[test]
    fn expands_rgb565_into_sink_layout() {
        let source = [0x00, 0xf8, 0xe0, 0x07, 0x1f, 0x00];
        let mut target = [0; 12];
        assert!(convert_scanout_row(
            DRM_FORMAT_RGB565,
            DRM_FORMAT_ABGR8888,
            3,
            &source,
            &mut target,
        ));
        assert_eq!(
            target,
            [0xff, 0x00, 0x00, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0x00, 0xff, 0xff]
        );
    }

    #[test]
    fn framebuffer_copy_converts_once_and_honors_both_strides() {
        let source = [
            1, 2, 3, 4, 5, 6, 7, 8, 0xaa, 0xaa, 0xaa, 0xaa, 9, 10, 11, 12, 13, 14, 15,
            16, 0xbb, 0xbb, 0xbb, 0xbb,
        ];
        let mut target = [0xee; 24];
        {
            let framebuffer = GpuDisplayFramebuffer::new(
                VolatileSlice::new(&mut target),
                12,
                4,
                DRM_FORMAT_XRGB8888,
            );
            framebuffer.copy_from_frame(&ScanoutFrame {
                bytes: &source,
                stride: 12,
                width: 2,
                height: 2,
                fourcc: DRM_FORMAT_ABGR8888,
                damage: Damage::Full,
            });
        }
        assert_eq!(
            target,
            [
                3, 2, 1, 4, 7, 6, 5, 8, 0xee, 0xee, 0xee, 0xee, 11, 10, 9, 12, 15, 14,
                13, 16, 0xee, 0xee, 0xee, 0xee,
            ]
        );
    }
}
