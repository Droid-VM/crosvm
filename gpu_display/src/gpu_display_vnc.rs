// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

use std::collections::VecDeque;
use std::os::raw::c_char;
use std::os::raw::c_int;
use std::sync::Arc;
use std::sync::Mutex;

use base::AsRawDescriptor;
use base::Event;
use base::RawDescriptor;
use base::VolatileSlice;
use linux_input_sys::virtio_input_event;
use vm_control::gpu::DisplayParameters;

use crate::DisplayT;
use crate::EventDeviceKind;
use crate::GpuDisplayError;
use crate::GpuDisplayEvents;
use crate::GpuDisplayFramebuffer;
use crate::GpuDisplayResult;
use crate::GpuDisplaySurface;
use crate::SurfaceType;
use crate::SysDisplayT;

const VNC_INPUT_NONE: c_int = 0;
const VNC_INPUT_KEY: u8 = 1;
const VNC_INPUT_POINTER: u8 = 2;

#[repr(C)]
#[derive(Default, Clone)]
struct VncInputEvent {
    event_type: u8,
    down: u8,
    linux_keycode: u16,
    x: i32,
    y: i32,
    button_mask: u8,
}

extern "C" {
    fn vnc_server_create(
        width: c_int,
        height: c_int,
        port: c_int,
        password: *const c_char,
    ) -> *mut std::ffi::c_void;
    fn vnc_server_start(server: *mut std::ffi::c_void);
    fn vnc_server_has_input_events(server: *mut std::ffi::c_void) -> c_int;
    fn vnc_server_resize(
        server: *mut std::ffi::c_void,
        width: c_int,
        height: c_int,
    ) -> c_int;
    fn vnc_server_update_framebuffer(
        server: *mut std::ffi::c_void,
        data: *const u8,
        size: u32,
    );
    fn vnc_server_destroy(server: *mut std::ffi::c_void);
    fn vnc_server_set_input_event_fd(server: *mut std::ffi::c_void, fd: c_int);
    fn vnc_server_poll_input_event(
        server: *mut std::ffi::c_void,
        out: *mut VncInputEvent,
    ) -> c_int;
    fn vnc_server_set_cursor(
        server: *mut std::ffi::c_void,
        argb: *const u8,
        width: c_int,
        height: c_int,
        hot_x: c_int,
        hot_y: c_int,
    );
    fn vnc_server_set_cursor_pos(server: *mut std::ffi::c_void, x: c_int, y: c_int);
    #[allow(clippy::too_many_arguments)]
        server: *mut std::ffi::c_void,
        clean: *const u8,
        clean_size: u32,
        cursor_argb: *const u8,
        cw: c_int,
        ch: c_int,
        cx: c_int,
        cy: c_int,
        visible: c_int,
        full: c_int,
    );
}

struct VncServerHandle {
    ptr: *mut std::ffi::c_void,
}

unsafe impl Send for VncServerHandle {}
unsafe impl Sync for VncServerHandle {}

impl Drop for VncServerHandle {
    fn drop(&mut self) {
        if !self.ptr.is_null() {
            unsafe { vnc_server_destroy(self.ptr) };
        }
    }
}

struct SharedFramebuffer {
    width: u32,
    height: u32,
    /// The guest scanout with NO cursor composited into it. Kept pristine: it is the source the
    /// bridge restores from when the pointer moves off a pixel, which is why this design needs no
    /// save-under-cursor buffer at all.
    data: Vec<u8>,
    server: Arc<VncServerHandle>,
    cursor: CursorState,
}

/// The guest's hardware cursor, as last reported by virtio-gpu.
#[derive(Default)]
struct CursorState {
    pixels: Vec<u8>,
    width: u32,
    height: u32,
    /// Top-left corner of the cursor image, as the guest reported it. Signed: it goes negative
    /// when the pointer is within the hotspot of the left or top edge. No hotspot is kept beside
    /// it: the bridge draws the image at this corner, and the hotspot is the guest's business.
    x: i32,
    y: i32,
    visible: bool,
}

impl SharedFramebuffer {
        let c = &self.cursor;
        let has_img = !c.pixels.is_empty() && c.width > 0 && c.height > 0;
        unsafe {
                self.server.ptr,
                if has_img { c.pixels.as_ptr() } else { std::ptr::null() },
                c.width as c_int,
                c.height as c_int,
                c.x as c_int,
                c.y as c_int,
                (c.visible && has_img) as c_int,
                full as c_int,
            )
        }
    }
}

}

struct VncSurface {
    width: u32,
    #[allow(dead_code)]
    height: u32,
    shared_fb: Arc<Mutex<SharedFramebuffer>>,
    local_buffer: Vec<u8>,
}

impl VncSurface {
    fn new(width: u32, height: u32, shared_fb: Arc<Mutex<SharedFramebuffer>>) -> Self {
        let buf_size = (width as usize) * (height as usize) * 4;
        VncSurface {
            width,
            height,
            shared_fb,
            local_buffer: vec![0u8; buf_size],
        }
    }
    /// What this replaces is not one copy but a chain of them. On the CPU route the producer's
    /// actual layout is converted into this VNC sink's BGRX framebuffer at the copy boundary, then
    /// `flip` copies it into `data`. Here the GPU reads the guest pages directly, the same channel
    /// exchange rides along inside the blit
}

impl GpuDisplaySurface for VncSurface {
    fn framebuffer(&mut self) -> Option<GpuDisplayFramebuffer> {
        let stride = self.width * 4;
        let buf_len = self.local_buffer.len();
        let expected = (self.width as usize) * (self.height as usize) * 4;
        if buf_len != expected {
            base::error!(
                "VNC: framebuffer size mismatch! buf={} expected={} ({}x{})",
                buf_len, expected, self.width, self.height
            );
        }
        Some(GpuDisplayFramebuffer::new(
            VolatileSlice::new(self.local_buffer.as_mut_slice()),
            stride,
            4,
            crate::DRM_FORMAT_XRGB8888,
        ))
    }

    fn flip(&mut self) {
        if let Ok(mut fb) = self.shared_fb.lock() {
            let copy_len = fb.data.len().min(self.local_buffer.len());
            if fb.data.len() != self.local_buffer.len() {
                base::error!(
                    "VNC: flip size mismatch! shared_fb={} local={} copy_len={}",
                    fb.data.len(), self.local_buffer.len(), copy_len
                );
            }
            fb.data[..copy_len].copy_from_slice(&self.local_buffer[..copy_len]);
            // fb.data stays cursor-free; the bridge blends the pointer on its way out.
        }
    }

}

/// The guest's hardware cursor, published to VNC clients two ways at once.
///
/// It is composited into the outgoing frame by our own bridge, AND handed to LibVNCServer as an
/// RFB cursor. The composited one is the one that has to be right: the DroidVM app drives the
/// pointer over its own channel, so a VNC client can be a passive viewer whose idea of where the
/// pointer is has nothing to do with the guest's. The RFB cursor is what lets a client that
/// speaks the Cursor pseudo-encoding move the pointer for free, and it doubles as an independent
/// rendering of the same data -- differencing a frame grabbed with the encoding against one
/// grabbed without it is how the hotspot bug in the composited path was caught.
///
/// that to the two rectangles the pointer left and entered rather than a whole frame.
struct VncCursorSurface {
    width: u32,
    height: u32,
    hot_x: u32,
    hot_y: u32,
    server: Arc<VncServerHandle>,
    shared_fb: Arc<Mutex<SharedFramebuffer>>,
    pixels: Vec<u8>,
}

impl GpuDisplaySurface for VncCursorSurface {
    fn framebuffer(&mut self) -> Option<GpuDisplayFramebuffer> {
        Some(GpuDisplayFramebuffer::new(
            VolatileSlice::new(self.pixels.as_mut_slice()),
            self.width * 4,
            4,
            crate::DRM_FORMAT_ARGB8888,
        ))
    }

    fn flip(&mut self) {
        // Publish to our own compositor: this is the one that works when the DroidVM app drives
        // the pointer in RELATIVE mode, where a client drawing at its own pointer position would
        // put the cursor somewhere unrelated to where the guest thinks it is.
        if let Ok(mut fb) = self.shared_fb.lock() {
            fb.cursor.pixels.clear();
            fb.cursor.pixels.extend_from_slice(&self.pixels);
            fb.cursor.width = self.width;
            fb.cursor.height = self.height;
            fb.cursor.visible = true;
        }
        // And as an RFB cursor, which is what a client with the Cursor pseudo-encoding draws
        // itself. Unlike the composited copy this one carries the hotspot, because LibVNCServer
        // positions by the pointer and subtracts it.
        // SAFETY: `pixels` is width*height*4 bytes and outlives the call.
        unsafe {
            vnc_server_set_cursor(
                self.server.ptr,
                self.pixels.as_ptr(),
                self.width as c_int,
                self.height as c_int,
                self.hot_x as c_int,
                self.hot_y as c_int,
            )
        }
    }

    fn set_cursor_hotspot(&mut self, hot_x: u32, hot_y: u32) {
        self.hot_x = hot_x;
        self.hot_y = hot_y;
    }

    /// Tell LibVNCServer where the pointer is.
    ///
    /// Not redundant with the pointer events it already sees: the DroidVM app drives input over
    /// its own channel, so a VNC client can be a passive VIEWER that never sends a PointerEvent.
    /// Its cursorX/cursorY would then stay at the origin and the composited pointer would sit in
    /// the top-left corner no matter where the guest actually put it.
    fn set_position(&mut self, x: i32, y: i32) {
        if let Ok(mut fb) = self.shared_fb.lock() {
            fb.cursor.x = x;
            fb.cursor.y = y;
            // Partial: only the rectangles the pointer left and entered. Without this the pointer
            // would only move when the guest happened to send a frame.
        // LibVNCServer wants the POINTER, not the image: it draws the cursor at
        // cursorX - hot_x. (x,y) is the image origin, so the hotspot goes back on here.
        // SAFETY: server handle is valid for this surface's lifetime.
        unsafe {
            vnc_server_set_cursor_pos(
                self.server.ptr,
                x + self.hot_x as c_int,
                y + self.hot_y as c_int,
            )
            unsafe {
                vnc_server_update_framebuffer(
                    fb.server.ptr,
                    fb.data.as_ptr(),
                    copy_len as u32,
                );
            }
        }
    }

    fn set_cursor_visible(&mut self, visible: bool) {
        if let Ok(mut fb) = self.shared_fb.lock() {
            fb.cursor.visible = visible;
        if !visible {
            // SAFETY: null pixels is the bridge's hide request.
            unsafe { vnc_server_set_cursor(self.server.ptr, std::ptr::null(), 0, 0, 0, 0) }
        }
    }

}

pub struct DisplayVnc {
    event: Event,
    width: u32,
    height: u32,
    server: Arc<VncServerHandle>,
    shared_fb: Option<Arc<Mutex<SharedFramebuffer>>>,
    input_queue: VecDeque<VncInputEvent>,
    next_tracking_id: i32,
    prev_button_mask: u8,
const VNC_ABS_MAX: i32 = 0x7FFF;

/// Scale a VNC framebuffer coordinate in `0..extent` (where `extent` is the live framebuffer
/// width/height, updated on guest resize) to `0..=VNC_ABS_MAX`.
fn vnc_norm_abs(v: i32, extent: u32) -> i32 {
    let extent = extent.max(1) as i64;
    (((v.max(0) as i64) * (VNC_ABS_MAX as i64)) / extent).clamp(0, VNC_ABS_MAX as i64) as i32
}

impl DisplayVnc {
    pub fn new_tcp(
        addr: &str,
        width: u32,
        height: u32,
        password: Option<String>,
    ) -> GpuDisplayResult<DisplayVnc> {
        let event = Event::new().map_err(|_| GpuDisplayError::CreateEvent)?;

        let port = addr
            .rsplit(':')
            .next()
            .and_then(|p| p.parse::<c_int>().ok())
            .unwrap_or(5900);

        let c_password;
        let password_ptr = match &password {
            Some(pwd) => {
                c_password = std::ffi::CString::new(pwd.as_str())
                    .map_err(|_| GpuDisplayError::Allocate)?;
                c_password.as_ptr()
            }
            None => std::ptr::null(),
        };

        let server_ptr = unsafe {
            vnc_server_create(width as c_int, height as c_int, port, password_ptr)
        };
        if server_ptr.is_null() {
            base::error!("VNC server failed to start on port {}", port);
            return Err(GpuDisplayError::Allocate);
        }

        unsafe {
            vnc_server_set_input_event_fd(server_ptr, event.as_raw_descriptor());
        }

        unsafe { vnc_server_start(server_ptr) };
        base::info!("VNC server started on TCP port {}", port);

        let server = Arc::new(VncServerHandle { ptr: server_ptr });

        Ok(DisplayVnc {
            event,
            width,
            height,
            server,
            shared_fb: None,
            input_queue: VecDeque::new(),
            next_tracking_id: 0,
            prev_button_mask: 0,
        })
    }

    fn drain_c_events(&mut self) {
        loop {
            let mut ev = VncInputEvent::default();
            let t = unsafe { vnc_server_poll_input_event(self.server.ptr, &mut ev) };
            if t == VNC_INPUT_NONE as c_int {
                break;
            }
            self.input_queue.push_back(ev);
        }
        let _ = self.event.wait_timeout(std::time::Duration::ZERO);
    }

    /// Mouse mode (qemu usb-tablet equivalent): absolute position on every event (hover
    /// works), button transitions from the RFB mask, wheel as REL_WHEEL.
    /// RFB button mask: bit0=left, bit1=middle, bit2=right, bit3/4=wheel up/down.
        let cur_mask = ev.button_mask;
        let prev_mask = self.prev_button_mask;
        self.prev_button_mask = cur_mask;
        let changed = cur_mask ^ prev_mask;

        let mut events = vec![
            virtio_input_event::absolute_x(vnc_norm_abs(ev.x, self.width)),
            virtio_input_event::absolute_y(vnc_norm_abs(ev.y, self.height)),
        ];
        if changed & 0x01 != 0 {
            events.push(virtio_input_event::left_click(cur_mask & 0x01 != 0));
        }
        if changed & 0x02 != 0 {
            events.push(virtio_input_event::middle_click(cur_mask & 0x02 != 0));
        }
        if changed & 0x04 != 0 {
            events.push(virtio_input_event::right_click(cur_mask & 0x04 != 0));
        }
        if changed & 0x08 != 0 && cur_mask & 0x08 != 0 {
            events.push(virtio_input_event::wheel(1));
        }
        if changed & 0x10 != 0 && cur_mask & 0x10 != 0 {
            events.push(virtio_input_event::wheel(-1));
        }
    }

    fn convert_next_event(&mut self) -> Option<GpuDisplayEvents> {
        let ev = self.input_queue.pop_front()?;

        match ev.event_type {
            VNC_INPUT_KEY => {
                let pressed = ev.down != 0;
                let events = vec![virtio_input_event::key(
                    ev.linux_keycode,
                    pressed,
                    false,
                )];
                Some(GpuDisplayEvents {
                    events,
                    device_type: EventDeviceKind::Keyboard,
                })
            }
            VNC_INPUT_POINTER => {
                let cur_mask = ev.button_mask;
                let prev_mask = self.prev_button_mask;
                self.prev_button_mask = cur_mask;

                let btn1_now = cur_mask & 1;
                let btn1_prev = prev_mask & 1;

                if btn1_now != 0 && btn1_prev == 0 {
                    let tid = self.next_touch_tracking_id();
                    let events = vec![
                        virtio_input_event::multitouch_slot(0),
                        virtio_input_event::multitouch_tracking_id(tid),
                        virtio_input_event::multitouch_absolute_x(ev.x),
                        virtio_input_event::multitouch_absolute_y(ev.y),
                        virtio_input_event::touch(true),
                    ];
                    Some(GpuDisplayEvents {
                        events,
                        device_type: EventDeviceKind::Touchscreen,
                    })
                } else if btn1_now != 0 && btn1_prev != 0 {
                    let tid = self.current_tracking_id();
                    let events = vec![
                        virtio_input_event::multitouch_slot(0),
                        virtio_input_event::multitouch_tracking_id(tid),
                        virtio_input_event::multitouch_absolute_x(ev.x),
                        virtio_input_event::multitouch_absolute_y(ev.y),
                        virtio_input_event::touch(true),
                    ];
                    Some(GpuDisplayEvents {
                        events,
                        device_type: EventDeviceKind::Touchscreen,
                    })
                } else if btn1_now == 0 && btn1_prev != 0 {
                    let events = vec![
                        virtio_input_event::multitouch_slot(0),
                        virtio_input_event::multitouch_tracking_id(-1),
                        virtio_input_event::touch(false),
                    ];
                    Some(GpuDisplayEvents {
                        events,
                        device_type: EventDeviceKind::Touchscreen,
                    })
                } else {
                    None
                }
            }
            _ => None,
        }
    }
}

impl DisplayT for DisplayVnc {
    fn pending_events(&self) -> bool {
        !self.input_queue.is_empty()
            || unsafe { vnc_server_has_input_events(self.server.ptr) != 0 }
    }

    fn next_event(&mut self) -> GpuDisplayResult<u64> {
        self.drain_c_events();
        Ok(0)
    }

    fn handle_next_event(
        &mut self,
        _surface: &mut Box<dyn GpuDisplaySurface>,
    ) -> Option<GpuDisplayEvents> {
        self.convert_next_event()
    }

    fn handle_next_event_without_surface(&mut self) -> Option<GpuDisplayEvents> {
        self.convert_next_event()
    }

    fn create_surface(
        &mut self,
        parent_surface_id: Option<u32>,
        _surface_id: u32,
        _scanout_id: Option<u32>,
        display_params: &DisplayParameters,
        surf_type: SurfaceType,
    ) -> GpuDisplayResult<Box<dyn GpuDisplaySurface>> {
        // A parented surface is virtio-gpu's cursor (see VirtioGpu::update_cursor). It must be
        // handled BEFORE the sizing below: the cursor scanout carries no display_params of its
        // own, so crosvm passes DisplayParameters::default() -- taking the size from there would
        // resize the whole VNC screen to the default resolution the moment a pointer appeared.
        if parent_surface_id.is_some() {
            if !matches!(surf_type, SurfaceType::Cursor) {
                return Err(GpuDisplayError::Unsupported);
            }
            // virtio-gpu's cursor plane is fixed at 64x64 and the guest's cursor resource is
            // allocated to match.
            let (width, height) = (64u32, 64u32);
            base::info!("VNC: created cursor surface {}x{}", width, height);
            let shared_fb = self
                .shared_fb
                .clone()
                .ok_or(GpuDisplayError::Unsupported)?;
            return Ok(Box::new(VncCursorSurface {
                width,
                height,
                // Replaced by the first set_cursor_hotspot, which virtio-gpu sends with the image.
                hot_x: 0,
                hot_y: 0,
                server: self.server.clone(),
                shared_fb,
                pixels: vec![0u8; (width as usize) * (height as usize) * 4],
            }));
        }

        let (req_w, req_h) = display_params.get_virtual_display_size();
        let width = if req_w != 0 { req_w } else { self.width };
        let height = if req_h != 0 { req_h } else { self.height };

        if width != self.width || height != self.height {
            base::info!(
                "VNC: resizing from {}x{} to {}x{}",
                self.width, self.height, width, height
            );
            let ret = unsafe {
                vnc_server_resize(
                    self.server.ptr,
                    width as c_int,
                    height as c_int,
                )
            };
            if ret != 0 {
                base::error!("VNC: failed to resize server");
                return Err(GpuDisplayError::Allocate);
            }
            self.width = width;
            self.height = height;
        }

        let buf_size = (width as usize) * (height as usize) * 4;
        let shared_fb = Arc::new(Mutex::new(SharedFramebuffer {
            width,
            height,
            data: vec![0u8; buf_size],
            server: self.server.clone(),
            cursor: CursorState::default(),
        }));

        self.shared_fb = Some(shared_fb.clone());

        base::info!("VNC: created surface {}x{}", width, height);
        Ok(Box::new(VncSurface::new(width, height, shared_fb)))
    }
}

impl SysDisplayT for DisplayVnc {}

impl AsRawDescriptor for DisplayVnc {
    fn as_raw_descriptor(&self) -> RawDescriptor {
        self.event.as_raw_descriptor()
    }
}
