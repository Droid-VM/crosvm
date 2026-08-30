// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

use std::cell::RefCell;
use std::collections::BTreeMap;
use std::collections::VecDeque;
use std::os::raw::c_char;
use std::os::raw::c_int;
use std::rc::Rc;
use std::sync::Arc;
use std::sync::Mutex;

use base::AsRawDescriptor;
use base::Event;
use base::RawDescriptor;
use base::VolatileSlice;
use linux_input_sys::virtio_input_event;
use vm_control::gpu::DisplayParameters;

use crate::vnc_blit::BlitMapping;
use crate::vnc_blit::VncBlitContext;
use crate::vnc_h264::H264Consumer;
use crate::DisplayT;
use crate::EventDevice;
use crate::GpuDisplayError;
use crate::GpuDisplayEvents;
use crate::GpuDisplayFramebuffer;
use crate::GpuDisplayResult;
use crate::GpuDisplaySurface;
use crate::SemaphoreTimepoint;
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
    fn vnc_server_has_clients(server: *mut std::ffi::c_void) -> c_int;
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
    fn vnc_server_offer_frame(
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
        gpu_blit_ctx: *mut std::ffi::c_void,
        gpu_import_id: i64,
    );
}

struct VncServerHandle {
    ptr: *mut std::ffi::c_void,
}

unsafe impl Send for VncServerHandle {}
unsafe impl Sync for VncServerHandle {}

impl VncServerHandle {
    /// Whether any RFB client is connected. Asked once per frame: it is a NULL check on
    /// LibVNCServer's client list, and it decides whether the frame's copies are worth making.
    fn has_clients(&self) -> bool {
        if self.ptr.is_null() {
            return false;
        }
        // SAFETY: ptr is a live server handle owned by this VncServerHandle.
        unsafe { vnc_server_has_clients(self.ptr) != 0 }
    }
}

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
    ///
    /// This is where the frame lives on the CPU transport, and also on the GPU transport whenever
    /// the blit target's rows turn out to be padded (see `VncSurface::flip_to`). `gpu_frame` says
    /// which.
    data: Vec<u8>,
    /// Set while the clean frame lives in the blit target rather than in `data`: the target is
    /// mapped for CPU reading and stays mapped until the next blit, so the pointer is good for
    /// cursor-only offers as well as the frame that produced it.
    gpu_frame: Option<GpuFrame>,
    server: Arc<VncServerHandle>,
    cursor: CursorState,
}

/// The last frame the GPU blitted, borrowed from the blit context that owns the mapping.
struct GpuFrame {
    /// Held so the mapping cannot outlive what it points into.
    ctx: Arc<VncBlitContext>,
    mapping: BlitMapping,
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
    /// Where the current clean frame is, whichever transport put it there.
    ///
    /// `None` means there is no frame to offer. Only the GPU transport can produce that: a mapping
    /// is invalidated by the next blit, and a surface that has been replaced but not yet released
    /// can still be holding one. Falling back to `data` there would be worse than doing nothing --
    /// on the GPU transport `data` was never written, so a cursor-only offer would repaint the
    /// rectangle the pointer left with black and report nothing.
    fn clean(&self) -> Option<(*const u8, u32)> {
        match &self.gpu_frame {
            Some(gpu) if gpu.ctx.mapping_is_current(&gpu.mapping) => {
                Some((gpu.mapping.pixels, gpu.mapping.size))
            }
            Some(_) => None,
            None => Some((self.data.as_ptr(), self.data.len() as u32)),
        }
    }

    /// Offer the frame to the bridge's consumers. `full` means the guest produced a new frame;
    /// otherwise not one guest pixel changed and only the pointer moved, which is what lets a
    /// cursor travel over a static desktop without costing a frame.
    ///
    /// `gpu` is the same picture while it is still a GPU object -- the blit context and the import
    /// the frame was blitted FROM -- for a consumer that would rather blit it again than read it.
    /// `None` on the CPU transport and on every cursor-only offer, where the import that produced
    /// the last frame may already have been released and `pixels` is the honest answer anyway.
    ///
    /// What each consumer makes of the offer is the bridge's business (vnc_frame_consumer.h).
    fn offer_frame(&mut self, full: bool, gpu: Option<(*mut std::ffi::c_void, i64)>) {
        let Some((pixels, size)) = self.clean() else {
            return;
        };
        let c = &self.cursor;
        let has_img = !c.pixels.is_empty() && c.width > 0 && c.height > 0;
        let (gpu_ctx, gpu_import) = gpu.unwrap_or((std::ptr::null_mut(), 0));
        // SAFETY: both buffers outlive the call; the bridge only reads them. `pixels` is either
        // `data` or a mapping `clean()` has just confirmed is the current one, and this thread is
        // the only one that can blit and so invalidate it. `gpu_ctx`/`gpu_import` are the caller's
        // live import, which is not released while this call is on its stack.
        unsafe {
            vnc_server_offer_frame(
                self.server.ptr,
                pixels,
                size,
                if has_img { c.pixels.as_ptr() } else { std::ptr::null() },
                c.width as c_int,
                c.height as c_int,
                c.x as c_int,
                c.y as c_int,
                (c.visible && has_img) as c_int,
                full as c_int,
                gpu_ctx,
                gpu_import,
            )
        }
    }
}

/// A source the sink has imported: the context that holds it, the native handle, and the geometry
/// it was declared with.
///
/// The context travels with the import rather than with the surface, and that is not tidiness. A
/// surface is created before any producer asks whether this sink can import anything -- the
/// simplefb bridge builds its transport against a surface it already has, and virtio-gpu imports on
/// its first flush -- so a surface handed a context at construction would always be handed `None`.
/// Attaching it here says the same thing more accurately anyway: an import cannot exist without the
/// context that made it.
///
/// The geometry is kept because it is what the blit is sized by: the Vulkan bridge allocates its
/// target to the SOURCE image's dimensions and refuses a target of any other size, so the number
/// has to travel from the import to the flip.
#[derive(Clone)]
struct VncImport {
    ctx: Arc<VncBlitContext>,
    handle: i64,
    /// The same dmabuf, imported a second time in the byte order a video encoder reads.
    ///
    /// 0 when there is no H.264 encoder on this display, which is the usual case. It is a
    /// separate import rather than a flag on the blit because the channel exchange is performed by
    /// the SOURCE image's declared format (`blitSourceFourcc`, C++ side), and that is fixed when
    /// the image is created -- so a frame that has to reach two consumers in two byte orders needs
    /// two source images. Neither of them copies anything: both are views of the guest's pages.
    encoder_handle: i64,
    width: u32,
    height: u32,
}

struct VncSurface {
    width: u32,
    height: u32,
    shared_fb: Arc<Mutex<SharedFramebuffer>>,
    local_buffer: Vec<u8>,
    /// Shared with the `DisplayVnc` that owns the import id space, exactly as the Android backend
    /// shares its own: imports are made through the display and used through the surface. Empty on
    /// a display with no GPU half -- including every display capped to `transport-cap=cpu`, whose
    /// import attempts are refused in `GpuDisplay` before they reach this backend at all.
    imports: Rc<RefCell<BTreeMap<u32, VncImport>>>,
}

impl VncSurface {
    fn new(
        width: u32,
        height: u32,
        shared_fb: Arc<Mutex<SharedFramebuffer>>,
        imports: Rc<RefCell<BTreeMap<u32, VncImport>>>,
    ) -> Self {
        let buf_size = (width as usize) * (height as usize) * 4;
        VncSurface {
            width,
            height,
            shared_fb,
            local_buffer: vec![0u8; buf_size],
            imports,
        }
    }

    /// The GPU transport: blit the guest's dmabuf into a CPU-readable buffer and offer THAT to the
    /// bridge, instead of a frame the producer copied for us.
    ///
    /// What this replaces is not one copy but a chain of them. On the CPU route the producer's
    /// actual layout is converted into this VNC sink's BGRX framebuffer at the copy boundary, then
    /// `flip` copies it into `data`. Here the GPU reads the guest pages directly, the same channel
    /// exchange rides along inside the blit
    /// (`blitSourceFourcc`, C++ side), and what the CPU touches afterwards is ordinary cached host
    /// memory instead of a write-combining guest mapping.
    ///
    /// It deliberately does NOT ask whether a client is connected. The frame arrives on the guest's
    /// own flush, so one skipped here is never offered again -- the same rule that keeps `flip` from
    /// short-circuiting. It also means the work a flush costs the guest is identical whether or not
    /// anybody is watching, which is what §7 wanted verified.
    fn blit_and_offer(&mut self, import_id: u32) -> anyhow::Result<()> {
        let import = self
            .imports
            .borrow()
            .get(&import_id)
            .cloned()
            .ok_or_else(|| anyhow::anyhow!("invalid VNC display import id {}", import_id))?;

        // The blit target is sized to the source, and the offer is read as a screen-sized picture
        // packed to the screen's width, so a source of some other size cannot be offered at all --
        // it would be published as a sheared frame with nothing to say so. Refusing sends this
        // resource to the CPU path, which clips properly.
        if import.width != self.width || import.height != self.height {
            return Err(anyhow::anyhow!(
                "import is {}x{} but this VNC screen is {}x{}",
                import.width,
                import.height,
                self.width,
                self.height
            ));
        }

        let mut fb = self
            .shared_fb
            .lock()
            .map_err(|_| anyhow::anyhow!("VNC shared framebuffer lock poisoned"))?;
        // Before the blit, not after: the blit unmaps the target, so anything still pointing into
        // it is stale from that moment.
        fb.gpu_frame = None;

        if !import.ctx.blit(import.handle, import.width, import.height) {
            return Err(anyhow::anyhow!("Vulkan blit into the readback target failed"));
        }
        let mapping = import
            .ctx
            .map()
            .ok_or_else(|| anyhow::anyhow!("failed to map the readback target for CPU read"))?;
        // Closes the loop between what was asked for and what gralloc handed back. It follows from
        // the check above -- the target is allocated to the import's geometry -- so this is not
        // expected to fire; it is here because everything downstream indexes by `self.width` and a
        // disagreement would be published as a picture rather than reported.
        if mapping.width != self.width || mapping.height != self.height {
            return Err(anyhow::anyhow!(
                "readback target came back {}x{} for a {}x{} screen",
                mapping.width,
                mapping.height,
                self.width,
                self.height
            ));
        }

        let packed_stride = (self.width as usize) * 4;
        if mapping.stride_bytes as usize == packed_stride {
            // The bridge's offer is a pointer plus a size and its bands are offsets into it, all
            // computed from `width * 4`: it has no stride. When gralloc gives back exactly that, the
            // mapping IS the offer and nothing is copied on the way -- which is also what keeps step
            // 12's property intact, because ingest is handed the same shape of thing it has always
            // been handed and cannot tell the two transports apart.
            fb.gpu_frame = Some(GpuFrame {
                ctx: import.ctx.clone(),
                mapping,
            });
        } else {
            // Padded rows. The alternative -- give the offer a stride field -- was rejected: every
            // producer in the tree is packed, so it would add a case to the one function whose
            // byte-for-byte behaviour step 12 froze, in exchange for a copy that only a padded
            // gralloc pays. Repack into `data`, which exists and is exactly the right size, and
            // leave `gpu_frame` unset so cursor-only offers read the repacked frame too.
            let rows = (mapping.height as usize).min(self.height as usize);
            // SAFETY: the mapping is current (nothing has blitted since `map`) and describes
            // `size` readable bytes at `pixels`.
            let src = unsafe { std::slice::from_raw_parts(mapping.pixels, mapping.size as usize) };
            for y in 0..rows {
                let src_off = y * mapping.stride_bytes as usize;
                let dst_off = y * packed_stride;
                if src_off + packed_stride > src.len() || dst_off + packed_stride > fb.data.len() {
                    break;
                }
                fb.data[dst_off..dst_off + packed_stride]
                    .copy_from_slice(&src[src_off..src_off + packed_stride]);
            }
        }

        // The offer carries the GPU source as well as the pixels, so a consumer that wants the
        // picture in some other form -- the H.264 encoder wants it in a MediaCodec input buffer --
        // can blit it a second time from the same import instead of reading back what this one
        // just produced. Handing over `encoder_handle` rather than `handle` is what makes that
        // second blit land in R,G,B,A; see the field's own note.
        fb.offer_frame(
            true,
            Some((import.ctx.as_native_ptr(), import.encoder_handle)),
        );
        Ok(())
    }
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
            // There is deliberately no "skip this frame if nobody is connected" here, even though
            // the two full-frame copies below are provably wasted when there is no client.
            //
            // This flip is driven by the guest's virtio-gpu flush, not by a clock, so a frame
            // dropped here is not offered again. If the guest then goes idle there is no next
            // flush, and a client connecting afterwards has nothing to arrive to -- it is served
            // whatever the server framebuffer held when the last consumer left. Measured, not
            // feared: a client detached across a guest resolution change came back to a
            // permanently black screen at 0 bytes/s, while the same binary with a client held
            // across the same change was correct.
            //
            // Skipping is safe under a producer that returns on its own, which is why it lives in
            // simplefb_display_loop instead (GpuDisplay::has_consumer): that one is a 30 fps timer,
            // so a client arriving is noticed on the next tick and the frame is rebuilt within
            // 33 ms. Reinstating it here needs a way to re-present on consumer arrival -- the
            // frame is already retained in the bridge's last_clean, so a LibVNCServer
            // newClientHook restoring from it would serve -- and that belongs with the transport
            // work, not with this fix.
            let copy_len = fb.data.len().min(self.local_buffer.len());
            if fb.data.len() != self.local_buffer.len() {
                base::error!(
                    "VNC: flip size mismatch! shared_fb={} local={} copy_len={}",
                    fb.data.len(), self.local_buffer.len(), copy_len
                );
            }
            fb.data[..copy_len].copy_from_slice(&self.local_buffer[..copy_len]);
            // The CPU transport owns the frame again -- drop any mapping a previous GPU flip left,
            // so `clean()` reads what was just written rather than the last blit.
            fb.gpu_frame = None;
            // fb.data stays cursor-free; the bridge blends the pointer on its way out.
            fb.offer_frame(true, None);
        }
    }

    fn flip_to(
        &mut self,
        import_id: u32,
        _acquire_timepoint: Option<SemaphoreTimepoint>,
        _release_timepoint: Option<SemaphoreTimepoint>,
        _extra_info: Option<crate::FlipToExtraInfo>,
    ) -> anyhow::Result<sync::Waitable> {
        self.blit_and_offer(import_id)?;
        Ok(sync::Waitable::signaled())
    }

    /// Always `None`, and that is the acceptance condition of this whole step rather than an
    /// omission (plan §7).
    ///
    /// A `Some` here is a release fence, and the caller's contract for one is specific: virtio-gpu
    /// defers the RESOURCE_FLUSH virtio fence until it signals, so the guest's compositor does not
    /// get its buffer back -- does not complete its page flip, does not see a vblank -- until the
    /// sink says it is finished reading. That is right for the native sink, whose reader is
    /// SurfaceFlinger. It would be catastrophic here, because it would put a NETWORK service in the
    /// guest's vblank loop: a slow RFB client, or one on a congested link, would pace the guest's
    /// rendering, and a client that stopped reading would stop the guest.
    ///
    /// Nothing has to be deferred, either. By the time `flip_to` returns, the blit is complete (its
    /// fence was waited on), the pixels have been read out of the target, and the offer has been
    /// made -- the guest's dmabuf is not referenced by anything any more. The flip really is
    /// finished when the producer is told it is, so `None` is not a promise being dodged, it is the
    /// truth about a transport that reads its source synchronously.
    fn take_flip_completion_fence(&mut self) -> Option<base::SafeDescriptor> {
        None
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
/// The cost of compositing is a framebuffer update per pointer move; `offer_frame(false)` keeps
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
            fb.offer_frame(false, None);
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
            fb.offer_frame(false, None);
        }
        // LibVNCServer wants the POINTER, not the image: it draws the cursor at
        // cursorX - hot_x. (x,y) is the image origin, so the hotspot goes back on here.
        // SAFETY: server handle is valid for this surface's lifetime.
        unsafe {
            vnc_server_set_cursor_pos(
                self.server.ptr,
                x + self.hot_x as c_int,
                y + self.hot_y as c_int,
            )
        }
    }

    fn set_cursor_visible(&mut self, visible: bool) {
        if let Ok(mut fb) = self.shared_fb.lock() {
            fb.cursor.visible = visible;
            fb.offer_frame(false, None);
        }
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
    prev_button_mask: u8,
    /// The absolute pointer THIS server's clients drive, and no other server's.
    ///
    /// Held here rather than reached through `GpuDisplay`'s event-device map because that map is
    /// scoped to one display OWNER and fans out by device KIND, and neither is the scope these
    /// devices have. Two VNC servers on one owner both matched the one Tablet in it, each
    /// normalizing against its own framebuffer, so the guest received two screens' coordinates on
    /// one device with nothing to tell them apart; and the simplefb bridge is its own owner whose
    /// map is empty whenever a GPU device exists, so there every event of every kind was iterated
    /// over an empty list and dropped. A device that belongs to a binding lives in that binding.
    ///
    /// Owned outright, not shared: everything that writes to it is this sink's own event drain,
    /// which runs on one thread, so report interleaving is not a hazard that has to be excluded --
    /// it cannot arise. `None` on a `view-only=true` binding.
    tablet: Option<EventDevice>,
    /// The keyboard THIS server's clients type into. Same scope, same ownership and the same
    /// `None`-when-view-only as `tablet`.
    ///
    /// The guest ends up with one of these per non-view-only VNC screen, alongside the VM-global
    /// keyboard the `--input keyboard` socket still backs. That is not an accident of the wiring;
    /// it is the resource model. A keyboard could be routed by guest focus instead of by screen,
    /// but making it per-screen is what removes the last thing this sink has to share with anything
    /// -- no writer crosses a thread, so no lock, no interleaving, no shared failure.
    keyboard: Option<EventDevice>,
    /// The GPU half, once something has asked for it. `None` before the probe and after a probe
    /// that came back empty -- `blit_probed` tells those two apart, because "there is no blit
    /// driver on this machine" must be answered once and not re-attempted per resource.
    blit: Option<Arc<VncBlitContext>>,
    blit_probed: bool,
    /// Imports made against `blit`, keyed by the id `GpuDisplay` handed out. Shared with the
    /// surfaces, which are where flips happen.
    imports: Rc<RefCell<BTreeMap<u32, VncImport>>>,
    /// The hardware-encode rung: a second consumer on the frame bus, feeding the RFB broadcaster.
    ///
    /// Declared last so it is dropped last. Field order is drop order, and this one has to outlive
    /// `server`: destroying the server is what stops offers arriving, and an offer arriving after
    /// this was freed would be a callback into nothing.
    h264: Option<Arc<H264Consumer>>,
}

/// The VNC tablet advertises this fixed absolute-axis maximum. Every injected coordinate is scaled
/// to it against the *current* framebuffer size, so the guest cursor stays 1:1 with the pointer at
/// any resolution -- including after the guest auto-resizes the display -- without pinning the axis
/// range to a static config value. MUST equal the ABS_X/ABS_Y max the tablet was created with:
/// that device omits width/height and so advertises `NORMALIZED_ABS_MAX` (src/crosvm/config.rs),
/// which is this same number. Two constants that are required to be equal, in two crates, is not
/// tidy -- it is what the `--input absolute-mouse` feeder already relies on, and this sink is now
/// one more feeder of the same shape of device.
const VNC_ABS_MAX: i32 = 0x7FFF;

/// Scale a VNC framebuffer coordinate in `0..extent` (where `extent` is the live framebuffer
/// width/height, updated on guest resize) to `0..=VNC_ABS_MAX`.
fn vnc_norm_abs(v: i32, extent: u32) -> i32 {
    let extent = extent.max(1) as i64;
    (((v.max(0) as i64) * (VNC_ABS_MAX as i64)) / extent).clamp(0, VNC_ABS_MAX as i64) as i32
}

impl DisplayVnc {
    /// `hw_encode` says whether this binding may run the hardware H.264 encoder and serve the
    /// stream to RFB clients that ask for encoding 50. It is the transport ceiling's answer
    /// (`transport-cap=gpu-hw` or `auto`), resolved by the caller rather than read here, because
    /// the ceiling belongs to the binding and one sink serves several of them.
    ///
    /// There is no port to go with it. The stream leaves by the RFB port this server is already
    /// listening on, which is the whole of plans/H264_SINGLE_PORT.md: nothing extra is bound, so
    /// nothing extra can collide, be firewalled, or be told to a client.
    ///
    /// `tablet` and `keyboard` are this binding's own input devices, handed in rather than made
    /// here because the guest-facing halves of them have to be registered as virtio devices by the
    /// code that owns the VM's device list. Both `None` means `view-only=true`: no devices were
    /// built and RFB input is dropped on arrival.
    pub fn new_tcp(
        addr: &str,
        width: u32,
        height: u32,
        password: Option<String>,
        hw_encode: bool,
        tablet: Option<EventDevice>,
        keyboard: Option<EventDevice>,
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

        // Between create and start, because that is the window the bus's registration contract
        // names: the consumer list is read without a lock from the producer's thread, so it has to
        // be finished before any frame can be offered.
        let h264 = if hw_encode {
            H264Consumer::start(server_ptr)
        } else {
            None
        };

        unsafe { vnc_server_start(server_ptr) };
        base::info!("VNC server started on TCP port {}", port);

        let server = Arc::new(VncServerHandle { ptr: server_ptr });

        base::info!(
            "VNC port {}: input -> {}",
            port,
            match (tablet.is_some(), keyboard.is_some()) {
                (true, true) => "this binding's own tablet + keyboard",
                (true, false) => "this binding's own tablet (no keyboard)",
                (false, true) => "this binding's own keyboard (no tablet)",
                (false, false) => "dropped (view-only)",
            },
        );

        Ok(DisplayVnc {
            event,
            width,
            height,
            server,
            shared_fb: None,
            input_queue: VecDeque::new(),
            prev_button_mask: 0,
            tablet,
            keyboard,
            blit: None,
            blit_probed: false,
            imports: Rc::new(RefCell::new(BTreeMap::new())),
            h264,
        })
    }

    /// Brings up the GPU half once, on the first producer that asks whether it exists.
    ///
    /// Once, because the answer cannot change: it is "was a Vulkan blit driver named for this
    /// process, and did it come up". Lazily rather than in `new_tcp`, because a display capped to
    /// `transport-cap=cpu` must never load a driver at all -- `GpuDisplay::is_dmabuf_import_supported`
    /// answers the cap without asking the backend, so a capped display never reaches this and the
    /// measured behaviour (a capped run does not even dlopen turnip) is preserved.
    fn blit_context(&mut self) -> Option<&Arc<VncBlitContext>> {
        if !self.blit_probed {
            self.blit_probed = true;
            self.blit = VncBlitContext::open(self.width, self.height);
            match &self.blit {
                Some(_) => base::info!(
                    "VNC: GPU transport available ({}x{} readback target)",
                    self.width,
                    self.height
                ),
                None => base::info!("VNC: no GPU transport; frames will be copied by the CPU"),
            }
        }
        self.blit.as_ref()
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
    fn pointer_to_mouse_events(&mut self, ev: &VncInputEvent) -> Vec<virtio_input_event> {
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
        events
    }

    /// Takes one RFB event off the queue and writes it into this binding's own device.
    ///
    /// Direct, not through `GpuDisplayEvents` and the owner's fan-out. That route delivers to the
    /// event devices of the `GpuDisplay` this backend happens to be inside, matched by device kind
    /// -- which is wrong here in both directions at once. The simplefb bridge is its own
    /// `GpuDisplay` and its event-device list is empty whenever a GPU device exists, so every event
    /// of every kind, keys included, was iterated over an empty list and dropped: the device was
    /// there, the road was not. And where the list was not empty, matching by kind is matching by
    /// kind and nothing else, so two VNC servers' pointers landed on whichever tablet the owner
    /// held. Both are the same mistake -- delivery scoped to a display owner when it needed to be
    /// scoped to a binding -- so the devices written to here are the binding's own.
    ///
    /// A view-only binding still comes through here and still pops. Dropping the event is the
    /// point; leaving it queued would grow the queue for as long as somebody kept clicking.
    fn inject_next_event(&mut self) {
        let Some(ev) = self.input_queue.pop_front() else {
            return;
        };

        match ev.event_type {
            VNC_INPUT_KEY => {
                let Some(keyboard) = &mut self.keyboard else {
                    return;
                };
                let events = [virtio_input_event::key(ev.linux_keycode, ev.down != 0, false)];
                // One `send_report` call per RFB event, so the SYN_REPORT it appends closes exactly
                // the events that arrived together -- which is what a guest reads as one keystroke.
                if let Err(e) = keyboard.send_report(events.into_iter()) {
                    base::error!("VNC: keyboard event dropped: {}", e);
                }
            }
            VNC_INPUT_POINTER => {
                let events = self.pointer_to_mouse_events(&ev);
                // After the conversion, not before: `pointer_to_mouse_events` is what advances
                // `prev_button_mask`, and a view-only binding still has to track the mask it would
                // have reported. Otherwise a button held across the moment input came back would
                // produce a release for a press the guest never saw.
                let Some(tablet) = &mut self.tablet else {
                    return;
                };
                if let Err(e) = tablet.send_report(events.into_iter()) {
                    base::error!("VNC: pointer event dropped: {}", e);
                }
            }
            _ => {}
        }
    }
}

impl DisplayT for DisplayVnc {
    /// Whether this sink has a GPU half, which is whether a Vulkan blit context came up.
    ///
    /// The answer used to be a flat `false`, which was honest while there was no `import_resource`
    /// here at all. It is a probe now, and it is still the same kind of statement: a `true` costs
    /// the caller a real export and import attempt, so it must not be optimistic. The trait default
    /// -- `true` for every backend -- is exactly the failure this replaced, and the reason a probe
    /// answering from a trait default is worse than no probe.
    fn is_dmabuf_import_supported(&mut self) -> bool {
        self.blit_context().is_some()
    }

    /// Whether anything is waiting for frames -- any RFB client, or one on the H.264 stream.
    ///
    /// No RFB client used to be the whole answer: LibVNCServer's mark-as-modified walks an empty
    /// client list, so a frame pushed then is encoded for nobody and sent to nobody. Producers ask
    /// this before building a frame; the surface's own flip and the C bridge both check again, so
    /// a producer that ignores the answer is still correct, only wasteful.
    ///
    /// The H.264 consumer is part of the answer for a reason that has survived the side channel it
    /// was written for: an h264 client's pixel path is suppressed, so LibVNCServer marks nothing
    /// for it, and a screen watched over the stream alone would deadlock on itself -- no offers
    /// because nothing is watching, and nothing counted as watching because the encoder is what is
    /// watching.
    fn has_consumer(&self) -> bool {
        self.server.has_clients()
            || self
                .h264
                .as_ref()
                .map(|h264| h264.wants_frames())
                .unwrap_or(false)
    }

    /// Both kinds of client folded into one number, so a producer can see either of them arrive.
    ///
    /// The pixel half is the bool it always was. The stream half is a counter, because a client
    /// joining there is exactly the case the bool cannot report: it goes true while the RFB flag is
    /// already true, and the producer would then re-supply nothing and leave the new stream showing
    /// a screen that had stopped moving before it joined.
    fn consumer_generation(&self) -> u64 {
        let rfb = self.server.has_clients() as u64;
        let h264 = self
            .h264
            .as_ref()
            .map(|h264| h264.connect_generation())
            .unwrap_or(0);
        rfb | (h264 << 1)
    }

    fn pending_events(&self) -> bool {
        !self.input_queue.is_empty()
            || unsafe { vnc_server_has_input_events(self.server.ptr) != 0 }
    }

    fn next_event(&mut self) -> GpuDisplayResult<u64> {
        self.drain_c_events();
        Ok(0)
    }

    /// Always `None`: this backend has already delivered the event by the time it returns.
    ///
    /// The two hooks stay because they are what the owner's drain loop calls per queued event --
    /// `pending_events` says there is one, this consumes it -- and the loop must keep draining. It
    /// is only the DELIVERY that moved (see `inject_next_event`), so handing back `None` here is
    /// what stops the owner-scoped fan-out from also running, rather than an event going missing.
    fn handle_next_event(
        &mut self,
        _surface: &mut Box<dyn GpuDisplaySurface>,
    ) -> Option<GpuDisplayEvents> {
        self.inject_next_event();
        None
    }

    fn handle_next_event_without_surface(&mut self) -> Option<GpuDisplayEvents> {
        self.inject_next_event();
        None
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
            gpu_frame: None,
            server: self.server.clone(),
            cursor: CursorState::default(),
        }));

        self.shared_fb = Some(shared_fb.clone());

        base::info!("VNC: created surface {}x{}", width, height);
        Ok(Box::new(VncSurface::new(
            width,
            height,
            shared_fb,
            self.imports.clone(),
        )))
    }

    /// Imports a guest dmabuf as a blit source.
    ///
    /// The `fourcc` handed on is the guest's own declaration and stays that way across the FFI.
    /// The correction that makes the blit land in the byte order LibVNCServer serves lives beside
    /// the target it has to agree with, in the C++ (`blitSourceFourcc`) -- keeping it there means
    /// the rule is stated once, next to the AHardwareBuffer format that is half of it, rather than
    /// as a swizzle that two files each have to remember.
    fn import_resource(
        &mut self,
        import_id: u32,
        _surface_id: u32,
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
            return Err(anyhow::anyhow!("the VNC sink only imports DMA-BUFs"));
        };

        let wants_encoder_source = self.h264.is_some();
        let ctx = self
            .blit_context()
            .ok_or_else(|| anyhow::anyhow!("this VNC display has no GPU half"))?
            .clone();
        let handle = ctx
            .import_dmabuf(
                descriptor.as_raw_descriptor(),
                offset,
                stride,
                modifiers,
                linear_layout_verified,
                width,
                height,
                fourcc,
                /* exchange_red_blue= */ true,
            )
            .ok_or_else(|| anyhow::anyhow!("Turnip DMA-BUF import failed"))?;
        // The second view of the same pages, in the byte order the encoder reads. Only when there
        // is an encoder to read it: an import is a VkImage and an imported allocation, so making
        // one nothing will ever blit from is a cost with no reader. A failure here is not fatal --
        // the encoder falls to uploading the pixels the first import produces -- so it is
        // reported and the frame path carries on.
        let encoder_handle = if wants_encoder_source {
            match ctx.import_dmabuf(
                descriptor.as_raw_descriptor(),
                offset,
                stride,
                modifiers,
                linear_layout_verified,
                width,
                height,
                fourcc,
                /* exchange_red_blue= */ false,
            ) {
                Some(handle) => handle,
                None => {
                    base::error!(
                        "VNC h264: the encoder's view of resource {} could not be imported; \
                         its frames will be uploaded by the CPU",
                        import_id
                    );
                    0
                }
            }
        } else {
            0
        };
        self.imports.borrow_mut().insert(
            import_id,
            VncImport {
                ctx,
                handle,
                encoder_handle,
                width,
                height,
            },
        );
        Ok(())
    }

    fn release_import(&mut self, import_id: u32, _surface_id: u32) {
        let Some(import) = self.imports.borrow_mut().remove(&import_id) else {
            return;
        };
        import.ctx.release_import(import.handle);
        if import.encoder_handle != 0 {
            import.ctx.release_import(import.encoder_handle);
        }
    }
}

impl Drop for DisplayVnc {
    fn drop(&mut self) {
        // Before the imports are released, and before the server is destroyed: it tells the drain
        // thread to stop, so nothing is left that could ask for a blit from an import that is
        // about to go.
        if let Some(h264) = &self.h264 {
            h264.shutdown();
        }
        // The bridge frees whatever is left when it is destroyed, so this is not a leak fix -- it
        // is that a release is the display's to do while the display still exists, the same shape
        // the Android backend has. Surfaces are already gone by here (GpuDisplay drops them before
        // the backend), so nothing can be mid-flip.
        for import in std::mem::take(&mut *self.imports.borrow_mut()).into_values() {
            import.ctx.release_import(import.handle);
            if import.encoder_handle != 0 {
                import.ctx.release_import(import.encoder_handle);
            }
        }
    }
}

impl SysDisplayT for DisplayVnc {}

impl AsRawDescriptor for DisplayVnc {
    fn as_raw_descriptor(&self) -> RawDescriptor {
        self.event.as_raw_descriptor()
    }
}
