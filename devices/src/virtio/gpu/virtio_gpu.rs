// Copyright 2020 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use std::cell::RefCell;
use std::collections::BTreeMap as Map;
use std::collections::BTreeSet as Set;
use std::io::IoSliceMut;
use std::num::NonZeroU32;
use std::path::PathBuf;
use std::rc::Rc;
use std::result::Result;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::sync::Arc;

use anyhow::Context;
use base::error;
use base::info;
use base::FromRawDescriptor;
use base::IntoRawDescriptor;
use base::Protection;
use base::SafeDescriptor;
use base::VolatileSlice;
use gpu_display::*;
use hypervisor::MemCacheType;
use libc::c_void;
use rutabaga_gfx::ResourceCreate3D;
use rutabaga_gfx::ResourceCreateBlob;
use rutabaga_gfx::Rutabaga;
use rutabaga_gfx::RutabagaDescriptor;
#[cfg(windows)]
use rutabaga_gfx::RutabagaError;
use rutabaga_gfx::RutabagaFence;
use rutabaga_gfx::RutabagaFromRawDescriptor;
use rutabaga_gfx::RutabagaHandle;
use rutabaga_gfx::RutabagaIntoRawDescriptor;
use rutabaga_gfx::RutabagaIovec;
use rutabaga_gfx::Transfer3D;
use rutabaga_gfx::RUTABAGA_HANDLE_TYPE_MEM_DMABUF;
use rutabaga_gfx::RUTABAGA_HANDLE_TYPE_MEM_OPAQUE_FD;
use rutabaga_gfx::RUTABAGA_MAP_ACCESS_MASK;
use rutabaga_gfx::RUTABAGA_MAP_ACCESS_READ;
use rutabaga_gfx::RUTABAGA_MAP_ACCESS_RW;
use rutabaga_gfx::RUTABAGA_MAP_ACCESS_WRITE;
use rutabaga_gfx::RUTABAGA_MAP_CACHE_CACHED;
use rutabaga_gfx::RUTABAGA_MAP_CACHE_MASK;
use serde::Deserialize;
use serde::Serialize;
use sync::Mutex;
use sync::Waitable;
use vm_control::gpu::DisplayMode;
use vm_control::gpu::DisplayParameters;
use vm_control::gpu::GpuControlCommand;
use vm_control::gpu::GpuControlResult;
use vm_control::gpu::MouseMode;
use vm_control::VmMemorySource;
use vm_memory::udmabuf::UdmabufDriver;
use vm_memory::udmabuf::UdmabufDriverTrait;
use vm_memory::GuestAddress;
use vm_memory::GuestMemory;

use super::protocol::virtio_gpu_rect;
use super::protocol::GpuResponse;
use super::protocol::GpuResponse::*;
use super::protocol::GpuResponsePlaneInfo;
use super::protocol::VirtioGpuResult;
use super::protocol::VIRTIO_GPU_BLOB_FLAG_CREATE_GUEST_HANDLE;
use super::protocol::VIRTIO_GPU_BLOB_MEM_HOST3D;
use super::VirtioScanoutBlobData;
use crate::virtio::gpu::edid::DisplayInfo;
use crate::virtio::gpu::edid::EdidBytes;
use crate::virtio::gpu::snapshot::pack_directory_to_snapshot;
use crate::virtio::gpu::snapshot::unpack_snapshot_to_directory;
use crate::virtio::gpu::snapshot::DirectorySnapshot;
use crate::virtio::gpu::GpuDisplayParameters;
use crate::virtio::gpu::VIRTIO_GPU_MAX_SCANOUTS;
use crate::virtio::resource_bridge::BufferInfo;
use crate::virtio::resource_bridge::PlaneInfo;
use crate::virtio::resource_bridge::ResourceInfo;
use crate::virtio::resource_bridge::ResourceResponse;
use crate::virtio::SharedMemoryMapper;

const KGSL_ARENA_SENTINEL: u64 = 0xffff_ffff_ffff_f000;
const DRM_FORMAT_MOD_LINEAR: u64 = 0;
const DRM_FORMAT_MOD_INVALID: u64 = 0x00ff_ffff_ffff_ffff;

const fn drm_fourcc(a: u8, b: u8, c: u8, d: u8) -> u32 {
    (a as u32) | ((b as u32) << 8) | ((c as u32) << 16) | ((d as u32) << 24)
}

const DRM_FORMAT_XRGB8888: u32 = drm_fourcc(b'X', b'R', b'2', b'4');
const DRM_FORMAT_ARGB8888: u32 = drm_fourcc(b'A', b'R', b'2', b'4');
const DRM_FORMAT_XBGR8888: u32 = drm_fourcc(b'X', b'B', b'2', b'4');
const DRM_FORMAT_ABGR8888: u32 = drm_fourcc(b'A', b'B', b'2', b'4');

fn is_single_plane_8888_rgb(format: u32) -> bool {
    matches!(
        format,
        DRM_FORMAT_XRGB8888 | DRM_FORMAT_ARGB8888 | DRM_FORMAT_XBGR8888 | DRM_FORMAT_ABGR8888
    )
}

#[derive(Copy, Clone)]
struct KgslHostVisibleArena {
    host_va: u64,
    size: u64,
}

fn parse_arena_env_u64(name: &str) -> Option<u64> {
    let value = std::env::var(name).ok()?;
    if let Some(hex) = value.strip_prefix("0x") {
        u64::from_str_radix(hex, 16).ok()
    } else {
        value.parse().ok()
    }
}

fn kgsl_host_visible_arena() -> Option<KgslHostVisibleArena> {
    let host_va = parse_arena_env_u64("CROSVM_KGSL_ARENA_HOST_VA")?;
    let size = parse_arena_env_u64("CROSVM_KGSL_ARENA_SIZE")?;
    (host_va != 0 && size != 0).then_some(KgslHostVisibleArena { host_va, size })
}

pub fn to_rutabaga_descriptor(s: SafeDescriptor) -> RutabagaDescriptor {
    // SAFETY:
    // Safe because we own the SafeDescriptor at this point.
    unsafe { RutabagaDescriptor::from_raw_descriptor(s.into_raw_descriptor()) }
}

fn to_safe_descriptor(r: RutabagaDescriptor) -> SafeDescriptor {
    // SAFETY:
    // Safe because we own the SafeDescriptor at this point.
    unsafe { SafeDescriptor::from_raw_descriptor(r.into_raw_descriptor()) }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum DisplayImportState {
    Unknown,
    Imported { import_id: u32, surface_id: u32 },
    CpuFallback,
}

struct VirtioGpuResource {
    resource_id: u32,
    width: u32,
    height: u32,
    size: u64,
    shmem_offset: Option<u64>,
    scanout_data: Option<VirtioScanoutBlobData>,
    display_import_state: DisplayImportState,
    rutabaga_external_mapping: bool,
    arena_mapped: bool,

    // Only saved for snapshotting, so that we can re-attach backing iovecs with the correct new
    // host addresses.
    backing_iovecs: Option<Vec<(GuestAddress, usize)>>,
}

#[derive(Serialize, Deserialize)]
struct VirtioGpuResourceSnapshot {
    resource_id: u32,
    width: u32,
    height: u32,
    size: u64,

    backing_iovecs: Option<Vec<(GuestAddress, usize)>>,
    shmem_offset: Option<u64>,
}

impl VirtioGpuResource {
    /// Creates a new VirtioGpuResource with the given metadata.  Width and height are used by the
    /// display, while size is useful for hypervisor mapping.
    pub fn new(resource_id: u32, width: u32, height: u32, size: u64) -> VirtioGpuResource {
        VirtioGpuResource {
            resource_id,
            width,
            height,
            size,
            shmem_offset: None,
            scanout_data: None,
            display_import_state: DisplayImportState::Unknown,
            rutabaga_external_mapping: false,
            arena_mapped: false,
            backing_iovecs: None,
        }
    }

    fn snapshot(&self) -> VirtioGpuResourceSnapshot {
        // Only the 2D backend is fully supported and it doesn't use these fields. 3D is WIP.
        assert!(self.scanout_data.is_none());
        assert!(!matches!(
            self.display_import_state,
            DisplayImportState::Imported { .. }
        ));

        VirtioGpuResourceSnapshot {
            resource_id: self.resource_id,
            width: self.width,
            height: self.height,
            size: self.size,
            backing_iovecs: self.backing_iovecs.clone(),
            shmem_offset: self.shmem_offset,
        }
    }

    fn restore(s: VirtioGpuResourceSnapshot) -> Self {
        let mut resource = VirtioGpuResource::new(s.resource_id, s.width, s.height, s.size);
        resource.backing_iovecs = s.backing_iovecs;
        resource
    }

    fn transition_display_import(&mut self, display: &mut GpuDisplay, next: DisplayImportState) {
        if let DisplayImportState::Imported {
            import_id,
            surface_id,
        } = std::mem::replace(&mut self.display_import_state, next)
        {
            display.release_import(import_id, surface_id);
        }
    }
}

struct VirtioGpuScanout {
    width: u32,
    height: u32,
    scanout_type: SurfaceType,
    // If this scanout is a primary scanout, the scanout id.
    scanout_id: Option<u32>,
    // If this scanout is a primary scanout, the display properties.
    display_params: Option<GpuDisplayParameters>,
    // If this scanout is a cursor scanout, the scanout that this is cursor is overlayed onto.
    parent_surface_id: Option<u32>,

    surface_id: Option<u32>,
    parent_scanout_id: Option<u32>,

    resource_id: Option<NonZeroU32>,
    position: Option<(u32, u32)>,
    // Reused packed staging buffer for flushes into padded-stride window buffers.
    flush_staging: Vec<u8>,
}

#[derive(Serialize, Deserialize)]
struct VirtioGpuScanoutSnapshot {
    width: u32,
    height: u32,
    scanout_type: SurfaceType,
    scanout_id: Option<u32>,
    display_params: Option<GpuDisplayParameters>,

    // The surface IDs aren't guest visible. Instead of storing them and then having to fix up
    // `gpu_display` internals, we'll allocate new ones on restore. So, we just need to store
    // whether a surface was allocated and the parent's scanout ID.
    has_surface: bool,
    parent_scanout_id: Option<u32>,

    resource_id: Option<NonZeroU32>,
    position: Option<(u32, u32)>,
}

impl VirtioGpuScanout {
    fn new_primary(scanout_id: u32, params: GpuDisplayParameters) -> VirtioGpuScanout {
        let (width, height) = params.get_virtual_display_size();
        VirtioGpuScanout {
            width,
            height,
            scanout_type: SurfaceType::Scanout,
            scanout_id: Some(scanout_id),
            display_params: Some(params),
            parent_surface_id: None,
            surface_id: None,
            parent_scanout_id: None,
            resource_id: None,
            position: None,
            flush_staging: Vec::new(),
        }
    }

    fn new_cursor() -> VirtioGpuScanout {
        // Per virtio spec: "The mouse cursor image is a normal resource, except that it must be
        // 64x64 in size."
        VirtioGpuScanout {
            width: 64,
            height: 64,
            scanout_type: SurfaceType::Cursor,
            scanout_id: None,
            display_params: None,
            parent_surface_id: None,
            surface_id: None,
            parent_scanout_id: None,
            resource_id: None,
            position: None,
            flush_staging: Vec::new(),
        }
    }

    fn snapshot(&self) -> VirtioGpuScanoutSnapshot {
        VirtioGpuScanoutSnapshot {
            width: self.width,
            height: self.height,
            has_surface: self.surface_id.is_some(),
            resource_id: self.resource_id,
            scanout_type: self.scanout_type,
            scanout_id: self.scanout_id,
            display_params: self.display_params.clone(),
            parent_scanout_id: self.parent_scanout_id,
            position: self.position,
        }
    }

    fn restore(
        &mut self,
        snapshot: VirtioGpuScanoutSnapshot,
        parent_surface_id: Option<u32>,
        display: &Rc<RefCell<GpuDisplay>>,
    ) -> VirtioGpuResult {
        // Scanouts are mainly controlled by the host, we just need to make sure it looks same,
        // restore the resource_id association, and create a surface in the display.

        assert_eq!(self.width, snapshot.width);
        assert_eq!(self.height, snapshot.height);
        assert_eq!(self.scanout_type, snapshot.scanout_type);
        assert_eq!(self.scanout_id, snapshot.scanout_id);
        assert_eq!(self.display_params, snapshot.display_params);

        self.resource_id = snapshot.resource_id;
        if snapshot.has_surface {
            self.create_surface(display, parent_surface_id, None)?;
        } else {
            self.release_surface(display);
        }
        if let Some((x, y)) = snapshot.position {
            self.set_position(display, x, y)?;
        }

        Ok(OkNoData)
    }

    fn create_surface(
        &mut self,
        display: &Rc<RefCell<GpuDisplay>>,
        new_parent_surface_id: Option<u32>,
        new_scanout_rect: Option<virtio_gpu_rect>,
    ) -> VirtioGpuResult {
        let mut need_to_create = false;

        if self.surface_id.is_none() {
            need_to_create = true;
        }

        if self.parent_surface_id != new_parent_surface_id {
            self.parent_surface_id = new_parent_surface_id;
            need_to_create = true;
        }

        if let Some(new_scanout_rect) = new_scanout_rect {
            // The guest may request a new scanout size when modesetting happens (i.e. display
            // resolution change). Detect when that happens and re-allocate a surface with the new
            // size.
            //
            // Note that we do NOT update |self.display_params|, which is sourced from user input
            // (initial display parameters), and (as of the time of writing) only matters to EDID
            // information. EDID info shall remain the same for a given display even if the active
            // resolution has changed.
            let new_width = new_scanout_rect.width.to_native();
            let new_height = new_scanout_rect.height.to_native();
            if !(self.width == new_width && self.height == new_height) {
                self.width = new_width;
                self.height = new_height;
                need_to_create = true;
            }
        }

        if !need_to_create {
            return Ok(OkNoData);
        }

        self.release_surface(display);

        let mut display = display.borrow_mut();

        let display_params = match self.display_params.clone() {
            Some(mut params) => {
                // The sizes in |self.display_params| doesn't necessarily match the requested
                // surface size (see above note about when guest modesetting happens). Always
                // override display mode to match the requested size.
                params.mode = DisplayMode::Windowed(self.width, self.height);
                params
            }
            None => {
                DisplayParameters::default_with_mode(DisplayMode::Windowed(self.width, self.height))
            }
        };
        let surface_id = display.create_surface(
            self.parent_surface_id,
            self.scanout_id,
            &display_params,
            self.scanout_type,
        )?;

        self.surface_id = Some(surface_id);

        Ok(OkNoData)
    }

    fn release_surface(&mut self, display: &Rc<RefCell<GpuDisplay>>) {
        if let Some(surface_id) = self.surface_id {
            display.borrow_mut().release_surface(surface_id);
        }

        self.surface_id = None;
    }

    fn set_mouse_mode(
        &mut self,
        display: &Rc<RefCell<GpuDisplay>>,
        mouse_mode: MouseMode,
    ) -> VirtioGpuResult {
        if let Some(surface_id) = self.surface_id {
            display
                .borrow_mut()
                .set_mouse_mode(surface_id, mouse_mode)?;
        }
        Ok(OkNoData)
    }

    fn set_position(
        &mut self,
        display: &Rc<RefCell<GpuDisplay>>,
        x: u32,
        y: u32,
    ) -> VirtioGpuResult {
        if let Some(surface_id) = self.surface_id {
            display.borrow_mut().set_position(surface_id, x, y)?;
            self.position = Some((x, y));
        }
        Ok(OkNoData)
    }

    fn commit(&self, display: &Rc<RefCell<GpuDisplay>>) -> VirtioGpuResult {
        if let Some(surface_id) = self.surface_id {
            display.borrow_mut().commit(surface_id)?;
        }
        Ok(OkNoData)
    }

    fn flush(
        &mut self,
        display: &Rc<RefCell<GpuDisplay>>,
        resource: &mut VirtioGpuResource,
        rutabaga: &mut Rutabaga,
    ) -> Result<Option<Waitable>, GpuResponse> {
        let surface_id = match self.surface_id {
            Some(id) => id,
            _ => return Ok(None),
        };

        // Virtio cursors are ordinary 64x64 guest-backed resources, not arena-backed blobs with
        // verifiable linear DMA-BUF provenance. Keep their small, infrequent image updates on the
        // established CPU path instead of treating the expected import rejection as a failure.
        if matches!(self.scanout_type, SurfaceType::Scanout) {
            if let Some(import_id) = VirtioGpuScanout::import_resource_to_display(
                display, surface_id, resource, rutabaga,
            ) {
                let flip_result = display
                    .borrow_mut()
                    .flip_to(surface_id, import_id, None, None, None);
                match flip_result {
                    Ok(completion) => return Ok(Some(completion)),
                    Err(e) => {
                        error!(
                            "flip_to failed; switching resource to CPU fallback: {:#}",
                            e
                        );
                        resource.transition_display_import(
                            &mut display.borrow_mut(),
                            DisplayImportState::CpuFallback,
                        );
                    }
                }
            }
        }

        // No imported display resource is available, so update the surface with a copy.
        let mut display = display.borrow_mut();

        // Prevent overwriting a buffer that is currently being used by the compositor.
        if display.next_buffer_in_use(surface_id) {
            return Ok(None);
        }

        let fb = display
            .framebuffer_region(surface_id, 0, 0, self.width, self.height)
            .ok_or(ErrUnspec)?;

        let packed_stride = self.width as usize * 4;
        let fb_stride = fb.stride() as usize;
        if fb_stride == packed_stride {
            let mut transfer = Transfer3D::new_2d(0, 0, self.width, self.height, 0);
            transfer.stride = fb.stride();
            let fb_slice = fb.as_volatile_slice();
            let buf = IoSliceMut::new(
                // SAFETY: trivially safe
                unsafe { std::slice::from_raw_parts_mut(fb_slice.as_mut_ptr(), fb_slice.size()) },
            );
            rutabaga.transfer_read(0, resource.resource_id, transfer, Some(buf))?;
        } else {
            // The window buffer rows are padded (gralloc stride alignment), and the readback
            // backend writes tightly-packed rows no matter what Transfer3D::stride says (observed
            // with gfxstream at widths whose row size isn't aligned, e.g. 1440x900) -- every row
            // lands progressively shifted and the image smears. Read into a packed staging buffer
            // and re-stride into the window buffer by row.
            let mut transfer = Transfer3D::new_2d(0, 0, self.width, self.height, 0);
            transfer.stride = packed_stride as u32;
            let size = packed_stride * self.height as usize;
            if self.flush_staging.len() < size {
                self.flush_staging.resize(size, 0);
            }
            let staging = &mut self.flush_staging[..size];
            rutabaga.transfer_read(
                0,
                resource.resource_id,
                transfer,
                Some(IoSliceMut::new(staging)),
            )?;
            let fb_slice = fb.as_volatile_slice();
            for row in 0..self.height as usize {
                fb_slice
                    .sub_slice(row * fb_stride, packed_stride)
                    .map_err(|_| ErrUnspec)?
                    .copy_from(&staging[row * packed_stride..][..packed_stride]);
            }
        }

        display.flip(surface_id);
        Ok(None)
    }

    fn import_resource_to_display(
        display: &Rc<RefCell<GpuDisplay>>,
        surface_id: u32,
        resource: &mut VirtioGpuResource,
        rutabaga: &mut Rutabaga,
    ) -> Option<u32> {
        match resource.display_import_state {
            DisplayImportState::Imported {
                import_id,
                surface_id: import_surface_id,
            } if import_surface_id == surface_id => {
                return Some(import_id);
            }
            DisplayImportState::Imported { .. } => {
                resource.transition_display_import(
                    &mut display.borrow_mut(),
                    DisplayImportState::Unknown,
                );
            }
            DisplayImportState::CpuFallback => return None,
            DisplayImportState::Unknown => {}
        }

        if !display.borrow_mut().is_dmabuf_import_supported() {
            resource.display_import_state = DisplayImportState::CpuFallback;
            return None;
        }

        let imported =
            Self::try_import_resource_to_display(display, surface_id, resource, rutabaga);
        if imported.is_none() {
            base::warn!(
                "display resource {} import failed; caching CPU fallback",
                resource.resource_id
            );
            resource.display_import_state = DisplayImportState::CpuFallback;
        }
        imported
    }

    fn try_import_resource_to_display(
        display: &Rc<RefCell<GpuDisplay>>,
        surface_id: u32,
        resource: &mut VirtioGpuResource,
        rutabaga: &mut Rutabaga,
    ) -> Option<u32> {
        // Virgl's normal export for arena-backed resources is a shared-memory
        // fd. The display-only export creates a UDMABUF window over the exact
        // resource range while leaving the guest export contract untouched.
        let display_export = rutabaga.export_display_blob(resource.resource_id);
        let display_exported = display_export.is_ok();
        let exported = display_export
            .or_else(|_| rutabaga.export_blob(resource.resource_id))
            .ok()?;
        let handle_type = exported.handle_type;
        if handle_type != RUTABAGA_HANDLE_TYPE_MEM_DMABUF {
            base::warn!(
                "display resource {} exported unsupported handle type 0x{:x}",
                resource.resource_id,
                handle_type
            );
            return None;
        }
        let dmabuf = to_safe_descriptor(exported.os_handle);
        let query = match rutabaga.query(resource.resource_id) {
            Ok(query) => query,
            Err(e) => {
                base::warn!(
                    "display resource {} query failed: {:#}",
                    resource.resource_id,
                    e
                );
                return None;
            }
        };

        let (width, height, format, stride, source_offset) = match resource.scanout_data {
            Some(data) => (
                data.width,
                data.height,
                data.drm_format.into(),
                data.strides[0],
                data.offsets[0],
            ),
            None => (
                resource.width,
                resource.height,
                query.drm_fourcc,
                query.strides[0],
                query.offsets[0],
            ),
        };
        // The display-only UDMABUF is a window over this arena resource, so
        // byte zero of the new fd is already the start of the resource.
        let offset = if display_exported && resource.arena_mapped {
            0
        } else {
            source_offset
        };
        let min_stride = width.checked_mul(4);
        let image_size = u64::from(stride).checked_mul(u64::from(height));
        let image_end = image_size.and_then(|size| u64::from(source_offset).checked_add(size));
        let linear_layout_verified = display_exported
            && resource.arena_mapped
            && resource.shmem_offset.is_some()
            && source_offset == 0
            && min_stride.is_some_and(|minimum| stride >= minimum)
            && image_end.is_some_and(|end| end <= resource.size)
            && is_single_plane_8888_rgb(format)
            && matches!(
                query.modifier,
                DRM_FORMAT_MOD_INVALID | DRM_FORMAT_MOD_LINEAR
            );

        let import_id = display
            .borrow_mut()
            .import_resource(
                surface_id,
                DisplayExternalResourceImport::Dmabuf {
                    descriptor: &dmabuf,
                    offset,
                    stride,
                    modifiers: query.modifier,
                    linear_layout_verified,
                    width,
                    height,
                    fourcc: format,
                },
            )
            .ok()?;
        resource.display_import_state = DisplayImportState::Imported {
            import_id,
            surface_id,
        };
        Some(import_id)
    }
}

/// Handles functionality related to displays, input events and hypervisor memory management.
pub struct VirtioGpu {
    display: Rc<RefCell<GpuDisplay>>,
    scanouts: Map<u32, VirtioGpuScanout>,
    scanouts_updated: Arc<AtomicBool>,
    cursor_scanout: VirtioGpuScanout,
    mapper: Arc<Mutex<Option<Box<dyn SharedMemoryMapper>>>>,
    rutabaga: Rutabaga,
    resources: Map<u32, VirtioGpuResource>,
    external_blob: bool,
    fixed_blob_mapping: bool,
    udmabuf_driver: Option<UdmabufDriver>,
    snapshot_scratch_directory: Option<PathBuf>,
    deferred_snapshot_load: Option<VirtioGpuSnapshot>,
}

// Only the 2D mode is supported. Notes on `VirtioGpu` fields:
//
//   * display: re-initialized from scratch using the scanout snapshots
//   * scanouts: snapshot'd
//   * scanouts_updated: snapshot'd
//   * cursor_scanout: snapshot'd
//   * mapper: not needed for 2d mode
//   * rutabaga: re-initialized from scatch using the resource snapshots
//   * resources: snapshot'd
//   * external_blob: not needed for 2d mode
//   * udmabuf_driver: not needed for 2d mode
#[derive(Serialize, Deserialize)]
pub struct VirtioGpuSnapshot {
    scanouts: Map<u32, VirtioGpuScanoutSnapshot>,
    scanouts_updated: bool,
    cursor_scanout: VirtioGpuScanoutSnapshot,
    rutabaga: DirectorySnapshot,
    resources: Map<u32, VirtioGpuResourceSnapshot>,
}

#[derive(Serialize, Deserialize)]
struct RutabagaResourceSnapshotSerializable {
    resource_id: u32,

    width: u32,
    height: u32,
    host_mem_size: usize,

    backing_iovecs: Option<Vec<(GuestAddress, usize)>>,
    component_mask: u8,
    size: u64,
}

fn sglist_to_rutabaga_iovecs(
    vecs: &[(GuestAddress, usize)],
    mem: &GuestMemory,
) -> Result<Vec<RutabagaIovec>, ()> {
    if vecs
        .iter()
        .any(|&(addr, len)| mem.get_slice_at_addr(addr, len).is_err())
    {
        return Err(());
    }

    let mut rutabaga_iovecs: Vec<RutabagaIovec> = Vec::new();
    for &(addr, len) in vecs {
        let slice = mem.get_slice_at_addr(addr, len).unwrap();
        rutabaga_iovecs.push(RutabagaIovec {
            base: slice.as_mut_ptr() as *mut c_void,
            len,
        });
    }
    Ok(rutabaga_iovecs)
}

pub enum ProcessDisplayResult {
    Success,
    CloseRequested,
    Error(GpuDisplayError),
}

impl VirtioGpu {
    /// Creates a new instance of the VirtioGpu state tracker.
    pub fn new(
        display: GpuDisplay,
        display_params: Vec<GpuDisplayParameters>,
        display_event: Arc<AtomicBool>,
        rutabaga: Rutabaga,
        mapper: Arc<Mutex<Option<Box<dyn SharedMemoryMapper>>>>,
        external_blob: bool,
        fixed_blob_mapping: bool,
        udmabuf: bool,
        snapshot_scratch_directory: Option<PathBuf>,
    ) -> Option<VirtioGpu> {
        let mut udmabuf_driver = None;
        if udmabuf {
            udmabuf_driver = Some(
                UdmabufDriver::new()
                    .map_err(|e| error!("failed to initialize udmabuf: {}", e))
                    .ok()?,
            );
        }

        let scanouts = display_params
            .iter()
            .enumerate()
            .map(|(display_index, display_param)| {
                (
                    display_index as u32,
                    VirtioGpuScanout::new_primary(display_index as u32, display_param.clone()),
                )
            })
            .collect::<Map<_, _>>();
        let cursor_scanout = VirtioGpuScanout::new_cursor();

        Some(VirtioGpu {
            display: Rc::new(RefCell::new(display)),
            scanouts,
            scanouts_updated: display_event,
            cursor_scanout,
            mapper,
            rutabaga,
            resources: Default::default(),
            external_blob,
            fixed_blob_mapping,
            udmabuf_driver,
            deferred_snapshot_load: None,
            snapshot_scratch_directory,
        })
    }

    /// Imports the event device
    pub fn import_event_device(&mut self, event_device: EventDevice) -> VirtioGpuResult {
        let mut display = self.display.borrow_mut();
        let _event_device_id = display.import_event_device(event_device)?;
        Ok(OkNoData)
    }

    /// Gets a reference to the display passed into `new`.
    pub fn display(&mut self) -> &Rc<RefCell<GpuDisplay>> {
        &self.display
    }

    /// Gets the list of supported display resolutions as a slice of `(width, height, enabled)`
    /// tuples.
    pub fn display_info(&self) -> Vec<(u32, u32, bool)> {
        (0..VIRTIO_GPU_MAX_SCANOUTS)
            .map(|scanout_id| scanout_id as u32)
            .map(|scanout_id| {
                self.scanouts
                    .get(&scanout_id)
                    .map_or((0, 0, false), |scanout| {
                        (scanout.width, scanout.height, true)
                    })
            })
            .collect::<Vec<_>>()
    }

    // Connects new displays to the device.
    fn add_displays(&mut self, displays: Vec<DisplayParameters>) -> GpuControlResult {
        let requested_num_scanouts = self.scanouts.len() + displays.len();
        if requested_num_scanouts > VIRTIO_GPU_MAX_SCANOUTS {
            return GpuControlResult::TooManyDisplays {
                allowed: VIRTIO_GPU_MAX_SCANOUTS,
                requested: requested_num_scanouts,
            };
        }

        let mut available_scanout_ids = (0..VIRTIO_GPU_MAX_SCANOUTS)
            .map(|s| s as u32)
            .collect::<Set<u32>>();

        self.scanouts.keys().for_each(|scanout_id| {
            available_scanout_ids.remove(scanout_id);
        });

        for display_params in displays.into_iter() {
            let new_scanout_id = *available_scanout_ids.iter().next().unwrap();
            available_scanout_ids.remove(&new_scanout_id);

            self.scanouts.insert(
                new_scanout_id,
                VirtioGpuScanout::new_primary(new_scanout_id, display_params),
            );
        }

        self.scanouts_updated.store(true, Ordering::Relaxed);

        GpuControlResult::DisplaysUpdated
    }

    /// Returns the list of displays currently connected to the device.
    fn list_displays(&self) -> GpuControlResult {
        GpuControlResult::DisplayList {
            displays: self
                .scanouts
                .iter()
                .filter_map(|(scanout_id, scanout)| {
                    scanout
                        .display_params
                        .as_ref()
                        .cloned()
                        .map(|display_params| (*scanout_id, display_params))
                })
                .collect(),
        }
    }

    /// Removes the specified displays from the device.
    fn remove_displays(&mut self, display_ids: Vec<u32>) -> GpuControlResult {
        for display_id in display_ids {
            if let Some(mut scanout) = self.scanouts.remove(&display_id) {
                scanout.release_surface(&self.display);
            } else {
                return GpuControlResult::NoSuchDisplay { display_id };
            }
        }

        self.scanouts_updated.store(true, Ordering::Relaxed);
        GpuControlResult::DisplaysUpdated
    }

    fn set_display_mouse_mode(
        &mut self,
        display_id: u32,
        mouse_mode: MouseMode,
    ) -> GpuControlResult {
        match self.scanouts.get_mut(&display_id) {
            Some(scanout) => match scanout.set_mouse_mode(&self.display, mouse_mode) {
                Ok(_) => GpuControlResult::DisplayMouseModeSet,
                Err(e) => GpuControlResult::ErrString(e.to_string()),
            },
            None => GpuControlResult::NoSuchDisplay { display_id },
        }
    }

    /// Performs the given command to interact with or modify the device.
    pub fn process_gpu_control_command(&mut self, cmd: GpuControlCommand) -> GpuControlResult {
        match cmd {
            GpuControlCommand::AddDisplays { displays } => self.add_displays(displays),
            GpuControlCommand::ListDisplays => self.list_displays(),
            GpuControlCommand::RemoveDisplays { display_ids } => self.remove_displays(display_ids),
            GpuControlCommand::SetDisplayMouseMode {
                display_id,
                mouse_mode,
            } => self.set_display_mouse_mode(display_id, mouse_mode),
        }
    }

    /// Processes the internal `display` events and returns `true` if any display was closed.
    pub fn process_display(&mut self) -> ProcessDisplayResult {
        let mut display = self.display.borrow_mut();
        let result = display.dispatch_events();
        match result {
            Ok(_) => (),
            Err(e) => {
                error!("failed to dispatch events: {}", e);
                return ProcessDisplayResult::Error(e);
            }
        }

        for scanout in self.scanouts.values() {
            let close_requested = scanout
                .surface_id
                .map(|surface_id| display.close_requested(surface_id))
                .unwrap_or(false);

            if close_requested {
                return ProcessDisplayResult::CloseRequested;
            }
        }

        ProcessDisplayResult::Success
    }

    /// Sets the given resource id as the source of scanout to the display.
    pub fn set_scanout(
        &mut self,
        scanout_rect: virtio_gpu_rect,
        scanout_id: u32,
        resource_id: u32,
        scanout_data: Option<VirtioScanoutBlobData>,
    ) -> VirtioGpuResult {
        self.update_scanout_resource(
            SurfaceType::Scanout,
            Some(scanout_rect),
            scanout_id,
            scanout_data,
            resource_id,
        )
    }

    /// If the resource is the scanout resource, flush it to the display.
    pub fn flush_resource(&mut self, resource_id: u32) -> VirtioGpuResult {
        let completions = self.flush_resource_with_completion(resource_id)?;
        for completion in completions {
            completion.wait(None);
        }
        Ok(OkNoData)
    }

    /// Flushes a resource and returns the display completion waitables. Callers that cannot defer
    /// the virtqueue descriptor must wait for every returned completion before replying.
    pub fn flush_resource_with_completion(
        &mut self,
        resource_id: u32,
    ) -> Result<Vec<Waitable>, GpuResponse> {
        if resource_id == 0 {
            return Ok(Vec::new());
        }

        #[cfg(windows)]
        match self.rutabaga.resource_flush(resource_id) {
            Ok(_) => return Ok(Vec::new()),
            Err(RutabagaError::Unsupported) => {}
            Err(e) => return Err(ErrRutabaga(e)),
        }

        let resource = self
            .resources
            .get_mut(&resource_id)
            .ok_or(ErrInvalidResourceId)?;

        // `resource_id` has already been verified to be non-zero
        let resource_id = match NonZeroU32::new(resource_id) {
            Some(id) => Some(id),
            None => return Ok(Vec::new()),
        };

        let mut completions = Vec::new();
        for scanout in self.scanouts.values_mut() {
            if scanout.resource_id == resource_id {
                if let Some(completion) =
                    scanout.flush(&self.display, resource, &mut self.rutabaga)?
                {
                    completions.push(completion);
                }
            }
        }
        if self.cursor_scanout.resource_id == resource_id {
            if let Some(completion) =
                self.cursor_scanout
                    .flush(&self.display, resource, &mut self.rutabaga)?
            {
                completions.push(completion);
            }
        }

        Ok(completions)
    }

    /// Updates the cursor's memory to the given resource_id, and sets its position to the given
    /// coordinates.
    pub fn update_cursor(
        &mut self,
        resource_id: u32,
        scanout_id: u32,
        x: u32,
        y: u32,
    ) -> VirtioGpuResult {
        self.update_scanout_resource(SurfaceType::Cursor, None, scanout_id, None, resource_id)?;

        self.cursor_scanout.set_position(&self.display, x, y)?;

        self.flush_resource(resource_id)
    }

    /// Moves the cursor's position to the given coordinates.
    pub fn move_cursor(&mut self, _scanout_id: u32, x: u32, y: u32) -> VirtioGpuResult {
        self.cursor_scanout.set_position(&self.display, x, y)?;
        self.cursor_scanout.commit(&self.display)?;
        Ok(OkNoData)
    }

    /// Returns a uuid for the resource.
    pub fn resource_assign_uuid(&self, resource_id: u32) -> VirtioGpuResult {
        if !self.resources.contains_key(&resource_id) {
            return Err(ErrInvalidResourceId);
        }

        // TODO(stevensd): use real uuids once the virtio wayland protocol is updated to
        // handle more than 32 bits. For now, the virtwl driver knows that the uuid is
        // actually just the resource id.
        let mut uuid: [u8; 16] = [0; 16];
        for (idx, byte) in resource_id.to_be_bytes().iter().enumerate() {
            uuid[12 + idx] = *byte;
        }
        Ok(OkResourceUuid { uuid })
    }

    /// If supported, export the resource with the given `resource_id` to a file.
    pub fn export_resource(&mut self, resource_id: u32) -> ResourceResponse {
        let handle = match self.rutabaga.export_blob(resource_id) {
            Ok(handle) => to_safe_descriptor(handle.os_handle),
            Err(_) => return ResourceResponse::Invalid,
        };

        let q = match self.rutabaga.query(resource_id) {
            Ok(query) => query,
            Err(_) => return ResourceResponse::Invalid,
        };

        ResourceResponse::Resource(ResourceInfo::Buffer(BufferInfo {
            handle,
            planes: [
                PlaneInfo {
                    offset: q.offsets[0],
                    stride: q.strides[0],
                },
                PlaneInfo {
                    offset: q.offsets[1],
                    stride: q.strides[1],
                },
                PlaneInfo {
                    offset: q.offsets[2],
                    stride: q.strides[2],
                },
                PlaneInfo {
                    offset: q.offsets[3],
                    stride: q.strides[3],
                },
            ],
            modifier: q.modifier,
            guest_cpu_mappable: q.guest_cpu_mappable,
        }))
    }

    /// If supported, export the fence with the given `fence_id` to a file.
    pub fn export_fence(&mut self, fence_id: u64) -> ResourceResponse {
        match self.rutabaga.export_fence(fence_id) {
            Ok(handle) => ResourceResponse::Resource(ResourceInfo::Fence {
                handle: to_safe_descriptor(handle.os_handle),
            }),
            Err(_) => ResourceResponse::Invalid,
        }
    }

    /// Gets rutabaga's capset information associated with `index`.
    pub fn get_capset_info(&self, index: u32) -> VirtioGpuResult {
        if let Ok((capset_id, version, size)) = self.rutabaga.get_capset_info(index) {
            Ok(OkCapsetInfo {
                capset_id,
                version,
                size,
            })
        } else {
            // Any capset_id > 63 is invalid according to the virtio-gpu spec, so we can
            // intentionally poison the capset without stalling the guest kernel driver.
            base::warn!(
                "virtio-gpu get_capset_info(index={}) failed. intentionally poisoning response",
                index
            );
            Ok(OkCapsetInfo {
                capset_id: u32::MAX,
                version: 0,
                size: 0,
            })
        }
    }

    /// Gets a capset from rutabaga.
    pub fn get_capset(&self, capset_id: u32, version: u32) -> VirtioGpuResult {
        let capset = self.rutabaga.get_capset(capset_id, version)?;
        Ok(OkCapset(capset))
    }

    /// Forces rutabaga to use it's default context.
    pub fn force_ctx_0(&self) {
        self.rutabaga.force_ctx_0()
    }

    /// Creates a fence with the RutabagaFence that can be used to determine when the previous
    /// command completed.
    pub fn create_fence(&mut self, rutabaga_fence: RutabagaFence) -> VirtioGpuResult {
        self.rutabaga.create_fence(rutabaga_fence)?;
        Ok(OkNoData)
    }

    /// Polls the Rutabaga backend.
    pub fn event_poll(&self) {
        self.rutabaga.event_poll();
    }

    /// Gets a pollable eventfd that signals the device to wakeup and poll the
    /// Rutabaga backend.
    pub fn poll_descriptor(&self) -> Option<SafeDescriptor> {
        self.rutabaga.poll_descriptor().map(to_safe_descriptor)
    }

    /// Creates a 3D resource with the given properties and resource_id.
    pub fn resource_create_3d(
        &mut self,
        resource_id: u32,
        resource_create_3d: ResourceCreate3D,
    ) -> VirtioGpuResult {
        self.rutabaga
            .resource_create_3d(resource_id, resource_create_3d)?;

        let resource = VirtioGpuResource::new(
            resource_id,
            resource_create_3d.width,
            resource_create_3d.height,
            0,
        );

        // Rely on rutabaga to check for duplicate resource ids.
        self.resources.insert(resource_id, resource);
        Ok(self.result_from_query(resource_id))
    }

    /// Attaches backing memory to the given resource, represented by a `Vec` of `(address, size)`
    /// tuples in the guest's physical address space. Converts to RutabagaIovec from the memory
    /// mapping.
    pub fn attach_backing(
        &mut self,
        resource_id: u32,
        mem: &GuestMemory,
        vecs: Vec<(GuestAddress, usize)>,
    ) -> VirtioGpuResult {
        let resource = self
            .resources
            .get_mut(&resource_id)
            .ok_or(ErrInvalidResourceId)?;

        let rutabaga_iovecs = sglist_to_rutabaga_iovecs(&vecs[..], mem).map_err(|_| ErrUnspec)?;
        self.rutabaga.attach_backing(resource_id, rutabaga_iovecs)?;
        resource.backing_iovecs = Some(vecs);
        Ok(OkNoData)
    }

    /// Detaches any previously attached iovecs from the resource.
    pub fn detach_backing(&mut self, resource_id: u32) -> VirtioGpuResult {
        let resource = self
            .resources
            .get_mut(&resource_id)
            .ok_or(ErrInvalidResourceId)?;

        self.rutabaga.detach_backing(resource_id)?;
        resource.backing_iovecs = None;
        Ok(OkNoData)
    }

    /// Releases guest kernel reference on the resource.
    pub fn unref_resource(&mut self, resource_id: u32) -> VirtioGpuResult {
        let mut resource = self
            .resources
            .remove(&resource_id)
            .ok_or(ErrInvalidResourceId)?;

        resource
            .transition_display_import(&mut self.display.borrow_mut(), DisplayImportState::Unknown);

        if resource.rutabaga_external_mapping {
            self.rutabaga.unmap(resource_id)?;
        }

        self.rutabaga.unref_resource(resource_id)?;
        Ok(OkNoData)
    }

    /// Copies data to host resource from the attached iovecs. Can also be used to flush caches.
    pub fn transfer_write(
        &mut self,
        ctx_id: u32,
        resource_id: u32,
        transfer: Transfer3D,
    ) -> VirtioGpuResult {
        self.rutabaga
            .transfer_write(ctx_id, resource_id, transfer)?;
        Ok(OkNoData)
    }

    /// Copies data from the host resource to:
    ///    1) To the optional volatile slice
    ///    2) To the host resource's attached iovecs
    ///
    /// Can also be used to invalidate caches.
    pub fn transfer_read(
        &mut self,
        ctx_id: u32,
        resource_id: u32,
        transfer: Transfer3D,
        buf: Option<VolatileSlice>,
    ) -> VirtioGpuResult {
        let buf = buf.map(|vs| {
            IoSliceMut::new(
                // SAFETY: trivially safe
                unsafe { std::slice::from_raw_parts_mut(vs.as_mut_ptr(), vs.size()) },
            )
        });
        self.rutabaga
            .transfer_read(ctx_id, resource_id, transfer, buf)?;
        Ok(OkNoData)
    }

    /// Creates a blob resource using rutabaga.
    pub fn resource_create_blob(
        &mut self,
        ctx_id: u32,
        resource_id: u32,
        resource_create_blob: ResourceCreateBlob,
        vecs: Vec<(GuestAddress, usize)>,
        mem: &GuestMemory,
    ) -> VirtioGpuResult {
        let mut descriptor = None;
        let mut rutabaga_iovecs = None;

        if resource_create_blob.blob_flags & VIRTIO_GPU_BLOB_FLAG_CREATE_GUEST_HANDLE != 0 {
            descriptor = match self.udmabuf_driver {
                Some(ref driver) => Some(driver.create_udmabuf(mem, &vecs[..])?),
                None => return Err(ErrUnspec),
            }
        } else if resource_create_blob.blob_mem != VIRTIO_GPU_BLOB_MEM_HOST3D {
            rutabaga_iovecs =
                Some(sglist_to_rutabaga_iovecs(&vecs[..], mem).map_err(|_| ErrUnspec)?);
        }

        self.rutabaga.resource_create_blob(
            ctx_id,
            resource_id,
            resource_create_blob,
            rutabaga_iovecs,
            descriptor.map(|descriptor| RutabagaHandle {
                os_handle: to_rutabaga_descriptor(descriptor),
                handle_type: RUTABAGA_HANDLE_TYPE_MEM_DMABUF,
            }),
        )?;

        let resource = VirtioGpuResource::new(resource_id, 0, 0, resource_create_blob.size);

        // Rely on rutabaga to check for duplicate resource ids.
        self.resources.insert(resource_id, resource);
        Ok(self.result_from_query(resource_id))
    }

    /// Uses the hypervisor to map the rutabaga blob resource.
    ///
    /// When sandboxing is disabled, external_blob is unset and opaque fds are mapped by
    /// rutabaga as ExternalMapping.
    /// When sandboxing is enabled, external_blob is set and opaque fds must be mapped in the
    /// hypervisor process by Vulkano using metadata provided by Rutabaga::vulkan_info().
    pub fn resource_map_blob(&mut self, resource_id: u32, offset: u64) -> VirtioGpuResult {
        let configured_arena = kgsl_host_visible_arena();
        let arena = if offset == KGSL_ARENA_SENTINEL {
            configured_arena.ok_or(ErrUnspec)?
        } else {
            KgslHostVisibleArena {
                host_va: 0,
                size: 0,
            }
        };
        let resource = self
            .resources
            .get_mut(&resource_id)
            .ok_or(ErrInvalidResourceId)?;

        let map_info = self.rutabaga.map_info(resource_id).map_err(|_| ErrUnspec)?;

        if offset != KGSL_ARENA_SENTINEL {
            if let Some(arena) = configured_arena {
                if let Ok(mapping) = self.rutabaga.map(resource_id) {
                    let arena_end = arena.host_va.checked_add(arena.size).ok_or(ErrUnspec)?;
                    let mapping_end = mapping.ptr.checked_add(resource.size).ok_or(ErrUnspec)?;
                    let overlaps_arena = mapping.ptr < arena_end && mapping_end > arena.host_va;
                    self.rutabaga.unmap(resource_id).map_err(|_| ErrUnspec)?;

                    if overlaps_arena {
                        base::warn!(
                            "GPU-MAPBLOB: rejecting legacy offset 0x{:x} for arena-backed res={}",
                            offset,
                            resource_id,
                        );
                        return Err(ErrUnspec);
                    }
                }
            }
        }

        if offset == KGSL_ARENA_SENTINEL {
            let mapping = self.rutabaga.map(resource_id).map_err(|_| ErrUnspec)?;
            let arena_end = arena.host_va.checked_add(arena.size).ok_or(ErrUnspec)?;
            let mapping_end = mapping.ptr.checked_add(resource.size).ok_or(ErrUnspec)?;
            if mapping.ptr < arena.host_va
                || mapping_end > arena_end
                || mapping.size < resource.size
            {
                let _ = self.rutabaga.unmap(resource_id);
                base::warn!(
                    "GPU-MAPBLOB: res={} returned non-arena mapping ptr=0x{:x} size=0x{:x}",
                    resource_id,
                    mapping.ptr,
                    mapping.size,
                );
                return Err(ErrUnspec);
            }

            let arena_offset = mapping.ptr - arena.host_va;
            if arena_offset % base::pagesize() as u64 != 0 {
                let _ = self.rutabaga.unmap(resource_id);
                base::warn!(
                    "GPU-MAPBLOB: res={} returned unaligned arena offset 0x{:x}",
                    resource_id,
                    arena_offset,
                );
                return Err(ErrUnspec);
            }
            resource.shmem_offset = Some(arena_offset);
            resource.rutabaga_external_mapping = true;
            resource.arena_mapped = true;
            resource.transition_display_import(
                &mut self.display.borrow_mut(),
                DisplayImportState::Unknown,
            );
            return Ok(OkMapInfo {
                map_info: map_info & RUTABAGA_MAP_CACHE_MASK,
                gunyah_handle: None,
                arena_offset: Some(arena_offset),
            });
        }

        let mut source: Option<VmMemorySource> = None;
        match self.rutabaga.export_blob(resource_id) {
            Ok(export) => {
                let has_vk = self.rutabaga.vulkan_info(resource_id).is_ok();
                base::debug!(
                    "GPU-MAPBLOB: res={} export OK handle_type=0x{:x} vulkan_info={} offset={}",
                    resource_id,
                    export.handle_type,
                    has_vk,
                    offset,
                );
                if let Ok(vulkan_info) = self.rutabaga.vulkan_info(resource_id) {
                    source = Some(VmMemorySource::Vulkan {
                        descriptor: to_safe_descriptor(export.os_handle),
                        handle_type: export.handle_type,
                        memory_idx: vulkan_info.memory_idx,
                        device_uuid: vulkan_info.device_id.device_uuid,
                        driver_uuid: vulkan_info.device_id.driver_uuid,
                        size: resource.size,
                    });
                } else if export.handle_type != RUTABAGA_HANDLE_TYPE_MEM_OPAQUE_FD {
                    source = Some(VmMemorySource::Descriptor {
                        descriptor: to_safe_descriptor(export.os_handle),
                        offset: 0,
                        size: resource.size,
                    });
                }
            }
            Err(e) => {
                // Not an error: expected for ColorBuffers whose Vulkan memory this Adreno can't
                // export as AHB/dmabuf; falls through to the rutabaga host-ptr map below.
                base::debug!(
                    "GPU-MAPBLOB: res={} export_blob ERR {:?} offset={}",
                    resource_id,
                    e,
                    offset,
                );
            }
        }

        // qemu-android-gunyah parity: when export_blob yields no usable OS handle (e.g. a
        // ColorBuffer whose Vulkan memory this Adreno can't export as AHB/dmabuf), fall back to
        // rutabaga's host-pointer mapping — exactly what qemu does for every blob
        // (rutabaga_resource_map -> memory_region_init_ram_ptr(mapping.ptr)). This avoids the
        // InvalidRutabagaHandle dead-end. ExternalMapping (a raw host VA) is only unsafe when the
        // GPU device is sandboxed; this VM runs --disable-sandbox, so the pointer is valid in-proc.
        // NOTE: the original gate returned ErrUnspec here when external_blob/fixed_blob_mapping
        // were set; we deliberately relax it for the Gunyah + disable-sandbox configuration.
        if source.is_none() {
            if self.fixed_blob_mapping {
                return Err(ErrUnspec);
            }

            match self.rutabaga.map(resource_id) {
                Ok(mapping) => {
                    base::debug!(
                        "GPU-MAPBLOB: res={} export failed, fallback rutabaga.map() OK ptr=0x{:x} size={} (qemu host-ptr path)",
                        resource_id,
                        mapping.ptr,
                        mapping.size,
                    );
                    // resources mapped via rutabaga must also be marked for unmap via rutabaga.
                    resource.rutabaga_external_mapping = true;
                    source = Some(VmMemorySource::ExternalMapping {
                        ptr: mapping.ptr,
                        size: mapping.size,
                    });
                }
                Err(e) => {
                    base::warn!(
                        "GPU-MAPBLOB: res={} export failed AND rutabaga.map() ERR {:?} (not host-mappable)",
                        resource_id,
                        e,
                    );
                    return Err(ErrUnspec);
                }
            }
        };

        let prot = match map_info & RUTABAGA_MAP_ACCESS_MASK {
            RUTABAGA_MAP_ACCESS_READ => Protection::read(),
            RUTABAGA_MAP_ACCESS_WRITE => Protection::write(),
            RUTABAGA_MAP_ACCESS_RW => Protection::read_write(),
            _ => return Err(ErrUnspec),
        };

        let cache = if cfg!(feature = "noncoherent-dma")
            && map_info & RUTABAGA_MAP_CACHE_MASK != RUTABAGA_MAP_CACHE_CACHED
        {
            MemCacheType::CacheNonCoherent
        } else {
            MemCacheType::CacheCoherent
        };

        let gunyah_handle = self
            .mapper
            .lock()
            .as_mut()
            .expect("No backend request connection found")
            .add_mapping_blob(source.unwrap(), offset, prot, cache)
            .map_err(|_| ErrUnspec)?;

        resource.shmem_offset = Some(offset);
        resource
            .transition_display_import(&mut self.display.borrow_mut(), DisplayImportState::Unknown);
        // Access flags not a part of the virtio-gpu spec.
        Ok(OkMapInfo {
            map_info: map_info & RUTABAGA_MAP_CACHE_MASK,
            // On Gunyah, the guest must accept this memparcel handle to map the blob itself.
            gunyah_handle,
            arena_offset: None,
        })
    }

    /// Uses the hypervisor to unmap the blob resource.
    pub fn resource_unmap_blob(&mut self, resource_id: u32) -> VirtioGpuResult {
        // Gunyah: actually reclaim the SHARE'd blob now (instead of the old PIN no-op that left it
        // shared forever). remove_mapping -> UnregisterMemory -> Vm::unshare_blob does the
        // gh_rm_mem_reclaim. The guest's virtio-gpu driver releases its own stage-2 acceptance
        // (gunyah_guest_mem_release) BEFORE sending this UNMAP, so the host-side reclaim here is
        // safe and keeps the BAR offset free for clean reuse -- fixing the offset-0 mem_share
        // EINVAL that the lazy overlap-reclaim caused by orphaning still-live parcels.
        let resource = self
            .resources
            .get_mut(&resource_id)
            .ok_or(ErrInvalidResourceId)?;

        let shmem_offset = resource.shmem_offset.ok_or(ErrUnspec)?;
        resource
            .transition_display_import(&mut self.display.borrow_mut(), DisplayImportState::Unknown);

        if resource.arena_mapped {
            if resource.rutabaga_external_mapping {
                self.rutabaga.unmap(resource_id)?;
                resource.rutabaga_external_mapping = false;
            }
            resource.shmem_offset = None;
            resource.arena_mapped = false;
            return Ok(OkNoData);
        }

        self.mapper
            .lock()
            .as_mut()
            .expect("No backend request connection found")
            .remove_mapping(shmem_offset)
            .map_err(|_| ErrUnspec)?;
        resource.shmem_offset = None;

        if resource.rutabaga_external_mapping {
            self.rutabaga.unmap(resource_id)?;
            resource.rutabaga_external_mapping = false;
        }

        Ok(OkNoData)
    }

    /// Gets the EDID for the specified scanout ID. If that scanout is not enabled, it would return
    /// the EDID of a default display.
    pub fn get_edid(&self, scanout_id: u32) -> VirtioGpuResult {
        let display_info = match self.scanouts.get(&scanout_id) {
            Some(scanout) => {
                // Primary scanouts should always have display params.
                let params = scanout.display_params.as_ref().unwrap();
                DisplayInfo::new(params)
            }
            None => DisplayInfo::new(&Default::default()),
        };
        EdidBytes::new(&display_info)
    }

    /// Creates a rutabaga context.
    pub fn create_context(
        &mut self,
        ctx_id: u32,
        context_init: u32,
        context_name: Option<&str>,
    ) -> VirtioGpuResult {
        self.rutabaga
            .create_context(ctx_id, context_init, context_name)?;
        Ok(OkNoData)
    }

    /// Destroys a rutabaga context.
    pub fn destroy_context(&mut self, ctx_id: u32) -> VirtioGpuResult {
        self.rutabaga.destroy_context(ctx_id)?;
        Ok(OkNoData)
    }

    /// Attaches a resource to a rutabaga context.
    pub fn context_attach_resource(&mut self, ctx_id: u32, resource_id: u32) -> VirtioGpuResult {
        self.rutabaga.context_attach_resource(ctx_id, resource_id)?;
        Ok(OkNoData)
    }

    /// Detaches a resource from a rutabaga context.
    pub fn context_detach_resource(&mut self, ctx_id: u32, resource_id: u32) -> VirtioGpuResult {
        self.rutabaga.context_detach_resource(ctx_id, resource_id)?;
        Ok(OkNoData)
    }

    /// Submits a command buffer to a rutabaga context.
    pub fn submit_command(
        &mut self,
        ctx_id: u32,
        commands: &mut [u8],
        fence_ids: &[u64],
    ) -> VirtioGpuResult {
        self.rutabaga.submit_command(ctx_id, commands, fence_ids)?;
        Ok(OkNoData)
    }

    // Non-public function -- no doc comment needed!
    fn result_from_query(&mut self, resource_id: u32) -> GpuResponse {
        match self.rutabaga.query(resource_id) {
            Ok(query) => {
                let mut plane_info = Vec::with_capacity(4);
                for plane_index in 0..4 {
                    plane_info.push(GpuResponsePlaneInfo {
                        stride: query.strides[plane_index],
                        offset: query.offsets[plane_index],
                    });
                }
                let format_modifier = query.modifier;
                OkResourcePlaneInfo {
                    format_modifier,
                    plane_info,
                }
            }
            Err(_) => OkNoData,
        }
    }

    fn update_scanout_resource(
        &mut self,
        scanout_type: SurfaceType,
        scanout_rect: Option<virtio_gpu_rect>,
        scanout_id: u32,
        scanout_data: Option<VirtioScanoutBlobData>,
        resource_id: u32,
    ) -> VirtioGpuResult {
        let scanout: &mut VirtioGpuScanout;
        let mut scanout_parent_surface_id = None;

        match scanout_type {
            SurfaceType::Cursor => {
                let parent_scanout_id = scanout_id;

                scanout_parent_surface_id = self
                    .scanouts
                    .get(&parent_scanout_id)
                    .ok_or(ErrInvalidScanoutId)
                    .map(|parent_scanout| parent_scanout.surface_id)?;

                scanout = &mut self.cursor_scanout;
            }
            SurfaceType::Scanout => {
                scanout = self
                    .scanouts
                    .get_mut(&scanout_id)
                    .ok_or(ErrInvalidScanoutId)?;
            }
        };

        // Virtio spec: "The driver can use resource_id = 0 to disable a scanout."
        if resource_id == 0 {
            // Ignore any initial set_scanout(..., resource_id: 0) calls.
            if scanout.resource_id.is_some() {
                scanout.release_surface(&self.display);
            }

            scanout.resource_id = None;
            return Ok(OkNoData);
        }

        let resource = self
            .resources
            .get_mut(&resource_id)
            .ok_or(ErrInvalidResourceId)?;

        // Ensure scanout has a display surface.
        let previous_surface_id = scanout.surface_id;
        match scanout_type {
            SurfaceType::Cursor => {
                if let Some(scanout_parent_surface_id) = scanout_parent_surface_id {
                    scanout.create_surface(
                        &self.display,
                        Some(scanout_parent_surface_id),
                        scanout_rect,
                    )?;
                }
            }
            SurfaceType::Scanout => {
                scanout.create_surface(&self.display, None, scanout_rect)?;
            }
        }

        if resource.scanout_data != scanout_data || scanout.surface_id != previous_surface_id {
            resource.transition_display_import(
                &mut self.display.borrow_mut(),
                DisplayImportState::Unknown,
            );
        }
        resource.scanout_data = scanout_data;

        // `resource_id` has already been verified to be non-zero
        let resource_id = match NonZeroU32::new(resource_id) {
            Some(id) => id,
            None => return Ok(OkNoData),
        };
        scanout.resource_id = Some(resource_id);

        Ok(OkNoData)
    }

    pub fn suspend(&self) -> anyhow::Result<()> {
        self.rutabaga
            .suspend()
            .context("failed to suspend rutabaga")
    }

    /// Reset the device to a clean state on a guest-initiated device reset: forget each scanout's
    /// resource association and drop every resource/context (ours and rutabaga's), while keeping
    /// the rutabaga render server alive so re-init is instant. Lets a guest that takes the device
    /// over from another one (UEFI firmware -> OS) recreate resource ids from scratch -- rutabaga
    /// rejects a duplicate resource id otherwise.
    pub fn reset(&mut self) -> anyhow::Result<()> {
        for scanout in self.scanouts.values_mut() {
            scanout.resource_id = None;
            // Drop the previous guest's surface: update_scanout_resource() only recreates a
            // surface when the modeset size differs from scanout.width/height, so after the
            // restore below, an OS modeset to the configured size would otherwise keep using a
            // stale firmware-geometry surface (its content posted top-left into a larger frame).
            scanout.release_surface(&self.display);
            // Also restore the configured boot resolution. set_scanout() tracks the guest's
            // modesets in scanout.width/height, which GET_DISPLAY_INFO then reports; a device
            // reset means a NEW guest is taking over, and the previous guest's last modeset
            // (e.g. the UEFI firmware console at 800x600) must not leak into it. The guest
            // virtio-gpu driver prunes any EDID *preferred* mode that mismatches display info
            // by >16px, so a leaked firmware resolution permanently locks the OS out of the
            // configured mode and it falls back to an arbitrary (wrong-aspect) one.
            if let Some(params) = &scanout.display_params {
                let (width, height) = params.get_virtual_display_size();
                info!(
                    "gpu reset: scanout {:?} restored {}x{} -> {}x{}, surface dropped",
                    scanout.scanout_id, scanout.width, scanout.height, width, height
                );
                scanout.width = width;
                scanout.height = height;
            }
        }
        self.cursor_scanout.resource_id = None;
        self.cursor_scanout.release_surface(&self.display);
        {
            let mut display = self.display.borrow_mut();
            for resource in self.resources.values_mut() {
                resource.transition_display_import(&mut display, DisplayImportState::Unknown);
            }
        }
        self.resources.clear();
        self.rutabaga.reset().context("failed to reset rutabaga")?;
        Ok(())
    }

    pub fn snapshot(&self) -> anyhow::Result<VirtioGpuSnapshot> {
        let snapshot_directory_tempdir = if let Some(dir) = &self.snapshot_scratch_directory {
            tempfile::tempdir_in(dir).with_context(|| {
                format!(
                    "failed to create tempdir in {} for gpu rutabaga snapshot",
                    dir.display()
                )
            })?
        } else {
            tempfile::tempdir().context("failed to create tempdir for gpu rutabaga snapshot")?
        };
        let snapshot_directory = snapshot_directory_tempdir.path();

        Ok(VirtioGpuSnapshot {
            scanouts: self
                .scanouts
                .iter()
                .map(|(i, s)| (*i, s.snapshot()))
                .collect(),
            scanouts_updated: self.scanouts_updated.load(Ordering::SeqCst),
            cursor_scanout: self.cursor_scanout.snapshot(),
            rutabaga: {
                self.rutabaga
                    .snapshot(snapshot_directory)
                    .context("failed to snapshot rutabaga")?;

                pack_directory_to_snapshot(snapshot_directory).with_context(|| {
                    format!(
                        "failed to pack rutabaga snapshot from {}",
                        snapshot_directory.display()
                    )
                })?
            },
            resources: self
                .resources
                .iter()
                .map(|(i, r)| (*i, r.snapshot()))
                .collect(),
        })
    }

    pub fn restore(&mut self, snapshot: VirtioGpuSnapshot) -> anyhow::Result<()> {
        self.deferred_snapshot_load = Some(snapshot);
        Ok(())
    }

    pub fn resume(&mut self, mem: &GuestMemory) -> anyhow::Result<()> {
        if let Some(snapshot) = self.deferred_snapshot_load.take() {
            assert!(self.scanouts.keys().eq(snapshot.scanouts.keys()));
            for (i, s) in snapshot.scanouts.into_iter() {
                self.scanouts
                    .get_mut(&i)
                    .unwrap()
                    .restore(
                        s,
                        // Only the cursor scanout can have a parent.
                        None,
                        &self.display,
                    )
                    .context("failed to restore scanouts")?;
            }
            self.scanouts_updated
                .store(snapshot.scanouts_updated, Ordering::SeqCst);

            let cursor_parent_surface_id = snapshot
                .cursor_scanout
                .parent_scanout_id
                .and_then(|i| self.scanouts.get(&i).unwrap().surface_id);
            self.cursor_scanout
                .restore(
                    snapshot.cursor_scanout,
                    cursor_parent_surface_id,
                    &self.display,
                )
                .context("failed to restore cursor scanout")?;

            let snapshot_directory_tempdir = if let Some(dir) = &self.snapshot_scratch_directory {
                tempfile::tempdir_in(dir).with_context(|| {
                    format!(
                        "failed to create tempdir in {} for gpu rutabaga snapshot",
                        dir.display()
                    )
                })?
            } else {
                tempfile::tempdir().context("failed to create tempdir for gpu rutabaga snapshot")?
            };
            let snapshot_directory = snapshot_directory_tempdir.path();

            unpack_snapshot_to_directory(snapshot_directory, snapshot.rutabaga).with_context(
                || {
                    format!(
                        "failed to unpack rutabaga snapshot to {}",
                        snapshot_directory.display()
                    )
                },
            )?;
            self.rutabaga
                .restore(snapshot_directory)
                .context("failed to restore rutabaga")?;

            for (id, s) in snapshot.resources.into_iter() {
                let backing_iovecs = s.backing_iovecs.clone();
                let shmem_offset = s.shmem_offset;
                self.resources.insert(id, VirtioGpuResource::restore(s));
                if let Some(backing_iovecs) = backing_iovecs {
                    self.attach_backing(id, mem, backing_iovecs)
                        .context("failed to restore resource backing")?;
                }
                if let Some(shmem_offset) = shmem_offset {
                    self.resource_map_blob(id, shmem_offset)
                        .context("failed to restore resource mapping")?;
                }
            }
        }

        self.rutabaga.resume().context("failed to resume rutabaga")
    }
}
