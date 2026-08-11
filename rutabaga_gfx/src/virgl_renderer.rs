// Copyright 2020 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! virgl_renderer: Handles 3D virtio-gpu hypercalls using virglrenderer.
//! External code found at <https://gitlab.freedesktop.org/virgl/virglrenderer/>.

#![cfg(feature = "virgl_renderer")]

use std::ffi::CStr;
use std::io::Error as SysError;
use std::io::IoSliceMut;
use std::mem::size_of;
use std::mem::transmute;
use std::mem::ManuallyDrop;
use std::os::raw::c_char;
use std::os::raw::c_int;
use std::os::raw::c_void;
use std::panic::catch_unwind;
use std::process::abort;
use std::ptr::null_mut;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::sync::Arc;

use log::debug;
use log::error;
use log::info;
use log::warn;

use crate::generated::virgl_renderer_bindings::*;
use crate::renderer_utils::*;
use crate::rutabaga_core::RutabagaComponent;
use crate::rutabaga_core::RutabagaContext;
use crate::rutabaga_core::RutabagaResource;
use crate::rutabaga_os::AsRawDescriptor;
use crate::rutabaga_os::FromRawDescriptor;
use crate::rutabaga_os::IntoRawDescriptor;
use crate::rutabaga_os::OwnedDescriptor;
use crate::rutabaga_os::RawDescriptor;
use crate::rutabaga_utils::*;

type Query = virgl_renderer_export_query;

fn dup(rd: RawDescriptor) -> RutabagaResult<OwnedDescriptor> {
    // SAFETY:
    // Safe because the underlying raw descriptor is guaranteed valid by rd's existence.
    //
    // Note that we are cloning the underlying raw descriptor since we have no guarantee of
    // its existence after this function returns.
    let rd_as_safe_desc = ManuallyDrop::new(unsafe { OwnedDescriptor::from_raw_descriptor(rd) });

    // We have to clone rd because we have no guarantee ownership was transferred (rd is
    // borrowed).
    Ok(rd_as_safe_desc.try_clone()?)
}

/// The virtio-gpu backend state tracker which supports accelerated rendering.
pub struct VirglRenderer {}

struct VirglRendererContext {
    ctx_id: u32,
}

fn import_resource(resource: &mut RutabagaResource) -> RutabagaResult<()> {
    if (resource.component_mask & (1 << (RutabagaComponentType::VirglRenderer as u8))) != 0 {
        return Ok(());
    }

    if let Some(handle) = &resource.handle {
        if handle.handle_type == RUTABAGA_HANDLE_TYPE_MEM_DMABUF {
            let dmabuf_fd = handle.os_handle.try_clone()?.into_raw_descriptor();
            // SAFETY:
            // Safe because we are being passed a valid fd
            unsafe {
                let dmabuf_size = libc::lseek64(dmabuf_fd, 0, libc::SEEK_END);
                libc::lseek64(dmabuf_fd, 0, libc::SEEK_SET);
                let args = virgl_renderer_resource_import_blob_args {
                    res_handle: resource.resource_id,
                    blob_mem: resource.blob_mem,
                    fd_type: VIRGL_RENDERER_BLOB_FD_TYPE_DMABUF,
                    fd: dmabuf_fd,
                    size: dmabuf_size as u64,
                };
                let ret = virgl_renderer_resource_import_blob(&args);
                if ret != 0 {
                    // import_blob can fail if we've previously imported this resource,
                    // but in any case virglrenderer does not take ownership of the fd
                    // in error paths
                    //
                    // Because of the re-import case we must still fall through to the
                    // virgl_renderer_ctx_attach_resource() call.
                    libc::close(dmabuf_fd);
                    return Ok(());
                }
                resource.component_mask |= 1 << (RutabagaComponentType::VirglRenderer as u8);
            }
        }
    }

    Ok(())
}

impl RutabagaContext for VirglRendererContext {
    fn submit_cmd(
        &mut self,
        commands: &mut [u8],
        fence_ids: &[u64],
        _shareable_fences: Vec<RutabagaHandle>,
    ) -> RutabagaResult<()> {
        #[cfg(not(virgl_renderer_unstable))]
        if !fence_ids.is_empty() {
            return Err(RutabagaError::Unsupported);
        }
        if commands.len() % size_of::<u32>() != 0 {
            return Err(RutabagaError::InvalidCommandSize(commands.len()));
        }
        let dword_count = (commands.len() / size_of::<u32>()) as i32;
        #[cfg(not(virgl_renderer_unstable))]
        // SAFETY:
        // Safe because the context and buffer are valid and virglrenderer will have been
        // initialized if there are Context instances.
        let ret = unsafe {
            virgl_renderer_submit_cmd(
                commands.as_mut_ptr() as *mut c_void,
                self.ctx_id as i32,
                dword_count,
            )
        };
        #[cfg(virgl_renderer_unstable)]
        // SAFETY:
        // Safe because the context and buffers are valid and virglrenderer will have been
        // initialized if there are Context instances.
        let ret = unsafe {
            virgl_renderer_submit_cmd2(
                commands.as_mut_ptr() as *mut c_void,
                self.ctx_id as i32,
                dword_count,
                fence_ids.as_ptr() as *mut u64,
                fence_ids.len() as u32,
            )
        };
        ret_to_res(ret)
    }

    fn attach(&mut self, resource: &mut RutabagaResource) {
        match import_resource(resource) {
            Ok(()) => (),
            Err(e) => error!("importing resource failing with {}", e),
        }

        // SAFETY:
        // The context id and resource id must be valid because the respective instances ensure
        // their lifetime.
        unsafe {
            virgl_renderer_ctx_attach_resource(self.ctx_id as i32, resource.resource_id as i32);
        }
    }

    fn detach(&mut self, resource: &RutabagaResource) {
        // SAFETY:
        // The context id and resource id must be valid because the respective instances ensure
        // their lifetime.
        unsafe {
            virgl_renderer_ctx_detach_resource(self.ctx_id as i32, resource.resource_id as i32);
        }
    }

    fn component_type(&self) -> RutabagaComponentType {
        RutabagaComponentType::VirglRenderer
    }

    fn context_create_fence(
        &mut self,
        fence: RutabagaFence,
    ) -> RutabagaResult<Option<RutabagaHandle>> {
        // RutabagaFence::flags are not compatible with virglrenderer's fencing API and currently
        // virglrenderer context's assume all fences on a single timeline are MERGEABLE, and enforce
        // this assumption.
        let flags: u32 = VIRGL_RENDERER_FENCE_FLAG_MERGEABLE;

        // TODO(b/315870313): Add safety comment
        #[allow(clippy::undocumented_unsafe_blocks)]
        let ret = unsafe {
            virgl_renderer_context_create_fence(
                fence.ctx_id,
                flags,
                fence.ring_idx as u32,
                fence.fence_id,
            )
        };
        ret_to_res(ret)?;
        Ok(None)
    }
}

impl Drop for VirglRendererContext {
    fn drop(&mut self) {
        // SAFETY:
        // The context is safe to destroy because nothing else can be referencing it.
        unsafe {
            virgl_renderer_context_destroy(self.ctx_id);
        }
    }
}

extern "C" fn log_callback(
    level: virgl_log_level_flags,
    message: *const c_char,
    _user_data: *mut c_void,
) {
    if message.is_null() {
        return;
    }

    // The C side formats each message once and preserves its level.  Keep the
    // CStr borrowed so disabled Rust log levels do not require another buffer.
    // SAFETY: virglrenderer passes a valid NUL-terminated string that remains
    // alive for the duration of this synchronous callback.
    let message = unsafe { CStr::from_ptr(message) };
    match level {
        VIRGL_LOG_LEVEL_DEBUG => debug!("{}", message.to_string_lossy()),
        VIRGL_LOG_LEVEL_INFO => info!("{}", message.to_string_lossy()),
        VIRGL_LOG_LEVEL_WARNING => warn!("{}", message.to_string_lossy()),
        VIRGL_LOG_LEVEL_ERROR => error!("{}", message.to_string_lossy()),
        _ => {}
    }
}

extern "C" fn write_context_fence(cookie: *mut c_void, ctx_id: u32, ring_idx: u32, fence_id: u64) {
    catch_unwind(|| {
        assert!(!cookie.is_null());
        // TODO(b/315870313): Add safety comment
        #[allow(clippy::undocumented_unsafe_blocks)]
        let cookie = unsafe { &*(cookie as *mut RutabagaCookie) };

        // Call fence completion callback
        if let Some(handler) = &cookie.fence_handler {
            handler.call(RutabagaFence {
                flags: RUTABAGA_FLAG_FENCE | RUTABAGA_FLAG_INFO_RING_IDX,
                fence_id,
                ctx_id,
                ring_idx: ring_idx as u8,
            });
        }
    })
    .unwrap_or_else(|_| abort())
}

// TODO(b/315870313): Add safety comment
#[allow(clippy::undocumented_unsafe_blocks)]
unsafe extern "C" fn write_fence(cookie: *mut c_void, fence: u32) {
    catch_unwind(|| {
        assert!(!cookie.is_null());
        let cookie = &*(cookie as *mut RutabagaCookie);

        // Call fence completion callback
        if let Some(handler) = &cookie.fence_handler {
            handler.call(RutabagaFence {
                flags: RUTABAGA_FLAG_FENCE,
                fence_id: fence as u64,
                ctx_id: 0,
                ring_idx: 0,
            });
        }
    })
    .unwrap_or_else(|_| abort())
}

// TODO(b/315870313): Add safety comment
#[allow(clippy::undocumented_unsafe_blocks)]
unsafe extern "C" fn get_server_fd(cookie: *mut c_void, version: u32) -> c_int {
    catch_unwind(|| {
        assert!(!cookie.is_null());
        let cookie = &mut *(cookie as *mut RutabagaCookie);

        if version != 0 {
            return -1;
        }

        // Transfer the fd ownership to virglrenderer.
        cookie
            .render_server_fd
            .take()
            .map(OwnedDescriptor::into_raw_descriptor)
            .unwrap_or(-1)
    })
    .unwrap_or_else(|_| abort())
}

const VIRGL_RENDERER_CALLBACKS: &virgl_renderer_callbacks = &virgl_renderer_callbacks {
    version: 3,
    write_fence: Some(write_fence),
    create_gl_context: None,
    destroy_gl_context: None,
    make_current: None,
    get_drm_fd: None,
    write_context_fence: Some(write_context_fence),
    get_server_fd: Some(get_server_fd),
    get_egl_display: None,
};

/// Retrieves metadata suitable for export about this resource. If "export_fd" is true,
/// performs an export of this resource so that it may be imported by other processes.
fn export_query(resource_id: u32) -> RutabagaResult<Query> {
    let mut query: Query = Default::default();
    query.hdr.stype = VIRGL_RENDERER_STRUCTURE_TYPE_EXPORT_QUERY;
    query.hdr.stype_version = 0;
    query.hdr.size = size_of::<Query>() as u32;
    query.in_resource_id = resource_id;
    query.in_export_fds = 0;

    let ret =
        // SAFETY:
        // Safe because the image parameters are stack variables of the correct type.
        unsafe { virgl_renderer_execute(&mut query as *mut _ as *mut c_void, query.hdr.size) };

    ret_to_res(ret)?;
    Ok(query)
}

/// The host-owned pool windows a virgl-family renderer sub-allocates from, as (host VA, size):
/// the drm2kgsl native-context arena and the venus_host transport pool. Announced by the VMM
/// before the GPU device is built; empty when neither pool was configured. A resource whose
/// persistent map_ptr falls inside one of these windows is pool-resident: the guest maps it at
/// pool_base+offset from the map response (MAP_INFO_POOL) and no runtime SHARE happens.
fn virgl_pool_windows() -> &'static [(u64, u64)] {
    static WINDOWS: std::sync::OnceLock<Vec<(u64, u64)>> = std::sync::OnceLock::new();
    WINDOWS.get_or_init(|| {
        let read = |va_key: &str, size_key: &str| -> Option<(u64, u64)> {
            let va: u64 = std::env::var(va_key).ok()?.parse().ok()?;
            let size: u64 = std::env::var(size_key).ok()?.parse().ok()?;
            (va != 0 && size != 0).then_some((va, size))
        };
        let mut v = Vec::new();
        if let Some(w) = read("CROSVM_DRM2KGSL_ARENA_HOST_VA", "CROSVM_DRM2KGSL_ARENA_SIZE") {
            v.push(w);
        }
        if let Some(w) = read("VENUS_POOL_HOST_VA", "VENUS_POOL_SIZE") {
            v.push(w);
        }
        v
    })
}

/// Byte offset of a resource inside the drm2kgsl pool, or None if it does not live there.
///
/// Asked at creation, when the drm2kgsl backend has just recorded the arena pointer on the
/// resource, and asked through an accessor that only READS it. The obvious alternative --
/// map() then unmap() -- is not a query: virglrenderer's map records res->mapped and its
/// unmap munmaps a dmabuf, so probing with the pair tears down mappings the renderer is
/// still using.
fn drm2kgsl_pool_offset(resource_id: u32) -> Option<u64> {
    let windows = virgl_pool_windows();
    if windows.is_empty() {
        return None;
    }
    let mut ptr: *mut c_void = null_mut();
    let mut size: u64 = 0;
    // SAFETY: the accessor only reads virgl_resource fields and writes the two out params.
    let ret = unsafe { virgl_renderer_resource_get_map_ptr(resource_id, &mut ptr, &mut size) };
    if ret != 0 {
        return None;
    }
    let addr = ptr as u64;
    let end = addr.checked_add(size)?;
    windows.iter().find_map(|&(pool_va, pool_size)| {
        (addr >= pool_va && end <= pool_va.checked_add(pool_size)?).then(|| addr - pool_va)
    })
}

impl VirglRenderer {
    pub fn init(
        virglrenderer_flags: VirglRendererFlags,
        fence_handler: RutabagaFenceHandler,
        render_server_fd: Option<OwnedDescriptor>,
    ) -> RutabagaResult<Box<dyn RutabagaComponent>> {
        if cfg!(debug_assertions) {
            // TODO(b/315870313): Add safety comment
            #[allow(clippy::undocumented_unsafe_blocks)]
            let ret = unsafe { libc::dup2(libc::STDOUT_FILENO, libc::STDERR_FILENO) };
            if ret == -1 {
                warn!(
                    "unable to dup2 stdout to stderr: {}",
                    SysError::last_os_error()
                );
            }
        }

        // virglrenderer is a global state backed library that uses thread bound OpenGL contexts.
        // Initialize it only once and use the non-send/non-sync Renderer struct to keep things tied
        // to whichever thread called this function first.
        static INIT_ONCE: AtomicBool = AtomicBool::new(false);
        if INIT_ONCE
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Acquire)
            .is_err()
        {
            return Err(RutabagaError::AlreadyInUse);
        }

        // max_level() only exposes the global upper bound.  Crosvm commonly
        // keeps that bound at TRACE while its logger filters individual
        // targets, so query this module's actual target before choosing the C
        // threshold.  This prevents vasprintf() for records Rust would drop.
        let log_level = if log::log_enabled!(target: module_path!(), log::Level::Debug) {
            VIRGL_LOG_LEVEL_DEBUG
        } else if log::log_enabled!(target: module_path!(), log::Level::Info) {
            VIRGL_LOG_LEVEL_INFO
        } else if log::log_enabled!(target: module_path!(), log::Level::Warn) {
            VIRGL_LOG_LEVEL_WARNING
        } else if log::log_enabled!(target: module_path!(), log::Level::Error) {
            VIRGL_LOG_LEVEL_ERROR
        } else {
            VIRGL_LOG_LEVEL_SILENT
        };

        // Keep virgl's C-side filtering in sync with crosvm's logger so a
        // disabled debug message is rejected before vasprintf().
        unsafe {
            virgl_set_log_level(log_level);
            virgl_set_log_callback(Some(log_callback), null_mut(), None);
        }

        // Cookie is intentionally never freed because virglrenderer never gets uninitialized.
        // Otherwise, Resource and Context would become invalid because their lifetime is not tied
        // to the Renderer instance. Doing so greatly simplifies the ownership for users of this
        // library.
        let cookie = Box::into_raw(Box::new(RutabagaCookie {
            render_server_fd,
            fence_handler: Some(fence_handler),
            debug_handler: None,
        }));

        // SAFETY:
        // Safe because a valid cookie and set of callbacks is used and the result is checked for
        // error.
        let ret = unsafe {
            virgl_renderer_init(
                cookie as *mut c_void,
                virglrenderer_flags.into(),
                transmute(VIRGL_RENDERER_CALLBACKS),
            )
        };

        ret_to_res(ret)?;
        Ok(Box::new(VirglRenderer {}))
    }

    fn map_info(&self, resource_id: u32) -> RutabagaResult<u32> {
        let mut map_info = 0;
        // TODO(b/315870313): Add safety comment
        #[allow(clippy::undocumented_unsafe_blocks)]
        let ret = unsafe { virgl_renderer_resource_get_map_info(resource_id, &mut map_info) };
        ret_to_res(ret)?;

        Ok(map_info | RUTABAGA_MAP_ACCESS_RW)
    }

    fn query(&self, resource_id: u32) -> RutabagaResult<Resource3DInfo> {
        let query = export_query(resource_id)?;
        if query.out_num_fds == 0 {
            return Err(RutabagaError::Unsupported);
        }

        // virglrenderer unfortunately doesn't return the width or height, so map to zero.
        Ok(Resource3DInfo {
            width: 0,
            height: 0,
            drm_fourcc: query.out_fourcc,
            strides: query.out_strides,
            offsets: query.out_offsets,
            modifier: query.out_modifier,
            guest_cpu_mappable: false,
        })
    }

    fn export_blob(&self, resource_id: u32) -> RutabagaResult<Arc<RutabagaHandle>> {
        let mut fd_type = 0;
        let mut fd = 0;
        // TODO(b/315870313): Add safety comment
        #[allow(clippy::undocumented_unsafe_blocks)]
        let ret =
            unsafe { virgl_renderer_resource_export_blob(resource_id, &mut fd_type, &mut fd) };
        ret_to_res(ret)?;

        // SAFETY:
        // Safe because the FD was just returned by a successful virglrenderer
        // call so it must be valid and owned by us.
        let handle = unsafe { OwnedDescriptor::from_raw_descriptor(fd) };

        let handle_type = match fd_type {
            VIRGL_RENDERER_BLOB_FD_TYPE_DMABUF => RUTABAGA_HANDLE_TYPE_MEM_DMABUF,
            VIRGL_RENDERER_BLOB_FD_TYPE_SHM => RUTABAGA_HANDLE_TYPE_MEM_SHM,
            VIRGL_RENDERER_BLOB_FD_TYPE_OPAQUE => RUTABAGA_HANDLE_TYPE_MEM_OPAQUE_FD,
            _ => {
                return Err(RutabagaError::Unsupported);
            }
        };

        Ok(Arc::new(RutabagaHandle {
            os_handle: handle,
            handle_type,
        }))
    }

    fn export_display_blob(&self, resource_id: u32) -> RutabagaResult<RutabagaHandle> {
        let mut fd_type = 0;
        let mut fd = -1;
        // SAFETY: virglrenderer owns the resource and returns a new fd on success.
        let ret = unsafe {
            virgl_renderer_resource_export_display_blob(resource_id, &mut fd_type, &mut fd)
        };
        ret_to_res(ret)?;
        if fd_type != VIRGL_RENDERER_BLOB_FD_TYPE_DMABUF || fd < 0 {
            if fd >= 0 {
                // The C API only returns ownership for the DMABUF case.
                unsafe { libc::close(fd) };
            }
            return Err(RutabagaError::Unsupported);
        }
        // SAFETY: fd is newly returned and owned by this handle.
        Ok(RutabagaHandle {
            os_handle: unsafe { OwnedDescriptor::from_raw_descriptor(fd) },
            handle_type: RUTABAGA_HANDLE_TYPE_MEM_DMABUF,
        })
    }
}

impl Drop for VirglRenderer {
    fn drop(&mut self) {
        // SAFETY:
        // Safe because virglrenderer is initialized.
        //
        // This invalidates all context ids and resource ids.  It is fine because struct Rutabaga
        // makes sure contexts and resources are dropped before this is reached.  Even if it did
        // not, virglrenderer is designed to deal with invalid ids safely.
        unsafe {
            virgl_renderer_cleanup(null_mut());
        }
    }
}

impl RutabagaComponent for VirglRenderer {
    fn export_display_blob(&self, resource_id: u32) -> RutabagaResult<RutabagaHandle> {
        VirglRenderer::export_display_blob(self, resource_id)
    }

    fn get_capset_info(&self, capset_id: u32) -> (u32, u32) {
        let mut version = 0;
        let mut size = 0;
        // SAFETY:
        // Safe because virglrenderer is initialized by now and properly size stack variables are
        // used for the pointers.
        unsafe {
            virgl_renderer_get_cap_set(capset_id, &mut version, &mut size);
        }
        (version, size)
    }

    fn get_capset(&self, capset_id: u32, version: u32) -> Vec<u8> {
        let (_, max_size) = self.get_capset_info(capset_id);
        let mut buf = vec![0u8; max_size as usize];
        // SAFETY:
        // Safe because virglrenderer is initialized by now and the given buffer is sized properly
        // for the given cap id/version.
        unsafe {
            virgl_renderer_fill_caps(capset_id, version, buf.as_mut_ptr() as *mut c_void);
        }
        buf
    }

    fn force_ctx_0(&self) {
        // TODO(b/315870313): Add safety comment
        #[allow(clippy::undocumented_unsafe_blocks)]
        unsafe {
            virgl_renderer_force_ctx_0()
        };
    }

    fn create_fence(&mut self, fence: RutabagaFence) -> RutabagaResult<()> {
        // A fence carrying a ring index belongs to a per-context timeline (venus
        // queue rings, drm native-context submit queues); handing it to the global
        // virgl_renderer_create_fence retires it on vrend's GL timeline instead and
        // the guest's per-ring fence never signals. Venus's WSI present blocked on
        // exactly that (sync_wait(-1) on the EXECBUF out-fence), wedging the whole
        // desktop behind the first swapchain buffer.
        // TODO(b/315870313): Add safety comment
        #[allow(clippy::undocumented_unsafe_blocks)]
        let ret = if fence.flags & RUTABAGA_FLAG_INFO_RING_IDX != 0 {
            unsafe {
                virgl_renderer_context_create_fence(
                    fence.ctx_id,
                    VIRGL_RENDERER_FENCE_FLAG_MERGEABLE,
                    fence.ring_idx.into(),
                    fence.fence_id,
                )
            }
        } else {
            unsafe { virgl_renderer_create_fence(fence.fence_id as i32, fence.ctx_id) }
        };
        ret_to_res(ret)
    }

    fn event_poll(&self) {
        // TODO(b/315870313): Add safety comment
        #[allow(clippy::undocumented_unsafe_blocks)]
        unsafe {
            virgl_renderer_poll()
        };
    }

    fn poll_descriptor(&self) -> Option<OwnedDescriptor> {
        // SAFETY:
        // Safe because it can be called anytime and returns -1 in the event of an error.
        let fd = unsafe { virgl_renderer_get_poll_fd() };
        if fd >= 0 {
            let descriptor: RawDescriptor = fd as RawDescriptor;
            if let Ok(dup_fd) = dup(descriptor) {
                return Some(dup_fd);
            }
        }
        None
    }

    fn create_3d(
        &self,
        resource_id: u32,
        resource_create_3d: ResourceCreate3D,
    ) -> RutabagaResult<RutabagaResource> {
        let mut args = virgl_renderer_resource_create_args {
            handle: resource_id,
            target: resource_create_3d.target,
            format: resource_create_3d.format,
            bind: resource_create_3d.bind,
            width: resource_create_3d.width,
            height: resource_create_3d.height,
            depth: resource_create_3d.depth,
            array_size: resource_create_3d.array_size,
            last_level: resource_create_3d.last_level,
            nr_samples: resource_create_3d.nr_samples,
            flags: resource_create_3d.flags,
        };

        // SAFETY:
        // Safe because virglrenderer is initialized by now, and the return value is checked before
        // returning a new resource. The backing buffers are not supplied with this call.
        let ret = unsafe { virgl_renderer_resource_create(&mut args, null_mut(), 0) };
        ret_to_res(ret)?;

        Ok(RutabagaResource {
            resource_id,
            handle: self.export_blob(resource_id).ok(),
            blob: false,
            blob_mem: 0,
            blob_flags: 0,
            map_info: None,
            info_2d: None,
            info_3d: self.query(resource_id).ok(),
            vulkan_info: None,
            backing_iovecs: None,
            component_mask: 1 << (RutabagaComponentType::VirglRenderer as u8),
            size: 0,
            mapping: None,
            pool_offset: None,
        })
    }

    fn attach_backing(
        &self,
        resource_id: u32,
        vecs: &mut Vec<RutabagaIovec>,
    ) -> RutabagaResult<()> {
        // SAFETY:
        // Safe because the backing is into guest memory that we store a reference count for.
        let ret = unsafe {
            virgl_renderer_resource_attach_iov(
                resource_id as i32,
                vecs.as_mut_ptr() as *mut iovec,
                vecs.len() as i32,
            )
        };
        ret_to_res(ret)
    }

    fn detach_backing(&self, resource_id: u32) {
        // SAFETY:
        // Safe as we don't need the old backing iovecs returned and the reference to the guest
        // memory can be dropped as it will no longer be needed for this resource.
        unsafe {
            virgl_renderer_resource_detach_iov(resource_id as i32, null_mut(), null_mut());
        }
    }

    fn unref_resource(&self, resource_id: u32) {
        // SAFETY:
        // The resource is safe to unreference destroy because no user of these bindings can still
        // be holding a reference.
        unsafe {
            virgl_renderer_resource_unref(resource_id);
        }
    }

    fn transfer_write(
        &self,
        ctx_id: u32,
        resource: &mut RutabagaResource,
        transfer: Transfer3D,
    ) -> RutabagaResult<()> {
        if transfer.is_empty() {
            return Ok(());
        }

        let mut transfer_box = VirglBox {
            x: transfer.x,
            y: transfer.y,
            z: transfer.z,
            w: transfer.w,
            h: transfer.h,
            d: transfer.d,
        };

        // SAFETY:
        // Safe because only stack variables of the appropriate type are used.
        let ret = unsafe {
            virgl_renderer_transfer_write_iov(
                resource.resource_id,
                ctx_id,
                transfer.level as i32,
                transfer.stride,
                transfer.layer_stride,
                &mut transfer_box as *mut VirglBox as *mut virgl_box,
                transfer.offset,
                null_mut(),
                0,
            )
        };
        ret_to_res(ret)
    }

    fn transfer_read(
        &self,
        ctx_id: u32,
        resource: &mut RutabagaResource,
        transfer: Transfer3D,
        buf: Option<IoSliceMut>,
    ) -> RutabagaResult<()> {
        if transfer.is_empty() {
            return Ok(());
        }

        let mut transfer_box = VirglBox {
            x: transfer.x,
            y: transfer.y,
            z: transfer.z,
            w: transfer.w,
            h: transfer.h,
            d: transfer.d,
        };

        let mut iov = RutabagaIovec {
            base: null_mut(),
            len: 0,
        };

        let (iovecs, num_iovecs) = match buf {
            Some(mut buf) => {
                iov.base = buf.as_mut_ptr() as *mut c_void;
                iov.len = buf.len();
                (&mut iov as *mut RutabagaIovec as *mut iovec, 1)
            }
            None => (null_mut(), 0),
        };

        // SAFETY:
        // Safe because only stack variables of the appropriate type are used.
        let ret = unsafe {
            virgl_renderer_transfer_read_iov(
                resource.resource_id,
                ctx_id,
                transfer.level,
                transfer.stride,
                transfer.layer_stride,
                &mut transfer_box as *mut VirglBox as *mut virgl_box,
                transfer.offset,
                iovecs,
                num_iovecs,
            )
        };
        ret_to_res(ret)
    }

    #[allow(unused_variables)]
    fn create_blob(
        &mut self,
        ctx_id: u32,
        resource_id: u32,
        resource_create_blob: ResourceCreateBlob,
        mut iovec_opt: Option<Vec<RutabagaIovec>>,
        handle_opt: Option<RutabagaHandle>,
    ) -> RutabagaResult<RutabagaResource> {
        let mut iovec_ptr = null_mut();
        let mut num_iovecs = 0;
        if let Some(ref mut iovecs) = iovec_opt {
            iovec_ptr = iovecs.as_mut_ptr();
            num_iovecs = iovecs.len();
        }

        // GUEST-ALLOC: the guest allocated these pages and the GPU device turned the blob's
        // iovecs into a dma-buf. virglrenderer's create_blob has nowhere to put it -- the DRM
        // backend needs it inside get_blob() -- so park it on the context first. Only the VMM
        // can build it: it alone holds the guest memfd the pages live in.
        //
        // Errors are not fatal here on purpose. -ENOTSUP means this context does not implement
        // guest-allocated blobs, in which case create_blob below behaves exactly as it did
        // before and the resource is backed the old way.
        if let Some(ref handle) = handle_opt {
            if handle.handle_type == RUTABAGA_HANDLE_TYPE_MEM_DMABUF {
                // Hand over a DUP, not the descriptor itself. virgl_renderer_resource_set_guest_blob_fd
                // takes ownership, and `handle` still owns os_handle and closes it when this
                // function returns -- passing the raw fd makes both sides close the same one.
                //
                // The second close lands on whatever inherited that fd number in between, which
                // is a long-lived socket often enough to matter: it showed up as the VNC server
                // failing a read with EBADF and dropping its client, a symptom with nothing in it
                // to suggest a blob descriptor.
                let dup_fd = handle.os_handle.try_clone()?.into_raw_descriptor();
                // SAFETY: dup_fd is a fresh descriptor this call gives away; nothing here closes it.
                let ret = unsafe {
                    virgl_renderer_resource_set_guest_blob_fd(
                        ctx_id,
                        resource_create_blob.blob_id,
                        dup_fd,
                    )
                };
                if ret != 0 {
                    // SAFETY: ownership only transfers on success, so the dup is still ours.
                    unsafe { libc::close(dup_fd) };
                    // -ENOTSUP just means this context does not implement guest-allocated
                    // blobs, which is not a problem: create_blob below behaves as before.
                    if ret != -(libc::ENOTSUP as i32) {
                        log::warn!(
                            "set_guest_blob_fd(ctx={} blob_id={}) failed: {}",
                            ctx_id,
                            resource_create_blob.blob_id,
                            ret
                        );
                    }
                }
            }
        }

        let resource_create_args = virgl_renderer_resource_create_blob_args {
            res_handle: resource_id,
            ctx_id,
            blob_mem: resource_create_blob.blob_mem,
            blob_flags: resource_create_blob.blob_flags,
            blob_id: resource_create_blob.blob_id,
            size: resource_create_blob.size,
            iovecs: iovec_ptr as *const iovec,
            num_iovs: num_iovecs as u32,
        };

        // TODO(b/315870313): Add safety comment
        #[allow(clippy::undocumented_unsafe_blocks)]
        let ret = unsafe { virgl_renderer_resource_create_blob(&resource_create_args) };
        ret_to_res(ret)?;

        // TODO(b/244591751): assign vulkan_info to support opaque_fd mapping via Vulkano when
        // sandboxing (hence external_blob) is enabled.
        Ok(RutabagaResource {
            resource_id,
            handle: self.export_blob(resource_id).ok(),
            blob: true,
            blob_mem: resource_create_blob.blob_mem,
            blob_flags: resource_create_blob.blob_flags,
            map_info: self.map_info(resource_id).ok(),
            info_2d: None,
            info_3d: self.query(resource_id).ok(),
            vulkan_info: None,
            backing_iovecs: iovec_opt,
            component_mask: 1 << (RutabagaComponentType::VirglRenderer as u8),
            size: resource_create_blob.size,
            mapping: None,
            pool_offset: drm2kgsl_pool_offset(resource_id),
        })
    }

    fn map(&self, resource_id: u32) -> RutabagaResult<RutabagaMapping> {
        let mut map: *mut c_void = null_mut();
        let mut size: u64 = 0;
        // SAFETY:
        // Safe because virglrenderer wraps and validates use of GL/VK.
        let ret = unsafe { virgl_renderer_resource_map(resource_id, &mut map, &mut size) };
        if ret != 0 {
            return Err(RutabagaError::MappingFailed(ret));
        }

        Ok(RutabagaMapping {
            ptr: map as u64,
            size,
        })
    }

    fn unmap(&self, resource_id: u32) -> RutabagaResult<()> {
        // SAFETY:
        // Safe because virglrenderer is initialized by now.
        let ret = unsafe { virgl_renderer_resource_unmap(resource_id) };
        ret_to_res(ret)
    }

    #[allow(unused_variables)]
    fn export_fence(&self, fence_id: u64) -> RutabagaResult<RutabagaHandle> {
        #[cfg(virgl_renderer_unstable)]
        {
            let mut fd: i32 = 0;
            // SAFETY:
            // Safe because the parameters are stack variables of the correct type.
            let ret = unsafe { virgl_renderer_export_fence(fence_id, &mut fd) };
            ret_to_res(ret)?;

            // SAFETY:
            // Safe because the FD was just returned by a successful virglrenderer call so it must
            // be valid and owned by us.
            let fence = unsafe { OwnedDescriptor::from_raw_descriptor(fd) };
            Ok(RutabagaHandle {
                os_handle: fence,
                handle_type: RUTABAGA_HANDLE_TYPE_SIGNAL_SYNC_FD,
            })
        }
        #[cfg(not(virgl_renderer_unstable))]
        Err(RutabagaError::Unsupported)
    }

    #[allow(unused_variables)]
    fn create_context(
        &self,
        ctx_id: u32,
        context_init: u32,
        context_name: Option<&str>,
        _fence_handler: RutabagaFenceHandler,
    ) -> RutabagaResult<Box<dyn RutabagaContext>> {
        let mut name: &str = "gpu_renderer";
        if let Some(name_string) = context_name.filter(|s| !s.is_empty()) {
            name = name_string;
        }

        // SAFETY:
        // Safe because virglrenderer is initialized by now and the context name is statically
        // allocated. The return value is checked before returning a new context.
        let ret = unsafe {
            match context_init {
                0 => virgl_renderer_context_create(
                    ctx_id,
                    name.len() as u32,
                    name.as_ptr() as *const c_char,
                ),
                _ => virgl_renderer_context_create_with_flags(
                    ctx_id,
                    context_init,
                    name.len() as u32,
                    name.as_ptr() as *const c_char,
                ),
            }
        };
        ret_to_res(ret)?;
        Ok(Box::new(VirglRendererContext { ctx_id }))
    }
}
