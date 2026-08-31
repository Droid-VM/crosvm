// Copyright 2017 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! GPU related things
//! depends on "gpu" feature
static_assertions::assert_cfg!(feature = "gpu");

use std::collections::HashMap;
use std::env;
use std::path::PathBuf;

use base::linux::move_proc_to_cgroup;
use jail::*;
use serde::Deserialize;
use serde::Serialize;
use serde_keyvalue::FromKeyValues;

use super::*;
use crate::crosvm::config::Config;
#[cfg(any(feature = "vnc", feature = "android_display"))]
use crate::crosvm::config::DisplayScreen;
#[cfg(feature = "vnc")]
use crate::crosvm::config::DEFAULT_VNC_HOST;
#[cfg(feature = "vnc")]
use crate::crosvm::config::DEFAULT_VNC_PORT;

pub struct GpuCacheInfo<'a> {
    directory: Option<&'a str>,
    environment: Vec<(&'a str, &'a str)>,
}

pub fn get_gpu_cache_info<'a>(
    cache_dir: Option<&'a String>,
    cache_size: Option<&'a String>,
    foz_db_list_path: Option<&'a String>,
    sandbox: bool,
) -> GpuCacheInfo<'a> {
    let mut dir = None;
    let mut env = Vec::new();

    // TODO (renatopereyra): Remove deprecated env vars once all src/third_party/mesa* are updated.
    if let Some(cache_dir) = cache_dir {
        if !Path::new(cache_dir).exists() {
            warn!("shader caching dir {} does not exist", cache_dir);
            // Deprecated in https://gitlab.freedesktop.org/mesa/mesa/-/merge_requests/15390
            env.push(("MESA_GLSL_CACHE_DISABLE", "true"));

            env.push(("MESA_SHADER_CACHE_DISABLE", "true"));
        } else if cfg!(any(target_arch = "arm", target_arch = "aarch64")) && sandbox {
            warn!("shader caching not yet supported on ARM with sandbox enabled");
            // Deprecated in https://gitlab.freedesktop.org/mesa/mesa/-/merge_requests/15390
            env.push(("MESA_GLSL_CACHE_DISABLE", "true"));

            env.push(("MESA_SHADER_CACHE_DISABLE", "true"));
        } else {
            dir = Some(cache_dir.as_str());

            // Deprecated in https://gitlab.freedesktop.org/mesa/mesa/-/merge_requests/15390
            env.push(("MESA_GLSL_CACHE_DISABLE", "false"));
            env.push(("MESA_GLSL_CACHE_DIR", cache_dir.as_str()));

            env.push(("MESA_SHADER_CACHE_DISABLE", "false"));
            env.push(("MESA_SHADER_CACHE_DIR", cache_dir.as_str()));

            env.push(("MESA_DISK_CACHE_DATABASE", "1"));

            if let Some(foz_db_list_path) = foz_db_list_path {
                env.push(("MESA_DISK_CACHE_COMBINE_RW_WITH_RO_FOZ", "1"));
                env.push((
                    "MESA_DISK_CACHE_READ_ONLY_FOZ_DBS_DYNAMIC_LIST",
                    foz_db_list_path,
                ));
            }

            if let Some(cache_size) = cache_size {
                // Deprecated in https://gitlab.freedesktop.org/mesa/mesa/-/merge_requests/15390
                env.push(("MESA_GLSL_CACHE_MAX_SIZE", cache_size.as_str()));

                env.push(("MESA_SHADER_CACHE_MAX_SIZE", cache_size.as_str()));
            }
        }
    }

    GpuCacheInfo {
        directory: dir,
        environment: env,
    }
}

/// `vnc_input` carries what a VNC exporter bound to `gpu-0` injects into: this screen's own
/// absolute pointer and keyboard. Built by the caller, because the virtio-input devices behind them
/// go into the same device list as this one and only the caller holds it. Empty on every other
/// configuration -- no VNC binding on this screen, a `view-only=true` one, or a native exporter.
pub fn create_gpu_device(
    cfg: &Config,
    exit_evt_wrtube: &SendTube,
    gpu_control_tube: Tube,
    resource_bridges: Vec<Tube>,
    render_server_fd: Option<SafeDescriptor>,
    has_vfio_gfx_device: bool,
    event_devices: Vec<EventDevice>,
    #[cfg(feature = "vnc")] vnc_input: gpu_display::VncBindingInput,
) -> DeviceResult {
    let is_sandboxed = cfg.jail_config.is_some();
    let mut gpu_params = cfg.gpu_parameters.clone().unwrap();

    if is_sandboxed {
        gpu_params.snapshot_scratch_path = Some(Path::new("/tmpfs-gpu-snapshot").to_path_buf());
    }

    // DroidVM: plumb gfxstream's host-allocation knobs to the in-process renderer before the
    // GPU/jail process forks. They describe how gfxstream backs the fresh shmem it allocates and
    // hands to the host Vulkan driver; generic VM memory registration never reads them.
    //
    // Only when gfxstream is the renderer. One binary carries both backends and the drm2kgsl
    // native context runs through virglrenderer, where every name below is dead weight -- and
    // an inherited GFXSTREAM_* in a drm2kgsl process environment reads like a misconfiguration to
    // anyone debugging one.
    #[cfg(feature = "gfxstream")]
    if gpu_params.mode == devices::virtio::GpuMode::ModeGfxstream {
        // Presence is the dynamic-VRAM switch. A value of 0 is an unmetered configuration, not
        // "off"; absence is what keeps this renderer allocation policy disabled.
        let host_dynamic = !gpu_params.udmabuf && gpu_params.vram_limit.is_some();
        // All of these have a consumer only for gfxstream host-alloc with dynamic VRAM enabled.
        if host_dynamic {
            env::set_var(
                "GFXSTREAM_POOL_BLOB_MAX_KB",
                gpu_params.pool_blob_max_kb.unwrap_or(4096).to_string(),
            );
            let limit_mb = match gpu_params.vram_limit {
                Some(n) if n > 0 => n.to_string(),
                // Explicitly unlimited: use a sentinel above any emulated heap. gfxstream still
                // clamps the value it reports to the guest heap size.
                _ => "1048576".to_string(), // 1 TiB
            };
            env::set_var("GFXSTREAM_VRAM_LIMIT_MB", &limit_mb);
            env::set_var("GFXSTREAM_VRAM_BUDGET_MB", limit_mb);
            env::set_var(
                "GFXSTREAM_VRAM_FOLIO_THRESHOLD_KB",
                gpu_params.vram_folio_threshold_kb.unwrap_or(1024).to_string(),
            );
            env::set_var(
                "GFXSTREAM_VRAM_EXCEED_POLICY",
                match gpu_params.vram_exceed_policy {
                    Some(devices::virtio::gpu::VramExceedPolicy::Oom) => "oom",
                    _ => "fallback",
                },
            );
        } else {
            // Do not let a launcher environment opt guest-alloc or static host-alloc into a
            // renderer policy that belongs exclusively to dynamic host-alloc.
            for name in [
                "GFXSTREAM_POOL_BLOB_MAX_KB",
                "GFXSTREAM_VRAM_LIMIT_MB",
                "GFXSTREAM_VRAM_BUDGET_MB",
                "GFXSTREAM_VRAM_FOLIO_THRESHOLD_KB",
                "GFXSTREAM_VRAM_EXCEED_POLICY",
            ] {
                env::remove_var(name);
            }
        }
        // Guest-alloc pool partition: the host-owned slice serving all gfx host-alloc requests
        // (ASG rings etc.). Only meaningful with udmabuf=true; consumed by gfxstream in the
        // pVM guest-alloc mode (stage 2).
        if gpu_params.udmabuf {
            env::set_var(
                "GFXSTREAM_POOL_HOST_MB",
                gpu_params.gfx_host_pre_alloc_mb.unwrap_or(64).to_string(),
            );
        }
        if gpu_params.gunyah_pvm == Some(true) {
            // Gunyah SHARE mappings are permanent and cannot be re-pointed, so the RingBlob
            // backing must be pinned (never freed/recycled). Only Qualcomm/Gunyah needs this.
            env::set_var("GFXSTREAM_GUNYAH_PIN_RINGBLOB", "1");
        }
    }

    if gpu_params.fixed_blob_mapping {
        if has_vfio_gfx_device {
            // TODO(b/323368701): make fixed_blob_mapping compatible with vfio dma_buf mapping for
            // GPU pci passthrough.
            debug!("gpu fixed blob mapping disabled: not compatible with passthrough GPU.");
            gpu_params.fixed_blob_mapping = false;
        } else if cfg!(feature = "vulkano") {
            // TODO(b/244591751): make fixed_blob_mapping compatible with vulkano for opaque_fd blob
            // mapping.
            debug!("gpu fixed blob mapping disabled: not compatible with vulkano");
            gpu_params.fixed_blob_mapping = false;
        }
    }

    // external_blob must be enforced to ensure that a blob can be exported to a mappable descriptor
    // (dma_buf, shmem, ...), since:
    //   - is_sandboxed implies that blob mapping will be done out-of-process by the crosvm
    //     hypervisor process.
    //   - fixed_blob_mapping is not yet compatible with VmMemorySource::ExternalMapping
    //   - udmabuf (guest-alloc): the guest owns the pool and hands the host an external descriptor
    //     per blob. gfxstream's guest-handle blob path (VirtioGpuResource::create, the
    //     STREAM_BLOB_MEM_GUEST|CREATE_GUEST_HANDLE branch) is gated on ExternalBlob; with it off,
    //     ResourceCreateBlob falls through to no-premapped-external-blob-mapping and returns
    //     ComponentError(-22), so guest Vulkan can't allocate device memory. Under --disable-sandbox
    //     with the gfxstream feature (fixed_blob_mapping=false) that gate was always off, which is
    //     what broke app-driven gfxstream guest-alloc. drm2kgsl is unaffected: its guest pool goes
    //     through virglrenderer's own path, not the gfxstream ExternalBlob one.
    gpu_params.external_blob = is_sandboxed || gpu_params.fixed_blob_mapping || gpu_params.udmabuf;

    // Implicit launch is not allowed when sandboxed. A socket fd from a separate sandboxed
    // render_server process must be provided instead.
    gpu_params.allow_implicit_render_server_exec =
        gpu_params.allow_implicit_render_server_exec && !is_sandboxed;

    let mut display_backends = vec![
        virtio::DisplayBackend::X(cfg.x_display.clone()),
        virtio::DisplayBackend::Stub,
    ];

    // This device provides one screen, `gpu-0`, so it takes the one exporter bound to that screen
    // and nothing else. An exporter bound to `simplefb` belongs to the simplefb device's screen,
    // which presents on its own display and is no part of this.
    //
    // `display_backends` remains an ordered try-in-turn list, but the two entries that used to
    // race for the front of it can no longer both be here: config rejects two exporters on one
    // screen, so at most one of the two inserts below runs. The list is a fallback chain again
    // (Wayland/X/Stub), not a silent winner-takes-all -- which is what it was when
    // `insert(0, Android)` followed by `insert(0, VncTcp)` put VNC in front and left
    // `AServiceManager_addService` uncalled, so the app's native display waited on a binder that
    // was never registered.
    //
    // The exporter also carries this binding's transport ceiling, collected alongside the backend
    // it belongs to. At most one of the two arms below runs (config rejects two exporters on one
    // screen), so there is exactly one answer here, and no binding at all means the default: take
    // whatever gets negotiated.
    #[cfg(any(feature = "vnc", feature = "android_display"))]
    let mut transport_cap = crate::crosvm::config::TransportCap::Auto;

    #[cfg(feature = "android_display")]
    if let Some(service) = cfg.android_display_service_for(DisplayScreen::Gpu0) {
        display_backends.insert(0, virtio::DisplayBackend::Android(service.name.clone()));
        transport_cap = service.transport_cap;
    }

    #[cfg(feature = "vnc")]
    if let Some(vnc_cfg) = cfg.vnc_server_for(DisplayScreen::Gpu0) {
        let host = vnc_cfg.host.as_deref().unwrap_or(DEFAULT_VNC_HOST);
        let port = vnc_cfg.port.unwrap_or(DEFAULT_VNC_PORT);
        let addr = format!("{}:{}", host, port);
        let (w, h) = cfg
            .display_input_width
            .zip(cfg.display_input_height)
            .unwrap_or((1280, 720));
        display_backends.insert(
            0,
            virtio::DisplayBackend::VncTcp {
                addr,
                width: w,
                height: h,
                password: vnc_cfg.password.clone(),
                // Resolved from the ceiling here rather than inside the sink, because the ceiling
                // belongs to the binding and one sink serves several of them. A binding capped
                // below `gpu-hw` builds no broker at all, so a client that asks for encoding 50
                // there is served pixels and told nothing -- which is what an old server looks
                // like, and what the app is written to fall back from.
                hw_encode: vnc_cfg.h264_enabled(),
                // This screen's own devices, parked for whichever `build` call opens the sink.
                vnc_input: std::sync::Arc::new(sync::Mutex::new(vnc_input)),
            },
        );
        transport_cap = vnc_cfg.transport_cap;
    }

    // Use the unnamed socket for GPU display screens.
    if let Some(socket_path) = cfg.wayland_socket_paths.get("") {
        display_backends.insert(
            0,
            virtio::DisplayBackend::Wayland(Some(socket_path.to_owned())),
        );
    }

    #[allow(unused_mut)]
    let mut dev = virtio::Gpu::new(
        exit_evt_wrtube
            .try_clone()
            .context("failed to clone tube")?,
        gpu_control_tube,
        resource_bridges,
        display_backends,
        &gpu_params,
        render_server_fd,
        event_devices,
        virtio::base_features(cfg.protection_type),
        &cfg.wayland_socket_paths,
        cfg.gpu_cgroup_path.as_ref(),
    );
    #[cfg(any(feature = "vnc", feature = "android_display"))]
    if !transport_cap.allows_gpu_copy() {
        dev.cap_transport_to_cpu();
    }

    let jail = if let Some(jail_config) = cfg.jail_config.as_ref() {
        let mut config = SandboxConfig::new(jail_config, "gpu_device");
        config.bind_mounts = true;
        // Allow changes made externally take effect immediately to allow shaders to be dynamically
        // added by external processes.
        config.remount_mode = Some(libc::MS_SLAVE);
        let mut jail = create_gpu_minijail(
            &jail_config.pivot_root,
            &config,
            /* render_node_only= */ false,
            gpu_params.snapshot_scratch_path.as_deref(),
        )?;

        // Prepare GPU shader disk cache directory.
        let cache_info = get_gpu_cache_info(
            gpu_params.cache_path.as_ref(),
            gpu_params.cache_size.as_ref(),
            None,
            cfg.jail_config.is_some(),
        );

        if let Some(dir) = cache_info.directory {
            // Manually bind mount recursively to allow DLC shader caches
            // to be propagated to the GPU process.
            jail.mount(dir, dir, "", (libc::MS_BIND | libc::MS_REC) as usize)?;
        }
        for (key, val) in cache_info.environment {
            env::set_var(key, val);
        }

        // Bind mount the wayland socket's directory into jail's root. This is necessary since
        // each new wayland context must open() the socket. If the wayland socket is ever
        // destroyed and remade in the same host directory, new connections will be possible
        // without restarting the wayland device.
        for socket_path in cfg.wayland_socket_paths.values() {
            let dir = socket_path.parent().with_context(|| {
                format!(
                    "wayland socket path '{}' has no parent",
                    socket_path.display(),
                )
            })?;
            jail.mount(dir, dir, "", (libc::MS_BIND | libc::MS_REC) as usize)?;
        }

        Some(jail)
    } else {
        None
    };

    Ok(VirtioDeviceStub {
        dev: Box::new(dev),
        jail,
    })
}

#[derive(Debug, Deserialize, Serialize, FromKeyValues, PartialEq, Eq)]
#[serde(deny_unknown_fields, rename_all = "kebab-case")]
pub struct GpuRenderServerParameters {
    pub path: PathBuf,
    pub cache_path: Option<String>,
    pub cache_size: Option<String>,
    pub foz_db_list_path: Option<String>,
    pub precompiled_cache_path: Option<String>,
    pub ld_preload_path: Option<String>,
}

fn get_gpu_render_server_environment(
    cache_info: Option<&GpuCacheInfo>,
    ld_preload_path: Option<&String>,
) -> Result<Vec<String>> {
    let mut env = HashMap::<String, String>::new();
    let os_env_len = env::vars_os().count();

    if let Some(cache_info) = cache_info {
        env.reserve(os_env_len + cache_info.environment.len());
        for (key, val) in cache_info.environment.iter() {
            env.insert(key.to_string(), val.to_string());
        }
    } else {
        env.reserve(os_env_len);
    }

    for (key_os, val_os) in env::vars_os() {
        // minijail should accept OsStr rather than str...
        let into_string_err = |_| anyhow!("invalid environment key/val");
        let key = key_os.into_string().map_err(into_string_err)?;
        let val = val_os.into_string().map_err(into_string_err)?;
        env.entry(key).or_insert(val);
    }

    // for debugging purpose, avoid appending if LD_PRELOAD has been set outside
    if !env.contains_key("LD_PRELOAD") {
        if let Some(ld_preload_path) = ld_preload_path {
            env.insert("LD_PRELOAD".to_string(), ld_preload_path.to_string());
        }
    }

    // TODO(b/323284290): workaround to advertise 2 graphics queues in ANV
    if !env.contains_key("ANV_QUEUE_OVERRIDE") {
        env.insert("ANV_QUEUE_OVERRIDE".to_string(), "gc=2".to_string());
    }

    // TODO(b/237493180, b/284517235): workaround to enable ETC2/ASTC format emulation in Mesa
    // TODO(b/284361281, b/328827736): workaround to enable legacy sparse binding in RADV
    let driconf_options = [
        "radv_legacy_sparse_binding",
        "radv_require_etc2",
        "vk_require_etc2",
        "vk_require_astc",
    ];
    for opt in driconf_options {
        if !env.contains_key(opt) {
            env.insert(opt.to_string(), "true".to_string());
        }
    }

    // TODO(b/339766043): workaround to disable Vulkan protected memory feature in Mali
    if !env.contains_key("MALI_BASE_PROTECTED_MEMORY_HEAP_SIZE") {
        env.insert(
            "MALI_BASE_PROTECTED_MEMORY_HEAP_SIZE".to_string(),
            "0".to_string(),
        );
    }

    Ok(env.iter().map(|(k, v)| format!("{}={}", k, v)).collect())
}

pub fn start_gpu_render_server(
    cfg: &Config,
    render_server_parameters: &GpuRenderServerParameters,
) -> Result<(Minijail, SafeDescriptor)> {
    let (server_socket, client_socket) =
        UnixSeqpacket::pair().context("failed to create render server socket")?;

    let (jail, cache_info) = if let Some(jail_config) = cfg.jail_config.as_ref() {
        let mut config = SandboxConfig::new(jail_config, "gpu_render_server");
        // Allow changes made externally take effect immediately to allow shaders to be dynamically
        // added by external processes.
        config.remount_mode = Some(libc::MS_SLAVE);
        config.bind_mounts = true;
        // Run as root in the jail to keep capabilities after execve, which is needed for
        // mounting to work.  All capabilities will be dropped afterwards.
        config.run_as = RunAsUser::Root;
        let mut jail = create_gpu_minijail(
            &jail_config.pivot_root,
            &config,
            /* render_node_only= */ true,
            /* snapshot_scratch_path= */ None,
        )?;

        let cache_info = get_gpu_cache_info(
            render_server_parameters.cache_path.as_ref(),
            render_server_parameters.cache_size.as_ref(),
            render_server_parameters.foz_db_list_path.as_ref(),
            true,
        );

        if let Some(dir) = cache_info.directory {
            // Manually bind mount recursively to allow DLC shader caches
            // to be propagated to the GPU process.
            jail.mount(dir, dir, "", (libc::MS_BIND | libc::MS_REC) as usize)?;
        }
        if let Some(precompiled_cache_dir) = &render_server_parameters.precompiled_cache_path {
            jail.mount_bind(precompiled_cache_dir, precompiled_cache_dir, true)?;
        }

        // bind mount /dev/log for syslog
        let log_path = Path::new("/dev/log");
        if log_path.exists() {
            jail.mount_bind(log_path, log_path, true)?;
        }

        (jail, Some(cache_info))
    } else {
        (
            create_default_minijail().context("failed to create jail")?,
            None,
        )
    };

    let inheritable_fds = [
        server_socket.as_raw_descriptor(),
        libc::STDOUT_FILENO,
        libc::STDERR_FILENO,
    ];

    let cmd = &render_server_parameters.path;
    let cmd_str = cmd
        .to_str()
        .ok_or_else(|| anyhow!("invalid render server path"))?;
    let fd_str = server_socket.as_raw_descriptor().to_string();
    let args = [cmd_str, "--socket-fd", &fd_str];

    let env = Some(get_gpu_render_server_environment(
        cache_info.as_ref(),
        render_server_parameters.ld_preload_path.as_ref(),
    )?);
    let mut envp: Option<Vec<&str>> = None;
    if let Some(ref env) = env {
        envp = Some(env.iter().map(AsRef::as_ref).collect());
    }

    let render_server_pid = jail
        .run_command(minijail::Command::new_for_path(
            cmd,
            &inheritable_fds,
            &args,
            envp.as_deref(),
        )?)
        .context("failed to start gpu render server")?;

    if let Some(gpu_server_cgroup_path) = &cfg.gpu_server_cgroup_path {
        move_proc_to_cgroup(gpu_server_cgroup_path.to_path_buf(), render_server_pid)?;
    }

    Ok((jail, SafeDescriptor::from(client_socket)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crosvm::config::from_key_values;

    #[test]
    fn parse_gpu_render_server_parameters() {
        let res: GpuRenderServerParameters = from_key_values("path=/some/path").unwrap();
        assert_eq!(
            res,
            GpuRenderServerParameters {
                path: "/some/path".into(),
                cache_path: None,
                cache_size: None,
                foz_db_list_path: None,
                precompiled_cache_path: None,
                ld_preload_path: None,
            }
        );

        let res: GpuRenderServerParameters = from_key_values("/some/path").unwrap();
        assert_eq!(
            res,
            GpuRenderServerParameters {
                path: "/some/path".into(),
                cache_path: None,
                cache_size: None,
                foz_db_list_path: None,
                precompiled_cache_path: None,
                ld_preload_path: None,
            }
        );

        let res: GpuRenderServerParameters =
            from_key_values("path=/some/path,cache-path=/cache/path,cache-size=16M").unwrap();
        assert_eq!(
            res,
            GpuRenderServerParameters {
                path: "/some/path".into(),
                cache_path: Some("/cache/path".into()),
                cache_size: Some("16M".into()),
                foz_db_list_path: None,
                precompiled_cache_path: None,
                ld_preload_path: None,
            }
        );

        let res: GpuRenderServerParameters = from_key_values(
            "path=/some/path,cache-path=/cache/path,cache-size=16M,foz-db-list-path=/db/list/path,precompiled-cache-path=/precompiled/path",
        )
        .unwrap();
        assert_eq!(
            res,
            GpuRenderServerParameters {
                path: "/some/path".into(),
                cache_path: Some("/cache/path".into()),
                cache_size: Some("16M".into()),
                foz_db_list_path: Some("/db/list/path".into()),
                precompiled_cache_path: Some("/precompiled/path".into()),
                ld_preload_path: None,
            }
        );

        let res: GpuRenderServerParameters =
            from_key_values("path=/some/path,ld-preload-path=/ld/preload/path").unwrap();
        assert_eq!(
            res,
            GpuRenderServerParameters {
                path: "/some/path".into(),
                cache_path: None,
                cache_size: None,
                foz_db_list_path: None,
                precompiled_cache_path: None,
                ld_preload_path: Some("/ld/preload/path".into()),
            }
        );

        let res =
            from_key_values::<GpuRenderServerParameters>("cache-path=/cache/path,cache-size=16M");
        assert!(res.is_err());

        let res = from_key_values::<GpuRenderServerParameters>("");
        assert!(res.is_err());
    }
}
