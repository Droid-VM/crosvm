// Copyright 2026 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use anyhow::bail;
use anyhow::Context;
use argh::FromArgs;
use base::RawDescriptor;
use cros_async::Executor;

use crate::virtio::media::android_camera_backend::AndroidCameraBackend;
use crate::virtio::media::camera_config;
use crate::virtio::media::loopback_config;
use crate::virtio::media::simple_capture_config;
use crate::virtio::media::MediaDeviceKind;
use crate::virtio::vhost::user::device::media::MediaBackend;
use crate::virtio::vhost::user::device::media::MediaBackendParams;
use crate::virtio::vhost::user::device::BackendConnection;

#[derive(FromArgs)]
#[argh(subcommand, name = "media")]
/// virtio-media device
pub struct Options {
    #[argh(option, arg_name = "PATH", hidden_help)]
    /// deprecated - please use --socket-path instead
    socket: Option<String>,
    #[argh(option, arg_name = "PATH")]
    /// path to the vhost-user socket to bind to.
    /// If this flag is set, --fd cannot be specified.
    socket_path: Option<String>,
    #[argh(option, arg_name = "FD")]
    /// file descriptor of a connected vhost-user socket.
    /// If this flag is set, --socket-path cannot be specified.
    fd: Option<RawDescriptor>,
    #[argh(option, arg_name = "JSON")]
    /// JSON-encoded parameters, as crosvm writes them when it spawns this backend itself: the
    /// device kind and its options, the guest-physical base of the media_host pool, and the
    /// guest-physical windows the host may touch. There is no key=value form -- a backend
    /// crosvm launched does not need a human-writable command line.
    config_json: String,
}

/// Starts a vhost-user media device.
/// Returns an error if the given `args` is invalid or the device fails to run.
pub fn run_media_device(opts: Options) -> anyhow::Result<()> {
    let params: MediaBackendParams =
        serde_json::from_str(&opts.config_json).context("failed to parse --config-json")?;
    let ex = Executor::new().context("Failed to create executor")?;
    let conn =
        BackendConnection::from_opts(opts.socket.as_deref(), opts.socket_path.as_deref(), opts.fd)?;

    // One arm per device type: the backend is generic over the device it runs, so each kind is
    // its own instantiation. The list is the in-VMM one (`create_virtio_media_device`) and must
    // refuse the same kinds it does, by name.
    match params.kind {
        MediaDeviceKind::Simple => {
            use virtio_media::devices::SimpleCaptureDevice;

            let backend = MediaBackend::new(
                params,
                simple_capture_config(),
                |event_queue, guest_mapper, mapper, allocator| {
                    Ok(SimpleCaptureDevice::new(
                        event_queue,
                        guest_mapper,
                        mapper,
                        allocator,
                    ))
                },
            )?;
            ex.run_until(conn.run_backend(backend, &ex))?
        }
        MediaDeviceKind::Loopback => {
            use virtio_media::devices::LoopbackDevice;

            let card = params.card.clone();
            let backend = MediaBackend::new(
                params,
                loopback_config(card.as_deref().unwrap_or("droidvm loopback")),
                |event_queue, guest_mapper, mapper, allocator| {
                    Ok(LoopbackDevice::new(
                        event_queue,
                        guest_mapper,
                        mapper,
                        allocator,
                    ))
                },
            )?;
            ex.run_until(conn.run_backend(backend, &ex))?
        }
        MediaDeviceKind::Camera => {
            use virtio_media::devices::camera::CameraBackend;
            use virtio_media::devices::CameraDevice;

            // The camera is described here, once, before the frontend is spoken to: a camera
            // this uid cannot see (or no NDK at all) ends the helper now, which the VMM reports
            // by name, rather than leaving a device that never answers. The first NDK touch
            // is this call, and it happens after the uid drop -- this process was exec'd as
            // the app's uid -- so the binder pool it starts belongs to that uid. Nothing is
            // opened: the camera is taken at STREAMON, on the stream's own thread.
            let camera = AndroidCameraBackend::new(params.camera_id.as_deref())?;
            let card = params
                .card
                .clone()
                .unwrap_or_else(|| format!("camera {}", camera.info().id));
            let backend = MediaBackend::new(
                params,
                camera_config(&card),
                move |event_queue, guest_mapper, mapper, allocator| {
                    Ok(CameraDevice::new(
                        camera.clone(),
                        event_queue,
                        guest_mapper,
                        mapper,
                        allocator,
                    ))
                },
            )?;
            ex.run_until(conn.run_backend(backend, &ex))?
        }
        kind @ (MediaDeviceKind::Decoder | MediaDeviceKind::Encoder) => {
            bail!(
                "--virtio-media kind={:?} is not implemented yet (only simple, loopback and \
                 camera are)",
                kind
            )
        }
    }
}
