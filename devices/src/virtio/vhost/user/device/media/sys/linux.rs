// Copyright 2026 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use std::sync::mpsc;
use std::sync::mpsc::RecvTimeoutError;
use std::thread;
use std::time::Duration;

use anyhow::bail;
use anyhow::Context;
use argh::FromArgs;
use base::error;
use base::RawDescriptor;
use cros_async::Executor;

use crate::virtio::media::android_camera_backend::AndroidCameraBackend;
use crate::virtio::media::android_codec_backend::MediaCodecDecoderBackend;
use crate::virtio::media::camera_config;
use crate::virtio::media::decoder_config;
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
    // its own instantiation. Which kinds exist is `MediaDeviceKind::support`'s table, the same
    // one the VMM reads before it launches this process, and the refusal below is that table's
    // wording -- so both sides refuse the same kinds with the same words (B4 §7.1, D15).
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
            // opened: the camera is taken at STREAMON, on the stream's own thread. The wait
            // for it is bounded, because nothing else bounds the VMM's handshake (R4).
            let camera = enumerate_camera(params.camera_id.clone())?;
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
        MediaDeviceKind::Decoder => {
            use virtio_media::devices::VideoDecoder;

            // As the camera: the codec store is read here, once, before the frontend is spoken
            // to, so a platform with no usable hardware decoder (or no media NDK) ends the
            // helper now, by name; the codec itself is created at STREAMON(OUTPUT). The store's
            // tables have no lock, which is why this is the one walk, on one thread, after the
            // uid drop (design §7.2). The wait is bounded for the same reason the camera's is.
            let allow_sw = params.allow_sw;
            let decoder = enumerate_on_thread(
                "media_codec_enum",
                "codec enumeration",
                "the codec store (media.player) is not responding to this uid",
                move || MediaCodecDecoderBackend::new(allow_sw),
            )?;
            let card = params
                .card
                .clone()
                .unwrap_or_else(|| "droidvm decoder".to_string());
            let backend = MediaBackend::new(
                params,
                decoder_config(&card),
                move |event_queue, guest_mapper, mapper, allocator| {
                    Ok(VideoDecoder::new(
                        decoder.clone(),
                        event_queue,
                        guest_mapper,
                        mapper,
                        allocator,
                    ))
                },
            )?;
            ex.run_until(conn.run_backend(backend, &ex))?
        }
        kind @ MediaDeviceKind::Encoder => {
            bail!("{}", kind.unimplemented_message())
        }
    }
}

/// How long the helper waits for the camera enumeration. `list_cameras` is binder work
/// (`ACameraManager_getCameraIdList` plus one `getCameraCharacteristics` per camera) with no
/// timeout of its own, and it runs before the vhost-user handshake is answered, so a
/// `cameraserver` that never replies would hold the VMM inside `VhostUserFrontend::new` and the
/// VM would never start (`review-m4` R4). Longer than the frontend's own `open_stream` bound: an
/// enumeration is once per VM and a cold camera service is slow the first time. The codec
/// store's enumeration (`AMediaCodecStore_*`, fetched from `media.player`) is bounded by the
/// same value for the same reasons.
const ENUMERATION_TIMEOUT: Duration = Duration::from_secs(15);

/// Describe the camera on a thread of its own, so the wait for it is bounded.
fn enumerate_camera(camera_id: Option<String>) -> anyhow::Result<AndroidCameraBackend> {
    enumerate_on_thread(
        "media_camera_enum",
        "camera enumeration",
        "the camera service is not responding to this uid",
        move || AndroidCameraBackend::new(camera_id.as_deref()),
    )
}

/// Run `describe` on a thread called `thread_name` and wait at most [`ENUMERATION_TIMEOUT`] for
/// what it returns.
///
/// On a timeout the helper exits non-zero and the VMM reports it -- `child media helper (pid N)
/// exited` (`logs/vpu_wp/M3.md` §5.1), plus this line on the helper's inherited stderr -- which is
/// the whole point: a helper that cannot describe its device must fail visibly instead of leaving
/// the VMM blocked on a handshake it will never answer. The enumeration thread is not joined; it
/// may still be inside binder, and the process is on its way out.
fn enumerate_on_thread<T: Send + 'static>(
    thread_name: &'static str,
    what: &'static str,
    why: &'static str,
    describe: impl FnOnce() -> anyhow::Result<T> + Send + 'static,
) -> anyhow::Result<T> {
    let (tx, rx) = mpsc::sync_channel(1);
    thread::Builder::new()
        .name(thread_name.to_string())
        .spawn(move || {
            let _ = tx.send(describe());
        })
        .with_context(|| format!("cannot start the {what} thread"))?;

    match rx.recv_timeout(ENUMERATION_TIMEOUT) {
        Ok(described) => described,
        Err(RecvTimeoutError::Timeout) => {
            let secs = ENUMERATION_TIMEOUT.as_secs();
            error!(
                "{} did not answer within {} s: {}; the media helper exits so the VM fails to \
                 start instead of waiting for a device that will never be built",
                what, secs, why
            );
            bail!("{} did not answer within {} s", what, secs)
        }
        Err(RecvTimeoutError::Disconnected) => {
            bail!("the {} thread ended without an answer", what)
        }
    }
}
