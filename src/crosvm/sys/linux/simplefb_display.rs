// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

use std::sync::Arc;
use std::thread;
use std::time::Duration;
use std::time::Instant;

use anyhow::Context;
use anyhow::Result;
use base::error;
use base::info;
use base::warn;
use base::AsRawDescriptor;
use base::SafeDescriptor;
use base::WaitContext;
use gpu_display::Damage;
use gpu_display::EventDevice;
use gpu_display::GpuDisplay;
use gpu_display::GpuDisplayExt;
use gpu_display::PresentOutcome;
use gpu_display::ScanoutFrame;
use gpu_display::SurfaceType;
use vm_control::gpu::DisplayMode;
use vm_control::gpu::DisplayParameters;
use vm_memory::GuestAddress;
use vm_memory::GuestMemory;

use devices::virtio::ExternalScanout;

pub struct SimplefbDisplayParams {
    pub addr: u64,
    pub width: u32,
    pub height: u32,
    pub stride: u32,
    pub bpp: u32,
    pub size: u64,
    /// DRM fourcc of the framebuffer, from the same DT `format` string the guest is handed.
    pub fourcc: u32,
}

const fn drm_fourcc(a: u8, b: u8, c: u8, d: u8) -> u32 {
    (a as u32) | ((b as u32) << 8) | ((c as u32) << 16) | ((d as u32) << 24)
}

/// The DRM fourcc named by a simplefb device-tree `format` string.
///
/// The DT string is the only statement anyone makes about this framebuffer's byte order, and the
/// host has been acting on it implicitly: the default `a8r8g8b8` is ARGB8888, which in memory is
/// B,G,R,A -- the CPU pipeline's canonical order -- which is precisely why the bridge's plain copy
/// has always produced the right colours without a swizzle anywhere on this route. Naming it turns
/// that from a coincidence nobody wrote down into a field a sink can read, which is what the GPU
/// path will need when it picks a VkFormat from the fourcc instead of assuming BGRX.
///
/// The fallback matches the one the bpp lookup uses on an unrecognised string, for the same reason:
/// `a8r8g8b8` is what the device tree defaults to when nobody says otherwise.
pub fn simplefb_format_fourcc(format: &str) -> u32 {
    match format {
        "x8r8g8b8" => drm_fourcc(b'X', b'R', b'2', b'4'),
        "a8b8g8r8" => drm_fourcc(b'A', b'B', b'2', b'4'),
        "r8g8b8" => drm_fourcc(b'R', b'G', b'2', b'4'),
        "r5g6b5" => drm_fourcc(b'R', b'G', b'1', b'6'),
        _ => drm_fourcc(b'A', b'R', b'2', b'4'),
    }
}

/// Where simplefb frames go. Everything past opening the display is
/// backend-agnostic: the bridge only uses framebuffer()/flip(), so any
/// GpuDisplay backend works.
pub enum SimplefbDisplayTarget {
    Vnc {
        addr: String,
        password: Option<String>,
        /// Same `--vnc-server input=` interpretation as the virtio-gpu display path (see
        /// vnc_touch_input): true = legacy multi-touch, false = absolute-mouse tablet.
        touch_input: bool,
    },
    /// The Android Surface the app hands over through the display service binder. Input does
    /// NOT come through the display here -- it arrives on the `--input` evdev sockets, same as
    /// the virtio-gpu native-display path.
    Android { service_name: String },
    /// The virtio-gpu device's own display. Used whenever this VM has a GPU: there is one
    /// Surface, so there must be one writer, and the device is it -- we hand frames over and it
    /// shows them while the guest is not displaying through virtio-gpu itself. See
    /// `devices::virtio::ExternalScanout`.
    GpuDevice { scanout: Arc<ExternalScanout> },
}

const DEFAULT_FPS: u32 = 30;

/// Feeds guest framebuffer frames to the GPU device, which owns the one display.
///
/// Skips the copy entirely while the guest is displaying through virtio-gpu (the device tells us
/// so), and only submits when the framebuffer actually changed -- a frozen firmware framebuffer
/// behind a live guest desktop must not keep waking the worker, and an unchanged frame is not a
/// reason to take the display away from anyone.
fn simplefb_feed_loop(
    guest_mem: GuestMemory,
    params: &SimplefbDisplayParams,
    scanout: Arc<ExternalScanout>,
) -> Result<()> {
    let frame_duration = Duration::from_nanos(1_000_000_000 / DEFAULT_FPS as u64);
    let guest_addr = GuestAddress(params.addr);
    let fb_size = (params.stride as usize) * (params.height as usize);
    let mut read_buf = vec![0u8; fb_size];
    let mut last_buf: Vec<u8> = Vec::new();

    info!(
        "simplefb: feeding the gpu display: {}x{} stride={} addr={:#x} @ {}fps",
        params.width, params.height, params.stride, params.addr, DEFAULT_FPS,
    );

    let mut idle_pokes: u32 = 0;
    loop {
        let frame_start = Instant::now();
        if scanout.guest_owns() {
            // Nothing to send while the guest is driving the display -- but ownership can also
            // lapse on a clock (a guest that bound a scanout and stopped presenting), and that is
            // only ever re-evaluated on the worker's side of this event. Poke it about once a
            // second so a lapse is noticed without copying a frame nobody will look at.
            idle_pokes += 1;
            if idle_pokes >= DEFAULT_FPS {
                idle_pokes = 0;
                scanout.poke();
            }
        } else {
            if idle_pokes != 0 {
                // Just took the display back. The framebuffer may not have changed a byte since
                // we last looked at it (a Windows guest that has been sitting on the same picture
                // since the firmware handed over), and "unchanged" must not mean "not shown" on
                // the frame where the display became ours.
                idle_pokes = 0;
                last_buf.clear();
            }
            if guest_mem
                .read_exact_at_addr(&mut read_buf, guest_addr)
                .is_err()
            {
                info!("simplefb: guest memory no longer readable, exiting");
                break;
            }
            if last_buf != read_buf {
                last_buf.clear();
                last_buf.extend_from_slice(&read_buf);
                scanout.submit(&read_buf);
            }
        }
        let elapsed = frame_start.elapsed();
        if elapsed < frame_duration {
            thread::sleep(frame_duration - elapsed);
        }
    }
    Ok(())
}

pub fn start_simplefb_display_thread(
    guest_mem: GuestMemory,
    params: SimplefbDisplayParams,
    target: SimplefbDisplayTarget,
    event_devices: Vec<EventDevice>,
) -> Result<thread::JoinHandle<()>> {
    thread::Builder::new()
        .name("simplefb_display".into())
        .spawn(move || {
            // Handing frames to the GPU device needs no display of our own -- it owns the one
            // Surface, so this thread is a producer, not a presenter.
            if let SimplefbDisplayTarget::GpuDevice { scanout } = &target {
                if let Err(e) = simplefb_feed_loop(guest_mem, &params, scanout.clone()) {
                    error!("simplefb feed thread exited with error: {:?}", e);
                }
                return;
            }

            let display_result = match &target {
                SimplefbDisplayTarget::Vnc {
                    addr,
                    password,
                    touch_input,
                } => GpuDisplay::open_vnc_tcp(
                    addr,
                    params.width,
                    params.height,
                    password.clone(),
                    *touch_input,
                ),
                SimplefbDisplayTarget::Android { service_name } => {
                    GpuDisplay::open_android(service_name)
                }
                // Handled above; it never opens a display.
                SimplefbDisplayTarget::GpuDevice { .. } => unreachable!(),
            };
            let mut display = match display_result {
                Ok(d) => d,
                Err(e) => {
                    error!("simplefb: failed to open display: {:?}", e);
                    return;
                }
            };

            // Routes VNC input to the guest. The Android backend has no input of its own (the
            // app drives the `--input` evdev sockets instead), so this is a no-op there.
            for ed in event_devices {
                if let Err(e) = display.import_event_device(ed) {
                    error!("simplefb: failed to import event device: {:?}", e);
                }
            }

            if let Err(e) = simplefb_display_loop(guest_mem, &params, &mut display) {
                error!("simplefb display thread exited with error: {:?}", e);
            }
        })
        .context("failed to spawn simplefb display thread")
}

fn simplefb_display_loop(
    guest_mem: GuestMemory,
    params: &SimplefbDisplayParams,
    display: &mut GpuDisplay,
) -> Result<()> {
    let display_params = DisplayParameters::default_with_mode(DisplayMode::Windowed(
        params.width,
        params.height,
    ));

    let surface_id = display
        .create_surface(None, None, &display_params, SurfaceType::Scanout)
        .context("failed to create display surface")?;

    let frame_duration = Duration::from_nanos(1_000_000_000 / DEFAULT_FPS as u64);
    let guest_addr = GuestAddress(params.addr);
    let fb_size = (params.stride as usize) * (params.height as usize);
    let mut read_buf = vec![0u8; fb_size];
    let mut no_framebuffer: u64 = 0;

    info!(
        "simplefb display bridge: {}x{} stride={} bpp={} addr={:#x} @ {}fps",
        params.width, params.height, params.stride, params.bpp, params.addr, DEFAULT_FPS,
    );

    loop {
        let frame_start = Instant::now();

        // Process any pending VNC input events and route to EventDevices.
        if let Err(e) = display.dispatch_events() {
            match e {
                gpu_display::GpuDisplayError::ConnectionBroken => {
                    info!("simplefb: display connection closed, exiting");
                    break;
                }
                gpu_display::GpuDisplayError::IoError(ref ioe)
                    if ioe.kind() == std::io::ErrorKind::WouldBlock =>
                {
                    // Nonblocking input/event sockets may transiently return EAGAIN.
                    // This is not fatal; just retry on the next frame.
                }
                _ => {
                    error!("simplefb: dispatch_events error: {:?}", e);
                    break;
                }
            }
        }

        // Nothing downstream is positioned to see this frame -- VNC with no client is the case
        // that exists today. Everything below is then work done for nobody: a full framebuffer
        // read out of guest memory, a copy into the surface, and whatever the sink does with it,
        // repeated DEFAULT_FPS times a second for as long as the VM is up. dispatch_events above
        // still runs, which is what notices a client arriving, so this recovers by itself on the
        // next iteration.
        //
        // Deliberately after dispatch_events and before the read: the read is the expensive part
        // and it is the first thing that can be skipped without losing the ability to come back.
        if !display.has_consumer() {
            let elapsed = frame_start.elapsed();
            if elapsed < frame_duration {
                thread::sleep(frame_duration - elapsed);
            }
            continue;
        }

        if guest_mem
            .read_exact_at_addr(&mut read_buf, guest_addr)
            .is_err()
        {
            info!("simplefb: guest memory no longer readable, exiting");
            break;
        }

        let frame = ScanoutFrame {
            bytes: &read_buf,
            stride: params.stride,
            width: params.width,
            height: params.height,
            fourcc: params.fourcc,
            damage: Damage::Full,
        };
        match display.present_frame(surface_id, &frame) {
            PresentOutcome::Flipped => no_framebuffer = 0,
            PresentOutcome::NoFramebuffer => {
                // No framebuffer to write into: the sink never gave us one (an Android display
                // whose service lost the name race hands out a surface with no window behind it)
                // or it is transiently locked. Silence here cost a whole debugging session -- the
                // bridge looked perfectly healthy while presenting nothing -- so say it, backing
                // off so a permanent condition does not fill the log.
                no_framebuffer += 1;
                if no_framebuffer == 1 || no_framebuffer % 300 == 0 {
                    warn!(
                        "simplefb: no framebuffer from the display sink ({} frame(s) dropped)",
                        no_framebuffer
                    );
                }
                // The flip on this path is not presenting anything -- it is what releases a sink
                // that handed back nothing, and it has been here since before the copy moved out.
                display.flip(surface_id);
            }
        }

        let elapsed = frame_start.elapsed();
        if elapsed < frame_duration {
            thread::sleep(frame_duration - elapsed);
        }
    }

    display.release_surface(surface_id);
    Ok(())
}
