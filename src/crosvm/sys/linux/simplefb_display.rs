// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

use std::thread;
use std::time::Duration;
use std::time::Instant;

use anyhow::Context;
use anyhow::Result;
use base::error;
use base::info;
use base::AsRawDescriptor;
use base::SafeDescriptor;
use base::WaitContext;
use gpu_display::Damage;
use gpu_display::GpuDisplay;
use gpu_display::GpuDisplayExt;
use gpu_display::PresentOutcome;
use gpu_display::ScanoutFrame;
use gpu_display::SurfaceType;
use vm_control::gpu::DisplayMode;
use vm_control::gpu::DisplayParameters;
use vm_memory::FramebufferPrep;
use vm_memory::GuestAddress;
use vm_memory::GuestMemory;

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
/// B,G,R,A. Naming it lets the CPU edge compare the source with each sink's real layout, and lets
/// the GPU path pick its VkFormat from the same fact instead of assuming one byte order.
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

pub enum SimplefbDisplayTarget {
    Vnc {
        addr: String,
        password: Option<String>,
    },
    /// The Android Surface the app hands over through the display service binder. Input does
    /// NOT come through the display here -- it arrives on the `--input` evdev sockets, same as
    /// the virtio-gpu native-display path.
    Android { service_name: String },
}

const DEFAULT_FPS: u32 = 30;

pub fn start_simplefb_display_thread(
    guest_mem: GuestMemory,
    params: SimplefbDisplayParams,
    target: SimplefbDisplayTarget,
) -> Result<thread::JoinHandle<()>> {
    thread::Builder::new()
        .name("simplefb_display".into())
        .spawn(move || {
                SimplefbDisplayTarget::Vnc {
                    addr,
                    password,
                } => GpuDisplay::open_vnc_tcp(
                ),
                SimplefbDisplayTarget::Android { service_name } => {
                }
            };
            let display_result = GpuDisplay::open_vnc_tcp(
                &target.addr,
                params.width,
                params.height,
                target.password.clone(),
            );
            let mut display = match display_result {
                Ok(d) => d,
                Err(e) => {
                    error!("simplefb: failed to open display: {:?}", e);
                    return;
                }
            };

            // Import input event devices so VNC input is routed to guest.
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

    // Whether the pages under this region are somewhere the host can leave them, as established by
    // the backend that laid the region out (see `vm_memory::FramebufferPrep`). This is the one
    // question that has to be asked before the udmabuf below and not after: the udmabuf takes a
    // page reference on every page of the region, a referenced page cannot be migrated, and if any
    // of those pages is in CMA the guest's first touch of the framebuffer kills the vcpu --
    // `page fault at <gpa>, attempt: -12`, because gunyah's pin has nowhere to move it to. There is
    // no recovery from that and no way to notice it from here; the failure lands on the guest.
    //
    // Anything short of a positive answer means the CPU path, including "nobody said". A partial
    // answer would be the same lottery with better odds, and the floor -- copying the bytes -- costs
    // a memcpy per frame and cannot fail this way at all.
    match guest_mem.framebuffer_prep() {
        FramebufferPrep::PoolBacked => {}
        FramebufferPrep::NotPoolBacked(why) => {
            return Err(format!("framebuffer pages not pool-backed: {why}"));
        }
        FramebufferPrep::Unclaimed => {
            return Err("framebuffer pages not pool-backed: no hypervisor backend prepared the \
                        region, so nothing can say the guest's first fault will find a page it \
                        can pin"
                .to_string());
        }
    }

    // THE CROSSING (plan §4.4). Both transports now carry this source fourcc explicitly: the CPU
    // edge compares it with the sink framebuffer, while the GPU sink picks a VkFormat from it
    // (`vkFormatFromDrmFourcc`). The device tree's default `a8r8g8b8` is AR24, so declaring it lands
    // on B8G8R8A8_UNORM. Declare it wrong and every
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
                    PresentOutcome::NoFramebuffer => {
                    }
                }
        if let Some(fb) = display.framebuffer(surface_id) {
            let dst = fb.as_volatile_slice();
            let copy_len = dst.size().min(read_buf.len());
            dst.copy_from(&read_buf[..copy_len]);
        }
        display.flip(surface_id);

        let elapsed = frame_start.elapsed();
        if elapsed < frame_duration {
            thread::sleep(frame_duration - elapsed);
        }
    }

    display.release_surface(surface_id);
    Ok(())
}
