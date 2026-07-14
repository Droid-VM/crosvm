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
use gpu_display::EventDevice;
use gpu_display::GpuDisplay;
use gpu_display::GpuDisplayExt;
use gpu_display::SurfaceType;
use vm_control::gpu::DisplayMode;
use vm_control::gpu::DisplayParameters;
use vm_memory::GuestAddress;
use vm_memory::GuestMemory;

pub struct SimplefbDisplayParams {
    pub addr: u64,
    pub width: u32,
    pub height: u32,
    pub stride: u32,
    pub bpp: u32,
    pub size: u64,
}

pub struct VncDisplayTarget {
    pub addr: String,
    pub password: Option<String>,
}

const DEFAULT_FPS: u32 = 30;

pub fn start_simplefb_display_thread(
    guest_mem: GuestMemory,
    params: SimplefbDisplayParams,
    target: VncDisplayTarget,
    event_devices: Vec<EventDevice>,
    // Same `--vnc-server input=` interpretation as the virtio-gpu display path (see
    // vnc_touch_input): true = legacy multi-touch, false = absolute-mouse tablet.
    touch_input: bool,
) -> Result<thread::JoinHandle<()>> {
    thread::Builder::new()
        .name("simplefb_display".into())
        .spawn(move || {
            let display_result = GpuDisplay::open_vnc_tcp(
                &target.addr,
                params.width,
                params.height,
                target.password.clone(),
                touch_input,
            );
            let mut display = match display_result {
                Ok(d) => d,
                Err(e) => {
                    error!("simplefb: failed to open VNC display: {:?}", e);
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
