// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

//! What feeds a motion-sensor device (new_motion): the host's own accelerometer and gyroscope, and
//! an outside writer -- the host app that shows the guest, say -- whose events override them for as
//! long as they keep coming.
//!
//! The host's sensors report in the host device's natural frame, which is the guest's whenever
//! nobody presents the guest turned. A writer that turns the guest's picture, or wants the guest to
//! see something else entirely, sends its own values instead. It has to keep sending: an override
//! lapses OVERRIDE_HOLD after its last frame, and when the writer stops, or closes its connection,
//! the host's sensors take over again. So a guest keeps real motion data when nothing is showing it.

use std::io::ErrorKind;
use std::io::Read;
use std::io::Write;
use std::os::unix::net::UnixStream;
use std::sync::Arc;
use std::sync::Mutex;
use std::thread;
use std::time::Duration;
use std::time::Instant;

use anyhow::Context;
use base::info;
use base::warn;
use data_model::Le16;
use data_model::SLe32;
use linux_input_sys::constants::*;
use linux_input_sys::virtio_input_event;
use linux_input_sys::InputEventDecoder;
use zerocopy::IntoBytes;

use super::defaults::MOTION_ACCEL_MAX;
use super::defaults::MOTION_ACCEL_PER_G;
use super::defaults::MOTION_GYRO_MAX;
use super::defaults::MOTION_GYRO_PER_DEG_S;

/// How long an outside writer's last frame holds off the host's sensors.
const OVERRIDE_HOLD: Duration = Duration::from_secs(1);

/// The device's end of the feed, shared by the threads that write to it.
struct Output {
    device: UnixStream,
    // While set and in the future, frames from the host's sensors are dropped.
    override_until: Option<Instant>,
}

impl Output {
    fn write(&mut self, frame: &[virtio_input_event]) -> bool {
        match self.device.write_all(frame.as_bytes()) {
            Ok(()) => true,
            Err(e) => {
                warn!("motion sensors: the device's feed is gone: {}", e);
                false
            }
        }
    }

    fn write_external(&mut self, frame: &[virtio_input_event]) -> bool {
        self.override_until = Some(Instant::now() + OVERRIDE_HOLD);
        self.write(frame)
    }

    // A frame from the host's sensors, unless an outside writer is overriding them. False once the
    // device's feed is gone.
    fn write_host(&mut self, frame: &[virtio_input_event]) -> bool {
        if matches!(self.override_until, Some(until) if Instant::now() < until) {
            return true;
        }
        self.write(frame)
    }
}

/// Feed a motion-sensor device through `device` (the other end of the socket new_motion reads):
/// from the host's sensors when `host_sensors` is set and the host has them, and from `external`
/// (virtio_input_event frames, each ending in SYN_REPORT), which overrides them while it writes.
pub fn spawn_motion_feeder(
    external: Option<UnixStream>,
    device: UnixStream,
    host_sensors: bool,
) -> anyhow::Result<()> {
    let output = Arc::new(Mutex::new(Output {
        device,
        override_until: None,
    }));
    // The feed must outlive both sources: with neither (or after the host's sensors fail), a closed
    // socket would read as an error on the device's side. It lives as long as the VM's process.
    std::mem::forget(output.clone());
    if let Some(external) = external {
        let output = output.clone();
        thread::Builder::new()
            .name("v_motion_ext".to_owned())
            .spawn(move || forward_external(external, &output))
            .context("failed to start the motion sensors' outside feed")?;
    }
    if host_sensors {
        let output = output.clone();
        thread::Builder::new()
            .name("v_motion_host".to_owned())
            .spawn(move || host::run(&output))
            .context("failed to start the host's motion sensors")?;
    }
    Ok(())
}

fn forward_external(mut external: UnixStream, output: &Mutex<Output>) {
    let mut buf = [0u8; virtio_input_event::SIZE * 32];
    let mut pending: Vec<u8> = Vec::new();
    let mut frame: Vec<virtio_input_event> = Vec::new();
    loop {
        let read = match external.read(&mut buf) {
            Ok(0) => break,
            Ok(read) => read,
            Err(e) if e.kind() == ErrorKind::Interrupted => continue,
            Err(e) => {
                warn!("motion sensors: reading the outside feed failed: {}", e);
                break;
            }
        };
        pending.extend_from_slice(&buf[..read]);
        let whole = pending.len() / virtio_input_event::SIZE * virtio_input_event::SIZE;
        for bytes in pending[..whole].chunks_exact(virtio_input_event::SIZE) {
            let event = virtio_input_event::decode(bytes);
            frame.push(event);
            if event.type_.to_native() == EV_SYN && event.code.to_native() == SYN_REPORT {
                if !output.lock().unwrap().write_external(&frame) {
                    return;
                }
                frame.clear();
            }
        }
        pending.drain(..whole);
    }
    // The writer has gone: the host's sensors take over at once.
    info!("motion sensors: the outside feed closed; the host's sensors take over");
    output.lock().unwrap().override_until = None;
}

fn timestamp_event(timestamp_ns: i64) -> virtio_input_event {
    // MSC_TIMESTAMP counts microseconds and wraps, as hid-playstation's does.
    virtio_input_event {
        type_: Le16::from(EV_MSC),
        code: Le16::from(MSC_TIMESTAMP),
        value: SLe32::from((timestamp_ns / 1000) as u32 as i32),
    }
}

/// An acceleration in m/s^2 as the device's units. The device reports the gravity vector (upright,
/// Y reads -1 g); a sensor that reports the acceleration it measures (Android: upright, Y reads
/// +9.81 m/s^2) is negated by its caller.
fn accel_units(meters_per_s2: f32) -> i32 {
    let units = meters_per_s2 / 9.80665 * MOTION_ACCEL_PER_G as f32;
    (units.round() as i32).clamp(-MOTION_ACCEL_MAX, MOTION_ACCEL_MAX)
}

/// An angular velocity in rad/s as the device's units.
fn gyro_units(radians_per_s: f32) -> i32 {
    let units = radians_per_s.to_degrees() * MOTION_GYRO_PER_DEG_S as f32;
    (units.round() as i32).clamp(-MOTION_GYRO_MAX - 1, MOTION_GYRO_MAX)
}

/// One frame of three axes (ABS_X/Y/Z or ABS_RX/RY/RZ) with its timestamp.
fn axes_frame(first_axis: u16, values: [i32; 3], timestamp_ns: i64) -> [virtio_input_event; 5] {
    [
        virtio_input_event::absolute(first_axis, values[0]),
        virtio_input_event::absolute(first_axis + 1, values[1]),
        virtio_input_event::absolute(first_axis + 2, values[2]),
        timestamp_event(timestamp_ns),
        virtio_input_event::syn(),
    ]
}

#[cfg(target_os = "android")]
mod host {
    //! Android's sensors through the NDK (ASensorManager), loaded from libandroid.so at run time
    //! as the display backend loads SurfaceControl, so crosvm links nothing new. crosvm runs as
    //! root here; the sensor service only holds back continuous sensors from app uids in the
    //! background, so it reports whether or not an app is showing the guest.

    use std::ffi::c_char;
    use std::ffi::c_int;
    use std::ffi::c_void;
    use std::ptr;
    use std::sync::Mutex;

    use base::info;
    use base::warn;

    use super::accel_units;
    use super::axes_frame;
    use super::gyro_units;
    use super::Output;
    use linux_input_sys::constants::ABS_RX;
    use linux_input_sys::constants::ABS_X;

    // <android/sensor.h>
    const ASENSOR_TYPE_ACCELEROMETER: c_int = 1;
    const ASENSOR_TYPE_GYROSCOPE: c_int = 4;
    // <android/looper.h>
    const ALOOPER_PREPARE_ALLOW_NON_CALLBACKS: c_int = 1;
    const QUEUE_IDENT: c_int = 1;
    // 50 Hz: enough for the guest's orientation and for motion controls.
    const SAMPLING_PERIOD_US: i32 = 20_000;

    // ASensorEvent from <android/sensor.h>: 104 bytes; for both sensors data[0..3] is x, y, z.
    #[repr(C)]
    #[derive(Clone, Copy)]
    struct ASensorEvent {
        version: i32,
        sensor: i32,
        type_: i32,
        reserved0: i32,
        timestamp: i64,
        data: [f32; 16],
        flags: u32,
        reserved1: [i32; 3],
    }
    const _: () = assert!(std::mem::size_of::<ASensorEvent>() == 104);

    impl Default for ASensorEvent {
        fn default() -> Self {
            ASensorEvent {
                version: 0,
                sensor: 0,
                type_: 0,
                reserved0: 0,
                timestamp: 0,
                data: [0.0; 16],
                flags: 0,
                reserved1: [0; 3],
            }
        }
    }

    type GetInstanceForPackage = unsafe extern "C" fn(*const c_char) -> *mut c_void;
    type GetDefaultSensor = unsafe extern "C" fn(*mut c_void, c_int) -> *const c_void;
    type CreateEventQueue = unsafe extern "C" fn(
        *mut c_void,
        *mut c_void,
        c_int,
        *const c_void,
        *mut c_void,
    ) -> *mut c_void;
    type RegisterSensor = unsafe extern "C" fn(*mut c_void, *const c_void, i32, i64) -> c_int;
    type GetEvents = unsafe extern "C" fn(*mut c_void, *mut ASensorEvent, usize) -> isize;
    type LooperPrepare = unsafe extern "C" fn(c_int) -> *mut c_void;
    type LooperPollOnce =
        unsafe extern "C" fn(c_int, *mut c_int, *mut c_int, *mut *mut c_void) -> c_int;

    struct Ndk {
        get_instance_for_package: GetInstanceForPackage,
        get_default_sensor: GetDefaultSensor,
        create_event_queue: CreateEventQueue,
        register_sensor: RegisterSensor,
        get_events: GetEvents,
        looper_prepare: LooperPrepare,
        looper_poll_once: LooperPollOnce,
    }

    impl Ndk {
        fn load() -> Option<Ndk> {
            // SAFETY: dlopen with a NUL-terminated name; the handle is kept for the process's life.
            let lib = unsafe { libc::dlopen(c"libandroid.so".as_ptr(), libc::RTLD_NOW) };
            if lib.is_null() {
                return None;
            }
            macro_rules! symbol {
                ($name:literal, $ty:ty) => {{
                    // SAFETY: dlsym on a live handle with a NUL-terminated name.
                    let address = unsafe { libc::dlsym(lib, $name.as_ptr()) };
                    if address.is_null() {
                        return None;
                    }
                    // SAFETY: the NDK declares the symbol with exactly this signature.
                    unsafe { std::mem::transmute::<*mut c_void, $ty>(address) }
                }};
            }
            Some(Ndk {
                get_instance_for_package: symbol!(
                    c"ASensorManager_getInstanceForPackage",
                    GetInstanceForPackage
                ),
                get_default_sensor: symbol!(c"ASensorManager_getDefaultSensor", GetDefaultSensor),
                create_event_queue: symbol!(c"ASensorManager_createEventQueue", CreateEventQueue),
                register_sensor: symbol!(c"ASensorEventQueue_registerSensor", RegisterSensor),
                get_events: symbol!(c"ASensorEventQueue_getEvents", GetEvents),
                looper_prepare: symbol!(c"ALooper_prepare", LooperPrepare),
                looper_poll_once: symbol!(c"ALooper_pollOnce", LooperPollOnce),
            })
        }
    }

    pub(super) fn run(output: &Mutex<Output>) {
        let Some(ndk) = Ndk::load() else {
            warn!("motion sensors: the NDK's sensor calls are not available; no host sensors");
            return;
        };
        // SAFETY: each call is the NDK's, with arguments of the types it declares; the looper is
        // this thread's, and the queue is polled only here.
        let queue = unsafe {
            let looper = (ndk.looper_prepare)(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
            let manager = (ndk.get_instance_for_package)(c"crosvm".as_ptr());
            if looper.is_null() || manager.is_null() {
                warn!("motion sensors: no sensor manager; no host sensors");
                return;
            }
            let queue = (ndk.create_event_queue)(
                manager,
                looper,
                QUEUE_IDENT,
                ptr::null(),
                ptr::null_mut(),
            );
            if queue.is_null() {
                warn!("motion sensors: no sensor event queue; no host sensors");
                return;
            }
            let mut registered = 0;
            for (kind, label) in [
                (ASENSOR_TYPE_ACCELEROMETER, "accelerometer"),
                (ASENSOR_TYPE_GYROSCOPE, "gyroscope"),
            ] {
                let sensor = (ndk.get_default_sensor)(manager, kind);
                if sensor.is_null() {
                    warn!("motion sensors: the host has no {}", label);
                } else if (ndk.register_sensor)(queue, sensor, SAMPLING_PERIOD_US, 0) != 0 {
                    warn!("motion sensors: the host's {} would not start", label);
                } else {
                    registered += 1;
                }
            }
            if registered == 0 {
                return;
            }
            queue
        };
        info!("motion sensors: feeding the host's accelerometer and gyroscope");
        let mut events = [ASensorEvent::default(); 16];
        loop {
            // SAFETY: blocks on this thread's looper until the queue has events.
            let ident = unsafe {
                (ndk.looper_poll_once)(-1, ptr::null_mut(), ptr::null_mut(), ptr::null_mut())
            };
            if ident != QUEUE_IDENT {
                continue;
            }
            loop {
                // SAFETY: `events` holds `events.len()` ASensorEvents for the queue to fill.
                let count =
                    unsafe { (ndk.get_events)(queue, events.as_mut_ptr(), events.len()) };
                if count <= 0 {
                    break;
                }
                for event in &events[..count as usize] {
                    let [x, y, z] = [event.data[0], event.data[1], event.data[2]];
                    let frame = match event.type_ {
                        // Android reports what the accelerometer measures, the reaction to gravity
                        // (upright, Y is +9.81); the device reports gravity itself.
                        ASENSOR_TYPE_ACCELEROMETER => axes_frame(
                            ABS_X,
                            [accel_units(-x), accel_units(-y), accel_units(-z)],
                            event.timestamp,
                        ),
                        ASENSOR_TYPE_GYROSCOPE => axes_frame(
                            ABS_RX,
                            [gyro_units(x), gyro_units(y), gyro_units(z)],
                            event.timestamp,
                        ),
                        _ => continue,
                    };
                    if !output.lock().unwrap().write_host(&frame) {
                        return;
                    }
                }
            }
        }
    }
}

#[cfg(not(target_os = "android"))]
mod host {
    use std::sync::Mutex;

    use base::warn;

    use super::Output;

    pub(super) fn run(_output: &Mutex<Output>) {
        warn!("motion sensors: this host has no sensors crosvm can read");
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn units() {
        assert_eq!(accel_units(9.80665), MOTION_ACCEL_PER_G);
        assert_eq!(accel_units(-1000.0), -MOTION_ACCEL_MAX);
        assert_eq!(gyro_units(std::f32::consts::PI), 180 * MOTION_GYRO_PER_DEG_S);
    }

    #[test]
    fn override_holds_host_frames_back() {
        let (device, mut guest) = UnixStream::pair().unwrap();
        let mut output = Output {
            device,
            override_until: None,
        };
        let frame = axes_frame(ABS_X, [1, 2, 3], 0);
        assert!(output.write_host(&frame));
        assert!(output.write_external(&frame));
        assert!(output.write_host(&frame));
        drop(output);
        let mut bytes = Vec::new();
        guest.read_to_end(&mut bytes).unwrap();
        // The host frame written during the override never reached the device.
        assert_eq!(bytes.len(), 2 * frame.len() * virtio_input_event::SIZE);
    }
}
