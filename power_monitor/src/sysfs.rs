// Copyright 2026 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! A power-monitor backend that reads the host's `/sys/class/power_supply`, for a goldfish battery
//! on a host with no ChromeOS powerd (an Android phone, or a plain Linux box). Generic: it uses
//! only the standard Linux power_supply sysfs class, nothing device- or guest-specific. The monitor
//! re-reads on a periodic timer; the kernel's power_supply uevents would be lower latency, but a
//! battery's state changes slowly and polling keeps the backend simple.

use std::error::Error;
use std::fs;
use std::path::Path;
use std::path::PathBuf;
use std::time::Duration;
use std::time::SystemTime;

use base::AsRawDescriptor;
use base::RawDescriptor;
use base::ReadNotifier;
use base::Timer;
use base::TimerTrait;

use crate::BatteryData;
use crate::BatteryStatus;
use crate::PowerClient;
use crate::PowerData;
use crate::PowerMonitor;

const SYSFS_POWER_SUPPLY: &str = "/sys/class/power_supply";
const POLL_INTERVAL: Duration = Duration::from_secs(2);

fn read_trimmed(path: &Path) -> Option<String> {
    fs::read_to_string(path).ok().map(|s| s.trim().to_string())
}

fn read_u32(dir: &Path, name: &str) -> Option<u32> {
    read_trimmed(&dir.join(name))?.parse().ok()
}

fn read_i64(dir: &Path, name: &str) -> Option<i64> {
    read_trimmed(&dir.join(name))?.parse().ok()
}

/// The system battery (the first supply of type "Battery" that is not a peripheral's own, so a
/// stylus or headset battery is skipped) and every mains/USB/wireless supply, whose `online` tells
/// whether external power is attached.
fn find_supplies() -> (Option<PathBuf>, Vec<PathBuf>) {
    let mut battery = None;
    let mut external = Vec::new();
    if let Ok(entries) = fs::read_dir(SYSFS_POWER_SUPPLY) {
        for entry in entries.flatten() {
            let dir = entry.path();
            match read_trimmed(&dir.join("type")).unwrap_or_default().as_str() {
                "Battery" => {
                    let scope = read_trimmed(&dir.join("scope")).unwrap_or_default();
                    if battery.is_none() && scope != "Device" {
                        battery = Some(dir);
                    }
                }
                "Mains" | "USB" | "USB_PD" | "USB_PD_DRP" | "Wireless" | "BrickID" => {
                    external.push(dir)
                }
                _ => {}
            }
        }
    }
    (battery, external)
}

fn read_power_data(battery: &Option<PathBuf>, external: &[PathBuf]) -> PowerData {
    let ac_online = external
        .iter()
        .any(|dir| read_u32(dir, "online") == Some(1));
    let battery = battery.as_ref().and_then(|dir| {
        if read_u32(dir, "present") == Some(0) {
            return None;
        }
        let status = match read_trimmed(&dir.join("status"))
            .unwrap_or_default()
            .as_str()
        {
            "Charging" => BatteryStatus::Charging,
            "Discharging" => BatteryStatus::Discharging,
            "Not charging" | "Full" => BatteryStatus::NotCharging,
            _ => BatteryStatus::Unknown,
        };
        Some(BatteryData {
            status,
            percent: read_u32(dir, "capacity").unwrap_or(0).min(100),
            // sysfs reports microvolts and microamps, the units PowerData wants.
            voltage: read_u32(dir, "voltage_now").unwrap_or(0),
            // current_now is signed (negative while discharging) but PowerData::current is
            // unsigned, so report its magnitude; the guest derives direction from `status`.
            current: read_i64(dir, "current_now")
                .map(|c| c.unsigned_abs() as u32)
                .unwrap_or(0),
            charge_counter: read_u32(dir, "charge_counter").unwrap_or(0),
            charge_full: read_u32(dir, "charge_full")
                .or_else(|| read_u32(dir, "charge_full_design"))
                .unwrap_or(0),
        })
    });
    PowerData { ac_online, battery }
}

/// One-shot reader for the battery device's initial value.
pub struct SysfsClient {
    battery: Option<PathBuf>,
    external: Vec<PathBuf>,
    last_request: Option<SystemTime>,
}

impl SysfsClient {
    pub fn connect() -> std::result::Result<Box<dyn PowerClient>, Box<dyn Error>> {
        let (battery, external) = find_supplies();
        Ok(Box::new(SysfsClient {
            battery,
            external,
            last_request: None,
        }))
    }
}

impl PowerClient for SysfsClient {
    fn get_power_data(&mut self) -> std::result::Result<PowerData, Box<dyn Error>> {
        self.last_request = Some(SystemTime::now());
        Ok(read_power_data(&self.battery, &self.external))
    }

    fn last_request_timestamp(&self) -> Option<SystemTime> {
        self.last_request
    }
}

/// Periodic monitor: its timer fd signals the battery device's worker, which then calls
/// `read_message` to pick up the latest state.
pub struct SysfsMonitor {
    timer: Timer,
    battery: Option<PathBuf>,
    external: Vec<PathBuf>,
}

impl SysfsMonitor {
    pub fn connect() -> std::result::Result<Box<dyn PowerMonitor>, Box<dyn Error>> {
        let mut timer = Timer::new()?;
        timer.reset_repeating(POLL_INTERVAL)?;
        let (battery, external) = find_supplies();
        Ok(Box::new(SysfsMonitor {
            timer,
            battery,
            external,
        }))
    }
}

impl PowerMonitor for SysfsMonitor {
    fn read_message(&mut self) -> std::result::Result<Option<PowerData>, Box<dyn Error>> {
        // Consume the expired timer so its fd stops signalling, then re-scan (a supply can appear
        // or vanish) and read.
        self.timer.mark_waited()?;
        let (battery, external) = find_supplies();
        self.battery = battery;
        self.external = external;
        Ok(Some(read_power_data(&self.battery, &self.external)))
    }
}

impl AsRawDescriptor for SysfsMonitor {
    fn as_raw_descriptor(&self) -> RawDescriptor {
        self.timer.as_raw_descriptor()
    }
}

impl ReadNotifier for SysfsMonitor {
    fn get_read_notifier(&self) -> &dyn AsRawDescriptor {
        self
    }
}
