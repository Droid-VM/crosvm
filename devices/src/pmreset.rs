// Copyright 2024 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! Minimal ACPI Reduced-Hardware power device for aarch64.
//!
//! Windows-on-ARM does not implement PSCI, so it cannot power off or reboot the
//! guest through the usual arm64 path. Instead its FADT (produced by edk2's
//! DynamicTablesPkg) advertises ACPI Reduced-Hardware SLEEP_CONTROL_REG /
//! SLEEP_STATUS_REG / RESET_REG in system memory. This device backs those MMIO
//! registers and turns guest writes into crosvm VM events, exactly the way the
//! x86 i8042 device turns port 0x64 writes into a reset.
//!
//! Register block (relative to the base the FADT points at):
//!   0x00  SLEEP_CONTROL_REG  write with SLP_EN set -> S5 -> VmEventType::Exit
//!   0x04  SLEEP_STATUS_REG   read returns 0 (WAK_STS clear); writes clear-only
//!   0x08  RESET_REG          write RESET_VALUE (0x42) -> VmEventType::Reset
//!
//! The addresses/values here must stay in sync with edk2's ArmFadtGenerator.c
//! (SLEEP_CONTROL_ADDR/SLEEP_STATUS_ADDR/RESET_REG_ADDR/RESET_VALUE).

use base::error;
use base::SendTube;
use base::VmEventType;
use snapshot::AnySnapshot;

use crate::pci::CrosvmDeviceId;
use crate::BusAccessInfo;
use crate::BusDevice;
use crate::DeviceId;
use crate::Suspendable;

/// Offsets within the register block, relative to the device base address.
const SLEEP_CONTROL_OFFSET: u64 = 0x0;
const SLEEP_STATUS_OFFSET: u64 = 0x4;
const RESET_OFFSET: u64 = 0x8;

/// ACPI reduced-hardware SLEEP_CONTROL_REG bit: SLP_EN (bit 5) triggers the
/// transition to the sleep state named by SLP_TYP (bits 2..4). Only S5
/// (SLP_TYP == 0, i.e. soft-off) is defined by our FADT/DSDT, so any write with
/// SLP_EN set is treated as power-off.
const SLP_EN: u8 = 1 << 5;

/// Value the FADT's RESET_REG expects to request a warm reset. Must match
/// edk2 ArmFadtGenerator.c DROIDVM_PMRESET_RESET_VALUE.
const RESET_VALUE: u8 = 0x42;

/// Minimal ACPI reduced-hardware power controller: power-off + reset only.
pub struct PmReset {
    vm_evt_wrtube: SendTube,
}

impl PmReset {
    /// Constructs the device. `vm_evt_wrtube` is the shared VM-event channel
    /// (the same one i8042/vmwdt use); Exit shuts crosvm down, Reset reboots.
    pub fn new(vm_evt_wrtube: SendTube) -> PmReset {
        PmReset { vm_evt_wrtube }
    }

    fn signal(&self, event: VmEventType) {
        if let Err(e) = self.vm_evt_wrtube.send::<VmEventType>(&event) {
            error!("pmreset: failed to send {:?} VM event: {}", event, e);
        }
    }
}

impl BusDevice for PmReset {
    fn device_id(&self) -> DeviceId {
        CrosvmDeviceId::PmReset.into()
    }

    fn debug_label(&self) -> String {
        "pmreset".to_owned()
    }

    fn read(&mut self, _info: BusAccessInfo, data: &mut [u8]) {
        // SLEEP_STATUS_REG reads back with WAK_STS clear; everything else 0.
        for b in data.iter_mut() {
            *b = 0;
        }
    }

    fn write(&mut self, info: BusAccessInfo, data: &[u8]) {
        if data.is_empty() {
            return;
        }
        match info.offset {
            SLEEP_CONTROL_OFFSET => {
                if data[0] & SLP_EN != 0 {
                    // SLP_TYP == 0 (S5 soft-off) is the only state we advertise.
                    self.signal(VmEventType::Exit);
                }
            }
            RESET_OFFSET => {
                if data[0] == RESET_VALUE {
                    self.signal(VmEventType::Reset);
                }
            }
            // SLEEP_STATUS_OFFSET writes are status-clear only: nothing to do.
            _ => {}
        }
    }
}

impl Suspendable for PmReset {
    fn snapshot(&mut self) -> anyhow::Result<AnySnapshot> {
        AnySnapshot::to_any(())
    }

    fn restore(&mut self, _data: AnySnapshot) -> anyhow::Result<()> {
        Ok(())
    }

    fn sleep(&mut self) -> anyhow::Result<()> {
        Ok(())
    }

    fn wake(&mut self) -> anyhow::Result<()> {
        Ok(())
    }
}
