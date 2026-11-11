// Copyright 2026 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! The doorbell of GICv3 message-based interrupts (`--mbi-spis`), in the layout of a GICv2m MSI
//! frame: the device tree's `mbi-alias` and the ACPI MADT's GIC MSI Frame both point here.
//!
//! crosvm's own devices never write it: an MSI-X vector of a virtio-pci device is an event that
//! the irq chip relays to the SPI the message names. The frame is for the guest: Linux's ACPI
//! GICv2m driver reads MSI_TYPER, and a write to MSI_SETSPI_NS (data = INTID) raises that SPI.
//! macOS's PCIe bridge (AppleVirtualPlatformPCIEMSIController) takes the frame too, and refuses
//! one whose MSI_TYPER has bit 31 clear (QEMU's GICv2m sets it for vmapple as `macos_compat`);
//! Linux masks the fields and Windows takes them from the MADT, so the bit is always set.
//!
//!   0x008  MSI_TYPER      read: 1 << 31 | base INTID << 16 | number of SPIs
//!   0x040  MSI_SETSPI_NS  write: INTID to raise
//!   0xfcc  MSI_IIDR       read: 0

use base::error;
use base::Event;
use snapshot::AnySnapshot;

use crate::pci::CrosvmDeviceId;
use crate::BusAccessInfo;
use crate::BusDevice;
use crate::DeviceId;
use crate::Suspendable;

const MSI_TYPER: u64 = 0x008;
const MSI_TYPER_VALID: u32 = 1 << 31;
/// Offset of the doorbell register: an MSI's address is the frame's base plus this.
pub const MBI_SETSPI_NS: u64 = 0x040;

pub struct MbiFrame {
    base_intid: u32,
    spis: Vec<Event>,
}

impl MbiFrame {
    /// `spis` raise INTIDs `base_intid..base_intid + spis.len()`.
    pub fn new(base_intid: u32, spis: Vec<Event>) -> MbiFrame {
        MbiFrame { base_intid, spis }
    }
}

impl BusDevice for MbiFrame {
    fn device_id(&self) -> DeviceId {
        CrosvmDeviceId::MbiFrame.into()
    }

    fn debug_label(&self) -> String {
        "mbi-frame".to_owned()
    }

    fn read(&mut self, info: BusAccessInfo, data: &mut [u8]) {
        let value: u32 = match info.offset {
            MSI_TYPER => MSI_TYPER_VALID | (self.base_intid << 16) | self.spis.len() as u32,
            _ => 0,
        };
        for (i, b) in data.iter_mut().enumerate() {
            *b = value.to_le_bytes().get(i).copied().unwrap_or(0);
        }
    }

    fn write(&mut self, info: BusAccessInfo, data: &[u8]) {
        if info.offset != MBI_SETSPI_NS || data.len() < 2 {
            return;
        }
        let intid = u16::from_le_bytes([data[0], data[1]]) as u32 & 0x3ff;
        let spi = intid.checked_sub(self.base_intid).map(|spi| spi as usize);
        match spi.and_then(|spi| self.spis.get(spi)) {
            Some(event) => {
                if let Err(e) = event.signal() {
                    error!("mbi-frame: failed to raise INTID {}: {}", intid, e);
                }
            }
            None => error!("mbi-frame: INTID {} is not an MBI SPI", intid),
        }
    }
}

impl Suspendable for MbiFrame {
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
