// Copyright 2020 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use anyhow::anyhow;
use base::Event;
use base::Result;
use hypervisor::DeviceKind;
use snapshot::AnySnapshot;

use crate::IrqChip;

pub trait IrqChipAArch64: IrqChip {
    // Clones this trait as a `Box` version of itself.
    fn try_box_clone(&self) -> Result<Box<dyn IrqChipAArch64>>;

    // Get this as the super-trait IrqChip.
    fn as_irq_chip(&self) -> &dyn IrqChip;

    // Get this as the mutable super-trait IrqChip.
    fn as_irq_chip_mut(&mut self) -> &mut dyn IrqChip;

    /// Get the version of VGIC that this chip is emulating. Currently KVM may either implement
    /// VGIC version 2 or 3.
    fn get_vgic_version(&self) -> DeviceKind;

    /// Once all the VCPUs have been enabled, finalize the irq chip.
    fn finalize(&self) -> Result<()>;

    /// Message-based interrupts (GICv3 MBI, `--mbi-spis`): give each of the `count` SPIs from
    /// `base_spi` an edge line, as the targets of the guest's MSI/MSI-X messages (address
    /// `doorbell`, data the INTID). Called once, after every line the board wires itself is
    /// registered: an edge event registered after this is a device's MSI vector. Returns the
    /// events that raise those SPIs, for a doorbell frame the guest writes itself.
    fn enable_mbi(&mut self, _base_spi: u32, _count: u32, _doorbell: u64) -> Result<Vec<Event>> {
        Err(base::Error::new(libc::ENOTSUP))
    }

    /// The line of the MBI SPI that MSI `gsi` is routed to, for the device to signal directly.
    fn mbi_route_event(&self, _gsi: u32) -> Option<Event> {
        None
    }

    // Snapshot irqchip.
    fn snapshot(&self, _cpus_num: usize) -> anyhow::Result<AnySnapshot> {
        Err(anyhow!("Snapshot not yet implemented for AArch64"))
    }

    fn restore(&mut self, _data: AnySnapshot, _vcpus_num: usize) -> anyhow::Result<()> {
        Err(anyhow!("Restore not yet implemented for AArch64"))
    }
}
