// Copyright 2023 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use std::collections::BTreeMap;

use base::error;
use base::Error;
use base::Result;
use cros_fdt::Fdt;
use cros_fdt::FdtNode;
use libc::ENOENT;
use libc::ENOTSUP;
use libc::ENOTTY;
use snapshot::AnySnapshot;
use vm_memory::GuestAddress;
use vm_memory::MemoryRegionPurpose;

use super::GunyahVcpu;
use super::GunyahVm;
use crate::AArch64SysRegId;
use crate::Hypervisor;
use crate::PsciVersion;
use crate::VcpuAArch64;
use crate::VcpuRegAArch64;
use crate::VmAArch64;
use crate::PSCI_0_2;

const GIC_FDT_IRQ_TYPE_SPI: u32 = 0;

const IRQ_TYPE_EDGE_RISING: u32 = 0x00000001;
const IRQ_TYPE_LEVEL_HIGH: u32 = 0x00000004;

fn fdt_create_shm_device(
    parent: &mut FdtNode,
    index: u32,
    guest_addr: GuestAddress,
) -> cros_fdt::Result<()> {
    let shm_name = format!("shm-{:x}", index);
    let shm_node = parent.subnode_mut(&shm_name)?;
    shm_node.set_prop("vdevice-type", "shm")?;
    shm_node.set_prop("peer-default", ())?;
    shm_node.set_prop("dma_base", 0u64)?;
    let mem_node = shm_node.subnode_mut("memory")?;
    // We have to add the shm device for RM to accept the swiotlb memparcel.
    // Memparcel is only used on android14-6.1. Once android14-6.1 is EOL
    // we should be able to remove all the times we call fdt_create_shm_device()
    mem_node.set_prop("optional", ())?;
    mem_node.set_prop("label", index)?;
    mem_node.set_prop("#address-cells", 2u32)?;
    mem_node.set_prop("base", guest_addr.offset())
}

impl VmAArch64 for GunyahVm {
    fn get_hypervisor(&self) -> &dyn Hypervisor {
        &self.gh
    }

    fn load_protected_vm_firmware(
        &mut self,
        fw_addr: GuestAddress,
        fw_max_size: u64,
    ) -> Result<()> {
        self.set_protected_vm_firmware_ipa(fw_addr, fw_max_size)
    }

    fn create_vcpu(&self, id: usize) -> Result<Box<dyn VcpuAArch64>> {
        Ok(Box::new(GunyahVm::create_vcpu(self, id)?))
    }

    fn create_fdt(&self, fdt: &mut Fdt, phandles: &BTreeMap<&str, u32>) -> cros_fdt::Result<()> {
        let top_node = fdt.root_mut().subnode_mut("gunyah-vm-config")?;

        top_node.set_prop("image-name", "crosvm-vm")?;
        top_node.set_prop("os-type", "linux")?;

        let memory_node = top_node.subnode_mut("memory")?;
        memory_node.set_prop("#address-cells", 2u32)?;
        memory_node.set_prop("#size-cells", 2u32)?;

        // The gunyah-vm-config/memory node defines the VM's IPA layout
        // [base-address, base-address + size-max). Gunyah only builds stage-2
        // mappings (and generates MMIO exits) for IPAs WITHIN this layout;
        // accesses outside cause stage-2 aborts injected into the guest (SIGBUS).
        //
        // base-address MUST stay at the primary GuestMemoryRegion (PHYS_MEM_START):
        // for --protected-vm-without-firmware crosvm emits no firmware-address, so
        // the Gunyah RM uses base-address to locate the guest kernel (loaded there).
        // Setting it to 0 makes the RM fail to find the kernel -> VM init fails with
        // ENODEV ("No such device") and never starts.
        //
        // Previously crosvm set NO size-max, so the layout did not extend past RAM
        // and the host-visible virtio-gpu BAR (placed just above RAM in the 64-bit
        // PCI MMIO window, see aarch64 get_system_allocator_config) fell outside it:
        // the runtime SHARE was accepted by the ioctl but never mapped into the guest
        // stage-2 -> guest SIGBUS on the gfxstream ASG ring. Extend size-max to cover
        // RAM plus the high-MMIO window above it (>= 2 GiB headroom, minimum 4 GiB),
        // keeping the BAR inside the layout.
        const GIB: u64 = 1 << 30;
        let mut base_address: Option<u64> = None;
        let mut ram_top: u64 = 0;
        let mut firmware_set = false;
        // Lowest IPA crosvm hands to the guest, captured to bound the RM-lowmem fence
        // below (the firmware window when present, else the payload/RAM base).
        let mut firmware_base: Option<u64> = None;
        for region in self.guest_mem.regions() {
            let region_end = region.guest_addr.offset() + region.size as u64;
            if region_end > ram_top {
                ram_top = region_end;
            }
            match region.options.purpose {
                MemoryRegionPurpose::GuestMemoryRegion => {
                    // Assume the first GuestMemoryRegion contains the payload.
                    if base_address.is_none() {
                        base_address = Some(region.guest_addr.offset());
                    }
                }
                MemoryRegionPurpose::ProtectedFirmwareRegion => {
                    if firmware_set {
                        // Should only have one protected firmware memory region.
                        error!("Multiple ProtectedFirmwareRegions unexpected.");
                        unreachable!()
                    }
                    firmware_set = true;
                    firmware_base = Some(region.guest_addr.offset());
                    memory_node.set_prop("firmware-address", region.guest_addr.offset())?;
                }
                _ => {}
            }
        }

        // Keep base-address at the payload region (see comment above). size-max is the
        // layout *length* from base and must cover, in order of increasing IPA:
        //   (1) guest RAM [base, ram_top) -- this already includes the GPU pre-alloc pool
        //       regions, which are appended after RAM as first-class guest_mem regions, and
        //   (2) the high-MMIO PCI window placed just above RAM (the host-visible
        //       virtio-gpu BAR needs a stage-2 mapping, else the guest SIGBUSes).
        // The window top is a deterministic function of ram_top — derive it from the
        // exact same formula as aarch64 get_system_allocator_config
        // (gunyah_high_mmio_window_top) rather than a magic headroom. Rounded up to a
        // GiB, 4 GiB minimum.
        let base_address = base_address.unwrap_or(0);
        const PLAT_MMIO_SIZE: u64 = 0x800000; // AARCH64_PLATFORM_MMIO_SIZE
        const BAR_TARGET: u64 = 4 * GIB;
        let window_base = ram_top + PLAT_MMIO_SIZE;
        let window_top = window_base.next_multiple_of(BAR_TARGET) + BAR_TARGET + (1u64 << 29);
        let size_max =
            core::cmp::max((window_top - base_address).next_multiple_of(GIB), 4 * GIB);
        memory_node.set_prop("base-address", base_address)?;
        memory_node.set_prop("size-max", size_max)?;

        let interrupts_node = top_node.subnode_mut("interrupts")?;
        interrupts_node.set_prop("config", *phandles.get("intc").unwrap())?;

        let vcpus_node = top_node.subnode_mut("vcpus")?;
        vcpus_node.set_prop("affinity", "proxy")?;

        let vdev_node = top_node.subnode_mut("vdevices")?;
        vdev_node.set_prop("generate", "/hypervisor")?;

        for irq in self.routes.lock().iter() {
            let bell_name = format!("bell-{:x}", irq.irq);
            let bell_node = vdev_node.subnode_mut(&bell_name)?;
            bell_node.set_prop("vdevice-type", "doorbell")?;
            let path_name = format!("/hypervisor/bell-{:x}", irq.irq);
            bell_node.set_prop("generate", path_name)?;
            bell_node.set_prop("label", irq.irq)?;
            bell_node.set_prop("peer-default", ())?;
            bell_node.set_prop("source-can-clear", ())?;

            let interrupt_type = if irq.level {
                IRQ_TYPE_LEVEL_HIGH
            } else {
                IRQ_TYPE_EDGE_RISING
            };
            let interrupts = [GIC_FDT_IRQ_TYPE_SPI, irq.irq, interrupt_type];
            bell_node.set_prop("interrupts", &interrupts)?;
        }

        // PROBE: declare an rm-rpc vdevice so RM builds a RM<->guest message-queue
        // pair and generates /hypervisor/qcom,resource-mgr (compatible
        // "gunyah-resource-manager") in the guest DT with reg = <tx_capid rx_capid>.
        // Format mirrors Qualcomm kalama/monaco-vm.dtsi. This validates whether RM
        // will grant rm-rpc to this protected VM; if VM_START fails here, RM is
        // rejecting it. (No console-dev, to avoid disturbing the guest console.)
        let rm_node = vdev_node.subnode_mut("rm-rpc")?;
        rm_node.set_prop("vdevice-type", "rm-rpc")?;
        rm_node.set_prop("generate", "/hypervisor/qcom,resource-mgr")?;
        rm_node.set_prop("message-size", 0xf0u32)?;
        rm_node.set_prop("queue-depth", 0x8u32)?;

        for region in self.guest_mem.regions() {
            let create_shm_node = match region.options.purpose {
                MemoryRegionPurpose::Bios => false,
                // GPU pre-alloc pool: SHARE'd like swiotlb/framebuffer — declare an shm
                // vdevice so the RM builds the memparcel and the guest gets a stage-2
                // mapping without any runtime accept.
                MemoryRegionPurpose::GpuPool => true,
                // Guest-alloc pool: same — needs the shm vdevice + stage-2 mapping so the
                // guest driver can allocate from it and the host resolves its mem-entries.
                MemoryRegionPurpose::GpuPoolGuest => true,
                // drm2kgsl arena: same -- shm vdevice + stage-2 mapping, no runtime accept.
                MemoryRegionPurpose::Drm2KgslPool => true,
                // EDK2 preload pool: its zero-length boot floor creates no fixed mapping; the
                // complete range is installed later by runtime SHARE plus guest MEM_ACCEPT.
                MemoryRegionPurpose::Edk2PreloadPool => true,
                MemoryRegionPurpose::GuestMemoryRegion => false,
                // Described by the "firmware-address" property
                MemoryRegionPurpose::ProtectedFirmwareRegion => false,
                MemoryRegionPurpose::ReservedMemory => false,
                MemoryRegionPurpose::SharedFramebuffer => true,
                MemoryRegionPurpose::StaticSwiotlbRegion => true,
            };

            if create_shm_node {
                fdt_create_shm_device(
                    vdev_node,
                    region.index.try_into().unwrap(),
                    region.guest_addr,
                )?;
            }
        }

        // Fence off the Gunyah RM's low-IPA memory donation.
        //
        // When the RM creates this pVM it donates a low-IPA memory region (empirically
        // ~40-60 MiB fragmented within [0x40000000, 0x44406000)) that lives BELOW
        // base-address and OUTSIDE the [base-address, base-address+size-max) layout we
        // declare above. The RM PREPENDS it to the guest /memory reg, so the guest treats
        // it as ordinary System RAM (it lands in ZONE_DMA). But the RM maps that donated
        // region RW-but-NOT-executable in the host stage-2 -- it hands it out as data RAM.
        // crosvm's own LENT RAM at base-address is GH_MEM_ALLOW_EXEC and is always
        // exec-clean; the donated low region is statically non-executable.
        //
        // Under memory pressure the guest falls back to ZONE_DMA and places executable
        // pages (JIT, .so text) in the donated region, then takes SIGBUS (BUS_OBJERR,
        // si_addr==pc) on the instruction fetch. This is the Minecraft/gnome-shell crash;
        // an exec-probe confirms every no-exec page is in the 0x40000000 bucket and the
        // high LENT RAM never strips even under 2.8 GiB of pressure.
        //
        // Reserve the low gap [FLOOR, resv_top) as no-map so the guest drops it from
        // memblock at early boot and never allocates code (or anything) there. resv_top is
        // the lowest IPA crosvm itself hands the guest -- the firmware window when this is a
        // firmware-mode pVM, otherwise the payload/RAM base -- so the fence never overlaps
        // anything crosvm placed. The range extends past the observed donation to absorb any
        // RM-side variation; reserving the non-RAM remainder is harmless (memblock only
        // removes the intersection with real memory). Losing the donated ~40-60 MiB is
        // immaterial next to the multi-GiB LENT RAM.
        //
        // FLOOR is the lowest IPA the RM's donation has ever occupied (empirically the
        // fragments live in [0x40000000, 0x44406000)). 0x40000000 sits just above the GIC
        // distributor window (aarch64 AARCH64_GIC_DIST_BASE = 0x40000000 - dist_size), so no
        // guest RAM can legitimately exist below it; it is the natural bottom of the fence.
        const GUNYAH_RM_LOWMEM_FLOOR: u64 = 0x4000_0000;
        let resv_top = firmware_base.unwrap_or(base_address);
        if resv_top > GUNYAH_RM_LOWMEM_FLOOR {
            let resv_size = resv_top - GUNYAH_RM_LOWMEM_FLOOR;
            let resv = fdt.root_mut().subnode_mut("reserved-memory")?;
            resv.set_prop("#address-cells", 2u32)?;
            resv.set_prop("#size-cells", 2u32)?;
            resv.set_prop("ranges", ())?;
            let node =
                resv.subnode_mut(&format!("gunyah-rm-lowmem@{:x}", GUNYAH_RM_LOWMEM_FLOOR))?;
            node.set_prop("reg", &[GUNYAH_RM_LOWMEM_FLOOR, resv_size])?;
            node.set_prop("no-map", ())?;
        }

        Ok(())
    }

    fn init_arch(
        &self,
        payload_entry_address: GuestAddress,
        fdt_address: GuestAddress,
        fdt_size: usize,
    ) -> Result<()> {
        // The payload entry is the memory address where the kernel starts.
        // This memory region contains both the DTB and the kernel image,
        // so ensure they are located together.

        let (dtb_mapping, _, dtb_obj_offset) = self
            .guest_mem
            .find_region(fdt_address)
            .map_err(|_| Error::new(ENOENT))?;
        let (payload_mapping, payload_offset, payload_obj_offset) = self
            .guest_mem
            .find_region(payload_entry_address)
            .map_err(|_| Error::new(ENOENT))?;

        if !std::ptr::eq(dtb_mapping, payload_mapping) || dtb_obj_offset != payload_obj_offset {
            panic!("DTB and payload are not part of same memory region.");
        }

        if self.vm_id.is_some() && self.pas_id.is_some() {
            // Gunyah will find the metadata about the Qualcomm Trusted VM in the
            // first few pages (decided at build time) of the primary payload region.
            // This metadata consists of the elf header which tells Gunyah where
            // the different elf segments (kernel/DTB/ramdisk) are. As we send the entire
            // primary payload as a single memory parcel to Gunyah, with the offsets from
            // the elf header, Gunyah can find the VM DTBOs.
            // Pass on the primary payload region start address and its size for Qualcomm
            // Trusted VMs.
            for region in self.guest_mem.regions() {
                if region.guest_addr.offset() == payload_entry_address.offset() {
                    self.set_vm_auth_type_to_qcom_trusted_vm(payload_entry_address, region.size.try_into().unwrap());
                    break;
                }
            }
        }

        self.set_dtb_config(fdt_address, fdt_size)?;

        // Gunyah sets the PC to the payload entry point for protected VMs without firmware.
        // It needs to be 0 as Gunyah assumes it to be kernel start.
        if self.hv_cfg.protection_type.isolates_memory() &&
           !self.hv_cfg.protection_type.runs_firmware() && payload_offset != 0 {
            panic!("Payload offset must be zero");
        }

        if let Err(e) = self.set_boot_pc(payload_entry_address.offset()) {
            if e.errno() == ENOTTY {
                // GH_VM_SET_BOOT_CONTEXT ioctl is not supported, but returning success
                // for backward compatibility when the offset is zero.
                if payload_offset != 0 {
                    panic!("Payload offset must be zero");
                }
            } else {
                return Err(e);
            }
        }

        self.start()?;

        Ok(())
    }
}

impl VcpuAArch64 for GunyahVcpu {
    fn init(&self, _features: &[crate::VcpuFeature]) -> Result<()> {
        Ok(())
    }

    fn init_pmu(&self, _irq: u64) -> Result<()> {
        Err(Error::new(ENOTSUP))
    }

    fn has_pvtime_support(&self) -> bool {
        false
    }

    fn init_pvtime(&self, _pvtime_ipa: u64) -> Result<()> {
        Err(Error::new(ENOTSUP))
    }

    fn set_one_reg(&self, _reg_id: VcpuRegAArch64, _data: u64) -> Result<()> {
        unimplemented!()
    }

    fn get_one_reg(&self, _reg_id: VcpuRegAArch64) -> Result<u64> {
        Err(Error::new(ENOTSUP))
    }

    fn set_vector_reg(&self, _reg_num: u8, _data: u128) -> Result<()> {
        unimplemented!()
    }

    fn get_vector_reg(&self, _reg_num: u8) -> Result<u128> {
        unimplemented!()
    }

    fn get_psci_version(&self) -> Result<PsciVersion> {
        Ok(PSCI_0_2)
    }

    fn set_guest_debug(&self, _addrs: &[GuestAddress], _enable_singlestep: bool) -> Result<()> {
        Err(Error::new(ENOTSUP))
    }

    fn get_max_hw_bps(&self) -> Result<usize> {
        Err(Error::new(ENOTSUP))
    }

    fn get_system_regs(&self) -> Result<BTreeMap<AArch64SysRegId, u64>> {
        Err(Error::new(ENOTSUP))
    }

    fn get_cache_info(&self) -> Result<BTreeMap<u8, u64>> {
        Err(Error::new(ENOTSUP))
    }

    fn set_cache_info(&self, _cache_info: BTreeMap<u8, u64>) -> Result<()> {
        Err(Error::new(ENOTSUP))
    }

    fn hypervisor_specific_snapshot(&self) -> anyhow::Result<AnySnapshot> {
        unimplemented!()
    }

    fn hypervisor_specific_restore(&self, _data: AnySnapshot) -> anyhow::Result<()> {
        unimplemented!()
    }
}
