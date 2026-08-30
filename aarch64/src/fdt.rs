// Copyright 2018 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use std::collections::BTreeMap;
use std::collections::HashSet;
use std::fs::File;
use std::fs::OpenOptions;
use std::io::Write;
use std::path::PathBuf;

use arch::apply_device_tree_overlays;
use arch::serial::SerialDeviceInfo;
use arch::CpuSet;
use arch::DtbOverlay;
#[cfg(any(target_os = "android", target_os = "linux"))]
use arch::PlatformBusResources;
use arch::SmbiosOptions;
use base::open_file_or_duplicate;
use cros_fdt::Error;
use cros_fdt::Fdt;
use cros_fdt::Result;
// This is a Battery related constant
use devices::bat::GOLDFISHBAT_MMIO_LEN;
use devices::pl030::PL030_AMBA_ID;
use devices::pl061::GPIO_PIN_POWER_BUTTON;
use devices::pl061::GPIO_PIN_SLEEP_BUTTON;
use devices::pl061::PL061_AMBA_ID;
use devices::IommuDevType;
use devices::PciAddress;
use devices::PciInterruptPin;
use hypervisor::PsciVersion;
use hypervisor::VmAArch64;
use hypervisor::PSCI_0_2;
use hypervisor::PSCI_1_0;
use rand::rngs::OsRng;
use rand::RngCore;
use resources::AddressRange;
use vm_memory::GuestAddress;
use vm_memory::GuestMemory;
use vm_memory::MemoryRegionPurpose;

// These are GIC address-space location constants.
use crate::AARCH64_GIC_CPUI_BASE;
use crate::AARCH64_GIC_CPUI_SIZE;
use crate::AARCH64_GIC_DIST_BASE;
use crate::AARCH64_GIC_DIST_SIZE;
use crate::AARCH64_GIC_REDIST_SIZE;
use crate::AARCH64_PMU_IRQ;
use crate::AARCH64_PROTECTED_VM_FW_START;
// These are GPIO (PL061) related constants
use crate::AARCH64_GPIO_ADDR;
use crate::AARCH64_GPIO_IRQ;
use crate::AARCH64_GPIO_SIZE;
// These are RTC related constants
use crate::AARCH64_RTC_ADDR;
use crate::AARCH64_RTC_IRQ;
use crate::AARCH64_RTC_SIZE;
// These are serial device related constants.
use crate::AARCH64_SERIAL_SPEED;
use crate::AARCH64_VIRTFREQ_BASE;
use crate::AARCH64_VIRTFREQ_SIZE;
use crate::AARCH64_VIRTFREQ_V2_SIZE;
use crate::AARCH64_VMWDT_IRQ;

// This is an arbitrary number to specify the node for the GIC.
// If we had a more complex interrupt architecture, then we'd need an enum for
// these.
const PHANDLE_GIC: u32 = 1;
const PHANDLE_RESTRICTED_DMA_POOL: u32 = 2;
const PHANDLE_SIMPLEFB_RESERVED: u32 = 3;
const PHANDLE_GPIO: u32 = 4;

// Shared fixed clock (apb_pclk) for the AMBA PrimeCell devices (PL030 RTC and
// PL061 GPIO).
const PCLK_PHANDLE: u32 = 24;

// CPUs are assigned phandles starting with this number.
const PHANDLE_CPU0: u32 = 0x100;

const PHANDLE_OPP_DOMAIN_BASE: u32 = 0x1000;

// pKVM pvIOMMUs are assigned phandles starting with this number.
const PHANDLE_PKVM_PVIOMMU: u32 = 0x2000;

// These are specified by the Linux GIC bindings
const GIC_FDT_IRQ_NUM_CELLS: u32 = 3;
const GIC_FDT_IRQ_TYPE_SPI: u32 = 0;
const GIC_FDT_IRQ_TYPE_PPI: u32 = 1;
const GIC_FDT_IRQ_PPI_CPU_SHIFT: u32 = 8;
const GIC_FDT_IRQ_PPI_CPU_MASK: u32 = 0xff << GIC_FDT_IRQ_PPI_CPU_SHIFT;
const IRQ_TYPE_EDGE_RISING: u32 = 0x00000001;
const IRQ_TYPE_LEVEL_HIGH: u32 = 0x00000004;
const IRQ_TYPE_LEVEL_LOW: u32 = 0x00000008;

fn create_memory_node(fdt: &mut Fdt, guest_mem: &GuestMemory) -> Result<()> {
    let mut mem_reg_prop = Vec::new();
    let mut previous_memory_region_end = None;
    // A pseudo-unprotected VM's window is left out for the same reason and one more: nothing has
    // been handed to the hypervisor for it yet, and the guest is not told it exists until the
    // shim has accepted it and pointed `/memory` here itself. The handoff page is not the guest's
    // RAM either -- it is the one place the host can still write once the boot region is lent.
    let hidden: HashSet<u64> = guest_mem
        .filter(|r| {
            r.options.step_size != 0
                || matches!(
                    r.options.purpose,
                    MemoryRegionPurpose::SharedGuestRam | MemoryRegionPurpose::ShimHandoff
                )
        })
    let mut regions = guest_mem.guest_memory_regions();
    regions.sort();
    for region in regions {
        if region.0.offset() == AARCH64_PROTECTED_VM_FW_START {
            continue;
        }
        if hidden.contains(&region.0.offset()) {
        // Merge with the previous region if possible.
        if let Some(previous_end) = previous_memory_region_end {
            if region.0 == previous_end {
                *mem_reg_prop.last_mut().unwrap() += region.1 as u64;
                previous_memory_region_end =
                    Some(previous_end.checked_add(region.1 as u64).unwrap());
                continue;
            }
            assert!(region.0 > previous_end, "Memory regions overlap");
        }

        mem_reg_prop.push(region.0.offset());
        mem_reg_prop.push(region.1 as u64);
        previous_memory_region_end = Some(region.0.checked_add(region.1 as u64).unwrap());
    }

    let memory_node = fdt.root_mut().subnode_mut("memory")?;
    memory_node.set_prop("device_type", "memory")?;
    memory_node.set_prop("reg", mem_reg_prop)?;
    Ok(())
}

fn create_resv_memory_node(
    fdt: &mut Fdt,
    resv_addr_and_size: (Option<GuestAddress>, u64),
    simplefb_cfg: Option<&SimplefbDtConfig>,
) -> Result<u32> {
    let (resv_addr, resv_size) = resv_addr_and_size;

    let resv_memory_node = fdt.root_mut().subnode_mut("reserved-memory")?;
    resv_memory_node.set_prop("#address-cells", 0x2u32)?;
    resv_memory_node.set_prop("#size-cells", 0x2u32)?;
    resv_memory_node.set_prop("ranges", ())?;

    let restricted_dma_pool_node = if let Some(resv_addr) = resv_addr {
        let node =
            resv_memory_node.subnode_mut(&format!("restricted_dma_reserved@{:x}", resv_addr.0))?;
        node.set_prop("reg", &[resv_addr.0, resv_size])?;
        node
    } else {
        let node = resv_memory_node.subnode_mut("restricted_dma_reserved")?;
        node.set_prop("size", resv_size)?;
        node
    };
    restricted_dma_pool_node.set_prop("phandle", PHANDLE_RESTRICTED_DMA_POOL)?;
    restricted_dma_pool_node.set_prop("compatible", "restricted-dma-pool")?;
    restricted_dma_pool_node.set_prop("alignment", base::pagesize() as u64)?;

    if let Some(sfb) = simplefb_cfg {
        if sfb.addr >= crate::AARCH64_PHYS_MEM_START {
            let sfb_node = fdt
                .root_mut()
                .subnode_mut("reserved-memory")?
                .subnode_mut(&format!("simplefb_reserved@{:x}", sfb.addr))?;
            sfb_node.set_prop("reg", &[sfb.addr, sfb.size])?;
            sfb_node.set_prop("no-map", ())?;
            sfb_node.set_prop("phandle", PHANDLE_SIMPLEFB_RESERVED)?;
        }
    }

    Ok(PHANDLE_RESTRICTED_DMA_POOL)
}

/// The parameters of a growable pool: the numbers a guest driver cannot work out from `reg`.
struct GrowablePool {
    /// Where the pre-shared floor ends. Below it the memory is backed at boot; above it a grant
    /// must be asked for and must have returned before anything touches the address. There is no
    /// recoverable fault to fall back on -- a read of an ungranted address returns zeros with no
    /// error at all, and a write kills the VM.
    pre_alloc_size: u64,
    /// The granularity a grant must be requested in. A misaligned request is refused by the host
    /// rather than rounded, deliberately: rounding would hand out memory nobody asked for.
    step_size: u64,
    /// Index into the host's growable-pool table, which is ordered by address. With one growable
    /// pool this is 0; it is emitted rather than assumed so that adding a second one cannot
    /// silently shift it.
    pool_id: u32,
}

/// Announce one pool to the guest as a `/reserved-memory` node.
///
/// A pool is a slab of guest-physical address space the host can reach: SHARE'd rather than lent,
/// so both sides see the same pages even in a protected VM. `name` says who the pool is for and
/// which side allocates from it -- `gfx_host`, `gpu_guest`, `drm2kgsl_host` -- and it is a WIRE
/// NAME: the guest driver finds its pool by matching this name prefix under `/reserved-memory`, so
/// changing one is a cross-repo change.
///
/// # Adding a pool
///
/// 1. `vm_memory::MemoryRegionPurpose`: give it its own variant. Sharing an existing variant means
///    sharing that pool's region, which is exactly what the separate variants exist to prevent.
/// 2. `aarch64/src/lib.rs`: carve the region out after guest RAM and collect its `(gpa, size)`.
/// 3. Here: one more `create_pool_node()` call, named `<consumer>_<side>`.
/// 4. The consuming driver: find the node by that name prefix.
///
/// Nothing else needs touching, because both of the things that read this node are generic:
///
/// * The Gunyah RM blesses the range by matching this node's `reg` against the lend=false
///   memparcel registered before VM start (`vm_creation.c
///   find_memparcel_for_resmem_node_by_address`). It walks every `/reserved-memory` child and
///   never looks at `compatible`.
/// * edk2's `GunyahPoolAcpiDxe` walks every `droidvm,pool` node and emits one ACPI device per pool
///   under `\_SB`, so a guest that cannot read a device tree -- Windows -- still finds its pool.
///   `pool-name` becomes the ACPI `_UID`, so a pool shows up as e.g. `ACPI\DRVM0001\gpu_guest`.
///
/// # Why `no-map`, and why the `compatible` must stay unknown to Linux
///
/// The RM blesses by `reg`, so `compatible` is free for us to use -- but only for a string Linux
/// has no `RESERVEDMEM_OF_DECLARE` handler for. Dynamic tracing of `gunyah_gup_demand_page`
/// (2026-06-19) showed that with `restricted-dma-pool` the guest kernel EAGERLY initialises a DMA
/// bounce pool here, zeroing the whole range during early boot, and then PSCI-resets before it
/// ever probes virtio/PCI (zero MMIO exits observed). `no-map` plus a vendor `compatible` keeps
/// the region out of the kernel's RAM/linear map -- no eager init, no crash -- while still letting
/// a driver map it on demand.
///
/// # `acpi_hid`
///
/// `None` puts the pool on the shared `_HID` (`DRVM0001`), where one Windows provider driver binds
/// every pool and hands them out by `_UID`. Override it only for a pool that needs its own Windows
/// driver: Windows matches an INF on `ACPI\<_HID>` alone -- the instance id never takes part -- so
/// a private driver needs a private `_HID`. edk2 then keeps `DRVM0001` as the `_CID`, so the
/// shared provider still picks the pool up when that driver is not installed. Nothing needs this
/// today; it is here so that adding a pool with its own driver stays a one-line change.
fn create_pool_node(
    fdt: &mut Fdt,
    name: &str,
    gpa: u64,
    size: u64,
    acpi_hid: Option<&str>,
    growable: Option<GrowablePool>,
) -> Result<()> {
    let resv = fdt.root_mut().subnode_mut("reserved-memory")?;
    // Set the parent's cells in case the swiotlb path did not create them (it normally does).
    resv.set_prop("#address-cells", 0x2u32)?;
    resv.set_prop("#size-cells", 0x2u32)?;
    resv.set_prop("ranges", ())?;

    let node = resv.subnode_mut(&format!("{}@{:x}", name, gpa))?;
    // `droidvm,pool` is what edk2 scans for; the more specific string comes first, as usual for a
    // compatible list. Both are vendor strings no Linux reserved-memory handler claims.
    if growable.is_some() {
        node.set_prop("compatible", &["droidvm,dynamic-pool", "droidvm,pool"][..])?;
    } else {
        node.set_prop("compatible", "droidvm,pool")?;
    }
    node.set_prop("no-map", ())?;
    // The name again, as a property: a consumer that reaches the node by phandle or by scanning
    // for the compatible never sees the node name, and edk2 turns this into the ACPI `_UID`.
    node.set_prop("droidvm,pool-name", name)?;
    if let Some(hid) = acpi_hid {
        node.set_prop("droidvm,acpi-hid", hid)?;
    }
    if let Some(g) = growable {
        node.set_prop("droidvm,pre-alloc-size", g.pre_alloc_size)?;
        node.set_prop("droidvm,step-size", g.step_size)?;
        node.set_prop("droidvm,pool-id", g.pool_id)?;
    }
    Ok(())
}

fn create_cpu_nodes(
    fdt: &mut Fdt,
    num_cpus: u32,
    cpu_mpidr_generator: &impl Fn(usize) -> Option<u64>,
    cpu_clusters: Vec<CpuSet>,
    cpu_capacity: BTreeMap<usize, u32>,
    dynamic_power_coefficient: BTreeMap<usize, u32>,
    cpu_frequencies: BTreeMap<usize, Vec<u32>>,
) -> Result<()> {
    let root_node = fdt.root_mut();
    let cpus_node = root_node.subnode_mut("cpus")?;
    cpus_node.set_prop("#address-cells", 0x1u32)?;
    cpus_node.set_prop("#size-cells", 0x0u32)?;

    for cpu_id in 0..num_cpus {
        let reg = u32::try_from(
            cpu_mpidr_generator(cpu_id.try_into().unwrap()).ok_or(Error::PropertyValueInvalid)?,
        )
        .map_err(|_| Error::PropertyValueTooLarge)?;
        let cpu_name = format!("cpu@{:x}", reg);
        let cpu_node = cpus_node.subnode_mut(&cpu_name)?;
        cpu_node.set_prop("device_type", "cpu")?;
        cpu_node.set_prop("compatible", "arm,armv8")?;
        if num_cpus > 1 {
            cpu_node.set_prop("enable-method", "psci")?;
        }
        cpu_node.set_prop("reg", reg)?;
        cpu_node.set_prop("phandle", PHANDLE_CPU0 + cpu_id)?;

        if let Some(pwr_coefficient) = dynamic_power_coefficient.get(&(cpu_id as usize)) {
            cpu_node.set_prop("dynamic-power-coefficient", *pwr_coefficient)?;
        }
        if let Some(capacity) = cpu_capacity.get(&(cpu_id as usize)) {
            cpu_node.set_prop("capacity-dmips-mhz", *capacity)?;
        }
        // Placed inside cpu nodes for ease of parsing for some secure firmwares(PvmFw).
        if let Some(frequencies) = cpu_frequencies.get(&(cpu_id as usize)) {
            cpu_node.set_prop("operating-points-v2", PHANDLE_OPP_DOMAIN_BASE + cpu_id)?;
            let opp_table_node = cpu_node.subnode_mut(&format!("opp_table{}", cpu_id))?;
            opp_table_node.set_prop("phandle", PHANDLE_OPP_DOMAIN_BASE + cpu_id)?;
            opp_table_node.set_prop("compatible", "operating-points-v2")?;
            for freq in frequencies.iter() {
                let opp_hz = (*freq) as u64 * 1000;
                let opp_node = opp_table_node.subnode_mut(&format!("opp{}", opp_hz))?;
                opp_node.set_prop("opp-hz", opp_hz)?;
            }
        }
    }

    if !cpu_clusters.is_empty() {
        let cpu_map_node = cpus_node.subnode_mut("cpu-map")?;
        for (cluster_idx, cpus) in cpu_clusters.iter().enumerate() {
            let cluster_node = cpu_map_node.subnode_mut(&format!("cluster{}", cluster_idx))?;
            for (core_idx, cpu_id) in cpus.iter().enumerate() {
                let core_node = cluster_node.subnode_mut(&format!("core{}", core_idx))?;
                core_node.set_prop("cpu", PHANDLE_CPU0 + *cpu_id as u32)?;
            }
        }
    }

    Ok(())
}

fn create_gic_node(fdt: &mut Fdt, is_gicv3: bool, num_cpus: u64) -> Result<()> {
    let mut gic_reg_prop = [AARCH64_GIC_DIST_BASE, AARCH64_GIC_DIST_SIZE, 0, 0];

    let intc_node = fdt.root_mut().subnode_mut("intc")?;
    if is_gicv3 {
        intc_node.set_prop("compatible", "arm,gic-v3")?;
        gic_reg_prop[2] = AARCH64_GIC_DIST_BASE - (AARCH64_GIC_REDIST_SIZE * num_cpus);
        gic_reg_prop[3] = AARCH64_GIC_REDIST_SIZE * num_cpus;
    } else {
        intc_node.set_prop("compatible", "arm,cortex-a15-gic")?;
        gic_reg_prop[2] = AARCH64_GIC_CPUI_BASE;
        gic_reg_prop[3] = AARCH64_GIC_CPUI_SIZE;
    }
    intc_node.set_prop("#interrupt-cells", GIC_FDT_IRQ_NUM_CELLS)?;
    intc_node.set_prop("interrupt-controller", ())?;
    intc_node.set_prop("reg", &gic_reg_prop)?;
    intc_node.set_prop("phandle", PHANDLE_GIC)?;
    intc_node.set_prop("#address-cells", 2u32)?;
    intc_node.set_prop("#size-cells", 2u32)?;
    add_symbols_entry(fdt, "intc", "/intc")?;
    Ok(())
}

fn create_timer_node(fdt: &mut Fdt, num_cpus: u32) -> Result<()> {
    // These are fixed interrupt numbers for the timer device.
    let irqs = [13, 14, 11, 10];
    let compatible = "arm,armv8-timer";
    let cpu_mask: u32 =
        (((1 << num_cpus) - 1) << GIC_FDT_IRQ_PPI_CPU_SHIFT) & GIC_FDT_IRQ_PPI_CPU_MASK;

    let mut timer_reg_cells = Vec::new();
    for &irq in &irqs {
        timer_reg_cells.push(GIC_FDT_IRQ_TYPE_PPI);
        timer_reg_cells.push(irq);
        timer_reg_cells.push(cpu_mask | IRQ_TYPE_LEVEL_LOW);
    }

    let timer_node = fdt.root_mut().subnode_mut("timer")?;
    timer_node.set_prop("compatible", compatible)?;
    timer_node.set_prop("interrupts", timer_reg_cells)?;
    timer_node.set_prop("always-on", ())?;
    Ok(())
}

fn create_virt_cpufreq_node(fdt: &mut Fdt, num_cpus: u64) -> Result<()> {
    let compatible = "virtual,android-v-only-cpufreq";
    let vcf_node = fdt.root_mut().subnode_mut("cpufreq")?;
    let reg = [AARCH64_VIRTFREQ_BASE, AARCH64_VIRTFREQ_SIZE * num_cpus];

    vcf_node.set_prop("compatible", compatible)?;
    vcf_node.set_prop("reg", &reg)?;
    Ok(())
}

fn create_virt_cpufreq_v2_node(fdt: &mut Fdt, num_cpus: u64) -> Result<()> {
    let compatible = "qemu,virtual-cpufreq";
    let vcf_node = fdt.root_mut().subnode_mut("cpufreq")?;
    let reg = [AARCH64_VIRTFREQ_BASE, AARCH64_VIRTFREQ_V2_SIZE * num_cpus];

    vcf_node.set_prop("compatible", compatible)?;
    vcf_node.set_prop("reg", &reg)?;
    Ok(())
}

fn create_pmu_node(fdt: &mut Fdt, num_cpus: u32) -> Result<()> {
    let compatible = "arm,armv8-pmuv3";
    let cpu_mask: u32 =
        (((1 << num_cpus) - 1) << GIC_FDT_IRQ_PPI_CPU_SHIFT) & GIC_FDT_IRQ_PPI_CPU_MASK;
    let irq = [
        GIC_FDT_IRQ_TYPE_PPI,
        AARCH64_PMU_IRQ,
        cpu_mask | IRQ_TYPE_LEVEL_HIGH,
    ];

    let pmu_node = fdt.root_mut().subnode_mut("pmu")?;
    pmu_node.set_prop("compatible", compatible)?;
    pmu_node.set_prop("interrupts", &irq)?;
    Ok(())
}

fn create_serial_node(fdt: &mut Fdt, addr: u64, size: u64, irq: u32) -> Result<()> {
    let serial_reg_prop = [addr, size];
    let irq = [GIC_FDT_IRQ_TYPE_SPI, irq, IRQ_TYPE_EDGE_RISING];

    let serial_node = fdt
        .root_mut()
        .subnode_mut(&format!("U6_16550A@{:x}", addr))?;
    serial_node.set_prop("compatible", "ns16550a")?;
    serial_node.set_prop("reg", &serial_reg_prop)?;
    serial_node.set_prop("clock-frequency", AARCH64_SERIAL_SPEED)?;
    serial_node.set_prop("interrupts", &irq)?;

    Ok(())
}

fn create_serial_nodes(fdt: &mut Fdt, serial_devices: &[SerialDeviceInfo]) -> Result<()> {
    for dev in serial_devices {
        create_serial_node(fdt, dev.address, dev.size, dev.irq)?;
    }

    Ok(())
}

/// Emit an `arm,sbsa-uart` node for the standalone SBSA UART. EDK2's dynamic
/// tables turn this into an ACPI SPCR (SBSA subtype) + an `ARMHB000` SSDT device
/// that Windows-on-ARM's `SerPL011.sys` binds to. The interrupt is declared
/// EDGE_RISING to match crosvm's edge irqfd (register_edge_irq_event); the device
/// emulates a level line over that edge via its `irq_asserted` latch. A LEVEL
/// declaration here would leave the GIC line asserted after EOI (crosvm never
/// deasserts an edge irqfd) and storm the guest with "irq N: nobody cared".
fn create_sbsa_uart_node(fdt: &mut Fdt, base: u64, irq: u32) -> Result<()> {
    let reg = [base, 0x1000u64];
    let interrupts = [GIC_FDT_IRQ_TYPE_SPI, irq, IRQ_TYPE_EDGE_RISING];
    let node = fdt.root_mut().subnode_mut(&format!("pl011@{:x}", base))?;
    // Both compatibles on purpose: EDK2 (patched SerialPortParser) classifies
    // "arm,pl011" as the PL011 DBG2 subtype, so the SSDT device gets _HID ARMH0011 --
    // the only ID Windows' inbox serpl011.inf binds. Linux ignores the pl011 entry
    // (no "arm,primecell", so no AMBA device) and binds its sbsa-uart platform
    // driver via the second string, which needs no clocks property.
    node.set_prop("compatible", &["arm,pl011", "arm,sbsa-uart"][..])?;
    node.set_prop("reg", &reg)?;
    node.set_prop("interrupts", &interrupts)?;
    node.set_prop("current-speed", 115200u32)?;
    Ok(())
}

fn psci_compatible(version: &PsciVersion) -> Vec<&str> {
    // The PSCI kernel driver only supports compatible strings for the following
    // backward-compatible versions.
    let supported = [(PSCI_1_0, "arm,psci-1.0"), (PSCI_0_2, "arm,psci-0.2")];

    let mut compatible: Vec<_> = supported
        .iter()
        .filter(|&(v, _)| *version >= *v)
        .map(|&(_, c)| c)
        .collect();

    // The PSCI kernel driver also supports PSCI v0.1, which is NOT forward-compatible.
    if compatible.is_empty() {
        compatible = vec!["arm,psci"];
    }

    compatible
}

fn create_psci_node(fdt: &mut Fdt, version: &PsciVersion) -> Result<()> {
    let compatible = psci_compatible(version);
    let psci_node = fdt.root_mut().subnode_mut("psci")?;
    psci_node.set_prop("compatible", compatible.as_slice())?;
    // Only support aarch64 guest
    psci_node.set_prop("method", "hvc")?;
    Ok(())
}

fn create_chosen_node(
    fdt: &mut Fdt,
    cmdline: &str,
    initrd: Option<(GuestAddress, usize)>,
    stdout_path: Option<&str>,
    smbios: &SmbiosOptions,
) -> Result<()> {
    let chosen_node = fdt.root_mut().subnode_mut("chosen")?;
    chosen_node.set_prop("linux,pci-probe-only", 1u32)?;
    chosen_node.set_prop("bootargs", cmdline)?;
    if let Some(stdout_path) = stdout_path {
        // Used by android bootloader for boot console output
        chosen_node.set_prop("stdout-path", stdout_path)?;
    }

    // DroidVM: forward SMBIOS identity strings (from `--smbios`) to the guest firmware. EDK2's
    // SmbiosPlatformDxe reads these /chosen properties and publishes them in its SMBIOS tables
    // (Type 4 processor version = the CPU name Windows displays). Linux kernels ignore them.
    if let Some(v) = &smbios.processor_version {
        chosen_node.set_prop("droidvm,smbios-processor-version", v.as_str())?;
    }
    if let Some(v) = &smbios.product_name {
        chosen_node.set_prop("droidvm,smbios-product-name", v.as_str())?;
    }
    if let Some(v) = &smbios.manufacturer {
        chosen_node.set_prop("droidvm,smbios-manufacturer", v.as_str())?;
    }

    let mut kaslr_seed_bytes = [0u8; 8];
    rand::rng().fill_bytes(&mut kaslr_seed_bytes);
    let kaslr_seed = u64::from_le_bytes(kaslr_seed_bytes);
    chosen_node.set_prop("kaslr-seed", kaslr_seed)?;

    let mut rng_seed_bytes = [0u8; 256];
    rand::rng().fill_bytes(&mut rng_seed_bytes);
    chosen_node.set_prop("rng-seed", &rng_seed_bytes)?;

    if let Some((initrd_addr, initrd_size)) = initrd {
        let initrd_start: u64 = initrd_addr.offset();
        let initrd_end: u64 = initrd_start + initrd_size as u64;
        chosen_node.set_prop("linux,initrd-start", initrd_start)?;
        chosen_node.set_prop("linux,initrd-end", initrd_end)?;
    }

    Ok(())
}

fn create_config_node(fdt: &mut Fdt, kernel_region: AddressRange) -> Result<()> {
    let addr: u32 = kernel_region
        .start
        .try_into()
        .map_err(|_| Error::PropertyValueTooLarge)?;
    let size: u32 = kernel_region
        .len()
        .expect("invalid kernel_region")
        .try_into()
        .map_err(|_| Error::PropertyValueTooLarge)?;

    let config_node = fdt.root_mut().subnode_mut("config")?;
    config_node.set_prop("kernel-address", addr)?;
    config_node.set_prop("kernel-size", size)?;
    Ok(())
}

fn create_kvm_cpufreq_node(fdt: &mut Fdt) -> Result<()> {
    let vcf_node = fdt.root_mut().subnode_mut("cpufreq")?;
    vcf_node.set_prop("compatible", "virtual,kvm-cpufreq")?;
    Ok(())
}

#[cfg(any(target_os = "android", target_os = "linux"))]
fn get_pkvm_pviommu_ids(platform_dev_resources: &Vec<PlatformBusResources>) -> Result<Vec<u32>> {
    let mut ids = HashSet::new();

    for res in platform_dev_resources {
        for iommu in &res.iommus {
            if let (IommuDevType::PkvmPviommu, Some(id), _) = iommu {
                ids.insert(*id);
            }
        }
    }

    Ok(Vec::from_iter(ids))
}

fn create_pkvm_pviommu_node(fdt: &mut Fdt, index: usize, id: u32) -> Result<u32> {
    let name = format!("pviommu{index}");
    let phandle = PHANDLE_PKVM_PVIOMMU
        .checked_add(index.try_into().unwrap())
        .unwrap();

    let iommu_node = fdt.root_mut().subnode_mut(&name)?;
    iommu_node.set_prop("phandle", phandle)?;
    iommu_node.set_prop("#iommu-cells", 1u32)?;
    iommu_node.set_prop("compatible", "pkvm,pviommu")?;
    iommu_node.set_prop("id", id)?;

    Ok(phandle)
}

/// PCI host controller address range.
///
/// This represents a single entry in the "ranges" property for a PCI host controller.
///
/// See [PCI Bus Binding to Open Firmware](https://www.openfirmware.info/data/docs/bus.pci.pdf)
/// and https://www.kernel.org/doc/Documentation/devicetree/bindings/pci/host-generic-pci.txt
/// for more information.
#[derive(Copy, Clone)]
pub struct PciRange {
    pub space: PciAddressSpace,
    pub bus_address: u64,
    pub cpu_physical_address: u64,
    pub size: u64,
    pub prefetchable: bool,
}

/// PCI address space.
#[derive(Copy, Clone)]
#[allow(dead_code)]
pub enum PciAddressSpace {
    /// PCI configuration space
    Configuration = 0b00,
    /// I/O space
    Io = 0b01,
    /// 32-bit memory space
    Memory = 0b10,
    /// 64-bit memory space
    Memory64 = 0b11,
}

/// Location of memory-mapped PCI configuration space.
#[derive(Copy, Clone)]
pub struct PciConfigRegion {
    /// Physical address of the base of the memory-mapped PCI configuration region.
    pub base: u64,
    /// Size of the PCI configuration region in bytes.
    pub size: u64,
}

/// Location of memory-mapped vm watchdog
#[derive(Copy, Clone)]
pub struct VmWdtConfig {
    /// Physical address of the base of the memory-mapped vm watchdog region.
    pub base: u64,
    /// Size of the vm watchdog region in bytes.
    pub size: u64,
    /// The internal clock frequency of the watchdog.
    pub clock_hz: u32,
    /// The expiration timeout measured in seconds.
    pub timeout_sec: u32,
}

fn create_pci_nodes(
    fdt: &mut Fdt,
    pci_irqs: Vec<(PciAddress, u32, PciInterruptPin)>,
    cfg: PciConfigRegion,
    ranges: &[PciRange],
    dma_pool_phandle: Option<u32>,
) -> Result<()> {
    // Add devicetree nodes describing a PCI generic host controller.
    // See Documentation/devicetree/bindings/pci/host-generic-pci.txt in the kernel
    // and "PCI Bus Binding to IEEE Std 1275-1994".
    let ranges: Vec<u32> = ranges
        .iter()
        .flat_map(|r| {
            let ss = r.space as u32;
            let p = r.prefetchable as u32;
            [
                // BUS_ADDRESS(3) encoded as defined in OF PCI Bus Binding
                (ss << 24) | (p << 30),
                (r.bus_address >> 32) as u32,
                r.bus_address as u32,
                // CPU_PHYSICAL(2)
                (r.cpu_physical_address >> 32) as u32,
                r.cpu_physical_address as u32,
                // SIZE(2)
                (r.size >> 32) as u32,
                r.size as u32,
            ]
        })
        .collect();

    let bus_range = [0u32, 0u32]; // Only bus 0
    let reg = [cfg.base, cfg.size];

    let mut interrupts: Vec<u32> = Vec::new();
    let masks = [0xf800u32, 0, 0, 0x7];

    for (address, irq_num, irq_pin) in pci_irqs.iter() {
        // PCI_DEVICE(3)
        interrupts.push(address.to_config_address(0, 8));
        interrupts.push(0);
        interrupts.push(0);

        // INT#(1)
        interrupts.push(irq_pin.to_mask() + 1);

        // CONTROLLER(PHANDLE)
        interrupts.push(PHANDLE_GIC);
        interrupts.push(0);
        interrupts.push(0);

        // CONTROLLER_DATA(3)
        interrupts.push(GIC_FDT_IRQ_TYPE_SPI);
        interrupts.push(*irq_num);
        interrupts.push(IRQ_TYPE_LEVEL_HIGH);

    }

    let pci_node = fdt.root_mut().subnode_mut("pci")?;
    pci_node.set_prop("compatible", "pci-host-ecam-generic")?;
    pci_node.set_prop("device_type", "pci")?;
    pci_node.set_prop("ranges", ranges)?;
    pci_node.set_prop("bus-range", &bus_range)?;
    pci_node.set_prop("#address-cells", 3u32)?;
    pci_node.set_prop("#size-cells", 2u32)?;
    pci_node.set_prop("reg", &reg)?;
    pci_node.set_prop("#interrupt-cells", 1u32)?;
    pci_node.set_prop("interrupt-map", interrupts)?;
    pci_node.set_prop("interrupt-map-mask", &masks)?;
    pci_node.set_prop("dma-coherent", ())?;
    if let Some(dma_pool_phandle) = dma_pool_phandle {
        pci_node.set_prop("memory-region", dma_pool_phandle)?;
    }
    Ok(())
}

fn create_rtc_node(fdt: &mut Fdt) -> Result<()> {
    // the kernel driver for pl030 really really wants a clock node
    // associated with an AMBA device or it will fail to probe, so we
    // need to make up a clock node to associate with the pl030 rtc
    // node and an associated handle with a unique phandle value. The same
    // clock is shared with the PL061 GPIO controller.
    let clock_node = fdt.root_mut().subnode_mut("pclk@3M")?;
    clock_node.set_prop("#clock-cells", 0u32)?;
    clock_node.set_prop("compatible", "fixed-clock")?;
    clock_node.set_prop("clock-frequency", 3141592u32)?;
    clock_node.set_prop("phandle", PCLK_PHANDLE)?;

    let rtc_name = format!("rtc@{:x}", AARCH64_RTC_ADDR);
    let reg = [AARCH64_RTC_ADDR, AARCH64_RTC_SIZE];
    // Same as the PL061 below: the PL030 alarm is an IrqEdgeEvent, so declare the line edge-
    // triggered instead of level (a level declaration on an edge doorbell storms after EOI).
    let irq = [GIC_FDT_IRQ_TYPE_SPI, AARCH64_RTC_IRQ, IRQ_TYPE_EDGE_RISING];

    let rtc_node = fdt.root_mut().subnode_mut(&rtc_name)?;
    rtc_node.set_prop("compatible", "arm,primecell")?;
    rtc_node.set_prop("arm,primecell-periphid", PL030_AMBA_ID)?;
    rtc_node.set_prop("reg", &reg)?;
    rtc_node.set_prop("interrupts", &irq)?;
    rtc_node.set_prop("clocks", PCLK_PHANDLE)?;
    rtc_node.set_prop("clock-names", "apb_pclk")?;
    Ok(())
}

/// Create a flattened device tree node for the PL061 GPIO controller along with
/// a `gpio-keys` node that maps two of its lines to the power and sleep buttons.
///
/// The guest's `gpio-keys` driver obtains its interrupt from the PL061's gpio
/// irqchip (via `gpiod_to_irq`), so the keys node only references the controller
/// through its `gpios` property. This mirrors the QEMU `virt` machine.
fn create_gpio_node(fdt: &mut Fdt) -> Result<()> {
    // Linux input event codes (see include/uapi/linux/input-event-codes.h).
    const KEY_POWER: u32 = 116;
    const KEY_SLEEP: u32 = 142;

    let gpio_name = format!("gpio@{:x}", AARCH64_GPIO_ADDR);
    let reg = [AARCH64_GPIO_ADDR, AARCH64_GPIO_SIZE];
    // The PL061 model injects its aggregate interrupt through an IrqEdgeEvent (one edge per
    // rising transition of MIS -- the Gunyah irqchip only supports edge irqfds, and the doorbell
    // vdevice behind it is declared IRQ_TYPE_EDGE_RISING in the hypervisor DT). Declaring it
    // LEVEL_HIGH here made the guest GIC treat the doorbell as a level line that nobody ever
    // deasserts: the first power-button press left vCPU0 spinning in the interrupt handler
    // (RCU stalls, soft lockups, hung tasks -- a hard guest hang on every "Power" press).
    let irq = [GIC_FDT_IRQ_TYPE_SPI, AARCH64_GPIO_IRQ, IRQ_TYPE_EDGE_RISING];

    let gpio_node = fdt.root_mut().subnode_mut(&gpio_name)?;
    gpio_node.set_prop("compatible", &["arm,pl061", "arm,primecell"])?;
    gpio_node.set_prop("arm,primecell-periphid", PL061_AMBA_ID)?;
    gpio_node.set_prop("reg", &reg)?;
    gpio_node.set_prop("interrupts", &irq)?;
    gpio_node.set_prop("gpio-controller", ())?;
    gpio_node.set_prop("#gpio-cells", 2u32)?;
    gpio_node.set_prop("clocks", PCLK_PHANDLE)?;
    gpio_node.set_prop("clock-names", "apb_pclk")?;
    gpio_node.set_prop("phandle", PHANDLE_GPIO)?;

    let keys_node = fdt.root_mut().subnode_mut("gpio-keys")?;
    keys_node.set_prop("compatible", "gpio-keys")?;

    let poweroff_node = keys_node.subnode_mut("poweroff")?;
    poweroff_node.set_prop("label", "GPIO Key Poweroff")?;
    poweroff_node.set_prop("linux,code", KEY_POWER)?;
    poweroff_node.set_prop("gpios", &[PHANDLE_GPIO, GPIO_PIN_POWER_BUTTON, 0])?;
    // Allow the power button to wake the guest from suspend (s2idle), matching
    // crosvm's x86 behaviour where resume emulates a power-button press.
    poweroff_node.set_prop("wakeup-source", ())?;

    let suspend_node = keys_node.subnode_mut("suspend")?;
    suspend_node.set_prop("label", "GPIO Key Suspend")?;
    suspend_node.set_prop("linux,code", KEY_SLEEP)?;
    suspend_node.set_prop("gpios", &[PHANDLE_GPIO, GPIO_PIN_SLEEP_BUTTON, 0])?;
    suspend_node.set_prop("wakeup-source", ())?;

    Ok(())
}

/// Create a flattened device tree node for Goldfish Battery device.
///
/// # Arguments
///
/// * `fdt` - An Fdt in which the node is created
/// * `mmio_base` - The MMIO base address of the battery
/// * `irq` - The IRQ number of the battery
fn create_battery_node(fdt: &mut Fdt, mmio_base: u64, irq: u32) -> Result<()> {
    let reg = [mmio_base, GOLDFISHBAT_MMIO_LEN];
    let irqs = [GIC_FDT_IRQ_TYPE_SPI, irq, IRQ_TYPE_LEVEL_HIGH];
    let bat_node = fdt.root_mut().subnode_mut("goldfish_battery")?;
    bat_node.set_prop("compatible", "google,goldfish-battery")?;
    bat_node.set_prop("reg", &reg)?;
    bat_node.set_prop("interrupts", &irqs)?;
    Ok(())
}

/// Configuration for the simple framebuffer device tree node.
pub struct SimplefbDtConfig {
    pub addr: u64,
    pub size: u64,
    pub width: u32,
    pub height: u32,
    pub stride: u32,
    pub format: String,
}

/// Creates a device tree node for a simple framebuffer (simplefb).
///
/// The node follows the Linux `simple-framebuffer` binding:
///   compatible = "simple-framebuffer"
///   reg = <addr size>
///   width / height / stride / format
///   status = "okay"
fn create_simplefb_node(fdt: &mut Fdt, cfg: &SimplefbDtConfig, has_resv: bool) -> Result<()> {
    let node_name = format!("framebuffer@{:x}", cfg.addr);
    let reg = [cfg.addr, cfg.size];
    let fb_node = fdt.root_mut().subnode_mut(&node_name)?;
    fb_node.set_prop("compatible", "simple-framebuffer")?;
    fb_node.set_prop("reg", &reg)?;
    fb_node.set_prop("width", cfg.width)?;
    fb_node.set_prop("height", cfg.height)?;
    fb_node.set_prop("stride", cfg.stride)?;
    fb_node.set_prop("format", cfg.format.as_str())?;
    fb_node.set_prop("status", "okay")?;
    if has_resv {
        fb_node.set_prop("memory-region", PHANDLE_SIMPLEFB_RESERVED)?;
    }
    Ok(())
}

pub struct PflashDtConfig {
    pub base: u64,
    pub size: u64,
    pub block_size: u32,
}

fn create_pflash_node(fdt: &mut Fdt, cfg: &PflashDtConfig) -> Result<()> {
    let node = fdt
        .root_mut()
        .subnode_mut(&format!("pflash@{:x}", cfg.base))?;
    node.set_prop("compatible", "cfi-flash")?;
    node.set_prop("reg", &[cfg.base, cfg.size])?;
    node.set_prop("erase-size", cfg.block_size)?;
    node.set_prop("no-unaligned-direct-access", ())?;
    node.set_prop("status", "okay")?;
    Ok(())
}

fn create_vmwdt_node(fdt: &mut Fdt, vmwdt_cfg: VmWdtConfig, num_cpus: u32) -> Result<()> {
    let vmwdt_name = format!("vmwdt@{:x}", vmwdt_cfg.base);
    let reg = [vmwdt_cfg.base, vmwdt_cfg.size];
    let cpu_mask: u32 =
        (((1 << num_cpus) - 1) << GIC_FDT_IRQ_PPI_CPU_SHIFT) & GIC_FDT_IRQ_PPI_CPU_MASK;
    let irq = [
        GIC_FDT_IRQ_TYPE_PPI,
        AARCH64_VMWDT_IRQ,
        cpu_mask | IRQ_TYPE_EDGE_RISING,
    ];

    let vmwdt_node = fdt.root_mut().subnode_mut(&vmwdt_name)?;
    vmwdt_node.set_prop("compatible", "qemu,vcpu-stall-detector")?;
    vmwdt_node.set_prop("reg", &reg)?;
    vmwdt_node.set_prop("clock-frequency", vmwdt_cfg.clock_hz)?;
    vmwdt_node.set_prop("timeout-sec", vmwdt_cfg.timeout_sec)?;
    vmwdt_node.set_prop("interrupts", &irq)?;
    Ok(())
}

// Add a node path to __symbols__ node of the FDT, so it can be referenced by an overlay.
fn add_symbols_entry(fdt: &mut Fdt, symbol: &str, path: &str) -> Result<()> {
    // Ensure the path points to a valid node with a defined phandle
    let Some(target_node) = fdt.get_node(path) else {
        return Err(Error::InvalidPath(format!("{path} does not exist")));
    };
    target_node
        .get_prop::<u32>("phandle")
        .or_else(|| target_node.get_prop("linux,phandle"))
        .ok_or_else(|| Error::InvalidPath(format!("{path} must have a phandle")))?;
    // Add the label -> path mapping.
    let symbols_node = fdt.root_mut().subnode_mut("__symbols__")?;
    symbols_node.set_prop(symbol, path)?;
    Ok(())
}

/// Creates a flattened device tree containing all of the parameters for the
/// kernel and loads it into the guest memory at the specified offset.
///
/// # Arguments
///
/// * `fdt_max_size` - The amount of space reserved for the device tree
/// * `guest_mem` - The guest memory object
/// * `pci_irqs` - List of PCI device address to PCI interrupt number and pin mappings
/// * `pci_cfg` - Location of the memory-mapped PCI configuration space.
/// * `pci_ranges` - Memory ranges accessible via the PCI host controller.
/// * `num_cpus` - Number of virtual CPUs the guest will have
/// * `fdt_address` - The offset into physical memory for the device tree
/// * `cmdline` - The kernel commandline
/// * `initrd` - An optional tuple of initrd guest physical address and size
/// * `android_fstab` - An optional file holding Android fstab entries
/// * `is_gicv3` - True if gicv3, false if v2
/// * `psci_version` - the current PSCI version
/// * `swiotlb` - Reserve a memory pool for DMA. Tuple of base address and size.
/// * `bat_mmio_base_and_irq` - The battery base address and irq number
/// * `vmwdt_cfg` - The virtual watchdog configuration
/// * `dump_device_tree_blob` - Option path to write DTB to
/// * `vm_generator` - Callback to add additional nodes to DTB. create_vm uses Aarch64Vm::create_fdt
pub fn create_fdt(
    fdt_max_size: usize,
    guest_mem: &GuestMemory,
    pci_irqs: Vec<(PciAddress, u32, PciInterruptPin)>,
    pci_cfg: PciConfigRegion,
    pci_ranges: &[PciRange],
    #[cfg(any(target_os = "android", target_os = "linux"))] platform_dev_resources: Vec<
        PlatformBusResources,
    >,
    num_cpus: u32,
    cpu_mpidr_generator: &impl Fn(usize) -> Option<u64>,
    cpu_clusters: Vec<CpuSet>,
    cpu_capacity: BTreeMap<usize, u32>,
    cpu_frequencies: BTreeMap<usize, Vec<u32>>,
    fdt_address: GuestAddress,
    cmdline: &str,
    kernel_region: AddressRange,
    initrd: Option<(GuestAddress, usize)>,
    android_fstab: Option<File>,
    is_gicv3: bool,
    use_pmu: bool,
    psci_version: PsciVersion,
    swiotlb: Option<(Option<GuestAddress>, u64)>,
    gpu_resv: Option<(u64, u64)>,
    gpu_guest_resv: Option<(u64, u64, u64, u64)>,
    shim_handoff_resv: Option<(u64, u64)>,
    bat_mmio_base_and_irq: Option<(u64, u32)>,
    vmwdt_cfg: VmWdtConfig,
    simplefb_cfg: Option<SimplefbDtConfig>,
    dump_device_tree_blob: Option<PathBuf>,
    vm_generator: &impl Fn(&mut Fdt, &BTreeMap<&str, u32>) -> cros_fdt::Result<()>,
    dynamic_power_coefficient: BTreeMap<usize, u32>,
    device_tree_overlays: Vec<DtbOverlay>,
    serial_devices: &[SerialDeviceInfo],
    virt_cpufreq_v2: bool,
    is_kvm: bool,
    smbios: &SmbiosOptions,
    pflash_cfg: Option<PflashDtConfig>,
    sbsa_uart_cfg: Option<(u64, u32, bool)>,
) -> Result<()> {
    let mut fdt = Fdt::new(&[]);
    let mut phandles_key_cache = Vec::new();
    let mut phandles = BTreeMap::new();

    // The whole thing is put into one giant node with some top level properties
    let root_node = fdt.root_mut();
    root_node.set_prop("interrupt-parent", PHANDLE_GIC)?;
    phandles.insert("intc", PHANDLE_GIC);
    root_node.set_prop("compatible", "linux,dummy-virt")?;
    root_node.set_prop("#address-cells", 0x2u32)?;
    root_node.set_prop("#size-cells", 0x2u32)?;
    if let Some(android_fstab) = android_fstab {
        arch::android::create_android_fdt(&mut fdt, android_fstab)?;
    }
    let stdout_path = if let Some((base, _irq, true)) = sbsa_uart_cfg {
        // The SBSA UART is the console: point the guest bootloader (and thus the
        // EDK2-synthesised ACPI SPCR) at it instead of COM1.
        Some(format!("/pl011@{:x}", base))
    } else {
        serial_devices
            .first()
            .map(|first_serial| format!("/U6_16550A@{:x}", first_serial.address))
    };
    create_chosen_node(&mut fdt, cmdline, initrd, stdout_path.as_deref(), smbios)?;
    create_config_node(&mut fdt, kernel_region)?;
    create_memory_node(&mut fdt, guest_mem)?;
    let dma_pool_phandle = match swiotlb {
        Some(x) => {
            let phandle = create_resv_memory_node(&mut fdt, x, simplefb_cfg.as_ref())?;
            phandles.insert("restricted_dma_reserved", phandle);
            Some(phandle)
        }
        None => {
            // Even without swiotlb, create reserved-memory for simplefb if it's
            // within the RAM range (protected VM case).
            if let Some(ref sfb) = simplefb_cfg {
                if sfb.addr >= crate::AARCH64_PHYS_MEM_START {
                    let resv_memory_node = fdt.root_mut().subnode_mut("reserved-memory")?;
                    resv_memory_node.set_prop("#address-cells", 0x2u32)?;
                    resv_memory_node.set_prop("#size-cells", 0x2u32)?;
                    resv_memory_node.set_prop("ranges", ())?;
                    let sfb_node = resv_memory_node
                        .subnode_mut(&format!("simplefb_reserved@{:x}", sfb.addr))?;
                    sfb_node.set_prop("reg", &[sfb.addr, sfb.size])?;
                    sfb_node.set_prop("no-map", ())?;
                    sfb_node.set_prop("phandle", PHANDLE_SIMPLEFB_RESERVED)?;
                }
            }
            None
        }
    };

    // The pools. See `create_pool_node` for what a pool node carries and how to add one.

    // gfxstream's host-visible pool: the host renderer allocates every host-visible blob from it
    // and the guest maps them by pool-relative offset.
    if let Some((gpa, size)) = gpu_resv {
        create_pool_node(&mut fdt, "gfx_host", gpa, size, None, None)?;
    }

    // The guest-alloc pool: the guest virtio-gpu driver owns a page allocator over this range and
    // sub-allocates BLOB_MEM_GUEST from it, handing the host ordinary mem-entries.
    let mut next_growable_pool_id = 0;
    if let Some((gpa, size, prealloc, step)) = gpu_guest_resv {
        let growable = if step != 0 {
            let pool_id = next_growable_pool_id;
            next_growable_pool_id += 1;
            Some(GrowablePool {
                pre_alloc_size: prealloc,
                step_size: step,
                pool_id,
            })
        } else {
            None
        };
        create_pool_node(&mut fdt, "gpu_guest", gpa, size, None, growable)?;
    }

    // The shim's handoff page. It is a SHARE'd region like the pools, and it needs this node for
    // the same reason they do: the resource manager blesses a memparcel by finding a
    // `/reserved-memory` child whose `reg` matches it, and refuses to start a VM that was handed
    // a parcel no node accounts for (measured on sm8650 / android14-6.1: without this node
    // GH_VM_START answers NORESOURCE, and the whole VM never runs). The guest ignores it -- the
    // compatible is a vendor string no Linux handler claims, and `no-map` keeps it out of the
    // linear map -- and the shim does not read it either, since crosvm patches the address
    // straight into the shim's header.
    if let Some((gpa, size)) = shim_handoff_resv {
        let resv = fdt.root_mut().subnode_mut("reserved-memory")?;
        resv.set_prop("#address-cells", 0x2u32)?;
        resv.set_prop("#size-cells", 0x2u32)?;
        resv.set_prop("ranges", ())?;
        let node = resv.subnode_mut(&format!("shim_handoff@{:x}", gpa))?;
        // Its own compatible rather than `droidvm,pool`: edk2's GunyahPoolAcpiDxe turns every
        // pool node into an ACPI device, and this is not a pool -- it is one page of protocol
        // between the host and the shim, finished with before anything else runs.
        node.set_prop("compatible", "droidvm,shim-handoff")?;
        node.set_prop("reg", &[gpa, size])?;
        node.set_prop("no-map", ())?;
    }

    // The growable test pool: exists so the grow/shrink path can be exercised end to end without
    // disturbing the production pools, which are all fully pre-shared and must stay that way.
        let growable = if step != 0 {
            let pool_id = next_growable_pool_id;
            next_growable_pool_id += 1;
            Some(GrowablePool {
                pre_alloc_size: prealloc,
                step_size: step,
                pool_id,
            })
        } else {
            None
        };
        // DROIDVM_POOL_HIDE=dt|shm|both (diagnostic): omit the reserved-memory node for the test
        // pools. The sm8650-era RM refuses a `/reserved-memory` child whose `reg` does not match
        // an accepted memparcel exactly, which is one of three tangled explanations for why a
        // partially-shared pool fails to start there -- the others being the shm vdevice node and
        // the region itself. Each has to be removable on its own or the cause stays unknown.
        let hide = std::env::var("DROIDVM_POOL_HIDE").unwrap_or_default();
        if hide == "dt" || hide == "both" {
            base::warn!(
                "GH-POOL: DROIDVM_POOL_HIDE={} -- omitting the {} reserved-memory node ({:#x}+{:#x})",
                hide, name, gpa, size,
            );
            continue;
        }
    // drm2kgsl's native-context arena: the guest allocates nothing from it -- it holds the host's
    // own msm shmem rings -- so nothing in the guest matches this name. It is announced anyway so
    // the RM blesses the range.
        create_pool_node(&mut fdt, "drm2kgsl_host", gpa, size, None, None)?;
    create_cpu_nodes(
        &mut fdt,
        num_cpus,
        cpu_mpidr_generator,
        cpu_clusters,
        cpu_capacity,
        dynamic_power_coefficient,
        cpu_frequencies.clone(),
    )?;
    create_gic_node(&mut fdt, is_gicv3, num_cpus as u64)?;
    create_timer_node(&mut fdt, num_cpus)?;
    if use_pmu {
        create_pmu_node(&mut fdt, num_cpus)?;
    }
    create_serial_nodes(&mut fdt, serial_devices)?;
    if let Some((base, irq, _is_console)) = sbsa_uart_cfg {
        create_sbsa_uart_node(&mut fdt, base, irq)?;
    }
    create_psci_node(&mut fdt, &psci_version)?;
    create_pci_nodes(&mut fdt, pci_irqs, pci_cfg, pci_ranges, dma_pool_phandle)?;
    create_rtc_node(&mut fdt)?;
    create_gpio_node(&mut fdt)?;
    if let Some((bat_mmio_base, bat_irq)) = bat_mmio_base_and_irq {
        create_battery_node(&mut fdt, bat_mmio_base, bat_irq)?;
    }
    if let Some(ref sfb_cfg) = simplefb_cfg {
        let has_resv = swiotlb.is_some() || sfb_cfg.addr >= crate::AARCH64_PHYS_MEM_START;
        create_simplefb_node(&mut fdt, sfb_cfg, has_resv)?;
    }
    if let Some(ref pflash_cfg) = pflash_cfg {
        create_pflash_node(&mut fdt, pflash_cfg)?;
    }
    create_vmwdt_node(&mut fdt, vmwdt_cfg, num_cpus)?;
    if is_kvm {
        create_kvm_cpufreq_node(&mut fdt)?;
    }
    vm_generator(&mut fdt, &phandles)?;
    if !cpu_frequencies.is_empty() {
        if virt_cpufreq_v2 {
            create_virt_cpufreq_v2_node(&mut fdt, num_cpus as u64)?;
        } else {
            create_virt_cpufreq_node(&mut fdt, num_cpus as u64)?;
        }
    }

    let pviommu_ids = get_pkvm_pviommu_ids(&platform_dev_resources)?;

    let cache_offset = phandles_key_cache.len();
    // Hack to extend the lifetime of the Strings as keys of phandles (i.e. &str).
    phandles_key_cache.extend(pviommu_ids.iter().map(|id| format!("pviommu{id}")));
    let pviommu_phandle_keys = &phandles_key_cache[cache_offset..];

    for (index, (id, key)) in pviommu_ids.iter().zip(pviommu_phandle_keys).enumerate() {
        let phandle = create_pkvm_pviommu_node(&mut fdt, index, *id)?;
        phandles.insert(key, phandle);
    }

    // Done writing base FDT, now apply DT overlays
    apply_device_tree_overlays(
        &mut fdt,
        device_tree_overlays,
        #[cfg(any(target_os = "android", target_os = "linux"))]
        platform_dev_resources,
        #[cfg(any(target_os = "android", target_os = "linux"))]
        &phandles,
    )?;

    let fdt_final = fdt.finish()?;

    if let Some(file_path) = dump_device_tree_blob {
        let mut fd = open_file_or_duplicate(
            &file_path,
            OpenOptions::new()
                .read(true)
                .create(true)
                .truncate(true)
                .write(true),
        )
        .map_err(|e| Error::FdtIoError(e.into()))?;
        fd.write_all(&fdt_final)
            .map_err(|e| Error::FdtDumpIoError(e, file_path.clone()))?;
    }

    if fdt_final.len() > fdt_max_size {
        return Err(Error::TotalSizeTooLarge);
    }

    let written = guest_mem
        .write_at_addr(fdt_final.as_slice(), fdt_address)
        .map_err(|_| Error::FdtGuestMemoryWriteError)?;
    if written < fdt_final.len() {
        return Err(Error::FdtGuestMemoryWriteError);
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pflash_node() {
        let mut fdt = Fdt::new(&[]);
        let config = PflashDtConfig {
            base: 0x9000_0000,
            size: 0xc_0000,
            block_size: 0x1000,
        };

        create_pflash_node(&mut fdt, &config).unwrap();

        let node = fdt.get_node("/pflash@90000000").unwrap();
        assert_eq!(node.get_prop::<String>("compatible").unwrap(), "cfi-flash");
        assert_eq!(node.get_prop::<Vec<u64>>("reg").unwrap(), [config.base, config.size]);
        assert_eq!(node.get_prop::<u32>("erase-size").unwrap(), config.block_size);
        assert!(node.get_prop::<()>("no-unaligned-direct-access").is_some());
        assert_eq!(node.get_prop::<String>("status").unwrap(), "okay");
    }

    #[test]
    fn psci_compatible_v0_1() {
        assert_eq!(
            psci_compatible(&PsciVersion::new(0, 1).unwrap()),
            vec!["arm,psci"]
        );
    }

    #[test]
    fn psci_compatible_v0_2() {
        assert_eq!(
            psci_compatible(&PsciVersion::new(0, 2).unwrap()),
            vec!["arm,psci-0.2"]
        );
    }

    #[test]
    fn psci_compatible_v0_5() {
        // Only the 0.2 version supported by the kernel should be added.
        assert_eq!(
            psci_compatible(&PsciVersion::new(0, 5).unwrap()),
            vec!["arm,psci-0.2"]
        );
    }

    #[test]
    fn psci_compatible_v1_0() {
        // Both 1.0 and 0.2 should be listed, in that order.
        assert_eq!(
            psci_compatible(&PsciVersion::new(1, 0).unwrap()),
            vec!["arm,psci-1.0", "arm,psci-0.2"]
        );
    }

    #[test]
    fn psci_compatible_v1_5() {
        // Only the 1.0 and 0.2 versions supported by the kernel should be listed.
        assert_eq!(
            psci_compatible(&PsciVersion::new(1, 5).unwrap()),
            vec!["arm,psci-1.0", "arm,psci-0.2"]
        );
    }

    #[test]
    fn symbols_entries() {
        const TEST_SYMBOL: &str = "dev";
        const TEST_PATH: &str = "/dev";

        let mut fdt = Fdt::new(&[]);
        add_symbols_entry(&mut fdt, TEST_SYMBOL, TEST_PATH).expect_err("missing node");

        fdt.root_mut().subnode_mut(TEST_SYMBOL).unwrap();
        add_symbols_entry(&mut fdt, TEST_SYMBOL, TEST_PATH).expect_err("missing phandle");

        let intc_node = fdt.get_node_mut(TEST_PATH).unwrap();
        intc_node.set_prop("phandle", 1u32).unwrap();
        add_symbols_entry(&mut fdt, TEST_SYMBOL, TEST_PATH).expect("valid path");

        let symbols = fdt.get_node("/__symbols__").unwrap();
        assert_eq!(symbols.get_prop::<String>(TEST_SYMBOL).unwrap(), TEST_PATH);
    }
}
