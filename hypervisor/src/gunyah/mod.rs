// Copyright 2023 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
mod aarch64;

mod gunyah_sys;
mod mthp;
use std::cmp::Reverse;
use std::collections::BTreeMap;
use std::collections::BinaryHeap;
use std::collections::HashSet;
use std::ffi::CString;
use std::fs::File;
use std::mem::size_of;
use std::os::raw::c_ulong;
use std::os::unix::prelude::OsStrExt;
use std::path::Path;
use std::path::PathBuf;
use std::sync::Arc;

use anyhow::Context;
use base::errno_result;
use base::error;
use base::info;
use base::ioctl;
use base::ioctl_with_mut_ref;
use base::ioctl_with_ref;
use base::ioctl_with_val;
use base::pagesize;
use base::warn;
use base::Error;
use base::FromRawDescriptor;
use base::MemoryMapping;
use base::MemoryMappingBuilder;
use base::MmapError;
use base::RawDescriptor;
use gunyah_sys::*;
use libc::open;
use libc::EFAULT;
use libc::EEXIST;
use libc::EINVAL;
use libc::EIO;
use libc::ENOENT;
use libc::ENOSPC;
use libc::ENOTSUP;
use libc::EOVERFLOW;
use libc::O_CLOEXEC;
use libc::O_RDWR;
use sync::Mutex;
use vm_memory::MemoryRegionPurpose;

use crate::*;

pub struct Gunyah {
    gunyah: SafeDescriptor,
}

impl AsRawDescriptor for Gunyah {
    fn as_raw_descriptor(&self) -> RawDescriptor {
        self.gunyah.as_raw_descriptor()
    }
}

impl Gunyah {
    pub fn new_with_path(device_path: &Path) -> Result<Gunyah> {
        let c_path = CString::new(device_path.as_os_str().as_bytes()).unwrap();
        // SAFETY:
        // Open calls are safe because we give a nul-terminated string and verify the result.
        let ret = unsafe { open(c_path.as_ptr(), O_RDWR | O_CLOEXEC) };
        if ret < 0 {
            return errno_result();
        }
        Ok(Gunyah {
            // SAFETY:
            // Safe because we verify that ret is valid and we own the fd.
            gunyah: unsafe { SafeDescriptor::from_raw_descriptor(ret) },
        })
    }

    pub fn new() -> Result<Gunyah> {
        Gunyah::new_with_path(&PathBuf::from("/dev/gunyah"))
    }
}

impl Hypervisor for Gunyah {
    fn try_clone(&self) -> Result<Self>
    where
        Self: Sized,
    {
        Ok(Gunyah {
            gunyah: self.gunyah.try_clone()?,
        })
    }

    fn check_capability(&self, cap: HypervisorCap) -> bool {
        match cap {
            HypervisorCap::UserMemory => true,
            HypervisorCap::ArmPmuV3 => false,
            HypervisorCap::ImmediateExit => true,
            HypervisorCap::StaticSwiotlbAllocationRequired => true,
            HypervisorCap::HypervisorInitializedBootContext => true,
            HypervisorCap::S390UserSigp | HypervisorCap::TscDeadlineTimer => false,
            #[cfg(target_arch = "x86_64")]
            HypervisorCap::Xcrs | HypervisorCap::CalibratedTscLeafRequired => false,
        }
    }
}

unsafe fn android_lend_user_memory_region(
    vm: &SafeDescriptor,
    slot: MemSlot,
    read_only: bool,
    guest_addr: u64,
    memory_size: u64,
    userspace_addr: *mut u8,
) -> Result<()> {
    let mut flags = 0;

    flags |= GH_MEM_ALLOW_READ | GH_MEM_ALLOW_EXEC;
    if !read_only {
        flags |= GH_MEM_ALLOW_WRITE;
    }

    let region = gh_userspace_memory_region {
        label: slot,
        flags,
        guest_phys_addr: guest_addr,
        memory_size,
        userspace_addr: userspace_addr as u64,
    };

    let ret = ioctl_with_ref(vm, GH_VM_ANDROID_LEND_USER_MEM, &region);
    if ret == 0 {
        Ok(())
    } else {
        errno_result()
    }
}

// Wrapper around GH_SET_USER_MEMORY_REGION ioctl, which creates, modifies, or deletes a mapping
// from guest physical to host user pages.
//
// SAFETY:
// Safe when the guest regions are guaranteed not to overlap.
unsafe fn set_user_memory_region(
    vm: &SafeDescriptor,
    slot: MemSlot,
    read_only: bool,
    allow_exec: bool,
    guest_addr: u64,
    memory_size: u64,
    userspace_addr: *mut u8,
) -> Result<()> {
    let mut flags = 0;

    // In protected VMs, SHARE'd memory (GH_VM_SET_USER_MEM_REGION) cannot be made executable —
    // requesting GH_MEM_ALLOW_EXEC prevents Gunyah from creating valid stage-2 mappings, so the
    // guest faults (SIGBUS) on access. Only LEND'd memory gets exec at stage-2. Host-visible
    // virtio-gpu blobs (gfxstream ASG rings) are data-only and must be SHARE'd without exec.
    flags |= GH_MEM_ALLOW_READ;
    if allow_exec {
        flags |= GH_MEM_ALLOW_EXEC;
    }
    if !read_only {
        flags |= GH_MEM_ALLOW_WRITE;
    }

    let region = gh_userspace_memory_region {
        label: slot,
        flags,
        guest_phys_addr: guest_addr,
        memory_size,
        userspace_addr: userspace_addr as u64,
    };

    let ret = ioctl_with_ref(vm, GH_VM_SET_USER_MEM_REGION, &region);
    if ret == 0 {
        Ok(())
    } else {
        errno_result()
    }
}

fn map_cma_region(
    vm: &SafeDescriptor,
    slot: MemSlot,
    lend: bool,
    read_only: bool,
    guest_addr: u64,
    guest_mem_fd: u32,
    size: u64,
    offset: u64,
) -> Result<()> {
    let mut flags = 0;
    flags |= GUNYAH_MEM_ALLOW_READ | GUNYAH_MEM_ALLOW_EXEC;
    if !read_only {
        flags |= GUNYAH_MEM_ALLOW_WRITE;
    }
    if lend {
        flags |= GUNYAH_MEM_FORCE_LEND;
    }
    else {
        flags |= GUNYAH_MEM_FORCE_SHARE;
    }
    let region = gunyah_map_cma_mem_args {
        label: slot,
        guest_addr,
        flags,
        guest_mem_fd,
        offset,
        size,
    };
    // SAFETY: safe because the return value is checked.
    let ret = unsafe { ioctl_with_ref(vm, GH_VM_ANDROID_MAP_CMA_MEM, &region) };
    if ret == 0 {
        Ok(())
    } else {
        errno_result()
    }
}

#[derive(PartialEq, Eq, Hash)]
pub struct GunyahIrqRoute {
    irq: u32,
    level: bool,
}

/// Selects how host-visible virtio-gpu blobs (gfxstream RingBlob / ASG rings) are mapped into
/// the guest. Set via `--hypervisor "gunyah[blob_mode=...]"`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum GunyahBlobMode {
    /// Host SHAREs the blob and returns the RM memparcel handle; the guest accepts it into its
    /// own stage-2 with `gh_rm_mem_accept`. Required on host kernels that do NOT push a runtime
    /// SHARE into a *protected* guest's stage-2 (the device's 6.1 mainline gunyah). Needs the
    /// host `GH_VM_ANDROID_SHARE_BLOB` ioctl + the guest-side accept path.
    #[default]
    GuestAccept,
    /// Plain `add_memory_region`: a runtime `GH_VM_SET_USER_MEM_REGION` SHARE with EEXIST-reuse —
    /// the qemu-android-gunyah path. The guest never accepts (handle is `None`); it relies on the
    /// host kernel itself mapping the runtime SHARE into the running guest's stage-2 (6.6 vendor
    /// kernels with addrspace_map/memextent_donate). No custom ioctl, no guest-kernel change.
    HostShare,
}

pub struct GunyahVm {
    gh: Gunyah,
    vm: SafeDescriptor,
    vm_id: Option<u16>,
    pas_id: Option<u32>,
    guest_mem: GuestMemory,
    mem_regions: Arc<Mutex<BTreeMap<MemSlot, (Box<dyn MappedRegion>, GuestAddress)>>>,
    /// A min heap of MemSlot numbers that were used and then removed and can now be re-used
    mem_slot_gaps: Arc<Mutex<BinaryHeap<Reverse<MemSlot>>>>,
    /// Host mappings whose slots were "removed" at runtime but must be kept alive.
    ///
    /// Gunyah SHARE mappings (used for host-visible virtio-gpu blobs) are permanent and
    /// cannot be safely unshared, so we never munmap the host backing nor recycle the slot.
    /// See `remove_memory_region`.
    pinned_regions: Arc<Mutex<Vec<Box<dyn MappedRegion>>>>,
    /// Host backings for runtime-shared virtio-gpu blobs, keyed by label (BAR page).
    /// Re-sharing a label drops the previous backing (host already reclaimed it),
    /// so this does not grow with blob map/unmap churn.
    blob_regions: Arc<Mutex<BTreeMap<u32, Box<dyn MappedRegion>>>>,
    routes: Arc<Mutex<HashSet<GunyahIrqRoute>>>,
    hv_cfg: crate::Config,
    /// How host-visible virtio-gpu blobs are mapped into the guest (see [`GunyahBlobMode`]).
    blob_mode: GunyahBlobMode,
}

impl AsRawDescriptor for GunyahVm {
    fn as_raw_descriptor(&self) -> RawDescriptor {
        self.vm.as_raw_descriptor()
    }
}

impl GunyahVm {
    pub fn new(gh: &Gunyah, vm_id: Option<u16>, pas_id: Option<u32>, guest_mem: GuestMemory, cfg: Config, blob_mode: GunyahBlobMode) -> Result<GunyahVm> {
        // SAFETY:
        // Safe because we know gunyah is a real gunyah fd as this module is the only one that can
        // make Gunyah objects.
        let ret = unsafe { ioctl_with_val(gh, GH_CREATE_VM, 0 as c_ulong) };
        if ret < 0 {
            return errno_result();
        }

        // SAFETY:
        // Safe because we verify that ret is valid and we own the fd.
        let vm_descriptor = unsafe { SafeDescriptor::from_raw_descriptor(ret) };
        // Slot counter for chunked LEND: starts after the last region index.
        let mut next_lend_slot = guest_mem.num_regions() as usize;
        for region in guest_mem.regions() {
            let lend = if cfg.protection_type.isolates_memory() {
                match region.options.purpose {
                    MemoryRegionPurpose::Bios => true,
                    MemoryRegionPurpose::GuestMemoryRegion => true,
                    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
                    MemoryRegionPurpose::ProtectedFirmwareRegion => true,
                    MemoryRegionPurpose::ReservedMemory => true,
                    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
                    MemoryRegionPurpose::SharedFramebuffer => false,
                    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
                    MemoryRegionPurpose::StaticSwiotlbRegion => false,
                }
            } else {
                false
            };
            if region.options.file_backed.is_some() {
                map_cma_region(
                        &vm_descriptor,
                        region.index as MemSlot,
                        lend,
                        !region.options.file_backed.unwrap().writable,
                        region.guest_addr.offset(),
                        region.shm.as_raw_descriptor().try_into().unwrap(),
                        region.size.try_into().unwrap(),
                        region.shm_offset,
                )?;
            } else if lend {
                let region_size: u64 = region.size.try_into().unwrap();
                let host_ptr = region.host_addr as *mut u8;
                let guest_base = region.guest_addr.offset();

                if let Some(mthp_mode) = cfg.prepare_lend_mthp {
                    // Full mTHP preparation: drop caches, enable mTHP,
                    // populate in batches, cascading MADV_COLLAPSE, mlock.
                    // SAFETY: host_ptr is a valid mapping of region_size bytes.
                    let prep = unsafe { mthp::prepare_lend_region(host_ptr, region_size) };

                    let chunks = match mthp_mode {
                        // Single-parcel: keep the whole prepared region in one
                        // LEND (eager-parcel kernels, e.g. sm8650, would hit
                        // RM NORESOURCE if split into many parcels).
                        LendMthpMode::Single => Vec::new(),
                        // Chunked: split into <=256MB parcels (demand-paging
                        // kernels, e.g. sm8750).
                        LendMthpMode::Chunked => {
                            mthp::compute_lend_chunks(region_size, Some(&prep))
                        }
                    };
                    if chunks.is_empty() {
                        // Region small enough for a single LEND slot.
                        // SAFETY: guest regions are guaranteed not to overlap.
                        unsafe {
                            android_lend_user_memory_region(
                                &vm_descriptor,
                                region.index as MemSlot,
                                false,
                                guest_base,
                                region_size,
                                host_ptr,
                            )?;
                        }
                    } else {
                        // Chunked LEND – each chunk gets its own slot.
                        for (ci, chunk) in chunks.iter().enumerate() {
                            let slot = if ci == 0 {
                                region.index as MemSlot
                            } else {
                                next_lend_slot as MemSlot
                            };
                            if ci > 0 {
                                next_lend_slot += 1;
                            }
                            // SAFETY: chunks are non-overlapping sub-ranges
                            // within a single guest region.
                            unsafe {
                                android_lend_user_memory_region(
                                    &vm_descriptor,
                                    slot,
                                    false,
                                    guest_base + chunk.offset,
                                    chunk.size,
                                    host_ptr.add(chunk.offset as usize),
                                )?;
                            }
                        }
                    }
                } else {
                    // No mTHP preparation – simple single-slot LEND.
                    // SAFETY: guest regions are guaranteed not to overlap.
                    unsafe {
                        android_lend_user_memory_region(
                            &vm_descriptor,
                            region.index as MemSlot,
                            false,
                            guest_base,
                            region_size,
                            host_ptr,
                        )?;
                    }
                }
            } else {
                // SAFETY:
                // Safe because the guest regions are guarnteed not to overlap.
                unsafe {
                    set_user_memory_region(
                        &vm_descriptor,
                        region.index as MemSlot,
                        false,
                        // SHARE'd memory is executable only in non-protected VMs (where guest RAM
                        // itself is SHARE'd). In protected VMs these are data-only regions.
                        !cfg.protection_type.isolates_memory(),
                        region.guest_addr.offset(),
                        region.size.try_into().unwrap(),
                        region.host_addr as *mut u8,
                    )?;
                }
            }
        }

        warn!(
            "GUNYAH-BLOB-MODE: {}",
            match blob_mode {
                GunyahBlobMode::GuestAccept => "guest-accept (host SHARE + guest mem_accept)",
                GunyahBlobMode::HostShare =>
                    "host-share (qemu-style runtime SET_USER_MEM_REGION SHARE)",
            }
        );

        Ok(GunyahVm {
            gh: gh.try_clone()?,
            vm: vm_descriptor,
            vm_id,
            pas_id,
            guest_mem,
            mem_regions: Arc::new(Mutex::new(BTreeMap::new())),
            mem_slot_gaps: Arc::new(Mutex::new(BinaryHeap::new())),
            pinned_regions: Arc::new(Mutex::new(Vec::new())),
            blob_regions: Arc::new(Mutex::new(BTreeMap::new())),
            routes: Arc::new(Mutex::new(HashSet::new())),
            hv_cfg: cfg,
            blob_mode,
        })
    }

    pub fn set_vm_auth_type_to_qcom_trusted_vm(&self, payload_start: GuestAddress, payload_size: u64) -> Result<()> {
        let gunyah_qtvm_auth_arg = gunyah_qtvm_auth_arg {
            vm_id: self.vm_id.expect("VM ID not specified for a QTVM"),
            pas_id: self.pas_id.expect("PAS ID not specified for a QTVM"),
            // QTVMs have the metadata needed for authentication at the start of the guest addrspace.
            guest_phys_addr: payload_start.offset(),
            size: payload_size,
        };
        let gunyah_auth_desc = gunyah_auth_desc {
            type_: gunyah_auth_type_GUNYAH_QCOM_TRUSTED_VM_TYPE,
            arg_size: size_of::<gunyah_qtvm_auth_arg>() as u32,
            arg: &gunyah_qtvm_auth_arg as *const gunyah_qtvm_auth_arg as u64,
        };
        // SAFETY: safe because the return value is checked.
        let ret = unsafe { ioctl_with_ref(self, GH_VM_ANDROID_SET_AUTH_TYPE, &gunyah_auth_desc) };
        if ret == 0 {
            Ok(())
        } else {
            errno_result()
        }
    }

    fn create_vcpu(&self, id: usize) -> Result<GunyahVcpu> {
        let gh_fn_vcpu_arg = gh_fn_vcpu_arg {
            id: id.try_into().unwrap(),
        };

        let function_desc = gh_fn_desc {
            type_: GH_FN_VCPU,
            arg_size: size_of::<gh_fn_vcpu_arg>() as u32,
            // Safe because kernel is expecting pointer with non-zero arg_size
            arg: &gh_fn_vcpu_arg as *const gh_fn_vcpu_arg as u64,
        };

        // SAFETY:
        // Safe because we know that our file is a VM fd and we verify the return result.
        let fd = unsafe { ioctl_with_ref(self, GH_VM_ADD_FUNCTION, &function_desc) };
        if fd < 0 {
            return errno_result();
        }

        // SAFETY:
        // Wrap the vcpu now in case the following ? returns early. This is safe because we verified
        // the value of the fd and we own the fd.
        let vcpu = unsafe { File::from_raw_descriptor(fd) };

        // SAFETY:
        // Safe because we know this is a Gunyah VCPU
        let res = unsafe { ioctl(&vcpu, GH_VCPU_MMAP_SIZE) };
        if res < 0 {
            return errno_result();
        }
        let run_mmap_size = res as usize;

        let run_mmap = MemoryMappingBuilder::new(run_mmap_size)
            .from_file(&vcpu)
            .build()
            .map_err(|_| Error::new(ENOSPC))?;

        Ok(GunyahVcpu {
            vm: self.vm.try_clone()?,
            vcpu,
            id,
            run_mmap: Arc::new(run_mmap),
        })
    }

    pub fn register_irqfd(&self, label: u32, evt: &Event, level: bool) -> Result<()> {
        let gh_fn_irqfd_arg = gh_fn_irqfd_arg {
            fd: evt.as_raw_descriptor() as u32,
            label,
            flags: if level { GH_IRQFD_LEVEL } else { 0 },
            ..Default::default()
        };

        let function_desc = gh_fn_desc {
            type_: GH_FN_IRQFD,
            arg_size: size_of::<gh_fn_irqfd_arg>() as u32,
            // SAFETY:
            // Safe because kernel is expecting pointer with non-zero arg_size
            arg: &gh_fn_irqfd_arg as *const gh_fn_irqfd_arg as u64,
        };

        // SAFETY: safe because the return value is checked.
        let ret = unsafe { ioctl_with_ref(self, GH_VM_ADD_FUNCTION, &function_desc) };
        if ret == 0 {
            self.routes
                .lock()
                .insert(GunyahIrqRoute { irq: label, level });
            Ok(())
        } else {
            errno_result()
        }
    }

    pub fn unregister_irqfd(&self, label: u32, _evt: &Event) -> Result<()> {
        let gh_fn_irqfd_arg = gh_fn_irqfd_arg {
            label,
            ..Default::default()
        };

        let function_desc = gh_fn_desc {
            type_: GH_FN_IRQFD,
            arg_size: size_of::<gh_fn_irqfd_arg>() as u32,
            // Safe because kernel is expecting pointer with non-zero arg_size
            arg: &gh_fn_irqfd_arg as *const gh_fn_irqfd_arg as u64,
        };

        // SAFETY: safe because memory is not modified and the return value is checked.
        let ret = unsafe { ioctl_with_ref(self, GH_VM_REMOVE_FUNCTION, &function_desc) };
        if ret == 0 {
            Ok(())
        } else {
            errno_result()
        }
    }

    pub fn try_clone(&self) -> Result<Self>
    where
        Self: Sized,
    {
        Ok(GunyahVm {
            gh: self.gh.try_clone()?,
            vm: self.vm.try_clone()?,
            vm_id: self.vm_id,
            pas_id: self.pas_id,
            guest_mem: self.guest_mem.clone(),
            mem_regions: self.mem_regions.clone(),
            mem_slot_gaps: self.mem_slot_gaps.clone(),
            pinned_regions: self.pinned_regions.clone(),
            blob_regions: self.blob_regions.clone(),
            routes: self.routes.clone(),
            hv_cfg: self.hv_cfg,
            blob_mode: self.blob_mode,
        })
    }

    fn set_dtb_config(&self, fdt_address: GuestAddress, fdt_size: usize) -> Result<()> {
        let dtb_config = gh_vm_dtb_config {
            guest_phys_addr: fdt_address.offset(),
            size: fdt_size.try_into().unwrap(),
        };

        // SAFETY:
        // Safe because we know this is a Gunyah VM
        let ret = unsafe { ioctl_with_ref(self, GH_VM_SET_DTB_CONFIG, &dtb_config) };
        if ret == 0 {
            Ok(())
        } else {
            errno_result()
        }
    }

    fn set_protected_vm_firmware_ipa(&self, fw_addr: GuestAddress, fw_size: u64) -> Result<()> {
        let fw_config = gh_vm_firmware_config {
            guest_phys_addr: fw_addr.offset(),
            size: fw_size,
        };

        // SAFETY:
        // Safe because we know this is a Gunyah VM
        let ret = unsafe { ioctl_with_ref(self, GH_VM_ANDROID_SET_FW_CONFIG, &fw_config) };
        if ret == 0 {
            Ok(())
        } else {
            errno_result()
        }
    }

    fn set_boot_pc(&self, value: u64) -> Result<()> {
        self.set_boot_context(gh_vm_boot_context_reg::REG_SET_PC, 0, value)
    }

    // Sets the boot context for the Gunyah VM by specifying the register type, index, and value.
    fn set_boot_context(
        &self,
        reg_type: gh_vm_boot_context_reg::Type,
        reg_idx: u8,
        value: u64,
    ) -> Result<()> {
        let reg_id = boot_context_reg_id(reg_type, reg_idx);
        let boot_context = gh_vm_boot_context {
            reg: reg_id,
            value,
            ..Default::default()
        };

        // SAFETY: Safe because we ensure the boot_context is correctly initialized
        // and the ioctl call is checked.
        let ret = unsafe { ioctl_with_ref(self, GH_VM_SET_BOOT_CONTEXT, &boot_context) };
        if ret == 0 {
            Ok(())
        } else {
            errno_result()
        }
    }

    fn start(&self) -> Result<()> {
        // SAFETY: safe because memory is not modified and the return value is checked.
        let ret = unsafe { ioctl(self, GH_VM_START) };
        if ret == 0 {
            Ok(())
        } else {
            errno_result()
        }
    }

    fn handle_inflate(&self, guest_addr: GuestAddress, size: u64) -> Result<()> {
        let range = gunyah_address_range {
            guest_phys_addr: guest_addr.0,
            size,
        };

        // SAFETY: Safe because we know this is a Gunyah VM
        let ret = unsafe { ioctl_with_ref(self, GH_VM_RECLAIM_REGION, &range) };
        if ret != 0 {
            warn!("Gunyah failed to reclaim {:?}", range);
            return errno_result();
        }

        match self.guest_mem.remove_range(guest_addr, size) {
            Ok(_) => Ok(()),
            Err(vm_memory::Error::MemoryAccess(_, MmapError::SystemCallFailed(e))) => Err(e),
            Err(_) => Err(Error::new(EIO)),
        }
    }
}

impl Vm for GunyahVm {
    fn try_clone(&self) -> Result<Self>
    where
        Self: Sized,
    {
        Ok(GunyahVm {
            gh: self.gh.try_clone()?,
            vm: self.vm.try_clone()?,
            vm_id: self.vm_id,
            pas_id: self.pas_id,
            guest_mem: self.guest_mem.clone(),
            mem_regions: self.mem_regions.clone(),
            mem_slot_gaps: self.mem_slot_gaps.clone(),
            pinned_regions: self.pinned_regions.clone(),
            blob_regions: self.blob_regions.clone(),
            routes: self.routes.clone(),
            hv_cfg: self.hv_cfg,
            blob_mode: self.blob_mode,
        })
    }

    fn try_clone_descriptor(&self) -> Result<SafeDescriptor> {
        error!("try_clone_descriptor hasn't been tested on gunyah, returning -ENOTSUP");
        Err(Error::new(ENOTSUP))
    }

    fn hypervisor_kind(&self) -> HypervisorKind {
        HypervisorKind::Gunyah
    }

    fn check_capability(&self, c: VmCap) -> bool {
        match c {
            VmCap::DirtyLog => false,
            // Strictly speaking, Gunyah supports pvclock, but Gunyah takes care
            // of it and crosvm doesn't need to do anything for it
            VmCap::PvClock => false,
            VmCap::Protected => true,
            VmCap::EarlyInitCpuid => false,
            #[cfg(target_arch = "x86_64")]
            VmCap::BusLockDetect => false,
            VmCap::ReadOnlyMemoryRegion => false,
            VmCap::MemNoncoherentDma => false,
            #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
            VmCap::Sve => false,
        }
    }

    fn get_guest_phys_addr_bits(&self) -> u8 {
        40
    }

    fn get_memory(&self) -> &GuestMemory {
        &self.guest_mem
    }

    fn supports_blob_share(&self) -> bool {
        // `guest-accept` routes blobs through `share_blob` (host SHARE + guest mem_accept);
        // `host-share` falls back to `add_memory_region` (qemu-style runtime SHARE, no accept).
        self.blob_mode == GunyahBlobMode::GuestAccept
    }

    /// Runtime-SHARE a host blob to the running guest and return the resource-manager memparcel
    /// handle. Unlike `add_memory_region` (which SHARE's but never reaches a protected guest's
    /// stage-2 -> SIGBUS), this exposes the handle so the guest can `gh_rm_mem_accept` it itself
    /// at `guest_addr`. The host keeps access (SHARE), as gfxstream host-visible blobs require.
    /// The backing is pinned for the VM's lifetime (SHARE is permanent).
    fn share_blob(
        &mut self,
        guest_addr: GuestAddress,
        mem_region: Box<dyn MappedRegion>,
        read_only: bool,
    ) -> Result<u32> {
        let pgsz = pagesize() as u64;
        let size = (mem_region.size() as u64 + pgsz - 1) / pgsz * pgsz;

        // Deterministic label per BAR page: the same GPA reused over time (blobs are
        // mapped/freed at the same host-visible BAR offsets) maps to the same label, so the
        // host kernel reclaims the stale parcel before re-sharing the new backing. Distinct
        // concurrent blobs sit at distinct GPAs -> distinct labels (no false collision).
        let label = (guest_addr.offset() >> 12) as u32;

        // Host-visible blobs are data-only: READ|WRITE, never EXEC.
        let mut flags = GH_MEM_ALLOW_READ;
        if !read_only {
            flags |= GH_MEM_ALLOW_WRITE;
        }

        // The runtime-SHARE ioctl is served by the out-of-tree `gunyah_share_mod` module via its
        // own `/dev/gunyah_share` char device (the in-tree gh_vm_ioctl has no such case), so the
        // gunyah VM fd is passed as a request field rather than as the ioctl target.
        let share_dev = File::open("/dev/gunyah_share")
            .map_err(|e| Error::new(e.raw_os_error().unwrap_or(EINVAL)))?;

        let mut blob = ghsm_share_blob {
            vm_fd: self.vm.as_raw_descriptor(),
            label,
            flags,
            mem_handle: 0,
            guest_phys_addr: guest_addr.offset(),
            memory_size: size,
            userspace_addr: mem_region.as_ptr() as u64,
        };

        // SAFETY: the ioctl reads the request and writes back mem_handle into `blob`; the return
        // value is checked.
        let ret = unsafe { ioctl_with_mut_ref(&share_dev, GHSM_SHARE_BLOB, &mut blob) };
        if ret != 0 {
            return errno_result();
        }

        // Keep the host backing mapped while the parcel is shared, keyed by label. The host
        // kernel already reclaimed+unpinned the previous parcel for this label, so dropping the
        // old crosvm mapping here is safe and bounds RSS under blob map/unmap churn.
        let old = self.blob_regions.lock().insert(label, mem_region);
        drop(old);
        warn!(
            "GUNYAH-SHARE-BLOB: gpa=0x{:x} size=0x{:x} label={} handle=0x{:x}",
            guest_addr.offset(),
            size,
            label,
            blob.mem_handle,
        );
        Ok(blob.mem_handle)
    }

    fn unshare_blob(&mut self, label: u32) -> Result<()> {
        // The guest has unmapped this host-visible blob and (per the virtio-gpu guest driver)
        // already released its own stage-2 acceptance, so it is now safe to reclaim the SHARE on
        // the host side: the GHSM module does gh_rm_mem_reclaim + unpin and drops it from the VM's
        // memory_mappings list. This keeps host and guest in sync so the BAR offset can be reused
        // without colliding with a stale parcel (the old PIN no-op left it shared forever, forcing
        // an unsafe lazy overlap-reclaim that orphaned still-live parcels -> mem_share EINVAL).
        let share_dev = File::open("/dev/gunyah_share")
            .map_err(|e| Error::new(e.raw_os_error().unwrap_or(EINVAL)))?;

        let unshare = ghsm_unshare_blob {
            vm_fd: self.vm.as_raw_descriptor(),
            label,
        };

        // SAFETY: the ioctl only reads the request; the return value is checked.
        let ret = unsafe { ioctl_with_ref(&share_dev, GHSM_UNSHARE_BLOB, &unshare) };
        if ret != 0 {
            // ENOENT just means it was already reclaimed (e.g. by a prior overlap) -- not fatal.
            warn!("GUNYAH-UNSHARE-BLOB: label={} ioctl ret={}", label, ret);
        } else {
            warn!("GUNYAH-UNSHARE-BLOB: label={} reclaimed", label);
        }

        // Drop the host backing mapping we kept alive while shared.
        self.blob_regions.lock().remove(&label);
        Ok(())
    }

    fn add_memory_region(
        &mut self,
        guest_addr: GuestAddress,
        mem_region: Box<dyn MappedRegion>,
        read_only: bool,
        _log_dirty_pages: bool,
        _cache: MemCacheType,
    ) -> Result<MemSlot> {
        let pgsz = pagesize() as u64;
        // Gunyah require to set the user memory region with page size aligned size. Safe to extend
        // the mem.size() to be page size aligned because the mmap will round up the size to be
        // page size aligned if it is not.
        let size = (mem_region.size() as u64 + pgsz - 1) / pgsz * pgsz;
        let end_addr = guest_addr.checked_add(size).ok_or(Error::new(EOVERFLOW))?;

        if self.guest_mem.range_overlap(guest_addr, end_addr) {
            return Err(Error::new(ENOSPC));
        }

        let mut regions = self.mem_regions.lock();
        let mut gaps = self.mem_slot_gaps.lock();
        let slot = match gaps.pop() {
            Some(gap) => gap.0,
            None => (regions.len() + self.guest_mem.num_regions() as usize) as MemSlot,
        };

        // Diagnostic: log what we are about to SHARE. (Do not read the mapping here — for the
        // SingleMappingOnFirst BAR arena it is PROT_NONE until blobs are mapped into it.)
        warn!(
            "GUNYAH-ADD: slot={} gpa=0x{:x} share_size=0x{:x} region_size=0x{:x} hva={:p} exec={} ro={}",
            slot,
            guest_addr.offset(),
            size,
            mem_region.size(),
            mem_region.as_ptr(),
            !self.hv_cfg.protection_type.isolates_memory(),
            read_only,
        );

        // SAFETY: safe because memory is not modified and the return value is checked.
        let res = unsafe {
            set_user_memory_region(
                &self.vm,
                slot,
                read_only,
                // Host-visible virtio-gpu blobs (gfxstream ASG rings) are SHARE'd data — never
                // executable. In protected VMs, requesting exec on SHARE'd memory breaks the
                // stage-2 mapping and the guest SIGBUSes on access.
                !self.hv_cfg.protection_type.isolates_memory(),
                guest_addr.offset(),
                size,
                mem_region.as_ptr(),
            )
        };

        let res = if let Err(ref e) = res {
            if e.errno() == EEXIST {
                // Gunyah workaround: this GPA was already SHARE'd and a Gunyah SHARE is
                // permanent. This happens when the guest re-maps a host-visible virtio-gpu
                // blob at a BAR offset that was used before. We must NOT fall back to
                // android_lend here: LEND would hand the pages to the guest exclusively, so
                // the host (gfxstream) could no longer see them. Instead, treat EEXIST as a
                // successful re-use of the existing SHARE. The backing physical pages stay
                // stable because gfxstream pins RingBlob memory (GFXSTREAM_GUNYAH_PIN_RINGBLOB),
                // so the existing SHARE already points at the correct pages.
                warn!(
                    "Gunyah set_user_memory_region returned EEXIST for slot {} \
                     at GPA 0x{:x} size 0x{:x}; reusing existing SHARE",
                    slot,
                    guest_addr.offset(),
                    size,
                );
                Ok(())
            } else {
                res
            }
        } else {
            res
        };

        if let Err(e) = res {
            gaps.push(Reverse(slot));
            return Err(e);
        }
        warn!(
            "GUNYAH-ADD: SHARE established slot={} gpa=0x{:x} size=0x{:x}",
            slot,
            guest_addr.offset(),
            size,
        );
        regions.insert(slot, (mem_region, guest_addr));
        Ok(slot)
    }

    fn msync_memory_region(&mut self, slot: MemSlot, offset: usize, size: usize) -> Result<()> {
        let mut regions = self.mem_regions.lock();
        let (mem, _) = regions.get_mut(&slot).ok_or_else(|| Error::new(ENOENT))?;

        mem.msync(offset, size).map_err(|err| match err {
            MmapError::InvalidAddress => Error::new(EFAULT),
            MmapError::NotPageAligned => Error::new(EINVAL),
            MmapError::SystemCallFailed(e) => e,
            _ => Error::new(EIO),
        })
    }

    fn madvise_pageout_memory_region(
        &mut self,
        _slot: MemSlot,
        _offset: usize,
        _size: usize,
    ) -> Result<()> {
        Err(Error::new(ENOTSUP))
    }

    fn madvise_remove_memory_region(
        &mut self,
        _slot: MemSlot,
        _offset: usize,
        _size: usize,
    ) -> Result<()> {
        Err(Error::new(ENOTSUP))
    }

    fn remove_memory_region(&mut self, slot: MemSlot) -> Result<Box<dyn MappedRegion>> {
        let mut regions = self.mem_regions.lock();
        let (region, guest_addr) = regions.remove(&slot).ok_or_else(|| Error::new(ENOENT))?;

        // Gunyah workaround: a Gunyah SHARE is permanent and cannot be reliably unshared.
        // Removing it and later re-SHARE'ing a different physical page at the same GPA makes
        // the guest read stale data. So instead of unsharing, we:
        //   * skip the memory_size=0 ioctl (it does not reliably unshare on Gunyah),
        //   * keep the host mapping alive forever so its pages are never munmap'd (the
        //     gfxstream-side RingBlob backing is likewise pinned), and
        //   * do NOT recycle the slot, so a later mapping is given a fresh slot/GPA.
        // The caller is handed a tiny placeholder mapping to satisfy the API; dropping it is
        // harmless and does not affect the still-active SHARE.
        warn!(
            "Gunyah: not unsharing slot {} (GPA 0x{:x}); keeping host mapping alive \
             (SHARE is permanent)",
            slot,
            guest_addr.offset(),
        );
        self.pinned_regions.lock().push(region);

        let placeholder = MemoryMappingBuilder::new(pagesize())
            .build()
            .map_err(|_| Error::new(EINVAL))?;
        Ok(Box::new(placeholder))
    }

    fn create_device(&self, _kind: DeviceKind) -> Result<SafeDescriptor> {
        unimplemented!()
    }

    fn get_dirty_log(&self, _slot: MemSlot, _dirty_log: &mut [u8]) -> Result<()> {
        unimplemented!()
    }

    fn register_ioevent(
        &mut self,
        evt: &Event,
        addr: IoEventAddress,
        datamatch: Datamatch,
    ) -> Result<()> {
        let (do_datamatch, datamatch_value, datamatch_len) = match datamatch {
            Datamatch::AnyLength => (false, 0, 0),
            Datamatch::U8(v) => match v {
                Some(u) => (true, u as u64, 1),
                None => (false, 0, 1),
            },
            Datamatch::U16(v) => match v {
                Some(u) => (true, u as u64, 2),
                None => (false, 0, 2),
            },
            Datamatch::U32(v) => match v {
                Some(u) => (true, u as u64, 4),
                None => (false, 0, 4),
            },
            Datamatch::U64(v) => match v {
                Some(u) => (true, u, 8),
                None => (false, 0, 8),
            },
        };

        let mut flags = 0;
        if do_datamatch {
            flags |= 1 << GH_IOEVENTFD_DATAMATCH;
        }

        let maddr = if let IoEventAddress::Mmio(maddr) = addr {
            maddr
        } else {
            todo!()
        };

        let gh_fn_ioeventfd_arg = gh_fn_ioeventfd_arg {
            fd: evt.as_raw_descriptor(),
            datamatch: datamatch_value,
            len: datamatch_len,
            addr: maddr,
            flags,
            ..Default::default()
        };

        let function_desc = gh_fn_desc {
            type_: GH_FN_IOEVENTFD,
            arg_size: size_of::<gh_fn_ioeventfd_arg>() as u32,
            arg: &gh_fn_ioeventfd_arg as *const gh_fn_ioeventfd_arg as u64,
        };

        // SAFETY: safe because memory is not modified and the return value is checked.
        let ret = unsafe { ioctl_with_ref(self, GH_VM_ADD_FUNCTION, &function_desc) };
        if ret == 0 {
            Ok(())
        } else {
            errno_result()
        }
    }

    fn unregister_ioevent(
        &mut self,
        _evt: &Event,
        addr: IoEventAddress,
        _datamatch: Datamatch,
    ) -> Result<()> {
        let maddr = if let IoEventAddress::Mmio(maddr) = addr {
            maddr
        } else {
            todo!()
        };

        let gh_fn_ioeventfd_arg = gh_fn_ioeventfd_arg {
            addr: maddr,
            ..Default::default()
        };

        let function_desc = gh_fn_desc {
            type_: GH_FN_IOEVENTFD,
            arg_size: size_of::<gh_fn_ioeventfd_arg>() as u32,
            arg: &gh_fn_ioeventfd_arg as *const gh_fn_ioeventfd_arg as u64,
        };

        // SAFETY: safe because memory is not modified and the return value is checked.
        let ret = unsafe { ioctl_with_ref(self, GH_VM_REMOVE_FUNCTION, &function_desc) };
        if ret == 0 {
            Ok(())
        } else {
            errno_result()
        }
    }

    fn handle_io_events(&self, _addr: IoEventAddress, _data: &[u8]) -> Result<()> {
        Ok(())
    }

    fn get_pvclock(&self) -> Result<ClockState> {
        unimplemented!()
    }

    fn set_pvclock(&self, _state: &ClockState) -> Result<()> {
        unimplemented!()
    }

    fn add_fd_mapping(
        &mut self,
        slot: u32,
        offset: usize,
        size: usize,
        fd: &dyn AsRawDescriptor,
        fd_offset: u64,
        prot: Protection,
    ) -> Result<()> {
        let mut regions = self.mem_regions.lock();
        let (region, _) = regions.get_mut(&slot).ok_or_else(|| Error::new(EINVAL))?;

        match region.add_fd_mapping(offset, size, fd, fd_offset, prot) {
            Ok(()) => Ok(()),
            Err(MmapError::SystemCallFailed(e)) => Err(e),
            Err(_) => Err(Error::new(EIO)),
        }
    }

    fn remove_mapping(&mut self, slot: u32, offset: usize, size: usize) -> Result<()> {
        let mut regions = self.mem_regions.lock();
        let (region, _) = regions.get_mut(&slot).ok_or_else(|| Error::new(EINVAL))?;

        match region.remove_mapping(offset, size) {
            Ok(()) => Ok(()),
            Err(MmapError::SystemCallFailed(e)) => Err(e),
            Err(_) => Err(Error::new(EIO)),
        }
    }

    fn handle_balloon_event(&mut self, event: BalloonEvent) -> Result<()> {
        match event {
            BalloonEvent::Inflate(m) => self.handle_inflate(m.guest_address, m.size),
            BalloonEvent::Deflate(m) => Ok(()),
            BalloonEvent::BalloonTargetReached(_) => Ok(()),
        }
    }
}

const GH_RM_EXIT_TYPE_VM_EXIT: u16 = 0;
const GH_RM_EXIT_TYPE_PSCI_POWER_OFF: u16 = 1;
const GH_RM_EXIT_TYPE_PSCI_SYSTEM_RESET: u16 = 2;
const GH_RM_EXIT_TYPE_PSCI_SYSTEM_RESET2: u16 = 3;
const GH_RM_EXIT_TYPE_WDT_BITE: u16 = 4;
const GH_RM_EXIT_TYPE_HYP_ERROR: u16 = 5;
const GH_RM_EXIT_TYPE_ASYNC_EXT_ABORT: u16 = 6;
const GH_RM_EXIT_TYPE_VM_FORCE_STOPPED: u16 = 7;

pub struct GunyahVcpu {
    vm: SafeDescriptor,
    vcpu: File,
    id: usize,
    run_mmap: Arc<MemoryMapping>,
}

struct GunyahVcpuSignalHandle {
    run_mmap: Arc<MemoryMapping>,
}

impl VcpuSignalHandleInner for GunyahVcpuSignalHandle {
    fn signal_immediate_exit(&self) {
        // SAFETY: we ensure `run_mmap` is a valid mapping of `kvm_run` at creation time, and the
        // `Arc` ensures the mapping still exists while we hold a reference to it.
        unsafe {
            let run = self.run_mmap.as_ptr() as *mut gh_vcpu_run;
            (*run).immediate_exit = 1;
        }
    }
}

impl AsRawDescriptor for GunyahVcpu {
    fn as_raw_descriptor(&self) -> RawDescriptor {
        self.vcpu.as_raw_descriptor()
    }
}

impl Vcpu for GunyahVcpu {
    fn try_clone(&self) -> Result<Self>
    where
        Self: Sized,
    {
        let vcpu = self.vcpu.try_clone()?;

        Ok(GunyahVcpu {
            vm: self.vm.try_clone()?,
            vcpu,
            id: self.id,
            run_mmap: self.run_mmap.clone(),
        })
    }

    fn as_vcpu(&self) -> &dyn Vcpu {
        self
    }

    fn run(&mut self) -> Result<VcpuExit> {
        // SAFETY:
        // Safe because we know our file is a VCPU fd and we verify the return result.
        let ret = unsafe { ioctl(self, GH_VCPU_RUN) };
        if ret != 0 {
            return errno_result();
        }

        // SAFETY:
        // Safe because we know we mapped enough memory to hold the gh_vcpu_run struct
        // because the kernel told us how large it is.
        let run = unsafe { &mut *(self.run_mmap.as_ptr() as *mut gh_vcpu_run) };
        match run.exit_reason {
            GH_VCPU_EXIT_MMIO => Ok(VcpuExit::Mmio),
            GH_VCPU_EXIT_STATUS => {
                // SAFETY:
                // Safe because the exit_reason (which comes from the kernel) told us which
                // union field to use.
                let status = unsafe { &mut run.__bindgen_anon_1.status };
                match status.status {
                    GH_VM_STATUS_GH_VM_STATUS_LOAD_FAILED => Ok(VcpuExit::FailEntry {
                        hardware_entry_failure_reason: 0,
                    }),
                    GH_VM_STATUS_GH_VM_STATUS_CRASHED => Ok(VcpuExit::SystemEventCrash),
                    GH_VM_STATUS_GH_VM_STATUS_EXITED => {
                        info!("exit type {}", status.exit_info.type_);
                        match status.exit_info.type_ {
                            GH_RM_EXIT_TYPE_VM_EXIT => Ok(VcpuExit::SystemEventShutdown),
                            GH_RM_EXIT_TYPE_PSCI_POWER_OFF => Ok(VcpuExit::SystemEventShutdown),
                            GH_RM_EXIT_TYPE_PSCI_SYSTEM_RESET => Ok(VcpuExit::SystemEventReset),
                            GH_RM_EXIT_TYPE_PSCI_SYSTEM_RESET2 => Ok(VcpuExit::SystemEventReset),
                            GH_RM_EXIT_TYPE_WDT_BITE => Ok(VcpuExit::SystemEventCrash),
                            GH_RM_EXIT_TYPE_HYP_ERROR => Ok(VcpuExit::SystemEventCrash),
                            GH_RM_EXIT_TYPE_ASYNC_EXT_ABORT => Ok(VcpuExit::SystemEventCrash),
                            GH_RM_EXIT_TYPE_VM_FORCE_STOPPED => Ok(VcpuExit::SystemEventShutdown),
                            r => {
                                warn!("Unknown exit type: {}", r);
                                Err(Error::new(EINVAL))
                            }
                        }
                    }
                    r => {
                        warn!("Unknown vm status: {}", r);
                        Err(Error::new(EINVAL))
                    }
                }
            }
            GH_VCPU_EXIT_PAGE_FAULT => {
                let pf = unsafe { &run.__bindgen_anon_1.page_fault };
                warn!("page fault at {:#x}, attempt: {}", pf.phys_addr, pf.attempt);
                Err(Error::new(-pf.attempt))
            }
            r => {
                warn!("unknown gh exit reason: {}", r);
                Err(Error::new(EINVAL))
            }
        }
    }

    fn id(&self) -> usize {
        self.id
    }

    fn set_immediate_exit(&self, exit: bool) {
        // SAFETY:
        // Safe because we know we mapped enough memory to hold the kvm_run struct because the
        // kernel told us how large it was. The pointer is page aligned so casting to a different
        // type is well defined, hence the clippy allow attribute.
        let run = unsafe { &mut *(self.run_mmap.as_ptr() as *mut gh_vcpu_run) };
        run.immediate_exit = exit.into();
    }

    fn signal_handle(&self) -> VcpuSignalHandle {
        VcpuSignalHandle {
            inner: Box::new(GunyahVcpuSignalHandle {
                run_mmap: self.run_mmap.clone(),
            }),
        }
    }

    fn handle_mmio(&self, handle_fn: &mut dyn FnMut(IoParams) -> Result<()>) -> Result<()> {
        // SAFETY:
        // Safe because we know we mapped enough memory to hold the gh_vcpu_run struct because the
        // kernel told us how large it was. The pointer is page aligned so casting to a different
        // type is well defined
        let run = unsafe { &mut *(self.run_mmap.as_ptr() as *mut gh_vcpu_run) };
        // Verify that the handler is called in the right context.
        assert!(run.exit_reason == GH_VCPU_EXIT_MMIO);
        // SAFETY:
        // Safe because the exit_reason (which comes from the kernel) told us which
        // union field to use.
        let mmio = unsafe { &mut run.__bindgen_anon_1.mmio };
        let address = mmio.phys_addr;
        let data = &mut mmio.data[..mmio.len as usize];
        if mmio.is_write != 0 {
            handle_fn(IoParams {
                address,
                operation: IoOperation::Write(data),
            })
        } else {
            handle_fn(IoParams {
                address,
                operation: IoOperation::Read(data),
            })
        }
    }

    fn handle_io(&self, _handle_fn: &mut dyn FnMut(IoParams)) -> Result<()> {
        unreachable!()
    }

    fn on_suspend(&self) -> Result<()> {
        Ok(())
    }

    unsafe fn enable_raw_capability(&self, _cap: u32, _args: &[u64; 4]) -> Result<()> {
        unimplemented!()
    }
}
