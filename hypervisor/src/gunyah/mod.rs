// Copyright 2023 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
mod aarch64;

mod gunyah_sys;
mod mthp;
pub mod shim_abi;
mod pin;
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
use base::ioctl_with_ref;
use base::ioctl_with_val;
use base::pagesize;
use base::warn;
use base::Error;
use base::FromRawDescriptor;
use base::IntoRawDescriptor;
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
        { base::error!("GH-DIAG: GH_VM_ANDROID_LEND_USER_MEM failed"); errno_result() }
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

    let ret = ioctl_with_ref(vm, GH_VM_SET_USER_MEM_REGION, &region);
    if ret == 0 {
        Ok(())
    } else {
        { base::error!("GH-DIAG: GH_VM_SET_USER_MEM_REGION failed"); errno_result() }
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
        { base::error!("GH-DIAG: GH_VM_ANDROID_MAP_CMA_MEM failed"); errno_result() }
    }
}

#[derive(PartialEq, Eq, Hash)]
pub struct GunyahIrqRoute {
    irq: u32,
    level: bool,
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
    /// Long-term pins held over runtime-shared blobs between the SHARE ioctl and the guest's
    /// accept, keyed by the same label as `blob_regions`. See `pin.rs`.
    blob_pins: Arc<Mutex<BTreeMap<u32, pin::LongtermPin>>>,
    routes: Arc<Mutex<HashSet<GunyahIrqRoute>>>,
    hv_cfg: crate::Config,
}

impl AsRawDescriptor for GunyahVm {
    fn as_raw_descriptor(&self) -> RawDescriptor {
        self.vm.as_raw_descriptor()
    }
}

impl GunyahVm {
    pub fn new(gh: &Gunyah, vm_id: Option<u16>, pas_id: Option<u32>, guest_mem: GuestMemory, cfg: Config) -> Result<GunyahVm> {
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
                    // The window: neither lent nor shared before boot. It is handed over after
                    // GH_VM_START as a memparcel the guest accepts, which is the whole point.
                    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
                    MemoryRegionPurpose::SharedGuestRam => false,
                    // The handoff page is SHARE'd like a pool, because the host has to keep
                    // writing to it after the VM has started.
                    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
                    MemoryRegionPurpose::ShimHandoff => false,
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
                        | MemoryRegionPurpose::ShimHandoff
                // The window of a pseudo-unprotected VM is not handed over here at all. It is
                // SHARE'd after GH_VM_START, because the guest has to accept it itself and there
                // is no guest to accept anything until then.
                //
                // What does happen here is the folio preparation, and it is not optional: the
                // reserve pool serves order-9 folios, and a parcel built from 4 KiB pages carries
                // one mem_entry per page -- a 4 GiB window would be a million of them. The same
                // preparation the LEND'd guest RAM of an ordinary protected VM gets, for the same
                // reason.
                #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
                if region.options.purpose == MemoryRegionPurpose::SharedGuestRam {
                    // SAFETY: host_addr is a valid mapping of region.size bytes.
                    let prep = unsafe {
                        mthp::prepare_lend_region(region.host_addr as *mut u8, full_size)
                    };
                    if !prep.populated || !prep.mlocked {
                        error!(
                            "GH-SHIM: window gpa={:#x} size={:#x} could not be populated and \
                             pinned (reserve pool exhausted?) -- refusing to start, because the \
                             share would either fail or hand the guest memory the host may still \
                             move under it",
                            region.guest_addr.offset(),
                            full_size,
                        );
                        return Err(Error::new(libc::ENOMEM));
                    }
                    base::info!(
                        "GH-SHIM: window gpa={:#x} size={:#x} prepared; it is shared after \
                         GH_VM_START and accepted by the shim",
                // SAFETY:
                // Safe because the guest regions are guarnteed not to overlap.
                unsafe {
                    set_user_memory_region(
                        &vm_descriptor,
                        region.index as MemSlot,
                        false,
                        region.guest_addr.offset(),
                        full_size,
                    );
                    continue;
                }

                // Same pin probe as the LEND path above: a SHARE'd pool whose pages cannot be
                // pinned takes the host down inside the ioctl, so find out here instead.
                let _pin = if share_len != 0 {
                    pin::LongtermPin::ensure_pinnable(
                        region.host_addr as u64,
                        share_len,
                        pin::PinSite::PreBoot,
                    )
                    .map_err(|e| {
                        error!(
                            "GH: refusing to SHARE gpa={:#x} size={:#x} -- the host cannot pin it",
                            region.guest_addr.offset(),
                            share_len
                        );
                        e
                    })?
                } else {
                    None
                };
                }
            }
        }

        Ok(GunyahVm {
            gh: gh.try_clone()?,
            vm: vm_descriptor,
            vm_id,
            pas_id,
            guest_mem,
            mem_regions: Arc::new(Mutex::new(BTreeMap::new())),
            mem_slot_gaps: Arc::new(Mutex::new(BinaryHeap::new())),
            blob_pins: Arc::new(Mutex::new(BTreeMap::new())),
            routes: Arc::new(Mutex::new(HashSet::new())),
            hv_cfg: cfg,
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
            { base::error!("GH-DIAG: GH_VM_ANDROID_SET_AUTH_TYPE failed"); errno_result() }
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
        if res < 0 { base::error!("GH-DIAG: GH_VCPU_MMAP_SIZE failed"); }
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
            { base::error!("GH-DIAG: GH_VM_ADD_FUNCTION failed"); errno_result() }
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
            { base::error!("GH-DIAG: GH_VM_REMOVE_FUNCTION failed"); errno_result() }
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
            blob_pins: self.blob_pins.clone(),
            routes: self.routes.clone(),
            hv_cfg: self.hv_cfg,
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
            { base::error!("GH-DIAG: GH_VM_SET_DTB_CONFIG failed"); errno_result() }
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
            { base::error!("GH-DIAG: GH_VM_ANDROID_SET_FW_CONFIG failed"); errno_result() }
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
            { base::error!("GH-DIAG: GH_VM_SET_BOOT_CONTEXT failed"); errno_result() }
        }
    }

    fn start(&self) -> Result<()> {
        // SAFETY: safe because memory is not modified and the return value is checked.
        let ret = unsafe { ioctl(self, GH_VM_START) };
        if ret == 0 {
            self.share_probe();
            self.share_guest_ram_window()?;
            Ok(())
        } else {
            base::error!("GH-DIAG: GH_VM_START failed");
            errno_result()
        }
    }

    /// Hand the guest's RAM over, now that there is a VM to hand it to.
    ///
    /// This is the whole of the pseudo-unprotected mode on the host side. The window has been
    /// sitting in a memfd with the payload already written into it; sharing it makes a memparcel
    /// the guest can accept, and the handles go in the handoff page for the shim to find. Nothing
    /// here can happen earlier: a memparcel handle does not exist until the VM does.
    ///
    /// The order matters at the end. `ready` is written last, after every parcel is shared and
    /// every handle recorded, because the shim spins on it and would otherwise read a handle of
    /// zero and ask the resource manager to accept nothing.
    fn share_guest_ram_window(&self) -> Result<()> {
        #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
        {
            use crate::gunyah::shim_abi as abi;

            if !self.hv_cfg.protection_type.shares_guest_ram() {
                return Ok(());
            }
            let mem = self.guest_mem.clone();
            let window = mem
                .regions()
                .find(|r| r.options.purpose == MemoryRegionPurpose::SharedGuestRam);
            let handoff = mem
                .regions()
                .find(|r| r.options.purpose == MemoryRegionPurpose::ShimHandoff);
            let (Some(window), Some(handoff)) = (window, handoff) else {
                base::error!("GH-SHIM: no window or no handoff region; refusing to start");
                return Err(Error::new(EINVAL));
            };

            let size: u64 = window.size.try_into().unwrap();
            let ho = handoff.guest_addr;

            // DROIDVM_SHIM_PARCEL_MB: hand the window over in parcels of at most this many MiB.
            //
            // Zero, the default, means one parcel however big the window is. It is worth a knob
            // because the cost of building a parcel is not the same everywhere: on android14-6.1
            // the resource manager assembles it from every folio there and then (~3.4 s for
            // 4 GiB), while the 6.12 driver demand-pages it (~60 ms). Splitting spends memparcels
            // -- MAX_MEMPARCEL_PER_VM is 1024 for the whole VM, shared with Android's own -- and
            // buys nothing unless something is measured to be faster for it, so nobody gets it
            // without asking.
            let chunk = std::env::var("DROIDVM_SHIM_PARCEL_MB")
                .ok()
                .and_then(|v| v.parse::<u64>().ok())
                .map(|mb| mb << 20)
                .filter(|c| *c != 0)
                // A parcel is built out of folios; a chunk that is not a whole number of them
                // would hand the guest a boundary in the middle of one.
                .map(|c| c.next_multiple_of(2 << 20).min(size))
                .unwrap_or(size);
            let nparcels = size.div_ceil(chunk);
            if nparcels > abi::SHIM_MAX_PARCELS as u64 {
                base::error!(
                    "GH-SHIM: {:#x} of window in {:#x} chunks needs {} parcels; the handoff page                      holds {}",
                    size,
                    chunk,
                    nparcels,
                    abi::SHIM_MAX_PARCELS,
                );
                return Err(Error::new(EINVAL));
            }

            let started = std::time::Instant::now();
            for i in 0..nparcels {
                let off = i * chunk;
                let len = chunk.min(size - off);
                let at = window
                    .guest_addr
                    .checked_add(off)
                    .ok_or_else(|| Error::new(EINVAL))?;
                // A second mapping of the region's own memfd, because share_blob wants something
                // it can hold for as long as the parcel lives. The payload the host wrote into it
                // before the VM started is already there and stays there: sharing does not
                // sanitise, which is what lets the window arrive with a kernel in it.
                let dup = base::clone_descriptor(&base::Descriptor(window.shm.as_raw_descriptor()))
                    .map_err(|_| Error::new(EINVAL))?;
                // SAFETY: the descriptor was just duplicated from the region's own memfd and is
                // owned by this File from here on.
                let file = unsafe { std::fs::File::from_raw_descriptor(dup.into_raw_descriptor()) };
                let mapping = MemoryMappingBuilder::new(len.try_into().unwrap())
                    .from_file(&file)
                    .offset(window.shm_offset + off)
                    .build()
                    .map_err(|e| {
                        base::error!("GH-SHIM: cannot map the window to share it: {:?}", e);
                        Error::new(EINVAL)
                    })?;
                // exec: the payload runs from here.
                let parcel = abi::ShimParcel {
                    handle,
                    reserved: 0,
                    base: at.offset(),
                    size: len,
                };
                let dst = ho
                    .checked_add(core::mem::offset_of!(abi::ShimHandoff, parcel) as u64)
                    .and_then(|a| {
                        a.checked_add(i * core::mem::size_of::<abi::ShimParcel>() as u64)
                    })
                    .ok_or_else(|| Error::new(EINVAL))?;
                // SAFETY: ShimParcel is repr(C) and made of integers; every byte pattern is valid.
                let bytes = unsafe {
                    std::slice::from_raw_parts(
                        &parcel as *const abi::ShimParcel as *const u8,
                        core::mem::size_of::<abi::ShimParcel>(),
                    )
                };
                mem.write_all_at_addr(bytes, dst)
                    .map_err(|_| Error::new(EINVAL))?;
            }
            base::info!(
                "GH-SHIM: window {:#x}+{:#x} shared as {} parcel(s) in {:?}",
                window.guest_addr.offset(),
                size,
                nparcels,
                started.elapsed(),
            );

            mem.write_obj_at_addr(
                nparcels as u32,
                ho.checked_add(core::mem::offset_of!(abi::ShimHandoff, nparcels) as u64)
                    .ok_or_else(|| Error::new(EINVAL))?,
            )
            .map_err(|_| Error::new(EINVAL))?;
            // Last, and only now.
            mem.write_obj_at_addr(
                1u64,
                ho.checked_add(core::mem::offset_of!(abi::ShimHandoff, ready) as u64)
                    .ok_or_else(|| Error::new(EINVAL))?,
            )
            .map_err(|_| Error::new(EINVAL))?;

            watch_shim(mem, ho);
        }
        Ok(())
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

/// Say what the shim did.
///
/// Everything the shim can go wrong at happens before there is a console, an interrupt controller
/// or anyone listening, so the handoff page is the only way it can speak. Without something
/// reading it, a shim that refuses to hand over is a VM that sits there: no output, no exit, no
/// clue. This turns that into one log line.
///
/// It watches from a thread because the vcpus have not started yet when the window is shared --
/// they are started by the caller, some way further up -- and stops as soon as the shim reaches
/// the point where it jumps, which is the last thing it does.
#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
fn watch_shim(mem: GuestMemory, handoff: GuestAddress) {
    use crate::gunyah::shim_abi as abi;

    std::thread::Builder::new()
        .name("gh_shim_watch".into())
        .spawn(move || {
            let status_at = handoff.unchecked_add(core::mem::offset_of!(abi::ShimHandoff, status) as u64);
            let error_at = handoff.unchecked_add(core::mem::offset_of!(abi::ShimHandoff, error) as u64);
            let probe_at =
                handoff.unchecked_add(core::mem::offset_of!(abi::ShimHandoff, exec_probe) as u64);
            let msg_at = handoff.unchecked_add(core::mem::offset_of!(abi::ShimHandoff, msg) as u64);
            let mut last = 0u64;
            // Ten seconds is far longer than the shim's own timeouts; past that it is not slow,
            // it is not running, and the thread has nothing left to add.
            for _ in 0..1000 {
                // Volatile: this is a poll of memory another CPU is writing, and the read has to
                // happen every time round rather than once.
                let status: u64 = mem.read_obj_from_addr_volatile(status_at).unwrap_or(0);
                if status != last {
                    last = status;
                    match status {
                        abi::SHIM_STATUS_RUNNING => base::info!("GH-SHIM: shim is running"),
                        abi::SHIM_STATUS_ACCEPTED => {
                            let probe: u64 = mem.read_obj_from_addr(probe_at).unwrap_or(0);
                            if probe != 0 {
                                base::info!(
                                    "GH-SHIM: window accepted; the guest executed out of it and \
                                     got {} (42 means yes)",
                                    probe
                                );
                            } else {
                                base::info!("GH-SHIM: window accepted");
                            }
                        }
                        abi::SHIM_STATUS_DT_DONE => base::info!("GH-SHIM: /memory now names the window"),
                        abi::SHIM_STATUS_JUMPING => {
                            base::info!("GH-SHIM: handing over to the payload");
                            return;
                        }
                        s if s & abi::SHIM_STATUS_ERROR != 0 => {
                            let mut msg = [0u8; 256];
                            let _ = mem.read_exact_at_addr(&mut msg, msg_at);
                            let end = msg.iter().position(|&c| c == 0).unwrap_or(msg.len());
                            let error: u64 = mem.read_obj_from_addr(error_at).unwrap_or(0);
                            base::error!(
                                "GH-SHIM: gave up: {} (error {:#x})",
                                String::from_utf8_lossy(&msg[..end]),
                                error,
                            );
                            return;
                        }
                        other => base::info!("GH-SHIM: status {:#x}", other),
                    }
                }
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
            if last == 0 {
                base::error!(
                    "GH-SHIM: nothing was written to the handoff page in ten seconds -- the shim \
                     never ran, or never found it"
                );
            }
        })
        .map(|_| ())
        .unwrap_or_else(|e| base::warn!("GH-SHIM: could not watch the handoff page: {}", e));
}

    /// Diagnostic: SHARE scratch regions at explicitly chosen GPAs, right after GH_VM_START, and
    /// report what the host side says. Driven by
    ///
    ///   GH_SHARE_PROBE=<gpa>[:<size_kib>][,<gpa>[:<size_kib>]]...   (hex or decimal GPAs)
    ///
    /// This exists to test the claim behind virtio_gpu's `host_visible_guard`: that a blob landing
    /// exactly at the host-visible BAR base cannot be shared. It deliberately involves no GPU, no
    /// blob and no allocator -- just gh_vm_mem_alloc + gh_rm_mem_share for a GPA we pick. Each
    /// scratch page is filled with a recognisable pattern (the GPA itself, repeated) so a guest
    /// that accepts the parcel can prove the mapping really landed.
    ///
    /// The parcels are left shared (and the handles printed) so a guest-side test module can accept
    /// them at the same GPA; nothing else in crosvm knows about these regions, so they are never
    /// handed out to a device.
    fn share_probe(&self) {
        let Some(spec) = std::env::var_os("GH_SHARE_PROBE") else {
            return;
        };
        let spec = spec.to_string_lossy().to_string();
        for item in spec.split(',').filter(|s| !s.trim().is_empty()) {
            let mut parts = item.trim().split(':');
            let gpa_str = parts.next().unwrap_or("").trim();
            let size_kib: u64 = parts
                .next()
                .and_then(|v| v.trim().parse::<u64>().ok())
                .unwrap_or(2048);
            let gpa = match gpa_str.strip_prefix("0x").or_else(|| gpa_str.strip_prefix("0X")) {
                Some(hex) => u64::from_str_radix(hex, 16).ok(),
                None => gpa_str.parse::<u64>().ok(),
            };
            let Some(gpa) = gpa else {
                base::error!("GH-SHARE-PROBE: cannot parse gpa {:?}", gpa_str);
                continue;
            };
            let size = size_kib * 1024;

            let region = match MemoryMappingBuilder::new(size as usize).build() {
                Ok(m) => m,
                Err(e) => {
                    base::error!("GH-SHARE-PROBE: gpa={:#x} scratch mmap failed: {:?}", gpa, e);
                    continue;
                }
            };
            // Pattern: the GPA repeated, so the guest can tell whose page it is looking at.
            let pattern: Vec<u8> = gpa.to_le_bytes().repeat(512);
            let _ = region.write_slice(&pattern, 0);

                Ok(handle) => base::error!(
                    "GH-SHARE-PROBE: gpa={:#x} size={:#x} => SHARED handle={:#x} \
                     (accept it in the guest with gunyah_guest_mem_accept)",
                    gpa,
                    size,
                    handle
                ),
                Err(e) => base::error!(
                    "GH-SHARE-PROBE: gpa={:#x} size={:#x} => FAILED {:?}",
                    gpa,
                    size,
                    e
                ),
            }
        }
    }

        &self,
        exec: bool,
        // X in the guest's ACL entry. The boot-time SHARE path cannot have it -- the driver maps
        // those regions non-executable and the guest faults on the first instruction fetch -- but
        // a runtime memparcel carries whatever rights its ACL asks for, and the platform's SCM
        // assign leaves HLOS its own RWX either way, so the host does not lose access. Measured on
        // sm8650: with X the guest runs code out of the parcel, without it the same call raises
        // SIGBUS while the same page still reads and writes.
        //
        // The window of a pseudo-unprotected VM needs it, because the payload runs from there.
        // Host-visible blobs do not, and do not get it. GH_SHARE_EXEC forces it on for everything
        // and exists only to repeat that measurement.
        if exec || std::env::var_os("GH_SHARE_EXEC").is_some_and(|v| v != "0") {
            flags |= GH_MEM_ALLOW_EXEC;
        }
        // Probe the pin before the module takes its own (see pin.rs). A blob whose pages cannot
        // be pinned must fail as this one request -- the guest gets an allocation error and the
        // VM keeps running -- rather than inside the SHARE, where the same condition has taken
        // the whole host down. Dropped on the error paths below; on success it is held until the
        // guest has accepted (or failed to), then released via `release_share_pin`.
        let probe = pin::LongtermPin::ensure_pinnable(
            mem_region.as_ptr() as u64,
            size,
            pin::PinSite::Share,
        )?;

        let share_started = std::time::Instant::now();
        let share_elapsed = share_started.elapsed();
            let e = Error::last();
            base::error!(
                "GUNYAH-SHARE-BLOB: gpa={:#x} size={:#x} flags={:#x} FAILED after {:?}: {}",
                guest_addr.offset(),
                size,
                flags,
                share_elapsed,
                e,
            );
            return Err(e);
        }
        // One line per share at info level once the range is big enough to be a pool grant rather
        // than a blob: the cost of a single RM MEM_SHARE as a function of its size is the number
        // the pseudo-unprotected window sizing depends on, and it is not derivable from anything
        // else the host prints.
        if size >= (32 << 20) {
            base::info!(
                "GUNYAH-SHARE-BLOB: gpa={:#x} size={:#x} flags={:#x} handle={:#x} took {:?}",
                guest_addr.offset(),
                size,
                flags,
                blob.mem_handle,
                share_elapsed,
            );
        }
        if let Some(probe) = probe {
            self.blob_pins.lock().insert(label, probe);
        self.blob_pins.lock().remove(&label);
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
            blob_pins: self.blob_pins.clone(),
            routes: self.routes.clone(),
            hv_cfg: self.hv_cfg,
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

        // Host-visible blobs are data: rings, buffers, framebuffers. Nothing executes from them.
    fn release_share_pin(&self, guest_addr: GuestAddress) {
        // Same deterministic label the SHARE used (gpa>>12). Under GH_NO_UNSHARE the SHARE uses
        // monotonic labels and nothing is ever reclaimed, so the pin stays with the leaked
        // parcel -- that flag is test-only and already leaks the parcel itself.
        let label = (guest_addr.offset() >> 12) as u32;
        self.blob_pins.lock().remove(&label);
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

        // SAFETY: safe because memory is not modified and the return value is checked.
        let res = unsafe {
            set_user_memory_region(
                &self.vm,
                slot,
                read_only,
                guest_addr.offset(),
                size,
                mem_region.as_ptr(),
            )
        };

        let res = if let Err(ref e) = res {
            if e.errno() == EEXIST {
                warn!(
                    "Gunyah set_user_memory_region failed with EEXIST for slot {} \
                     at GPA 0x{:x} size 0x{:x}, trying android_lend fallback",
                    slot,
                    guest_addr.offset(),
                    size,
                );
                // SAFETY: safe because memory is not modified and the return value is checked.
                unsafe {
                    android_lend_user_memory_region(
                        &self.vm,
                        slot,
                        read_only,
                        guest_addr.offset(),
                        size,
                        mem_region.as_ptr(),
                    )
                }
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
        let guest_addr = match regions.get(&slot) {
            Some((_, addr)) => addr.offset(),
            None => return Err(Error::new(ENOENT)),
        };
        // SAFETY:
        // Safe because the slot is checked against the list of memory slots.
        // Passing memory_size=0 signals the hypervisor to remove the region.
        let res = unsafe {
            set_user_memory_region(
                &self.vm,
                slot,
                false,
                guest_addr,
                0,
                std::ptr::null_mut(),
            )
        };
        if let Err(e) = res {
            warn!("Gunyah remove_memory_region ioctl failed for slot {}: {}", slot, e);
        }
        self.mem_slot_gaps.lock().push(Reverse(slot));
        Ok(regions.remove(&slot).unwrap().0)
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
            { base::error!("GH-DIAG: GH_VM_ADD_FUNCTION failed"); errno_result() }
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
            { base::error!("GH-DIAG: GH_VM_REMOVE_FUNCTION failed"); errno_result() }
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
