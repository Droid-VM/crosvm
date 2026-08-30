// Copyright 2017 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! Track memory regions that are mapped to the guest VM.

use std::convert::AsRef;
use std::convert::TryFrom;
use std::fs::File;
use std::io::Read;
use std::io::Write;
use std::marker::Send;
use std::marker::Sync;
use std::result;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::collections::BTreeMap;
use std::sync::Arc;
use std::sync::Mutex;

use anyhow::bail;
use anyhow::Context;
use base::pagesize;
use base::AsRawDescriptor;
use base::AsRawDescriptors;
use base::Error as SysError;
use base::MappedRegion;
use base::MemoryMapping;
use base::MemoryMappingBuilder;
use base::MmapError;
use base::RawDescriptor;
use base::SharedMemory;
use base::VolatileMemory;
use base::VolatileMemoryError;
use base::VolatileSlice;
use cros_async::mem;
use cros_async::BackingMemory;
use remain::sorted;
use serde::Deserialize;
use serde::Serialize;
use serde_keyvalue::FromKeyValues;
use snapshot::AnySnapshot;
use thiserror::Error;
use zerocopy::FromBytes;
use zerocopy::Immutable;
use zerocopy::IntoBytes;

use crate::guest_address::GuestAddress;
use crate::pool_grants::GrantError;
use crate::pool_grants::PoolGrants;

mod sys;
pub use sys::MemoryPolicy;

#[sorted]
#[derive(Error, Debug)]
pub enum Error {
    #[error("failed to map guest memory to file: {0}")]
    FiledBackedMemoryMappingFailed(#[source] MmapError),
    #[error("failed to open file for file backed mapping: {0}")]
    FiledBackedOpenFailed(#[source] std::io::Error),
    #[error("invalid guest address {0}")]
    InvalidGuestAddress(GuestAddress),
    #[error("invalid offset {0}")]
    InvalidOffset(u64),
    #[error("size {0} must not be zero")]
    InvalidSize(usize),
    #[error("invalid guest memory access at addr={0}: {1}")]
    MemoryAccess(GuestAddress, #[source] MmapError),
    #[error("failed to set seals on shm region: {0}")]
    MemoryAddSealsFailed(#[source] SysError),
    #[error("failed to create shm region: {0}")]
    MemoryCreationFailed(#[source] SysError),
    #[error("failed to map guest memory: {0}")]
    MemoryMappingFailed(#[source] MmapError),
    #[error("guest memory region {0}+{1:#x} is not page aligned")]
    MemoryNotAligned(GuestAddress, u64),
    #[error("memory regions overlap")]
    MemoryRegionOverlap,
    #[error("memory region size {0} is too large")]
    MemoryRegionTooLarge(u128),
    #[error("growable pool at {0}: {1}")]
    PoolParams(GuestAddress, String),
    #[error("host access to lent memory region at {0} (purpose={1:?}) in protected VM")]
    ProtectedMemoryAccess(GuestAddress, MemoryRegionPurpose),
    #[error("incomplete read of {completed} instead of {expected} bytes")]
    ShortRead { expected: usize, completed: usize },
    #[error("incomplete write of {completed} instead of {expected} bytes")]
    ShortWrite { expected: usize, completed: usize },
    #[error("DescriptorChain split is out of bounds: {0}")]
    SplitOutOfBounds(usize),
    #[error("host access to an ungranted address {0} in a growable pool")]
    UngrantedPoolAccess(GuestAddress),
    #[error("{0}")]
    VolatileMemoryAccess(#[source] VolatileMemoryError),
}

pub type Result<T> = result::Result<T, Error>;

/// A file-like object backing `MemoryRegion`.
#[derive(Clone, Debug)]
pub enum BackingObject {
    Shm(Arc<SharedMemory>),
    File(Arc<File>),
}

impl AsRawDescriptor for BackingObject {
    fn as_raw_descriptor(&self) -> RawDescriptor {
        match self {
            Self::Shm(shm) => shm.as_raw_descriptor(),
            Self::File(f) => f.as_raw_descriptor(),
        }
    }
}

impl AsRef<dyn AsRawDescriptor + Sync + Send> for BackingObject {
    fn as_ref(&self) -> &(dyn AsRawDescriptor + Sync + Send + 'static) {
        match self {
            BackingObject::Shm(shm) => shm.as_ref(),
            BackingObject::File(f) => f.as_ref(),
        }
    }
}

/// What the hypervisor backend was able to promise about the pages behind the `SharedFramebuffer`
/// region, recorded at VM creation and read much later by whoever wants to hand those pages to
/// someone who will hold a reference on them.
///
/// The one consumer is the simplefb bridge's GPU transport. A udmabuf over this region takes a
/// plain page reference on every page (`shmem_read_mapping_page` on GKI 6.6 -- a reference, not a
/// pin), and a referenced page cannot be migrated. If the page is sitting in a CMA pageblock when
/// the guest first touches it, gunyah's fault-time `FOLL_LONGTERM` pin has to migrate it out and
/// cannot, and the guest takes `page fault at <gpa>, attempt: -12` -- a vcpu OOM seconds after the
/// bridge announces `transport=gpu-blit`. So the question the GPU path has to ask first is not
/// "can I make a dmabuf" but "is this memory somewhere the host can leave it", and only the
/// backend that laid the region out knows the answer.
///
/// `Unclaimed` is not "probably fine": it is "nobody has said", and the GPU path treats it exactly
/// like a refusal. A backend that wants the fast path has to earn it by recording `PoolBacked`.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub enum FramebufferPrep {
    /// No hypervisor backend has spoken for this region.
    #[default]
    Unclaimed,
    /// Every 2 MiB of the region is a present, non-movable folio, and will still be one when the
    /// guest faults on it.
    PoolBacked,
    /// It is not, and this is why. Carried as prose because the only thing done with it is to put
    /// it in the line that explains why the bridge is on the CPU path.
    NotPoolBacked(String),
}

/// For MemoryRegion::regions
pub struct MemoryRegionInformation<'a> {
    pub index: usize,
    pub guest_addr: GuestAddress,
    pub size: usize,
    pub host_addr: usize,
    pub shm: &'a BackingObject,
    pub shm_offset: u64,
    pub options: MemoryRegionOptions,
}

#[sorted]
#[derive(Clone, Copy, Debug, Default, PartialOrd, PartialEq, Eq, Ord)]
pub enum MemoryRegionPurpose {
    /// BIOS/firmware ROM
    Bios,

    /// DroidVM: drm2kgsl native-context arena. A third SHARE-blessed pool, treated exactly like
    /// GpuPool for access/bless/hugepage, that virglrenderer's drm2kgsl backend sub-allocates every
    /// GPU BO from. Distinct from GpuPool so a build carrying both renderers cannot hand the
    /// drm2kgsl arena to gfxstream's HostVisiblePool (which is handed any GpuPool region), so the
    /// two can be sized independently, and so it gets its own `drm2kgsl_host` DT node.
    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    Drm2KgslPool,

    /// DroidVM: a GROWABLE test pool. Declared to the guest at its full size but SHARE'd only up
    /// to `pre_alloc_size` at boot; the rest is granted at runtime over virtio-gunyah-accept's
    /// pool queue. Exists so the growable path can be exercised end to end without disturbing the
    /// three production pools, which are all fully pre-shared and must stay that way.
    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    DynamicTestPool,

    /// DroidVM: GPU pre-alloc pool (gfxstream HOST-visible pool). Appended after
    /// guest RAM, SHARE'd (not lent) at boot on protected Gunyah so the host renderer and the guest
    /// reach the same pages, hugepage-prepared, and announced as a no-map reserved-memory node.
    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    GpuPool,

    /// DroidVM: gfxstream GUEST-alloc pool. A second SHARE-blessed region (treated exactly like
    /// GpuPool for access/bless/hugepage) that the guest virtio-gpu driver owns and sub-allocates
    /// BLOB_MEM_GUEST from in guest-alloc mode. Distinct so it gets its own `gpu_guest`
    /// DT node and is NOT handed to the host gfxstream HostVisiblePool (which sees only GpuPool).
    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    GpuPoolGuest,

    /// General purpose guest memory
    #[default]
    GuestMemoryRegion,

    /// PVMFW
    ProtectedFirmwareRegion,

    /// An area that should be backed by a GuestMemory region but reported as reserved to the
    /// guest.
    ReservedMemory,

    /// Framebuffer memory shared between host and guest (not lent in protected VM mode).
    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    SharedFramebuffer,

    /// DroidVM: the guest's RAM in a pseudo-unprotected VM.
    ///
    /// Declared to crosvm like any region -- it is a memfd the host writes the payload into
    /// before the VM starts -- but never handed to the hypervisor at boot. It appears in no
    /// device tree node, so the resource manager has nothing to object to, and arrives in the
    /// guest as a runtime memparcel the boot shim accepts. From then on it is ordinary memory
    /// that both sides can reach, which is the point: no bounce pool, and a stock kernel with no
    /// CONFIG_RESTRICTED_DMA_POOL boots.
    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    SharedGuestRam,

    /// DroidVM: the page a pseudo-unprotected VM's host and shim talk through.
    ///
    /// SHARE'd at boot like a pool, because the memparcel handles it carries do not exist until
    /// after GH_VM_START -- by which time the boot region is lent and the host can no longer
    /// write to it. It is also the shim's only way to report: a failure written here is a log
    /// line, and a failure with nowhere to go is a VM that hangs.
    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    ShimHandoff,

    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    StaticSwiotlbRegion,
}

#[derive(Clone, Debug, Serialize, Deserialize, FromKeyValues, PartialEq, Eq, PartialOrd, Ord)]
#[serde(deny_unknown_fields)]
pub struct FileBackedMappingParameters {
    pub path: std::path::PathBuf,
    #[serde(rename = "addr")]
    pub address: u64,
    pub size: u64,
    #[serde(default)]
    pub offset: u64,
    #[serde(rename = "rw", default)]
    pub writable: bool,
    #[serde(default)]
    pub sync: bool,
    #[serde(default)]
    pub align: bool,
    /// Whether the mapping is for RAM or MMIO.
    #[serde(default)]
    pub ram: bool,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct MemoryRegionOptions {
    /// Some hypervisors (presently: Gunyah) need explicit knowledge about
    /// which memory region is used for protected firwmare, static swiotlb,
    /// or general purpose guest memory.
    pub purpose: MemoryRegionPurpose,
    /// Alignment for the mapping of this region. This intends to be used for
    /// arm64 KVM support where a block alignment is required for transparent
    /// huge-pages support
    pub align: u64,
    /// Backing file params.
    pub file_backed: Option<FileBackedMappingParameters>,
    /// When true, this region gets its own dedicated SharedMemory backing
    /// instead of sharing the global guest memory memfd.  This is needed for
    /// Gunyah protected VMs where mixing lend/share operations on the same
    /// memfd causes conflicts.
    pub isolate_backing: bool,

    /// How much of this region is SHARE'd to a protected guest at boot. `None` means all of it,
    /// which is what every region that is not a growable pool wants.
    ///
    /// A growable pool is declared to the guest at its FULL size but backed only in part, with the
    /// remainder filled in at runtime as the guest asks for it. That works because the region is
    /// still created whole -- a sparse memfd, so host VA rather than host RAM -- and `size` is
    /// what feeds `ram_top` and hence the RM's `size-max`, so the RM untags the entire window
    /// whether or not it is backed. Measured on device: see plans/DYNAMIC_POOL_PLAN.md.
    ///
    /// Build the region full and share part of it. Building it small and extending it later is the
    /// version that fails, with RM_ERROR_MEM_INVALID (0xa) on the first runtime accept.
    pub pre_alloc_size: Option<u64>,

    /// Granularity of a runtime grant into the unbacked remainder, in bytes.
    ///
    /// `0` means this pool does not grow at all: everything is shared before boot, which is what
    /// the three existing pools do and must keep doing. Non-zero must be a multiple of 2 MiB --
    /// the folio the share path is built around, and the granularity the reserve pool serves.
    ///
    /// Each grant costs one RM memparcel, and MAX_MEMPARCEL_PER_VM is 1024 across the whole VM --
    /// shared with Android's own parcels, and never released until the phone reboots for anything
    /// a killed VMM left behind. A 2 MiB step therefore exhausts the quota at 2 GiB and takes the
    /// phone with it; 32-64 MiB is the range that leaves room.
    pub step_size: u64,

    /// Cap on this pool's simultaneously live grants. Zero means "as many as the window holds",
    /// which is only safe for a small window: the real limit is MAX_MEMPARCEL_PER_VM = 1024 for
    /// the entire VM, and it is shared with Android's own parcels, so several pools each sizing
    /// themselves against the window can add up past it. Budget it across pools, not per pool.
    pub max_grants: u32,
}

impl MemoryRegionOptions {
    pub fn new() -> MemoryRegionOptions {
        Default::default()
    }

    pub fn purpose(mut self, purpose: MemoryRegionPurpose) -> Self {
        self.purpose = purpose;
        self
    }

    pub fn align(mut self, alignment: u64) -> Self {
        self.align = alignment;
        self
    }

    pub fn file_backed(mut self, params: FileBackedMappingParameters) -> Self {
        self.file_backed = Some(params);
        self
    }

    pub fn isolate_backing(mut self) -> Self {
        self.isolate_backing = true;
        self
    }

    /// Declare this region as a growable pool: `pre_alloc` bytes shared at boot, the remainder
    /// left for runtime grants of `step` bytes each. `step == 0` means the pool never grows and
    /// `pre_alloc` must be the whole region.
    pub fn growable_pool(mut self, pre_alloc: u64, step: u64) -> Self {
        self.pre_alloc_size = Some(pre_alloc);
        self.step_size = step;
        self
    }

    /// Cap on simultaneously live grants for this pool. See `max_grants`.
    pub fn max_grants(mut self, n: u32) -> Self {
        self.max_grants = n;
        self
    }

    /// Bytes SHARE'd at boot. Regions that never set `pre_alloc_size` get all of `size`, which
    /// keeps every non-pool region and the three pre-existing pools on exactly the path they are
    /// on today.
    pub fn boot_share_len(&self, size: u64) -> u64 {
        self.pre_alloc_size.map_or(size, |p| p.min(size))
    }

    /// Validate the pool parameters against a region size. Returns the reason it is not usable, so
    /// the caller can refuse at configuration time rather than at first grant -- an invalid step
    /// otherwise surfaces as drm_buddy returning -EINVAL inside the guest, which reads as a guest
    /// driver bug rather than a command line mistake.
    pub fn pool_param_error(&self, size: u64) -> Option<String> {
        const MIN_STEP: u64 = 2 << 20;
        let pre_alloc = self.pre_alloc_size.unwrap_or(size);
        if pre_alloc > size {
            return Some(format!(
                "pre_alloc_size {:#x} exceeds the region size {:#x}",
                pre_alloc, size
            ));
        }
        if self.step_size == 0 {
            // A pool that cannot grow must be fully backed, or the remainder is memory the guest
            // has been told about and can never obtain -- a read there silently returns zeros.
            return (pre_alloc != size).then(|| {
                format!(
                    "step_size is 0 (pool does not grow) but pre_alloc_size {:#x} != size {:#x}; \
                     the difference could never be granted",
                    pre_alloc, size
                )
            });
        }
        if self.step_size < MIN_STEP {
            return Some(format!(
                "step_size {:#x} is below the {:#x} minimum (a grant is shared as 2 MiB folios)",
                self.step_size, MIN_STEP
            ));
        }
        // A multiple of a folio, not a power of two. The power-of-two rule was written down as
        // "drm_buddy rejects anything else", and that turned out to be about a different number:
        // the guest's allocator is initialised with PAGE_SIZE as its chunk (virtgpu_vram.c
        // `drm_buddy_init(..., PAGE_SIZE)`), and never sees the step at all. What the step really
        // has to satisfy is the share path -- grants are made of 2 MiB folios -- and the
        // bookkeeping below, which wants the window and the floor to be whole numbers of steps.
        if self.step_size % MIN_STEP != 0 {
            return Some(format!(
                "step_size {:#x} is not a multiple of {:#x} (a grant is shared as 2 MiB folios)",
                self.step_size, MIN_STEP
            ));
        }
        if size % self.step_size != 0 || pre_alloc % self.step_size != 0 {
            return Some(format!(
                "size {:#x} and pre_alloc_size {:#x} must both be multiples of step_size {:#x}",
                size, pre_alloc, self.step_size
            ));
        }
        // A growable pool must have SOMETHING shared before boot, and at least a folio of it.
        //
        // The pool's `/reserved-memory` node declares the pre-shared floor, and the Gunyah
        // resource manager on android14-6.1 requires every such node's `reg` to match an accepted
        // memparcel exactly -- base, size, and a single mapping. A zero floor leaves nothing for
        // it to match and the VM does not start at all: GH_VM_START answers ENODEV, which reads
        // as a broken VMM rather than as the one line of configuration it is. (Measured on
        // sm8650: with the node the VM is refused, without it the same VM boots -- see
        // plans/PSEUDO_UNPROTECTED_SHIM_PLAN.md.)
        //
        // The floor is also where the pool's identity lives: it is the range a driver may touch
        // without asking, and the node that carries the pool's name, id, step and total size.
        const MIN_FLOOR: u64 = 2 << 20;
        if pre_alloc < MIN_FLOOR {
            return Some(format!(
                "pre_alloc_size {:#x} is below the {:#x} minimum for a growable pool; the                  reserved-memory node has to describe a memparcel that really exists",
                pre_alloc, MIN_FLOOR
            ));
        }
        None
    }
}

/// A regions of memory mapped memory.
/// Holds the memory mapping with its offset in guest memory.
/// Also holds the backing object for the mapping and the offset in that object of the mapping.
#[derive(Debug)]
pub struct MemoryRegion {
    mapping: MemoryMapping,
    guest_base: GuestAddress,

    shared_obj: BackingObject,
    obj_offset: u64,

    options: MemoryRegionOptions,
}

impl MemoryRegion {
    /// Creates a new MemoryRegion using the given SharedMemory object to later be attached to a VM
    /// at `guest_base` address in the guest.
    pub fn new_from_shm(
        size: u64,
        guest_base: GuestAddress,
        offset: u64,
        shm: Arc<SharedMemory>,
    ) -> Result<Self> {
        let mapping = MemoryMappingBuilder::new(size as usize)
            .from_shared_memory(shm.as_ref())
            .offset(offset)
            .build()
            .map_err(Error::MemoryMappingFailed)?;
        Ok(MemoryRegion {
            mapping,
            guest_base,
            shared_obj: BackingObject::Shm(shm),
            obj_offset: offset,
            options: Default::default(),
        })
    }

    /// Creates a new MemoryRegion using the given file to get available later at `guest_base`
    /// address in the guest.
    pub fn new_from_file(
        size: u64,
        guest_base: GuestAddress,
        offset: u64,
        file: Arc<File>,
    ) -> Result<Self> {
        let mapping = MemoryMappingBuilder::new(size as usize)
            .from_file(&file)
            .offset(offset)
            .build()
            .map_err(Error::MemoryMappingFailed)?;
        Ok(MemoryRegion {
            mapping,
            guest_base,
            shared_obj: BackingObject::File(file),
            obj_offset: offset,
            options: Default::default(),
        })
    }

    fn start(&self) -> GuestAddress {
        self.guest_base
    }

    fn end(&self) -> GuestAddress {
        // unchecked_add is safe as the region bounds were checked when it was created.
        self.guest_base.unchecked_add(self.mapping.size() as u64)
    }

    fn contains(&self, addr: GuestAddress) -> bool {
        addr >= self.guest_base && addr < self.end()
    }
}

/// Tracks memory regions and where they are mapped in the guest, along with shm
/// descriptors of the underlying memory regions.
#[derive(Clone, Debug)]
pub struct GuestMemory {
    regions: Arc<[MemoryRegion]>,
    locked: bool,
    /// When true, host access to lent (non-shared) memory regions is forbidden.
    /// Set after memory is donated to a protected VM (e.g. Gunyah).
    protected: Arc<AtomicBool>,
    /// Which parts of each growable pool are currently backed, keyed by the pool's base address.
    /// Empty unless a region declared a non-zero `step_size`, so nothing that exists today pays
    /// for it. See pool_grants.rs for why the host has to keep this rather than deriving it.
    grants: Arc<Mutex<BTreeMap<u64, PoolGrants>>>,
    /// What the hypervisor backend promised about the `SharedFramebuffer` region's pages. Written
    /// once while the VM is being built, read by the simplefb bridge long afterwards; it lives
    /// here rather than in a process global because it is a fact about one VM's memory.
    fb_prep: Arc<Mutex<FramebufferPrep>>,
}

impl AsRawDescriptors for GuestMemory {
    /// USE WITH CAUTION, the descriptors returned here are not necessarily
    /// files!
    fn as_raw_descriptors(&self) -> Vec<RawDescriptor> {
        self.regions
            .iter()
            .map(|r| r.shared_obj.as_raw_descriptor())
            .collect()
    }
}

impl GuestMemory {
    /// Creates backing shm for GuestMemory regions
    fn create_shm(ranges: &[(GuestAddress, u64, MemoryRegionOptions)]) -> Result<SharedMemory> {
        let mut aligned_size = 0;
        let pg_size = pagesize();
        for range in ranges {
            if range.2.file_backed.is_some() || range.2.isolate_backing {
                // Regions with a backing file or isolated backing don't use
                // part of the global `SharedMemory`.
                continue;
            }
            if range.1 % pg_size as u64 != 0 {
                return Err(Error::MemoryNotAligned(range.0, range.1));
            }

            aligned_size += range.1;
        }

        // NOTE: Some tests rely on the GuestMemory's name when capturing metrics.
        let name = "crosvm_guest";
        // Shm must be mut even though it is only updated on Unix systems.
        #[allow(unused_mut)]
        let mut shm = SharedMemory::new(name, aligned_size).map_err(Error::MemoryCreationFailed)?;

        sys::finalize_shm(&mut shm)?;

        Ok(shm)
    }

    /// Creates a container for guest memory regions.
    /// Valid memory regions are specified as a Vec of (Address, Size, MemoryRegionOptions)
    pub fn new_with_options(
        ranges: &[(GuestAddress, u64, MemoryRegionOptions)],
    ) -> Result<GuestMemory> {
        // Refuse bad pool parameters here, before a single page is mapped. Left to run, a step
        // the share path cannot honour surfaces as an error deep inside the guest driver, and
        // a pre_alloc short of the size on a non-growable pool surfaces as reads of the shortfall
        // silently returning zeros -- both of which read as guest driver bugs rather than as the
        // command line mistake they are.
        for (addr, size, options) in ranges {
            if let Some(why) = options.pool_param_error(*size) {
                return Err(Error::PoolParams(*addr, why));
            }
        }

        // Create shm
        let shm = Arc::new(GuestMemory::create_shm(ranges)?);

        // Create memory regions
        let mut regions = Vec::<MemoryRegion>::new();
        let mut shm_offset = 0;

        for range in ranges {
            if let Some(last) = regions.last() {
                if last
                    .guest_base
                    .checked_add(last.mapping.size() as u64)
                    .map_or(true, |a| a > range.0)
                {
                    return Err(Error::MemoryRegionOverlap);
                }
            }

            let size = usize::try_from(range.1)
                .map_err(|_| Error::MemoryRegionTooLarge(range.1 as u128))?;
            if let Some(file_backed) = &range.2.file_backed {
                assert_eq!(usize::try_from(file_backed.size).unwrap(), size);
                let file = file_backed.open().map_err(Error::FiledBackedOpenFailed)?;
                let mapping = MemoryMappingBuilder::new(size)
                    .from_file(&file)
                    .offset(file_backed.offset)
                    .align(range.2.align)
                    .protection(if file_backed.writable {
                        base::Protection::read_write()
                    } else {
                        base::Protection::read()
                    })
                    .build()
                    .map_err(Error::FiledBackedMemoryMappingFailed)?;
                regions.push(MemoryRegion {
                    mapping,
                    guest_base: range.0,
                    shared_obj: BackingObject::File(Arc::new(file)),
                    obj_offset: file_backed.offset,
                    options: range.2.clone(),
                });
            } else if range.2.isolate_backing {
                // Create a dedicated SharedMemory for this region so its
                // backing fd is separate from the global guest memory memfd.
                let iso_shm = SharedMemory::new("crosvm_isolated", range.1)
                    .map_err(Error::MemoryCreationFailed)?;
                let iso_shm = Arc::new(iso_shm);
                let mapping = MemoryMappingBuilder::new(size)
                    .from_shared_memory(iso_shm.as_ref())
                    .offset(0)
                    .align(range.2.align)
                    .build()
                    .map_err(Error::MemoryMappingFailed)?;
                regions.push(MemoryRegion {
                    mapping,
                    guest_base: range.0,
                    shared_obj: BackingObject::Shm(iso_shm),
                    obj_offset: 0,
                    options: range.2.clone(),
                });
            } else {
                let mapping = MemoryMappingBuilder::new(size)
                    .from_shared_memory(shm.as_ref())
                    .offset(shm_offset)
                    .align(range.2.align)
                    .build()
                    .map_err(Error::MemoryMappingFailed)?;
                regions.push(MemoryRegion {
                    mapping,
                    guest_base: range.0,
                    shared_obj: BackingObject::Shm(shm.clone()),
                    obj_offset: shm_offset,
                    options: range.2.clone(),
                });
                shm_offset += size as u64;
            }
        }

        // One entry per growable pool; regions with step_size == 0 -- which is every region that
        // exists today -- contribute nothing, so this map stays empty on the existing paths.
        let grants = regions
            .iter()
            .filter(|r| r.options.step_size != 0)
            .map(|r| {
                let size = r.mapping.size() as u64;
                (
                    r.guest_base.offset(),
                    PoolGrants::new(
                        r.guest_base,
                        size,
                        r.options.boot_share_len(size),
                        r.options.step_size,
                        r.options.max_grants,
                    ),
                )
            })
            .collect::<BTreeMap<_, _>>();

        Ok(GuestMemory {
            regions: Arc::from(regions),
            locked: false,
            protected: Arc::new(AtomicBool::new(false)),
            grants: Arc::new(Mutex::new(grants)),
            fb_prep: Arc::new(Mutex::new(FramebufferPrep::Unclaimed)),
        })
    }

    /// Creates a container for guest memory regions.
    /// Valid memory regions are specified as a Vec of (Address, Size) tuples sorted by Address.
    pub fn new(ranges: &[(GuestAddress, u64)]) -> Result<GuestMemory> {
        GuestMemory::new_with_options(
            ranges
                .iter()
                .map(|(addr, size)| (*addr, *size, Default::default()))
                .collect::<Vec<(GuestAddress, u64, MemoryRegionOptions)>>()
                .as_slice(),
        )
    }

    /// Creates a `GuestMemory` from a collection of MemoryRegions.
    pub fn from_regions(mut regions: Vec<MemoryRegion>) -> Result<Self> {
        // Sort the regions and ensure non overlap.
        regions.sort_by(|a, b| a.guest_base.cmp(&b.guest_base));

        if regions.len() > 1 {
            let mut prev_end = regions[0]
                .guest_base
                .checked_add(regions[0].mapping.size() as u64)
                .ok_or(Error::MemoryRegionOverlap)?;
            for region in &regions[1..] {
                if prev_end > region.guest_base {
                    return Err(Error::MemoryRegionOverlap);
                }
                prev_end = region
                    .guest_base
                    .checked_add(region.mapping.size() as u64)
                    .ok_or(Error::MemoryRegionTooLarge(
                        region.guest_base.0 as u128 + region.mapping.size() as u128,
                    ))?;
            }
        }

        // One entry per growable pool; regions with step_size == 0 -- which is every region that
        // exists today -- contribute nothing, so this map stays empty on the existing paths.
        let grants = regions
            .iter()
            .filter(|r| r.options.step_size != 0)
            .map(|r| {
                let size = r.mapping.size() as u64;
                (
                    r.guest_base.offset(),
                    PoolGrants::new(
                        r.guest_base,
                        size,
                        r.options.boot_share_len(size),
                        r.options.step_size,
                        r.options.max_grants,
                    ),
                )
            })
            .collect::<BTreeMap<_, _>>();

        Ok(GuestMemory {
            regions: Arc::from(regions),
            locked: false,
            protected: Arc::new(AtomicBool::new(false)),
            grants: Arc::new(Mutex::new(grants)),
            fb_prep: Arc::new(Mutex::new(FramebufferPrep::Unclaimed)),
        })
    }

    // Whether `MemoryPolicy::LOCK_GUEST_MEMORY` was set.
    pub fn locked(&self) -> bool {
        self.locked
    }

    /// Mark this GuestMemory as belonging to a protected VM.  After this call,
    /// any host-side access to a region whose purpose is **not**
    /// `SharedFramebuffer` or `StaticSwiotlbRegion` will return
    /// `Error::ProtectedMemoryAccess` instead of risking a SIGBUS.
    pub fn set_protected(&self) {
        self.protected.store(true, Ordering::Release);
    }

    /// Record what the hypervisor backend managed to promise about the `SharedFramebuffer`
    /// region's pages. Called once, while the VM is being built.
    pub fn set_framebuffer_prep(&self, prep: FramebufferPrep) {
        *self.fb_prep.lock().expect("framebuffer prep record poisoned") = prep;
    }

    /// What was recorded. `Unclaimed` when no backend said anything, which is the answer the
    /// framebuffer's GPU transport must refuse on -- see [`FramebufferPrep`].
    pub fn framebuffer_prep(&self) -> FramebufferPrep {
        self.fb_prep
            .lock()
            .expect("framebuffer prep record poisoned")
            .clone()
    }

    /// Base address of the `pool_id`-th growable pool, ordered by address.
    ///
    /// The id is an index rather than a name because it has to survive a wire round trip in 32
    /// bits and be trivially checkable; the guest learns which is which from the device tree node
    /// order, which is the same ordering.
    pub fn pool_base(&self, pool_id: u32) -> Option<GuestAddress> {
        let grants = self.grants.lock().expect("pool grant table poisoned");
        grants.keys().nth(pool_id as usize).copied().map(GuestAddress)
    }

    /// Take a host-side reference on every growable pool the iovecs touch, after checking that
    /// all of them are backed.
    ///
    /// Two jobs in one pass, because they need the same lookup:
    ///
    ///   * REFUSE an import over unbacked pool memory. The udmabuf path resolves guest addresses
    ///     through `find_region`/`shm_region`, which are not gated by `check_host_access`, so
    ///     without this a guest could have a dma-buf built over a HOLE in the sparse pool memfd.
    ///     Reading such a buffer allocates host memory on the spot -- a guest-triggerable,
    ///     unaccounted, never-reclaimed host allocation.
    ///
    ///   * Hold the grant until the import is gone, so a shrink cannot pull memory out from under
    ///     a live dma-buf. The guest cannot get this right by itself: its RESOURCE_UNREF is
    ///     fire-and-forget, so it returns a buffer to its own allocator while the host still has
    ///     it mapped.
    ///
    /// All or nothing: if any iovec is unbacked, nothing is referenced. Addresses outside every
    /// growable pool are ignored, so callers do not have to know which is which.
    pub fn pool_ref_iovecs(&self, iovecs: &[(GuestAddress, usize)]) -> std::result::Result<(), GrantError> {
        let mut grants = self.grants.lock().expect("pool grant table poisoned");
        if grants.is_empty() {
            return Ok(());
        }
        // Check everything before touching anything, so a refusal leaves no counts raised.
        for (addr, len) in iovecs {
            let start = addr.offset();
            let end = start
                .checked_add(*len as u64)
                .ok_or(GrantError::NotBacked)?;
            for (base, pool) in grants.iter() {
                let pool_end = base
                    .checked_add(pool.size())
                    .ok_or(GrantError::NotBacked)?;
                if start < pool_end && *base < end {
                    // An iovec that crosses a pool boundary cannot be safely represented by a
                    // single pool grant lookup. Reject it even when it starts outside the pool;
                    // otherwise a guest could smuggle an unbacked suffix into an otherwise valid
                    // dma-buf by choosing a start address in ordinary RAM.
                    if start < *base || end > pool_end {
                        return Err(GrantError::NotBacked);
                    }
                    if !pool.range_backed(start - *base, *len as u64) {
                        return Err(GrantError::NotBacked);
                    }
                    pool.ref_range_available(start - *base, *len as u64)?;
                }
            }
        }
        for (addr, len) in iovecs {
            for (base, pool) in grants.iter_mut() {
                let a = addr.offset();
                if a >= *base && a < *base + pool.size() {
                    pool.ref_range(a - *base, *len as u64)
                        .expect("pool iovec was validated immediately above");
                }
            }
        }
        Ok(())
    }

    /// Drop references taken by [`Self::pool_ref_iovecs`]. Must be handed the same iovecs.
    pub fn pool_unref_iovecs(&self, iovecs: &[(GuestAddress, usize)]) {
        let mut grants = self.grants.lock().expect("pool grant table poisoned");
        if grants.is_empty() {
            return;
        }
        for (addr, len) in iovecs {
            for (base, pool) in grants.iter_mut() {
                let a = addr.offset();
                if a >= *base && a < *base + pool.size() {
                    pool.unref_range(a - *base, *len as u64);
                }
            }
        }
    }

    /// Grant granularity of a pool, or `None` if there is no such pool.
    pub fn pool_step(&self, pool_id: u32) -> Option<u64> {
        let grants = self.grants.lock().expect("pool grant table poisoned");
        grants.values().nth(pool_id as usize).map(|p| p.step())
    }

    /// Live grants in a pool, for the guest's reconciliation query.
    pub fn pool_live_grants(&self, pool_id: u32) -> Option<usize> {
        let grants = self.grants.lock().expect("pool grant table poisoned");
        grants.values().nth(pool_id as usize).map(|p| p.live_grants())
    }

    /// Query whether a specific range in a growable pool is backed. This is used to reconcile a
    /// request whose guest-side wait timed out: the host may have completed the SHARE/UNSHARE even
    /// though the response was lost, and a count alone cannot identify that range.
    pub fn pool_range_backed(&self, pool_id: u32, offset: u64, len: u64) -> Option<bool> {
        let grants = self.grants.lock().expect("pool grant table poisoned");
        grants
            .values()
            .nth(pool_id as usize)
            .map(|p| p.range_backed(offset, len))
    }

    /// Check a guest-originated grow request. Does not modify anything.
    pub fn pool_validate_share(
        &self,
        pool_id: u32,
        offset: u64,
        len: u64,
    ) -> std::result::Result<(), GrantError> {
        let grants = self.grants.lock().expect("pool grant table poisoned");
        grants
            .values()
            .nth(pool_id as usize)
            .ok_or(GrantError::NotGrowable)?
            .validate_share(offset, len)
    }

    /// Reserve a guest-originated shrink request while the host unregisters the mapping. This
    /// blocks new dma-buf references until [`Self::pool_finish_unshare`] or
    /// [`Self::pool_cancel_unshare`] is called.
    pub fn pool_begin_unshare(
        &self,
        pool_id: u32,
        offset: u64,
        len: u64,
    ) -> std::result::Result<(), GrantError> {
        let mut grants = self.grants.lock().expect("pool grant table poisoned");
        grants
            .values_mut()
            .nth(pool_id as usize)
            .ok_or(GrantError::NotGrowable)?
            .begin_unshare(offset, len)
    }

    /// Undo a failed guest-originated shrink reservation.
    pub fn pool_cancel_unshare(&self, pool_id: u32, offset: u64, len: u64) {
        let mut grants = self.grants.lock().expect("pool grant table poisoned");
        if let Some(pool) = grants.values_mut().nth(pool_id as usize) {
            pool.cancel_unshare(offset, len);
        }
    }

    /// Record a completed grant. One grant is one memparcel, whatever its length.
    pub fn pool_mark_granted(
        &self,
        pool_id: u32,
        offset: u64,
        len: u64,
        handle: u32,
    ) -> std::result::Result<(), GrantError> {
        let mut grants = self.grants.lock().expect("pool grant table poisoned");
        grants
            .values_mut()
            .nth(pool_id as usize)
            .ok_or(GrantError::NotGrowable)?
            .mark_granted(offset, len, handle)
    }

    /// Complete a previously reserved guest-originated shrink and return the RM handle recorded
    /// for it. The range must name a grant exactly: the RM reclaims a parcel whole.
    pub fn pool_finish_unshare(
        &self,
        pool_id: u32,
        offset: u64,
        len: u64,
    ) -> std::result::Result<u32, GrantError> {
        let mut grants = self.grants.lock().expect("pool grant table poisoned");
        grants
            .values_mut()
            .nth(pool_id as usize)
            .ok_or(GrantError::NotGrowable)?
            .finish_unshare(offset, len)
    }

    /// A pool region is host-accessible, but a GROWABLE one is only accessible where it is
    /// actually backed.
    ///
    /// This is not belt-and-braces. Measured on device: reading an ungranted address inside a
    /// declared window returns zeros -- no fault, no error, no log, and the VM keeps running --
    /// while writing it kills the vcpu. The silent-zero direction is the one that matters here,
    /// because it turns a host-side accounting bug into wrong data with nothing to show for it.
    /// A region with step_size == 0 is fully pre-shared and answers yes everywhere, which is every
    /// pool that exists today.
    fn check_pool_backed_range(
        &self,
        region: &MemoryRegion,
        guest_addr: GuestAddress,
        len: u64,
    ) -> Result<()> {
        if region.options.step_size == 0 {
            return Ok(());
        }
        let grants = self.grants.lock().expect("pool grant table poisoned");
        let offset = guest_addr
            .offset()
            .checked_sub(region.guest_base.offset())
            .ok_or(Error::InvalidGuestAddress(guest_addr))?;
        match grants.get(&region.guest_base.offset()) {
            Some(p) if p.range_backed(offset, len) => Ok(()),
            // Untracked or ungranted both mean "do not touch it".
            _ => Err(Error::UngrantedPoolAccess(guest_addr)),
        }
    }

    /// Check a complete host-access range, including the part after the first byte. The mapping
    /// APIs otherwise only identify the region from their start address, which would let an
    /// access beginning in a backed prefix run into an unbacked suffix of a growable pool.
    fn check_host_access_range(&self, guest_addr: GuestAddress, len: usize) -> Result<()> {
        self.check_host_access(guest_addr)?;
        if len == 0 || !self.protected.load(Ordering::Acquire) {
            return Ok(());
        }

        let end = guest_addr
            .offset()
            .checked_add(len as u64)
            .ok_or(Error::InvalidGuestAddress(guest_addr))?;
        let region = self
            .regions
            .iter()
            .find(|r| r.contains(guest_addr))
            .ok_or(Error::InvalidGuestAddress(guest_addr))?;
        if end > region.end().offset() {
            return Err(Error::InvalidGuestAddress(guest_addr));
        }

        if region.options.step_size != 0 {
            self.check_pool_backed_range(region, guest_addr, len as u64)?;
        }
        Ok(())
    }

    /// Check whether `guest_addr` falls in a host-accessible region.
    /// Returns `Ok(())` when the access is safe, or an error describing why
    /// the host must not touch that address.
    fn check_host_access(&self, guest_addr: GuestAddress) -> Result<()> {
        if !self.protected.load(Ordering::Acquire) {
            return Ok(());
        }
        let region = self
            .regions
            .iter()
            .find(|r| r.contains(guest_addr))
            .ok_or(Error::InvalidGuestAddress(guest_addr))?;
        match region.options.purpose {
            // The GPU pool is SHARE'd (never lent), so the host keeps access — same as the
            // framebuffer/swiotlb regions (crosvm reads scanout data straight from the pool).
            #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
            MemoryRegionPurpose::GpuPool => self.check_pool_backed_range(region, guest_addr, 1),
            // Guest-alloc pool: SHARE'd like GpuPool; the host resolves guest-blob mem-entries
            // that point into it via get_slice_at_addr (the whole point of guest-alloc).
            #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
            MemoryRegionPurpose::GpuPoolGuest => {
                self.check_pool_backed_range(region, guest_addr, 1)
            }
            // drm2kgsl arena: SHARE'd like the gfx pools. that backend lives in this
            // process and sub-allocates BOs out of it, so the host needs access.
            #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
            MemoryRegionPurpose::Drm2KgslPool => {
                self.check_pool_backed_range(region, guest_addr, 1)
            }
            // The one region where this actually gates anything today: everything else has
            // step_size == 0 and answers yes everywhere.
            #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
            MemoryRegionPurpose::DynamicTestPool => {
                self.check_pool_backed_range(region, guest_addr, 1)
            }
            // The window of a pseudo-unprotected VM, and the page the shim is told about it
            // through. Both are SHARE'd rather than lent -- that is the whole of the mode -- so
            // the host keeps its access, and needs it: the window IS the guest's RAM, and every
            // virtqueue the guest publishes lives in it. Gating them here is what a protected VM
            // needs and what this one exists to avoid.
            #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
            MemoryRegionPurpose::SharedGuestRam => Ok(()),
            #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
            MemoryRegionPurpose::ShimHandoff => Ok(()),
            #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
            MemoryRegionPurpose::SharedFramebuffer => Ok(()),
            #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
            MemoryRegionPurpose::StaticSwiotlbRegion => Ok(()),
            other => Err(Error::ProtectedMemoryAccess(guest_addr, other)),
        }
    }

    /// Returns the end address of memory.
    ///
    /// # Examples
    ///
    /// ```
    /// # use base::MemoryMapping;
    /// # use vm_memory::{GuestAddress, GuestMemory};
    /// # fn test_end_addr() -> Result<(), ()> {
    ///     let start_addr = GuestAddress(0x1000);
    ///     let mut gm = GuestMemory::new(&vec![(start_addr, 0x400)]).map_err(|_| ())?;
    ///     assert_eq!(start_addr.checked_add(0x400), Some(gm.end_addr()));
    ///     Ok(())
    /// # }
    /// ```
    pub fn end_addr(&self) -> GuestAddress {
        self.regions
            .iter()
            .max_by_key(|region| region.start())
            .map_or(GuestAddress(0), MemoryRegion::end)
    }

    /// Returns the guest addresses and sizes of the memory regions.
    pub fn guest_memory_regions(&self) -> Vec<(GuestAddress, usize)> {
        self.regions
            .iter()
            .map(|region| (region.guest_base, region.mapping.size()))
            .collect()
    }

    /// Returns the total size of memory in bytes.
    pub fn memory_size(&self) -> u64 {
        self.regions
            .iter()
            .map(|region| region.mapping.size() as u64)
            .sum()
    }

    /// Returns true if the given address is within the memory range available to the guest.
    pub fn address_in_range(&self, addr: GuestAddress) -> bool {
        self.regions.iter().any(|region| region.contains(addr))
    }

    /// Returns true if the given range (start, end) is overlap with the memory range
    /// available to the guest.
    pub fn range_overlap(&self, start: GuestAddress, end: GuestAddress) -> bool {
        self.regions
            .iter()
            .any(|region| region.start() < end && start < region.end())
    }

    /// Returns an address `addr + offset` if it's in range.
    ///
    /// This function doesn't care whether a region `[addr, addr + offset)` is in range or not. To
    /// guarantee it's a valid range, use `is_valid_range()` instead.
    pub fn checked_offset(&self, addr: GuestAddress, offset: u64) -> Option<GuestAddress> {
        addr.checked_add(offset).and_then(|a| {
            if self.address_in_range(a) {
                Some(a)
            } else {
                None
            }
        })
    }

    /// Returns true if the given range `[start, start + length)` is a valid contiguous memory
    /// range available to the guest and it's backed by a single underlying memory region.
    pub fn is_valid_range(&self, start: GuestAddress, length: u64) -> bool {
        if length == 0 {
            return false;
        }

        let end = if let Some(end) = start.checked_add(length - 1) {
            end
        } else {
            return false;
        };

        self.regions
            .iter()
            .any(|region| region.start() <= start && end < region.end())
    }

    /// Returns the size of the memory region in bytes.
    pub fn num_regions(&self) -> u64 {
        self.regions.len() as u64
    }

    pub fn regions(&self) -> impl Iterator<Item = MemoryRegionInformation> {
        self.regions
            .iter()
            .enumerate()
            .map(|(index, region)| MemoryRegionInformation {
                index,
                guest_addr: region.start(),
                size: region.mapping.size(),
                host_addr: region.mapping.as_ptr() as usize,
                shm: &region.shared_obj,
                shm_offset: region.obj_offset,
                options: region.options.clone(),
            })
    }

    /// Writes a slice to guest memory at the specified guest address.
    /// Returns the number of bytes written.  The number of bytes written can
    /// be less than the length of the slice if there isn't enough room in the
    /// memory region.
    ///
    /// # Examples
    /// * Write a slice at guestaddress 0x200.
    ///
    /// ```
    /// # use base::MemoryMapping;
    /// # use vm_memory::{GuestAddress, GuestMemory};
    /// # fn test_write_u64() -> Result<(), ()> {
    /// #   let start_addr = GuestAddress(0x1000);
    /// #   let mut gm = GuestMemory::new(&vec![(start_addr, 0x400)]).map_err(|_| ())?;
    ///     let res = gm.write_at_addr(&[1,2,3,4,5], GuestAddress(0x200)).map_err(|_| ())?;
    ///     assert_eq!(5, res);
    ///     Ok(())
    /// # }
    /// ```
    pub fn write_at_addr(&self, buf: &[u8], guest_addr: GuestAddress) -> Result<usize> {
        self.check_host_access_range(guest_addr, buf.len())?;
        let (mapping, offset, _) = self.find_region(guest_addr)?;
        mapping
            .write_slice(buf, offset)
            .map_err(|e| Error::MemoryAccess(guest_addr, e))
    }

    /// Writes the entire contents of a slice to guest memory at the specified
    /// guest address.
    ///
    /// Returns an error if there isn't enough room in the memory region to
    /// complete the entire write. Part of the data may have been written
    /// nevertheless.
    ///
    /// # Examples
    ///
    /// ```
    /// use vm_memory::{guest_memory, GuestAddress, GuestMemory};
    ///
    /// fn test_write_all() -> guest_memory::Result<()> {
    ///     let ranges = &[(GuestAddress(0x1000), 0x400)];
    ///     let gm = GuestMemory::new(ranges)?;
    ///     gm.write_all_at_addr(b"zyxwvut", GuestAddress(0x1200))
    /// }
    /// ```
    pub fn write_all_at_addr(&self, buf: &[u8], guest_addr: GuestAddress) -> Result<()> {
        let expected = buf.len();
        let completed = self.write_at_addr(buf, guest_addr)?;
        if expected == completed {
            Ok(())
        } else {
            Err(Error::ShortWrite {
                expected,
                completed,
            })
        }
    }

    /// Reads to a slice from guest memory at the specified guest address.
    /// Returns the number of bytes read.  The number of bytes read can
    /// be less than the length of the slice if there isn't enough room in the
    /// memory region.
    ///
    /// # Examples
    /// * Read a slice of length 16 at guestaddress 0x200.
    ///
    /// ```
    /// # use base::MemoryMapping;
    /// # use vm_memory::{GuestAddress, GuestMemory};
    /// # fn test_write_u64() -> Result<(), ()> {
    /// #   let start_addr = GuestAddress(0x1000);
    /// #   let mut gm = GuestMemory::new(&vec![(start_addr, 0x400)]).map_err(|_| ())?;
    ///     let buf = &mut [0u8; 16];
    ///     let res = gm.read_at_addr(buf, GuestAddress(0x200)).map_err(|_| ())?;
    ///     assert_eq!(16, res);
    ///     Ok(())
    /// # }
    /// ```
    pub fn read_at_addr(&self, buf: &mut [u8], guest_addr: GuestAddress) -> Result<usize> {
        self.check_host_access_range(guest_addr, buf.len())?;
        let (mapping, offset, _) = self.find_region(guest_addr)?;
        mapping
            .read_slice(buf, offset)
            .map_err(|e| Error::MemoryAccess(guest_addr, e))
    }

    /// Reads from guest memory at the specified address to fill the entire
    /// buffer.
    ///
    /// Returns an error if there isn't enough room in the memory region to fill
    /// the entire buffer. Part of the buffer may have been filled nevertheless.
    ///
    /// # Examples
    ///
    /// ```
    /// use vm_memory::{guest_memory, GuestAddress, GuestMemory};
    ///
    /// fn test_read_exact() -> guest_memory::Result<()> {
    ///     let ranges = &[(GuestAddress(0x1000), 0x400)];
    ///     let gm = GuestMemory::new(ranges)?;
    ///     let mut buffer = [0u8; 0x200];
    ///     gm.read_exact_at_addr(&mut buffer, GuestAddress(0x1200))
    /// }
    /// ```
    pub fn read_exact_at_addr(&self, buf: &mut [u8], guest_addr: GuestAddress) -> Result<()> {
        let expected = buf.len();
        let completed = self.read_at_addr(buf, guest_addr)?;
        if expected == completed {
            Ok(())
        } else {
            Err(Error::ShortRead {
                expected,
                completed,
            })
        }
    }

    /// Reads an object from guest memory at the given guest address.
    ///
    /// # Examples
    /// * Read a u64 from two areas of guest memory backed by separate mappings.
    ///
    /// ```
    /// # use vm_memory::{GuestAddress, GuestMemory};
    /// # fn test_read_u64() -> Result<u64, ()> {
    /// #     let start_addr1 = GuestAddress(0x0);
    /// #     let start_addr2 = GuestAddress(0x400);
    /// #     let mut gm = GuestMemory::new(&vec![(start_addr1, 0x400), (start_addr2, 0x400)])
    /// #         .map_err(|_| ())?;
    ///       let num1: u64 = gm.read_obj_from_addr(GuestAddress(32)).map_err(|_| ())?;
    ///       let num2: u64 = gm.read_obj_from_addr(GuestAddress(0x400+32)).map_err(|_| ())?;
    /// #     Ok(num1 + num2)
    /// # }
    /// ```
    pub fn read_obj_from_addr<T: FromBytes>(&self, guest_addr: GuestAddress) -> Result<T> {
        self.check_host_access_range(guest_addr, std::mem::size_of::<T>())?;
        let (mapping, offset, _) = self.find_region(guest_addr)?;
        mapping
            .read_obj(offset)
            .map_err(|e| Error::MemoryAccess(guest_addr, e))
    }

    /// Reads an object from guest memory at the given guest address.
    /// Reading from a volatile area isn't strictly safe as it could change
    /// mid-read.  However, as long as the type T is plain old data and can
    /// handle random initialization, everything will be OK.
    ///
    /// The read operation will be volatile, i.e. it will not be reordered by
    /// the compiler and is suitable for I/O, but must be aligned. When reading
    /// from regular memory, prefer [`GuestMemory::read_obj_from_addr`].
    ///
    /// # Examples
    /// * Read a u64 from two areas of guest memory backed by separate mappings.
    ///
    /// ```
    /// # use vm_memory::{GuestAddress, GuestMemory};
    /// # fn test_read_u64() -> Result<u64, ()> {
    /// #     let start_addr1 = GuestAddress(0x0);
    /// #     let start_addr2 = GuestAddress(0x400);
    /// #     let mut gm = GuestMemory::new(&vec![(start_addr1, 0x400), (start_addr2, 0x400)])
    /// #         .map_err(|_| ())?;
    ///       let num1: u64 = gm.read_obj_from_addr_volatile(GuestAddress(32)).map_err(|_| ())?;
    ///       let num2: u64 = gm.read_obj_from_addr_volatile(GuestAddress(0x400+32)).map_err(|_| ())?;
    /// #     Ok(num1 + num2)
    /// # }
    /// ```
    pub fn read_obj_from_addr_volatile<T: FromBytes>(&self, guest_addr: GuestAddress) -> Result<T> {
        self.check_host_access_range(guest_addr, std::mem::size_of::<T>())?;
        let (mapping, offset, _) = self.find_region(guest_addr)?;
        mapping
            .read_obj_volatile(offset)
            .map_err(|e| Error::MemoryAccess(guest_addr, e))
    }

    /// Writes an object to the memory region at the specified guest address.
    /// Returns Ok(()) if the object fits, or Err if it extends past the end.
    ///
    /// # Examples
    /// * Write a u64 at guest address 0x1100.
    ///
    /// ```
    /// # use vm_memory::{GuestAddress, GuestMemory};
    /// # fn test_write_u64() -> Result<(), ()> {
    /// #   let start_addr = GuestAddress(0x1000);
    /// #   let mut gm = GuestMemory::new(&vec![(start_addr, 0x400)]).map_err(|_| ())?;
    ///     gm.write_obj_at_addr(55u64, GuestAddress(0x1100))
    ///         .map_err(|_| ())
    /// # }
    /// ```
    pub fn write_obj_at_addr<T: IntoBytes + Immutable>(
        &self,
        val: T,
        guest_addr: GuestAddress,
    ) -> Result<()> {
        self.check_host_access_range(guest_addr, std::mem::size_of::<T>())?;
        let (mapping, offset, _) = self.find_region(guest_addr)?;
        mapping
            .write_obj(val, offset)
            .map_err(|e| Error::MemoryAccess(guest_addr, e))
    }

    /// Writes an object to the memory region at the specified guest address.
    /// Returns Ok(()) if the object fits, or Err if it extends past the end.
    ///
    /// The write operation will be volatile, i.e. it will not be reordered by
    /// the compiler and is suitable for I/O, but must be aligned. When writing
    /// to regular memory, prefer [`GuestMemory::write_obj_at_addr`].
    /// # Examples
    /// * Write a u64 at guest address 0x1100.
    ///
    /// ```
    /// # use vm_memory::{GuestAddress, GuestMemory};
    /// # fn test_write_u64() -> Result<(), ()> {
    /// #   let start_addr = GuestAddress(0x1000);
    /// #   let mut gm = GuestMemory::new(&vec![(start_addr, 0x400)]).map_err(|_| ())?;
    ///     gm.write_obj_at_addr_volatile(55u64, GuestAddress(0x1100))
    ///         .map_err(|_| ())
    /// # }
    /// ```
    pub fn write_obj_at_addr_volatile<T: IntoBytes + Immutable>(
        &self,
        val: T,
        guest_addr: GuestAddress,
    ) -> Result<()> {
        self.check_host_access_range(guest_addr, std::mem::size_of::<T>())?;
        let (mapping, offset, _) = self.find_region(guest_addr)?;
        mapping
            .write_obj_volatile(val, offset)
            .map_err(|e| Error::MemoryAccess(guest_addr, e))
    }

    /// Returns a `VolatileSlice` of `len` bytes starting at `addr`. Returns an error if the slice
    /// is not a subset of this `GuestMemory`.
    ///
    /// # Examples
    /// * Write `99` to 30 bytes starting at guest address 0x1010.
    ///
    /// ```
    /// # use base::MemoryMapping;
    /// # use vm_memory::{GuestAddress, GuestMemory, GuestMemoryError};
    /// # fn test_volatile_slice() -> Result<(), GuestMemoryError> {
    /// #   let start_addr = GuestAddress(0x1000);
    /// #   let mut gm = GuestMemory::new(&vec![(start_addr, 0x400)])?;
    ///     let vslice = gm.get_slice_at_addr(GuestAddress(0x1010), 30)?;
    ///     vslice.write_bytes(99);
    /// #   Ok(())
    /// # }
    /// ```
    pub fn get_slice_at_addr(&self, addr: GuestAddress, len: usize) -> Result<VolatileSlice> {
        self.check_host_access_range(addr, len)?;
        self.regions
            .iter()
            .find(|region| region.contains(addr))
            .ok_or(Error::InvalidGuestAddress(addr))
            .and_then(|region| {
                // The cast to a usize is safe here because we know that `region.contains(addr)` and
                // it's not possible for a memory region to be larger than what fits in a usize.
                region
                    .mapping
                    .get_slice(addr.offset_from(region.start()) as usize, len)
                    .map_err(Error::VolatileMemoryAccess)
            })
    }
    /// Convert a GuestAddress into a pointer in the address space of this
    /// process. This should only be necessary for giving addresses to the
    /// kernel, as with vhost ioctls. Normal reads/writes to guest memory should
    /// be done through `write_obj_at_addr`, `read_obj_from_addr`, etc.
    ///
    /// # Arguments
    /// * `guest_addr` - Guest address to convert.
    ///
    /// # Examples
    ///
    /// ```
    /// # use vm_memory::{GuestAddress, GuestMemory};
    /// # fn test_host_addr() -> Result<(), ()> {
    ///     let start_addr = GuestAddress(0x1000);
    ///     let mut gm = GuestMemory::new(&vec![(start_addr, 0x500)]).map_err(|_| ())?;
    ///     let addr = gm.get_host_address(GuestAddress(0x1200)).unwrap();
    ///     println!("Host address is {:p}", addr);
    ///     Ok(())
    /// # }
    /// ```
    pub fn get_host_address(&self, guest_addr: GuestAddress) -> Result<*const u8> {
        self.check_host_access(guest_addr)?;
        let (mapping, offset, _) = self.find_region(guest_addr)?;
        Ok(
            // SAFETY:
            // This is safe; `find_region` already checks that offset is in
            // bounds.
            unsafe { mapping.as_ptr().add(offset) } as *const u8,
        )
    }

    /// Convert a GuestAddress into a pointer in the address space of this
    /// process, and verify that the provided size define a valid range within
    /// a single memory region. Similar to get_host_address(), this should only
    /// be used for giving addresses to the kernel.
    ///
    /// # Arguments
    /// * `guest_addr` - Guest address to convert.
    /// * `size` - Size of the address range to be converted.
    ///
    /// # Examples
    ///
    /// ```
    /// # use vm_memory::{GuestAddress, GuestMemory};
    /// # fn test_host_addr() -> Result<(), ()> {
    ///     let start_addr = GuestAddress(0x1000);
    ///     let mut gm = GuestMemory::new(&vec![(start_addr, 0x500)]).map_err(|_| ())?;
    ///     let addr = gm.get_host_address_range(GuestAddress(0x1200), 0x200).unwrap();
    ///     println!("Host address is {:p}", addr);
    ///     Ok(())
    /// # }
    /// ```
    pub fn get_host_address_range(
        &self,
        guest_addr: GuestAddress,
        size: usize,
    ) -> Result<*const u8> {
        if size == 0 {
            return Err(Error::InvalidSize(size));
        }

        self.check_host_access_range(guest_addr, size)?;

        // Assume no overlap among regions
        let (mapping, offset, _) = self.find_region(guest_addr)?;

        if mapping
            .size()
            .checked_sub(offset)
            .map_or(true, |v| v < size)
        {
            return Err(Error::InvalidGuestAddress(guest_addr));
        }

        Ok(
            //SAFETY:
            // This is safe; `find_region` already checks that offset is in
            // bounds.
            unsafe { mapping.as_ptr().add(offset) } as *const u8,
        )
    }

    /// Returns a reference to the region that backs the given address.
    pub fn shm_region(
        &self,
        guest_addr: GuestAddress,
    ) -> Result<&(dyn AsRawDescriptor + Send + Sync)> {
        self.regions
            .iter()
            .find(|region| region.contains(guest_addr))
            .ok_or(Error::InvalidGuestAddress(guest_addr))
            .map(|region| region.shared_obj.as_ref())
    }

    /// Returns the region that contains the memory at `offset` from the base of guest memory.
    pub fn offset_region(&self, offset: u64) -> Result<&(dyn AsRawDescriptor + Send + Sync)> {
        self.shm_region(
            self.checked_offset(self.regions[0].guest_base, offset)
                .ok_or(Error::InvalidOffset(offset))?,
        )
    }

    /// Loops over all guest memory regions of `self`, and returns the
    /// target region that contains `guest_addr`. On success, this
    /// function returns a tuple with the following fields:
    ///
    /// (i) the memory mapping associated with the target region.
    /// (ii) the relative offset from the start of the target region to `guest_addr`.
    /// (iii) the absolute offset from the start of the backing object to the target region.
    ///
    /// If no target region is found, an error is returned.
    pub fn find_region(&self, guest_addr: GuestAddress) -> Result<(&MemoryMapping, usize, u64)> {
        self.regions
            .iter()
            .find(|region| region.contains(guest_addr))
            .ok_or(Error::InvalidGuestAddress(guest_addr))
            .map(|region| {
                (
                    &region.mapping,
                    guest_addr.offset_from(region.start()) as usize,
                    region.obj_offset,
                )
            })
    }

    /// Convert a GuestAddress into an offset within the associated shm region.
    ///
    /// A `GuestMemory` may have multiple backing objects and the offset is
    /// only meaningful in relation to the associated backing object, so a
    /// reference to it is included in the return value.
    ///
    /// Due to potential gaps within GuestMemory, it is helpful to know the
    /// offset within the shm where a given address is found. This offset
    /// can then be passed to another process mapping the shm to read data
    /// starting at that address.
    ///
    /// # Arguments
    /// * `guest_addr` - Guest address to convert.
    ///
    /// # Examples
    ///
    /// ```
    /// # use vm_memory::{GuestAddress, GuestMemory};
    /// let addr_a = GuestAddress(0x10000);
    /// let addr_b = GuestAddress(0x80000);
    /// let mut gm = GuestMemory::new(&vec![
    ///     (addr_a, 0x20000),
    ///     (addr_b, 0x30000)]).expect("failed to create GuestMemory");
    /// let (_backing_object, offset) = gm.offset_from_base(GuestAddress(0x95000))
    ///                .expect("failed to get offset");
    /// assert_eq!(offset, 0x35000);
    /// ```
    pub fn offset_from_base(
        &self,
        guest_addr: GuestAddress,
    ) -> Result<(&(dyn AsRawDescriptor + Send + Sync), u64)> {
        self.regions
            .iter()
            .find(|region| region.contains(guest_addr))
            .ok_or(Error::InvalidGuestAddress(guest_addr))
            .map(|region| {
                (
                    region.shared_obj.as_ref(),
                    region.obj_offset + guest_addr.offset_from(region.start()),
                )
            })
    }

    /// Copy all guest memory into `w`.
    ///
    /// # Safety
    /// Must have exclusive access to the guest memory for the duration of the
    /// call (e.g. all vCPUs and devices must be stopped).
    ///
    /// Returns a JSON object that contains metadata about the underlying memory regions to allow
    /// validation checks at restore time.
    #[deny(unsafe_op_in_unsafe_fn)]
    pub unsafe fn snapshot<T: Write>(
        &self,
        w: &mut T,
        compress: bool,
    ) -> anyhow::Result<AnySnapshot> {
        fn go(
            this: &GuestMemory,
            w: &mut impl Write,
        ) -> anyhow::Result<Vec<MemoryRegionSnapshotMetadata>> {
            let mut regions = Vec::new();
            for region in this.regions.iter() {
                let data_ranges = region
                    .find_data_ranges()
                    .context("find_data_ranges failed")?;
                for range in &data_ranges {
                    let region_vslice = region
                        .mapping
                        .get_slice(range.start, range.end - range.start)?;
                    // SAFETY:
                    // 1. The data is guaranteed to be present & of expected length by the
                    //    `VolatileSlice`.
                    // 2. Aliasing the `VolatileSlice`'s memory is safe because a. The only mutable
                    //    reference to it is held by the guest, and the guest's VCPUs are stopped
                    //    (guaranteed by caller), so that mutable reference can be ignored (aliasing
                    //    is only an issue if temporal overlap occurs, and it does not here). b.
                    //    Some host code does manipulate guest memory through raw pointers. This
                    //    aliases the underlying memory of the slice, so we must ensure that host
                    //    code is not running (the caller guarantees this).
                    w.write_all(unsafe {
                        std::slice::from_raw_parts(region_vslice.as_ptr(), region_vslice.size())
                    })?;
                }
                regions.push(MemoryRegionSnapshotMetadata {
                    guest_base: region.guest_base.0,
                    size: region.mapping.size(),
                    data_ranges,
                });
            }
            Ok(regions)
        }

        let regions = if compress {
            let mut w = lz4_flex::frame::FrameEncoder::new(w);
            let regions = go(self, &mut w)?;
            w.finish()?;
            regions
        } else {
            go(self, w)?
        };

        AnySnapshot::to_any(MemorySnapshotMetadata {
            regions,
            compressed: compress,
        })
    }

    /// Restore the guest memory using the bytes from `r`.
    ///
    /// # Safety
    /// Must have exclusive access to the guest memory for the duration of the
    /// call (e.g. all vCPUs and devices must be stopped).
    ///
    /// Returns an error if `metadata` doesn't match the configuration of the `GuestMemory` or if
    /// `r` doesn't produce exactly as many bytes as needed.
    #[deny(unsafe_op_in_unsafe_fn)]
    pub unsafe fn restore<T: Read>(&self, metadata: AnySnapshot, r: &mut T) -> anyhow::Result<()> {
        let metadata: MemorySnapshotMetadata = AnySnapshot::from_any(metadata)?;

        let mut r: Box<dyn Read> = if metadata.compressed {
            Box::new(lz4_flex::frame::FrameDecoder::new(r))
        } else {
            Box::new(r)
        };

        if self.regions.len() != metadata.regions.len() {
            bail!(
                "snapshot expected {} memory regions but VM has {}",
                metadata.regions.len(),
                self.regions.len()
            );
        }
        for (region, metadata) in self.regions.iter().zip(metadata.regions.iter()) {
            let MemoryRegionSnapshotMetadata {
                guest_base,
                size,
                data_ranges,
            } = metadata;
            if region.guest_base.0 != *guest_base || region.mapping.size() != *size {
                bail!("snapshot memory regions don't match VM memory regions");
            }

            let mut prev_end = 0;
            for range in data_ranges {
                let hole_size = range
                    .start
                    .checked_sub(prev_end)
                    .context("invalid data range")?;
                if hole_size > 0 {
                    region.zero_range(prev_end, hole_size)?;
                }
                let region_vslice = region
                    .mapping
                    .get_slice(range.start, range.end - range.start)?;

                // SAFETY:
                // See `Self::snapshot` for the detailed safety statement, and
                // note that both mutable and non-mutable aliasing is safe.
                r.read_exact(unsafe {
                    std::slice::from_raw_parts_mut(region_vslice.as_mut_ptr(), region_vslice.size())
                })?;

                prev_end = range.end;
            }
            let hole_size = region
                .mapping
                .size()
                .checked_sub(prev_end)
                .context("invalid data range")?;
            if hole_size > 0 {
                region.zero_range(prev_end, hole_size)?;
            }
        }

        // Should always be at EOF at this point.
        let mut buf = [0];
        if r.read(&mut buf)? != 0 {
            bail!("too many bytes");
        }

        Ok(())
    }
}

#[derive(Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
struct MemorySnapshotMetadata {
    regions: Vec<MemoryRegionSnapshotMetadata>,
    compressed: bool,
}

#[derive(Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
struct MemoryRegionSnapshotMetadata {
    guest_base: u64,
    size: usize,
    // Ranges of the mmap that are stored in the snapshot file. All other ranges of the region are
    // zeros.
    data_ranges: Vec<std::ops::Range<usize>>,
}

// SAFETY:
// It is safe to implement BackingMemory because GuestMemory can be mutated any time already.
unsafe impl BackingMemory for GuestMemory {
    fn get_volatile_slice(
        &self,
        mem_range: cros_async::MemRegion,
    ) -> mem::Result<VolatileSlice<'_>> {
        self.get_slice_at_addr(GuestAddress(mem_range.offset), mem_range.len)
            .map_err(|_| mem::Error::InvalidOffset(mem_range.offset, mem_range.len))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_alignment() {
        let start_addr1 = GuestAddress(0x0);
        let start_addr2 = GuestAddress(0x10000);

        assert!(GuestMemory::new(&[(start_addr1, 0x100), (start_addr2, 0x400)]).is_err());
        assert!(GuestMemory::new(&[(start_addr1, 0x10000), (start_addr2, 0x10000)]).is_ok());
    }

    #[test]
    fn two_regions() {
        let start_addr1 = GuestAddress(0x0);
        let start_addr2 = GuestAddress(0x10000);
        // The memory regions are `[0x0, 0x10000)`, `[0x10000, 0x20000)`.
        let gm = GuestMemory::new(&[(start_addr1, 0x10000), (start_addr2, 0x10000)]).unwrap();

        // Although each address in `[0x0, 0x20000)` is valid, `is_valid_range()` returns false for
        // a range that is across multiple underlying regions.
        assert!(gm.is_valid_range(GuestAddress(0x5000), 0x5000));
        assert!(gm.is_valid_range(GuestAddress(0x10000), 0x5000));
        assert!(!gm.is_valid_range(GuestAddress(0x5000), 0x10000));
    }

    #[test]
    fn overlap_memory() {
        let start_addr1 = GuestAddress(0x0);
        let start_addr2 = GuestAddress(0x10000);
        assert!(GuestMemory::new(&[(start_addr1, 0x20000), (start_addr2, 0x20000)]).is_err());
    }

    #[test]
    fn region_hole() {
        let start_addr1 = GuestAddress(0x0);
        let start_addr2 = GuestAddress(0x40000);
        // The memory regions are `[0x0, 0x20000)`, `[0x40000, 0x60000)`.
        let gm = GuestMemory::new(&[(start_addr1, 0x20000), (start_addr2, 0x20000)]).unwrap();

        assert!(gm.address_in_range(GuestAddress(0x10000)));
        assert!(!gm.address_in_range(GuestAddress(0x30000)));
        assert!(gm.address_in_range(GuestAddress(0x50000)));
        assert!(!gm.address_in_range(GuestAddress(0x60000)));
        assert!(!gm.address_in_range(GuestAddress(0x60000)));
        assert!(gm.range_overlap(GuestAddress(0x10000), GuestAddress(0x30000)),);
        assert!(!gm.range_overlap(GuestAddress(0x30000), GuestAddress(0x40000)),);
        assert!(gm.range_overlap(GuestAddress(0x30000), GuestAddress(0x70000)),);
        assert_eq!(gm.checked_offset(GuestAddress(0x10000), 0x10000), None);
        assert_eq!(
            gm.checked_offset(GuestAddress(0x50000), 0x8000),
            Some(GuestAddress(0x58000))
        );
        assert_eq!(gm.checked_offset(GuestAddress(0x50000), 0x10000), None);
        assert!(gm.is_valid_range(GuestAddress(0x0), 0x10000));
        assert!(gm.is_valid_range(GuestAddress(0x0), 0x20000));
        assert!(!gm.is_valid_range(GuestAddress(0x0), 0x20000 + 1));

        // While `checked_offset(GuestAddress(0x10000), 0x40000)` succeeds because 0x50000 is a
        // valid address, `is_valid_range(GuestAddress(0x10000), 0x40000)` returns `false`
        // because there is a hole inside of [0x10000, 0x50000).
        assert_eq!(
            gm.checked_offset(GuestAddress(0x10000), 0x40000),
            Some(GuestAddress(0x50000))
        );
        assert!(!gm.is_valid_range(GuestAddress(0x10000), 0x40000));
    }

    #[test]
    fn test_read_u64() {
        let start_addr1 = GuestAddress(0x0);
        let start_addr2 = GuestAddress(0x10000);
        let gm = GuestMemory::new(&[(start_addr1, 0x10000), (start_addr2, 0x10000)]).unwrap();

        let val1: u64 = 0xaa55aa55aa55aa55;
        let val2: u64 = 0x55aa55aa55aa55aa;
        gm.write_obj_at_addr(val1, GuestAddress(0x500)).unwrap();
        gm.write_obj_at_addr(val2, GuestAddress(0x10000 + 32))
            .unwrap();
        let num1: u64 = gm.read_obj_from_addr(GuestAddress(0x500)).unwrap();
        let num2: u64 = gm.read_obj_from_addr(GuestAddress(0x10000 + 32)).unwrap();
        assert_eq!(val1, num1);
        assert_eq!(val2, num2);
    }

    #[test]
    fn test_memory_size() {
        let start_region1 = GuestAddress(0x0);
        let size_region1 = 0x10000;
        let start_region2 = GuestAddress(0x10000);
        let size_region2 = 0x20000;
        let gm = GuestMemory::new(&[(start_region1, size_region1), (start_region2, size_region2)])
            .unwrap();

        let mem_size = gm.memory_size();
        assert_eq!(mem_size, size_region1 + size_region2);
    }

    // Get the base address of the mapping for a GuestAddress.
    fn get_mapping(mem: &GuestMemory, addr: GuestAddress) -> Result<*const u8> {
        Ok(mem.find_region(addr)?.0.as_ptr() as *const u8)
    }

    #[test]
    fn guest_to_host() {
        let start_addr1 = GuestAddress(0x0);
        let start_addr2 = GuestAddress(0x10000);
        let mem = GuestMemory::new(&[(start_addr1, 0x10000), (start_addr2, 0x40000)]).unwrap();

        // Verify the host addresses match what we expect from the mappings.
        let addr1_base = get_mapping(&mem, start_addr1).unwrap();
        let addr2_base = get_mapping(&mem, start_addr2).unwrap();
        let host_addr1 = mem.get_host_address(start_addr1).unwrap();
        let host_addr2 = mem.get_host_address(start_addr2).unwrap();
        assert_eq!(host_addr1, addr1_base);
        assert_eq!(host_addr2, addr2_base);

        // Check that a bad address returns an error.
        let bad_addr = GuestAddress(0x123456);
        assert!(mem.get_host_address(bad_addr).is_err());
    }

    #[test]
    fn guest_to_host_range() {
        let start_addr1 = GuestAddress(0x0);
        let start_addr2 = GuestAddress(0x10000);
        let mem = GuestMemory::new(&[(start_addr1, 0x10000), (start_addr2, 0x40000)]).unwrap();

        // Verify the host addresses match what we expect from the mappings.
        let addr1_base = get_mapping(&mem, start_addr1).unwrap();
        let addr2_base = get_mapping(&mem, start_addr2).unwrap();
        let host_addr1 = mem.get_host_address_range(start_addr1, 0x10000).unwrap();
        let host_addr2 = mem.get_host_address_range(start_addr2, 0x10000).unwrap();
        assert_eq!(host_addr1, addr1_base);
        assert_eq!(host_addr2, addr2_base);

        let host_addr3 = mem.get_host_address_range(start_addr2, 0x20000).unwrap();
        assert_eq!(host_addr3, addr2_base);

        // Check that a valid guest address with an invalid size returns an error.
        assert!(mem.get_host_address_range(start_addr1, 0x20000).is_err());

        // Check that a bad address returns an error.
        let bad_addr = GuestAddress(0x123456);
        assert!(mem.get_host_address_range(bad_addr, 0x10000).is_err());
    }

    #[test]
    fn shm_offset() {
        let start_region1 = GuestAddress(0x0);
        let size_region1 = 0x10000;
        let start_region2 = GuestAddress(0x10000);
        let size_region2 = 0x20000;
        let gm = GuestMemory::new(&[(start_region1, size_region1), (start_region2, size_region2)])
            .unwrap();

        gm.write_obj_at_addr(0x1337u16, GuestAddress(0x0)).unwrap();
        gm.write_obj_at_addr(0x0420u16, GuestAddress(0x10000))
            .unwrap();

        for region in gm.regions() {
            let shm = match region.shm {
                BackingObject::Shm(s) => s,
                _ => {
                    panic!("backing object isn't SharedMemory");
                }
            };
            let mmap = MemoryMappingBuilder::new(region.size)
                .from_shared_memory(shm)
                .offset(region.shm_offset)
                .build()
                .unwrap();

            if region.index == 0 {
                assert!(mmap.read_obj::<u16>(0x0).unwrap() == 0x1337u16);
            }

            if region.index == 1 {
                assert!(mmap.read_obj::<u16>(0x0).unwrap() == 0x0420u16);
            }
        }
    }

    #[test]
    // Disabled for non-x86 because test infra uses qemu-user, which doesn't support MADV_REMOVE.
    #[cfg(target_arch = "x86_64")]
    fn snapshot_restore() {
        let regions = &[
            // Hole at start.
            (GuestAddress(0x0), 0x10000),
            // Hole at end.
            (GuestAddress(0x10000), 0x10000),
            // Hole in middle.
            (GuestAddress(0x20000), 0x10000),
            // All holes.
            (GuestAddress(0x30000), 0x10000),
            // No holes.
            (GuestAddress(0x40000), 0x1000),
        ];
        let writes = &[
            (GuestAddress(0x0FFF0), 1u64),
            (GuestAddress(0x10000), 2u64),
            (GuestAddress(0x29000), 3u64),
            (GuestAddress(0x40000), 4u64),
        ];

        let gm = GuestMemory::new(regions).unwrap();
        for &(addr, value) in writes {
            gm.write_obj_at_addr(value, addr).unwrap();
        }

        let mut data = tempfile::tempfile().unwrap();
        // SAFETY:
        // no vm is running
        let metadata_json = unsafe { gm.snapshot(&mut data, false).unwrap() };
        let metadata: MemorySnapshotMetadata =
            AnySnapshot::from_any(metadata_json.clone()).unwrap();

        #[cfg(unix)]
        assert_eq!(
            metadata,
            MemorySnapshotMetadata {
                regions: vec![
                    MemoryRegionSnapshotMetadata {
                        guest_base: 0,
                        size: 0x10000,
                        data_ranges: vec![0x0F000..0x10000],
                    },
                    MemoryRegionSnapshotMetadata {
                        guest_base: 0x10000,
                        size: 0x10000,
                        data_ranges: vec![0x00000..0x01000],
                    },
                    MemoryRegionSnapshotMetadata {
                        guest_base: 0x20000,
                        size: 0x10000,
                        data_ranges: vec![0x09000..0x0A000],
                    },
                    MemoryRegionSnapshotMetadata {
                        guest_base: 0x30000,
                        size: 0x10000,
                        data_ranges: vec![],
                    },
                    MemoryRegionSnapshotMetadata {
                        guest_base: 0x40000,
                        size: 0x1000,
                        data_ranges: vec![0x00000..0x01000],
                    }
                ],
                compressed: false,
            }
        );
        // We can't detect the holes on Windows yet.
        #[cfg(windows)]
        assert_eq!(
            metadata,
            MemorySnapshotMetadata {
                regions: vec![
                    MemoryRegionSnapshotMetadata {
                        guest_base: 0,
                        size: 0x10000,
                        data_ranges: vec![0x00000..0x10000],
                    },
                    MemoryRegionSnapshotMetadata {
                        guest_base: 0x10000,
                        size: 0x10000,
                        data_ranges: vec![0x00000..0x10000],
                    },
                    MemoryRegionSnapshotMetadata {
                        guest_base: 0x20000,
                        size: 0x10000,
                        data_ranges: vec![0x00000..0x10000],
                    },
                    MemoryRegionSnapshotMetadata {
                        guest_base: 0x30000,
                        size: 0x10000,
                        data_ranges: vec![0x00000..0x10000],
                    },
                    MemoryRegionSnapshotMetadata {
                        guest_base: 0x40000,
                        size: 0x1000,
                        data_ranges: vec![0x00000..0x01000],
                    }
                ],
                compressed: false,
            }
        );

        std::mem::drop(gm);

        let gm2 = GuestMemory::new(regions).unwrap();

        // Write to a hole so we can assert the restore zeroes it.
        let hole_addr = GuestAddress(0x30000);
        gm2.write_obj_at_addr(8u64, hole_addr).unwrap();

        use std::io::Seek;
        data.seek(std::io::SeekFrom::Start(0)).unwrap();
        // SAFETY:
        // no vm is running
        unsafe { gm2.restore(metadata_json, &mut data).unwrap() };

        assert_eq!(gm2.read_obj_from_addr::<u64>(hole_addr).unwrap(), 0);
        for &(addr, value) in writes {
            assert_eq!(gm2.read_obj_from_addr::<u64>(addr).unwrap(), value);
        }
    }
}
