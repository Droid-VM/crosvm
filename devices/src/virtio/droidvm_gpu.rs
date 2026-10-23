// Copyright 2026 DroidVM
// SPDX-License-Identifier: BSD-3-Clause

//! droidvm-gpu: the host side of UniXNU's paravirtual Adreno backend for an XNU guest
//! (UniXNU-droidvm package/DroidVMGpu.cpp, wire in package/DroidVMGpuWire.h).
//!
//! The guest keeps the native Adreno driver's user-client ABI, so its Turnip builds command streams
//! itself; this device only does what the hardware layer under that ABI did, on the host's KGSL:
//! - a BO is guest memory (the RAM of a pseudo-unprotected VM is shared with the host): IMPORT turns
//!   guest physical ranges into a udmabuf and KGSL imports it as one object;
//! - BIND places a range of an object at a guest-chosen GPU address inside the one VBO reserved at
//!   activation, which INFO reports as the guest's VA range;
//! - SUBMIT queues IBs with the guest's timestamp (user-generated timestamps) on the KGSL context of the guest client
//!   that sent them (NativeAdrenoPublish's slot: the render server, the widgets, WebKit, ...). The guest numbers its
//!   submissions in one sequence across its clients, so each context's timestamps rise; RETIRED is the guest's
//!   timeline across them (everything below the oldest submission still running anywhere), and WAIT waits for that one.
//!
//! One request at a time, handled synchronously by the worker; the guest polls the used ring.
//!
//! The guest is not trusted with the host's GPU or memory: whatever it sends, the worst it may do is lose its own GPU.
//! - IMPORT takes only memory host and guest share (the guest's RAM and the framebuffer of a pseudo-unprotected VM).
//!   The rest is lent to the VM, and a GPU access to memory the host has given away faults in the hypervisor or the
//!   firmware, below anything Linux can recover. Live imports are capped in count and in bytes.
//! - BIND stays inside the object it names and inside the VBO, page-aligned.
//! - The KGSL contexts are created without fault tolerance (KGSL invalidates one on a fault instead of replaying the
//!   guest's work) and in the lowest priority band, so the host's own GPU work goes first; a KGSL that refuses either
//!   keeps what it accepts. A context KGSL invalidates, or that caused a GPU reset, is destroyed and its client's work
//!   refused (EDEADLK): that client has lost its GPU, the others keep theirs. CLIENT starts a slot's next user afresh.
//! - Waits are bounded, so the worker always gets back to its kill event and the VM can stop.

use std::collections::BTreeMap;
use std::collections::VecDeque;
use std::fs::File;
use std::fs::OpenOptions;
use std::io;
use std::io::Write;

use anyhow::anyhow;
use anyhow::Context;
use base::error;
use base::info;
use base::warn;
use base::AsRawDescriptor;
use base::Event;
use base::EventToken;
use base::RawDescriptor;
use base::WaitContext;
use base::WorkerThread;
use vm_memory::udmabuf::UdmabufDriver;
use vm_memory::udmabuf::UdmabufDriverTrait;
use vm_memory::GuestAddress;
use vm_memory::GuestMemory;
use vm_memory::MemoryRegionPurpose;
use zerocopy::FromBytes;
use zerocopy::Immutable;
use zerocopy::IntoBytes;
use zerocopy::KnownLayout;

use super::DescriptorChain;
use super::DeviceType;
use super::Interrupt;
use super::Queue;
use super::VirtioDevice;

const QUEUE_SIZE: u16 = 16;
const QUEUE_SIZES: &[u16] = &[QUEUE_SIZE];

const WIRE_VERSION: u32 = 3;
const MAX_RANGES: u32 = 1024;
const MAX_IBS: u32 = 16;
const WAIT_MAX_MS: u32 = 4;
/// Live imports at most: each costs the host kernel page arrays, a scatter list and GPU page tables.
const MAX_OBJECTS: usize = 4096;
/// Live imported bytes at most, in units of the shared memory: the same pages may be imported more than once (one
/// IOSurface for two clients), but not without bound.
const MAX_IMPORTED_SHARES: u64 = 2;
/// An ioctl interrupted this many times in a row gives up, so the worker always returns to its kill event.
const EINTR_RETRIES: u32 = 8;
/// Every this many submissions the GPU reset status is read even when nothing failed.
const RESET_CHECK_INTERVAL: u64 = 64;
/// Refused requests logged at most (a guest can repeat one forever).
const REFUSALS_LOGGED: u64 = 16;
const PAGE: u64 = 4096;

const OP_INFO: u32 = 1;
const OP_IMPORT: u32 = 2;
const OP_FREE: u32 = 3;
const OP_BIND: u32 = 4;
const OP_SUBMIT: u32 = 5;
const OP_RETIRED: u32 = 6;
const OP_WAIT: u32 = 7;
const OP_TICKS: u32 = 8;
const OP_CLIENT: u32 = 9;

/// Guest clients (NativeAdrenoPublish's slots) with a KGSL context of their own: the guest's 32 slots, each a 1 GiB window of
/// the VBO (the wallpaper's poster extensions, widgets, Settings and WebKit's processes took the ten there were before).
const MAX_CLIENTS: u16 = 32;

const IMPORT_READ_ONLY: u32 = 1;
const IMPORT_COHERENT: u32 = 2;
const IMPORT_CACHED: u32 = 4;

#[repr(C)]
#[derive(Clone, Copy, Default, FromBytes, Immutable, IntoBytes, KnownLayout)]
struct Request {
    op: u32,
    bytes: u32,
}
#[repr(C)]
#[derive(Clone, Copy, Default, FromBytes, Immutable, IntoBytes, KnownLayout)]
struct Response {
    status: i32,
    bytes: u32,
}
#[repr(C)]
#[derive(Clone, Copy, Default, FromBytes, Immutable, IntoBytes, KnownLayout)]
struct Info {
    version: u32,
    chip_id: u32,
    kgsl_chip_id: u32,
    gmem_bytes: u32,
    gmem_base: u64,
    uche_trap_base: u64,
    va_start: u64,
    va_size: u64,
    highest_bank_bit: u32,
    ubwc_mode: u32,
    faults: u64,
}
#[repr(C)]
#[derive(Clone, Copy, Default, FromBytes, Immutable, IntoBytes, KnownLayout)]
struct Range {
    address: u64,
    bytes: u64,
}
#[repr(C)]
#[derive(Clone, Copy, Default, FromBytes, Immutable, IntoBytes, KnownLayout)]
struct Import {
    count: u32,
    flags: u32,
}
#[repr(C)]
#[derive(Clone, Copy, Default, FromBytes, Immutable, IntoBytes, KnownLayout)]
struct ImportReply {
    object: u32,
    coherent: u32,
    bytes: u64,
}
#[repr(C)]
#[derive(Clone, Copy, Default, FromBytes, Immutable, IntoBytes, KnownLayout)]
struct Free {
    object: u32,
    reserved: u32,
}
#[repr(C)]
#[derive(Clone, Copy, Default, FromBytes, Immutable, IntoBytes, KnownLayout)]
struct Bind {
    object: u32,
    op: u32,
    offset: u64,
    iova: u64,
    bytes: u64,
}
#[repr(C)]
#[derive(Clone, Copy, Default, FromBytes, Immutable, IntoBytes, KnownLayout)]
struct Ib {
    iova: u64,
    bytes: u64,
}
#[repr(C)]
#[derive(Clone, Copy, Default, FromBytes, Immutable, IntoBytes, KnownLayout)]
struct Submit {
    timestamp: u32,
    count: u16,
    client: u16,
}
#[repr(C)]
#[derive(Clone, Copy, Default, FromBytes, Immutable, IntoBytes, KnownLayout)]
struct ClientReset {
    client: u32,
    reserved: u32,
}
#[repr(C)]
#[derive(Clone, Copy, Default, FromBytes, Immutable, IntoBytes, KnownLayout)]
struct Timestamp {
    timestamp: u32,
    timeout_ms: u32,
}
#[repr(C)]
#[derive(Clone, Copy, Default, FromBytes, Immutable, IntoBytes, KnownLayout)]
struct Ticks {
    ticks: u64,
}

/// KGSL's uapi (msm_kgsl.h), the parts this device uses. arm64 layouts.
mod kgsl {
    pub const IOC_TYPE: u64 = 0x09;
    const fn ioc(dir: u64, nr: u64, size: usize) -> u64 {
        (dir << 30) | ((size as u64) << 16) | (IOC_TYPE << 8) | nr
    }
    const W: u64 = 1;
    const RW: u64 = 3;

    pub const PROP_DEVICE_INFO: u32 = 0x1;
    pub const PROP_GPU_RESET_STAT: u32 = 0x9;
    pub const PROP_UCHE_GMEM_VADDR: u32 = 0x13;
    pub const PROP_HIGHEST_BANK_BIT: u32 = 0x17;
    pub const PROP_UBWC_MODE: u32 = 0x1b;
    pub const PROP_UCHE_TRAP_BASE: u32 = 0x2f;

    pub const CONTEXT_SAVE_GMEM: u32 = 0x1;
    pub const CONTEXT_NO_GMEM_ALLOC: u32 = 0x2;
    pub const CONTEXT_SUBMIT_IB_LIST: u64 = 0x4;
    pub const CONTEXT_PREAMBLE: u32 = 0x10;
    pub const CONTEXT_USER_GENERATED_TS: u32 = 0x80;
    pub const CONTEXT_NO_FAULT_TOLERANCE: u32 = 0x200;
    pub const CONTEXT_PRIORITY_SHIFT: u32 = 12;
    /// The highest priority number is the lowest priority: KGSL places it in its last ringbuffer.
    pub const CONTEXT_PRIORITY_LOWEST: u32 = 0xf << CONTEXT_PRIORITY_SHIFT;
    pub const CTX_STAT_NO_ERROR: u32 = 0;
    pub const CTX_STAT_INNOCENT_CONTEXT_RESET_EXT: u32 = 2;
    pub const CMDLIST_IB: u32 = 0x1;
    pub const MEMFLAGS_GPUREADONLY: u64 = 1 << 24;
    pub const CACHEMODE_SHIFT: u64 = 26;
    pub const CACHEMODE_WRITECOMBINE: u64 = 0;
    pub const CACHEMODE_WRITEBACK: u64 = 3;
    pub const MEMFLAGS_IOCOHERENT: u64 = 1 << 31;
    pub const MEMFLAGS_VBO: u64 = 1 << 34;
    pub const USER_MEM_TYPE_DMABUF: u32 = 3;
    pub const TIMESTAMP_RETIRED: u32 = 2;

    #[repr(C)]
    #[derive(Default)]
    pub struct DevInfo {
        pub device_id: u32,
        pub chip_id: u32,
        pub mmu_enabled: u32,
        pub gmem_gpubaseaddr: u64,
        pub gpu_id: u32,
        pub gmem_sizebytes: u64,
    }
    #[repr(C)]
    pub struct GetProperty {
        pub type_: u32,
        pub value: u64,
        pub sizebytes: u64,
    }
    #[repr(C)]
    #[derive(Default)]
    pub struct GpuObjAlloc {
        pub size: u64,
        pub flags: u64,
        pub va_len: u64,
        pub mmapsize: u64,
        pub id: u32,
        pub metadata_len: u32,
        pub metadata: u64,
    }
    #[repr(C)]
    #[derive(Default)]
    pub struct GpuObjFree {
        pub flags: u64,
        pub priv_: u64,
        pub id: u32,
        pub type_: u32,
        pub len: u32,
    }
    #[repr(C)]
    #[derive(Default)]
    pub struct GpuObjInfo {
        pub gpuaddr: u64,
        pub flags: u64,
        pub size: u64,
        pub va_len: u64,
        pub va_addr: u64,
        pub id: u32,
    }
    #[repr(C)]
    #[derive(Default)]
    pub struct GpuObjImport {
        pub priv_: u64,
        pub priv_len: u64,
        pub flags: u64,
        pub type_: u32,
        pub id: u32,
    }
    #[repr(C)]
    pub struct GpuObjImportDmaBuf {
        pub fd: i32,
    }
    #[repr(C)]
    #[derive(Default)]
    pub struct BindRange {
        pub child_offset: u64,
        pub target_offset: u64,
        pub length: u64,
        pub child_id: u32,
        pub op: u32,
    }
    #[repr(C)]
    #[derive(Default)]
    pub struct BindRanges {
        pub ranges: u64,
        pub ranges_nents: u32,
        pub ranges_size: u32,
        pub id: u32,
        pub flags: u32,
        pub fence_id: i32,
        pub padding: u32,
    }
    #[repr(C)]
    #[derive(Default)]
    pub struct DrawCtxtCreate {
        pub flags: u32,
        pub drawctxt_id: u32,
    }
    #[repr(C)]
    #[derive(Default, Clone, Copy)]
    pub struct CommandObject {
        pub offset: u64,
        pub gpuaddr: u64,
        pub size: u64,
        pub flags: u32,
        pub id: u32,
    }
    /// `kgsl_command_syncpoint` of type `KGSL_CMD_SYNCPOINT_TYPE_TIMESTAMP`: the command waits until `context_id` retires
    /// `timestamp`, or its events are cancelled (the context invalidated or destroyed).
    pub const SYNCPOINT_TYPE_TIMESTAMP: u32 = 0;
    #[repr(C)]
    #[derive(Default, Clone, Copy)]
    pub struct CommandSyncpoint {
        pub priv_: u64,
        pub size: u64,
        pub type_: u32,
        pub pad: u32,
    }
    #[repr(C)]
    #[derive(Default, Clone, Copy)]
    pub struct SyncpointTimestamp {
        pub context_id: u32,
        pub timestamp: u32,
    }
    #[repr(C)]
    #[derive(Default)]
    pub struct GpuCommand {
        pub flags: u64,
        pub cmdlist: u64,
        pub cmdsize: u32,
        pub numcmds: u32,
        pub objlist: u64,
        pub objsize: u32,
        pub numobjs: u32,
        pub synclist: u64,
        pub syncsize: u32,
        pub numsyncs: u32,
        pub context_id: u32,
        pub timestamp: u32,
    }
    #[repr(C)]
    #[derive(Default)]
    pub struct WaitTimestamp {
        pub context_id: u32,
        pub timestamp: u32,
        pub timeout: u32,
    }
    #[repr(C)]
    #[derive(Default)]
    pub struct ReadTimestamp {
        pub context_id: u32,
        pub type_: u32,
        pub timestamp: u32,
    }

    pub const GETPROPERTY: u64 = ioc(RW, 0x2, std::mem::size_of::<GetProperty>());
    pub const WAITTIMESTAMP_CTXTID: u64 = ioc(W, 0x7, std::mem::size_of::<WaitTimestamp>());
    pub const DRAWCTXT_CREATE: u64 = ioc(RW, 0x13, std::mem::size_of::<DrawCtxtCreate>());
    pub const DRAWCTXT_DESTROY: u64 = ioc(W, 0x14, std::mem::size_of::<u32>());
    pub const READTIMESTAMP_CTXTID: u64 = ioc(RW, 0x16, std::mem::size_of::<ReadTimestamp>());
    pub const GPUOBJ_ALLOC: u64 = ioc(RW, 0x45, std::mem::size_of::<GpuObjAlloc>());
    pub const GPUOBJ_FREE: u64 = ioc(W, 0x46, std::mem::size_of::<GpuObjFree>());
    pub const GPUOBJ_INFO: u64 = ioc(RW, 0x47, std::mem::size_of::<GpuObjInfo>());
    pub const GPUOBJ_IMPORT: u64 = ioc(RW, 0x48, std::mem::size_of::<GpuObjImport>());
    pub const GPU_COMMAND: u64 = ioc(RW, 0x4a, std::mem::size_of::<GpuCommand>());
    pub const GPUMEM_BIND_RANGES: u64 = ioc(RW, 0x56, std::mem::size_of::<BindRanges>());
}

/// Sizes tried for the VBO that holds every guest GPU address, largest first.
const VBO_SIZES: &[u64] = &[1 << 40, 1 << 38, 1 << 37, 1 << 36, 1 << 34, 1 << 32];

/// One guest client's KGSL context and its submissions not yet retired, oldest first.
#[derive(Default)]
struct Client {
    context: u32,
    pending: VecDeque<u32>,
    /// KGSL invalidated the context, or it caused a GPU reset: the client's work is refused until CLIENT.
    lost: bool,
}

struct Kgsl {
    file: File,
    /// The flags every client's context is created with (the first set KGSL accepted at activation).
    context_flags: u32,
    clients: BTreeMap<u16, Client>,
    /// The latest timestamp the guest submitted: the timeline once nothing is pending.
    last_submitted: u32,
    vbo: u32,
    info: Info,
    /// Live imports and their sizes.
    objects: BTreeMap<u32, u64>,
    /// Their total, and its cap.
    imported: u64,
    import_limit: u64,
    /// The guest-physical spans IMPORT may take (start, end): the memory host and guest share.
    shared: Vec<(u64, u64)>,
    udmabuf: UdmabufDriver,
    submits: u64,
    refusals: u64,
}

impl Kgsl {
    fn ioctl<T>(&self, request: u64, arg: &mut T) -> io::Result<()> {
        let mut interrupted = 0;
        loop {
            // SAFETY: `arg` is a live, correctly laid out argument of `request` (the kgsl uapi layouts above).
            let ret = unsafe {
                libc::ioctl(self.file.as_raw_descriptor(), request as _, arg as *mut T as *mut libc::c_void)
            };
            if ret == 0 {
                return Ok(());
            }
            let error = io::Error::last_os_error();
            interrupted += 1;
            if error.raw_os_error() != Some(libc::EINTR) || interrupted > EINTR_RETRIES {
                return Err(error);
            }
        }
    }

    /// Logs a refused request, the first few times only.
    fn refuse(&mut self, status: i32, what: std::fmt::Arguments) -> i32 {
        self.refusals += 1;
        if self.refusals <= REFUSALS_LOGGED {
            warn!("droidvm-gpu: refused {} ({})", what, -status);
        }
        status
    }

    fn property<T>(&self, type_: u32, value: &mut T) -> io::Result<()> {
        let mut request = kgsl::GetProperty {
            type_,
            value: value as *mut T as u64,
            sizebytes: std::mem::size_of::<T>() as u64,
        };
        self.ioctl(kgsl::GETPROPERTY, &mut request)
    }

    fn open(mem: &GuestMemory) -> anyhow::Result<Kgsl> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open("/dev/kgsl-3d0")
            .context("open /dev/kgsl-3d0")?;
        let udmabuf = UdmabufDriver::new().map_err(|e| anyhow!("open /dev/udmabuf: {}", e))?;
        let shared: Vec<(u64, u64)> = mem
            .regions()
            .filter(|region| {
                matches!(
                    region.options.purpose,
                    MemoryRegionPurpose::SharedGuestRam | MemoryRegionPurpose::SharedFramebuffer
                )
            })
            .map(|region| (region.guest_addr.offset(), region.guest_addr.offset() + region.size as u64))
            .collect();
        if shared.is_empty() {
            // Without a shared region every guest page is lent to the VM: nothing may go to the host's GPU.
            return Err(anyhow!("no guest memory is shared with the host (not a pseudo-unprotected VM?)"));
        }
        let shared_bytes: u64 = shared.iter().map(|(start, end)| end - start).sum();
        let mut kgsl = Kgsl {
            file,
            context_flags: 0,
            clients: BTreeMap::new(),
            last_submitted: 0,
            vbo: 0,
            info: Info::default(),
            objects: BTreeMap::new(),
            imported: 0,
            import_limit: shared_bytes.saturating_mul(MAX_IMPORTED_SHARES),
            shared,
            udmabuf,
            submits: 0,
            refusals: 0,
        };
        let mut device = kgsl::DevInfo::default();
        kgsl.property(kgsl::PROP_DEVICE_INFO, &mut device)
            .context("KGSL_PROP_DEVICE_INFO")?;
        let mut gmem_base = 0u64;
        let _ = kgsl.property(kgsl::PROP_UCHE_GMEM_VADDR, &mut gmem_base);
        let mut highest_bank_bit = 0u32;
        let _ = kgsl.property(kgsl::PROP_HIGHEST_BANK_BIT, &mut highest_bank_bit);
        let mut ubwc_mode = 0u32;
        let _ = kgsl.property(kgsl::PROP_UBWC_MODE, &mut ubwc_mode);
        let mut uche_trap_base = 0u64;
        if kgsl.property(kgsl::PROP_UCHE_TRAP_BASE, &mut uche_trap_base).is_err() {
            uche_trap_base = 0x1_ffff_ffff_f000;
        }
        let (vbo, va_start, va_size) = kgsl.reserve_vbo()?;
        kgsl.vbo = vbo;
        // drm2kgsl's flags (Turnip's own, proven on this device), with the containment added in front. A KGSL may refuse a
        // priority band with EINVAL (drm2kgsl saw that), so each protection is dropped only when the one before is refused.
        let base = kgsl::CONTEXT_SAVE_GMEM
            | kgsl::CONTEXT_NO_GMEM_ALLOC
            | kgsl::CONTEXT_PREAMBLE
            | kgsl::CONTEXT_USER_GENERATED_TS;
        let attempts = [
            (
                base | kgsl::CONTEXT_NO_FAULT_TOLERANCE | kgsl::CONTEXT_PRIORITY_LOWEST,
                "no fault tolerance, lowest priority",
            ),
            (base | kgsl::CONTEXT_NO_FAULT_TOLERANCE, "no fault tolerance"),
            (base, "KGSL's defaults only"),
        ];
        let mut protection = None;
        for (flags, what) in attempts {
            let mut create = kgsl::DrawCtxtCreate {
                flags,
                drawctxt_id: 0,
            };
            match kgsl.ioctl(kgsl::DRAWCTXT_CREATE, &mut create) {
                Ok(()) => {
                    // The render server's (client 0); the others are created with the same flags when they first submit.
                    kgsl.context_flags = flags;
                    kgsl.clients.insert(
                        0,
                        Client {
                            context: create.drawctxt_id,
                            ..Default::default()
                        },
                    );
                    protection = Some(what);
                    break;
                }
                Err(e) if e.raw_os_error() == Some(libc::EINVAL) => continue,
                Err(e) => return Err(e).context("DRAWCTXT_CREATE"),
            }
        }
        let protection = protection.ok_or_else(|| anyhow!("DRAWCTXT_CREATE refused every flag set"))?;
        kgsl.info = Info {
            version: WIRE_VERSION,
            // freedreno's table keys most chips with the fuse byte cleared; the guest's Turnip
            // matches it exactly or through its 0xff wildcard, so the raw value goes along too.
            chip_id: device.chip_id,
            kgsl_chip_id: device.chip_id,
            gmem_bytes: device.gmem_sizebytes as u32,
            gmem_base,
            uche_trap_base,
            va_start,
            va_size,
            highest_bank_bit,
            ubwc_mode,
            faults: 0,
        };
        info!(
            "droidvm-gpu: KGSL chip {:#x} gmem {:#x}@{:#x} hbb {} ubwc {} VBO {} {:#x}+{:#x} client contexts ({}); \
             imports from {} shared span(s), at most {} objects and {:#x} bytes",
            device.chip_id,
            device.gmem_sizebytes,
            gmem_base,
            highest_bank_bit,
            ubwc_mode,
            vbo,
            va_start,
            va_size,
            protection,
            kgsl.shared.len(),
            MAX_OBJECTS,
            kgsl.import_limit
        );
        Ok(kgsl)
    }

    /// Whether KGSL reset the GPU because of `context` since the last query (KGSL_PROP_GPU_RESET_STAT, Turnip's lost-device
    /// check). A reset another context caused is no reason to stop; a status it cannot read is.
    fn guilty(&self, context: u32) -> bool {
        let mut status = context;
        match self.property(kgsl::PROP_GPU_RESET_STAT, &mut status) {
            Ok(()) => status != kgsl::CTX_STAT_NO_ERROR && status != kgsl::CTX_STAT_INNOCENT_CONTEXT_RESET_EXT,
            Err(_) => true,
        }
    }

    /// The client's context, created on its first submission; EDEADLK for a client that lost its GPU.
    fn context_for(&mut self, client: u16) -> Result<u32, i32> {
        if let Some(entry) = self.clients.get(&client) {
            return if entry.lost { Err(-libc::EDEADLK) } else { Ok(entry.context) };
        }
        let mut create = kgsl::DrawCtxtCreate {
            flags: self.context_flags,
            drawctxt_id: 0,
        };
        if let Err(e) = self.ioctl(kgsl::DRAWCTXT_CREATE, &mut create) {
            return Err(self.refuse(
                -e.raw_os_error().unwrap_or(libc::EIO),
                format_args!("a context for client {}: {}", client, e),
            ));
        }
        info!("droidvm-gpu: client {} has KGSL context {}", client, create.drawctxt_id);
        self.clients.insert(
            client,
            Client {
                context: create.drawctxt_id,
                ..Default::default()
            },
        );
        Ok(create.drawctxt_id)
    }

    fn destroy_context(&self, context: u32) {
        if context != 0 {
            let mut id = context;
            let _ = self.ioctl(kgsl::DRAWCTXT_DESTROY, &mut id);
        }
    }

    /// The client lost its GPU: its context is destroyed and its submissions count as done, so the guest's timeline moves
    /// on for the others; everything it sends is refused until CLIENT gives its slot to the next user.
    fn lose(&mut self, client: u16, why: &str) {
        let context = match self.clients.get_mut(&client) {
            Some(entry) if !entry.lost => {
                entry.lost = true;
                entry.pending.clear();
                std::mem::take(&mut entry.context)
            }
            _ => return,
        };
        self.info.faults += 1;
        error!("droidvm-gpu: client {} lost its GPU context {} ({}); the other clients keep theirs", client, context, why);
        self.destroy_context(context);
    }

    /// After a failed submission or wait, and now and then: a context that caused a GPU reset is lost.
    fn check_resets(&mut self, after: &str) {
        let guilty: Vec<u16> = self
            .clients
            .iter()
            .filter(|(_, entry)| !entry.lost && self.guilty(entry.context))
            .map(|(&client, _)| client)
            .collect();
        for client in guilty {
            self.lose(client, &format!("it caused a GPU reset, seen after {}", after));
        }
    }

    /// CLIENT: the slot's previous user is gone (it waited for its work before closing); the next one starts afresh.
    fn reset_client(&mut self, client: u16) {
        if let Some(entry) = self.clients.remove(&client) {
            self.destroy_context(entry.context);
        }
    }

    /// Frees every import, the VBO and the contexts: nothing of the guest stays mapped in the host's GPU.
    fn release_all(&mut self) {
        for (id, _) in std::mem::take(&mut self.objects) {
            self.free_object(id);
        }
        self.imported = 0;
        if self.vbo != 0 {
            self.free_object(self.vbo);
            self.vbo = 0;
        }
        for (_, entry) in std::mem::take(&mut self.clients) {
            self.destroy_context(entry.context);
        }
    }

    fn free(&mut self, id: u32) -> bool {
        match self.objects.remove(&id) {
            Some(bytes) => {
                self.imported -= bytes;
                self.free_object(id);
                true
            }
            None => false,
        }
    }

    fn reserve_vbo(&self) -> anyhow::Result<(u32, u64, u64)> {
        for &size in VBO_SIZES {
            let mut alloc = kgsl::GpuObjAlloc {
                size,
                flags: kgsl::MEMFLAGS_VBO,
                ..Default::default()
            };
            if self.ioctl(kgsl::GPUOBJ_ALLOC, &mut alloc).is_err() {
                continue;
            }
            let mut info = kgsl::GpuObjInfo {
                id: alloc.id,
                ..Default::default()
            };
            if self.ioctl(kgsl::GPUOBJ_INFO, &mut info).is_err() {
                self.free_object(alloc.id);
                continue;
            }
            return Ok((alloc.id, info.gpuaddr, size));
        }
        Err(anyhow!("no VBO size was accepted"))
    }

    fn free_object(&self, id: u32) {
        let mut free = kgsl::GpuObjFree {
            id,
            ..Default::default()
        };
        if let Err(e) = self.ioctl(kgsl::GPUOBJ_FREE, &mut free) {
            warn!("droidvm-gpu: GPUOBJ_FREE {}: {}", id, e);
        }
    }

    fn import(&mut self, mem: &GuestMemory, ranges: &[Range], flags: u32) -> Result<ImportReply, i32> {
        let mut total = 0u64;
        let mut iovecs = Vec::with_capacity(ranges.len());
        for range in ranges {
            if range.address % PAGE != 0 || range.bytes == 0 || range.bytes % PAGE != 0 {
                return Err(-libc::EINVAL);
            }
            let end = range.address.checked_add(range.bytes).ok_or(-libc::EINVAL)?;
            if !self.shared.iter().any(|&(start, stop)| start <= range.address && end <= stop) {
                return Err(self.refuse(
                    -libc::EPERM,
                    format_args!("IMPORT of {:#x}+{:#x}: not memory the host shares", range.address, range.bytes),
                ));
            }
            total = total.checked_add(range.bytes).ok_or(-libc::EINVAL)?;
            iovecs.push((GuestAddress(range.address), range.bytes as usize));
        }
        if self.objects.len() >= MAX_OBJECTS || self.imported.saturating_add(total) > self.import_limit {
            let (objects, imported) = (self.objects.len(), self.imported);
            return Err(self.refuse(
                -libc::ENOSPC,
                format_args!("IMPORT of {:#x} bytes: {} objects, {:#x} bytes imported already", total, objects, imported),
            ));
        }
        let dmabuf = self.udmabuf.create_udmabuf(mem, &iovecs).map_err(|e| {
            warn!("droidvm-gpu: udmabuf over {} ranges: {}", ranges.len(), e);
            -libc::EFAULT
        })?;
        let mut private = kgsl::GpuObjImportDmaBuf {
            fd: dmabuf.as_raw_descriptor(),
        };
        let mut import = kgsl::GpuObjImport {
            priv_: &mut private as *mut _ as u64,
            priv_len: std::mem::size_of::<kgsl::GpuObjImportDmaBuf>() as u64,
            flags: import_flags(flags),
            type_: kgsl::USER_MEM_TYPE_DMABUF,
            id: 0,
        };
        // KGSL's attachment keeps the pages; the udmabuf descriptor closes with `dmabuf`.
        self.ioctl(kgsl::GPUOBJ_IMPORT, &mut import).map_err(|e| {
            warn!("droidvm-gpu: GPUOBJ_IMPORT of {:#x} bytes: {}", total, e);
            -e.raw_os_error().unwrap_or(libc::EIO)
        })?;
        self.objects.insert(import.id, total);
        self.imported += total;
        let mut info = kgsl::GpuObjInfo {
            id: import.id,
            ..Default::default()
        };
        let coherent = self.ioctl(kgsl::GPUOBJ_INFO, &mut info).is_ok()
            && info.flags & kgsl::MEMFLAGS_IOCOHERENT != 0;
        Ok(ImportReply {
            object: import.id,
            coherent: coherent as u32,
            bytes: total,
        })
    }

    fn bind(&mut self, bind: &Bind) -> i32 {
        let va_end = self.info.va_start + self.info.va_size;
        let object_bytes = match self.objects.get(&bind.object) {
            Some(&bytes) => bytes,
            None => return -libc::EINVAL,
        };
        // A bind (op 1) maps [offset, offset + bytes) of the object, which must lie inside it; KGSL works in pages.
        let inside_object = bind.op == 2 || bind.offset.checked_add(bind.bytes).map_or(false, |end| end <= object_bytes);
        if (bind.op != 1 && bind.op != 2)
            || bind.bytes == 0
            || bind.iova < self.info.va_start
            || bind.iova.checked_add(bind.bytes).map_or(true, |end| end > va_end)
        {
            return -libc::EINVAL;
        }
        if !inside_object || bind.offset % PAGE != 0 || bind.iova % PAGE != 0 || bind.bytes % PAGE != 0 {
            return self.refuse(
                -libc::EINVAL,
                format_args!(
                    "BIND of object {} ({:#x} bytes) +{:#x} at {:#x} ({:#x})",
                    bind.object, object_bytes, bind.offset, bind.iova, bind.bytes
                ),
            );
        }
        // KGSL refuses an unbind with a child offset: the child's id and the VBO range name what goes.
        let mut range = kgsl::BindRange {
            child_offset: if bind.op == 2 { 0 } else { bind.offset },
            target_offset: bind.iova - self.info.va_start,
            length: bind.bytes,
            child_id: bind.object,
            op: bind.op,
        };
        let mut ranges = kgsl::BindRanges {
            ranges: &mut range as *mut _ as u64,
            ranges_nents: 1,
            ranges_size: std::mem::size_of::<kgsl::BindRange>() as u32,
            id: self.vbo,
            ..Default::default()
        };
        match self.ioctl(kgsl::GPUMEM_BIND_RANGES, &mut ranges) {
            Ok(()) => 0,
            Err(e) => {
                warn!(
                    "droidvm-gpu: BIND_RANGES op {} object {} +{:#x} at {:#x} ({:#x}): {}",
                    bind.op, bind.object, bind.offset, bind.iova, bind.bytes, e
                );
                -e.raw_os_error().unwrap_or(libc::EIO)
            }
        }
    }

    fn submit(&mut self, client: u16, timestamp: u32, ibs: &[Ib]) -> i32 {
        let context = match self.context_for(client) {
            Ok(context) => context,
            Err(status) => return status,
        };
        // Every earlier submission, whichever client's context ran it, comes first. With one context the GPU ran the guest's
        // commands in submission order, and its processes rely on that between them: an app's drawing into a surface before the
        // render server samples it, the render server's reads before the app draws the surface's next contents. Apart, contexts
        // run in any order (the render server showed one layer's contents in another's place), so each command waits for the other
        // clients' latest one on the GPU. A lost context's events are cancelled, which ends the wait.
        let waits: Vec<kgsl::SyncpointTimestamp> = self
            .clients
            .iter()
            .filter(|(other, entry)| **other != client && !entry.lost)
            .filter_map(|(_, entry)| {
                entry.pending.back().map(|&timestamp| kgsl::SyncpointTimestamp {
                    context_id: entry.context,
                    timestamp,
                })
            })
            .collect();
        let syncs: Vec<kgsl::CommandSyncpoint> = waits
            .iter()
            .map(|wait| kgsl::CommandSyncpoint {
                priv_: wait as *const kgsl::SyncpointTimestamp as u64,
                size: std::mem::size_of::<kgsl::SyncpointTimestamp>() as u64,
                type_: kgsl::SYNCPOINT_TYPE_TIMESTAMP,
                pad: 0,
            })
            .collect();
        let commands: Vec<kgsl::CommandObject> = ibs
            .iter()
            .map(|ib| kgsl::CommandObject {
                offset: 0,
                gpuaddr: ib.iova,
                size: ib.bytes,
                flags: kgsl::CMDLIST_IB,
                id: self.vbo, // the IBs live in the VBO's mapping
            })
            .collect();
        let mut command = kgsl::GpuCommand {
            flags: kgsl::CONTEXT_SUBMIT_IB_LIST,
            cmdlist: commands.as_ptr() as u64,
            cmdsize: std::mem::size_of::<kgsl::CommandObject>() as u32,
            numcmds: commands.len() as u32,
            synclist: if syncs.is_empty() { 0 } else { syncs.as_ptr() as u64 },
            syncsize: std::mem::size_of::<kgsl::CommandSyncpoint>() as u32,
            numsyncs: syncs.len() as u32,
            context_id: context,
            timestamp,
            ..Default::default()
        };
        let result = self.ioctl(kgsl::GPU_COMMAND, &mut command);
        self.submits += 1;
        match result {
            Ok(()) => {
                if let Some(entry) = self.clients.get_mut(&client) {
                    entry.pending.push_back(timestamp);
                }
                self.last_submitted = timestamp;
                if self.submits % RESET_CHECK_INTERVAL == 0 {
                    self.check_resets("a submission");
                }
                0
            }
            Err(e) => {
                warn!("droidvm-gpu: GPU_COMMAND client {} ts {}: {}", client, timestamp, e);
                // EDEADLK and EPROTO: KGSL invalidated the context (a fault in this client's commands).
                if matches!(e.raw_os_error(), Some(libc::EDEADLK) | Some(libc::EPROTO)) || self.guilty(context) {
                    self.lose(client, &format!("GPU_COMMAND: {}", e));
                    return -libc::EDEADLK;
                }
                self.check_resets("a failed submission");
                -e.raw_os_error().unwrap_or(libc::EIO)
            }
        }
    }

    /// The guest's timeline: everything below the oldest submission still running in any context has retired.
    fn retired(&mut self) -> Result<u32, i32> {
        let mut failed = Vec::new();
        for (&client, entry) in self.clients.iter_mut() {
            if entry.lost || entry.pending.is_empty() {
                continue;
            }
            let mut read = kgsl::ReadTimestamp {
                context_id: entry.context,
                type_: kgsl::TIMESTAMP_RETIRED,
                timestamp: 0,
            };
            // SAFETY: as in ioctl(); `read` is the READTIMESTAMP_CTXTID argument.
            let ret = unsafe {
                libc::ioctl(self.file.as_raw_descriptor(), kgsl::READTIMESTAMP_CTXTID as _, &mut read as *mut _ as *mut libc::c_void)
            };
            if ret != 0 {
                failed.push(client);
                continue;
            }
            while entry.pending.front().is_some_and(|&ts| ts <= read.timestamp) {
                entry.pending.pop_front();
            }
        }
        for client in failed {
            self.lose(client, "its retired timestamp could not be read");
        }
        Ok(self
            .clients
            .values()
            .filter_map(|entry| entry.pending.front().copied())
            .min()
            .map_or(self.last_submitted, |oldest| oldest - 1))
    }

    /// Waits (bounded) for the submission the guest's timeline waits on: the oldest still running, if it is at or below
    /// `timestamp`.
    fn wait(&mut self, timestamp: u32, timeout_ms: u32) -> i32 {
        match self.retired() {
            Ok(retired) if retired >= timestamp => return 0,
            Ok(_) => {}
            Err(status) => return status,
        }
        let Some((client, context, target)) = self
            .clients
            .iter()
            .filter_map(|(&client, entry)| entry.pending.front().map(|&oldest| (oldest, client, entry)))
            .min_by_key(|(oldest, _, _)| *oldest)
            .map(|(_, client, entry)| {
                let target = entry.pending.iter().copied().take_while(|&ts| ts <= timestamp).last();
                (client, entry.context, target.unwrap_or(*entry.pending.front().unwrap()))
            })
        else {
            return 0;
        };
        // At least 1 ms: a zero timeout may mean "forever" to KGSL, and the worker has to get back to its kill event.
        let mut wait = kgsl::WaitTimestamp {
            context_id: context,
            timestamp: target,
            timeout: timeout_ms.clamp(1, WAIT_MAX_MS),
        };
        match self.ioctl(kgsl::WAITTIMESTAMP_CTXTID, &mut wait) {
            Ok(()) => 0,
            Err(e) => {
                if e.raw_os_error() == Some(libc::ETIMEDOUT) {
                    return -libc::ETIMEDOUT;
                }
                if matches!(e.raw_os_error(), Some(libc::EDEADLK) | Some(libc::EPROTO)) || self.guilty(context) {
                    // Its work counts as done now: the timeline moves on, and the caller reads it again.
                    self.lose(client, &format!("WAITTIMESTAMP: {}", e));
                    return 0;
                }
                -e.raw_os_error().unwrap_or(libc::EIO)
            }
        }
    }
}

impl Drop for Kgsl {
    fn drop(&mut self) {
        self.release_all();
    }
}

/// KGSL flags of an import: drm2kgsl's choice for guest memory (IO-coherent, with the cache mode
/// of the guest's own mapping), and read-only when the guest says the GPU only reads it.
fn import_flags(flags: u32) -> u64 {
    let mut kgsl_flags = if flags & IMPORT_CACHED != 0 {
        kgsl::CACHEMODE_WRITEBACK << kgsl::CACHEMODE_SHIFT
    } else {
        kgsl::CACHEMODE_WRITECOMBINE << kgsl::CACHEMODE_SHIFT
    };
    if flags & IMPORT_COHERENT != 0 {
        kgsl_flags |= kgsl::MEMFLAGS_IOCOHERENT;
    }
    if flags & IMPORT_READ_ONLY != 0 {
        kgsl_flags |= kgsl::MEMFLAGS_GPUREADONLY;
    }
    kgsl_flags
}

fn host_ticks() -> u64 {
    let value: u64;
    // SAFETY: reads the EL0-accessible virtual counter.
    unsafe { std::arch::asm!("mrs {}, cntvct_el0", out(reg) value) };
    value
}

struct Worker {
    queue: Queue,
    mem: GuestMemory,
    kgsl: Kgsl,
}

impl Worker {
    /// Runs one request; returns the response header and payload.
    fn handle(&mut self, chain: &mut DescriptorChain) -> (Response, Vec<u8>) {
        let reader = &mut chain.reader;
        let request: Request = match reader.read_obj() {
            Ok(request) => request,
            Err(_) => return (Response { status: -libc::EINVAL, bytes: 0 }, Vec::new()),
        };
        let fail = |status: i32| (Response { status, bytes: 0 }, Vec::new());
        let ok = |payload: Vec<u8>| {
            (
                Response {
                    status: 0,
                    bytes: payload.len() as u32,
                },
                payload,
            )
        };
        match request.op {
            OP_INFO => ok(self.kgsl.info.as_bytes().to_vec()),
            OP_IMPORT => {
                let header: Import = match reader.read_obj() {
                    Ok(header) => header,
                    Err(_) => return fail(-libc::EINVAL),
                };
                if header.count == 0 || header.count > MAX_RANGES {
                    return fail(-libc::EINVAL);
                }
                let mut ranges = Vec::with_capacity(header.count as usize);
                for _ in 0..header.count {
                    match reader.read_obj::<Range>() {
                        Ok(range) => ranges.push(range),
                        Err(_) => return fail(-libc::EINVAL),
                    }
                }
                match self.kgsl.import(&self.mem, &ranges, header.flags) {
                    Ok(reply) => ok(reply.as_bytes().to_vec()),
                    Err(status) => fail(status),
                }
            }
            OP_FREE => match reader.read_obj::<Free>() {
                Ok(free) if self.kgsl.free(free.object) => ok(Vec::new()),
                _ => fail(-libc::EINVAL),
            },
            OP_BIND => match reader.read_obj::<Bind>() {
                Ok(bind) => match self.kgsl.bind(&bind) {
                    0 => ok(Vec::new()),
                    status => fail(status),
                },
                Err(_) => fail(-libc::EINVAL),
            },
            OP_SUBMIT => {
                let header: Submit = match reader.read_obj() {
                    Ok(header) => header,
                    Err(_) => return fail(-libc::EINVAL),
                };
                if header.timestamp == 0 || header.count == 0 || u32::from(header.count) > MAX_IBS || header.client >= MAX_CLIENTS {
                    return fail(-libc::EINVAL);
                }
                let mut ibs = Vec::with_capacity(header.count as usize);
                for _ in 0..header.count {
                    match reader.read_obj::<Ib>() {
                        Ok(ib) if ib.bytes != 0 && ib.bytes % 4 == 0 => ibs.push(ib),
                        _ => return fail(-libc::EINVAL),
                    }
                }
                match self.kgsl.submit(header.client, header.timestamp, &ibs) {
                    0 => ok(Vec::new()),
                    status => fail(status),
                }
            }
            OP_RETIRED => match self.kgsl.retired() {
                Ok(timestamp) => ok(Timestamp {
                    timestamp,
                    timeout_ms: 0,
                }
                .as_bytes()
                .to_vec()),
                Err(status) => fail(status),
            },
            OP_WAIT => match reader.read_obj::<Timestamp>() {
                Ok(wait) => match self.kgsl.wait(wait.timestamp, wait.timeout_ms) {
                    0 => ok(Vec::new()),
                    status => fail(status),
                },
                Err(_) => fail(-libc::EINVAL),
            },
            OP_TICKS => ok(Ticks { ticks: host_ticks() }.as_bytes().to_vec()),
            OP_CLIENT => match reader.read_obj::<ClientReset>() {
                Ok(reset) if reset.client < u32::from(MAX_CLIENTS) => {
                    self.kgsl.reset_client(reset.client as u16);
                    ok(Vec::new())
                }
                _ => fail(-libc::EINVAL),
            },
            _ => fail(-libc::ENOSYS),
        }
    }

    fn process_queue(&mut self) {
        while let Some(mut chain) = self.queue.pop() {
            let (response, payload) = self.handle(&mut chain);
            let writer = &mut chain.writer;
            let written = if writer.write_obj(response).is_ok() && writer.write_all(&payload).is_ok() {
                writer.bytes_written()
            } else {
                warn!("droidvm-gpu: response buffer too small for op reply");
                writer.bytes_written()
            };
            self.queue.add_used(chain, written as u32);
        }
        self.queue.trigger_interrupt();
    }

    fn run(&mut self, kill_evt: Event) -> anyhow::Result<()> {
        #[derive(EventToken)]
        enum Token {
            Queue,
            Kill,
        }
        let wait_ctx = WaitContext::build_with(&[
            (self.queue.event(), Token::Queue),
            (&kill_evt, Token::Kill),
        ])
        .context("failed creating WaitContext")?;
        loop {
            let events = wait_ctx.wait().context("failed polling for events")?;
            for event in events.iter().filter(|e| e.is_readable) {
                match event.token {
                    Token::Queue => {
                        self.queue.event().wait().context("failed reading queue Event")?;
                        self.process_queue();
                    }
                    Token::Kill => return Ok(()),
                }
            }
        }
    }
}

/// The droidvm-gpu device. KGSL is opened when the guest activates it.
pub struct DroidVmGpu {
    worker_thread: Option<WorkerThread<Worker>>,
    virtio_features: u64,
}

impl DroidVmGpu {
    pub fn new(virtio_features: u64) -> DroidVmGpu {
        DroidVmGpu {
            worker_thread: None,
            virtio_features,
        }
    }
}

impl VirtioDevice for DroidVmGpu {
    fn keep_rds(&self) -> Vec<RawDescriptor> {
        Vec::new()
    }

    fn device_type(&self) -> DeviceType {
        DeviceType::DroidVmGpu
    }

    fn queue_max_sizes(&self) -> &[u16] {
        QUEUE_SIZES
    }

    fn features(&self) -> u64 {
        self.virtio_features
    }

    fn activate(
        &mut self,
        mem: GuestMemory,
        _interrupt: Interrupt,
        mut queues: BTreeMap<usize, Queue>,
    ) -> anyhow::Result<()> {
        if queues.len() != 1 {
            return Err(anyhow!("expected 1 queue, got {}", queues.len()));
        }
        let queue = queues.remove(&0).unwrap();
        let kgsl = Kgsl::open(&mem)?;
        self.worker_thread = Some(WorkerThread::start("v_droidvm_gpu", move |kill_evt| {
            let mut worker = Worker { queue, mem, kgsl };
            if let Err(e) = worker.run(kill_evt) {
                error!("droidvm-gpu worker failed: {:#}", e);
            }
            worker
        }));
        Ok(())
    }

    fn reset(&mut self) -> anyhow::Result<()> {
        // Dropping the worker drops its KGSL session: every object, the VBO and the context go.
        if let Some(worker_thread) = self.worker_thread.take() {
            let _worker = worker_thread.stop();
        }
        Ok(())
    }
}
