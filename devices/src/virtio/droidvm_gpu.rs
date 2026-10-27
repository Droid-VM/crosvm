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
//! - SUBMIT queues IBs on the device's single KGSL context with the guest's timestamp (user-generated
//!   timestamps), so the guest's in-order fence timeline is KGSL's; RETIRED and WAIT read it.
//!
//! One request at a time, handled synchronously by the worker; the guest polls the used ring.

use std::collections::BTreeMap;
use std::collections::BTreeSet;
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

const WIRE_VERSION: u32 = 1;
const MAX_RANGES: u32 = 1024;
// IBs per OP_SUBMIT: a guest built with 16 (the first macOS kext) still fits; macOS sends up to 64 (build-droidvm-gpu.py).
const MAX_IBS: u32 = 64;
const WAIT_MAX_MS: u32 = 4;

const OP_INFO: u32 = 1;
const OP_IMPORT: u32 = 2;
const OP_FREE: u32 = 3;
const OP_BIND: u32 = 4;
const OP_SUBMIT: u32 = 5;
const OP_RETIRED: u32 = 6;
const OP_WAIT: u32 = 7;
const OP_TICKS: u32 = 8;

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
    count: u32,
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
    pub const PROP_UCHE_GMEM_VADDR: u32 = 0x13;
    pub const PROP_HIGHEST_BANK_BIT: u32 = 0x17;
    pub const PROP_UBWC_MODE: u32 = 0x1b;
    pub const PROP_UCHE_TRAP_BASE: u32 = 0x2f;

    pub const CONTEXT_SAVE_GMEM: u32 = 0x1;
    pub const CONTEXT_NO_GMEM_ALLOC: u32 = 0x2;
    pub const CONTEXT_SUBMIT_IB_LIST: u64 = 0x4;
    pub const CONTEXT_PREAMBLE: u32 = 0x10;
    pub const CONTEXT_USER_GENERATED_TS: u32 = 0x80;
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

struct Kgsl {
    file: File,
    context: u32,
    vbo: u32,
    info: Info,
    objects: BTreeSet<u32>,
    udmabuf: UdmabufDriver,
}

impl Kgsl {
    fn ioctl<T>(&self, request: u64, arg: &mut T) -> io::Result<()> {
        loop {
            // SAFETY: `arg` is a live, correctly laid out argument of `request` (the kgsl uapi layouts above).
            let ret = unsafe {
                libc::ioctl(self.file.as_raw_descriptor(), request as _, arg as *mut T as *mut libc::c_void)
            };
            if ret == 0 {
                return Ok(());
            }
            let error = io::Error::last_os_error();
            if error.raw_os_error() != Some(libc::EINTR) {
                return Err(error);
            }
        }
    }

    fn property<T>(&self, type_: u32, value: &mut T) -> io::Result<()> {
        let mut request = kgsl::GetProperty {
            type_,
            value: value as *mut T as u64,
            sizebytes: std::mem::size_of::<T>() as u64,
        };
        self.ioctl(kgsl::GETPROPERTY, &mut request)
    }

    fn open() -> anyhow::Result<Kgsl> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .open("/dev/kgsl-3d0")
            .context("open /dev/kgsl-3d0")?;
        let udmabuf = UdmabufDriver::new().map_err(|e| anyhow!("open /dev/udmabuf: {}", e))?;
        let mut kgsl = Kgsl {
            file,
            context: 0,
            vbo: 0,
            info: Info::default(),
            objects: BTreeSet::new(),
            udmabuf,
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
        let mut create = kgsl::DrawCtxtCreate {
            flags: kgsl::CONTEXT_SAVE_GMEM
                | kgsl::CONTEXT_NO_GMEM_ALLOC
                | kgsl::CONTEXT_PREAMBLE
                | kgsl::CONTEXT_USER_GENERATED_TS,
            drawctxt_id: 0,
        };
        kgsl.ioctl(kgsl::DRAWCTXT_CREATE, &mut create)
            .context("DRAWCTXT_CREATE")?;
        kgsl.context = create.drawctxt_id;
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
            "droidvm-gpu: KGSL chip {:#x} gmem {:#x}@{:#x} hbb {} ubwc {} VBO {} {:#x}+{:#x} context {}",
            device.chip_id,
            device.gmem_sizebytes,
            gmem_base,
            highest_bank_bit,
            ubwc_mode,
            vbo,
            va_start,
            va_size,
            kgsl.context
        );
        Ok(kgsl)
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
            if range.address % 4096 != 0 || range.bytes == 0 || range.bytes % 4096 != 0 {
                return Err(-libc::EINVAL);
            }
            total = total.checked_add(range.bytes).ok_or(-libc::EINVAL)?;
            iovecs.push((GuestAddress(range.address), range.bytes as usize));
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
        self.objects.insert(import.id);
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

    fn bind(&self, bind: &Bind) -> i32 {
        let va_end = self.info.va_start + self.info.va_size;
        if !self.objects.contains(&bind.object)
            || (bind.op != 1 && bind.op != 2)
            || bind.bytes == 0
            || bind.iova < self.info.va_start
            || bind.iova.checked_add(bind.bytes).map_or(true, |end| end > va_end)
        {
            return -libc::EINVAL;
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

    fn submit(&mut self, timestamp: u32, ibs: &[Ib]) -> i32 {
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
            context_id: self.context,
            timestamp,
            ..Default::default()
        };
        match self.ioctl(kgsl::GPU_COMMAND, &mut command) {
            Ok(()) => 0,
            Err(e) => {
                self.info.faults += 1;
                warn!("droidvm-gpu: GPU_COMMAND ts {}: {}", timestamp, e);
                -e.raw_os_error().unwrap_or(libc::EIO)
            }
        }
    }

    fn retired(&self) -> Result<u32, i32> {
        let mut read = kgsl::ReadTimestamp {
            context_id: self.context,
            type_: kgsl::TIMESTAMP_RETIRED,
            timestamp: 0,
        };
        self.ioctl(kgsl::READTIMESTAMP_CTXTID, &mut read)
            .map_err(|e| -e.raw_os_error().unwrap_or(libc::EIO))?;
        Ok(read.timestamp)
    }

    fn wait(&self, timestamp: u32, timeout_ms: u32) -> i32 {
        let mut wait = kgsl::WaitTimestamp {
            context_id: self.context,
            timestamp,
            timeout: timeout_ms.min(WAIT_MAX_MS),
        };
        match self.ioctl(kgsl::WAITTIMESTAMP_CTXTID, &mut wait) {
            Ok(()) => 0,
            Err(e) => -e.raw_os_error().unwrap_or(libc::EIO),
        }
    }
}

impl Drop for Kgsl {
    fn drop(&mut self) {
        for &id in &self.objects {
            self.free_object(id);
        }
        if self.vbo != 0 {
            self.free_object(self.vbo);
        }
        if self.context != 0 {
            let mut id = self.context;
            let _ = self.ioctl(kgsl::DRAWCTXT_DESTROY, &mut id);
        }
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
                Ok(free) if self.kgsl.objects.remove(&free.object) => {
                    self.kgsl.free_object(free.object);
                    ok(Vec::new())
                }
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
                if header.timestamp == 0 || header.count == 0 || header.count > MAX_IBS {
                    return fail(-libc::EINVAL);
                }
                let mut ibs = Vec::with_capacity(header.count as usize);
                for _ in 0..header.count {
                    match reader.read_obj::<Ib>() {
                        Ok(ib) if ib.bytes != 0 && ib.bytes % 4 == 0 => ibs.push(ib),
                        _ => return fail(-libc::EINVAL),
                    }
                }
                match self.kgsl.submit(header.timestamp, &ibs) {
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
        let kgsl = Kgsl::open()?;
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
