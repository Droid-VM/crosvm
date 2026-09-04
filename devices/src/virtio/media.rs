// Copyright 2024 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! Support for virtio-media devices in crosvm.
//!
//! This module provides implementation for the virtio-media traits required to make virtio-media
//! devices operate under crosvm. Sub-modules then integrate these devices with crosvm.
//!
//! Host-owned (`MMAP`) buffers come from one of two backings (`VPU_DESIGN.md` §3.2):
//!
//! * `Bar`, the upstream shape: a memfd per buffer, mapped into a 4 GiB PCI shared-memory BAR
//!   the guest reaches through `virtio_get_shm_region()`. What a KVM VM without a pool gets.
//! * `Pool`: buffers are slices of the `media_host` pool (`--pre-alloc media-host-mb=N`), which
//!   the guest already maps as a whole from its `media_host` reserved-memory node; the device
//!   declares no shared-memory region at all, and the offset it answers `MMAP` with is the
//!   buffer's offset inside the pool. This is the only shape that works on Gunyah, whose 64-bit
//!   MMIO window has room for one 4 GiB BAR and the GPU already has it.
//!
//! Guest-owned (`USERPTR`) buffers are resolved in `guest_buf` (`VPU_DESIGN.md` §3.4).

#[cfg(feature = "video-decoder")]
pub mod decoder_adapter;
pub mod guest_buf;

use std::collections::BTreeMap;
use std::fs::File;
use std::os::fd::AsRawFd;
use std::os::fd::BorrowedFd;
use std::os::fd::OwnedFd;
use std::path::Path;
use std::path::PathBuf;
use std::ptr::NonNull;
use std::rc::Rc;
use std::sync::Arc;

use anyhow::Context;
use base::error;
use base::info;
use base::AsRawDescriptor;
use base::Descriptor;
use base::Event;
use base::EventToken;
use base::EventType;
use base::MappedRegion;
use base::MemoryMapping;
use base::MemoryMappingBuilder;
use base::Protection;
use base::SafeDescriptor;
use base::WaitContext;
use base::WorkerThread;
use resources::address_allocator::AddressAllocator;
use resources::AddressRange;
use resources::Alloc;
use sync::Mutex;
use virtio_media::io::WriteToDescriptorChain;
use virtio_media::poll::SessionPoller;
use virtio_media::protocol::SgEntry;
use virtio_media::protocol::V4l2Event;
use virtio_media::protocol::VirtioMediaDeviceConfig;
use virtio_media::GuestMappingError;
use virtio_media::GuestMemoryRange;
use virtio_media::HostBuffer;
use virtio_media::MemFdAllocator;
use virtio_media::VirtioMediaBufferAllocator;
use virtio_media::VirtioMediaDevice;
use virtio_media::VirtioMediaDeviceRunner;
use virtio_media::VirtioMediaEventQueue;
use virtio_media::VirtioMediaGuestMemoryMapper;
use virtio_media::VirtioMediaHostMemoryMapper;
use vm_control::VmMemorySource;
use vm_memory::GuestAddress;
use vm_memory::GuestMemory;
use vm_memory::MediaPoolHandle;

use crate::virtio::copy_config;
use crate::virtio::device_constants::media::QUEUE_SIZES;
#[cfg(feature = "video-decoder")]
use crate::virtio::device_constants::video::VideoBackendType;
use crate::virtio::media::guest_buf::GuestBufferImport;
use crate::virtio::DeviceType;
use crate::virtio::Interrupt;
use crate::virtio::Queue;
use crate::virtio::Reader;
use crate::virtio::SharedMemoryMapper;
use crate::virtio::SharedMemoryRegion;
use crate::virtio::VirtioDevice;
use crate::virtio::Writer;

/// Structure supporting the implementation of `VirtioMediaEventQueue` for sending events to the
/// driver.
struct EventQueue(Queue);

impl VirtioMediaEventQueue for EventQueue {
    /// Wait until an event descriptor becomes available and send `event` to the guest.
    fn send_event(&mut self, event: V4l2Event) {
        let mut desc;

        loop {
            match self.0.pop() {
                Some(d) => {
                    desc = d;
                    break;
                }
                None => {
                    if let Err(e) = self.0.event().wait() {
                        error!("could not obtain a descriptor to send event to: {:#}", e);
                        return;
                    }
                }
            }
        }

        if let Err(e) = match event {
            V4l2Event::Error(event) => WriteToDescriptorChain::write_obj(&mut desc.writer, event),
            V4l2Event::DequeueBuffer(event) => {
                WriteToDescriptorChain::write_obj(&mut desc.writer, event)
            }
            V4l2Event::Event(event) => WriteToDescriptorChain::write_obj(&mut desc.writer, event),
        } {
            error!("failed to write event: {}", e);
        }

        let written = desc.writer.bytes_written() as u32;
        self.0.add_used(desc, written);
        self.0.trigger_interrupt();
    }
}

/// A `SharedMemoryMapper` behind an `Arc`, allowing it to be shared.
///
/// This is required by the fact that devices can be activated several times, but the mapper is
/// only provided once. This might be a defect of the `VirtioDevice` interface.
#[derive(Clone)]
struct ArcedMemoryMapper(Arc<Mutex<Box<dyn SharedMemoryMapper>>>);

impl From<Box<dyn SharedMemoryMapper>> for ArcedMemoryMapper {
    fn from(mapper: Box<dyn SharedMemoryMapper>) -> Self {
        Self(Arc::new(Mutex::new(mapper)))
    }
}

impl SharedMemoryMapper for ArcedMemoryMapper {
    fn add_mapping(
        &mut self,
        source: VmMemorySource,
        offset: u64,
        prot: Protection,
        cache: hypervisor::MemCacheType,
    ) -> anyhow::Result<()> {
        self.0.lock().add_mapping(source, offset, prot, cache)
    }

    fn remove_mapping(&mut self, offset: u64) -> anyhow::Result<()> {
        self.0.lock().remove_mapping(offset)
    }

    fn as_raw_descriptor(&self) -> Option<base::RawDescriptor> {
        self.0.lock().as_raw_descriptor()
    }
}

/// Size of the shared-memory BAR a `Bar` backing asks for.
const HOST_MAPPER_RANGE: u64 = 1 << 32;

/// The VM-wide `media_host` pool: one offset space, shared by every media device.
///
/// It has to be VM-wide. A pool offset is what the device answers `VIDIOC_QUERYBUF` with and the
/// guest adds the `media_host` node's base to it, so two devices with private allocators over the
/// same window both hand out offset 0, 0x1000, ... and the guest maps the same physical bytes for
/// two unrelated buffers -- while one device's `release` frees space the other is still filling
/// (`VPU_DESIGN.md` §2.2 sizes the one 256 MiB pool for every media device in the VM).
///
/// The descriptor is dup'ed and the whole pool mapped exactly once, here; a buffer's own dup is
/// what the crate's `HostBuffer` carries.
pub struct MediaPoolAllocator {
    pool: MediaPoolHandle,
    /// The pool's offset space, `[0, pool.size)`, page aligned.
    allocator: AddressAllocator,
    /// Our own mapping of the pool, so that buffers can be filled from this process.
    host_map: MemoryMapping,
    /// Key for the next allocation; `AddressAllocator` wants every live allocation to have a
    /// distinct one.
    next_id: usize,
    /// Id for the next lease.
    next_owner: u64,
    /// `pool offset -> (owner, size)` for every slice handed out. The owner is what lets a
    /// device that goes away -- a virtio reset drops its worker and every buffer with it -- give
    /// its slices back to a pool the rest of the VM keeps using.
    live: BTreeMap<u64, (u64, u64)>,
    /// Bytes currently handed out, for the exhaustion log line.
    used: u64,
}

impl MediaPoolAllocator {
    fn new(pool: MediaPoolHandle) -> anyhow::Result<Self> {
        let file = File::from(
            pool.fd
                .try_clone()
                .context("cannot dup the media_host pool descriptor")?,
        );
        let host_map = MemoryMappingBuilder::new(pool.size as usize)
            .from_file(&file)
            .offset(pool.fd_offset)
            .build()
            .context("cannot map the media_host pool")?;
        let allocator = AddressAllocator::new(
            AddressRange::from_start_and_end(0, pool.size - 1),
            Some(base::pagesize() as u64),
            None,
        )?;
        Ok(MediaPoolAllocator {
            pool,
            allocator,
            host_map,
            next_id: 0,
            next_owner: 0,
            live: BTreeMap::new(),
            used: 0,
        })
    }

    /// Carve `len` bytes out of the pool for `owner`. `card` only names the device in the
    /// exhaustion log -- with one pool for the whole VM, "the pool is full" is useless without
    /// knowing who asked.
    fn allocate(&mut self, len: u64, owner: u64, card: &str) -> Result<HostBuffer, i32> {
        if len == 0 {
            return Err(libc::EINVAL);
        }
        let page = base::pagesize() as u64;
        let size = len.checked_next_multiple_of(page).ok_or(libc::EINVAL)?;
        let id = self.next_id;
        self.next_id += 1;
        let (used, pool_size) = (self.used, self.pool.size);
        // Every allocation gets its own key; the offsets are what matter, the key is only a name.
        let offset = self
            .allocator
            .allocate(size, Alloc::Anon(id), "media buffer".into())
            .map_err(|e| {
                // The one place exhaustion surfaces: REQBUFS/CREATE_BUFS get ENOMEM, and the log
                // says which device asked and how full the pool was (VPU_DESIGN.md §4.1).
                error!(
                    "virtio-media: media_host pool exhausted for \"{}\": {} bytes requested with \
                     {} of {} in use ({:?})",
                    card, size, used, pool_size, e
                );
                libc::ENOMEM
            })?;

        let fd: OwnedFd = match self.pool.fd.try_clone() {
            Ok(fd) => fd.into(),
            Err(e) => {
                error!("virtio-media: cannot dup the pool descriptor: {}", e);
                let _ = self.allocator.release_containing(offset);
                return Err(libc::EIO);
            }
        };
        self.used += size;
        self.live.insert(offset, (owner, size));
        // SAFETY: `offset + size <= pool.size` (the allocator's range), and `host_map` maps all
        // `pool.size` bytes for as long as this allocator exists, which is longer than any buffer
        // it hands out (buffers come back through `release`, and a lease that dies takes its
        // buffers' space with it).
        let ptr = unsafe { self.host_map.as_ptr().add(offset as usize) };
        let ptr = match NonNull::new(ptr) {
            Some(ptr) => ptr,
            None => {
                self.free(offset);
                return Err(libc::EIO);
            }
        };
        // SAFETY: `ptr` maps the `len` bytes at `pool.fd_offset + offset` of `fd`, and stays
        // valid for the life of the allocator (see above).
        Ok(unsafe {
            HostBuffer::from_raw_parts(
                fd,
                self.pool.fd_offset + offset,
                len,
                ptr,
                Some(offset),
            )
        })
    }

    /// Give one offset back to the offset space.
    fn free(&mut self, offset: u64) {
        match self.live.remove(&offset) {
            Some((_, size)) => {
                self.used = self.used.saturating_sub(size);
                if let Err(e) = self.allocator.release_containing(offset) {
                    error!(
                        "virtio-media: releasing a buffer at pool offset {:#x} that is not \
                         allocated: {}",
                        offset, e
                    );
                }
            }
            None => error!(
                "virtio-media: releasing a buffer at pool offset {:#x} that is not allocated",
                offset
            ),
        }
    }

    fn release(&mut self, buf: HostBuffer) {
        if let Some(offset) = buf.pool_offset {
            self.free(offset);
        }
        // Dropping closes the descriptor dup; the pool mapping is ours, not the buffer's.
        drop(buf);
    }

    /// Take back everything a lease still holds. Its buffers are gone with the device (dropping
    /// a `HostBuffer` frees nothing in the pool by design), so only the offset space is at stake.
    fn release_owner(&mut self, owner: u64) {
        let offsets: Vec<u64> = self
            .live
            .iter()
            .filter(|(_, (o, _))| *o == owner)
            .map(|(offset, _)| *offset)
            .collect();
        if !offsets.is_empty() {
            info!(
                "virtio-media: reclaiming {} media_host buffers from a device that went away",
                offsets.len()
            );
        }
        for offset in offsets {
            self.free(offset);
        }
    }
}

/// Handle on the VM-wide [`MediaPoolAllocator`], created once per VM and cloned into every media
/// device.
#[derive(Clone)]
pub struct MediaPool {
    inner: Arc<Mutex<MediaPoolAllocator>>,
    /// The pool descriptor, for `keep_rds`. The allocator owns the only dup this side keeps.
    fd: base::RawDescriptor,
}

impl MediaPool {
    /// Map the pool and build its allocator. One dup of the descriptor and one host mapping, for
    /// the whole VM.
    pub fn new(pool: MediaPoolHandle) -> anyhow::Result<Self> {
        let fd = pool.fd.as_raw_descriptor();
        let (gpa, size) = (pool.gpa, pool.size);
        let inner = MediaPoolAllocator::new(pool)?;
        info!(
            "virtio-media: serving MMAP buffers from the media_host pool (gpa {:#x}, {} MiB)",
            gpa,
            size >> 20
        );
        Ok(MediaPool {
            inner: Arc::new(Mutex::new(inner)),
            fd,
        })
    }

    pub fn as_raw_descriptor(&self) -> base::RawDescriptor {
        self.fd
    }

    /// One device's lease on the pool. What the lease still holds when it is dropped goes back
    /// to the pool, so a device that is reset does not eat the VM's pool for good.
    fn lease(&self, card: String) -> PoolBufferAllocator {
        let owner = {
            let mut pool = self.inner.lock();
            pool.next_owner += 1;
            pool.next_owner
        };
        PoolBufferAllocator {
            pool: Arc::clone(&self.inner),
            card,
            owner,
        }
    }
}

/// One device's lease on the VM-wide pool.
pub struct PoolBufferAllocator {
    pool: Arc<Mutex<MediaPoolAllocator>>,
    /// The device's V4L2 card name, for the exhaustion log.
    card: String,
    owner: u64,
}

impl VirtioMediaBufferAllocator for PoolBufferAllocator {
    fn allocate(&mut self, len: u64) -> Result<HostBuffer, i32> {
        self.pool.lock().allocate(len, self.owner, &self.card)
    }

    fn release(&mut self, buf: HostBuffer) {
        self.pool.lock().release(buf)
    }
}

impl Drop for PoolBufferAllocator {
    fn drop(&mut self) {
        self.pool.lock().release_owner(self.owner);
    }
}

/// Where a device's host-owned (`MMAP`) buffers come from.
pub enum BufferAllocator {
    /// Upstream: a memfd per buffer, mapped on demand into the PCI shared-memory BAR.
    Memfd(MemFdAllocator),
    /// DroidVM: slices of the VM-wide `media_host` pool the guest maps as a whole.
    Pool(PoolBufferAllocator),
}

impl VirtioMediaBufferAllocator for BufferAllocator {
    fn allocate(&mut self, len: u64) -> Result<HostBuffer, i32> {
        match self {
            BufferAllocator::Memfd(memfd) => memfd.allocate(len),
            BufferAllocator::Pool(pool) => pool.allocate(len),
        }
    }

    fn release(&mut self, buf: HostBuffer) {
        match self {
            BufferAllocator::Memfd(memfd) => memfd.release(buf),
            BufferAllocator::Pool(pool) => pool.release(buf),
        }
    }
}

/// How a device's host-owned buffers are made visible to the guest (`VPU_DESIGN.md` §3.2).
///
/// This is per device -- a BAR belongs to one PCI function, and with a pool there is nothing to
/// map at all -- while the buffers themselves come from [`BufferAllocator`], which with a pool is
/// VM-wide. Never take the pool lock while holding this: the mapper never calls the allocator.
pub enum HostMapper {
    /// Upstream: buffers are mapped on demand into the PCI shared-memory BAR.
    Bar {
        shm_mapper: ArcedMemoryMapper,
        /// The BAR's offset space, `[0, HOST_MAPPER_RANGE)`, page aligned.
        allocator: AddressAllocator,
    },
    /// DroidVM: the guest maps the whole pool, so a buffer's offset inside it is its address and
    /// nothing has to be mapped per buffer.
    Pool,
}

impl HostMapper {
    fn bar(shm_mapper: ArcedMemoryMapper) -> anyhow::Result<Self> {
        Ok(HostMapper::Bar {
            shm_mapper,
            allocator: AddressAllocator::new(
                AddressRange::from_start_and_end(0, HOST_MAPPER_RANGE - 1),
                Some(base::pagesize() as u64),
                None,
            )?,
        })
    }
}

impl VirtioMediaHostMemoryMapper for HostMapper {
    fn add_mapping(&mut self, buffer: &HostBuffer, offset: u64, rw: bool) -> Result<u64, i32> {
        match self {
            HostMapper::Bar {
                shm_mapper,
                allocator,
            } => {
                // TODO: technically `offset` can be used twice if a buffer is deleted and some
                // other takes its place...
                let shm_offset = allocator
                    .allocate(buffer.len, Alloc::FileBacked(offset), "".into())
                    .map_err(|_| libc::ENOMEM)?;

                let descriptor: SafeDescriptor = buffer
                    .fd
                    .try_clone()
                    .map_err(|_| libc::EIO)?
                    .into();
                match shm_mapper.add_mapping(
                    VmMemorySource::Descriptor {
                        descriptor,
                        offset: buffer.fd_offset,
                        size: buffer.len,
                    },
                    shm_offset,
                    if rw {
                        Protection::read_write()
                    } else {
                        Protection::read()
                    },
                    hypervisor::MemCacheType::CacheCoherent,
                ) {
                    Ok(()) => Ok(shm_offset),
                    Err(e) => {
                        let _ = allocator.release_containing(shm_offset);
                        error!("failed to map memory buffer: {:#}", e);
                        Err(libc::EINVAL)
                    }
                }
            }
            // The guest maps the whole pool; a buffer's offset inside it is its address. `rw` is
            // not ours to enforce: the guest kernel decides what its own VMA may do, and the pool
            // is mapped read-write once for the VM.
            HostMapper::Pool => buffer.pool_offset.ok_or_else(|| {
                error!("virtio-media: a buffer that is not in the pool cannot be mapped");
                libc::EINVAL
            }),
        }
    }

    fn remove_mapping(&mut self, offset: u64) -> Result<(), i32> {
        match self {
            HostMapper::Bar {
                shm_mapper,
                allocator,
            } => {
                let _ = allocator.release_containing(offset);
                shm_mapper.remove_mapping(offset).map_err(|_| libc::EINVAL)
            }
            // Nothing was mapped for it.
            HostMapper::Pool => Ok(()),
        }
    }
}

impl GuestMemoryRange for GuestBufferImport {
    fn as_ptr(&self) -> *const u8 {
        GuestBufferImport::as_ptr(self)
    }

    fn as_mut_ptr(&mut self) -> *mut u8 {
        GuestBufferImport::as_mut_ptr(self)
    }
}

/// Newtype to implement `VirtioMediaGuestMemoryMapper` on `GuestMemory`.
///
/// Whether to use a direct mapping or to copy the guest data into a shadow buffer is decided by
/// the size of the guest mapping (see `guest_buf::MAPPING_THRESHOLD`). A device that says which
/// way it will access the buffer (`new_mapping_for`) gets a mapping that can only be used that
/// way: an OUTPUT buffer is the guest's own memory, which on a protected VM it shared with the
/// host and goes on trusting, so the host maps it read-only (`VPU_DESIGN.md` §3.4).
pub struct GuestMemoryMapper(GuestMemory);

impl GuestMemoryMapper {
    pub fn new(mem: GuestMemory) -> Self {
        Self(mem)
    }

    pub fn guest_memory(&self) -> &GuestMemory {
        &self.0
    }
}

impl GuestMemoryMapper {
    fn import(&self, sgs: Vec<SgEntry>, prot: Protection) -> anyhow::Result<GuestBufferImport> {
        let ranges: Vec<(GuestAddress, usize)> = sgs
            .iter()
            .map(|sg| (GuestAddress(sg.start), sg.len as usize))
            .collect();
        GuestBufferImport::new(&self.0, ranges, prot).map_err(|e| {
            // The errno travels as the error's root so the device can hand it to the guest.
            anyhow::Error::from(GuestMappingError(e.errno())).context(e.to_string())
        })
    }
}

impl VirtioMediaGuestMemoryMapper for GuestMemoryMapper {
    type GuestMemoryMapping = GuestBufferImport;

    fn new_mapping(&self, sgs: Vec<SgEntry>) -> anyhow::Result<Self::GuestMemoryMapping> {
        self.import(sgs, Protection::read_write())
    }

    fn new_mapping_for(
        &self,
        sgs: Vec<SgEntry>,
        writable: bool,
    ) -> anyhow::Result<Self::GuestMemoryMapping> {
        self.import(
            sgs,
            if writable {
                Protection::read_write()
            } else {
                Protection::read()
            },
        )
    }
}

#[derive(EventToken, Debug)]
enum Token {
    CommandQueue,
    V4l2Session(u32),
    Kill,
}

/// Newtype to implement `SessionPoller` on `Rc<WaitContext<Token>>`.
#[derive(Clone)]
struct WaitContextPoller(Rc<WaitContext<Token>>);

impl SessionPoller for WaitContextPoller {
    fn add_session(&self, session: BorrowedFd, session_id: u32) -> Result<(), i32> {
        self.0
            .add_for_event(
                &Descriptor(session.as_raw_fd()),
                EventType::Read,
                Token::V4l2Session(session_id),
            )
            .map_err(|e| e.errno())
    }

    fn remove_session(&self, session: BorrowedFd) {
        let _ = self.0.delete(&Descriptor(session.as_raw_fd()));
    }
}

/// Worker to operate a virtio-media device inside a worker thread.
struct Worker<D: VirtioMediaDevice<Reader, Writer>> {
    runner: VirtioMediaDeviceRunner<Reader, Writer, D, WaitContextPoller>,
    cmd_queue: Queue,
    wait_ctx: Rc<WaitContext<Token>>,
}

impl<D> Worker<D>
where
    D: VirtioMediaDevice<Reader, Writer>,
{
    /// Create a new worker instance for `device`.
    fn new(
        device: D,
        cmd_queue: Queue,
        kill_evt: Event,
        wait_ctx: Rc<WaitContext<Token>>,
    ) -> anyhow::Result<Self> {
        wait_ctx
            .add_many(&[
                (cmd_queue.event(), Token::CommandQueue),
                (&kill_evt, Token::Kill),
            ])
            .context("when adding worker events to wait context")?;

        Ok(Self {
            runner: VirtioMediaDeviceRunner::new(device, WaitContextPoller(Rc::clone(&wait_ctx))),
            cmd_queue,
            wait_ctx,
        })
    }

    fn run(&mut self) -> anyhow::Result<()> {
        loop {
            let wait_events = self.wait_ctx.wait().context("Wait error")?;

            for wait_event in wait_events.iter() {
                match wait_event.token {
                    Token::CommandQueue => {
                        let _ = self.cmd_queue.event().wait();
                        while let Some(mut desc) = self.cmd_queue.pop() {
                            self.runner
                                .handle_command(&mut desc.reader, &mut desc.writer);
                            // Return the descriptor to the guest.
                            let written = desc.writer.bytes_written() as u32;
                            self.cmd_queue.add_used(desc, written);
                            self.cmd_queue.trigger_interrupt();
                        }
                    }
                    Token::Kill => {
                        return Ok(());
                    }
                    Token::V4l2Session(session_id) => {
                        let session = match self.runner.sessions.get_mut(&session_id) {
                            Some(session) => session,
                            None => {
                                base::error!(
                                    "received event for non-registered session {}",
                                    session_id
                                );
                                continue;
                            }
                        };

                        if let Err(e) = self.runner.device.process_events(session) {
                            base::error!(
                                "error while processing events for session {}: {:#}",
                                session_id,
                                e
                            );
                            if let Some(session) = self.runner.sessions.remove(&session_id) {
                                self.runner.device.close_session(session);
                            }
                        }
                    }
                }
            }
        }
    }
}

/// Implements the required traits to operate a [`VirtioMediaDevice`] under crosvm.
struct CrosvmVirtioMediaDevice<
    D: VirtioMediaDevice<Reader, Writer>,
    F: Fn(EventQueue, GuestMemoryMapper, HostMapper, BufferAllocator) -> anyhow::Result<D>,
> {
    /// Closure to create the device once all its resources are acquired.
    create_device: F,
    /// Virtio configuration area.
    config: VirtioMediaDeviceConfig,

    /// Virtio device features.
    base_features: u64,
    /// The VM-wide `media_host` pool, when the VM has one: `MMAP` buffers are served out of it
    /// and no shared-memory BAR is declared.
    pool: Option<MediaPool>,
    /// Mapper to make host video buffers visible to the guest, in `Bar` mode.
    ///
    /// We unfortunately need to put it behind a `Arc` because the mapper is only passed once,
    /// whereas the device can be activated several times, so we need to keep a reference to it
    /// even after it is passed to the device.
    shm_mapper: Option<ArcedMemoryMapper>,
    /// Worker thread for the device.
    worker_thread: Option<WorkerThread<()>>,
}

impl<D, F> CrosvmVirtioMediaDevice<D, F>
where
    D: VirtioMediaDevice<Reader, Writer>,
    F: Fn(EventQueue, GuestMemoryMapper, HostMapper, BufferAllocator) -> anyhow::Result<D>,
{
    fn new(
        base_features: u64,
        config: VirtioMediaDeviceConfig,
        pool: Option<MediaPool>,
        create_device: F,
    ) -> Self {
        Self {
            base_features,
            config,
            pool,
            shm_mapper: None,
            create_device,
            worker_thread: None,
        }
    }
}

impl<D, F> VirtioDevice for CrosvmVirtioMediaDevice<D, F>
where
    D: VirtioMediaDevice<Reader, Writer> + Send + 'static,
    F: Fn(EventQueue, GuestMemoryMapper, HostMapper, BufferAllocator) -> anyhow::Result<D> + Send,
{
    fn keep_rds(&self) -> Vec<base::RawDescriptor> {
        let mut keep_rds = Vec::new();

        if let Some(fd) = self.shm_mapper.as_ref().and_then(|m| m.as_raw_descriptor()) {
            keep_rds.push(fd);
        }
        if let Some(pool) = &self.pool {
            keep_rds.push(pool.as_raw_descriptor());
        }

        keep_rds
    }

    fn device_type(&self) -> DeviceType {
        DeviceType::Media
    }

    fn queue_max_sizes(&self) -> &[u16] {
        QUEUE_SIZES
    }

    fn features(&self) -> u64 {
        self.base_features
    }

    fn read_config(&self, offset: u64, data: &mut [u8]) {
        copy_config(data, 0, self.config.as_ref(), offset);
    }

    fn activate(
        &mut self,
        mem: vm_memory::GuestMemory,
        _interrupt: Interrupt,
        mut queues: BTreeMap<usize, Queue>,
    ) -> anyhow::Result<()> {
        if queues.len() != QUEUE_SIZES.len() {
            anyhow::bail!(
                "wrong number of queues are passed: expected {}, actual {}",
                QUEUE_SIZES.len(),
                queues.len()
            );
        }

        let cmd_queue = queues.remove(&0).context("missing queue 0")?;
        let event_queue = EventQueue(queues.remove(&1).context("missing queue 1")?);

        // The offset space is the pool's, and the pool belongs to the VM; the mapper is this
        // device's. A lease that outlives its device gives the device's slices back.
        let (mapper, allocator) = match &self.pool {
            Some(pool) => (
                HostMapper::Pool,
                BufferAllocator::Pool(pool.lease(card_str(&self.config.card))),
            ),
            None => {
                let shm_mapper = self
                    .shm_mapper
                    .clone()
                    .context("shared memory mapper was not specified")?;
                (
                    HostMapper::bar(shm_mapper)?,
                    BufferAllocator::Memfd(MemFdAllocator::new()),
                )
            }
        };

        let wait_ctx = WaitContext::new()?;
        let device = (self.create_device)(
            event_queue,
            GuestMemoryMapper::new(mem),
            mapper,
            allocator,
        )?;

        let worker_thread = WorkerThread::start("v_media_worker", move |e| {
            let wait_ctx = Rc::new(wait_ctx);
            let mut worker = match Worker::new(device, cmd_queue, e, wait_ctx) {
                Ok(worker) => worker,
                Err(e) => {
                    error!("failed to create virtio-media worker: {:#}", e);
                    return;
                }
            };
            if let Err(e) = worker.run() {
                error!("virtio_media worker exited with error: {:#}", e);
            }
        });

        self.worker_thread = Some(worker_thread);
        Ok(())
    }

    fn reset(&mut self) -> anyhow::Result<()> {
        if let Some(worker_thread) = self.worker_thread.take() {
            worker_thread.stop();
        }

        Ok(())
    }

    fn get_shared_memory_region(&self) -> Option<SharedMemoryRegion> {
        // With a pool the guest already has every buffer mapped; declaring a BAR would only
        // fail to fit on Gunyah (VPU_DESIGN.md §0.2, §3.2).
        if self.pool.is_some() {
            return None;
        }
        Some(SharedMemoryRegion {
            id: 0,
            // We need a 32-bit address space as m2m devices start their CAPTURE buffers' offsets
            // at 2GB.
            length: HOST_MAPPER_RANGE,
        })
    }

    fn set_shared_memory_mapper(&mut self, mapper: Box<dyn SharedMemoryMapper>) {
        self.shm_mapper = Some(ArcedMemoryMapper::from(mapper));
    }
}

/// The card name a config area carries, for log lines that have to say which device spoke.
fn card_str(card: &[u8; 32]) -> String {
    let end = card.iter().position(|&b| b == 0).unwrap_or(card.len());
    String::from_utf8_lossy(&card[..end]).into_owned()
}

/// The `card` field of the config area: `name`, truncated to fit with a terminating NUL.
fn card_name(name: &str) -> [u8; 32] {
    let mut card = [0u8; 32];
    let n = name.len().min(card.len() - 1);
    card[..n].copy_from_slice(&name.as_bytes()[..n]);
    card
}

/// Create a simple media capture device.
///
/// This device can only generate a fixed pattern at a fixed resolution, and should only be used
/// for checking that the virtio-media pipeline is working properly.
pub fn create_virtio_media_simple_capture_device(
    features: u64,
    pool: Option<MediaPool>,
) -> Box<dyn VirtioDevice> {
    use virtio_media::devices::SimpleCaptureDevice;
    use virtio_media::v4l2r::ioctl::Capabilities;

    let device = CrosvmVirtioMediaDevice::new(
        features,
        VirtioMediaDeviceConfig {
            device_caps: (Capabilities::VIDEO_CAPTURE | Capabilities::STREAMING).bits(),
            // VFL_TYPE_VIDEO
            device_type: 0,
            card: card_name("simple_device"),
        },
        pool,
        |event_queue, guest_mapper, mapper, allocator| {
            Ok(SimpleCaptureDevice::new(
                event_queue,
                guest_mapper,
                mapper,
                allocator,
            ))
        },
    );

    Box::new(device)
}

/// Create a memory-to-memory loopback device: OUTPUT buffers are copied into CAPTURE buffers.
///
/// A test device for the buffer memory model (`VPU_DESIGN.md` §4.2): both queues take `MMAP` and
/// `USERPTR` buffers, so guest-owned and host-owned buffers can be exercised in every
/// combination from the guest.
pub fn create_virtio_media_loopback_device(
    features: u64,
    card: &str,
    pool: Option<MediaPool>,
) -> Box<dyn VirtioDevice> {
    use virtio_media::devices::LoopbackDevice;
    use virtio_media::v4l2r::ioctl::Capabilities;

    let device = CrosvmVirtioMediaDevice::new(
        features,
        VirtioMediaDeviceConfig {
            device_caps: (Capabilities::VIDEO_M2M_MPLANE | Capabilities::STREAMING).bits(),
            // VFL_TYPE_VIDEO
            device_type: 0,
            card: card_name(card),
        },
        pool,
        |event_queue, guest_mapper, mapper, allocator| {
            Ok(LoopbackDevice::new(
                event_queue,
                guest_mapper,
                mapper,
                allocator,
            ))
        },
    );

    Box::new(device)
}

/// Create a proxy device for a host V4L2 device.
///
/// Since V4L2 is a Linux-specific API, this is only available on Linux targets.
///
/// The proxied buffers belong to the host device, not to any pool, so this device always uses
/// the shared-memory BAR shape (which does not fit on Gunyah next to a GPU).
#[cfg(any(target_os = "android", target_os = "linux"))]
pub fn create_virtio_media_v4l2_proxy_device<P: AsRef<Path>>(
    features: u64,
    device_path: P,
) -> anyhow::Result<Box<dyn VirtioDevice>> {
    use virtio_media::devices::V4l2ProxyDevice;
    use virtio_media::v4l2r;
    use virtio_media::v4l2r::ioctl::Capabilities;

    let device = v4l2r::device::Device::open(
        device_path.as_ref(),
        v4l2r::device::DeviceConfig::new().non_blocking_dqbuf(),
    )?;
    let mut device_caps = device.caps().device_caps();

    // We are only exposing one device worth of capabilities.
    device_caps.remove(Capabilities::DEVICE_CAPS);

    // Read-write is not supported by design.
    device_caps.remove(Capabilities::READWRITE);

    let config = VirtioMediaDeviceConfig {
        device_caps: device_caps.bits(),
        // VFL_TYPE_VIDEO
        device_type: 0,
        card: card_name(&device.caps().card),
    };
    let device_path = PathBuf::from(device_path.as_ref());

    let device = CrosvmVirtioMediaDevice::new(
        features,
        config,
        None,
        // The proxied buffers belong to the host V4L2 device, so the proxy has no allocator of
        // its own to be handed.
        move |event_queue, guest_mapper, mapper, _allocator| {
            let device =
                V4l2ProxyDevice::new(device_path.clone(), event_queue, guest_mapper, mapper);

            Ok(device)
        },
    );

    Ok(Box::new(device))
}

/// Create a decoder adapter device.
///
/// This is a regular virtio-media decoder device leveraging the virtio-video decoder backends.
#[cfg(feature = "video-decoder")]
pub fn create_virtio_media_decoder_adapter_device(
    features: u64,
    _gpu_tube: base::Tube,
    backend: VideoBackendType,
    pool: Option<MediaPool>,
) -> anyhow::Result<Box<dyn VirtioDevice>> {
    use decoder_adapter::VirtioVideoAdapter;
    use virtio_media::devices::video_decoder::VideoDecoder;
    use virtio_media::v4l2r::ioctl::Capabilities;

    #[cfg(feature = "ffmpeg")]
    use crate::virtio::video::decoder::backend::ffmpeg::FfmpegDecoder;
    #[cfg(feature = "vaapi")]
    use crate::virtio::video::decoder::backend::vaapi::VaapiDecoder;
    #[cfg(feature = "libvda")]
    use crate::virtio::video::decoder::backend::vda::LibvdaDecoder;
    use crate::virtio::video::decoder::DecoderBackend;

    let card_name_str = format!("{:?} decoder adapter", backend).to_lowercase();
    let config = VirtioMediaDeviceConfig {
        device_caps: (Capabilities::VIDEO_M2M_MPLANE | Capabilities::STREAMING).bits(),
        // VFL_TYPE_VIDEO
        device_type: 0,
        card: card_name(&card_name_str),
    };

    let create_device = move |event_queue, _, mapper: HostMapper, allocator: BufferAllocator| {
        let backend = match backend {
            #[cfg(feature = "libvda")]
            VideoBackendType::Libvda => {
                LibvdaDecoder::new(libvda::decode::VdaImplType::Gavda)?.into_trait_object()
            }
            #[cfg(feature = "libvda")]
            VideoBackendType::LibvdaVd => {
                LibvdaDecoder::new(libvda::decode::VdaImplType::Gavd)?.into_trait_object()
            }
            #[cfg(feature = "vaapi")]
            VideoBackendType::Vaapi => VaapiDecoder::new()?.into_trait_object(),
            #[cfg(feature = "ffmpeg")]
            VideoBackendType::Ffmpeg => FfmpegDecoder::new().into_trait_object(),
        };

        let adapter = VirtioVideoAdapter::new(backend);
        let decoder = VideoDecoder::new(adapter, event_queue, mapper, allocator);

        Ok(decoder)
    };

    Ok(Box::new(CrosvmVirtioMediaDevice::new(
        features,
        config,
        pool,
        create_device,
    )))
}
