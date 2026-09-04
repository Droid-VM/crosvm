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

/// What backs host-owned (`MMAP`) buffers, and how the guest gets to see them.
///
/// One value implements both of the crate's host-side traits: it hands buffers out
/// (`VirtioMediaBufferAllocator`) and makes them visible to the guest
/// (`VirtioMediaHostMemoryMapper`), because in `Pool` mode those are the same act -- a buffer's
/// place in the pool *is* its guest mapping.
pub enum HostBacking {
    /// Upstream: a memfd per buffer, mapped on demand into the PCI shared-memory BAR.
    Bar {
        shm_mapper: ArcedMemoryMapper,
        /// The BAR's offset space, `[0, HOST_MAPPER_RANGE)`, page aligned.
        allocator: AddressAllocator,
        memfd: MemFdAllocator,
    },
    /// DroidVM: buffers are slices of the `media_host` pool the guest maps as a whole.
    Pool {
        pool: MediaPoolHandle,
        /// The pool's offset space, `[0, pool.size)`, page aligned.
        allocator: AddressAllocator,
        /// Our own mapping of the pool, so that buffers can be filled from this process.
        host_map: MemoryMapping,
        /// Key for the next allocation; `AddressAllocator` wants every live allocation to have a
        /// distinct one.
        next_id: usize,
        /// Bytes currently handed out, for the exhaustion log line.
        used: u64,
    },
}

impl HostBacking {
    fn bar(shm_mapper: ArcedMemoryMapper) -> anyhow::Result<Self> {
        Ok(HostBacking::Bar {
            shm_mapper,
            allocator: AddressAllocator::new(
                AddressRange::from_start_and_end(0, HOST_MAPPER_RANGE - 1),
                Some(base::pagesize() as u64),
                None,
            )?,
            memfd: MemFdAllocator::new(),
        })
    }

    /// A backing over `pool`, with its own dup of the descriptor and its own mapping of the pool
    /// (a device can be activated more than once, and a helper process could not use the VMM's
    /// `host_va` anyway).
    fn pool(pool: &MediaPoolHandle) -> anyhow::Result<Self> {
        let fd = pool
            .fd
            .try_clone()
            .context("cannot dup the media_host pool descriptor")?;
        let file = File::from(
            fd.try_clone()
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
        info!(
            "virtio-media: serving MMAP buffers from the media_host pool (gpa {:#x}, {} MiB)",
            pool.gpa,
            pool.size >> 20
        );
        Ok(HostBacking::Pool {
            pool: MediaPoolHandle {
                fd,
                fd_offset: pool.fd_offset,
                host_va: pool.host_va,
                gpa: pool.gpa,
                size: pool.size,
            },
            allocator,
            host_map,
            next_id: 0,
            used: 0,
        })
    }
}

impl VirtioMediaHostMemoryMapper for HostBacking {
    fn add_mapping(&mut self, buffer: &HostBuffer, offset: u64, rw: bool) -> Result<u64, i32> {
        match self {
            HostBacking::Bar {
                shm_mapper,
                allocator,
                ..
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
            // The guest maps the whole pool; a buffer's offset inside it is its address.
            HostBacking::Pool { .. } => buffer.pool_offset.ok_or_else(|| {
                error!("virtio-media: a buffer that is not in the pool cannot be mapped");
                libc::EINVAL
            }),
        }
    }

    fn remove_mapping(&mut self, offset: u64) -> Result<(), i32> {
        match self {
            HostBacking::Bar {
                shm_mapper,
                allocator,
                ..
            } => {
                let _ = allocator.release_containing(offset);
                shm_mapper.remove_mapping(offset).map_err(|_| libc::EINVAL)
            }
            // Nothing was mapped for it.
            HostBacking::Pool { .. } => Ok(()),
        }
    }
}

impl VirtioMediaBufferAllocator for HostBacking {
    fn allocate(&mut self, len: u64) -> Result<HostBuffer, i32> {
        match self {
            HostBacking::Bar { memfd, .. } => memfd.allocate(len),
            HostBacking::Pool {
                pool,
                allocator,
                host_map,
                next_id,
                used,
            } => {
                if len == 0 {
                    return Err(libc::EINVAL);
                }
                let page = base::pagesize() as u64;
                let size = len.checked_next_multiple_of(page).ok_or(libc::EINVAL)?;
                let id = *next_id;
                *next_id += 1;
                // Every allocation gets its own key; the pool is private to this device, so a
                // plain counter is as good a name as any.
                let offset = allocator
                    .allocate(size, Alloc::Anon(id), "media buffer".into())
                    .map_err(|e| {
                        // The one place exhaustion surfaces: REQBUFS/CREATE_BUFS get ENOMEM,
                        // and the log says how full the pool was (VPU_DESIGN.md §4.1).
                        error!(
                            "virtio-media: media_host pool exhausted: {} bytes requested with \
                             {} of {} in use ({:?})",
                            size, used, pool.size, e
                        );
                        libc::ENOMEM
                    })?;
                *used += size;

                let fd: OwnedFd = match pool.fd.try_clone() {
                    Ok(fd) => fd.into(),
                    Err(e) => {
                        error!("virtio-media: cannot dup the pool descriptor: {}", e);
                        let _ = allocator.release_containing(offset);
                        *used -= size;
                        return Err(libc::EIO);
                    }
                };
                // SAFETY: `offset + size <= pool.size` (the allocator's range), and `host_map`
                // maps all `pool.size` bytes for as long as this backing exists, which is longer
                // than any buffer it hands out (buffers come back through `release`).
                let ptr = unsafe { host_map.as_ptr().add(offset as usize) };
                let ptr = NonNull::new(ptr).ok_or(libc::EIO)?;
                // SAFETY: `ptr` maps the `len` bytes at `pool.fd_offset + offset` of `fd`, and
                // stays valid for the life of the backing (see above).
                Ok(unsafe {
                    HostBuffer::from_raw_parts(fd, pool.fd_offset + offset, len, ptr, Some(offset))
                })
            }
        }
    }

    fn release(&mut self, buf: HostBuffer) {
        match self {
            HostBacking::Bar { memfd, .. } => memfd.release(buf),
            HostBacking::Pool {
                allocator, used, ..
            } => {
                if let Some(offset) = buf.pool_offset {
                    match allocator.release_containing(offset) {
                        Ok(range) => *used = used.saturating_sub(range.len().unwrap_or(0)),
                        Err(e) => error!(
                            "virtio-media: releasing a buffer at pool offset {:#x} that is not \
                             allocated: {}",
                            offset, e
                        ),
                    }
                }
                // Dropping closes the descriptor dup; the pool mapping is ours, not the
                // buffer's.
                drop(buf);
            }
        }
    }
}

/// A `HostBacking` a device can hold from two places at once: as its host memory mapper and as
/// its buffer allocator.
///
/// The lock is uncontended -- a device runs on one worker thread -- and exists only because the
/// two crate traits are implemented by one object.
#[derive(Clone)]
pub struct SharedHostBacking(Arc<Mutex<HostBacking>>);

impl SharedHostBacking {
    pub fn new(backing: HostBacking) -> Self {
        Self(Arc::new(Mutex::new(backing)))
    }
}

impl VirtioMediaHostMemoryMapper for SharedHostBacking {
    fn add_mapping(&mut self, buffer: &HostBuffer, offset: u64, rw: bool) -> Result<u64, i32> {
        self.0.lock().add_mapping(buffer, offset, rw)
    }

    fn remove_mapping(&mut self, shm_offset: u64) -> Result<(), i32> {
        self.0.lock().remove_mapping(shm_offset)
    }
}

impl VirtioMediaBufferAllocator for SharedHostBacking {
    fn allocate(&mut self, len: u64) -> Result<HostBuffer, i32> {
        self.0.lock().allocate(len)
    }

    fn release(&mut self, buf: HostBuffer) {
        self.0.lock().release(buf)
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
/// the size of the guest mapping (see `guest_buf::MAPPING_THRESHOLD`). The crate's trait does not
/// say which way the device will access the memory, so the mapping is read-write; a device that
/// knows can build a `GuestBufferImport` with the right protection itself.
pub struct GuestMemoryMapper(GuestMemory);

impl GuestMemoryMapper {
    pub fn new(mem: GuestMemory) -> Self {
        Self(mem)
    }

    pub fn guest_memory(&self) -> &GuestMemory {
        &self.0
    }
}

impl VirtioMediaGuestMemoryMapper for GuestMemoryMapper {
    type GuestMemoryMapping = GuestBufferImport;

    fn new_mapping(&self, sgs: Vec<SgEntry>) -> anyhow::Result<Self::GuestMemoryMapping> {
        let ranges: Vec<(GuestAddress, usize)> = sgs
            .iter()
            .map(|sg| (GuestAddress(sg.start), sg.len as usize))
            .collect();
        GuestBufferImport::new(&self.0, ranges, Protection::read_write()).map_err(|e| {
            // The errno travels as the error's root so the device can hand it to the guest.
            anyhow::Error::from(GuestMappingError(e.errno())).context(e.to_string())
        })
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
    F: Fn(EventQueue, GuestMemoryMapper, SharedHostBacking) -> anyhow::Result<D>,
> {
    /// Closure to create the device once all its resources are acquired.
    create_device: F,
    /// Virtio configuration area.
    config: VirtioMediaDeviceConfig,

    /// Virtio device features.
    base_features: u64,
    /// The `media_host` pool, when the VM has one: `MMAP` buffers are served out of it and no
    /// shared-memory BAR is declared.
    pool: Option<MediaPoolHandle>,
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
    F: Fn(EventQueue, GuestMemoryMapper, SharedHostBacking) -> anyhow::Result<D>,
{
    fn new(
        base_features: u64,
        config: VirtioMediaDeviceConfig,
        pool: Option<MediaPoolHandle>,
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
    F: Fn(EventQueue, GuestMemoryMapper, SharedHostBacking) -> anyhow::Result<D> + Send,
{
    fn keep_rds(&self) -> Vec<base::RawDescriptor> {
        let mut keep_rds = Vec::new();

        if let Some(fd) = self.shm_mapper.as_ref().and_then(|m| m.as_raw_descriptor()) {
            keep_rds.push(fd);
        }
        if let Some(pool) = &self.pool {
            keep_rds.push(pool.fd.as_raw_descriptor());
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

        let backing = match &self.pool {
            Some(pool) => HostBacking::pool(pool)?,
            None => {
                let shm_mapper = self
                    .shm_mapper
                    .clone()
                    .context("shared memory mapper was not specified")?;
                HostBacking::bar(shm_mapper)?
            }
        };

        let wait_ctx = WaitContext::new()?;
        let device = (self.create_device)(
            event_queue,
            GuestMemoryMapper::new(mem),
            SharedHostBacking::new(backing),
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
    pool: Option<MediaPoolHandle>,
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
        |event_queue, guest_mapper, backing| {
            Ok(SimpleCaptureDevice::new(
                event_queue,
                guest_mapper,
                backing.clone(),
                backing,
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
    pool: Option<MediaPoolHandle>,
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
        |event_queue, guest_mapper, backing| {
            Ok(LoopbackDevice::new(
                event_queue,
                guest_mapper,
                backing.clone(),
                backing,
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
        move |event_queue, guest_mapper, backing| {
            let device =
                V4l2ProxyDevice::new(device_path.clone(), event_queue, guest_mapper, backing);

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
    pool: Option<MediaPoolHandle>,
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

    let create_device = move |event_queue, _, backing: SharedHostBacking| {
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
        let decoder = VideoDecoder::new(adapter, event_queue, backing.clone(), backing);

        Ok(decoder)
    };

    Ok(Box::new(CrosvmVirtioMediaDevice::new(
        features,
        config,
        pool,
        create_device,
    )))
}
