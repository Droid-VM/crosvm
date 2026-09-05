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
//! * `Bar`, the upstream shape: a memfd per buffer, mapped into a 4 GiB PCI shared-memory BAR the
//!   guest reaches through `virtio_get_shm_region()`. What a KVM VM without a pool gets.
//! * `Pool`: buffers are slices of the `media_host` pool (`--pre-alloc media-host-mb=N`), which the
//!   guest already maps as a whole from its `media_host` reserved-memory node; the device declares
//!   no shared-memory region at all, and the offset it answers `MMAP` with is the buffer's offset
//!   inside the pool. This is the only shape that works on Gunyah, whose 64-bit MMIO window has
//!   room for one 4 GiB BAR and the GPU already has it.
//!
//! Guest-owned (`USERPTR`) buffers are resolved in `guest_buf` (`VPU_DESIGN.md` §3.4).
//!
//! A device runs either inside the VMM (`CrosvmVirtioMediaDevice`, a `VirtioDevice`) or in a
//! vhost-user helper process under the app's uid (`vhost::user::device::media`, `VPU_DESIGN.md`
//! §6). Both drive the same [`Worker`] on the same kind of OS thread ([`start_worker`]) with the
//! same [`EventQueue`], [`GuestMemoryMapper`], [`HostMapper`] and [`BufferAllocator`]; what
//! differs is who hands them the queues and who answers whether the host may touch a guest
//! address (`guest_buf::HostAccessPolicy`).

pub mod android_camera_backend;
pub mod android_codec_backend;
pub mod guest_buf;
pub mod kill;
pub mod pool;

use std::collections::BTreeMap;
use std::os::fd::AsRawFd;
use std::os::fd::BorrowedFd;
use std::path::Path;
use std::path::PathBuf;
use std::rc::Rc;
use std::sync::Arc;

use anyhow::Context;
use base::error;
use base::warn;
use base::Descriptor;
use base::EventToken;
use base::EventType;
use base::Protection;
use base::SafeDescriptor;
use base::WaitContext;
use base::WorkerThread;
use resources::address_allocator::AddressAllocator;
use resources::AddressRange;
use resources::Alloc;
use serde::Deserialize;
use serde::Serialize;
use serde_keyvalue::FromKeyValues;
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

use crate::virtio::copy_config;
use crate::virtio::device_constants::media::QUEUE_SIZES;
#[cfg(feature = "video-decoder")]
use crate::virtio::device_constants::video::VideoBackendType;
use crate::virtio::media::guest_buf::GuestBufferImport;
use crate::virtio::media::guest_buf::HostAccessPolicy;
use crate::virtio::media::kill::KillSignal;
use crate::virtio::media::kill::Wakeup;
pub use crate::virtio::media::pool::MediaPool;
use crate::virtio::media::pool::PoolBufferAllocator;
use crate::virtio::DeviceType;
use crate::virtio::Interrupt;
use crate::virtio::Queue;
use crate::virtio::Reader;
use crate::virtio::SharedMemoryMapper;
use crate::virtio::SharedMemoryRegion;
use crate::virtio::VirtioDevice;
use crate::virtio::Writer;

/// What a `--virtio-media` device is (`VPU_DESIGN.md` §3.5).
///
/// Lives here rather than in the VMM's config because a vhost-user helper is told the same thing
/// in its parameters and must spell it the same way.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, FromKeyValues)]
#[serde(deny_unknown_fields, rename_all = "lowercase")]
pub enum MediaDeviceKind {
    /// The crate's pattern-generating capture device; what `--simple-media-device` makes.
    Simple,
    /// A memory-to-memory device copying OUTPUT buffers into CAPTURE buffers, for testing the
    /// buffer memory model from the guest.
    Loopback,
    /// A host camera, over the Camera2 NDK; helper only (`uid=` required).
    Camera,
    /// A host video decoder, over the MediaCodec NDK; helper only (`uid=` required).
    Decoder,
    /// A host video encoder, over the MediaCodec NDK; helper only (`uid=` required).
    Encoder,
}

/// Where the device for a [`MediaDeviceKind`] is built (`VPU_DESIGN.md` §3.5, §6, §7.1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MediaDeviceSupport {
    /// In the VMM's own process, and in a helper process when `uid=` asks for one.
    InVmmOrHelper,
    /// In a helper only: the device talks to a host service that resolves the caller from the
    /// real uid, so there is nothing the VMM could build for itself and `uid=` is required.
    HelperOnly,
    /// Nowhere yet. The kind still parses, so a command line written for a later milestone is
    /// refused by name instead of starting something else.
    Unimplemented,
}

impl MediaDeviceKind {
    /// Every kind, so that a list of kinds in a message cannot drift from the enum.
    pub const ALL: [MediaDeviceKind; 5] = [
        MediaDeviceKind::Simple,
        MediaDeviceKind::Loopback,
        MediaDeviceKind::Camera,
        MediaDeviceKind::Decoder,
        MediaDeviceKind::Encoder,
    ];

    /// The `kind=` spelling `--virtio-media` parses, which is what a message to an operator
    /// should use: `Debug` says `Camera`, the command line says `camera`.
    pub fn as_str(self) -> &'static str {
        match self {
            MediaDeviceKind::Simple => "simple",
            MediaDeviceKind::Loopback => "loopback",
            MediaDeviceKind::Camera => "camera",
            MediaDeviceKind::Decoder => "decoder",
            MediaDeviceKind::Encoder => "encoder",
        }
    }

    /// Where this kind's device is built -- the one table saying which kinds exist and where.
    ///
    /// The VMM reads it *before* it decides whether to run the device in a helper, so a kind
    /// nothing implements is refused there, by name, rather than launching a helper that
    /// refuses it on the far side of a vhost-user socket the VMM can then only report as reset
    /// (`logs/vpu_wp/B4-acceptance.md` §7.1, defect D15). The helper reads it too, so both
    /// sides refuse the same kinds with the same words.
    pub fn support(self) -> MediaDeviceSupport {
        match self {
            MediaDeviceKind::Simple | MediaDeviceKind::Loopback => {
                MediaDeviceSupport::InVmmOrHelper
            }
            // cameraserver refuses uid 0, so the camera exists in the helper alone (design §7.1);
            // the codecs follow the one process model (design §7.4).
            MediaDeviceKind::Camera | MediaDeviceKind::Decoder | MediaDeviceKind::Encoder => {
                MediaDeviceSupport::HelperOnly
            }
        }
    }

    /// The refusal an unimplemented kind gets, worded identically wherever it is refused. Every
    /// kind of today is implemented; the wording stays for the next one added to the enum.
    pub fn unimplemented_message(self) -> String {
        format!(
            "--virtio-media kind={} is not implemented yet (only {} are)",
            self.as_str(),
            Self::implemented_kinds()
        )
    }

    /// The kinds something implements, as prose: `simple, loopback and camera`.
    fn implemented_kinds() -> String {
        let names: Vec<&'static str> = Self::ALL
            .iter()
            .filter(|kind| kind.support() != MediaDeviceSupport::Unimplemented)
            .map(|kind| kind.as_str())
            .collect();
        match names.split_last() {
            None => "none".to_string(),
            Some((last, [])) => last.to_string(),
            Some((last, rest)) => format!("{} and {}", rest.join(", "), last),
        }
    }
}

#[cfg(test)]
mod media_device_kind_tests {
    use super::MediaDeviceKind;
    use super::MediaDeviceSupport;

    /// The table is the contract two processes rely on: the VMM refuses an unimplemented kind
    /// before it launches a helper, and the helper refuses it with the same words. Anything
    /// added to the enum has to be given a place here.
    #[test]
    fn every_kind_has_a_place_and_a_command_line_name() {
        assert_eq!(MediaDeviceKind::ALL.len(), 5);
        for kind in MediaDeviceKind::ALL {
            // Same spelling `--virtio-media kind=` parses, i.e. `Debug` lowercased.
            assert_eq!(kind.as_str(), format!("{:?}", kind).to_lowercase());
        }
        assert_eq!(
            MediaDeviceKind::Simple.support(),
            MediaDeviceSupport::InVmmOrHelper
        );
        assert_eq!(
            MediaDeviceKind::Loopback.support(),
            MediaDeviceSupport::InVmmOrHelper
        );
        assert_eq!(
            MediaDeviceKind::Camera.support(),
            MediaDeviceSupport::HelperOnly
        );
        assert_eq!(
            MediaDeviceKind::Decoder.support(),
            MediaDeviceSupport::HelperOnly
        );
        assert_eq!(
            MediaDeviceKind::Encoder.support(),
            MediaDeviceSupport::HelperOnly
        );
        assert!(MediaDeviceKind::ALL
            .iter()
            .all(|kind| kind.support() != MediaDeviceSupport::Unimplemented));
    }

    /// Every kind is implemented since M7, so the refusal is what a sixth kind would get: it
    /// names the kind and lists all five, the list being built from the table, never typed.
    #[test]
    fn the_refusal_names_the_kind_and_lists_what_exists() {
        assert_eq!(
            MediaDeviceKind::Encoder.unimplemented_message(),
            "--virtio-media kind=encoder is not implemented yet (only simple, loopback, camera, \
             decoder and encoder are)"
        );
    }
}

/// Structure supporting the implementation of `VirtioMediaEventQueue` for sending events to the
/// driver.
///
/// The queue is shared rather than owned because the device that holds this is dropped on the
/// worker thread, and a vhost-user backend has to give the queue back to the frontend afterwards
/// (`stop_queue`); the in-VMM device never asks for it again. Only the worker thread ever locks
/// it.
pub struct EventQueue {
    queue: Arc<Mutex<Queue>>,
    /// The worker's kill event, so that waiting for the guest to hand over a descriptor cannot
    /// outlive the worker (`kill`'s module documentation; `logs/vpu_wp/M3.md` §9 item 2).
    /// [`start_worker`] arms it before the device that holds this queue is built.
    kill: KillSignal,
}

impl EventQueue {
    pub fn new(queue: Queue) -> Self {
        Self {
            queue: Arc::new(Mutex::new(queue)),
            kill: KillSignal::new(),
        }
    }

    /// A second handle on the queue, for whoever has to recover it once the device is gone.
    pub fn shared(&self) -> Arc<Mutex<Queue>> {
        Arc::clone(&self.queue)
    }

    /// The slot [`start_worker`] fills with the worker thread's kill event.
    pub fn kill_signal(&self) -> KillSignal {
        self.kill.clone()
    }
}

impl VirtioMediaEventQueue for EventQueue {
    /// Wait until an event descriptor becomes available and send `event` to the guest.
    ///
    /// A guest that supplies none -- because it is gone, or being reset -- would block the worker
    /// here forever, and with it the `reset` or `stop_queue` that is trying to join it, so the
    /// wait also ends on the worker's kill event; the event is then dropped, which is what a
    /// device being torn down owes the guest anyway.
    fn send_event(&mut self, event: V4l2Event) {
        let mut queue = self.queue.lock();
        let mut desc;

        loop {
            match queue.pop() {
                Some(d) => {
                    desc = d;
                    break;
                }
                None => match self.kill.wait_or_kill(queue.event()) {
                    Ok(Wakeup::Ready) => (),
                    Ok(Wakeup::Killed) => {
                        warn!("media worker is stopping: an event for the guest is dropped");
                        return;
                    }
                    Err(e) => {
                        error!("could not obtain a descriptor to send event to: {:#}", e);
                        return;
                    }
                },
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
        queue.add_used(desc, written);
        queue.trigger_interrupt();
    }
}

/// A `SharedMemoryMapper` behind an `Arc`, allowing it to be shared.
///
/// This is required by the fact that devices can be activated several times, but the mapper is
/// only provided once. This might be a defect of the `VirtioDevice` interface.
#[derive(Clone)]
pub struct ArcedMemoryMapper(Arc<Mutex<Box<dyn SharedMemoryMapper>>>);

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

                let descriptor: SafeDescriptor =
                    buffer.fd.try_clone().map_err(|_| libc::EIO)?.into();
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

    /// The trait's default is `usize::MAX`, "no bound I can state"; this one can state it, for
    /// both shapes an import takes. A `GuestArenaMapping` answers the length it mapped and a
    /// `GuestShadowMapping` the length of the vector it copied into, and both are the sum of the
    /// scatter-gather entries the ioctl layer actually read -- which is the number a device needs
    /// (review-m4 R2: the guest's `length` field and the list it sizes are two separate things
    /// the guest chooses).
    fn len(&self) -> usize {
        GuestBufferImport::len(self)
    }
}

/// Implements `VirtioMediaGuestMemoryMapper` on a `GuestMemory` and a `HostAccessPolicy`.
///
/// Whether to use a direct mapping or to copy the guest data into a shadow buffer is decided by
/// the size of the guest mapping (see `guest_buf::MAPPING_THRESHOLD`). A device that says which
/// way it will access the buffer (`new_mapping_for`) gets a mapping that can only be used that
/// way: an OUTPUT buffer is the guest's own memory, which on a protected VM it shared with the
/// host and goes on trusting, so the host maps it read-only (`VPU_DESIGN.md` §3.4).
///
/// Before any entry is mapped the policy is asked whether the host may touch it: inside the VMM
/// the `GuestMemory` answers for itself, in a helper the VMM's window list does (`VPU_DESIGN.md`
/// §6.2). Either way a refusal reaches the guest as `EFAULT`, never as a fault in the host.
pub struct GuestMemoryMapper {
    mem: GuestMemory,
    policy: HostAccessPolicy,
}

impl GuestMemoryMapper {
    /// The in-VMM shape: `mem` knows its purposes and whether the VM is protected.
    pub fn new(mem: GuestMemory) -> Self {
        Self::with_policy(mem, HostAccessPolicy::GuestMemory)
    }

    pub fn with_policy(mem: GuestMemory, policy: HostAccessPolicy) -> Self {
        Self { mem, policy }
    }

    pub fn guest_memory(&self) -> &GuestMemory {
        &self.mem
    }
}

impl GuestMemoryMapper {
    fn import(&self, sgs: Vec<SgEntry>, prot: Protection) -> anyhow::Result<GuestBufferImport> {
        let ranges: Vec<(GuestAddress, usize)> = sgs
            .iter()
            .map(|sg| (GuestAddress(sg.start), sg.len as usize))
            .collect();
        GuestBufferImport::new(&self.mem, ranges, prot, &self.policy).map_err(|e| {
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
pub enum Token {
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
///
/// Shared by the in-VMM device and the vhost-user backend; see [`start_worker`].
pub struct Worker<D: VirtioMediaDevice<Reader, Writer>> {
    runner: VirtioMediaDeviceRunner<Reader, Writer, D, WaitContextPoller>,
    cmd_queue: Queue,
    wait_ctx: Rc<WaitContext<Token>>,
}

impl<D> Worker<D>
where
    D: VirtioMediaDevice<Reader, Writer>,
{
    /// Create a new worker instance for `device`. `wait_ctx` must already carry the command
    /// queue's event as `Token::CommandQueue` and the kill event as `Token::Kill` (see
    /// [`start_worker`]).
    pub fn new(device: D, cmd_queue: Queue, wait_ctx: Rc<WaitContext<Token>>) -> Self {
        Self {
            runner: VirtioMediaDeviceRunner::new(device, WaitContextPoller(Rc::clone(&wait_ctx))),
            cmd_queue,
            wait_ctx,
        }
    }

    /// Give the command queue back, dropping the device and its sessions.
    pub fn into_cmd_queue(self) -> Queue {
        self.cmd_queue
    }

    pub fn run(&mut self) -> anyhow::Result<()> {
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

/// Run a virtio-media device on its own OS thread until the thread is told to stop.
///
/// This is the one thread model both process shapes use (`VPU_DESIGN.md` §6.2): the device is
/// built by `create_device` *on the worker thread* and dropped there when the thread ends, so a
/// device that is `!Send` -- M4's camera, whose NDK handles are raw pointers -- can be opened and
/// closed on the thread that drives it; only the closure that builds it has to be `Send`. The
/// thread hands the command queue back when it exits, which is how a vhost-user backend returns
/// the queue to the frontend in `stop_queue`; the event queue is recovered through the
/// [`EventQueue::shared`] handle once the device is gone.
///
/// A device that cannot be built, or a wait context that cannot be armed, is logged and the queue
/// is simply given back: the guest then sees a device that never answers, and the VM lives.
///
/// `kill_signal` is the event queue's ([`EventQueue::kill_signal`]): it is armed with this
/// thread's kill event before the device is built, so a `send_event` that blocks waiting for the
/// guest ends when the thread is told to stop instead of holding the joiner forever (`kill`).
pub fn start_worker<D, F>(
    create_device: F,
    cmd_queue: Queue,
    wait_ctx: WaitContext<Token>,
    kill_signal: KillSignal,
) -> WorkerThread<Queue>
where
    D: VirtioMediaDevice<Reader, Writer> + 'static,
    F: FnOnce() -> anyhow::Result<D> + Send + 'static,
{
    WorkerThread::start("v_media_worker", move |kill_evt| {
        // Before the device exists, so no event can be sent before the wait can be interrupted.
        if let Err(e) = kill_signal.arm(&kill_evt) {
            error!("virtio-media events will not be interruptible: {:#}", e);
        }
        if let Err(e) = wait_ctx
            .add_many(&[
                (cmd_queue.event(), Token::CommandQueue),
                (&kill_evt, Token::Kill),
            ])
            .context("when adding worker events to wait context")
        {
            error!("failed to create virtio-media worker: {:#}", e);
            return cmd_queue;
        }
        let device = match create_device() {
            Ok(device) => device,
            Err(e) => {
                error!("failed to create virtio-media device: {:#}", e);
                return cmd_queue;
            }
        };
        let mut worker = Worker::new(device, cmd_queue, Rc::new(wait_ctx));
        if let Err(e) = worker.run() {
            error!("virtio_media worker exited with error: {:#}", e);
        }
        worker.into_cmd_queue()
    })
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
    worker_thread: Option<WorkerThread<Queue>>,
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
        let event_queue = EventQueue::new(queues.remove(&1).context("missing queue 1")?);

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
        let kill_signal = event_queue.kill_signal();
        let device =
            (self.create_device)(event_queue, GuestMemoryMapper::new(mem), mapper, allocator)?;

        // The device was built here, on the VMM's thread, so that a device that cannot be built
        // fails the activation rather than only a log line; it is moved to the worker thread.
        self.worker_thread = Some(start_worker(
            move || Ok(device),
            cmd_queue,
            wait_ctx,
            kill_signal,
        ));
        Ok(())
    }

    fn reset(&mut self) -> anyhow::Result<()> {
        if let Some(worker_thread) = self.worker_thread.take() {
            // The queue comes back with the thread; there is nothing to do with it here.
            let _ = worker_thread.stop();
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
pub(crate) fn card_str(card: &[u8; 32]) -> String {
    let end = card.iter().position(|&b| b == 0).unwrap_or(card.len());
    String::from_utf8_lossy(&card[..end]).into_owned()
}

/// The `card` field of the config area: `name`, truncated to fit with a terminating NUL.
pub(crate) fn card_name(name: &str) -> [u8; 32] {
    let mut card = [0u8; 32];
    let n = name.len().min(card.len() - 1);
    card[..n].copy_from_slice(&name.as_bytes()[..n]);
    card
}

/// The virtio config area of a `simple` device, wherever it runs.
pub fn simple_capture_config() -> VirtioMediaDeviceConfig {
    use virtio_media::v4l2r::ioctl::Capabilities;

    VirtioMediaDeviceConfig {
        device_caps: (Capabilities::VIDEO_CAPTURE | Capabilities::STREAMING).bits(),
        // VFL_TYPE_VIDEO
        device_type: 0,
        card: card_name("simple_device"),
    }
}

/// The virtio config area of a `loopback` device called `card`, wherever it runs.
pub fn loopback_config(card: &str) -> VirtioMediaDeviceConfig {
    use virtio_media::v4l2r::ioctl::Capabilities;

    VirtioMediaDeviceConfig {
        device_caps: (Capabilities::VIDEO_M2M_MPLANE | Capabilities::STREAMING).bits(),
        // VFL_TYPE_VIDEO
        device_type: 0,
        card: card_name(card),
    }
}

/// The virtio config area of a `camera` device called `card`: a multi-planar capture device
/// (`VPU_DESIGN.md` §7.1).
pub fn camera_config(card: &str) -> VirtioMediaDeviceConfig {
    use virtio_media::v4l2r::ioctl::Capabilities;

    VirtioMediaDeviceConfig {
        device_caps: (Capabilities::VIDEO_CAPTURE_MPLANE | Capabilities::STREAMING).bits(),
        // VFL_TYPE_VIDEO
        device_type: 0,
        card: card_name(card),
    }
}

/// The virtio config area of a `decoder` device called `card`: a multi-planar memory-to-memory
/// device, the kernel's stateful decoder interface (`VPU_DESIGN.md` §7.2).
pub fn decoder_config(card: &str) -> VirtioMediaDeviceConfig {
    use virtio_media::v4l2r::ioctl::Capabilities;

    VirtioMediaDeviceConfig {
        device_caps: (Capabilities::VIDEO_M2M_MPLANE | Capabilities::STREAMING).bits(),
        // VFL_TYPE_VIDEO
        device_type: 0,
        card: card_name(card),
    }
}

/// The virtio config area of an `encoder` device called `card`: a multi-planar memory-to-memory
/// device, the kernel's stateful encoder interface (`VPU_DESIGN.md` §7.3).
pub fn encoder_config(card: &str) -> VirtioMediaDeviceConfig {
    use virtio_media::v4l2r::ioctl::Capabilities;

    VirtioMediaDeviceConfig {
        device_caps: (Capabilities::VIDEO_M2M_MPLANE | Capabilities::STREAMING).bits(),
        // VFL_TYPE_VIDEO
        device_type: 0,
        card: card_name(card),
    }
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

    let device = CrosvmVirtioMediaDevice::new(
        features,
        simple_capture_config(),
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

    let device = CrosvmVirtioMediaDevice::new(
        features,
        loopback_config(card),
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

/// The virtio-video decoder adapter is gone.
///
/// `decoder_adapter.rs` implemented the fork's *previous* `VideoDecoderBackend`, the
/// virtio-video-shaped one; `logs/vpu_wp/M6-crate.md` §1 replaced that trait with the one the
/// MediaCodec backend (`android_codec_backend`) implements, and no soong target ever built the
/// adapter (design §4.4: "do not reference"). Rather than leave a file that breaks the
/// `video-decoder` feature build, the adapter was removed and this entry point, which the
/// `--virtio-media-adapter` command line still reaches under that feature, says so.
#[cfg(feature = "video-decoder")]
pub fn create_virtio_media_decoder_adapter_device(
    _features: u64,
    _gpu_tube: base::Tube,
    backend: VideoBackendType,
    _pool: Option<MediaPool>,
) -> anyhow::Result<Box<dyn VirtioDevice>> {
    anyhow::bail!(
        "the virtio-media decoder adapter over the {:?} virtio-video backend was removed with the \
         decoder trait it implemented (logs/vpu_wp/M6-crate.md §1); the host video decoder is \
         --virtio-media kind=decoder,uid=<app uid>",
        backend
    )
}
