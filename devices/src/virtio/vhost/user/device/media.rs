// Copyright 2026 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! The vhost-user virtio-media backend: a media device in a process of its own, under the app's
//! uid (`VPU_DESIGN.md` §6).
//!
//! `cameraserver` and the codec services decide what a client may do from the *real* uid of the
//! process that calls them, and refuse uid 0. crosvm is root, so a camera or codec device cannot
//! live in the VMM: it lives in `crosvm device media`, exec'd by the VMM
//! (`device_helper::launch`) with the app's uid, and the VMM talks to it through the ordinary
//! vhost-user frontend. The guest cannot tell the two shapes apart.
//!
//! What this process is *not* told, and how it copes:
//!
//! * It has no `ProtectionType`. The VMM stamps the one protection-dependent feature bit,
//!   `VIRTIO_F_ACCESS_PLATFORM`, into the parameters (`access_platform`), the way the snd helper is
//!   told; the frontend can only mask features, never add one.
//! * Its `GuestMemory` is rebuilt from `SET_MEM_TABLE`, which carries every region of the VMM's --
//!   the `media_host` pool included, memfd and all -- but no purpose and no protection flag. So the
//!   pool is found again by its guest-physical base (`pool_gpa`,
//!   [`crate::virtio::media::pool::pool_handle_at`]) at the first `start_queue`, when the table has
//!   arrived; and the host's right to touch a guest scatter-gather entry is decided by the window
//!   list the VMM computed (`access_windows`, `HostAccessPolicy::Windows`), not by the memory
//!   itself, so a bad entry is `EFAULT` to the guest rather than a `SIGBUS` here.
//! * It serves `MMAP` buffers from the pool only. The frontend forwards a shared-memory BAR to the
//!   GPU alone, so the `Bar` shape is not available out of process, and the VMM refuses `uid=`
//!   without a pool rather than start a device with no usable buffers.
//!
//! The device itself, its worker thread, and every trait implementation it is built from are the
//! in-VMM ones (`crate::virtio::media`); the two queues arrive one `start_queue` at a time and
//! the worker starts once both are here. The device is built *on* the worker thread, by the
//! factory the backend was given, so a device that is not `Send` -- the camera, whose NDK
//! handles must stay on the thread that made them -- is created, driven and dropped on one
//! thread (`logs/vpu_wp/M3.md` §6 item 1); the price is that a factory that fails is a log line
//! and a device that never answers, rather than a `start_queue` error.

pub mod sys;

use std::sync::Arc;

use anyhow::bail;
use anyhow::Context;
use base::error;
use base::warn;
use base::WaitContext;
use base::WorkerThread;
use hypervisor::ProtectionType;
use resources::AddressRange;
use serde::Deserialize;
use serde::Serialize;
use snapshot::AnySnapshot;
use sync::Mutex;
pub use sys::run_media_device;
pub use sys::Options;
use virtio_media::protocol::VirtioMediaDeviceConfig;
use virtio_media::VirtioMediaDevice;
use virtio_sys::virtio_config::VIRTIO_F_ACCESS_PLATFORM;
use vm_memory::GuestMemory;
use vmm_vhost::message::VhostUserProtocolFeatures;
use vmm_vhost::VHOST_USER_F_PROTOCOL_FEATURES;

use crate::virtio;
use crate::virtio::copy_config;
use crate::virtio::device_constants::media::QUEUE_SIZES;
use crate::virtio::media::card_str;
use crate::virtio::media::guest_buf::HostAccessPolicy;
use crate::virtio::media::pool::pool_handle_at;
use crate::virtio::media::start_worker;
use crate::virtio::media::BufferAllocator;
use crate::virtio::media::EventQueue;
use crate::virtio::media::GuestMemoryMapper;
use crate::virtio::media::HostMapper;
use crate::virtio::media::MediaDeviceKind;
use crate::virtio::media::MediaPool;
use crate::virtio::vhost::user::device::handler::Error as DeviceError;
use crate::virtio::vhost::user::device::handler::VhostUserDevice;
use crate::virtio::Queue;
use crate::virtio::Reader;
use crate::virtio::Writer;

/// Everything the helper is told, as `--config-json`. Built by the VMM
/// (`create_virtio_media_device`) from the `--virtio-media` option and its own `GuestMemory`.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MediaBackendParams {
    /// Which device to be. `simple`, `loopback`, `camera` and `decoder` exist today.
    pub kind: MediaDeviceKind,
    /// The V4L2 card name (`--virtio-media card=`); each kind has a default.
    #[serde(default)]
    pub card: Option<String>,
    /// Which host camera to expose (`kind=camera`).
    #[serde(default)]
    pub camera_id: Option<String>,
    /// `main` or `aux` (`kind=camera`).
    #[serde(default)]
    pub role: Option<String>,
    /// Offer software codecs too, not only hardware-accelerated ones (`kind=decoder`,
    /// `kind=encoder`; design §7.4).
    #[serde(default)]
    pub allow_sw: bool,
    /// Guest-physical base of the `media_host` pool. The region that starts here in the memory
    /// table the frontend sends is the pool; `MMAP` buffers are carved out of it.
    pub pool_gpa: u64,
    /// `(guest-physical base, size)` of every window the host may touch: the pools, the swiotlb
    /// region, shared RAM -- everything but memory lent to a protected guest. A scatter-gather
    /// entry outside all of them is `EFAULT`. For an unprotected VM this is every region.
    pub access_windows: Vec<(u64, u64)>,
    /// Advertise `VIRTIO_F_ACCESS_PLATFORM`, which makes the guest put this device's vrings and
    /// buffers through the DMA API instead of at addresses the host cannot reach. Set by the VMM
    /// from the VM's protection type; this process has no way to know.
    #[serde(default)]
    pub access_platform: bool,
}

/// The device once both queues are here: the thread that runs it and the handle through which
/// the event queue is recovered when it stops.
struct Running {
    thread: WorkerThread<Queue>,
    event_queue: Arc<Mutex<Queue>>,
}

/// A media device served over vhost-user, built from the same parts as the in-VMM one.
pub struct MediaBackend<D, F> {
    params: MediaBackendParams,
    config: VirtioMediaDeviceConfig,
    /// Shared with each worker thread, which calls it once to build its device.
    create_device: Arc<F>,
    avail_features: u64,
    /// `params.access_windows`, checked once.
    windows: Vec<AddressRange>,
    /// The `media_host` pool, found in the memory table at the first `start_queue`.
    pool: Option<MediaPool>,
    /// Queues handed over and not yet running (both must be here before the worker starts),
    /// or handed back by a stopped worker and not yet returned to the frontend.
    queues: [Option<Queue>; 2],
    running: Option<Running>,
    _device: std::marker::PhantomData<fn() -> D>,
}

impl<D, F> MediaBackend<D, F>
where
    D: VirtioMediaDevice<Reader, Writer> + 'static,
    F: Fn(EventQueue, GuestMemoryMapper, HostMapper, BufferAllocator) -> anyhow::Result<D>
        + Send
        + Sync
        + 'static,
{
    pub fn new(
        params: MediaBackendParams,
        config: VirtioMediaDeviceConfig,
        create_device: F,
    ) -> anyhow::Result<Self> {
        let windows = params
            .access_windows
            .iter()
            .map(|&(base, size)| {
                AddressRange::from_start_and_size(base, size)
                    .filter(|w| !w.is_empty())
                    .with_context(|| format!("bad access window {base:#x}+{size:#x}"))
            })
            .collect::<anyhow::Result<Vec<_>>>()?;
        // Unprotected unconditionally, because this process does not know what kind of VM it is
        // serving and cannot ask. The one protection-dependent bit is passed in instead: without
        // it the guest skips the DMA API for this device, puts its vrings at guest-physical
        // addresses, and the frontend fails to translate them at activate -- before any command.
        let mut avail_features = virtio::base_features(ProtectionType::Unprotected)
            | 1 << VHOST_USER_F_PROTOCOL_FEATURES;
        if params.access_platform {
            avail_features |= 1 << VIRTIO_F_ACCESS_PLATFORM;
        }
        Ok(MediaBackend {
            params,
            config,
            create_device: Arc::new(create_device),
            avail_features,
            windows,
            pool: None,
            queues: [None, None],
            running: None,
            _device: std::marker::PhantomData,
        })
    }

    /// Both queues are here: build the device and start its thread.
    fn start(&mut self, mem: GuestMemory) -> anyhow::Result<()> {
        // The pool is looked for in the memory table the first time a queue starts, because
        // that is the first time there is a memory table. It is the VM's, so it outlives every
        // reset; a later `SET_MEM_TABLE` describes the same memfd again.
        if self.pool.is_none() {
            let handle = pool_handle_at(&mem, self.params.pool_gpa)
                .context("the media_host pool is not in the memory table")?;
            self.pool = Some(MediaPool::new(handle).context("cannot set up the media_host pool")?);
        }
        let pool = self.pool.as_ref().expect("pool was just set");

        let cmd_queue = self.queues[0].take().context("command queue not started")?;
        let event_queue =
            EventQueue::new(self.queues[1].take().context("event queue not started")?);
        let shared_event_queue = event_queue.shared();
        let kill_signal = event_queue.kill_signal();

        let guest_mapper =
            GuestMemoryMapper::with_policy(mem, HostAccessPolicy::Windows(self.windows.clone()));
        let allocator = BufferAllocator::Pool(pool.lease(card_str(&self.config.card)));
        let wait_ctx = WaitContext::new().context("cannot create the worker's wait context")?;
        // The device is built on the worker thread (see the module documentation): only the
        // factory and the parts it is handed cross threads, never the device.
        let create_device = Arc::clone(&self.create_device);
        let create = move || {
            create_device(event_queue, guest_mapper, HostMapper::Pool, allocator)
                .context("cannot create the media device")
        };

        self.running = Some(Running {
            thread: start_worker(create, cmd_queue, wait_ctx, kill_signal),
            event_queue: shared_event_queue,
        });
        Ok(())
    }

    /// Stop the worker, dropping the device, and take both queues back.
    fn stop(&mut self) {
        if let Some(Running {
            thread,
            event_queue,
        }) = self.running.take()
        {
            let cmd_queue = thread.stop();
            // The device -- and with it its `EventQueue` -- was dropped on the thread that has
            // just been joined, so this is the last handle.
            let event_queue = match Arc::try_unwrap(event_queue) {
                Ok(queue) => queue.into_inner(),
                Err(_) => panic!("the event queue is still shared after the worker stopped"),
            };
            self.queues[0] = Some(cmd_queue);
            self.queues[1] = Some(event_queue);
        }
    }
}

impl<D, F> VhostUserDevice for MediaBackend<D, F>
where
    D: VirtioMediaDevice<Reader, Writer> + 'static,
    F: Fn(EventQueue, GuestMemoryMapper, HostMapper, BufferAllocator) -> anyhow::Result<D>
        + Send
        + Sync
        + 'static,
{
    fn max_queue_num(&self) -> usize {
        QUEUE_SIZES.len()
    }

    fn features(&self) -> u64 {
        self.avail_features
    }

    fn protocol_features(&self) -> VhostUserProtocolFeatures {
        // No SHARED_MEMORY_REGIONS and no BACKEND_REQ: with the pool there is nothing to map
        // per buffer and no BAR to declare.
        VhostUserProtocolFeatures::CONFIG
            | VhostUserProtocolFeatures::MQ
            | VhostUserProtocolFeatures::DEVICE_STATE
    }

    fn read_config(&self, offset: u64, data: &mut [u8]) {
        copy_config(data, 0, self.config.as_ref(), offset);
    }

    fn start_queue(&mut self, idx: usize, queue: Queue, mem: GuestMemory) -> anyhow::Result<()> {
        if idx >= QUEUE_SIZES.len() {
            bail!("attempted to start unknown queue: {}", idx);
        }
        if self.running.is_some() {
            warn!("virtio-media: starting queue {idx} without stopping the running worker");
            self.stop();
        }
        if self.queues[idx].is_some() {
            warn!("virtio-media: queue {idx} started twice; the earlier one is dropped");
        }
        self.queues[idx] = Some(queue);
        // The frontend sends the queues one at a time, in an order of its choosing; the worker
        // needs both.
        if self.queues.iter().all(Option::is_some) {
            self.start(mem)?;
        }
        Ok(())
    }

    fn stop_queue(&mut self, idx: usize) -> anyhow::Result<Queue> {
        // Stopping either queue stops the device: the worker cannot run on one.
        self.stop();
        self.queues
            .get_mut(idx)
            .and_then(Option::take)
            .ok_or_else(|| anyhow::Error::new(DeviceError::WorkerNotFound))
    }

    fn reset(&mut self) {
        self.stop();
        self.queues = [None, None];
    }

    fn enter_suspended_state(&mut self) -> anyhow::Result<()> {
        // Called on construction and once every queue is stopped; the worker goes with the
        // queues, so there is nothing left to suspend.
        if self.running.is_some() {
            error!("virtio-media: suspending with the worker still running");
            self.stop();
        }
        Ok(())
    }

    fn snapshot(&mut self) -> anyhow::Result<AnySnapshot> {
        // As the gpu backend: nothing of the device is captured. A media session is a stream in
        // flight, which a snapshot could not resume anyway.
        AnySnapshot::to_any(())
    }

    fn restore(&mut self, data: AnySnapshot) -> anyhow::Result<()> {
        let () = AnySnapshot::from_any(data)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The parameters the VMM writes are the parameters the helper reads: every field survives
    /// the JSON trip, `kind` is spelled as on the command line, the optional fields may be
    /// missing, and a key that is not ours is an error rather than silently ignored.
    #[test]
    fn params_round_trip_through_json() {
        let params = MediaBackendParams {
            kind: MediaDeviceKind::Loopback,
            card: Some("lb0".into()),
            camera_id: None,
            role: Some("main".into()),
            allow_sw: true,
            pool_gpa: 0x1_4000_0000,
            access_windows: vec![(0x1_4000_0000, 0x1000_0000), (0x9000_0000, 0x40_0000)],
            access_platform: true,
        };
        let json = serde_json::to_string(&params).unwrap();
        assert!(json.contains("\"kind\":\"loopback\""), "{json}");
        let back: MediaBackendParams = serde_json::from_str(&json).unwrap();
        assert_eq!(back, params);

        let minimal: MediaBackendParams = serde_json::from_str(
            r#"{"kind":"simple","pool_gpa":4096,"access_windows":[[4096,8192]]}"#,
        )
        .unwrap();
        assert_eq!(minimal.kind, MediaDeviceKind::Simple);
        assert_eq!(
            (minimal.card, minimal.camera_id, minimal.role),
            (None, None, None)
        );
        assert_eq!(minimal.pool_gpa, 4096);
        assert_eq!(minimal.access_windows, vec![(4096, 8192)]);
        assert!(!minimal.access_platform);
        assert!(!minimal.allow_sw);

        assert!(serde_json::from_str::<MediaBackendParams>(
            r#"{"kind":"webcam","pool_gpa":0,"access_windows":[]}"#
        )
        .is_err());
        assert!(serde_json::from_str::<MediaBackendParams>(
            r#"{"kind":"simple","pool_gpa":0,"access_windows":[],"uid":1000}"#
        )
        .is_err());
        assert!(serde_json::from_str::<MediaBackendParams>(r#"{"kind":"simple"}"#).is_err());
    }
}
