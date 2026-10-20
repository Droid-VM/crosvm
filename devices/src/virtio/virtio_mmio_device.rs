// Copyright 2022 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! The virtio-mmio transport (virtio 1.x "modern" register layout only).
//!
//! Upstream removed this transport in 2024 because nothing could select it. DroidVM brings it back
//! for guests whose drivers only speak virtio-mmio -- UniXNU's NativeVirtioBlock under XNU, which
//! has no PCI host bridge driver for a generic ECAM. The device is a plain MMIO bus device in the
//! VMM process; its interrupt is a level line with resample, the same shape as a virtio-pci INTx,
//! because that is the path the Gunyah irqfd already carries for virtio-pci.

use std::collections::BTreeMap;

use anyhow::anyhow;
use anyhow::Context;
use base::error;
use base::warn;
use base::AsRawDescriptors;
use base::Event;
use base::RawDescriptor;
use base::Result;
use base::WorkerThread;
use hypervisor::Datamatch;
use virtio_sys::virtio_config::VIRTIO_CONFIG_S_ACKNOWLEDGE;
use virtio_sys::virtio_config::VIRTIO_CONFIG_S_DRIVER;
use virtio_sys::virtio_config::VIRTIO_CONFIG_S_DRIVER_OK;
use virtio_sys::virtio_config::VIRTIO_CONFIG_S_FAILED;
use virtio_sys::virtio_config::VIRTIO_CONFIG_S_FEATURES_OK;
use virtio_sys::virtio_config::VIRTIO_CONFIG_S_NEEDS_RESET;
use virtio_sys::virtio_mmio::*;
use vm_memory::GuestAddress;
use vm_memory::GuestMemory;

use super::*;
use crate::pci::CrosvmDeviceId;
use crate::BusAccessInfo;
use crate::BusDevice;
use crate::BusDeviceObj;
use crate::DeviceId;
use crate::IrqLevelEvent;
use crate::Suspendable;

const VIRT_MAGIC: u32 = 0x74726976; /* 'virt' */
const VIRT_VERSION: u8 = 2;
const VIRT_VENDOR: u32 = 0x4D565243; /* 'CRVM' */
/// The register block plus the device configuration space that follows it at 0x100.
pub const VIRTIO_MMIO_REGION_SZ: u64 = 0x200;

/// Implements the
/// [MMIO](http://docs.oasis-open.org/virtio/virtio/v1.0/cs04/virtio-v1.0-cs04.html#x1-1090002)
/// transport for virtio devices.
pub struct VirtioMmioDevice {
    device: Box<dyn VirtioDevice>,
    device_activated: bool,

    interrupt: Option<Interrupt>,
    interrupt_evt: Option<IrqLevelEvent>,
    interrupt_resample_worker: Option<WorkerThread<()>>,
    queues: Vec<QueueConfig>,
    queue_evts: Vec<Event>,
    mem: GuestMemory,
    device_feature_select: u32,
    driver_feature_select: u32,
    queue_select: u16,
    driver_status: u8,
    mmio_base: u64,
    irq_num: u32,
    config_generation: u32,
}

impl VirtioMmioDevice {
    /// Constructs a new MMIO transport for the given virtio device.
    pub fn new(mem: GuestMemory, device: Box<dyn VirtioDevice>) -> Result<Self> {
        let mut queue_evts = Vec::new();
        for _ in device.queue_max_sizes() {
            queue_evts.push(Event::new()?)
        }
        let queues = device
            .queue_max_sizes()
            .iter()
            .map(|&s| QueueConfig::new(s, device.features()))
            .collect();

        Ok(VirtioMmioDevice {
            device,
            device_activated: false,
            interrupt: None,
            interrupt_evt: None,
            interrupt_resample_worker: None,
            queues,
            queue_evts,
            mem,
            device_feature_select: 0,
            driver_feature_select: 0,
            queue_select: 0,
            driver_status: 0,
            mmio_base: 0,
            irq_num: 0,
            config_generation: 0,
        })
    }

    /// The queue-notify ioevents: one per queue, matching the queue index written to QueueNotify.
    /// Valid once `set_mmio_base` has placed the device.
    pub fn ioevents(&self) -> Vec<(&Event, u64, Datamatch)> {
        self.queue_evts
            .iter()
            .enumerate()
            .map(|(i, event)| {
                (
                    event,
                    self.mmio_base + VIRTIO_MMIO_QUEUE_NOTIFY as u64,
                    Datamatch::U32(Some(i.try_into().unwrap())),
                )
            })
            .collect()
    }

    fn is_driver_ready(&self) -> bool {
        let ready_bits = (VIRTIO_CONFIG_S_ACKNOWLEDGE
            | VIRTIO_CONFIG_S_DRIVER
            | VIRTIO_CONFIG_S_DRIVER_OK
            | VIRTIO_CONFIG_S_FEATURES_OK) as u8;
        self.driver_status == ready_bits && self.driver_status & VIRTIO_CONFIG_S_FAILED as u8 == 0
    }

    /// Determines if the driver has requested the device reset itself
    fn is_reset_requested(&self) -> bool {
        self.driver_status == DEVICE_RESET as u8
    }

    fn device_type(&self) -> u32 {
        self.device.device_type() as u32
    }

    /// Activates the underlying `VirtioDevice`. `assign_irq` has to be called first.
    fn activate(&mut self) -> anyhow::Result<()> {
        let interrupt_evt = if let Some(ref evt) = self.interrupt_evt {
            evt.try_clone()
                .with_context(|| format!("{} failed to clone interrupt_evt", self.debug_label()))?
        } else {
            return Err(anyhow!("{} interrupt_evt is none", self.debug_label()));
        };

        let mem = self.mem.clone();
        // No MSI-X: a level line whose resample re-asserts while the interrupt status is non-zero,
        // exactly a virtio-pci device on INTx.
        let interrupt = Interrupt::new(
            interrupt_evt,
            None,
            VIRTIO_MSI_NO_VECTOR,
            #[cfg(target_arch = "x86_64")]
            None,
        );
        self.interrupt_resample_worker = interrupt.spawn_resample_thread();
        self.interrupt = Some(interrupt.clone());

        // Use ready queues and their events.
        let queues = self
            .queues
            .iter_mut()
            .zip(self.queue_evts.iter())
            .enumerate()
            .filter(|(_, (q, _))| q.ready())
            .map(|(queue_index, (queue, evt))| {
                let queue_evt = evt.try_clone().context("failed to clone queue_evt")?;
                Ok((
                    queue_index,
                    queue
                        .activate(&mem, queue_evt, interrupt.clone())
                        .context("failed to activate queue")?,
                ))
            })
            .collect::<anyhow::Result<BTreeMap<usize, Queue>>>()?;

        if let Err(e) = self.device.activate(mem, interrupt, queues) {
            error!("{} activate failed: {:#}", self.debug_label(), e);
            self.driver_status |= VIRTIO_CONFIG_S_NEEDS_RESET as u8;
        } else {
            self.device_activated = true;
        }

        Ok(())
    }

    fn read_mmio(&self, info: BusAccessInfo, data: &mut [u8]) {
        // The device configuration space takes accesses of any width.
        if info.offset >= VIRTIO_MMIO_CONFIG as u64 {
            self.device
                .read_config(info.offset - VIRTIO_MMIO_CONFIG as u64, data);
            return;
        }

        if data.len() != std::mem::size_of::<u32>() {
            warn!(
                "{}: unsupported read length {}, only support 4 bytes read",
                self.debug_label(),
                data.len()
            );
            return;
        }

        let val = match info.offset as u32 {
            VIRTIO_MMIO_MAGIC_VALUE => VIRT_MAGIC,
            VIRTIO_MMIO_VERSION => VIRT_VERSION.into(), // legacy is not supported
            VIRTIO_MMIO_DEVICE_ID => self.device_type(),
            VIRTIO_MMIO_VENDOR_ID => VIRT_VENDOR,
            VIRTIO_MMIO_DEVICE_FEATURES => {
                if self.device_feature_select < 2 {
                    (self.device.features() >> (self.device_feature_select * 32)) as u32
                } else {
                    0
                }
            }
            VIRTIO_MMIO_QUEUE_NUM_MAX => self.with_queue(|q| q.max_size()).unwrap_or(0).into(),
            VIRTIO_MMIO_QUEUE_PFN => {
                warn!(
                    "{}: read from legacy register {}, in non-legacy mode",
                    self.debug_label(),
                    info.offset,
                );
                0
            }
            VIRTIO_MMIO_QUEUE_READY => self.with_queue(|q| q.ready()).unwrap_or(false).into(),
            VIRTIO_MMIO_INTERRUPT_STATUS => {
                if let Some(interrupt) = &self.interrupt {
                    interrupt.read_interrupt_status().into()
                } else {
                    0
                }
            }
            VIRTIO_MMIO_STATUS => self.driver_status.into(),
            VIRTIO_MMIO_CONFIG_GENERATION => self.config_generation,
            // No shared memory regions: a selected region has length -1 per the spec.
            VIRTIO_MMIO_SHM_LEN_LOW | VIRTIO_MMIO_SHM_LEN_HIGH => u32::MAX,
            VIRTIO_MMIO_SHM_BASE_LOW | VIRTIO_MMIO_SHM_BASE_HIGH => 0,
            _ => {
                warn!("{}: unsupported read address {}", self.debug_label(), info);
                return;
            }
        };

        let val_arr = val.to_le_bytes();
        data.copy_from_slice(&val_arr);
    }

    fn write_mmio(&mut self, info: BusAccessInfo, data: &[u8]) {
        if info.offset >= VIRTIO_MMIO_CONFIG as u64 {
            self.device
                .write_config(info.offset - VIRTIO_MMIO_CONFIG as u64, data);
            return;
        }

        if data.len() != std::mem::size_of::<u32>() {
            warn!(
                "{}: unsupported write length {}, only support 4 bytes write",
                self.debug_label(),
                data.len()
            );
            return;
        }

        // This unwrap cannot fail since data.len() is checked.
        let val = u32::from_le_bytes(data.try_into().unwrap());

        macro_rules! hi {
            ($q:expr, $get:ident, $set:ident, $x:expr) => {
                $q.$set(GuestAddress(
                    ($q.$get().offset() & 0xffffffff) | (($x as u64) << 32),
                ))
            };
        }
        macro_rules! lo {
            ($q:expr, $get:ident, $set:ident, $x:expr) => {
                $q.$set(GuestAddress(
                    ($q.$get().offset() & !0xffffffff) | ($x as u64),
                ))
            };
        }

        match info.offset as u32 {
            VIRTIO_MMIO_DEVICE_FEATURES_SEL => self.device_feature_select = val,
            VIRTIO_MMIO_DRIVER_FEATURES_SEL => self.driver_feature_select = val,
            VIRTIO_MMIO_DRIVER_FEATURES => {
                if self.driver_feature_select < 2 {
                    let features: u64 = (val as u64) << (self.driver_feature_select * 32);
                    self.device.ack_features(features);
                    for queue in self.queues.iter_mut() {
                        queue.ack_features(features);
                    }
                } else {
                    warn!(
                        "invalid ack_features (page {}, value 0x{:x})",
                        self.driver_feature_select, val
                    );
                }
            }
            VIRTIO_MMIO_GUEST_PAGE_SIZE | VIRTIO_MMIO_QUEUE_ALIGN | VIRTIO_MMIO_QUEUE_PFN => warn!(
                "{}: write to legacy register {}, in non-legacy mode",
                self.debug_label(),
                info.offset,
            ),
            VIRTIO_MMIO_QUEUE_SEL => self.queue_select = val as u16,
            VIRTIO_MMIO_QUEUE_NUM => self.with_queue_mut(|q| q.set_size(val as u16)),
            VIRTIO_MMIO_QUEUE_READY => self.with_queue_mut(|q| q.set_ready(val == 1)),
            // Normally consumed by the ioevent before it reaches the bus; deliver it here too, so
            // a hypervisor without (datamatch) ioeventfd still works.
            VIRTIO_MMIO_QUEUE_NOTIFY => {
                if let Some(evt) = self.queue_evts.get(val as usize) {
                    if let Err(e) = evt.signal() {
                        error!("{}: queue notify failed: {}", self.debug_label(), e);
                    }
                }
            }
            VIRTIO_MMIO_INTERRUPT_ACK => {
                if let Some(interrupt) = &self.interrupt {
                    interrupt.clear_interrupt_status_bits(val as u8)
                }
            }
            VIRTIO_MMIO_STATUS => self.driver_status = val as u8,
            VIRTIO_MMIO_QUEUE_DESC_LOW => {
                self.with_queue_mut(|q| lo!(q, desc_table, set_desc_table, val))
            }
            VIRTIO_MMIO_QUEUE_DESC_HIGH => {
                self.with_queue_mut(|q| hi!(q, desc_table, set_desc_table, val))
            }
            VIRTIO_MMIO_QUEUE_AVAIL_LOW => {
                self.with_queue_mut(|q| lo!(q, avail_ring, set_avail_ring, val))
            }
            VIRTIO_MMIO_QUEUE_AVAIL_HIGH => {
                self.with_queue_mut(|q| hi!(q, avail_ring, set_avail_ring, val))
            }
            VIRTIO_MMIO_QUEUE_USED_LOW => {
                self.with_queue_mut(|q| lo!(q, used_ring, set_used_ring, val))
            }
            VIRTIO_MMIO_QUEUE_USED_HIGH => {
                self.with_queue_mut(|q| hi!(q, used_ring, set_used_ring, val))
            }
            VIRTIO_MMIO_SHM_SEL => {}
            _ => {
                warn!("{}: unsupported write address {}", self.debug_label(), info);
                return;
            }
        };

        if !self.device_activated && self.is_driver_ready() {
            if let Err(e) = self.activate() {
                error!("failed to activate device: {:#}", e);
            }
        }

        // Device has been reset by the driver
        if self.is_reset_requested() {
            if self.device_activated {
                if let Err(e) = self.device.reset() {
                    error!("failed to reset {} device: {:#}", self.debug_label(), e);
                    return;
                }
            }
            self.device_activated = false;
            // reset queues
            self.queues.iter_mut().for_each(QueueConfig::reset);
            // select queue 0 by default
            self.queue_select = 0;
            // reset interrupt
            self.interrupt = None;
            self.interrupt_resample_worker = None;
        }
    }

    fn with_queue<U, F>(&self, f: F) -> Option<U>
    where
        F: FnOnce(&QueueConfig) -> U,
    {
        self.queues.get(self.queue_select as usize).map(f)
    }

    fn with_queue_mut<F>(&mut self, f: F)
    where
        F: FnOnce(&mut QueueConfig),
    {
        if let Some(queue) = self.queues.get_mut(self.queue_select as usize) {
            f(queue);
        }
    }

    /// Places the device: the register block starts at `base` and spans `VIRTIO_MMIO_REGION_SZ`.
    pub fn set_mmio_base(&mut self, base: u64) {
        self.mmio_base = base;
    }

    pub fn mmio_base(&self) -> u64 {
        self.mmio_base
    }

    pub fn assign_irq(&mut self, irq_evt: &IrqLevelEvent, irq_num: u32) {
        self.interrupt_evt = Some(irq_evt.try_clone().unwrap());
        self.irq_num = irq_num;
    }

    pub fn irq_num(&self) -> u32 {
        self.irq_num
    }

    pub fn keep_rds(&self) -> Vec<RawDescriptor> {
        let mut rds = self.device.keep_rds();
        if let Some(interrupt_evt) = &self.interrupt_evt {
            rds.extend(interrupt_evt.as_raw_descriptors());
        }
        rds
    }
}

impl BusDeviceObj for VirtioMmioDevice {
    fn as_virtio_mmio_device(&self) -> Option<&VirtioMmioDevice> {
        Some(self)
    }
    fn as_virtio_mmio_device_mut(&mut self) -> Option<&mut VirtioMmioDevice> {
        Some(self)
    }
    fn into_virtio_mmio_device(self: Box<Self>) -> Option<Box<VirtioMmioDevice>> {
        Some(self)
    }
}

impl BusDevice for VirtioMmioDevice {
    fn debug_label(&self) -> String {
        format!("mmio{}", self.device.debug_label())
    }

    fn device_id(&self) -> DeviceId {
        CrosvmDeviceId::VirtioMmio.into()
    }

    fn read(&mut self, info: BusAccessInfo, data: &mut [u8]) {
        self.read_mmio(info, data)
    }

    fn write(&mut self, info: BusAccessInfo, data: &[u8]) {
        self.write_mmio(info, data)
    }

    fn on_sandboxed(&mut self) {
        self.device.on_device_sandboxed();
    }
}

// Snapshot/restore is not supported on this transport (the trait defaults refuse it).
impl Suspendable for VirtioMmioDevice {}
