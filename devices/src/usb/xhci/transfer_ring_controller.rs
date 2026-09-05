// Copyright 2019 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use std::sync::Arc;
use std::sync::Weak;

use anyhow::Context;
use base::Event;
use sync::Mutex;
use vm_memory::GuestMemory;

use super::device_slot::DeviceSlot;
use super::interrupter::Interrupter;
use super::usb_hub::UsbPort;
use super::xhci_abi::TransferDescriptor;
use super::xhci_transfer::XhciTransferManager;
use crate::usb::xhci::ring_buffer_controller::Error as RingBufferControllerError;
use crate::usb::xhci::ring_buffer_controller::RingBufferController;
use crate::usb::xhci::ring_buffer_controller::TransferDescriptorHandler;
use crate::utils::EventLoop;

/// Transfer ring controller manages transfer ring.
pub type TransferRingController = RingBufferController<TransferRingTrbHandler>;

#[derive(Clone)]
pub enum TransferRingControllers {
    Endpoint(Arc<TransferRingController>),
    /// One entry per primary Stream Context, index = stream id - 1, length
    /// 2^(MaxPStreams+1) - 1. `None` is a Not Valid Stream Context (spec 6.2.4.1, Table 6-13):
    /// the guest only initialises the streams it opened, and a doorbell on any other is an
    /// error at use, not at Configure Endpoint.
    Stream(Vec<Option<Arc<TransferRingController>>>),
}

pub type TransferRingControllerError = RingBufferControllerError;

/// TransferRingTrbHandler handles trbs on transfer ring.
pub struct TransferRingTrbHandler {
    mem: GuestMemory,
    port: Arc<UsbPort>,
    interrupter: Arc<Mutex<Interrupter>>,
    slot_id: u8,
    endpoint_id: u8,
    transfer_manager: XhciTransferManager,
    stream_id: Option<u16>,
}

impl TransferDescriptorHandler for TransferRingTrbHandler {
    fn handle_transfer_descriptor(
        &self,
        descriptor: TransferDescriptor,
        completion_event: Event,
    ) -> anyhow::Result<()> {
        let xhci_transfer = self.transfer_manager.create_transfer(
            self.mem.clone(),
            self.port.clone(),
            self.interrupter.clone(),
            self.slot_id,
            self.endpoint_id,
            descriptor,
            completion_event,
            self.stream_id,
        );
        xhci_transfer
            .send_to_backend_if_valid()
            .context("failed to send transfer to backend")
    }

    fn stop(&self) -> bool {
        let backend = self.port.backend_device();
        if backend.is_some() {
            self.transfer_manager.cancel_all();
            true
        } else {
            false
        }
    }

    fn is_quiesced(&self) -> bool {
        !self.transfer_manager.has_pending_transfers()
    }
}

impl TransferRingController {
    pub fn new(
        mem: GuestMemory,
        port: Arc<UsbPort>,
        event_loop: Arc<EventLoop>,
        interrupter: Arc<Mutex<Interrupter>>,
        slot_id: u8,
        endpoint_id: u8,
        device_slot: Weak<DeviceSlot>,
        stream_id: Option<u16>,
    ) -> Result<Arc<TransferRingController>, TransferRingControllerError> {
        let name = match stream_id {
            Some(stream_id) => format!(
                "transfer ring slot_{} ep_{} stream_{}",
                slot_id, endpoint_id, stream_id
            ),
            None => format!("transfer ring slot_{} ep_{}", slot_id, endpoint_id),
        };
        RingBufferController::new_with_handler(
            name,
            mem.clone(),
            event_loop,
            TransferRingTrbHandler {
                mem,
                port,
                interrupter,
                slot_id,
                endpoint_id,
                transfer_manager: XhciTransferManager::new(device_slot),
                stream_id,
            },
        )
    }
}
