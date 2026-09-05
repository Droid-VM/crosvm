// Copyright 2019 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use std::sync::Arc;
use std::sync::Weak;

use anyhow::Context;
use base::debug;
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
use crate::usb::xhci::ring_buffer_controller::StoppedTd;
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

    /// The earliest transfer this stop cancelled is the descriptor in progress: its Stopped
    /// Transfer Event goes out now, ahead of the Command Completion the parked ring releases,
    /// and the ring is left at its first TRB.
    fn finish_stop(&self) -> anyhow::Result<Option<StoppedTd>> {
        let stopped = match self.transfer_manager.take_stopped() {
            Some(stopped) => stopped,
            None => return Ok(None),
        };
        debug!(
            "xhci: slot {} endpoint {} stream {:?}: descriptor at {:#x} stopped at trb {:#x}, {} bytes moved, {} left ({:?})",
            self.slot_id,
            self.endpoint_id,
            self.stream_id,
            stopped.first_trb.0,
            stopped.trb_pointer,
            stopped.bytes,
            stopped.residual,
            stopped.completion_code
        );
        self.interrupter
            .lock()
            .send_transfer_event_trb(
                stopped.completion_code,
                stopped.trb_pointer,
                stopped.residual,
                // ED is clear: the pointer is the TRB in progress, not an Event Data value.
                false,
                self.slot_id,
                self.endpoint_id,
            )
            .context("cannot send stopped transfer event")?;
        Ok(Some(StoppedTd {
            first_trb: stopped.first_trb,
            cycle: stopped.cycle,
            bytes: stopped.bytes,
        }))
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

#[cfg(test)]
mod tests {
    use usb_util::TransferStatus;
    use vm_memory::GuestAddress;

    use super::super::test_util::normal_td;
    use super::super::test_util::Fixture;
    use super::super::test_util::SLOT_ID;
    use super::super::xhci_abi::TrbCompletionCode;
    use super::*;
    use crate::utils::FailHandle;

    /// The stop cancelled two transfers; the ring parks, and its handler sends one Stopped
    /// Transfer Event -- for the descriptor dequeued first, pointing at the TRB in progress with
    /// that TRB's residual, ED clear -- and leaves the ring at that descriptor's first TRB.
    #[test]
    fn finish_stop_reports_the_earliest_cancelled_descriptor_and_leaves_the_ring_at_it() {
        let f = Fixture::new();
        let handler = TransferRingTrbHandler {
            mem: f.mem.clone(),
            port: f.port(),
            interrupter: f.interrupter.clone(),
            slot_id: SLOT_ID,
            endpoint_id: 3,
            transfer_manager: XhciTransferManager::default(),
            stream_id: None,
        };
        assert_eq!(
            handler.finish_stop().unwrap(),
            None,
            "nothing was in flight"
        );

        let first = f.transfer(
            &handler.transfer_manager,
            3,
            normal_td(0x2000, &[0x100, 0x200]),
        );
        let second = f.transfer(&handler.transfer_manager, 3, normal_td(0x2020, &[0x300]));
        // The later descriptor is reaped first.
        second
            .on_transfer_complete(&TransferStatus::Cancelled, 0)
            .unwrap();
        first
            .on_transfer_complete(&TransferStatus::Cancelled, 0x150)
            .unwrap();

        assert_eq!(
            handler.finish_stop().unwrap(),
            Some(StoppedTd {
                first_trb: GuestAddress(0x2000),
                cycle: true,
                bytes: 0x150,
            })
        );
        let events = f.transfer_events();
        assert_eq!(events.len(), 1, "one Stopped event per ring");
        let event = &events[0];
        assert_eq!(
            event.get_completion_code().unwrap(),
            TrbCompletionCode::Stopped
        );
        assert_eq!(event.get_trb_pointer(), 0x2010);
        assert_eq!(event.get_trb_transfer_length(), 0x1b0);
        assert_eq!(event.get_event_data(), 0);
        assert_eq!(event.get_slot_id(), SLOT_ID);
        assert_eq!(event.get_endpoint_id(), 3);

        assert_eq!(handler.finish_stop().unwrap(), None, "reported once");
        assert_eq!(f.transfer_events().len(), 1);
        assert!(!f.fail_handle.failed());
    }
}
