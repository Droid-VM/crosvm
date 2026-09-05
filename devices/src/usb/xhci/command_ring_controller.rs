// Copyright 2019 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#![allow(clippy::result_large_err)]

use std::sync::Arc;

use anyhow::Context;
use base::error;
use base::warn;
use base::Error as SysError;
use base::Event;
use remain::sorted;
use sync::Mutex;
use thiserror::Error;
use vm_memory::GuestAddress;
use vm_memory::GuestMemory;

use super::device_slot::DeviceSlot;
use super::device_slot::DeviceSlots;
use super::device_slot::Error as DeviceSlotError;
use super::interrupter::Error as InterrupterError;
use super::interrupter::Interrupter;
use super::ring_buffer_controller::Error as RingBufferControllerError;
use super::ring_buffer_controller::RingBufferController;
use super::ring_buffer_controller::TransferDescriptorHandler;
use super::xhci_abi::AddressDeviceCommandTrb;
use super::xhci_abi::AddressedTrb;
use super::xhci_abi::ConfigureEndpointCommandTrb;
use super::xhci_abi::DisableSlotCommandTrb;
use super::xhci_abi::Error as TrbError;
use super::xhci_abi::EvaluateContextCommandTrb;
use super::xhci_abi::ResetDeviceCommandTrb;
use super::xhci_abi::ResetEndpointCommandTrb;
use super::xhci_abi::SetTRDequeuePointerCommandTrb;
use super::xhci_abi::StopEndpointCommandTrb;
use super::xhci_abi::TransferDescriptor;
use super::xhci_abi::TrbCast;
use super::xhci_abi::TrbCompletionCode;
use super::xhci_abi::TrbType;
use super::xhci_regs::valid_slot_id;
use super::xhci_regs::MAX_SLOTS;
use crate::utils::EventLoop;

#[sorted]
#[derive(Error, Debug)]
pub enum Error {
    #[error("bad slot id: {0}")]
    BadSlotId(u8),
    #[error("failed to cast trb: {0}")]
    CastTrb(TrbError),
    #[error("failed to config endpoint: {0}")]
    ConfigEndpoint(DeviceSlotError),
    #[error("failed to disable slot: {0}")]
    DisableSlot(DeviceSlotError),
    #[error("failed to evaluate context: {0}")]
    EvaluateContext(DeviceSlotError),
    #[error("failed to reset slot: {0}")]
    ResetSlot(DeviceSlotError),
    #[error("failed to send interrupt: {0}")]
    SendInterrupt(InterrupterError),
    #[error("failed to set address: {0}")]
    SetAddress(DeviceSlotError),
    #[error("failed to set dequeue pointer: {0}")]
    SetDequeuePointer(DeviceSlotError),
    #[error("failed to stop endpoint: {0}")]
    StopEndpoint(DeviceSlotError),
    #[error("failed to write event: {0}")]
    WriteEvent(SysError),
}

type Result<T> = std::result::Result<T, Error>;

pub type CommandRingController = RingBufferController<CommandRingTrbHandler>;
pub type CommandRingControllerError = RingBufferControllerError;

impl CommandRingController {
    pub fn new(
        mem: GuestMemory,
        event_loop: Arc<EventLoop>,
        slots: DeviceSlots,
        interrupter: Arc<Mutex<Interrupter>>,
    ) -> std::result::Result<Arc<CommandRingController>, RingBufferControllerError> {
        RingBufferController::new_with_handler(
            String::from("command ring"),
            mem,
            event_loop,
            CommandRingTrbHandler::new(slots, interrupter),
        )
    }
}

pub struct CommandRingTrbHandler {
    slots: DeviceSlots,
    interrupter: Arc<Mutex<Interrupter>>,
}

impl CommandRingTrbHandler {
    fn new(slots: DeviceSlots, interrupter: Arc<Mutex<Interrupter>>) -> Self {
        CommandRingTrbHandler { slots, interrupter }
    }

    fn slot(&self, slot_id: u8) -> Result<Arc<DeviceSlot>> {
        self.slots.slot(slot_id).ok_or(Error::BadSlotId(slot_id))
    }

    fn command_completion_callback(
        interrupter: &Arc<Mutex<Interrupter>>,
        completion_code: TrbCompletionCode,
        slot_id: u8,
        trb_addr: u64,
        event: &Event,
    ) -> Result<()> {
        interrupter
            .lock()
            .send_command_completion_trb(completion_code, slot_id, GuestAddress(trb_addr))
            .map_err(Error::SendInterrupt)?;
        event.signal().map_err(Error::WriteEvent)
    }

    fn enable_slot(&self, atrb: &AddressedTrb, event: Event) -> Result<()> {
        for slot_id in 1..=MAX_SLOTS {
            if self.slot(slot_id)?.enable() {
                return CommandRingTrbHandler::command_completion_callback(
                    &self.interrupter,
                    TrbCompletionCode::Success,
                    slot_id,
                    atrb.gpa,
                    &event,
                );
            }
        }

        CommandRingTrbHandler::command_completion_callback(
            &self.interrupter,
            TrbCompletionCode::NoSlotsAvailableError,
            0,
            atrb.gpa,
            &event,
        )
    }

    fn disable_slot(&self, atrb: &AddressedTrb, event: Event) -> Result<()> {
        let trb = atrb
            .trb
            .cast::<DisableSlotCommandTrb>()
            .map_err(Error::CastTrb)?;
        let slot_id = trb.get_slot_id();
        if valid_slot_id(slot_id) {
            let gpa = atrb.gpa;
            let interrupter = self.interrupter.clone();
            self.slots
                .disable_slot(slot_id, move |completion_code| {
                    CommandRingTrbHandler::command_completion_callback(
                        &interrupter,
                        completion_code,
                        slot_id,
                        gpa,
                        &event,
                    )
                    .map_err(|e| {
                        error!("failed to run command completion callback: {}", e);
                    })
                })
                .map_err(Error::DisableSlot)
        } else {
            CommandRingTrbHandler::command_completion_callback(
                &self.interrupter,
                TrbCompletionCode::TrbError,
                slot_id,
                atrb.gpa,
                &event,
            )
        }
    }

    fn address_device(&self, atrb: &AddressedTrb, event: Event) -> Result<()> {
        let trb = atrb
            .trb
            .cast::<AddressDeviceCommandTrb>()
            .map_err(Error::CastTrb)?;
        let slot_id = trb.get_slot_id();
        let completion_code = {
            if valid_slot_id(slot_id) {
                self.slot(slot_id)?
                    .set_address(trb)
                    .map_err(Error::SetAddress)?
            } else {
                TrbCompletionCode::TrbError
            }
        };
        CommandRingTrbHandler::command_completion_callback(
            &self.interrupter,
            completion_code,
            slot_id,
            atrb.gpa,
            &event,
        )
    }

    fn configure_endpoint(&self, atrb: &AddressedTrb, event: Event) -> Result<()> {
        let trb = atrb
            .trb
            .cast::<ConfigureEndpointCommandTrb>()
            .map_err(Error::CastTrb)?;
        let slot_id = trb.get_slot_id();
        let completion_code = {
            if valid_slot_id(slot_id) {
                self.slot(slot_id)?
                    .configure_endpoint(trb)
                    .map_err(Error::ConfigEndpoint)?
            } else {
                TrbCompletionCode::TrbError
            }
        };
        CommandRingTrbHandler::command_completion_callback(
            &self.interrupter,
            completion_code,
            slot_id,
            atrb.gpa,
            &event,
        )
    }

    fn evaluate_context(&self, atrb: &AddressedTrb, event: Event) -> Result<()> {
        let trb = atrb
            .trb
            .cast::<EvaluateContextCommandTrb>()
            .map_err(Error::CastTrb)?;
        let slot_id = trb.get_slot_id();
        let completion_code = {
            if valid_slot_id(slot_id) {
                self.slot(slot_id)?
                    .evaluate_context(trb)
                    .map_err(Error::EvaluateContext)?
            } else {
                TrbCompletionCode::TrbError
            }
        };
        CommandRingTrbHandler::command_completion_callback(
            &self.interrupter,
            completion_code,
            slot_id,
            atrb.gpa,
            &event,
        )
    }

    fn reset_device(&self, atrb: &AddressedTrb, event: Event) -> Result<()> {
        let trb = atrb
            .trb
            .cast::<ResetDeviceCommandTrb>()
            .map_err(Error::CastTrb)?;
        let slot_id = trb.get_slot_id();
        if valid_slot_id(slot_id) {
            let gpa = atrb.gpa;
            let interrupter = self.interrupter.clone();
            self.slots
                .reset_slot(slot_id, move |completion_code| {
                    CommandRingTrbHandler::command_completion_callback(
                        &interrupter,
                        completion_code,
                        slot_id,
                        gpa,
                        &event,
                    )
                    .map_err(|e| {
                        error!("command completion callback failed: {}", e);
                    })
                })
                .map_err(Error::ResetSlot)
        } else {
            CommandRingTrbHandler::command_completion_callback(
                &self.interrupter,
                TrbCompletionCode::TrbError,
                slot_id,
                atrb.gpa,
                &event,
            )
        }
    }

    fn stop_endpoint(&self, atrb: &AddressedTrb, event: Event) -> Result<()> {
        let trb = atrb
            .trb
            .cast::<StopEndpointCommandTrb>()
            .map_err(Error::CastTrb)?;
        let slot_id = trb.get_slot_id();
        let endpoint_id = trb.get_endpoint_id();
        if valid_slot_id(slot_id) {
            let gpa = atrb.gpa;
            let interrupter = self.interrupter.clone();
            self.slots
                .stop_endpoint(slot_id, endpoint_id, move |completion_code| {
                    CommandRingTrbHandler::command_completion_callback(
                        &interrupter,
                        completion_code,
                        slot_id,
                        gpa,
                        &event,
                    )
                    .map_err(|e| {
                        error!("command completion callback failed: {}", e);
                    })
                })
                .map_err(Error::StopEndpoint)?;
            Ok(())
        } else {
            error!("stop endpoint trb has invalid slot id {}", slot_id);
            CommandRingTrbHandler::command_completion_callback(
                &self.interrupter,
                TrbCompletionCode::TrbError,
                slot_id,
                atrb.gpa,
                &event,
            )
        }
    }

    fn reset_endpoint(&self, atrb: &AddressedTrb, event: Event) -> Result<()> {
        let trb = atrb
            .trb
            .cast::<ResetEndpointCommandTrb>()
            .map_err(Error::CastTrb)?;
        let slot_id = trb.get_slot_id();
        let endpoint_id = trb.get_endpoint_id();
        if valid_slot_id(slot_id) {
            let gpa = atrb.gpa;
            let interrupter = self.interrupter.clone();
            self.slots
                .reset_endpoint(slot_id, endpoint_id, move |completion_code| {
                    CommandRingTrbHandler::command_completion_callback(
                        &interrupter,
                        completion_code,
                        slot_id,
                        gpa,
                        &event,
                    )
                    .map_err(|e| {
                        error!("command completion callback failed: {}", e);
                    })
                })
                .map_err(Error::StopEndpoint)?;
            Ok(())
        } else {
            error!("reset endpoint trb has invalid slot id {}", slot_id);
            CommandRingTrbHandler::command_completion_callback(
                &self.interrupter,
                TrbCompletionCode::TrbError,
                slot_id,
                atrb.gpa,
                &event,
            )
        }
    }

    fn set_tr_dequeue_ptr(&self, atrb: &AddressedTrb, event: Event) -> Result<()> {
        let trb = atrb
            .trb
            .cast::<SetTRDequeuePointerCommandTrb>()
            .map_err(Error::CastTrb)?;
        let slot_id = trb.get_slot_id();
        let endpoint_id = trb.get_endpoint_id();
        let stream_id = trb.get_stream_id();
        let stream_context_type = trb.get_stream_context_type();
        // See Set TR Dequeue Pointer Trb in spec.
        let dequeue_ptr = trb.get_dequeue_ptr().get_gpa().offset();
        let dequeue_cycle_state = trb.get_dequeue_cycle_state();
        let completion_code = {
            if valid_slot_id(slot_id) {
                self.slot(slot_id)?
                    .set_tr_dequeue_ptr(
                        endpoint_id,
                        stream_id,
                        stream_context_type,
                        dequeue_ptr,
                        dequeue_cycle_state,
                    )
                    .map_err(Error::SetDequeuePointer)?
            } else {
                error!("stop endpoint trb has invalid slot id {}", slot_id);
                TrbCompletionCode::TrbError
            }
        };
        CommandRingTrbHandler::command_completion_callback(
            &self.interrupter,
            completion_code,
            slot_id,
            atrb.gpa,
            &event,
        )
    }
}

impl TransferDescriptorHandler for CommandRingTrbHandler {
    fn handle_transfer_descriptor(
        &self,
        descriptor: TransferDescriptor,
        complete_event: Event,
    ) -> anyhow::Result<()> {
        // Command descriptor always consist of a single TRB.
        assert_eq!(descriptor.len(), 1);
        let atrb = &descriptor[0];
        let command_result = match atrb.trb.get_trb_type() {
            Ok(TrbType::EnableSlotCommand) => self.enable_slot(atrb, complete_event),
            Ok(TrbType::DisableSlotCommand) => self.disable_slot(atrb, complete_event),
            Ok(TrbType::AddressDeviceCommand) => self.address_device(atrb, complete_event),
            Ok(TrbType::ConfigureEndpointCommand) => self.configure_endpoint(atrb, complete_event),
            Ok(TrbType::EvaluateContextCommand) => self.evaluate_context(atrb, complete_event),
            Ok(TrbType::ResetDeviceCommand) => self.reset_device(atrb, complete_event),
            Ok(TrbType::NoopCommand) => CommandRingTrbHandler::command_completion_callback(
                &self.interrupter,
                TrbCompletionCode::Success,
                0,
                atrb.gpa,
                &complete_event,
            ),
            Ok(TrbType::ResetEndpointCommand) => self.reset_endpoint(atrb, complete_event),
            Ok(TrbType::StopEndpointCommand) => self.stop_endpoint(atrb, complete_event),
            Ok(TrbType::SetTRDequeuePointerCommand) => {
                self.set_tr_dequeue_ptr(atrb, complete_event)
            }
            _ => {
                warn!(
                    // We are not handling type 14,15,16. See table 6.4.6.
                    "Unexpected command ring trb type: {}",
                    atrb.trb
                );
                match self.interrupter.lock().send_command_completion_trb(
                    TrbCompletionCode::TrbError,
                    0,
                    GuestAddress(atrb.gpa),
                ) {
                    Err(e) => Err(Error::SendInterrupt(e)),
                    Ok(_) => complete_event.signal().map_err(Error::WriteEvent),
                }
            }
        };
        command_result.context("command ring TRB failed")
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use base::EventWaitResult;

    use super::super::device_slot::test_util::*;
    use super::super::xhci_abi::DeviceSlotState;
    use super::super::xhci_abi::Trb;
    use super::*;
    use crate::utils::FailHandle;

    fn signaled(event: &Event) -> bool {
        matches!(
            event.wait_timeout(Duration::from_millis(200)).unwrap(),
            EventWaitResult::Signaled
        )
    }

    /// Runs one command through the handler the way the command ring does and returns the
    /// event it was handed to signal when done.
    fn run(f: &Fixture, trb: Trb) -> Event {
        let handler = CommandRingTrbHandler::new(f.slots.clone(), f.interrupter.clone());
        let complete = Event::new().unwrap();
        handler
            .handle_transfer_descriptor(
                vec![AddressedTrb {
                    trb,
                    gpa: COMMAND_TRB,
                }],
                complete.try_clone().unwrap(),
            )
            .unwrap();
        complete
    }

    #[test]
    fn a_rejected_command_still_completes() {
        // Windows' loop: a command that came back as an error from the handler was never
        // answered, the command ring's event handler was dropped, and every later command
        // timed out. A guest mistake is a completion code, and the ring must go on.
        let f = Fixture::new();
        let mut ctx = f.device_context();
        ctx.slot_context.set_slot_state(DeviceSlotState::Default);
        f.set_device_context(ctx);
        f.write_stream_endpoint_input_context(3, 3);
        f.write_stream_context_array(1..16);

        let complete = run(&f, f.configure_endpoint_command());
        assert!(signaled(&complete), "the command ring must be released");
        assert!(signaled(&f.irq));
        let completions = f.command_completions();
        assert_eq!(completions.len(), 1);
        assert_eq!(
            completions[0].get_completion_code().unwrap(),
            TrbCompletionCode::ContextStateError
        );
        assert_eq!(completions[0].get_trb_pointer(), COMMAND_TRB);
        assert_eq!(completions[0].get_slot_id(), SLOT_ID);
        assert!(!f.fail_handle.failed());
    }

    #[test]
    fn configure_endpoint_with_not_valid_streams_returns_success_and_signals_event() {
        let f = Fixture::new();
        f.write_stream_endpoint_input_context(3, 3);
        f.write_stream_context_array(1..9);

        let complete = run(&f, f.configure_endpoint_command());
        assert!(signaled(&complete));
        assert!(signaled(&f.irq));
        let completions = f.command_completions();
        assert_eq!(completions.len(), 1);
        assert_eq!(
            completions[0].get_completion_code().unwrap(),
            TrbCompletionCode::Success
        );
        assert_eq!(completions[0].get_trb_pointer(), COMMAND_TRB);
        assert!(!f.fail_handle.failed());
    }
}
