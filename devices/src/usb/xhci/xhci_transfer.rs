// Copyright 2019 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use std::cmp::min;
use std::fmt;
use std::fmt::Display;
use std::mem;
use std::sync::atomic::AtomicU64;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::sync::Weak;

use base::debug;
use base::error;
use base::info;
use base::warn;
use base::Error as SysError;
use base::Event;
use bit_field::Error as BitFieldError;
use remain::sorted;
use sync::Mutex;
use thiserror::Error;
use usb_util::TransferStatus;
use usb_util::UsbRequestSetup;
use vm_memory::GuestAddress;
use vm_memory::GuestMemory;
use vm_memory::GuestMemoryError;

use super::device_slot::DeviceSlot;
use super::interrupter::Error as InterrupterError;
use super::interrupter::Interrupter;
use super::scatter_gather_buffer::Error as BufferError;
use super::scatter_gather_buffer::ScatterGatherBuffer;
use super::usb_hub::Error as HubError;
use super::usb_hub::UsbPort;
use super::xhci_abi::AddressedTrb;
use super::xhci_abi::Error as TrbError;
use super::xhci_abi::EventDataTrb;
use super::xhci_abi::SetupStageTrb;
use super::xhci_abi::TransferDescriptor;
use super::xhci_abi::TrbCast;
use super::xhci_abi::TrbCompletionCode;
use super::xhci_abi::TrbType;
use super::xhci_regs::MAX_INTERRUPTER;

#[sorted]
#[derive(Error, Debug)]
pub enum Error {
    #[error("unexpected trb type: {0:?}")]
    BadTrbType(TrbType),
    #[error("cannot cast trb: {0}")]
    CastTrb(TrbError),
    #[error("cannot create transfer buffer: {0}")]
    CreateBuffer(BufferError),
    #[error("cannot detach from port: {0}")]
    DetachPort(HubError),
    #[error("failed to halt the endpoint: {0}")]
    HaltEndpoint(u8),
    #[error("failed to read guest memory: {0}")]
    ReadGuestMemory(GuestMemoryError),
    #[error("cannot send interrupt: {0}")]
    SendInterrupt(InterrupterError),
    #[error("failed to submit transfer to backend")]
    SubmitTransfer,
    #[error("cannot get transfer length: {0}")]
    TransferLength(TrbError),
    #[error("cannot get trb type: {0}")]
    TrbType(BitFieldError),
    #[error("cannot write completion event: {0}")]
    WriteCompletionEvent(SysError),
    #[error("failed to write guest memory: {0}")]
    WriteGuestMemory(GuestMemoryError),
}

type Result<T> = std::result::Result<T, Error>;

/// Type of usb endpoints.
#[derive(PartialEq, Eq, Clone, Copy, Debug)]
pub enum TransferDirection {
    In,
    Out,
    Control,
}

/// Current state of xhci transfer.
pub enum XhciTransferState {
    Created,
    /// When transfer is submitted, it will contain a transfer callback, which should be invoked
    /// when the transfer is cancelled.
    Submitted {
        cancel_callback: Box<dyn FnOnce() + Send>,
    },
    Cancelling,
    Cancelled,
    Completed,
}

impl XhciTransferState {
    /// Try to cancel this transfer, if it's possible.
    pub fn try_cancel(&mut self) {
        match mem::replace(self, XhciTransferState::Created) {
            XhciTransferState::Submitted { cancel_callback } => {
                *self = XhciTransferState::Cancelling;
                cancel_callback();
            }
            XhciTransferState::Cancelling => {
                error!("Another cancellation is already issued.");
            }
            _ => {
                *self = XhciTransferState::Cancelled;
            }
        }
    }
}

/// Type of a transfer received handled by transfer ring.
pub enum XhciTransferType {
    // Normal means bulk transfer or interrupt transfer, depending on endpoint type.
    // See spec 4.11.2.1.
    Normal,
    // See usb spec for setup stage, data stage and status stage,
    // see xHCI spec 4.11.2.2 for corresponding trbs.
    SetupStage,
    DataStage,
    StatusStage,
    // See xHCI spec 4.11.2.3.
    Isochronous,
    // See xHCI spec 6.4.1.4.
    Noop,
}

impl Display for XhciTransferType {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        use self::XhciTransferType::*;

        match self {
            Normal => write!(f, "Normal"),
            SetupStage => write!(f, "SetupStage"),
            DataStage => write!(f, "DataStage"),
            StatusStage => write!(f, "StatusStage"),
            Isochronous => write!(f, "Isochronous"),
            Noop => write!(f, "Noop"),
        }
    }
}

/// The descriptor an endpoint had in progress when it stopped (spec 4.6.9), as its ring reports
/// it when it parks: the Stopped Transfer Event (6.4.2.1) and where the ring is left, the
/// descriptor's first TRB with the cycle state the ring had there.
#[derive(Debug, PartialEq, Eq)]
pub struct StoppedTransfer {
    /// Creation order; of several cancelled transfers the ring keeps the earliest.
    seq: u64,
    pub first_trb: GuestAddress,
    pub cycle: bool,
    /// What the descriptor moved before it stopped: its Stopped EDTLA (6.2.4.1).
    pub bytes: u32,
    pub completion_code: TrbCompletionCode,
    /// The TRB in progress, ED = 0.
    pub trb_pointer: u64,
    /// What that TRB had left to move; 0 with `StoppedLengthInvalid`.
    pub residual: u32,
}

/// Xhci Transfer manager holds reference to all ongoing transfers. Can cancel them all if
/// needed.
#[derive(Clone)]
pub struct XhciTransferManager {
    transfers: Arc<Mutex<Vec<Weak<Mutex<XhciTransferState>>>>>,
    device_slot: Weak<DeviceSlot>,
    /// Of the transfers the stop under way cancelled, the one dequeued first: the ring is left
    /// at it and it alone is reported Stopped. Cleared when a stop begins.
    stopped: Arc<Mutex<Option<StoppedTransfer>>>,
    /// Creation order of the transfers, so the earliest of several cancelled ones is known.
    next_seq: Arc<AtomicU64>,
}

impl XhciTransferManager {
    /// Create a new manager.
    pub fn new(device_slot: Weak<DeviceSlot>) -> XhciTransferManager {
        XhciTransferManager {
            transfers: Arc::new(Mutex::new(Vec::new())),
            device_slot,
            stopped: Arc::new(Mutex::new(None)),
            next_seq: Arc::new(AtomicU64::new(0)),
        }
    }

    /// Build a new XhciTransfer. Endpoint id is the id in xHCI device slot.
    pub fn create_transfer(
        &self,
        mem: GuestMemory,
        port: Arc<UsbPort>,
        interrupter: Arc<Mutex<Interrupter>>,
        slot_id: u8,
        endpoint_id: u8,
        transfer_trbs: TransferDescriptor,
        completion_event: Event,
        stream_id: Option<u16>,
    ) -> XhciTransfer {
        assert!(!transfer_trbs.is_empty());
        let transfer_dir = {
            if endpoint_id == 0 {
                TransferDirection::Control
            } else if (endpoint_id % 2) == 0 {
                TransferDirection::Out
            } else {
                TransferDirection::In
            }
        };
        let t = XhciTransfer {
            manager: self.clone(),
            seq: self.next_seq.fetch_add(1, Ordering::Relaxed),
            state: Arc::new(Mutex::new(XhciTransferState::Created)),
            mem,
            port,
            interrupter,
            transfer_completion_event: completion_event,
            slot_id,
            endpoint_id,
            transfer_dir,
            transfer_trbs,
            device_slot: self.device_slot.clone(),
            stream_id,
        };
        self.transfers.lock().push(Arc::downgrade(&t.state));
        t
    }

    /// Cancel all current transfers.
    pub fn cancel_all(&self) {
        // A new stop: what an earlier one left unread -- the ring halted before it parked --
        // is not this stop's descriptor in progress.
        *self.stopped.lock() = None;
        self.transfers.lock().iter().for_each(|t| {
            let state = match t.upgrade() {
                Some(state) => state,
                None => {
                    error!("transfer is already cancelled or finished");
                    return;
                }
            };
            state.lock().try_cancel();
        });
    }

    /// Returns true while any transfer this manager created is still alive. A transfer removes
    /// itself here when it is dropped, which happens once the backend has finished with it and its
    /// completion has been reported, so this answers whether the endpoint is really quiescent.
    pub fn has_pending_transfers(&self) -> bool {
        self.transfers.lock().iter().any(|t| t.upgrade().is_some())
    }

    /// The descriptor the stop under way left in progress, once: the earliest of those its
    /// transfers reported cancelled. `None` when none was (the ring stopped idle, or a transfer
    /// that raced the cancel completed instead).
    pub fn take_stopped(&self) -> Option<StoppedTransfer> {
        self.stopped.lock().take()
    }

    /// Keeps the earliest of the transfers this stop cancelled. That the ring may rewind to it
    /// leans on an invariant this code does not check: a USB pipe completes in order and usbfs
    /// reaps in completion order, so every transfer that really completed did so ahead of the
    /// earliest cancelled one -- the descriptors at and behind the rewind point are exactly the
    /// unfinished ones, and none that already reported success is run again.
    fn record_stopped(&self, stopped: StoppedTransfer) {
        let mut earliest = self.stopped.lock();
        let is_earlier = match earliest.as_ref() {
            Some(e) => stopped.seq < e.seq,
            None => true,
        };
        if is_earlier {
            *earliest = Some(stopped);
        }
    }

    fn remove_transfer(&self, t: &Arc<Mutex<XhciTransferState>>) {
        let mut transfers = self.transfers.lock();
        match transfers.iter().position(|wt| match wt.upgrade() {
            Some(wt) => Arc::ptr_eq(&wt, t),
            None => false,
        }) {
            None => error!("attempted to remove unknow transfer"),
            Some(i) => {
                transfers.swap_remove(i);
            }
        }
    }
}

impl Default for XhciTransferManager {
    fn default() -> Self {
        Self::new(Weak::new())
    }
}

/// Xhci transfer denotes a transfer initiated by guest os driver. It will be submitted to a
/// XhciBackendDevice.
pub struct XhciTransfer {
    manager: XhciTransferManager,
    seq: u64,
    state: Arc<Mutex<XhciTransferState>>,
    mem: GuestMemory,
    port: Arc<UsbPort>,
    interrupter: Arc<Mutex<Interrupter>>,
    slot_id: u8,
    // id of endpoint in device slot.
    endpoint_id: u8,
    transfer_dir: TransferDirection,
    transfer_trbs: TransferDescriptor,
    transfer_completion_event: Event,
    device_slot: Weak<DeviceSlot>,
    stream_id: Option<u16>,
}

impl Drop for XhciTransfer {
    fn drop(&mut self) {
        self.manager.remove_transfer(&self.state);
    }
}

impl fmt::Debug for XhciTransfer {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "xhci_transfer slot id: {}, endpoint id {}, transfer_dir {:?}, transfer_trbs {:?}",
            self.slot_id, self.endpoint_id, self.transfer_dir, self.transfer_trbs
        )
    }
}

impl XhciTransfer {
    /// Get state of this transfer.
    pub fn state(&self) -> &Arc<Mutex<XhciTransferState>> {
        &self.state
    }

    /// Get transfer type.
    pub fn get_transfer_type(&self) -> Result<XhciTransferType> {
        // We can figure out transfer type from the first trb.
        // See transfer descriptor description in xhci spec for more details.
        match self.transfer_trbs[0]
            .trb
            .get_trb_type()
            .map_err(Error::TrbType)?
        {
            TrbType::Normal => Ok(XhciTransferType::Normal),
            TrbType::SetupStage => Ok(XhciTransferType::SetupStage),
            TrbType::DataStage => Ok(XhciTransferType::DataStage),
            TrbType::StatusStage => Ok(XhciTransferType::StatusStage),
            TrbType::Isoch => Ok(XhciTransferType::Isochronous),
            TrbType::Noop => Ok(XhciTransferType::Noop),
            t => Err(Error::BadTrbType(t)),
        }
    }

    /// Create a scatter gather buffer for the given xhci transfer
    pub fn create_buffer(&self) -> Result<ScatterGatherBuffer> {
        ScatterGatherBuffer::new(self.mem.clone(), self.transfer_trbs.clone())
            .map_err(Error::CreateBuffer)
    }

    /// Create a usb request setup for the control transfer buffer
    pub fn create_usb_request_setup(&self) -> Result<UsbRequestSetup> {
        let trb = self.transfer_trbs[0]
            .trb
            .checked_cast::<SetupStageTrb>()
            .map_err(Error::CastTrb)?;
        Ok(UsbRequestSetup::new(
            trb.get_request_type(),
            trb.get_request(),
            trb.get_value(),
            trb.get_index(),
            trb.get_length(),
        ))
    }

    /// Get endpoint number.
    pub fn get_endpoint_number(&self) -> u8 {
        // See spec 4.5.1 for dci.
        self.endpoint_id / 2
    }

    /// get transfer direction.
    pub fn get_transfer_dir(&self) -> TransferDirection {
        self.transfer_dir
    }

    /// get stream id.
    pub fn get_stream_id(&self) -> Option<u16> {
        self.stream_id
    }

    /// This functions should be invoked when transfer is completed (or failed).
    pub fn on_transfer_complete(
        &self,
        status: &TransferStatus,
        bytes_transferred: u32,
    ) -> Result<()> {
        match status {
            TransferStatus::NoDevice => {
                info!("xhci: device disconnected, detaching from port");
                // No Transfer Event -- the guest learns of the disconnect through the port. The
                // ring is still signalled: a Stop Endpoint waiting on this transfer (its
                // discarded URBs are reaped -ENODEV, not -ENOENT, when the device disappears
                // under the stop) parks only when the completion is signalled, and the Stop
                // Endpoint Command Completion -- with every command behind it, the Disable Slot
                // for this very disconnect included -- waits on that. On a running ring the
                // signal just wakes it to find the backend gone.
                self.transfer_completion_event
                    .signal()
                    .map_err(Error::WriteCompletionEvent)?;
                return match self.port.detach() {
                    Ok(()) => Ok(()),
                    // It's acceptable for the port to be already disconnected
                    // as asynchronous transfer completions are processed.
                    Err(HubError::AlreadyDetached(_e)) => Ok(()),
                    Err(e) => Err(Error::DetachPort(e)),
                };
            }
            TransferStatus::Cancelled => {
                // The endpoint stopped with this descriptor in progress (spec 4.6.9): it is
                // reported with a Stopped Transfer Event and the ring is left at it. Both happen
                // when the ring parks (the handler's `finish_stop`), so the event precedes the
                // Stop Endpoint Command Completion, and a drained-ahead ring with several
                // descriptors cancelled reports the earliest one only.
                self.manager
                    .record_stopped(self.stopped_at(bytes_transferred)?);
                return self
                    .transfer_completion_event
                    .signal()
                    .map_err(Error::WriteCompletionEvent);
            }
            TransferStatus::Completed => {
                self.transfer_completion_event
                    .signal()
                    .map_err(Error::WriteCompletionEvent)?;
            }
            TransferStatus::Stalled => {
                warn!("xhci: endpoint is stalled. set state to Halted");
                if let Some(device_slot) = self.device_slot.upgrade() {
                    device_slot
                        .halt_endpoint(self.endpoint_id)
                        .map_err(|_| Error::HaltEndpoint(self.endpoint_id))?;
                }
                self.transfer_completion_event
                    .signal()
                    .map_err(Error::WriteCompletionEvent)?;
            }
            _ => {
                // Transfer failed, we are not handling this correctly yet. Guest kernel might see
                // short packets for in transfer and might think control transfer is successful. It
                // will eventually find out device is in a wrong state.
                self.transfer_completion_event
                    .signal()
                    .map_err(Error::WriteCompletionEvent)?;
            }
        }

        let mut edtla: u32 = 0;
        // As noted in xHCI spec 4.11.3.1
        // Transfer Event TRB only occurs under the following conditions:
        //   1. If the Interrupt On Completion flag is set.
        //   2. When a short transfer occurs during the execution of a Transfer TRB and the
        //      Interrupt-on-Short Packet flag is set.
        //   3. If an error occurs during the execution of a Transfer TRB.
        for atrb in &self.transfer_trbs {
            edtla += atrb.trb.transfer_length().map_err(Error::TransferLength)?;
            if atrb.trb.interrupt_on_completion()
                || (atrb.trb.interrupt_on_short_packet() && edtla > bytes_transferred)
            {
                // For details about event data trb and EDTLA, see spec 4.11.5.2.
                if atrb.trb.get_trb_type().map_err(Error::TrbType)? == TrbType::EventData {
                    // The event reports what the TD moved so far (EDTLA, spec 4.11.5.2) and
                    // carries the TD's own outcome: a stalled or short TD is not a success just
                    // because the Event Data TRB itself had nothing to transfer.
                    let tlength = min(edtla, bytes_transferred);
                    let code = if *status == TransferStatus::Stalled {
                        TrbCompletionCode::StallError
                    } else if edtla > bytes_transferred {
                        TrbCompletionCode::ShortPacket
                    } else {
                        TrbCompletionCode::Success
                    };
                    self.interrupter
                        .lock()
                        .send_transfer_event_trb(
                            code,
                            atrb.trb
                                .cast::<EventDataTrb>()
                                .map_err(Error::CastTrb)?
                                .get_event_data(),
                            tlength,
                            true,
                            self.slot_id,
                            self.endpoint_id,
                        )
                        .map_err(Error::SendInterrupt)?;
                } else if *status == TransferStatus::Stalled {
                    debug!("xhci: on transfer complete stalled");
                    let residual_transfer_length = edtla - bytes_transferred;
                    self.interrupter
                        .lock()
                        .send_transfer_event_trb(
                            TrbCompletionCode::StallError,
                            atrb.gpa,
                            residual_transfer_length,
                            // ED is clear: the pointer is the TRB that completed, not an Event
                            // Data value.
                            false,
                            self.slot_id,
                            self.endpoint_id,
                        )
                        .map_err(Error::SendInterrupt)?;
                } else {
                    // For Short Transfer details, see xHCI spec 4.10.1.1.
                    if edtla > bytes_transferred {
                        debug!("xhci: on transfer complete short packet");
                        let residual_transfer_length = edtla - bytes_transferred;
                        self.interrupter
                            .lock()
                            .send_transfer_event_trb(
                                TrbCompletionCode::ShortPacket,
                                atrb.gpa,
                                residual_transfer_length,
                                false,
                                self.slot_id,
                                self.endpoint_id,
                            )
                            .map_err(Error::SendInterrupt)?;
                    } else {
                        debug!("xhci: on transfer complete success");
                        self.interrupter
                            .lock()
                            .send_transfer_event_trb(
                                TrbCompletionCode::Success,
                                atrb.gpa,
                                0, // transfer length
                                false,
                                self.slot_id,
                                self.endpoint_id,
                            )
                            .map_err(Error::SendInterrupt)?;
                    }
                }
            }
        }
        Ok(())
    }

    /// Where this descriptor stopped after `bytes_transferred` of it moved. The first data TRB
    /// not moved whole is the TRB in progress and the event carries what it had left (spec
    /// 6.4.2.1); a descriptor whose data TRBs all moved stopped at its last one with nothing
    /// left; one without a data TRB (a bare Event Data or No-op TRB) has no length to report,
    /// which is Stopped - Length Invalid. The pointer is always a transfer TRB, ED = 0
    /// (4.11.5.2). The ring is left at the descriptor's first TRB, whose cycle bit is the
    /// consumer cycle state the ring had there.
    fn stopped_at(&self, bytes_transferred: u32) -> Result<StoppedTransfer> {
        let first = &self.transfer_trbs[0];
        let mut edtla: u32 = 0;
        let mut last_data_trb = None;
        let mut in_progress = None;
        for atrb in &self.transfer_trbs {
            // The TRB types that advance the EDTLA (spec 4.11.5.2): Normal, Data Stage and
            // Isoch. A Setup Stage TRB moves its 8 bytes outside the data path and does not.
            let carries_data = matches!(
                atrb.trb.get_trb_type().map_err(Error::TrbType)?,
                TrbType::Normal | TrbType::DataStage | TrbType::Isoch
            );
            if !carries_data {
                continue;
            }
            edtla += atrb.trb.transfer_length().map_err(Error::TransferLength)?;
            last_data_trb = Some(atrb);
            if edtla > bytes_transferred {
                in_progress = Some((atrb, edtla - bytes_transferred));
                break;
            }
        }
        let (completion_code, trb_pointer, residual) =
            match in_progress.or_else(|| last_data_trb.map(|atrb| (atrb, 0))) {
                Some((atrb, residual)) => (TrbCompletionCode::Stopped, atrb.gpa, residual),
                None => (TrbCompletionCode::StoppedLengthInvalid, first.gpa, 0),
            };
        Ok(StoppedTransfer {
            seq: self.seq,
            first_trb: GuestAddress(first.gpa),
            cycle: first.trb.get_cycle(),
            bytes: bytes_transferred,
            completion_code,
            trb_pointer,
            residual,
        })
    }

    /// Send this transfer to backend if it's a valid transfer.
    pub fn send_to_backend_if_valid(self) -> Result<()> {
        if self.validate_transfer()? {
            // Backend should invoke on transfer complete when transfer is completed.
            let port = self.port.clone();
            let mut backend = port.backend_device();
            match &mut *backend {
                Some(backend) => {
                    let (slot_id, endpoint_id) = (self.slot_id, self.endpoint_id);
                    backend.lock().submit_xhci_transfer(self).map_err(|e| {
                        // The controller dies on this; say why before it does.
                        error!(
                            "xhci: backend rejected transfer on slot {} endpoint {}: {}",
                            slot_id, endpoint_id, e
                        );
                        Error::SubmitTransfer
                    })?
                }
                None => {
                    error!("backend is already disconnected");
                    self.transfer_completion_event
                        .signal()
                        .map_err(Error::WriteCompletionEvent)?;
                }
            }
        } else {
            error!("invalid td on transfer ring");
            self.transfer_completion_event
                .signal()
                .map_err(Error::WriteCompletionEvent)?;
        }
        Ok(())
    }

    // Check each trb in the transfer descriptor for invalid or out of bounds
    // parameters. Returns true iff the transfer descriptor is valid.
    fn validate_transfer(&self) -> Result<bool> {
        let mut valid = true;
        for atrb in &self.transfer_trbs {
            if !trb_is_valid(atrb) {
                self.interrupter
                    .lock()
                    .send_transfer_event_trb(
                        TrbCompletionCode::TrbError,
                        atrb.gpa,
                        0,
                        false,
                        self.slot_id,
                        self.endpoint_id,
                    )
                    .map_err(Error::SendInterrupt)?;
                valid = false;
            }
        }
        Ok(valid)
    }
}

fn trb_is_valid(atrb: &AddressedTrb) -> bool {
    let can_be_in_transfer_ring = match atrb.trb.can_be_in_transfer_ring() {
        Ok(v) => v,
        Err(e) => {
            error!("unknown error {:?}", e);
            return false;
        }
    };
    can_be_in_transfer_ring && (atrb.trb.interrupter_target() < MAX_INTERRUPTER)
}

#[cfg(test)]
mod tests {
    use super::super::test_util::normal_td;
    use super::super::test_util::Fixture;
    use super::super::xhci_abi::NoopTrb;
    use super::super::xhci_abi::NormalTrb;
    use super::super::xhci_abi::Trb;
    use super::*;

    /// A TRB of `ty` (an Event Data or No-op TRB) at `gpa`, ending its descriptor.
    fn data_less_trb(ty: TrbType, gpa: u64) -> AddressedTrb {
        let mut trb = NoopTrb::new();
        trb.set_trb_type(ty);
        trb.set_cycle(true);
        trb.set_chain(false);
        AddressedTrb {
            trb: *trb.cast::<Trb>().unwrap(),
            gpa,
        }
    }

    /// What a cancelled transfer of `td` with `bytes` moved records for its ring.
    fn stopped(f: &Fixture, td: TransferDescriptor, bytes: u32) -> StoppedTransfer {
        let manager = XhciTransferManager::default();
        f.transfer(&manager, 3, td)
            .on_transfer_complete(&TransferStatus::Cancelled, bytes)
            .unwrap();
        manager.take_stopped().unwrap()
    }

    #[test]
    fn cancelled_td_reports_stopped_at_the_trb_in_progress() {
        let f = Fixture::new();
        // 0x150 of 0x100 + 0x200 + 0x300 moved: the second TRB is in progress, 0x1b0 left of it.
        let td = stopped(&f, normal_td(0x2000, &[0x100, 0x200, 0x300]), 0x150);
        assert_eq!(td.completion_code, TrbCompletionCode::Stopped);
        assert_eq!(td.trb_pointer, 0x2010);
        assert_eq!(td.residual, 0x1b0);
        assert_eq!(td.first_trb, GuestAddress(0x2000));
        assert!(td.cycle);
        assert_eq!(td.bytes, 0x150);

        // Nothing moved: the first TRB, whole.
        let td = stopped(&f, normal_td(0x2000, &[0x100, 0x200, 0x300]), 0);
        assert_eq!(td.completion_code, TrbCompletionCode::Stopped);
        assert_eq!(td.trb_pointer, 0x2000);
        assert_eq!(td.residual, 0x100);
        assert_eq!(td.bytes, 0);

        // Everything moved: the last TRB, with nothing left.
        let td = stopped(&f, normal_td(0x2000, &[0x100, 0x200, 0x300]), 0x600);
        assert_eq!(td.completion_code, TrbCompletionCode::Stopped);
        assert_eq!(td.trb_pointer, 0x2020);
        assert_eq!(td.residual, 0);

        // A descriptor ending in an Event Data TRB: the pointer is the data TRB, never the
        // Event Data one (ED = 0, spec 4.11.5.2).
        let mut td = normal_td(0x2000, &[0x100]);
        td[0].trb.cast_mut::<NormalTrb>().unwrap().set_chain(true);
        td.push(data_less_trb(TrbType::EventData, 0x2010));
        let td = stopped(&f, td, 0);
        assert_eq!(td.completion_code, TrbCompletionCode::Stopped);
        assert_eq!(td.trb_pointer, 0x2000);
        assert_eq!(td.residual, 0x100);
        assert_eq!(td.first_trb, GuestAddress(0x2000));
    }

    #[test]
    fn a_cancelled_td_without_data_trbs_is_stopped_length_invalid() {
        let f = Fixture::new();
        for ty in [TrbType::EventData, TrbType::Noop] {
            let td = stopped(&f, vec![data_less_trb(ty, 0x2000)], 0);
            assert_eq!(
                td.completion_code,
                TrbCompletionCode::StoppedLengthInvalid,
                "{ty:?}"
            );
            assert_eq!(td.trb_pointer, 0x2000);
            assert_eq!(td.residual, 0);
            assert_eq!(td.first_trb, GuestAddress(0x2000));
            assert!(td.cycle);
        }
    }

    /// A transfer reaped after the device disconnected (-ENODEV) sends no Transfer Event, but
    /// its ring is still signalled: a Stop Endpoint waiting on it parks only on that signal,
    /// and the command ring dequeues nothing more until the stop is answered.
    #[test]
    fn a_no_device_completion_still_signals_the_ring() {
        let f = Fixture::new();
        let manager = XhciTransferManager::default();
        let signal = base::Event::new().unwrap();
        let transfer = manager.create_transfer(
            f.mem.clone(),
            f.port(),
            f.interrupter.clone(),
            super::super::test_util::SLOT_ID,
            3,
            normal_td(0x2000, &[0x100]),
            signal.try_clone().unwrap(),
            None,
        );
        transfer
            .on_transfer_complete(&TransferStatus::NoDevice, 0)
            .unwrap();
        assert_eq!(
            signal
                .wait_timeout(std::time::Duration::from_millis(200))
                .unwrap(),
            base::EventWaitResult::Signaled,
            "the ring a stop is waiting on must be signalled even though the device is gone"
        );
        assert_eq!(manager.take_stopped(), None, "not the descriptor in progress");
    }

    /// Several transfers cancelled by one stop (a drained-ahead ring): the ring is left at the
    /// one dequeued first, whichever order they are reaped in, and a new stop starts afresh.
    #[test]
    fn the_earliest_cancelled_transfer_is_the_one_the_ring_stops_at() {
        let f = Fixture::new();
        let manager = XhciTransferManager::default();
        let first = f.transfer(&manager, 3, normal_td(0x2000, &[0x100]));
        let second = f.transfer(&manager, 3, normal_td(0x2010, &[0x100]));
        let third = f.transfer(&manager, 3, normal_td(0x2020, &[0x100]));
        third
            .on_transfer_complete(&TransferStatus::Cancelled, 0)
            .unwrap();
        second
            .on_transfer_complete(&TransferStatus::Cancelled, 0)
            .unwrap();
        // The first completed before the cancel landed: it is not part of the stopped set.
        first
            .on_transfer_complete(&TransferStatus::Completed, 0x100)
            .unwrap();
        let td = manager.take_stopped().unwrap();
        assert_eq!(td.first_trb, GuestAddress(0x2010));
        assert_eq!(manager.take_stopped(), None, "reported once");

        // What an earlier stop left behind is not the next one's descriptor in progress.
        third
            .on_transfer_complete(&TransferStatus::Cancelled, 0)
            .unwrap();
        manager.cancel_all();
        assert_eq!(manager.take_stopped(), None);
    }
}
