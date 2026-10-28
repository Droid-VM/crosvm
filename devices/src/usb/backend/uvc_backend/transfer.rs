// Copyright 2026 The crosvm Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use std::sync::Weak;

use base::error;
use sync::Mutex;
use usb_util::TransferBuffer;
use usb_util::TransferStatus;

use crate::usb::backend::error::Error as BackendError;
use crate::usb::backend::error::Result as BackendResult;
use crate::usb::backend::transfer::BackendTransfer;
use crate::usb::backend::transfer::BackendTransferType;
use crate::usb::backend::transfer::GenericTransferHandle;

/// Implementation of a generic USB transfer for the UVC backend. The transfer
/// never reaches usbdevfs, so it carries its own buffer, status and completion
/// callback.
pub struct UvcTransfer {
    /// Buffer holding the control setup followed by its data, or the payload of
    /// a bulk transfer.
    pub buffer: TransferBuffer,
    /// Status of the transfer, reported to the xhci layer on completion.
    status: TransferStatus,
    /// Amount of data actually moved by this transfer.
    pub actual_length: usize,
    /// USB endpoint associated with this transfer.
    pub endpoint: u8,
    /// Callback executed once the transfer completes.
    pub callback: Option<Box<dyn Fn(UvcTransfer) + Send + Sync>>,
}

impl UvcTransfer {
    pub fn new(endpoint: u8, buffer: TransferBuffer) -> UvcTransfer {
        UvcTransfer {
            buffer,
            // The default status is `Error`; `complete_transfer` turns it into
            // `Completed` unless the device explicitly reported a failure.
            status: TransferStatus::Error,
            actual_length: 0,
            endpoint,
            callback: None,
        }
    }

    /// Overrides the status the transfer completes with.
    pub fn set_status(&mut self, status: TransferStatus) {
        self.status = status;
    }

    /// Finalizes the transfer and hands it to the xhci completion callback.
    pub fn complete_transfer(mut self) {
        if self.status == TransferStatus::Error {
            self.status = TransferStatus::Completed;
        }

        if let Some(cb) = self.callback.take() {
            cb(self);
        }
    }
}

impl BackendTransfer for UvcTransfer {
    fn status(&self) -> TransferStatus {
        self.status
    }

    fn actual_length(&self) -> usize {
        self.actual_length
    }

    fn buffer(&self) -> &TransferBuffer {
        &self.buffer
    }

    fn set_callback<C: 'static + Fn(BackendTransferType) + Send + Sync>(&mut self, cb: C) {
        let callback = move |t: UvcTransfer| cb(BackendTransferType::UvcDevice(t));
        self.callback = Some(Box::new(callback));
    }
}

/// Implementation of a cancel handler for `UvcTransfer`.
pub struct UvcTransferHandle {
    pub weak_transfer: Weak<Mutex<Option<UvcTransfer>>>,
}

impl GenericTransferHandle for UvcTransferHandle {
    fn cancel(&self) -> BackendResult<()> {
        let rc_transfer = match self.weak_transfer.upgrade() {
            None => {
                return Err(BackendError::TransferHandleAlreadyComplete);
            }
            Some(rc_transfer) => rc_transfer,
        };

        let mut lock = rc_transfer.lock();

        let mut transfer = match lock.take() {
            Some(t) => t,
            None => {
                error!("uvc: transfer has already been lost while being cancelled");
                return Err(BackendError::TransferHandleAlreadyComplete);
            }
        };
        transfer.set_status(TransferStatus::Cancelled);
        *lock = Some(transfer);
        Ok(())
    }
}
