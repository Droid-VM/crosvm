// Copyright 2019 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use std::sync::Arc;

use anyhow::Context;
use base::error;
use sync::Mutex;
use usb_util::TransferStatus;

use crate::usb::backend::device::BackendDeviceType;
use crate::usb::backend::error::Error;
use crate::usb::backend::error::Result;
use crate::usb::xhci::xhci_transfer::XhciTransfer;
use crate::usb::xhci::xhci_transfer::XhciTransferState;
use crate::utils::EventHandler;

#[macro_export]
/// Allows dispatching a function call to all its enum value implementations.
/// See `BackendDeviceType` in usb/backend/device.rs for an example usage of it.
///
/// # Arguments
///
/// * `self` - Replacement for the local `self` reference in the function call.
/// * `enum` - Enum name that the macro is matching on.
/// * `types` - Space-separated list of value types of the given enum.
/// * `func` - Function name that will be called by each match arm.
/// * `param` - Optional parameters needed for the given function call.
macro_rules! multi_dispatch {
    ($self:ident, $enum:ident, $($types:ident )+, $func:ident) => {
        match $self {
            $(
                $enum::$types(device) => device.$func(),
            )+
        }
    };
    ($self:ident, $enum:ident, $($types:ident )+, $func:ident, $param:expr) => {
        match $self {
            $(
                $enum::$types(device) => device.$func($param),
            )+
        }
    };
    ($self:ident, $enum:ident, $($types:ident )+, $func:ident, $param1:expr, $param2: expr) => {
        match $self {
            $(
                $enum::$types(device) => device.$func($param1, $param2),
            )+
        }
    };
    ($self:ident, $enum:ident, $($types:ident )+, $func:ident, $param1:expr, $param2: expr, $param3: expr) => {
        match $self {
            $(
                $enum::$types(device) => device.$func($param1, $param2, $param3),
            )+
        }
    };
}

pub(crate) use multi_dispatch;

pub struct UsbUtilEventHandler {
    pub device: Arc<Mutex<BackendDeviceType>>,
}

impl EventHandler for UsbUtilEventHandler {
    fn on_event(&self) -> anyhow::Result<()> {
        match &mut *self.device.lock() {
            BackendDeviceType::HostDevice(host_device) => host_device
                .device
                .lock()
                .poll_transfers()
                .context("UsbUtilEventHandler poll_transfers failed"),
            BackendDeviceType::FidoDevice(fido_device) => fido_device
                .read_hidraw_file()
                .context("FidoDeviceEventHandler failed to read hidraw device"),
        }
    }
}

/// Helper function to update xhci_transfer state.
///
/// Only a URB the kernel really unlinked (-ENOENT) is `Cancelled`. One whose cancel was issued
/// but that came back with any other status had finished on the device before the discard
/// landed -- usbfs answers the discard with EINVAL then -- and its data and status are as real
/// as any other completion's: it is `Completed` and reports its ordinary Transfer Event, as on
/// hardware, where a TD that completes as the endpoint stops is not the one in progress.
/// Reported as cancelled instead, it was a TD with neither a completion nor a Stopped event;
/// Windows re-queued it after its watchdog and waited on a device that had already answered.
///
/// One consequence of completions keeping their status: a STALL reaped while a Stop Endpoint
/// is under way takes the ordinary Stalled arm and halts the endpoint mid-stop. That is not a
/// data race -- reaps, command handling and the context writes all run on the one xhci event
/// loop -- but the halt parks the ring, which answers the stop with Success while the Endpoint
/// Context says Halted; hardware, where the stall landed first, would have answered Context
/// State Error (4.6.9). The guest sees the Halted state and recovers through Reset Endpoint,
/// as it would on hardware.
pub fn update_transfer_state(
    xhci_transfer: &Arc<XhciTransfer>,
    status: TransferStatus,
) -> Result<()> {
    let mut state = xhci_transfer.state().lock();

    match (status, &*state) {
        // -ENOENT can only come from our own discard: the URB really was unlinked. The state
        // is `Cancelling` then (`try_cancel` sets it before the ioctl), or still `Submitted`
        // for a backend that cancels without the ring's involvement (a fido timeout).
        (
            TransferStatus::Cancelled,
            XhciTransferState::Cancelling | XhciTransferState::Submitted { .. },
        ) => {
            *state = XhciTransferState::Cancelled;
        }
        (TransferStatus::Cancelled, _) => {
            error!("xhci transfer state is invalid");
            *state = XhciTransferState::Cancelled;
            return Err(Error::BadXhciTransferState);
        }
        (_, XhciTransferState::Cancelling | XhciTransferState::Submitted { .. }) => {
            *state = XhciTransferState::Completed;
        }
        _ => {
            error!("xhci trasfer state is invalid");
            *state = XhciTransferState::Completed;
            return Err(Error::BadXhciTransferState);
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::usb::xhci::test_util::normal_td;
    use crate::usb::xhci::test_util::Fixture;
    use crate::usb::xhci::xhci_transfer::XhciTransferManager;

    fn cancelling_transfer(f: &Fixture) -> Arc<XhciTransfer> {
        let t = f.transfer(
            &XhciTransferManager::default(),
            3,
            normal_td(0x2000, &[0x100]),
        );
        *t.state().lock() = XhciTransferState::Cancelling;
        Arc::new(t)
    }

    /// The URB was discarded after it had completed (usbfs: EINVAL on the discard, then the
    /// URB reaped with its real status): the transfer completed, and reports so.
    #[test]
    fn a_transfer_reaped_complete_after_its_cancel_is_completed() {
        let f = Fixture::new();
        for status in [
            TransferStatus::Completed,
            TransferStatus::Error,
            TransferStatus::Stalled,
        ] {
            let t = cancelling_transfer(&f);
            update_transfer_state(&t, status).unwrap();
            assert!(
                matches!(*t.state().lock(), XhciTransferState::Completed),
                "a cancelling transfer reaped with a status other than Cancelled completed"
            );
        }

        // Only a URB the kernel unlinked (-ENOENT) was really cancelled.
        let t = cancelling_transfer(&f);
        update_transfer_state(&t, TransferStatus::Cancelled).unwrap();
        assert!(matches!(*t.state().lock(), XhciTransferState::Cancelled));

        // A backend that cancels without the ring's involvement (a fido timeout) reaps a
        // still-Submitted transfer as cancelled.
        let t = f.transfer(
            &XhciTransferManager::default(),
            3,
            normal_td(0x2000, &[0x100]),
        );
        *t.state().lock() = XhciTransferState::Submitted {
            cancel_callback: Box::new(|| ()),
        };
        let t = Arc::new(t);
        update_transfer_state(&t, TransferStatus::Cancelled).unwrap();
        assert!(matches!(*t.state().lock(), XhciTransferState::Cancelled));
    }
}
