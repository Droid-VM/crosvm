// Copyright 2019 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use std::mem::size_of;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::sync::Weak;

use base::debug;
use base::error;
use base::info;
use base::warn;
use bit_field::Error as BitFieldError;
use remain::sorted;
use sync::Mutex;
use thiserror::Error;
use vm_memory::GuestAddress;
use vm_memory::GuestMemory;
use vm_memory::GuestMemoryError;

use super::interrupter::Interrupter;
use super::transfer_ring_controller::TransferRingController;
use super::transfer_ring_controller::TransferRingControllerError;
use super::transfer_ring_controller::TransferRingControllers;
use super::usb_hub;
use super::usb_hub::UsbHub;
use super::xhci_abi::AddressDeviceCommandTrb;
use super::xhci_abi::ConfigureEndpointCommandTrb;
use super::xhci_abi::DequeuePtr;
use super::xhci_abi::DeviceContext;
use super::xhci_abi::DeviceSlotState;
use super::xhci_abi::EndpointContext;
use super::xhci_abi::EndpointState;
use super::xhci_abi::EvaluateContextCommandTrb;
use super::xhci_abi::InputControlContext;
use super::xhci_abi::SlotContext;
use super::xhci_abi::StreamContext;
use super::xhci_abi::TrbCompletionCode;
use super::xhci_abi::DEVICE_CONTEXT_ENTRY_SIZE;
use super::xhci_abi::STREAM_CONTEXT_SIZE;
use super::xhci_backend_device::XhciBackendDevice;
use super::xhci_regs::valid_max_pstreams;
use super::xhci_regs::valid_slot_id;
use super::xhci_regs::MAX_PORTS;
use super::xhci_regs::MAX_SLOTS;
use crate::register_space::Register;
use crate::usb::backend::error::Error as BackendProviderError;
use crate::usb::xhci::ring_buffer_stop_cb::fallible_closure;
use crate::usb::xhci::ring_buffer_stop_cb::RingBufferStopCallback;
use crate::utils::EventLoop;
use crate::utils::FailHandle;

#[sorted]
#[derive(Error, Debug)]
pub enum Error {
    #[error("bad device context: {0}")]
    BadDeviceContextAddr(GuestAddress),
    #[error("device slot get a bad endpoint id: {0}")]
    BadEndpointId(u8),
    #[error("bad input context address: {0}")]
    BadInputContextAddr(GuestAddress),
    #[error("device slot get a bad port id: {0}")]
    BadPortId(u8),
    #[error("callback failed")]
    CallbackFailed,
    #[error("failed to create transfer controller: {0}")]
    CreateTransferController(TransferRingControllerError),
    #[error("failed to get endpoint state: {0}")]
    GetEndpointState(BitFieldError),
    #[error("failed to get port: {0}")]
    GetPort(u8),
    #[error("failed to get slot context state: {0}")]
    GetSlotContextState(BitFieldError),
    #[error("failed to get trc: {0}")]
    GetTrc(u8),
    #[error("failed to read guest memory: {0}")]
    ReadGuestMemory(GuestMemoryError),
    #[error("failed to reset port: {0}")]
    ResetPort(BackendProviderError),
    #[error("failed to upgrade weak reference")]
    WeakReferenceUpgrade,
    #[error("failed to write guest memory: {0}")]
    WriteGuestMemory(GuestMemoryError),
}

type Result<T> = std::result::Result<T, Error>;

/// See spec 4.5.1 for dci.
/// index 0: Control endpoint. Device Context Index: 1.
/// index 1: Endpoint 1 out. Device Context Index: 2
/// index 2: Endpoint 1 in. Device Context Index: 3.
/// index 3: Endpoint 2 out. Device Context Index: 4
/// ...
/// index 30: Endpoint 15 in. Device Context Index: 31
pub const TRANSFER_RING_CONTROLLERS_INDEX_END: usize = 31;
/// End of device context index.
pub const DCI_INDEX_END: u8 = (TRANSFER_RING_CONTROLLERS_INDEX_END + 1) as u8;
/// Device context index of first transfer endpoint.
pub const FIRST_TRANSFER_ENDPOINT_DCI: u8 = 2;

fn valid_endpoint_id(endpoint_id: u8) -> bool {
    endpoint_id < DCI_INDEX_END && endpoint_id > 0
}

/// Reads the primary Stream Context Array of an endpoint with `max_pstreams`, one entry per
/// stream id 1..2^(MaxPStreams+1)-1 (entry 0 is reserved, spec 4.12.2), each the ring position
/// of an initialised stream or `None` for one that is not.
///
/// The array is walked entry by entry, never as a fixed 16-entry block: a MaxPStreams=1 array is
/// 64 bytes and may sit at the end of a page. A Stream Context Type of 0 is Not Valid (spec
/// 6.2.4.1, Table 6-13) -- the guest only initialises the streams it opened, Windows' USBXHCI in
/// particular leaves every other entry of its power-of-two array zero -- and a doorbell on such a
/// stream is an error at use, not at Configure Endpoint (4.12.1.1). The types 2..7 belong to
/// secondary arrays and hubs, which this controller does not do; the guest is told through the
/// same use-time path. Only what the spec validates at Configure Endpoint (4.6.6) is an error
/// here: a MaxPStreams beyond MaxPSASize, or an array the guest did not back with memory.
fn read_stream_contexts(
    mem: &GuestMemory,
    stream_context_array_addr: GuestAddress,
    max_pstreams: u8,
) -> std::result::Result<Vec<Option<(GuestAddress, bool)>>, TrbCompletionCode> {
    if !valid_max_pstreams(max_pstreams) {
        return Err(TrbCompletionCode::ParameterError);
    }
    let pstreams = 1usize << (max_pstreams + 1);
    let mut stream_contexts = Vec::with_capacity(pstreams - 1);
    for stream_id in 1..pstreams {
        let addr = stream_context_array_addr
            .checked_add((stream_id * STREAM_CONTEXT_SIZE) as u64)
            .ok_or(TrbCompletionCode::ParameterError)?;
        let stream_context: StreamContext = mem
            .read_obj_from_addr(addr)
            .map_err(|_| TrbCompletionCode::ParameterError)?;
        stream_contexts.push(match stream_context.get_stream_context_type() {
            0 => None,
            1 => Some((
                stream_context.get_tr_dequeue_pointer().get_gpa(),
                stream_context.get_dequeue_cycle_state(),
            )),
            context_type => {
                warn!(
                    "stream context array {:#x}: stream {} has type {}, treating as Not Valid",
                    stream_context_array_addr.0, stream_id, context_type
                );
                None
            }
        });
    }
    Ok(stream_contexts)
}

#[derive(Clone)]
pub struct DeviceSlots {
    fail_handle: Arc<dyn FailHandle>,
    hub: Arc<UsbHub>,
    slots: Vec<Arc<DeviceSlot>>,
}

impl DeviceSlots {
    pub fn new(
        fail_handle: Arc<dyn FailHandle>,
        dcbaap: Register<u64>,
        hub: Arc<UsbHub>,
        interrupter: Arc<Mutex<Interrupter>>,
        event_loop: Arc<EventLoop>,
        mem: GuestMemory,
    ) -> DeviceSlots {
        let mut slots = Vec::new();
        for slot_id in 1..=MAX_SLOTS {
            slots.push(Arc::new(DeviceSlot::new(
                slot_id,
                dcbaap.clone(),
                hub.clone(),
                interrupter.clone(),
                event_loop.clone(),
                mem.clone(),
            )));
        }
        DeviceSlots {
            fail_handle,
            hub,
            slots,
        }
    }

    /// Note that slot id starts from 1. Slot index start from 0.
    pub fn slot(&self, slot_id: u8) -> Option<Arc<DeviceSlot>> {
        if valid_slot_id(slot_id) {
            Some(self.slots[slot_id as usize - 1].clone())
        } else {
            error!(
                "trying to index a wrong slot id {}, max slot = {}",
                slot_id, MAX_SLOTS
            );
            None
        }
    }

    /// Reset the device connected to a specific port.
    pub fn reset_port(&self, port_id: u8) -> Result<()> {
        if let Some(port) = self.hub.get_port(port_id) {
            if let Some(backend_device) = port.backend_device().as_mut() {
                backend_device.lock().reset().map_err(Error::ResetPort)?;
            }
        }

        // No device on port, so nothing to reset.
        Ok(())
    }

    /// Reset every device slot and the host hub: the slot half of a host controller reset. Only
    /// once every transfer ring is stopped -- run it from the callback given to `stop_all`.
    pub fn reset_all(&self) -> std::result::Result<(), usb_hub::Error> {
        info!("xhci: resetting all device slots and the host hub");
        for slot in &self.slots {
            slot.reset();
        }
        self.hub.reset()
    }

    /// Stop all devices. The auto callback will be executed when all trc is stopped. It could
    /// happen asynchronously, if there are any pending transfers.
    pub fn stop_all(&self, auto_callback: RingBufferStopCallback) {
        for slot in &self.slots {
            slot.stop_all_trc(auto_callback.clone());
        }
    }

    /// Disable a slot. This might happen asynchronously, if there is any pending transfers. The
    /// callback will be invoked when slot is actually disabled.
    pub fn disable_slot<
        C: FnMut(TrbCompletionCode) -> std::result::Result<(), ()> + 'static + Send,
    >(
        &self,
        slot_id: u8,
        cb: C,
    ) -> Result<()> {
        xhci_trace!("device slot {} is being disabled", slot_id);
        DeviceSlot::disable(
            self.fail_handle.clone(),
            &self.slots[slot_id as usize - 1],
            cb,
        )
    }

    /// Reset a slot. This is a shortcut call for DeviceSlot::reset_slot.
    pub fn reset_slot<
        C: FnMut(TrbCompletionCode) -> std::result::Result<(), ()> + 'static + Send,
    >(
        &self,
        slot_id: u8,
        cb: C,
    ) -> Result<()> {
        xhci_trace!("device slot {} is resetting", slot_id);
        DeviceSlot::reset_slot(
            self.fail_handle.clone(),
            &self.slots[slot_id as usize - 1],
            cb,
        )
    }

    pub fn stop_endpoint<
        C: FnMut(TrbCompletionCode) -> std::result::Result<(), ()> + 'static + Send,
    >(
        &self,
        slot_id: u8,
        endpoint_id: u8,
        cb: C,
    ) -> Result<()> {
        self.slots[slot_id as usize - 1].stop_endpoint(self.fail_handle.clone(), endpoint_id, cb)
    }

    pub fn reset_endpoint<
        C: FnMut(TrbCompletionCode) -> std::result::Result<(), ()> + 'static + Send,
    >(
        &self,
        slot_id: u8,
        endpoint_id: u8,
        cb: C,
    ) -> Result<()> {
        self.slots[slot_id as usize - 1].reset_endpoint(self.fail_handle.clone(), endpoint_id, cb)
    }
}

// Usb port id. Valid ids starts from 1, to MAX_PORTS.
struct PortId(Mutex<u8>);

impl PortId {
    fn new() -> Self {
        PortId(Mutex::new(0))
    }

    fn set(&self, value: u8) -> Result<()> {
        if !(1..=MAX_PORTS).contains(&value) {
            return Err(Error::BadPortId(value));
        }
        *self.0.lock() = value;
        Ok(())
    }

    fn reset(&self) {
        *self.0.lock() = 0;
    }

    fn get(&self) -> Result<u8> {
        let val = *self.0.lock();
        if val == 0 {
            return Err(Error::BadPortId(val));
        }
        Ok(val)
    }
}

pub struct DeviceSlot {
    slot_id: u8,
    port_id: PortId, // Valid port id starts from 1, to MAX_PORTS.
    dcbaap: Register<u64>,
    hub: Arc<UsbHub>,
    interrupter: Arc<Mutex<Interrupter>>,
    event_loop: Arc<EventLoop>,
    mem: GuestMemory,
    enabled: AtomicBool,
    transfer_ring_controllers: Mutex<Vec<Option<TransferRingControllers>>>,
    /// How often the host was asked to free an endpoint's streams; the tests have no backend
    /// device to observe the call on.
    #[cfg(test)]
    host_streams_freed: std::sync::atomic::AtomicUsize,
}

impl DeviceSlot {
    /// Create a new device slot.
    pub fn new(
        slot_id: u8,
        dcbaap: Register<u64>,
        hub: Arc<UsbHub>,
        interrupter: Arc<Mutex<Interrupter>>,
        event_loop: Arc<EventLoop>,
        mem: GuestMemory,
    ) -> Self {
        let mut transfer_ring_controllers = Vec::new();
        transfer_ring_controllers.resize_with(TRANSFER_RING_CONTROLLERS_INDEX_END, || None);
        DeviceSlot {
            slot_id,
            port_id: PortId::new(),
            #[cfg(test)]
            host_streams_freed: std::sync::atomic::AtomicUsize::new(0),
            dcbaap,
            hub,
            interrupter,
            event_loop,
            mem,
            enabled: AtomicBool::new(false),
            transfer_ring_controllers: Mutex::new(transfer_ring_controllers),
        }
    }

    /// The ring behind endpoint index `i` (DCI - 1) and `stream_id`. `None` when the endpoint is
    /// not configured, the stream id does not fit the endpoint (spec 4.12.2: it is 0 on an
    /// endpoint without streams and 1..2^(MaxPStreams+1)-1 on one with them), or the Stream
    /// Context it names is Not Valid.
    fn get_trc(&self, i: usize, stream_id: u16) -> Option<Arc<TransferRingController>> {
        let trcs = self.transfer_ring_controllers.lock();
        match &trcs[i] {
            Some(TransferRingControllers::Endpoint(trc)) => {
                if stream_id != 0 {
                    warn!(
                        "device slot {}: endpoint {} has no streams, ignoring stream id {}",
                        self.slot_id,
                        i + 1,
                        stream_id
                    );
                    return None;
                }
                Some(trc.clone())
            }
            Some(TransferRingControllers::Stream(trcs)) => {
                let index = stream_id as usize;
                if index == 0 || index > trcs.len() {
                    warn!(
                        "device slot {}: endpoint {} has {} streams, ignoring stream id {}",
                        self.slot_id,
                        i + 1,
                        trcs.len(),
                        stream_id
                    );
                    return None;
                }
                match &trcs[index - 1] {
                    Some(trc) => Some(trc.clone()),
                    None => {
                        // Windows may probe a stream it never opened; nothing to run.
                        debug!(
                            "device slot {}: endpoint {} stream {} is Not Valid",
                            self.slot_id,
                            i + 1,
                            stream_id
                        );
                        None
                    }
                }
            }
            None => None,
        }
    }

    fn get_trcs(&self, i: usize) -> Option<TransferRingControllers> {
        let trcs = self.transfer_ring_controllers.lock();
        trcs[i].clone()
    }

    fn set_trcs(&self, i: usize, trc: Option<TransferRingControllers>) {
        let old = {
            let mut trcs = self.transfer_ring_controllers.lock();
            std::mem::replace(&mut trcs[i], trc)
        };
        // The old ring goes with the lock released: its drop takes it off the event loop and
        // runs whatever stop callback it still held, and neither may find this slot locked.
        drop(old);
    }

    fn trc_len(&self) -> usize {
        self.transfer_ring_controllers.lock().len()
    }

    /// The arguments are identical to the fields in each doorbell register. The
    /// target value:
    /// 1: Reserved
    /// 2: Control endpoint
    /// 3: Endpoint 1 out
    /// 4: Endpoint 1 in
    /// 5: Endpoint 2 out
    /// ...
    /// 32: Endpoint 15 in
    ///
    /// Steam ID will be useful when host controller support streams.
    /// The stream ID must be zero for endpoints that do not have streams
    /// configured.
    /// This function will return false if it fails to trigger transfer ring start.
    pub fn ring_doorbell(&self, target: u8, stream_id: u16) -> Result<bool> {
        if !valid_endpoint_id(target) {
            error!(
                "device slot {}: Invalid target written to doorbell register. target: {}",
                self.slot_id, target
            );
            return Ok(false);
        }
        xhci_trace!(
            "device slot {}: ring_doorbell target = {} stream_id = {}",
            self.slot_id,
            target,
            stream_id
        );
        // See DCI in spec.
        let endpoint_index = (target - 1) as usize;
        if self.get_trcs(endpoint_index).is_none() {
            error!("Device endpoint is not inited");
            return Ok(false);
        }
        let transfer_ring_controller = match self.get_trc(endpoint_index, stream_id) {
            Some(tr) => tr,
            // A stream id that does not fit the endpoint, or names a Not Valid Stream Context:
            // already logged by get_trc, nothing to run.
            None => return Ok(false),
        };
        let mut context = self.get_device_context()?;
        let endpoint_state = context.endpoint_context[endpoint_index]
            .get_endpoint_state()
            .map_err(Error::GetEndpointState)?;
        if endpoint_state == EndpointState::Running || endpoint_state == EndpointState::Stopped {
            if endpoint_state == EndpointState::Stopped {
                context.endpoint_context[endpoint_index].set_endpoint_state(EndpointState::Running);
                self.set_device_context(context)?;
            }
            // The endpoint is started; run its rings. On an endpoint with streams the
            // doorbell's stream id is a hint (spec 4.12.2): hardware brought back to running
            // by any doorbell serves whichever stream the device ERDYs, so every populated
            // stream ring runs again -- not only the addressed one. A Stop Endpoint rewound
            // each ring whose TD it cancelled to that TD but reported only one of them
            // (4.12.1.1: one stream is current); the guest re-arms the other TD without ever
            // addressing that stream in a doorbell (run5: Windows' UASPStor after a cold-attach
            // stop, stranding the TD ~16 s until its request timeout). An empty ring parks
            // again at once; a rewound one re-executes its TD, and the fresh backend transfer
            // fetches the data the device still holds for that stream. A halted endpoint is
            // not restarted here (the state check above).
            match self.get_trcs(endpoint_index) {
                Some(TransferRingControllers::Stream(trcs)) => {
                    let mut restarted = 0usize;
                    for trc in trcs.iter().flatten() {
                        trc.start();
                        restarted += 1;
                    }
                    debug!(
                        "xhci: slot {} ep {}: doorbell stream {} restarts {} stream rings",
                        self.slot_id, target, stream_id, restarted
                    );
                }
                _ => transfer_ring_controller.start(),
            }
        } else {
            error!("doorbell rung when endpoint state is {:?}", endpoint_state);
        }
        Ok(true)
    }

    /// Enable the slot. This function returns false if it's already enabled.
    pub fn enable(&self) -> bool {
        let was_already_enabled = self.enabled.swap(true, Ordering::SeqCst);
        if was_already_enabled {
            error!("device slot is already enabled");
        }
        !was_already_enabled
    }

    /// Disable this device slot. If the slot is not enabled, callback will be invoked immediately
    /// with error. Otherwise, callback will be invoked when all trc is stopped.
    pub fn disable<C: FnMut(TrbCompletionCode) -> std::result::Result<(), ()> + 'static + Send>(
        fail_handle: Arc<dyn FailHandle>,
        slot: &Arc<DeviceSlot>,
        mut callback: C,
    ) -> Result<()> {
        if slot.enabled.load(Ordering::SeqCst) {
            let slot_weak = Arc::downgrade(slot);
            let auto_callback =
                RingBufferStopCallback::new(fallible_closure(fail_handle, move || {
                    // Slot should still be alive when the callback is invoked. If it's not, there
                    // must be a bug somewhere.
                    let slot = slot_weak.upgrade().ok_or(Error::WeakReferenceUpgrade)?;
                    let mut device_context = slot.get_device_context()?;
                    device_context
                        .slot_context
                        .set_slot_state(DeviceSlotState::DisabledOrEnabled);
                    slot.set_device_context(device_context)?;
                    slot.reset();
                    debug!(
                        "device slot {}: all trc disabled, sending trb",
                        slot.slot_id
                    );
                    callback(TrbCompletionCode::Success).map_err(|_| Error::CallbackFailed)
                }));
            slot.stop_all_trc(auto_callback);
            Ok(())
        } else {
            callback(TrbCompletionCode::SlotNotEnabledError).map_err(|_| Error::CallbackFailed)
        }
    }

    // Assigns the device address and initializes slot and endpoint 0 context.
    pub fn set_address(
        self: &Arc<Self>,
        trb: &AddressDeviceCommandTrb,
    ) -> Result<TrbCompletionCode> {
        if !self.enabled.load(Ordering::SeqCst) {
            error!(
                "trying to set address to a disabled device slot {}",
                self.slot_id
            );
            return Ok(TrbCompletionCode::SlotNotEnabledError);
        }
        let device_context = self.get_device_context()?;
        let state = device_context
            .slot_context
            .get_slot_state()
            .map_err(Error::GetSlotContextState)?;
        match state {
            DeviceSlotState::DisabledOrEnabled => {}
            DeviceSlotState::Default if !trb.get_block_set_address_request() => {}
            _ => {
                error!("slot {} has unexpected slot state", self.slot_id);
                return Ok(TrbCompletionCode::ContextStateError);
            }
        }

        // Copy all fields of the slot context and endpoint 0 context from the input context
        // to the output context.
        let input_context_ptr = GuestAddress(trb.get_input_context_pointer());
        // Copy slot context.
        self.copy_context(input_context_ptr, 0)?;
        // Copy control endpoint context.
        self.copy_context(input_context_ptr, 1)?;

        // Read back device context.
        let mut device_context = self.get_device_context()?;
        let port_id = device_context.slot_context.get_root_hub_port_number();
        self.port_id.set(port_id)?;
        debug!(
            "port id {} is assigned to slot id {}",
            port_id, self.slot_id
        );

        // Initialize the control endpoint. Endpoint id = 1.
        let trc = TransferRingController::new(
            self.mem.clone(),
            self.hub.get_port(port_id).ok_or(Error::GetPort(port_id))?,
            self.event_loop.clone(),
            self.interrupter.clone(),
            self.slot_id,
            1,
            Arc::downgrade(self),
            None,
        )
        .map_err(Error::CreateTransferController)?;
        self.set_trcs(0, Some(TransferRingControllers::Endpoint(trc)));

        // Assign slot ID as device address if block_set_address_request is not set.
        if trb.get_block_set_address_request() {
            device_context
                .slot_context
                .set_slot_state(DeviceSlotState::Default);
        } else {
            let port = self.hub.get_port(port_id).ok_or(Error::GetPort(port_id))?;
            match port.backend_device().as_mut() {
                Some(backend) => {
                    backend.lock().set_address(self.slot_id as u32);
                }
                None => {
                    return Ok(TrbCompletionCode::TransactionError);
                }
            }

            device_context
                .slot_context
                .set_usb_device_address(self.slot_id);
            device_context
                .slot_context
                .set_slot_state(DeviceSlotState::Addressed);
        }

        // TODO(jkwang) trc should always exists. Fix this.
        self.get_trc(0, 0)
            .ok_or(Error::GetTrc(0))?
            .set_dequeue_pointer(
                device_context.endpoint_context[0]
                    .get_tr_dequeue_pointer()
                    .get_gpa(),
            );

        self.get_trc(0, 0)
            .ok_or(Error::GetTrc(0))?
            .set_consumer_cycle_state(device_context.endpoint_context[0].get_dequeue_cycle_state());

        // Setting endpoint 0 to running
        device_context.endpoint_context[0].set_endpoint_state(EndpointState::Running);
        self.set_device_context(device_context)?;
        Ok(TrbCompletionCode::Success)
    }

    // Adds or drops multiple endpoints in the device slot.
    pub fn configure_endpoint(
        self: &Arc<Self>,
        trb: &ConfigureEndpointCommandTrb,
    ) -> Result<TrbCompletionCode> {
        // Spec 4.6.6: the command is only defined for a slot in the Addressed or Configured
        // state.
        let slot_state = self
            .get_device_context()?
            .slot_context
            .get_slot_state()
            .map_err(Error::GetSlotContextState)?;
        if slot_state != DeviceSlotState::Addressed && slot_state != DeviceSlotState::Configured {
            warn!(
                "device slot {}: configure endpoint in slot state {:?}: {:?}",
                self.slot_id,
                slot_state,
                TrbCompletionCode::ContextStateError
            );
            return Ok(TrbCompletionCode::ContextStateError);
        }
        let input_context_ptr = GuestAddress(trb.get_input_context_pointer());
        // An Input Context the guest did not back with memory is the guest's mistake, not a
        // host fault: a Parameter Error, and the command ring goes on.
        let unbacked_input_context = |e: Error| -> Result<TrbCompletionCode> {
            warn!(
                "device slot {}: cannot read the input context at {:#x} ({}): {:?}",
                self.slot_id,
                input_context_ptr.0,
                e,
                TrbCompletionCode::ParameterError
            );
            Ok(TrbCompletionCode::ParameterError)
        };
        let input_control_context = if trb.get_deconfigure() {
            // From section 4.6.6 of the xHCI spec:
            // Setting the deconfigure (DC) flag to '1' in the Configure Endpoint Command
            // TRB is equivalent to setting Input Context Drop Context flags 2-31 to '1'
            // and Add Context 2-31 flags to '0'.
            let mut c = InputControlContext::new();
            c.set_add_context_flags(0);
            c.set_drop_context_flags(0xfffffffc);
            c
        } else {
            match self.mem.read_obj_from_addr(input_context_ptr) {
                Ok(c) => c,
                Err(e) => return unbacked_input_context(Error::ReadGuestMemory(e)),
            }
        };

        // Spec 4.6.6: a rejected command leaves the Output Device Context as it was. So every
        // Input Endpoint Context this command adds is read and checked before any endpoint is
        // dropped, any context copied or any host stream allocated; after this loop the only
        // thing left to fail is the host itself.
        let mut to_add = Vec::new();
        for device_context_index in 1..DCI_INDEX_END {
            if !input_control_context.add_context_flag(device_context_index) {
                continue;
            }
            let endpoint_context =
                match self.read_input_endpoint_context(input_context_ptr, device_context_index) {
                    Ok(endpoint_context) => endpoint_context,
                    Err(e) => return unbacked_input_context(e),
                };
            match self.check_endpoint_context(device_context_index, &endpoint_context) {
                Ok(stream_contexts) => {
                    to_add.push((device_context_index, endpoint_context, stream_contexts))
                }
                Err(code) => return Ok(code),
            }
        }

        // Drops and adds on different endpoints are independent, and on the same endpoint the
        // drop comes first (Linux issues Drop + Add for a stream endpoint it reconfigures).
        for device_context_index in 1..DCI_INDEX_END {
            if input_control_context.drop_context_flag(device_context_index) {
                self.drop_one_endpoint(device_context_index)?;
            }
        }
        let mut added = Vec::new();
        for (device_context_index, endpoint_context, stream_contexts) in to_add {
            let code = self.add_one_endpoint(
                device_context_index,
                endpoint_context,
                stream_contexts.as_deref(),
            )?;
            if code != TrbCompletionCode::Success {
                // The host would not give this endpoint its streams. The rejected endpoint's
                // Output Endpoint Context was never written; take back what this command added
                // and keep the slot state. An endpoint this command dropped stays dropped: its
                // rings are gone, and the guest's next Configure Endpoint puts them back.
                for dci in added {
                    self.drop_one_endpoint(dci)?;
                }
                return Ok(code);
            }
            added.push(device_context_index);
        }

        if trb.get_deconfigure() {
            self.set_state(DeviceSlotState::Addressed)?;
        } else {
            self.set_state(DeviceSlotState::Configured)?;
        }
        Ok(TrbCompletionCode::Success)
    }

    // Evaluates the device context by reading new values for certain fields of
    // the slot context and/or control endpoint context.
    pub fn evaluate_context(&self, trb: &EvaluateContextCommandTrb) -> Result<TrbCompletionCode> {
        if !self.enabled.load(Ordering::SeqCst) {
            return Ok(TrbCompletionCode::SlotNotEnabledError);
        }
        // TODO(jkwang) verify this
        // The spec has multiple contradictions about validating context parameters in sections
        // 4.6.7, 6.2.3.3. To keep things as simple as possible we do no further validation here.
        let input_control_context: InputControlContext = self
            .mem
            .read_obj_from_addr(GuestAddress(trb.get_input_context_pointer()))
            .map_err(Error::ReadGuestMemory)?;

        let mut device_context = self.get_device_context()?;
        if input_control_context.add_context_flag(0) {
            let input_slot_context: SlotContext = self
                .mem
                .read_obj_from_addr(GuestAddress(
                    trb.get_input_context_pointer() + DEVICE_CONTEXT_ENTRY_SIZE as u64,
                ))
                .map_err(Error::ReadGuestMemory)?;
            device_context
                .slot_context
                .set_interrupter_target(input_slot_context.get_interrupter_target());

            device_context
                .slot_context
                .set_max_exit_latency(input_slot_context.get_max_exit_latency());
        }

        // From 6.2.3.3: "Endpoint Contexts 2 throught 31 shall not be evaluated by the Evaluate
        // Context Command".
        if input_control_context.add_context_flag(1) {
            let ep0_context: EndpointContext = self
                .mem
                .read_obj_from_addr(GuestAddress(
                    trb.get_input_context_pointer() + 2 * DEVICE_CONTEXT_ENTRY_SIZE as u64,
                ))
                .map_err(Error::ReadGuestMemory)?;
            device_context.endpoint_context[0]
                .set_max_packet_size(ep0_context.get_max_packet_size());
        }
        self.set_device_context(device_context)?;
        Ok(TrbCompletionCode::Success)
    }

    /// Reset the device slot to default state and deconfigures all but the
    /// control endpoint.
    pub fn reset_slot<
        C: FnMut(TrbCompletionCode) -> std::result::Result<(), ()> + 'static + Send,
    >(
        fail_handle: Arc<dyn FailHandle>,
        slot: &Arc<DeviceSlot>,
        mut callback: C,
    ) -> Result<()> {
        let weak_s = Arc::downgrade(slot);
        let auto_callback =
            RingBufferStopCallback::new(fallible_closure(fail_handle, move || -> Result<()> {
                let s = weak_s.upgrade().ok_or(Error::WeakReferenceUpgrade)?;
                for i in FIRST_TRANSFER_ENDPOINT_DCI..DCI_INDEX_END {
                    s.drop_one_endpoint(i)?;
                }
                let mut ctx = s.get_device_context()?;
                ctx.slot_context.set_slot_state(DeviceSlotState::Default);
                ctx.slot_context.set_context_entries(1);
                ctx.slot_context.set_root_hub_port_number(0);
                s.set_device_context(ctx)?;
                callback(TrbCompletionCode::Success).map_err(|_| Error::CallbackFailed)?;
                Ok(())
            }));
        slot.stop_all_trc(auto_callback);
        Ok(())
    }

    /// Stop all transfer ring controllers.
    pub fn stop_all_trc(&self, auto_callback: RingBufferStopCallback) {
        for i in 0..self.trc_len() {
            if let Some(trcs) = self.get_trcs(i) {
                match trcs {
                    TransferRingControllers::Endpoint(trc) => {
                        trc.stop(auto_callback.clone());
                    }
                    TransferRingControllers::Stream(trcs) => {
                        for trc in trcs.iter().flatten() {
                            trc.stop(auto_callback.clone());
                        }
                    }
                }
            }
        }
    }

    /// Writes a stream ring's position back to its Stream Context, the way the controller does
    /// whenever a stream leaves the Move Data state (spec 4.12.1.1). Only that one 16-byte entry
    /// is touched: the guest owns the rest of the array, and the entry's Stopped EDTLA and
    /// reserved dwords stay as they are.
    fn write_back_stream_context(
        &self,
        stream_context_array_addr: GuestAddress,
        stream_id: u16,
        dequeue_pointer: GuestAddress,
        dequeue_cycle_state: bool,
    ) -> Result<()> {
        write_stream_context(
            &self.mem,
            stream_context_array_addr,
            stream_id,
            dequeue_pointer,
            dequeue_cycle_state,
            None,
        )
    }

    /// Stop an endpoint.
    ///
    /// The endpoint state goes to Stopped here. The ring position -- the Endpoint Context's TR
    /// Dequeue Pointer and DCS, or those of each populated Stream Context plus the Stopped EDTLA
    /// of a stream whose ring stopped with a descriptor in progress -- is written from the stop
    /// callback, once every ring has actually stopped and been left at the descriptor it was
    /// executing (spec 4.6.9), right before the Command Completion. Read before the stop, as it
    /// used to be, the pointer was the one past that descriptor.
    ///
    /// On a stream endpoint the command generates exactly ONE Stopped Transfer Event however
    /// many stream rings had a TD in flight (spec 4.12.1.1: the endpoint has one current
    /// stream, and 4.6.9 promises one event for the TD in progress): the rings share one claim
    /// on it, the first to finish its stop with a cancelled TD emits, and every other rewinds
    /// silently -- its Stream Context still gets its pointer and DCS, but no event and no
    /// Stopped EDTLA. Windows' USBXHCI discards a second Stopped event as a duplicate and
    /// strands the URB it reported (run4: 16 s on the UAS data-in endpoint's cold attach).
    pub fn stop_endpoint<
        C: FnMut(TrbCompletionCode) -> std::result::Result<(), ()> + 'static + Send,
    >(
        &self,
        fail_handle: Arc<dyn FailHandle>,
        endpoint_id: u8,
        mut cb: C,
    ) -> Result<()> {
        if !valid_endpoint_id(endpoint_id) {
            error!("trb indexing wrong endpoint id");
            return cb(TrbCompletionCode::TrbError).map_err(|_| Error::CallbackFailed);
        }
        let index = endpoint_id - 1;
        let mut device_context = self.get_device_context()?;
        let endpoint_context = &mut device_context.endpoint_context[index as usize];
        // Spec 4.6.9: the command is valid only on a Running endpoint; hardware answers any
        // other state with a Context State Error. Without this a second Stop Endpoint issued
        // before the first completes would re-arm a fresh claim onto rings still Stopping under
        // the first command and could report a second Stopped event.
        if endpoint_context
            .get_endpoint_state()
            .map_err(Error::GetEndpointState)?
            != EndpointState::Running
        {
            // Linux stops already-stopped endpoints legitimately; the Context State Error
            // completion below is the guest-visible answer.
            debug!("endpoint at index {} is not running", index);
            return cb(TrbCompletionCode::ContextStateError).map_err(|_| Error::CallbackFailed);
        }
        // The callback's last clone lives until the endpoint state below is in guest memory, so
        // a ring that stops at once answers after it, like one that stops later. The closures
        // hold their rings weakly: a ring keeps its stop callback until it parks, and must not
        // keep itself alive through it. A ring that is gone by then had its slot reset from
        // under the stop, and the guest gave the context up with it.
        let _stop = match self.get_trcs(index as usize) {
            Some(TransferRingControllers::Endpoint(trc)) => {
                let mem = self.mem.clone();
                let device_context_addr = self.get_device_context_addr()?;
                let ring = Arc::downgrade(&trc);
                let auto_cb = RingBufferStopCallback::new(fallible_closure(
                    fail_handle,
                    move || -> Result<()> {
                        if let Some(trc) = ring.upgrade() {
                            write_endpoint_context_position(
                                &mem,
                                device_context_addr,
                                endpoint_id,
                                trc.get_dequeue_pointer(),
                                trc.get_consumer_cycle_state(),
                            )?;
                        }
                        cb(TrbCompletionCode::Success).map_err(|_| Error::CallbackFailed)
                    },
                ));
                trc.stop(auto_cb.clone());
                Some(auto_cb)
            }
            Some(TransferRingControllers::Stream(trcs)) => {
                let stream_context_array_addr = endpoint_context.get_tr_dequeue_pointer().get_gpa();
                let mem = self.mem.clone();
                let rings: Vec<(u16, Weak<TransferRingController>)> = trcs
                    .iter()
                    .enumerate()
                    .filter_map(|(i, trc)| {
                        trc.as_ref()
                            .map(|trc| ((i + 1) as u16, Arc::downgrade(trc)))
                    })
                    .collect();
                let auto_cb = RingBufferStopCallback::new(fallible_closure(
                    fail_handle,
                    move || -> Result<()> {
                        for (stream_id, ring) in &rings {
                            if let Some(trc) = ring.upgrade() {
                                write_stream_context(
                                    &mem,
                                    stream_context_array_addr,
                                    *stream_id,
                                    trc.get_dequeue_pointer(),
                                    trc.get_consumer_cycle_state(),
                                    // The Stopped EDTLA goes to the one stream whose ring
                                    // emitted the Stopped event (6.2.4.1: the stream that left
                                    // the Move Data state mid-TD); a silently rewound ring's
                                    // entry keeps the guest's value.
                                    trc.stopped_td()
                                        .filter(|td| td.reported)
                                        .map(|td| td.bytes),
                                )?;
                            }
                        }
                        cb(TrbCompletionCode::Success).map_err(|_| Error::CallbackFailed)
                    },
                ));
                // This command's one Stopped event, whichever ring finishes with a cancelled
                // TD first: set on every ring before its stop, so no ring can park ahead of
                // its own claim.
                let stop_event_claim = Arc::new(AtomicBool::new(false));
                for trc in trcs.iter().flatten() {
                    trc.set_stop_event_claim(stop_event_claim.clone());
                    trc.stop(auto_cb.clone());
                }
                Some(auto_cb)
            }
            None => {
                error!("endpoint at index {} is not started", index);
                cb(TrbCompletionCode::ContextStateError).map_err(|_| Error::CallbackFailed)?;
                None
            }
        };
        endpoint_context.set_endpoint_state(EndpointState::Stopped);
        self.set_device_context(device_context)?;
        Ok(())
    }

    /// Reset an endpoint.
    pub fn reset_endpoint<
        C: FnMut(TrbCompletionCode) -> std::result::Result<(), ()> + 'static + Send,
    >(
        &self,
        fail_handle: Arc<dyn FailHandle>,
        endpoint_id: u8,
        mut cb: C,
    ) -> Result<()> {
        if !valid_endpoint_id(endpoint_id) {
            error!("trb indexing wrong endpoint id");
            return cb(TrbCompletionCode::TrbError).map_err(|_| Error::CallbackFailed);
        }
        let index = endpoint_id - 1;
        let mut device_context = self.get_device_context()?;
        let endpoint_context = &mut device_context.endpoint_context[index as usize];
        if endpoint_context
            .get_endpoint_state()
            .map_err(Error::GetEndpointState)?
            != EndpointState::Halted
        {
            error!("endpoint at index {} is not halted", index);
            return cb(TrbCompletionCode::ContextStateError).map_err(|_| Error::CallbackFailed);
        }
        match self.get_trcs(index as usize) {
            Some(TransferRingControllers::Endpoint(trc)) => {
                let auto_cb = RingBufferStopCallback::new(fallible_closure(
                    fail_handle,
                    move || -> Result<()> {
                        cb(TrbCompletionCode::Success).map_err(|_| Error::CallbackFailed)
                    },
                ));
                trc.stop(auto_cb);
                let dequeue_pointer = trc.get_dequeue_pointer();
                let dcs = trc.get_consumer_cycle_state();
                endpoint_context.set_tr_dequeue_pointer(DequeuePtr::new(dequeue_pointer));
                endpoint_context.set_dequeue_cycle_state(dcs);
            }
            Some(TransferRingControllers::Stream(trcs)) => {
                let stream_context_array_addr = endpoint_context.get_tr_dequeue_pointer().get_gpa();
                let auto_cb = RingBufferStopCallback::new(fallible_closure(
                    fail_handle,
                    move || -> Result<()> {
                        cb(TrbCompletionCode::Success).map_err(|_| Error::CallbackFailed)
                    },
                ));
                for (i, trc) in trcs.iter().enumerate() {
                    let trc = match trc {
                        Some(trc) => trc,
                        None => continue,
                    };
                    let dequeue_pointer = trc.get_dequeue_pointer();
                    let dcs = trc.get_consumer_cycle_state();
                    trc.stop(auto_cb.clone());
                    self.write_back_stream_context(
                        stream_context_array_addr,
                        (i + 1) as u16,
                        dequeue_pointer,
                        dcs,
                    )?;
                }
            }
            None => {
                error!("endpoint at index {} is not started", index);
                cb(TrbCompletionCode::ContextStateError).map_err(|_| Error::CallbackFailed)?;
            }
        }
        endpoint_context.set_endpoint_state(EndpointState::Stopped);
        self.set_device_context(device_context)?;
        Ok(())
    }

    /// Set transfer ring dequeue pointer.
    ///
    /// The command carries the consumer cycle state to use at the new pointer (spec 6.4.3.9,
    /// DCS): software is free to point the ring anywhere, including past a link TRB it never let
    /// the controller walk, or at a freshly initialised ring, and the cycle it hands over is the
    /// only way to know which TRBs there are its. Keeping the old cycle state would make a valid
    /// TRB at the new pointer look unowned and the ring look empty.
    ///
    /// On an endpoint with streams the command targets one Stream Context (spec 4.6.10): the
    /// pointer, DCS and Stream Context Type go into entry `stream_id` of the array, and the
    /// Endpoint Context's TR Dequeue Pointer -- which is the array's address -- is left alone.
    /// Writing the ring pointer there instead, as this used to, made every later Stop Endpoint
    /// and halt read "stream contexts" out of the ring and write them back over its TRBs. A
    /// Stream Context the guest had left Not Valid and now sets with type 1 gets its ring here:
    /// that is how software (re)initialises a stream.
    pub fn set_tr_dequeue_ptr(
        self: &Arc<Self>,
        endpoint_id: u8,
        stream_id: u16,
        stream_context_type: u8,
        ptr: u64,
        dequeue_cycle_state: bool,
    ) -> Result<TrbCompletionCode> {
        if !valid_endpoint_id(endpoint_id) {
            error!("trb indexing wrong endpoint id");
            return Ok(TrbCompletionCode::TrbError);
        }
        let index = (endpoint_id - 1) as usize;
        let endpoint_context = self.get_device_context()?.endpoint_context[index];
        if endpoint_context.get_max_primary_streams() > 0 {
            return self.set_stream_tr_dequeue_ptr(
                endpoint_id,
                &endpoint_context,
                stream_id,
                stream_context_type,
                GuestAddress(ptr),
                dequeue_cycle_state,
            );
        }
        match self.get_trc(index, stream_id) {
            Some(trc) => {
                trc.set_dequeue_pointer(GuestAddress(ptr));
                trc.set_consumer_cycle_state(dequeue_cycle_state);
                let mut ctx = self.get_device_context()?;
                ctx.endpoint_context[index]
                    .set_tr_dequeue_pointer(DequeuePtr::new(GuestAddress(ptr)));
                ctx.endpoint_context[index].set_dequeue_cycle_state(dequeue_cycle_state);
                self.set_device_context(ctx)?;
                Ok(TrbCompletionCode::Success)
            }
            None => {
                error!("set tr dequeue ptr failed due to no trc started");
                Ok(TrbCompletionCode::ContextStateError)
            }
        }
    }

    /// Set TR Dequeue Pointer for `stream_id` of an endpoint with streams; see
    /// `set_tr_dequeue_ptr`.
    fn set_stream_tr_dequeue_ptr(
        self: &Arc<Self>,
        endpoint_id: u8,
        endpoint_context: &EndpointContext,
        stream_id: u16,
        stream_context_type: u8,
        ptr: GuestAddress,
        dequeue_cycle_state: bool,
    ) -> Result<TrbCompletionCode> {
        let index = (endpoint_id - 1) as usize;
        let reject = |code: TrbCompletionCode, why: &str| -> Result<TrbCompletionCode> {
            warn!(
                "device slot {}: endpoint {} stream {}: set tr dequeue pointer: {}: {:?}",
                self.slot_id, endpoint_id, stream_id, why, code
            );
            Ok(code)
        };
        let trcs = match self.get_trcs(index) {
            Some(TransferRingControllers::Stream(trcs)) => trcs,
            _ => {
                return reject(
                    TrbCompletionCode::ContextStateError,
                    "endpoint not configured",
                )
            }
        };
        if stream_id == 0 || stream_id as usize > trcs.len() {
            return reject(
                TrbCompletionCode::TrbError,
                &format!("endpoint has {} streams", trcs.len()),
            );
        }
        let endpoint_state = endpoint_context
            .get_endpoint_state()
            .map_err(Error::GetEndpointState)?;
        if endpoint_state != EndpointState::Stopped && endpoint_state != EndpointState::Error {
            return reject(
                TrbCompletionCode::ContextStateError,
                &format!("endpoint state is {:?}", endpoint_state),
            );
        }

        let stream_context_array_addr = endpoint_context.get_tr_dequeue_pointer().get_gpa();
        let addr = stream_context_array_addr
            .checked_add(stream_id as u64 * STREAM_CONTEXT_SIZE as u64)
            .ok_or(Error::BadDeviceContextAddr(stream_context_array_addr))?;
        let mut stream_context: StreamContext = self
            .mem
            .read_obj_from_addr(addr)
            .map_err(Error::ReadGuestMemory)?;
        stream_context.set_tr_dequeue_pointer(DequeuePtr::new(ptr));
        stream_context.set_dequeue_cycle_state(dequeue_cycle_state);
        stream_context.set_stream_context_type(stream_context_type);
        self.mem
            .write_obj_at_addr(stream_context, addr)
            .map_err(Error::WriteGuestMemory)?;

        match &trcs[stream_id as usize - 1] {
            Some(_) if stream_context_type != 1 => {
                // The guest took the stream back: its Stream Context now says Not Valid (Table
                // 6-13), so its ring goes and a doorbell on it is ignored like on any other Not
                // Valid stream.
                debug!(
                    "device slot {}: endpoint {} stream {} made Not Valid (stream context type {}) by set tr dequeue pointer",
                    self.slot_id, endpoint_id, stream_id, stream_context_type
                );
                self.set_stream_trc(index, stream_id, None);
            }
            Some(trc) => {
                trc.set_dequeue_pointer(ptr);
                trc.set_consumer_cycle_state(dequeue_cycle_state);
            }
            None if stream_context_type == 1 => {
                debug!(
                    "device slot {}: endpoint {} stream {} initialised at {:#x} by set tr dequeue pointer",
                    self.slot_id, endpoint_id, stream_id, ptr.0
                );
                let trc =
                    self.create_stream_trc(endpoint_id, stream_id, ptr, dequeue_cycle_state)?;
                self.set_stream_trc(index, stream_id, Some(trc));
            }
            None => {
                debug!(
                    "device slot {}: endpoint {} stream {} stays Not Valid (stream context type {})",
                    self.slot_id, endpoint_id, stream_id, stream_context_type
                );
            }
        }
        Ok(TrbCompletionCode::Success)
    }

    /// Puts a ring (or none) behind stream `stream_id` of endpoint index `i`, which must hold
    /// streams.
    fn set_stream_trc(&self, i: usize, stream_id: u16, trc: Option<Arc<TransferRingController>>) {
        let mut trcs = self.transfer_ring_controllers.lock();
        if let Some(TransferRingControllers::Stream(trcs)) = &mut trcs[i] {
            trcs[stream_id as usize - 1] = trc;
        }
    }

    // Reset and reset_slot are different.
    // Reset_slot handles command ring `reset slot` command. It will reset the slot state.
    // Reset handles xhci reset. It will destroy everything.
    fn reset(&self) {
        for i in 0..self.trc_len() {
            // The host's streams go with the rings: left allocated, the next Configure
            // Endpoint's USBDEVFS_ALLOC_STREAMS fails with EINVAL and the guest never gets
            // its endpoint back after a controller reset or a slot disable.
            if let Some(TransferRingControllers::Stream(_)) = self.get_trcs(i) {
                self.free_host_streams((i + 1) as u8);
            }
            self.set_trcs(i, None);
        }
        debug!("resetting device slot {}!", self.slot_id);
        self.enabled.store(false, Ordering::SeqCst);
        self.port_id.reset();
    }

    /// A transfer ring controller for one stream of endpoint `device_context_index`, positioned
    /// where its Stream Context says.
    fn create_stream_trc(
        self: &Arc<Self>,
        device_context_index: u8,
        stream_id: u16,
        dequeue_pointer: GuestAddress,
        dequeue_cycle_state: bool,
    ) -> Result<Arc<TransferRingController>> {
        let trc = TransferRingController::new(
            self.mem.clone(),
            self.hub
                .get_port(self.port_id.get()?)
                .ok_or(Error::GetPort(self.port_id.get()?))?,
            self.event_loop.clone(),
            self.interrupter.clone(),
            self.slot_id,
            device_context_index,
            Arc::downgrade(self),
            Some(stream_id),
        )
        .map_err(Error::CreateTransferController)?;
        trc.set_dequeue_pointer(dequeue_pointer);
        trc.set_consumer_cycle_state(dequeue_cycle_state);
        Ok(trc)
    }

    /// One ring per initialised Stream Context; a Not Valid entry stays `None`.
    fn create_stream_trcs(
        self: &Arc<Self>,
        stream_contexts: &[Option<(GuestAddress, bool)>],
        device_context_index: u8,
    ) -> Result<TransferRingControllers> {
        let mut trcs = Vec::with_capacity(stream_contexts.len());
        // Stream ID 0 is reserved (xHCI spec Section 4.12.2): entry i is stream i + 1.
        for (i, stream_context) in stream_contexts.iter().enumerate() {
            trcs.push(match stream_context {
                Some((dequeue_pointer, dequeue_cycle_state)) => Some(self.create_stream_trc(
                    device_context_index,
                    (i + 1) as u16,
                    *dequeue_pointer,
                    *dequeue_cycle_state,
                )?),
                None => None,
            });
        }
        Ok(TransferRingControllers::Stream(trcs))
    }

    /// The usb endpoint address of a device context index: number in the low bits, bit 7 for
    /// IN.
    fn endpoint_address(device_context_index: u8) -> u8 {
        let mut endpoint_address = device_context_index / 2;
        if device_context_index % 2 == 1 {
            endpoint_address |= 1u8 << 7;
        }
        endpoint_address
    }

    /// Hands the host's streams for an endpoint back. A failure is logged and otherwise
    /// ignored: the device may simply be gone, or the slot may have lost its port to a
    /// controller reset the guest did not follow up, and there is nothing else to do with it.
    fn free_host_streams(&self, device_context_index: u8) {
        #[cfg(test)]
        self.host_streams_freed
            .fetch_add(1, std::sync::atomic::Ordering::SeqCst);
        let port_id = match self.port_id.get() {
            Ok(port_id) => port_id,
            Err(e) => {
                warn!(
                    "device slot {}: endpoint {}: no port to free host streams on: {}",
                    self.slot_id, device_context_index, e
                );
                return;
            }
        };
        if let Some(port) = self.hub.get_port(port_id) {
            if let Some(backend_device) = port.backend_device().as_mut() {
                if let Err(e) = backend_device
                    .lock()
                    .free_streams(Self::endpoint_address(device_context_index))
                {
                    warn!(
                        "device slot {}: endpoint {} failed to free host streams: {}",
                        self.slot_id, device_context_index, e
                    );
                }
            }
        }
    }

    /// The Input Endpoint Context of `device_context_index` in the Input Context at
    /// `input_context_ptr`.
    fn read_input_endpoint_context(
        &self,
        input_context_ptr: GuestAddress,
        device_context_index: u8,
    ) -> Result<EndpointContext> {
        self.mem
            .read_obj_from_addr(
                input_context_ptr
                    .checked_add(
                        (device_context_index as u64 + 1) * DEVICE_CONTEXT_ENTRY_SIZE as u64,
                    )
                    .ok_or(Error::BadInputContextAddr(input_context_ptr))?,
            )
            .map_err(Error::ReadGuestMemory)
    }

    /// What Configure Endpoint validates in an Input Endpoint Context (spec 4.6.6), before the
    /// command touches anything: a context the guest got wrong is a completion code. For an
    /// endpoint with streams the result carries its Stream Context Array as read now.
    fn check_endpoint_context(
        &self,
        device_context_index: u8,
        endpoint_context: &EndpointContext,
    ) -> std::result::Result<Option<Vec<Option<(GuestAddress, bool)>>>, TrbCompletionCode> {
        let reject = |code: TrbCompletionCode, why: &str| {
            warn!(
                "device slot {}: endpoint {}: {}: {:?}",
                self.slot_id, device_context_index, why, code
            );
            Err(code)
        };
        let endpoint_type = endpoint_context.get_endpoint_type();
        if endpoint_type == 0 {
            return reject(TrbCompletionCode::ParameterError, "EP Type is Not Valid");
        }
        let max_pstreams = endpoint_context.get_max_primary_streams();
        if max_pstreams == 0 {
            return Ok(None);
        }
        if !valid_max_pstreams(max_pstreams) {
            return reject(
                TrbCompletionCode::ParameterError,
                &format!("MaxPStreams {} exceeds MaxPSASize", max_pstreams),
            );
        }
        if endpoint_type != 2 && endpoint_type != 6 {
            // Stream is only supported on a bulk endpoint
            return reject(
                TrbCompletionCode::ParameterError,
                &format!("streams on EP Type {}", endpoint_type),
            );
        }
        if endpoint_context.get_linear_stream_array() != 1 {
            // We only support Linear Stream Context Array for now
            return reject(
                TrbCompletionCode::ParameterError,
                "secondary stream arrays (LSA = 0) are not supported",
            );
        }
        match read_stream_contexts(
            &self.mem,
            endpoint_context.get_tr_dequeue_pointer().get_gpa(),
            max_pstreams,
        ) {
            Ok(stream_contexts) => Ok(Some(stream_contexts)),
            Err(code) => reject(code, "cannot read the stream context array"),
        }
    }

    /// Adds an endpoint from an Input Endpoint Context `check_endpoint_context` passed, with
    /// `stream_contexts` for one with streams. The context reaches the Output Device Context
    /// only once the endpoint has its rings and its host streams (spec 4.6.6: a rejected command
    /// leaves the Output Device Context as it was); a host that will not give the streams is a
    /// completion code, `Err` is for the host side only.
    fn add_one_endpoint(
        self: &Arc<Self>,
        device_context_index: u8,
        mut endpoint_context: EndpointContext,
        stream_contexts: Option<&[Option<(GuestAddress, bool)>]>,
    ) -> Result<TrbCompletionCode> {
        xhci_trace!(
            "adding one endpoint, device context index {}",
            device_context_index
        );
        let transfer_ring_index = (device_context_index - 1) as usize;
        let tr_dequeue_pointer = endpoint_context.get_tr_dequeue_pointer().get_gpa();
        let endpoint_type = endpoint_context.get_endpoint_type();
        if let Some(TransferRingControllers::Stream(_)) = self.get_trcs(transfer_ring_index) {
            // Added again without a drop in between, with or without streams this time: the
            // host still holds the streams of the last time, and would answer a second
            // allocation with EINVAL.
            warn!(
                "device slot {}: endpoint {} already has host streams, freeing them first",
                self.slot_id, device_context_index
            );
            self.free_host_streams(device_context_index);
        }
        let trcs = match stream_contexts {
            Some(stream_contexts) => {
                let trcs = self.create_stream_trcs(stream_contexts, device_context_index)?;
                if let Some(port) = self.hub.get_port(self.port_id.get()?) {
                    if let Some(backend_device) = port.backend_device().as_mut() {
                        // One host stream per array entry; entry 0 is not a stream.
                        let streams = stream_contexts.len() as u16;
                        if let Err(e) = backend_device
                            .lock()
                            .alloc_streams(Self::endpoint_address(device_context_index), streams)
                        {
                            warn!(
                                "device slot {}: endpoint {}: cannot allocate {} host streams: {}: {:?}",
                                self.slot_id,
                                device_context_index,
                                streams,
                                e,
                                TrbCompletionCode::ResourceError
                            );
                            return Ok(TrbCompletionCode::ResourceError);
                        }
                    }
                }
                trcs
            }
            None => {
                let trc = TransferRingController::new(
                    self.mem.clone(),
                    self.hub
                        .get_port(self.port_id.get()?)
                        .ok_or(Error::GetPort(self.port_id.get()?))?,
                    self.event_loop.clone(),
                    self.interrupter.clone(),
                    self.slot_id,
                    device_context_index,
                    Arc::downgrade(self),
                    None,
                )
                .map_err(Error::CreateTransferController)?;
                trc.set_dequeue_pointer(tr_dequeue_pointer);
                trc.set_consumer_cycle_state(endpoint_context.get_dequeue_cycle_state());
                // Endpoint context types 1 (Isoch Out) and 5 (Isoch In): an isochronous ring has
                // to be drained ahead of the completions, or the stream underruns. Streams are
                // bulk only, so the branch above never needs this.
                trc.set_dequeue_all(endpoint_type == 1 || endpoint_type == 5);
                TransferRingControllers::Endpoint(trc)
            }
        };
        self.set_trcs(transfer_ring_index, Some(trcs));
        let mut device_context = self.get_device_context()?;
        endpoint_context.set_endpoint_state(EndpointState::Running);
        device_context.endpoint_context[transfer_ring_index] = endpoint_context;
        self.set_device_context(device_context)?;
        Ok(TrbCompletionCode::Success)
    }

    fn drop_one_endpoint(self: &Arc<Self>, device_context_index: u8) -> Result<()> {
        let endpoint_index = (device_context_index - 1) as usize;
        let mut device_context = self.get_device_context()?;
        let endpoint_context = &mut device_context.endpoint_context[endpoint_index];
        if endpoint_context.get_max_primary_streams() > 0 {
            self.free_host_streams(device_context_index);
        }
        self.set_trcs(endpoint_index, None);
        endpoint_context.set_endpoint_state(EndpointState::Disabled);
        self.set_device_context(device_context)
    }

    fn get_device_context(&self) -> Result<DeviceContext> {
        let ctx = self
            .mem
            .read_obj_from_addr(self.get_device_context_addr()?)
            .map_err(Error::ReadGuestMemory)?;
        Ok(ctx)
    }

    fn set_device_context(&self, device_context: DeviceContext) -> Result<()> {
        self.mem
            .write_obj_at_addr(device_context, self.get_device_context_addr()?)
            .map_err(Error::WriteGuestMemory)
    }

    fn copy_context(
        &self,
        input_context_ptr: GuestAddress,
        device_context_index: u8,
    ) -> Result<()> {
        // Note that it could be slot context or device context. They have the same size. Won't
        // make a difference here.
        let ctx: EndpointContext = self
            .mem
            .read_obj_from_addr(
                input_context_ptr
                    .checked_add(
                        (device_context_index as u64 + 1) * DEVICE_CONTEXT_ENTRY_SIZE as u64,
                    )
                    .ok_or(Error::BadInputContextAddr(input_context_ptr))?,
            )
            .map_err(Error::ReadGuestMemory)?;
        xhci_trace!("copy_context {:?}", ctx);
        let device_context_ptr = self.get_device_context_addr()?;
        self.mem
            .write_obj_at_addr(
                ctx,
                device_context_ptr
                    .checked_add(device_context_index as u64 * DEVICE_CONTEXT_ENTRY_SIZE as u64)
                    .ok_or(Error::BadDeviceContextAddr(device_context_ptr))?,
            )
            .map_err(Error::WriteGuestMemory)
    }

    fn get_device_context_addr(&self) -> Result<GuestAddress> {
        let addr: u64 = self
            .mem
            .read_obj_from_addr(GuestAddress(
                self.dcbaap.get_value() + size_of::<u64>() as u64 * self.slot_id as u64,
            ))
            .map_err(Error::ReadGuestMemory)?;
        Ok(GuestAddress(addr))
    }

    fn set_state(&self, state: DeviceSlotState) -> Result<()> {
        let mut ctx = self.get_device_context()?;
        ctx.slot_context.set_slot_state(state);
        self.set_device_context(ctx)
    }

    pub fn halt_endpoint(&self, endpoint_id: u8) -> Result<()> {
        if !valid_endpoint_id(endpoint_id) {
            return Err(Error::BadEndpointId(endpoint_id));
        }
        let index = endpoint_id - 1;
        let mut device_context = self.get_device_context()?;
        let endpoint_context = &mut device_context.endpoint_context[index as usize];
        // A halted endpoint processes no more TRBs until software resets it and rings the
        // doorbell again (spec 4.8.3): park the ring here, or the controller would carry on
        // executing whatever the guest had queued behind the failed TD and answer the retries the
        // guest issues after its Set TR Dequeue Pointer twice.
        match self.get_trcs(index as usize) {
            Some(trcs) => match trcs {
                TransferRingControllers::Endpoint(trc) => {
                    trc.halt();
                    endpoint_context
                        .set_tr_dequeue_pointer(DequeuePtr::new(trc.get_dequeue_pointer()));
                    endpoint_context.set_dequeue_cycle_state(trc.get_consumer_cycle_state());
                }
                TransferRingControllers::Stream(trcs) => {
                    let stream_context_array_addr =
                        endpoint_context.get_tr_dequeue_pointer().get_gpa();
                    for (i, trc) in trcs.iter().enumerate() {
                        let trc = match trc {
                            Some(trc) => trc,
                            None => continue,
                        };
                        trc.halt();
                        self.write_back_stream_context(
                            stream_context_array_addr,
                            (i + 1) as u16,
                            trc.get_dequeue_pointer(),
                            trc.get_consumer_cycle_state(),
                        )?;
                    }
                }
            },
            None => {
                error!("trc for endpoint {} not found", endpoint_id);
                return Err(Error::BadEndpointId(endpoint_id));
            }
        }
        endpoint_context.set_endpoint_state(EndpointState::Halted);
        self.set_device_context(device_context)?;
        Ok(())
    }
}

/// Writes a stream ring's position into entry `stream_id` of the Stream Context Array at
/// `stream_context_array_addr`, and its Stopped EDTLA when the ring stopped with a descriptor in
/// progress (spec 6.2.4.1). Only that one 16-byte entry is touched, and only those fields: the
/// guest owns the rest of the array, and an entry whose ring stopped with nothing in progress
/// keeps whatever Stopped EDTLA it had.
fn write_stream_context(
    mem: &GuestMemory,
    stream_context_array_addr: GuestAddress,
    stream_id: u16,
    dequeue_pointer: GuestAddress,
    dequeue_cycle_state: bool,
    stopped_edtla: Option<u32>,
) -> Result<()> {
    let addr = stream_context_array_addr
        .checked_add(stream_id as u64 * STREAM_CONTEXT_SIZE as u64)
        .ok_or(Error::BadDeviceContextAddr(stream_context_array_addr))?;
    let mut stream_context: StreamContext = mem
        .read_obj_from_addr(addr)
        .map_err(Error::ReadGuestMemory)?;
    stream_context.set_tr_dequeue_pointer(DequeuePtr::new(dequeue_pointer));
    stream_context.set_dequeue_cycle_state(dequeue_cycle_state);
    if let Some(stopped_edtla) = stopped_edtla {
        // A 24-bit field.
        stream_context.set_stopped_edtla(stopped_edtla & 0xff_ffff);
    }
    mem.write_obj_at_addr(stream_context, addr)
        .map_err(Error::WriteGuestMemory)
}

/// Writes a ring's position into the Endpoint Context of `device_context_index` in the Device
/// Context at `device_context_addr`, leaving the rest of that context as it is.
fn write_endpoint_context_position(
    mem: &GuestMemory,
    device_context_addr: GuestAddress,
    device_context_index: u8,
    dequeue_pointer: GuestAddress,
    dequeue_cycle_state: bool,
) -> Result<()> {
    let addr = device_context_addr
        .checked_add(device_context_index as u64 * DEVICE_CONTEXT_ENTRY_SIZE as u64)
        .ok_or(Error::BadDeviceContextAddr(device_context_addr))?;
    let mut endpoint_context: EndpointContext = mem
        .read_obj_from_addr(addr)
        .map_err(Error::ReadGuestMemory)?;
    endpoint_context.set_tr_dequeue_pointer(DequeuePtr::new(dequeue_pointer));
    endpoint_context.set_dequeue_cycle_state(dequeue_cycle_state);
    mem.write_obj_at_addr(endpoint_context, addr)
        .map_err(Error::WriteGuestMemory)
}

/// A device slot over guest memory, for this module's tests and the command ring's.
#[cfg(test)]
pub(crate) mod test_util {
    use std::sync::atomic::AtomicBool;
    use std::sync::atomic::Ordering;
    use std::thread::JoinHandle;

    use base::pagesize;
    use base::Event;

    use super::super::usb_hub::UsbPort;
    use super::super::xhci_abi::AddressedTrb;
    use super::super::xhci_abi::CommandCompletionEventTrb;
    use super::super::xhci_abi::EventRingSegmentTableEntry;
    use super::super::xhci_abi::NormalTrb;
    use super::super::xhci_abi::TransferDescriptor;
    use super::super::xhci_abi::TransferEventTrb;
    use super::super::xhci_abi::Trb;
    use super::super::xhci_abi::TrbCast;
    use super::super::xhci_abi::TrbType;
    use super::super::xhci_regs::init_xhci_mmio_space_and_regs;
    use super::super::xhci_transfer::XhciTransfer;
    use super::super::xhci_transfer::XhciTransferManager;
    use super::*;

    /// Event ring segment: 16 TRBs.
    pub const EVENT_RING: u64 = 0x100;
    pub const DCBAA: u64 = 0x200;
    pub const DEVICE_CONTEXT: u64 = 0x1000;
    pub const INPUT_CONTEXT: u64 = 0x1400;
    /// Room for the largest primary array this controller takes (16 entries).
    pub const STREAM_CONTEXT_ARRAY: u64 = 0x1900;
    /// Stream `id` starts its ring at `STREAM_RING + 0x100 * id`.
    pub const STREAM_RING: u64 = 0x1a00;
    /// Where a command TRB "was" on the command ring, echoed by its completion event.
    pub const COMMAND_TRB: u64 = 0x3000;
    /// The slot every test drives.
    pub const SLOT_ID: u8 = 1;
    pub const PORT_ID: u8 = 1;

    pub struct TestFailHandle(AtomicBool);

    impl FailHandle for TestFailHandle {
        fn fail(&self) {
            self.0.store(true, Ordering::SeqCst);
        }

        fn failed(&self) -> bool {
            self.0.load(Ordering::SeqCst)
        }
    }

    pub struct Fixture {
        pub mem: GuestMemory,
        pub slots: DeviceSlots,
        pub hub: Arc<UsbHub>,
        pub interrupter: Arc<Mutex<Interrupter>>,
        pub fail_handle: Arc<TestFailHandle>,
        pub irq: Event,
        event_loop: Arc<EventLoop>,
        join: Option<JoinHandle<()>>,
    }

    impl Fixture {
        /// Slot 1 enabled and Addressed on port 1, which has no backend device behind it; an
        /// enabled interrupter over a one-segment event ring.
        pub fn new() -> Fixture {
            let mem = GuestMemory::new(&[(GuestAddress(0), 4 * pagesize() as u64)]).unwrap();
            let mut entry = EventRingSegmentTableEntry::new();
            entry.set_ring_segment_base_address(EVENT_RING);
            entry.set_ring_segment_size(16);
            mem.write_obj_at_addr(entry, GuestAddress(0x8)).unwrap();
            mem.write_obj_at_addr(DEVICE_CONTEXT, GuestAddress(DCBAA + 8 * SLOT_ID as u64))
                .unwrap();

            let (_mmio, regs) = init_xhci_mmio_space_and_regs();
            regs.dcbaap.set_value(DCBAA);
            let irq = Event::new().unwrap();
            let mut interrupter =
                Interrupter::new(mem.clone(), irq.try_clone().unwrap(), &regs).unwrap();
            interrupter.set_event_ring_seg_table_size(1).unwrap();
            interrupter
                .set_event_ring_seg_table_base_addr(GuestAddress(0x8))
                .unwrap();
            interrupter
                .set_event_ring_dequeue_pointer(GuestAddress(EVENT_RING), false)
                .unwrap();
            interrupter.set_enabled(true).unwrap();
            let interrupter = Arc::new(Mutex::new(interrupter));
            let hub = Arc::new(UsbHub::new(&regs, interrupter.clone()));
            let fail_handle = Arc::new(TestFailHandle(AtomicBool::new(false)));
            let (event_loop, join) = EventLoop::start("test".to_string(), None).unwrap();
            let event_loop = Arc::new(event_loop);
            let slots = DeviceSlots::new(
                fail_handle.clone(),
                regs.dcbaap.clone(),
                hub.clone(),
                interrupter.clone(),
                event_loop.clone(),
                mem.clone(),
            );
            let fixture = Fixture {
                mem,
                slots,
                hub,
                interrupter,
                fail_handle,
                irq,
                event_loop,
                join: Some(join),
            };
            let slot = fixture.slot();
            assert!(slot.enable());
            slot.port_id.set(PORT_ID).unwrap();
            let mut ctx = fixture.device_context();
            ctx.slot_context.set_slot_state(DeviceSlotState::Addressed);
            ctx.slot_context.set_root_hub_port_number(PORT_ID);
            fixture.set_device_context(ctx);
            fixture
        }

        pub fn slot(&self) -> Arc<DeviceSlot> {
            self.slots.slot(SLOT_ID).unwrap()
        }

        /// The port slot 1 sits on; nothing is attached behind it.
        pub fn port(&self) -> Arc<UsbPort> {
            self.hub.get_port(PORT_ID).unwrap()
        }

        /// The fixture's event loop, for a test that builds its own ring controller on it.
        pub fn event_loop(&self) -> Arc<EventLoop> {
            self.event_loop.clone()
        }

        /// A transfer of `td` on `endpoint_id` of slot 1, made by `manager`.
        pub fn transfer(
            &self,
            manager: &XhciTransferManager,
            endpoint_id: u8,
            td: TransferDescriptor,
        ) -> XhciTransfer {
            manager.create_transfer(
                self.mem.clone(),
                self.port(),
                self.interrupter.clone(),
                SLOT_ID,
                endpoint_id,
                td,
                Event::new().unwrap(),
                None,
            )
        }

        pub fn device_context(&self) -> DeviceContext {
            self.mem
                .read_obj_from_addr(GuestAddress(DEVICE_CONTEXT))
                .unwrap()
        }

        pub fn set_device_context(&self, ctx: DeviceContext) {
            self.mem
                .write_obj_at_addr(ctx, GuestAddress(DEVICE_CONTEXT))
                .unwrap();
        }

        pub fn endpoint_context(&self, dci: u8) -> EndpointContext {
            self.device_context().endpoint_context[dci as usize - 1]
        }

        pub fn set_endpoint_state(&self, dci: u8, state: EndpointState) {
            let mut ctx = self.device_context();
            ctx.endpoint_context[dci as usize - 1].set_endpoint_state(state);
            self.set_device_context(ctx);
        }

        pub fn stream_context(&self, stream_id: u16) -> StreamContext {
            self.mem
                .read_obj_from_addr(stream_context_addr(stream_id))
                .unwrap()
        }

        pub fn set_stream_context(&self, stream_id: u16, ctx: StreamContext) {
            self.mem
                .write_obj_at_addr(ctx, stream_context_addr(stream_id))
                .unwrap();
        }

        /// An Input Context with these Input Control Context flags and `ep` as the Input
        /// Endpoint Context of `dci`.
        pub fn write_input_context(
            &self,
            drop_flags: u32,
            add_flags: u32,
            dci: u8,
            ep: EndpointContext,
        ) {
            let mut icc = InputControlContext::new();
            icc.set_drop_context_flags(drop_flags);
            icc.set_add_context_flags(add_flags);
            self.mem
                .write_obj_at_addr(icc, GuestAddress(INPUT_CONTEXT))
                .unwrap();
            self.mem
                .write_obj_at_addr(
                    ep,
                    GuestAddress(
                        INPUT_CONTEXT + (dci as u64 + 1) * DEVICE_CONTEXT_ENTRY_SIZE as u64,
                    ),
                )
                .unwrap();
        }

        /// An Input Context that adds `dci` as a bulk endpoint (IN for odd, OUT for even)
        /// with a primary Stream Context Array of `2^(max_pstreams+1)` entries at
        /// `STREAM_CONTEXT_ARRAY`.
        pub fn write_stream_endpoint_input_context(&self, dci: u8, max_pstreams: u8) {
            self.write_input_context(0, 1 << dci, dci, stream_endpoint_context(dci, max_pstreams));
        }

        /// How often the slot asked the host to free an endpoint's streams.
        pub fn host_streams_freed(&self) -> usize {
            self.slot()
                .host_streams_freed
                .load(std::sync::atomic::Ordering::SeqCst)
        }

        /// Stream Contexts `valid` initialised (SCT = 1, ring at `STREAM_RING + 0x100 * id`,
        /// DCS = 1), every other entry of the 16 left Not Valid.
        pub fn write_stream_context_array(&self, valid: std::ops::Range<u16>) {
            for stream_id in 0..16u16 {
                let mut ctx = StreamContext::new();
                if valid.contains(&stream_id) {
                    ctx.set_stream_context_type(1);
                    ctx.set_tr_dequeue_pointer(DequeuePtr::new(stream_ring(stream_id)));
                    ctx.set_dequeue_cycle_state(true);
                }
                self.set_stream_context(stream_id, ctx);
            }
        }

        pub fn configure_endpoint_trb(&self) -> ConfigureEndpointCommandTrb {
            let mut trb = ConfigureEndpointCommandTrb::new();
            trb.set_trb_type(TrbType::ConfigureEndpointCommand);
            trb.set_slot_id(SLOT_ID);
            trb.set_input_context_pointer(INPUT_CONTEXT);
            trb
        }

        /// The Configure Endpoint command as it would come off the command ring.
        pub fn configure_endpoint_command(&self) -> Trb {
            let mut trb = Trb::new();
            *trb.cast_mut::<ConfigureEndpointCommandTrb>().unwrap() = self.configure_endpoint_trb();
            trb
        }

        /// Every Command Completion Event on the event ring so far.
        pub fn command_completions(&self) -> Vec<CommandCompletionEventTrb> {
            let mut completions = Vec::new();
            for i in 0..16u64 {
                let trb: Trb = self
                    .mem
                    .read_obj_from_addr(GuestAddress(EVENT_RING + i * size_of::<Trb>() as u64))
                    .unwrap();
                match trb.get_trb_type() {
                    Ok(TrbType::CommandCompletionEvent) => {
                        completions.push(*trb.cast::<CommandCompletionEventTrb>().unwrap())
                    }
                    _ => break,
                }
            }
            completions
        }

        /// Every Transfer Event on the event ring so far, in order; events of other kinds
        /// between them are skipped.
        pub fn transfer_events(&self) -> Vec<TransferEventTrb> {
            let mut events = Vec::new();
            for i in 0..16u64 {
                let trb: Trb = self
                    .mem
                    .read_obj_from_addr(GuestAddress(EVENT_RING + i * size_of::<Trb>() as u64))
                    .unwrap();
                match trb.get_trb_type() {
                    Ok(TrbType::TransferEvent) => {
                        events.push(*trb.cast::<TransferEventTrb>().unwrap())
                    }
                    Ok(_) => continue,
                    Err(_) => break,
                }
            }
            events
        }
    }

    impl Drop for Fixture {
        fn drop(&mut self) {
            self.event_loop.stop();
            if let Some(join) = self.join.take() {
                join.join().unwrap();
            }
        }
    }

    pub fn stream_context_addr(stream_id: u16) -> GuestAddress {
        GuestAddress(STREAM_CONTEXT_ARRAY + stream_id as u64 * STREAM_CONTEXT_SIZE as u64)
    }

    /// A bulk Endpoint Context for `dci` (IN for odd, OUT for even) without streams, its ring at
    /// `STREAM_RING`.
    pub fn bulk_endpoint_context(dci: u8) -> EndpointContext {
        let mut ep = EndpointContext::new();
        ep.set_endpoint_type(if dci % 2 == 1 { 6 } else { 2 });
        ep.set_max_packet_size(1024);
        ep.set_tr_dequeue_pointer(DequeuePtr::new(GuestAddress(STREAM_RING)));
        ep.set_dequeue_cycle_state(true);
        ep
    }

    /// A bulk Endpoint Context for `dci` with a primary Stream Context Array of
    /// `2^(max_pstreams+1)` entries at `STREAM_CONTEXT_ARRAY`.
    pub fn stream_endpoint_context(dci: u8, max_pstreams: u8) -> EndpointContext {
        let mut ep = bulk_endpoint_context(dci);
        ep.set_max_primary_streams(max_pstreams);
        ep.set_linear_stream_array(1);
        ep.set_tr_dequeue_pointer(DequeuePtr::new(GuestAddress(STREAM_CONTEXT_ARRAY)));
        ep.set_dequeue_cycle_state(false);
        ep
    }

    pub fn stream_ring(stream_id: u16) -> GuestAddress {
        GuestAddress(STREAM_RING + 0x100 * stream_id as u64)
    }

    /// A transfer descriptor of one Normal TRB per entry of `lengths`, chained, with cycle bit
    /// set, laid out from `base` the way a ring holds them (16 bytes apart). The TRBs are
    /// addressed only: nothing is written to guest memory.
    pub fn normal_td(base: u64, lengths: &[u32]) -> TransferDescriptor {
        lengths
            .iter()
            .enumerate()
            .map(|(i, len)| {
                let mut trb = NormalTrb::new();
                trb.set_trb_type(TrbType::Normal);
                trb.set_trb_transfer_length(*len);
                trb.set_cycle(true);
                trb.set_chain(i + 1 < lengths.len());
                AddressedTrb {
                    trb: *trb.cast::<Trb>().unwrap(),
                    gpa: base + i as u64 * size_of::<Trb>() as u64,
                }
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use base::pagesize;
    use zerocopy::IntoBytes;

    use super::test_util::*;
    use super::*;
    use crate::usb::xhci::ring_buffer_controller::StoppedTd;

    fn stream_context(sct: u8, ring: u64, dcs: bool) -> StreamContext {
        let mut ctx = StreamContext::new();
        ctx.set_stream_context_type(sct);
        ctx.set_tr_dequeue_pointer(DequeuePtr::new(GuestAddress(ring)));
        ctx.set_dequeue_cycle_state(dcs);
        ctx
    }

    fn write_array(mem: &GuestMemory, base: u64, entries: &[StreamContext]) {
        for (i, entry) in entries.iter().enumerate() {
            mem.write_obj_at_addr(*entry, GuestAddress(base + i as u64 * 16))
                .unwrap();
        }
    }

    #[test]
    fn stream_array_with_not_valid_entries_is_accepted() {
        // What Windows' USBXHCI hands over for a UASPStor device that opened 8 streams: a
        // 16-entry array with entries 1..8 initialised and the rest Not Valid.
        let mem = GuestMemory::new(&[(GuestAddress(0), pagesize() as u64)]).unwrap();
        let mut entries = vec![StreamContext::new(); 16];
        for (i, entry) in entries.iter_mut().enumerate().take(9).skip(1) {
            *entry = stream_context(1, 0x800 + 0x40 * i as u64, i % 2 == 0);
        }
        write_array(&mem, 0x100, &entries);

        let streams = read_stream_contexts(&mem, GuestAddress(0x100), 3).unwrap();
        assert_eq!(streams.len(), 15);
        assert_eq!(streams.iter().filter(|s| s.is_some()).count(), 8);
        for stream_id in 1..=8usize {
            assert_eq!(
                streams[stream_id - 1],
                Some((
                    GuestAddress(0x800 + 0x40 * stream_id as u64),
                    stream_id % 2 == 0
                )),
                "stream {}",
                stream_id
            );
        }
        assert!(streams[8..].iter().all(|s| s.is_none()));
    }

    #[test]
    fn small_stream_array_at_page_end_is_not_over_read() {
        // MaxPStreams = 1 is a 4-entry, 64-byte array; reading it as 16 entries would run off
        // the end of guest memory here.
        let mem = GuestMemory::new(&[(GuestAddress(0), pagesize() as u64)]).unwrap();
        let base = pagesize() as u64 - 64;
        let mut entries = vec![StreamContext::new(); 4];
        entries[1] = stream_context(1, 0x100, true);
        entries[3] = stream_context(1, 0x300, false);
        write_array(&mem, base, &entries);

        let streams = read_stream_contexts(&mem, GuestAddress(base), 1).unwrap();
        assert_eq!(
            streams,
            vec![
                Some((GuestAddress(0x100), true)),
                None,
                Some((GuestAddress(0x300), false)),
            ]
        );
    }

    #[test]
    fn reserved_sct_is_not_valid_not_error() {
        let mem = GuestMemory::new(&[(GuestAddress(0), pagesize() as u64)]).unwrap();
        let mut entries = vec![StreamContext::new(); 4];
        entries[1] = stream_context(1, 0x100, true);
        // A secondary-array or hub type; nothing this controller runs.
        entries[2] = stream_context(2, 0x200, true);
        entries[3] = stream_context(7, 0x300, true);
        write_array(&mem, 0x400, &entries);

        let streams = read_stream_contexts(&mem, GuestAddress(0x400), 1).unwrap();
        assert_eq!(streams, vec![Some((GuestAddress(0x100), true)), None, None]);
    }

    #[test]
    fn bad_max_pstreams_maps_to_parameter_error() {
        let mem = GuestMemory::new(&[(GuestAddress(0), pagesize() as u64)]).unwrap();
        assert_eq!(
            read_stream_contexts(&mem, GuestAddress(0x100), 5),
            Err(TrbCompletionCode::ParameterError)
        );
    }

    #[test]
    fn unbacked_stream_array_maps_to_parameter_error() {
        let mem = GuestMemory::new(&[(GuestAddress(0), pagesize() as u64)]).unwrap();
        assert_eq!(
            read_stream_contexts(&mem, GuestAddress(pagesize() as u64 - 16), 1),
            Err(TrbCompletionCode::ParameterError)
        );
    }

    fn stream_trcs(slot: &DeviceSlot, dci: u8) -> Vec<Option<Arc<TransferRingController>>> {
        match slot.get_trcs(dci as usize - 1) {
            Some(TransferRingControllers::Stream(trcs)) => trcs,
            Some(TransferRingControllers::Endpoint(_)) => panic!("endpoint {} has no streams", dci),
            None => panic!("endpoint {} is not configured", dci),
        }
    }

    #[test]
    fn configure_endpoint_with_not_valid_streams_is_success() {
        let f = Fixture::new();
        f.write_stream_endpoint_input_context(3, 3);
        f.write_stream_context_array(1..9);

        let code = f
            .slot()
            .configure_endpoint(&f.configure_endpoint_trb())
            .unwrap();
        assert_eq!(code, TrbCompletionCode::Success);

        let trcs = stream_trcs(&f.slot(), 3);
        assert_eq!(trcs.len(), 15);
        for (i, trc) in trcs.iter().enumerate() {
            let stream_id = i as u16 + 1;
            match trc {
                Some(trc) => {
                    assert!(stream_id <= 8, "stream {} must be Not Valid", stream_id);
                    assert_eq!(trc.get_dequeue_pointer(), stream_ring(stream_id));
                    assert!(trc.get_consumer_cycle_state());
                }
                None => assert!(stream_id > 8, "stream {} must have a ring", stream_id),
            }
        }
        let ctx = f.device_context();
        assert_eq!(
            ctx.slot_context.get_slot_state().unwrap(),
            DeviceSlotState::Configured
        );
        assert_eq!(
            ctx.endpoint_context[2].get_endpoint_state().unwrap(),
            EndpointState::Running
        );
        // The endpoint context still points at the array, not at any one ring.
        assert_eq!(
            ctx.endpoint_context[2].get_tr_dequeue_pointer().get_gpa(),
            GuestAddress(STREAM_CONTEXT_ARRAY)
        );
        assert!(!f.fail_handle.failed());
    }

    #[test]
    fn configure_endpoint_rejects_too_many_streams_with_parameter_error() {
        let f = Fixture::new();
        f.write_stream_endpoint_input_context(3, 5);
        f.write_stream_context_array(1..16);

        let code = f
            .slot()
            .configure_endpoint(&f.configure_endpoint_trb())
            .unwrap();
        assert_eq!(code, TrbCompletionCode::ParameterError);
        assert!(f.slot().get_trcs(2).is_none());
        assert_eq!(
            f.device_context().slot_context.get_slot_state().unwrap(),
            DeviceSlotState::Addressed
        );
    }

    #[test]
    fn configure_endpoint_takes_back_what_it_added_when_a_later_endpoint_is_rejected() {
        let f = Fixture::new();
        // DCI 2 is fine, DCI 3 asks for streams on an interrupt endpoint.
        f.write_stream_endpoint_input_context(2, 3);
        f.write_stream_context_array(1..16);
        let mut icc = InputControlContext::new();
        icc.set_add_context_flags((1 << 2) | (1 << 3));
        f.mem
            .write_obj_at_addr(icc, GuestAddress(INPUT_CONTEXT))
            .unwrap();
        let mut ep = EndpointContext::new();
        ep.set_endpoint_type(7);
        ep.set_max_packet_size(64);
        ep.set_max_primary_streams(1);
        ep.set_linear_stream_array(1);
        ep.set_tr_dequeue_pointer(DequeuePtr::new(GuestAddress(STREAM_CONTEXT_ARRAY)));
        f.mem
            .write_obj_at_addr(
                ep,
                GuestAddress(INPUT_CONTEXT + 4 * DEVICE_CONTEXT_ENTRY_SIZE as u64),
            )
            .unwrap();

        let code = f
            .slot()
            .configure_endpoint(&f.configure_endpoint_trb())
            .unwrap();
        assert_eq!(code, TrbCompletionCode::ParameterError);
        assert!(f.slot().get_trcs(1).is_none(), "DCI 2 must be taken back");
        assert!(f.slot().get_trcs(2).is_none());
        let ctx = f.device_context();
        assert_eq!(
            ctx.slot_context.get_slot_state().unwrap(),
            DeviceSlotState::Addressed
        );
        assert_eq!(
            ctx.endpoint_context[1].get_endpoint_state().unwrap(),
            EndpointState::Disabled
        );
    }

    #[test]
    fn configure_endpoint_needs_an_addressed_slot() {
        let f = Fixture::new();
        let mut ctx = f.device_context();
        ctx.slot_context.set_slot_state(DeviceSlotState::Default);
        f.set_device_context(ctx);
        f.write_stream_endpoint_input_context(3, 3);
        f.write_stream_context_array(1..16);

        let code = f
            .slot()
            .configure_endpoint(&f.configure_endpoint_trb())
            .unwrap();
        assert_eq!(code, TrbCompletionCode::ContextStateError);
        assert!(f.slot().get_trcs(2).is_none());
    }

    #[test]
    fn configure_endpoint_with_an_unbacked_input_context_is_a_parameter_error() {
        let f = Fixture::new();
        let mut trb = f.configure_endpoint_trb();
        trb.set_input_context_pointer(0x1_0000_0000);

        let code = f.slot().configure_endpoint(&trb).unwrap();
        assert_eq!(code, TrbCompletionCode::ParameterError);
        assert_eq!(
            f.device_context().slot_context.get_slot_state().unwrap(),
            DeviceSlotState::Addressed
        );
    }

    /// Slot 1 with DCI 3 configured for streams 1..8 of 15, Stopped, ready for the commands
    /// software issues after a stop or a halt.
    fn stopped_stream_endpoint() -> Fixture {
        let f = Fixture::new();
        f.write_stream_endpoint_input_context(3, 3);
        f.write_stream_context_array(1..9);
        assert_eq!(
            f.slot()
                .configure_endpoint(&f.configure_endpoint_trb())
                .unwrap(),
            TrbCompletionCode::Success
        );
        f.set_endpoint_state(3, EndpointState::Stopped);
        f
    }

    #[test]
    fn a_rejected_add_leaves_the_output_endpoint_context_alone() {
        let f = stopped_stream_endpoint();
        let before = f.endpoint_context(3);
        // The guest re-adds the endpoint with a Stream Context Array it did not back with
        // memory. Spec 4.6.6: the Output Device Context stays as it was.
        let mut ep = stream_endpoint_context(3, 3);
        ep.set_tr_dequeue_pointer(DequeuePtr::new(GuestAddress(0x1_0000_0000)));
        f.write_input_context(0, 1 << 3, 3, ep);

        let code = f
            .slot()
            .configure_endpoint(&f.configure_endpoint_trb())
            .unwrap();
        assert_eq!(code, TrbCompletionCode::ParameterError);
        assert_eq!(f.endpoint_context(3).as_bytes(), before.as_bytes());
        assert_eq!(
            stream_trcs(&f.slot(), 3)
                .iter()
                .filter(|t| t.is_some())
                .count(),
            8
        );
        assert_eq!(f.host_streams_freed(), 0);

        // So the endpoint still stops through the array it really has, instead of an Err from
        // the bogus one taking the command ring down.
        f.set_endpoint_state(3, EndpointState::Running);
        let completed = Arc::new(Mutex::new(None));
        let done = completed.clone();
        f.slot()
            .stop_endpoint(f.fail_handle.clone(), 3, move |code| {
                *done.lock() = Some(code);
                Ok(())
            })
            .unwrap();
        assert_eq!(*completed.lock(), Some(TrbCompletionCode::Success));
        assert_eq!(
            f.stream_context(1).get_tr_dequeue_pointer().get_gpa(),
            stream_ring(1)
        );
        assert!(!f.fail_handle.failed());
    }

    #[test]
    fn a_rejected_add_does_not_drop_the_endpoint_first() {
        let f = stopped_stream_endpoint();
        // Linux reconfigures a stream endpoint with Drop + Add in one command; here the Add
        // asks for more streams than the controller has, so nothing may happen to the endpoint.
        f.write_input_context(1 << 3, 1 << 3, 3, stream_endpoint_context(3, 5));

        let code = f
            .slot()
            .configure_endpoint(&f.configure_endpoint_trb())
            .unwrap();
        assert_eq!(code, TrbCompletionCode::ParameterError);
        assert_eq!(
            stream_trcs(&f.slot(), 3)
                .iter()
                .filter(|t| t.is_some())
                .count(),
            8
        );
        let ep = f.endpoint_context(3);
        assert_eq!(ep.get_endpoint_state().unwrap(), EndpointState::Stopped);
        assert_eq!(ep.get_max_primary_streams(), 3);
        assert_eq!(f.host_streams_freed(), 0);
    }

    #[test]
    fn readding_a_stream_endpoint_without_streams_frees_the_host_streams() {
        let f = stopped_stream_endpoint();
        // Add only, no Drop: the endpoint turns into a plain bulk endpoint, and the host's
        // streams have to go with the rings or the next allocation gets EINVAL.
        f.write_input_context(0, 1 << 3, 3, bulk_endpoint_context(3));

        let code = f
            .slot()
            .configure_endpoint(&f.configure_endpoint_trb())
            .unwrap();
        assert_eq!(code, TrbCompletionCode::Success);
        assert_eq!(f.host_streams_freed(), 1);
        match f.slot().get_trcs(2) {
            Some(TransferRingControllers::Endpoint(trc)) => {
                assert_eq!(trc.get_dequeue_pointer(), GuestAddress(STREAM_RING));
            }
            _ => panic!("DCI 3 must be a plain endpoint now"),
        }
        let ep = f.endpoint_context(3);
        assert_eq!(ep.get_max_primary_streams(), 0);
        assert_eq!(ep.get_endpoint_state().unwrap(), EndpointState::Running);
    }

    #[test]
    fn readding_a_stream_endpoint_with_streams_frees_the_host_streams_first() {
        let f = stopped_stream_endpoint();
        f.write_stream_endpoint_input_context(3, 3);

        let code = f
            .slot()
            .configure_endpoint(&f.configure_endpoint_trb())
            .unwrap();
        assert_eq!(code, TrbCompletionCode::Success);
        assert_eq!(f.host_streams_freed(), 1);
        assert_eq!(stream_trcs(&f.slot(), 3).len(), 15);
    }

    #[test]
    fn deconfiguring_a_slot_that_lost_its_port_is_not_an_error() {
        let f = stopped_stream_endpoint();
        // A controller reset took the port away; the guest goes on with its stale Output
        // Device Context. Its mistake is not the command ring's death.
        f.slot().port_id.reset();
        let mut trb = f.configure_endpoint_trb();
        trb.set_deconfigure(true);

        let code = f.slot().configure_endpoint(&trb).unwrap();
        assert_eq!(code, TrbCompletionCode::Success);
        assert!(f.slot().get_trcs(2).is_none());
        assert_eq!(f.host_streams_freed(), 1);
        assert_eq!(
            f.device_context().slot_context.get_slot_state().unwrap(),
            DeviceSlotState::Addressed
        );
    }

    #[test]
    fn reset_frees_the_host_streams_of_every_stream_endpoint() {
        let f = stopped_stream_endpoint();
        f.slot().reset();
        assert_eq!(f.host_streams_freed(), 1);
        assert!(f.slot().get_trcs(2).is_none());
    }

    #[test]
    fn set_tr_dequeue_with_a_not_valid_type_takes_the_stream_ring_away() {
        let f = stopped_stream_endpoint();

        let code = f
            .slot()
            .set_tr_dequeue_ptr(3, 2, 0, stream_ring(2).0, true)
            .unwrap();
        assert_eq!(code, TrbCompletionCode::Success);
        assert_eq!(f.stream_context(2).get_stream_context_type(), 0);
        let trcs = stream_trcs(&f.slot(), 3);
        assert!(trcs[1].is_none());
        assert_eq!(trcs.iter().filter(|t| t.is_some()).count(), 7);

        // A doorbell on it is now ignored like on any other Not Valid stream.
        f.set_endpoint_state(3, EndpointState::Running);
        assert!(!f.slot().ring_doorbell(3, 2).unwrap());
        assert!(f.slot().ring_doorbell(3, 1).unwrap());
        assert!(!f.fail_handle.failed());
    }

    #[test]
    fn set_tr_dequeue_on_stream_endpoint_keeps_sca_pointer() {
        let f = stopped_stream_endpoint();
        let new_ptr = stream_ring(2).unchecked_add(0x50);

        let code = f
            .slot()
            .set_tr_dequeue_ptr(3, 2, 1, new_ptr.0, false)
            .unwrap();
        assert_eq!(code, TrbCompletionCode::Success);

        // The Endpoint Context still names the array, not the ring (spec 4.6.10).
        let ep = f.endpoint_context(3);
        assert_eq!(
            ep.get_tr_dequeue_pointer().get_gpa(),
            GuestAddress(STREAM_CONTEXT_ARRAY)
        );
        assert_eq!(ep.get_endpoint_state().unwrap(), EndpointState::Stopped);
        // Stream Context 2 carries the new position; its neighbours are untouched.
        let sc = f.stream_context(2);
        assert_eq!(sc.get_tr_dequeue_pointer().get_gpa(), new_ptr);
        assert!(!sc.get_dequeue_cycle_state());
        assert_eq!(sc.get_stream_context_type(), 1);
        assert_eq!(
            f.stream_context(1).get_tr_dequeue_pointer().get_gpa(),
            stream_ring(1)
        );
        assert_eq!(
            f.stream_context(3).get_tr_dequeue_pointer().get_gpa(),
            stream_ring(3)
        );
        // And the ring itself moved.
        let trcs = stream_trcs(&f.slot(), 3);
        let trc = trcs[1].as_ref().unwrap();
        assert_eq!(trc.get_dequeue_pointer(), new_ptr);
        assert!(!trc.get_consumer_cycle_state());
        assert_eq!(
            trcs[0].as_ref().unwrap().get_dequeue_pointer(),
            stream_ring(1)
        );
    }

    #[test]
    fn set_tr_dequeue_initialises_a_not_valid_stream() {
        let f = stopped_stream_endpoint();
        assert!(stream_trcs(&f.slot(), 3)[8].is_none());

        let code = f
            .slot()
            .set_tr_dequeue_ptr(3, 9, 1, stream_ring(9).0, true)
            .unwrap();
        assert_eq!(code, TrbCompletionCode::Success);

        let sc = f.stream_context(9);
        assert_eq!(sc.get_stream_context_type(), 1);
        assert_eq!(sc.get_tr_dequeue_pointer().get_gpa(), stream_ring(9));
        let trcs = stream_trcs(&f.slot(), 3);
        assert_eq!(trcs.iter().filter(|t| t.is_some()).count(), 9);
        assert_eq!(
            trcs[8].as_ref().unwrap().get_dequeue_pointer(),
            stream_ring(9)
        );
        // A stream the guest can now ring.
        f.set_endpoint_state(3, EndpointState::Running);
        assert!(f.slot().ring_doorbell(3, 9).unwrap());
    }

    #[test]
    fn set_tr_dequeue_with_a_stream_id_off_the_array_is_a_trb_error() {
        let f = stopped_stream_endpoint();
        for stream_id in [0u16, 16, 255] {
            assert_eq!(
                f.slot()
                    .set_tr_dequeue_ptr(3, stream_id, 1, stream_ring(1).0, true)
                    .unwrap(),
                TrbCompletionCode::TrbError,
                "stream id {}",
                stream_id
            );
        }
        assert_eq!(
            f.endpoint_context(3).get_tr_dequeue_pointer().get_gpa(),
            GuestAddress(STREAM_CONTEXT_ARRAY)
        );
    }

    #[test]
    fn set_tr_dequeue_on_a_running_stream_endpoint_is_a_context_state_error() {
        let f = stopped_stream_endpoint();
        f.set_endpoint_state(3, EndpointState::Running);
        assert_eq!(
            f.slot()
                .set_tr_dequeue_ptr(3, 1, 1, stream_ring(1).0, true)
                .unwrap(),
            TrbCompletionCode::ContextStateError
        );
        assert_eq!(
            f.stream_context(1).get_tr_dequeue_pointer().get_gpa(),
            stream_ring(1)
        );
    }

    #[test]
    fn stop_endpoint_writes_back_only_populated_streams() {
        let f = stopped_stream_endpoint();
        f.set_endpoint_state(3, EndpointState::Running);
        // The guest owns everything the controller does not write: a Stopped EDTLA on a live
        // entry, and whatever it keeps in the Not Valid ones.
        let mut sc = f.stream_context(1);
        sc.set_stopped_edtla(0x1234);
        f.set_stream_context(1, sc);
        let mut poison = StreamContext::new();
        poison.set_stopped_edtla(0xabcdef);
        poison.set_reserved1(0x5a);
        poison.set_reserved2(0xdeadbeef);
        for stream_id in 9..16 {
            f.set_stream_context(stream_id, poison);
        }

        let completed = Arc::new(Mutex::new(None));
        let done = completed.clone();
        f.slot()
            .stop_endpoint(f.fail_handle.clone(), 3, move |code| {
                *done.lock() = Some(code);
                Ok(())
            })
            .unwrap();
        assert_eq!(*completed.lock(), Some(TrbCompletionCode::Success));

        assert_eq!(
            f.endpoint_context(3).get_endpoint_state().unwrap(),
            EndpointState::Stopped
        );
        assert_eq!(
            f.endpoint_context(3).get_tr_dequeue_pointer().get_gpa(),
            GuestAddress(STREAM_CONTEXT_ARRAY)
        );
        for stream_id in 1..9 {
            let sc = f.stream_context(stream_id);
            assert_eq!(sc.get_stream_context_type(), 1, "stream {}", stream_id);
            assert_eq!(
                sc.get_tr_dequeue_pointer().get_gpa(),
                stream_ring(stream_id),
                "stream {}",
                stream_id
            );
            assert!(sc.get_dequeue_cycle_state());
        }
        assert_eq!(f.stream_context(1).get_stopped_edtla(), 0x1234);
        for stream_id in 9..16 {
            assert_eq!(
                f.stream_context(stream_id).as_bytes(),
                poison.as_bytes(),
                "stream {} is the guest's",
                stream_id
            );
        }
        assert!(!f.fail_handle.failed());
    }

    /// The Stream Context is written from the ring as it stands when the stop is answered --
    /// the position a stop that rewound the ring left it at -- with the endpoint already
    /// Stopped in guest memory, and a stream whose ring had nothing in progress keeps the
    /// Stopped EDTLA the guest gave it.
    #[test]
    fn stop_endpoint_writes_the_context_from_the_ring_the_callback_sees() {
        let f = stopped_stream_endpoint();
        f.set_endpoint_state(3, EndpointState::Running);
        let mut sc = f.stream_context(2);
        sc.set_stopped_edtla(0x1234);
        f.set_stream_context(2, sc);
        // Stream 2 runs: its ring is empty (nothing in that memory carries its cycle bit), so it
        // parks where it stands. Where that is, is then moved to a marker.
        assert!(f.slot().ring_doorbell(3, 2).unwrap());
        let marker = stream_ring(2).unchecked_add(0x40);
        let trc = stream_trcs(&f.slot(), 3)[1].clone().unwrap();
        trc.set_dequeue_pointer(marker);

        let seen = Arc::new(Mutex::new(None));
        let s = seen.clone();
        let mem = f.mem.clone();
        f.slot()
            .stop_endpoint(f.fail_handle.clone(), 3, move |code| {
                let sc: StreamContext = mem.read_obj_from_addr(stream_context_addr(2)).unwrap();
                let ctx: DeviceContext = mem
                    .read_obj_from_addr(GuestAddress(DEVICE_CONTEXT))
                    .unwrap();
                *s.lock() = Some((
                    code,
                    sc.get_tr_dequeue_pointer().get_gpa(),
                    sc.get_dequeue_cycle_state(),
                    sc.get_stopped_edtla(),
                    ctx.endpoint_context[2].get_endpoint_state().unwrap(),
                ));
                Ok(())
            })
            .unwrap();
        assert_eq!(
            *seen.lock(),
            Some((
                TrbCompletionCode::Success,
                marker,
                true,
                0x1234,
                EndpointState::Stopped
            )),
            "at the completion the context carries the ring's position and the Stopped state"
        );
        assert_eq!(
            f.stream_context(1).get_tr_dequeue_pointer().get_gpa(),
            stream_ring(1)
        );
        assert!(!f.fail_handle.failed());
    }

    /// The stop cancelled a TD on two stream rings; only the one that won the command's one
    /// Stopped event carries a Stopped EDTLA into its Stream Context -- the silently rewound
    /// ring's entry keeps the guest's value, though its pointer and DCS are written back.
    #[test]
    fn stop_endpoint_writes_stopped_edtla_only_for_the_stream_that_reported() {
        let f = stopped_stream_endpoint();
        f.set_endpoint_state(3, EndpointState::Running);
        for stream_id in [2u16, 3] {
            let mut sc = f.stream_context(stream_id);
            sc.set_stopped_edtla(0x1234);
            f.set_stream_context(stream_id, sc);
        }
        // What the rings' stops record: stream 2's won the claim and emitted, stream 3's lost
        // and rewound silently.
        let trcs = stream_trcs(&f.slot(), 3);
        trcs[1].as_ref().unwrap().set_stopped_td_for_test(Some(StoppedTd {
            first_trb: stream_ring(2),
            cycle: true,
            bytes: 0x150,
            reported: true,
        }));
        trcs[2].as_ref().unwrap().set_stopped_td_for_test(Some(StoppedTd {
            first_trb: stream_ring(3),
            cycle: true,
            bytes: 0x99,
            reported: false,
        }));

        let completed = Arc::new(Mutex::new(None));
        let done = completed.clone();
        f.slot()
            .stop_endpoint(f.fail_handle.clone(), 3, move |code| {
                *done.lock() = Some(code);
                Ok(())
            })
            .unwrap();
        assert_eq!(*completed.lock(), Some(TrbCompletionCode::Success));

        assert_eq!(
            f.stream_context(2).get_stopped_edtla(),
            0x150,
            "the reporting stream gets its Stopped EDTLA"
        );
        assert_eq!(
            f.stream_context(3).get_stopped_edtla(),
            0x1234,
            "the silent stream keeps the guest's"
        );
        for stream_id in [2u16, 3] {
            assert_eq!(
                f.stream_context(stream_id).get_tr_dequeue_pointer().get_gpa(),
                stream_ring(stream_id),
                "stream {stream_id}: the position is written back either way"
            );
            assert!(f.stream_context(stream_id).get_dequeue_cycle_state());
        }
        assert!(!f.fail_handle.failed());
    }

    /// A Stop Endpoint on a stream endpoint arms every populated ring with one shared claim on
    /// the command's one Stopped event, before any ring stops.
    #[test]
    fn stop_endpoint_hands_every_stream_ring_one_shared_stop_event_claim() {
        let f = stopped_stream_endpoint();
        f.set_endpoint_state(3, EndpointState::Running);
        let completed = Arc::new(Mutex::new(None));
        let done = completed.clone();
        f.slot()
            .stop_endpoint(f.fail_handle.clone(), 3, move |code| {
                *done.lock() = Some(code);
                Ok(())
            })
            .unwrap();
        assert_eq!(*completed.lock(), Some(TrbCompletionCode::Success));

        let trcs = stream_trcs(&f.slot(), 3);
        let claim = trcs[0]
            .as_ref()
            .unwrap()
            .stop_event_claim_for_test()
            .expect("the stop must arm stream 1");
        for (i, trc) in trcs.iter().enumerate() {
            if let Some(trc) = trc {
                let c = trc
                    .stop_event_claim_for_test()
                    .unwrap_or_else(|| panic!("stream {} was not armed", i + 1));
                assert!(
                    Arc::ptr_eq(&c, &claim),
                    "stream {} must share the command's one claim",
                    i + 1
                );
            }
        }
        assert!(
            !claim.load(Ordering::SeqCst),
            "no ring had a TD in flight: the event goes unclaimed"
        );
        assert!(!f.fail_handle.failed());
    }

    /// Spec 4.6.9: Stop Endpoint is valid only on a Running endpoint -- hardware answers any
    /// other state with a Context State Error. An endpoint already Stopped (what a
    /// spec-violating second stop, sent before the first completed, would find) is left alone:
    /// no ring is armed with a claim, so no second Stopped event can follow.
    #[test]
    fn stop_endpoint_of_a_not_running_endpoint_is_a_context_state_error() {
        let f = stopped_stream_endpoint();
        let completed = Arc::new(Mutex::new(None));
        let done = completed.clone();
        f.slot()
            .stop_endpoint(f.fail_handle.clone(), 3, move |code| {
                *done.lock() = Some(code);
                Ok(())
            })
            .unwrap();
        assert_eq!(*completed.lock(), Some(TrbCompletionCode::ContextStateError));
        for (i, trc) in stream_trcs(&f.slot(), 3).iter().enumerate() {
            if let Some(trc) = trc {
                assert!(
                    trc.stop_event_claim_for_test().is_none(),
                    "stream {} must not be armed by a rejected stop",
                    i + 1
                );
            }
        }
        assert_eq!(
            f.endpoint_context(3).get_endpoint_state().unwrap(),
            EndpointState::Stopped
        );
        assert!(!f.fail_handle.failed());
    }

    /// A plain endpoint has no sibling rings to share its Stopped event with: its stop sets no
    /// claim, and its lone ring reports as before.
    #[test]
    fn stop_endpoint_without_streams_sets_no_claim() {
        let f = Fixture::new();
        f.write_input_context(0, 1 << 3, 3, bulk_endpoint_context(3));
        assert_eq!(
            f.slot()
                .configure_endpoint(&f.configure_endpoint_trb())
                .unwrap(),
            TrbCompletionCode::Success
        );
        f.slot()
            .stop_endpoint(f.fail_handle.clone(), 3, |_| Ok(()))
            .unwrap();
        let trc = match f.slot().get_trcs(2) {
            Some(TransferRingControllers::Endpoint(trc)) => trc,
            _ => panic!("DCI 3 must be a plain endpoint"),
        };
        assert!(trc.stop_event_claim_for_test().is_none());
        assert!(!f.fail_handle.failed());
    }

    /// The same for an endpoint without streams: the Endpoint Context's TR Dequeue Pointer and
    /// DCS come from the ring at the completion, and the rest of the context stays.
    #[test]
    fn stop_endpoint_without_streams_writes_the_ring_position_at_the_completion() {
        let f = Fixture::new();
        f.write_input_context(0, 1 << 3, 3, bulk_endpoint_context(3));
        assert_eq!(
            f.slot()
                .configure_endpoint(&f.configure_endpoint_trb())
                .unwrap(),
            TrbCompletionCode::Success
        );
        let trc = match f.slot().get_trcs(2) {
            Some(TransferRingControllers::Endpoint(trc)) => trc,
            _ => panic!("DCI 3 must be a plain endpoint"),
        };
        let marker = GuestAddress(STREAM_RING + 0x40);
        trc.set_dequeue_pointer(marker);
        trc.set_consumer_cycle_state(false);

        let seen = Arc::new(Mutex::new(None));
        let s = seen.clone();
        let mem = f.mem.clone();
        f.slot()
            .stop_endpoint(f.fail_handle.clone(), 3, move |code| {
                let ctx: DeviceContext = mem
                    .read_obj_from_addr(GuestAddress(DEVICE_CONTEXT))
                    .unwrap();
                let ep = ctx.endpoint_context[2];
                *s.lock() = Some((
                    code,
                    ep.get_tr_dequeue_pointer().get_gpa(),
                    ep.get_dequeue_cycle_state(),
                    ep.get_endpoint_state().unwrap(),
                    ep.get_max_packet_size(),
                ));
                Ok(())
            })
            .unwrap();
        assert_eq!(
            *seen.lock(),
            Some((
                TrbCompletionCode::Success,
                marker,
                false,
                EndpointState::Stopped,
                1024
            ))
        );
        assert!(!f.fail_handle.failed());
    }

    /// A stream whose ring stopped with a descriptor in progress gets its Stopped EDTLA (24
    /// bits, spec 6.2.4.1); one that stopped idle keeps the value it had. Nothing else in the
    /// entry moves.
    #[test]
    fn stream_context_write_back_sets_the_stopped_edtla_of_a_stopped_stream_only() {
        let f = Fixture::new();
        f.write_stream_context_array(1..3);
        for stream_id in 1..3 {
            let mut sc = f.stream_context(stream_id);
            sc.set_stopped_edtla(0x1234);
            sc.set_reserved1(0x5a);
            sc.set_reserved2(0xdeadbeef);
            f.set_stream_context(stream_id, sc);
        }

        write_stream_context(
            &f.mem,
            GuestAddress(STREAM_CONTEXT_ARRAY),
            1,
            GuestAddress(0x5000),
            false,
            Some(0x0123_4567),
        )
        .unwrap();
        write_stream_context(
            &f.mem,
            GuestAddress(STREAM_CONTEXT_ARRAY),
            2,
            GuestAddress(0x6000),
            true,
            None,
        )
        .unwrap();

        let sc = f.stream_context(1);
        assert_eq!(sc.get_tr_dequeue_pointer().get_gpa(), GuestAddress(0x5000));
        assert!(!sc.get_dequeue_cycle_state());
        assert_eq!(sc.get_stopped_edtla(), 0x23_4567);
        assert_eq!(sc.get_stream_context_type(), 1);
        assert_eq!(sc.get_reserved1(), 0x5a);
        assert_eq!(sc.get_reserved2(), 0xdeadbeef);
        let sc = f.stream_context(2);
        assert_eq!(sc.get_tr_dequeue_pointer().get_gpa(), GuestAddress(0x6000));
        assert!(sc.get_dequeue_cycle_state());
        assert_eq!(sc.get_stopped_edtla(), 0x1234);
        assert_eq!(sc.get_reserved2(), 0xdeadbeef);
    }

    #[test]
    fn doorbell_on_a_not_valid_stream_is_ignored() {
        let f = Fixture::new();
        f.write_stream_endpoint_input_context(3, 3);
        f.write_stream_context_array(1..9);
        assert_eq!(
            f.slot()
                .configure_endpoint(&f.configure_endpoint_trb())
                .unwrap(),
            TrbCompletionCode::Success
        );

        let slot = f.slot();
        assert!(!slot.ring_doorbell(3, 9).unwrap(), "Not Valid stream");
        assert!(
            !slot.ring_doorbell(3, 0).unwrap(),
            "stream id 0 is reserved"
        );
        assert!(!slot.ring_doorbell(3, 16).unwrap(), "beyond the array");
        assert!(slot.ring_doorbell(3, 8).unwrap());
        assert!(!f.fail_handle.failed());
    }

    /// run5 cold1/cold3: a Stop Endpoint cancelled a TD on two stream rings, reported the one
    /// Stopped event for one of them and rewound the other silently. Windows re-points both
    /// streams, re-arms the silent ring's TD, and rings the doorbell naming only the reported
    /// stream -- the silent ring must run again from that doorbell (spec 4.12.2: the stream id
    /// is a hint; hardware serves whichever stream the device ERDYs), or its TD is stranded
    /// until the class driver's ~16 s request timeout.
    #[test]
    fn doorbell_restarts_every_stream_ring_of_the_endpoint() {
        let f = stopped_stream_endpoint();
        f.set_endpoint_state(3, EndpointState::Running);
        f.slot()
            .stop_endpoint(f.fail_handle.clone(), 3, |_| Ok(()))
            .unwrap();
        // The stop parked every populated ring and armed each with the command's claim; make
        // streams 2 and 3 look like the stop cancelled a TD on both -- stream 2 won the one
        // Stopped event, stream 3 rewound silently: the two-cancelled-TD state of a cold
        // attach.
        let trcs = stream_trcs(&f.slot(), 3);
        trcs[1]
            .as_ref()
            .unwrap()
            .set_stopped_td_for_test(Some(StoppedTd {
                first_trb: stream_ring(2),
                cycle: true,
                bytes: 0x150,
                reported: true,
            }));
        trcs[2]
            .as_ref()
            .unwrap()
            .set_stopped_td_for_test(Some(StoppedTd {
                first_trb: stream_ring(3),
                cycle: true,
                bytes: 0x99,
                reported: false,
            }));
        for (i, trc) in trcs.iter().enumerate() {
            if let Some(trc) = trc {
                assert!(
                    trc.stop_event_claim_for_test().is_some(),
                    "stream {} must be armed by the stop",
                    i + 1
                );
            }
        }

        // The doorbell names only stream 2, as Windows' did.
        assert!(f.slot().ring_doorbell(3, 2).unwrap());

        assert_eq!(
            f.endpoint_context(3).get_endpoint_state().unwrap(),
            EndpointState::Running
        );
        // `start()` ran on every populated ring: it consumes the stale claim and the
        // stopped-TD record. Stream 3 -- the silent ring, still standing at its rewound TD --
        // is the one an addressed-ring-only doorbell would have left parked.
        for (i, trc) in trcs.iter().enumerate() {
            if let Some(trc) = trc {
                assert!(
                    trc.stop_event_claim_for_test().is_none(),
                    "stream {} must be started by the doorbell",
                    i + 1
                );
                assert!(
                    trc.stopped_td().is_none(),
                    "stream {} must be started by the doorbell",
                    i + 1
                );
            }
        }
        assert_eq!(
            trcs[2].as_ref().unwrap().get_dequeue_pointer(),
            stream_ring(3),
            "the silent ring re-runs from its rewound TD"
        );
        assert!(!f.fail_handle.failed());
    }

    /// A doorbell whose stream id is Not Valid, reserved (0), or off the array still starts
    /// nothing -- none of the endpoint's populated rings either.
    #[test]
    fn doorbell_on_a_not_valid_stream_starts_no_ring() {
        let f = stopped_stream_endpoint();
        f.set_endpoint_state(3, EndpointState::Running);
        f.slot()
            .stop_endpoint(f.fail_handle.clone(), 3, |_| Ok(()))
            .unwrap();
        for bad in [0u16, 9, 16] {
            assert!(!f.slot().ring_doorbell(3, bad).unwrap(), "stream id {bad}");
        }
        for (i, trc) in stream_trcs(&f.slot(), 3).iter().enumerate() {
            if let Some(trc) = trc {
                assert!(
                    trc.stop_event_claim_for_test().is_some(),
                    "stream {} must not be started by an ignored doorbell",
                    i + 1
                );
            }
        }
        assert_eq!(
            f.endpoint_context(3).get_endpoint_state().unwrap(),
            EndpointState::Stopped
        );
        assert!(!f.fail_handle.failed());
    }

    /// A doorbell on an endpoint without streams starts its one ring, as before; a stream id
    /// on such an endpoint stays ignored.
    #[test]
    fn doorbell_on_a_plain_endpoint_starts_its_ring() {
        let f = Fixture::new();
        f.write_input_context(0, 1 << 3, 3, bulk_endpoint_context(3));
        assert_eq!(
            f.slot()
                .configure_endpoint(&f.configure_endpoint_trb())
                .unwrap(),
            TrbCompletionCode::Success
        );
        let trc = match f.slot().get_trcs(2) {
            Some(TransferRingControllers::Endpoint(trc)) => trc,
            _ => panic!("DCI 3 must be a plain endpoint"),
        };
        trc.set_stopped_td_for_test(Some(StoppedTd {
            first_trb: GuestAddress(STREAM_RING),
            cycle: true,
            bytes: 1,
            reported: true,
        }));

        assert!(
            !f.slot().ring_doorbell(3, 5).unwrap(),
            "a stream id on a streamless endpoint is ignored"
        );
        assert!(
            trc.stopped_td().is_some(),
            "an ignored doorbell starts nothing"
        );

        assert!(f.slot().ring_doorbell(3, 0).unwrap());
        assert_eq!(
            f.endpoint_context(3).get_endpoint_state().unwrap(),
            EndpointState::Running
        );
        assert!(trc.stopped_td().is_none(), "the ring was started");
        assert!(!f.fail_handle.failed());
    }
}
