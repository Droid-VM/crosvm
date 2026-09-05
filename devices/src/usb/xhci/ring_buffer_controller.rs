// Copyright 2019 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use std::fmt;
use std::fmt::Display;
use std::sync::atomic::AtomicBool;
use std::sync::atomic::Ordering;
use std::sync::Arc;
use std::sync::MutexGuard;

use anyhow::Context;
use base::debug;
use base::error;
use base::info;
use base::Error as SysError;
use base::Event;
use base::EventType;
use remain::sorted;
use sync::Mutex;
use thiserror::Error;
use vm_memory::GuestAddress;
use vm_memory::GuestMemory;

use super::ring_buffer::RingBuffer;
use super::ring_buffer_stop_cb::RingBufferStopCallback;
use super::xhci_abi::TransferDescriptor;
use crate::utils;
use crate::utils::EventHandler;
use crate::utils::EventLoop;

#[sorted]
#[derive(Error, Debug)]
pub enum Error {
    #[error("failed to add event to event loop: {0}")]
    AddEvent(utils::Error),
    #[error("failed to create event: {0}")]
    CreateEvent(SysError),
}

type Result<T> = std::result::Result<T, Error>;

#[derive(PartialEq, Copy, Clone, Eq)]
enum RingBufferState {
    /// Running: RingBuffer is running, consuming transfer descriptor.
    Running,
    /// Stopping: Some thread requested RingBuffer stop. It will stop when current descriptor is
    /// handled.
    Stopping,
    /// Stopped: RingBuffer already stopped.
    Stopped,
}

/// TransferDescriptorHandler handles transfer descriptor. User should implement this trait and
/// build a ring buffer controller with the struct.
pub trait TransferDescriptorHandler {
    /// Process descriptor asynchronously, write complete_event when done.
    fn handle_transfer_descriptor(
        &self,
        descriptor: TransferDescriptor,
        complete_event: Event,
    ) -> anyhow::Result<()>;

    /// Stop is called when trying to stop ring buffer controller. Returns true when stop must be
    /// performed asynchronously. This happens because the handler is handling some descriptor
    /// asynchronously, the stop callback of ring buffer controller must be called after the
    /// `async` part is handled or canceled. If the TransferDescriptorHandler decide it could stop
    /// immediately, it could return false.
    /// For example, if a handler submitted a transfer but the transfer has not yet finished. Then
    /// guest kernel requests to stop the ring buffer controller. Transfer descriptor handler will
    /// return true, thus RingBufferController would transfer to Stopping state. It will be stopped
    /// when all pending transfer completed.
    /// On the other hand, if hander does not have any pending transfers, it would return false.
    fn stop(&self) -> bool {
        true
    }

    /// Returns false while the handler still has descriptors it has not finished with. The ring
    /// buffer controller parks itself in `Stopped` -- which releases the stop callback the guest
    /// is waiting on -- only once this is true, so a handler holding several descriptors in flight
    /// must not report itself quiesced early. A handler that tracks nothing is quiesced by
    /// definition.
    fn is_quiesced(&self) -> bool {
        true
    }
}

/// Upper bound on the descriptors one `on_event` hands off when the whole ring is drained. A ring
/// segment holds fewer TRBs than this, so a guest never hits it in normal use; the cap is there
/// because the ring is guest memory, and one whose link TRB never toggles the consumer cycle state
/// yields descriptors forever, which would spin here with `state` held and wedge the event loop
/// this controller shares with every other endpoint.
const MAX_DEQUEUE_PER_EVENT: usize = 256;

/// RingBufferController owns a ring buffer. It lives on a event_loop. It will pop out transfer
/// descriptor and let TransferDescriptorHandler handle it.
pub struct RingBufferController<T: 'static + TransferDescriptorHandler> {
    name: String,
    state: Mutex<RingBufferState>,
    stop_callback: Mutex<Vec<RingBufferStopCallback>>,
    ring_buffer: Mutex<RingBuffer>,
    handler: Mutex<T>,
    event_loop: Arc<EventLoop>,
    event: Event,
    dequeue_all: AtomicBool,
    /// Set by `halt`; `on_event` parks the ring when it sees it.
    halt_requested: AtomicBool,
}

impl<T: 'static + TransferDescriptorHandler> Display for RingBufferController<T> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "RingBufferController `{}`", self.name)
    }
}

impl<T: Send> RingBufferController<T>
where
    T: 'static + TransferDescriptorHandler,
{
    /// Create a ring buffer controller and add it to event loop.
    pub fn new_with_handler(
        name: String,
        mem: GuestMemory,
        event_loop: Arc<EventLoop>,
        handler: T,
    ) -> Result<Arc<RingBufferController<T>>> {
        let evt = Event::new().map_err(Error::CreateEvent)?;
        let controller = Arc::new(RingBufferController {
            name: name.clone(),
            state: Mutex::new(RingBufferState::Stopped),
            stop_callback: Mutex::new(Vec::new()),
            ring_buffer: Mutex::new(RingBuffer::new(name, mem)),
            handler: Mutex::new(handler),
            event_loop: event_loop.clone(),
            event: evt,
            dequeue_all: AtomicBool::new(false),
            halt_requested: AtomicBool::new(false),
        });
        let event_handler: Arc<dyn EventHandler> = controller.clone();
        event_loop
            .add_event(
                &controller.event,
                EventType::Read,
                Arc::downgrade(&event_handler),
            )
            .map_err(Error::AddEvent)?;
        Ok(controller)
    }

    fn lock_ring_buffer(&self) -> MutexGuard<RingBuffer> {
        self.ring_buffer.lock()
    }

    /// Get dequeue pointer of the internal ring buffer.
    pub fn get_dequeue_pointer(&self) -> GuestAddress {
        self.lock_ring_buffer().get_dequeue_pointer()
    }

    /// Set dequeue pointer of the internal ring buffer.
    pub fn set_dequeue_pointer(&self, ptr: GuestAddress) {
        xhci_trace!("{}: set_dequeue_pointer({:x})", self.name, ptr.0);
        // Fast because this should only happen during xhci setup.
        self.lock_ring_buffer().set_dequeue_pointer(ptr);
    }

    /// Get consumer cycle state.
    pub fn get_consumer_cycle_state(&self) -> bool {
        self.lock_ring_buffer().get_consumer_cycle_state()
    }

    /// Set consumer cycle state.
    pub fn set_consumer_cycle_state(&self, state: bool) {
        xhci_trace!("{}: set consumer cycle state: {}", self.name, state);
        // Fast because this should only happen during xhci setup.
        self.lock_ring_buffer().set_consumer_cycle_state(state);
    }

    /// Set whether every transfer descriptor the guest has queued is dequeued on each event
    /// instead of just the first one.
    ///
    /// An isochronous ring is drained ahead because a packet that reaches the host after its
    /// service interval is simply lost: real hardware walks the whole ring as frames tick, and
    /// handing the backend one descriptor at a time and waiting for its completion cannot keep an
    /// audio or video stream fed. Bulk, interrupt and control rings keep one descriptor per event.
    pub fn set_dequeue_all(&self, enabled: bool) {
        self.dequeue_all.store(enabled, Ordering::Relaxed);
    }

    /// Start the ring buffer.
    pub fn start(&self) {
        xhci_trace!("start {}", self.name);
        // A doorbell after the guest reset a halted endpoint means run; a halt that has not been
        // acted on yet is stale.
        self.halt_requested.store(false, Ordering::SeqCst);
        let mut state = self.state.lock();
        if *state != RingBufferState::Running {
            *state = RingBufferState::Running;
            if let Err(e) = self.event.signal() {
                error!("cannot start event ring: {}", e);
            }
        }
    }

    /// Park the ring because its endpoint just halted on an error. Hardware executes nothing
    /// more from a halted ring until software resets the endpoint and rings the doorbell, so the
    /// descriptors the guest queued behind the failed one must stay where they are. Nothing is in
    /// flight any more (the failing descriptor is the one completing), so any stop the guest is
    /// waiting on is answered now.
    pub fn halt(&self) {
        xhci_trace!("halt {}", self.name);
        // Parked from `on_event` rather than here: a transfer can complete synchronously from
        // inside the handler, on the event-loop thread that already holds `state`.
        self.halt_requested.store(true, Ordering::SeqCst);
        if let Err(e) = self.event.signal() {
            error!("cannot signal ring buffer controller halt: {}", e);
        }
    }

    /// Stop the ring buffer asynchronously.
    pub fn stop(&self, callback: RingBufferStopCallback) {
        xhci_trace!("stop {}", self.name);
        let mut state = self.state.lock();
        if *state == RingBufferState::Stopped {
            info!("xhci: {} is already stopped", self.name);
            return;
        }
        // Only wait when there is something to wait for. A ring that is Running but idle (the
        // guest rang the doorbell and nothing was queued, or a descriptor was answered without a
        // completion signal) has no completion coming to move it to Stopped, and the guest's Stop
        // Endpoint command would never be answered.
        let handler = self.handler.lock();
        if handler.stop() && !handler.is_quiesced() {
            *state = RingBufferState::Stopping;
            self.stop_callback.lock().push(callback);
        } else {
            *state = RingBufferState::Stopped;
        }
    }
}

impl<T> Drop for RingBufferController<T>
where
    T: 'static + TransferDescriptorHandler,
{
    fn drop(&mut self) {
        // Remove self from the event loop.
        if let Err(e) = self.event_loop.remove_event_for_descriptor(&self.event) {
            error!(
                "cannot remove ring buffer controller from event loop: {}",
                e
            );
        }
    }
}

impl<T> EventHandler for RingBufferController<T>
where
    T: 'static + TransferDescriptorHandler + Send,
{
    fn on_event(&self) -> anyhow::Result<()> {
        // `self.event` triggers ring buffer controller to run.
        self.event.wait().context("cannot read from event")?;
        if self.halt_requested.swap(false, Ordering::SeqCst) {
            // The endpoint halted on an error: hardware executes nothing more from this ring
            // until software resets the endpoint and rings the doorbell (spec 4.8.3). Nothing
            // is in flight any more, so a stop the guest is waiting on is answered now.
            let mut state = self.state.lock();
            *state = RingBufferState::Stopped;
            self.stop_callback.lock().clear();
            return Ok(());
        }
        let dequeue_all = self.dequeue_all.load(Ordering::Relaxed);
        let mut state = self.state.lock();

        match *state {
            RingBufferState::Stopped => return Ok(()),
            RingBufferState::Stopping => {
                // Reaching Stopped releases the stop callback, which is the guest's answer that
                // nothing is outstanding any more. A drained-ahead ring has several descriptors in
                // flight at once, so hold that answer back until the handler is done with all of
                // them; each of their completions signals us again.
                if dequeue_all && !self.handler.lock().is_quiesced() {
                    return Ok(());
                }
                debug!("xhci: {}: stopping ring buffer controller", self.name);
                *state = RingBufferState::Stopped;
                self.stop_callback.lock().clear();
                return Ok(());
            }
            RingBufferState::Running => {}
        }

        let transfer_descriptor = self
            .lock_ring_buffer()
            .dequeue_transfer_descriptor()
            .context("cannot dequeue transfer descriptor")?;

        let transfer_descriptor = match transfer_descriptor {
            Some(t) => t,
            None => {
                // An empty ring is not a quiescent one while descriptors dequeued ahead are still
                // in flight: parking in Stopped here would both release the stop callback early
                // and let a later `stop()` take its `already stopped` path without cancelling
                // them. Stay Running and let their completions bring us back.
                if !dequeue_all || self.handler.lock().is_quiesced() {
                    debug!(
                        "xhci: {}: ring empty at {:#x}, parking",
                        self.name,
                        self.get_dequeue_pointer().0
                    );
                    *state = RingBufferState::Stopped;
                    self.stop_callback.lock().clear();
                }
                return Ok(());
            }
        };

        let event = self.event.try_clone().context("cannot clone event")?;
        self.handler
            .lock()
            .handle_transfer_descriptor(transfer_descriptor, event)?;

        if dequeue_all {
            // Keep handing descriptors off until the ring runs dry. The state is deliberately left
            // Running in here: only the first dequeue of an event may park the controller, so an
            // empty ring on the next completion still stops it and a doorbell restarts it.
            let mut dequeued: usize = 1;
            loop {
                if dequeued >= MAX_DEQUEUE_PER_EVENT {
                    // Whatever is left stays on the ring. Signal ourselves so it is picked up on
                    // the next pass with `state` released in between, giving the rest of the event
                    // loop -- and any vcpu blocked on a doorbell -- a turn.
                    self.event.signal().context("cannot signal event")?;
                    break;
                }

                let transfer_descriptor = self
                    .lock_ring_buffer()
                    .dequeue_transfer_descriptor()
                    .context("cannot dequeue transfer descriptor")?;

                let transfer_descriptor = match transfer_descriptor {
                    Some(t) => t,
                    None => break,
                };

                let event = self.event.try_clone().context("cannot clone event")?;
                self.handler
                    .lock()
                    .handle_transfer_descriptor(transfer_descriptor, event)?;
                dequeued += 1;
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::mem::size_of;
    use std::sync::mpsc::channel;
    use std::sync::mpsc::Sender;

    use base::pagesize;

    use super::super::xhci_abi::LinkTrb;
    use super::super::xhci_abi::NormalTrb;
    use super::super::xhci_abi::Trb;
    use super::super::xhci_abi::TrbType;
    use super::*;

    struct TestHandler {
        sender: Sender<i32>,
    }

    impl TransferDescriptorHandler for TestHandler {
        fn handle_transfer_descriptor(
            &self,
            descriptor: TransferDescriptor,
            complete_event: Event,
        ) -> anyhow::Result<()> {
            for atrb in descriptor {
                assert_eq!(atrb.trb.get_trb_type().unwrap(), TrbType::Normal);
                self.sender.send(atrb.trb.get_parameter() as i32).unwrap();
            }
            complete_event.signal().unwrap();
            Ok(())
        }
    }

    fn setup_mem() -> GuestMemory {
        let trb_size = size_of::<Trb>() as u64;
        let gm = GuestMemory::new(&[(GuestAddress(0), pagesize() as u64)]).unwrap();

        // Structure of ring buffer:
        //  0x100  --> 0x200  --> 0x300
        //  trb 1  |   trb 3  |   trb 5
        //  trb 2  |   trb 4  |   trb 6
        //  l trb  -   l trb  -   l trb to 0x100
        let mut trb = NormalTrb::new();
        trb.set_trb_type(TrbType::Normal);
        trb.set_data_buffer(1);
        trb.set_chain(true);
        gm.write_obj_at_addr(trb, GuestAddress(0x100)).unwrap();

        trb.set_data_buffer(2);
        gm.write_obj_at_addr(trb, GuestAddress(0x100 + trb_size))
            .unwrap();

        let mut ltrb = LinkTrb::new();
        ltrb.set_trb_type(TrbType::Link);
        ltrb.set_ring_segment_pointer(0x200);
        gm.write_obj_at_addr(ltrb, GuestAddress(0x100 + 2 * trb_size))
            .unwrap();

        trb.set_data_buffer(3);
        gm.write_obj_at_addr(trb, GuestAddress(0x200)).unwrap();

        // Chain bit is false.
        trb.set_data_buffer(4);
        trb.set_chain(false);
        gm.write_obj_at_addr(trb, GuestAddress(0x200 + 1 * trb_size))
            .unwrap();

        ltrb.set_ring_segment_pointer(0x300);
        gm.write_obj_at_addr(ltrb, GuestAddress(0x200 + 2 * trb_size))
            .unwrap();

        trb.set_data_buffer(5);
        trb.set_chain(true);
        gm.write_obj_at_addr(trb, GuestAddress(0x300)).unwrap();

        // Chain bit is false.
        trb.set_data_buffer(6);
        trb.set_chain(false);
        gm.write_obj_at_addr(trb, GuestAddress(0x300 + 1 * trb_size))
            .unwrap();

        ltrb.set_ring_segment_pointer(0x100);
        gm.write_obj_at_addr(ltrb, GuestAddress(0x300 + 2 * trb_size))
            .unwrap();
        gm
    }

    #[test]
    fn test_ring_buffer_controller() {
        let (tx, rx) = channel();
        let mem = setup_mem();
        let (l, j) = EventLoop::start("test".to_string(), None).unwrap();
        let l = Arc::new(l);
        let controller = RingBufferController::new_with_handler(
            "".to_string(),
            mem,
            l.clone(),
            TestHandler { sender: tx },
        )
        .unwrap();
        controller.set_dequeue_pointer(GuestAddress(0x100));
        controller.set_consumer_cycle_state(false);
        controller.start();
        assert_eq!(rx.recv().unwrap(), 1);
        assert_eq!(rx.recv().unwrap(), 2);
        assert_eq!(rx.recv().unwrap(), 3);
        assert_eq!(rx.recv().unwrap(), 4);
        assert_eq!(rx.recv().unwrap(), 5);
        assert_eq!(rx.recv().unwrap(), 6);
        l.stop();
        j.join().unwrap();
    }

    /// Hands each descriptor's completion event back to the test instead of signalling it, so
    /// the test decides when a transfer "completes".
    struct HeldHandler {
        sender: Sender<(i32, Event)>,
    }

    impl TransferDescriptorHandler for HeldHandler {
        fn handle_transfer_descriptor(
            &self,
            descriptor: TransferDescriptor,
            complete_event: Event,
        ) -> anyhow::Result<()> {
            let first = descriptor[0].trb.get_parameter() as i32;
            self.sender.send((first, complete_event)).unwrap();
            Ok(())
        }
    }

    #[test]
    fn halt_parks_the_ring_until_the_next_doorbell() {
        let (tx, rx) = channel();
        let mem = setup_mem();
        let (l, j) = EventLoop::start("test".to_string(), None).unwrap();
        let l = Arc::new(l);
        let controller = RingBufferController::new_with_handler(
            "".to_string(),
            mem,
            l.clone(),
            HeldHandler { sender: tx },
        )
        .unwrap();
        controller.set_dequeue_pointer(GuestAddress(0x100));
        controller.set_consumer_cycle_state(false);
        controller.start();
        let (first, complete) = rx.recv().unwrap();
        assert_eq!(first, 1);

        // The endpoint halts on this descriptor: park the ring, then let the completion arrive.
        // The descriptor queued behind it (5, 6) must stay on the ring.
        controller.halt();
        complete.signal().unwrap();
        assert!(
            rx.recv_timeout(std::time::Duration::from_millis(200))
                .is_err(),
            "a halted ring must not hand out the next descriptor"
        );

        // Software resets the endpoint and rings the doorbell: the ring resumes where it was.
        controller.start();
        let (next, _) = rx.recv().unwrap();
        assert_eq!(next, 5);
        l.stop();
        j.join().unwrap();
    }
}
