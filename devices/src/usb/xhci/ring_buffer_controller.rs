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

    /// Called once, when a ring that was Stopping parks, before the stop callback is released.
    /// A handler that had a descriptor in progress reports it here -- a transfer ring sends its
    /// Stopped Transfer Event (spec 4.6.9), which therefore precedes the Stop Endpoint Command
    /// Completion -- and returns where the ring is to be left: at that descriptor's first TRB,
    /// with the cycle state the ring had there, so it runs again from there unless software
    /// moves the ring with Set TR Dequeue Pointer. Of several descriptors in flight (a
    /// drained-ahead ring) it is the earliest; the later ones stay on the ring behind it. `None`
    /// leaves the ring where it is. A halted ring is never asked: its failing descriptor was
    /// already reported.
    fn finish_stop(&self) -> anyhow::Result<Option<StoppedTd>> {
        Ok(None)
    }
}

/// Where a stopped ring is left (spec 4.6.9): the first TRB of the descriptor that was in
/// progress, the cycle state the ring had there, and what that descriptor had moved -- its
/// Stopped EDTLA (6.2.4.1).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StoppedTd {
    pub first_trb: GuestAddress,
    pub cycle: bool,
    pub bytes: u32,
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
    /// The descriptor the last stop left the ring at; `None` when it stopped with nothing in
    /// progress, or has run since.
    stopped: Mutex<Option<StoppedTd>>,
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
            stopped: Mutex::new(None),
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

    /// The descriptor the last stop left the ring at (spec 4.6.9), for the context write-back
    /// that follows the stop; `None` when the ring stopped with nothing in progress, or has run
    /// since.
    pub fn stopped_td(&self) -> Option<StoppedTd> {
        *self.stopped.lock()
    }

    /// Start the ring buffer.
    pub fn start(&self) {
        xhci_trace!("start {}", self.name);
        // A doorbell after the guest reset a halted endpoint means run; a halt that has not been
        // acted on yet is stale.
        self.halt_requested.store(false, Ordering::SeqCst);
        // So is where the last stop left the ring: it is about to move.
        *self.stopped.lock() = None;
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
            debug!("xhci: {} is already stopped", self.name);
            return;
        }
        // Only wait when there is something to wait for. A ring that is Running but idle (the
        // guest rang the doorbell and nothing was queued, or a descriptor was answered without a
        // completion signal) has no completion coming to move it to Stopped, and the guest's Stop
        // Endpoint command would never be answered.
        let pending = {
            let handler = self.handler.lock();
            handler.stop() && !handler.is_quiesced()
        };
        if pending {
            *state = RingBufferState::Stopping;
            self.stop_callback.lock().push(callback);
        } else {
            // Parked here rather than from `on_event`, so the stops already waiting on this
            // ring -- it was Stopping, its last transfer completed and signalled, and the
            // completion has not been dispatched yet -- are answered now, ahead of this one
            // (`callback` drops last, on return). A Stopped ring never keeps a callback: one
            // left behind would fire whenever the ring is dropped, wherever that happens.
            *state = RingBufferState::Stopped;
            self.stop_callback.lock().clear();
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
                // The descriptor in progress is reported Stopped and the ring left at it (spec
                // 4.6.9) before the stop callback below answers the guest, so the Transfer
                // Event precedes the Stop Endpoint Command Completion.
                let stopped = self
                    .handler
                    .lock()
                    .finish_stop()
                    .context("cannot finish stop")?;
                if let Some(td) = &stopped {
                    debug!(
                        "xhci: {}: stopped at {:#x} (cycle {}) with {} bytes of the descriptor moved",
                        self.name, td.first_trb.0, td.cycle, td.bytes
                    );
                    let mut ring_buffer = self.lock_ring_buffer();
                    ring_buffer.set_dequeue_pointer(td.first_trb);
                    ring_buffer.set_consumer_cycle_state(td.cycle);
                }
                *self.stopped.lock() = stopped;
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

    /// The command ring's shape: a handler that tracks nothing across a descriptor (the trait's
    /// default `stop` and `is_quiesced`), so a ring that is Running with nothing to wait for
    /// stops the moment it is asked -- a host controller reset must not hang on it -- and the
    /// pointer set afterwards is where the next start runs from.
    #[test]
    fn stop_of_an_idle_running_ring_completes_at_once_and_restarts_from_the_new_pointer() {
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
        let (first, _held) = rx.recv().unwrap();
        assert_eq!(first, 1);

        // Running, and the handler reports nothing to wait for: the stop callback fires before
        // `stop` returns, on this thread.
        let stopped = Arc::new(AtomicBool::new(false));
        let flag = stopped.clone();
        controller.stop(RingBufferStopCallback::new(move || {
            flag.store(true, Ordering::SeqCst);
        }));
        assert!(
            stopped.load(Ordering::SeqCst),
            "stopping an idle ring must complete synchronously"
        );

        // The guest programs a new ring (CRCR after HCRST) and rings the doorbell.
        controller.set_dequeue_pointer(GuestAddress(0x300));
        controller.set_consumer_cycle_state(false);
        controller.start();
        let (next, _) = rx.recv().unwrap();
        assert_eq!(next, 5, "the restarted ring must run from the new pointer");
        l.stop();
        j.join().unwrap();
    }

    /// A transfer ring's shape: the handler reports whether a transfer is still in flight, and
    /// the test decides; asked to finish a stop it answers with the position the test preset
    /// and notes the call in `order`, against the stop callback.
    struct TrackedHandler {
        sender: Sender<(i32, Event)>,
        quiesced: Arc<AtomicBool>,
        rewind: Arc<Mutex<Option<StoppedTd>>>,
        order: Arc<Mutex<Vec<&'static str>>>,
    }

    impl TrackedHandler {
        fn new(
            sender: Sender<(i32, Event)>,
            quiesced: &Arc<AtomicBool>,
            rewind: &Arc<Mutex<Option<StoppedTd>>>,
            order: &Arc<Mutex<Vec<&'static str>>>,
        ) -> TrackedHandler {
            TrackedHandler {
                sender,
                quiesced: quiesced.clone(),
                rewind: rewind.clone(),
                order: order.clone(),
            }
        }
    }

    impl TransferDescriptorHandler for TrackedHandler {
        fn handle_transfer_descriptor(
            &self,
            descriptor: TransferDescriptor,
            complete_event: Event,
        ) -> anyhow::Result<()> {
            let first = descriptor[0].trb.get_parameter() as i32;
            self.sender.send((first, complete_event)).unwrap();
            Ok(())
        }

        fn is_quiesced(&self) -> bool {
            self.quiesced.load(Ordering::SeqCst)
        }

        fn finish_stop(&self) -> anyhow::Result<Option<StoppedTd>> {
            self.order.lock().push("stopped");
            Ok(*self.rewind.lock())
        }
    }

    /// Waits for `done` on the event loop, up to a few seconds.
    fn wait_for(done: impl Fn() -> bool) {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(5);
        while !done() {
            assert!(std::time::Instant::now() < deadline, "timed out");
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
    }

    /// Three one-TRB descriptors (1, 2, 3 at 0x100, 0x110, 0x120) and a link back to 0x100 that
    /// toggles the cycle, so a drained-ahead ring runs dry after them.
    fn setup_mem_three_tds() -> GuestMemory {
        let trb_size = size_of::<Trb>() as u64;
        let gm = GuestMemory::new(&[(GuestAddress(0), pagesize() as u64)]).unwrap();
        for i in 0..3u64 {
            let mut trb = NormalTrb::new();
            trb.set_trb_type(TrbType::Normal);
            trb.set_data_buffer(i + 1);
            trb.set_chain(false);
            gm.write_obj_at_addr(trb, GuestAddress(0x100 + i * trb_size))
                .unwrap();
        }
        let mut ltrb = LinkTrb::new();
        ltrb.set_trb_type(TrbType::Link);
        ltrb.set_ring_segment_pointer(0x100);
        ltrb.set_toggle_cycle(true);
        gm.write_obj_at_addr(ltrb, GuestAddress(0x100 + 3 * trb_size))
            .unwrap();
        gm
    }

    /// A Stop Endpoint with a transfer in flight: when the cancelled transfer completes the ring
    /// reports it through the handler, is left at its first TRB, and only then answers the stop
    /// -- the Stopped Transfer Event precedes the Command Completion (spec 4.6.9). A doorbell
    /// without a Set TR Dequeue Pointer runs the descriptor again.
    #[test]
    fn stop_with_a_transfer_in_flight_rewinds_to_it_and_reports_before_the_callback() {
        let (tx, rx) = channel();
        let mem = setup_mem();
        let (l, j) = EventLoop::start("test".to_string(), None).unwrap();
        let l = Arc::new(l);
        let quiesced = Arc::new(AtomicBool::new(false));
        let rewind = Arc::new(Mutex::new(None));
        let order = Arc::new(Mutex::new(Vec::new()));
        let controller = RingBufferController::new_with_handler(
            "".to_string(),
            mem,
            l.clone(),
            TrackedHandler::new(tx, &quiesced, &rewind, &order),
        )
        .unwrap();
        controller.set_dequeue_pointer(GuestAddress(0x100));
        controller.set_consumer_cycle_state(false);
        controller.start();
        let (first, complete) = rx.recv().unwrap();
        assert_eq!(first, 1);
        // TD 1 (TRBs 1..4, over the link at 0x120) is dequeued: the ring stands past it.
        assert_eq!(controller.get_dequeue_pointer(), GuestAddress(0x220));
        assert_eq!(controller.stopped_td(), None);

        let stopped = StoppedTd {
            first_trb: GuestAddress(0x100),
            cycle: false,
            bytes: 0x40,
        };
        *rewind.lock() = Some(stopped);
        let o = order.clone();
        controller.stop(RingBufferStopCallback::new(move || {
            o.lock().push("callback")
        }));
        assert!(order.lock().is_empty(), "the stop waits for the transfer");

        // The cancelled transfer completes.
        complete.signal().unwrap();
        wait_for(|| order.lock().len() == 2);
        assert_eq!(*order.lock(), vec!["stopped", "callback"]);
        assert_eq!(controller.get_dequeue_pointer(), GuestAddress(0x100));
        assert!(!controller.get_consumer_cycle_state());
        assert_eq!(controller.stopped_td(), Some(stopped));

        controller.start();
        let (again, _) = rx.recv().unwrap();
        assert_eq!(again, 1, "the stopped descriptor runs again");
        assert_eq!(controller.stopped_td(), None);
        l.stop();
        j.join().unwrap();
    }

    /// A drained-ahead ring with three descriptors in flight is stopped; the first completed on
    /// the device, the other two were cancelled. The ring reports once, when the last of them
    /// is done, and is left at the earliest unfinished descriptor; the one behind it stays on
    /// the ring and runs after it.
    #[test]
    fn stop_of_a_drained_ring_rewinds_to_the_earliest_unfinished_descriptor() {
        let (tx, rx) = channel();
        let mem = setup_mem_three_tds();
        let (l, j) = EventLoop::start("test".to_string(), None).unwrap();
        let l = Arc::new(l);
        let quiesced = Arc::new(AtomicBool::new(false));
        let rewind = Arc::new(Mutex::new(None));
        let order = Arc::new(Mutex::new(Vec::new()));
        let controller = RingBufferController::new_with_handler(
            "".to_string(),
            mem,
            l.clone(),
            TrackedHandler::new(tx, &quiesced, &rewind, &order),
        )
        .unwrap();
        controller.set_dequeue_all(true);
        controller.set_dequeue_pointer(GuestAddress(0x100));
        controller.set_consumer_cycle_state(false);
        controller.start();
        let (_, done1) = rx.recv().unwrap();
        let (second, done2) = rx.recv().unwrap();
        let (third, done3) = rx.recv().unwrap();
        assert_eq!((second, third), (2, 3));

        let o = order.clone();
        controller.stop(RingBufferStopCallback::new(move || {
            o.lock().push("callback")
        }));
        // TD 2 is the earliest the handler could not finish.
        let stopped = StoppedTd {
            first_trb: GuestAddress(0x110),
            cycle: false,
            bytes: 0,
        };
        *rewind.lock() = Some(stopped);
        done1.signal().unwrap();
        done2.signal().unwrap();
        std::thread::sleep(std::time::Duration::from_millis(100));
        assert!(
            order.lock().is_empty(),
            "two descriptors still in flight: the stop waits"
        );

        quiesced.store(true, Ordering::SeqCst);
        done3.signal().unwrap();
        wait_for(|| order.lock().len() == 2);
        assert_eq!(
            *order.lock(),
            vec!["stopped", "callback"],
            "reported exactly once, before the callback"
        );
        assert_eq!(controller.get_dequeue_pointer(), GuestAddress(0x110));
        assert!(!controller.get_consumer_cycle_state());
        assert_eq!(controller.stopped_td(), Some(stopped));

        controller.start();
        let (next, _) = rx.recv().unwrap();
        assert_eq!(next, 2);
        let (next, _) = rx.recv().unwrap();
        assert_eq!(
            next, 3,
            "the descriptor behind the stopped one is still there"
        );
        l.stop();
        j.join().unwrap();
    }

    /// A halt parks the ring where it stands: the failing descriptor was already reported, so
    /// the handler is not asked and nothing is rewound.
    #[test]
    fn halt_does_not_rewind() {
        let (tx, rx) = channel();
        let mem = setup_mem();
        let (l, j) = EventLoop::start("test".to_string(), None).unwrap();
        let l = Arc::new(l);
        let quiesced = Arc::new(AtomicBool::new(false));
        let rewind = Arc::new(Mutex::new(None));
        let order = Arc::new(Mutex::new(Vec::new()));
        let controller = RingBufferController::new_with_handler(
            "".to_string(),
            mem,
            l.clone(),
            TrackedHandler::new(tx, &quiesced, &rewind, &order),
        )
        .unwrap();
        controller.set_dequeue_pointer(GuestAddress(0x100));
        controller.set_consumer_cycle_state(false);
        controller.start();
        let (first, complete) = rx.recv().unwrap();
        assert_eq!(first, 1);
        *rewind.lock() = Some(StoppedTd {
            first_trb: GuestAddress(0x100),
            cycle: false,
            bytes: 0,
        });

        controller.halt();
        complete.signal().unwrap();
        assert!(
            rx.recv_timeout(std::time::Duration::from_millis(200))
                .is_err(),
            "a halted ring must not hand out the next descriptor"
        );
        assert!(order.lock().is_empty(), "the handler is not asked");
        assert_eq!(controller.get_dequeue_pointer(), GuestAddress(0x220));
        assert_eq!(controller.stopped_td(), None);

        controller.start();
        let (next, _) = rx.recv().unwrap();
        assert_eq!(next, 5);
        l.stop();
        j.join().unwrap();
    }

    /// A ring that is Stopping (a Stop Endpoint waits on it) whose transfer has completed but
    /// whose completion has not been dispatched yet is asked to stop again (HCRST). It parks
    /// at once -- and answers the stop it already held, in order, before the new one. Before
    /// the fix the first callback stayed in the parked ring and fired whenever the ring was
    /// dropped, from under the slot lock during the reset.
    #[test]
    fn stop_of_a_stopping_ring_that_went_quiet_answers_every_waiting_stop() {
        let (tx, rx) = channel();
        let mem = setup_mem();
        let (l, j) = EventLoop::start("test".to_string(), None).unwrap();
        let l = Arc::new(l);
        let quiesced = Arc::new(AtomicBool::new(true));
        let controller = RingBufferController::new_with_handler(
            "".to_string(),
            mem,
            l.clone(),
            TrackedHandler::new(
                tx,
                &quiesced,
                &Arc::new(Mutex::new(None)),
                &Arc::new(Mutex::new(Vec::new())),
            ),
        )
        .unwrap();
        controller.set_dequeue_pointer(GuestAddress(0x100));
        controller.set_consumer_cycle_state(false);
        controller.start();
        let (first, _held) = rx.recv().unwrap();
        assert_eq!(first, 1);

        // In flight: the first stop waits.
        quiesced.store(false, Ordering::SeqCst);
        let order = Arc::new(Mutex::new(Vec::new()));
        let o = order.clone();
        controller.stop(RingBufferStopCallback::new(move || {
            o.lock().push("endpoint")
        }));
        assert!(
            order.lock().is_empty(),
            "a stop with a transfer in flight must wait"
        );

        // The transfer completes (the completion is not dispatched: nothing signals the ring)
        // and a reset asks the ring to stop again.
        quiesced.store(true, Ordering::SeqCst);
        let o = order.clone();
        controller.stop(RingBufferStopCallback::new(move || o.lock().push("reset")));
        assert_eq!(
            *order.lock(),
            vec!["endpoint", "reset"],
            "both stops are answered before `stop` returns, the older one first"
        );
        l.stop();
        j.join().unwrap();
    }
}
