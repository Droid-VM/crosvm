// Copyright 2019 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use std::time::Duration;
use std::time::Instant;

use base::Clock;
use base::Error as SysError;
use base::Event;
use base::Timer;
use base::TimerTrait;
use remain::sorted;
use thiserror::Error;
use vm_memory::GuestAddress;
use vm_memory::GuestMemory;

use super::event_ring::Error as EventRingError;
use super::event_ring::EventRing;
use super::xhci_abi::CommandCompletionEventTrb;
use super::xhci_abi::Error as TrbError;
use super::xhci_abi::PortStatusChangeEventTrb;
use super::xhci_abi::TransferEventTrb;
use super::xhci_abi::Trb;
use super::xhci_abi::TrbCast;
use super::xhci_abi::TrbCompletionCode;
use super::xhci_abi::TrbType;
use super::xhci_regs::XhciRegs;
use super::xhci_regs::ERDP_EVENT_HANDLER_BUSY;
use super::xhci_regs::IMAN_INTERRUPT_PENDING;
use super::xhci_regs::USB_STS_EVENT_INTERRUPT;
use crate::register_space::Register;

#[sorted]
#[derive(Error, Debug)]
pub enum Error {
    #[error("cannot add event: {0}")]
    AddEvent(EventRingError),
    #[error("cannot arm interrupt moderation timer: {0}")]
    ArmModerationTimer(SysError),
    #[error("cannot cast trb: {0}")]
    CastTrb(TrbError),
    #[error("cannot create interrupt moderation timer: {0}")]
    CreateModerationTimer(SysError),
    #[error("cannot send interrupt: {0}")]
    SendInterrupt(SysError),
    #[error("cannot set seg table base addr: {0}")]
    SetSegTableBaseAddr(EventRingError),
    #[error("cannot set seg table size: {0}")]
    SetSegTableSize(EventRingError),
    #[error("cannot wait on interrupt moderation timer: {0}")]
    WaitModerationTimer(SysError),
}

type Result<T> = std::result::Result<T, Error>;

/// See spec 4.17 for interrupters. Controller can send an event back to guest kernel driver
/// through interrupter.
pub struct Interrupter {
    interrupt_evt: Event,
    usbsts: Register<u32>,
    iman: Register<u32>,
    erdp: Register<u64>,
    event_handler_busy: bool,
    enabled: bool,
    moderation_interval: u16,
    moderation_counter: u16,
    /// Fires when the current moderation window ends, so an event posted inside the window is
    /// still delivered once it closes. Real hardware holds the interrupt back until the window
    /// ends; it never forgets it.
    moderation_timer: Timer,
    /// True while `moderation_timer` is armed for the end of the current window, so the many
    /// events that a busy endpoint posts inside one window cost one timer arm, not one each.
    moderation_timer_armed: bool,
    event_ring: EventRing,
    last_interrupt_time: Instant,
    clock: Clock,
}

impl Interrupter {
    /// Create a new interrupter.
    pub fn new(mem: GuestMemory, irq_evt: Event, regs: &XhciRegs) -> Result<Self> {
        let clock = Clock::new();
        let moderation_timer = Timer::new().map_err(Error::CreateModerationTimer)?;
        Ok(Interrupter {
            interrupt_evt: irq_evt,
            usbsts: regs.usbsts.clone(),
            iman: regs.iman.clone(),
            erdp: regs.erdp.clone(),
            event_handler_busy: false,
            enabled: false,
            moderation_interval: 4000, // default to 1ms as per xhci 5.5.2.2
            moderation_counter: 0,     // xhci specs leave this as undefined
            moderation_timer,
            moderation_timer_armed: false,
            event_ring: EventRing::new(mem),
            last_interrupt_time: clock.now(),
            clock,
        })
    }

    /// The timer that ends a moderation window. The owner registers it on the event loop and
    /// calls `on_moderation_timer` when it fires.
    pub fn moderation_timer(&self) -> &Timer {
        &self.moderation_timer
    }

    /// The moderation window has ended: deliver whatever the window held back.
    pub fn on_moderation_timer(&mut self) -> Result<()> {
        self.moderation_timer
            .mark_waited()
            .map_err(Error::WaitModerationTimer)?;
        self.moderation_timer_armed = false;
        self.interrupt_if_needed()
    }

    /// Returns true if event ring is empty.
    pub fn event_ring_is_empty(&self) -> bool {
        self.event_ring.is_empty()
    }

    /// Add event to event ring.
    fn add_event(&mut self, trb: Trb) -> Result<()> {
        self.event_ring.add_event(trb).map_err(Error::AddEvent)?;
        self.interrupt_if_needed()
    }

    /// Send port status change trb for port.
    pub fn send_port_status_change_trb(&mut self, port_id: u8) -> Result<()> {
        let mut trb = Trb::new();
        let psctrb = trb
            .cast_mut::<PortStatusChangeEventTrb>()
            .map_err(Error::CastTrb)?;
        psctrb.set_port_id(port_id);
        psctrb.set_completion_code(TrbCompletionCode::Success);
        psctrb.set_trb_type(TrbType::PortStatusChangeEvent);
        self.add_event(trb)
    }

    /// Send command completion trb.
    pub fn send_command_completion_trb(
        &mut self,
        completion_code: TrbCompletionCode,
        slot_id: u8,
        trb_addr: GuestAddress,
    ) -> Result<()> {
        let mut trb = Trb::new();
        let ctrb = trb
            .cast_mut::<CommandCompletionEventTrb>()
            .map_err(Error::CastTrb)?;
        ctrb.set_trb_pointer(trb_addr.0);
        ctrb.set_command_completion_parameter(0);
        ctrb.set_completion_code(completion_code);
        ctrb.set_trb_type(TrbType::CommandCompletionEvent);
        ctrb.set_vf_id(0);
        ctrb.set_slot_id(slot_id);
        self.add_event(trb)
    }

    /// Send transfer event trb.
    pub fn send_transfer_event_trb(
        &mut self,
        completion_code: TrbCompletionCode,
        trb_pointer: u64,
        transfer_length: u32,
        event_data: bool,
        slot_id: u8,
        endpoint_id: u8,
    ) -> Result<()> {
        let mut trb = Trb::new();
        let event_trb = trb.cast_mut::<TransferEventTrb>().map_err(Error::CastTrb)?;
        event_trb.set_trb_pointer(trb_pointer);
        event_trb.set_trb_transfer_length(transfer_length);
        event_trb.set_completion_code(completion_code);
        event_trb.set_event_data(event_data.into());
        event_trb.set_trb_type(TrbType::TransferEvent);
        event_trb.set_endpoint_id(endpoint_id);
        event_trb.set_slot_id(slot_id);
        self.add_event(trb)
    }

    /// Enable/Disable this interrupter.
    pub fn set_enabled(&mut self, enabled: bool) -> Result<()> {
        xhci_trace!("interrupter set_enabled({})", enabled);
        self.enabled = enabled;
        self.interrupt_if_needed()
    }

    /// Set interrupt moderation.
    pub fn set_moderation(&mut self, interval: u16, counter: u16) -> Result<()> {
        xhci_trace!("interrupter set_moderation({}, {})", interval, counter);
        self.moderation_interval = interval;
        self.moderation_counter = counter;
        // The window just changed length; let the next check arm the timer for the new end.
        self.moderation_timer_armed = false;
        self.interrupt_if_needed()
    }

    /// Set event ring seg table size.
    pub fn set_event_ring_seg_table_size(&mut self, size: u16) -> Result<()> {
        xhci_trace!("interrupter set_event_ring_seg_table_size({})", size);
        self.event_ring
            .set_seg_table_size(size)
            .map_err(Error::SetSegTableSize)
    }

    /// Set event ring segment table base address.
    pub fn set_event_ring_seg_table_base_addr(&mut self, addr: GuestAddress) -> Result<()> {
        xhci_trace!("interrupter set_table_base_addr({:#x})", addr.0);
        self.event_ring
            .set_seg_table_base_addr(addr)
            .map_err(Error::SetSegTableBaseAddr)
    }

    /// Set event ring dequeue pointer.
    pub fn set_event_ring_dequeue_pointer(&mut self, addr: GuestAddress, busy: bool) -> Result<()> {
        xhci_trace!(
            "interrupter set_dequeue_pointer(addr = {:#x}, busy = {})",
            addr.0,
            busy
        );
        self.event_ring.set_dequeue_pointer(addr);
        self.event_handler_busy = busy;
        self.interrupt_if_needed()
    }

    /// Send and interrupt.
    pub fn interrupt(&mut self) -> Result<()> {
        self.event_handler_busy = true;
        self.usbsts.set_bits(USB_STS_EVENT_INTERRUPT);
        self.iman.set_bits(IMAN_INTERRUPT_PENDING);
        self.erdp.set_bits(ERDP_EVENT_HANDLER_BUSY);
        self.moderation_counter = self.moderation_interval;
        self.last_interrupt_time = self.clock.now();
        self.interrupt_evt.signal().map_err(Error::SendInterrupt)
    }

    fn interrupt_interval(&self) -> Duration {
        // Formula from xhci spec 4.17.2 in nanoseconds, but we use the imodc value instead of the
        // imodi value because our implementation automatically adjusts the range of the duration
        // based on the remaining time left in the moderation counter, which may be software
        // defined.
        Duration::new(0, 250 * u32::from(self.moderation_counter))
    }

    fn interrupt_if_needed(&mut self) -> Result<()> {
        if !self.enabled || self.event_handler_busy || self.event_ring.is_empty() {
            return Ok(());
        }
        let elapsed = self.last_interrupt_time.elapsed();
        let interval = self.interrupt_interval();
        if elapsed >= interval {
            return self.interrupt();
        }
        // Inside the moderation window. Dropping the interrupt here would strand the event: a
        // guest waiting on exactly this completion, with nothing else in flight to post another
        // event, would wait forever (Windows' USBXHCI times its commands out after a few seconds
        // and resets the controller). Hold the interrupt until the window ends instead, the way
        // hardware does.
        if !self.moderation_timer_armed {
            self.moderation_timer
                .reset_oneshot(interval - elapsed)
                .map_err(Error::ArmModerationTimer)?;
            self.moderation_timer_armed = true;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::mem::size_of;
    use std::thread;

    use base::pagesize;
    use base::EventWaitResult;

    use super::*;
    use crate::usb::xhci::xhci_abi::EventRingSegmentTableEntry;
    use crate::usb::xhci::xhci_regs::init_xhci_mmio_space_and_regs;

    const SEGMENT_BASE: u64 = 0x100;
    const TRB_SIZE: u64 = size_of::<Trb>() as u64;

    /// An enabled interrupter over a one-segment event ring, with the guest-side irq event.
    fn interrupter() -> (Interrupter, Event) {
        let gm = GuestMemory::new(&[(GuestAddress(0), pagesize() as u64)]).unwrap();
        let mut entry = EventRingSegmentTableEntry::new();
        entry.set_ring_segment_base_address(SEGMENT_BASE);
        entry.set_ring_segment_size(16);
        gm.write_obj_at_addr(entry, GuestAddress(0x8)).unwrap();
        let (_mmio, regs) = init_xhci_mmio_space_and_regs();
        let irq = Event::new().unwrap();
        let mut intr = Interrupter::new(gm, irq.try_clone().unwrap(), &regs).unwrap();
        intr.set_event_ring_seg_table_size(1).unwrap();
        intr.set_event_ring_seg_table_base_addr(GuestAddress(0x8))
            .unwrap();
        intr.set_event_ring_dequeue_pointer(GuestAddress(SEGMENT_BASE), false)
            .unwrap();
        intr.set_enabled(true).unwrap();
        (intr, irq)
    }

    fn signaled(irq: &Event) -> bool {
        matches!(
            irq.wait_timeout(Duration::from_millis(200)).unwrap(),
            EventWaitResult::Signaled
        )
    }

    #[test]
    fn no_moderation_interrupts_every_event() {
        let (mut intr, irq) = interrupter();
        intr.set_moderation(0, 0).unwrap();
        intr.send_port_status_change_trb(1).unwrap();
        assert!(signaled(&irq));
        // Guest consumes the event and clears EHB.
        intr.set_event_ring_dequeue_pointer(GuestAddress(SEGMENT_BASE + TRB_SIZE), false)
            .unwrap();
        intr.send_port_status_change_trb(2).unwrap();
        assert!(signaled(&irq));
    }

    #[test]
    fn event_inside_moderation_window_is_delivered_when_it_ends() {
        let (mut intr, irq) = interrupter();
        // Counter 0 lets the first event interrupt at once; the interrupt then opens an
        // 8 ms window (32000 * 250 ns) for everything after it.
        intr.set_moderation(32000, 0).unwrap();
        intr.send_port_status_change_trb(1).unwrap();
        assert!(signaled(&irq));

        // Guest handles that event, clears EHB, and a second event lands inside the window.
        intr.set_event_ring_dequeue_pointer(GuestAddress(SEGMENT_BASE + TRB_SIZE), false)
            .unwrap();
        intr.send_port_status_change_trb(2).unwrap();
        assert!(intr.moderation_timer_armed, "window must arm the timer");
        assert!(
            !matches!(
                irq.wait_timeout(Duration::from_millis(2)).unwrap(),
                EventWaitResult::Signaled
            ),
            "no interrupt inside the window"
        );

        // The window ends: the timer fires and the held-back event is delivered. Before the
        // fix nothing ever fired again and the guest waited forever.
        thread::sleep(Duration::from_millis(12));
        intr.on_moderation_timer().unwrap();
        assert!(!intr.moderation_timer_armed);
        assert!(
            signaled(&irq),
            "held-back event must interrupt once the window ends"
        );
    }

    #[test]
    fn many_events_in_one_window_arm_the_timer_once() {
        let (mut intr, irq) = interrupter();
        intr.set_moderation(32000, 0).unwrap();
        intr.send_port_status_change_trb(1).unwrap();
        assert!(signaled(&irq));
        intr.set_event_ring_dequeue_pointer(GuestAddress(SEGMENT_BASE + TRB_SIZE), false)
            .unwrap();
        for port in 2..6 {
            intr.send_port_status_change_trb(port).unwrap();
        }
        assert!(intr.moderation_timer_armed);
        thread::sleep(Duration::from_millis(12));
        intr.on_moderation_timer().unwrap();
        assert!(signaled(&irq));
    }
}
