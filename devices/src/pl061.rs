// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Derived from QEMU (GPL-2.0-or-later): hw/gpio/pl061.c, non-Luminary variant.
// The additional permissions do not extend to QEMU copyright in this file.
//
// This file previously carried a "Copyright 2024 The ChromiumOS Authors" BSD notice.
// That was boilerplate copied from a sibling file; ChromiumOS did not write this one.

//! ARM PrimeCell PL061 General Purpose IO controller.
//!
//! Only the functionality required to drive a `gpio-keys` power/sleep button is
//! implemented: the register model and interrupt-generation logic match the
//! upstream QEMU `hw/gpio/pl061.c` device (non-Luminary variant). The
//! [`PmResource`] implementation drives input pins high for a short window
//! (matching QEMU's `gpio-key` helper) so the guest's `gpio-keys` driver sees a
//! complete press/release and reports `KEY_POWER` / `KEY_SLEEP`.
//!
//! The PL061's aggregate interrupt is delivered to the interrupt controller as
//! an *edge* event (one injection per rising transition of the masked interrupt
//! status). This matches the other aarch64 platform devices (RTC, vmwdt); the
//! Gunyah irqchip only properly supports edge irqfds.

use std::sync::Arc;
use std::time::Duration;

use anyhow::Context;
use base::error;
use base::warn;
use base::Event;
use base::EventToken;
use base::Timer;
use base::TimerTrait;
use base::WaitContext;
use base::WorkerThread;
use serde::Deserialize;
use serde::Serialize;
use snapshot::AnySnapshot;
use sync::Mutex;
use vm_control::PmResource;

use crate::pci::CrosvmDeviceId;
use crate::BusAccessInfo;
use crate::BusDevice;
use crate::DeviceId;
use crate::IrqEdgeEvent;
use crate::Suspendable;

/// Number of GPIO lines provided by a single PL061.
const N_GPIOS: u32 = 8;

/// GPIO line used as the virtual power button.
pub const GPIO_PIN_POWER_BUTTON: u32 = 3;
/// GPIO line used as the virtual sleep button.
pub const GPIO_PIN_SLEEP_BUTTON: u32 = 4;

/// Size of the PL061 MMIO register window.
pub const PL061_MMIO_SIZE: u64 = 0x1000;

/// Duration the button line is held high, matching QEMU's gpio-key device.
const GPIO_KEY_LATENCY: Duration = Duration::from_millis(100);

// Register offsets (data register occupies 0x000..0x400 with banked addressing).
const GPIODIR: u64 = 0x400; // Direction
const GPIOIS: u64 = 0x404; // Interrupt sense
const GPIOIBE: u64 = 0x408; // Interrupt both edges
const GPIOIEV: u64 = 0x40c; // Interrupt event
const GPIOIE: u64 = 0x410; // Interrupt mask
const GPIORIS: u64 = 0x414; // Raw interrupt status
const GPIOMIS: u64 = 0x418; // Masked interrupt status
const GPIOIC: u64 = 0x41c; // Interrupt clear
const GPIOAFSEL: u64 = 0x420; // Alternate function select
const GPIO_ID_LOW: u64 = 0xfd0; // First ID register
const GPIO_ID_HIGH: u64 = 0xfff; // Last ID register

// AMBA peripheral + PrimeCell ID bytes for the non-Luminary PL061. Each byte is
// at a 4-byte stride starting at 0xfd0 (periphid 0x00041061, cellid 0xb105f00d).
const PL061_ID: [u8; 12] = [
    0x00, 0x00, 0x00, 0x00, 0x61, 0x10, 0x04, 0x00, 0x0d, 0xf0, 0x05, 0xb1,
];

/// AMBA peripheral id, also advertised via the `arm,primecell-periphid` FDT
/// property.
pub const PL061_AMBA_ID: u32 = 0x00041061;

#[derive(Default, Clone, Serialize, Deserialize)]
struct Pl061Regs {
    data: u8,
    old_in_data: u8,
    dir: u8,
    isense: u8,
    ibe: u8,
    iev: u8,
    im: u8,
    istate: u8,
    afsel: u8,
    // Whether the aggregate interrupt is currently considered asserted. Used to
    // inject exactly one edge per rising transition of `(istate & im)`.
    asserted: bool,
}

impl Pl061Regs {
    /// Recompute the interrupt state and, on a rising transition of the masked
    /// interrupt status, inject an edge interrupt. Mirrors `pl061_update()` in
    /// QEMU (input path only; no GPIO outputs are wired to consumers).
    fn update(&mut self, irq: &IrqEdgeEvent) {
        // Latch edge interrupts for input pins whose level changed.
        let changed = (self.old_in_data ^ self.data) & !self.dir;
        if changed != 0 {
            self.old_in_data = self.data;
            for i in 0..N_GPIOS {
                let mask = 1u8 << i;
                if changed & mask != 0 && self.isense & mask == 0 {
                    if self.ibe & mask != 0 {
                        // Any edge triggers the interrupt.
                        self.istate |= mask;
                    } else {
                        // Edge selected by IEV.
                        self.istate |= !(self.data ^ self.iev) & mask;
                    }
                }
            }
        }

        // Level-sensitive interrupts.
        self.istate |= !(self.data ^ self.iev) & self.isense;

        let active = self.istate & self.im != 0;
        if active && !self.asserted {
            if let Err(e) = irq.trigger() {
                error!("pl061: failed to assert interrupt: {}", e);
            }
        }
        self.asserted = active;
    }

    /// Drive an input pin high or low and refresh the interrupt state.
    fn set_pin(&mut self, pin: u32, level: bool, irq: &IrqEdgeEvent) {
        let mask = 1u8 << pin;
        // Only pins configured as inputs can be driven externally.
        if self.dir & mask == 0 {
            if level {
                self.data |= mask;
            } else {
                self.data &= !mask;
            }
            self.update(irq);
        }
    }

    fn read_reg(&self, offset: u64) -> u32 {
        match offset {
            // Data register: bits [9:2] of the offset mask the returned pins.
            0x0..=0x3ff => (self.data & ((offset >> 2) as u8)) as u32,
            GPIODIR => self.dir as u32,
            GPIOIS => self.isense as u32,
            GPIOIBE => self.ibe as u32,
            GPIOIEV => self.iev as u32,
            GPIOIE => self.im as u32,
            GPIORIS => self.istate as u32,
            GPIOMIS => (self.istate & self.im) as u32,
            GPIOAFSEL => self.afsel as u32,
            GPIO_ID_LOW..=GPIO_ID_HIGH => PL061_ID
                .get(((offset - GPIO_ID_LOW) >> 2) as usize)
                .copied()
                .unwrap_or(0) as u32,
            _ => {
                warn!("pl061: bad read offset {:#x}", offset);
                0
            }
        }
    }

    fn write_reg(&mut self, offset: u64, value: u32, irq: &IrqEdgeEvent) {
        let byte = value as u8;
        match offset {
            // Data register: only pins configured as outputs and selected by the
            // address mask are updated.
            0x0..=0x3ff => {
                let mask = ((offset >> 2) as u8) & self.dir;
                self.data = (self.data & !mask) | (byte & mask);
                self.update(irq);
            }
            GPIODIR => {
                self.dir = byte;
                self.update(irq);
            }
            GPIOIS => {
                self.isense = byte;
                self.update(irq);
            }
            GPIOIBE => {
                self.ibe = byte;
                self.update(irq);
            }
            GPIOIEV => {
                self.iev = byte;
                self.update(irq);
            }
            GPIOIE => {
                self.im = byte;
                self.update(irq);
            }
            GPIOIC => {
                self.istate &= !byte;
                self.update(irq);
            }
            GPIOAFSEL => {
                self.afsel = byte;
                self.update(irq);
            }
            _ => warn!("pl061: bad write offset {:#x}", offset),
        }
    }
}

#[derive(EventToken)]
enum Token {
    PowerPress,
    SleepPress,
    PowerRelease,
    SleepRelease,
    Kill,
}

#[allow(clippy::too_many_arguments)]
fn run_worker(
    regs: Arc<Mutex<Pl061Regs>>,
    irq: IrqEdgeEvent,
    power_press: Event,
    sleep_press: Event,
    mut power_timer: Timer,
    mut sleep_timer: Timer,
    kill_evt: Event,
) {
    let wait_ctx: WaitContext<Token> = match WaitContext::build_with(&[
        (&power_press, Token::PowerPress),
        (&sleep_press, Token::SleepPress),
        (&power_timer, Token::PowerRelease),
        (&sleep_timer, Token::SleepRelease),
        (&kill_evt, Token::Kill),
    ]) {
        Ok(pc) => pc,
        Err(e) => {
            error!("pl061: failed to build WaitContext: {}", e);
            return;
        }
    };

    'poll: loop {
        let events = match wait_ctx.wait() {
            Ok(v) => v,
            Err(e) => {
                error!("pl061: error while polling for events: {}", e);
                break;
            }
        };

        for event in events.iter().filter(|e| e.is_readable) {
            match event.token {
                Token::PowerPress => {
                    let _ = power_press.wait();
                    regs.lock().set_pin(GPIO_PIN_POWER_BUTTON, true, &irq);
                    if let Err(e) = power_timer.reset_oneshot(GPIO_KEY_LATENCY) {
                        error!("pl061: failed to arm power button timer: {}", e);
                    }
                }
                Token::PowerRelease => {
                    let _ = power_timer.mark_waited();
                    regs.lock().set_pin(GPIO_PIN_POWER_BUTTON, false, &irq);
                }
                Token::SleepPress => {
                    let _ = sleep_press.wait();
                    regs.lock().set_pin(GPIO_PIN_SLEEP_BUTTON, true, &irq);
                    if let Err(e) = sleep_timer.reset_oneshot(GPIO_KEY_LATENCY) {
                        error!("pl061: failed to arm sleep button timer: {}", e);
                    }
                }
                Token::SleepRelease => {
                    let _ = sleep_timer.mark_waited();
                    regs.lock().set_pin(GPIO_PIN_SLEEP_BUTTON, false, &irq);
                }
                Token::Kill => break 'poll,
            }
        }
    }
}

/// An emulated ARM PL061 GPIO controller wired up as a power/sleep button.
pub struct Pl061 {
    regs: Arc<Mutex<Pl061Regs>>,
    irq: IrqEdgeEvent,
    power_press: Event,
    sleep_press: Event,
    // Joined (after signalling the kill event) when the device is dropped.
    _worker: WorkerThread<()>,
}

impl Pl061 {
    /// Constructs a PL061 device. `irq` is the edge-triggered interrupt line
    /// routed to the interrupt controller.
    pub fn new(irq: IrqEdgeEvent) -> anyhow::Result<Pl061> {
        let regs = Arc::new(Mutex::new(Pl061Regs::default()));
        let power_press = Event::new().context("failed to create power button event")?;
        let sleep_press = Event::new().context("failed to create sleep button event")?;
        let power_timer = Timer::new().context("failed to create power button timer")?;
        let sleep_timer = Timer::new().context("failed to create sleep button timer")?;

        let worker_regs = regs.clone();
        let worker_irq = irq.try_clone().context("failed to clone irq event")?;
        let worker_power = power_press
            .try_clone()
            .context("failed to clone power button event")?;
        let worker_sleep = sleep_press
            .try_clone()
            .context("failed to clone sleep button event")?;
        let worker = WorkerThread::start("pl061 worker", move |kill_evt| {
            run_worker(
                worker_regs,
                worker_irq,
                worker_power,
                worker_sleep,
                power_timer,
                sleep_timer,
                kill_evt,
            )
        });

        Ok(Pl061 {
            regs,
            irq,
            power_press,
            sleep_press,
            _worker: worker,
        })
    }
}

impl BusDevice for Pl061 {
    fn device_id(&self) -> DeviceId {
        CrosvmDeviceId::Pl061.into()
    }

    fn debug_label(&self) -> String {
        "Pl061".to_owned()
    }

    fn read(&mut self, info: BusAccessInfo, data: &mut [u8]) {
        let val = self.regs.lock().read_reg(info.offset);
        // Registers are at most 32 bits wide; zero-fill any bytes beyond the
        // low 4 so a wider (e.g. 8-byte) guest access neither shifts a u32 out
        // of range (a panic in debug builds) nor returns stale bytes.
        for (i, b) in data.iter_mut().enumerate() {
            *b = if i < 4 { (val >> (i * 8)) as u8 } else { 0 };
        }
    }

    fn write(&mut self, info: BusAccessInfo, data: &[u8]) {
        let mut val = 0u32;
        for (i, b) in data.iter().enumerate().take(4) {
            val |= (*b as u32) << (i * 8);
        }
        self.regs.lock().write_reg(info.offset, val, &self.irq);
    }
}

impl PmResource for Pl061 {
    fn pwrbtn_evt(&mut self) {
        if let Err(e) = self.power_press.signal() {
            error!("pl061: failed to signal power button: {}", e);
        }
    }

    fn slpbtn_evt(&mut self) {
        if let Err(e) = self.sleep_press.signal() {
            error!("pl061: failed to signal sleep button: {}", e);
        }
    }
}

impl Suspendable for Pl061 {
    fn snapshot(&mut self) -> anyhow::Result<AnySnapshot> {
        AnySnapshot::to_any(self.regs.lock().clone())
            .with_context(|| format!("error serializing {}", self.debug_label()))
    }

    fn restore(&mut self, data: AnySnapshot) -> anyhow::Result<()> {
        let deser: Pl061Regs = AnySnapshot::from_any(data)
            .with_context(|| format!("failed to deserialize {}", self.debug_label()))?;
        *self.regs.lock() = deser;
        Ok(())
    }

    fn sleep(&mut self) -> anyhow::Result<()> {
        Ok(())
    }

    fn wake(&mut self) -> anyhow::Result<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bus_addr(offset: u64) -> BusAccessInfo {
        BusAccessInfo {
            address: offset,
            offset,
            id: 0,
        }
    }

    #[test]
    fn primecell_ids_readable() {
        let mut dev = Pl061::new(IrqEdgeEvent::new().unwrap()).unwrap();
        // periphid bytes at 0xfe0/0xfe4/0xfe8/0xfec.
        let mut byte = [0u8];
        dev.read(bus_addr(0xfe0), &mut byte);
        assert_eq!(byte[0], 0x61);
        dev.read(bus_addr(0xff0), &mut byte);
        assert_eq!(byte[0], 0x0d);
    }

    #[test]
    fn power_button_raises_interrupt() {
        let irq = IrqEdgeEvent::new().unwrap();
        let mut dev = Pl061::new(irq.try_clone().unwrap()).unwrap();

        // Configure the power pin as an input with a both-edge interrupt
        // enabled, mirroring what the gpio-keys driver does.
        let mask = 1u8 << GPIO_PIN_POWER_BUTTON;
        dev.write(bus_addr(GPIOIBE), &[mask]); // both edges
        dev.write(bus_addr(GPIOIE), &[mask]); // unmask

        dev.pwrbtn_evt();

        // The worker should drive the pin and assert the interrupt line.
        irq.get_trigger().wait().unwrap();

        // Masked interrupt status should reflect the pending interrupt.
        let mut reg = [0u8];
        dev.read(bus_addr(GPIOMIS), &mut reg);
        assert_eq!(reg[0] & mask, mask);
    }
}
