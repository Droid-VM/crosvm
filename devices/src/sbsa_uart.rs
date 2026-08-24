// Copyright 2026 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! Emulates an ARM SBSA-compatible UART (a documented subset of the PrimeCell
//! PL011). Unlike the 8250/16550 `Serial` device, this presents a 4 KiB
//! MMIO register bank with 32-bit registers and identifies itself to the guest
//! as `arm,sbsa-uart` (FDT) / `ARMHB000` (ACPI), which is what Windows-on-ARM's
//! inbox `SerPL011.sys` binds to.
//!
//! It is deliberately minimal: no baud generation, no hardware FIFOs, no DMA and
//! no modem-status lines. Bytes move one at a time through the same host
//! input/output plumbing (`SerialInput` / `io::Write`) as [`crate::Serial`], so
//! it works in a protected VM without touching guest memory.

use std::collections::VecDeque;
use std::io;
use std::sync::atomic::AtomicU16;
use std::sync::atomic::Ordering;
use std::sync::mpsc::channel;
use std::sync::mpsc::Receiver;
use std::sync::mpsc::TryRecvError;
use std::sync::Arc;

use anyhow::Context;
use base::error;
use base::warn;
use base::Event;
use base::EventToken;
use base::FileSync;
use base::RawDescriptor;
use base::Result;
use base::WaitContext;
use base::WorkerThread;
use hypervisor::ProtectionType;
use serde::Deserialize;
use serde::Serialize;
use snapshot::AnySnapshot;

use crate::bus::BusAccessInfo;
use crate::pci::CrosvmDeviceId;
use crate::serial_device::SerialInput;
use crate::serial_device::SerialOptions;
use crate::suspendable::DeviceState;
use crate::suspendable::Suspendable;
use crate::sys::serial_device::SerialDevice;
use crate::BusDevice;
use crate::DeviceId;

// Register offsets (byte offset into the 4 KiB MMIO window).
const UARTDR: u64 = 0x000; // Data register
const UARTRSR: u64 = 0x004; // Receive status / error clear
const UARTFR: u64 = 0x018; // Flag register (RO)
const UARTILPR: u64 = 0x020; // IrDA low-power counter
const UARTIBRD: u64 = 0x024; // Integer baud rate
const UARTFBRD: u64 = 0x028; // Fractional baud rate
const UARTLCR_H: u64 = 0x02c; // Line control
const UARTCR: u64 = 0x030; // Control
const UARTIFLS: u64 = 0x034; // Interrupt FIFO level select
const UARTIMSC: u64 = 0x038; // Interrupt mask set/clear
const UARTRIS: u64 = 0x03c; // Raw interrupt status (RO)
const UARTMIS: u64 = 0x040; // Masked interrupt status (RO)
const UARTICR: u64 = 0x044; // Interrupt clear (WO)
const UARTDMACR: u64 = 0x048; // DMA control
const UARTPERIPHID0: u64 = 0xfe0; // PeripheralID/PrimeCellID window (0xFE0..=0xFFC)

// Flag register (UARTFR) bits.
const FR_RXFE: u16 = 1 << 4; // Receive FIFO empty
const FR_TXFF: u16 = 1 << 5; // Transmit FIFO full (never, here)
const FR_TXFE: u16 = 1 << 7; // Transmit FIFO empty (always, here)

// Interrupt bits, shared by RIS / MIS / IMSC / ICR.
const INT_RX: u16 = 1 << 4; // RXI
const INT_TX: u16 = 1 << 5; // TXI

// Standard PrimeCell identification bytes (PL011). SBSA drivers ignore these,
// but returning them is harmless and lets a full-PL011 probe succeed too.
const PL011_ID: [u8; 8] = [0x11, 0x10, 0x14, 0x00, 0x0d, 0xf0, 0x05, 0xb1];

// PL011 reset value of UARTCR: TXE (bit 8) and RXE (bit 9) set, UARTEN clear.
const DEFAULT_CR: u16 = 0x0300;

/// ARM SBSA / PL011-subset UART.
pub struct SbsaUart {
    // Interrupt mask (UARTIMSC). Shared with the input worker so it can decide
    // whether an arriving RX byte should raise the guest interrupt.
    imsc: Arc<AtomicU16>,
    // Latched raw interrupt status (UARTRIS). Only the TX bit is latched here;
    // the RX bit is derived from `in_buffer` so it clears as the guest drains.
    ris: u16,
    // Whether we currently hold the (edge) IRQ line asserted. We emulate a level
    // line over the edge irqfd: signal only on the 0->1 transition of the masked
    // status and clear when it falls, so a busy TX driver (write-then-mask) can't
    // make us emit spurious edges after it masks TXIM ("irq N: nobody cared").
    irq_asserted: bool,

    // Line/control registers. Cosmetic for a virtual UART, but stored so reads
    // return what was written and snapshots round-trip.
    ibrd: u16,
    fbrd: u16,
    lcr_h: u16,
    cr: u16,
    ifls: u16,
    dmacr: u16,

    interrupt_evt: Event,

    // Host input/output, identical plumbing to `Serial`.
    in_buffer: VecDeque<u8>,
    in_channel: Option<Receiver<u8>>,
    input: Option<Box<dyn SerialInput>>,
    out: Option<Box<dyn io::Write + Send>>,

    device_state: DeviceState,
    worker: Option<WorkerThread<Box<dyn SerialInput>>>,
}

impl SerialDevice for SbsaUart {
    fn new(
        _protection_type: ProtectionType,
        interrupt_evt: Event,
        input: Option<Box<dyn SerialInput>>,
        out: Option<Box<dyn io::Write + Send>>,
        _sync: Option<Box<dyn FileSync + Send>>,
        _options: SerialOptions,
        _keep_rds: Vec<RawDescriptor>,
    ) -> SbsaUart {
        SbsaUart::new_common(interrupt_evt, input, out)
    }
}

impl SbsaUart {
    pub(crate) fn new_common(
        interrupt_evt: Event,
        input: Option<Box<dyn SerialInput>>,
        out: Option<Box<dyn io::Write + Send>>,
    ) -> SbsaUart {
        SbsaUart {
            imsc: Arc::new(AtomicU16::new(0)),
            ris: 0,
            irq_asserted: false,
            ibrd: 0,
            fbrd: 0,
            lcr_h: 0,
            cr: DEFAULT_CR,
            ifls: 0,
            dmacr: 0,
            interrupt_evt,
            in_buffer: VecDeque::new(),
            in_channel: None,
            input,
            out,
            device_state: DeviceState::Awake,
            worker: None,
        }
    }

    /// Returns a unique ID for the device.
    pub fn device_id() -> DeviceId {
        CrosvmDeviceId::SbsaUart.into()
    }

    /// Returns a debug label. Used when setting up `IrqEventSource`.
    pub fn debug_label() -> String {
        "sbsa-uart".to_owned()
    }

    fn flag_register(&self) -> u16 {
        let mut fr = FR_TXFE; // TX drains instantly.
        if self.in_buffer.is_empty() {
            fr |= FR_RXFE;
        }
        // FR_TXFF and BUSY are never asserted.
        let _ = FR_TXFF;
        fr
    }

    fn raw_int_status(&self) -> u16 {
        let mut r = self.ris;
        if !self.in_buffer.is_empty() {
            r |= INT_RX;
        }
        r
    }

    fn masked_int_status(&self) -> u16 {
        self.raw_int_status() & self.imsc.load(Ordering::SeqCst)
    }

    /// Re-evaluate the IRQ line. Emulates a level line over the edge irqfd: assert
    /// (signal once) on the 0->1 transition of the masked interrupt status, and clear
    /// the latch when it falls, so we never emit stray edges while already asserted.
    fn update_interrupt(&mut self) {
        if self.masked_int_status() != 0 {
            if !self.irq_asserted {
                self.irq_asserted = true;
                if let Err(e) = self.interrupt_evt.signal() {
                    error!("sbsa-uart failed to signal interrupt: {}", e);
                }
            }
        } else {
            self.irq_asserted = false;
        }
    }

    // Write a single byte of data to `self.out`.
    fn transmit(&mut self, v: u8) {
        if let Some(out) = self.out.as_mut() {
            if let Err(e) = out.write_all(&[v]).and_then(|_| out.flush()) {
                error!("sbsa-uart failed write: {}", e);
            }
        }
    }

    fn read_reg(&mut self, offset: u64) -> u32 {
        match offset {
            UARTDR => self.in_buffer.pop_front().unwrap_or(0) as u32,
            UARTRSR => 0,
            UARTFR => self.flag_register() as u32,
            UARTILPR => 0,
            UARTIBRD => self.ibrd as u32,
            UARTFBRD => self.fbrd as u32,
            UARTLCR_H => self.lcr_h as u32,
            UARTCR => self.cr as u32,
            UARTIFLS => self.ifls as u32,
            UARTIMSC => self.imsc.load(Ordering::SeqCst) as u32,
            UARTRIS => self.raw_int_status() as u32,
            UARTMIS => self.masked_int_status() as u32,
            UARTDMACR => self.dmacr as u32,
            o if (UARTPERIPHID0..=0xffc).contains(&o) => {
                let idx = ((o - UARTPERIPHID0) / 4) as usize;
                *PL011_ID.get(idx).unwrap_or(&0) as u32
            }
            _ => 0,
        }
    }

    fn write_reg(&mut self, offset: u64, v: u32) {
        match offset {
            UARTDR => {
                self.transmit((v & 0xff) as u8);
                self.ris |= INT_TX;
            }
            UARTRSR => {} // Writing clears error flags; we model none.
            UARTILPR => {}
            UARTIBRD => self.ibrd = v as u16,
            UARTFBRD => self.fbrd = v as u16,
            UARTLCR_H => self.lcr_h = v as u16,
            UARTCR => self.cr = v as u16,
            UARTIFLS => self.ifls = v as u16,
            UARTIMSC => self.imsc.store(v as u16, Ordering::SeqCst),
            UARTICR => self.ris &= !(v as u16),
            UARTDMACR => self.dmacr = v as u16,
            _ => {}
        }
    }

    fn spawn_input_thread(&mut self) {
        let mut rx = match self.input.take() {
            Some(input) => input,
            None => return,
        };

        let (send_channel, recv_channel) = channel();
        let imsc = self.imsc.clone();
        let interrupt_evt = match self.interrupt_evt.try_clone() {
            Ok(e) => e,
            Err(e) => {
                error!("failed to clone interrupt event: {}", e);
                return;
            }
        };

        self.worker = Some(WorkerThread::start(
            format!("{} input thread", self.debug_label()),
            move |kill_evt| {
                let mut rx_buf = [0u8; 1];

                #[derive(EventToken)]
                enum Token {
                    Kill,
                    SerialEvent,
                }

                let wait_ctx_res: Result<WaitContext<Token>> = WaitContext::build_with(&[
                    (&kill_evt, Token::Kill),
                    (rx.get_read_notifier(), Token::SerialEvent),
                ]);
                let wait_ctx = match wait_ctx_res {
                    Ok(ctx) => ctx,
                    Err(e) => {
                        error!("Failed to create wait context. {}", e);
                        return rx;
                    }
                };
                loop {
                    let events = match wait_ctx.wait() {
                        Ok(events) => events,
                        Err(e) => {
                            error!("Failed to wait for events. {}", e);
                            return rx;
                        }
                    };
                    for event in events.iter() {
                        match event.token {
                            Token::Kill => return rx,
                            Token::SerialEvent => match rx.read(&mut rx_buf) {
                                Ok(0) => return rx,
                                Ok(_n) => {
                                    if send_channel.send(rx_buf[0]).is_err() {
                                        return rx;
                                    }
                                    if (imsc.load(Ordering::SeqCst) & INT_RX) != 0 {
                                        interrupt_evt.signal().unwrap();
                                    }
                                }
                                Err(e) => {
                                    if e.kind() != io::ErrorKind::Interrupted {
                                        error!("failed to read serial input: {}", e);
                                        return rx;
                                    }
                                }
                            },
                        }
                    }
                }
            },
        ));
        self.in_channel = Some(recv_channel);
    }

    fn drain_in_channel(&mut self) {
        loop {
            let in_channel = match self.in_channel.as_ref() {
                Some(v) => v,
                None => return,
            };
            match in_channel.try_recv() {
                Ok(byte) => self.in_buffer.push_back(byte),
                Err(TryRecvError::Empty) => break,
                Err(TryRecvError::Disconnected) => {
                    self.in_channel = None;
                    return;
                }
            }
        }
    }
}

fn u32_from_le(data: &[u8]) -> u32 {
    let mut buf = [0u8; 4];
    let n = data.len().min(4);
    buf[..n].copy_from_slice(&data[..n]);
    u32::from_le_bytes(buf)
}

impl BusDevice for SbsaUart {
    fn device_id(&self) -> DeviceId {
        CrosvmDeviceId::SbsaUart.into()
    }

    fn debug_label(&self) -> String {
        "sbsa-uart".to_owned()
    }

    fn write(&mut self, info: BusAccessInfo, data: &[u8]) {
        if matches!(self.device_state, DeviceState::Sleep) {
            panic!("Unexpected action: Attempt to write to sbsa-uart while asleep");
        }
        self.write_reg(info.offset, u32_from_le(data));
        self.update_interrupt();
    }

    fn read(&mut self, info: BusAccessInfo, data: &mut [u8]) {
        if matches!(self.device_state, DeviceState::Sleep) {
            panic!("Unexpected action: Attempt to read from sbsa-uart while asleep");
        }
        if self.input.is_some() {
            self.spawn_input_thread();
        }
        self.drain_in_channel();

        let v = self.read_reg(info.offset).to_le_bytes();
        // Re-evaluate the latched line: a read may have drained RX and lowered it,
        // clearing the latch so the next genuine interrupt can be delivered.
        self.update_interrupt();
        for (i, b) in data.iter_mut().enumerate() {
            *b = *v.get(i).unwrap_or(&0);
        }
    }
}

#[derive(Serialize, Deserialize)]
struct SbsaUartSnapshot {
    imsc: u16,
    ris: u16,
    ibrd: u16,
    fbrd: u16,
    lcr_h: u16,
    cr: u16,
    ifls: u16,
    dmacr: u16,
    in_buffer: VecDeque<u8>,
    has_input: bool,
    has_output: bool,
}

impl Suspendable for SbsaUart {
    fn snapshot(&mut self) -> anyhow::Result<AnySnapshot> {
        self.spawn_input_thread();
        if let Some(worker) = self.worker.take() {
            self.input = Some(worker.stop());
        }
        self.drain_in_channel();
        let snap = SbsaUartSnapshot {
            imsc: self.imsc.load(Ordering::SeqCst),
            ris: self.ris,
            ibrd: self.ibrd,
            fbrd: self.fbrd,
            lcr_h: self.lcr_h,
            cr: self.cr,
            ifls: self.ifls,
            dmacr: self.dmacr,
            in_buffer: self.in_buffer.clone(),
            has_input: self.input.is_some(),
            has_output: self.out.is_some(),
        };
        AnySnapshot::to_any(snap).context("error serializing")
    }

    fn restore(&mut self, data: AnySnapshot) -> anyhow::Result<()> {
        let snap: SbsaUartSnapshot = AnySnapshot::from_any(data).context("error deserializing")?;
        self.imsc = Arc::new(AtomicU16::new(snap.imsc));
        self.ris = snap.ris;
        self.ibrd = snap.ibrd;
        self.fbrd = snap.fbrd;
        self.lcr_h = snap.lcr_h;
        self.cr = snap.cr;
        self.ifls = snap.ifls;
        self.dmacr = snap.dmacr;
        self.in_buffer = snap.in_buffer;
        if snap.has_input && self.input.is_none() {
            warn!("Restore sbsa-uart input missing when restore expected an input");
        }
        if snap.has_output && self.out.is_none() {
            warn!("Restore sbsa-uart out missing when restore expected an out");
        }
        Ok(())
    }

    fn sleep(&mut self) -> anyhow::Result<()> {
        if !matches!(self.device_state, DeviceState::Sleep) {
            self.device_state = DeviceState::Sleep;
            if let Some(worker) = self.worker.take() {
                self.input = Some(worker.stop());
            }
            self.drain_in_channel();
            self.in_channel = None;
        }
        Ok(())
    }

    fn wake(&mut self) -> anyhow::Result<()> {
        if !matches!(self.device_state, DeviceState::Awake) {
            self.device_state = DeviceState::Awake;
            if self.input.is_some() {
                self.spawn_input_thread();
            }
        }
        Ok(())
    }
}
