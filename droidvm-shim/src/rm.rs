// Copyright 2026 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! The one resource-manager call the shim makes: accept a memparcel the host shared.
//!
//! The wire format is the same one `gunyah_guest.c` uses in the guest kernel, and the same one
//! EDK2's GunyahPreloadDxe uses -- three implementations of a protocol nobody publishes, kept in
//! step by saying so here.

use core::arch::asm;

use crate::cache_flush;

const GH_HCALL_MSGQ_SEND: u64 = 0xC600_801B;
const GH_HCALL_MSGQ_RECV: u64 = 0xC600_801C;
const GH_MSGQ_TX_PUSH: u64 = 1 << 0;
const GH_ERROR_OK: u64 = 0;

const RPC_API: u8 = 0x21;
const RPC_TYPE_REQUEST: u8 = 0x01;
const RPC_TYPE_REPLY: u8 = 0x02;
const RPC_TYPE_MASK: u8 = 0x03;
const RPC_MEM_ACCEPT: u32 = 0x5100_0011;

const MEM_TYPE_NORMAL: u8 = 0;
const TRANS_TYPE_SHARE: u8 = 2;
const ACCEPT_MAP_CONTIGUOUS: u8 = 1 << 4;
const ACCEPT_DONE: u8 = 1 << 7;

const MSGQ_MSG_SIZE: usize = 240;

/// How long to wait for a reply. There is no timer here and nothing else to do, so this is a spin
/// count rather than a duration: the resource manager answers in microseconds when it answers,
/// and a shim that waits forever is a VM that hangs with no explanation.
const REPLY_SPINS: u32 = 200_000_000;

#[derive(Debug, Clone, Copy)]
pub enum Error {
    /// The message queue itself refused the send; the payload is the hypercall's error.
    Send(u64),
    /// The resource manager answered, and said no. The payload is its error code.
    Refused(u32),
    /// No reply within [`REPLY_SPINS`].
    Timeout,
}

/// What a wait actually saw, for when it saw nothing useful.
#[derive(Debug, Clone, Copy, Default)]
pub struct WaitTrace {
    /// Messages the queue handed us while we waited.
    pub received: u32,
    /// The last error the receive hypercall returned, and the size it reported.
    pub last_err: u64,
    pub last_len: u64,
    /// The header of the last message we did get: api, type, seq, msg_id.
    pub last_hdr: [u8; 8],
    /// The sequence the answer came back with, and how often it was not the one we sent.
    pub last_seq: u16,
    pub wrong_seq: u32,
}

pub struct Rm {
    tx: u64,
    rx: u64,
    seq: u16,
    txbuf: [u8; 64],
    rxbuf: [u8; MSGQ_MSG_SIZE],
    /// What the last wait saw. Read after a timeout, which is the only time it means anything.
    pub trace: WaitTrace,
}

/// SAFETY: `hvc` is only ever used for the two message-queue calls above, which take scalars and
/// a buffer the caller owns, and clobber nothing the compiler is holding.
unsafe fn hvc(f: u64, a1: u64, a2: u64, a3: u64, a4: u64) -> (u64, u64) {
    let mut x0 = f;
    let mut x1 = a1;
    let x2 = a2;
    let x3 = a3;
    let x4 = a4;
    asm!(
        "hvc #0",
        inout("x0") x0,
        inout("x1") x1,
        in("x2") x2,
        in("x3") x3,
        in("x4") x4,
        lateout("x5") _, lateout("x6") _, lateout("x7") _,
        lateout("x8") _, lateout("x9") _, lateout("x10") _, lateout("x11") _,
        lateout("x12") _, lateout("x13") _, lateout("x14") _, lateout("x15") _,
        lateout("x16") _, lateout("x17") _,
        options(nostack),
    );
    (x0, x1)
}

impl Rm {
    pub fn new(tx: u64, rx: u64) -> Self {
        Rm {
            tx,
            rx,
            seq: 0,
            txbuf: [0; 64],
            rxbuf: [0; MSGQ_MSG_SIZE],
            trace: WaitTrace::default(),
        }
    }

    /// Throw away everything waiting in the receive queue.
    ///
    /// Every message here is a notification: the shim sends one request at a time and waits for
    /// its reply before sending another, so nothing else it cares about can be in flight. Returns
    /// how many it dropped, which is worth printing while bringing a device up.
    pub fn drain(&mut self) -> u32 {
        let mut n = 0;
        loop {
            // SAFETY: the queue is the one the device tree named, and the buffer is ours.
            let (err, got) = unsafe {
                hvc(
                    GH_HCALL_MSGQ_RECV,
                    self.rx,
                    self.rxbuf.as_mut_ptr() as u64,
                    self.rxbuf.len() as u64,
                    0,
                )
            };
            cache_flush(self.rxbuf.as_ptr() as u64, self.rxbuf.len());
            if err != GH_ERROR_OK || got == 0 {
                return n;
            }
            n += 1;
        }
    }

    /// Take a shared memparcel at a fixed address.
    ///
    /// MAP_CONTIGUOUS with a single sgl entry covering the whole thing is the one shape that works
    /// for a parcel whose pages are scattered: with the flag off the resource manager expects one
    /// sgl entry per physically contiguous run, which is a layout the guest has no way to know.
    /// The ACL descriptor is empty on purpose -- the rights are the ones the host asked for when
    /// it shared, and validating them here would only be able to disagree, not to change them.
    pub fn mem_accept(&mut self, handle: u32, gpa: u64, size: u64) -> Result<(), Error> {
        // Empty the queue before asking anything, because what is already in it is not ours and
        // the space it takes is.
        //
        // The resource manager notifies the guest once per parcel the host shares, and the queue
        // the device tree gives us is eight messages deep. Hand a VM its memory in eight parcels
        // and the queue is full before the shim has run an instruction; the reply to the third
        // accept then has nowhere to land and is simply lost, which arrives here as a request
        // that never gets an answer. Draining first also costs nothing when there is one parcel.
        self.drain();
        self.seq = self.seq.wrapping_add(1);
        let seq = self.seq;
        let mut n = 0usize;
        let mut put = |bytes: &[u8], at: &mut usize| {
            self.txbuf[*at..*at + bytes.len()].copy_from_slice(bytes);
            *at += bytes.len();
        };
        put(&[RPC_API, RPC_TYPE_REQUEST], &mut n);
        put(&seq.to_le_bytes(), &mut n);
        put(&RPC_MEM_ACCEPT.to_le_bytes(), &mut n);
        put(&handle.to_le_bytes(), &mut n);
        put(
            &[
                MEM_TYPE_NORMAL,
                TRANS_TYPE_SHARE,
                ACCEPT_MAP_CONTIGUOUS | ACCEPT_DONE,
                0,
            ],
            &mut n,
        );
        put(&0u32.to_le_bytes(), &mut n); // validate_label
        put(&0u32.to_le_bytes(), &mut n); // acl_desc: no entries
        put(&1u16.to_le_bytes(), &mut n); // sgl_desc: one entry
        put(&0u16.to_le_bytes(), &mut n); // map_vmid 0 = to self
        put(&gpa.to_le_bytes(), &mut n);
        put(&size.to_le_bytes(), &mut n);
        put(&0u16.to_le_bytes(), &mut n); // mem_attr_desc: none
        put(&0u16.to_le_bytes(), &mut n);

        // The whole reason the second request of a VM used to fail.
        //
        // The shim runs with the MMU off, so this write went to memory and not into a cache line.
        // Whoever reads the buffer on the other side of the hypercall reads it cached, and it has
        // read this same address before -- so without pushing the line out, the second request is
        // served from the bytes of the first: the resource manager accepts a parcel it has already
        // accepted, answers ARGUMENT_INVALID, and stamps the reply with the *first* request's
        // sequence number, which is exactly what was observed.
        cache_flush(self.txbuf.as_ptr() as u64, n);
        // SAFETY: the queue is the one the device tree named, and the buffer is ours.
        let (err, _) = unsafe {
            hvc(
                GH_HCALL_MSGQ_SEND,
                self.tx,
                n as u64,
                self.txbuf.as_ptr() as u64,
                GH_MSGQ_TX_PUSH,
            )
        };
        if err != GH_ERROR_OK {
            return Err(Error::Send(err));
        }

        self.trace = WaitTrace::default();
        for _ in 0..REPLY_SPINS {
            // SAFETY: as above; the buffer is ours and its length is passed honestly.
            let (err, got) = unsafe {
                hvc(
                    GH_HCALL_MSGQ_RECV,
                    self.rx,
                    self.rxbuf.as_mut_ptr() as u64,
                    self.rxbuf.len() as u64,
                    0,
                )
            };
            // Same hazard mirrored: the reply was written through a cacheable mapping and this
            // read is not one, so clean the line to memory before believing any of it.
            cache_flush(self.rxbuf.as_ptr() as u64, self.rxbuf.len());
            self.trace.last_err = err;
            self.trace.last_len = got;
            if err != GH_ERROR_OK || (got as usize) < 12 {
                continue;
            }
            self.trace.received += 1;
            self.trace.last_hdr.copy_from_slice(&self.rxbuf[..8]);
            let r = &self.rxbuf;
            if r[1] & RPC_TYPE_MASK != RPC_TYPE_REPLY {
                continue; // a notification, not our answer
            }
            if u32::from_le_bytes([r[4], r[5], r[6], r[7]]) != RPC_MEM_ACCEPT {
                continue;
            }
            // Strictly the sequence we sent, the same way the guest kernel's own client does it.
            // A reply carrying someone else's number was worth counting while this was being
            // brought up -- it turned out to mean the resource manager had been handed a stale
            // copy of the previous request -- so the count stays, and a timeout prints it.
            self.trace.last_seq = u16::from_le_bytes([r[2], r[3]]);
            if self.trace.last_seq != seq {
                self.trace.wrong_seq += 1;
                continue;
            }
            let code = u32::from_le_bytes([r[8], r[9], r[10], r[11]]);
            return if code == 0 {
                Ok(())
            } else {
                Err(Error::Refused(code))
            };
        }
        Err(Error::Timeout)
    }
}
