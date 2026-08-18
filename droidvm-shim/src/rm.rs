// Copyright 2026 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! The one resource-manager call the shim makes: accept a memparcel the host shared.
//!
//! The wire format is the same one `gunyah_guest.c` uses in the guest kernel, and the same one
//! EDK2's GunyahPreloadDxe uses -- three implementations of a protocol nobody publishes, kept in
//! step by saying so here.

use core::arch::asm;

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

pub struct Rm {
    tx: u64,
    rx: u64,
    seq: u16,
    txbuf: [u8; 64],
    rxbuf: [u8; MSGQ_MSG_SIZE],
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
            if err != GH_ERROR_OK || (got as usize) < 12 {
                continue;
            }
            let r = &self.rxbuf;
            if r[1] & RPC_TYPE_MASK != RPC_TYPE_REPLY {
                continue; // a notification, not our answer
            }
            if u16::from_le_bytes([r[2], r[3]]) != seq {
                continue;
            }
            if u32::from_le_bytes([r[4], r[5], r[6], r[7]]) != RPC_MEM_ACCEPT {
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
