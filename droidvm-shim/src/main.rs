// Copyright 2026 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! The first thing a pseudo-unprotected VM runs.
//!
//! crosvm starts this VM with only a small lent region -- this shim and the device tree -- and
//! leaves the guest's real memory as a hole: declared to nobody, backed by nothing, until the host
//! SHAREs it as a Gunyah memparcel and the guest accepts it. The host cannot accept on the guest's
//! behalf (the resource manager refuses MEM_ACCEPT_FLAG_MAP_OTHER), so something inside the VM has
//! to do it before any payload runs. That is this.
//!
//! It does four things and then gets out of the way:
//!
//!   1. finds the resource manager's message-queue capabilities in the device tree,
//!   2. reads the handles the host left in the handoff page and accepts each parcel,
//!   3. points `/memory` at the window,
//!   4. jumps to the payload with x0 still holding the device tree.
//!
//! There is no jumping back and nothing patches the payload: it sits at its own address, and this
//! is simply what the hypervisor started instead.

#![no_std]
#![no_main]

use core::arch::global_asm;
use core::panic::PanicInfo;
use core::ptr;

mod rm;

use shim_fdt as fdt;

// One definition, compiled into both sides. See the file for why.
#[path = "../../hypervisor/src/gunyah/shim_abi.rs"]
mod abi;

use abi::*;

global_asm!(include_str!("start.S"));

// Patched by crosvm before the VM starts: everything else in the image is position-independent,
// but the payload and handoff addresses are absolute and only the host knows them.
extern "C" {
    static shim_header: ShimHeader;
}

/// The handoff page, for the panic handler as much as for the main path: a shim that dies with
/// nothing written there is a VM that hangs for no visible reason.
static mut HANDOFF: *mut ShimHandoff = ptr::null_mut();

fn handoff() -> Option<&'static mut ShimHandoff> {
    // SAFETY: set once, before anything can race with it, from an address the host chose and
    // shared with us. Single-threaded from first instruction to last.
    unsafe {
        let p = HANDOFF;
        if p.is_null() {
            None
        } else {
            Some(&mut *p)
        }
    }
}

fn say(text: &str) {
    if let Some(h) = handoff() {
        let n = text.len().min(h.msg.len() - 1);
        h.msg[..n].copy_from_slice(&text.as_bytes()[..n]);
        h.msg[n] = 0;
    }
}

fn die(err: u64, text: &str) -> ! {
    if let Some(h) = handoff() {
        h.error = err;
        say(text);
        h.status = SHIM_STATUS_ERROR;
    }
    hang()
}

fn hang() -> ! {
    loop {
        // SAFETY: wfi is a hint; there is nothing left to do and nobody to wake us.
        unsafe { core::arch::asm!("wfi", options(nomem, nostack)) };
    }
}

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // No formatting: the message would need an allocator and the interesting part is that we
    // panicked at all. A bounds check that fired means the device tree was not what we assumed.
    die(0, "shim panicked (a bounds check fired: the device tree is not what we assumed)")
}

/// Called from the entry code with the device tree pointer the hypervisor put in x0. The return
/// value is the address to jump to.
#[no_mangle]
pub extern "C" fn shim_main(dtb: *mut u8) -> u64 {
    // SAFETY: the header lives in our own image, patched by crosvm before the VM started.
    let hdr = unsafe { ptr::read_volatile(ptr::addr_of!(shim_header)) };
    if hdr.magic != SHIM_HEADER_MAGIC || hdr.version != SHIM_ABI_VERSION {
        // Nothing to complain into: the handoff address is in the header we just failed to trust.
        hang()
    }

    // SAFETY: the address came from the header; the region is shared with us for the VM's life.
    unsafe { HANDOFF = hdr.handoff as *mut ShimHandoff };
    let Some(h) = handoff() else { hang() };
    if h.magic != SHIM_HANDOFF_MAGIC || h.version != SHIM_ABI_VERSION {
        hang()
    }
    h.status = SHIM_STATUS_RUNNING;

    if hdr.payload == 0 {
        die(0, "no payload address in the header")
    }

    if h.nparcels > 0 {
        if h.nparcels as usize > SHIM_MAX_PARCELS {
            die(h.nparcels as u64, "more parcels than the handoff page can hold")
        }
        // `ready` is written last by the host, so everything above it is only meaningful once it
        // is set. Volatile because the host writes it from outside this CPU's view of the world.
        let mut spun = 0u64;
        while unsafe { ptr::read_volatile(ptr::addr_of!(h.ready)) } == 0 {
            spun += 1;
            if spun > 200_000_000 {
                die(0, "the host never finished sharing the window")
            }
        }

        // The device tree, for the two capability ids the accept needs. This is why the tree has
        // to be parsed BEFORE the window exists: there is nowhere else those numbers come from.
        //
        // SAFETY: the hypervisor handed us this pointer, and the header it starts with says how
        // long it is; every access past this point is bounds-checked against that length.
        let blob = unsafe {
            let len = match fdt_total_size(dtb) {
                Some(l) => l,
                None => die(0, "the device tree pointer is not a device tree"),
            };
            core::slice::from_raw_parts_mut(dtb, len)
        };
        let mut tree = match fdt::Fdt::new(blob) {
            Ok(t) => t,
            Err(_) => die(0, "the device tree header did not parse"),
        };
        let (at, len) = match tree.find_prop(
            fdt::Match::Compatible("gunyah-resource-manager"),
            "reg",
        ) {
            Ok(hit) => hit,
            Err(_) => die(0, "no gunyah-resource-manager node in the device tree"),
        };
        if len < 16 {
            die(len as u64, "the resource manager node's reg is too short")
        }
        let reg = tree.prop_bytes(at, 16).unwrap_or(&[]);
        let tx = fdt::be64(&reg[0..8]).unwrap_or(0);
        let rx = fdt::be64(&reg[8..16]).unwrap_or(0);
        let mut rm = rm::Rm::new(tx, rx);

        for i in 0..h.nparcels as usize {
            let p = h.parcel[i];
            match rm.mem_accept(p.handle, p.base, p.size) {
                Ok(()) => {}
                Err(rm::Error::Refused(code)) => {
                    h.error = code as u64;
                    die(code as u64, "the resource manager refused MEM_ACCEPT")
                }
                Err(rm::Error::Timeout) => die(0, "MEM_ACCEPT never got a reply"),
                Err(rm::Error::Send(e)) => die(e, "could not send MEM_ACCEPT to the resource manager"),
            }
        }
        h.status = SHIM_STATUS_ACCEPTED;

        if hdr.flags & SHIM_FLAG_PROBE_EXEC != 0 {
            h.exec_probe = probe_exec(h.parcel[0].base);
        }

        if hdr.flags & SHIM_FLAG_NO_DT_REWRITE == 0 {
            if tree.set_memory_window(h.parcel[0].base, h.parcel[0].size).is_err() {
                die(0, "could not point /memory at the window")
            }
            h.status = SHIM_STATUS_DT_DONE;
        }
    }

    h.status = SHIM_STATUS_JUMPING;
    hdr.payload
}

/// Can the guest execute out of the window?
///
/// The whole design turns on this, and it is the one question no probe inside Linux can answer:
/// arm64 will not produce an executable kernel mapping of a range like this, and a user mapping
/// with PXN cleared is refused by FEAT_PAN3. Here the MMU is off, so there is no stage-1
/// permission to blame -- if this faults, stage 2 refused the fetch, and the VM dies loudly
/// rather than a kernel crashing much later for reasons nobody can trace back.
fn probe_exec(base: u64) -> u64 {
    const MOV_X0_42: u32 = 0xd280_0540;
    const RET: u32 = 0xd65f_03c0;
    // SAFETY: the window was accepted above, so these addresses are real memory the guest owns.
    unsafe {
        let code = base as *mut u32;
        ptr::write_volatile(code, MOV_X0_42);
        ptr::write_volatile(code.add(1), RET);
        core::arch::asm!("dsb sy", "ic iallu", "dsb sy", "isb", options(nostack));
        let f: extern "C" fn() -> u64 = core::mem::transmute(code);
        f()
    }
}

/// The `totalsize` field, read without trusting anything else about the blob yet.
///
/// SAFETY: the caller must pass the pointer the hypervisor put in x0.
unsafe fn fdt_total_size(dtb: *const u8) -> Option<usize> {
    let magic = u32::from_be_bytes([
        ptr::read_volatile(dtb),
        ptr::read_volatile(dtb.add(1)),
        ptr::read_volatile(dtb.add(2)),
        ptr::read_volatile(dtb.add(3)),
    ]);
    if magic != 0xd00d_feed {
        return None;
    }
    let total = u32::from_be_bytes([
        ptr::read_volatile(dtb.add(4)),
        ptr::read_volatile(dtb.add(5)),
        ptr::read_volatile(dtb.add(6)),
        ptr::read_volatile(dtb.add(7)),
    ]) as usize;
    // A tree smaller than its own header, or larger than the slot crosvm reserves for it, is not
    // one we were given.
    if total < 40 || total > 8 * 1024 * 1024 {
        return None;
    }
    Some(total)
}
