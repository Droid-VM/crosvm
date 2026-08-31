// Copyright 2026 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! The contract between crosvm and the boot shim.
//!
//! This file is compiled into BOTH sides: crosvm builds it as part of the hypervisor crate, and
//! `droidvm-shim` includes it by path. There is deliberately no second copy, because the two
//! things it describes are written by one side and read by the other with no way to check: a
//! mismatched field offset is a VM that starts, hangs, and says nothing at all.
//!
//! Two structures, because the information arrives at two different times.
//!
//! The header is patched into the shim image before the VM starts -- the shim needs to know where
//! its payload and its handoff page are, and that is all static. After GH_VM_START the boot
//! region is lent, so the host cannot write to it any more, which is why anything discovered
//! later has to arrive somewhere else.
//!
//! The handoff page is that somewhere else: a small region shared rather than lent, so the host
//! keeps write access to it for the VM's whole life. It carries what only exists after the VM has
//! started -- the memparcel handles -- and it carries the shim's answer back, which is the
//! difference between a boot failure that explains itself and one that looks like a hang.

#![allow(dead_code)]

/// "DVMUVHSM", little-endian.
pub const SHIM_HEADER_MAGIC: u64 = 0x4d53_4856_554d_5644;
/// "MVSHANDO", little-endian.
pub const SHIM_HANDOFF_MAGIC: u64 = 0x4f44_4e41_4853_564d;
pub const SHIM_ABI_VERSION: u32 = 1;

/// The most parcels one window may be shared as. A window is one parcel unless something forces
/// it to be split, so this is headroom rather than a target: every parcel costs one of the 1024
/// the resource manager allows per VM, shared with Android's own and not returned until reboot.
pub const SHIM_MAX_PARCELS: usize = 32;

/// Offset of the header within the shim image. Offset 0 is the entry branch.
pub const SHIM_HEADER_OFFSET: usize = 8;

/// Patched into the shim image before the VM starts.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ShimHeader {
    pub magic: u64,
    pub version: u32,
    pub flags: u32,
    /// Where to jump once the window is accepted: the kernel or the firmware, in the window.
    pub payload: u64,
    /// Guest-physical address of the [`ShimHandoff`], in the shared handoff region.
    pub handoff: u64,
    /// How far the device tree may grow while being rewritten. Nothing uses it yet -- the rewrite
    /// keeps every property's length -- but a shim that ever needs to add a node will, and the
    /// field costs nothing now and an ABI bump later.
    pub dtb_max_size: u64,
    pub reserved: [u64; 3],
}

/// Accept the window, then leave `/memory` alone. For bring-up, where the payload is expected to
/// boot on the region it was given rather than on the window.
pub const SHIM_FLAG_NO_DT_REWRITE: u32 = 1 << 0;
/// Write two instructions into the window and call them, reporting what came back in
/// [`ShimHandoff::exec_probe`]. This is the one question no probe inside Linux can answer: with
/// the MMU off there is no stage-1 permission to blame, so a fault here is stage 2 refusing the
/// fetch.
pub const SHIM_FLAG_PROBE_EXEC: u32 = 1 << 1;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct ShimParcel {
    pub handle: u32,
    pub reserved: u32,
    pub base: u64,
    pub size: u64,
}

/// Host to shim, then shim to host, in memory the host shares rather than lends.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct ShimHandoff {
    pub magic: u64,
    pub version: u32,
    pub nparcels: u32,
    /// Written last by the host and read first by the shim. Everything above is only meaningful
    /// once it is non-zero; a shim that outran the host's sharing loop would otherwise accept a
    /// handle that is still zero.
    pub ready: u64,
    pub parcel: [ShimParcel; SHIM_MAX_PARCELS],
    // ---- the shim's answer ----
    pub status: u64,
    /// The resource manager's error code where there is one, so a refusal can be looked up rather
    /// than guessed at.
    pub error: u64,
    pub exec_probe: u64,
    /// One line of plain text for the host to log. A boot that fails here has no console of its
    /// own, so this is the only thing that can say why.
    pub msg: [u8; 256],
}

pub const SHIM_STATUS_RUNNING: u64 = 1;
pub const SHIM_STATUS_ACCEPTED: u64 = 2;
pub const SHIM_STATUS_DT_DONE: u64 = 3;
pub const SHIM_STATUS_JUMPING: u64 = 4;
pub const SHIM_STATUS_ERROR: u64 = 0xe000;

impl Default for ShimHandoff {
    fn default() -> Self {
        Self {
            magic: SHIM_HANDOFF_MAGIC,
            version: SHIM_ABI_VERSION,
            nparcels: 0,
            ready: 0,
            parcel: [ShimParcel::default(); SHIM_MAX_PARCELS],
            status: 0,
            error: 0,
            exec_probe: 0,
            msg: [0u8; 256],
        }
    }
}
