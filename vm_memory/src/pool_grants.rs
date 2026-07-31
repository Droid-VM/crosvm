// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

//! Which parts of a growable pool are currently backed.
//!
//! A growable pool is declared to the guest at its full size but SHARE'd only in part at boot; the
//! guest asks for the rest at runtime. Somebody has to remember which step-sized slots have been
//! granted, and it has to be the host, for three separate reasons:
//!
//!   * **Label collisions.** A runtime SHARE is reclaimed by a label derived from the address
//!     (`gpa >> 12`), not by a handle looked up in a table -- see `runtime_unshare` in
//!     hypervisor/src/gunyah/mod.rs. Two live SHAREs at one address therefore collapse onto one
//!     label and the wrong one gets reclaimed. Nothing in crosvm could previously answer "is this
//!     address already shared", because no such table existed.
//!
//!   * **Request validation.** Grow and shrink requests arrive from the guest, so the range has to
//!     be checked against what the pool actually owns before it reaches the hypervisor.
//!
//!   * **Host access.** Reading an ungranted address in a declared window does NOT fault. Measured
//!     on device: a read returns zeros with no error, no log and a surviving VM, while a write
//!     kills the vcpu. The silent-zero direction is the dangerous one -- wrong data with nothing
//!     to show for it -- so the host's own accesses have to be gated on this table too, and gated
//!     for reads and not just writes.
//!
//! Slots are numbered from the start of the growable part, i.e. slot `i` is
//! `[base + pre_alloc + i*step, +step)`. The pre-allocated floor is not represented: it is shared
//! before boot under a different label space (the shm vdevice node's region index rather than
//! `gpa >> 12`) and consequently can never be reclaimed at runtime, which is exactly what a floor
//! should be.

use std::collections::BTreeMap;

use crate::GuestAddress;

/// Why a grow or shrink request was refused. Carried back to the guest as a plain errno, but kept
/// as a distinct value here so tests can assert the specific reason rather than "some error".
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GrantError {
    /// The pool does not grow (`step_size == 0`); it is entirely pre-shared.
    NotGrowable,
    /// Zero-length request.
    EmptyRange,
    /// Below the pre-allocated floor, which is not the runtime allocator's to hand out.
    BelowFloor,
    /// Past the end of the declared window.
    PastWindow,
    /// Offset or length is not a multiple of the pool's step.
    Misaligned,
    /// Some slot in the range is already granted (grow) -- granting again would collide on the
    /// `gpa >> 12` reclaim label.
    AlreadyGranted,
    /// Some slot in the range is not granted (shrink).
    NotGranted,
    /// Would exceed the pool's own cap on live grants. Each grant is an RM memparcel, and
    /// MAX_MEMPARCEL_PER_VM is 1024 for the whole VM -- shared with Android's own parcels, and
    /// not released by anything short of a reboot for whatever a killed VMM left behind.
    QuotaExceeded,
}

impl GrantError {
    pub fn as_errno(self) -> i32 {
        match self {
            GrantError::NotGrowable => libc::EOPNOTSUPP,
            GrantError::QuotaExceeded => libc::EDQUOT,
            GrantError::AlreadyGranted => libc::EEXIST,
            GrantError::NotGranted => libc::ENOENT,
            _ => libc::EINVAL,
        }
    }
}

/// The backed/unbacked map of one growable pool.
#[derive(Debug)]
pub struct PoolGrants {
    base: u64,
    size: u64,
    pre_alloc: u64,
    step: u64,
    max_grants: u32,
    /// Slot index -> RM memparcel handle. Absent means ungranted.
    granted: BTreeMap<u32, u32>,
}

impl PoolGrants {
    pub fn new(base: GuestAddress, size: u64, pre_alloc: u64, step: u64, max_grants: u32) -> Self {
        PoolGrants {
            base: base.offset(),
            size,
            pre_alloc,
            step,
            max_grants,
            granted: BTreeMap::new(),
        }
    }

    pub fn step(&self) -> u64 {
        self.step
    }

    pub fn live_grants(&self) -> usize {
        self.granted.len()
    }

    /// Slot indices covered by `[offset, offset+len)`, after the range checks that do not depend
    /// on what is currently granted.
    fn slots(&self, offset: u64, len: u64) -> Result<std::ops::Range<u32>, GrantError> {
        if self.step == 0 {
            return Err(GrantError::NotGrowable);
        }
        if len == 0 {
            return Err(GrantError::EmptyRange);
        }
        if offset < self.pre_alloc {
            return Err(GrantError::BelowFloor);
        }
        // Checked arithmetic throughout: offset and len arrive from the guest.
        let end = offset.checked_add(len).ok_or(GrantError::PastWindow)?;
        if end > self.size {
            return Err(GrantError::PastWindow);
        }
        if offset % self.step != 0 || len % self.step != 0 {
            return Err(GrantError::Misaligned);
        }
        let first = ((offset - self.pre_alloc) / self.step) as u32;
        let count = (len / self.step) as u32;
        Ok(first..first + count)
    }

    /// Check a grow request. Does not modify anything: a refused request must leave no trace, so
    /// the guest can retry a different range without having to reconcile.
    pub fn validate_share(&self, offset: u64, len: u64) -> Result<(), GrantError> {
        let slots = self.slots(offset, len)?;
        if slots.clone().any(|s| self.granted.contains_key(&s)) {
            return Err(GrantError::AlreadyGranted);
        }
        if self.granted.len() + slots.len() > self.max_grants as usize {
            return Err(GrantError::QuotaExceeded);
        }
        Ok(())
    }

    /// Check a shrink request.
    pub fn validate_unshare(&self, offset: u64, len: u64) -> Result<(), GrantError> {
        let slots = self.slots(offset, len)?;
        if slots.clone().any(|s| !self.granted.contains_key(&s)) {
            return Err(GrantError::NotGranted);
        }
        Ok(())
    }

    /// Record a completed grant. `handles` is one RM memparcel handle per step in the range.
    pub fn mark_granted(&mut self, offset: u64, len: u64, handles: &[u32]) -> Result<(), GrantError> {
        let slots = self.slots(offset, len)?;
        debug_assert_eq!(slots.len(), handles.len());
        for (s, h) in slots.zip(handles.iter()) {
            self.granted.insert(s, *h);
        }
        Ok(())
    }

    /// Forget a released range, returning the handles that were recorded for it.
    pub fn take_granted(&mut self, offset: u64, len: u64) -> Result<Vec<u32>, GrantError> {
        let slots = self.slots(offset, len)?;
        let mut out = Vec::with_capacity(slots.len());
        for s in slots {
            out.push(self.granted.remove(&s).ok_or(GrantError::NotGranted)?);
        }
        Ok(out)
    }

    /// Every live grant, for teardown and for answering the guest's reconciliation query.
    pub fn drain_all(&mut self) -> Vec<(u64, u32)> {
        std::mem::take(&mut self.granted)
            .into_iter()
            .map(|(slot, handle)| (self.base + self.pre_alloc + slot as u64 * self.step, handle))
            .collect()
    }

    /// Is this guest physical address backed right now? The pre-allocated floor always is; above
    /// it, only granted slots are. Addresses outside the pool are not this pool's business and
    /// answer `false`.
    pub fn is_backed(&self, gpa: GuestAddress) -> bool {
        let a = gpa.offset();
        if a < self.base || a >= self.base + self.size {
            return false;
        }
        let off = a - self.base;
        if off < self.pre_alloc {
            return true;
        }
        if self.step == 0 {
            return false;
        }
        let slot = ((off - self.pre_alloc) / self.step) as u32;
        self.granted.contains_key(&slot)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MB: u64 = 1 << 20;

    /// 256 MiB window, 64 MiB floor, 32 MiB step -> 6 growable slots.
    fn pool() -> PoolGrants {
        PoolGrants::new(GuestAddress(0x1_9000_0000), 256 * MB, 64 * MB, 32 * MB, 64)
    }

    #[test]
    fn accepts_an_aligned_request_in_the_growable_part() {
        let p = pool();
        assert_eq!(p.validate_share(64 * MB, 32 * MB), Ok(()));
        assert_eq!(p.validate_share(96 * MB, 64 * MB), Ok(()));
        // Exactly up to the end of the window.
        assert_eq!(p.validate_share(224 * MB, 32 * MB), Ok(()));
    }

    #[test]
    fn rejects_a_pool_that_does_not_grow() {
        let p = PoolGrants::new(GuestAddress(0), 256 * MB, 256 * MB, 0, 0);
        assert_eq!(
            p.validate_share(0, 32 * MB),
            Err(GrantError::NotGrowable),
            "step 0 means fully pre-shared; there is nothing to grant"
        );
    }

    #[test]
    fn rejects_the_pre_allocated_floor() {
        let p = pool();
        // The floor is shared before boot under the shm node's label space, so a runtime unshare
        // could never reclaim it; handing it out again would be a second live SHARE at one gpa.
        assert_eq!(p.validate_share(0, 32 * MB), Err(GrantError::BelowFloor));
        assert_eq!(
            p.validate_share(32 * MB, 32 * MB),
            Err(GrantError::BelowFloor)
        );
    }

    #[test]
    fn rejects_past_the_window() {
        let p = pool();
        assert_eq!(
            p.validate_share(224 * MB, 64 * MB),
            Err(GrantError::PastWindow)
        );
        assert_eq!(
            p.validate_share(256 * MB, 32 * MB),
            Err(GrantError::PastWindow)
        );
        // Guest-supplied values: an overflowing offset+len must not wrap into a valid range.
        assert_eq!(
            p.validate_share(64 * MB, u64::MAX),
            Err(GrantError::PastWindow)
        );
    }

    #[test]
    fn rejects_misalignment_in_either_offset_or_length() {
        let p = pool();
        assert_eq!(
            p.validate_share(64 * MB + 4096, 32 * MB),
            Err(GrantError::Misaligned)
        );
        assert_eq!(
            p.validate_share(64 * MB, 32 * MB + 4096),
            Err(GrantError::Misaligned)
        );
        assert_eq!(p.validate_share(64 * MB, 2 * MB), Err(GrantError::Misaligned));
    }

    #[test]
    fn rejects_an_empty_range() {
        assert_eq!(pool().validate_share(64 * MB, 0), Err(GrantError::EmptyRange));
    }

    #[test]
    fn rejects_granting_the_same_slot_twice() {
        let mut p = pool();
        p.mark_granted(64 * MB, 32 * MB, &[7]).unwrap();
        assert_eq!(
            p.validate_share(64 * MB, 32 * MB),
            Err(GrantError::AlreadyGranted)
        );
        // Also when the new request merely overlaps the granted slot.
        assert_eq!(
            p.validate_share(64 * MB, 64 * MB),
            Err(GrantError::AlreadyGranted)
        );
    }

    #[test]
    fn rejects_exceeding_the_memparcel_quota() {
        // Two slots allowed; the window has six.
        let mut p = PoolGrants::new(GuestAddress(0), 256 * MB, 64 * MB, 32 * MB, 2);
        p.mark_granted(64 * MB, 64 * MB, &[1, 2]).unwrap();
        assert_eq!(
            p.validate_share(128 * MB, 32 * MB),
            Err(GrantError::QuotaExceeded)
        );
        // And a single request larger than the whole quota is refused up front.
        let q = PoolGrants::new(GuestAddress(0), 256 * MB, 64 * MB, 32 * MB, 2);
        assert_eq!(
            q.validate_share(64 * MB, 96 * MB),
            Err(GrantError::QuotaExceeded)
        );
    }

    #[test]
    fn rejects_releasing_what_was_never_granted() {
        let mut p = pool();
        assert_eq!(
            p.validate_unshare(64 * MB, 32 * MB),
            Err(GrantError::NotGranted)
        );
        p.mark_granted(64 * MB, 32 * MB, &[7]).unwrap();
        assert_eq!(p.validate_unshare(64 * MB, 32 * MB), Ok(()));
        // A range that is only partly granted is refused whole.
        assert_eq!(
            p.validate_unshare(64 * MB, 64 * MB),
            Err(GrantError::NotGranted)
        );
    }

    #[test]
    fn a_refused_request_changes_nothing() {
        let mut p = pool();
        p.mark_granted(64 * MB, 32 * MB, &[7]).unwrap();
        let before = p.live_grants();
        for (off, len) in [
            (0, 32 * MB),              // below floor
            (64 * MB, 32 * MB),        // already granted
            (64 * MB + 4096, 32 * MB), // misaligned
            (240 * MB, 32 * MB),       // past window
        ] {
            let _ = p.validate_share(off, len);
        }
        assert_eq!(p.live_grants(), before);
        assert!(p.is_backed(GuestAddress(0x1_9000_0000 + 64 * MB)));
    }

    #[test]
    fn is_backed_tracks_the_floor_and_the_grants() {
        let mut p = pool();
        let base = 0x1_9000_0000u64;
        // The floor is always backed.
        assert!(p.is_backed(GuestAddress(base)));
        assert!(p.is_backed(GuestAddress(base + 64 * MB - 1)));
        // The growable part is not, until it is granted. This is the case that silently reads as
        // zeros on the guest side rather than faulting, which is why the host must ask.
        assert!(!p.is_backed(GuestAddress(base + 64 * MB)));
        p.mark_granted(64 * MB, 32 * MB, &[7]).unwrap();
        assert!(p.is_backed(GuestAddress(base + 64 * MB)));
        assert!(p.is_backed(GuestAddress(base + 96 * MB - 1)));
        assert!(!p.is_backed(GuestAddress(base + 96 * MB)));
        // Outside the pool entirely.
        assert!(!p.is_backed(GuestAddress(base - 1)));
        assert!(!p.is_backed(GuestAddress(base + 256 * MB)));
    }

    #[test]
    fn take_granted_returns_the_handles_and_frees_the_slots() {
        let mut p = pool();
        p.mark_granted(64 * MB, 64 * MB, &[11, 12]).unwrap();
        assert_eq!(p.take_granted(64 * MB, 64 * MB).unwrap(), vec![11, 12]);
        assert_eq!(p.live_grants(), 0);
        assert!(!p.is_backed(GuestAddress(0x1_9000_0000 + 64 * MB)));
        // Now grantable again -- reusing an address is safe, the RM merges the range back.
        assert_eq!(p.validate_share(64 * MB, 64 * MB), Ok(()));
    }

    #[test]
    fn drain_all_reports_addresses_not_offsets() {
        let mut p = pool();
        p.mark_granted(96 * MB, 32 * MB, &[42]).unwrap();
        assert_eq!(
            p.drain_all(),
            vec![(0x1_9000_0000 + 96 * MB, 42)],
            "teardown needs the gpa, because the reclaim label is derived from it"
        );
        assert_eq!(p.live_grants(), 0);
    }
}
