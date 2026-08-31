// SPDX-License-Identifier: GPL-2.0-or-later
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
//! A grant is a variable-length extent, step-aligned, above the pre-allocated floor -- not a fixed
//! slot -- because one grant is one RM memparcel however large it is. The floor itself is not
//! represented: it is shared before boot under a different label space (the shm vdevice node's
//! region index rather than `gpa >> 12`) and so can never be reclaimed at runtime, which is
//! exactly what a floor should be.

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
    /// The range overlaps a live grant. Granting again would collide on the `gpa >> 12` reclaim
    /// label, which is derived from the address rather than looked up.
    AlreadyGranted,
    /// No grant starts here.
    NotGranted,
    /// A grant starts here but is a different length. A grant is ONE memparcel and the RM reclaims
    /// it whole, so it can only be released exactly as it was taken. A caller that wants to give
    /// part of it back has to release all of it and re-grow the remainder.
    PartialRelease,
    /// Some part of the range is not inside a live grant. From the host's own accesses this is
    /// the silent-zero case -- a read of unbacked pool memory returns zeros rather than faulting
    /// -- so it has to be refused explicitly.
    NotBacked,
    /// The range is still referenced by something on the host: a dma-buf built over it, a GPU
    /// mapping, a scanout iovec. The guest cannot know this, because its RESOURCE_UNREF is
    /// fire-and-forget and it drops a buffer from its allocator while the host still holds it.
    Busy,
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
            GrantError::PartialRelease => libc::ERANGE,
            GrantError::NotBacked => libc::EFAULT,
            GrantError::Busy => libc::EBUSY,
            _ => libc::EINVAL,
        }
    }
}

/// The backed/unbacked map of one growable pool.
///
/// Grants are variable-length extents rather than fixed slots, because a grant is exactly one RM
/// memparcel however large it is. Handing out 192 MiB in one request costs one parcel; handing out
/// the same 192 MiB as six 32 MiB requests costs six. With MAX_MEMPARCEL_PER_VM at 1024 for the
/// whole VM -- shared with Android's, and not returned by anything short of a reboot for what a
/// killed VMM leaves behind -- that difference decides whether a pool can be large at all.
///
/// The price is that a grant is also the unit of RELEASE: `runtime_unshare` derives its reclaim
/// label from the base address, so the RM gives back what it took, entire. A caller that wants
/// fine-grained release grows in small requests; one that wants quota efficiency grows in big
/// ones. That choice is made at grow time and cannot be revised later, which is why a partial
/// release is refused loudly rather than approximated.
/// One live grant: one RM memparcel, and how many host-side objects are built over it.
///
/// The count is per GRANT rather than per page because the grant is the unit of release -- there
/// is no way to give back part of a parcel, so finer tracking would answer a question that cannot
/// be acted on.
#[derive(Debug)]
struct Grant {
    len: u64,
    handle: u32,
    /// Host objects referencing any part of this grant: dma-bufs, GPU mappings, scanout iovecs.
    refs: u32,
    /// Set while the host is unregistering the guest mapping. New host references must not
    /// enter the grant until the unregister and bookkeeping have completed.
    releasing: bool,
}

#[derive(Debug)]
pub struct PoolGrants {
    base: u64,
    size: u64,
    pre_alloc: u64,
    step: u64,
    max_grants: u32,
    /// Offset from the pool base -> the grant there. Non-overlapping by construction.
    granted: BTreeMap<u64, Grant>,
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

    pub fn size(&self) -> u64 {
        self.size
    }

    pub fn step(&self) -> u64 {
        self.step
    }

    /// Live grants, which is also the number of memparcels this pool is holding.
    pub fn live_grants(&self) -> usize {
        self.granted.len()
    }

    /// Range checks that do not depend on what is currently granted.
    fn check_range(&self, offset: u64, len: u64) -> Result<(), GrantError> {
        if self.step == 0 {
            return Err(GrantError::NotGrowable);
        }
        if len == 0 {
            return Err(GrantError::EmptyRange);
        }
        if offset < self.pre_alloc {
            return Err(GrantError::BelowFloor);
        }
        // Checked throughout: offset and len arrive from the guest.
        let end = offset.checked_add(len).ok_or(GrantError::PastWindow)?;
        if end > self.size {
            return Err(GrantError::PastWindow);
        }
        if offset % self.step != 0 || len % self.step != 0 {
            return Err(GrantError::Misaligned);
        }
        Ok(())
    }

    /// Does `[offset, offset+len)` touch any live grant?
    fn overlaps(&self, offset: u64, len: u64) -> bool {
        // The grant starting at or before `offset`, plus everything starting inside the range.
        if let Some((&o, g)) = self.granted.range(..=offset).next_back() {
            if o + g.len > offset {
                return true;
            }
        }
        self.granted.range(offset..offset + len).next().is_some()
    }

    /// Offsets of the grants that `[offset, offset+len)` falls inside, or None if any part of the
    /// range is not backed. The floor counts as backed and contributes no grant, since it is
    /// never released.
    fn covering_grants(&self, offset: u64, len: u64) -> Option<Vec<u64>> {
        let end = offset.checked_add(len)?;
        if end > self.size {
            return None;
        }
        let mut out = Vec::new();
        let mut at = offset;
        // The pre-shared floor is permanently backed; skip past whatever part of the range is in
        // it before looking for grants.
        if at < self.pre_alloc {
            at = self.pre_alloc.min(end);
        }
        while at < end {
            let (&o, g) = self.granted.range(..=at).next_back()?;
            if at >= o + g.len {
                return None; // a hole
            }
            out.push(o);
            at = o + g.len;
        }
        Some(out)
    }

    /// Is every byte of `[offset, offset+len)` backed right now?
    pub fn range_backed(&self, offset: u64, len: u64) -> bool {
        len == 0 || self.covering_grants(offset, len).is_some()
    }

    /// Take a host-side reference on every grant the range touches. All or nothing: a range that
    /// is partly unbacked refs nothing, so a caller cannot leave counts raised on a failure.
    pub fn ref_range(&mut self, offset: u64, len: u64) -> Result<(), GrantError> {
        if len == 0 {
            return Ok(());
        }
        let grants = self.covering_grants(offset, len).ok_or(GrantError::NotBacked)?;
        if grants
            .iter()
            .any(|o| self.granted.get(o).is_some_and(|g| g.releasing))
        {
            return Err(GrantError::Busy);
        }
        for o in grants {
            let g = self.granted.get_mut(&o).expect("just found");
            g.refs = g.refs.saturating_add(1);
        }
        Ok(())
    }

    /// Check whether a new host reference may be added to this range. This is separate from
    /// [`range_backed`] because a range remains physically backed while its guest mapping is
    /// being torn down, but accepting a new reference during that window would race the punch.
    pub fn ref_range_available(&self, offset: u64, len: u64) -> Result<(), GrantError> {
        if len == 0 {
            return Ok(());
        }
        let grants = self.covering_grants(offset, len).ok_or(GrantError::NotBacked)?;
        if grants
            .iter()
            .any(|o| self.granted.get(o).is_some_and(|g| g.releasing))
        {
            return Err(GrantError::Busy);
        }
        Ok(())
    }

    /// Drop a reference taken by [`ref_range`]. Saturating rather than panicking on underflow:
    /// losing track of a count is bad, but taking the VM down over it is worse, and the count
    /// only ever gates a shrink.
    pub fn unref_range(&mut self, offset: u64, len: u64) {
        if len == 0 {
            return;
        }
        let Some(grants) = self.covering_grants(offset, len) else {
            return;
        };
        for o in grants {
            let g = self.granted.get_mut(&o).expect("just found");
            g.refs = g.refs.saturating_sub(1);
        }
    }

    /// Check a grow request. Modifies nothing: a refused request must leave no trace, so the guest
    /// can try a different range without reconciling first.
    pub fn validate_share(&self, offset: u64, len: u64) -> Result<(), GrantError> {
        self.check_range(offset, len)?;
        if self.overlaps(offset, len) {
            return Err(GrantError::AlreadyGranted);
        }
        // One grant, one memparcel, whatever its length.
        if self.max_grants != 0 && self.granted.len() + 1 > self.max_grants as usize {
            return Err(GrantError::QuotaExceeded);
        }
        Ok(())
    }

    /// Check a shrink request. Must name a grant exactly.
    pub fn validate_unshare(&self, offset: u64, len: u64) -> Result<(), GrantError> {
        self.check_range(offset, len)?;
        match self.granted.get(&offset) {
            None => Err(GrantError::NotGranted),
            Some(g) if g.len != len => Err(GrantError::PartialRelease),
            // Refuse while the host still has something built over it. The guest cannot make this
            // call correctly on its own: it drops a buffer from its allocator the moment it sends
            // RESOURCE_UNREF, without waiting to hear that the host is done.
            Some(g) if g.refs != 0 => Err(GrantError::Busy),
            Some(_) => Ok(()),
        }
    }

    /// Record a completed grant.
    pub fn mark_granted(&mut self, offset: u64, len: u64, handle: u32) -> Result<(), GrantError> {
        self.validate_share(offset, len)?;
        self.granted.insert(
            offset,
            Grant {
                len,
                handle,
                refs: 0,
                releasing: false,
            },
        );
        Ok(())
    }

    /// Reserve a grant for unregistering. The reservation closes the gap between checking the
    /// reference count and unregistering the guest mapping: new dma-buf/resource references are
    /// rejected until the caller either finishes or cancels the operation.
    pub fn begin_unshare(&mut self, offset: u64, len: u64) -> Result<(), GrantError> {
        self.check_range(offset, len)?;
        match self.granted.get_mut(&offset) {
            None => Err(GrantError::NotGranted),
            Some(g) if g.len != len => Err(GrantError::PartialRelease),
            Some(g) if g.refs != 0 || g.releasing => Err(GrantError::Busy),
            Some(g) => {
                g.releasing = true;
                Ok(())
            }
        }
    }

    /// Cancel a failed unregister and make the grant available again. A missing or mismatched
    /// grant is intentionally ignored: the caller only uses this after a successful begin.
    pub fn cancel_unshare(&mut self, offset: u64, len: u64) {
        if let Some(g) = self.granted.get_mut(&offset) {
            if g.len == len && g.releasing {
                g.releasing = false;
            }
        }
    }

    /// Complete a previously reserved unregister and forget the grant.
    pub fn finish_unshare(&mut self, offset: u64, len: u64) -> Result<u32, GrantError> {
        self.check_range(offset, len)?;
        match self.granted.get(&offset) {
            None => Err(GrantError::NotGranted),
            Some(g) if g.len != len => Err(GrantError::PartialRelease),
            Some(g) if g.refs != 0 || !g.releasing => Err(GrantError::Busy),
            Some(_) => Ok(self.granted.remove(&offset).expect("validated above").handle),
        }
    }

    /// Forget a released grant, returning its RM handle.
    pub fn take_granted(&mut self, offset: u64, len: u64) -> Result<u32, GrantError> {
        self.validate_unshare(offset, len)?;
        Ok(self.granted.remove(&offset).expect("validated above").handle)
    }

    /// Every live grant as (gpa, len, handle), for teardown and for the guest's reconciliation.
    /// Addresses rather than offsets, because the reclaim label is derived from the address.
    pub fn drain_all(&mut self) -> Vec<(u64, u64, u32)> {
        std::mem::take(&mut self.granted)
            .into_iter()
            .map(|(off, g)| (self.base + off, g.len, g.handle))
            .collect()
    }

    /// Is this guest physical address backed right now? The pre-allocated floor always is; above
    /// it, only addresses inside a live grant. Outside the pool answers false.
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
        self.granted
            .range(..=off)
            .next_back()
            .is_some_and(|(&o, g)| off < o + g.len)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const MB: u64 = 1 << 20;

    /// 256 MiB window, 64 MiB floor, 32 MiB step, 64 grants allowed.
    fn pool() -> PoolGrants {
        PoolGrants::new(GuestAddress(0x1_9000_0000), 256 * MB, 64 * MB, 32 * MB, 64)
    }

    #[test]
    fn accepts_one_step_and_whole_multiples_of_it() {
        let p = pool();
        assert_eq!(p.validate_share(64 * MB, 32 * MB), Ok(()));
        // The point of allowing multiples: 192 MiB in one request is ONE memparcel.
        assert_eq!(p.validate_share(64 * MB, 192 * MB), Ok(()));
        // Exactly up to the end of the window.
        assert_eq!(p.validate_share(224 * MB, 32 * MB), Ok(()));
    }

    #[test]
    fn a_large_grant_costs_one_memparcel_not_one_per_step() {
        let mut p = PoolGrants::new(GuestAddress(0), 256 * MB, 64 * MB, 32 * MB, 1);
        // Six steps, one grant, and the quota is one.
        assert_eq!(p.mark_granted(64 * MB, 192 * MB, 7), Ok(()));
        assert_eq!(p.live_grants(), 1);
    }

    #[test]
    fn rejects_a_pool_that_does_not_grow() {
        let p = PoolGrants::new(GuestAddress(0), 256 * MB, 256 * MB, 0, 0);
        assert_eq!(p.validate_share(0, 32 * MB), Err(GrantError::NotGrowable));
    }

    #[test]
    fn rejects_the_pre_allocated_floor() {
        // Shared before boot under the shm node's label space, so a runtime unshare could never
        // reclaim it; handing it out would be a second live SHARE at one address.
        let p = pool();
        assert_eq!(p.validate_share(0, 32 * MB), Err(GrantError::BelowFloor));
        assert_eq!(p.validate_share(32 * MB, 32 * MB), Err(GrantError::BelowFloor));
    }

    #[test]
    fn rejects_past_the_window() {
        let p = pool();
        assert_eq!(p.validate_share(224 * MB, 64 * MB), Err(GrantError::PastWindow));
        assert_eq!(p.validate_share(256 * MB, 32 * MB), Err(GrantError::PastWindow));
        // Guest-supplied: an overflowing offset+len must not wrap into a valid range.
        assert_eq!(p.validate_share(64 * MB, u64::MAX), Err(GrantError::PastWindow));
    }

    #[test]
    fn rejects_misalignment_in_either_offset_or_length() {
        let p = pool();
        assert_eq!(p.validate_share(64 * MB + 4096, 32 * MB), Err(GrantError::Misaligned));
        assert_eq!(p.validate_share(64 * MB, 32 * MB + 4096), Err(GrantError::Misaligned));
        assert_eq!(p.validate_share(64 * MB, 2 * MB), Err(GrantError::Misaligned));
    }

    #[test]
    fn rejects_an_empty_range() {
        assert_eq!(pool().validate_share(64 * MB, 0), Err(GrantError::EmptyRange));
    }

    #[test]
    fn rejects_any_overlap_with_a_live_grant() {
        let mut p = pool();
        p.mark_granted(96 * MB, 64 * MB, 7).unwrap();
        // Exactly on top.
        assert_eq!(p.validate_share(96 * MB, 64 * MB), Err(GrantError::AlreadyGranted));
        // Starting inside it.
        assert_eq!(p.validate_share(128 * MB, 32 * MB), Err(GrantError::AlreadyGranted));
        // Starting before and running into it -- the case a naive "is this offset taken" misses.
        assert_eq!(p.validate_share(64 * MB, 64 * MB), Err(GrantError::AlreadyGranted));
        // Enclosing it entirely.
        assert_eq!(p.validate_share(64 * MB, 192 * MB), Err(GrantError::AlreadyGranted));
        // Abutting on either side is fine.
        assert_eq!(p.validate_share(64 * MB, 32 * MB), Ok(()));
        assert_eq!(p.validate_share(160 * MB, 32 * MB), Ok(()));
    }

    #[test]
    fn rejects_exceeding_the_memparcel_quota() {
        let mut p = PoolGrants::new(GuestAddress(0), 256 * MB, 64 * MB, 32 * MB, 2);
        p.mark_granted(64 * MB, 32 * MB, 1).unwrap();
        p.mark_granted(96 * MB, 32 * MB, 2).unwrap();
        assert_eq!(p.validate_share(128 * MB, 32 * MB), Err(GrantError::QuotaExceeded));
        // But one BIG grant would have fitted, which is the whole reason multiples are allowed.
        let q = PoolGrants::new(GuestAddress(0), 256 * MB, 64 * MB, 32 * MB, 2);
        assert_eq!(q.validate_share(64 * MB, 192 * MB), Ok(()));
    }

    #[test]
    fn a_grant_must_be_released_exactly_as_it_was_taken() {
        let mut p = pool();
        p.mark_granted(64 * MB, 192 * MB, 7).unwrap();
        // The RM reclaims a parcel whole, so giving part of it back is not expressible.
        assert_eq!(p.validate_unshare(64 * MB, 32 * MB), Err(GrantError::PartialRelease));
        assert_eq!(p.validate_unshare(64 * MB, 224 * MB), Err(GrantError::PastWindow));
        // Not at the start of the grant either.
        assert_eq!(p.validate_unshare(96 * MB, 32 * MB), Err(GrantError::NotGranted));
        assert_eq!(p.validate_unshare(64 * MB, 192 * MB), Ok(()));
    }

    #[test]
    fn rejects_releasing_what_was_never_granted() {
        let p = pool();
        assert_eq!(p.validate_unshare(64 * MB, 32 * MB), Err(GrantError::NotGranted));
    }

    #[test]
    fn a_refused_request_changes_nothing() {
        let mut p = pool();
        p.mark_granted(64 * MB, 32 * MB, 7).unwrap();
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
        assert!(p.is_backed(GuestAddress(base)));
        assert!(p.is_backed(GuestAddress(base + 64 * MB - 1)));
        // Not until granted. This is the case that silently reads as zeros on the guest side
        // rather than faulting, which is why the host has to ask rather than wait for an error.
        assert!(!p.is_backed(GuestAddress(base + 64 * MB)));
        p.mark_granted(64 * MB, 64 * MB, 7).unwrap();
        assert!(p.is_backed(GuestAddress(base + 64 * MB)));
        assert!(p.is_backed(GuestAddress(base + 128 * MB - 1)));
        assert!(!p.is_backed(GuestAddress(base + 128 * MB)));
        assert!(!p.is_backed(GuestAddress(base - 1)));
        assert!(!p.is_backed(GuestAddress(base + 256 * MB)));
    }

    #[test]
    fn take_granted_returns_the_handle_and_frees_the_extent() {
        let mut p = pool();
        p.mark_granted(64 * MB, 64 * MB, 11).unwrap();
        assert_eq!(p.take_granted(64 * MB, 64 * MB), Ok(11));
        assert_eq!(p.live_grants(), 0);
        assert!(!p.is_backed(GuestAddress(0x1_9000_0000 + 64 * MB)));
        // Grantable again: reusing an address is safe, the RM merges the range back.
        assert_eq!(p.validate_share(64 * MB, 64 * MB), Ok(()));
    }

    #[test]
    fn a_referenced_grant_cannot_be_released() {
        let mut p = pool();
        p.mark_granted(64 * MB, 32 * MB, 7).unwrap();
        assert_eq!(p.validate_unshare(64 * MB, 32 * MB), Ok(()));

        // A dma-buf gets built over part of it.
        p.ref_range(64 * MB, 4 * MB).unwrap();
        assert_eq!(
            p.validate_unshare(64 * MB, 32 * MB),
            Err(GrantError::Busy),
            "the guest cannot know this: its RESOURCE_UNREF does not wait for the host"
        );

        p.unref_range(64 * MB, 4 * MB);
        assert_eq!(p.validate_unshare(64 * MB, 32 * MB), Ok(()));
    }

    #[test]
    fn an_unshare_reservation_blocks_new_references_until_cancelled_or_finished() {
        let mut p = pool();
        p.mark_granted(64 * MB, 32 * MB, 7).unwrap();
        p.begin_unshare(64 * MB, 32 * MB).unwrap();
        assert_eq!(
            p.ref_range(64 * MB, 4 * MB),
            Err(GrantError::Busy),
            "a resource create racing unregister must not enter the grant"
        );
        assert_eq!(p.validate_unshare(64 * MB, 32 * MB), Err(GrantError::Busy));

        p.cancel_unshare(64 * MB, 32 * MB);
        p.ref_range(64 * MB, 4 * MB).unwrap();
        p.unref_range(64 * MB, 4 * MB);
        p.begin_unshare(64 * MB, 32 * MB).unwrap();
        assert_eq!(p.finish_unshare(64 * MB, 32 * MB), Ok(7));
        assert_eq!(p.live_grants(), 0);
    }

    #[test]
    fn a_reference_spanning_two_grants_holds_both() {
        let mut p = pool();
        p.mark_granted(64 * MB, 32 * MB, 1).unwrap();
        p.mark_granted(96 * MB, 32 * MB, 2).unwrap();
        // One buffer across the join -- legal, because the whole window is one memfd region.
        p.ref_range(80 * MB, 32 * MB).unwrap();
        assert_eq!(p.validate_unshare(64 * MB, 32 * MB), Err(GrantError::Busy));
        assert_eq!(p.validate_unshare(96 * MB, 32 * MB), Err(GrantError::Busy));
        p.unref_range(80 * MB, 32 * MB);
        assert_eq!(p.validate_unshare(64 * MB, 32 * MB), Ok(()));
        assert_eq!(p.validate_unshare(96 * MB, 32 * MB), Ok(()));
    }

    #[test]
    fn refusing_a_partly_unbacked_range_refs_nothing() {
        let mut p = pool();
        p.mark_granted(64 * MB, 32 * MB, 1).unwrap();
        // Starts in a grant, runs off the end of it into a hole.
        assert_eq!(p.ref_range(80 * MB, 32 * MB), Err(GrantError::NotBacked));
        // The grant it did touch must be releasable, or a rejected import would pin memory.
        assert_eq!(p.validate_unshare(64 * MB, 32 * MB), Ok(()));
    }

    #[test]
    fn range_backed_covers_the_floor_and_spans_adjacent_grants() {
        let mut p = pool();
        // The floor is permanently backed and contributes no grant.
        assert!(p.range_backed(0, 64 * MB));
        assert!(!p.range_backed(0, 96 * MB));
        p.mark_granted(64 * MB, 32 * MB, 1).unwrap();
        // A range straddling the floor and a grant.
        assert!(p.range_backed(32 * MB, 64 * MB));
        assert!(!p.range_backed(64 * MB, 64 * MB));
        p.mark_granted(96 * MB, 32 * MB, 2).unwrap();
        // Two abutting grants read as one continuous backed range.
        assert!(p.range_backed(64 * MB, 64 * MB));
        assert!(!p.range_backed(64 * MB, 96 * MB));
    }

    #[test]
    fn drain_all_reports_addresses_not_offsets() {
        let mut p = pool();
        p.mark_granted(96 * MB, 64 * MB, 42).unwrap();
        assert_eq!(
            p.drain_all(),
            vec![(0x1_9000_0000 + 96 * MB, 64 * MB, 42)],
            "teardown needs the gpa, because the reclaim label is derived from it"
        );
        assert_eq!(p.live_grants(), 0);
    }
}
