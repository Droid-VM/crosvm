    /// Set while the host is unregistering the guest mapping. New host references must not
    /// enter the grant until the unregister and bookkeeping have completed.
    releasing: bool,
        if grants
            .iter()
            .any(|o| self.granted.get(o).is_some_and(|g| g.releasing))
        {
            return Err(GrantError::Busy);
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

        if self.max_grants != 0 && self.granted.len() + 1 > self.max_grants as usize {
        self.granted.insert(
            offset,
            Grant {
                len,
                handle,
                refs: 0,
                releasing: false,
            },
        );
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

