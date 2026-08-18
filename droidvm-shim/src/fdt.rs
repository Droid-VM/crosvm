// Copyright 2026 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// `no_std` for the shim, which has no std to speak of, but not under `cfg(test)`: the test
// harness needs std, and this file is arithmetic over a byte slice that has no reason to wait for
// a phone to be told it is wrong.
#![cfg_attr(not(test), no_std)]

//! Just enough flattened device tree to do two things, and nothing more.
//!
//! NO LIBFDT, deliberately. What the shim needs is one property read -- the resource manager's
//! message-queue capabilities -- and one property overwrite that does not change the property's
//! length. Neither moves a byte of the structure block, so a few hundred lines replace a library,
//! a build dependency, and the temptation to do more here than belongs here.
//!
//! Every read is bounds-checked against the blob. The tree arrives having been rewritten by the
//! resource manager, and this code runs before anything else in the VM with no console to report
//! from: a walk that wanders off the end would be a hang with no explanation, so it returns an
//! error instead and the caller writes that into the handoff page.

const FDT_MAGIC: u32 = 0xd00d_feed;
const FDT_BEGIN_NODE: u32 = 1;
const FDT_END_NODE: u32 = 2;
const FDT_PROP: u32 = 3;
const FDT_NOP: u32 = 4;
const FDT_END: u32 = 9;

/// Nesting deeper than this is not a tree we were given; it is a blob that went wrong.
const MAX_DEPTH: usize = 24;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    BadMagic,
    Truncated,
    BadTag,
    TooDeep,
    NotFound,
}

/// What to look for: a node by the name of a direct child of the root, or by a string in some
/// node's `compatible` list at any depth.
#[derive(Clone, Copy)]
pub enum Match<'a> {
    RootChild(&'a str),
    Compatible(&'a str),
}

pub struct Fdt<'a> {
    blob: &'a mut [u8],
    struct_off: usize,
    struct_len: usize,
    strings_off: usize,
    strings_len: usize,
}

fn be32(b: &[u8], at: usize) -> Result<u32, Error> {
    let s = b.get(at..at + 4).ok_or(Error::Truncated)?;
    Ok(u32::from_be_bytes([s[0], s[1], s[2], s[3]]))
}

pub fn be64(b: &[u8]) -> Result<u64, Error> {
    let s = b.get(..8).ok_or(Error::Truncated)?;
    Ok(u64::from_be_bytes([
        s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7],
    ]))
}

pub fn put_be64(b: &mut [u8], v: u64) -> Result<(), Error> {
    let s = b.get_mut(..8).ok_or(Error::Truncated)?;
    s.copy_from_slice(&v.to_be_bytes());
    Ok(())
}

/// A NUL-terminated string starting at `at`, as bytes, without the terminator.
fn cstr(b: &[u8], at: usize) -> Result<&[u8], Error> {
    let rest = b.get(at..).ok_or(Error::Truncated)?;
    let end = rest.iter().position(|&c| c == 0).ok_or(Error::Truncated)?;
    Ok(&rest[..end])
}

impl<'a> Fdt<'a> {
    pub fn new(blob: &'a mut [u8]) -> Result<Self, Error> {
        if be32(blob, 0)? != FDT_MAGIC {
            return Err(Error::BadMagic);
        }
        let struct_off = be32(blob, 8)? as usize;
        let strings_off = be32(blob, 12)? as usize;
        let strings_len = be32(blob, 32)? as usize;
        let struct_len = be32(blob, 36)? as usize;
        // Every later access is bounds-checked anyway; refusing here means an obviously corrupt
        // header is reported as such rather than as "node not found" thirty lines later.
        if struct_off
            .checked_add(struct_len)
            .is_none_or(|end| end > blob.len())
            || strings_off
                .checked_add(strings_len)
                .is_none_or(|end| end > blob.len())
        {
            return Err(Error::Truncated);
        }
        Ok(Fdt {
            blob,
            struct_off,
            struct_len,
            strings_off,
            strings_len,
        })
    }

    fn prop_name(&self, nameoff: usize) -> Result<&[u8], Error> {
        let strings = self
            .blob
            .get(self.strings_off..self.strings_off + self.strings_len)
            .ok_or(Error::Truncated)?;
        cstr(strings, nameoff)
    }

    /// Where the value of `prop` lives inside the blob, for the node `want` picks out.
    ///
    /// The state that matters is per node, not global: `/hypervisor` holds a doorbell or a message
    /// queue for every virtual device, each with its own `reg`, and the resource manager's node is
    /// one child among them. A walk that remembered "something matched" and "the last reg I saw"
    /// would hand back whichever came last.
    pub fn find_prop(&self, want: Match, prop: &str) -> Result<(usize, usize), Error> {
        struct Frame {
            val: Option<(usize, usize)>,
            matched: bool,
        }
        let mut stack: [Frame; MAX_DEPTH] = core::array::from_fn(|_| Frame {
            val: None,
            matched: false,
        });
        let mut depth = 0usize;
        let mut p = self.struct_off;
        let end = self.struct_off + self.struct_len;

        while p < end {
            let tag = be32(self.blob, p)?;
            p += 4;
            match tag {
                FDT_BEGIN_NODE => {
                    let name = cstr(self.blob, p)?;
                    p += (name.len() + 4) & !3;
                    if depth >= MAX_DEPTH {
                        return Err(Error::TooDeep);
                    }
                    stack[depth].val = None;
                    // Depth 0 is the root, so its direct children open at depth 1.
                    stack[depth].matched = match want {
                        Match::RootChild(n) => depth == 1 && name == n.as_bytes(),
                        Match::Compatible(_) => false,
                    };
                    depth += 1;
                }
                FDT_END_NODE => {
                    depth = depth.checked_sub(1).ok_or(Error::BadTag)?;
                    if stack[depth].matched {
                        if let Some(hit) = stack[depth].val {
                            return Ok(hit);
                        }
                    }
                }
                FDT_PROP => {
                    let len = be32(self.blob, p)? as usize;
                    let nameoff = be32(self.blob, p + 4)? as usize;
                    let val_at = p + 8;
                    if val_at.checked_add(len).is_none_or(|e| e > end) {
                        return Err(Error::Truncated);
                    }
                    p = val_at + ((len + 3) & !3);
                    let frame = stack.get_mut(depth.wrapping_sub(1)).ok_or(Error::BadTag)?;
                    let pname = self.prop_name(nameoff)?;
                    if let Match::Compatible(c) = want {
                        if pname == b"compatible" {
                            let val = self.blob.get(val_at..val_at + len).ok_or(Error::Truncated)?;
                            // A compatible is a list of NUL-separated strings.
                            if val.split(|&b| b == 0).any(|s| s == c.as_bytes()) {
                                frame.matched = true;
                            }
                        }
                    }
                    if pname == prop.as_bytes() {
                        frame.val = Some((val_at, len));
                    }
                }
                FDT_NOP => {}
                FDT_END => return Err(Error::NotFound),
                _ => return Err(Error::BadTag),
            }
        }
        Err(Error::NotFound)
    }

    pub fn prop_bytes(&self, at: usize, len: usize) -> Result<&[u8], Error> {
        self.blob.get(at..at + len).ok_or(Error::Truncated)
    }

    /// Point `/memory` at the window, and at nothing else.
    ///
    /// Everything the property arrives with is memory the payload must not treat as RAM: the boot
    /// region is lent, so the host cannot see it and a virtio buffer placed there would not work;
    /// the resource manager's own low-memory donation is mapped without execute permission, which
    /// is a fault that surfaces much later as a crash in whatever happened to be JITted into it;
    /// a pool's floor and the handoff page belong to their drivers. The window is the guest's
    /// memory, and it is the one range that was never in this property to begin with.
    ///
    /// The whole window goes in the FIRST range and the rest are emptied, rather than being split
    /// evenly across them: EDK2 reads only the first range (PrePi's FindMemnode) and would
    /// otherwise run in a fraction of the memory it was given, while Linux skips a zero-sized
    /// range outright. Either way the property keeps its length, so the structure block does not
    /// move and no library is needed to move it.
    pub fn set_memory_window(&mut self, base: u64, size: u64) -> Result<(), Error> {
        let (at, len) = self.find_prop(Match::RootChild("memory"), "reg")?;
        if len < 16 || len % 16 != 0 {
            return Err(Error::Truncated);
        }
        let entries = len / 16;
        let reg = self.blob.get_mut(at..at + len).ok_or(Error::Truncated)?;
        put_be64(&mut reg[0..8], base)?;
        put_be64(&mut reg[8..16], size)?;
        for i in 1..entries {
            put_be64(&mut reg[i * 16..i * 16 + 8], 0)?;
            put_be64(&mut reg[i * 16 + 8..i * 16 + 16], 0)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const SAMPLE: &[u8] = include_bytes!("../tests/sample.dtb");

    fn blob() -> Vec<u8> {
        SAMPLE.to_vec()
    }

    #[test]
    fn finds_the_resource_manager_and_not_its_neighbours() {
        let mut b = blob();
        let tree = Fdt::new(&mut b).unwrap();
        let (at, len) = tree
            .find_prop(Match::Compatible("gunyah-resource-manager"), "reg")
            .expect("the resource manager node is in the sample tree");
        assert_eq!(len, 16, "one address/size pair");
        let reg = tree.prop_bytes(at, 16).unwrap();
        assert_eq!(be64(&reg[0..8]).unwrap(), 0x2a, "tx capability");
        assert_eq!(be64(&reg[8..16]).unwrap(), 0x2b, "rx capability");
    }

    #[test]
    fn a_missing_node_is_missing_rather_than_a_neighbour() {
        let mut b = blob();
        let tree = Fdt::new(&mut b).unwrap();
        assert!(tree
            .find_prop(Match::Compatible("nothing-like-this"), "reg")
            .is_err());
        assert!(tree.find_prop(Match::RootChild("no-such-node"), "reg").is_err());
    }

    #[test]
    fn the_window_goes_in_the_first_range_and_the_rest_are_emptied() {
        let mut b = blob();
        let mut tree = Fdt::new(&mut b).unwrap();
        let (_, len) = tree.find_prop(Match::RootChild("memory"), "reg").unwrap();
        assert_eq!(len, 32, "the sample tree has two ranges");

        tree.set_memory_window(0x8040_0000, 0x4000_0000).unwrap();

        let (at, len) = tree.find_prop(Match::RootChild("memory"), "reg").unwrap();
        assert_eq!(len, 32, "the property keeps its length, so nothing moved");
        let reg = tree.prop_bytes(at, 32).unwrap();
        // EDK2 reads only the first range (PrePi's FindMemnode), so the whole window has to be there.
        assert_eq!(be64(&reg[0..8]).unwrap(), 0x8040_0000);
        assert_eq!(be64(&reg[8..16]).unwrap(), 0x4000_0000);
        // Linux skips a zero-sized range outright, which is how the rest disappear.
        assert_eq!(be64(&reg[16..24]).unwrap(), 0);
        assert_eq!(be64(&reg[24..32]).unwrap(), 0);
    }

    #[test]
    fn nothing_of_what_was_there_survives() {
        // The boot region is lent and the host cannot see it; the resource manager's donation is
        // mapped without execute permission; a pool's floor belongs to its driver. None of it is the
        // guest's memory, so none of it may still be in /memory when the payload reads it.
        let mut b = blob();
        let mut tree = Fdt::new(&mut b).unwrap();
        tree.set_memory_window(0x8040_0000, 0x4000_0000).unwrap();
        let (at, len) = tree.find_prop(Match::RootChild("memory"), "reg").unwrap();
        let reg = tree.prop_bytes(at, len).unwrap();
        for pair in reg.chunks(16).skip(1) {
            assert_eq!(be64(&pair[8..16]).unwrap(), 0, "a leftover range is still RAM");
        }
        assert_ne!(be64(&reg[0..8]).unwrap(), 0x4000_0000, "the donation is gone");
        assert_ne!(be64(&reg[0..8]).unwrap(), 0x8000_0000, "the boot region is gone");
    }

    #[test]
    fn a_corrupt_blob_is_refused_rather_than_walked() {
        let mut b = blob();
        b[0] = 0;
        assert!(Fdt::new(&mut b).is_err(), "bad magic");

        let mut b = blob();
        // A structure block that claims to run past the end of the blob.
        b[36..40].copy_from_slice(&0xffff_0000u32.to_be_bytes());
        assert!(Fdt::new(&mut b).is_err(), "truncated structure block");
    }
}
