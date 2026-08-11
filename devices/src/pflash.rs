// Copyright 2022 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! Programmable flash device that supports the minimum interface that OVMF
//! requires. This is purpose-built to allow OVMF to store UEFI variables in
//! the same way that it stores them on QEMU.
//!
//! For that reason it's heavily based on [QEMU's pflash implementation], while
//! taking even more shortcuts, chief among them being the complete lack of CFI
//! tables, which systems would normally use to learn how to use the device.
//!
//! In addition to full-width reads, we support single byte and word writes,
//! block erases, and status requests, which OVMF uses to probe and program the
//! device.
//!
//! Note that without SMM support in crosvm (which it doesn't yet have) this
//! device is directly accessible to potentially malicious kernels. With SMM
//! and the appropriate changes to this device this could be made more secure
//! by ensuring only the BIOS is able to touch the pflash.
//!
//! [QEMU's pflash implementation]: https://github.com/qemu/qemu/blob/master/hw/block/pflash_cfi01.c

use std::path::PathBuf;

use anyhow::bail;
use base::error;
use base::VolatileSlice;
use disk::DiskFile;
use serde::Deserialize;
use serde::Serialize;
use snapshot::AnySnapshot;

use crate::pci::CrosvmDeviceId;
use crate::BusAccessInfo;
use crate::BusDevice;
use crate::DeviceId;
use crate::Suspendable;

const COMMAND_WRITE_BYTE: u8 = 0x10;
const COMMAND_BLOCK_ERASE: u8 = 0x20;
const COMMAND_CLEAR_STATUS: u8 = 0x50;
const COMMAND_READ_STATUS: u8 = 0x70;
const COMMAND_READ_DEVICE_ID: u8 = 0x90;
const COMMAND_BUFFERED_PROGRAM: u8 = 0xe8;
const COMMAND_BLOCK_ERASE_CONFIRM: u8 = 0xd0;
const COMMAND_READ_ARRAY: u8 = 0xff;

const STATUS_READY: u8 = 0x80;

fn pflash_parameters_default_block_size() -> u32 {
    // 4K
    4 * (1 << 10)
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct PflashParameters {
    pub path: PathBuf,
    #[serde(default = "pflash_parameters_default_block_size")]
    pub block_size: u32,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
enum State {
    ReadArray,
    ReadStatus,
    ReadDeviceId,
    BlockErase(u64),
    Write(u64),
    BufferedProgramCount(u64),
    BufferedProgramData {
        next_offset: u64,
        remaining: u64,
    },
    BufferedProgramConfirm,
}

pub struct Pflash {
    image: Box<dyn DiskFile>,
    image_size: u64,
    block_size: u32,

    state: State,
    status: u8,
}

impl Pflash {
    pub fn new(image: Box<dyn DiskFile>, block_size: u32) -> anyhow::Result<Pflash> {
        if !block_size.is_power_of_two() {
            bail!("Block size {} is not a power of 2", block_size);
        }
        let image_size = image.get_len()?;
        if image_size % block_size as u64 != 0 {
            bail!(
                "Disk size {} is not a multiple of block size {}",
                image_size,
                block_size
            );
        }

        Ok(Pflash {
            image,
            image_size,
            block_size,
            state: State::ReadArray,
            status: STATUS_READY,
        })
    }

    fn read_status(&self, data: &mut [u8]) {
        for (index, value) in data.iter_mut().enumerate() {
            *value = if index % 2 == 0 { self.status } else { 0 };
        }
    }
}

impl BusDevice for Pflash {
    fn device_id(&self) -> DeviceId {
        CrosvmDeviceId::Pflash.into()
    }

    fn debug_label(&self) -> String {
        "pflash".to_owned()
    }

    fn read(&mut self, info: BusAccessInfo, data: &mut [u8]) {
        let offset = info.offset;
        match &self.state {
            State::ReadArray => {
                if offset + data.len() as u64 >= self.image_size {
                    error!("pflash read request beyond disk");
                    return;
                }
                if let Err(e) = self
                    .image
                    .read_exact_at_volatile(VolatileSlice::new(data), offset)
                {
                    error!("pflash failed to read: {}", e);
                }
            }
            State::ReadStatus => {
                self.state = State::ReadArray;
                self.read_status(data);
            }
            State::ReadDeviceId => {
                self.state = State::ReadArray;
                data.fill(0);
            }
            State::BufferedProgramCount(_) => {
                self.read_status(data);
            }
            _ => {
                error!(
                    "pflash received unexpected read in state {:?}, recovering to ReadArray mode",
                    self.state
                );
                self.state = State::ReadArray;
            }
        }
    }

    fn write(&mut self, info: BusAccessInfo, data: &[u8]) {
        if data.is_empty() {
            return;
        }
        let command = data[0];
        let offset = info.offset;

        match self.state {
            State::BufferedProgramCount(start_offset) => {
                if command == COMMAND_BUFFERED_PROGRAM {
                    self.state = State::BufferedProgramCount(offset);
                    return;
                }
                if command > 31 {
                    error!("invalid pflash buffered program word count {}", command);
                    self.state = State::ReadArray;
                    return;
                }
                let byte_count = (u64::from(command) + 1) * 4;
                if start_offset + byte_count > self.image_size {
                    error!(
                        "pflash buffered write at offset {} with size {} exceeds image size {}",
                        start_offset, byte_count, self.image_size
                    );
                    self.state = State::ReadArray;
                    return;
                }
                self.state = State::BufferedProgramData {
                    next_offset: start_offset,
                    remaining: byte_count,
                };
            }
            State::BufferedProgramData {
                next_offset,
                remaining,
            } => {
                if offset != next_offset || data.len() as u64 > remaining {
                    error!(
                        "invalid pflash buffered write at offset {} with size {}; expected offset {} with at most {} bytes remaining",
                        offset,
                        data.len(),
                        next_offset,
                        remaining
                    );
                    self.state = State::ReadArray;
                    return;
                }
                if let Err(e) = self.image.write_all_at_volatile(
                    VolatileSlice::new(&mut data.to_vec()),
                    offset,
                ) {
                    error!("failed to write buffered data to pflash: {}", e);
                    self.state = State::ReadArray;
                    return;
                }

                let remaining = remaining - data.len() as u64;
                self.state = if remaining == 0 {
                    State::BufferedProgramConfirm
                } else {
                    State::BufferedProgramData {
                        next_offset: next_offset + data.len() as u64,
                        remaining,
                    }
                };
            }
            State::BufferedProgramConfirm => {
                self.state = State::ReadArray;
                self.status = STATUS_READY;
                if command != COMMAND_BLOCK_ERASE_CONFIRM {
                    error!(
                        "pflash buffered write confirm data {}, wanted {}",
                        command, COMMAND_BLOCK_ERASE_CONFIRM
                    );
                }
            }
            State::Write(expected_offset) => {
                self.state = State::ReadArray;
                self.status = STATUS_READY;

                if offset != expected_offset {
                    error!("pflash received write for offset {} that doesn't match offset from WRITE_BYTE command {}", offset, expected_offset);
                    return;
                }
                if offset >= self.image_size {
                    error!(
                        "pflash offset {} greater than image size {}",
                        offset, self.image_size
                    );
                    return;
                }

                if offset + data.len() as u64 > self.image_size {
                    error!(
                        "pflash write at offset {} with size {} exceeds image size {}",
                        offset,
                        data.len(),
                        self.image_size
                    );
                    return;
                }

                if let Err(e) = self.image.write_all_at_volatile(
                    VolatileSlice::new(&mut data.to_vec()),
                    offset,
                ) {
                    error!("failed to write to pflash: {}", e);
                }
            }
            State::BlockErase(expected_offset) => {
                self.state = State::ReadArray;
                self.status = STATUS_READY;

                if command != COMMAND_BLOCK_ERASE_CONFIRM {
                    error!("pflash write data {} after BLOCK_ERASE command, wanted COMMAND_BLOCK_ERASE_CONFIRM", command);
                    return;
                }
                if offset != expected_offset {
                    error!("pflash offset {} for BLOCK_ERASE_CONFIRM command does not match the one for BLOCK_ERASE {}", offset, expected_offset);
                    return;
                }
                if offset >= self.image_size {
                    error!(
                        "pflash block erase attempt offset {} beyond image size {}",
                        offset, self.image_size
                    );
                    return;
                }
                if offset % self.block_size as u64 != 0 {
                    error!(
                        "pflash block erase offset {} not on block boundary with block size {}",
                        offset, self.block_size
                    );
                    return;
                }

                if let Err(e) = self.image.write_all_at_volatile(
                    VolatileSlice::new(&mut [0xff].repeat(self.block_size.try_into().unwrap())),
                    offset,
                ) {
                    error!("pflash failed to erase block: {}", e);
                }
            }
            _ => {
                // If we're not expecting anything else then assume this is a
                // command to transition states.
                match command {
                    COMMAND_READ_ARRAY => {
                        self.state = State::ReadArray;
                        self.status = STATUS_READY;
                    }
                    COMMAND_READ_STATUS => self.state = State::ReadStatus,
                    COMMAND_READ_DEVICE_ID => self.state = State::ReadDeviceId,
                    COMMAND_CLEAR_STATUS => {
                        self.state = State::ReadArray;
                        self.status = STATUS_READY;
                    }
                    COMMAND_WRITE_BYTE => self.state = State::Write(offset),
                    COMMAND_BLOCK_ERASE => self.state = State::BlockErase(offset),
                    COMMAND_BUFFERED_PROGRAM => {
                        self.state = State::BufferedProgramCount(offset)
                    }
                    _ => {
                        error!("received unexpected/unsupported pflash command {}, ignoring and returning to read mode", command);
                        self.state = State::ReadArray
                    }
                }
            }
        }
    }
}

impl Suspendable for Pflash {
    fn snapshot(&mut self) -> anyhow::Result<AnySnapshot> {
        AnySnapshot::to_any((self.status, self.state))
    }

    fn restore(&mut self, data: AnySnapshot) -> anyhow::Result<()> {
        let (status, state) = AnySnapshot::from_any(data)?;
        self.status = status;
        self.state = state;
        Ok(())
    }

    fn sleep(&mut self) -> anyhow::Result<()> {
        // TODO(schuffelen): Flush the disk after lifting flush() from AsyncDisk to DiskFile
        Ok(())
    }

    fn wake(&mut self) -> anyhow::Result<()> {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use base::FileReadWriteAtVolatile;
    use tempfile::tempfile;

    use super::*;

    const IMAGE_SIZE: usize = 4 * (1 << 20); // 4M
    const BLOCK_SIZE: u32 = 4 * (1 << 10); // 4K

    fn empty_image() -> Box<dyn DiskFile> {
        let f = Box::new(tempfile().unwrap());
        f.write_all_at_volatile(VolatileSlice::new(&mut [0xff].repeat(IMAGE_SIZE)), 0)
            .unwrap();
        f
    }

    fn new(f: Box<dyn DiskFile>) -> Pflash {
        Pflash::new(f, BLOCK_SIZE).unwrap()
    }

    fn off(offset: u64) -> BusAccessInfo {
        BusAccessInfo {
            offset,
            address: 0,
            id: 0,
        }
    }

    #[test]
    fn read() {
        let f = empty_image();
        let mut want = [0xde, 0xad, 0xbe, 0xef];
        let offset = 0x1000;
        f.write_all_at_volatile(VolatileSlice::new(&mut want), offset)
            .unwrap();

        let mut pflash = new(f);
        let mut got = [0u8; 4];
        pflash.read(off(offset), &mut got[..]);
        assert_eq!(want, got);
    }

    #[test]
    fn write() {
        let f = empty_image();
        let want = [0xdeu8];
        let offset = 0x1000;

        let mut pflash = new(f);
        pflash.write(off(offset), &[COMMAND_WRITE_BYTE]);
        pflash.write(off(offset), &want);

        // Make sure the data reads back correctly over the bus...
        pflash.write(off(0), &[COMMAND_READ_ARRAY]);
        let mut got = [0u8; 1];
        pflash.read(off(offset), &mut got);
        assert_eq!(want, got);

        // And from the backing file itself...
        pflash
            .image
            .read_exact_at_volatile(VolatileSlice::new(&mut got), offset)
            .unwrap();
        assert_eq!(want, got);

        // And when we recreate the device.
        let mut pflash = new(pflash.image);
        pflash.read(off(offset), &mut got);
        assert_eq!(want, got);

        // Finally make sure our status is ready.
        let mut got = [0u8; 4];
        pflash.write(off(offset), &[COMMAND_READ_STATUS]);
        pflash.read(off(offset), &mut got);
        let want = [STATUS_READY, 0, STATUS_READY, 0];
        assert_eq!(want, got);
    }

    #[test]
    fn write_word_with_dual_lane_command() {
        let f = empty_image();
        let want = [0xde, 0xad, 0xbe, 0xef];
        let offset = 0x1000;

        let mut pflash = new(f);
        pflash.write(
            off(offset),
            &[COMMAND_WRITE_BYTE, 0, COMMAND_WRITE_BYTE, 0],
        );
        pflash.write(off(offset), &want);

        pflash.write(off(0), &[COMMAND_READ_ARRAY, 0, COMMAND_READ_ARRAY, 0]);
        let mut got = [0u8; 4];
        pflash.read(off(offset), &mut got);
        assert_eq!(want, got);
    }

    #[test]
    fn buffered_program() {
        let f = empty_image();
        let first = [0xde, 0xad, 0xbe, 0xef];
        let second = [0x12, 0x34, 0x56, 0x78];
        let offset = 0x1000;

        let mut pflash = new(f);
        pflash.write(
            off(offset),
            &[COMMAND_BUFFERED_PROGRAM, 0, COMMAND_BUFFERED_PROGRAM, 0],
        );
        let mut status = [0u8; 4];
        pflash.read(off(offset), &mut status);
        assert_eq!(status, [STATUS_READY, 0, STATUS_READY, 0]);

        pflash.write(
            off(offset),
            &[COMMAND_BUFFERED_PROGRAM, 0, COMMAND_BUFFERED_PROGRAM, 0],
        );
        pflash.read(off(offset), &mut status);
        assert_eq!(status, [STATUS_READY, 0, STATUS_READY, 0]);

        pflash.write(off(offset), &[1, 0, 1, 0]);
        pflash.write(off(offset), &first);
        pflash.write(off(offset + 4), &second);
        pflash.write(
            off(0),
            &[
                COMMAND_BLOCK_ERASE_CONFIRM,
                0,
                COMMAND_BLOCK_ERASE_CONFIRM,
                0,
            ],
        );

        pflash.write(off(0), &[COMMAND_READ_ARRAY, 0, COMMAND_READ_ARRAY, 0]);
        let mut got = [0u8; 8];
        pflash.read(off(offset), &mut got);
        assert_eq!(got, [first, second].concat().as_slice());
    }

    #[test]
    fn read_device_lock_status() {
        let mut pflash = new(empty_image());
        pflash.write(
            off(8),
            &[COMMAND_READ_DEVICE_ID, 0, COMMAND_READ_DEVICE_ID, 0],
        );

        let mut status = [0xff; 4];
        pflash.read(off(8), &mut status);
        assert_eq!(status, [0; 4]);
    }

    #[test]
    fn clear_status_preserves_ready_bit() {
        let mut pflash = new(empty_image());
        pflash.status = 0xff;
        pflash.write(off(0), &[COMMAND_CLEAR_STATUS]);
        pflash.write(off(0), &[COMMAND_READ_STATUS]);

        let mut status = [0; 4];
        pflash.read(off(0), &mut status);
        assert_eq!(status, [STATUS_READY, 0, STATUS_READY, 0]);
    }

    #[test]
    fn erase() {
        let f = empty_image();
        let mut data = [0xde, 0xad, 0xbe, 0xef];
        let offset = 0x1000;
        f.write_all_at_volatile(VolatileSlice::new(&mut data), offset)
            .unwrap();
        f.write_all_at_volatile(VolatileSlice::new(&mut data), offset * 2)
            .unwrap();

        let mut pflash = new(f);
        pflash.write(off(offset), &[COMMAND_BLOCK_ERASE]);
        pflash.write(off(offset), &[COMMAND_BLOCK_ERASE_CONFIRM]);

        pflash.write(off(0), &[COMMAND_READ_ARRAY]);
        let mut got = [0u8; 4];
        pflash.read(off(offset), &mut got);
        let want = [0xffu8; 4];
        assert_eq!(want, got);

        let want = data;
        pflash.read(off(offset * 2), &mut got);
        assert_eq!(want, got);

        // Make sure our status is ready.
        pflash.write(off(offset), &[COMMAND_READ_STATUS]);
        pflash.read(off(offset), &mut got);
        let want = [STATUS_READY; 4];
        assert_eq!(want, got);
    }

    #[test]
    fn status() {
        let f = empty_image();
        let mut data = [0xde, 0xad, 0xbe, 0xff];
        let offset = 0x0;
        f.write_all_at_volatile(VolatileSlice::new(&mut data), offset)
            .unwrap();

        let mut pflash = new(f);

        // Make sure we start off in the "ready" status.
        pflash.write(off(offset), &[COMMAND_READ_STATUS]);
        let mut got = [0u8; 4];
        pflash.read(off(offset), &mut got);
        let want = [STATUS_READY; 4];
        assert_eq!(want, got);

        // Make sure we can clear the status properly.
        pflash.write(off(offset), &[COMMAND_CLEAR_STATUS]);
        pflash.write(off(offset), &[COMMAND_READ_STATUS]);
        pflash.read(off(offset), &mut got);
        let want = [0; 4];
        assert_eq!(want, got);

        // We implicitly jump back into READ_ARRAY mode after reading the,
        // status but for OVMF's probe we require that this doesn't actually
        // affect the cleared status.
        pflash.read(off(offset), &mut got);
        pflash.write(off(offset), &[COMMAND_READ_STATUS]);
        pflash.read(off(offset), &mut got);
        let want = [0; 4];
        assert_eq!(want, got);
    }

    #[test]
    fn overwrite() {
        let f = empty_image();
        let data = [0];
        let offset = off((16 * IMAGE_SIZE).try_into().unwrap());

        // Ensure a write past the pflash device doesn't grow the backing file.
        let mut pflash = new(f);
        let old_size = pflash.image.get_len().unwrap();
        assert_eq!(old_size, IMAGE_SIZE as u64);

        pflash.write(offset, &[COMMAND_WRITE_BYTE]);
        pflash.write(offset, &data);

        let new_size = pflash.image.get_len().unwrap();
        assert_eq!(new_size, IMAGE_SIZE as u64);
    }
}
