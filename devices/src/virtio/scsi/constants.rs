// Copyright 2023 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#![deny(missing_docs)]
//! This file contains values specified in spec.
//! SPC-3: <https://www.t10.org/cgi-bin/ac.pl?t=f&f=spc3r23.pdf>
//! SAM-5: <https://www.t10.org/cgi-bin/ac.pl?t=f&f=sam5r21.pdf>

// SCSI opcodes
/// Opcode for TEST UNIT READY command.
pub const TEST_UNIT_READY: u8 = 0x00;
/// Opcode for REQUEST SENSE command.
pub const REQUEST_SENSE: u8 = 0x03;
/// Opcode for READ(6) command.
pub const READ_6: u8 = 0x08;
/// Opcode for INQUIRY command.
pub const INQUIRY: u8 = 0x12;
/// Opcode for MODE SELECT(6) command.
pub const MODE_SELECT_6: u8 = 0x15;
/// Opcode for MODE SENSE(6) command.
pub const MODE_SENSE_6: u8 = 0x1a;
/// Opcode for START STOP UNIT command.
pub const START_STOP_UNIT: u8 = 0x1b;
/// Opcode for PREVENT ALLOW MEDIUM REMOVAL command.
pub const PREVENT_ALLOW_MEDIUM_REMOVAL: u8 = 0x1e;
/// Opcode for READ CAPACITY(10) command.
pub const READ_CAPACITY_10: u8 = 0x25;
/// Opcode for READ(10) command.
pub const READ_10: u8 = 0x28;
/// Opcode for WRITE(10) command.
pub const WRITE_10: u8 = 0x2a;
/// Opcode for SYNCHRONIZE CACHE(10) command.
pub const SYNCHRONIZE_CACHE_10: u8 = 0x35;
/// Opcode for WRITE SAME(10) command.
pub const WRITE_SAME_10: u8 = 0x41;
/// Opcode for UNMAP command.
pub const UNMAP: u8 = 0x42;
/// Opcode for READ TOC/PMA/ATIP command.
pub const READ_TOC: u8 = 0x43;
/// Opcode for GET CONFIGURATION command.
pub const GET_CONFIGURATION: u8 = 0x46;
/// Opcode for GET EVENT STATUS NOTIFICATION command.
pub const GET_EVENT_STATUS_NOTIFICATION: u8 = 0x4a;
/// Opcode for MODE SENSE(10) command.
pub const MODE_SENSE_10: u8 = 0x5a;
/// Opcode for WRITE SAME(16) command.
pub const WRITE_SAME_16: u8 = 0x93;
/// Opcode for SERVICE ACTION IN(16) command.
pub const SERVICE_ACTION_IN_16: u8 = 0x9e;
/// Opcode for REPORT LUNS command.
pub const REPORT_LUNS: u8 = 0xa0;
/// Opcode for MAINTENANCE IN command.
pub const MAINTENANCE_IN: u8 = 0xa3;
/// Opcode for READ(12) command.
pub const READ_12: u8 = 0xa8;

// The service actions of MAINTENANCE IN command.
/// REPORT SUPPORTED TASK MANAGEMENT FUNCTIONS
pub const REPORT_SUPPORTED_TASK_MANAGEMENT_FUNCTIONS: u8 = 0x0d;

// The service actions of SERVICE ACTION IN(16) command.
/// READ CAPACITY(16)
pub const READ_CAPACITY_16: u8 = 0x10;

// SAM status code
/// Indicates the completion of the command without error.
pub const GOOD: u8 = 0x00;
/// Indicates that sense data has been delivered in the buffer.
pub const CHECK_CONDITION: u8 = 0x02;

// Device Types
/// Indicates the id of disk type.
pub const TYPE_DISK: u8 = 0x00;
/// Indicates the id of cdrom type.
pub const TYPE_CDROM: u8 = 0x05;

// SENSE KEYS
/// Indicates that there is no specific sense data to be reported.
pub const NO_SENSE: u8 = 0x00;
/// Indicates an error that may have been caused by a flaw in the medium or an error in the
/// recorded data.
pub const MEDIUM_ERROR: u8 = 0x03;
/// Indicates an illegal request.
pub const ILLEGAL_REQUEST: u8 = 0x05;
/// Indicates that a unit attention condition has been established.
pub const UNIT_ATTENTION: u8 = 0x06;
