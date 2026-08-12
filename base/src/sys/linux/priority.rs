// Copyright 2018 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use std::mem::MaybeUninit;

use super::errno_result;
use super::Result;

/// Enables real time thread priorities in the current thread up to `limit`.
pub fn set_rt_prio_limit(limit: u64) -> Result<()> {
    let rt_limit_arg = libc::rlimit64 {
        rlim_cur: limit,
        rlim_max: limit,
    };
    // SAFETY:
    // Safe because the kernel doesn't modify memory that is accessible to the process here.
    let res = unsafe { libc::setrlimit64(libc::RLIMIT_RTPRIO, &rt_limit_arg) };

    if res != 0 {
        errno_result()
    } else {
        Ok(())
    }
}

/// Sets the current thread to be scheduled using the FIFO real time class with `priority`.
///
/// Unlike [`set_rt_round_robin`], a FIFO thread is never preempted by an equal-priority peer: it
/// runs until it blocks. That is what a latency-critical, block-on-ioctl worker wants -- it removes
/// the round-robin timeslice from the wakeup-to-work path.
///
/// The caller needs either `CAP_SYS_NICE` or an `RLIMIT_RTPRIO` ceiling at least as high as
/// `priority` (see [`set_rt_prio_limit`]).
pub fn set_rt_fifo(priority: i32) -> Result<()> {
    // SAFETY:
    // Safe because sched_param only contains primitive types for which zero
    // initialization is valid.
    let mut sched_param: libc::sched_param = unsafe { MaybeUninit::zeroed().assume_init() };
    sched_param.sched_priority = priority;

    let res =
        // SAFETY:
        // Safe because the kernel doesn't modify memory that is accessible to the process here.
        unsafe { libc::pthread_setschedparam(libc::pthread_self(), libc::SCHED_FIFO, &sched_param) };

    if res != 0 {
        // pthread_setschedparam returns the error number rather than setting errno.
        return Err(super::Error::new(res));
    }
    Ok(())
}

/// Sets the current thread to be scheduled using the round robin real time class with `priority`.
pub fn set_rt_round_robin(priority: i32) -> Result<()> {
    // SAFETY:
    // Safe because sched_param only contains primitive types for which zero
    // initialization is valid.
    let mut sched_param: libc::sched_param = unsafe { MaybeUninit::zeroed().assume_init() };
    sched_param.sched_priority = priority;

    let res =
        // SAFETY:
        // Safe because the kernel doesn't modify memory that is accessible to the process here.
        unsafe { libc::pthread_setschedparam(libc::pthread_self(), libc::SCHED_RR, &sched_param) };

    if res != 0 {
        errno_result()
    } else {
        Ok(())
    }
}
