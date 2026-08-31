// Copyright 2020 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use std::time::Duration;

use base::Result as SysResult;
use base::Timer;
use base::TimerTrait;

use crate::AsyncResult;
use crate::Error;
use crate::Executor;
use crate::IntoAsync;
use crate::IoSource;

/// An async version of base::Timer.
pub struct TimerAsync<T: TimerTrait + IntoAsync> {
    pub(crate) io_source: IoSource<T>,
}

impl<T: TimerTrait + IntoAsync> TimerAsync<T> {
    pub fn new(timer: T, ex: &Executor) -> AsyncResult<TimerAsync<T>> {
        ex.async_from(timer)
            .map(|io_source| TimerAsync { io_source })
    }

    /// Gets the next value from the timer.
    ///
    /// NOTE: on Windows, this may return/wake early. See `base::Timer` docs
    /// for details.
    pub async fn wait(&self) -> AsyncResult<()> {
        self.wait_sys().await
    }

    /// Sets the timer to expire after `dur`. Cancels any existing timer.
    pub fn reset_oneshot(&mut self, dur: Duration) -> SysResult<()> {
        self.io_source.as_source_mut().reset_oneshot(dur)
    }

    /// Sets the timer to expire repeatedly at intervals of `dur`. Cancels any existing timer.
    pub fn reset_repeating(&mut self, dur: Duration) -> SysResult<()> {
        self.io_source.as_source_mut().reset_repeating(dur)
    }

    /// Disarms the timer.
    pub fn clear(&mut self) -> SysResult<()> {
        self.io_source.as_source_mut().clear()
    }
}

impl TimerAsync<Timer> {
    /// Async sleep for the given duration.
    ///
    /// NOTE: on Windows, this sleep may wake early. See `base::Timer` docs
    /// for details.
    pub async fn sleep(ex: &Executor, dur: Duration) -> std::result::Result<(), Error> {
        // A zero duration means "do not wait", but an itimerspec of zero *disarms* a timerfd, so
        // arming one with it produces a descriptor that never becomes readable and a sleep that
        // never returns. Callers pacing against a wall-clock schedule reach zero the moment they
        // fall behind it, which on a phone is a matter of time rather than a possibility, so the
        // cost of getting this wrong is the whole task wedged. Ask for the shortest armed timer
        // instead of the disarmed one; the executor still gets its poll.
        let dur = if dur.is_zero() {
            Duration::from_nanos(1)
        } else {
            dur
        };
        let mut tfd = Timer::new().map_err(Error::Timer)?;
        tfd.reset_oneshot(dur).map_err(Error::Timer)?;
        let t = TimerAsync::new(tfd, ex).map_err(Error::TimerAsync)?;
        t.wait().await.map_err(Error::TimerAsync)?;
        Ok(())
    }
}

impl IntoAsync for Timer {}
