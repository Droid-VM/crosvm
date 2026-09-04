// Copyright 2024 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! Waking a virtio-media worker that is blocked outside its wait loop.
//!
//! The worker thread ([`super::Worker::run`]) watches its kill event, so a `reset` or a
//! vhost-user `stop_queue` ends it promptly -- as long as it is *in* that loop. It is not while
//! it is inside a device call, and one device call blocks by design: sending an event to the
//! guest waits for the guest to supply an event-queue descriptor ([`super::EventQueue`]). A guest
//! that has none left (it crashed, it is being reset, its driver was unloaded mid-stream) would
//! never provide one, so the blocked worker would never see the kill event, `WorkerThread::stop`
//! would never join, and the VMM's `get_vring_base` -- or the in-VMM `reset` -- would hang with
//! it (`logs/vpu_wp/M3.md` §9 item 2).
//!
//! The fix is to wait on the kill event at the same time. The kill event belongs to
//! `WorkerThread::start`, which only hands it to the thread body, whereas the `EventQueue` is
//! built before the thread starts; [`KillSignal`] is the slot that carries it across, armed by
//! [`super::start_worker`] on the worker thread before the device is built (and therefore before
//! any event can be sent).
//!
//! The kill event is deliberately **not** consumed here: the wait contexts are level-triggered
//! `epoll` sets, so leaving the eventfd signaled is what lets [`super::Worker::run`] see its own
//! `Token::Kill` and return once the device call unwinds.

use std::sync::Arc;
use std::sync::OnceLock;

use anyhow::Context;
use base::Event;
use base::EventToken;
use base::WaitContext;

/// Why a [`KillSignal::wait_or_kill`] returned.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Wakeup {
    /// The event that was waited for is signaled (and was consumed).
    Ready,
    /// The worker has been told to stop; what was waited for may never happen.
    Killed,
}

#[derive(EventToken)]
enum Token {
    /// The event the caller is waiting for.
    Ready,
    /// The worker's kill event.
    Kill,
}

/// A handle on the worker thread's kill event, shared with whoever may block outside the
/// worker's own wait loop.
///
/// Cloning shares the slot. It is filled once, by the worker thread; a `KillSignal` that was
/// never armed simply waits forever on the event it was given, which is the behaviour that
/// preceded it.
#[derive(Clone, Default)]
pub struct KillSignal(Arc<OnceLock<Event>>);

impl KillSignal {
    pub fn new() -> Self {
        Self::default()
    }

    /// Put the worker's kill event in the slot. Called once, on the worker thread, before the
    /// device exists; a second call keeps the first event and is ignored.
    pub fn arm(&self, kill_evt: &Event) -> anyhow::Result<()> {
        let kill_evt = kill_evt
            .try_clone()
            .context("cannot clone the worker's kill event")?;
        let _ = self.0.set(kill_evt);
        Ok(())
    }

    /// Whether the slot carries an event, i.e. whether a wait can be interrupted at all.
    pub fn is_armed(&self) -> bool {
        self.0.get().is_some()
    }

    /// Block until `event` is signaled -- consuming it, as [`Event::wait`] does -- or until the
    /// worker is killed, whichever comes first.
    ///
    /// A kill wins a tie: when both are signaled the caller is told to give up, because the
    /// descriptor it would go on to use is about to be handed back to the frontend.
    pub fn wait_or_kill(&self, event: &Event) -> anyhow::Result<Wakeup> {
        let Some(kill_evt) = self.0.get() else {
            event.wait().context("cannot wait for an event")?;
            return Ok(Wakeup::Ready);
        };

        // Built per wait rather than kept: blocking here at all means the guest has starved the
        // device of descriptors, so this is neither a hot path nor one worth caching a `dup`ed
        // epoll set for.
        let wait_ctx: WaitContext<Token> =
            WaitContext::build_with(&[(event, Token::Ready), (kill_evt, Token::Kill)])
                .context("cannot create the wait context for an event")?;
        let events = wait_ctx.wait().context("cannot wait for an event")?;
        if events.iter().any(|e| matches!(e.token, Token::Kill)) {
            return Ok(Wakeup::Killed);
        }
        event.wait().context("cannot wait for an event")?;
        Ok(Wakeup::Ready)
    }
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use base::EventWaitResult;

    use super::*;

    /// Unarmed, a wait is the plain blocking wait it always was.
    #[test]
    fn an_unarmed_signal_waits_for_the_event_itself() {
        let signal = KillSignal::new();
        assert!(!signal.is_armed());
        let event = Event::new().unwrap();
        event.signal().unwrap();
        assert_eq!(signal.wait_or_kill(&event).unwrap(), Wakeup::Ready);
        // It was consumed: nothing is left for a second wait, which is why the caller loops.
        assert_eq!(
            event.wait_timeout(Duration::ZERO).unwrap(),
            EventWaitResult::TimedOut
        );
    }

    /// Armed and idle, the event still comes through -- and only the event is consumed.
    #[test]
    fn an_armed_signal_still_passes_the_event_on() {
        let kill_evt = Event::new().unwrap();
        let signal = KillSignal::new();
        signal.arm(&kill_evt).unwrap();
        assert!(signal.is_armed());
        let event = Event::new().unwrap();
        event.signal().unwrap();
        assert_eq!(signal.wait_or_kill(&event).unwrap(), Wakeup::Ready);
    }

    /// The kill event ends a wait that nothing else would ever end, and stays signaled so the
    /// worker's own wait context sees it too.
    #[test]
    fn a_kill_ends_a_wait_nothing_else_would() {
        let kill_evt = Event::new().unwrap();
        let signal = KillSignal::new();
        signal.arm(&kill_evt).unwrap();
        // Nothing will ever signal this one: this is the guest that supplies no descriptor.
        let event = Event::new().unwrap();
        kill_evt.signal().unwrap();
        assert_eq!(signal.wait_or_kill(&event).unwrap(), Wakeup::Killed);
        // Still signaled, twice over: `Worker::run` must be able to see it.
        assert_eq!(signal.wait_or_kill(&event).unwrap(), Wakeup::Killed);
        assert_eq!(
            kill_evt.wait_timeout(Duration::ZERO).unwrap(),
            EventWaitResult::Signaled
        );
    }

    /// A kill that arrives while the wait is already blocked wakes it up.
    #[test]
    fn a_kill_wakes_a_wait_that_is_already_blocked() {
        let kill_evt = Event::new().unwrap();
        let signal = KillSignal::new();
        signal.arm(&kill_evt).unwrap();
        let event = Event::new().unwrap();
        let killer = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(50));
            kill_evt.signal().unwrap();
        });
        assert_eq!(signal.wait_or_kill(&event).unwrap(), Wakeup::Killed);
        killer.join().unwrap();
    }

    /// When both are signaled the kill wins: the descriptor is about to go back to the frontend.
    #[test]
    fn a_kill_wins_a_tie() {
        let kill_evt = Event::new().unwrap();
        let signal = KillSignal::new();
        signal.arm(&kill_evt).unwrap();
        let event = Event::new().unwrap();
        event.signal().unwrap();
        kill_evt.signal().unwrap();
        assert_eq!(signal.wait_or_kill(&event).unwrap(), Wakeup::Killed);
    }
}
