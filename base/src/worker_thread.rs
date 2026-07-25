// Copyright 2023 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! Worker thread abstraction

use std::panic;
use std::thread;
use std::thread::JoinHandle;
use std::thread::Thread;

use crate::Error;
use crate::Event;

/// Wrapper object for creating a worker thread that can be stopped by signaling an event.
/// Per-thread alternate signal stack, so the fatal-signal handler can run even when this thread's
/// own stack is exhausted or unusable. Best-effort: a failure just leaves the previous behaviour.
#[cfg(any(target_os = "android", target_os = "linux"))]
fn install_altstack() {
    thread_local! {
        static ALTSTACK: std::cell::RefCell<Option<Box<[u8]>>> = const { std::cell::RefCell::new(None) };
    }
    const ALTSTACK_SIZE: usize = 128 * 1024;
    ALTSTACK.with(|slot| {
        let mut slot = slot.borrow_mut();
        if slot.is_some() {
            return;
        }
        let mut stack = vec![0u8; ALTSTACK_SIZE].into_boxed_slice();
        // SAFETY: `stack` outlives the sigaltstack registration -- it is parked in a thread-local
        // that is dropped only when the thread exits, after which no handler can run on it.
        unsafe {
            let ss = libc::stack_t {
                ss_sp: stack.as_mut_ptr() as *mut libc::c_void,
                ss_flags: 0,
                ss_size: ALTSTACK_SIZE,
            };
            libc::sigaltstack(&ss, std::ptr::null_mut());
        }
        *slot = Some(stack);
    });
}

#[cfg(not(any(target_os = "android", target_os = "linux")))]
fn install_altstack() {}

pub struct WorkerThread<T: Send + 'static> {
    worker: Option<(Event, JoinHandle<T>)>,
}

impl<T: Send + 'static> WorkerThread<T> {
    /// Starts a worker thread named `thread_name` running the `thread_func` function.
    ///
    /// The `thread_func` implementation must monitor the provided `Event` and return from the
    /// thread when it is signaled.
    ///
    /// Call [`stop()`](Self::stop) to stop the thread.
    pub fn start<F>(thread_name: impl Into<String>, thread_func: F) -> Self
    where
        F: FnOnce(Event) -> T + Send + 'static,
    {
        let stop_event = Event::new().expect("Event::new() failed");
        let thread_stop_event = stop_event.try_clone().expect("Event::try_clone() failed");

        let thread_handle = thread::Builder::new()
            .name(thread_name.into())
            .spawn(move || {
                // DroidVM: give this thread its own alternate signal stack. sigaltstack is
                // per-thread and crosvm only sets one on the main thread, so a fatal fault here --
                // in particular a stack overflow, where the thread's own stack cannot host the
                // handler -- kills the process with no diagnostic at all. With an altstack the
                // crash handler still runs and prints the faulting pc.
                install_altstack();
                thread_func(thread_stop_event)
            })
            .expect("thread spawn failed");

        WorkerThread {
            worker: Some((stop_event, thread_handle)),
        }
    }

    /// Stops the worker thread.
    ///
    /// Returns the value returned by the function running in the thread.
    pub fn stop(mut self) -> T {
        // The only time the internal `Option` should be `None` is in a `drop` after `stop`, so this
        // `expect()` should never fail.
        self.stop_internal().expect("invalid worker state")
    }

    // `stop_internal` accepts a reference so it can be called from `drop`.
    #[doc(hidden)]
    fn stop_internal(&mut self) -> Option<T> {
        self.worker.take().map(|(stop_event, thread_handle)| {
            // There is nothing the caller can do to handle `stop_event.signal()` failure, and we
            // don't want to leave the thread running, so panic in that case.
            stop_event
                .signal()
                .expect("WorkerThread stop event signal failed");

            match thread_handle.join() {
                Ok(v) => v,
                Err(e) => panic::resume_unwind(e),
            }
        })
    }

    /// Signal thread's stop event. Unlike stop, the function doesn't wait
    /// on joining the thread.
    /// The function can be called multiple times.
    /// Calling `stop` or `drop` will internally signal the stop event again
    /// and join the thread.
    pub fn signal(&mut self) -> Result<(), Error> {
        if let Some((event, _)) = &mut self.worker {
            event.signal()
        } else {
            Ok(())
        }
    }

    /// Returns a handle to the running thread.
    pub fn thread(&self) -> &Thread {
        // The only time the internal `Option` should be `None` is in a `drop` after `stop`, so this
        // `unwrap()` should never fail.
        self.worker.as_ref().unwrap().1.thread()
    }
}

impl<T: Send + 'static> Drop for WorkerThread<T> {
    /// Stops the thread if the `WorkerThread` is dropped without calling [`stop()`](Self::stop).
    fn drop(&mut self) {
        let _ = self.stop_internal();
    }
}
