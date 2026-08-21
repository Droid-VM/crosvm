// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright DroidVM contributors
// Additional permissions apply; see ADDITIONAL-PERMISSIONS in the repository root.

//! A second source of frames for the GPU device's display: the VMM's own simplefb bridge.
//!
//! A VM can have two things that produce a picture -- the guest's virtio-gpu driver, and the
//! linear framebuffer the firmware set up, which crosvm polls (see `simplefb_display.rs`). Which
//! one carries the picture is the guest's decision, not ours, and it changes during a single
//! boot: the firmware draws through virtio-gpu, then an OS with no virtio-gpu driver (Windows,
//! whose Basic Display Driver only knows the UEFI framebuffer) writes the linear one for the rest
//! of its life, while an OS that does have the driver (Linux, once kwin modesets) uses virtio-gpu
//! and leaves the linear framebuffer frozen at the boot logo.
//!
//! There is exactly one place to put a picture -- the app hands crosvm a single Surface -- so
//! there has to be exactly one writer. That writer is the GPU device's display: the bridge hands
//! its frames here instead of opening a display of its own. Two displays under one Android
//! service name is what this replaces, and it was silent: servicemanager keeps one registration,
//! the app's Surface went to whichever producer won the race, and the loser painted into nothing
//! for the rest of the VM's life.
//!
//! Who wins is decided by the guest's own scanout state rather than by which source drew most
//! recently. "Most recent" ping-pongs: an idle desktop presents about once a second, so any
//! timeout short enough to notice Windows would also let the bridge cut in between two of its
//! frames -- with different geometry, which means a surface reconfigure and a visible jump each
//! time. A scanout that the guest has set is a state, so it holds until the guest clears it or
//! stops presenting entirely; see `VirtioGpu::guest_owns_display`.

use std::sync::atomic::AtomicBool;
use std::sync::atomic::AtomicU64;
use std::sync::atomic::Ordering;
use std::sync::Arc;

use base::Event;
use sync::Mutex;

/// The frame the simplefb bridge wants shown, and the arbitration state around it.
///
/// Both sides hold an `Arc`: the bridge thread submits frames, the GPU worker consumes them.
pub struct ExternalScanout {
    width: u32,
    height: u32,
    stride: u32,
    /// The most recent frame. Overwritten in place -- a display only ever wants the newest one.
    frame: Mutex<Vec<u8>>,
    /// Bumped on every submit; the worker repaints only when it moves.
    seq: AtomicU64,
    /// Last `seq` the worker painted.
    painted: AtomicU64,
    /// Set by the worker: the guest is driving the display itself, so submissions are pointless.
    /// The bridge reads it to skip the copy entirely (it still polls, cheaply, so a handover in
    /// the other direction is noticed immediately).
    guest_owns: AtomicBool,
    /// Wakes the GPU worker when a frame is submitted.
    event: Event,
}

impl ExternalScanout {
    pub fn new(width: u32, height: u32, stride: u32) -> base::Result<Arc<ExternalScanout>> {
        Ok(Arc::new(ExternalScanout {
            width,
            height,
            stride,
            frame: Mutex::new(Vec::new()),
            seq: AtomicU64::new(0),
            painted: AtomicU64::new(0),
            guest_owns: AtomicBool::new(false),
            event: Event::new()?,
        }))
    }

    pub fn width(&self) -> u32 {
        self.width
    }

    pub fn height(&self) -> u32 {
        self.height
    }

    pub fn stride(&self) -> u32 {
        self.stride
    }

    /// The descriptor the GPU worker waits on.
    pub fn event(&self) -> &Event {
        &self.event
    }

    /// Whether the guest is currently driving the display. Producer-side hint only: the worker
    /// checks again before painting, so a stale `false` costs one wasted copy, never a wrong
    /// picture.
    pub fn guest_owns(&self) -> bool {
        self.guest_owns.load(Ordering::Relaxed)
    }

    pub(crate) fn set_guest_owns(&self, owns: bool) {
        self.guest_owns.store(owns, Ordering::Relaxed);
    }

    /// Offers a frame. While the guest owns the display the frame is dropped -- but the worker is
    /// still woken, so it re-evaluates ownership.
    ///
    /// Waking it matters: `guest_owns` is only ever recomputed on the worker's side of this event.
    /// Returning here without signalling would make the first observation of guest ownership
    /// permanent -- no submit, no event, no re-evaluation -- and the bridge could never take the
    /// display back when the guest let go of it. That handover is the case this exists for: the
    /// firmware paints through virtio-gpu, `reset()` at OS handover unbinds the scanout, and an OS
    /// with no virtio-gpu driver never binds one again.
    pub fn submit(&self, data: &[u8]) {
        if self.guest_owns() {
            let _ = self.event.signal();
            return;
        }
        {
            let mut frame = self.frame.lock();
            frame.clear();
            frame.extend_from_slice(data);
        }
        self.seq.fetch_add(1, Ordering::Release);
        let _ = self.event.signal();
    }

    /// Wakes the worker without offering anything.
    ///
    /// The producer needs this because ownership can also expire on a clock: a guest that bound a
    /// scanout and then stopped presenting loses the display after a grace period, and that is
    /// decided on the worker's side. While the guest owns it the producer has nothing to submit,
    /// so without an occasional poke there would be no event, no re-evaluation, and the display
    /// would stay with a guest that had stopped drawing.
    pub fn poke(&self) {
        let _ = self.event.signal();
    }

    /// Hands the newest unpainted frame to `f`. Returns false when there is nothing new.
    pub(crate) fn take_frame<F: FnOnce(&[u8])>(&self, f: F) -> bool {
        let seq = self.seq.load(Ordering::Acquire);
        if seq == self.painted.load(Ordering::Relaxed) {
            return false;
        }
        let frame = self.frame.lock();
        if frame.is_empty() {
            return false;
        }
        f(&frame);
        self.painted.store(seq, Ordering::Relaxed);
        true
    }
}
