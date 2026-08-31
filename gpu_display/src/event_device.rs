// Copyright 2019 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use std::collections::VecDeque;
use std::fmt;
use std::io;
use std::io::Read;
use std::io::Write;
use std::iter::ExactSizeIterator;

use base::AsRawDescriptor;
use base::RawDescriptor;
use base::ReadNotifier;
use base::StreamChannel;
use linux_input_sys::virtio_input_event;
use linux_input_sys::InputEventDecoder;
use serde::Deserialize;
use serde::Serialize;
use zerocopy::FromZeros;
use zerocopy::IntoBytes;

const EVENT_SIZE: usize = virtio_input_event::SIZE;
const EVENT_BUFFER_LEN_MAX: usize = 64 * EVENT_SIZE;

#[derive(Copy, Clone, Debug, Deserialize, Eq, Hash, PartialEq, Serialize)]
pub enum EventDeviceKind {
    /// Produces relative mouse motions, wheel, and button clicks while the real mouse is captured.
    Mouse,
    /// Produces absolute pointer motion (qemu usb-tablet equivalent): a 1:1 cursor with hover,
    /// buttons and wheel. Kept distinct from `Mouse` so a relative mouse and an absolute tablet
    /// can be two independent, separately-routed devices at once (the dispatch is by-kind).
    Tablet,
    /// Produces absolute motion and touch events from the display window's events.
    Touchscreen,
    /// Produces key events while the display window has focus.
    Keyboard,
}

/// Encapsulates a virtual event device, such as a mouse or keyboard
#[derive(Deserialize, Serialize)]
pub struct EventDevice {
    kind: EventDeviceKind,
    event_buffer: VecDeque<u8>,
    event_socket: StreamChannel,
}

impl EventDevice {
    pub fn new(kind: EventDeviceKind, mut event_socket: StreamChannel) -> EventDevice {
        let _ = event_socket.set_nonblocking(true);
        EventDevice {
            kind,
            event_buffer: Default::default(),
            event_socket,
        }
    }

    #[inline]
    pub fn mouse(event_socket: StreamChannel) -> EventDevice {
        Self::new(EventDeviceKind::Mouse, event_socket)
    }

    #[inline]
    pub fn tablet(event_socket: StreamChannel) -> EventDevice {
        Self::new(EventDeviceKind::Tablet, event_socket)
    }

    #[inline]
    pub fn touchscreen(event_socket: StreamChannel) -> EventDevice {
        Self::new(EventDeviceKind::Touchscreen, event_socket)
    }

    #[inline]
    pub fn keyboard(event_socket: StreamChannel) -> EventDevice {
        Self::new(EventDeviceKind::Keyboard, event_socket)
    }

    #[inline]
    pub fn kind(&self) -> EventDeviceKind {
        self.kind
    }

    /// Flushes the buffered events that did not fit into the underlying transport, if any.
    ///
    /// Returns `Ok(true)` if, after this function returns, there all the buffer of events is
    /// empty.
    pub fn flush_buffered_events(&mut self) -> io::Result<bool> {
        while !self.event_buffer.is_empty() {
            let written = match self.event_socket.write(self.event_buffer.as_slices().0) {
                Ok(written) => written,
                Err(e) if e.kind() == io::ErrorKind::WouldBlock => return Ok(false),
                Err(e) => return Err(e),
            };
            if written == 0 {
                return Ok(false);
            }
            self.event_buffer.drain(..written);
        }
        Ok(true)
    }

    pub fn is_buffered_events_empty(&self) -> bool {
        self.event_buffer.is_empty()
    }

    /// Determines if there is space in the event buffer for the given number
    /// of events. The buffer is capped at `EVENT_BUFFER_LEN_MAX`.
    #[inline]
    fn can_buffer_events(&self, num_events: usize) -> bool {
        let event_bytes = match EVENT_SIZE.checked_mul(num_events) {
            Some(bytes) => bytes,
            None => return false,
        };
        let free_bytes = EVENT_BUFFER_LEN_MAX.saturating_sub(self.event_buffer.len());

        free_bytes >= event_bytes
    }

    pub fn send_report<E: IntoIterator<Item = virtio_input_event>>(
        &mut self,
        events: E,
    ) -> io::Result<bool>
    where
        E::IntoIter: ExactSizeIterator,
    {
        let it = events.into_iter();

        if !self.can_buffer_events(it.len() + 1) {
            return Ok(false);
        }

        for event in it {
            let bytes = event.as_bytes();
            self.event_buffer.extend(bytes.iter());
        }

        self.event_buffer
            .extend(virtio_input_event::syn().as_bytes().iter());

        self.flush_buffered_events()
    }

    /// Sends the given `event`, returning `Ok(true)` if, after this function returns, there are no
    /// buffered events remaining.
    pub fn send_event_encoded(&mut self, event: virtio_input_event) -> io::Result<bool> {
        if !self.flush_buffered_events()? {
            return Ok(false);
        }

        let bytes = event.as_bytes();
        let written = match self.event_socket.write(bytes) {
            Ok(written) => written,
            Err(e) if e.kind() == io::ErrorKind::WouldBlock => 0,
            Err(e) => return Err(e),
        };

        if written == bytes.len() {
            return Ok(true);
        }

        if self.can_buffer_events(1) {
            self.event_buffer.extend(bytes[written..].iter());
        }

        Ok(false)
    }

    pub fn recv_event_encoded(&self) -> io::Result<virtio_input_event> {
        let mut event = virtio_input_event::new_zeroed();
        (&self.event_socket).read_exact(event.as_mut_bytes())?;
        Ok(event)
    }
}

impl AsRawDescriptor for EventDevice {
    fn as_raw_descriptor(&self) -> RawDescriptor {
        self.event_socket.as_raw_descriptor()
    }
}

impl ReadNotifier for EventDevice {
    fn get_read_notifier(&self) -> &dyn AsRawDescriptor {
        self.event_socket.get_read_notifier()
    }
}

impl fmt::Debug for EventDevice {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Event device ({:?})", self.kind)
    }
}

/// The input devices belonging to one display binding, on their way to that binding's sink.
///
/// A VNC binding gets an absolute pointer and a keyboard of its own, created with the guest device
/// and handed to the sink that injects into them. Both `None` is a view-only binding: no devices
/// were made and the sink drops RFB input.
///
/// They travel as a pair because they are created together, taken together, and named after the
/// same screen -- and because "which of these two `Option<EventDevice>`s went where" is exactly the
/// question a named pair answers for free. It lives here, next to `EventDevice` rather than beside
/// either consumer, because the two consumers (the virtio-gpu display chain and the simplefb
/// bridge) are in different crates and neither is upstream of the other.
#[derive(Default)]
pub struct VncBindingInput {
    /// Absolute pointer, named for the screen this binding serves.
    pub tablet: Option<EventDevice>,
    /// Keyboard, named for the same screen.
    pub keyboard: Option<EventDevice>,
}
