// Copyright 2026 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use std::sync::Arc;

use anyhow::Context;
use base::error;
use base::EventType;
use sync::Mutex;

use super::interrupter::Interrupter;
use crate::utils::EventHandler;
use crate::utils::EventLoop;

/// Delivers the interrupt that a moderation window held back once the window ends.
///
/// The interrupter arms its timer when an event lands inside the window; this handler runs on
/// the xHCI event loop when that timer fires and asks the interrupter to check again.
pub struct IntrModerationHandler {
    interrupter: Arc<Mutex<Interrupter>>,
}

impl IntrModerationHandler {
    /// Register the interrupter's moderation timer on the event loop.
    pub fn start(
        event_loop: &EventLoop,
        interrupter: Arc<Mutex<Interrupter>>,
    ) -> Option<Arc<IntrModerationHandler>> {
        let handler = Arc::new(IntrModerationHandler { interrupter });
        let tmp_handler: Arc<dyn EventHandler> = handler.clone();
        let added = {
            let interrupter = handler.interrupter.lock();
            event_loop.add_event(
                interrupter.moderation_timer(),
                EventType::Read,
                Arc::downgrade(&tmp_handler),
            )
        };
        if let Err(e) = added {
            error!("cannot add intr moderation handler to event loop: {}", e);
            return None;
        }
        Some(handler)
    }
}

impl EventHandler for IntrModerationHandler {
    fn on_event(&self) -> anyhow::Result<()> {
        self.interrupter
            .lock()
            .on_moderation_timer()
            .context("cannot deliver moderated interrupt")
    }
}
