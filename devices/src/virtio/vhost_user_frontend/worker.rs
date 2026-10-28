// Copyright 2021 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use std::sync::atomic::fence;
use std::sync::atomic::Ordering;
use std::sync::Arc;

use anyhow::bail;
use anyhow::Context;
use base::info;
use base::warn;
#[cfg(windows)]
use base::CloseNotifier;
use base::Event;
use base::EventToken;
use base::EventType;
use base::ReadNotifier;
use base::WaitContext;
use sync::Mutex;
use vm_memory::GuestAddress;
use vm_memory::GuestMemory;
use vmm_vhost::BackendClient;
use vmm_vhost::Error as VhostError;

use crate::virtio::vhost_user_frontend::handler::BackendReqHandler;
use crate::virtio::Interrupt;
use crate::virtio::VIRTIO_MSI_NO_VECTOR;

/// VIRTQ_AVAIL_F_NO_INTERRUPT: the driver set this in a queue's avail-ring flags to ask the device
/// not to raise an interrupt for that queue's completions (it polls instead).
const VIRTQ_AVAIL_F_NO_INTERRUPT: u16 = 0x1;

pub struct Worker {
    pub kill_evt: Event,
    pub non_msix_evt: Event,
    pub backend_req_handler: Option<BackendReqHandler>,
    pub backend_client: Arc<Mutex<BackendClient>>,
    /// Guest memory and each active queue's avail-ring address, so the shared non-MSI-X interrupt
    /// can honour VIRTQ_AVAIL_F_NO_INTERRUPT. Empty / `event_idx` true means always signal.
    pub mem: GuestMemory,
    pub avail_rings: Vec<GuestAddress>,
    pub event_idx: bool,
}

impl Worker {
    /// Whether the shared non-MSI-X interrupt should be asserted now. On virtio-mmio the backend
    /// kicks one event for all queues and does not look at the suppression flags, so we check them
    /// here as an in-process device would: signal unless every active queue has asked for no
    /// interrupt. With EVENT_IDX the decision needs the used index the backend owns, so signal.
    fn should_signal(&self) -> bool {
        if self.event_idx || self.avail_rings.is_empty() {
            return true;
        }
        fence(Ordering::SeqCst);
        for &avail_ring in &self.avail_rings {
            match self.mem.read_obj_from_addr_volatile::<u16>(avail_ring) {
                Ok(flags) if flags & VIRTQ_AVAIL_F_NO_INTERRUPT != 0 => {}
                // A queue that wants interrupts, or a read we could not make: signal to be safe.
                _ => return true,
            }
        }
        false
    }
}

impl Worker {
    pub fn run(&mut self, interrupt: Interrupt) -> anyhow::Result<()> {
        #[derive(EventToken)]
        enum Token {
            Kill,
            NonMsixEvt,
            ReqHandlerRead,
            #[cfg(target_os = "windows")]
            ReqHandlerClose,
            // monitor whether backend_client_fd is broken
            BackendCloseNotify,
        }
        let wait_ctx = WaitContext::build_with(&[
            (&self.non_msix_evt, Token::NonMsixEvt),
            (&self.kill_evt, Token::Kill),
        ])
        .context("failed to build WaitContext")?;

        if let Some(backend_req_handler) = self.backend_req_handler.as_mut() {
            wait_ctx
                .add(
                    backend_req_handler.get_read_notifier(),
                    Token::ReqHandlerRead,
                )
                .context("failed to add backend req handler to WaitContext")?;

            #[cfg(target_os = "windows")]
            wait_ctx
                .add(
                    backend_req_handler.get_close_notifier(),
                    Token::ReqHandlerClose,
                )
                .context("failed to add backend req handler close notifier to WaitContext")?;
        }

        #[cfg(any(target_os = "android", target_os = "linux"))]
        wait_ctx
            .add_for_event(
                self.backend_client.lock().get_read_notifier(),
                EventType::None,
                Token::BackendCloseNotify,
            )
            .context("failed to add backend client close notifier to WaitContext")?;
        #[cfg(target_os = "windows")]
        wait_ctx
            .add(
                self.backend_client.lock().get_close_notifier(),
                Token::BackendCloseNotify,
            )
            .context("failed to add backend client close notifier to WaitContext")?;

        'wait: loop {
            let events = wait_ctx.wait().context("WaitContext::wait() failed")?;
            for event in events {
                match event.token {
                    Token::Kill => {
                        break 'wait;
                    }
                    Token::NonMsixEvt => {
                        // The vhost-user protocol allows the backend to signal events, but for
                        // non-MSI-X devices, a device must also update the interrupt status mask.
                        // `non_msix_evt` proxies events from the vhost-user backend to update the
                        // status mask.
                        let _ = self.non_msix_evt.wait();

                        // The parameter vector of signal_used_queue is used only when msix is
                        // enabled. Suppress the line when every queue set VIRTQ_AVAIL_F_NO_INTERRUPT
                        // (a polling driver), so an unhandled level IRQ does not storm the guest.
                        if self.should_signal() {
                            interrupt.signal_used_queue(VIRTIO_MSI_NO_VECTOR);
                        }
                    }
                    Token::ReqHandlerRead => {
                        let Some(backend_req_handler) = self.backend_req_handler.as_mut() else {
                            continue;
                        };

                        match backend_req_handler.handle_request() {
                            Ok(_) => (),
                            Err(VhostError::ClientExit) | Err(VhostError::Disconnect) => {
                                info!("backend req handler connection closed");
                                // Stop monitoring `backend_req_handler` as the client closed
                                // the connection.
                                let _ = wait_ctx.delete(backend_req_handler.get_read_notifier());
                                #[cfg(target_os = "windows")]
                                let _ = wait_ctx.delete(backend_req_handler.get_close_notifier());
                                self.backend_req_handler = None;
                            }
                            Err(e) => return Err(e).context("failed to handle vhost-user request"),
                        }
                    }
                    #[cfg(target_os = "windows")]
                    Token::ReqHandlerClose => {
                        let Some(backend_req_handler) = self.backend_req_handler.as_mut() else {
                            continue;
                        };

                        info!("backend req handler connection closed");
                        let _ = wait_ctx.delete(backend_req_handler.get_read_notifier());
                        let _ = wait_ctx.delete(backend_req_handler.get_close_notifier());
                        self.backend_req_handler = None;
                    }
                    Token::BackendCloseNotify => {
                        // For linux domain socket, the close notifier fd is same with read/write
                        // notifier We need check whether the event is caused by socket broken.
                        #[cfg(any(target_os = "android", target_os = "linux"))]
                        if !event.is_hungup {
                            warn!("event besides hungup should not be notified");
                            continue;
                        }
                        bail!("Backend device disconnected early");
                    }
                }
            }
        }

        Ok(())
    }
}
