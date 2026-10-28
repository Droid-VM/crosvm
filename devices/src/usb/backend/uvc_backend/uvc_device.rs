// Copyright 2026 The crosvm Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! A virtual USB Video Class camera backend.
//!
//! The device enumerates as a UVC 1.1 MJPEG camera and streams a still JPEG
//! test pattern as fast as the guest asks for it. It is meant to give a Windows
//! guest a driver free webcam without passing through a host camera.

use std::cmp;
use std::sync::Arc;
use std::sync::RwLock;

use base::error;
use base::info;
use base::AsRawDescriptor;
use base::RawDescriptor;
use sync::Mutex;
use usb_util::parse_usbfs_descriptors;
use usb_util::ConfigDescriptorTree;
use usb_util::ControlRequestDataPhaseTransferDirection;
use usb_util::ControlRequestType;
use usb_util::DeviceDescriptorTree;
use usb_util::DeviceSpeed;
use usb_util::EndpointDirection;
use usb_util::EndpointType;
use usb_util::Error as UsbUtilError;
use usb_util::StandardControlRequest;
use usb_util::TransferBuffer;
use usb_util::TransferStatus;
use usb_util::UsbRequestSetup;
use zerocopy::FromBytes;
use zerocopy::IntoBytes;

use crate::usb::backend::device::BackendDevice;
use crate::usb::backend::device::BackendDeviceType;
use crate::usb::backend::device::DeviceState;
use crate::usb::backend::endpoint::ControlEndpointState;
use crate::usb::backend::endpoint::UsbEndpoint;
use crate::usb::backend::error::Error as BackendError;
use crate::usb::backend::error::Result as BackendResult;
use crate::usb::backend::transfer::BackendTransferHandle;
use crate::usb::backend::transfer::BackendTransferType;
use crate::usb::backend::transfer::ControlTransferState;
use crate::usb::backend::uvc_backend::camera;
use crate::usb::backend::uvc_backend::descriptors;
use crate::usb::backend::uvc_backend::transfer::UvcTransfer;
use crate::usb::backend::uvc_backend::transfer::UvcTransferHandle;
use crate::usb::xhci::xhci_backend_device::BackendType;
use crate::usb::xhci::xhci_backend_device::UsbDeviceAddress;
use crate::usb::xhci::xhci_backend_device::XhciBackendDevice;
use crate::utils::AsyncJobQueue;
use crate::utils::EventLoop;

/// Number of header bytes prepended to every video payload transfer: a UVC
/// payload header with a two byte header length and no presentation time.
const PAYLOAD_HEADER_LEN: usize = 2;

/// State of the video stream, shared between queued transfer jobs.
struct StreamState {
    /// Probe/commit structure most recently negotiated with the guest.
    probe: [u8; descriptors::PROBE_LEN],
    /// Whether the guest committed the negotiated parameters, i.e. whether it
    /// expects payload on the bulk endpoint.
    committed: bool,
    /// Alternate setting selected on the video streaming interface.
    alt_setting: u8,
    /// Frame currently being sent.
    frame: Vec<u8>,
    /// Offset of the next payload byte inside `frame`.
    offset: usize,
    /// Frame id bit, toggled for every new frame.
    fid: u8,
    /// Number of payload transfers handled so far, used for logging.
    transfers: u64,
    /// Number of status endpoint transfers handled so far, used for logging.
    status_transfers: u64,
    /// Frames fully handed to the guest, used to measure the delivered rate.
    frames_sent: u64,
    /// Sequence number of the camera frame being sent, used to avoid sending the
    /// same picture again while the capture loop has nothing newer.
    frame_seq: u64,
    /// Whether the static test picture still has to be sent once, so that the
    /// guest has something to show while the camera is being opened.
    test_pattern_pending: bool,
}

impl StreamState {
    fn new() -> StreamState {
        StreamState {
            probe: descriptors::default_probe(),
            committed: false,
            alt_setting: 0,
            frame: descriptors::TEST_PATTERN.to_vec(),
            // Start at the end so that the first transfer starts a new frame.
            offset: descriptors::TEST_PATTERN.len(),
            fid: 0,
            transfers: 0,
            status_transfers: 0,
            frames_sent: 0,
            frame_seq: 0,
            test_pattern_pending: true,
        }
    }

    /// Rewinds the stream to the start of a frame.
    fn restart(&mut self) {
        self.offset = self.frame.len();
        self.fid = 0;
        self.test_pattern_pending = true;
        self.frame_seq = 0;
    }
}

/// Reply of a control request.
enum Reply {
    /// A data stage carrying these bytes, truncated to the requested length.
    Data(Vec<u8>),
    /// A status only reply, for requests whose data stage came from the guest.
    Ack,
    /// The request is not supported, stall the endpoint.
    Stall,
}

/// The virtual USB camera.
pub struct UvcDevice {
    state: Arc<RwLock<DeviceState>>,
    control_transfer_state: Arc<RwLock<ControlTransferState>>,
    transfer_job_queue: Arc<AsyncJobQueue>,
    stream: Arc<Mutex<StreamState>>,
    /// Which of the VM's cameras this device streams.
    camera_slot: usize,
}

impl UvcDevice {
    pub fn new(
        state: DeviceState,
        event_loop: Arc<EventLoop>,
        camera_slot: usize,
        camera_id: String,
    ) -> BackendResult<UvcDevice> {
        let control_transfer_state = ControlTransferState {
            ctl_ep_state: ControlEndpointState::SetupStage,
            control_request_setup: UsbRequestSetup::new(0, 0, 0, 0, 0),
            executed: false,
        };
        let job_queue = AsyncJobQueue::init(&event_loop).map_err(BackendError::StartAsyncJobQueue)?;
        let device = UvcDevice {
            state: Arc::new(RwLock::new(state)),
            control_transfer_state: Arc::new(RwLock::new(control_transfer_state)),
            transfer_job_queue: job_queue,
            stream: Arc::new(Mutex::new(StreamState::new())),
            camera_slot,
        };
        info!(
            "uvc: camera {camera_slot} (host camera {camera_id}) created, \
             config descriptor {} bytes, frame {} bytes",
            descriptors::config_descriptor().len(),
            descriptors::TEST_PATTERN.len()
        );
        camera::spawn_camera(camera_slot, camera_id);
        Ok(device)
    }

    /// Handles a control transfer. The transfer buffer starts with the setup
    /// packet followed by the data stage sent by the guest, and is replaced by
    /// the setup packet followed by the response for the guest.
    fn handle_control(
        transfer: &mut UvcTransfer,
        stream: &Arc<Mutex<StreamState>>,
        camera_slot: usize,
    ) -> BackendResult<()> {
        transfer.actual_length = 0;
        let setup = match &transfer.buffer {
            TransferBuffer::Vector(v) => match UsbRequestSetup::read_from_prefix(v) {
                Ok((setup, _)) => setup,
                Err(_) => return Err(BackendError::MalformedBackendTransfer),
            },
            _ => return Err(BackendError::MalformedBackendTransfer),
        };

        let length = setup.length as usize;
        let is_device_to_host =
            setup.get_direction() == ControlRequestDataPhaseTransferDirection::DeviceToHost;
        // `UsbRequestSetup` is packed, so copy the fields out before formatting them.
        let request_type = setup.request_type;
        let request = setup.request;
        let value = setup.value;
        let index = setup.index;
        info!(
            "uvc: ctl bmRequestType=0x{request_type:02x} bRequest=0x{request:02x} \
             wValue=0x{value:04x} wIndex=0x{index:04x} wLength={length}"
        );

        let reply = match setup.get_type() {
            ControlRequestType::Standard => {
                if !is_device_to_host {
                    Reply::Ack
                } else {
                    match setup.request {
                        x if x == StandardControlRequest::GetDescriptor as u8 => {
                            UvcDevice::handle_get_descriptor(&setup, length, camera_slot)
                        }
                        x if x == StandardControlRequest::GetStatus as u8 => {
                            Reply::Data(vec![0u8; 2])
                        }
                        x if x == StandardControlRequest::GetInterface as u8 => {
                            Reply::Data(vec![stream.lock().alt_setting])
                        }
                        x if x == StandardControlRequest::GetConfiguration as u8 => {
                            Reply::Data(vec![descriptors::CONFIG_VALUE])
                        }
                        _ => Reply::Stall,
                    }
                }
            }
            ControlRequestType::Class => {
                UvcDevice::handle_class_request(
                    transfer,
                    &setup,
                    length,
                    is_device_to_host,
                    stream,
                    camera_slot,
                )
            }
            _ => Reply::Stall,
        };

        let data = match reply {
            Reply::Data(data) => data,
            Reply::Ack => {
                transfer.buffer = TransferBuffer::Vector(setup.as_bytes().to_vec());
                return Ok(());
            }
            Reply::Stall => {
                let request = setup.request;
                let value = setup.value;
                let index = setup.index;
                info!(
                    "uvc: stalling unsupported control request bRequest=0x{:02x} value=0x{:04x} \
                     index=0x{:04x}",
                    request, value, index
                );
                transfer.set_status(TransferStatus::Stalled);
                return Ok(());
            }
        };

        let transferred = cmp::min(data.len(), length);
        let mut response = setup.as_bytes().to_vec();
        response.extend_from_slice(&data[..transferred]);
        transfer.actual_length = transferred;
        transfer.buffer = TransferBuffer::Vector(response);
        Ok(())
    }

    /// Handles GET_DESCRIPTOR requests the generic backend layer leaves to the
    /// device driver: device, string, class specific interface and device
    /// qualifier descriptors.
    fn handle_get_descriptor(
        setup: &UsbRequestSetup,
        length: usize,
        camera_slot: usize,
    ) -> Reply {
        let descriptor_type = (setup.value >> 8) as u8;
        let descriptor_index = (setup.value & 0xff) as u8;
        let interface = (setup.index & 0xff) as u8;
        match descriptor_type {
            // DEVICE
            0x01 => Reply::Data(descriptors::device_descriptor(camera_slot).to_vec()),
            // CONFIGURATION: normally answered by the generic backend layer.
            0x02 => {
                let config = descriptors::config_descriptor();
                let count = cmp::min(length, config.len());
                Reply::Data(config[..count].to_vec())
            }
            // STRING: only the language id string, the device carries no
            // strings.
            0x03 => match descriptor_index {
                0 => Reply::Data(vec![0x04, 0x03, 0x09, 0x04]),
                _ => Reply::Stall,
            },
            // CS_INTERFACE: the video driver re-reads the class specific
            // descriptors of its interfaces.
            0x24 => {
                let config = descriptors::config_descriptor();
                let block = descriptors::class_specific_descriptors(&config, interface);
                if block.is_empty() {
                    Reply::Stall
                } else {
                    let count = cmp::min(length, block.len());
                    Reply::Data(block[..count].to_vec())
                }
            }
            _ => {
                let request = setup.request;
                let value = setup.value;
                let index = setup.index;
                info!(
                    "uvc: stalling unsupported descriptor type 0x{:02x} (bRequest=0x{:02x} \
                     value=0x{:04x} index=0x{:04x} length={})",
                    descriptor_type, request, value, index, length
                );
                Reply::Stall
            }
        }
    }

    /// Handles UVC class specific requests, which carry the video streaming
    /// probe and commit negotiation.
    fn handle_class_request(
        transfer: &UvcTransfer,
        setup: &UsbRequestSetup,
        length: usize,
        is_device_to_host: bool,
        stream: &Arc<Mutex<StreamState>>,
        camera_slot: usize,
    ) -> Reply {
        let selector = (setup.value >> 8) as u8;
        let interface = (setup.index & 0xff) as u8;
        let entity = (setup.index >> 8) as u8;
        info!(
            "uvc: class request bRequest=0x{:02x} selector=0x{:02x} interface={} entity={} \
             length={}",
            setup.request, selector, interface, entity, length
        );

        match (setup.request, selector) {
            (descriptors::request::GET_LEN, descriptors::selector::PROBE)
            | (descriptors::request::GET_LEN, descriptors::selector::COMMIT) => {
                Reply::Data((descriptors::PROBE_LEN as u16).to_le_bytes().to_vec())
            }
            (descriptors::request::GET_INFO, descriptors::selector::PROBE) => {
                // Get and set are supported on the probe control.
                Reply::Data(vec![0x03])
            }
            (descriptors::request::GET_INFO, descriptors::selector::COMMIT) => {
                // Only set is supported on the commit control.
                Reply::Data(vec![0x02])
            }
            (descriptors::request::GET_CUR, descriptors::selector::PROBE) => {
                Reply::Data(stream.lock().probe.to_vec())
            }
            (descriptors::request::GET_DEF, descriptors::selector::PROBE)
            | (descriptors::request::GET_MIN, descriptors::selector::PROBE)
            | (descriptors::request::GET_MAX, descriptors::selector::PROBE)
            | (descriptors::request::GET_RES, descriptors::selector::PROBE) => {
                Reply::Data(descriptors::default_probe().to_vec())
            }
            (descriptors::request::SET_CUR, descriptors::selector::PROBE) => {
                UvcDevice::set_probe(transfer, length, is_device_to_host, stream);
                Reply::Ack
            }
            (descriptors::request::SET_CUR, descriptors::selector::COMMIT) => {
                // A commit that announces neither a frame size nor a payload is how
                // the host turns streaming off again; everything else starts one.
                let sizes = UvcDevice::commit_sizes(transfer, length, is_device_to_host);
                let streaming =
                    matches!(sizes, Some((frame, payload)) if frame != 0 && payload != 0);
                camera::set_streaming(camera_slot, streaming);
                let mut stream = stream.lock();
                stream.committed = streaming;
                stream.restart();
                info!(
                    "uvc: streaming committed={} interval={} max frame={} max payload={}",
                    streaming,
                    u32::from_le_bytes(stream.probe[4..8].try_into().unwrap_or([0; 4])),
                    u32::from_le_bytes(stream.probe[18..22].try_into().unwrap_or([0; 4])),
                    u32::from_le_bytes(stream.probe[22..26].try_into().unwrap_or([0; 4])),
                );
                Reply::Ack
            }
            _ => Reply::Stall,
        }
    }

    /// Returns the frame and payload sizes announced by a commit control write.
    fn commit_sizes(
        transfer: &UvcTransfer,
        length: usize,
        is_device_to_host: bool,
    ) -> Option<(u32, u32)> {
        if is_device_to_host {
            return None;
        }
        let payload = match &transfer.buffer {
            TransferBuffer::Vector(v) => {
                let end = cmp::min(8 + length, v.len());
                v.get(8..end)?
            }
            _ => return None,
        };
        if payload.len() < 26 {
            return None;
        }
        Some((
            u32::from_le_bytes(payload[18..22].try_into().ok()?),
            u32::from_le_bytes(payload[22..26].try_into().ok()?),
        ))
    }

    /// Stores the probe structure sent by the guest, clamped to what this
    /// camera can actually deliver.
    fn set_probe(
        transfer: &UvcTransfer,
        length: usize,
        is_device_to_host: bool,
        stream: &Arc<Mutex<StreamState>>,
    ) {
        if is_device_to_host {
            return;
        }
        let payload = match &transfer.buffer {
            TransferBuffer::Vector(v) => {
                let end = cmp::min(8 + length, v.len());
                v.get(8..end).unwrap_or(&[])
            }
            _ => return,
        };

        let mut stream = stream.lock();
        let mut probe = stream.probe;
        let copied = cmp::min(payload.len(), probe.len());
        probe[..copied].copy_from_slice(&payload[..copied]);

        let requested = (
            probe[2],
            probe[3],
            u32::from_le_bytes(probe[4..8].try_into().unwrap_or([0; 4])),
            u32::from_le_bytes(probe[18..22].try_into().unwrap_or([0; 4])),
            u32::from_le_bytes(probe[22..26].try_into().unwrap_or([0; 4])),
            probe[30],
        );
        // Clamp the negotiated values to the one format and frame this device
        // implements. The buffer sizes are echoed back so that the negotiated
        // payload size never grows above what the host proposed: usbvideo.sys
        // rejects a probe whose dwMaxPayloadTransferSize it did not ask for.
        // Echo the frame the host picked, clamped to the three sizes we
        // advertise, and only the two intervals we implement.
        let frame_index = if (1..=descriptors::FRAMES.len() as u8).contains(&requested.1) {
            requested.1
        } else {
            descriptors::FRAMES.len() as u8
        };
        let (frame_width, frame_height) = descriptors::FRAMES[frame_index as usize - 1];
        let interval = if requested.2 == descriptors::FRAME_INTERVAL_30 {
            descriptors::FRAME_INTERVAL_30
        } else {
            descriptors::FRAME_INTERVAL_60
        };
        probe[2] = 1; // bFormatIndex
        probe[3] = frame_index;
        probe[4..8].copy_from_slice(&interval.to_le_bytes());
        probe[8..18].fill(0); // key frame rate, p frame rate, quality, window, delay
        probe[18..22].copy_from_slice(&(frame_width as u32 * frame_height as u32 * 2).to_le_bytes());
        // A bulk endpoint carries one packet worth of payload per transfer, so
        // the negotiated payload size must stay within wMaxPacketSize no matter
        // what the host asked for.
        probe[22..26].copy_from_slice(&descriptors::MAX_PAYLOAD_TRANSFER_SIZE.to_le_bytes());
        probe[26..30].fill(0); // dwClockFrequency
        probe[30] = 0; // bmFramingInfo
        probe[31] = 0x10; // bPreferredVersion
        probe[32] = 0x10; // bMinVersion
        probe[33] = 0x10; // bMaxVersion
        stream.probe = probe;
        stream.restart();
        info!(
            "uvc: probe request format={} frame={} interval={} max frame={} max payload={} \
             framing={:#04x}, stored interval={} max frame={} max payload={}",
            requested.0,
            requested.1,
            requested.2,
            requested.3,
            requested.4,
            requested.5,
            u32::from_le_bytes(probe[4..8].try_into().unwrap_or([0; 4])),
            u32::from_le_bytes(probe[18..22].try_into().unwrap_or([0; 4])),
            u32::from_le_bytes(probe[22..26].try_into().unwrap_or([0; 4])),
        );
    }

    /// Fills a bulk IN transfer with the next chunk of the current frame.
    fn handle_bulk_in(
        transfer: &mut UvcTransfer,
        stream: &Arc<Mutex<StreamState>>,
        camera_slot: usize,
    ) -> BackendResult<()> {
        transfer.actual_length = 0;
        let buffer_len = match &transfer.buffer {
            TransferBuffer::Vector(v) => v.len(),
            _ => return Err(BackendError::MalformedBackendTransfer),
        };
        if buffer_len <= PAYLOAD_HEADER_LEN {
            return Ok(());
        }

        camera::touch_activity(camera_slot);
        let mut stream = stream.lock();
        if !stream.committed {
            // The guest has not committed a stream yet: answer with a short
            // packet so that it keeps polling instead of hanging.
            return Ok(());
        }

        if stream.offset >= stream.frame.len() {
            // A new frame is due. Only start one when the capture loop produced a
            // picture we have not sent yet: handing out the same frame as fast as
            // the host asks would flood the bus with copies and starve the real
            // frames. Without a new picture the transfer completes empty, which
            // makes the host wait instead of spinning.
            match camera::take_latest_frame(camera_slot) {
                Some((seq, live)) if seq != stream.frame_seq => {
                    stream.frame = live;
                    stream.frame_seq = seq;
                    stream.offset = 0;
                    stream.fid ^= 1;
                }
                _ if stream.test_pattern_pending => {
                    // Show the static picture once while the camera opens.
                    stream.frame = descriptors::TEST_PATTERN.to_vec();
                    stream.test_pattern_pending = false;
                    stream.offset = 0;
                    stream.fid ^= 1;
                }
                _ => return Ok(()),
            }
        }

        let frame_len = stream.frame.len();
        let payload_len = cmp::min(frame_len - stream.offset, buffer_len - PAYLOAD_HEADER_LEN);
        let end_of_frame = stream.offset + payload_len == frame_len;
        let mut header_info = stream.fid & 0x01;
        if end_of_frame {
            header_info |= 0x02;
        }

        if let TransferBuffer::Vector(v) = &mut transfer.buffer {
            v[0] = PAYLOAD_HEADER_LEN as u8;
            v[1] = header_info;
            v[PAYLOAD_HEADER_LEN..PAYLOAD_HEADER_LEN + payload_len]
                .copy_from_slice(&stream.frame[stream.offset..stream.offset + payload_len]);
        }
        stream.offset += payload_len;
        if end_of_frame {
            stream.frames_sent += 1;
            if stream.frames_sent % 60 == 0 {
                info!(
                    "uvc: delivered {} frames, last frame {} bytes",
                    stream.frames_sent, frame_len
                );
            }
        }
        transfer.actual_length = PAYLOAD_HEADER_LEN + payload_len;
        stream.transfers += 1;
        if stream.transfers <= 8 || stream.transfers % 5000 == 0 {
            info!(
                "uvc: bulk transfer #{} buffer={} actual={} offset={} fid={} eof={}",
                stream.transfers,
                buffer_len,
                transfer.actual_length,
                stream.offset,
                stream.fid,
                end_of_frame
            );
        }
        Ok(())
    }

    /// Completes a status interrupt IN transfer with a short packet: the camera
    /// has no asynchronous events to report.
    fn handle_status_in(
        transfer: &mut UvcTransfer,
        stream: &Arc<Mutex<StreamState>>,
    ) -> BackendResult<()> {
        transfer.actual_length = 0;
        let buffer_len = match &transfer.buffer {
            TransferBuffer::Vector(v) => v.len(),
            _ => 0,
        };
        let mut stream = stream.lock();
        stream.status_transfers += 1;
        if stream.status_transfers <= 4 || stream.status_transfers % 500 == 0 {
            info!(
                "uvc: status transfer #{} buffer={}",
                stream.status_transfers, buffer_len
            );
        }
        Ok(())
    }

    /// Records the alternate setting the guest selected on a video interface.
    /// Selecting alt setting 1 on the streaming interface starts the stream.
    pub fn set_interface(&mut self, interface: u8, alt: u8) -> BackendResult<TransferStatus> {
        if interface != descriptors::STREAM_INTERFACE {
            return Ok(TransferStatus::Completed);
        }
        info!("uvc: streaming interface alternate setting {alt}");
        // Alternate setting 0 means the host stopped the stream; anything else
        // starts it. The commit control is the authoritative signal, but this
        // keeps the camera closed when the host switches away without one.
        if alt == 0 {
            camera::set_streaming(self.camera_slot, false);
        }
        let mut stream = self.stream.lock();
        stream.alt_setting = alt;
        stream.committed = alt != 0;
        stream.restart();
        Ok(TransferStatus::Completed)
    }

    /// Drops any stream the guest may be running.
    fn stop_stream(&self) {
        camera::set_streaming(self.camera_slot, false);
        let mut stream = self.stream.lock();
        stream.committed = false;
        stream.restart();
    }
}

impl AsRawDescriptor for UvcDevice {
    fn as_raw_descriptor(&self) -> RawDescriptor {
        // The camera has no host file descriptor to wait on.
        -1
    }
}

impl BackendDevice for UvcDevice {
    fn submit_backend_transfer(
        &mut self,
        transfer: BackendTransferType,
    ) -> BackendResult<BackendTransferHandle> {
        let transfer = match transfer {
            BackendTransferType::UvcDevice(transfer) => transfer,
            _ => return Err(BackendError::MalformedBackendTransfer),
        };

        let endpoint = transfer.endpoint;
        let stream = self.stream.clone();
        let camera_slot = self.camera_slot;
        let arc_transfer = Arc::new(Mutex::new(Some(transfer)));
        let cancel_handle = UvcTransferHandle {
            weak_transfer: Arc::downgrade(&arc_transfer),
        };

        self.transfer_job_queue
            .queue_job(move || {
                let mut lock = arc_transfer.lock();
                let mut transfer = match lock.take() {
                    Some(transfer) => transfer,
                    None => {
                        error!("uvc: transfer disappeared before it could be handled");
                        return;
                    }
                };
                drop(lock);

                let result = if endpoint == 0 {
                    UvcDevice::handle_control(&mut transfer, &stream, camera_slot)
                } else if endpoint == descriptors::STATUS_ENDPOINT {
                    UvcDevice::handle_status_in(&mut transfer, &stream)
                } else {
                    UvcDevice::handle_bulk_in(&mut transfer, &stream, camera_slot)
                };
                if let Err(e) = result {
                    error!("uvc: endpoint {endpoint} transfer failed: {e:#}");
                    transfer.set_status(TransferStatus::Error);
                }
                transfer.complete_transfer();
            })
            .map_err(BackendError::QueueAsyncJob)?;

        Ok(BackendTransferHandle::new(cancel_handle))
    }

    fn detach_event_handler(&self, _event_loop: &Arc<EventLoop>) -> BackendResult<()> {
        self.stop_stream();
        Ok(())
    }

    fn request_transfer_buffer(&mut self, size: usize) -> TransferBuffer {
        TransferBuffer::Vector(vec![0u8; size])
    }

    fn build_bulk_transfer(
        &mut self,
        ep_addr: u8,
        transfer_buffer: TransferBuffer,
        _stream_id: Option<u16>,
    ) -> BackendResult<BackendTransferType> {
        let buffer_len = match &transfer_buffer {
            TransferBuffer::Vector(v) => v.len(),
            _ => 0,
        };
        // Only the first few submits are worth logging: one frame is split into
        // dozens of bulk transfers, and logging each of them slows the stream.
        static BUILD_COUNT: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
        let build_count = BUILD_COUNT.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
        if build_count < 4 || build_count % 2000 == 0 {
            info!("uvc: build bulk transfer #{build_count} ep=0x{ep_addr:02x} buffer={buffer_len}");
        }
        Ok(BackendTransferType::UvcDevice(UvcTransfer::new(
            ep_addr,
            transfer_buffer,
        )))
    }

    fn build_interrupt_transfer(
        &mut self,
        ep_addr: u8,
        transfer_buffer: TransferBuffer,
    ) -> BackendResult<BackendTransferType> {
        Ok(BackendTransferType::UvcDevice(UvcTransfer::new(
            ep_addr,
            transfer_buffer,
        )))
    }

    fn build_isochronous_transfer(
        &mut self,
        _ep_addr: u8,
        _transfer_buffer: TransferBuffer,
        _packet_lengths: &[u32],
    ) -> BackendResult<BackendTransferType> {
        Err(BackendError::MalformedBackendTransfer)
    }

    fn get_control_transfer_state(&mut self) -> Arc<RwLock<ControlTransferState>> {
        self.control_transfer_state.clone()
    }

    fn get_device_state(&mut self) -> Arc<RwLock<DeviceState>> {
        self.state.clone()
    }

    fn get_active_config_descriptor(&mut self) -> BackendResult<ConfigDescriptorTree> {
        self.get_config_descriptor_by_index(0)
    }

    fn get_config_descriptor(&mut self, config: u8) -> BackendResult<ConfigDescriptorTree> {
        let device_descriptor = self.get_device_descriptor_tree()?;
        if let Some(config_descriptor) = device_descriptor.get_config_descriptor(config) {
            return Ok(config_descriptor.clone());
        }
        Err(BackendError::GetConfigDescriptor(
            UsbUtilError::DescriptorParse,
        ))
    }

    fn get_config_descriptor_by_index(
        &mut self,
        config_index: u8,
    ) -> BackendResult<ConfigDescriptorTree> {
        let device_descriptor = self.get_device_descriptor_tree()?;
        if let Some(config_descriptor) =
            device_descriptor.get_config_descriptor_by_index(config_index)
        {
            return Ok(config_descriptor.clone());
        }
        Err(BackendError::GetConfigDescriptor(
            UsbUtilError::DescriptorParse,
        ))
    }

    fn get_device_descriptor_tree(&mut self) -> BackendResult<DeviceDescriptorTree> {
        let mut descbuf = descriptors::device_descriptor(self.camera_slot).to_vec();
        descbuf.extend_from_slice(&descriptors::config_descriptor());
        parse_usbfs_descriptors(&descbuf).map_err(BackendError::GetDeviceDescriptor)
    }

    fn get_active_configuration(&mut self) -> BackendResult<u8> {
        let descriptor_tree = self.get_device_descriptor_tree()?;
        if let Some(config_descriptor) = descriptor_tree.get_config_descriptor_by_index(0) {
            return Ok(config_descriptor.bConfigurationValue);
        }
        Err(BackendError::GetActiveConfig(UsbUtilError::DescriptorParse))
    }

    fn set_active_configuration(&mut self, config: u8) -> BackendResult<()> {
        if config != 0 && config != descriptors::CONFIG_VALUE {
            error!("uvc: requested configuration {config}, only the default one exists");
            return Err(BackendError::BadBackendProviderState);
        }
        self.stop_stream();
        Ok(())
    }

    fn clear_feature(&mut self, _value: u16, _index: u16) -> BackendResult<TransferStatus> {
        Ok(TransferStatus::Completed)
    }

    fn create_endpoints(&mut self, _config_descriptor: &ConfigDescriptorTree) -> BackendResult<()> {
        let device_state = self.get_device_state();
        let (fail_handle, job_queue) = {
            let state = device_state.read().unwrap();
            (state.fail_handle.clone(), state.job_queue.clone())
        };
        let mut endpoints = Vec::new();
        endpoints.push(UsbEndpoint::new(
            fail_handle.clone(),
            job_queue.clone(),
            1,
            EndpointDirection::DeviceToHost,
            EndpointType::Bulk,
        ));
        endpoints.push(UsbEndpoint::new(
            fail_handle,
            job_queue,
            2,
            EndpointDirection::DeviceToHost,
            EndpointType::Interrupt,
        ));
        device_state.write().unwrap().endpoints = endpoints;
        Ok(())
    }
}

impl XhciBackendDevice for UvcDevice {
    fn get_backend_type(&self) -> BackendType {
        BackendType::Usb2
    }

    fn get_vid(&self) -> u16 {
        descriptors::VENDOR_ID
    }

    fn get_pid(&self) -> u16 {
        descriptors::PRODUCT_ID
    }

    fn set_address(&mut self, _address: UsbDeviceAddress) {
        // The guest address is meaningful to the xhci layer only.
    }

    fn reset(&mut self) -> BackendResult<()> {
        self.stop_stream();
        Ok(())
    }

    fn get_speed(&self) -> Option<DeviceSpeed> {
        Some(DeviceSpeed::High)
    }

    fn alloc_streams(&self, _ep: u8, _num_streams: u16) -> BackendResult<()> {
        // Bulk endpoints of this camera do not support streams.
        Ok(())
    }

    fn free_streams(&self, _ep: u8) -> BackendResult<()> {
        Ok(())
    }

    fn stop(&mut self) {
        self.stop_stream();
    }
}

/// Creates the virtual camera and wraps it in the backend device enum, ready to
/// be connected to the xhci hub.
pub fn attach_virtual_camera(
    device_state: DeviceState,
    event_loop: Arc<EventLoop>,
    camera_slot: usize,
    camera_id: String,
) -> BackendResult<Arc<Mutex<BackendDeviceType>>> {
    let device = UvcDevice::new(device_state, event_loop, camera_slot, camera_id)?;
    Ok(Arc::new(Mutex::new(BackendDeviceType::UvcDevice(device))))
}


