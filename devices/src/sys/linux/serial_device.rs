// Copyright 2022 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use std::borrow::Cow;
use std::fs::File;
use std::fs::OpenOptions;
use std::io;
use std::io::ErrorKind;
use std::io::Read;
use std::io::Write;
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::io::FromRawFd;
use std::os::unix::net::UnixDatagram;
use std::os::unix::net::UnixStream;
use std::path::Path;
use std::path::PathBuf;
use std::thread;
use std::time::Duration;

use base::error;
use base::info;
use base::read_raw_stdin;
use base::AsRawDescriptor;
use base::Event;
use base::FileSync;
use base::RawDescriptor;
use base::ReadNotifier;
use hypervisor::ProtectionType;

use crate::serial_device::Error;
use crate::serial_device::SerialInput;
use crate::serial_device::SerialOptions;
use crate::serial_device::SerialParameters;

pub const SYSTEM_SERIAL_TYPE_NAME: &str = "UnixSocket";

// This wrapper is used in place of the libstd native version because we don't want
// buffering for stdin.
pub struct ConsoleInput(std::io::Stdin);

impl ConsoleInput {
    pub fn new() -> Self {
        Self(std::io::stdin())
    }
}

impl io::Read for ConsoleInput {
    fn read(&mut self, out: &mut [u8]) -> io::Result<usize> {
        read_raw_stdin(out).map_err(|e| e.into())
    }
}

impl ReadNotifier for ConsoleInput {
    fn get_read_notifier(&self) -> &dyn AsRawDescriptor {
        &self.0
    }
}

impl SerialInput for ConsoleInput {}

/// Abstraction over serial-like devices that can be created given an event and optional input and
/// output streams.
pub trait SerialDevice {
    fn new(
        protection_type: ProtectionType,
        interrupt_evt: Event,
        input: Option<Box<dyn SerialInput>>,
        output: Option<Box<dyn io::Write + Send>>,
        sync: Option<Box<dyn FileSync + Send>>,
        options: SerialOptions,
        keep_rds: Vec<RawDescriptor>,
    ) -> Self;
}

// The maximum length of a path that can be used as the address of a
// unix socket. Note that this includes the null-terminator.
pub const MAX_SOCKET_PATH_LENGTH: usize = 108;

struct WriteSocket {
    sock: UnixDatagram,
    buf: Vec<u8>,
}

const BUF_CAPACITY: usize = 1024;

impl WriteSocket {
    pub fn new(s: UnixDatagram) -> WriteSocket {
        WriteSocket {
            sock: s,
            buf: Vec::with_capacity(BUF_CAPACITY),
        }
    }

    pub fn send_buf(&self, buf: &[u8]) -> io::Result<usize> {
        const SEND_RETRY: usize = 2;
        let mut sent = 0;
        for _ in 0..SEND_RETRY {
            match self.sock.send(buf) {
                Ok(bytes_sent) => {
                    sent = bytes_sent;
                    break;
                }
                Err(e) => info!("Send error: {:?}", e),
            }
        }
        Ok(sent)
    }
}

impl io::Write for WriteSocket {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        let last_newline_idx = match buf.iter().rposition(|&x| x == b'\n') {
            Some(newline_idx) => Some(self.buf.len() + newline_idx),
            None => None,
        };
        self.buf.extend_from_slice(buf);

        match last_newline_idx {
            Some(last_newline_idx) => {
                for line in (self.buf[..last_newline_idx]).split(|&x| x == b'\n') {
                    // Also drop CR+LF line endings.
                    let send_line = match line.split_last() {
                        Some((b'\r', trimmed)) => trimmed,
                        _ => line,
                    };
                    if self.send_buf(send_line).is_err() {
                        break;
                    }
                }
                self.buf.drain(..=last_newline_idx);
            }
            None => {
                if self.buf.len() >= BUF_CAPACITY {
                    if let Err(e) = self.send_buf(&self.buf) {
                        info!("Couldn't send full buffer. {:?}", e);
                    }
                    self.buf.clear();
                }
            }
        }
        Ok(buf.len())
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

pub(crate) fn create_system_type_serial_device<T: SerialDevice>(
    param: &SerialParameters,
    protection_type: ProtectionType,
    evt: Event,
    input: Option<Box<dyn SerialInput>>,
    keep_rds: &mut Vec<RawDescriptor>,
) -> std::result::Result<T, Error> {
    match &param.path {
        Some(path) => {
            // If the path is longer than 107 characters,
            // then we won't be able to connect directly
            // to it. Instead we can shorten the path by
            // opening the containing directory and using
            // /proc/self/fd/*/ to access it via a shorter
            // path.
            let mut path_cow = Cow::<Path>::Borrowed(path);
            let mut _dir_fd = None;
            if path.as_os_str().len() >= MAX_SOCKET_PATH_LENGTH {
                let mut short_path = PathBuf::with_capacity(MAX_SOCKET_PATH_LENGTH);
                short_path.push("/proc/self/fd/");

                let parent_path = path
                    .parent()
                    .ok_or_else(|| Error::InvalidPath(path.clone()))?;
                let file_name = path
                    .file_name()
                    .ok_or_else(|| Error::InvalidPath(path.clone()))?;

                // We don't actually want to open this
                // directory for reading, but the stdlib
                // requires all files be opened as at
                // least one of readable, writeable, or
                // appeandable.
                let dir = OpenOptions::new()
                    .read(true)
                    .open(parent_path)
                    .map_err(|e| Error::FileOpen(e, parent_path.into()))?;

                short_path.push(dir.as_raw_descriptor().to_string());
                short_path.push(file_name);
                path_cow = Cow::Owned(short_path);
                _dir_fd = Some(dir);
            }

            // The shortened path may still be too long,
            // in which case we must give up here.
            if path_cow.as_os_str().len() >= MAX_SOCKET_PATH_LENGTH {
                return Err(Error::InvalidPath(path_cow.into()));
            }

            // There's a race condition between
            // vmlog_forwarder making the logging socket and
            // crosvm starting up, so we loop here until it's
            // available.
            let sock = UnixDatagram::unbound().map_err(Error::SocketCreate)?;
            loop {
                match sock.connect(&path_cow) {
                    Ok(_) => break,
                    Err(e) => {
                        match e.kind() {
                            ErrorKind::NotFound | ErrorKind::ConnectionRefused => {
                                // logging socket doesn't
                                // exist yet, sleep for 10 ms
                                // and try again.
                                thread::sleep(Duration::from_millis(10))
                            }
                            _ => {
                                error!("Unexpected error connecting to logging socket: {:?}", e);
                                return Err(Error::SocketConnect(e));
                            }
                        }
                    }
                };
            }
            keep_rds.push(sock.as_raw_descriptor());
            let output: Option<Box<dyn Write + Send>> = Some(Box::new(WriteSocket::new(sock)));
            Ok(T::new(
                protection_type,
                evt,
                input,
                output,
                None,
                Default::default(),
                keep_rds.to_vec(),
            ))
        }
        None => Err(Error::PathRequired),
    }
}

/// Creates a serial device that use the given UnixStream path for both input and output.
pub(crate) fn create_unix_stream_serial_device<T: SerialDevice>(
    param: &SerialParameters,
    protection_type: ProtectionType,
    evt: Event,
    keep_rds: &mut Vec<RawDescriptor>,
) -> std::result::Result<T, Error> {
    let path = param.path.as_ref().ok_or(Error::PathRequired)?;
    let input = UnixStream::connect(path).map_err(Error::SocketConnect)?;
    let output = input.try_clone().map_err(Error::CloneUnixStream)?;
    keep_rds.push(input.as_raw_descriptor());
    keep_rds.push(output.as_raw_descriptor());

    Ok(T::new(
        protection_type,
        evt,
        Some(Box::new(input)),
        Some(Box::new(output)),
        None,
        SerialOptions {
            name: param.name.clone(),
            out_timestamp: param.out_timestamp,
            console: param.console,
            pci_address: param.pci_address,
        },
        keep_rds.to_vec(),
    ))
}

/// Reader over a non-blocking fd shared with the writer side: converts WouldBlock
/// into Interrupted so the serial input worker treats a spurious wakeup as harmless
/// and goes back to waiting instead of exiting its loop.
struct NonBlockSerialInput(File);

impl Read for NonBlockSerialInput {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self.0.read(buf) {
            Err(e) if e.kind() == ErrorKind::WouldBlock => {
                Err(io::Error::new(ErrorKind::Interrupted, e))
            }
            r => r,
        }
    }
}

impl ReadNotifier for NonBlockSerialInput {
    fn get_read_notifier(&self) -> &dyn AsRawDescriptor {
        &self.0
    }
}

impl SerialInput for NonBlockSerialInput {}

/// Writer over a non-blocking fd. When the far side can't accept more data (a pty
/// whose slave nobody drains, a USB gadget port with no host attached) the bytes are
/// dropped instead of blocking: the write happens on the vcpu thread servicing the
/// serial MMIO exit, and stalling it would freeze the guest.
struct NonBlockSerialOutput {
    dev: File,
    // For type=pty: our own open of the slave, held for the lifetime of the device so
    // the master never returns HUP/EOF when external consumers close the slave.
    _keep_open: Option<File>,
}

impl Write for NonBlockSerialOutput {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        match self.dev.write(buf) {
            Err(e) if e.kind() == ErrorKind::WouldBlock => Ok(buf.len()),
            r => r,
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        Ok(())
    }
}

/// Puts `fd` into raw mode (when it is a terminal) and makes it non-blocking.
fn set_raw_and_nonblocking(fd: libc::c_int) -> io::Result<()> {
    // SAFETY: fd is a valid open descriptor owned by the caller; termios is a plain
    // out-param struct and name lookups write within its bounds.
    unsafe {
        let mut termios: libc::termios = std::mem::zeroed();
        if libc::tcgetattr(fd, &mut termios) == 0 {
            libc::cfmakeraw(&mut termios);
            if libc::tcsetattr(fd, libc::TCSANOW, &termios) != 0 {
                return Err(io::Error::last_os_error());
            }
        }
        // (tcgetattr failing means fd isn't a tty; raw mode is then a no-op.)
        let flags = libc::fcntl(fd, libc::F_GETFL);
        if flags < 0 || libc::fcntl(fd, libc::F_SETFL, flags | libc::O_NONBLOCK) < 0 {
            return Err(io::Error::last_os_error());
        }
    }
    Ok(())
}

/// Creates a serial device backed by a freshly-allocated pseudo-terminal. crosvm
/// keeps the master for both directions (and holds one open of the slave so it never
/// sees HUP); external consumers open the slave (/dev/pts/N). `param.path`, if set,
/// is created as a symlink pointing at the slave.
pub(crate) fn create_pty_serial_device<T: SerialDevice>(
    param: &SerialParameters,
    protection_type: ProtectionType,
    evt: Event,
    keep_rds: &mut Vec<RawDescriptor>,
) -> std::result::Result<T, Error> {
    let ptmx_path = PathBuf::from("/dev/ptmx");
    // SAFETY: posix_openpt returns a fresh fd that is owned solely by `master` below.
    let master_fd = unsafe { libc::posix_openpt(libc::O_RDWR | libc::O_NOCTTY) };
    if master_fd < 0 {
        return Err(Error::FileOpen(io::Error::last_os_error(), ptmx_path));
    }
    // SAFETY: master_fd is valid and uniquely owned by this File.
    let master = unsafe { File::from_raw_fd(master_fd) };

    // SAFETY: grantpt/unlockpt operate on our valid fd; ptsname_r writes at most
    // name_buf.len() bytes, NUL-terminated, into name_buf.
    let pts_path = unsafe {
        if libc::grantpt(master_fd) != 0 || libc::unlockpt(master_fd) != 0 {
            return Err(Error::FileOpen(io::Error::last_os_error(), ptmx_path));
        }
        let mut name_buf = [0u8; 128];
        if libc::ptsname_r(
            master_fd,
            name_buf.as_mut_ptr() as *mut libc::c_char,
            name_buf.len(),
        ) != 0
        {
            return Err(Error::FileOpen(io::Error::last_os_error(), ptmx_path));
        }
        let cstr = std::ffi::CStr::from_ptr(name_buf.as_ptr() as *const libc::c_char);
        PathBuf::from(cstr.to_string_lossy().into_owned())
    };

    // Raw is mandatory: the default pty line discipline would echo guest output
    // straight back into guest input. Non-blocking so a slow or absent consumer can
    // never stall the vcpu (excess output is dropped by NonBlockSerialOutput).
    set_raw_and_nonblocking(master_fd).map_err(|e| Error::FileOpen(e, ptmx_path))?;

    let keep_open = OpenOptions::new()
        .read(true)
        .write(true)
        .custom_flags(libc::O_NOCTTY)
        .open(&pts_path)
        .map_err(|e| Error::FileOpen(e, pts_path.clone()))?;

    if let Some(link) = &param.path {
        let _ = std::fs::remove_file(link);
        std::os::unix::fs::symlink(&pts_path, link)
            .map_err(|e| Error::FileCreate(e, link.clone()))?;
        info!(
            "pty serial: {} -> {}",
            link.display(),
            pts_path.display()
        );
    } else {
        info!("pty serial at {}", pts_path.display());
    }

    let master_reader = master.try_clone().map_err(Error::FileClone)?;
    keep_rds.push(master.as_raw_descriptor());
    keep_rds.push(master_reader.as_raw_descriptor());
    keep_rds.push(keep_open.as_raw_descriptor());

    Ok(T::new(
        protection_type,
        evt,
        Some(Box::new(NonBlockSerialInput(master_reader))),
        Some(Box::new(NonBlockSerialOutput {
            dev: master,
            _keep_open: Some(keep_open),
        })),
        None,
        SerialOptions {
            name: param.name.clone(),
            out_timestamp: param.out_timestamp,
            console: param.console,
            pci_address: param.pci_address,
        },
        keep_rds.to_vec(),
    ))
}

/// Creates a serial device backed by an existing character device (e.g. a USB gadget
/// serial port such as /dev/ttyGS0), opened raw and non-blocking for both directions.
/// An explicit `input=` path takes precedence over reading the device itself.
pub(crate) fn create_dev_serial_device<T: SerialDevice>(
    param: &SerialParameters,
    protection_type: ProtectionType,
    evt: Event,
    input: Option<Box<dyn SerialInput>>,
    keep_rds: &mut Vec<RawDescriptor>,
) -> std::result::Result<T, Error> {
    let path = param.path.as_ref().ok_or(Error::PathRequired)?;
    let dev = OpenOptions::new()
        .read(true)
        .write(true)
        .custom_flags(libc::O_NOCTTY | libc::O_NONBLOCK)
        .open(path)
        .map_err(|e| Error::FileOpen(e, path.clone()))?;
    set_raw_and_nonblocking(dev.as_raw_descriptor()).map_err(|e| Error::FileOpen(e, path.clone()))?;

    let input: Option<Box<dyn SerialInput>> = match input {
        Some(input) => Some(input),
        None => {
            let reader = dev.try_clone().map_err(Error::FileClone)?;
            keep_rds.push(reader.as_raw_descriptor());
            Some(Box::new(NonBlockSerialInput(reader)))
        }
    };
    keep_rds.push(dev.as_raw_descriptor());

    Ok(T::new(
        protection_type,
        evt,
        input,
        Some(Box::new(NonBlockSerialOutput {
            dev,
            _keep_open: None,
        })),
        None,
        SerialOptions {
            name: param.name.clone(),
            out_timestamp: param.out_timestamp,
            console: param.console,
            pci_address: param.pci_address,
        },
        keep_rds.to_vec(),
    ))
}
