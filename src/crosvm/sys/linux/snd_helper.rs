// Copyright 2026 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! Runs the virtio-snd audio backend in a child process under an unprivileged uid.
//!
//! Android decides whether a stream is audible from the uid that opened it, and silences uid 0 in
//! both directions -- playback comes out at -inf dB, capture returns zeroes. crosvm runs as root,
//! so an in-process audio device is muted by the platform no matter what it does. The way out is
//! for the audio backend to live in a process that is not root and talk to the VMM over
//! vhost-user.
//!
//! Two things about the shape of this are deliberate:
//!
//! * **exec, not just fork.** A bare fork is cheaper and crosvm already has the machinery for it
//!   (see `ext2::launch`), but the child has to reach `audioserver` over binder, and libbinder's
//!   state -- its `/dev/binder` fd and its thread pool -- does not survive a fork. crosvm has
//!   already initialised binder by this point for the Android display service. exec buys the
//!   child a clean address space, which is precisely what it needs.
//! * **socketpair, not a socket file.** vhost-user is just a protocol over a `SOCK_STREAM`; a
//!   filesystem path only exists so that two unrelated processes can find each other. Here they
//!   are related, so the fd is inherited: nothing to place in the filesystem, no rendezvous, and
//!   no window in which the VMM has to poll for the backend to come up.

use std::os::fd::AsRawFd;
use std::os::unix::net::UnixStream;
use std::os::unix::process::CommandExt;
use std::process::Command;

use anyhow::anyhow;
use anyhow::Context;
use anyhow::Result;
use base::Pid;
use devices::virtio::snd::parameters::Parameters as SndParameters;

/// Spawns the backend and returns the VMM's end of the connection along with the child's pid.
pub fn launch(mut params: SndParameters) -> Result<(UnixStream, Pid)> {
    let uid = params
        .uid
        .ok_or_else(|| anyhow!("snd_helper::launch called without a uid"))?;
    let gid = params.gid.unwrap_or(uid);
    let supp_gids = std::mem::take(&mut params.supp_gids);
    // Whatever the child does, it must not try to spawn a backend of its own.
    params.uid = None;
    params.gid = None;

    let (vmm_end, backend_end) = UnixStream::pair().context("failed to create snd socketpair")?;
    let backend_fd = backend_end.as_raw_fd();
    // The child is handed this fd by number, so it has to survive the exec.
    clear_cloexec(backend_fd).context("failed to clear CLOEXEC on the snd socket")?;

    // Build everything the child needs before forking, the group list included: after the fork
    // there is only one thread, and anything that takes an allocator lock another thread was
    // holding would deadlock.
    let config = serde_json::to_string(&params).context("failed to serialise snd parameters")?;
    let fd_arg = backend_fd.to_string();
    let parent_pid = std::process::id();

    let mut command = Command::new("/proc/self/exe");
    command
        .arg("device")
        .arg("snd")
        .arg("--fd")
        .arg(&fd_arg)
        .arg("--config-json")
        .arg(&config);

    // SAFETY: the closure runs between fork and exec, and calls only async-signal-safe libc
    // functions. It allocates nothing -- every argument was formatted above.
    unsafe {
        command.pre_exec(move || {
            // Set the supplementary groups first, then the group, then the user: each step
            // needs the privilege that the next one gives away. crosvm's own groups are root's,
            // so an empty list is the safe default -- carrying any of them into an unprivileged
            // process would be a privilege leak. Anything the backend genuinely needs was named
            // on the command line by whoever knows.
            let (count, ptr) = if supp_gids.is_empty() {
                (0, std::ptr::null())
            } else {
                (supp_gids.len() as libc::c_int, supp_gids.as_ptr())
            };
            if libc::setgroups(count as libc::size_t, ptr) != 0 {
                return Err(std::io::Error::last_os_error());
            }
            if libc::setgid(gid) != 0 {
                return Err(std::io::Error::last_os_error());
            }
            if libc::setuid(uid) != 0 {
                return Err(std::io::Error::last_os_error());
            }
            // Only now: changing credentials clears the parent-death signal (`commit_creds()`
            // zeroes `task->pdeath_signal` whenever euid/egid change), so setting it before the
            // setuid above would silently leave the child able to outlive crosvm. It survives the
            // exec that follows because crosvm is not a set-user-ID binary.
            if libc::prctl(libc::PR_SET_PDEATHSIG, libc::SIGKILL) != 0 {
                return Err(std::io::Error::last_os_error());
            }
            // Close the race the line above cannot: had crosvm died while we were dropping
            // privileges, the death signal was armed too late to ever arrive.
            if libc::getppid() as u32 != parent_pid {
                libc::_exit(1);
            }
            Ok(())
        })
    };

    let child = command.spawn().context("failed to spawn the snd backend")?;
    // The backend owns its end now. Holding a copy here would keep the connection open after the
    // child died, and the VMM would wait forever for a peer that is gone.
    drop(backend_end);
    Ok((vmm_end, child.id() as Pid))
}

fn clear_cloexec(fd: std::os::fd::RawFd) -> std::io::Result<()> {
    // SAFETY: `fd` is owned by the caller and stays open for the duration of these calls.
    let flags = unsafe { libc::fcntl(fd, libc::F_GETFD) };
    if flags < 0 {
        return Err(std::io::Error::last_os_error());
    }
    // SAFETY: as above.
    if unsafe { libc::fcntl(fd, libc::F_SETFD, flags & !libc::FD_CLOEXEC) } < 0 {
        return Err(std::io::Error::last_os_error());
    }
    Ok(())
}
