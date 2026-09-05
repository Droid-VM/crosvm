// Copyright 2026 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! Runs a vhost-user device backend in a child process under an unprivileged uid.
//!
//! Android decides what a process may do from the uid it runs as, and crosvm runs as root. That
//! is fatal for two families of device: audio, because the platform silences uid 0 in both
//! directions -- playback comes out at -inf dB, capture returns zeroes -- and camera and codecs,
//! because `cameraserver` and the codec services resolve the calling package from the real uid
//! and refuse uid 0 outright. The way out is the same for both: the device's backend lives in a
//! process that is not root and talks to the VMM over vhost-user. This is the launcher they
//! share; `crosvm device snd` and `crosvm device media` are what it starts.
//!
//! Two things about the shape of this are deliberate:
//!
//! * **exec, not just fork.** A bare fork is cheaper and crosvm already has the machinery for it
//!   (see `ext2::launch`), but the child has to reach `audioserver` / `cameraserver` over binder,
//!   and libbinder's state -- its `/dev/binder` fd and its thread pool -- does not survive a fork.
//!   crosvm has already initialised binder by this point for the Android display service. exec
//!   buys the child a clean address space, which is precisely what it needs.
//! * **socketpair, not a socket file.** vhost-user is just a protocol over a `SOCK_STREAM`; a
//!   filesystem path only exists so that two unrelated processes can find each other. Here they
//!   are related, so the fd is inherited: nothing to place in the filesystem, no rendezvous, and
//!   no window in which the VMM has to poll for the backend to come up.
//!
//! The child inherits nothing else: crosvm's descriptors are `CLOEXEC` by construction, so the
//! socket is the one it gets, by number. Everything else it needs travels as JSON in argv.
//!
//! One thing that does *not* travel by itself is the log level. crosvm's syslog reads no
//! environment variable -- `base::syslog`'s `LogConfig` defaults its filter to `"info"` and only
//! the `--log-level` argument moves it -- and a helper is a fresh `exec` of `/proc/self/exe`, so
//! without help every backend runs pinned at `info` however the VMM was started. That made every
//! `debug!` in a helper dead weight on a shipped build: defect D57, which cost the D37
//! investigation its device-side instrument (`logs/vpu_wp/B10-acceptance.md` §7, §13). The VMM
//! records its own filter when it initialises its logger (`sys::linux::main::init_log`) and
//! `launch` replays it here, so `crosvm --log-level debug run ...` gives its snd and media
//! helpers the same level.

use std::os::fd::AsRawFd;
use std::os::unix::net::UnixStream;
use std::os::unix::process::CommandExt;
use std::process::Command;
use std::sync::OnceLock;

use anyhow::Context;
use anyhow::Result;
use base::Pid;

/// The VMM's own `--log-level`, as parsed into `LogConfig`'s filter.
///
/// Set once, by the VMM, before any device is created; read by `launch`. A `OnceLock` rather than
/// a parameter because every caller of `launch` is several layers below the command line, and
/// because a helper process never sets it -- it has no children of its own.
static VMM_LOG_FILTER: OnceLock<String> = OnceLock::new();

/// Records the log filter the VMM was started with, so helpers can be started at the same level.
///
/// Idempotent and never fatal: a second call (or a call in a process that is itself a helper) is
/// dropped, because the level a helper should run at is the one the VMM asked for first.
pub(crate) fn set_vmm_log_filter(filter: String) {
    let _ = VMM_LOG_FILTER.set(filter);
}

/// The filter a helper launched now would be started at, for the launch log lines.
///
/// `"info (default)"` when nothing was recorded, because that is what the child will apply and
/// the distinction -- inherited or fallen back to -- is the whole of D57.
pub(crate) fn vmm_log_filter() -> String {
    match VMM_LOG_FILTER.get() {
        Some(filter) => filter.clone(),
        None => "info (default)".to_string(),
    }
}

/// Spawns `crosvm [--log-level <filter>] device <subcommand> --fd N --config-json <params_json>
/// [--pool-fd M]` as `uid`:`gid` with exactly `supp_gids` as supplementary groups, and returns
/// the VMM's end of the connection along with the child's pid.
///
/// `params_json` is whatever the subcommand's `--config-json` expects; the caller serialises it,
/// and must strip anything that would make the child try to spawn a backend of its own. The log
/// filter is whatever `set_vmm_log_filter` recorded, and is omitted if nothing did (D57).
///
/// `pool_fd` is a second descriptor for the child, inherited by number exactly like the
/// vhost-user socket (its CLOEXEC is cleared here): media's tube to the VMM's `media_host` pool
/// allocator. The caller keeps its own end and its own copy of this one -- dropping the copy
/// after the spawn is the caller's job, so that the child's death is an EOF and not a
/// half-open socket the VMM itself keeps alive.
pub fn launch(
    subcommand: &str,
    params_json: String,
    uid: u32,
    gid: u32,
    supp_gids: Vec<u32>,
    pool_fd: Option<std::os::fd::RawFd>,
) -> Result<(UnixStream, Pid)> {
    let (vmm_end, backend_end) = UnixStream::pair()
        .with_context(|| format!("failed to create the {subcommand} socketpair"))?;
    let backend_fd = backend_end.as_raw_fd();
    // The child is handed this fd by number, so it has to survive the exec.
    clear_cloexec(backend_fd)
        .with_context(|| format!("failed to clear CLOEXEC on the {subcommand} socket"))?;
    if let Some(pool_fd) = pool_fd {
        clear_cloexec(pool_fd)
            .with_context(|| format!("failed to clear CLOEXEC on the {subcommand} pool fd"))?;
    }

    // Build everything the child needs before forking, the group list included: after the fork
    // there is only one thread, and anything that takes an allocator lock another thread was
    // holding would deadlock.
    let fd_arg = backend_fd.to_string();
    let parent_pid = std::process::id();

    let mut command = Command::new("/proc/self/exe");
    // `--log-level` is a *top-level* crosvm option, so it has to precede the `device` subcommand;
    // argh stops taking global options once a subcommand name appears. The filter travels as one
    // argv element, which keeps a compound filter (`"info,base=debug"`) intact -- there is no
    // shell here, and crosvm's own arg preprocessing only splits `--flag=value`, which this is
    // not. If the VMM never recorded a filter -- a unit test, or a path that launches a helper
    // before `init_log` -- nothing is passed and the child keeps the `info` default (D57).
    if let Some(filter) = VMM_LOG_FILTER.get() {
        command.arg("--log-level").arg(filter);
    }
    command
        .arg("device")
        .arg(subcommand)
        .arg("--fd")
        .arg(&fd_arg)
        .arg("--config-json")
        .arg(&params_json);
    if let Some(pool_fd) = pool_fd {
        command.arg("--pool-fd").arg(pool_fd.to_string());
    }

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
            // setresuid, not setuid: setuid()'s kernel path only calls set_user() -- which
            // updates cred->user, the field commit_creds() gates the per-uid RLIMIT_NPROC
            // charge on -- inside its CAP_SETUID branch. A drop that reaches the saved-uid
            // path instead updates cred->ucounts but not cred->user, so the app uid's NPROC
            // counter is never incremented for this process yet is decremented when it exits;
            // it drifts until fork() for that uid fails and the app "won't open" until reboot.
            // setresuid() calls set_user() unconditionally on a real-uid change, keeping the
            // accounting consistent. All three ids go to uid, exactly as setuid did.
            if libc::setresuid(uid, uid, uid) != 0 {
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

    let child = command
        .spawn()
        .with_context(|| format!("failed to spawn the {subcommand} backend"))?;
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
