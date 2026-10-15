// Copyright 2026 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! Android display ownership. The exec'd sentinel owns restoration, including SIGKILL/OOM.
//! No shell or inherited DRM descriptors are needed; property overrides are restored.
use std::ffi::CString;
use std::fs::{self, File, OpenOptions};
use std::io::{BufRead, BufReader, Write};
use std::os::fd::{AsRawFd, FromRawFd};
use std::os::unix::fs::OpenOptionsExt;
use std::os::unix::process::CommandExt;
use std::process::{Child, ChildStdin, Command, Stdio};
use std::time::{Duration, Instant};

use anyhow::{bail, ensure, Context, Result};
use serde::{Deserialize, Serialize};

const COMPOSER: &str = "vendor.qti.hardware.display.composer";
const SF: &str = "surfaceflinger";
const SUSPEND: &str = "system_suspend";
const WAKE: &str = "crosvm_kms";

// Verified against warsaw OS4.0.0.7's actual hangdetect.ko, not the generic MTK ABI.
// HANG_KICK takes a scalar timeout in seconds, despite its _IOR encoding.
const HANG_KICK: u32 = 0x8004700a;
struct HangWatchdog {
    device: File,
    restore_seconds: Option<libc::c_ulong>,
}

impl HangWatchdog {
    fn open() -> Result<Self> {
        ensure!(
            property("ro.build.version.incremental")? == "OS4.0.0.7.XHPCNXM",
            "KMS hangdetect ABI has not been verified on this Android build"
        );
        // READ_ALL is non-destructive. The driver has no state-query ioctl: its
        // similarly named monitor_status ioctl only returns a userspace mailbox.
        // Fail closed unless a recent periodic kernel status is available.
        let mut log = vec![0u8; 4 * 1024 * 1024];
        // SAFETY: log is writable for the supplied length; action 3 does not clear it.
        let n = unsafe { libc::klogctl(3, log.as_mut_ptr().cast(), log.len() as _) };
        ensure!(
            n >= 0,
            "read hangdetect status: {}",
            std::io::Error::last_os_error()
        );
        let log = String::from_utf8_lossy(&log[..n as usize]);
        let line = log
            .lines()
            .rev()
            .find(|l| l.contains("hang_detect thread counts down "))
            .context("no hangdetect status; refusing to freeze Android")?;
        let timestamp: f64 = line
            .split_once('[')
            .context("kernel timestamp")?
            .1
            .split_once(']')
            .context("kernel timestamp end")?
            .0
            .trim()
            .parse()?;
        let uptime: f64 = fs::read_to_string("/proc/uptime")?
            .split_whitespace()
            .next()
            .context("uptime")?
            .parse()?;
        ensure!(
            (0.0..=45.0).contains(&(uptime - timestamp)),
            "stale hangdetect status"
        );
        let status = line
            .split_once("counts down ")
            .context("hangdetect counter")?
            .1;
        let (counts, state) = status.split_once(", status ").context("hangdetect state")?;
        let (remaining, total) = counts.split_once(':').context("hangdetect timeout")?;
        let remaining: u32 = remaining.parse()?;
        let total: u32 = total.parse()?;
        let restore_seconds = if state.starts_with("0.") {
            0
        } else {
            ensure!(
                state.starts_with("1.") && remaining >= 3 && (3..=120).contains(&total),
                "unsafe or unsupported hangdetect counter: {status}"
            );
            (total * 30) as libc::c_ulong
        };
        Ok(Self {
            device: OpenOptions::new()
                .read(true)
                .write(true)
                .open("/dev/Monitor_Hang")?,
            restore_seconds: Some(restore_seconds),
        })
    }

    fn restore(&mut self) -> Result<()> {
        if let Some(seconds) = self.restore_seconds {
            self.kick(seconds)?;
            self.restore_seconds = None;
        }
        Ok(())
    }

    fn kick(&self, seconds: libc::c_ulong) -> Result<()> {
        // SAFETY: this verified ioctl accepts a scalar, not a pointer. Only 0 or
        // bounded positive values are used. argv[0] must pass the module's prefix check.
        let ret = unsafe { libc::ioctl(self.device.as_raw_fd(), HANG_KICK as _, seconds) };
        ensure!(
            ret == 0,
            "hangdetect kick: {}",
            std::io::Error::last_os_error()
        );
        Ok(())
    }
}

impl Drop for HangWatchdog {
    fn drop(&mut self) {
        if let Err(e) = self.restore() {
            eprintln!("kms-sentinel: failed to restore hangdetect: {e:#}");
        }
    }
}

#[derive(Debug, Serialize, Deserialize)]
struct Snapshot {
    parent: i32,
    server: i32,
    server_start: String,
    suspend_running: bool,
    secondary_running: bool,
    stability: Vec<(String, String)>,
}

fn property(key: &str) -> Result<String> {
    let key = CString::new(key)?;
    let mut value = [0 as libc::c_char; 92];
    // SAFETY: property_get writes at most PROP_VALUE_MAX bytes into value.
    let n = unsafe { libc::__system_property_get(key.as_ptr(), value.as_mut_ptr()) };
    ensure!(
        n >= 0 && (n as usize) < value.len(),
        "invalid property length"
    );
    Ok(String::from_utf8(
        value[..n as usize].iter().map(|b| *b as u8).collect(),
    )?)
}

fn set_property(key: &str, value: &str) -> Result<()> {
    let key = CString::new(key)?;
    let value = CString::new(value)?;
    // SAFETY: both arguments are NUL-terminated strings valid for this call.
    ensure!(
        unsafe { libc::__system_property_set(key.as_ptr(), value.as_ptr()) } == 0,
        "setting property {:?} failed",
        key
    );
    Ok(())
}

fn wait_service(name: &str, state: &str) -> Result<()> {
    let until = Instant::now() + Duration::from_secs(8);
    loop {
        if property(&format!("init.svc.{name}"))? == state {
            return Ok(());
        }
        ensure!(
            Instant::now() < until,
            "service {name} did not become {state}"
        );
        std::thread::sleep(Duration::from_millis(50));
    }
}

fn service(name: &str, start: bool) -> Result<()> {
    set_property(if start { "ctl.start" } else { "ctl.stop" }, name)?;
    wait_service(name, if start { "running" } else { "stopped" })
}

fn process_start(pid: i32) -> Result<String> {
    let stat = fs::read_to_string(format!("/proc/{pid}/stat"))?;
    // comm can contain spaces and parentheses. Field 22 follows the final closing parenthesis.
    Ok(stat
        .rsplit_once(") ")
        .context("invalid proc stat")?
        .1
        .split_whitespace()
        .nth(19)
        .context("missing start time")?
        .to_owned())
}

fn snapshot() -> Result<Snapshot> {
    ensure!(
        property(&format!("init.svc.{SF}"))? == "running",
        "SurfaceFlinger is not running"
    );
    ensure!(
        property(&format!("init.svc.{COMPOSER}"))? == "running",
        "composer is not running"
    );
    let mut servers = Vec::new();
    for entry in fs::read_dir("/proc")? {
        let entry = entry?;
        let Some(pid) = entry
            .file_name()
            .to_str()
            .and_then(|s| s.parse::<i32>().ok())
        else {
            continue;
        };
        if fs::read_to_string(entry.path().join("comm"))
            .unwrap_or_default()
            .trim()
            == "system_server"
        {
            servers.push(pid);
        }
    }
    ensure!(servers.len() == 1, "expected exactly one system_server");
    let server = servers[0];
    let status = fs::read_to_string(format!("/proc/{server}/status"))?;
    ensure!(
        !status
            .lines()
            .any(|s| s.starts_with("State:") && (s.contains("T (") || s.contains("t ("))),
        "system_server was already stopped"
    );
    let mut stability = Vec::new();
    for name in ["scout", "nativehang", "nativehangII", "qcom_hang_task"] {
        let key = format!("persist.sys.stability.{name}.enable");
        stability.push((key.clone(), property(&key)?));
    }
    Ok(Snapshot {
        parent: std::process::id() as i32,
        server,
        server_start: process_start(server)?,
        secondary_running: property("init.svc.zygote_secondary")? == "running",
        suspend_running: property(&format!("init.svc.{SUSPEND}"))? == "running",
        stability,
    })
}

fn signal_server(s: &Snapshot, signal: i32) -> Result<()> {
    ensure!(
        process_start(s.server)? == s.server_start,
        "system_server identity changed"
    );
    // SAFETY: positive PID verified against its saved start time; signal is STOP or CONT.
    ensure!(
        unsafe { libc::kill(s.server, signal) } == 0,
        "signal system_server failed"
    );
    Ok(())
}

fn restore(s: &Snapshot) -> Result<()> {
    // Try every step even when an earlier one fails; never leave the framework frozen just
    // because a display HAL could not restart. The sentinel reports any failed step.
    let mut failures = Vec::new();
    let mut attempt = |label: &str, result: Result<()>| {
        if let Err(e) = result {
            failures.push(format!("{label}: {e:#}"));
        }
    };
    // SF cannot safely reuse this framework's old layer handles. Explicitly
    // rebuild Android userspace on return; the host kernel and sentinel survive.
    if s.secondary_running {
        attempt("stop secondary zygote", service("zygote_secondary", false));
    }
    let stopped = service("zygote", false);
    if stopped.is_err() {
        // Do not leave the old framework frozen if init refused the stop.
        attempt("resume old system_server", signal_server(s, libc::SIGCONT));
    }
    attempt("stop zygote", stopped);
    attempt("old framework exit", (|| -> Result<()> {
        let until = Instant::now() + Duration::from_secs(8);
        while process_start(s.server).ok().as_deref() == Some(s.server_start.as_str()) {
            ensure!(Instant::now() < until, "old system_server did not exit");
            std::thread::sleep(Duration::from_millis(50));
        }
        Ok(())
    })());
    attempt("composer", service(COMPOSER, true));
    attempt("surfaceflinger", service(SF, true));
    attempt("start zygote", service("zygote", true));
    if s.secondary_running {
        attempt("start secondary zygote", service("zygote_secondary", true));
    }
    attempt(
        "wake unlock",
        fs::write("/sys/power/wake_unlock", WAKE).map_err(Into::into),
    );
    if s.suspend_running {
        attempt("system_suspend", service(SUSPEND, true));
    }
    for (key, value) in &s.stability {
        attempt(key, set_property(key, value));
    }
    ensure!(
        failures.is_empty(),
        "restoration failed: {}",
        failures.join("; ")
    );
    Ok(())
}

// "running" only means init forked a process. Require a new framework, its
// compositor boot handshake, and ten seconds without a display/framework restart.
fn verify_restoration(s: &Snapshot, watchdog: &HangWatchdog) -> Result<()> {
    let sf_pid = property("init.svc_debug_pid.surfaceflinger")?;
    let composer_pid = property(&format!("init.svc_debug_pid.{COMPOSER}"))?;
    ensure!(
        !sf_pid.is_empty() && !composer_pid.is_empty(),
        "missing restored service PID"
    );
    let deadline = Instant::now() + Duration::from_secs(90);
    let mut stable: Option<(i32, String, Instant)> = None;
    loop {
        watchdog.kick(120)?;
        ensure!(
            property(&format!("init.svc.{SF}"))? == "running"
                && property(&format!("init.svc.{COMPOSER}"))? == "running"
                && property("init.svc_debug_pid.surfaceflinger")? == sf_pid
                && property(&format!("init.svc_debug_pid.{COMPOSER}"))? == composer_pid,
            "Android display service died/restarted during restoration"
        );
        let mut server = None;
        for entry in fs::read_dir("/proc")? {
            let entry = entry?;
            if fs::read_to_string(entry.path().join("comm"))
                .unwrap_or_default()
                .trim()
                == "system_server"
            {
                if let Some(pid) = entry
                    .file_name()
                    .to_str()
                    .and_then(|p| p.parse::<i32>().ok())
                {
                    if let Ok(start) = process_start(pid) {
                        server = Some((pid, start));
                    }
                }
            }
        }
        let ready = property("service.bootanim.exit")? == "1"
            && property("init.svc.bootanim")? == "stopped"
            && property("sys.boot_completed")? == "1";
        if let Some((pid, start)) = server {
            ensure!(
                pid != s.server || start != s.server_start,
                "old system_server survived framework restart"
            );
            if let Some((expected_pid, expected_start, since)) = &stable {
                ensure!(
                    pid == *expected_pid && start == *expected_start && ready,
                    "Android framework became unstable after boot"
                );
                if since.elapsed() >= Duration::from_secs(10) {
                    return Ok(());
                }
            } else if ready {
                stable = Some((pid, start, Instant::now()));
            }
        } else {
            ensure!(stable.is_none(), "Android framework exited after boot");
        }
        ensure!(
            Instant::now() < deadline,
            "Android framework boot did not complete within 90 seconds"
        );
        std::thread::sleep(Duration::from_secs(1));
    }
}

pub(crate) struct Takeover {
    command: Option<ChildStdin>,
    sentinel: Child,
}

impl Takeover {
    pub(crate) fn begin() -> Result<Self> {
        let s = snapshot()?;
        let mut sentinel = Command::new(std::env::current_exe()?)
            .arg0("/system/bin/crosvm-kms-sentinel")
            .arg("kms-sentinel")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .stderr(Stdio::inherit())
            .spawn()
            .context("spawn KMS sentinel")?;
        let mut command = sentinel.stdin.take().context("sentinel stdin")?;
        serde_json::to_writer(&mut command, &s)?;
        command.write_all(b"\n")?;
        command.flush()?;
        let ready = sentinel.stdout.take().context("sentinel stdout")?;
        let mut poll = libc::pollfd {
            fd: ready.as_raw_fd(),
            events: libc::POLLIN,
            revents: 0,
        };
        // SAFETY: valid single-element pollfd array, bounded startup handshake.
        let n = unsafe { libc::poll(&mut poll, 1, 5000) };
        let mut line = String::new();
        if n <= 0 || BufReader::new(ready).read_line(&mut line)? == 0 || line != "ready\n" {
            drop(command);
            let _ = sentinel.wait();
            bail!("KMS sentinel did not become ready");
        }
        let guard = Self {
            command: Some(command),
            sentinel,
        };
        // Restoration is armed before the first mutation. A partial failure drops the guard.
        fs::write("/sys/power/wake_lock", WAKE).context("acquire KMS wake lock")?;
        if s.suspend_running {
            service(SUSPEND, false)?;
        }
        for (key, _) in &s.stability {
            set_property(key, "false")?;
        }
        signal_server(&s, libc::SIGSTOP)?;
        service(SF, false)?;
        // Stopping SF can restart the HAL; wait for init to settle before stopping it.
        std::thread::sleep(Duration::from_secs(2));
        service(COMPOSER, false)?;
        base::info!(
            "kms: Android display stopped; framework will restart on exit; sentinel pid={}",
            guard.sentinel.id()
        );
        Ok(guard)
    }
}

impl Drop for Takeover {
    fn drop(&mut self) {
        if let Some(mut command) = self.command.take() {
            let _ = command.write_all(b"restore\n");
        }
        match self.sentinel.wait() {
            Ok(status) if status.success() => base::info!("kms: Android display restored"),
            result => base::error!("kms: sentinel restoration result: {:?}", result),
        }
    }
}

/// Invoked before ordinary command parsing, only on Android builds with KMS support.
pub(crate) fn sentinel_main() -> Result<()> {
    // Detach from the launching terminal/process group; the sentinel must survive parent death.
    // SAFETY: setsid takes no pointers and affects only this fresh exec'd process.
    ensure!(unsafe { libc::setsid() } >= 0, "sentinel setsid failed");
    let lock = OpenOptions::new()
        .read(true)
        .write(true)
        .create(true)
        .mode(0o600)
        .custom_flags(libc::O_NOFOLLOW | libc::O_CLOEXEC)
        .open("/data/local/tmp/crosvm-kms.lock")?;
    // SAFETY: lock is a live file descriptor. Never let two sentinels restore over each other.
    ensure!(
        unsafe { libc::flock(lock.as_raw_fd(), libc::LOCK_EX | libc::LOCK_NB) } == 0,
        "another KMS session owns the display"
    );
    let mut input = BufReader::new(std::io::stdin());
    let mut line = String::new();
    input.read_line(&mut line)?;
    let s: Snapshot = serde_json::from_str(&line)?;
    // SAFETY: getppid has no arguments.
    ensure!(
        s.parent == unsafe { libc::getppid() },
        "sentinel parent mismatch"
    );
    // pidfd avoids dependence on pipe EOF: forked VMM children might retain a pipe writer.
    // SAFETY: pidfd_open takes a PID and flags, and returns a new owned fd on success.
    let fd = unsafe { libc::syscall(libc::SYS_pidfd_open, s.parent, 0) } as i32;
    ensure!(fd >= 0, "pidfd_open: {}", std::io::Error::last_os_error());
    // SAFETY: fd is newly returned by pidfd_open and owned by this process.
    let parent = unsafe { File::from_raw_fd(fd) };
    let mut watchdog = HangWatchdog::open()?;
    watchdog.kick(120)?;
    println!("ready");
    std::io::stdout().flush()?;
    let mut polls = [
        libc::pollfd {
            fd: parent.as_raw_fd(),
            events: libc::POLLIN,
            revents: 0,
        },
        libc::pollfd {
            fd: 0,
            events: libc::POLLIN,
            revents: 0,
        },
    ];
    loop {
        // SAFETY: polls has exactly two initialized entries.
        let n = unsafe { libc::poll(polls.as_mut_ptr(), polls.len() as _, 10_000) };
        if n < 0 && std::io::Error::last_os_error().kind() == std::io::ErrorKind::Interrupted {
            continue;
        }
        if n == 0 {
            if let Err(e) = watchdog.kick(120) {
                eprintln!("kms-sentinel: {e:#}; terminating VM to restore Android");
                // SAFETY: pidfd_send_signal targets the same parent even if its PID is reused.
                unsafe {
                    libc::syscall(
                        libc::SYS_pidfd_send_signal,
                        parent.as_raw_fd(),
                        libc::SIGKILL,
                        0usize,
                        0,
                    );
                }
                break;
            }
            continue;
        }
        break;
    }
    let restored = restore(&s).and_then(|_| verify_restoration(&s, &watchdog));
    let watchdog_restored = watchdog.restore();
    restored?;
    watchdog_restored?;
    eprintln!("kms-sentinel: Android display restored");
    Ok(())
}
