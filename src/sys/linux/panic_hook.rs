// Copyright 2019 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

use std::env;
use std::fs::File;
use std::io::stderr;
use std::io::Read;
use std::panic;
use std::panic::PanicInfo;
use std::process::abort;
use std::string::String;

use base::error;
use base::FromRawDescriptor;
use base::IntoRawDescriptor;
use libc::close;
use libc::dup;
use libc::dup2;
use libc::pipe2;
use libc::O_NONBLOCK;
use libc::STDERR_FILENO;

// Opens a pipe and puts the write end into the stderr FD slot. On success, returns the read end of
// the pipe and the old stderr as a pair of files.
//
// This may fail if, for example, the file descriptor numbers have been exhausted.
fn redirect_stderr() -> Option<(File, File)> {
    let mut fds = [-1, -1];
    // SAFETY: Trivially safe because the return value is checked.
    unsafe {
        let old_stderr = dup(STDERR_FILENO);
        if old_stderr == -1 {
            return None;
        }
        // Safe because pipe2 will only ever write two integers to our array and we check output.
        let mut ret = pipe2(fds.as_mut_ptr(), O_NONBLOCK);
        if ret != 0 {
            // Leaks FDs, but not important right before abort.
            return None;
        }
        // Safe because the FD we are duplicating is owned by us.
        ret = dup2(fds[1], STDERR_FILENO);
        if ret == -1 {
            // Leaks FDs, but not important right before abort.
            return None;
        }
        // The write end is no longer needed.
        close(fds[1]);
        // Safe because each of the fds was the result of a successful FD creation syscall.
        Some((
            File::from_raw_descriptor(fds[0]),
            File::from_raw_descriptor(old_stderr),
        ))
    }
}

// Sets stderr to the given file. Returns true on success.
fn restore_stderr(stderr: File) -> bool {
    let descriptor = stderr.into_raw_descriptor();

    // SAFETY:
    // Safe because descriptor is guaranteed to be valid and replacing stderr
    // should be an atomic operation.
    unsafe { dup2(descriptor, STDERR_FILENO) != -1 }
}

// Sends as much information about the panic as possible to syslog.
fn log_panic_info(default_panic: &(dyn Fn(&PanicInfo) + Sync + Send + 'static), info: &PanicInfo) {
    // Grab a lock of stderr to prevent concurrent threads from trampling on our stderr capturing
    // procedure. The default_panic procedure likely uses stderr.lock as well, but the mutex inside
    // stderr is reentrant, so it will not dead-lock on this thread.
    let stderr = stderr();
    let _stderr_lock = stderr.lock();

    // Redirect stderr to a pipe we can read from later.
    let (mut read_file, old_stderr) = match redirect_stderr() {
        Some(f) => f,
        None => {
            error!(
                "failed to capture stderr during panic: {}",
                std::io::Error::last_os_error()
            );
            // Fallback to stderr only panic logging.
            env::set_var("RUST_BACKTRACE", "1");
            default_panic(info);
            return;
        }
    };
    // Only through the default panic handler can we get a stacktrace. It only ever prints to
    // stderr, hence all the previous code to redirect it to a pipe we can read.
    env::set_var("RUST_BACKTRACE", "1");
    default_panic(info);

    // Closes the write end of the pipe so that we can reach EOF in read_to_string. Also allows
    // others to write to stderr without failure.
    if !restore_stderr(old_stderr) {
        error!("failed to restore stderr during panic");
        return;
    }
    drop(_stderr_lock);

    let mut panic_output = String::new();
    // Ignore errors and print what we got.
    let _ = read_file.read_to_string(&mut panic_output);
    // Split by line because the logging facilities do not handle embedded new lines well.
    for line in panic_output.lines() {
        error!("{}", line);
    }
}

/// The intent of our panic hook is to get panic info and a stacktrace into the syslog, even for
/// jailed subprocesses. It will always abort on panic to ensure a minidump is generated.
///
/// Note that jailed processes will usually have a stacktrace of <unknown> because the backtrace
/// routines attempt to open this binary and are unable to do so in a jail.
pub fn set_panic_hook() {
    let default_panic = panic::take_hook();
    panic::set_hook(Box::new(move |info| {
        log_panic_info(default_panic.as_ref(), info);
        // Abort to trigger the crash reporter so that a minidump is generated.
        abort();
    }));
}

// DroidVM: dedicated alternate signal stack for the crash handler so it can still unwind even on a
// stack overflow. 256 KiB is plenty for backtrace capture.
const CRASH_ALTSTACK_SIZE: usize = 256 * 1024;
static mut CRASH_ALTSTACK: [u8; CRASH_ALTSTACK_SIZE] = [0u8; CRASH_ALTSTACK_SIZE];

fn write_all_stderr(bytes: &[u8]) {
    let mut off = 0;
    while off < bytes.len() {
        // SAFETY: writing a slice of our own buffer to the stderr fd.
        let n = unsafe {
            libc::write(
                STDERR_FILENO,
                bytes[off..].as_ptr() as *const libc::c_void,
                bytes.len() - off,
            )
        };
        if n <= 0 {
            break;
        }
        off += n as usize;
    }
}

/// DroidVM crash handler for fatal, non-panic signals (SIGSEGV/SIGBUS/SIGILL/SIGFPE/SIGABRT).
///
/// Rust panics already go through `set_panic_hook`, but a raw SIGSEGV (e.g. a null deref in native
/// C++ display glue or in the gfxstream backend, or during device teardown on VM stop) bypasses it
/// entirely. On Android debuggerd normally captures such crashes into a tombstone, but that path
/// forks a `crash_dump` helper via `clone()`, which fails with EAGAIN ("second clone failed") when
/// the process is under teardown resource pressure -- leaving no backtrace at all. This handler runs
/// *in-process* on an alternate stack, so it needs no fork/clone and prints a symbolized backtrace
/// straight to stderr (captured by the app's console log) before letting the default disposition
/// (re-)raise for the tombstone. `SA_RESETHAND` guarantees the second fault terminates the process.
/// Parse one /proc/self/maps line into (start, end, perms, path).
fn parse_maps_line(line: &str) -> Option<(usize, usize, &str, &str)> {
    let mut it = line.split_whitespace();
    let range = it.next()?;
    let perms = it.next().unwrap_or("");
    let _off = it.next();
    let _dev = it.next();
    let _inode = it.next();
    let path = it.next().unwrap_or("");
    let mut halves = range.split('-');
    let start = usize::from_str_radix(halves.next().unwrap_or(""), 16).ok()?;
    let end = usize::from_str_radix(halves.next().unwrap_or(""), 16).ok()?;
    Some((start, end, perms, path))
}

/// Given a runtime address, find the module (file) it maps into and return "path @vaddr 0x.." where
/// vaddr = addr - module_load_base (the lowest mapping of that same file = the PIE/ASLR load bias).
/// That vaddr feeds straight into `addr2line -f -e <path> 0x<vaddr>` offline. Empty if not found.
fn addr_to_module_offset(maps: &str, addr: usize) -> String {
    // First: which file-backed executable mapping contains addr?
    let mut hit_path = "";
    for line in maps.lines() {
        if let Some((start, end, perms, path)) = parse_maps_line(line) {
            if addr >= start && addr < end && perms.contains('x') && path.starts_with('/') {
                hit_path = path;
                break;
            }
        }
    }
    if hit_path.is_empty() {
        return String::new();
    }
    // Second: the load base is the lowest start among all mappings of that same file.
    let mut base = usize::MAX;
    for line in maps.lines() {
        if let Some((start, _end, _perms, path)) = parse_maps_line(line) {
            if path == hit_path && start < base {
                base = start;
            }
        }
    }
    format!("{} @vaddr 0x{:x}", hit_path, addr.wrapping_sub(base))
}

extern "C" fn crash_signal_handler(
    signum: libc::c_int,
    info: *mut libc::siginfo_t,
    ucontext: *mut libc::c_void,
) {
    // Best-effort fault address from siginfo.
    let fault_addr = if info.is_null() {
        0usize
    } else {
        // SAFETY: kernel-provided siginfo for a memory-fault signal; si_addr is valid to read.
        unsafe { (*info).si_addr() as usize }
    };
    // SAFETY: gettid is always safe.
    let tid = unsafe { libc::gettid() };

    // Pull the faulting PC and link register straight out of the signal machine context. This is the
    // only reliable way to locate the crash: std::backtrace unwinds the *handler's* stack and cannot
    // cross the signal trampoline, so it just yields <unknown> handler frames. We read the arm64
    // kernel ucontext at fixed offsets (the libc crate's ucontext_t omits the 120-byte sigmask
    // padding, so its uc_mcontext offset is wrong).
    let (mc_fault, lr, sp, pc) = unsafe { crash_regs(ucontext) };

    // Read our own memory map so the raw runtime addresses can be turned into file offsets for
    // addr2line. read_to_string allocates, but for a clean null deref the allocator is intact.
    let maps = std::fs::read_to_string("/proc/self/maps").unwrap_or_default();

    let mut out = String::with_capacity(768);
    out.push_str("\n==== DroidVM crosvm crash handler ====\n");
    out.push_str(&format!(
        "fatal signal {} tid {} fault_addr 0x{:x} (mc.fault 0x{:x})\n",
        signum, tid, fault_addr, mc_fault
    ));
    out.push_str(&format!("pc 0x{:x}  {}\n", pc, addr_to_module_offset(&maps, pc)));
    out.push_str(&format!("lr 0x{:x}  {}\n", lr, addr_to_module_offset(&maps, lr)));
    out.push_str(&format!("sp 0x{:x}\n", sp));
    // Raw dump of the first 34 machine-context u64 words (fault_address, x0..x30, sp, pc, pstate)
    // as a safety net in case the fixed offsets are off; each code-range word is resolved.
    out.push_str("mcontext words (off:val [module@vaddr]):\n");
    if !ucontext.is_null() {
        for i in 0..44usize {
            let off = 168 + i * 8; // dump wide; real uc_mcontext base = 176 here (see crash_regs)
            let w = unsafe { read_u64_at(ucontext, off) } as usize;
            let sym = addr_to_module_offset(&maps, w);
            if sym.is_empty() {
                out.push_str(&format!("  +{}: 0x{:x}\n", off, w));
            } else {
                out.push_str(&format!("  +{}: 0x{:x}  {}\n", off, w, sym));
            }
        }
    }
    out.push_str("(symbolize: llvm-symbolizer -e <module> <vaddr>)\n");
    out.push_str("==== end crash handler ====\n");
    write_all_stderr(out.as_bytes());
}

/// Read a u64 at byte `off` from the ucontext pointer (unaligned-safe).
unsafe fn read_u64_at(ucontext: *mut libc::c_void, off: usize) -> u64 {
    let p = (ucontext as *const u8).add(off) as *const u64;
    p.read_unaligned()
}

/// Extract (fault_address, lr, sp, pc) from an arm64 signal ucontext using fixed kernel offsets.
/// uc_mcontext base = 168; struct sigcontext = { fault_address@0, regs[31]@8.., sp@256, pc@264 }.
unsafe fn crash_regs(ucontext: *mut libc::c_void) -> (usize, usize, usize, usize) {
    if ucontext.is_null() {
        return (0, 0, 0, 0);
    }
    #[cfg(target_arch = "aarch64")]
    {
        // Empirically uc_mcontext (struct sigcontext) sits at ucontext offset 176 on this
        // bionic/arm64 (validated: regs[30] at +424 resolves to a stable code address).
        const MC: usize = 176;
        let fault = read_u64_at(ucontext, MC) as usize; // fault_address
        let lr = read_u64_at(ucontext, MC + 8 + 30 * 8) as usize; // regs[30] @ +424
        let sp = read_u64_at(ucontext, MC + 8 + 31 * 8) as usize; // sp @ +432
        let pc = read_u64_at(ucontext, MC + 8 + 31 * 8 + 8) as usize; // pc @ +440
        (fault, lr, sp, pc)
    }
    #[cfg(not(target_arch = "aarch64"))]
    {
        let _ = ucontext;
        (0, 0, 0, 0)
    }
}

/// Installs [`crash_signal_handler`] for the fatal, non-panic signals. Idempotent-safe to call once
/// from `crosvm_main` right after `set_panic_hook`.
pub fn install_crash_handler() {
    // SAFETY: standard sigaltstack + sigaction setup with a statically-sized alt stack. All raw
    // pointers refer to stack-local or 'static storage that outlives the calls.
    unsafe {
        let mut ss: libc::stack_t = std::mem::zeroed();
        ss.ss_sp = std::ptr::addr_of_mut!(CRASH_ALTSTACK) as *mut libc::c_void;
        ss.ss_size = CRASH_ALTSTACK_SIZE;
        ss.ss_flags = 0;
        libc::sigaltstack(&ss, std::ptr::null_mut());

        let mut act: libc::sigaction = std::mem::zeroed();
        act.sa_sigaction = crash_signal_handler as *const () as libc::sighandler_t;
        act.sa_flags = libc::SA_SIGINFO | libc::SA_ONSTACK | libc::SA_RESETHAND;
        libc::sigemptyset(&mut act.sa_mask);

        // Only the hardware fault signals that bypass the Rust panic hook. SIGABRT is intentionally
        // excluded -- Rust panics abort() through set_panic_hook, which already logs a stacktrace.
        for &sig in &[libc::SIGSEGV, libc::SIGBUS, libc::SIGILL, libc::SIGFPE] {
            libc::sigaction(sig, &act, std::ptr::null_mut());
        }
    }
}
