// Copyright 2022 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#[cfg(target_arch = "x86_64")]
use std::arch::x86_64::__cpuid;
#[cfg(target_arch = "x86_64")]
use std::arch::x86_64::__cpuid_count;
use std::collections::BTreeMap;
use std::path::PathBuf;
use std::str::FromStr;
use std::time::Duration;

use arch::set_default_serial_parameters;
use arch::CpuSet;
use arch::FdtPosition;
use arch::PciConfig;
use arch::Pstore;
use arch::SmbiosOptions;
#[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
use arch::SveConfig;
use arch::VcpuAffinity;
use base::debug;
use base::pagesize;
use cros_async::ExecutorKind;
use devices::serial_device::SerialHardware;
use devices::serial_device::SerialParameters;
use devices::virtio::block::DiskOption;
#[cfg(any(feature = "video-decoder", feature = "video-encoder"))]
use devices::virtio::device_constants::video::VideoDeviceConfig;
#[cfg(feature = "gpu")]
use devices::virtio::gpu::GpuParameters;
use devices::virtio::scsi::ScsiOption;
#[cfg(feature = "audio")]
use devices::virtio::snd::parameters::Parameters as SndParameters;
#[cfg(all(windows, feature = "gpu"))]
use devices::virtio::vhost::user::device::gpu::sys::windows::GpuBackendConfig;
#[cfg(all(windows, feature = "gpu"))]
use devices::virtio::vhost::user::device::gpu::sys::windows::GpuVmmConfig;
#[cfg(all(windows, feature = "gpu"))]
use devices::virtio::vhost::user::device::gpu::sys::windows::InputEventSplitConfig;
#[cfg(all(windows, feature = "gpu"))]
use devices::virtio::vhost::user::device::gpu::sys::windows::WindowProcedureThreadSplitConfig;
#[cfg(all(windows, feature = "audio"))]
use devices::virtio::vhost::user::device::snd::sys::windows::SndSplitConfig;
use devices::virtio::vsock::VsockConfig;
use devices::virtio::DeviceType;
#[cfg(feature = "net")]
use devices::virtio::NetParameters;
use devices::FwCfgParameters;
use devices::PciAddress;
use devices::PflashParameters;
use devices::StubPciParameters;
#[cfg(target_arch = "x86_64")]
use hypervisor::CpuHybridType;
use hypervisor::LendMthpMode;
use hypervisor::ProtectionType;
use jail::JailConfig;
use resources::AddressRange;
use serde::Deserialize;
use serde::Deserializer;
use serde::Serialize;
use serde_keyvalue::FromKeyValues;
use vm_control::BatteryType;
use vm_memory::FileBackedMappingParameters;
#[cfg(target_arch = "x86_64")]
use x86_64::check_host_hybrid_support;
#[cfg(target_arch = "x86_64")]
use x86_64::CpuIdCall;

pub(crate) use super::sys::HypervisorKind;
#[cfg(any(target_os = "android", target_os = "linux"))]
use crate::crosvm::sys::config::SharedDir;

cfg_if::cfg_if! {
    if #[cfg(any(target_os = "android", target_os = "linux"))] {
        #[cfg(feature = "gpu")]
        use crate::crosvm::sys::GpuRenderServerParameters;

        #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
        static VHOST_SCMI_PATH: &str = "/dev/vhost-scmi";
    } else if #[cfg(windows)] {
        use base::{Event, Tube};
    }
}

// by default, if enabled, the balloon WS features will use 4 bins.
#[cfg(feature = "balloon")]
const VIRTIO_BALLOON_WS_DEFAULT_NUM_BINS: u8 = 4;

/// Indicates the location and kind of executable kernel for a VM.
#[allow(dead_code)]
#[derive(Debug, Serialize, Deserialize)]
pub enum Executable {
    /// An executable intended to be run as a BIOS directly.
    Bios(PathBuf),
    /// A elf linux kernel, loaded and executed by crosvm.
    Kernel(PathBuf),
    /// Path to a plugin executable that is forked by crosvm.
    Plugin(PathBuf),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, FromKeyValues)]
#[serde(deny_unknown_fields, rename_all = "kebab-case")]
pub enum IrqChipKind {
    /// All interrupt controllers are emulated in the kernel.
    Kernel,
    /// APIC is emulated in the kernel.  All other interrupt controllers are in userspace.
    Split,
    /// All interrupt controllers are emulated in userspace.
    Userspace,
}

/// The core types in hybrid architecture.
#[cfg(target_arch = "x86_64")]
#[derive(Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(deny_unknown_fields, rename_all = "kebab-case")]
pub struct CpuCoreType {
    /// Intel Atom.
    pub atom: CpuSet,
    /// Intel Core.
    pub core: CpuSet,
}

#[derive(Debug, Default, PartialEq, Eq, Deserialize, Serialize, FromKeyValues)]
#[serde(deny_unknown_fields, rename_all = "kebab-case")]
pub struct CpuOptions {
    /// Number of CPU cores.
    #[serde(default)]
    pub num_cores: Option<usize>,
    /// Vector of CPU ids to be grouped into the same cluster.
    #[serde(default)]
    pub clusters: Vec<CpuSet>,
    /// Core Type of CPUs.
    #[cfg(target_arch = "x86_64")]
    pub core_types: Option<CpuCoreType>,
    /// Select which CPU to boot from.
    #[serde(default)]
    pub boot_cpu: Option<usize>,
    /// Vector of CPU ids to be grouped into the same freq domain.
    #[serde(default)]
    pub freq_domains: Vec<CpuSet>,
    /// Scalable Vector Extension.
    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    pub sve: Option<SveConfig>,
}

/// The screen an exporter (a VNC server, an Android display service) binds to.
///
/// A screen is whoever is producing pictures, and the simplefb device and the virtio-gpu device
/// are two parallel display devices each providing one -- not two sources contending for a single
/// output. An exporter binds to exactly one screen, and a screen carries at most one exporter.
/// There is deliberately no "both" and no "any": mirroring is a non-goal, and letting the VMM pick
/// for the user is what produced the silent race this replaces, where a VNC server and an Android
/// display service configured together left the app's Surface waiting forever on a binder that was
/// never registered.
#[cfg(any(feature = "vnc", feature = "android_display"))]
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum DisplayScreen {
    /// virtio-gpu scanout 0.
    ///
    /// Spelled with its scanout index because multi-scanout is a path left open rather than one
    /// taken: `gpu-1` and up do not exist until more than one scanout is enabled, and nothing
    /// here should read as if scanout 0 were the only one there could be.
    #[serde(rename = "gpu-0")]
    Gpu0,
    /// The simplefb device's single screen, whose geometry the device tree fixes.
    #[serde(rename = "simplefb")]
    Simplefb,
}

#[cfg(any(feature = "vnc", feature = "android_display"))]
impl DisplayScreen {
    /// The name this screen is written as on the command line, for error messages that have to
    /// quote back what the user typed (or what the compat default resolved to).
    pub fn as_str(self) -> &'static str {
        match self {
            DisplayScreen::Gpu0 => "gpu-0",
            DisplayScreen::Simplefb => "simplefb",
        }
    }

    /// The option that has to be present for this screen to exist at all.
    fn source_option(self) -> &'static str {
        match self {
            DisplayScreen::Gpu0 => "--gpu",
            DisplayScreen::Simplefb => "--simplefb",
        }
    }
}

/// How far up the transport ladder a screen's exporter is allowed to go.
///
/// A CAP, never a request, and the distinction is the whole reason this is expressible at all.
/// Which transport a frame actually takes is negotiated at run time between what the source can
/// export and what the sink can import (CPU copy always available, GPU copy when both ends manage
/// a dmabuf), and a user who *asks* for GPU copy on a source that cannot export one gets either a
/// silent downgrade or a loud refusal -- the first is the "looks like it worked" failure this
/// project keeps running into, the second turns a preference into a boot failure. A ceiling has
/// neither problem: every value is satisfiable, because lowering the ceiling can only remove
/// options from a negotiation that always has CPU copy at the bottom.
///
/// This is a debugging control, not a product setting (plan §4.6). What a panel shows the user is
/// the negotiated *result*, read-only.
#[cfg(any(feature = "vnc", feature = "android_display"))]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum TransportCap {
    /// Take whatever the two ends negotiate.
    #[default]
    Auto,
    /// Refuse dmabuf import on this binding, so the negotiation can only land on a CPU copy. The
    /// per-binding equivalent of the process-wide `GPU_SCANOUT_FORCE_TRANSFER=1`.
    Cpu,
    /// Allow the GPU blit and stop there: no hardware video encoder on this binding, whatever the
    /// device could do. This is the rung the app has been able to say since before there was
    /// anything above it, and it means what it always meant -- the difference is that there is now
    /// a rung above it for it to be below.
    Gpu,
    /// Allow the GPU blit and the hardware encoder above it: the VNC sink's H.264 stream may come
    /// up on this binding. Distinct from `auto` only in intent -- `auto` also permits it --
    /// so that a caller can pin the ceiling here and have the meaning survive a future rung being
    /// added on top.
    GpuHw,
}

#[cfg(any(feature = "vnc", feature = "android_display"))]
impl TransportCap {
    /// Whether this ceiling leaves the dmabuf import available at all.
    ///
    /// Every value except `cpu` does. Written as a question rather than as `!= Cpu` at each call
    /// site so that adding a rung does not mean auditing the comparisons for the ones that meant
    /// "anything above the floor".
    pub fn allows_gpu_copy(self) -> bool {
        !matches!(self, TransportCap::Cpu)
    }

    /// Whether this ceiling leaves the hardware encoder available.
    ///
    /// `gpu` caps below it, which is the whole reason the value had to be spellable: the app
    /// already sends `transport-cap=gpu` for bindings that should blit but not encode, and if that
    /// were read as "anything the negotiation can reach" the encoder would come up on every one of
    /// them.
    pub fn allows_hw_encode(self) -> bool {
        matches!(self, TransportCap::Auto | TransportCap::GpuHw)
    }
}

/// Address a VNC server listens on when the option does not say. Kept here rather than repeated at
/// each consumer because validation has to compare the same effective port the server will bind.
#[cfg(feature = "vnc")]
pub const DEFAULT_VNC_HOST: &str = "0.0.0.0";
#[cfg(feature = "vnc")]
pub const DEFAULT_VNC_PORT: u32 = 5900;

/// One VNC server: where it listens, and which screen's frames it serves.
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize, FromKeyValues)]
#[serde(deny_unknown_fields, rename_all = "kebab-case")]
#[cfg(feature = "vnc")]
pub struct VncConfig {
    #[serde(default)]
    pub host: Option<String>,
    #[serde(default)]
    pub port: Option<u32>,
    #[serde(default)]
    pub password: Option<String>,
    /// Whether this binding's clients only watch.
    ///
    /// `false` (the default) is a working VNC session: crosvm builds this binding an absolute
    /// pointer and a keyboard of its own and injects the server's RFB events into them. `true`
    /// builds neither and drops RFB pointer and key events on arrival -- what a screen whose input
    /// devices the user switched off asks for.
    ///
    /// A property of the BINDING, not of the VM. Two screens can differ, which is the whole reason
    /// this could not stay where it was: the retired `input=` key selected between shapes of a
    /// VM-global pointer set, and a set shared by two servers cannot express "this screen watches,
    /// that one is driven" -- nor could the guest tell which screen a coordinate came from, since
    /// both servers normalized against their own framebuffer into the same device.
    ///
    /// There is deliberately NO `input=` here, and `deny_unknown_fields` above is what makes that a
    /// refusal rather than a shrug -- the same reasoning as the retired `h264-port` below. A
    /// command line that still names it was written against a crosvm whose input wiring this one
    /// does not have, and starting anyway would hand its author a pointer landing on the wrong
    /// screen or on nothing at all.
    #[serde(default)]
    pub view_only: bool,
    /// Which screen this server shows. `None` on the wire means "not said"; `validate_config`
    /// resolves it and writes the answer back, so nothing downstream ever sees `None`.
    #[serde(default)]
    pub screen: Option<DisplayScreen>,
    /// Ceiling on this binding's transport (see `TransportCap`).
    ///
    /// Accepted here for the same reason both exporters take `screen=`: the two options describe
    /// the same kind of thing and a caller should not have to remember which half of the surface
    /// exists on which one. Both of its upper rungs mean something on this sink now -- `gpu` is
    /// the Vulkan blit that step 11 gave it, `gpu-hw` the hardware H.264 encoder above that.
    #[serde(default)]
    pub transport_cap: TransportCap,
    // There is deliberately NO `h264-port` here, and `deny_unknown_fields` above is what makes
    // that a refusal rather than a shrug. The H.264 stream used to have a side channel of its own
    // on `port + 100`; it now rides the RFB port as encoding 50
    // (plans/H264_SINGLE_PORT.md). A command line that still names the retired key was written
    // against a crosvm that answered on a socket this one does not open, so starting anyway would
    // hand its author a VM whose stream is silently missing. Mixed deploys are already forbidden
    // in this project, and a silently dropped key is how a stale config passes a gate.
}

#[cfg(feature = "vnc")]
impl VncConfig {
    /// This server's effective RFB port.
    pub fn effective_port(&self) -> u32 {
        self.port.unwrap_or(DEFAULT_VNC_PORT)
    }

    /// Whether this binding may run the hardware H.264 encoder and serve its stream to RFB clients
    /// that ask for encoding 50.
    ///
    /// The one place the ceiling is read for this question, so that "is hardware encoding allowed
    /// here" cannot answer differently in the two sinks that ask it. `false` means no broker is
    /// built at all, so a client that asks for 50 there is served pixels and told nothing -- which
    /// is exactly what an older server looks like on the wire, and what a client has to handle
    /// anyway.
    pub fn h264_enabled(&self) -> bool {
        self.transport_cap.allows_hw_encode()
    }

    /// Whether this binding gets input devices of its own (a tablet and a keyboard).
    ///
    /// Asked as a question rather than as `!view_only` at each call site, for the same reason
    /// `TransportCap` does it: the two places that must agree are the device creation and the
    /// sink's injection, and disagreement is silent in both directions -- a device nothing writes
    /// to, or events written to nothing.
    pub fn wants_input_devices(&self) -> bool {
        !self.view_only
    }
}

/// One Android display service: the name crosvm registers with the service manager, and which
/// screen's frames go into the Surface the app hands back through it.
///
/// `name` comes first so the older bare form keeps parsing: `--android-display-service droidvm_x`
/// means exactly `--android-display-service name=droidvm_x`, the same shape that lets `--pmem
/// /path/to/disk.img` and `--block /path/to/disk.img,ro=true` share one option. (A service
/// literally named `name` or `screen` would be read as a key instead; no name generator produces
/// those, and the failure is a parse error rather than a wrong binding.)
#[cfg(feature = "android_display")]
#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize, FromKeyValues)]
#[serde(deny_unknown_fields, rename_all = "kebab-case")]
pub struct AndroidDisplayServiceConfig {
    /// Name to register the display service under. The app derives it and looks it up under the
    /// same name, so it is the identity of the channel, not just a label.
    pub name: String,
    /// Which screen this service shows. `None` on the wire means "not said"; see `VncConfig`.
    #[serde(default)]
    pub screen: Option<DisplayScreen>,
    /// Ceiling on this binding's transport (see `TransportCap`). This is the sink where the cap
    /// currently changes anything: it is the one with a Vulkan blit behind it.
    #[serde(default)]
    pub transport_cap: TransportCap,
}

#[derive(Debug, Default, Serialize, Deserialize, FromKeyValues)]
#[serde(deny_unknown_fields, rename_all = "kebab-case")]
pub struct DtboOption {
    /// Overlay file to apply to the base device tree.
    pub path: PathBuf,
    /// Whether to only apply device tree nodes which belong to a VFIO device.
    #[serde(rename = "filter", default)]
    pub filter_devs: bool,
}

#[derive(Debug, Default, Deserialize, Serialize, FromKeyValues, PartialEq, Eq)]
#[serde(deny_unknown_fields, rename_all = "kebab-case")]
pub struct MemOptions {
    /// Amount of guest memory in MiB.
    #[serde(default)]
    pub size: Option<u64>,
}

fn deserialize_swap_interval<'de, D: Deserializer<'de>>(
    deserializer: D,
) -> Result<Option<Duration>, D::Error> {
    let ms = Option::<u64>::deserialize(deserializer)?;
    match ms {
        None => Ok(None),
        Some(ms) => Ok(Some(Duration::from_millis(ms))),
    }
}

#[derive(
    Clone, Debug, Default, Serialize, Deserialize, PartialEq, Eq, serde_keyvalue::FromKeyValues,
)]
#[serde(deny_unknown_fields, rename_all = "kebab-case")]
pub struct PmemOption {
    /// Path to the diks image.
    pub path: PathBuf,
    /// Whether the disk is read-only.
    #[serde(default)]
    pub ro: bool,
    /// If set, add a kernel command line option making this the root device. Can only be set once.
    #[serde(default)]
    pub root: bool,
    /// Experimental option to specify the size in bytes of an anonymous virtual memory area that
    /// will be created to back this device.
    #[serde(default)]
    pub vma_size: Option<u64>,
    /// Experimental option to specify interval for periodic swap out of memory mapping
    #[serde(
        default,
        deserialize_with = "deserialize_swap_interval",
        rename = "swap-interval-ms"
    )]
    pub swap_interval: Option<Duration>,
}

#[derive(Serialize, Deserialize, FromKeyValues)]
#[serde(deny_unknown_fields, rename_all = "kebab-case")]
pub struct VhostUserOption {
    pub socket: PathBuf,

    /// Maximum number of entries per queue (default: 32768)
    pub max_queue_size: Option<u16>,
}

#[derive(Serialize, Deserialize, FromKeyValues)]
#[serde(deny_unknown_fields, rename_all = "kebab-case")]
pub struct VhostUserFrontendOption {
    /// Device type
    #[serde(rename = "type")]
    pub type_: devices::virtio::DeviceType,

    /// Path to the vhost-user backend socket to connect to
    pub socket: PathBuf,

    /// Maximum number of entries per queue (default: 32768)
    pub max_queue_size: Option<u16>,

    /// Preferred PCI address
    pub pci_address: Option<PciAddress>,
}

#[derive(Serialize, Deserialize, FromKeyValues)]
#[serde(deny_unknown_fields, rename_all = "kebab-case")]
pub struct VhostUserFsOption {
    #[serde(alias = "socket")]
    pub socket_path: Option<PathBuf>,
    /// File descriptor of connected socket
    pub socket_fd: Option<u32>,
    pub tag: Option<String>,

    /// Maximum number of entries per queue (default: 32768)
    pub max_queue_size: Option<u16>,
}

pub fn parse_vhost_user_fs_option(param: &str) -> Result<VhostUserFsOption, String> {
    // Allow the previous `--vhost-user-fs /path/to/socket:fs-tag` format for compatibility.
    // This will unfortunately prevent parsing of valid comma-separated FromKeyValues options that
    // contain a ":" character (e.g. in a socket filename), but those were not supported in the old
    // format either, so we can live with it until the deprecated format is removed.
    // TODO(b/218223240): Remove support for the deprecated format (and use `FromKeyValues`
    // directly instead of `from_str_fn`) once enough time has passed.
    if param.contains(':') {
        // (socket:tag)
        let mut components = param.split(':');
        let socket = PathBuf::from(
            components
                .next()
                .ok_or("missing socket path for `vhost-user-fs`")?,
        );
        let tag = components
            .next()
            .ok_or("missing tag for `vhost-user-fs`")?
            .to_owned();

        log::warn!(
            "`--vhost-user-fs` with colon-separated options is deprecated; \
            please use `--vhost-user-fs {},tag={}` instead",
            socket.display(),
            tag,
        );

        Ok(VhostUserFsOption {
            socket_path: Some(socket),
            tag: Some(tag),
            max_queue_size: None,
            socket_fd: None,
        })
    } else {
        from_key_values::<VhostUserFsOption>(param)
    }
}

pub const DEFAULT_TOUCH_DEVICE_HEIGHT: u32 = 1024;
pub const DEFAULT_TOUCH_DEVICE_WIDTH: u32 = 1280;

/// Fixed ABS_X/ABS_Y max for a "normalized" absolute-pointer / touch device -- used when an
/// `--input absolute-mouse`/`multi-touch` is given with no explicit width/height. The feeder
/// scales its coordinates to this range against the live display size, so the mapping is
/// resolution-independent and survives guest auto-resize (same scheme the VNC pointer path uses;
/// MUST equal `VNC_ABS_MAX` in gpu_display/src/gpu_display_vnc.rs). An explicit width/height keeps
/// the legacy pixel-sized range for backward compatibility.
pub const NORMALIZED_ABS_MAX: u32 = 0x7FFF;

#[derive(Serialize, Deserialize, Debug, FromKeyValues)]
#[serde(deny_unknown_fields, rename_all = "kebab-case")]
pub struct TouchDeviceOption {
    pub path: PathBuf,
    pub width: Option<u32>,
    pub height: Option<u32>,
    pub name: Option<String>,
}

/// Try to parse a colon-separated touch device option.
///
/// The expected format is "PATH:WIDTH:HEIGHT:NAME", with all fields except PATH being optional.
fn parse_touch_device_option_legacy(s: &str) -> Option<TouchDeviceOption> {
    let mut it = s.split(':');
    let path = PathBuf::from(it.next()?.to_owned());
    let width = if let Some(width) = it.next() {
        Some(width.trim().parse().ok()?)
    } else {
        None
    };
    let height = if let Some(height) = it.next() {
        Some(height.trim().parse().ok()?)
    } else {
        None
    };
    let name = it.next().map(|name| name.trim().to_string());
    if it.next().is_some() {
        return None;
    }

    Some(TouchDeviceOption {
        path,
        width,
        height,
        name,
    })
}

/// Parse virtio-input touch device options from a string.
///
/// This function only exists to enable the use of the deprecated colon-separated form
/// ("PATH:WIDTH:HEIGHT:NAME"); once the deprecation period is over, this function should be removed
/// in favor of using the derived `FromKeyValues` function directly.
pub fn parse_touch_device_option(s: &str) -> Result<TouchDeviceOption, String> {
    if s.contains(':') {
        if let Some(touch_spec) = parse_touch_device_option_legacy(s) {
            log::warn!(
                "colon-separated touch device options are deprecated; \
                please use --input instead"
            );
            return Ok(touch_spec);
        }
    }

    from_key_values::<TouchDeviceOption>(s)
}

/// virtio-input device configuration
#[derive(Serialize, Deserialize, Debug, FromKeyValues, Eq, PartialEq)]
#[serde(deny_unknown_fields, rename_all = "kebab-case")]
pub enum InputDeviceOption {
    Evdev {
        path: PathBuf,
    },
    // `name` is here for a different reason than the touch devices' below, and the difference is
    // worth stating because it looks like the same field. A keyboard reports no coordinates, so it
    // is not mapped to an output and nothing silently breaks if it is renamed. It is named because
    // there is now more than one of them: a keyboard belongs to a scanout, so the guest sees one
    // per screen with input enabled, and several devices all called "Crosvm Virtio Keyboard <idx>"
    // is a list nobody can read -- the idx counts emission order, so it is not even stable per
    // screen. Omitted keeps that generated name, which is still right for a single unnamed keyboard.
    //
    // CROSS-REPO SEAM: the DroidVM daemon passes `DroidVM Keyboard (<screenId>)` here for a
    // natively exported screen, the same string crosvm generates itself for a VNC-exported one (see
    // `vnc_keyboard_device_name`, sys/linux.rs). One format, two producers, so a screen keeps its
    // keyboard's name when it changes exporter.
    Keyboard {
        path: PathBuf,
        name: Option<String>,
    },
    // No `name` here, deliberately. The relative mouse is the VM's and not a scanout's -- it is
    // what the app's MOUSE console mode drives, a pointer that walks from one output to the next --
    // so there is exactly one and it has no per-screen identity to carry.
    Mouse {
        path: PathBuf,
    },
    // Absolute-pointing mouse (qemu usb-tablet profile): ABS_X/ABS_Y + buttons + wheel. Unlike
    // SingleTouch (a BTN_TOUCH touchscreen) it reports a position continuously, so the guest gets
    // pointer hover, right-click and scroll -- what the app's "tablet" input mode maps a host
    // mouse/stylus onto.
    //
    // `name` is here for the same reason the touch devices have one: an absolute coordinate only
    // means something against one output's geometry, so a VM with several outputs wants one of
    // these per output, and evdev has no field saying which output a device belongs to. The guest
    // is told by hand -- kwin stores the mapping by device name, `xinput map-to-output` takes one,
    // Windows' Tablet PC setup remembers the one it was pointed at -- so the name is the only
    // handle a per-output mapping has. Without it the device is "Crosvm Virtio Absolute Mouse
    // <idx>", an index that shifts when the set of devices changes.
    AbsoluteMouse {
        path: PathBuf,
        width: Option<u32>,
        height: Option<u32>,
        name: Option<String>,
    },
    MultiTouch {
        path: PathBuf,
        width: Option<u32>,
        height: Option<u32>,
        name: Option<String>,
    },
    Rotary {
        path: PathBuf,
    },
    SingleTouch {
        path: PathBuf,
        width: Option<u32>,
        height: Option<u32>,
        name: Option<String>,
    },
    Switches {
        path: PathBuf,
    },
    Trackpad {
        path: PathBuf,
        width: Option<u32>,
        height: Option<u32>,
        name: Option<String>,
    },
    MultiTouchTrackpad {
        path: PathBuf,
        width: Option<u32>,
        height: Option<u32>,
        name: Option<String>,
    },
    #[serde(rename_all = "kebab-case")]
    Custom {
        path: PathBuf,
        config_path: PathBuf,
    },
}

fn parse_hex_or_decimal(maybe_hex_string: &str) -> Result<u64, String> {
    // Parse string starting with 0x as hex and others as numbers.
    if let Some(hex_string) = maybe_hex_string.strip_prefix("0x") {
        u64::from_str_radix(hex_string, 16)
    } else if let Some(hex_string) = maybe_hex_string.strip_prefix("0X") {
        u64::from_str_radix(hex_string, 16)
    } else {
        u64::from_str(maybe_hex_string)
    }
    .map_err(|e| format!("invalid numeric value {}: {}", maybe_hex_string, e))
}

pub fn parse_mmio_address_range(s: &str) -> Result<Vec<AddressRange>, String> {
    s.split(",")
        .map(|s| {
            let r: Vec<&str> = s.split("-").collect();
            if r.len() != 2 {
                return Err(invalid_value_err(s, "invalid range"));
            }
            let parse = |s: &str| -> Result<u64, String> {
                match parse_hex_or_decimal(s) {
                    Ok(v) => Ok(v),
                    Err(_) => Err(invalid_value_err(s, "expected u64 value")),
                }
            };
            Ok(AddressRange {
                start: parse(r[0])?,
                end: parse(r[1])?,
            })
        })
        .collect()
}

pub fn validate_serial_parameters(params: &SerialParameters) -> Result<(), String> {
    if params.stdin && params.input.is_some() {
        return Err("Cannot specify both stdin and input options".to_string());
    }
    if params.num < 1 {
        return Err(invalid_value_err(
            params.num.to_string(),
            "Serial port num must be at least 1",
        ));
    }

    if params.hardware == SerialHardware::Serial && params.num > 4 {
        return Err(invalid_value_err(
            format!("{}", params.num),
            "Serial port num must be 4 or less",
        ));
    }

    if params.pci_address.is_some() && params.hardware != SerialHardware::VirtioConsole {
        return Err(invalid_value_err(
            params.pci_address.unwrap().to_string(),
            "Providing serial PCI address is only supported for virtio-console hardware type",
        ));
    }

    Ok(())
}

pub fn parse_serial_options(s: &str) -> Result<SerialParameters, String> {
    let params: SerialParameters = from_key_values(s)?;

    validate_serial_parameters(&params)?;

    Ok(params)
}

pub fn parse_bus_id_addr(v: &str) -> Result<(u8, u8, u16, u16), String> {
    debug!("parse_bus_id_addr: {}", v);
    let mut ids = v.split(':');
    let errorre = move |item| move |e| format!("{}: {}", item, e);
    match (ids.next(), ids.next(), ids.next(), ids.next()) {
        (Some(bus_id), Some(addr), Some(vid), Some(pid)) => {
            let bus_id = bus_id.parse::<u8>().map_err(errorre("bus_id"))?;
            let addr = addr.parse::<u8>().map_err(errorre("addr"))?;
            let vid = u16::from_str_radix(vid, 16).map_err(errorre("vid"))?;
            let pid = u16::from_str_radix(pid, 16).map_err(errorre("pid"))?;
            Ok((bus_id, addr, vid, pid))
        }
        _ => Err(String::from("BUS_ID:ADDR:BUS_NUM:DEV_NUM")),
    }
}

pub fn invalid_value_err<T: AsRef<str>, S: ToString>(value: T, expected: S) -> String {
    format!("invalid value {}: {}", value.as_ref(), expected.to_string())
}

#[derive(Debug, Serialize, Deserialize, FromKeyValues)]
#[serde(deny_unknown_fields, rename_all = "kebab-case")]
pub struct BatteryConfig {
    #[serde(rename = "type", default)]
    pub type_: BatteryType,
}

/// Pre-allocated GPU pool sizes (MB). Every one of these is a boot-blessed region -- SHARE'd once,
/// folio-backed -- that something sub-allocates from, so no blob needs a runtime per-blob SHARE.
///
/// The names are `<route>-<who allocates>-mb`. The route prefix is `gfx` for gfxstream and `drm`
/// for the DRM native context; the middle word says which side owns the allocator inside the pool,
/// which is the distinction that actually changes behaviour:
///   host   the renderer sub-allocates, and the guest is handed offsets into the region.
///   guest  the guest virtio-gpu driver sub-allocates with drm_buddy and hands the host pages.
///
/// Sizes are needed very early (they shape the guest memory layout), so crosvm exports them to the
/// renderer as env before the GPU process forks rather than expecting the user to hand-export them.
///
/// `--pre-alloc "gfx-host-mb=256,gfx-guest-mb=1024"`
/// `--pre-alloc "drm-host-mb=8,drm-guest-mb=1024"`
#[derive(Clone, Debug, Serialize, Deserialize, FromKeyValues)]
#[serde(deny_unknown_fields, rename_all = "kebab-case")]
pub struct PreAllocConfig {
    /// gfxstream HOST-visible pool size (MB): the host-alloc pool the gfxstream HostVisiblePool
    /// sub-allocates from (ASG rings + host-visible blobs). Absent => 0 (host pre-alloc off ->
    /// runtime-share). Its own SHARE-blessed GpuPool region + `gfx_host` DT node.
    pub gfx_host_mb: Option<u64>,
    /// GUEST-allocated pool size (MB): a SHARE-blessed region the guest virtio-gpu driver owns
    /// and sub-allocates every guest-alloc blob from with drm_buddy, handing the host dma-bufs
    /// built over those pages. Absent => 0 (no guest-alloc pool). Its own GpuPoolGuest region +
    /// `gpu_guest` DT node.
    ///
    /// One knob for both renderers on purpose. The guest driver keeps a single pool and a single
    /// allocator and cannot tell which renderer is asking -- it takes whichever reserved-memory
    /// node it finds -- and only one renderer runs in a VM anyway. Two per-route names would be
    /// two names for one thing, and setting both would silently pin a whole second pool the guest
    /// never touches.
    pub gpu_guest_mb: Option<u64>,
    /// Bytes of the guest-alloc pool SHARE'd before boot. Defaults to `gpu-guest-mb`, preserving
    /// the current fully pre-shared pool. A smaller value enables runtime growth in `step` chunks.
    pub gpu_guest_prealloc_mb: Option<u64>,
    /// Runtime growth/reclaim granularity for the guest-alloc pool. Zero or absent keeps the pool
    /// fully pre-shared; non-zero values must satisfy the growable-pool alignment rules.
    pub gpu_guest_step_mb: Option<u64>,
    /// Maximum number of simultaneously live runtime memparcels for the guest-alloc pool. Zero
    /// leaves the host-side cap unset; the actual RM quota is shared by the whole VM.
    pub gpu_guest_max_grants: Option<u32>,

    /// DRM native context HOST-allocated pool size (MB): the region virglrenderer's DRM backend
    /// sub-allocates from. Since BO backing moved to the guest this holds only the per-context msm
    /// shmem rings -- 16 KiB each -- so single-digit MB is the right size, not the gigabyte the
    /// BO pool needed. Absent => 0, and the rings fall back to a runtime SHARE apiece, which is
    /// the round trip this route exists to avoid. Its own Drm2KgslPool region + `drm2kgsl_host` DT
    /// node. Only meaningful with `--gpu backend=virglrenderer`.
    pub drm_host_mb: Option<u64>,

    /// venus (vkr) HOST-allocated pool size (MB): where the venus transport shmems (per-instance
    /// ring + CS/reply chunks) are served from. The vkr pool merge is landed (virglrenderer vkr
    /// sub-allocates every blob_id==0 shmem from this region and publishes map_ptr, so the guest
    /// maps pool_base+offset with no runtime SHARE); venus's real VkDeviceMemory is separately
    /// guest-alloc in the shared gpu-guest pool. Absent/0 => vkr falls back to a per-blob memfd +
    /// runtime SHARE apiece (the round trip this pool exists to avoid; fatal to the fragile sm8650
    /// RM). Size for the peak transport working set (cs pool alone is >=8M per instance). Its own
    /// VenusPool region + `venus_host` DT node. Only meaningful with
    /// `--gpu backend=virglrenderer,context-types=venus`.
    pub venus_host_mb: Option<u64>,

    /// Take every pool below out of the VM's system RAM instead of adding it on top of `--mem`.
    ///
    /// Off by default, which is the arithmetic these pools have always had: `--mem 4096` with a
    /// 1 GiB pool asks the host for 5 GiB. On, the same VM asks for 4 GiB and gives the guest 3 --
    /// so what a VM costs is what was typed, which is the number worth being able to predict when
    /// the memory comes out of a fixed reserve pool.
    pub alloc_from_vm_sys_ram: Option<bool>,

    /// Growable TEST pool: total window size (MB). Declared to the guest whole but backed only up
    /// to `test-pool-prealloc-mb`; the rest is granted at runtime as the guest asks, a
    /// `test-pool-step-mb` multiple at a time.
    ///
    /// Exists to exercise the growable-pool path end to end -- the three pools above are all
    /// fully pre-shared (step 0) and cannot. Nothing in the guest uses it except the test driver.
    pub test_pool_mb: Option<u64>,
    /// Bytes of the test pool SHARE'd before boot. Defaults to the whole window, i.e. an ordinary
    /// non-growable pool, so setting only `test-pool-mb` changes nothing about how it behaves.
    pub test_pool_prealloc_mb: Option<u64>,
    /// Grant granularity for the test pool (MB). Must be >= 2 and a power of two. 0, or absent,
    /// means the pool does not grow.
    ///
    /// One grant is one RM memparcel however many steps it spans, and MAX_MEMPARCEL_PER_VM is
    /// 1024 for the whole VM -- so this is not "how much to allocate at once", it is the smallest
    /// piece the guest can ever release. Small steps buy fine-grained release and spend quota.
    pub test_pool_step_mb: Option<u64>,
}

#[derive(Clone, Debug, Serialize, Deserialize, FromKeyValues)]
#[serde(deny_unknown_fields, rename_all = "kebab-case")]
pub struct SimplefbConfig {
    pub width: u32,
    pub height: u32,
    #[serde(default = "default_simplefb_format")]
    pub format: String,
    /// How many times a second the host looks at the framebuffer.
    ///
    /// This framebuffer has no way to announce a new frame -- the guest maps it write-combining and
    /// nothing traps -- so the only thing that decides when a picture exists is this rate. It is a
    /// property of the host's watcher, not of the device the guest sees: the device tree says
    /// nothing about it and the guest cannot tell what it is set to.
    #[serde(default = "default_simplefb_poll_hz")]
    pub poll_hz: u32,
}

fn default_simplefb_format() -> String {
    "a8r8g8b8".to_string()
}

/// The rate the simplefb bridge polled at for as long as the rate was not settable.
pub const DEFAULT_SIMPLEFB_POLL_HZ: u32 = 30;

/// Above this the watcher is asking for more work than a display can show. It is a sanity bound on
/// a knob whose cost is linear in it, not a claim about what the hardware can do.
const MAX_SIMPLEFB_POLL_HZ: u32 = 240;

fn default_simplefb_poll_hz() -> u32 {
    DEFAULT_SIMPLEFB_POLL_HZ
}

pub fn parse_cpu_btreemap_u32(s: &str) -> Result<BTreeMap<usize, u32>, String> {
    let mut parsed_btreemap: BTreeMap<usize, u32> = BTreeMap::default();
    for cpu_pair in s.split(',') {
        let assignment: Vec<&str> = cpu_pair.split('=').collect();
        if assignment.len() != 2 {
            return Err(invalid_value_err(
                cpu_pair,
                "Invalid CPU pair syntax, missing '='",
            ));
        }
        let cpu = assignment[0].parse().map_err(|_| {
            invalid_value_err(assignment[0], "CPU index must be a non-negative integer")
        })?;
        let val = assignment[1].parse().map_err(|_| {
            invalid_value_err(assignment[1], "CPU property must be a non-negative integer")
        })?;
        if parsed_btreemap.insert(cpu, val).is_some() {
            return Err(invalid_value_err(cpu_pair, "CPU index must be unique"));
        }
    }
    Ok(parsed_btreemap)
}

#[cfg(all(
    any(target_arch = "arm", target_arch = "aarch64"),
    any(target_os = "android", target_os = "linux")
))]
pub fn parse_cpu_frequencies(s: &str) -> Result<BTreeMap<usize, Vec<u32>>, String> {
    let mut cpu_frequencies: BTreeMap<usize, Vec<u32>> = BTreeMap::default();
    for cpufreq_assigns in s.split(';') {
        let assignment: Vec<&str> = cpufreq_assigns.split('=').collect();
        if assignment.len() != 2 {
            return Err(invalid_value_err(
                cpufreq_assigns,
                "invalid CPU freq syntax",
            ));
        }
        let cpu = assignment[0].parse().map_err(|_| {
            invalid_value_err(assignment[0], "CPU index must be a non-negative integer")
        })?;
        let freqs = assignment[1]
            .split(',')
            .map(|x| x.parse::<u32>().unwrap())
            .collect::<Vec<_>>();
        if cpu_frequencies.insert(cpu, freqs).is_some() {
            return Err(invalid_value_err(
                cpufreq_assigns,
                "CPU index must be unique",
            ));
        }
    }
    Ok(cpu_frequencies)
}

pub fn from_key_values<'a, T: Deserialize<'a>>(value: &'a str) -> Result<T, String> {
    serde_keyvalue::from_key_values(value).map_err(|e| e.to_string())
}

/// Parse a list of guest to host CPU mappings.
///
/// Each mapping consists of a single guest CPU index mapped to one or more host CPUs in the form
/// accepted by `CpuSet::from_str`:
///
///  `<GUEST-CPU>=<HOST-CPU-SET>[:<GUEST-CPU>=<HOST-CPU-SET>[:...]]`
pub fn parse_cpu_affinity(s: &str) -> Result<VcpuAffinity, String> {
    if s.contains('=') {
        let mut affinity_map = BTreeMap::new();
        for cpu_pair in s.split(':') {
            let assignment: Vec<&str> = cpu_pair.split('=').collect();
            if assignment.len() != 2 {
                return Err(invalid_value_err(
                    cpu_pair,
                    "invalid VCPU assignment syntax",
                ));
            }
            let guest_cpu = assignment[0].parse().map_err(|_| {
                invalid_value_err(assignment[0], "CPU index must be a non-negative integer")
            })?;
            let host_cpu_set = CpuSet::from_str(assignment[1])?;
            if affinity_map.insert(guest_cpu, host_cpu_set).is_some() {
                return Err(invalid_value_err(cpu_pair, "VCPU index must be unique"));
            }
        }
        Ok(VcpuAffinity::PerVcpu(affinity_map))
    } else {
        Ok(VcpuAffinity::Global(CpuSet::from_str(s)?))
    }
}

pub fn executable_is_plugin(executable: &Option<Executable>) -> bool {
    matches!(executable, Some(Executable::Plugin(_)))
}

pub fn parse_pflash_parameters(s: &str) -> Result<PflashParameters, String> {
    let pflash_parameters: PflashParameters = from_key_values(s)?;

    Ok(pflash_parameters)
}

// BTreeMaps serialize fine, as long as their keys are trivial types. A tuple does not
// work, hence the need to convert to/from a vector form.
mod serde_serial_params {
    use std::iter::FromIterator;

    use serde::Deserializer;
    use serde::Serializer;

    use super::*;

    pub fn serialize<S>(
        params: &BTreeMap<(SerialHardware, u8), SerialParameters>,
        ser: S,
    ) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        let v: Vec<(&(SerialHardware, u8), &SerialParameters)> = params.iter().collect();
        serde::Serialize::serialize(&v, ser)
    }

    pub fn deserialize<'a, D>(
        de: D,
    ) -> Result<BTreeMap<(SerialHardware, u8), SerialParameters>, D::Error>
    where
        D: Deserializer<'a>,
    {
        let params: Vec<((SerialHardware, u8), SerialParameters)> =
            serde::Deserialize::deserialize(de)?;
        Ok(BTreeMap::from_iter(params))
    }
}

/// Aggregate of all configurable options for a running VM.
#[derive(Serialize, Deserialize)]
#[remain::sorted]
pub struct Config {
    #[cfg(all(target_arch = "x86_64", unix))]
    pub ac_adapter: bool,
    pub acpi_tables: Vec<PathBuf>,
    /// Android display services, one per screen they export. Empty is the normal case.
    #[cfg(feature = "android_display")]
    pub android_display_service: Vec<AndroidDisplayServiceConfig>,
    pub android_fstab: Option<PathBuf>,
    pub async_executor: Option<ExecutorKind>,
    #[cfg(feature = "balloon")]
    pub balloon: bool,
    #[cfg(feature = "balloon")]
    pub balloon_bias: i64,
    #[cfg(feature = "balloon")]
    pub balloon_control: Option<PathBuf>,
    #[cfg(feature = "balloon")]
    pub balloon_page_reporting: bool,
    #[cfg(feature = "balloon")]
    pub balloon_ws_num_bins: u8,
    #[cfg(feature = "balloon")]
    pub balloon_ws_reporting: bool,
    pub battery_config: Option<BatteryConfig>,
    #[cfg(windows)]
    pub block_control_tube: Vec<Tube>,
    #[cfg(windows)]
    pub block_vhost_user_tube: Vec<Tube>,
    #[cfg(any(target_os = "android", target_os = "linux"))]
    pub boost_uclamp: bool,
    pub boot_cpu: usize,
    #[cfg(target_arch = "x86_64")]
    pub break_linux_pci_config_io: bool,
    #[cfg(windows)]
    pub broker_shutdown_event: Option<Event>,
    #[cfg(target_arch = "x86_64")]
    pub bus_lock_ratelimit: u64,
    #[cfg(any(target_os = "android", target_os = "linux"))]
    pub coiommu_param: Option<devices::CoIommuParameters>,
    pub core_scheduling: bool,
    pub cpu_capacity: BTreeMap<usize, u32>, // CPU index -> capacity
    pub cpu_clusters: Vec<CpuSet>,
    pub cpu_freq_domains: Vec<CpuSet>,
    #[cfg(all(
        any(target_arch = "arm", target_arch = "aarch64"),
        any(target_os = "android", target_os = "linux")
    ))]
    pub cpu_frequencies_khz: BTreeMap<usize, Vec<u32>>, // CPU index -> frequencies
    #[cfg(all(
        any(target_arch = "arm", target_arch = "aarch64"),
        any(target_os = "android", target_os = "linux")
    ))]
    pub cpu_ipc_ratio: BTreeMap<usize, u32>, // CPU index -> IPC Ratio
    #[cfg(feature = "crash-report")]
    pub crash_pipe_name: Option<String>,
    #[cfg(feature = "crash-report")]
    pub crash_report_uuid: Option<String>,
    pub delay_rt: bool,
    pub device_tree_overlay: Vec<DtboOption>,
    pub disable_virtio_intx: bool,
    pub disks: Vec<DiskOption>,
    pub display_input_height: Option<u32>,
    pub display_input_width: Option<u32>,
    pub display_window_keyboard: bool,
    pub display_window_mouse: bool,
    pub dump_device_tree_blob: Option<PathBuf>,
    pub dynamic_power_coefficient: BTreeMap<usize, u32>,
    pub enable_fw_cfg: bool,
    pub enable_hwp: bool,
    pub executable_path: Option<Executable>,
    #[cfg(windows)]
    pub exit_stats: bool,
    pub fdt_position: Option<FdtPosition>,
    pub file_backed_mappings_mmio: Vec<FileBackedMappingParameters>,
    pub file_backed_mappings_ram: Vec<FileBackedMappingParameters>,
    pub force_calibrated_tsc_leaf: bool,
    pub force_s2idle: bool,
    pub fw_cfg_parameters: Vec<FwCfgParameters>,
    #[cfg(feature = "gdb")]
    pub gdb: Option<u32>,
    #[cfg(all(windows, feature = "gpu"))]
    pub gpu_backend_config: Option<GpuBackendConfig>,
    #[cfg(all(unix, feature = "gpu"))]
    pub gpu_cgroup_path: Option<PathBuf>,
    #[cfg(feature = "gpu")]
    pub gpu_parameters: Option<GpuParameters>,
    #[cfg(all(unix, feature = "gpu"))]
    pub gpu_render_server_parameters: Option<GpuRenderServerParameters>,
    #[cfg(all(unix, feature = "gpu"))]
    pub gpu_server_cgroup_path: Option<PathBuf>,
    #[cfg(all(windows, feature = "gpu"))]
    pub gpu_vmm_config: Option<GpuVmmConfig>,
    pub host_cpu_topology: bool,
    #[cfg(windows)]
    pub host_guid: Option<String>,
    pub hugepages: bool,
    pub hypervisor: Option<HypervisorKind>,
    #[cfg(feature = "balloon")]
    pub init_memory: Option<u64>,
    pub initrd_path: Option<PathBuf>,
    #[cfg(all(windows, feature = "gpu"))]
    pub input_event_split_config: Option<InputEventSplitConfig>,
    pub irq_chip: Option<IrqChipKind>,
    pub itmt: bool,
    pub jail_config: Option<JailConfig>,
    #[cfg(windows)]
    pub kernel_log_file: Option<String>,
    #[cfg(any(target_os = "android", target_os = "linux"))]
    pub lock_guest_memory: bool,
    #[cfg(windows)]
    pub log_file: Option<String>,
    #[cfg(windows)]
    pub logs_directory: Option<String>,
    #[cfg(all(feature = "media", feature = "video-decoder"))]
    pub media_decoder: Vec<VideoDeviceConfig>,
    pub memory: Option<u64>,
    pub memory_file: Option<PathBuf>,
    pub mmio_address_ranges: Vec<AddressRange>,
    #[cfg(target_arch = "aarch64")]
    pub mte: bool,
    pub name: Option<String>,
    #[cfg(feature = "net")]
    pub net: Vec<NetParameters>,
    #[cfg(windows)]
    pub net_vhost_user_tube: Option<Tube>,
    pub no_i8042: bool,
    pub no_pmu: bool,
    pub no_rtc: bool,
    pub no_smt: bool,
    pub params: Vec<String>,
    pub pci_config: PciConfig,
    #[cfg(feature = "pci-hotplug")]
    pub pci_hotplug_slots: Option<u8>,
    pub per_vm_core_scheduling: bool,
    pub pflash_parameters: Option<PflashParameters>,
    #[cfg(feature = "plugin")]
    pub plugin_gid_maps: Vec<crate::crosvm::plugin::GidMap>,
    #[cfg(feature = "plugin")]
    pub plugin_mounts: Vec<crate::crosvm::plugin::BindMount>,
    pub plugin_root: Option<PathBuf>,
    #[cfg(any(target_os = "android", target_os = "linux"))]
    pub pmem_ext2: Vec<crate::crosvm::sys::config::PmemExt2Option>,
    pub pmems: Vec<PmemOption>,
    /// Host-owned pre-allocated GPU pool sizes (`--pre-alloc`; gfx host / guest). Exported to the
    /// renderer as NCTX_GFX_POOL_MB env at startup.
    pub pre_alloc: Option<PreAllocConfig>,
    pub prepare_lend_mthp: Option<LendMthpMode>,
    #[cfg(feature = "process-invariants")]
    pub process_invariants_data_handle: Option<u64>,
    #[cfg(feature = "process-invariants")]
    pub process_invariants_data_size: Option<usize>,
    #[cfg(windows)]
    pub product_channel: Option<String>,
    #[cfg(windows)]
    pub product_name: Option<String>,
    #[cfg(windows)]
    pub product_version: Option<String>,
    pub protection_type: ProtectionType,
    pub pstore: Option<Pstore>,
    #[cfg(feature = "pvclock")]
    pub pvclock: bool,
    /// Must be `Some` iff `protection_type == ProtectionType::UnprotectedWithFirmware`.
    pub pvm_fw: Option<PathBuf>,
    pub restore_path: Option<PathBuf>,
    pub rng: bool,
    pub rt_cpus: CpuSet,
    pub scsis: Vec<ScsiOption>,
    #[serde(with = "serde_serial_params")]
    pub serial_parameters: BTreeMap<(SerialHardware, u8), SerialParameters>,
    #[cfg(windows)]
    pub service_pipe_name: Option<String>,
    #[cfg(any(target_os = "android", target_os = "linux"))]
    #[serde(skip)]
    pub shared_dirs: Vec<SharedDir>,
    #[cfg(feature = "media")]
    pub simple_media_device: bool,
    pub simplefb: Option<SimplefbConfig>,
    #[cfg(any(feature = "slirp-ring-capture", feature = "slirp-debug"))]
    pub slirp_capture_file: Option<String>,
    pub smbios: SmbiosOptions,
    #[cfg(all(windows, feature = "audio"))]
    pub snd_split_configs: Vec<SndSplitConfig>,
    pub socket_path: Option<PathBuf>,
    #[cfg(feature = "audio")]
    pub sound: Option<PathBuf>,
    pub stub_pci_devices: Vec<StubPciParameters>,
    pub suspended: bool,
    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    pub sve: Option<SveConfig>,
    pub swap_dir: Option<PathBuf>,
    pub swiotlb: Option<u64>,
    #[cfg(target_os = "android")]
    pub task_profiles: Vec<String>,
    #[cfg(any(target_os = "android", target_os = "linux"))]
    pub unmap_guest_memory_on_fork: bool,
    pub usb: bool,
    #[cfg(any(target_os = "android", target_os = "linux"))]
    #[cfg(feature = "media")]
    pub v4l2_proxy: Vec<PathBuf>,
    pub vcpu_affinity: Option<VcpuAffinity>,
    pub vcpu_cgroup_path: Option<PathBuf>,
    pub vcpu_count: Option<usize>,
    #[cfg(target_arch = "x86_64")]
    pub vcpu_hybrid_type: BTreeMap<usize, CpuHybridType>, // CPU index -> hybrid type
    #[cfg(any(target_os = "android", target_os = "linux"))]
    pub vfio: Vec<super::sys::config::VfioOption>,
    #[cfg(any(target_os = "android", target_os = "linux"))]
    pub vfio_isolate_hotplug: bool,
    #[cfg(any(target_os = "android", target_os = "linux"))]
    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    pub vhost_scmi: bool,
    #[cfg(any(target_os = "android", target_os = "linux"))]
    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    pub vhost_scmi_device: PathBuf,
    pub vhost_user: Vec<VhostUserFrontendOption>,
    pub vhost_user_connect_timeout_ms: Option<u64>,
    pub vhost_user_fs: Vec<VhostUserFsOption>,
    #[cfg(feature = "video-decoder")]
    pub video_dec: Vec<VideoDeviceConfig>,
    #[cfg(feature = "video-encoder")]
    pub video_enc: Vec<VideoDeviceConfig>,
    #[cfg(all(
        any(target_arch = "arm", target_arch = "aarch64"),
        any(target_os = "android", target_os = "linux")
    ))]
    pub virt_cpufreq: bool,
    pub virt_cpufreq_v2: bool,
    pub virtio_input: Vec<InputDeviceOption>,
    #[cfg(feature = "audio")]
    #[serde(skip)]
    pub virtio_snds: Vec<SndParameters>,
    /// VNC servers, one per screen they export. Empty is the normal case.
    #[cfg(feature = "vnc")]
    pub vnc_server: Vec<VncConfig>,
    pub vsock: Option<VsockConfig>,
    #[cfg(feature = "vtpm")]
    pub vtpm_proxy: bool,
    pub wayland_socket_paths: BTreeMap<String, PathBuf>,
    #[cfg(all(windows, feature = "gpu"))]
    pub window_procedure_thread_split_config: Option<WindowProcedureThreadSplitConfig>,
    pub x_display: Option<String>,
}

impl Default for Config {
    fn default() -> Config {
        Config {
            #[cfg(all(target_arch = "x86_64", unix))]
            ac_adapter: false,
            acpi_tables: Vec::new(),
            #[cfg(feature = "android_display")]
            android_display_service: Vec::new(),
            android_fstab: None,
            async_executor: None,
            #[cfg(feature = "balloon")]
            balloon: true,
            #[cfg(feature = "balloon")]
            balloon_bias: 0,
            #[cfg(feature = "balloon")]
            balloon_control: None,
            #[cfg(feature = "balloon")]
            balloon_page_reporting: false,
            #[cfg(feature = "balloon")]
            balloon_ws_num_bins: VIRTIO_BALLOON_WS_DEFAULT_NUM_BINS,
            #[cfg(feature = "balloon")]
            balloon_ws_reporting: false,
            battery_config: None,
            boot_cpu: 0,
            #[cfg(windows)]
            block_control_tube: Vec::new(),
            #[cfg(windows)]
            block_vhost_user_tube: Vec::new(),
            #[cfg(target_arch = "x86_64")]
            break_linux_pci_config_io: false,
            #[cfg(windows)]
            broker_shutdown_event: None,
            #[cfg(target_arch = "x86_64")]
            bus_lock_ratelimit: 0,
            #[cfg(any(target_os = "android", target_os = "linux"))]
            coiommu_param: None,
            core_scheduling: true,
            #[cfg(feature = "crash-report")]
            crash_pipe_name: None,
            #[cfg(feature = "crash-report")]
            crash_report_uuid: None,
            cpu_capacity: BTreeMap::new(),
            cpu_clusters: Vec::new(),
            #[cfg(all(
                any(target_arch = "arm", target_arch = "aarch64"),
                any(target_os = "android", target_os = "linux")
            ))]
            cpu_frequencies_khz: BTreeMap::new(),
            cpu_freq_domains: Vec::new(),
            #[cfg(all(
                any(target_arch = "arm", target_arch = "aarch64"),
                any(target_os = "android", target_os = "linux")
            ))]
            cpu_ipc_ratio: BTreeMap::new(),
            delay_rt: false,
            device_tree_overlay: Vec::new(),
            disks: Vec::new(),
            disable_virtio_intx: false,
            display_input_height: None,
            display_input_width: None,
            display_window_keyboard: false,
            display_window_mouse: false,
            dump_device_tree_blob: None,
            dynamic_power_coefficient: BTreeMap::new(),
            enable_fw_cfg: false,
            enable_hwp: false,
            executable_path: None,
            #[cfg(windows)]
            exit_stats: false,
            fdt_position: None,
            file_backed_mappings_mmio: Vec::new(),
            file_backed_mappings_ram: Vec::new(),
            prepare_lend_mthp: None,
            force_calibrated_tsc_leaf: false,
            force_s2idle: false,
            fw_cfg_parameters: Vec::new(),
            #[cfg(feature = "gdb")]
            gdb: None,
            #[cfg(all(windows, feature = "gpu"))]
            gpu_backend_config: None,
            #[cfg(feature = "gpu")]
            gpu_parameters: None,
            #[cfg(all(unix, feature = "gpu"))]
            gpu_render_server_parameters: None,
            #[cfg(all(unix, feature = "gpu"))]
            gpu_cgroup_path: None,
            #[cfg(all(unix, feature = "gpu"))]
            gpu_server_cgroup_path: None,
            #[cfg(all(windows, feature = "gpu"))]
            gpu_vmm_config: None,
            host_cpu_topology: false,
            #[cfg(windows)]
            host_guid: None,
            #[cfg(windows)]
            product_version: None,
            #[cfg(windows)]
            product_channel: None,
            hugepages: false,
            hypervisor: None,
            #[cfg(feature = "balloon")]
            init_memory: None,
            initrd_path: None,
            #[cfg(all(windows, feature = "gpu"))]
            input_event_split_config: None,
            irq_chip: None,
            itmt: false,
            jail_config: if !cfg!(feature = "default-no-sandbox") {
                Some(Default::default())
            } else {
                None
            },
            #[cfg(windows)]
            kernel_log_file: None,
            #[cfg(any(target_os = "android", target_os = "linux"))]
            lock_guest_memory: false,
            #[cfg(windows)]
            log_file: None,
            #[cfg(windows)]
            logs_directory: None,
            #[cfg(any(target_os = "android", target_os = "linux"))]
            boost_uclamp: false,
            #[cfg(all(feature = "media", feature = "video-decoder"))]
            media_decoder: Default::default(),
            memory: None,
            memory_file: None,
            mmio_address_ranges: Vec::new(),
            #[cfg(target_arch = "aarch64")]
            mte: false,
            name: None,
            #[cfg(feature = "net")]
            net: Vec::new(),
            #[cfg(windows)]
            net_vhost_user_tube: None,
            no_i8042: false,
            no_pmu: false,
            no_rtc: false,
            no_smt: false,
            params: Vec::new(),
            pci_config: Default::default(),
            #[cfg(feature = "pci-hotplug")]
            pci_hotplug_slots: None,
            per_vm_core_scheduling: false,
            pflash_parameters: None,
            #[cfg(feature = "plugin")]
            plugin_gid_maps: Vec::new(),
            #[cfg(feature = "plugin")]
            plugin_mounts: Vec::new(),
            plugin_root: None,
            #[cfg(any(target_os = "android", target_os = "linux"))]
            pmem_ext2: Vec::new(),
            pmems: Vec::new(),
            pre_alloc: None,
            #[cfg(feature = "process-invariants")]
            process_invariants_data_handle: None,
            #[cfg(feature = "process-invariants")]
            process_invariants_data_size: None,
            #[cfg(windows)]
            product_name: None,
            protection_type: ProtectionType::Unprotected,
            pstore: None,
            #[cfg(feature = "pvclock")]
            pvclock: false,
            pvm_fw: None,
            restore_path: None,
            rng: true,
            rt_cpus: Default::default(),
            serial_parameters: BTreeMap::new(),
            scsis: Vec::new(),
            #[cfg(windows)]
            service_pipe_name: None,
            #[cfg(any(target_os = "android", target_os = "linux"))]
            shared_dirs: Vec::new(),
            #[cfg(feature = "media")]
            simple_media_device: Default::default(),
            simplefb: None,
            #[cfg(any(feature = "slirp-ring-capture", feature = "slirp-debug"))]
            slirp_capture_file: None,
            smbios: SmbiosOptions::default(),
            #[cfg(all(windows, feature = "audio"))]
            snd_split_configs: Vec::new(),
            socket_path: None,
            #[cfg(feature = "audio")]
            sound: None,
            stub_pci_devices: Vec::new(),
            suspended: false,
            #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
            sve: None,
            swap_dir: None,
            swiotlb: None,
            #[cfg(target_os = "android")]
            task_profiles: Vec::new(),
            #[cfg(any(target_os = "android", target_os = "linux"))]
            unmap_guest_memory_on_fork: false,
            usb: true,
            vcpu_affinity: None,
            vcpu_cgroup_path: None,
            vcpu_count: None,
            #[cfg(target_arch = "x86_64")]
            vcpu_hybrid_type: BTreeMap::new(),
            #[cfg(any(target_os = "android", target_os = "linux"))]
            vfio: Vec::new(),
            #[cfg(any(target_os = "android", target_os = "linux"))]
            vfio_isolate_hotplug: false,
            #[cfg(any(target_os = "android", target_os = "linux"))]
            #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
            vhost_scmi: false,
            #[cfg(any(target_os = "android", target_os = "linux"))]
            #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
            vhost_scmi_device: PathBuf::from(VHOST_SCMI_PATH),
            vhost_user: Vec::new(),
            vhost_user_connect_timeout_ms: None,
            vhost_user_fs: Vec::new(),
            #[cfg(feature = "vnc")]
            vnc_server: Vec::new(),
            vsock: None,
            #[cfg(feature = "video-decoder")]
            video_dec: Vec::new(),
            #[cfg(feature = "video-encoder")]
            video_enc: Vec::new(),
            #[cfg(all(
                any(target_arch = "arm", target_arch = "aarch64"),
                any(target_os = "android", target_os = "linux")
            ))]
            virt_cpufreq: false,
            virt_cpufreq_v2: false,
            virtio_input: Vec::new(),
            #[cfg(feature = "audio")]
            virtio_snds: Vec::new(),
            #[cfg(any(target_os = "android", target_os = "linux"))]
            #[cfg(feature = "media")]
            v4l2_proxy: Vec::new(),
            #[cfg(feature = "vtpm")]
            vtpm_proxy: false,
            wayland_socket_paths: BTreeMap::new(),
            #[cfg(windows)]
            window_procedure_thread_split_config: None,
            x_display: None,
        }
    }
}

impl Config {
    /// The VNC server bound to `screen`, or `None` if that screen has no VNC exporter.
    ///
    /// Answers with the resolved binding, so it is only meaningful after `validate_config` has
    /// run; before that every entry still reads as unspecified and matches no screen. Callers are
    /// all downstream of validation.
    #[cfg(feature = "vnc")]
    pub fn vnc_server_for(&self, screen: DisplayScreen) -> Option<&VncConfig> {
        self.vnc_server.iter().find(|v| v.screen == Some(screen))
    }

    /// The Android display service bound to `screen`, or `None` if that screen has no native
    /// exporter. Same resolution caveat as `vnc_server_for`.
    #[cfg(feature = "android_display")]
    pub fn android_display_service_for(
        &self,
        screen: DisplayScreen,
    ) -> Option<&AndroidDisplayServiceConfig> {
        self.android_display_service
            .iter()
            .find(|s| s.screen == Some(screen))
    }
}

/// Resolves each exporter's screen and enforces the one-exporter-per-screen rules.
///
/// This runs at validation rather than at parse time because "which screen did you mean" cannot be
/// answered from the option alone: it depends on which display devices the rest of the command
/// line configured, and that is only settled once everything has been folded into the `Config`.
/// The answer is written back into the entry, so no consumer downstream ever has to decide again
/// what an unspecified screen meant -- and none of them can decide it differently.
#[cfg(any(feature = "vnc", feature = "android_display"))]
fn validate_display_exporters(cfg: &mut Config) -> Result<(), String> {
    // Which screens exist. `validate_gpu_config` has already run and given a GPU with no
    // `displays=` its one default display, so the presence of the device is the whole question.
    #[cfg(feature = "gpu")]
    let has_gpu_screen = cfg.gpu_parameters.is_some();
    #[cfg(not(feature = "gpu"))]
    let has_gpu_screen = false;
    let has_simplefb_screen = cfg.simplefb.is_some();

    let screen_exists = |screen: DisplayScreen| match screen {
        DisplayScreen::Gpu0 => has_gpu_screen,
        DisplayScreen::Simplefb => has_simplefb_screen,
    };

    // Compat default for an exporter that named no screen. Before screens were expressible there
    // was one display and the GPU device owned it whenever there was a GPU device at all, with the
    // simplefb bridge feeding that same display; only with no GPU did simplefb present on its own.
    // Resolving the same way keeps every command line that parsed yesterday binding where it bound
    // yesterday.
    let default_screen = if has_gpu_screen {
        DisplayScreen::Gpu0
    } else {
        DisplayScreen::Simplefb
    };

    // Resolve first, then judge, so every message below can name a concrete screen.
    #[cfg(feature = "vnc")]
    for vnc in cfg.vnc_server.iter_mut() {
        let _ = vnc.screen.get_or_insert(default_screen);
    }
    #[cfg(feature = "android_display")]
    for service in cfg.android_display_service.iter_mut() {
        let _ = service.screen.get_or_insert(default_screen);
    }

    // A binding to a screen that does not exist. Rejecting is the point: the alternative is an
    // exporter that is configured, reports no error, and never shows anything -- which is how a
    // dropped display service turned into "the app's display is permanently blank" with nothing
    // in the log to say so.
    #[cfg(feature = "vnc")]
    for vnc in cfg.vnc_server.iter() {
        let screen = vnc.screen.expect("resolved above");
        if !screen_exists(screen) {
            return Err(format!(
                "`vnc-server` is bound to screen `{}`, but no {} device is configured",
                screen.as_str(),
                screen.source_option(),
            ));
        }
    }
    #[cfg(feature = "android_display")]
    for service in cfg.android_display_service.iter() {
        let screen = service.screen.expect("resolved above");
        if !screen_exists(screen) {
            return Err(format!(
                "`android-display-service` `{}` is bound to screen `{}`, but no {} device is \
                 configured",
                service.name,
                screen.as_str(),
                screen.source_option(),
            ));
        }
    }

    // Two servers cannot hold the same port, and two services cannot hold the same name -- the
    // second one loses at bind/register time, far from the option that caused it. Checked before
    // the per-screen rule so that a plain copy-pasted duplicate is reported as a duplicate rather
    // than as a screen conflict.
    #[cfg(feature = "vnc")]
    {
        // One listener per server, since the H.264 stream was folded onto the RFB port: there is
        // no derived second port left to collide with anything. What is still worth catching here
        // is the plain copy-paste, because the loser of a duplicate finds out at bind time, far
        // from the option that caused it.
        let mut ports: Vec<u32> = Vec::with_capacity(cfg.vnc_server.len());
        for vnc in cfg.vnc_server.iter() {
            let port = vnc.effective_port();
            if ports.contains(&port) {
                return Err(format!(
                    "port {} is claimed by two `vnc-server` options; each needs its own",
                    port
                ));
            }
            ports.push(port);
        }
    }
    #[cfg(feature = "android_display")]
    {
        let mut names: Vec<&str> = Vec::with_capacity(cfg.android_display_service.len());
        for service in cfg.android_display_service.iter() {
            if names.contains(&service.name.as_str()) {
                return Err(format!(
                    "two `android-display-service` options use the name `{}`; each needs its own",
                    service.name
                ));
            }
            names.push(service.name.as_str());
        }
    }

    // One exporter per screen. Mirroring -- one screen feeding both a VNC server and the app's
    // Surface -- is a deliberate non-goal, not a limitation waiting to be lifted: the choice was
    // between making the existing "both configured" case mirror and making it an error, and this
    // is the error. A screen with no exporter at all stays perfectly legal.
    for screen in [DisplayScreen::Gpu0, DisplayScreen::Simplefb] {
        let mut exporters: Vec<String> = Vec::new();
        #[cfg(feature = "vnc")]
        for vnc in cfg.vnc_server.iter().filter(|v| v.screen == Some(screen)) {
            exporters.push(format!(
                "`vnc-server` on port {}",
                vnc.port.unwrap_or(DEFAULT_VNC_PORT)
            ));
        }
        #[cfg(feature = "android_display")]
        for service in cfg
            .android_display_service
            .iter()
            .filter(|s| s.screen == Some(screen))
        {
            exporters.push(format!("`android-display-service` `{}`", service.name));
        }
        if exporters.len() > 1 {
            return Err(format!(
                "screen `{}` has {} exporters ({}); a screen drives at most one, and mirroring one \
                 screen onto several exporters is not supported. Give each a `screen=` of its own.",
                screen.as_str(),
                exporters.len(),
                exporters.join(", "),
            ));
        }
    }

    Ok(())
}

pub fn validate_config(cfg: &mut Config) -> std::result::Result<(), String> {
    if cfg.executable_path.is_none() {
        return Err("Executable is not specified".to_string());
    }

    if cfg.plugin_root.is_some() && !executable_is_plugin(&cfg.executable_path) {
        return Err("`plugin-root` requires `plugin`".to_string());
    }

    #[cfg(feature = "gpu")]
    {
        crate::crosvm::gpu_config::validate_gpu_config(cfg)?;
    }
    #[cfg(feature = "gdb")]
    if cfg.gdb.is_some() && cfg.vcpu_count.unwrap_or(1) != 1 {
        return Err("`gdb` requires the number of vCPU to be 1".to_string());
    }
    if cfg.host_cpu_topology {
        if cfg.no_smt {
            return Err(
                "`host-cpu-topology` cannot be set at the same time as `no_smt`, since \
                the smt of the Guest is the same as that of the Host when \
                `host-cpu-topology` is set."
                    .to_string(),
            );
        }

        let pcpu_count =
            base::number_of_logical_cores().expect("Could not read number of logical cores");
        if let Some(vcpu_count) = cfg.vcpu_count {
            if pcpu_count != vcpu_count {
                return Err(format!(
                    "`host-cpu-topology` requires the count of vCPUs({}) to equal the \
                            count of CPUs({}) on host.",
                    vcpu_count, pcpu_count
                ));
            }
        } else {
            cfg.vcpu_count = Some(pcpu_count);
        }

        match &cfg.vcpu_affinity {
            None => {
                let mut affinity_map = BTreeMap::new();
                for cpu_id in 0..cfg.vcpu_count.unwrap() {
                    affinity_map.insert(cpu_id, CpuSet::new([cpu_id]));
                }
                cfg.vcpu_affinity = Some(VcpuAffinity::PerVcpu(affinity_map));
            }
            _ => {
                return Err(
                    "`host-cpu-topology` requires not to set `cpu-affinity` at the same time"
                        .to_string(),
                );
            }
        }

        if !cfg.cpu_capacity.is_empty() {
            return Err(
                "`host-cpu-topology` requires not to set `cpu-capacity` at the same time"
                    .to_string(),
            );
        }

        if !cfg.cpu_clusters.is_empty() {
            return Err(
                "`host-cpu-topology` requires not to set `cpu clusters` at the same time"
                    .to_string(),
            );
        }
    }

    if cfg.boot_cpu >= cfg.vcpu_count.unwrap_or(1) {
        log::warn!("boot_cpu selection cannot be higher than vCPUs available, defaulting to 0");
        cfg.boot_cpu = 0;
    }

    #[cfg(all(
        any(target_arch = "arm", target_arch = "aarch64"),
        any(target_os = "android", target_os = "linux")
    ))]
    if !cfg.cpu_frequencies_khz.is_empty() {
        if !cfg.virt_cpufreq_v2 {
            return Err("`cpu-frequencies` requires `virt-cpufreq-upstream`".to_string());
        }

        if cfg.host_cpu_topology {
            return Err(
                "`host-cpu-topology` cannot be used with 'cpu-frequencies` at the same time"
                    .to_string(),
            );
        }
    }

    #[cfg(all(
        any(target_arch = "arm", target_arch = "aarch64"),
        any(target_os = "android", target_os = "linux")
    ))]
    if cfg.virt_cpufreq {
        if !cfg.host_cpu_topology && (cfg.vcpu_affinity.is_none() || cfg.cpu_capacity.is_empty()) {
            return Err("`virt-cpufreq` requires 'host-cpu-topology' enabled or \
                       have vcpu_affinity and cpu_capacity configured"
                .to_string());
        }
    }
    #[cfg(target_arch = "x86_64")]
    if !cfg.vcpu_hybrid_type.is_empty() {
        if cfg.host_cpu_topology {
            return Err("`core-types` cannot be set with `host-cpu-topology`.".to_string());
        }
        check_host_hybrid_support(&CpuIdCall::new(__cpuid_count, __cpuid))
            .map_err(|e| format!("the cpu doesn't support `core-types`: {}", e))?;
        if cfg.vcpu_hybrid_type.len() != cfg.vcpu_count.unwrap_or(1) {
            return Err("`core-types` must be set for all virtual CPUs".to_string());
        }
        for cpu_id in 0..cfg.vcpu_count.unwrap_or(1) {
            if !cfg.vcpu_hybrid_type.contains_key(&cpu_id) {
                return Err("`core-types` must be set for all virtual CPUs".to_string());
            }
        }
    }
    #[cfg(target_arch = "x86_64")]
    if cfg.enable_hwp && !cfg.host_cpu_topology {
        return Err("setting `enable-hwp` requires `host-cpu-topology` is set.".to_string());
    }
    #[cfg(target_arch = "x86_64")]
    if cfg.itmt {
        use std::collections::BTreeSet;
        // ITMT only works on the case each vCPU is 1:1 mapping to a pCPU.
        // `host-cpu-topology` has already set this 1:1 mapping. If no
        // `host-cpu-topology`, we need check the cpu affinity setting.
        if !cfg.host_cpu_topology {
            // only VcpuAffinity::PerVcpu supports setting cpu affinity
            // for each vCPU.
            if let Some(VcpuAffinity::PerVcpu(v)) = &cfg.vcpu_affinity {
                // ITMT allows more pCPUs than vCPUs.
                if v.len() != cfg.vcpu_count.unwrap_or(1) {
                    return Err("`itmt` requires affinity to be set for every vCPU.".to_string());
                }

                let mut pcpu_set = BTreeSet::new();
                for cpus in v.values() {
                    if cpus.len() != 1 {
                        return Err(
                            "`itmt` requires affinity to be set 1 pCPU for 1 vCPU.".to_owned()
                        );
                    }
                    // Ensure that each vCPU corresponds to a different pCPU to avoid pCPU sharing,
                    // otherwise it will seriously affect the ITMT scheduling optimization effect.
                    if !pcpu_set.insert(cpus[0]) {
                        return Err(
                            "`cpu_host` requires affinity to be set different pVPU for each vCPU."
                                .to_owned(),
                        );
                    }
                }
            } else {
                return Err("`itmt` requires affinity to be set for every vCPU.".to_string());
            }
        }
        if !cfg.enable_hwp {
            return Err("setting `itmt` requires `enable-hwp` is set.".to_string());
        }
    }

    #[cfg(feature = "balloon")]
    {
        if !cfg.balloon && cfg.balloon_control.is_some() {
            return Err("'balloon-control' requires enabled balloon".to_string());
        }

        if !cfg.balloon && cfg.balloon_page_reporting {
            return Err("'balloon_page_reporting' requires enabled balloon".to_string());
        }
    }

    #[cfg(any(target_os = "android", target_os = "linux"))]
    if cfg.lock_guest_memory && cfg.jail_config.is_none() {
        return Err("'lock-guest-memory' and 'disable-sandbox' are mutually exclusive".to_string());
    }

    // TODO(b/253386409): Vmm-swap only support sandboxed devices until vmm-swap use
    // `devices::Suspendable` to suspend devices.
    #[cfg(feature = "swap")]
    if cfg.swap_dir.is_some() && cfg.jail_config.is_none() {
        return Err("'swap' and 'disable-sandbox' are mutually exclusive".to_string());
    }

    set_default_serial_parameters(
        &mut cfg.serial_parameters,
        cfg.vhost_user
            .iter()
            .any(|opt| opt.type_ == DeviceType::Console),
    );

    for mapping in cfg
        .file_backed_mappings_mmio
        .iter_mut()
        .chain(cfg.file_backed_mappings_ram.iter_mut())
    {
        validate_file_backed_mapping(mapping)?;
    }

    for pmem in cfg.pmems.iter() {
        validate_pmem(pmem)?;
    }

    if let Some(simplefb) = cfg.simplefb.as_ref() {
        validate_simplefb(simplefb)?;
    }

    // After the display devices are all known: which screens exist is exactly the input this
    // needs, and it writes each exporter's resolved screen back into the config.
    #[cfg(any(feature = "vnc", feature = "android_display"))]
    validate_display_exporters(cfg)?;

    // Validate platform specific things
    super::sys::config::validate_config(cfg)
}

fn validate_file_backed_mapping(mapping: &mut FileBackedMappingParameters) -> Result<(), String> {
    let pagesize_mask = pagesize() as u64 - 1;
    let aligned_address = mapping.address & !pagesize_mask;
    let aligned_size =
        ((mapping.address + mapping.size + pagesize_mask) & !pagesize_mask) - aligned_address;

    if mapping.align {
        mapping.address = aligned_address;
        mapping.size = aligned_size;
    } else if aligned_address != mapping.address || aligned_size != mapping.size {
        return Err(
            "--file-backed-mapping addr and size parameters must be page size aligned".to_string(),
        );
    }

    Ok(())
}

fn validate_simplefb(simplefb: &SimplefbConfig) -> Result<(), String> {
    // Zero is the value worth rejecting by name: it reads like "do not poll" but the bridge divides
    // by it to get a frame duration, so what it would actually mean is decided by arithmetic rather
    // than by anyone. There is no "off" here -- not configuring `--simplefb` is the off switch.
    if simplefb.poll_hz == 0 {
        return Err("`simplefb` poll-hz must be at least 1".to_string());
    }
    if simplefb.poll_hz > MAX_SIMPLEFB_POLL_HZ {
        return Err(format!(
            "`simplefb` poll-hz must be at most {}",
            MAX_SIMPLEFB_POLL_HZ
        ));
    }

    Ok(())
}

fn validate_pmem(pmem: &PmemOption) -> Result<(), String> {
    if (pmem.swap_interval.is_some() && pmem.vma_size.is_none())
        || (pmem.swap_interval.is_none() && pmem.vma_size.is_some())
    {
        return Err(
            "--pmem vma-size and swap-interval parameters must be specified together".to_string(),
        );
    }

    if pmem.ro && pmem.swap_interval.is_some() {
        return Err(
            "--pmem swap-interval parameter can only be set for writable pmem device".to_string(),
        );
    }

    Ok(())
}

#[cfg(test)]
#[allow(clippy::needless_update)]
mod tests {
    use argh::FromArgs;
    use devices::PciClassCode;
    use devices::StubPciParameters;
    #[cfg(target_arch = "x86_64")]
    use uuid::uuid;

    use super::*;

    fn config_from_args(args: &[&str]) -> Config {
        crate::crosvm::cmdline::RunCommand::from_args(&[], args)
            .unwrap()
            .try_into()
            .unwrap()
    }

    /// Same as `config_from_args` but keeps a validation failure instead of panicking on it, so a
    /// test can say which rule rejected the command line and not merely that something did.
    #[cfg(any(feature = "vnc", feature = "android_display"))]
    fn config_from_args_result(args: &[&str]) -> std::result::Result<Config, String> {
        Config::try_from(crate::crosvm::cmdline::RunCommand::from_args(&[], args).unwrap())
    }

    /// A `--simplefb` that parses; the geometry is never used by these tests, only its presence.
    #[cfg(any(feature = "vnc", feature = "android_display"))]
    const SIMPLEFB_ARG: &str = "width=1280,height=720";

    #[test]
    fn parse_cpu_opts() {
        let res: CpuOptions = from_key_values("").unwrap();
        assert_eq!(res, CpuOptions::default());

        // num_cores
        let res: CpuOptions = from_key_values("12").unwrap();
        assert_eq!(
            res,
            CpuOptions {
                num_cores: Some(12),
                ..Default::default()
            }
        );

        let res: CpuOptions = from_key_values("num-cores=16").unwrap();
        assert_eq!(
            res,
            CpuOptions {
                num_cores: Some(16),
                ..Default::default()
            }
        );

        // clusters
        let res: CpuOptions = from_key_values("clusters=[[0],[1],[2],[3]]").unwrap();
        assert_eq!(
            res,
            CpuOptions {
                clusters: vec![
                    CpuSet::new([0]),
                    CpuSet::new([1]),
                    CpuSet::new([2]),
                    CpuSet::new([3])
                ],
                ..Default::default()
            }
        );

        let res: CpuOptions = from_key_values("clusters=[[0-3]]").unwrap();
        assert_eq!(
            res,
            CpuOptions {
                clusters: vec![CpuSet::new([0, 1, 2, 3])],
                ..Default::default()
            }
        );

        let res: CpuOptions = from_key_values("clusters=[[0,2],[1,3],[4-7,12]]").unwrap();
        assert_eq!(
            res,
            CpuOptions {
                clusters: vec![
                    CpuSet::new([0, 2]),
                    CpuSet::new([1, 3]),
                    CpuSet::new([4, 5, 6, 7, 12])
                ],
                ..Default::default()
            }
        );

        #[cfg(target_arch = "x86_64")]
        {
            let res: CpuOptions = from_key_values("core-types=[atom=[1,3-7],core=[0,2]]").unwrap();
            assert_eq!(
                res,
                CpuOptions {
                    core_types: Some(CpuCoreType {
                        atom: CpuSet::new([1, 3, 4, 5, 6, 7]),
                        core: CpuSet::new([0, 2])
                    }),
                    ..Default::default()
                }
            );
        }

        // All together
        let res: CpuOptions = from_key_values("16,clusters=[[0],[4-6],[7]]").unwrap();
        assert_eq!(
            res,
            CpuOptions {
                num_cores: Some(16),
                clusters: vec![CpuSet::new([0]), CpuSet::new([4, 5, 6]), CpuSet::new([7])],
                ..Default::default()
            }
        );

        let res: CpuOptions = from_key_values("clusters=[[0-7],[30-31]],num-cores=32").unwrap();
        assert_eq!(
            res,
            CpuOptions {
                num_cores: Some(32),
                clusters: vec![CpuSet::new([0, 1, 2, 3, 4, 5, 6, 7]), CpuSet::new([30, 31])],
                ..Default::default()
            }
        );
    }

    #[test]
    fn parse_cpu_set_single() {
        assert_eq!(
            CpuSet::from_str("123").expect("parse failed"),
            CpuSet::new([123])
        );
    }

    #[test]
    fn parse_cpu_set_list() {
        assert_eq!(
            CpuSet::from_str("0,1,2,3").expect("parse failed"),
            CpuSet::new([0, 1, 2, 3])
        );
    }

    #[test]
    fn parse_cpu_set_range() {
        assert_eq!(
            CpuSet::from_str("0-3").expect("parse failed"),
            CpuSet::new([0, 1, 2, 3])
        );
    }

    #[test]
    fn parse_cpu_set_list_of_ranges() {
        assert_eq!(
            CpuSet::from_str("3-4,7-9,18").expect("parse failed"),
            CpuSet::new([3, 4, 7, 8, 9, 18])
        );
    }

    #[test]
    fn parse_cpu_set_repeated() {
        // For now, allow duplicates - they will be handled gracefully by the vec to cpu_set_t
        // conversion.
        assert_eq!(
            CpuSet::from_str("1,1,1").expect("parse failed"),
            CpuSet::new([1, 1, 1])
        );
    }

    #[test]
    fn parse_cpu_set_negative() {
        // Negative CPU numbers are not allowed.
        CpuSet::from_str("-3").expect_err("parse should have failed");
    }

    #[test]
    fn parse_cpu_set_reverse_range() {
        // Ranges must be from low to high.
        CpuSet::from_str("5-2").expect_err("parse should have failed");
    }

    #[test]
    fn parse_cpu_set_open_range() {
        CpuSet::from_str("3-").expect_err("parse should have failed");
    }

    #[test]
    fn parse_cpu_set_extra_comma() {
        CpuSet::from_str("0,1,2,").expect_err("parse should have failed");
    }

    #[test]
    fn parse_cpu_affinity_global() {
        assert_eq!(
            parse_cpu_affinity("0,5-7,9").expect("parse failed"),
            VcpuAffinity::Global(CpuSet::new([0, 5, 6, 7, 9])),
        );
    }

    #[test]
    fn parse_cpu_affinity_per_vcpu_one_to_one() {
        let mut expected_map = BTreeMap::new();
        expected_map.insert(0, CpuSet::new([0]));
        expected_map.insert(1, CpuSet::new([1]));
        expected_map.insert(2, CpuSet::new([2]));
        expected_map.insert(3, CpuSet::new([3]));
        assert_eq!(
            parse_cpu_affinity("0=0:1=1:2=2:3=3").expect("parse failed"),
            VcpuAffinity::PerVcpu(expected_map),
        );
    }

    #[test]
    fn parse_cpu_affinity_per_vcpu_sets() {
        let mut expected_map = BTreeMap::new();
        expected_map.insert(0, CpuSet::new([0, 1, 2]));
        expected_map.insert(1, CpuSet::new([3, 4, 5]));
        expected_map.insert(2, CpuSet::new([6, 7, 8]));
        assert_eq!(
            parse_cpu_affinity("0=0,1,2:1=3-5:2=6,7-8").expect("parse failed"),
            VcpuAffinity::PerVcpu(expected_map),
        );
    }

    #[test]
    fn parse_mem_opts() {
        let res: MemOptions = from_key_values("").unwrap();
        assert_eq!(res.size, None);

        let res: MemOptions = from_key_values("1024").unwrap();
        assert_eq!(res.size, Some(1024));

        let res: MemOptions = from_key_values("size=0x4000").unwrap();
        assert_eq!(res.size, Some(16384));
    }

    #[test]
    fn parse_serial_vaild() {
        parse_serial_options("type=syslog,num=1,console=true,stdin=true")
            .expect("parse should have succeded");
    }

    #[test]
    fn parse_serial_virtio_console_vaild() {
        parse_serial_options("type=syslog,num=5,console=true,stdin=true,hardware=virtio-console")
            .expect("parse should have succeded");
    }

    #[test]
    fn parse_serial_valid_no_num() {
        parse_serial_options("type=syslog").expect("parse should have succeded");
    }

    #[test]
    fn parse_serial_equals_in_value() {
        let parsed = parse_serial_options("type=syslog,path=foo=bar==.log")
            .expect("parse should have succeded");
        assert_eq!(parsed.path, Some(PathBuf::from("foo=bar==.log")));
    }

    #[test]
    fn parse_serial_invalid_type() {
        parse_serial_options("type=wormhole,num=1").expect_err("parse should have failed");
    }

    #[test]
    fn parse_serial_invalid_num_upper() {
        parse_serial_options("type=syslog,num=5").expect_err("parse should have failed");
    }

    #[test]
    fn parse_serial_invalid_num_lower() {
        parse_serial_options("type=syslog,num=0").expect_err("parse should have failed");
    }

    #[test]
    fn parse_serial_virtio_console_invalid_num_lower() {
        parse_serial_options("type=syslog,hardware=virtio-console,num=0")
            .expect_err("parse should have failed");
    }

    #[test]
    fn parse_serial_invalid_num_string() {
        parse_serial_options("type=syslog,num=number3").expect_err("parse should have failed");
    }

    #[test]
    fn parse_serial_invalid_option() {
        parse_serial_options("type=syslog,speed=lightspeed").expect_err("parse should have failed");
    }

    #[test]
    fn parse_serial_invalid_two_stdin() {
        assert!(TryInto::<Config>::try_into(
            crate::crosvm::cmdline::RunCommand::from_args(
                &[],
                &[
                    "--serial",
                    "num=1,type=stdout,stdin=true",
                    "--serial",
                    "num=2,type=stdout,stdin=true"
                ]
            )
            .unwrap()
        )
        .is_err())
    }

    #[test]
    fn parse_serial_pci_address_valid_for_virtio() {
        let parsed =
            parse_serial_options("type=syslog,hardware=virtio-console,pci-address=00:0e.0")
                .expect("parse should have succeded");
        assert_eq!(
            parsed.pci_address,
            Some(PciAddress {
                bus: 0,
                dev: 14,
                func: 0
            })
        );
    }

    #[test]
    fn parse_serial_pci_address_valid_for_legacy_virtio() {
        let parsed =
            parse_serial_options("type=syslog,hardware=legacy-virtio-console,pci-address=00:0e.0")
                .expect("parse should have succeded");
        assert_eq!(
            parsed.pci_address,
            Some(PciAddress {
                bus: 0,
                dev: 14,
                func: 0
            })
        );
    }

    #[test]
    fn parse_serial_pci_address_failed_for_serial() {
        parse_serial_options("type=syslog,hardware=serial,pci-address=00:0e.0")
            .expect_err("expected pci-address error for serial hardware");
    }

    #[test]
    fn parse_serial_pci_address_failed_for_debugcon() {
        parse_serial_options("type=syslog,hardware=debugcon,pci-address=00:0e.0")
            .expect_err("expected pci-address error for debugcon hardware");
    }

    #[test]
    fn parse_battery_valid() {
        let bat_config: BatteryConfig = from_key_values("type=goldfish").unwrap();
        assert_eq!(bat_config.type_, BatteryType::Goldfish);
    }

    #[test]
    fn parse_battery_valid_no_type() {
        let bat_config: BatteryConfig = from_key_values("").unwrap();
        assert_eq!(bat_config.type_, BatteryType::Goldfish);
    }

    #[test]
    fn parse_battery_invalid_parameter() {
        from_key_values::<BatteryConfig>("tyep=goldfish").expect_err("parse should have failed");
    }

    #[test]
    fn parse_battery_invalid_type_value() {
        from_key_values::<BatteryConfig>("type=xxx").expect_err("parse should have failed");
    }

    #[test]
    fn parse_irqchip_kernel() {
        let cfg = TryInto::<Config>::try_into(
            crate::crosvm::cmdline::RunCommand::from_args(
                &[],
                &["--irqchip", "kernel", "/dev/null"],
            )
            .unwrap(),
        )
        .unwrap();

        assert_eq!(cfg.irq_chip, Some(IrqChipKind::Kernel));
    }

    #[test]
    fn parse_irqchip_split() {
        let cfg = TryInto::<Config>::try_into(
            crate::crosvm::cmdline::RunCommand::from_args(
                &[],
                &["--irqchip", "split", "/dev/null"],
            )
            .unwrap(),
        )
        .unwrap();

        assert_eq!(cfg.irq_chip, Some(IrqChipKind::Split));
    }

    #[test]
    fn parse_irqchip_userspace() {
        let cfg = TryInto::<Config>::try_into(
            crate::crosvm::cmdline::RunCommand::from_args(
                &[],
                &["--irqchip", "userspace", "/dev/null"],
            )
            .unwrap(),
        )
        .unwrap();

        assert_eq!(cfg.irq_chip, Some(IrqChipKind::Userspace));
    }

    #[test]
    fn parse_stub_pci() {
        let params = from_key_values::<StubPciParameters>("0000:01:02.3,vendor=0xfffe,device=0xfffd,class=0xffc1c2,subsystem_vendor=0xfffc,subsystem_device=0xfffb,revision=0xa").unwrap();
        assert_eq!(params.address.bus, 1);
        assert_eq!(params.address.dev, 2);
        assert_eq!(params.address.func, 3);
        assert_eq!(params.vendor, 0xfffe);
        assert_eq!(params.device, 0xfffd);
        assert_eq!(params.class.class as u8, PciClassCode::Other as u8);
        assert_eq!(params.class.subclass, 0xc1);
        assert_eq!(params.class.programming_interface, 0xc2);
        assert_eq!(params.subsystem_vendor, 0xfffc);
        assert_eq!(params.subsystem_device, 0xfffb);
        assert_eq!(params.revision, 0xa);
    }

    #[test]
    fn parse_file_backed_mapping_valid() {
        let params = from_key_values::<FileBackedMappingParameters>(
            "addr=0x1000,size=0x2000,path=/dev/mem,offset=0x3000,rw,sync",
        )
        .unwrap();
        assert_eq!(params.address, 0x1000);
        assert_eq!(params.size, 0x2000);
        assert_eq!(params.path, PathBuf::from("/dev/mem"));
        assert_eq!(params.offset, 0x3000);
        assert!(params.writable);
        assert!(params.sync);
    }

    #[test]
    fn parse_file_backed_mapping_incomplete() {
        assert!(
            from_key_values::<FileBackedMappingParameters>("addr=0x1000,size=0x2000")
                .unwrap_err()
                .contains("missing field `path`")
        );
        assert!(
            from_key_values::<FileBackedMappingParameters>("size=0x2000,path=/dev/mem")
                .unwrap_err()
                .contains("missing field `addr`")
        );
        assert!(
            from_key_values::<FileBackedMappingParameters>("addr=0x1000,path=/dev/mem")
                .unwrap_err()
                .contains("missing field `size`")
        );
    }

    #[test]
    fn parse_file_backed_mapping_unaligned_addr() {
        let mut params =
            from_key_values::<FileBackedMappingParameters>("addr=0x1001,size=0x2000,path=/dev/mem")
                .unwrap();
        assert!(validate_file_backed_mapping(&mut params)
            .unwrap_err()
            .contains("aligned"));
    }
    #[test]
    fn parse_file_backed_mapping_unaligned_size() {
        let mut params =
            from_key_values::<FileBackedMappingParameters>("addr=0x1000,size=0x2001,path=/dev/mem")
                .unwrap();
        assert!(validate_file_backed_mapping(&mut params)
            .unwrap_err()
            .contains("aligned"));
    }

    #[test]
    fn parse_file_backed_mapping_align() {
        let addr = pagesize() as u64 * 3 + 42;
        let size = pagesize() as u64 - 0xf;
        let mut params = from_key_values::<FileBackedMappingParameters>(&format!(
            "addr={addr},size={size},path=/dev/mem,align",
        ))
        .unwrap();
        assert_eq!(params.address, addr);
        assert_eq!(params.size, size);
        validate_file_backed_mapping(&mut params).unwrap();
        assert_eq!(params.address, pagesize() as u64 * 3);
        assert_eq!(params.size, pagesize() as u64 * 2);
    }

    #[test]
    fn parse_fw_cfg_valid_path() {
        let cfg = TryInto::<Config>::try_into(
            crate::crosvm::cmdline::RunCommand::from_args(
                &[],
                &["--fw-cfg", "name=bar,path=data.bin", "/dev/null"],
            )
            .unwrap(),
        )
        .unwrap();

        assert_eq!(cfg.fw_cfg_parameters.len(), 1);
        assert_eq!(cfg.fw_cfg_parameters[0].name, "bar".to_string());
        assert_eq!(cfg.fw_cfg_parameters[0].string, None);
        assert_eq!(cfg.fw_cfg_parameters[0].path, Some("data.bin".into()));
    }

    #[test]
    fn parse_fw_cfg_valid_string() {
        let cfg = TryInto::<Config>::try_into(
            crate::crosvm::cmdline::RunCommand::from_args(
                &[],
                &["--fw-cfg", "name=bar,string=foo", "/dev/null"],
            )
            .unwrap(),
        )
        .unwrap();

        assert_eq!(cfg.fw_cfg_parameters.len(), 1);
        assert_eq!(cfg.fw_cfg_parameters[0].name, "bar".to_string());
        assert_eq!(cfg.fw_cfg_parameters[0].string, Some("foo".to_string()));
        assert_eq!(cfg.fw_cfg_parameters[0].path, None);
    }

    #[test]
    fn parse_dtbo() {
        let cfg: Config = crate::crosvm::cmdline::RunCommand::from_args(
            &[],
            &[
                "--device-tree-overlay",
                "/path/to/dtbo1",
                "--device-tree-overlay",
                "/path/to/dtbo2",
                "/dev/null",
            ],
        )
        .unwrap()
        .try_into()
        .unwrap();

        assert_eq!(cfg.device_tree_overlay.len(), 2);
        for (opt, p) in cfg
            .device_tree_overlay
            .into_iter()
            .zip(["/path/to/dtbo1", "/path/to/dtbo2"])
        {
            assert_eq!(opt.path, PathBuf::from(p));
            assert!(!opt.filter_devs);
        }
    }

    #[test]
    #[cfg(any(target_os = "android", target_os = "linux"))]
    fn parse_dtbo_filtered() {
        let cfg: Config = crate::crosvm::cmdline::RunCommand::from_args(
            &[],
            &[
                "--vfio",
                "/path/to/dev,dt-symbol=mydev",
                "--device-tree-overlay",
                "/path/to/dtbo1,filter",
                "--device-tree-overlay",
                "/path/to/dtbo2,filter",
                "/dev/null",
            ],
        )
        .unwrap()
        .try_into()
        .unwrap();

        assert_eq!(cfg.device_tree_overlay.len(), 2);
        for (opt, p) in cfg
            .device_tree_overlay
            .into_iter()
            .zip(["/path/to/dtbo1", "/path/to/dtbo2"])
        {
            assert_eq!(opt.path, PathBuf::from(p));
            assert!(opt.filter_devs);
        }

        assert!(TryInto::<Config>::try_into(
            crate::crosvm::cmdline::RunCommand::from_args(
                &[],
                &["--device-tree-overlay", "/path/to/dtbo,filter", "/dev/null"],
            )
            .unwrap(),
        )
        .is_err());
    }

    #[test]
    fn parse_fw_cfg_invalid_no_name() {
        assert!(
            crate::crosvm::cmdline::RunCommand::from_args(&[], &["--fw-cfg", "string=foo",])
                .is_err()
        );
    }

    #[cfg(any(feature = "video-decoder", feature = "video-encoder"))]
    #[test]
    fn parse_video() {
        use devices::virtio::device_constants::video::VideoBackendType;

        #[cfg(feature = "libvda")]
        {
            let params: VideoDeviceConfig = from_key_values("libvda").unwrap();
            assert_eq!(params.backend, VideoBackendType::Libvda);

            let params: VideoDeviceConfig = from_key_values("libvda-vd").unwrap();
            assert_eq!(params.backend, VideoBackendType::LibvdaVd);
        }

        #[cfg(feature = "ffmpeg")]
        {
            let params: VideoDeviceConfig = from_key_values("ffmpeg").unwrap();
            assert_eq!(params.backend, VideoBackendType::Ffmpeg);
        }

        #[cfg(feature = "vaapi")]
        {
            let params: VideoDeviceConfig = from_key_values("vaapi").unwrap();
            assert_eq!(params.backend, VideoBackendType::Vaapi);
        }
    }

    #[test]
    fn parse_vhost_user_option() {
        let opt: VhostUserOption = from_key_values("/10mm").unwrap();
        assert_eq!(opt.socket.to_str(), Some("/10mm"));
        assert_eq!(opt.max_queue_size, None);

        let opt: VhostUserOption = from_key_values("/10mm,max-queue-size=256").unwrap();
        assert_eq!(opt.socket.to_str(), Some("/10mm"));
        assert_eq!(opt.max_queue_size, Some(256));
    }

    #[test]
    fn parse_vhost_user_option_all_device_types() {
        fn test_device_type(type_string: &str, type_: DeviceType) {
            let vhost_user_arg = format!("{},socket=sock", type_string);

            let cfg = TryInto::<Config>::try_into(
                crate::crosvm::cmdline::RunCommand::from_args(
                    &[],
                    &["--vhost-user", &vhost_user_arg, "/dev/null"],
                )
                .unwrap(),
            )
            .unwrap();

            assert_eq!(cfg.vhost_user.len(), 1);
            let vu = &cfg.vhost_user[0];
            assert_eq!(vu.type_, type_);
        }

        test_device_type("net", DeviceType::Net);
        test_device_type("block", DeviceType::Block);
        test_device_type("console", DeviceType::Console);
        test_device_type("rng", DeviceType::Rng);
        test_device_type("balloon", DeviceType::Balloon);
        test_device_type("scsi", DeviceType::Scsi);
        test_device_type("9p", DeviceType::P9);
        test_device_type("gpu", DeviceType::Gpu);
        test_device_type("input", DeviceType::Input);
        test_device_type("vsock", DeviceType::Vsock);
        test_device_type("iommu", DeviceType::Iommu);
        test_device_type("sound", DeviceType::Sound);
        test_device_type("fs", DeviceType::Fs);
        test_device_type("pmem", DeviceType::Pmem);
        test_device_type("mac80211-hwsim", DeviceType::Mac80211HwSim);
        test_device_type("video-encoder", DeviceType::VideoEncoder);
        test_device_type("video-decoder", DeviceType::VideoDecoder);
        test_device_type("scmi", DeviceType::Scmi);
        test_device_type("wl", DeviceType::Wl);
        test_device_type("tpm", DeviceType::Tpm);
        test_device_type("pvclock", DeviceType::Pvclock);
    }

    #[test]
    fn parse_vhost_user_fs_deprecated() {
        let cfg = TryInto::<Config>::try_into(
            crate::crosvm::cmdline::RunCommand::from_args(
                &[],
                &["--vhost-user-fs", "my_socket:my_tag", "/dev/null"],
            )
            .unwrap(),
        )
        .unwrap();

        assert_eq!(cfg.vhost_user_fs.len(), 1);
        let fs = &cfg.vhost_user_fs[0];
        let socket = fs.socket_path.as_ref().unwrap();
        assert_eq!(socket.to_str(), Some("my_socket"));
        assert_eq!(fs.tag, Some("my_tag".to_string()));
        assert_eq!(fs.max_queue_size, None);
        assert_eq!(fs.socket_fd, None);
    }

    #[test]
    fn parse_vhost_user_fs() {
        let cfg = TryInto::<Config>::try_into(
            crate::crosvm::cmdline::RunCommand::from_args(
                &[],
                &["--vhost-user-fs", "my_socket,tag=my_tag", "/dev/null"],
            )
            .unwrap(),
        )
        .unwrap();

        assert_eq!(cfg.vhost_user_fs.len(), 1);
        let fs = &cfg.vhost_user_fs[0];
        let socket = fs.socket_path.as_ref().unwrap();
        assert_eq!(socket.to_str(), Some("my_socket"));
        assert_eq!(fs.tag, Some("my_tag".to_string()));
        assert_eq!(fs.max_queue_size, None);
    }

    #[test]
    fn parse_vhost_user_fs_explict_socket() {
        let cfg = TryInto::<Config>::try_into(
            crate::crosvm::cmdline::RunCommand::from_args(
                &[],
                &[
                    "--vhost-user-fs",
                    "socket=my_socket,tag=my_tag",
                    "/dev/null",
                ],
            )
            .unwrap(),
        )
        .unwrap();

        assert_eq!(cfg.vhost_user_fs.len(), 1);
        let fs = &cfg.vhost_user_fs[0];
        let socket = fs.socket_path.as_ref().unwrap();
        assert_eq!(socket.to_str(), Some("my_socket"));
        assert_eq!(fs.tag, Some("my_tag".to_string()));
        assert_eq!(fs.max_queue_size, None);
    }

    #[test]
    fn parse_vhost_user_fs_max_queue_size() {
        let cfg = TryInto::<Config>::try_into(
            crate::crosvm::cmdline::RunCommand::from_args(
                &[],
                &[
                    "--vhost-user-fs",
                    "my_socket,tag=my_tag,max-queue-size=256",
                    "/dev/null",
                ],
            )
            .unwrap(),
        )
        .unwrap();

        assert_eq!(cfg.vhost_user_fs.len(), 1);
        let fs = &cfg.vhost_user_fs[0];
        let socket = fs.socket_path.as_ref().unwrap();
        assert_eq!(socket.to_str(), Some("my_socket"));
        assert_eq!(fs.tag, Some("my_tag".to_string()));
        assert_eq!(fs.max_queue_size, Some(256));
    }

    #[test]
    fn parse_vhost_user_fs_no_tag() {
        let cfg = TryInto::<Config>::try_into(
            crate::crosvm::cmdline::RunCommand::from_args(
                &[],
                &["--vhost-user-fs", "my_socket", "/dev/null"],
            )
            .unwrap(),
        )
        .unwrap();

        assert_eq!(cfg.vhost_user_fs.len(), 1);
        let fs = &cfg.vhost_user_fs[0];
        let socket = fs.socket_path.as_ref().unwrap();
        assert_eq!(socket.to_str(), Some("my_socket"));
        assert_eq!(fs.tag, None);
        assert_eq!(fs.max_queue_size, None);
    }

    #[test]
    fn parse_vhost_user_fs_socket_fd() {
        let cfg = TryInto::<Config>::try_into(
            crate::crosvm::cmdline::RunCommand::from_args(
                &[],
                &[
                    "--vhost-user-fs",
                    "tag=my_tag,max-queue-size=256,socket-fd=1234",
                    "/dev/null",
                ],
            )
            .unwrap(),
        )
        .unwrap();

        assert_eq!(cfg.vhost_user_fs.len(), 1);
        let fs = &cfg.vhost_user_fs[0];
        assert!(fs.socket_path.is_none());
        assert_eq!(fs.tag, Some("my_tag".to_string()));
        assert_eq!(fs.max_queue_size, Some(256));
        assert_eq!(fs.socket_fd.unwrap(), 1234_u32);
    }

    #[cfg(target_arch = "x86_64")]
    #[test]
    fn parse_smbios_uuid() {
        let opt: SmbiosOptions =
            from_key_values("uuid=12e474af-2cc1-49d1-b0e5-d03a3e03ca03").unwrap();
        assert_eq!(
            opt.uuid,
            Some(uuid!("12e474af-2cc1-49d1-b0e5-d03a3e03ca03"))
        );

        from_key_values::<SmbiosOptions>("uuid=zzzz").expect_err("expected error parsing uuid");
    }

    #[test]
    fn parse_touch_legacy() {
        let cfg = TryInto::<Config>::try_into(
            crate::crosvm::cmdline::RunCommand::from_args(
                &[],
                &["--multi-touch", "my_socket:867:5309", "bzImage"],
            )
            .unwrap(),
        )
        .unwrap();

        assert_eq!(cfg.virtio_input.len(), 1);
        let multi_touch = cfg
            .virtio_input
            .iter()
            .find(|input| matches!(input, InputDeviceOption::MultiTouch { .. }))
            .unwrap();
        assert_eq!(
            *multi_touch,
            InputDeviceOption::MultiTouch {
                path: PathBuf::from("my_socket"),
                width: Some(867),
                height: Some(5309),
                name: None
            }
        );
    }

    #[test]
    fn parse_touch() {
        let cfg = TryInto::<Config>::try_into(
            crate::crosvm::cmdline::RunCommand::from_args(
                &[],
                &["--multi-touch", r"C:\path,width=867,height=5309", "bzImage"],
            )
            .unwrap(),
        )
        .unwrap();

        assert_eq!(cfg.virtio_input.len(), 1);
        let multi_touch = cfg
            .virtio_input
            .iter()
            .find(|input| matches!(input, InputDeviceOption::MultiTouch { .. }))
            .unwrap();
        assert_eq!(
            *multi_touch,
            InputDeviceOption::MultiTouch {
                path: PathBuf::from(r"C:\path"),
                width: Some(867),
                height: Some(5309),
                name: None
            }
        );
    }

    // An absolute pointer is per guest output, and a guest maps one to an output by name, so the
    // name has to survive the command line. It also has to survive the *whole* name: spaces and
    // parentheses are what a screen-derived name is made of ("DroidVM Tablet (gpu-0)"), and the
    // key-value parser runs an unquoted value to the next ',' or ']', so those characters arrive
    // intact and only a stray comma would need quoting.
    #[test]
    fn parse_absolute_mouse_name() {
        let cfg = TryInto::<Config>::try_into(
            crate::crosvm::cmdline::RunCommand::from_args(
                &[],
                &[
                    "--input",
                    "absolute-mouse[path=/tmp/tablet.sock,name=DroidVM Tablet (gpu-0)]",
                    "bzImage",
                ],
            )
            .unwrap(),
        )
        .unwrap();

        assert_eq!(cfg.virtio_input.len(), 1);
        assert_eq!(
            cfg.virtio_input[0],
            InputDeviceOption::AbsoluteMouse {
                path: PathBuf::from("/tmp/tablet.sock"),
                width: None,
                height: None,
                name: Some("DroidVM Tablet (gpu-0)".to_string()),
            }
        );
    }

    // Omitting it stays legal -- the normalized-range tablet with no per-output identity is still
    // the shape every caller used before the field existed.
    #[test]
    fn parse_absolute_mouse_without_name() {
        let cfg = TryInto::<Config>::try_into(
            crate::crosvm::cmdline::RunCommand::from_args(
                &[],
                &["--input", "absolute-mouse[path=/tmp/tablet.sock]", "bzImage"],
            )
            .unwrap(),
        )
        .unwrap();

        assert_eq!(
            cfg.virtio_input[0],
            InputDeviceOption::AbsoluteMouse {
                path: PathBuf::from("/tmp/tablet.sock"),
                width: None,
                height: None,
                name: None,
            }
        );
    }

    // A keyboard belongs to a scanout now, so there is one per screen with input enabled and each
    // carries a name to tell them apart in the guest's device list. The name is not an output
    // mapping key the way a tablet's is -- a keyboard reports no coordinates -- so nothing breaks
    // silently if it changes; what it buys is a readable list.
    #[test]
    fn parse_keyboard_with_name() {
        let cfg = TryInto::<Config>::try_into(
            crate::crosvm::cmdline::RunCommand::from_args(
                &[],
                &[
                    "--input",
                    "keyboard[path=/tmp/kbd.sock,name=DroidVM Keyboard (gpu-0)]",
                    "bzImage",
                ],
            )
            .unwrap(),
        )
        .unwrap();

        assert_eq!(
            cfg.virtio_input[0],
            InputDeviceOption::Keyboard {
                path: PathBuf::from("/tmp/kbd.sock"),
                name: Some("DroidVM Keyboard (gpu-0)".to_string()),
            }
        );
    }

    // Omitting it stays legal: a single unnamed keyboard is every command line written before the
    // field existed, and the generated name is still right for it.
    #[test]
    fn parse_keyboard_without_name() {
        let cfg = TryInto::<Config>::try_into(
            crate::crosvm::cmdline::RunCommand::from_args(
                &[],
                &["--input", "keyboard[path=/tmp/kbd.sock]", "bzImage"],
            )
            .unwrap(),
        )
        .unwrap();

        assert_eq!(
            cfg.virtio_input[0],
            InputDeviceOption::Keyboard {
                path: PathBuf::from("/tmp/kbd.sock"),
                name: None,
            }
        );
    }

    // Several keyboards is the ordinary case now (one per screen), and they must come out as
    // several INDEPENDENT devices -- distinct options, distinct names, and distinct generated
    // indices downstream, which is what keeps their unique-id strings apart.
    #[test]
    fn parse_several_named_keyboards() {
        let cfg = TryInto::<Config>::try_into(
            crate::crosvm::cmdline::RunCommand::from_args(
                &[],
                &[
                    "--input",
                    "keyboard[path=/tmp/kbd-gpu0.sock,name=DroidVM Keyboard (gpu-0)]",
                    "--input",
                    "keyboard[path=/tmp/kbd-fb.sock,name=DroidVM Keyboard (simplefb)]",
                    "bzImage",
                ],
            )
            .unwrap(),
        )
        .unwrap();

        assert_eq!(cfg.virtio_input.len(), 2);
        assert_eq!(
            cfg.virtio_input[0],
            InputDeviceOption::Keyboard {
                path: PathBuf::from("/tmp/kbd-gpu0.sock"),
                name: Some("DroidVM Keyboard (gpu-0)".to_string()),
            }
        );
        assert_eq!(
            cfg.virtio_input[1],
            InputDeviceOption::Keyboard {
                path: PathBuf::from("/tmp/kbd-fb.sock"),
                name: Some("DroidVM Keyboard (simplefb)".to_string()),
            }
        );
    }

    // The relative mouse is the VM's, not a screen's -- it carries no output binding, so it takes
    // no name. `deny_unknown_fields` makes that a parse failure rather than a silently ignored key,
    // which is the behaviour the app's emitter relies on to learn that a field does not exist.
    #[test]
    fn mouse_rejects_name() {
        assert!(crate::crosvm::cmdline::RunCommand::from_args(
            &[],
            &["--input", "mouse[path=/tmp/mouse.sock,name=DroidVM Mouse]", "bzImage"],
        )
        .is_err());
    }

    #[test]
    fn single_touch_spec_and_track_pad_spec_default_size() {
        let config: Config = crate::crosvm::cmdline::RunCommand::from_args(
            &[],
            &[
                "--single-touch",
                "/dev/single-touch-test",
                "--trackpad",
                "/dev/single-touch-test",
                "/dev/null",
            ],
        )
        .unwrap()
        .try_into()
        .unwrap();

        let single_touch = config
            .virtio_input
            .iter()
            .find(|input| matches!(input, InputDeviceOption::SingleTouch { .. }))
            .unwrap();
        let trackpad = config
            .virtio_input
            .iter()
            .find(|input| matches!(input, InputDeviceOption::Trackpad { .. }))
            .unwrap();

        assert_eq!(
            *single_touch,
            InputDeviceOption::SingleTouch {
                path: PathBuf::from("/dev/single-touch-test"),
                width: None,
                height: None,
                name: None
            }
        );
        assert_eq!(
            *trackpad,
            InputDeviceOption::Trackpad {
                path: PathBuf::from("/dev/single-touch-test"),
                width: None,
                height: None,
                name: None
            }
        );
    }

    #[cfg(feature = "gpu")]
    #[test]
    fn single_touch_spec_default_size_from_gpu() {
        let config: Config = crate::crosvm::cmdline::RunCommand::from_args(
            &[],
            &[
                "--single-touch",
                "/dev/single-touch-test",
                "--gpu",
                "width=1024,height=768",
                "/dev/null",
            ],
        )
        .unwrap()
        .try_into()
        .unwrap();

        let single_touch = config
            .virtio_input
            .iter()
            .find(|input| matches!(input, InputDeviceOption::SingleTouch { .. }))
            .unwrap();
        assert_eq!(
            *single_touch,
            InputDeviceOption::SingleTouch {
                path: PathBuf::from("/dev/single-touch-test"),
                width: None,
                height: None,
                name: None
            }
        );

        assert_eq!(config.display_input_width, Some(1024));
        assert_eq!(config.display_input_height, Some(768));
    }

    #[test]
    fn single_touch_spec_and_track_pad_spec_with_size() {
        let config: Config = crate::crosvm::cmdline::RunCommand::from_args(
            &[],
            &[
                "--single-touch",
                "/dev/single-touch-test:12345:54321",
                "--trackpad",
                "/dev/single-touch-test:5678:9876",
                "/dev/null",
            ],
        )
        .unwrap()
        .try_into()
        .unwrap();

        let single_touch = config
            .virtio_input
            .iter()
            .find(|input| matches!(input, InputDeviceOption::SingleTouch { .. }))
            .unwrap();
        let trackpad = config
            .virtio_input
            .iter()
            .find(|input| matches!(input, InputDeviceOption::Trackpad { .. }))
            .unwrap();

        assert_eq!(
            *single_touch,
            InputDeviceOption::SingleTouch {
                path: PathBuf::from("/dev/single-touch-test"),
                width: Some(12345),
                height: Some(54321),
                name: None
            }
        );
        assert_eq!(
            *trackpad,
            InputDeviceOption::Trackpad {
                path: PathBuf::from("/dev/single-touch-test"),
                width: Some(5678),
                height: Some(9876),
                name: None
            }
        );
    }

    #[cfg(feature = "gpu")]
    #[test]
    fn single_touch_spec_with_size_independent_from_gpu() {
        let config: Config = crate::crosvm::cmdline::RunCommand::from_args(
            &[],
            &[
                "--single-touch",
                "/dev/single-touch-test:12345:54321",
                "--gpu",
                "width=1024,height=768",
                "/dev/null",
            ],
        )
        .unwrap()
        .try_into()
        .unwrap();

        let single_touch = config
            .virtio_input
            .iter()
            .find(|input| matches!(input, InputDeviceOption::SingleTouch { .. }))
            .unwrap();

        assert_eq!(
            *single_touch,
            InputDeviceOption::SingleTouch {
                path: PathBuf::from("/dev/single-touch-test"),
                width: Some(12345),
                height: Some(54321),
                name: None
            }
        );

        assert_eq!(config.display_input_width, Some(1024));
        assert_eq!(config.display_input_height, Some(768));
    }

    #[test]
    fn virtio_switches() {
        let config: Config = crate::crosvm::cmdline::RunCommand::from_args(
            &[],
            &["--switches", "/dev/switches-test", "/dev/null"],
        )
        .unwrap()
        .try_into()
        .unwrap();

        let switches = config
            .virtio_input
            .iter()
            .find(|input| matches!(input, InputDeviceOption::Switches { .. }))
            .unwrap();

        assert_eq!(
            *switches,
            InputDeviceOption::Switches {
                path: PathBuf::from("/dev/switches-test")
            }
        );
    }

    #[test]
    fn virtio_rotary() {
        let config: Config = crate::crosvm::cmdline::RunCommand::from_args(
            &[],
            &["--rotary", "/dev/rotary-test", "/dev/null"],
        )
        .unwrap()
        .try_into()
        .unwrap();

        let rotary = config
            .virtio_input
            .iter()
            .find(|input| matches!(input, InputDeviceOption::Rotary { .. }))
            .unwrap();

        assert_eq!(
            *rotary,
            InputDeviceOption::Rotary {
                path: PathBuf::from("/dev/rotary-test")
            }
        );
    }

    #[cfg(any(target_arch = "arm", target_arch = "aarch64"))]
    #[test]
    fn parse_pci_cam() {
        assert_eq!(
            config_from_args(&["--pci", "cam=[start=0x123]", "/dev/null"]).pci_config,
            PciConfig {
                cam: Some(arch::MemoryRegionConfig {
                    start: 0x123,
                    size: None,
                }),
                ..PciConfig::default()
            }
        );
        assert_eq!(
            config_from_args(&["--pci", "cam=[start=0x123,size=0x456]", "/dev/null"]).pci_config,
            PciConfig {
                cam: Some(arch::MemoryRegionConfig {
                    start: 0x123,
                    size: Some(0x456),
                }),
                ..PciConfig::default()
            },
        );
    }

    #[cfg(target_arch = "x86_64")]
    #[test]
    fn parse_pci_ecam() {
        assert_eq!(
            config_from_args(&["--pci", "ecam=[start=0x123]", "/dev/null"]).pci_config,
            PciConfig {
                ecam: Some(arch::MemoryRegionConfig {
                    start: 0x123,
                    size: None,
                }),
                ..PciConfig::default()
            }
        );
        assert_eq!(
            config_from_args(&["--pci", "ecam=[start=0x123,size=0x456]", "/dev/null"]).pci_config,
            PciConfig {
                ecam: Some(arch::MemoryRegionConfig {
                    start: 0x123,
                    size: Some(0x456),
                }),
                ..PciConfig::default()
            },
        );
    }

    #[test]
    fn parse_pci_mem() {
        assert_eq!(
            config_from_args(&["--pci", "mem=[start=0x123]", "/dev/null"]).pci_config,
            PciConfig {
                mem: Some(arch::MemoryRegionConfig {
                    start: 0x123,
                    size: None,
                }),
                ..PciConfig::default()
            }
        );
        assert_eq!(
            config_from_args(&["--pci", "mem=[start=0x123,size=0x456]", "/dev/null"]).pci_config,
            PciConfig {
                mem: Some(arch::MemoryRegionConfig {
                    start: 0x123,
                    size: Some(0x456),
                }),
                ..PciConfig::default()
            },
        );
    }

    #[test]
    fn parse_pmem_options_missing_path() {
        assert!(from_key_values::<PmemOption>("")
            .unwrap_err()
            .contains("missing field `path`"));
    }

    #[test]
    fn parse_pmem_options_default_values() {
        let pmem = from_key_values::<PmemOption>("/path/to/disk.img").unwrap();
        assert_eq!(
            pmem,
            PmemOption {
                path: "/path/to/disk.img".into(),
                ro: false,
                root: false,
                vma_size: None,
                swap_interval: None,
            }
        );
    }

    #[test]
    fn parse_pmem_options_virtual_swap() {
        let pmem =
            from_key_values::<PmemOption>("virtual_path,vma-size=12345,swap-interval-ms=1000")
                .unwrap();
        assert_eq!(
            pmem,
            PmemOption {
                path: "virtual_path".into(),
                ro: false,
                root: false,
                vma_size: Some(12345),
                swap_interval: Some(Duration::new(1, 0)),
            }
        );
    }

    #[test]
    fn validate_pmem_missing_virtual_swap_param() {
        let pmem = from_key_values::<PmemOption>("virtual_path,swap-interval-ms=1000").unwrap();
        assert!(validate_pmem(&pmem)
            .unwrap_err()
            .contains("vma-size and swap-interval parameters must be specified together"));
    }

    #[test]
    fn validate_pmem_read_only_virtual_swap() {
        let pmem = from_key_values::<PmemOption>(
            "virtual_path,ro=true,vma-size=12345,swap-interval-ms=1000",
        )
        .unwrap();
        assert!(validate_pmem(&pmem)
            .unwrap_err()
            .contains("swap-interval parameter can only be set for writable pmem device"));
    }

    // ---- exporter-to-screen bindings ----
    //
    // The two things these have to pin down are that nothing expressible before this option
    // repeated changed meaning, and that the cases which used to resolve themselves silently now
    // say what they resolved to (or refuse to).

    /// The form every existing command line uses: a bare service name, no `screen=`.
    #[test]
    #[cfg(feature = "android_display")]
    fn parse_android_display_service_bare_name() {
        assert_eq!(
            from_key_values::<AndroidDisplayServiceConfig>("droidvm_disp_1").unwrap(),
            AndroidDisplayServiceConfig {
                name: "droidvm_disp_1".to_string(),
                screen: None,
                transport_cap: TransportCap::Auto,
            }
        );
    }

    #[test]
    #[cfg(feature = "android_display")]
    fn parse_android_display_service_key_values() {
        assert_eq!(
            from_key_values::<AndroidDisplayServiceConfig>("name=win_fb,screen=simplefb").unwrap(),
            AndroidDisplayServiceConfig {
                name: "win_fb".to_string(),
                screen: Some(DisplayScreen::Simplefb),
                transport_cap: TransportCap::Auto,
            }
        );
        // The two forms mix: the name keeps its implicit first position with a `screen=` after it.
        assert_eq!(
            from_key_values::<AndroidDisplayServiceConfig>("droidvm_disp_1,screen=gpu-0").unwrap(),
            AndroidDisplayServiceConfig {
                name: "droidvm_disp_1".to_string(),
                screen: Some(DisplayScreen::Gpu0),
                transport_cap: TransportCap::Auto,
            }
        );
    }

    #[test]
    #[cfg(feature = "vnc")]
    fn parse_vnc_server_screen() {
        let vnc = from_key_values::<VncConfig>("host=127.0.0.1,port=5900,password=s").unwrap();
        assert_eq!(vnc.port, Some(5900));
        assert_eq!(vnc.screen, None);

        let vnc = from_key_values::<VncConfig>("port=5901,screen=simplefb").unwrap();
        assert_eq!(vnc.screen, Some(DisplayScreen::Simplefb));

        let vnc = from_key_values::<VncConfig>("port=5901,screen=gpu-0").unwrap();
        assert_eq!(vnc.screen, Some(DisplayScreen::Gpu0));
    }

    /// The transport ceiling, on both exporters, spelled the same way on both.
    ///
    /// The key is a contract with the app, so what is pinned here is the surface as typed --
    /// `transport-cap=cpu` -- and not merely that some field ends up set.
    #[test]
    #[cfg(feature = "vnc")]
    fn parse_vnc_server_transport_cap() {
        // Unsaid means auto, which is what "let the two ends negotiate" is called.
        let vnc = from_key_values::<VncConfig>("port=5900").unwrap();
        assert_eq!(vnc.transport_cap, TransportCap::Auto);

        let vnc = from_key_values::<VncConfig>("port=5900,transport-cap=cpu").unwrap();
        assert_eq!(vnc.transport_cap, TransportCap::Cpu);

        let vnc = from_key_values::<VncConfig>("port=5900,transport-cap=auto").unwrap();
        assert_eq!(vnc.transport_cap, TransportCap::Auto);

        // It combines with the other per-binding key rather than replacing it.
        let vnc = from_key_values::<VncConfig>("port=5901,screen=simplefb,transport-cap=cpu")
            .unwrap();
        assert_eq!(vnc.screen, Some(DisplayScreen::Simplefb));
        assert_eq!(vnc.transport_cap, TransportCap::Cpu);
    }

    #[test]
    #[cfg(feature = "android_display")]
    fn parse_android_display_service_transport_cap() {
        let svc = from_key_values::<AndroidDisplayServiceConfig>("droidvm_disp_1").unwrap();
        assert_eq!(svc.transport_cap, TransportCap::Auto);

        let svc = from_key_values::<AndroidDisplayServiceConfig>(
            "name=win_fb,screen=simplefb,transport-cap=cpu",
        )
        .unwrap();
        assert_eq!(svc.screen, Some(DisplayScreen::Simplefb));
        assert_eq!(svc.transport_cap, TransportCap::Cpu);

        // The bare-name form keeps working with the key appended, same as `screen=`.
        let svc =
            from_key_values::<AndroidDisplayServiceConfig>("droidvm_disp_1,transport-cap=cpu")
                .unwrap();
        assert_eq!(svc.name, "droidvm_disp_1");
        assert_eq!(svc.transport_cap, TransportCap::Cpu);
    }

    /// A ceiling nobody defined is a refusal, not a fallback to auto. Accepting an unknown value
    /// silently would be the caller believing they had asked for something.
    ///
    /// `gpu` used to be in this list, as the plausible-but-unimplemented rung above `cpu`. It is
    /// not any more, and that is the whole change: the rungs below are what exist, so `zero-copy`
    /// -- the one that is still only a design (plan §4.7) -- takes its place here.
    #[test]
    #[cfg(feature = "vnc")]
    fn parse_transport_cap_rejects_unknown_value() {
        assert!(from_key_values::<VncConfig>("port=5900,transport-cap=zero-copy").is_err());
        assert!(from_key_values::<VncConfig>("port=5900,transport-cap=gpu-hardware").is_err());
        assert!(from_key_values::<VncConfig>("port=5900,transport_cap=cpu").is_err());
    }

    /// The two rungs step 13 adds, and the meaning that separates them.
    ///
    /// `gpu` is the one that has to be got right: the app has been sending it, and reading it as
    /// "whatever the negotiation can reach" would turn the hardware encoder on for every binding
    /// that only ever asked to blit.
    #[test]
    #[cfg(feature = "vnc")]
    fn parse_transport_cap_gpu_rungs() {
        let vnc = from_key_values::<VncConfig>("port=5900,transport-cap=gpu").unwrap();
        assert_eq!(vnc.transport_cap, TransportCap::Gpu);
        assert!(vnc.transport_cap.allows_gpu_copy());
        assert!(!vnc.transport_cap.allows_hw_encode());

        let vnc = from_key_values::<VncConfig>("port=5900,transport-cap=gpu-hw").unwrap();
        assert_eq!(vnc.transport_cap, TransportCap::GpuHw);
        assert!(vnc.transport_cap.allows_gpu_copy());
        assert!(vnc.transport_cap.allows_hw_encode());

        // The two ends of the ladder keep the meanings they had before it grew.
        assert!(!TransportCap::Cpu.allows_gpu_copy());
        assert!(!TransportCap::Cpu.allows_hw_encode());
        assert!(TransportCap::Auto.allows_gpu_copy());
        assert!(TransportCap::Auto.allows_hw_encode());
    }

    /// Whether this binding may run the hardware encoder, and the one key that used to say where
    /// its stream came out.
    ///
    /// The stream rides the RFB port now (plans/H264_SINGLE_PORT.md), so `h264-port=` names a
    /// listener this crosvm does not open. A command line that still carries it was written against
    /// a different server, and the whole point of `deny_unknown_fields` here is that it FAILS
    /// rather than starting a VM whose stream is silently missing -- a mixed deploy is already
    /// forbidden, and a quietly dropped key is how a stale config passes a gate.
    #[test]
    #[cfg(feature = "vnc")]
    fn parse_vnc_server_rejects_the_retired_h264_port() {
        assert!(from_key_values::<VncConfig>("port=5900,h264-port=7100").is_err());
        assert!(from_key_values::<VncConfig>("port=5900,transport-cap=gpu-hw,h264-port=7100")
            .is_err());
        // The underscore spelling too: neither form has a field to land in.
        assert!(from_key_values::<VncConfig>("port=5900,h264_port=7100").is_err());

        // What is left is the ceiling, and it still decides.
        let vnc = from_key_values::<VncConfig>("port=5900").unwrap();
        assert!(vnc.h264_enabled());
        let vnc = from_key_values::<VncConfig>("port=5900,transport-cap=gpu-hw").unwrap();
        assert!(vnc.h264_enabled());
        let vnc = from_key_values::<VncConfig>("port=5900,transport-cap=gpu").unwrap();
        assert!(!vnc.h264_enabled());
        let vnc = from_key_values::<VncConfig>("port=5900,transport-cap=cpu").unwrap();
        assert!(!vnc.h264_enabled());

        // And it combines with the keys that were already there.
        let vnc =
            from_key_values::<VncConfig>("port=5901,screen=simplefb,transport-cap=gpu-hw").unwrap();
        assert_eq!(vnc.screen, Some(DisplayScreen::Simplefb));
        assert!(vnc.h264_enabled());
    }

    /// Whether this binding's clients drive anything, and the key that used to answer a different
    /// question in the same place.
    ///
    /// `input=` selected between shapes of a VM-global pointer set that no longer exists, so a
    /// command line still carrying it was written against a crosvm whose input wiring this one does
    /// not have. It fails for the same reason `h264-port=` does, and is pinned here for the same
    /// reason: a quietly dropped key is how a stale config passes a gate -- here it would produce a
    /// VM whose pointer lands on the wrong screen, or nowhere.
    #[test]
    #[cfg(feature = "vnc")]
    fn parse_vnc_server_view_only_and_the_retired_input_key() {
        // Unsaid means driven, which is what makes a bare `--vnc-server` usable.
        let vnc = from_key_values::<VncConfig>("port=5900").unwrap();
        assert!(!vnc.view_only);
        assert!(vnc.wants_input_devices());

        let vnc = from_key_values::<VncConfig>("port=5900,view-only=true").unwrap();
        assert!(vnc.view_only);
        assert!(!vnc.wants_input_devices());

        let vnc = from_key_values::<VncConfig>("port=5900,view-only=false").unwrap();
        assert!(vnc.wants_input_devices());

        // It is per binding, not per VM: two servers, two answers.
        let vnc = from_key_values::<VncConfig>("port=5901,screen=simplefb,view-only=true").unwrap();
        assert_eq!(vnc.screen, Some(DisplayScreen::Simplefb));
        assert!(!vnc.wants_input_devices());

        // Every spelling of the retired key, including the ones that used to be valid values.
        assert!(from_key_values::<VncConfig>("port=5900,input=tablet").is_err());
        assert!(from_key_values::<VncConfig>("port=5900,input=mouse").is_err());
        assert!(from_key_values::<VncConfig>("port=5900,input=touch").is_err());
        assert!(from_key_values::<VncConfig>("port=5900,input=none").is_err());
    }

    /// A single `--vnc-server` with no `screen=` and a GPU present: the shape of every VNC command
    /// line written so far, and it has to keep meaning the GPU's screen.
    #[test]
    #[cfg(all(feature = "vnc", feature = "gpu"))]
    fn display_exporter_defaults_to_gpu_screen() {
        let cfg = config_from_args(&["--gpu", "", "--vnc-server", "port=5900", "/dev/null"]);
        assert_eq!(cfg.vnc_server.len(), 1);
        assert_eq!(cfg.vnc_server[0].screen, Some(DisplayScreen::Gpu0));
        assert!(cfg.vnc_server_for(DisplayScreen::Gpu0).is_some());
        assert!(cfg.vnc_server_for(DisplayScreen::Simplefb).is_none());
    }

    /// The other half of the compat default: with no GPU the only screen is simplefb's, which is
    /// where the bridge presented on its own before any of this was expressible.
    #[test]
    #[cfg(feature = "vnc")]
    fn display_exporter_defaults_to_simplefb_without_gpu() {
        let cfg = config_from_args(&[
            "--simplefb",
            SIMPLEFB_ARG,
            "--vnc-server",
            "port=5900",
            "/dev/null",
        ]);
        assert_eq!(cfg.vnc_server[0].screen, Some(DisplayScreen::Simplefb));
        assert!(cfg.vnc_server_for(DisplayScreen::Simplefb).is_some());
    }

    #[test]
    #[cfg(all(feature = "android_display", feature = "gpu"))]
    fn display_exporter_legacy_android_service_resolves() {
        let cfg = config_from_args(&[
            "--gpu",
            "",
            "--android-display-service",
            "droidvm_disp_1",
            "/dev/null",
        ]);
        assert_eq!(cfg.android_display_service.len(), 1);
        assert_eq!(
            cfg.android_display_service_for(DisplayScreen::Gpu0)
                .map(|s| s.name.as_str()),
            Some("droidvm_disp_1")
        );
    }

    /// Two screens, one exporter each: the arrangement the whole change exists to make sayable.
    #[test]
    #[cfg(all(feature = "vnc", feature = "android_display", feature = "gpu"))]
    fn display_exporter_one_per_screen() {
        let cfg = config_from_args(&[
            "--gpu",
            "",
            "--simplefb",
            SIMPLEFB_ARG,
            "--vnc-server",
            "port=5900,screen=gpu-0",
            "--android-display-service",
            "name=win_fb,screen=simplefb",
            "/dev/null",
        ]);
        assert!(cfg.vnc_server_for(DisplayScreen::Gpu0).is_some());
        assert!(cfg.vnc_server_for(DisplayScreen::Simplefb).is_none());
        assert!(cfg
            .android_display_service_for(DisplayScreen::Gpu0)
            .is_none());
        assert_eq!(
            cfg.android_display_service_for(DisplayScreen::Simplefb)
                .map(|s| s.name.as_str()),
            Some("win_fb")
        );
    }

    /// A screen with no exporter is legal -- nobody is watching it, which is a state, not a fault.
    #[test]
    #[cfg(all(feature = "vnc", feature = "gpu"))]
    fn display_exporter_screen_without_exporter_is_legal() {
        let cfg = config_from_args(&[
            "--gpu",
            "",
            "--simplefb",
            SIMPLEFB_ARG,
            "--vnc-server",
            "port=5900,screen=gpu-0",
            "/dev/null",
        ]);
        assert!(cfg.vnc_server_for(DisplayScreen::Simplefb).is_none());
    }

    /// Both exporters, neither naming a screen, with a GPU: this used to start, with VNC taking
    /// the display and the service never registering. It is now the one thing that changed.
    #[test]
    #[cfg(all(feature = "vnc", feature = "android_display", feature = "gpu"))]
    fn display_exporter_rejects_two_on_one_screen() {
        let err = config_from_args_result(&[
            "--gpu",
            "",
            "--vnc-server",
            "port=5900",
            "--android-display-service",
            "droidvm_disp_1",
            "/dev/null",
        ])
        .err()
        .expect("expected the command line to be rejected");
        assert!(err.contains("screen `gpu-0` has 2 exporters"), "{}", err);
        assert!(err.contains("at most one"), "{}", err);
    }

    #[test]
    #[cfg(all(feature = "vnc", feature = "gpu"))]
    fn display_exporter_rejects_duplicate_vnc_port() {
        let err = config_from_args_result(&[
            "--gpu",
            "",
            "--simplefb",
            SIMPLEFB_ARG,
            "--vnc-server",
            "port=5900,screen=gpu-0",
            "--vnc-server",
            "port=5900,screen=simplefb",
            "/dev/null",
        ])
        .err()
        .expect("expected the command line to be rejected");
        assert!(err.contains("port 5900"), "{}", err);
    }

    /// The default port counts as a port: one entry saying `port=5900` and one saying nothing are
    /// the same port, and the collision has to be caught on the value the server would bind.
    #[test]
    #[cfg(all(feature = "vnc", feature = "gpu"))]
    fn display_exporter_rejects_duplicate_default_vnc_port() {
        let err = config_from_args_result(&[
            "--gpu",
            "",
            "--simplefb",
            SIMPLEFB_ARG,
            "--vnc-server",
            "screen=gpu-0",
            "--vnc-server",
            "port=5900,screen=simplefb",
            "/dev/null",
        ])
        .err()
        .expect("expected the command line to be rejected");
        assert!(err.contains("port 5900"), "{}", err);
    }

    #[test]
    #[cfg(all(feature = "android_display", feature = "gpu"))]
    fn display_exporter_rejects_duplicate_service_name() {
        let err = config_from_args_result(&[
            "--gpu",
            "",
            "--simplefb",
            SIMPLEFB_ARG,
            "--android-display-service",
            "name=dup,screen=gpu-0",
            "--android-display-service",
            "name=dup,screen=simplefb",
            "/dev/null",
        ])
        .err()
        .expect("expected the command line to be rejected");
        assert!(err.contains("name `dup`"), "{}", err);
    }

    /// Naming a screen whose device is absent. Loud on purpose: the alternative is an exporter
    /// that is configured, reports nothing, and shows nothing for the life of the VM.
    #[test]
    #[cfg(all(feature = "vnc", feature = "gpu"))]
    fn display_exporter_rejects_unconfigured_screen() {
        let err = config_from_args_result(&[
            "--gpu",
            "",
            "--vnc-server",
            "port=5900,screen=simplefb",
            "/dev/null",
        ])
        .err()
        .expect("expected the command line to be rejected");
        assert!(err.contains("screen `simplefb`"), "{}", err);
        assert!(err.contains("--simplefb"), "{}", err);

        let err = config_from_args_result(&[
            "--simplefb",
            SIMPLEFB_ARG,
            "--vnc-server",
            "port=5900,screen=gpu-0",
            "/dev/null",
        ])
        .err()
        .expect("expected the command line to be rejected");
        assert!(err.contains("screen `gpu-0`"), "{}", err);
        assert!(err.contains("--gpu"), "{}", err);
    }

    /// An exporter with no display device at all. Before, the value was quietly dropped (the
    /// service) or quietly kept and never opened (the VNC server); either way nothing said so.
    #[test]
    #[cfg(feature = "vnc")]
    fn display_exporter_rejects_no_display_device() {
        let err = config_from_args_result(&["--vnc-server", "port=5900", "/dev/null"])
            .err()
            .expect("expected the command line to be rejected");
        assert!(err.contains("no --simplefb device is configured"), "{}", err);
    }
}
