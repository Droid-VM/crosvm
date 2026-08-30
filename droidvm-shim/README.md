# droidvm-shim

The first thing a pseudo-unprotected VM runs.

crosvm starts such a VM with only a small lent region -- this shim and the device tree -- and
leaves the guest's real memory as a hole: declared to nobody, backed by nothing, until the host
SHAREs it as a Gunyah memparcel and *the guest* accepts it. The host cannot accept on the guest's
behalf (the resource manager refuses `MEM_ACCEPT_FLAG_MAP_OTHER`), so something inside the VM has
to do it before any real payload runs. That is this.

    [0x8000_0000  boot region, lent]    this shim, then the device tree
    [        ...  window, shared RWX]   the kernel or the firmware, the initrd, and all the RAM
    [        ...  pools, MMIO]          unchanged from a protected VM

It reads the handles the host left in the handoff page, accepts each parcel, points `/memory` at
the window, and jumps to the payload with `x0` still holding the device tree. Nothing jumps back
and nothing patches the payload: it sits at its own address, and this is simply what the
hypervisor started instead.

## Building

`../../2-0_build_shim.sh` builds it and drops `shim.bin` next to the crate that embeds it. soong
cannot build this -- bare metal, own linker script, flat binary -- so it is built with cargo
beforehand and picked up by `include_bytes!`.

    cargo test --lib --target x86_64-unknown-linux-gnu    # the device-tree walk, on the host

## The ABI

`../hypervisor/src/gunyah/shim_abi.rs` is compiled into both sides. There is no second copy,
because a mismatched field offset here is a VM that starts, hangs, and says nothing at all.
