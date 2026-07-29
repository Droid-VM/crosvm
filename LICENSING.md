# Licensing

This repository holds two kinds of material and they are licensed differently.

## Material inherited from upstream

- **crosvm** (BSD-3-Clause) — https://chromium.googlesource.com/crosvm/crosvm

Every file that came from an upstream project stays under that project's
license. Nothing here relicenses it, and modifications to those files do not
relicense them either — a patched upstream file is still an upstream file.

## Material written for DroidVM

Files carrying `SPDX-License-Identifier: GPL-3.0-or-later` are DroidVM work
and are licensed under the GNU GPL, version 3 or later, **with the
additional permissions in `ADDITIONAL-PERMISSIONS`**.

Those permissions exist so this work can go upstream. They let anyone
relicense it under the terms an upstream project requires, for the purpose of
getting it merged there — and only for that purpose. Once upstream publishes
it, upstream's license governs that copy.

## Third-party material that is neither

Two added files are Rust ports of QEMU code and are **not** DroidVM-original:

- `hypervisor/src/gunyah/mthp.rs` — ported from QEMU's `gunyah_add_mem()`
- `devices/src/pl061.rs` — a translation of QEMU's `hw/gpio/pl061.c`

QEMU is GPL-licensed. These are marked `GPL-2.0-or-later` rather than
`GPL-3.0-or-later` so they stay compatible with the project they came from, and
the additional permissions do not extend to QEMU's copyright in them. Sending
either of them anywhere needs QEMU's license honoured, not this project's.

`gpu_display/src/vnc_server_bridge.{c,h}` are DroidVM-written but link
LibVNCServer (GPL-2.0-or-later); no LibVNCServer code is copied in.

`hypervisor/src/gunyah/gunyah_sys/bindings.rs` is bindgen output over Linux
UAPI headers. DroidVM's additions to it are ABI declarations, not expression.

## Contributing

See `CONTRIBUTING.md`. Sign-off is required; there is no CLA.
