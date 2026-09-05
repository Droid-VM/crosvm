// Copyright 2026 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! The host camera behind `--virtio-media kind=camera` (`VPU_DESIGN.md` §7.1): the
//! `CameraBackend` the virtio-media crate's camera device drives, over the Camera2 NDK through
//! the `android_camera` crate. On any other OS the same names exist and the constructor fails,
//! so the helper's `kind=camera` arm is the same code everywhere.
//!
//! Only ever built inside the app-uid helper (`crosvm device media`): `cameraserver` resolves
//! the caller's package from the real uid and refuses uid 0, so the in-VMM factory refuses
//! `kind=camera` without `uid=` rather than start a device that could never open its camera.

#[cfg(target_os = "android")]
mod android;
#[cfg(target_os = "android")]
pub use android::AndroidCameraBackend;
#[cfg(target_os = "android")]
pub use android::AndroidCameraStream;

#[cfg(not(target_os = "android"))]
mod stub;
#[cfg(not(target_os = "android"))]
pub use stub::AndroidCameraBackend;
#[cfg(not(target_os = "android"))]
pub use stub::AndroidCameraStream;
