// Copyright 2026 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! The host video decoder behind `--virtio-media kind=decoder` (`VPU_DESIGN.md` §7.2): the
//! `VideoDecoderBackend` the virtio-media crate's stateful decoder device drives, over the
//! MediaCodec NDK through the `android_codec` crate. On any other OS the same names exist and
//! the constructor fails, so the helper's `kind=decoder` arm is the same code everywhere.
//!
//! Only ever built inside the app-uid helper (`crosvm device media`): the codec services
//! resolve the caller from the real uid (design §7.4 keeps every codec device on the one
//! process model), so the in-VMM factory refuses `kind=decoder` without `uid=`.

#[cfg(target_os = "android")]
mod android;
#[cfg(target_os = "android")]
pub use android::MediaCodecDecoderBackend;
#[cfg(target_os = "android")]
pub use android::MediaCodecDecoderSession;

#[cfg(not(target_os = "android"))]
mod stub;
#[cfg(not(target_os = "android"))]
pub use stub::MediaCodecDecoderBackend;
#[cfg(not(target_os = "android"))]
pub use stub::MediaCodecDecoderSession;
