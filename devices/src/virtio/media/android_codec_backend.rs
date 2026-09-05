// Copyright 2026 The DroidVM Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! The host video codecs behind `--virtio-media kind=decoder` and `kind=encoder`
//! (`VPU_DESIGN.md` §7.2, §7.3): the `VideoDecoderBackend` and `VideoEncoderBackend` the
//! virtio-media crate's stateful decoder and encoder devices drive, over the MediaCodec NDK
//! through the `android_codec` crate. On any other OS the same names exist and the constructors
//! fail, so the helper's codec arms are the same code everywhere.
//!
//! Only ever built inside the app-uid helper (`crosvm device media`): the codec services
//! resolve the caller from the real uid (design §7.4 keeps every codec device on the one
//! process model), so the in-VMM factory refuses both kinds without `uid=`.

#[cfg(target_os = "android")]
mod android;
#[cfg(target_os = "android")]
mod android_encoder;
#[cfg(target_os = "android")]
pub use android::MediaCodecDecoderBackend;
#[cfg(target_os = "android")]
pub use android::MediaCodecDecoderSession;
#[cfg(target_os = "android")]
pub use android_encoder::MediaCodecEncoderBackend;
#[cfg(target_os = "android")]
pub use android_encoder::MediaCodecEncoderSession;

#[cfg(not(target_os = "android"))]
mod stub;
#[cfg(not(target_os = "android"))]
pub use stub::MediaCodecDecoderBackend;
#[cfg(not(target_os = "android"))]
pub use stub::MediaCodecDecoderSession;
#[cfg(not(target_os = "android"))]
pub use stub::MediaCodecEncoderBackend;
#[cfg(not(target_os = "android"))]
pub use stub::MediaCodecEncoderSession;
