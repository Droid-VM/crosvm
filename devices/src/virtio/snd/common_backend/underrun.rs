// Copyright 2026 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

//! Packet-loss concealment for playback underruns.
//!
//! When the guest has not queued a period in time the device still has to hand the endpoint a
//! period of something. Zeroes are the honest answer and the most audible one: a hole in the
//! waveform is two discontinuities, and the ear hears both as a click. Speech and music are
//! full of transients that make the hole obvious; a steady tone hides it, which is why the
//! symptom looks content-dependent even though the transport carries the same bytes either way.
//!
//! What we do instead is what VoIP has done for decades: continue the waveform from what came
//! before. The method matters less than three details around it, and getting any of them wrong
//! sounds worse than silence:
//!
//! * **Splice on a pitch period.** Repeating an arbitrary chunk restarts the waveform at the
//!   wrong phase and buzzes. Autocorrelation over the tail finds the period to repeat, so voiced
//!   audio continues in phase.
//! * **Decay.** Concealment that holds at full amplitude turns a dropout into an endless drone.
//!   The gain falls to nothing across a few periods, so a long outage becomes a fade rather than
//!   a stuck note.
//! * **Blend back.** Real audio resuming at its own phase against the synthetic tail is a fresh
//!   discontinuity -- exactly the click we set out to remove. The first good period after a hole
//!   is crossfaded over the tail we invented.
//!
//! Only 16-bit little-endian PCM is concealed; anything else falls back to silence rather than
//! reinterpreting samples it does not understand.

use base::warn;

/// Longest pitch period searched: 20ms, below the ~50Hz floor of anything worth continuing.
const MAX_PITCH_MS: usize = 20;
/// Shortest: 2ms, i.e. 500Hz. Shorter lags find harmonics rather than the fundamental.
const MIN_PITCH_MS: usize = 2;
/// Periods of concealment before the output has faded to silence.
const FADE_PERIODS: u32 = 3;
/// Crossfade applied when real audio resumes, in milliseconds.
const BLEND_MS: usize = 2;

pub struct UnderrunConcealer {
    channels: usize,
    frame_rate: u32,
    period_bytes: usize,
    /// Scratch the caller fills with the incoming period, and `history` the one before it. They
    /// are swapped rather than copied, and both live here rather than being allocated per
    /// period: a heap allocation ninety-odd times a second on the path we are trying to keep
    /// punctual is a worse idea than any of the copying it would save.
    scratch: Vec<u8>,
    history: Vec<u8>,
    have_history: bool,
    /// How many periods in a row have been concealed; drives the fade.
    consecutive: u32,
    /// The concealed audio most recently produced, for blending real audio back in.
    tail: Vec<i16>,
    concealed_last: bool,
}

impl UnderrunConcealer {
    /// Returns None when the stream is not 16-bit PCM, or the geometry makes no sense.
    pub fn new(
        channels: usize,
        frame_rate: u32,
        period_bytes: usize,
        sample_bytes: usize,
    ) -> Option<Self> {
        if sample_bytes != 2 || channels == 0 || frame_rate == 0 || period_bytes == 0 {
            warn!(
                "underrun concealment unavailable: channels={} rate={} period={} sample_bytes={}",
                channels, frame_rate, period_bytes, sample_bytes
            );
            return None;
        }
        if period_bytes % (channels * 2) != 0 {
            return None;
        }
        let samples = period_bytes / 2;
        Some(UnderrunConcealer {
            channels,
            frame_rate,
            period_bytes,
            scratch: vec![0; period_bytes],
            history: vec![0; period_bytes],
            have_history: false,
            consecutive: 0,
            tail: vec![0; samples],
            concealed_last: false,
        })
    }

    pub fn period_bytes(&self) -> usize {
        self.period_bytes
    }

    /// The buffer the caller should read the incoming period into.
    pub fn scratch_mut(&mut self) -> &mut [u8] {
        &mut self.scratch
    }

    /// Called once the scratch holds `len` bytes of real audio. Blends over the concealed tail
    /// when this is the first real period after a hole, then makes it the history -- by swapping
    /// the two buffers, so keeping the history costs a pointer move rather than a copy. Returns
    /// the slice to hand to the endpoint.
    pub fn commit_good_period(&mut self, len: usize) -> &[u8] {
        if len != self.period_bytes {
            // A short or empty period; nothing useful to continue from later.
            self.reset();
            return &self.scratch[..len];
        }
        if self.concealed_last {
            // Borrow dance: blend_in needs &mut self for `tail` while writing into `scratch`.
            let mut buf = std::mem::take(&mut self.scratch);
            self.blend_in(&mut buf);
            self.scratch = buf;
        }
        std::mem::swap(&mut self.scratch, &mut self.history);
        self.have_history = true;
        self.consecutive = 0;
        self.concealed_last = false;
        &self.history[..len]
    }

    /// Produces a continuation of the last good period, or None when there is nothing to
    /// continue from and the caller should write silence. The sample conversion happens here,
    /// on the rare path, rather than on every good period.
    pub fn conceal(&mut self) -> Option<&[u8]> {
        if !self.have_history {
            return None;
        }
        let gain_num = FADE_PERIODS.saturating_sub(self.consecutive);
        if gain_num == 0 {
            // Faded out: silence is now the honest continuation.
            self.tail.fill(0);
            self.concealed_last = true;
            self.consecutive = self.consecutive.saturating_add(1);
            return None;
        }

        let frames = (self.period_bytes / 2) / self.channels;
        let lag = self.pitch_period(frames);
        // Continue from one pitch period before the end, so the splice lands in phase.
        let start = frames.saturating_sub(lag);
        for frame in 0..frames {
            // Fade within the period as well as across periods, so a long hole slopes away
            // smoothly instead of stepping down once per period. At frame f of concealment
            // period k the gain is (FADE_PERIODS - k - f/frames) / FADE_PERIODS.
            let num = (gain_num as i64) * (frames as i64) - (frame as i64);
            let gain_q15 = ((num * 32768) / (FADE_PERIODS as i64 * frames as i64)).clamp(0, 32768);
            let src_frame = start + (frame % lag.max(1));
            for ch in 0..self.channels {
                let off = (src_frame * self.channels + ch) * 2;
                let s = i16::from_le_bytes([self.history[off], self.history[off + 1]]) as i64;
                let v = ((s * gain_q15) >> 15) as i16;
                self.tail[frame * self.channels + ch] = v;
                let dst = (frame * self.channels + ch) * 2;
                self.scratch[dst..dst + 2].copy_from_slice(&v.to_le_bytes());
            }
        }
        self.concealed_last = true;
        self.consecutive = self.consecutive.saturating_add(1);
        Some(&self.scratch)
    }

    /// Crossfades the start of a real period over the concealed tail, so the return to real
    /// audio is not itself a discontinuity.
    fn blend_in(&mut self, pcm: &mut [u8]) {
        let blend_frames = (self.frame_rate as usize * BLEND_MS / 1000)
            .min(self.tail.len() / self.channels);
        if blend_frames == 0 {
            return;
        }
        for frame in 0..blend_frames {
            // Raised-cosine would be smoother; linear is inaudible over 2ms and has no table.
            let w_q15 = ((frame as i64 + 1) * 32768) / (blend_frames as i64 + 1);
            for ch in 0..self.channels {
                let idx = frame * self.channels + ch;
                let off = idx * 2;
                let real = i16::from_le_bytes([pcm[off], pcm[off + 1]]) as i64;
                let synth = self.tail[idx] as i64;
                let mixed = ((real * w_q15) + (synth * (32768 - w_q15))) >> 15;
                pcm[off..off + 2].copy_from_slice(&(mixed as i16).to_le_bytes());
            }
        }
    }

    /// Lag, in frames, of the strongest self-similarity in the tail of the history. Falls back
    /// to the whole history when nothing correlates, which degrades to a plain repeat.
    fn pitch_period(&self, frames: usize) -> usize {
        let min_lag = (self.frame_rate as usize * MIN_PITCH_MS / 1000).max(1);
        let max_lag = (self.frame_rate as usize * MAX_PITCH_MS / 1000).min(frames / 2);
        if max_lag <= min_lag {
            return frames.max(1);
        }
        // Correlate on the channel-summed signal: pitch is a property of the source, not of the
        // stereo image, and one pass costs a fraction of a period's worth of work.
        let mono: Vec<i32> = (0..frames)
            .map(|f| {
                (0..self.channels)
                    .map(|ch| {
                        let off = (f * self.channels + ch) * 2;
                        i16::from_le_bytes([self.history[off], self.history[off + 1]]) as i32
                    })
                    .sum::<i32>()
            })
            .collect();
        let window = max_lag;
        let base = frames - window;
        let mut best_lag = frames.max(1);
        let mut best_score = f64::MIN;
        for lag in min_lag..=max_lag {
            let mut dot = 0f64;
            let mut energy = 0f64;
            for i in 0..window {
                let a = mono[base + i] as f64;
                let b = mono[base + i - lag] as f64;
                dot += a * b;
                energy += b * b;
            }
            if energy <= 0.0 {
                continue;
            }
            // Normalised, so a loud lag does not beat a well-matched one.
            let score = dot / energy.sqrt();
            if score > best_score {
                best_score = score;
                best_lag = lag;
            }
        }
        best_lag
    }

    fn reset(&mut self) {
        self.have_history = false;
        self.consecutive = 0;
        self.concealed_last = false;
        self.tail.fill(0);
    }
}
