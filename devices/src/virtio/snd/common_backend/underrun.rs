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
//! * **Splice where the waveform matches.** Continuing from an arbitrary point restarts the
//!   waveform at the wrong phase and buzzes. Every method here picks its splice by similarity to
//!   what came before rather than by position.
//! * **Decay.** Concealment that holds at full amplitude turns a dropout into an endless drone.
//!   The gain falls to nothing across a few periods, so a long outage becomes a fade rather than
//!   a stuck note.
//! * **Blend back.** Real audio resuming at its own phase against the synthetic tail is a fresh
//!   discontinuity -- exactly the click we set out to remove. The first good period after a hole
//!   is crossfaded over the tail we invented.
//!
//! Those three are shared. What differs between the modes is only how the continuation itself is
//! generated:
//!
//! * [`UnderrunMode::Repeat`] finds one pitch period by autocorrelation and repeats it. Cheapest,
//!   and exactly right for a held vowel or a sustained note; a long hole becomes that one period
//!   over and over, which is why the fade matters most here.
//! * [`UnderrunMode::Wsola`] re-searches at every splice for the window that best continues what
//!   it has already produced, and overlap-adds there. The continuation wanders through the
//!   history instead of looping one period, so a long hole does not turn into a held note. It
//!   costs one normalised correlation per splice rather than one per hole.
//! * [`UnderrunMode::Lpc`] fits an all-pole filter to the history and runs it on, which continues
//!   the *spectrum* rather than reusing samples. Resonances ring on and noise decays, which is
//!   what a codec's concealment does. It is the only mode here that can produce samples that
//!   were never in the history, and the only one whose output is not bounded by it -- hence the
//!   stability guard.
//!
//! Every format the device negotiates is concealed. Samples are converted to floats on the way
//! in and back on the way out, which happens only on the rare path: a good period is kept as the
//! bytes it arrived as, and nothing is converted until a hole actually opens. The width matters
//! more than it looks like it should -- a guest that negotiates 32-bit is the common case here,
//! and a concealer that quietly only handled 16-bit would be a setting that does nothing.

use audio_streams::SampleFormat;
use base::info;
use base::warn;

use crate::virtio::snd::parameters::UnderrunMode;

/// Longest pitch period searched: 20ms, below the ~50Hz floor of anything worth continuing.
const MAX_PITCH_MS: usize = 20;
/// Shortest: 2ms, i.e. 500Hz. Shorter lags find harmonics rather than the fundamental.
const MIN_PITCH_MS: usize = 2;
/// Periods of concealment before the output has faded to silence.
const FADE_PERIODS: u32 = 3;
/// Crossfade applied when real audio resumes, in milliseconds.
const BLEND_MS: usize = 2;
/// Overlap-add window for WSOLA. Long enough to hide a splice, short enough that the search has
/// somewhere to move within one period of history.
const WSOLA_OVERLAP_MS: usize = 5;
/// Prediction order for LPC. Sixteen poles at 48kHz is roughly eight resonances, which covers a
/// voice's formants and an instrument's first few partials.
const LPC_ORDER: usize = 16;
/// Added to the zero-lag autocorrelation before the recursion: a hair of white noise keeps the
/// matrix positive definite when the history is near-silent or perfectly periodic. It is small
/// because it is not free -- for a pure tone the matrix is singular, and this term is then the
/// only thing setting how fast the extrapolated tone decays. At 1e-4 a 1kHz tone was down 88%
/// within one 10ms period, which is a plucked note rather than a continuation.
const LPC_RIDGE: f64 = 1.000_001;
/// Bandwidth expansion applied to the fitted filter: every pole's radius is multiplied by this,
/// which is what actually guarantees the extrapolation decays rather than grows. The ridge above
/// cannot be trusted for that job at the size it has to be, and a filter that grows would be a
/// far louder failure than the click we set out to hide.
const LPC_EXPANSION: f64 = 0.9995;
/// Extrapolated sample magnitude that counts as the filter having run away, in the normalised
/// scale samples are decoded to. Reaching it abandons LPC for the rest of the hole rather than
/// clipping at full scale for the rest of the hole.
const LPC_RUNAWAY: f64 = 4.0;

/// One sample, as a float in [-1, 1]. Every method here works in that scale, so the geometry of
/// the concealment does not have to know what the stream negotiated.
fn decode(format: SampleFormat, pcm: &[u8], index: usize) -> f32 {
    let at = index * format.sample_bytes();
    match format {
        SampleFormat::U8 => (pcm[at] as f32 - 128.0) / 128.0,
        SampleFormat::S16LE => {
            i16::from_le_bytes([pcm[at], pcm[at + 1]]) as f32 / -(i16::MIN as f32)
        }
        // 24-bit samples travel in 4 bytes with the top one padding, so the value has to be
        // sign-extended out of the low three rather than read as an i32.
        SampleFormat::S24LE => {
            let raw = i32::from_le_bytes([pcm[at], pcm[at + 1], pcm[at + 2], pcm[at + 3]]);
            ((raw << 8) >> 8) as f32 / 8_388_608.0
        }
        SampleFormat::S32LE => {
            i32::from_le_bytes([pcm[at], pcm[at + 1], pcm[at + 2], pcm[at + 3]]) as f32
                / -(i32::MIN as f32)
        }
        SampleFormat::F32LE => f32::from_le_bytes([pcm[at], pcm[at + 1], pcm[at + 2], pcm[at + 3]]),
    }
}

/// The inverse. Clamped, because concealment is arithmetic on real audio and arithmetic can
/// leave the range the format can hold.
fn encode(format: SampleFormat, value: f32, pcm: &mut [u8], index: usize) {
    let at = index * format.sample_bytes();
    let v = value.clamp(-1.0, 1.0);
    match format {
        SampleFormat::U8 => pcm[at] = (v * 128.0 + 128.0).clamp(0.0, 255.0) as u8,
        SampleFormat::S16LE => {
            let s = (v * -(i16::MIN as f32)).clamp(i16::MIN as f32, i16::MAX as f32) as i16;
            pcm[at..at + 2].copy_from_slice(&s.to_le_bytes());
        }
        SampleFormat::S24LE => {
            let s = (v * 8_388_608.0).clamp(-8_388_608.0, 8_388_607.0) as i32;
            pcm[at..at + 4].copy_from_slice(&s.to_le_bytes());
        }
        SampleFormat::S32LE => {
            let s = (v as f64 * -(i32::MIN as f64)).clamp(i32::MIN as f64, i32::MAX as f64) as i32;
            pcm[at..at + 4].copy_from_slice(&s.to_le_bytes());
        }
        SampleFormat::F32LE => pcm[at..at + 4].copy_from_slice(&v.to_le_bytes()),
    }
}

pub struct UnderrunConcealer {
    mode: UnderrunMode,
    format: SampleFormat,
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
    tail: Vec<f32>,
    concealed_last: bool,

    // ---- state built once per hole, in `prepare` ----
    /// `history` decoded, so a method that reads it many times over does not re-decode it. The
    /// conversion is on the rare path by design.
    hist: Vec<f32>,
    /// The same, summed across channels: pitch and similarity are properties of the source
    /// rather than of the stereo image, and one pass costs a fraction of a period's work.
    mono: Vec<f32>,
    /// Unity-gain continuation for the current period, before the fade is applied.
    fill: Vec<f32>,
    prepared: bool,
    /// Repeat: the lag to loop, found once per hole rather than once per period.
    lag: usize,
    /// WSOLA: the frames most recently emitted, which the next splice has to continue from.
    template: Vec<f32>,
    overlap: usize,
    /// LPC: `channels * order` coefficients, and the running synthesis state (oldest first).
    lpc_a: Vec<f64>,
    lpc_state: Vec<f64>,
    /// The history run through the inverse filter: what drove the resonances, with the
    /// resonances taken out. Interleaved like the audio it came from.
    lpc_res: Vec<f32>,
    /// How far into the repeated excitation we are; carried across periods so a long hole is one
    /// continuous excitation rather than the same stretch restarted every period.
    lpc_pos: usize,
    lpc_order: usize,
    lpc_ok: bool,
}

impl UnderrunConcealer {
    /// Returns None when the mode wants silence, or when the geometry makes no sense -- in
    /// either case the caller writes zeroes as it did before.
    pub fn new(
        mode: UnderrunMode,
        channels: usize,
        frame_rate: u32,
        period_bytes: usize,
        format: SampleFormat,
    ) -> Option<Self> {
        if mode == UnderrunMode::Silence {
            return None;
        }
        let frame_bytes = channels * format.sample_bytes();
        if channels == 0 || frame_rate == 0 || period_bytes == 0 || period_bytes % frame_bytes != 0
        {
            warn!(
                "underrun concealment unavailable: channels={} rate={} period={} format={}",
                channels, frame_rate, period_bytes, format
            );
            return None;
        }
        // Said once per stream, because "which concealment am I actually getting" is otherwise
        // only answerable by the absence of the warning above, and an absence is not an answer.
        info!(
            "underrun concealment: {:?}, {} channels at {}Hz, {}, {} byte periods",
            mode, channels, frame_rate, format, period_bytes
        );
        let samples = period_bytes / format.sample_bytes();
        Some(UnderrunConcealer {
            mode,
            format,
            channels,
            frame_rate,
            period_bytes,
            scratch: vec![0; period_bytes],
            history: vec![0; period_bytes],
            have_history: false,
            consecutive: 0,
            tail: vec![0.0; samples],
            concealed_last: false,
            hist: vec![0.0; samples],
            mono: vec![0.0; samples / channels],
            fill: vec![0.0; samples],
            prepared: false,
            lag: 0,
            template: vec![0.0; samples],
            overlap: 0,
            lpc_a: vec![0.0; channels * LPC_ORDER],
            lpc_state: vec![0.0; channels * LPC_ORDER],
            lpc_res: vec![0.0; samples],
            lpc_pos: 0,
            lpc_order: 0,
            lpc_ok: false,
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
        // The history has moved on, so everything derived from it has to be built again.
        self.prepared = false;
        &self.history[..len]
    }

    /// Produces a continuation of the last good period, or None when there is nothing to
    /// continue from and the caller should write silence.
    pub fn conceal(&mut self) -> Option<&[u8]> {
        if !self.have_history {
            return None;
        }
        let gain_num = FADE_PERIODS.saturating_sub(self.consecutive);
        if gain_num == 0 {
            // Faded out: silence is now the honest continuation.
            self.tail.fill(0.0);
            self.concealed_last = true;
            self.consecutive = self.consecutive.saturating_add(1);
            return None;
        }

        let frames = self.frames();
        if !self.prepared {
            self.prepare(frames);
            self.prepared = true;
        }

        // Each method writes `frames` frames of unity-gain continuation into `fill`; the fade,
        // the tail bookkeeping and the byte conversion below are the same for all of them.
        match self.mode {
            // new() refuses to build a concealer for Silence, so this cannot be reached.
            UnderrunMode::Silence => return None,
            UnderrunMode::Repeat => self.fill_repeat(frames),
            UnderrunMode::Wsola => self.fill_wsola(frames),
            UnderrunMode::Lpc => self.fill_lpc(frames),
        }

        for frame in 0..frames {
            // Fade within the period as well as across periods, so a long hole slopes away
            // smoothly instead of stepping down once per period. At frame f of concealment
            // period k the gain is (FADE_PERIODS - k - f/frames) / FADE_PERIODS.
            let num = gain_num as f32 * frames as f32 - frame as f32;
            let gain = (num / (FADE_PERIODS as f32 * frames as f32)).clamp(0.0, 1.0);
            for ch in 0..self.channels {
                let idx = frame * self.channels + ch;
                let v = self.fill[idx] * gain;
                self.tail[idx] = v;
                encode(self.format, v, &mut self.scratch, idx);
            }
        }
        self.concealed_last = true;
        self.consecutive = self.consecutive.saturating_add(1);
        Some(&self.scratch)
    }

    /// Decodes the history and builds whatever the mode needs from it. Called on the first
    /// concealed period of a hole: the history does not change while the hole lasts, so a pitch
    /// search or a filter fit belongs here rather than in every period.
    fn prepare(&mut self, frames: usize) {
        for i in 0..self.hist.len() {
            self.hist[i] = decode(self.format, &self.history, i);
        }
        for frame in 0..frames {
            self.mono[frame] = (0..self.channels)
                .map(|ch| self.hist[frame * self.channels + ch])
                .sum();
        }
        // Found for every mode, not just Repeat: it is what the other two fall back to when the
        // history will not support them, and one correlation per hole is not worth branching on.
        self.lag = self.pitch_period(frames);
        match self.mode {
            UnderrunMode::Repeat => {}
            UnderrunMode::Wsola => {
                // Seed the template with the real audio immediately before the hole, so the
                // first splice is matched against the signal rather than against our own output.
                let want = (self.frame_rate as usize * WSOLA_OVERLAP_MS / 1000).max(1);
                self.overlap = want.min(frames / 4);
                let overlap = self.overlap;
                if overlap > 0 {
                    let base = (frames - overlap) * self.channels;
                    self.template[..overlap * self.channels]
                        .copy_from_slice(&self.hist[base..base + overlap * self.channels]);
                }
            }
            UnderrunMode::Lpc => self.fit_lpc(frames),
            UnderrunMode::Silence => {}
        }
    }

    /// Repeats one pitch period, in phase. `lag` frames before the end is where the waveform
    /// last looked like the end, so continuing from there is continuing the same cycle.
    fn fill_repeat(&mut self, frames: usize) {
        let lag = self.lag.max(1);
        let start = frames.saturating_sub(lag);
        for frame in 0..frames {
            let src = start + (frame % lag);
            for ch in 0..self.channels {
                self.fill[frame * self.channels + ch] = self.hist[src * self.channels + ch];
            }
        }
    }

    /// Waveform Similarity Overlap-Add. Emits the history in chunks, each one starting wherever
    /// it best continues what has already been emitted, crossfaded in over `overlap` frames. The
    /// search moves, so consecutive chunks come from different places and the hole does not
    /// collapse into a loop.
    fn fill_wsola(&mut self, frames: usize) {
        let overlap = self.overlap;
        // Not enough history to overlap-add within: a plain pitch repeat is the honest fallback.
        if overlap == 0 || frames < overlap * 2 {
            self.fill_repeat(frames);
            return;
        }
        // The chunk taken after each splice. Half a period keeps the search space wide while
        // still moving on quickly enough that a period holds two or three splices.
        let hop = (frames / 2).saturating_sub(overlap).max(1);
        // Borrow dance: the search reads `self`, the write needs it mutably.
        let mut fill = std::mem::take(&mut self.fill);
        let mut template = std::mem::take(&mut self.template);

        let mut at = 0;
        while at < frames {
            let src = self.best_offset(&template[..overlap * self.channels], frames, overlap, hop);
            let n = (frames - at).min(overlap + hop);
            for frame in 0..n {
                for ch in 0..self.channels {
                    let s = self.hist[(src + frame) * self.channels + ch];
                    let v = if frame < overlap {
                        // Linear crossfade from what we emitted onto what we are splicing in.
                        let w = (frame as f32 + 1.0) / (overlap as f32 + 1.0);
                        let t = template[frame * self.channels + ch];
                        s * w + t * (1.0 - w)
                    } else {
                        s
                    };
                    fill[(at + frame) * self.channels + ch] = v;
                }
            }
            at += n;
            // The next splice continues from what was just emitted.
            let end = at * self.channels;
            let want = overlap * self.channels;
            if end >= want {
                template[..want].copy_from_slice(&fill[end - want..end]);
            }
        }

        self.fill = fill;
        self.template = template;
    }

    /// Offset into the history whose leading `overlap` frames best match `template`, scored by
    /// normalised cross-correlation on the channel sum. Restricted so a whole chunk fits after
    /// it.
    fn best_offset(&self, template: &[f32], frames: usize, overlap: usize, hop: usize) -> usize {
        let last = frames.saturating_sub(overlap + hop);
        if last == 0 {
            return 0;
        }
        let tmpl: Vec<f32> = (0..overlap)
            .map(|f| {
                (0..self.channels)
                    .map(|ch| template[f * self.channels + ch])
                    .sum()
            })
            .collect();
        let mut best_at = 0;
        let mut best_score = f64::MIN;
        for at in 0..=last {
            let mut dot = 0f64;
            let mut energy = 0f64;
            for f in 0..overlap {
                let a = tmpl[f] as f64;
                let b = self.mono[at + f] as f64;
                dot += a * b;
                energy += b * b;
            }
            if energy <= 0.0 {
                continue;
            }
            // Normalised, so a loud candidate does not beat a well-matched one.
            let score = dot / energy.sqrt();
            if score > best_score {
                best_score = score;
                best_at = at;
            }
        }
        best_at
    }

    /// Fits one all-pole filter per channel, by autocorrelation and the Levinson-Durbin
    /// recursion, and inverse-filters the history to recover the excitation that drove it.
    /// Clears `lpc_ok` when the history has nothing to fit, which sends the mode down the repeat
    /// path for the rest of the hole.
    fn fit_lpc(&mut self, frames: usize) {
        self.lpc_ok = false;
        self.lpc_pos = 0;
        let order = LPC_ORDER.min(frames / 4);
        if order < 2 {
            return;
        }
        self.lpc_order = order;
        for ch in 0..self.channels {
            // Hamming-windowed, as the recursion assumes the frame is one stationary segment and
            // an abrupt edge would show up as broadband energy that is not in the signal.
            let windowed: Vec<f64> = (0..frames)
                .map(|f| {
                    let w = 0.54
                        - 0.46 * (std::f64::consts::TAU * f as f64 / (frames as f64 - 1.0)).cos();
                    self.hist[f * self.channels + ch] as f64 * w
                })
                .collect();
            let mut r = vec![0f64; order + 1];
            for (lag, slot) in r.iter_mut().enumerate() {
                *slot = (lag..frames).map(|f| windowed[f] * windowed[f - lag]).sum();
            }
            let a_slot = ch * LPC_ORDER..ch * LPC_ORDER + order;
            if r[0] <= 0.0 {
                // Digital silence: there is no spectrum to continue and no excitation to find.
                self.lpc_a[a_slot].fill(0.0);
                self.lpc_state[ch * LPC_ORDER..ch * LPC_ORDER + order].fill(0.0);
                for f in 0..frames {
                    self.lpc_res[f * self.channels + ch] = 0.0;
                }
                continue;
            }
            r[0] *= LPC_RIDGE;
            let mut a = levinson(&r, order);
            // Bandwidth expansion: a[k] scaled by gamma^(k+1) is the same filter with every pole
            // pulled in by gamma. A fit on ten milliseconds cannot place a pole near enough to
            // the circle to ring on its own anyway -- the excitation below is what sustains the
            // output -- so this costs nothing and rules out a filter that grows.
            let mut gamma = LPC_EXPANSION;
            for coeff in a.iter_mut() {
                *coeff *= gamma;
                gamma *= LPC_EXPANSION;
            }
            self.lpc_a[a_slot].copy_from_slice(&a);
            // Inverse-filter the history: e[n] = x[n] + sum a[k] x[n-1-k]. Whatever the filter
            // does not explain is what drove it, and that is what we have to keep supplying.
            for f in 0..frames {
                let mut e = self.hist[f * self.channels + ch] as f64;
                for (k, coeff) in a.iter().enumerate() {
                    if f > k {
                        e += coeff * self.hist[(f - 1 - k) * self.channels + ch] as f64;
                    }
                }
                self.lpc_res[f * self.channels + ch] = e as f32;
            }
            // Synthesis continues from the real samples the hole interrupted, unwindowed: the
            // filter has to carry on the signal, not the tapered copy it was measured on.
            for k in 0..order {
                let f = frames - order + k;
                self.lpc_state[ch * LPC_ORDER + k] = self.hist[f * self.channels + ch] as f64;
            }
        }
        self.lpc_ok = true;
    }

    /// Runs each channel's filter on, driven by the excitation repeated at the pitch period.
    ///
    /// This is the half that makes LPC worth having. The filter alone cannot sustain anything:
    /// fitted to ten milliseconds, its poles sit too far inside the unit circle and its output
    /// dies within a period. What continues is the excitation -- repeated at the pitch lag, as
    /// Repeat does with the waveform -- but repeating the *residual* is a much smaller lie: the
    /// formants come from the filter rather than from copied samples, so the loop is not audible
    /// as a loop. It is the split every VoIP codec's concealment makes.
    fn fill_lpc(&mut self, frames: usize) {
        if !self.lpc_ok {
            self.fill_repeat(frames);
            return;
        }
        let order = self.lpc_order;
        // Kept inside the fully-computed part of the residual: the first `order` samples of it
        // were inverse-filtered with history the buffer does not have.
        let lag = self.lag.clamp(1, frames.saturating_sub(order).max(1));
        let base = frames - lag;
        let mut fill = std::mem::take(&mut self.fill);
        let mut state = std::mem::take(&mut self.lpc_state);
        let mut ran_away = false;
        'channels: for ch in 0..self.channels {
            let a = &self.lpc_a[ch * LPC_ORDER..ch * LPC_ORDER + order];
            let st = &mut state[ch * LPC_ORDER..ch * LPC_ORDER + order];
            let mut pos = self.lpc_pos;
            for frame in 0..frames {
                // x[n] = e[n] - sum a[k] x[n-1-k]; st is oldest first, so a[k] pairs with
                // st[order-1-k].
                let mut acc = self.lpc_res[(base + pos % lag) * self.channels + ch] as f64;
                for (k, coeff) in a.iter().enumerate() {
                    acc -= coeff * st[order - 1 - k];
                }
                if !acc.is_finite() || acc.abs() > LPC_RUNAWAY {
                    ran_away = true;
                    break 'channels;
                }
                st.rotate_left(1);
                st[order - 1] = acc;
                fill[frame * self.channels + ch] = acc as f32;
                pos += 1;
            }
            // Every channel walks the same excitation, so the position advances once, after.
            if ch + 1 == self.channels {
                self.lpc_pos = pos;
            }
        }
        self.fill = fill;
        self.lpc_state = state;
        if ran_away {
            // Bandwidth expansion should have made this impossible; if it happened anyway the
            // filter is not describing this signal, and repeating what we have is safe.
            self.lpc_ok = false;
            self.fill_repeat(frames);
        }
    }

    /// Crossfades the start of a real period over the concealed tail, so the return to real
    /// audio is not itself a discontinuity.
    fn blend_in(&mut self, pcm: &mut [u8]) {
        let blend_frames =
            (self.frame_rate as usize * BLEND_MS / 1000).min(self.tail.len() / self.channels);
        if blend_frames == 0 {
            return;
        }
        for frame in 0..blend_frames {
            // Raised-cosine would be smoother; linear is inaudible over 2ms and has no table.
            let w = (frame as f32 + 1.0) / (blend_frames as f32 + 1.0);
            for ch in 0..self.channels {
                let idx = frame * self.channels + ch;
                let real = decode(self.format, pcm, idx);
                let mixed = real * w + self.tail[idx] * (1.0 - w);
                encode(self.format, mixed, pcm, idx);
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
        let window = max_lag;
        let base = frames - window;
        let mut best_lag = frames.max(1);
        let mut best_score = f64::MIN;
        for lag in min_lag..=max_lag {
            let mut dot = 0f64;
            let mut energy = 0f64;
            for i in 0..window {
                let a = self.mono[base + i] as f64;
                let b = self.mono[base + i - lag] as f64;
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

    /// Frames in one period.
    fn frames(&self) -> usize {
        self.period_bytes / self.format.sample_bytes() / self.channels
    }

    fn reset(&mut self) {
        self.have_history = false;
        self.consecutive = 0;
        self.concealed_last = false;
        self.prepared = false;
        self.tail.fill(0.0);
    }
}

/// Levinson-Durbin: solves the Yule-Walker system for the prediction-error filter's coefficients
/// given the autocorrelation `r[0..=order]`. Returned without the leading 1, in the convention
/// where the signal continues as `x[n] = -sum a[k] x[n-1-k]`.
fn levinson(r: &[f64], order: usize) -> Vec<f64> {
    let mut a = vec![0f64; order];
    let mut error = r[0];
    if error <= 0.0 {
        return a;
    }
    for i in 0..order {
        // Reflection coefficient for this order.
        let mut rr = -r[i + 1];
        for j in 0..i {
            rr -= a[j] * r[i - j];
        }
        rr /= error;
        a[i] = rr;
        // In-place symmetric update of the lower orders: a[j] += rr * a[i-1-j], both ends at
        // once so neither reads a value the other has already overwritten.
        for j in 0..((i + 1) >> 1) {
            let t1 = a[j];
            let t2 = a[i - 1 - j];
            a[j] = t1 + rr * t2;
            a[i - 1 - j] = t2 + rr * t1;
        }
        error -= rr * rr * error;
        // The residual has stopped shrinking; further orders would be fitting arithmetic noise.
        if error < 0.001 * r[0] {
            break;
        }
    }
    a
}

#[cfg(test)]
mod tests {
    use super::*;

    const RATE: u32 = 48000;
    const CHANNELS: usize = 2;
    /// 512 frames of stereo: 10.7ms, the shape a low-latency configuration actually uses. The
    /// byte count follows the format, which is the whole point of the tests below.
    const FRAMES: usize = 512;
    const ALL: [UnderrunMode; 3] = [UnderrunMode::Repeat, UnderrunMode::Wsola, UnderrunMode::Lpc];
    /// Every width the device can negotiate. 32-bit is not decoration: it is what the Windows
    /// guest here actually picks.
    const FORMATS: [SampleFormat; 5] = [
        SampleFormat::U8,
        SampleFormat::S16LE,
        SampleFormat::S24LE,
        SampleFormat::S32LE,
        SampleFormat::F32LE,
    ];

    fn period(format: SampleFormat) -> usize {
        FRAMES * CHANNELS * format.sample_bytes()
    }

    fn concealer(mode: UnderrunMode, format: SampleFormat) -> UnderrunConcealer {
        UnderrunConcealer::new(mode, CHANNELS, RATE, period(format), format).expect("built")
    }

    /// Renders a signal, in whatever format the concealer is running.
    fn render(format: SampleFormat, offset: usize, f: &dyn Fn(f64) -> f64) -> Vec<u8> {
        let mut out = vec![0u8; period(format)];
        for frame in 0..FRAMES {
            let v = f((offset + frame) as f64 / RATE as f64) as f32;
            for ch in 0..CHANNELS {
                encode(format, v, &mut out, frame * CHANNELS + ch);
            }
        }
        out
    }

    fn tone(format: SampleFormat, offset: usize, freq: f64) -> Vec<u8> {
        render(format, offset, &move |t| {
            0.25 * (std::f64::consts::TAU * freq * t).sin()
        })
    }

    /// Voiced audio: a pulse train through a few partials, which is what the modes actually
    /// differ on. A pure tone is continued the same way by all three.
    fn voice(format: SampleFormat, offset: usize) -> Vec<u8> {
        render(format, offset, &|t| {
            let mut v = 0.0;
            for (h, a) in [(1.0, 1.0), (2.0, 0.6), (7.0, 0.35), (12.0, 0.2), (20.0, 0.1)] {
                v += a * (std::f64::consts::TAU * 120.0 * h * t).sin();
            }
            0.25 * v / 2.25
        })
    }

    fn decoded(format: SampleFormat, pcm: &[u8]) -> Vec<f32> {
        (0..pcm.len() / format.sample_bytes())
            .map(|i| decode(format, pcm, i))
            .collect()
    }

    fn rms(s: &[f32]) -> f64 {
        (s.iter().map(|&v| (v as f64) * (v as f64)).sum::<f64>() / s.len() as f64).sqrt()
    }

    fn feed(c: &mut UnderrunConcealer, pcm: &[u8]) {
        c.scratch_mut().copy_from_slice(pcm);
        c.commit_good_period(pcm.len());
    }

    #[test]
    fn silence_has_no_concealer() {
        for format in FORMATS {
            assert!(UnderrunConcealer::new(
                UnderrunMode::Silence,
                CHANNELS,
                RATE,
                period(format),
                format
            )
            .is_none());
        }
    }

    /// A period that is not a whole number of frames is a stream description we cannot work
    /// from, whatever the mode.
    #[test]
    fn ragged_geometry_is_refused() {
        for mode in ALL {
            assert!(
                UnderrunConcealer::new(mode, CHANNELS, RATE, 4095, SampleFormat::S16LE).is_none()
            );
            assert!(UnderrunConcealer::new(mode, 0, RATE, 4096, SampleFormat::S16LE).is_none());
        }
    }

    #[test]
    fn no_history_means_silence() {
        for mode in ALL {
            assert!(concealer(mode, SampleFormat::S16LE).conceal().is_none(), "{mode:?}");
        }
    }

    /// The point of all three, at every width the device can negotiate. Checked as level rather
    /// than as waveform, because the methods legitimately differ on what they emit.
    ///
    /// The formats are what makes this worth repeating: the concealer used to handle 16-bit
    /// only, and the guest that most needs concealment negotiates 32-bit.
    #[test]
    fn every_mode_continues_a_tone_in_every_format() {
        for format in FORMATS {
            for mode in ALL {
                let mut c = concealer(mode, format);
                feed(&mut c, &tone(format, 0, 440.0));
                let out = c.conceal().expect("concealed").to_vec();
                let level = rms(&decoded(format, &out));
                // A sine at 0.25 has an RMS of 0.177; the first concealed period is faded to
                // between 1 and 2/3 of that. U8 is coarse enough to move the figure a little.
                assert!(level > 0.06, "{mode:?}/{format} produced {level}");
                assert!(level < 0.19, "{mode:?}/{format} produced {level}");
            }
        }
    }

    /// The fade is what keeps a long outage from becoming a drone, so it is the one behaviour
    /// every mode has to share exactly.
    #[test]
    fn every_mode_fades_to_silence() {
        for mode in ALL {
            let format = SampleFormat::S16LE;
            let mut c = concealer(mode, format);
            feed(&mut c, &tone(format, 0, 440.0));
            let mut levels = Vec::new();
            for _ in 0..FADE_PERIODS {
                match c.conceal() {
                    Some(pcm) => levels.push(rms(&decoded(format, pcm))),
                    None => levels.push(0.0),
                }
            }
            assert!(c.conceal().is_none(), "{mode:?} still concealing after the fade");
            for pair in levels.windows(2) {
                assert!(pair[1] < pair[0], "{mode:?} did not decay: {levels:?}");
            }
        }
    }

    /// Digital silence has no spectrum and no pitch. Continuing it must not invent anything --
    /// LPC is the mode that could, since it is the one not bounded by the history.
    #[test]
    fn every_mode_continues_silence_as_silence() {
        for format in FORMATS {
            for mode in ALL {
                let mut c = concealer(mode, format);
                let quiet = render(format, 0, &|_| 0.0);
                feed(&mut c, &quiet);
                if let Some(pcm) = c.conceal() {
                    assert_eq!(pcm, &quiet[..], "{mode:?}/{format} invented audio from silence");
                }
            }
        }
    }

    /// WSOLA's reason to exist: it re-searches at every splice, so a hole is not one window
    /// repeated. Fed a sweep -- where every window is different -- its output must not be
    /// periodic at the splice spacing the way a plain repeat's is.
    #[test]
    fn wsola_does_not_lock_to_one_window() {
        let format = SampleFormat::S16LE;
        let mut phase = 0f64;
        let mut sweep = vec![0u8; period(format)];
        for frame in 0..FRAMES {
            let freq = 200.0 + 2000.0 * (frame as f64 / FRAMES as f64);
            phase += std::f64::consts::TAU * freq / RATE as f64;
            for ch in 0..CHANNELS {
                encode(format, (0.25 * phase.sin()) as f32, &mut sweep, frame * CHANNELS + ch);
            }
        }
        let mut c = concealer(UnderrunMode::Wsola, format);
        feed(&mut c, &sweep);
        let out = decoded(format, c.conceal().expect("concealed"));
        // Compare the two halves of the concealed period. A repeat of one window would make them
        // near-identical; a moving search makes them differ.
        let half = out.len() / 2;
        let diff: f64 = (0..half)
            .map(|i| (out[i] as f64 - out[half + i] as f64).abs())
            .sum::<f64>()
            / half as f64;
        assert!(diff > 0.003, "halves differ by only {diff}");
    }

    /// LPC continues the spectrum rather than the samples, so on a pure tone it should still be
    /// a tone -- and one whose zero crossings keep the original's spacing.
    #[test]
    fn lpc_holds_the_pitch_of_a_tone() {
        let format = SampleFormat::S32LE;
        let mut c = concealer(UnderrunMode::Lpc, format);
        feed(&mut c, &tone(format, 0, 1000.0));
        let out = decoded(format, c.conceal().expect("concealed"));
        // 1000Hz over 512 frames at 48kHz is ~10.7 cycles, so ~21 crossings. The band is wide on
        // purpose: the point is that it is neither silence nor noise, and that the pitch did not
        // halve or double.
        let ch0: Vec<f32> = out.iter().step_by(CHANNELS).copied().collect();
        let crossings = ch0.windows(2).filter(|w| (w[0] >= 0.0) != (w[1] >= 0.0)).count();
        assert!((16..=27).contains(&crossings), "{crossings} crossings");
    }

    /// Each mode has to be doing its own thing. Both of the newer ones fall back to a plain
    /// repeat when the history will not support them, and a fallback that fired every time would
    /// pass every other test in this file while quietly making the setting a no-op.
    #[test]
    fn the_modes_do_not_collapse_into_each_other() {
        let format = SampleFormat::S16LE;
        let mut outs = Vec::new();
        for mode in ALL {
            let mut c = concealer(mode, format);
            feed(&mut c, &voice(format, 0));
            outs.push(decoded(format, c.conceal().expect("concealed")));
        }
        for (i, j) in [(0, 1), (0, 2), (1, 2)] {
            let diff: f64 = outs[i]
                .iter()
                .zip(outs[j].iter())
                .map(|(&a, &b)| (a as f64 - b as f64).abs())
                .sum::<f64>()
                / outs[i].len() as f64;
            assert!(diff > 0.006, "modes {i} and {j} differ by only {diff}");
        }
    }

    /// The blend back is shared, and it is the difference between "no click on the way out" and
    /// "one click on the way out". Real audio resuming must not land as a step.
    #[test]
    fn resuming_real_audio_is_blended() {
        let format = SampleFormat::S16LE;
        let mut c = concealer(UnderrunMode::Repeat, format);
        feed(&mut c, &tone(format, 0, 440.0));
        c.conceal().expect("concealed");
        // Resume with a tone that is deliberately out of phase with what was concealed.
        let resumed = tone(format, FRAMES / 3, 440.0);
        c.scratch_mut().copy_from_slice(&resumed);
        let out = c.commit_good_period(resumed.len()).to_vec();
        // The first samples are mixed with the tail, so they differ from the raw period; later
        // ones are untouched.
        let blend = (RATE as usize * BLEND_MS / 1000).min(FRAMES) * CHANNELS * format.sample_bytes();
        assert_ne!(out[..format.sample_bytes()], resumed[..format.sample_bytes()]);
        assert_eq!(out[blend..], resumed[blend..]);
    }

    /// A short period is a stream that is stopping, not a hole to paper over.
    #[test]
    fn short_period_drops_the_history() {
        let format = SampleFormat::S16LE;
        let mut c = concealer(UnderrunMode::Wsola, format);
        feed(&mut c, &tone(format, 0, 440.0));
        c.scratch_mut()[..4].copy_from_slice(&[1, 2, 3, 4]);
        assert_eq!(c.commit_good_period(4).len(), 4);
        assert!(c.conceal().is_none());
    }
}
