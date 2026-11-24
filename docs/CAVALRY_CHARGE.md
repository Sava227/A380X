# Cavalry charge (A/P OFF aural), rebuilt (installed 2026-11-16)

**Problem:** the AP disconnect sound "sounds wrong". The old `cavalry_charge.wav` was my own guess, a bugle melody (G-C-E-G-E-G rising arpeggio, 1.2 s). The real FWC tone is nothing like a melody.

**The real tone, measured** from two reference recordings: the AirbusA350 add-on's `cavalry_ch.wav` and PA_A388's `cavalry3.wav`. Both are the same audio, with identical aliasing artefacts.

- **Waveform:** a square wave that alternates between **1660 Hz** and **830 Hz** (one octave) every **40 ms**, high first: H L H L H, a **0.2 s burst**.
- **Silence:** **0.2 s**, so the pattern repeats every **0.4 s**.
- **Aliasing:** the square is generated naively at **16 kHz**, so higher harmonics fold back (for example 9 × 1660 Hz lands at 1060 Hz). That folding is part of its harsh character.

**The new file** comes from `tools/make_cavalry.py`. It is synthesized from those measurements; no audio was copied from the reference recordings, which are third-party and stay out of the GPL repository.

- **Generation:** a naive 16 kHz square, then band-limited FFT resampling of the periodic signal to 44.1 kHz.
- **Length:** one 0.4 s period. The plugin loops it, and the seam is exact.
- **Level:** short-term A-weighted RMS 0.30, the same as `crc_master_warning.wav`. The table gain stays 0.8.

**Checks:**

- Burst edges match the reference exactly: 0.201 / 0.399 / 0.601 / 0.799 s.
- Per-segment log-spectrum correlation with the reference is 0.95 (high), 0.89 (low) and 0.91 (high).
- See `build/new_vs_ref.png`.

**Installed:**

- `plugins/A380XAvionics/sounds/cockpit/cavalry_charge.wav` and `provenance.json`.
- The same two files in the source worktree `avionics-src-20261115/native/avionics/sounds/cockpit/`. These changes are not committed.
- No plugin rebuild was needed: the WAV is loaded at runtime, and its length is only used as a "> 0" check.

**Preview:** `build/cavalry_charge_preview_2s.wav` (5 cycles).

