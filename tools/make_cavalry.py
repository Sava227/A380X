"""A380X cavalry charge (A/P OFF aural), 2026-11-16.

Measured from the reference recordings (pattern, pitches and timbre are facts of the FWC tone; no audio is copied):
  * a square wave alternating between 1660 Hz and 830 Hz (one octave) every 40 ms, high first: H L H L H = 0.2 s burst;
  * 0.2 s silence; period 0.4 s, repeated while the warning is active;
  * generated as a naive (non band-limited) square at 16 kHz, which reproduces the aliased partials of the FWC tone
    (e.g. 9th harmonic of 1660 Hz folding to 1060 Hz); then band-limited resampling to 44.1 kHz.
One 0.4 s period per file; the plugin loops it, and the FFT resampling of the periodic signal makes the seam exact.
Usage: make_cavalry.py <out.wav> <target short-term A-weighted RMS>
"""
import sys, wave, numpy as np
FS0, FS = 16000, 44100
F_HI, F_LO, SEG, BURST, PERIOD = 1660.0, 830.0, 0.040, 0.200, 0.400

def synth16k():
    n = int(PERIOD * FS0); x = np.zeros(n); ph = 0.0
    for i in range(int(BURST * FS0)):
        f = F_HI if int(i / (SEG * FS0)) % 2 == 0 else F_LO
        x[i] = 1.0 if ph < 0.5 else -1.0
        ph = (ph + f / FS0) % 1.0
    return x

def resample_periodic(x, fs_in, fs_out):
    n_out = int(round(len(x) * fs_out / fs_in)); X = np.fft.rfft(x); Y = np.zeros(n_out // 2 + 1, complex)
    k = min(len(X), len(Y)); Y[:k] = X[:k]
    if len(x) % 2 == 0: Y[k - 1] *= 0.5 if k == len(X) else 1.0        # split the input Nyquist bin
    return np.fft.irfft(Y, n_out) * n_out / len(x)

def a_weight_rms_max(y, fs, win=0.05):
    Y = np.fft.rfft(y); f = np.fft.rfftfreq(len(y), 1 / fs); f2 = f ** 2
    ra = (12194 ** 2 * f2 ** 2) / ((f2 + 20.6 ** 2) * np.sqrt((f2 + 107.7 ** 2) * (f2 + 737.9 ** 2)) * (f2 + 12194 ** 2) + 1e-30)
    ya = np.fft.irfft(Y * ra / ra[np.argmin(abs(f - 1000))], len(y)); h = int(win * fs)
    return float(np.sqrt(np.convolve(ya ** 2, np.ones(h) / h, 'valid').max()))

if __name__ == '__main__':
    out, target = sys.argv[1], float(sys.argv[2])
    y = resample_periodic(synth16k(), FS0, FS)
    y *= target / a_weight_rms_max(y, FS)
    assert np.abs(y).max() < 0.98, np.abs(y).max()
    w = wave.open(out, 'wb'); w.setnchannels(1); w.setsampwidth(2); w.setframerate(FS)
    w.writeframes((np.round(y * 32767)).astype('<i2').tobytes()); w.close()
    print(f'{out}: {len(y)/FS:.3f} s, peak {np.abs(y).max():.3f}, A-weighted short-term RMS {a_weight_rms_max(y, FS):.3f}')
