"""
Capture the ESP32-S3 PPG CSV stream over serial, save raw samples to a CSV file,
then run an FFT on the IR AC channel and compare the dominant frequency against
the BPM reported by the firmware's peak detector.

Expected line format from firmware:  ir_raw,ir_ac,red_raw,bpm
ESP_LOG lines and other non-numeric output are ignored.

Usage:
    python validate_ppg.py --port COM5 --duration 60
    python validate_ppg.py --port /dev/ttyUSB0 --duration 90 --no-plot
"""
import argparse
import csv
import sys
import time
from pathlib import Path

import numpy as np
import serial


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--port", required=True,
                   help="Serial port (e.g. COM5 on Windows, /dev/ttyUSB0 on Linux)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--duration", type=float, default=60.0,
                   help="Capture duration in seconds (default 60)")
    p.add_argument("--fs", type=float, default=100.0,
                   help="Expected sample rate in Hz (default 100, matches firmware)")
    p.add_argument("--output", default="ppg_capture.csv")
    p.add_argument("--no-plot", action="store_true")
    return p.parse_args()


def is_data_line(line: str) -> bool:
    parts = line.split(",")
    if len(parts) != 4:
        return False
    try:
        for p in parts:
            float(p)
        return True
    except ValueError:
        return False


def capture(port: str, baud: int, duration: float, output: Path):
    print(f"Opening {port} @ {baud} baud...")
    ser = serial.Serial(port, baud, timeout=1)
    # Drop any partial buffered line so we start on a clean record
    ser.reset_input_buffer()
    ser.readline()

    rows = []
    deadline = time.time() + duration
    last_report = time.time()
    print(f"Capturing for {duration:.1f} s. Stay still for a clean spectrum.")

    while time.time() < deadline:
        line = ser.readline().decode("ascii", errors="replace").strip()
        if not line or not is_data_line(line):
            continue
        ir_raw, ir_ac, red_raw, bpm = line.split(",")
        rows.append((float(ir_raw), float(ir_ac), float(red_raw), float(bpm)))

        now = time.time()
        if now - last_report >= 5.0:
            remaining = deadline - now
            current_bpm = rows[-1][3]
            print(f"  {len(rows):5d} samples  |  detector bpm = {current_bpm:5.1f}"
                  f"  |  {remaining:4.1f} s left")
            last_report = now

    ser.close()

    with output.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["ir_raw", "ir_ac", "red_raw", "bpm"])
        w.writerows(rows)

    print(f"\nCaptured {len(rows)} samples -> {output}")
    return np.array(rows) if rows else np.empty((0, 4))


def analyze(samples: np.ndarray, fs: float, plot: bool):
    n_total = len(samples)
    if n_total < int(fs * 10):
        print("WARNING: less than 10 s of data captured; FFT will be noisy.")

    # Drop the first 15 s so the firmware detector has time to lock on after
    # the previous run's stale BPM is cleared.
    skip = min(int(fs * 15), n_total // 3)
    samples = samples[skip:]
    n = len(samples)
    print(f"Analyzing {n} samples (dropped first {skip} for detector lock-on).")

    ir_ac = samples[:, 1]
    bpm_series = samples[:, 3]

    sig = ir_ac - np.mean(ir_ac)
    win = np.hanning(n)
    sig_w = sig * win

    spectrum = np.abs(np.fft.rfft(sig_w))
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)

    # Heart-rate band: 0.7 - 3.5 Hz = 42 - 210 BPM
    f_lo, f_hi = 0.7, 3.5
    mask = (freqs >= f_lo) & (freqs <= f_hi)
    band_freqs = freqs[mask]
    band_spec = spectrum[mask]
    if band_spec.size == 0:
        print("Not enough frequency resolution. Capture more data.")
        return

    raw_peak_idx = int(np.argmax(band_spec))
    raw_peak_hz = float(band_freqs[raw_peak_idx])

    # Harmonic-aware scoring: for each candidate fundamental, sum the spectral
    # energy at f, 2f, 3f. A true fundamental scores higher than its sub-
    # harmonic because the sub-harmonic only contributes at every second
    # harmonic of the candidate.
    def harmonic_score(f0: float) -> float:
        score = 0.0
        for k in (1, 2, 3):
            fk = f0 * k
            if fk > freqs[-1]:
                break
            idx = int(np.argmin(np.abs(freqs - fk)))
            score += float(spectrum[idx])
        return score

    scores = np.array([harmonic_score(f) for f in band_freqs])
    hps_idx = int(np.argmax(scores))
    peak_hz = float(band_freqs[hps_idx])
    peak_bpm = peak_hz * 60.0

    valid_bpm = bpm_series[bpm_series > 0]
    detector_bpm = float(np.median(valid_bpm)) if valid_bpm.size else float("nan")

    noise = float(np.median(band_spec))
    snr = (band_spec[raw_peak_idx] / noise) if noise > 0 else float("inf")

    print()
    print("=" * 60)
    print(f"FFT raw peak                : {raw_peak_hz:6.3f} Hz   ->  {raw_peak_hz*60:5.1f} BPM")
    print(f"FFT harmonic-aware peak     : {peak_hz:6.3f} Hz   ->  {peak_bpm:5.1f} BPM")
    print(f"Firmware detector (median)  : {detector_bpm:5.1f} BPM")
    if not np.isnan(detector_bpm):
        diff = abs(peak_bpm - detector_bpm)
        verdict = "AGREE" if diff <= 3.0 else ("CLOSE" if diff <= 6.0 else "DISAGREE")
        print(f"Difference                  : {diff:5.1f} BPM  [{verdict}]")
        # Diagnose 2:1 / 1:2 disagreements explicitly
        if not np.isnan(detector_bpm) and detector_bpm > 0:
            ratio = peak_bpm / detector_bpm
            if 1.85 <= ratio <= 2.15:
                print("  -> FFT is ~2x detector: detector likely missed every 2nd beat")
            elif 0.46 <= ratio <= 0.54:
                print("  -> FFT is ~0.5x detector: detector likely double-counted (dicrotic notch)")
    print(f"Peak / median in HR band    : {snr:5.1f}x   (>5 is clean, <2 is noisy)")
    print("=" * 60)

    if not plot:
        return
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not installed; skipping plot. (pip install matplotlib)")
        return

    t = np.arange(n) / fs
    fig, axes = plt.subplots(2, 1, figsize=(10, 6))
    axes[0].plot(t, sig, lw=0.7)
    axes[0].set(title="IR AC time series (DC-removed)",
                xlabel="time (s)", ylabel="counts")
    axes[0].grid(alpha=0.3)

    axes[1].plot(freqs, spectrum, lw=0.7)
    axes[1].axvline(peak_hz, color="r", lw=1.0,
                    label=f"peak {peak_hz:.2f} Hz  ({peak_bpm:.0f} BPM)")
    if not np.isnan(detector_bpm):
        axes[1].axvline(detector_bpm / 60.0, color="g", lw=1.0, ls="--",
                        label=f"detector {detector_bpm:.0f} BPM")
    axes[1].set(title="FFT magnitude spectrum",
                xlabel="frequency (Hz)", ylabel="|X(f)|", xlim=(0, 6))
    axes[1].legend()
    axes[1].grid(alpha=0.3)
    plt.tight_layout()
    plt.show()


def main():
    args = parse_args()
    samples = capture(args.port, args.baud, args.duration, Path(args.output))
    if len(samples) == 0:
        print("No samples captured. Is the firmware streaming CSV on this port?")
        print("If `idf.py monitor` is running, close it first - it holds the port.")
        sys.exit(1)
    analyze(samples, args.fs, plot=not args.no_plot)


if __name__ == "__main__":
    main()
