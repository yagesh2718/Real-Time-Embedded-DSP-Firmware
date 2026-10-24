#!/usr/bin/env python3
"""
Real-Time Embedded DSP Firmware Simulator
Visualization & Spectral Analysis Script

Generates publication-quality plots from simulation and benchmark CSV telemetry:
1. Time-Domain Signal Analysis (Raw Simulated ADC vs FIR Lowpass vs IIR Biquad)
2. Frequency Domain (FFT Magnitude Spectrum & Stopband Attenuation)
3. Fixed-Point Q15 vs Floating-Point Quantization Error Analysis
4. Host Benchmark Scaling & Execution Time vs Block Size
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

def plot_signals(sim_csv_path, output_png_path):
    if not os.path.exists(sim_csv_path):
        print(f"[ERROR] Simulation CSV file not found: {sim_csv_path}")
        return False

    print(f"[PLOT] Loading simulation data from {sim_csv_path}...")
    data = np.genfromtxt(sim_csv_path, delimiter=',', names=True)

    time = data['TimeSeconds']
    raw = data['RawAdcNormalized']
    fir_flt = data['FirFilteredFloat']
    iir_flt = data['IirFilteredFloat']
    fir_q15 = data['FirFilteredQ15']
    err_q15 = data['QuantizationErrorQ15']

    fs = 1.0 / (time[1] - time[0])
    num_samples = len(time)

    # Compute FFT spectra
    def compute_spectrum(signal):
        w = np.hanning(len(signal))
        spec = np.fft.rfft(signal * w)
        freqs = np.fft.rfftfreq(len(signal), d=1.0/fs)
        mag_db = 20.0 * np.log10(np.abs(spec) / (len(signal) / 4.0) + 1e-12)
        return freqs, mag_db

    freqs, raw_spec_db = compute_spectrum(raw)
    _, fir_spec_db = compute_spectrum(fir_flt)
    _, iir_spec_db = compute_spectrum(iir_flt)

    fig, axs = plt.subplots(4, 1, figsize=(12, 14), constrained_layout=True)
    fig.suptitle('Real-Time Embedded DSP Firmware: Simulated ADC & Filter Telemetry',
                 fontsize=14, fontweight='bold')

    # 1. Full Time Domain Waveform
    axs[0].plot(time * 1000.0, raw, label='Raw Simulated ADC (250Hz + 2500Hz + Noise)',
                color='#1f77b4', alpha=0.4, linewidth=1.0)
    axs[0].plot(time * 1000.0, fir_flt, label='FIR Low-Pass Filter (31-tap, fc=1kHz)',
                color='#2ca02c', linewidth=1.5)
    axs[0].plot(time * 1000.0, iir_flt, label='IIR Butterworth Low-Pass (2nd-Order, fc=1kHz)',
                color='#d62728', linestyle='--', linewidth=1.3)
    axs[0].set_title('1. Time-Domain Signal Waveforms')
    axs[0].set_xlabel('Time (milliseconds)')
    axs[0].set_ylabel('Normalized Amplitude [-1, 1]')
    axs[0].grid(True, linestyle=':', alpha=0.6)
    axs[0].legend(loc='upper right')
    axs[0].set_xlim([0, 50]) # Show first 50 ms

    # 2. Zoomed Steady-State Detail & Group Delay
    zoom_start = int(0.010 * fs) # 10 ms
    zoom_end = int(0.025 * fs)   # 25 ms
    axs[1].plot(time[zoom_start:zoom_end] * 1000.0, raw[zoom_start:zoom_end],
                label='Raw ADC Input', color='#1f77b4', alpha=0.35, linewidth=1.2)
    axs[1].plot(time[zoom_start:zoom_end] * 1000.0, fir_flt[zoom_start:zoom_end],
                label='FIR Output (Linear Phase, Group Delay tau=1.5ms)', color='#2ca02c', linewidth=2.0)
    axs[1].plot(time[zoom_start:zoom_end] * 1000.0, iir_flt[zoom_start:zoom_end],
                label='Iir Biquad Output (Minimum Phase, Nonlinear Delay)', color='#d62728', linestyle='--', linewidth=1.8)
    axs[1].set_title('2. Zoomed Signal Detail: Phase Lag & 2500 Hz Interference Attenuation')
    axs[1].set_xlabel('Time (milliseconds)')
    axs[1].set_ylabel('Amplitude')
    axs[1].grid(True, linestyle=':', alpha=0.6)
    axs[1].legend(loc='upper right')

    # 3. Frequency Spectrum (FFT)
    axs[2].plot(freqs, raw_spec_db, label='Raw Input Spectrum', color='#1f77b4', alpha=0.5, linewidth=1.2)
    axs[2].plot(freqs, fir_spec_db, label='FIR 31-Tap Filtered (1 kHz Cutoff)', color='#2ca02c', linewidth=1.8)
    axs[2].plot(freqs, iir_spec_db, label='IIR Biquad Filtered', color='#d62728', linestyle='--', linewidth=1.5)
    axs[2].axvline(1000.0, color='gray', linestyle=':', label='Cutoff Freq fc = 1000 Hz')
    axs[2].set_title('3. Frequency Spectrum (FFT Magnitude with Hanning Window)')
    axs[2].set_xlabel('Frequency (Hz)')
    axs[2].set_ylabel('Magnitude (dBFS)')
    axs[2].set_ylim([-80, 5])
    axs[2].set_xlim([0, fs / 2.0]) # Up to Nyquist (5 kHz)
    axs[2].grid(True, linestyle=':', alpha=0.6)
    axs[2].legend(loc='upper right')

    # 4. Fixed-Point Q15 vs Floating-Point Quantization Error
    peak_err = np.max(np.abs(err_q15))
    mse = np.mean(err_q15**2)
    sqnr = 10.0 * np.log10(np.mean(fir_flt**2) / (mse + 1e-12))
    enob = (sqnr - 1.76) / 6.02

    axs[3].plot(time * 1000.0, err_q15 * 1e3, color='#9467bd', linewidth=1.0,
                label=f'Error e[n] = y_flt[n] - y_q15[n] (Max={peak_err*1e6:.1f} uV, SQNR={sqnr:.1f} dB, ENOB={enob:.2f} bits)')
    axs[3].axhline(0, color='black', linestyle='-', linewidth=0.8, alpha=0.5)
    axs[3].set_title('4. Fixed-Point Q15 Numerical Quantization Error Signal')
    axs[3].set_xlabel('Time (milliseconds)')
    axs[3].set_ylabel('Error (x10^-3 Normalized)')
    axs[3].grid(True, linestyle=':', alpha=0.6)
    axs[3].legend(loc='upper right')
    axs[3].set_xlim([0, 50])

    plt.savefig(output_png_path, dpi=300)
    plt.close()
    print(f"[PLOT] Signal analysis plot saved successfully to: {output_png_path}")
    return True

def plot_benchmarks(bench_csv_path, output_png_path):
    if not os.path.exists(bench_csv_path):
        print(f"[ERROR] Benchmark CSV file not found: {bench_csv_path}")
        return False

    print(f"[PLOT] Loading benchmark metrics from {bench_csv_path}...")
    data = np.genfromtxt(bench_csv_path, delimiter=',', names=True, dtype=None, encoding='utf-8')

    algorithms = np.unique(data['Algorithm'])
    block_sizes = np.unique(data['BlockSize'])

    fig, axs = plt.subplots(2, 2, figsize=(14, 10), constrained_layout=True)
    fig.suptitle('Host Execution Timing & Firmware DSP Throughput Benchmark Analysis',
                 fontsize=14, fontweight='bold')

    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd', '#8c564b']

    # 1. Mean Execution Time vs Block Size
    for idx, algo in enumerate(algorithms):
        algo_mask = (data['Algorithm'] == algo)
        bs = data['BlockSize'][algo_mask]
        mean_t = data['MeanBlockTimeUs'][algo_mask]
        axs[0, 0].plot(bs, mean_t, marker='o', label=algo, color=colors[idx % len(colors)], linewidth=1.8)

    axs[0, 0].set_title('Mean Block Processing Time vs Block Size (N)')
    axs[0, 0].set_xlabel('Block Size (Samples)')
    axs[0, 0].set_ylabel('Mean Execution Time (microseconds)')
    axs[0, 0].grid(True, linestyle=':', alpha=0.6)
    axs[0, 0].legend(fontsize=8)

    # 2. Throughput (Mega-Samples per Second)
    for idx, algo in enumerate(algorithms):
        algo_mask = (data['Algorithm'] == algo)
        bs = data['BlockSize'][algo_mask]
        tp = data['ThroughputMsps'][algo_mask]
        axs[0, 1].plot(bs, tp, marker='s', label=algo, color=colors[idx % len(colors)], linewidth=1.8)

    axs[0, 1].set_title('Throughput vs Block Size')
    axs[0, 1].set_xlabel('Block Size (Samples)')
    axs[0, 1].set_ylabel('Throughput (Mega-Samples/sec)')
    axs[0, 1].grid(True, linestyle=':', alpha=0.6)
    axs[0, 1].legend(fontsize=8)

    # 3. Worst Observed Execution Time (WOET) on Host vs Mean Time (Jitter Demonstration)
    fir31_mask = (data['Algorithm'] == 'FIR_Float_31Tap')
    if np.any(fir31_mask):
        bs_fir = data['BlockSize'][fir31_mask]
        mean_fir = data['MeanBlockTimeUs'][fir31_mask]
        woet_fir = data['WorstObservedTimeUs'][fir31_mask]

        bar_width = 0.35
        x_indices = np.arange(len(bs_fir))
        axs[1, 0].bar(x_indices - bar_width/2, mean_fir, width=bar_width,
                      label='Mean Execution Time (us)', color='#2ca02c', alpha=0.8)
        axs[1, 0].bar(x_indices + bar_width/2, woet_fir, width=bar_width,
                      label='Worst Observed Host Time (WOET in us)', color='#d62728', alpha=0.8)
        axs[1, 0].set_xticks(x_indices)
        axs[1, 0].set_xticklabels([str(b) for b in bs_fir])
        axs[1, 0].set_title('FIR 31-Tap: Mean vs Worst Observed Execution Time (Host OS Jitter)')
        axs[1, 0].set_xlabel('Block Size')
        axs[1, 0].set_ylabel('Time (microseconds)')
        axs[1, 0].set_yscale('log')
        axs[1, 0].grid(True, linestyle=':', alpha=0.6)
        axs[1, 0].legend()

    # 4. Real-Time Factor (RTF) across algorithms (N = 64)
    n64_mask = (data['BlockSize'] == 64)
    algos_64 = data['Algorithm'][n64_mask]
    rtf_64 = data['RealTimeFactor'][n64_mask]

    axs[1, 1].barh(algos_64, rtf_64, color='#1f77b4', alpha=0.7)
    axs[1, 1].set_title('Real-Time Factor (Block Size = 64, Fs = 10 kHz)')
    axs[1, 1].set_xlabel('Real-Time Factor (Execution Time / Audio Block Time)')
    axs[1, 1].axvline(1.0, color='red', linestyle='--', label='Real-time Deadline (1.0)')
    axs[1, 1].grid(True, linestyle=':', alpha=0.6)
    axs[1, 1].legend()

    plt.savefig(output_png_path, dpi=300)
    plt.close()
    print(f"[PLOT] Benchmark analysis plot saved successfully to: {output_png_path}")
    return True

if __name__ == '__main__':
    base_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sim_csv = os.path.join(base_dir, 'signal_simulation.csv')
    bench_csv = os.path.join(base_dir, 'benchmark_results.csv')

    out_sim_png = os.path.join(base_dir, 'signal_analysis.png')
    out_bench_png = os.path.join(base_dir, 'benchmark_analysis.png')

    plot_signals(sim_csv, out_sim_png)
    plot_benchmarks(bench_csv, out_bench_png)
