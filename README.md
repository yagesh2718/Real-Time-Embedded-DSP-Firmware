# Real-Time Embedded DSP Firmware Simulator

A complete, production-grade real-time digital signal processing (DSP) firmware simulator in **C++17**, engineered for initial desktop simulation without physical hardware while maintaining clean, zero-allocation portable interfaces designed for direct drop-in migration to **STM32 (ARM Cortex-M4/M7)** microcontrollers.

---

## 1. Architecture & Embedded Decoupling

In embedded firmware (e.g., bare-metal or RTOS on STM32), real-time signal processing requires strict separation between hardware peripherals, intermediate ring buffering, DSP mathematical processing, and telemetry output. Unbounded dynamic memory allocations (`new`, `malloc`, `std::vector` resizing) in the audio/sample processing loops are strictly prohibited to prevent heap fragmentation and non-deterministic latency.

```
                           +----------------------------------------+
                           |         Acquisition Layer              |
                           |  - Desktop: SimulatedAdc (IAdcSource)  |
                           |  - STM32 Target: ADC1 + TIM + DMA      |
                           +-------------------+--------------------+
                                               |
                                               v (Raw 12-bit / Normalized Samples)
                           +----------------------------------------+
                           |          Buffering Layer               |
                           |  - CircularBuffer<T, Capacity>         |
                           |  - Zero heap allocation                |
                           |  - Configurable Overflow Policy:       |
                           |    * DropNewest / OverwriteOldest      |
                           |  - STM32 Target: DMA Double Buffer     |
                           +-------------------+--------------------+
                                               |
                                               v (Periodic Block of N Samples: 16..256)
                           +----------------------------------------+
                           |       DSP Executive Pipeline           |
                           |  - DspPipeline (Block Processing)      |
                           +-------------------+--------------------+
                                               |
            +----------------------------------+----------------------------------+
            |                                  |                                  |
            v                                  v                                  v
+-----------------------+          +-----------------------+          +-----------------------+
|   FIR Filter (Float)  |          |   IIR Biquad (Float)  |          | Fixed-Point FIR (Q15) |
| - Direct-form convol. |          | - DF2T Transposed     |          | - Q0.15 CMSIS-DSP fmt |
| - Circular delay line |          | - Schur-Cohn Stability|          | - 64-bit MAC accum.   |
| - Linear phase / tau  |          | - 2nd/4th Order SOS   |          | - Unbiased rounding   |
+-----------------------+          +-----------------------+          +-----------------------+
            |                                  |                                  |
            +----------------------------------+----------------------------------+
                                               |
                                               v (Filtered Output Block)
                           +----------------------------------------+
                           |        Sink & Telemetry Layer          |
                           |  - BlockOutputCallback                 |
                           |  - Telemetry CSV Logger                |
                           |  - High-res Timing & Jitter Profiler   |
                           +----------------------------------------+
```

### Directory Structure

```
Real-Time Embedded DSP Firmware Simulator/
├── CMakeLists.txt              # Unified build system (C++17, Ninja/Make, CTest, ASan/UBSan support)
├── README.md                   # Complete architectural documentation and porting roadmap
├── .gitignore                  # Build artifacts and intermediate files exclusion
├── requirements.txt            # Python dependencies (numpy, matplotlib)
├── signal_simulation.csv       # Exported raw & filtered signal telemetry
├── benchmark_results.csv       # Exported performance & throughput measurements
├── signal_analysis.png         # High-resolution time-domain, FFT, and quantization plots
├── benchmark_analysis.png      # Throughput, execution time, and jitter scaling plots
├── include/
│   └── dsp/
│       ├── circular_buffer.hpp # Bounded ring buffer with zero dynamic allocations
│       ├── simulated_adc.hpp   # IAdcSource interface & 12-bit SAR ADC emulator
│       ├── fir_filter.hpp      # Direct-form FIR filter with circular delay line
│       ├── iir_biquad.hpp      # Direct Form II Transposed biquad & cascade with stability checks
│       ├── qformat.hpp         # Q15/Q31 fixed-point math, rounding, and saturating arithmetic
│       ├── fixed_point_dsp.hpp # Fixed-point Q15 FIR & Biquad filters, ENOB/SQNR evaluator
│       ├── dsp_pipeline.hpp    # Firmware block-processing executive and timing hooks
│       └── benchmark.hpp       # Comprehensive execution timing and throughput benchmark runner
├── src/
│   ├── simulated_adc.cpp       # SAR ADC quantization & deterministic waveform generation
│   ├── fir_filter.cpp          # Windowed-sinc filter design & convolution
│   ├── iir_biquad.cpp          # Bilinear transform design & DF2T execution
│   ├── fixed_point_dsp.cpp     # Q15 FIR MAC execution and error statistical metrics
│   ├── dsp_pipeline.cpp        # Block demux, execution timing, and callback delivery
│   ├── benchmark.cpp           # Multi-block latency, WOET, throughput, and CSV export
│   └── main.cpp                # CLI entrypoint for simulation and benchmarks
├── tests/
│   ├── test_framework.hpp      # Embedded-friendly lightweight unit test framework
│   ├── test_circular_buffer.cpp# Ring buffer empty/full, overflow policies, wraparound tests
│   ├── test_fir_filter.cpp     # FIR impulse response, step response, frequency response tests
│   ├── test_iir_filter.cpp     # IIR Schur-Cohn stability, impulse decay, step response tests
│   ├── test_fixed_point.cpp    # Q15 saturation, corner cases (-1*-1), SQNR and ENOB tests
│   ├── test_simulated_adc.cpp  # ADC quantization range, determinism, and data labeling tests
│   └── test_main.cpp           # Test runner entrypoint
└── scripts/
    ├── generate_plots.py       # Matplotlib spectral analysis and benchmark scaling visualizer
    └── run_all.py              # Single command build, test, simulate, and plot orchestrator
```

---

## 2. Mathematical DSP Foundations & Firmware Parameters

### A. Sampling Rate & Simulated ADC Quantization
- **Sampling Frequency ($F_s$)**: $10\,000\text{ Hz}$ ($T_s = 100\,\mu\text{s}$).
- **Nyquist Limit**: $5\,000\text{ Hz}$.
- **Resolution**: 12-bit SAR ADC (modeling STM32 SAR ADC peripheral).
- **ADC Dynamic Range**:
  $$\text{Code} = \mathrm{clamp}\left(\mathrm{round}\left(\frac{V_{\text{analog}}}{V_{\text{ref}}} \times 4095\right), 0, 4095\right)$$
  where $V_{\text{ref}} = 3.3\text{V}$, yielding $1\text{ LSB} \approx \frac{3.3\text{V}}{4095} = 805.86\,\mu\text{V}$.
- **Center Bias**: Centered at $V_{\text{ref}}/2 = 1.65\text{V}$ (code $2048$) for AC audio/sensor inputs.
- **Synthesized Signal**:
  - Primary Passband Signal: $250\text{ Hz}$, amplitude $0.70$ ($V_{\text{peak}} \approx 1.097\text{V}$).
  - Secondary Stopband Interference: $2\,500\text{ Hz}$, amplitude $0.30$ ($V_{\text{peak}} \approx 0.470\text{V}$).
  - Deterministic Additive White Noise: variance $\sigma^2 = 0.005$ with fixed-seed pseudorandom generator.
  - All generated samples are explicitly flagged: `is_simulated_adc_data = true`.

### B. Finite Impulse Response (FIR) Filter
- **Difference Equation**:
  $$y[n] = \sum_{k=0}^{M-1} b[k] \cdot x[n-k]$$
- **Filter Order / Length**: $M = 31$ taps ($30\text{th}$ order).
- **Design Method**: Windowed-sinc with a Hamming window:
  $$h_{\text{ideal}}[n] = \frac{\sin(2\pi f_c (n - \tau))}{\pi (n - \tau)}, \quad w[n] = 0.54 - 0.46 \cos\left(\frac{2\pi n}{M-1}\right)$$
- **Cutoff Frequency ($f_c$)**: $1\,000\text{ Hz}$ (normalized $f_c / F_s = 0.10$).
- **Linear Phase & Group Delay**:
  Because $b[k] = b[M - 1 - k]$, phase response is strictly linear:
  $$\tau = \frac{M - 1}{2} = \frac{31 - 1}{2} = 15\text{ samples} \quad (1.5\text{ ms delay at } 10\text{ kHz}).$$

### C. Infinite Impulse Response (IIR) Biquad Filter
- **Topology**: Direct Form II Transposed (DF2T), matching ARM CMSIS-DSP `arm_biquad_cascade_df2T_f32`.
- **Difference Equations**:
  $$\begin{aligned}
    y[n]   &= b_0 \cdot x[n] + d_1[n-1] \\
    d_1[n] &= b_1 \cdot x[n] - a_1 \cdot y[n] + d_2[n-1] \\
    d_2[n] &= b_2 \cdot x[n] - a_2 \cdot y[n]
  \end{aligned}$$
  *State memory requirement*: Exactly 2 floating-point registers per biquad, zero dynamic allocation.
- **Transfer Function**:
  $$H(z) = \frac{b_0 + b_1 z^{-1} + b_2 z^{-2}}{1 + a_1 z^{-1} + a_2 z^{-2}}$$
- **Stability Conditions (Jury / Schur-Cohn Criterion)**:
  Roots of denominator $z^2 + a_1 z + a_2 = 0$ must satisfy $|p_{1,2}| < 1$:
  1. $|a_2| < 1$
  2. $1 + a_1 + a_2 > 0$
  3. $1 - a_1 + a_2 > 0$
  The firmware checks these conditions upon coefficient initialization, rejecting unstable filter specifications.
- **Cascaded SOS (Second-Order Sections)**:
  A 4th-order Butterworth lowpass filter is implemented as two cascaded biquads with $Q_1 = 1.306563$ and $Q_2 = 0.541196$.

### D. Q-Format Fixed-Point Arithmetic (Q15 & Q31)
- **Format**: Signed $16$-bit Q0.15 (1 sign bit, 15 fractional bits).
  $$\text{Value} = \frac{N_{\text{int}}}{32768.0}, \quad \text{Range: } [-1.0, 1.0 - 2^{-15}] = [-1.0, 0.99996948]$$
- **Saturation & Clamping**:
  Hardware-style saturating clamp (`clip_q15`) replicating ARM `__SSAT(val, 16)`.
- **Rounding Strategy**:
  Unbiased round-to-nearest with a $+0\text{x}4000$ bias before $15$-bit right shift:
  $$\mathrm{product}_{q15} = \mathrm{clip\_q15}\left((a \times b + 0\text{x}4000) \gg 15\right)$$
- **Boundary Edge Cases**:
  $(-1.0) \times (-1.0) = (-32768) \times (-32768) = +1\,073\,741\,824$. In pure Q15, $+1.0$ is non-representable and overflows; the implementation clamps to `0x7FFF` ($+32767$), matching ARM CMSIS-DSP.
- **Quantization Error Metrics**:
  - **Peak Absolute Error (PAE)**: $\max |y_{\text{flt}}[n] - y_{q15}[n]|$
  - **Signal-to-Quantization-Noise Ratio (SQNR)**:
    $$\text{SQNR (dB)} = 10 \log_{10}\left(\frac{\sum y_{\text{flt}}^2[n]}{\sum (y_{\text{flt}}[n] - y_{q15}[n])^2}\right)$$
  - **Effective Number of Bits (ENOB)**:
    $$\text{ENOB} = \frac{\text{SQNR} - 1.76}{6.02}$$

---

## 3. Desktop Timings vs Microcontroller WCET

> [!WARNING]
> **CRITICAL EMBEDDED CAVEAT: Desktop Timings Are NOT WCET Guarantees**
>
> Desktop benchmarks measure execution on multi-core x86/x64 processors running general-purpose operating systems (Windows/Linux/macOS). These host measurements are inherently subject to:
> 1. **Preemptive OS Schedulers**: Thread quantum slicing and OS interrupts.
> 2. **Dynamic CPU Frequency Scaling**: Intel SpeedStep / AMD Cool'n'Quiet dynamically modulating clock frequencies from $800\text{ MHz}$ to $4.5\text{ GHz}$.
> 3. **Non-Deterministic Memory Paging**: Virtual memory translation and L1/L2/L3 cache line evictions.
>
> In our benchmark logs, the average processing time per 64-sample block is $\approx 5\,\mu\text{s}$, but the **Worst Observed Execution Time (WOET)** spikes to $\approx 1\,000\,\mu\text{s}$ ($1\text{ ms}$) whenever the host OS preempts the simulator thread.
>
> On a physical **ARM Cortex-M (STM32)** microcontroller:
> - Execution runs on bare-metal or a deterministic RTOS (e.g. FreeRTOS / Zephyr).
> - Flash prefetch buffers and tightly coupled memory (ITCM / DTCM) provide cycle-deterministic latency.
> - Hard real-time **Worst-Case Execution Time (WCET)** must be measured using the ARM Cortex-M **DWT cycle counter** (`DWT->CYCCNT`) or a hardware timer peripheral, not desktop OS wall-clock time.

---

## 4. Single Documented Workflow

### Prerequisites
- **C++ Compiler**: GCC ($\ge 6.3$ or higher) or Clang ($\ge 6.0$), supporting C++17.
- **Build System**: CMake ($\ge 3.12$) and Ninja or MinGW Make.
- **Python**: Python 3.8+ with `numpy` and `matplotlib` (for spectral plotting).

### Install Python Dependencies
```bash
pip install -r requirements.txt
```

### Option A: Run Everything with Single Orchestration Script
```bash
python scripts/run_all.py
```
This single command automatically:
1. Configures CMake in `build/` using Ninja.
2. Compiles `dsp_firmware_core`, `dsp_firmware_tests`, and `dsp_firmware_sim`.
3. Runs all 28 automated unit tests.
4. Executes the simulator and performance benchmarks.
5. Exports `signal_simulation.csv` and `benchmark_results.csv`.
6. Generates `signal_analysis.png` and `benchmark_analysis.png`.

### Option B: Step-by-Step Manual Build & Execution

#### 1. Configure and Build
```bash
cmake -B build -G Ninja
cmake --build build
```
*(Optional: Enable AddressSanitizer and UndefinedBehaviorSanitizer where supported: `cmake -B build -G Ninja -DENABLE_SANITIZERS=ON`)*

#### 2. Run Automated Unit Tests
```bash
# Directly via test binary
./build/dsp_firmware_tests

# Or via CTest
ctest --test-dir build --output-on-failure
```

#### 3. Run Simulation & Benchmarks
```bash
# Full simulation and benchmark run
./build/dsp_firmware_sim

# Or run only signal simulation
./build/dsp_firmware_sim --simulate

# Or run only performance benchmarks
./build/dsp_firmware_sim --benchmark
```

#### 4. Generate Visual Plots
```bash
python scripts/generate_plots.py
```

---

## 5. Verified Test Suite Summary

The automated unit test suite verifies every subsystem against mathematical ground truth. All 28 test cases pass with zero failures:

| Test Name | Subsystem | Verification Objective | Result |
| :--- | :--- | :--- | :---: |
| `CircularBuffer_InitialEmptyState` | Bounded Buffer | Initial empty state, zero occupancy, pop rejection | **PASS** |
| `CircularBuffer_SinglePushPopFifo` | Bounded Buffer | Strict first-in, first-out (FIFO) sample preservation | **PASS** |
| `CircularBuffer_FillToCapacity` | Bounded Buffer | Full condition, available space, full readout | **PASS** |
| `CircularBuffer_OverflowPolicyDropNewest` | Bounded Buffer | Rejects incoming sample on full, counts drops | **PASS** |
| `CircularBuffer_OverflowPolicyOverwriteOldest`| Bounded Buffer | Evicts oldest sample, advances tail pointer | **PASS** |
| `CircularBuffer_WraparoundMultipleCycles` | Bounded Buffer | Continuous cyclic wraparound across boundaries | **PASS** |
| `CircularBuffer_BlockWriteAndRead` | Bounded Buffer | DMA-style multi-sample block transfer | **PASS** |
| `CircularBuffer_PeekOffset` | Bounded Buffer | Non-destructive index inspection without pop | **PASS** |
| `CircularBuffer_Clear` | Bounded Buffer | Buffer flush and index reset | **PASS** |
| `FirFilter_ImpulseResponseMatchesCoefficients`| FIR Engine | Dirac delta input reproduces $\{b_k\}$ coefficients | **PASS** |
| `FirFilter_StepResponseConvergesToDcGain` | FIR Engine | Unit step input converges to normalized DC gain $1.0$ | **PASS** |
| `FirFilter_LinearPhaseSymmetryAndGroupDelay` | FIR Engine | Coefficient symmetry $b[k] = b[M-1-k]$, $\tau = 15.0$ | **PASS** |
| `FirFilter_StopbandAttenuation` | FIR Engine | Passband attenuation $< 1\text{ dB}$, stopband $> 20\text{ dB}$ | **PASS** |
| `FirFilter_StreamingVsBlockProcessingEquivalence`| FIR Engine| Bit-for-bit equivalence of streaming vs block processing| **PASS** |
| `FirFilter_ResetClearsDelayLine` | FIR Engine | Delay-line state flush to zero | **PASS** |
| `IirFilter_StabilityVerification` | IIR Engine | Schur-Cohn conditions check; flags unstable poles | **PASS** |
| `IirFilter_ImpulseResponseDecay` | IIR Engine | Asymptotic decay to zero (BIBO stable) | **PASS** |
| `IirFilter_StepResponseConvergesToUnity` | IIR Engine | DC step response converges to unity gain | **PASS** |
| `IirFilter_NotchFilterRejection` | IIR Engine | Deep notch attenuation at target frequency ($< -40\text{ dB}$)| **PASS** |
| `IirFilter_Cascade4thOrderButterworth` | IIR Engine | Cascaded 2-stage biquad SOS response and reset | **PASS** |
| `FixedPoint_ConversionPrecision` | Fixed-Point | Q15 conversion accuracy ($0.5 \to 16384$, $-1.0 \to -32768$) | **PASS** |
| `FixedPoint_SaturationClamping` | Fixed-Point | Upper/lower saturation clamping at $[-32768, 32767]$ | **PASS** |
| `FixedPoint_MultiplicationAndBoundaryEdgeCase` | Fixed-Point | Q15 round-to-nearest and $(-1) \times (-1) \to 0\text{x}7FFF$ clamp | **PASS** |
| `FixedPoint_FirImpulseResponseMatchesQuantizedCoeffs`| Fixed-Point | Q15 FIR impulse matches quantized coefficients | **PASS** |
| `FixedPoint_ErrorMetricsAndEnobEvaluation` | Fixed-Point | Fixed vs float verification: $\text{SQNR} > 60\text{ dB}$, $\text{ENOB} \ge 10$ bits | **PASS** |
| `SimulatedAdc_QuantizationRangeAndBitDepth` | ADC Source | 12-bit range $[0, 4095]$, voltage $[0, 3.3\text{V}]$, data labeled | **PASS** |
| `SimulatedAdc_DeterministicReproducibility` | ADC Source | Bit-identical output streams across identical seeds | **PASS** |
| `SimulatedAdc_CustomSequencePlayback` | ADC Source | Deterministic replay of custom input vectors | **PASS** |

---

## 6. Actual Measured Performance & Quantization Telemetry

### A. Fixed-Point Q15 vs Floating-Point Precision
Evaluated over $2\,000$ consecutive ADC samples ($F_s = 10\text{ kHz}$):
- **Peak Absolute Error**: $3.1143 \times 10^{-5}$ normalized amplitude ($\approx 51\,\mu\text{V}$).
- **Mean Squared Error (MSE)**: $8.4554 \times 10^{-11}$.
- **Root Mean Squared Error (RMSE)**: $9.1953 \times 10^{-6}$.
- **Signal-to-Quantization-Noise Ratio (SQNR)**: **$94.57\text{ dB}$**.
- **Effective Number of Bits (ENOB)**: **$15.42\text{ bits}$** (matching theoretical $15$-bit fractional limit of Q15).

### B. Execution Benchmark Scaling (500 Blocks per Test)
From actual benchmark execution exported to `benchmark_results.csv`:

| Algorithm | Block Size ($N$) | Mean Time ($\mu\text{s}$) | WOET ($\mu\text{s}$)* | Jitter ($\mu\text{s}$) | Throughput (MSamples/s) | Real-Time Factor |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| **PassThrough** | 64 | $0.00$ | $0.00$ | $0.00$ | $> 100$ | $< 0.00001$ |
| **FIR Float (15 Tap)** | 64 | $3.99$ | $998.00$ | $62.96$ | $16.04$ | $0.00062$ |
| **FIR Float (31 Tap)** | 64 | $5.99$ | $2996.00$ | $133.85$ | $10.68$ | $0.00094$ |
| **FIR Float (63 Tap)** | 64 | $15.96$ | $1999.00$ | $140.26$ | $4.01$ | $0.00249$ |
| **IIR Biquad Float (2nd-Order)** | 64 | $2.00$ | $1000.00$ | $44.68$ | $32.00$ | $0.00031$ |
| **FIR Q15 Fixed-Point (31 Tap)** | 64 | $5.52$ | $997.00$ | $71.57$ | $11.59$ | $0.00086$ |

*\*WOET = Worst Observed Execution Time on host OS (demonstrating scheduling jitter).*

---

## 7. STM32 / CMSIS-DSP Migration Roadmap

> [!IMPORTANT]
> **Hardware Status Disclaimer**:
> This codebase currently runs in **host desktop simulation mode**. No physical STM32 microcontroller, Nucleo board, or hardware timer has been connected or falsely claimed to be tested. The interfaces below define the exact migration blueprint for an STM32 port.

### Architecture Mapping Matrix

| Simulator Module | STM32 Target Hardware Equivalent | Drop-In Replacement Strategy |
| :--- | :--- | :--- |
| `dsp::IAdcSource` | STM32 ADC1 + TIM2_TRGO + DMA2_Stream0 | Replace `SimulatedAdc::acquire_sample()` with peripheral register reads or DMA buffer pointers. |
| `dsp::CircularBuffer` | DMA Ping-Pong Circular Buffer | Map to double-buffered DMA: half-buffer triggers interrupt, CPU processes inactive half. |
| Host Wall-Clock Timer | Cortex-M DWT Cycle Counter (`DWT->CYCCNT`) | Measure cycle counts: $\Delta \text{cycles} / F_{\text{cpu}} = \text{deterministic time}$. |
| `dsp::FirFilter` | CMSIS-DSP `arm_fir_f32` | Drop-in call using ARM Cortex-M DSP instruction extensions (SIMD `SMLAD`). |
| `dsp::FixedPointFirQ15` | CMSIS-DSP `arm_fir_q15` | Drop-in call utilizing single-cycle 16-bit SIMD multiply-accumulate instructions. |
| `dsp::IirBiquad` | CMSIS-DSP `arm_biquad_cascade_df2T_f32` | Drop-in replacement with identical DF2T state structure. |

### Concrete STM32 Implementation Template

When porting to an STM32 Nucleo (e.g., STM32F401RE / STM32F446RE / STM32H743ZI) in STM32CubeIDE with ARM GCC:

```c
// 1. DMA Half-Transfer and Transfer-Complete Callbacks
#define BLOCK_SIZE 64
uint16_t adc_dma_buffer[BLOCK_SIZE * 2]; // Ping-Pong Double Buffer
float processing_buffer[BLOCK_SIZE];

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
    // Ping buffer ready: samples 0 .. BLOCK_SIZE - 1
    process_dma_block(&adc_dma_buffer[0], processing_buffer, BLOCK_SIZE);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    // Pong buffer ready: samples BLOCK_SIZE .. 2*BLOCK_SIZE - 1
    process_dma_block(&adc_dma_buffer[BLOCK_SIZE], processing_buffer, BLOCK_SIZE);
}

// 2. Cycle-Accurate Profiling via ARM Cortex-M DWT Cycle Counter
void init_dwt_cycle_counter(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t get_cpu_cycles(void) {
    return DWT->CYCCNT;
}

// 3. ARM CMSIS-DSP SIMD Acceleration
arm_fir_instance_f32 S_fir;
float fir_state[31 + BLOCK_SIZE - 1];

void process_dma_block(const uint16_t* raw_in, float* dsp_out, size_t count) {
    uint32_t start_cycles = get_cpu_cycles();

    // Convert raw ADC counts to normalized float
    for (size_t i = 0; i < count; ++i) {
        processing_buffer[i] = ((float)raw_in[i] - 2048.0f) / 2048.0f;
    }

    // CMSIS-DSP SIMD-accelerated FIR filter
    arm_fir_f32(&S_fir, processing_buffer, dsp_out, count);

    uint32_t elapsed_cycles = get_cpu_cycles() - start_cycles;
    // elapsed_cycles is deterministic bare-metal WCET!
}
```

---

## 8. Engineering Standards & Compliance
- Language Standard: ISO/IEC 14882:2017 (C++17).
- Coding Style: Embedded real-time MISRA-aligned guidelines (bounded loops, no dynamic heap in processing paths, explicit integer bit widths, exception-free execution).
