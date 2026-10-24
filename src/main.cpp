#include "dsp/circular_buffer.hpp"
#include "dsp/simulated_adc.hpp"
#include "dsp/fir_filter.hpp"
#include "dsp/iir_biquad.hpp"
#include "dsp/fixed_point_dsp.hpp"
#include "dsp/dsp_pipeline.hpp"
#include "dsp/benchmark.hpp"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include <cstring>

namespace {

void run_signal_simulation(const std::string& csv_out_path, std::size_t num_samples = 2000) {
    std::cout << "[SIMULATION] Initializing deterministic simulated ADC...\n";
    std::cout << "  - Architecture: 12-bit SAR ADC emulation (0..4095 counts)\n";
    std::cout << "  - Vref: 3.3V, Mid-rail bias: 1.65V\n";
    std::cout << "  - Sampling frequency: 10,000 Hz\n";
    std::cout << "  - Primary Tone: 250 Hz (Passband, Amplitude: 0.7)\n";
    std::cout << "  - Secondary Tone: 2500 Hz (Stopband Interference, Amplitude: 0.3)\n";
    std::cout << "  - Additive Noise Variance: 0.005\n";

    dsp::SimulatedAdcConfig adc_cfg;
    adc_cfg.sample_rate_hz = 10000.0f;
    adc_cfg.waveform = dsp::WaveformType::MultiTone;
    adc_cfg.primary_freq_hz = 250.0f;
    adc_cfg.primary_amplitude = 0.7f;
    adc_cfg.secondary_freq_hz = 2500.0f;
    adc_cfg.secondary_amplitude = 0.3f;
    adc_cfg.noise_variance = 0.005f;
    adc_cfg.enable_quantization = true;

    dsp::SimulatedAdc adc(adc_cfg);

    // Filter configurations
    // 1. FIR Lowpass 31-tap, cutoff 1000 Hz
    auto fir_coeffs = dsp::FirFilter::design_lowpass(1000.0f, adc_cfg.sample_rate_hz, 31);
    dsp::FirFilter fir_float(fir_coeffs);

    // 2. IIR Biquad 2nd-order Butterworth Lowpass, cutoff 1000 Hz, Q=0.7071
    auto biquad_coeffs = dsp::IirBiquad::design_lowpass(1000.0f, adc_cfg.sample_rate_hz, 0.7071f);
    dsp::IirBiquad iir_float(biquad_coeffs);

    // 3. Fixed-Point Q15 FIR (same 31-tap coeffs)
    dsp::FixedPointFirQ15 fir_q15(fir_coeffs);

    std::vector<float> raw_samples;
    std::vector<float> fir_samples;
    std::vector<float> iir_samples;
    std::vector<float> q15_samples;
    std::vector<uint16_t> adc_counts;

    raw_samples.reserve(num_samples);
    fir_samples.reserve(num_samples);
    iir_samples.reserve(num_samples);
    q15_samples.reserve(num_samples);
    adc_counts.reserve(num_samples);

    for (std::size_t i = 0; i < num_samples; ++i) {
        dsp::SimulatedAdcSample s = adc.acquire_sample();
        raw_samples.push_back(s.normalized);
        adc_counts.push_back(s.raw_counts);

        float y_fir = fir_float.process_sample(s.normalized);
        fir_samples.push_back(y_fir);

        float y_iir = iir_float.process_sample(s.normalized);
        iir_samples.push_back(y_iir);

        dsp::q15_t in_q15 = dsp::qformat::float_to_q15(s.normalized);
        dsp::q15_t out_q15 = fir_q15.process_sample(in_q15);
        q15_samples.push_back(dsp::qformat::q15_to_float(out_q15));
    }

    // Evaluate quantization error
    dsp::DspErrorMetrics error_metrics = dsp::evaluate_quantization_error(fir_samples.data(),
                                                                          q15_samples.data(),
                                                                          num_samples);

    std::cout << "\n[QUANTIZATION ANALYSIS] Fixed-Point Q15 vs Floating-Point Reference:\n";
    std::cout << "  - Evaluated Samples:     " << error_metrics.sample_count << "\n";
    std::cout << "  - Peak Absolute Error:   " << std::scientific << std::setprecision(4)
              << error_metrics.peak_absolute_error << "\n";
    std::cout << "  - Mean Squared Error:    " << error_metrics.mean_squared_error << "\n";
    std::cout << "  - Root MSE (RMSE):       " << error_metrics.root_mean_squared_error << "\n";
    std::cout << "  - Signal-to-Noise (SQNR):" << std::fixed << std::setprecision(2)
              << error_metrics.sqnr_db << " dB\n";
    std::cout << "  - Effective Bits (ENOB): " << error_metrics.enob << " bits (Q15 theoretical max ~15.0)\n";

    // Export CSV
    std::ofstream csv(csv_out_path);
    if (!csv.is_open()) {
        std::cerr << "[ERROR] Could not open " << csv_out_path << " for writing!\n";
        return;
    }

    csv << "SampleIndex,TimeSeconds,RawAdcCounts,RawAdcNormalized,FirFilteredFloat,"
        << "IirFilteredFloat,FirFilteredQ15,QuantizationErrorQ15\n";

    for (std::size_t i = 0; i < num_samples; ++i) {
        float t = static_cast<float>(i) / adc_cfg.sample_rate_hz;
        float err = fir_samples[i] - q15_samples[i];
        csv << i << ","
            << std::fixed << std::setprecision(6) << t << ","
            << adc_counts[i] << ","
            << std::setprecision(6)
            << raw_samples[i] << ","
            << fir_samples[i] << ","
            << iir_samples[i] << ","
            << q15_samples[i] << ","
            << err << "\n";
    }

    std::cout << "[SIMULATION] Successfully exported signal telemetry to: " << csv_out_path << "\n";
}

void run_benchmarks(const std::string& csv_out_path) {
    std::cout << "\n[BENCHMARK] Executing comprehensive timing suite (500 blocks per configuration)...\n";
    dsp::BenchmarkRunner runner;
    auto records = runner.run_comprehensive_suite(500, 10000.0f);

    dsp::BenchmarkRunner::print_report(records);

    if (dsp::BenchmarkRunner::export_csv(csv_out_path, records)) {
        std::cout << "[BENCHMARK] Successfully exported benchmark metrics to: " << csv_out_path << "\n";
    } else {
        std::cerr << "[ERROR] Failed to write benchmark CSV to " << csv_out_path << "\n";
    }
}

} // namespace

int main(int argc, char* argv[]) {
    std::cout << "========================================================================\n";
    std::cout << "   REAL-TIME EMBEDDED DSP FIRMWARE SIMULATOR (C++17 / Embedded Architecture)\n";
    std::cout << "========================================================================\n";

    bool do_sim = true;
    bool do_bench = true;

    if (argc > 1) {
        if (std::strcmp(argv[1], "--simulate") == 0) {
            do_sim = true;
            do_bench = false;
        } else if (std::strcmp(argv[1], "--benchmark") == 0) {
            do_sim = false;
            do_bench = true;
        }
    }

    std::string sim_csv = "signal_simulation.csv";
    std::string bench_csv = "benchmark_results.csv";

    if (do_sim) {
        run_signal_simulation(sim_csv, 2000);
    }

    if (do_bench) {
        run_benchmarks(bench_csv);
    }

    std::cout << "\n[DONE] Execution completed successfully.\n";
    return 0;
}
