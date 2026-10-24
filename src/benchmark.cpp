#include "dsp/benchmark.hpp"
#include "dsp/simulated_adc.hpp"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <numeric>

namespace dsp {

BenchmarkRecord BenchmarkRunner::benchmark_config(const std::string& name,
                                                  FilterMode mode,
                                                  std::size_t num_taps,
                                                  std::size_t block_size,
                                                  std::size_t num_blocks,
                                                  float sample_rate_hz) {
    SimulatedAdcConfig adc_cfg;
    adc_cfg.sample_rate_hz = sample_rate_hz;
    adc_cfg.waveform = WaveformType::MultiTone;
    SimulatedAdc adc(adc_cfg);

    PipelineConfig pipe_cfg;
    pipe_cfg.block_size = block_size;
    pipe_cfg.filter_mode = mode;
    pipe_cfg.overflow_policy = OverflowPolicy::DropNewest;

    DspPipeline pipeline(adc, pipe_cfg);

    if (mode == FilterMode::FirFloat || mode == FilterMode::FirQ15FixedPoint) {
        auto coeffs = FirFilter::design_lowpass(1000.0f, sample_rate_hz, num_taps);
        pipeline.fir_filter().set_coefficients(coeffs);
        pipeline.fir_q15_filter().set_coefficients(coeffs.data(), coeffs.size());
    } else if (mode == FilterMode::IirFloat) {
        pipeline.iir_filter().set_coefficients(IirBiquad::design_lowpass(1000.0f, sample_rate_hz, 0.7071f));
    }

    // Warm-up cache with 20 blocks
    for (std::size_t w = 0; w < 20; ++w) {
        pipeline.step(block_size);
    }

    std::vector<double> block_times;
    block_times.reserve(num_blocks);

    pipeline.set_output_callback([&](const float*, const float*, std::size_t, const BlockExecutionStats& stats) {
        block_times.push_back(stats.execution_time_us);
    });

    // Run timed iterations
    for (std::size_t b = 0; b < num_blocks; ++b) {
        pipeline.step(block_size);
    }

    BenchmarkRecord rec;
    rec.algorithm_name = name;
    rec.block_size = block_size;
    rec.filter_order_taps = num_taps;
    rec.total_blocks = block_times.size();
    rec.total_samples = rec.total_blocks * block_size;

    if (block_times.empty()) {
        return rec;
    }

    double sum = 0.0;
    double min_t = 1e9;
    double max_t = 0.0;

    for (double t : block_times) {
        sum += t;
        if (t < min_t) min_t = t;
        if (t > max_t) max_t = t;
    }

    rec.mean_block_time_us = sum / static_cast<double>(block_times.size());
    rec.min_block_time_us = min_t;
    rec.max_block_time_us = max_t; // Worst observed host execution time

    double var_sum = 0.0;
    for (double t : block_times) {
        double d = t - rec.mean_block_time_us;
        var_sum += d * d;
    }
    rec.stddev_block_time_us = std::sqrt(var_sum / static_cast<double>(block_times.size()));

    double total_sec = sum * 1e-6;
    if (total_sec > 1e-9) {
        rec.throughput_msps = (static_cast<double>(rec.total_samples) / 1e6) / total_sec;
        rec.throughput_mbytes_sec = rec.throughput_msps * sizeof(float);
    }

    double real_time_block_us = (static_cast<double>(block_size) / static_cast<double>(sample_rate_hz)) * 1e6;
    rec.real_time_factor = (real_time_block_us > 0.0) ? (rec.mean_block_time_us / real_time_block_us) : 0.0;
    rec.host_cpu_load_percent = rec.real_time_factor * 100.0;

    return rec;
}

std::vector<BenchmarkRecord> BenchmarkRunner::run_comprehensive_suite(std::size_t blocks_per_test,
                                                                      float sample_rate_hz) {
    std::vector<BenchmarkRecord> results;

    const std::vector<std::size_t> block_sizes = {16, 32, 64, 128, 256};

    for (std::size_t bs : block_sizes) {
        // Baseline PassThrough
        results.push_back(benchmark_config("PassThrough", FilterMode::PassThrough, 0, bs, blocks_per_test, sample_rate_hz));

        // FIR Float with different tap lengths
        results.push_back(benchmark_config("FIR_Float_15Tap", FilterMode::FirFloat, 15, bs, blocks_per_test, sample_rate_hz));
        results.push_back(benchmark_config("FIR_Float_31Tap", FilterMode::FirFloat, 31, bs, blocks_per_test, sample_rate_hz));
        results.push_back(benchmark_config("FIR_Float_63Tap", FilterMode::FirFloat, 63, bs, blocks_per_test, sample_rate_hz));

        // IIR Biquad Float (2nd Order)
        results.push_back(benchmark_config("IIR_Biquad_Float", FilterMode::IirFloat, 5, bs, blocks_per_test, sample_rate_hz));

        // FIR Q15 Fixed-Point (31 Tap)
        results.push_back(benchmark_config("FIR_Q15_FixedPt_31Tap", FilterMode::FirQ15FixedPoint, 31, bs, blocks_per_test, sample_rate_hz));
    }

    return results;
}

bool BenchmarkRunner::export_csv(const std::string& filepath, const std::vector<BenchmarkRecord>& records) {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        return false;
    }

    // CSV Header
    file << "Algorithm,BlockSize,FilterTaps,TotalBlocks,TotalSamples,MeanBlockTimeUs,MinBlockTimeUs,"
         << "WorstObservedTimeUs,StdDevTimeUs,ThroughputMsps,ThroughputMBps,RealTimeFactor,HostCpuLoadPercent\n";

    for (const auto& r : records) {
        file << r.algorithm_name << ","
             << r.block_size << ","
             << r.filter_order_taps << ","
             << r.total_blocks << ","
             << r.total_samples << ","
             << std::fixed << std::setprecision(3)
             << r.mean_block_time_us << ","
             << r.min_block_time_us << ","
             << r.max_block_time_us << ","
             << r.stddev_block_time_us << ","
             << std::setprecision(2)
             << r.throughput_msps << ","
             << r.throughput_mbytes_sec << ","
             << std::setprecision(5)
             << r.real_time_factor << ","
             << std::setprecision(3)
             << r.host_cpu_load_percent << "\n";
    }

    return true;
}

void BenchmarkRunner::print_report(const std::vector<BenchmarkRecord>& records) {
    std::cout << "\n==========================================================================================================\n";
    std::cout << "                                FIRMWARE DSP BENCHMARK PERFORMANCE REPORT                                  \n";
    std::cout << "==========================================================================================================\n";
    std::cout << std::left
              << std::setw(24) << "Algorithm"
              << std::setw(10) << "BlockSize"
              << std::setw(14) << "Mean Time(us)"
              << std::setw(14) << "WOET(us)*"
              << std::setw(14) << "Jitter(us)"
              << std::setw(16) << "Throughput(MSps)"
              << std::setw(14) << "RealTimeFactor"
              << "\n";
    std::cout << "----------------------------------------------------------------------------------------------------------\n";

    for (const auto& r : records) {
        std::cout << std::left
                  << std::setw(24) << r.algorithm_name
                  << std::setw(10) << r.block_size
                  << std::setw(14) << std::fixed << std::setprecision(2) << r.mean_block_time_us
                  << std::setw(14) << r.max_block_time_us
                  << std::setw(14) << r.stddev_block_time_us
                  << std::setw(16) << std::setprecision(2) << r.throughput_msps
                  << std::setw(14) << std::setprecision(5) << r.real_time_factor
                  << "\n";
    }
    std::cout << "==========================================================================================================\n";
    std::cout << "* WOET = Worst Observed Execution Time on Host OS.\n";
    std::cout << "EMBEDDED HARDWARE CAVEAT:\n";
    std::cout << "  Host desktop execution timings reflect preemptive OS scheduling, thread quantum slicing, memory\n";
    std::cout << "  paging, and x86 dynamic CPU frequency scaling (SpeedStep/TurboBoost). These figures demonstrate\n";
    std::cout << "  algorithmic scaling across block sizes but DO NOT constitute deterministic Worst-Case Execution Time\n";
    std::cout << "  (WCET) guarantees for bare-metal STM32 / ARM Cortex-M hardware.\n";
    std::cout << "  On STM32, hard real-time cycle-accurate profiling must be performed using the Cortex-M DWT\n";
    std::cout << "  (Data Watchpoint and Trace) cycle counter (DWT->CYCCNT) or a free-running SysTick timer.\n";
    std::cout << "==========================================================================================================\n\n";
}

} // namespace dsp
