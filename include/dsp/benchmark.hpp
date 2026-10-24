#pragma once

#include "dsp/dsp_pipeline.hpp"
#include <string>
#include <vector>

namespace dsp {

/**
 * @brief Benchmark summary results for a single test configuration.
 */
struct BenchmarkRecord {
    std::string algorithm_name;
    std::size_t block_size{0};
    std::size_t filter_order_taps{0};
    std::size_t total_blocks{0};
    std::size_t total_samples{0};
    double mean_block_time_us{0.0};
    double min_block_time_us{0.0};
    double max_block_time_us{0.0}; ///< Worst observed host execution time
    double stddev_block_time_us{0.0}; ///< Execution jitter
    double throughput_msps{0.0};   ///< Mega-samples per second
    double throughput_mbytes_sec{0.0};
    double real_time_factor{0.0};  ///< Execution time / Real-time duration (< 1.0 means faster than real-time)
    double host_cpu_load_percent{0.0};
};

/**
 * @brief Runs systematic DSP benchmarks across block sizes and filter topologies.
 */
class BenchmarkRunner {
public:
    BenchmarkRunner() = default;

    /**
     * @brief Executes a comprehensive benchmark suite.
     * @param blocks_per_test Number of blocks to process per configuration (e.g. 500).
     * @param sample_rate_hz Simulated ADC sample rate for real-time factor calculation.
     */
    std::vector<BenchmarkRecord> run_comprehensive_suite(std::size_t blocks_per_test = 500,
                                                         float sample_rate_hz = 10000.0f);

    /**
     * @brief Exports benchmark results to a structured CSV file.
     * @param filepath Target CSV path.
     * @param records Benchmark records to write.
     * @return true if successfully saved.
     */
    static bool export_csv(const std::string& filepath, const std::vector<BenchmarkRecord>& records);

    /**
     * @brief Prints a formatted markdown/text report to stdout.
     */
    static void print_report(const std::vector<BenchmarkRecord>& records);

private:
    BenchmarkRecord benchmark_config(const std::string& name,
                                     FilterMode mode,
                                     std::size_t num_taps,
                                     std::size_t block_size,
                                     std::size_t num_blocks,
                                     float sample_rate_hz);
};

} // namespace dsp
