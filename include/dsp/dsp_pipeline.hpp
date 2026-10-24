#pragma once

#include "dsp/circular_buffer.hpp"
#include "dsp/simulated_adc.hpp"
#include "dsp/fir_filter.hpp"
#include "dsp/iir_biquad.hpp"
#include "dsp/fixed_point_dsp.hpp"
#include <cstddef>
#include <cstdint>
#include <chrono>
#include <vector>
#include <functional>
#include <memory>

namespace dsp {

/**
 * @brief Selectable processing filter mode in firmware pipeline.
 */
enum class FilterMode {
    PassThrough,
    FirFloat,
    IirFloat,
    FirQ15FixedPoint
};

/**
 * @brief Telemetry metrics for a single processed block.
 */
struct BlockExecutionStats {
    uint64_t block_index{0};
    std::size_t block_size{0};
    double execution_time_us{0.0};   ///< Measured CPU execution time on host (microseconds)
    double theoretical_block_us{0.0};///< Real-time duration of block = (N / Fs) * 1e6
    double cpu_load_percent{0.0};    ///< (execution_time / theoretical_block) * 100%
    std::size_t buffer_occupancy{0}; ///< Samples pending in buffer at block extraction
    std::size_t overflow_events{0};  ///< Cumulative dropped/overwritten sample count
};

/**
 * @brief Configuration for the real-time DSP pipeline.
 */
struct PipelineConfig {
    std::size_t block_size{64};       ///< Processing block size N (e.g., 32, 64, 128)
    std::size_t buffer_capacity{512}; ///< Circular buffer capacity
    FilterMode filter_mode{FilterMode::FirFloat};
    OverflowPolicy overflow_policy{OverflowPolicy::DropNewest};
};

/**
 * @brief Callback signature for processed block output delivery.
 */
using BlockOutputCallback = std::function<void(const float* raw_in,
                                               const float* filtered_out,
                                               std::size_t count,
                                               const BlockExecutionStats& stats)>;

/**
 * @brief Firmware DSP Processing Executive.
 *
 * Enforces strict separation of concerns:
 *   1. Acquisition (IAdcSource)
 *   2. Buffering (CircularBuffer FIFO)
 *   3. Periodic Block Processing (DSP Filters)
 *   4. Output Sink / Telemetry (BlockOutputCallback)
 *
 * Uses bounded static arrays for processing blocks with zero dynamic allocation
 * in the real-time sample processing path.
 */
class DspPipeline {
public:
    static constexpr std::size_t MAX_BLOCK_SIZE = 512;
    static constexpr std::size_t DEFAULT_RING_CAPACITY = 2048;

    DspPipeline(IAdcSource& adc_source, const PipelineConfig& config = PipelineConfig{});

    /**
     * @brief Steps the pipeline by acquiring 'samples_to_acquire' from ADC into buffer,
     * and processing all complete blocks available.
     * @return Number of blocks processed during this step.
     */
    std::size_t step(std::size_t samples_to_acquire = 0);

    /**
     * @brief Runs the pipeline continuously for a specified total number of samples.
     */
    void run_samples(std::size_t total_samples);

    /**
     * @brief Configures the output delivery callback.
     */
    void set_output_callback(BlockOutputCallback cb) { output_cb_ = std::move(cb); }

    /**
     * @brief Accessors for internal DSP filter modules.
     */
    FirFilter& fir_filter() noexcept { return fir_filter_; }
    IirBiquad& iir_filter() noexcept { return iir_filter_; }
    FixedPointFirQ15& fir_q15_filter() noexcept { return fir_q15_; }

    void set_filter_mode(FilterMode mode) noexcept { config_.filter_mode = mode; }
    FilterMode filter_mode() const noexcept { return config_.filter_mode; }

    const PipelineConfig& config() const noexcept { return config_; }
    CircularBuffer<float, DEFAULT_RING_CAPACITY>& circular_buffer() noexcept { return ring_buffer_; }

    void reset();

    /**
     * @brief Returns cumulative telemetry statistics.
     */
    uint64_t total_samples_processed() const noexcept { return total_samples_processed_; }
    uint64_t total_blocks_processed() const noexcept { return total_blocks_processed_; }
    double max_observed_execution_time_us() const noexcept { return max_block_time_us_; }
    double min_observed_execution_time_us() const noexcept { return min_block_time_us_; }
    double mean_execution_time_us() const noexcept;

private:
    void process_single_block(const float* raw_block, float* out_block, std::size_t count);

    IAdcSource& adc_;
    PipelineConfig config_;
    CircularBuffer<float, DEFAULT_RING_CAPACITY> ring_buffer_;

    FirFilter fir_filter_;
    IirBiquad iir_filter_;
    FixedPointFirQ15 fir_q15_;

    // Static scratchpad arrays for zero-allocation block processing
    std::array<float, MAX_BLOCK_SIZE> block_in_{};
    std::array<float, MAX_BLOCK_SIZE> block_out_{};
    std::array<q15_t, MAX_BLOCK_SIZE> block_q15_in_{};
    std::array<q15_t, MAX_BLOCK_SIZE> block_q15_out_{};

    BlockOutputCallback output_cb_{nullptr};

    uint64_t total_samples_processed_{0};
    uint64_t total_blocks_processed_{0};
    double total_execution_time_us_{0.0};
    double max_block_time_us_{0.0};
    double min_block_time_us_{1e9};
};

} // namespace dsp
