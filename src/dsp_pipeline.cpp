#include "dsp/dsp_pipeline.hpp"
#include <chrono>
#include <algorithm>

namespace dsp {

DspPipeline::DspPipeline(IAdcSource& adc_source, const PipelineConfig& config)
    : adc_(adc_source),
      config_(config),
      ring_buffer_(config.overflow_policy) {
    if (config_.block_size == 0) config_.block_size = 64;
    if (config_.block_size > MAX_BLOCK_SIZE) config_.block_size = MAX_BLOCK_SIZE;

    // Default 31-tap FIR and 1000 Hz Butterworth biquad
    fir_filter_.set_coefficients(FirFilter::design_lowpass(1000.0f, adc_.sample_rate_hz(), 31));
    iir_filter_.set_coefficients(IirBiquad::design_lowpass(1000.0f, adc_.sample_rate_hz(), 0.7071f));
    fir_q15_.set_coefficients(fir_filter_.coefficients(), fir_filter_.num_taps());
}

void DspPipeline::reset() {
    ring_buffer_.clear();
    ring_buffer_.reset_overflow_count();
    fir_filter_.reset();
    iir_filter_.reset();
    fir_q15_.reset();
    adc_.reset();
    total_samples_processed_ = 0;
    total_blocks_processed_ = 0;
    total_execution_time_us_ = 0.0;
    max_block_time_us_ = 0.0;
    min_block_time_us_ = 1e9;
}

double DspPipeline::mean_execution_time_us() const noexcept {
    return (total_blocks_processed_ > 0)
        ? (total_execution_time_us_ / static_cast<double>(total_blocks_processed_))
        : 0.0;
}

void DspPipeline::process_single_block(const float* raw_block, float* out_block, std::size_t count) {
    switch (config_.filter_mode) {
        case FilterMode::PassThrough:
            for (std::size_t i = 0; i < count; ++i) {
                out_block[i] = raw_block[i];
            }
            break;

        case FilterMode::FirFloat:
            fir_filter_.process_block(raw_block, out_block, count);
            break;

        case FilterMode::IirFloat:
            iir_filter_.process_block(raw_block, out_block, count);
            break;

        case FilterMode::FirQ15FixedPoint:
            for (std::size_t i = 0; i < count; ++i) {
                block_q15_in_[i] = qformat::float_to_q15(raw_block[i]);
            }
            fir_q15_.process_block(block_q15_in_.data(), block_q15_out_.data(), count);
            for (std::size_t i = 0; i < count; ++i) {
                out_block[i] = qformat::q15_to_float(block_q15_out_[i]);
            }
            break;
    }
}

std::size_t DspPipeline::step(std::size_t samples_to_acquire) {
    if (samples_to_acquire == 0) {
        samples_to_acquire = config_.block_size;
    }

    // 1. Acquisition phase: fetch samples from ADC
    for (std::size_t i = 0; i < samples_to_acquire; ++i) {
        SimulatedAdcSample s = adc_.acquire_sample();
        ring_buffer_.push(s.normalized);
    }

    // 2. Processing phase: process any complete blocks currently in circular buffer
    std::size_t blocks_processed = 0;
    const std::size_t N = config_.block_size;

    while (ring_buffer_.size() >= N) {
        // Read block from ring buffer
        std::size_t read_count = ring_buffer_.read_block(block_in_.data(), N);
        if (read_count < N) {
            break;
        }

        // Measure host execution time of pure DSP processing loop
        auto t_start = std::chrono::high_resolution_clock::now();
        process_single_block(block_in_.data(), block_out_.data(), N);
        auto t_end = std::chrono::high_resolution_clock::now();

        double elapsed_us = std::chrono::duration<double, std::micro>(t_end - t_start).count();

        // Theoretical real-time duration of block on target hardware
        double theoretical_us = (static_cast<double>(N) / static_cast<double>(adc_.sample_rate_hz())) * 1e6;
        double cpu_load = (theoretical_us > 0.0) ? (elapsed_us / theoretical_us) * 100.0 : 0.0;

        BlockExecutionStats stats;
        stats.block_index = total_blocks_processed_;
        stats.block_size = N;
        stats.execution_time_us = elapsed_us;
        stats.theoretical_block_us = theoretical_us;
        stats.cpu_load_percent = cpu_load;
        stats.buffer_occupancy = ring_buffer_.size();
        stats.overflow_events = ring_buffer_.overflow_count();

        // Update statistics
        total_samples_processed_ += N;
        total_blocks_processed_++;
        total_execution_time_us_ += elapsed_us;
        if (elapsed_us > max_block_time_us_) {
            max_block_time_us_ = elapsed_us;
        }
        if (elapsed_us < min_block_time_us_) {
            min_block_time_us_ = elapsed_us;
        }

        // Deliver block output
        if (output_cb_) {
            output_cb_(block_in_.data(), block_out_.data(), N, stats);
        }

        blocks_processed++;
    }

    return blocks_processed;
}

void DspPipeline::run_samples(std::size_t total_samples) {
    std::size_t step_size = config_.block_size;
    std::size_t acquired = 0;

    while (acquired < total_samples) {
        std::size_t to_acquire = std::min(step_size, total_samples - acquired);
        step(to_acquire);
        acquired += to_acquire;
    }
}

} // namespace dsp
