#include "dsp/simulated_adc.hpp"
#include <cmath>
#include <algorithm>

namespace dsp {

namespace {
constexpr float TWO_PI = 6.28318530717958647692f;
}

SimulatedAdc::SimulatedAdc(const SimulatedAdcConfig& config)
    : config_(config),
      sample_counter_(0),
      lcg_state_(config.random_seed),
      custom_index_(0),
      max_counts_((1u << config.bit_depth) - 1u) {}

void SimulatedAdc::reset() noexcept {
    sample_counter_ = 0;
    lcg_state_ = config_.random_seed;
    custom_index_ = 0;
}

void SimulatedAdc::set_custom_sequence(const std::vector<float>& sequence) {
    custom_sequence_ = sequence;
    custom_index_ = 0;
    config_.waveform = WaveformType::CustomSequence;
}

uint16_t SimulatedAdc::quantize_voltage(float v) const noexcept {
    if (v <= 0.0f) return 0;
    if (v >= config_.v_ref) return static_cast<uint16_t>(max_counts_);

    float ratio = v / config_.v_ref;
    float counts = std::round(ratio * static_cast<float>(max_counts_));
    if (counts < 0.0f) counts = 0.0f;
    if (counts > static_cast<float>(max_counts_)) counts = static_cast<float>(max_counts_);

    return static_cast<uint16_t>(counts);
}

float SimulatedAdc::generate_pseudo_noise() noexcept {
    // 32-bit Linear Congruential Generator (Numerical Recipes parameters)
    lcg_state_ = lcg_state_ * 1664525u + 1013904223u;
    // Map to [-1.0, +1.0]
    float uniform = (static_cast<float>(lcg_state_) / 4294967295.0f) * 2.0f - 1.0f;
    return uniform * std::sqrt(config_.noise_variance * 3.0f); // Uniform variance = a^2 / 3
}

float SimulatedAdc::compute_analog_voltage(uint64_t n) noexcept {
    float v_half = config_.v_ref * 0.5f;
    float norm_val = 0.0f;

    switch (config_.waveform) {
        case WaveformType::Sine: {
            float t = static_cast<float>(n) / config_.sample_rate_hz;
            norm_val = config_.primary_amplitude * std::sin(TWO_PI * config_.primary_freq_hz * t);
            break;
        }
        case WaveformType::MultiTone: {
            float t = static_cast<float>(n) / config_.sample_rate_hz;
            float tone1 = config_.primary_amplitude * std::sin(TWO_PI * config_.primary_freq_hz * t);
            float tone2 = config_.secondary_amplitude * std::sin(TWO_PI * config_.secondary_freq_hz * t);
            norm_val = tone1 + tone2;
            break;
        }
        case WaveformType::SineWithNoise: {
            float t = static_cast<float>(n) / config_.sample_rate_hz;
            float tone = config_.primary_amplitude * std::sin(TWO_PI * config_.primary_freq_hz * t);
            norm_val = tone + generate_pseudo_noise();
            break;
        }
        case WaveformType::Impulse: {
            norm_val = (n == 0) ? 1.0f : 0.0f;
            break;
        }
        case WaveformType::Step: {
            norm_val = (n >= 4) ? 1.0f : 0.0f;
            break;
        }
        case WaveformType::SquareWave: {
            float t = static_cast<float>(n) / config_.sample_rate_hz;
            float phase = std::fmod(t * config_.primary_freq_hz, 1.0f);
            norm_val = (phase < 0.5f) ? config_.primary_amplitude : -config_.primary_amplitude;
            break;
        }
        case WaveformType::CustomSequence: {
            if (!custom_sequence_.empty()) {
                norm_val = custom_sequence_[custom_index_ % custom_sequence_.size()];
                custom_index_++;
            } else {
                norm_val = 0.0f;
            }
            break;
        }
    }

    // Clamp normalized amplitude to [-1.0, 1.0] to prevent clipping before ADC
    norm_val = std::max(-1.0f, std::min(1.0f, norm_val));

    // Convert AC-coupled [-1.0, +1.0] to DC-biased voltage centered at Vref/2
    float v = v_half + norm_val * (v_half * 0.95f); // 95% full-scale to stay within linear rail
    return std::max(0.0f, std::min(config_.v_ref, v));
}

SimulatedAdcSample SimulatedAdc::acquire_sample() noexcept {
    SimulatedAdcSample sample;
    sample.sample_index = sample_counter_;

    float analog_v = compute_analog_voltage(sample_counter_);

    if (config_.enable_quantization) {
        sample.raw_counts = quantize_voltage(analog_v);
        sample.voltage = (static_cast<float>(sample.raw_counts) / static_cast<float>(max_counts_)) * config_.v_ref;
    } else {
        sample.voltage = analog_v;
        sample.raw_counts = quantize_voltage(analog_v);
    }

    // Convert back to AC normalized value [-1.0, +1.0] for DSP pipeline
    float v_half = config_.v_ref * 0.5f;
    sample.normalized = (sample.voltage - v_half) / (v_half * 0.95f);
    sample.normalized = std::max(-1.0f, std::min(1.0f, sample.normalized));

    sample_counter_++;
    return sample;
}

std::size_t SimulatedAdc::acquire_block(float* out_buffer, std::size_t count) noexcept {
    if (out_buffer == nullptr || count == 0) {
        return 0;
    }

    for (std::size_t i = 0; i < count; ++i) {
        SimulatedAdcSample s = acquire_sample();
        out_buffer[i] = s.normalized;
    }
    return count;
}

} // namespace dsp
