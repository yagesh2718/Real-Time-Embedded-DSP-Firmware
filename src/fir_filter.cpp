#include "dsp/fir_filter.hpp"
#include <cmath>
#include <numeric>
#include <algorithm>

namespace dsp {

namespace {
constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = 6.28318530717958647692f;
}

FirFilter::FirFilter() {
    set_coefficients(default_coeffs_31_tap());
}

FirFilter::FirFilter(const std::vector<float>& coeffs) {
    set_coefficients(coeffs);
}

FirFilter::FirFilter(const float* coeffs, std::size_t num_taps) {
    set_coefficients(coeffs, num_taps);
}

bool FirFilter::set_coefficients(const std::vector<float>& coeffs) {
    return set_coefficients(coeffs.data(), coeffs.size());
}

bool FirFilter::set_coefficients(const float* coeffs, std::size_t num_taps) {
    if (coeffs == nullptr || num_taps == 0 || num_taps > MAX_TAPS) {
        return false;
    }

    num_taps_ = num_taps;
    for (std::size_t i = 0; i < num_taps; ++i) {
        coeffs_[i] = coeffs[i];
    }
    reset();
    return true;
}

void FirFilter::reset() noexcept {
    state_.fill(0.0f);
    state_idx_ = 0;
}

float FirFilter::process_sample(float input) noexcept {
    if (num_taps_ == 0) {
        return input;
    }

    state_[state_idx_] = input;

    float accumulator = 0.0f;
    std::size_t read_idx = state_idx_;

    for (std::size_t k = 0; k < num_taps_; ++k) {
        accumulator += coeffs_[k] * state_[read_idx];
        if (read_idx == 0) {
            read_idx = num_taps_ - 1;
        } else {
            read_idx--;
        }
    }

    state_idx_++;
    if (state_idx_ >= num_taps_) {
        state_idx_ = 0;
    }

    return accumulator;
}

void FirFilter::process_block(const float* in, float* out, std::size_t count) noexcept {
    if (in == nullptr || out == nullptr) {
        return;
    }

    for (std::size_t i = 0; i < count; ++i) {
        out[i] = process_sample(in[i]);
    }
}

std::complex<float> FirFilter::frequency_response(float normalized_rad_per_sample) const noexcept {
    std::complex<float> h(0.0f, 0.0f);
    for (std::size_t k = 0; k < num_taps_; ++k) {
        float angle = -normalized_rad_per_sample * static_cast<float>(k);
        h += coeffs_[k] * std::complex<float>(std::cos(angle), std::sin(angle));
    }
    return h;
}

std::vector<float> FirFilter::design_lowpass(float cutoff_hz, float sample_rate_hz, std::size_t num_taps) {
    if (num_taps == 0) num_taps = 31;
    if (num_taps > MAX_TAPS) num_taps = MAX_TAPS;

    std::vector<float> h(num_taps);
    float fc = cutoff_hz / sample_rate_hz; // Normalized cutoff
    if (fc > 0.49f) fc = 0.49f;
    if (fc < 0.001f) fc = 0.001f;

    float tau = static_cast<float>(num_taps - 1) * 0.5f;
    float sum = 0.0f;

    for (std::size_t i = 0; i < num_taps; ++i) {
        float n = static_cast<float>(i) - tau;
        float sinc;
        if (std::abs(n) < 1e-6f) {
            sinc = 2.0f * fc;
        } else {
            sinc = std::sin(TWO_PI * fc * n) / (PI * n);
        }

        // Hamming window: 0.54 - 0.46 * cos(2*pi*i / (M-1))
        float window = 0.54f - 0.46f * std::cos((TWO_PI * static_cast<float>(i)) / static_cast<float>(num_taps - 1));
        h[i] = sinc * window;
        sum += h[i];
    }

    // Normalize DC gain to unity (0 dB)
    if (std::abs(sum) > 1e-7f) {
        for (std::size_t i = 0; i < num_taps; ++i) {
            h[i] /= sum;
        }
    }

    return h;
}

const std::vector<float>& FirFilter::default_coeffs_31_tap() {
    static const std::vector<float> default_coeffs = design_lowpass(1000.0f, 10000.0f, 31);
    return default_coeffs;
}

} // namespace dsp
