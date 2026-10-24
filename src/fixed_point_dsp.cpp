#include "dsp/fixed_point_dsp.hpp"
#include <cmath>
#include <algorithm>

namespace dsp {

FixedPointFirQ15::FixedPointFirQ15(const std::vector<float>& float_coeffs) {
    set_coefficients(float_coeffs.data(), float_coeffs.size());
}

FixedPointFirQ15::FixedPointFirQ15(const q15_t* q15_coeffs, std::size_t num_taps) {
    set_coefficients(q15_coeffs, num_taps);
}

bool FixedPointFirQ15::set_coefficients(const float* float_coeffs, std::size_t num_taps) {
    if (float_coeffs == nullptr || num_taps == 0 || num_taps > MAX_TAPS) {
        return false;
    }
    num_taps_ = num_taps;
    for (std::size_t i = 0; i < num_taps; ++i) {
        coeffs_[i] = qformat::float_to_q15(float_coeffs[i]);
    }
    reset();
    return true;
}

bool FixedPointFirQ15::set_coefficients(const q15_t* q15_coeffs, std::size_t num_taps) {
    if (q15_coeffs == nullptr || num_taps == 0 || num_taps > MAX_TAPS) {
        return false;
    }
    num_taps_ = num_taps;
    for (std::size_t i = 0; i < num_taps; ++i) {
        coeffs_[i] = q15_coeffs[i];
    }
    reset();
    return true;
}

void FixedPointFirQ15::reset() noexcept {
    state_.fill(0);
    state_idx_ = 0;
}

q15_t FixedPointFirQ15::process_sample(q15_t input) noexcept {
    if (num_taps_ == 0) return input;

    state_[state_idx_] = input;

    // 64-bit accumulator with guard bits, mirroring ARM CMSIS-DSP arm_fir_q15
    int64_t acc = 0;
    std::size_t read_idx = state_idx_;

    for (std::size_t k = 0; k < num_taps_; ++k) {
        acc += static_cast<int64_t>(coeffs_[k]) * static_cast<int64_t>(state_[read_idx]);
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

    // Rounding: add 0x4000 (0.5 in Q15 LSB) before right-shift by 15
    acc += 0x4000;
    int32_t result = static_cast<int32_t>(acc >> 15);
    return qformat::clip_q15(result);
}

void FixedPointFirQ15::process_block(const q15_t* in, q15_t* out, std::size_t count) noexcept {
    if (in == nullptr || out == nullptr) return;
    for (std::size_t i = 0; i < count; ++i) {
        out[i] = process_sample(in[i]);
    }
}

bool FixedPointBiquadQ15::set_coefficients(float b0, float b1, float b2, float a1, float a2) {
    // In biquad filters, a1 can be up to ~2.0, so we scale by 2^14 (16384) to preserve headroom
    constexpr float SCALE = 16384.0f;
    auto to_q14 = [](float f) -> int16_t {
        int32_t v = static_cast<int32_t>(std::round(f * SCALE));
        if (v > 32767) v = 32767;
        if (v < -32768) v = -32768;
        return static_cast<int16_t>(v);
    };

    b0_ = to_q14(b0);
    b1_ = to_q14(b1);
    b2_ = to_q14(b2);
    a1_ = to_q14(a1);
    a2_ = to_q14(a2);
    reset();
    return true;
}

void FixedPointBiquadQ15::reset() noexcept {
    x1_ = 0;
    x2_ = 0;
    y1_ = 0;
    y2_ = 0;
}

q15_t FixedPointBiquadQ15::process_sample(q15_t input) noexcept {
    // Direct Form I:
    // y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
    int64_t acc = static_cast<int64_t>(b0_) * input
                + static_cast<int64_t>(b1_) * x1_
                + static_cast<int64_t>(b2_) * x2_
                - static_cast<int64_t>(a1_) * y1_
                - static_cast<int64_t>(a2_) * y2_;

    // Add rounding bias for 14-bit right shift
    acc += 0x2000;
    int32_t y = static_cast<int32_t>(acc >> 14);
    q15_t y_sat = qformat::clip_q15(y);

    x2_ = x1_;
    x1_ = input;
    y2_ = y1_;
    y1_ = y_sat;

    return y_sat;
}

void FixedPointBiquadQ15::process_block(const q15_t* in, q15_t* out, std::size_t count) noexcept {
    if (in == nullptr || out == nullptr) return;
    for (std::size_t i = 0; i < count; ++i) {
        out[i] = process_sample(in[i]);
    }
}

DspErrorMetrics evaluate_quantization_error(const float* ref, const float* test, std::size_t count) {
    DspErrorMetrics m;
    m.sample_count = count;
    if (ref == nullptr || test == nullptr || count == 0) {
        return m;
    }

    double sse = 0.0;
    double sig_energy = 0.0;
    float max_err = 0.0f;

    for (std::size_t i = 0; i < count; ++i) {
        float r = ref[i];
        float t = test[i];
        float diff = std::abs(r - t);
        if (diff > max_err) {
            max_err = diff;
        }
        sse += static_cast<double>(diff) * static_cast<double>(diff);
        sig_energy += static_cast<double>(r) * static_cast<double>(r);
    }

    m.mean_squared_error = static_cast<float>(sse / static_cast<double>(count));
    m.root_mean_squared_error = std::sqrt(m.mean_squared_error);
    m.peak_absolute_error = max_err;

    if (sse < 1e-14) {
        m.sqnr_db = 99.0f; // Near perfect alignment
    } else {
        m.sqnr_db = static_cast<float>(10.0 * std::log10(sig_energy / sse));
    }

    m.enob = (m.sqnr_db - 1.76f) / 6.02f;
    if (m.enob < 0.0f) m.enob = 0.0f;

    return m;
}

} // namespace dsp
