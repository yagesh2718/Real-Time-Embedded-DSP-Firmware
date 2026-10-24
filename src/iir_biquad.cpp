#include "dsp/iir_biquad.hpp"
#include <cmath>
#include <stdexcept>

namespace dsp {

namespace {
constexpr float PI = 3.14159265358979323846f;
constexpr float TWO_PI = 6.28318530717958647692f;
}

bool BiquadCoeffs::is_stable() const noexcept {
    // Schur-Cohn / Jury stability conditions for 2nd order polynomial: 1 + a1*z^-1 + a2*z^-2 = 0
    if (std::abs(a2) >= 1.0f) {
        return false;
    }
    if ((1.0f + a1 + a2) <= 0.0f) {
        return false;
    }
    if ((1.0f - a1 + a2) <= 0.0f) {
        return false;
    }
    return true;
}

std::pair<std::complex<float>, std::complex<float>> BiquadCoeffs::poles() const noexcept {
    // Roots of z^2 + a1*z + a2 = 0
    float disc = a1 * a1 - 4.0f * a2;
    if (disc >= 0.0f) {
        float r1 = (-a1 + std::sqrt(disc)) * 0.5f;
        float r2 = (-a1 - std::sqrt(disc)) * 0.5f;
        return {std::complex<float>(r1, 0.0f), std::complex<float>(r2, 0.0f)};
    } else {
        float real_part = -a1 * 0.5f;
        float imag_part = std::sqrt(-disc) * 0.5f;
        return {std::complex<float>(real_part, imag_part), std::complex<float>(real_part, -imag_part)};
    }
}

std::complex<float> BiquadCoeffs::frequency_response(float normalized_rad_per_sample) const noexcept {
    // H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2) where z = e^{j*w}
    std::complex<float> z1 = std::polar(1.0f, -normalized_rad_per_sample);
    std::complex<float> z2 = std::polar(1.0f, -2.0f * normalized_rad_per_sample);

    std::complex<float> num = b0 + b1 * z1 + b2 * z2;
    std::complex<float> den = 1.0f + a1 * z1 + a2 * z2;
    return num / den;
}

IirBiquad::IirBiquad(const BiquadCoeffs& coeffs) {
    set_coefficients(coeffs, false);
}

bool IirBiquad::set_coefficients(const BiquadCoeffs& coeffs, bool enforce_stability) {
    if (enforce_stability && !coeffs.is_stable()) {
        return false;
    }
    coeffs_ = coeffs;
    reset();
    return true;
}

void IirBiquad::reset() noexcept {
    d1_ = 0.0f;
    d2_ = 0.0f;
}

float IirBiquad::process_sample(float x) noexcept {
    // Direct Form II Transposed (DF2T) execution:
    // y[n] = b0 * x[n] + d1[n-1]
    // d1[n] = b1 * x[n] - a1 * y[n] + d2[n-1]
    // d2[n] = b2 * x[n] - a2 * y[n]
    float y = coeffs_.b0 * x + d1_;
    d1_ = coeffs_.b1 * x - coeffs_.a1 * y + d2_;
    d2_ = coeffs_.b2 * x - coeffs_.a2 * y;
    return y;
}

void IirBiquad::process_block(const float* in, float* out, std::size_t count) noexcept {
    if (in == nullptr || out == nullptr) {
        return;
    }
    for (std::size_t i = 0; i < count; ++i) {
        out[i] = process_sample(in[i]);
    }
}

BiquadCoeffs IirBiquad::design_lowpass(float cutoff_hz, float sample_rate_hz, float Q) {
    BiquadCoeffs c;
    float w0 = TWO_PI * (cutoff_hz / sample_rate_hz);
    float cos_w0 = std::cos(w0);
    float sin_w0 = std::sin(w0);
    float alpha = sin_w0 / (2.0f * Q);

    float b0 = (1.0f - cos_w0) * 0.5f;
    float b1 = 1.0f - cos_w0;
    float b2 = (1.0f - cos_w0) * 0.5f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cos_w0;
    float a2 = 1.0f - alpha;

    float inv_a0 = 1.0f / a0;
    c.b0 = b0 * inv_a0;
    c.b1 = b1 * inv_a0;
    c.b2 = b2 * inv_a0;
    c.a1 = a1 * inv_a0;
    c.a2 = a2 * inv_a0;
    return c;
}

BiquadCoeffs IirBiquad::design_highpass(float cutoff_hz, float sample_rate_hz, float Q) {
    BiquadCoeffs c;
    float w0 = TWO_PI * (cutoff_hz / sample_rate_hz);
    float cos_w0 = std::cos(w0);
    float sin_w0 = std::sin(w0);
    float alpha = sin_w0 / (2.0f * Q);

    float b0 = (1.0f + cos_w0) * 0.5f;
    float b1 = -(1.0f + cos_w0);
    float b2 = (1.0f + cos_w0) * 0.5f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cos_w0;
    float a2 = 1.0f - alpha;

    float inv_a0 = 1.0f / a0;
    c.b0 = b0 * inv_a0;
    c.b1 = b1 * inv_a0;
    c.b2 = b2 * inv_a0;
    c.a1 = a1 * inv_a0;
    c.a2 = a2 * inv_a0;
    return c;
}

BiquadCoeffs IirBiquad::design_bandpass(float center_hz, float sample_rate_hz, float Q) {
    BiquadCoeffs c;
    float w0 = TWO_PI * (center_hz / sample_rate_hz);
    float cos_w0 = std::cos(w0);
    float sin_w0 = std::sin(w0);
    float alpha = sin_w0 / (2.0f * Q);

    float b0 = alpha;
    float b1 = 0.0f;
    float b2 = -alpha;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cos_w0;
    float a2 = 1.0f - alpha;

    float inv_a0 = 1.0f / a0;
    c.b0 = b0 * inv_a0;
    c.b1 = b1 * inv_a0;
    c.b2 = b2 * inv_a0;
    c.a1 = a1 * inv_a0;
    c.a2 = a2 * inv_a0;
    return c;
}

BiquadCoeffs IirBiquad::design_notch(float notch_hz, float sample_rate_hz, float Q) {
    BiquadCoeffs c;
    float w0 = TWO_PI * (notch_hz / sample_rate_hz);
    float cos_w0 = std::cos(w0);
    float sin_w0 = std::sin(w0);
    float alpha = sin_w0 / (2.0f * Q);

    float b0 = 1.0f;
    float b1 = -2.0f * cos_w0;
    float b2 = 1.0f;
    float a0 = 1.0f + alpha;
    float a1 = -2.0f * cos_w0;
    float a2 = 1.0f - alpha;

    float inv_a0 = 1.0f / a0;
    c.b0 = b0 * inv_a0;
    c.b1 = b1 * inv_a0;
    c.b2 = b2 * inv_a0;
    c.a1 = a1 * inv_a0;
    c.a2 = a2 * inv_a0;
    return c;
}

IirCascade::IirCascade(const std::vector<BiquadCoeffs>& stages) {
    for (const auto& s : stages) {
        add_stage(s);
    }
}

bool IirCascade::add_stage(const BiquadCoeffs& stage) {
    if (num_stages_ >= MAX_STAGES) {
        return false;
    }
    stages_[num_stages_].set_coefficients(stage, false);
    num_stages_++;
    return true;
}

float IirCascade::process_sample(float x) noexcept {
    float val = x;
    for (std::size_t i = 0; i < num_stages_; ++i) {
        val = stages_[i].process_sample(val);
    }
    return val;
}

void IirCascade::process_block(const float* in, float* out, std::size_t count) noexcept {
    if (in == nullptr || out == nullptr) return;
    for (std::size_t i = 0; i < count; ++i) {
        out[i] = process_sample(in[i]);
    }
}

void IirCascade::reset() noexcept {
    for (std::size_t i = 0; i < num_stages_; ++i) {
        stages_[i].reset();
    }
}

IirCascade IirCascade::design_butterworth_4th_lowpass(float cutoff_hz, float sample_rate_hz) {
    IirCascade cascade;
    // 4th order Butterworth decomposition: two biquad stages with Q1=1.306563 and Q2=0.541196
    BiquadCoeffs s1 = IirBiquad::design_lowpass(cutoff_hz, sample_rate_hz, 1.30656296f);
    BiquadCoeffs s2 = IirBiquad::design_lowpass(cutoff_hz, sample_rate_hz, 0.54119610f);
    cascade.add_stage(s1);
    cascade.add_stage(s2);
    return cascade;
}

} // namespace dsp
