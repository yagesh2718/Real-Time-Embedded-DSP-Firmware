#pragma once

#include <cstddef>
#include <array>
#include <vector>
#include <complex>

namespace dsp {

/**
 * @brief Biquad coefficient structure (normalized such that a0 = 1.0).
 *
 * Transfer function:
 *             b0 + b1 * z^{-1} + b2 * z^{-2}
 *   H(z) = ------------------------------------
 *              1 + a1 * z^{-1} + a2 * z^{-2}
 */
struct BiquadCoeffs {
    float b0{1.0f};
    float b1{0.0f};
    float b2{0.0f};
    float a1{0.0f};
    float a2{0.0f};

    /**
     * @brief Evaluates Biquad stability using the Schur-Cohn / Jury stability test.
     * Roots of denominator z^2 + a1*z + a2 = 0 must satisfy |z| < 1.
     * Conditions:
     *   1) |a2| < 1
     *   2) 1 + a1 + a2 > 0
     *   3) 1 - a1 + a2 > 0
     */
    bool is_stable() const noexcept;

    /**
     * @brief Returns the complex poles of the biquad denominator.
     */
    std::pair<std::complex<float>, std::complex<float>> poles() const noexcept;

    /**
     * @brief Computes frequency response H(e^{j omega}) at normalized frequency omega in [0, pi].
     */
    std::complex<float> frequency_response(float normalized_rad_per_sample) const noexcept;
};

/**
 * @brief Direct Form II Transposed (DF2T) Single-Biquad IIR Filter.
 *
 * Implements the DF2T difference equations:
 *   y[n]  = b0 * x[n] + d1[n-1]
 *   d1[n] = b1 * x[n] - a1 * y[n] + d2[n-1]
 *   d2[n] = b2 * x[n] - a2 * y[n]
 *
 * State requirements: Only 2 float registers (d1, d2), zero heap allocations.
 * Equivalent to CMSIS-DSP arm_biquad_cascade_df2T_f32 single biquad stage.
 */
class IirBiquad {
public:
    IirBiquad() = default;
    explicit IirBiquad(const BiquadCoeffs& coeffs);

    /**
     * @brief Processes a single sample in real time.
     */
    float process_sample(float x) noexcept;

    /**
     * @brief Processes a block of samples.
     */
    void process_block(const float* in, float* out, std::size_t count) noexcept;

    /**
     * @brief Resets filter state registers (d1, d2) to zero.
     */
    void reset() noexcept;

    /**
     * @brief Sets filter coefficients. Returns false if coefficients are unstable.
     */
    bool set_coefficients(const BiquadCoeffs& coeffs, bool enforce_stability = true);

    const BiquadCoeffs& coefficients() const noexcept { return coeffs_; }

    // --- Standard Filter Design Factories (Bilinear Transform) ---
    static BiquadCoeffs design_lowpass(float cutoff_hz, float sample_rate_hz, float Q = 0.70710678f);
    static BiquadCoeffs design_highpass(float cutoff_hz, float sample_rate_hz, float Q = 0.70710678f);
    static BiquadCoeffs design_bandpass(float center_hz, float sample_rate_hz, float Q = 2.0f);
    static BiquadCoeffs design_notch(float notch_hz, float sample_rate_hz, float Q = 10.0f);

private:
    BiquadCoeffs coeffs_{};
    float d1_{0.0f}; ///< Delay register 1
    float d2_{0.0f}; ///< Delay register 2
};

/**
 * @brief Cascaded Biquad (Second-Order Sections - SOS) IIR Filter.
 *
 * Suitable for higher-order filters (e.g. 4th-order, 6th-order Butterworth/Chebyshev)
 * while preserving numerical stability and minimizing quantization noise.
 */
class IirCascade {
public:
    static constexpr std::size_t MAX_STAGES = 4;

    IirCascade() = default;
    explicit IirCascade(const std::vector<BiquadCoeffs>& stages);

    float process_sample(float x) noexcept;
    void process_block(const float* in, float* out, std::size_t count) noexcept;
    void reset() noexcept;

    bool add_stage(const BiquadCoeffs& stage);
    std::size_t num_stages() const noexcept { return num_stages_; }

    /**
     * @brief 4th-order Butterworth Lowpass Filter (cascaded 2 biquads).
     */
    static IirCascade design_butterworth_4th_lowpass(float cutoff_hz, float sample_rate_hz);

private:
    std::array<IirBiquad, MAX_STAGES> stages_{};
    std::size_t num_stages_{0};
};

} // namespace dsp
