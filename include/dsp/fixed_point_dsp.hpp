#pragma once

#include "dsp/qformat.hpp"
#include <vector>
#include <array>
#include <cstddef>

namespace dsp {

/**
 * @brief Statistical accuracy metrics comparing fixed-point vs float reference.
 */
struct DspErrorMetrics {
    float mean_squared_error{0.0f};  ///< MSE: mean((ref - fixed)^2)
    float root_mean_squared_error{0.0f}; ///< RMSE
    float peak_absolute_error{0.0f}; ///< Max absolute deviation |ref - fixed|
    float sqnr_db{0.0f};             ///< Signal-to-Quantization-Noise-Ratio (dB)
    float enob{0.0f};                ///< Effective Number of Bits: (SQNR - 1.76) / 6.02
    std::size_t sample_count{0};
};

/**
 * @brief Fixed-point FIR filter operating strictly on Q15 samples and coefficients.
 *
 * Implements saturating MAC arithmetic with a 64-bit accumulator, round-to-nearest,
 * and zero dynamic memory allocations in the processing loop.
 * Direct equivalent of ARM CMSIS-DSP arm_fir_q15.
 */
class FixedPointFirQ15 {
public:
    static constexpr std::size_t MAX_TAPS = 128;

    FixedPointFirQ15() = default;
    explicit FixedPointFirQ15(const std::vector<float>& float_coeffs);
    FixedPointFirQ15(const q15_t* q15_coeffs, std::size_t num_taps);

    /**
     * @brief Quantizes and sets float coefficients into Q15.
     */
    bool set_coefficients(const float* float_coeffs, std::size_t num_taps);
    bool set_coefficients(const q15_t* q15_coeffs, std::size_t num_taps);

    /**
     * @brief Processes a single Q15 sample.
     */
    q15_t process_sample(q15_t input) noexcept;

    /**
     * @brief Processes a block of Q15 samples.
     */
    void process_block(const q15_t* in, q15_t* out, std::size_t count) noexcept;

    /**
     * @brief Resets the delay line to zero.
     */
    void reset() noexcept;

    std::size_t num_taps() const noexcept { return num_taps_; }
    const q15_t* coefficients() const noexcept { return coeffs_.data(); }

private:
    std::array<q15_t, MAX_TAPS> coeffs_{};
    std::array<q15_t, MAX_TAPS> state_{};
    std::size_t num_taps_{0};
    std::size_t state_idx_{0};
};

/**
 * @brief Fixed-point IIR Biquad in Q1.14 / Q15 format.
 * Uses 64-bit accumulator with post-shift to allow coefficient dynamic range [-2.0, 2.0).
 */
class FixedPointBiquadQ15 {
public:
    FixedPointBiquadQ15() = default;

    /**
     * @brief Initializes biquad from floating-point coefficients.
     * Note: Biquad denominator coefficients can range up to 2.0, so coefficients
     * are stored scaled by 2^14 (Q1.14 format) and accumulated into a 64-bit register.
     */
    bool set_coefficients(float b0, float b1, float b2, float a1, float a2);

    q15_t process_sample(q15_t input) noexcept;
    void process_block(const q15_t* in, q15_t* out, std::size_t count) noexcept;
    void reset() noexcept;

private:
    int16_t b0_{0}, b1_{0}, b2_{0};
    int16_t a1_{0}, a2_{0};
    int16_t x1_{0}, x2_{0}; ///< Input history
    int16_t y1_{0}, y2_{0}; ///< Output history
};

/**
 * @brief Compares floating point reference against fixed-point output.
 * Computes MSE, Peak Error, SQNR, and ENOB.
 */
DspErrorMetrics evaluate_quantization_error(const float* ref, const float* test, std::size_t count);

} // namespace dsp
