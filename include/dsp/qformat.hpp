#pragma once

#include <cstdint>
#include <cmath>
#include <algorithm>

namespace dsp {

/**
 * @brief Fixed-point data types matching ARM CMSIS-DSP conventions.
 */
using q15_t = int16_t; ///< Signed 16-bit Q0.15 format: 1 sign bit, 15 fractional bits
using q31_t = int32_t; ///< Signed 32-bit Q0.31 format: 1 sign bit, 31 fractional bits
using q63_t = int64_t; ///< 64-bit accumulator with guard bits for intermediate sums

namespace qformat {

// Constants for Q-format boundaries
constexpr q15_t Q15_MIN = -32768;
constexpr q15_t Q15_MAX = 32767;
constexpr float Q15_SCALE = 32768.0f;
constexpr float Q15_INV_SCALE = 1.0f / 32768.0f;

constexpr q31_t Q31_MIN = static_cast<q31_t>(-2147483647 - 1);
constexpr q31_t Q31_MAX = 2147483647;
constexpr double Q31_SCALE = 2147483648.0;
constexpr double Q31_INV_SCALE = 1.0 / 2147483648.0;

/**
 * @brief Saturating clamp to 16-bit Q15 range [-32768, 32767].
 * Mirrors ARM Cortex-M __SSAT(val, 16) hardware instruction.
 */
inline q15_t clip_q15(int32_t val) noexcept {
    if (val > Q15_MAX) return Q15_MAX;
    if (val < Q15_MIN) return Q15_MIN;
    return static_cast<q15_t>(val);
}

/**
 * @brief Saturating clamp to 32-bit Q31 range.
 * Mirrors ARM Cortex-M __SSAT(val, 32) instruction.
 */
inline q31_t clip_q31(int64_t val) noexcept {
    if (val > Q31_MAX) return Q31_MAX;
    if (val < Q31_MIN) return Q31_MIN;
    return static_cast<q31_t>(val);
}

/**
 * @brief Converts IEEE-754 single-precision float to Q15 with rounding and saturation.
 */
inline q15_t float_to_q15(float val) noexcept {
    if (std::isnan(val)) return 0;
    float scaled = val * Q15_SCALE;
    // Round to nearest integer (+0.5 for positive, -0.5 for negative)
    int32_t rounded = (scaled >= 0.0f) ? static_cast<int32_t>(scaled + 0.5f)
                                       : static_cast<int32_t>(scaled - 0.5f);
    return clip_q15(rounded);
}

/**
 * @brief Converts Q15 fixed-point to floating-point value.
 */
inline float q15_to_float(q15_t val) noexcept {
    return static_cast<float>(val) * Q15_INV_SCALE;
}

/**
 * @brief Converts float to Q31 with rounding and saturation.
 */
inline q31_t float_to_q31(float val) noexcept {
    if (std::isnan(val)) return 0;
    double scaled = static_cast<double>(val) * Q31_SCALE;
    int64_t rounded = (scaled >= 0.0) ? static_cast<int64_t>(scaled + 0.5)
                                      : static_cast<int64_t>(scaled - 0.5);
    return clip_q31(rounded);
}

/**
 * @brief Converts Q31 fixed-point to float.
 */
inline float q31_to_float(q31_t val) noexcept {
    return static_cast<float>(static_cast<double>(val) * Q31_INV_SCALE);
}

/**
 * @brief Saturated Q15 addition: a + b with overflow protection.
 */
inline q15_t q15_add(q15_t a, q15_t b) noexcept {
    return clip_q15(static_cast<int32_t>(a) + static_cast<int32_t>(b));
}

/**
 * @brief Saturated Q15 subtraction: a - b with underflow protection.
 */
inline q15_t q15_sub(q15_t a, q15_t b) noexcept {
    return clip_q15(static_cast<int32_t>(a) - static_cast<int32_t>(b));
}

/**
 * @brief Saturated Q15 multiplication with rounding:
 * Product is Q30, rounded and scaled back to Q15 with saturation.
 * Handles the special boundary case (-1.0) * (-1.0) -> +0x7FFF.
 */
inline q15_t q15_mul(q15_t a, q15_t b) noexcept {
    // (-32768) * (-32768) = 1073741824 (would overflow Q15)
    if (a == Q15_MIN && b == Q15_MIN) {
        return Q15_MAX;
    }
    int32_t product = static_cast<int32_t>(a) * static_cast<int32_t>(b);
    // Add bias of 0x4000 (0.5 in Q15 LSB) for unbiased round-to-nearest
    product += 0x4000;
    return clip_q15(product >> 15);
}

/**
 * @brief Saturated Q31 multiplication with rounding.
 */
inline q31_t q31_mul(q31_t a, q31_t b) noexcept {
    if (a == Q31_MIN && b == Q31_MIN) {
        return Q31_MAX;
    }
    int64_t product = static_cast<int64_t>(a) * static_cast<int64_t>(b);
    product += 0x40000000LL;
    return clip_q31(product >> 31);
}

} // namespace qformat
} // namespace dsp
