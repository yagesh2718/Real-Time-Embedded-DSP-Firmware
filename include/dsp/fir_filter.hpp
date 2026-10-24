#pragma once

#include <cstddef>
#include <vector>
#include <array>
#include <complex>

namespace dsp {

/**
 * @brief Direct-form finite impulse response (FIR) filter.
 *
 * Implements:
 *   y[n] = sum_{k=0}^{M-1} b[k] * x[n-k]
 *
 * Designed with a bounded circular state buffer so no dynamic memory allocation
 * occurs during sample or block processing.
 */
class FirFilter {
public:
    static constexpr std::size_t MAX_TAPS = 128;

    FirFilter();
    explicit FirFilter(const std::vector<float>& coeffs);
    FirFilter(const float* coeffs, std::size_t num_taps);

    /**
     * @brief Processes a single input sample and produces the filtered output.
     * @param input Raw input sample x[n].
     * @return Filtered output sample y[n].
     */
    float process_sample(float input) noexcept;

    /**
     * @brief Processes an entire block of samples in place or out of place.
     * @param in Input block array.
     * @param out Output destination array (can be identical to in).
     * @param count Number of samples to process.
     */
    void process_block(const float* in, float* out, std::size_t count) noexcept;

    /**
     * @brief Resets the delay-line state buffer to zero without changing coefficients.
     */
    void reset() noexcept;

    /**
     * @brief Sets new filter coefficients.
     */
    bool set_coefficients(const std::vector<float>& coeffs);
    bool set_coefficients(const float* coeffs, std::size_t num_taps);

    /**
     * @brief Number of active taps (filter order + 1).
     */
    std::size_t num_taps() const noexcept { return num_taps_; }

    /**
     * @brief Read-only view of active coefficients.
     */
    const float* coefficients() const noexcept { return coeffs_.data(); }

    /**
     * @brief Algorithmic group delay for symmetric linear-phase FIR (samples).
     * tau = (num_taps - 1) / 2
     */
    float group_delay() const noexcept {
        return (num_taps_ > 0) ? (static_cast<float>(num_taps_ - 1) * 0.5f) : 0.0f;
    }

    /**
     * @brief Computes frequency response H(e^{j omega}) at normalized frequency omega in [0, pi].
     */
    std::complex<float> frequency_response(float normalized_rad_per_sample) const noexcept;

    /**
     * @brief Utility factory: Designs a windowed-sinc lowpass FIR filter with a Hamming window.
     * @param cutoff_hz Filter -6 dB cutoff frequency in Hz.
     * @param sample_rate_hz System sampling rate in Hz.
     * @param num_taps Filter length (recommended odd integer, e.g., 31).
     */
    static std::vector<float> design_lowpass(float cutoff_hz, float sample_rate_hz, std::size_t num_taps = 31);

    /**
     * @brief Default pre-calculated 31-tap low-pass filter (cutoff 1000 Hz at 10 kHz Fs).
     */
    static const std::vector<float>& default_coeffs_31_tap();

private:
    std::array<float, MAX_TAPS> coeffs_{};
    std::array<float, MAX_TAPS> state_{};
    std::size_t num_taps_{0};
    std::size_t state_idx_{0};
};

} // namespace dsp
