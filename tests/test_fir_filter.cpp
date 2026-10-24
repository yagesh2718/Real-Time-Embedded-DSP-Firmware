#include "test_framework.hpp"
#include "dsp/fir_filter.hpp"
#include <cmath>
#include <vector>

TEST_CASE(FirFilter_ImpulseResponseMatchesCoefficients) {
    // FIR impulse response h[n] must match {b_k} exactly
    std::vector<float> coeffs = {0.1f, 0.2f, 0.4f, 0.2f, 0.1f};
    dsp::FirFilter fir(coeffs);

    // Apply unit impulse delta[n]
    float impulse_in[5] = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    for (std::size_t i = 0; i < coeffs.size(); ++i) {
        float y = fir.process_sample(impulse_in[i]);
        ASSERT_NEAR(y, coeffs[i], 1e-6f);
    }

    // Subsequent outputs must be zero once impulse clears delay line
    float y_tail = fir.process_sample(0.0f);
    ASSERT_NEAR(y_tail, 0.0f, 1e-6f);
}

TEST_CASE(FirFilter_StepResponseConvergesToDcGain) {
    auto coeffs = dsp::FirFilter::design_lowpass(1000.0f, 10000.0f, 31);
    dsp::FirFilter fir(coeffs);

    // Feeding unit step u[n] = 1.0 for n >= 0
    float last_output = 0.0f;
    for (int n = 0; n < 60; ++n) {
        last_output = fir.process_sample(1.0f);
    }

    // Once steady-state is reached (n >= M-1), output should equal sum(b_k) = 1.0
    ASSERT_NEAR(last_output, 1.0f, 1e-4f);
}

TEST_CASE(FirFilter_LinearPhaseSymmetryAndGroupDelay) {
    std::size_t num_taps = 31;
    auto coeffs = dsp::FirFilter::design_lowpass(1000.0f, 10000.0f, num_taps);
    dsp::FirFilter fir(coeffs);

    // Verify coefficient symmetry: b[k] == b[M - 1 - k]
    for (std::size_t k = 0; k < num_taps / 2; ++k) {
        ASSERT_NEAR(coeffs[k], coeffs[num_taps - 1 - k], 1e-6f);
    }

    // Group delay tau = (M - 1) / 2 = 15.0 samples
    ASSERT_NEAR(fir.group_delay(), 15.0f, 1e-6f);
}

TEST_CASE(FirFilter_StopbandAttenuation) {
    // 31-tap FIR lowpass with 1 kHz cutoff at 10 kHz Fs
    auto coeffs = dsp::FirFilter::design_lowpass(1000.0f, 10000.0f, 31);
    dsp::FirFilter fir(coeffs);

    // Evaluate frequency response in passband (200 Hz) and stopband (4000 Hz)
    float omega_pass = 2.0f * 3.14159265f * (200.0f / 10000.0f);
    float omega_stop = 2.0f * 3.14159265f * (4000.0f / 10000.0f);

    auto h_pass = fir.frequency_response(omega_pass);
    auto h_stop = fir.frequency_response(omega_stop);

    float mag_pass_db = 20.0f * std::log10(std::abs(h_pass));
    float mag_stop_db = 20.0f * std::log10(std::abs(h_stop));

    // Passband should have < 1 dB attenuation
    ASSERT_NEAR(mag_pass_db, 0.0f, 1.0f);

    // Stopband should have > 20 dB attenuation
    ASSERT_TRUE(mag_stop_db < -20.0f);
}

TEST_CASE(FirFilter_StreamingVsBlockProcessingEquivalence) {
    auto coeffs = dsp::FirFilter::design_lowpass(1200.0f, 10000.0f, 21);
    dsp::FirFilter fir_stream(coeffs);
    dsp::FirFilter fir_block(coeffs);

    std::vector<float> input(64);
    for (std::size_t i = 0; i < input.size(); ++i) {
        input[i] = std::sin(0.1f * static_cast<float>(i));
    }

    std::vector<float> out_stream(input.size());
    for (std::size_t i = 0; i < input.size(); ++i) {
        out_stream[i] = fir_stream.process_sample(input[i]);
    }

    std::vector<float> out_block(input.size());
    fir_block.process_block(input.data(), out_block.data(), input.size());

    for (std::size_t i = 0; i < input.size(); ++i) {
        ASSERT_NEAR(out_stream[i], out_block[i], 1e-6f);
    }
}

TEST_CASE(FirFilter_ResetClearsDelayLine) {
    auto coeffs = dsp::FirFilter::design_lowpass(1000.0f, 10000.0f, 15);
    dsp::FirFilter fir(coeffs);

    // Feed non-zero data
    for (int i = 0; i < 20; ++i) {
        fir.process_sample(1.0f);
    }

    fir.reset();

    // Now feeding zero should yield immediate zero
    float y = fir.process_sample(0.0f);
    ASSERT_NEAR(y, 0.0f, 1e-7f);
}
