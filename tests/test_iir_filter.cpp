#include "test_framework.hpp"
#include "dsp/iir_biquad.hpp"
#include <cmath>
#include <vector>

TEST_CASE(IirFilter_StabilityVerification) {
    // 1. Stable lowpass design
    auto stable_lp = dsp::IirBiquad::design_lowpass(1000.0f, 10000.0f, 0.7071f);
    ASSERT_TRUE(stable_lp.is_stable());

    auto poles = stable_lp.poles();
    ASSERT_TRUE(std::abs(poles.first) < 1.0f);
    ASSERT_TRUE(std::abs(poles.second) < 1.0f);

    // 2. Unstable coefficient test: |a2| >= 1.0
    dsp::BiquadCoeffs unstable_a2;
    unstable_a2.b0 = 1.0f;
    unstable_a2.a1 = -0.5f;
    unstable_a2.a2 = 1.05f; // Unstable (|a2| > 1)
    ASSERT_FALSE(unstable_a2.is_stable());

    // 3. Unstable coefficient test: violating 1 + a1 + a2 > 0
    dsp::BiquadCoeffs unstable_sum;
    unstable_sum.b0 = 1.0f;
    unstable_sum.a1 = -1.8f;
    unstable_sum.a2 = 0.95f; // 1 + (-1.8) + 0.95 = 0.15 > 0, but 1 - (-1.8) + 0.95 = 3.75
    // Let's create one where 1 + a1 + a2 <= 0:
    dsp::BiquadCoeffs unstable_root;
    unstable_root.b0 = 1.0f;
    unstable_root.a1 = -2.2f;
    unstable_root.a2 = 0.8f; // 1 - 2.2 + 0.8 = -0.4 < 0 -> Unstable
    ASSERT_FALSE(unstable_root.is_stable());
}

TEST_CASE(IirFilter_ImpulseResponseDecay) {
    auto coeffs = dsp::IirBiquad::design_lowpass(1000.0f, 10000.0f, 0.7071f);
    dsp::IirBiquad biquad(coeffs);

    // Feed unit impulse
    float y0 = biquad.process_sample(1.0f);
    ASSERT_TRUE(std::abs(y0) > 0.0f);

    // After impulse, output must decay towards zero without diverging
    float last_val = 0.0f;
    for (int i = 0; i < 100; ++i) {
        last_val = biquad.process_sample(0.0f);
    }

    ASSERT_NEAR(last_val, 0.0f, 1e-4f);
}

TEST_CASE(IirFilter_StepResponseConvergesToUnity) {
    auto coeffs = dsp::IirBiquad::design_lowpass(1000.0f, 10000.0f, 0.7071f);
    dsp::IirBiquad biquad(coeffs);

    float y = 0.0f;
    for (int n = 0; n < 200; ++n) {
        y = biquad.process_sample(1.0f);
    }

    // DC gain of lowpass should be 1.0
    ASSERT_NEAR(y, 1.0f, 1e-4f);
}

TEST_CASE(IirFilter_NotchFilterRejection) {
    // Notch filter at 1000 Hz, Fs = 10000 Hz
    auto notch_coeffs = dsp::IirBiquad::design_notch(1000.0f, 10000.0f, 10.0f);
    ASSERT_TRUE(notch_coeffs.is_stable());

    // Normalized rad per sample at notch frequency
    float omega_notch = 2.0f * 3.14159265f * (1000.0f / 10000.0f);
    auto h_notch = notch_coeffs.frequency_response(omega_notch);
    float notch_atten_db = 20.0f * std::log10(std::abs(h_notch) + 1e-12f);

    // Notch must have deep attenuation (< -40 dB)
    ASSERT_TRUE(notch_atten_db < -40.0f);
}

TEST_CASE(IirFilter_Cascade4thOrderButterworth) {
    auto cascade = dsp::IirCascade::design_butterworth_4th_lowpass(1000.0f, 10000.0f);
    ASSERT_EQ(cascade.num_stages(), 2u);

    // Test DC response
    float y = 0.0f;
    for (int n = 0; n < 300; ++n) {
        y = cascade.process_sample(1.0f);
    }
    ASSERT_NEAR(y, 1.0f, 1e-4f);

    // Reset test
    cascade.reset();
    float y_after_reset = cascade.process_sample(0.0f);
    ASSERT_NEAR(y_after_reset, 0.0f, 1e-7f);
}
