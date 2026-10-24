#include "test_framework.hpp"
#include "dsp/qformat.hpp"
#include "dsp/fixed_point_dsp.hpp"
#include "dsp/fir_filter.hpp"
#include <cmath>
#include <vector>

TEST_CASE(FixedPoint_ConversionPrecision) {
    using namespace dsp::qformat;

    ASSERT_EQ(float_to_q15(0.0f), 0);
    ASSERT_EQ(float_to_q15(0.5f), 16384);
    ASSERT_EQ(float_to_q15(-0.5f), -16384);
    ASSERT_EQ(float_to_q15(-1.0f), -32768);

    // 1.0f cannot be represented exactly in Q15; must saturate to 32767 (0x7FFF)
    ASSERT_EQ(float_to_q15(1.0f), 32767);

    // Round-trip precision for 0.25f
    dsp::q15_t q_val = float_to_q15(0.25f);
    ASSERT_EQ(q_val, 8192);
    ASSERT_NEAR(q15_to_float(q_val), 0.25f, 1e-5f);
}

TEST_CASE(FixedPoint_SaturationClamping) {
    using namespace dsp::qformat;

    // Positive overflow saturation
    ASSERT_EQ(float_to_q15(1.5f), Q15_MAX);
    ASSERT_EQ(float_to_q15(100.0f), Q15_MAX);

    // Negative underflow saturation
    ASSERT_EQ(float_to_q15(-1.5f), Q15_MIN);
    ASSERT_EQ(float_to_q15(-50.0f), Q15_MIN);

    // Additive saturation
    dsp::q15_t a = 20000;
    dsp::q15_t b = 20000;
    ASSERT_EQ(q15_add(a, b), Q15_MAX);

    // Subtractive saturation
    dsp::q15_t c = -20000;
    dsp::q15_t d = 20000;
    ASSERT_EQ(q15_sub(c, d), Q15_MIN);
}

TEST_CASE(FixedPoint_MultiplicationAndBoundaryEdgeCase) {
    using namespace dsp::qformat;

    // 0.5 * 0.5 = 0.25
    dsp::q15_t half = 16384;
    dsp::q15_t quarter = q15_mul(half, half);
    ASSERT_EQ(quarter, 8192);

    // 0.5 * -0.5 = -0.25
    dsp::q15_t neg_half = -16384;
    ASSERT_EQ(q15_mul(half, neg_half), -8192);

    // Critical DSP edge case: (-1.0) * (-1.0) = +1.0
    // In Q15, +1.0 overflows and MUST saturate to Q15_MAX (0x7FFF = 32767)
    ASSERT_EQ(q15_mul(Q15_MIN, Q15_MIN), Q15_MAX);
}

TEST_CASE(FixedPoint_FirImpulseResponseMatchesQuantizedCoeffs) {
    std::vector<float> float_coeffs = {0.05f, 0.2f, 0.5f, 0.2f, 0.05f};
    dsp::FixedPointFirQ15 fir_q15(float_coeffs);

    // Apply Q15 unit impulse (32767 is approximately 1.0)
    dsp::q15_t impulse_in[5] = {32767, 0, 0, 0, 0};

    for (std::size_t i = 0; i < float_coeffs.size(); ++i) {
        dsp::q15_t y_q15 = fir_q15.process_sample(impulse_in[i]);
        float y_float = dsp::qformat::q15_to_float(y_q15);
        ASSERT_NEAR(y_float, float_coeffs[i], 0.001f);
    }
}

TEST_CASE(FixedPoint_ErrorMetricsAndEnobEvaluation) {
    // 31-tap FIR comparison against floating point reference on sinusoidal test signal
    auto float_coeffs = dsp::FirFilter::design_lowpass(1000.0f, 10000.0f, 31);
    dsp::FirFilter fir_flt(float_coeffs);
    dsp::FixedPointFirQ15 fir_q15(float_coeffs);

    constexpr std::size_t N = 400;
    std::vector<float> ref_out(N);
    std::vector<float> q15_out(N);

    for (std::size_t i = 0; i < N; ++i) {
        float x = 0.7f * std::sin(2.0f * 3.14159265f * 250.0f * static_cast<float>(i) / 10000.0f);
        ref_out[i] = fir_flt.process_sample(x);

        dsp::q15_t x_q15 = dsp::qformat::float_to_q15(x);
        dsp::q15_t y_q15 = fir_q15.process_sample(x_q15);
        q15_out[i] = dsp::qformat::q15_to_float(y_q15);
    }

    auto metrics = dsp::evaluate_quantization_error(ref_out.data(), q15_out.data(), N);

    // Peak error in Q15 should be well under 0.005 (approx 12-14 bits precision)
    ASSERT_TRUE(metrics.peak_absolute_error < 0.005f);
    // SQNR should exceed 60 dB
    ASSERT_TRUE(metrics.sqnr_db > 60.0f);
    // ENOB should exceed 10 bits
    ASSERT_TRUE(metrics.enob >= 10.0f);
}
