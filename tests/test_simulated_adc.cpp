#include "test_framework.hpp"
#include "dsp/simulated_adc.hpp"
#include <vector>

TEST_CASE(SimulatedAdc_QuantizationRangeAndBitDepth) {
    dsp::SimulatedAdcConfig cfg;
    cfg.bit_depth = 12;
    cfg.v_ref = 3.3f;
    cfg.enable_quantization = true;
    cfg.waveform = dsp::WaveformType::Sine;

    dsp::SimulatedAdc adc(cfg);

    for (int i = 0; i < 200; ++i) {
        dsp::SimulatedAdcSample s = adc.acquire_sample();
        // 12-bit counts must strictly reside within [0, 4095]
        ASSERT_TRUE(s.raw_counts <= 4095);
        // Voltage must reside within [0.0, 3.3]
        ASSERT_TRUE(s.voltage >= 0.0f && s.voltage <= 3.3f);
        // Normalized input must be within [-1.0, 1.0]
        ASSERT_TRUE(s.normalized >= -1.0f && s.normalized <= 1.0f);
        // Must be explicitly labeled as synthetic simulated data
        ASSERT_TRUE(s.is_simulated_adc_data);
    }
}

TEST_CASE(SimulatedAdc_DeterministicReproducibility) {
    dsp::SimulatedAdcConfig cfg;
    cfg.random_seed = 98765;
    cfg.waveform = dsp::WaveformType::SineWithNoise;

    dsp::SimulatedAdc adc1(cfg);
    dsp::SimulatedAdc adc2(cfg);

    for (int i = 0; i < 100; ++i) {
        dsp::SimulatedAdcSample s1 = adc1.acquire_sample();
        dsp::SimulatedAdcSample s2 = adc2.acquire_sample();
        ASSERT_EQ(s1.raw_counts, s2.raw_counts);
        ASSERT_NEAR(s1.voltage, s2.voltage, 1e-6f);
    }
}

TEST_CASE(SimulatedAdc_CustomSequencePlayback) {
    dsp::SimulatedAdcConfig cfg;
    dsp::SimulatedAdc adc(cfg);

    std::vector<float> custom = {0.1f, 0.5f, -0.3f, 0.8f, -0.7f};
    adc.set_custom_sequence(custom);

    std::vector<float> acquired(custom.size());
    std::size_t count = adc.acquire_block(acquired.data(), custom.size());

    ASSERT_EQ(count, custom.size());
    for (std::size_t i = 0; i < custom.size(); ++i) {
        // Due to 12-bit quantization and scaling, should match within 0.02
        ASSERT_NEAR(acquired[i], custom[i], 0.02f);
    }
}
