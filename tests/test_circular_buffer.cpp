#include "test_framework.hpp"
#include "dsp/circular_buffer.hpp"

TEST_CASE(CircularBuffer_InitialEmptyState) {
    dsp::CircularBuffer<float, 8> buf;
    ASSERT_TRUE(buf.is_empty());
    ASSERT_FALSE(buf.is_full());
    ASSERT_EQ(buf.size(), 0u);
    ASSERT_EQ(buf.capacity(), 8u);
    ASSERT_EQ(buf.available(), 8u);

    float val = 0.0f;
    ASSERT_FALSE(buf.pop(val));
}

TEST_CASE(CircularBuffer_SinglePushPopFifo) {
    dsp::CircularBuffer<int, 4> buf;
    ASSERT_TRUE(buf.push(42));
    ASSERT_FALSE(buf.is_empty());
    ASSERT_EQ(buf.size(), 1u);
    ASSERT_EQ(buf.available(), 3u);

    int out_val = 0;
    ASSERT_TRUE(buf.pop(out_val));
    ASSERT_EQ(out_val, 42);
    ASSERT_TRUE(buf.is_empty());
}

TEST_CASE(CircularBuffer_FillToCapacity) {
    dsp::CircularBuffer<int, 4> buf;
    for (int i = 1; i <= 4; ++i) {
        ASSERT_TRUE(buf.push(i * 10));
    }
    ASSERT_TRUE(buf.is_full());
    ASSERT_EQ(buf.size(), 4u);
    ASSERT_EQ(buf.available(), 0u);

    // Verify FIFO read
    for (int i = 1; i <= 4; ++i) {
        int v = 0;
        ASSERT_TRUE(buf.pop(v));
        ASSERT_EQ(v, i * 10);
    }
    ASSERT_TRUE(buf.is_empty());
}

TEST_CASE(CircularBuffer_OverflowPolicyDropNewest) {
    dsp::CircularBuffer<int, 3> buf(dsp::OverflowPolicy::DropNewest);
    ASSERT_TRUE(buf.push(100));
    ASSERT_TRUE(buf.push(200));
    ASSERT_TRUE(buf.push(300));
    ASSERT_TRUE(buf.is_full());

    // 4th push should be rejected
    ASSERT_FALSE(buf.push(400));
    ASSERT_EQ(buf.overflow_count(), 1u);
    ASSERT_EQ(buf.size(), 3u);

    // Oldest values must remain intact
    int v = 0;
    ASSERT_TRUE(buf.pop(v));
    ASSERT_EQ(v, 100);
    ASSERT_TRUE(buf.pop(v));
    ASSERT_EQ(v, 200);
    ASSERT_TRUE(buf.pop(v));
    ASSERT_EQ(v, 300);
    ASSERT_TRUE(buf.is_empty());
}

TEST_CASE(CircularBuffer_OverflowPolicyOverwriteOldest) {
    dsp::CircularBuffer<int, 3> buf(dsp::OverflowPolicy::OverwriteOldest);
    ASSERT_TRUE(buf.push(10));
    ASSERT_TRUE(buf.push(20));
    ASSERT_TRUE(buf.push(30));
    ASSERT_TRUE(buf.is_full());

    // Pushing 40 should evict oldest (10) and accept 40
    ASSERT_TRUE(buf.push(40));
    ASSERT_EQ(buf.overflow_count(), 1u);
    ASSERT_EQ(buf.size(), 3u);

    // Contents must now be 20, 30, 40
    int v = 0;
    ASSERT_TRUE(buf.pop(v));
    ASSERT_EQ(v, 20);
    ASSERT_TRUE(buf.pop(v));
    ASSERT_EQ(v, 30);
    ASSERT_TRUE(buf.pop(v));
    ASSERT_EQ(v, 40);
    ASSERT_TRUE(buf.is_empty());
}

TEST_CASE(CircularBuffer_WraparoundMultipleCycles) {
    dsp::CircularBuffer<int, 5> buf;
    // Push and pop 50 items continuously across the wrap boundary
    for (int cycle = 0; cycle < 10; ++cycle) {
        for (int i = 0; i < 3; ++i) {
            ASSERT_TRUE(buf.push(cycle * 100 + i));
        }
        for (int i = 0; i < 3; ++i) {
            int v = 0;
            ASSERT_TRUE(buf.pop(v));
            ASSERT_EQ(v, cycle * 100 + i);
        }
    }
    ASSERT_TRUE(buf.is_empty());
}

TEST_CASE(CircularBuffer_BlockWriteAndRead) {
    dsp::CircularBuffer<float, 16> buf;
    float in_data[8] = {1.1f, 2.2f, 3.3f, 4.4f, 5.5f, 6.6f, 7.7f, 8.8f};
    std::size_t written = buf.write_block(in_data, 8);
    ASSERT_EQ(written, 8u);
    ASSERT_EQ(buf.size(), 8u);

    float out_data[8] = {0};
    std::size_t read_count = buf.read_block(out_data, 8);
    ASSERT_EQ(read_count, 8u);
    for (int i = 0; i < 8; ++i) {
        ASSERT_NEAR(out_data[i], in_data[i], 1e-6f);
    }
    ASSERT_TRUE(buf.is_empty());
}

TEST_CASE(CircularBuffer_PeekOffset) {
    dsp::CircularBuffer<int, 8> buf;
    buf.push(11);
    buf.push(22);
    buf.push(33);

    int peek_val = 0;
    ASSERT_TRUE(buf.peek(0, peek_val));
    ASSERT_EQ(peek_val, 11);

    ASSERT_TRUE(buf.peek(1, peek_val));
    ASSERT_EQ(peek_val, 22);

    ASSERT_TRUE(buf.peek(2, peek_val));
    ASSERT_EQ(peek_val, 33);

    ASSERT_FALSE(buf.peek(3, peek_val)); // Out of bounds
}

TEST_CASE(CircularBuffer_Clear) {
    dsp::CircularBuffer<int, 4> buf;
    buf.push(1);
    buf.push(2);
    buf.clear();
    ASSERT_TRUE(buf.is_empty());
    ASSERT_EQ(buf.size(), 0u);
    int v = 0;
    ASSERT_FALSE(buf.pop(v));
}
