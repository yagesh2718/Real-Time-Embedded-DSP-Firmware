#pragma once

#include <cstddef>
#include <cstdint>
#include <array>
#include <algorithm>
#include <utility>

namespace dsp {

/**
 * @brief Overflow handling policies when the circular buffer is full.
 */
enum class OverflowPolicy {
    DropNewest,      ///< Reject incoming sample if buffer is full (returns false).
    OverwriteOldest  ///< Overwrite oldest unread sample, advancing tail index.
};

/**
 * @brief Fixed-capacity, zero-heap-allocation circular ring buffer.
 *
 * Designed specifically for real-time firmware pipelines (e.g. ADC DMA FIFO).
 * Operates without any dynamic memory allocations in the critical sample path.
 *
 * @tparam T Element type (e.g., float, int16_t, q15_t)
 * @tparam Capacity Maximum number of elements buffer can store.
 */
template <typename T, std::size_t Capacity>
class CircularBuffer {
    static_assert(Capacity > 0, "CircularBuffer capacity must be greater than zero.");

public:
    explicit CircularBuffer(OverflowPolicy policy = OverflowPolicy::DropNewest) noexcept
        : policy_(policy), head_(0), tail_(0), count_(0), overflow_count_(0) {}

    /**
     * @brief Writes a single item into the circular buffer.
     * @param item Value to push.
     * @return true if written successfully; false if dropped due to DropNewest policy.
     */
    bool push(const T& item) noexcept {
        if (is_full()) {
            overflow_count_++;
            if (policy_ == OverflowPolicy::DropNewest) {
                return false;
            }
            // Overwrite oldest: advance tail to discard the oldest item
            tail_ = (tail_ + 1) % Capacity;
            count_--;
        }

        buffer_[head_] = item;
        head_ = (head_ + 1) % Capacity;
        count_++;
        return true;
    }

    /**
     * @brief Reads and removes the oldest item from the buffer.
     * @param[out] out_item Reference to receive popped value.
     * @return true if an item was popped; false if buffer was empty.
     */
    bool pop(T& out_item) noexcept {
        if (is_empty()) {
            return false;
        }

        out_item = buffer_[tail_];
        tail_ = (tail_ + 1) % Capacity;
        count_--;
        return true;
    }

    /**
     * @brief Peeks at the item at a specific offset from the tail without removing it.
     * @param offset Offset from tail (0 = oldest available item).
     * @param[out] out_item Reference to receive peeked value.
     * @return true if valid offset; false if offset >= count().
     */
    bool peek(std::size_t offset, T& out_item) const noexcept {
        if (offset >= count_) {
            return false;
        }
        std::size_t index = (tail_ + offset) % Capacity;
        out_item = buffer_[index];
        return true;
    }

    /**
     * @brief Pushes a contiguous block of samples.
     * @param data Pointer to input data array.
     * @param length Number of elements to write.
     * @return Number of elements successfully written.
     */
    std::size_t write_block(const T* data, std::size_t length) noexcept {
        if (data == nullptr || length == 0) {
            return 0;
        }

        std::size_t written = 0;
        for (std::size_t i = 0; i < length; ++i) {
            if (push(data[i])) {
                written++;
            } else if (policy_ == OverflowPolicy::DropNewest) {
                // Buffer full and policy rejects additions
                break;
            }
        }
        return written;
    }

    /**
     * @brief Pops up to `length` samples into a contiguous destination array.
     * @param out_data Pointer to destination array.
     * @param length Desired number of samples to read.
     * @return Actual number of elements extracted.
     */
    std::size_t read_block(T* out_data, std::size_t length) noexcept {
        if (out_data == nullptr || length == 0) {
            return 0;
        }

        std::size_t read_count = 0;
        for (std::size_t i = 0; i < length; ++i) {
            if (!pop(out_data[i])) {
                break;
            }
            read_count++;
        }
        return read_count;
    }

    /**
     * @brief Clears the buffer contents.
     */
    void clear() noexcept {
        head_ = 0;
        tail_ = 0;
        count_ = 0;
    }

    /**
     * @brief Returns current number of items in buffer.
     */
    constexpr std::size_t size() const noexcept {
        return count_;
    }

    /**
     * @brief Returns total storage capacity.
     */
    constexpr std::size_t capacity() const noexcept {
        return Capacity;
    }

    /**
     * @brief Returns available free space.
     */
    constexpr std::size_t available() const noexcept {
        return Capacity - count_;
    }

    /**
     * @brief Returns true if buffer contains no samples.
     */
    constexpr bool is_empty() const noexcept {
        return count_ == 0;
    }

    /**
     * @brief Returns true if buffer is at maximum capacity.
     */
    constexpr bool is_full() const noexcept {
        return count_ == Capacity;
    }

    /**
     * @brief Returns total number of overflow events observed.
     */
    std::size_t overflow_count() const noexcept {
        return overflow_count_;
    }

    /**
     * @brief Resets the overflow counter.
     */
    void reset_overflow_count() noexcept {
        overflow_count_ = 0;
    }

    /**
     * @brief Configures the overflow handling policy.
     */
    void set_policy(OverflowPolicy policy) noexcept {
        policy_ = policy;
    }

    /**
     * @brief Gets the active overflow policy.
     */
    OverflowPolicy policy() const noexcept {
        return policy_;
    }

private:
    std::array<T, Capacity> buffer_{};
    OverflowPolicy policy_;
    std::size_t head_;
    std::size_t tail_;
    std::size_t count_;
    std::size_t overflow_count_;
};

} // namespace dsp
