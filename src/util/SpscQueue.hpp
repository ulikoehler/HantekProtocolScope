#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <utility>

namespace ucan {

/// Single-producer / single-consumer ring buffer. T must be default
/// constructible and movable. push() fails when full, pop() fails when
/// empty; neither blocks.
template <typename T, std::size_t Capacity>
class SpscQueue {
    static_assert(Capacity > 1, "SpscQueue needs Capacity > 1");

public:
    bool push(T&& item) {
        const std::size_t h = head_.load(std::memory_order_relaxed);
        const std::size_t next = (h + 1) % Capacity;
        if (next == tail_.load(std::memory_order_acquire)) return false;
        slots_[h] = std::move(item);
        head_.store(next, std::memory_order_release);
        return true;
    }

    bool pop(T& out) {
        const std::size_t t = tail_.load(std::memory_order_relaxed);
        if (t == head_.load(std::memory_order_acquire)) return false;
        out = std::move(slots_[t]);
        tail_.store((t + 1) % Capacity, std::memory_order_release);
        return true;
    }

    [[nodiscard]] bool empty() const {
        return head_.load(std::memory_order_acquire) ==
               tail_.load(std::memory_order_acquire);
    }

private:
    std::array<T, Capacity> slots_{};
    std::atomic<std::size_t> head_{0};
    std::atomic<std::size_t> tail_{0};
};

} // namespace ucan
