#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace ucan {

/// Chunked rolling buffer of float samples in "valid sample" space.
/// Gaps in the USB stream are time-compressed (documented limitation).
class DisplayBuffer {
public:
    explicit DisplayBuffer(size_t capacitySamples)
        : capacity_(capacitySamples) {}

    /// Append n samples whose first sample has absolute index firstSample.
    void append(const float* data, size_t n, int64_t firstSample) {
        if (!n) return;
        if (chunks_.empty()) origin_ = firstSample;
        chunks_.push_back({firstSample, {data, data + n}});
        size_ += n;
        head_ = firstSample + static_cast<int64_t>(n);
        while (size_ > capacity_ && chunks_.size() > 1) {
            const auto& front = chunks_.front();
            origin_ = front.start + static_cast<int64_t>(front.values.size());
            size_ -= front.values.size();
            chunks_.pop_front();
        }
        if (size_ > capacity_) {
            // Single oversized chunk: drop its head.
            auto& front = chunks_.front();
            const size_t excess = size_ - capacity_;
            front.values.erase(front.values.begin(),
                               front.values.begin() + excess);
            front.start += static_cast<int64_t>(excess);
            origin_ = front.start;
            size_ = front.values.size();
        }
    }

    /// Copy samples with absolute indices in [first, first+count) into
    /// out, oldest first. The range is clamped to [origin(), head());
    /// out[0] then corresponds to max(first, origin()). Gaps between
    /// chunks are time-compressed (same limitation as extractLast).
    size_t extractRange(int64_t first, size_t count,
                        std::vector<float>& out) const {
        out.clear();
        if (!count || chunks_.empty()) return 0;
        const int64_t last = first + static_cast<int64_t>(count);
        first = std::max(first, origin_);
        const int64_t clampedLast = std::min(last, head_);
        if (clampedLast <= first) return 0;
        for (const auto& chunk : chunks_) {
            const int64_t cEnd =
                chunk.start + static_cast<int64_t>(chunk.values.size());
            const int64_t oStart = std::max(first, chunk.start);
            const int64_t oEnd = std::min(clampedLast, cEnd);
            if (oEnd <= oStart) continue;
            const size_t offset = static_cast<size_t>(oStart - chunk.start);
            out.insert(out.end(),
                       chunk.values.begin() + offset,
                       chunk.values.begin() + offset + (oEnd - oStart));
        }
        return out.size();
    }

    /// Copy the last n samples (or fewer) into out, oldest first.
    /// firstIndex receives the absolute index of out[0].
    size_t extractLast(size_t n, std::vector<float>& out,
                       int64_t& firstIndex) const {
        out.clear();
        firstIndex = head_;
        if (chunks_.empty()) return 0;
        const size_t take = std::min(n, size_);
        // Walk newest chunks first, collecting (chunk, fromOffset) parts.
        struct Part { const Chunk* chunk; size_t offset, count; };
        std::vector<Part> parts;
        size_t remaining = take;
        for (auto it = chunks_.rbegin();
             it != chunks_.rend() && remaining > 0; ++it) {
            const size_t count = std::min(it->values.size(), remaining);
            const size_t offset = it->values.size() - count;
            parts.push_back({&*it, offset, count});
            remaining -= count;
        }
        out.reserve(take);
        for (auto it = parts.rbegin(); it != parts.rend(); ++it) {
            const auto& [chunk, offset, count] = *it;
            out.insert(out.end(), chunk->values.begin() + offset,
                       chunk->values.begin() + offset + count);
        }
        firstIndex = head_ - static_cast<int64_t>(out.size());
        return out.size();
    }

    [[nodiscard]] int64_t origin() const noexcept { return origin_; }
    [[nodiscard]] int64_t head() const noexcept { return head_; }
    [[nodiscard]] size_t size() const noexcept { return size_; }

private:
    struct Chunk {
        int64_t start = 0;
        std::vector<float> values;
    };

    size_t capacity_;
    std::deque<Chunk> chunks_;
    size_t size_ = 0;
    int64_t origin_ = 0;
    int64_t head_ = 0;
};

} // namespace ucan
