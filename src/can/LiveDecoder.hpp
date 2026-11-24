#pragma once

#include "CanTypes.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <optional>
#include <vector>

namespace ucan::can {

/// Rolling-window live decoder — port of live_can.py. Feed contiguous
/// differential-voltage segments; decoded frames are reported through
/// onFrame. USB gaps must be signalled with reset() so stale tail
/// samples are not joined across the gap.
class LiveDecoder {
public:
    /// rateHz: scope rate, bitrate: CAN bitrate, thresholdV: decision
    /// threshold, samplePoint: bit phase for the reserve value (0..1).
    LiveDecoder(double rateHz, uint32_t bitrate, float thresholdV,
                float samplePoint = 0.7f);

    /// Analyze one contiguous segment. origin is the absolute sample
    /// index of segment[0] in valid-sample space.
    void analyze(std::span<const float> differential, int64_t origin);
    /// Drop the overlap tail (call on transport gaps).
    void reset() { tail_.clear(); }

    /// Access to the current analysis window (tail + last segment) so the
    /// virtual trigger can scan identical data.
    [[nodiscard]] std::span<const float> window() const noexcept {
        return window_;
    }
    [[nodiscard]] int64_t windowOrigin() const noexcept {
        return windowOrigin_;
    }

    void setThreshold(float v) { threshold_ = v; }
    void setBitrate(uint32_t v) {
        bitrate_ = v;
        samplesPerBit_ = rateHz_ / v;
        overlapSamples_ = static_cast<int64_t>(
            std::ceil(160.0 * samplesPerBit_));
    }

    std::function<void(const FrameEvent&)> onFrame;

    // Statistics (mirrors live_can.py summary counters).
    uint64_t framesTotal = 0, ackOk = 0, missingAck = 0, invalidTail = 0,
             longDominant = 0;
    float worstRecessiveMarginV = 1e9f, worstDominantMarginV = 1e9f;

private:
    std::pair<float, float> margins(std::span<const float> window,
                                    const CanFrame& frame) const;

    double rateHz_;
    uint32_t bitrate_;
    float threshold_;
    float samplePoint_;
    double samplesPerBit_;
    int64_t overlapSamples_;
    std::array<float, 10> phases_{};
    int phaseIndex_ = 0;
    float recessiveLimit_ = 0.5f, dominantLimit_ = 0.9f;
    std::vector<float> tail_;
    std::vector<float> window_;
    int64_t windowOrigin_ = 0;
    int64_t lastFrame_ = -1, lastRun_ = -1;
};

} // namespace ucan::can
