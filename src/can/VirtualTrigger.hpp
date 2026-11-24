#pragma once

#include "CanTypes.hpp"

#include <cstdint>
#include <span>
#include <vector>

namespace ucan::can {

/// Virtual trigger — port of classic_can.py ErrorDetector plus a
/// configurable holdoff. Scans contiguous differential-voltage segments
/// and reports long dominant runs and (in error mode) frames with a
/// missing ACK. Maintains its own overlap tail like LiveDecoder.
class VirtualTrigger {
public:
    enum class Mode : uint8_t { Error, Dominant };

    VirtualTrigger(double rateHz, uint32_t bitrate, float thresholdV,
                   Mode mode = Mode::Error, double holdoffUs = 0.0);

    void setThreshold(float v) { threshold_ = v; }
    void setBitrate(uint32_t v) { bitrate_ = v; samplesPerBit_ = rateHz_ / v; }
    void setMode(Mode m) { mode_ = m; }
    void setHoldoffUs(double us) {
        holdoffSamples_ = static_cast<int64_t>(us * rateHz_ / 1e6);
    }
    void reset() {
        tail_.clear();
        evaluatedUpTo_ = -1;
        lastAccepted_ = -1;
        lastCandidatePos_ = -1;
    }

    /// Absolute sample position of the newest trigger candidate seen by
    /// scan() — unfiltered (before dedup/holdoff). -1 = none yet.
    [[nodiscard]] int64_t lastCandidatePos() const noexcept {
        return lastCandidatePos_;
    }

    /// Absolute sample position of the newest candidate that passed the
    /// holdoff window — the scope view anchor. Holdoff gates ONLY this
    /// position: the TriggerEvent stream itself is unfiltered.
    [[nodiscard]] int64_t lastAcceptedPos() const noexcept {
        return lastAccepted_;
    }

    // Debug counters (analyzer thread only).
    uint64_t dbgScans = 0, dbgCandidates = 0, dbgSeenSkip = 0,
             dbgHoldoffSkip = 0, dbgAccepted = 0, dbgEvents = 0;
    int64_t dbgEvaluatedUpTo = -1, dbgLastAccepted = -1;

    /// Analyze one contiguous segment; origin = absolute sample index of
    /// segment[0]. Returns newly accepted trigger events.
    std::vector<TriggerEvent> scan(std::span<const float> differential,
                                   int64_t origin);

private:
    double rateHz_;
    uint32_t bitrate_;
    float threshold_;
    Mode mode_;
    double samplesPerBit_;
    int64_t minDominantSamples_;
    int64_t holdoffSamples_ = 0;
    std::vector<float> tail_;
    int64_t evaluatedUpTo_ = -1;
    int64_t lastAccepted_ = -1;
    int64_t lastCandidatePos_ = -1;
};

} // namespace ucan::can
