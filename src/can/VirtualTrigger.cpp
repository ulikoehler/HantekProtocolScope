#include "VirtualTrigger.hpp"

#include <algorithm>
#include <cmath>

namespace ucan::can {

VirtualTrigger::VirtualTrigger(double rateHz, uint32_t bitrate,
                               float thresholdV, Mode mode,
                               double holdoffUs)
    : rateHz_(rateHz), bitrate_(bitrate), threshold_(thresholdV),
      mode_(mode) {
    samplesPerBit_ = rateHz_ / bitrate_;
    minDominantSamples_ =
        static_cast<int64_t>(std::ceil(5.5 * samplesPerBit_));
    setHoldoffUs(holdoffUs);
}

std::vector<TriggerEvent> VirtualTrigger::scan(
    std::span<const float> differential, int64_t origin) {
    if (differential.empty()) return {};
    std::vector<float> window;
    window.reserve(tail_.size() + differential.size());
    window.insert(window.end(), tail_.begin(), tail_.end());
    window.insert(window.end(), differential.begin(), differential.end());
    const int64_t base = origin - static_cast<int64_t>(tail_.size());

    struct Candidate {
        int64_t position;
        TriggerEvent::Reason reason;
    };
    std::vector<Candidate> candidates;

    for (const auto& [a, b] :
         longDominantRuns(window, threshold_, minDominantSamples_, true)) {
        (void)b;
        candidates.push_back({base + a, TriggerEvent::Reason::LongDominant});
    }
    if (mode_ == Mode::Error) {
        for (const CanFrame& frame :
             decodeCapture(window, rateHz_, bitrate_, threshold_, true)) {
            if (frame.ack) continue;
            const int64_t ackSample = frame.start +
                static_cast<int64_t>(std::llround(frame.ackBit *
                                                  samplesPerBit_));
            candidates.push_back(
                {base + ackSample, TriggerEvent::Reason::MissingAck});
        }
    }

    ++dbgScans;
    dbgCandidates += candidates.size();
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& x, const Candidate& y) {
                  return x.position < y.position;
              });
    if (!candidates.empty())
        lastCandidatePos_ = candidates.back().position;

    std::vector<TriggerEvent> events;
    int64_t maxSeen = evaluatedUpTo_;
    for (const auto& candidate : candidates) {
        maxSeen = std::max(maxSeen, candidate.position);
        if (candidate.position <= evaluatedUpTo_) { ++dbgSeenSkip; continue; }
        // Every new candidate becomes an event — the trigger stream
        // (list/JSONL/markers) runs continuously, unfiltered.
        TriggerEvent event;
        event.position = candidate.position;
        event.timeUs = candidate.position * 1e6 / rateHz_;
        event.reason = candidate.reason;
        events.push_back(event);
        // Holdoff gates only the scope view anchor.
        if (lastAccepted_ < 0 ||
            candidate.position - lastAccepted_ >= holdoffSamples_) {
            lastAccepted_ = candidate.position;
            ++dbgAccepted;
        } else {
            ++dbgHoldoffSkip;
        }
    }
    evaluatedUpTo_ = maxSeen;
    dbgEvaluatedUpTo = evaluatedUpTo_;
    dbgLastAccepted = lastAccepted_;
    dbgEvents += events.size();

    // Keep an overlap tail so frames/runs crossing segment borders decode.
    const size_t keep = static_cast<size_t>(std::min<int64_t>(
        static_cast<int64_t>(std::ceil(160.0 * samplesPerBit_)),
        static_cast<int64_t>(window.size())));
    tail_.assign(window.end() - keep, window.end());
    return events;
}

} // namespace ucan::can
