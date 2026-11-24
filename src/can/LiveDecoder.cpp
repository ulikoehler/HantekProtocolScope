#include "LiveDecoder.hpp"

#include <algorithm>
#include <cmath>

namespace ucan::can {

LiveDecoder::LiveDecoder(double rateHz, uint32_t bitrate, float thresholdV,
                         float samplePoint)
    : rateHz_(rateHz), bitrate_(bitrate), threshold_(thresholdV),
      samplePoint_(samplePoint) {
    samplesPerBit_ = rateHz_ / bitrate_;
    overlapSamples_ = static_cast<int64_t>(
        std::ceil(160.0 * samplesPerBit_));
    for (int i = 0; i < 10; ++i) phases_[i] = i / 10.0f;
    phaseIndex_ = 0;
    for (int i = 1; i < 10; ++i)
        if (std::abs(phases_[i] - samplePoint_) <
            std::abs(phases_[phaseIndex_] - samplePoint_))
            phaseIndex_ = i;
}

void LiveDecoder::analyze(std::span<const float> differential,
                          int64_t origin) {
    if (differential.empty()) return;
    window_.assign(tail_.begin(), tail_.end());
    window_.insert(window_.end(), differential.begin(), differential.end());
    const int64_t base = origin - static_cast<int64_t>(tail_.size());
    windowOrigin_ = base;

    const int64_t minRun =
        static_cast<int64_t>(std::ceil(5.5 * samplesPerBit_));
    for (const auto& [a, b] :
         longDominantRuns(window_, threshold_, minRun, false)) {
        const int64_t pos = base + a;
        if (pos > lastRun_) {
            ++longDominant;
            lastRun_ = pos;
        }
    }

    for (const CanFrame& frame :
         decodeCapture(window_, rateHz_, bitrate_, threshold_, true)) {
        const int64_t position = base + frame.start;
        if (position <= lastFrame_) continue;
        lastFrame_ = position;
        ++framesTotal;
        ackOk += frame.ack && frame.ackDelimiterOk && frame.eofOk;
        missingAck += !frame.ack;
        invalidTail += !(frame.ackDelimiterOk && frame.eofOk);
        auto [recessive, dominant] = margins(window_, frame);
        worstRecessiveMarginV =
            std::min(worstRecessiveMarginV, recessive);
        worstDominantMarginV =
            std::min(worstDominantMarginV, dominant);
        if (onFrame) {
            FrameEvent event;
            event.frame = frame;
            event.position = position;
            event.timeUs = position * 1e6 / rateHz_;
            event.recessiveMarginV = recessive;
            event.dominantMarginV = dominant;
            onFrame(event);
        }
    }

    const size_t keep = std::min<size_t>(overlapSamples_, window_.size());
    tail_.assign(window_.end() - keep, window_.end());
}

std::pair<float, float> LiveDecoder::margins(
    std::span<const float> window, const CanFrame& frame) const {
    const int nBits = frame.crcDelimiterBit;
    const size_t n = window.size();
    std::array<float, 10> phaseRecessiveMax{}, phaseDominantMin{};
    phaseRecessiveMax.fill(-1e9f);
    phaseDominantMin.fill(1e9f);
    for (int bit = 0; bit < nBits; ++bit) {
        const int64_t refIndex = frame.start +
            static_cast<int64_t>((bit + 0.65) * samplesPerBit_);
        const bool isDominant =
            refIndex >= 0 && refIndex < static_cast<int64_t>(n) &&
            window[refIndex] >= threshold_;
        for (int p = 0; p < 10; ++p) {
            const int64_t index = frame.start +
                static_cast<int64_t>((bit + phases_[p]) * samplesPerBit_);
            if (index < 0 || index >= static_cast<int64_t>(n)) continue;
            const float v = window[index];
            if (isDominant)
                phaseDominantMin[p] = std::min(phaseDominantMin[p], v);
            else
                phaseRecessiveMax[p] = std::max(phaseRecessiveMax[p], v);
        }
    }
    return {recessiveLimit_ - phaseRecessiveMax[phaseIndex_],
            phaseDominantMin[phaseIndex_] - dominantLimit_};
}

} // namespace ucan::can
