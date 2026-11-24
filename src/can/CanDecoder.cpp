#include "CanTypes.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ucan::can {

namespace {

uint32_t bitValue(std::span<const uint8_t> bits) {
    uint32_t value = 0;
    for (uint8_t bit : bits) value = (value << 1) | bit;
    return value;
}

} // namespace

uint16_t crc15(std::span<const uint8_t> bits) {
    uint16_t checksum = 0;
    for (uint8_t bit : bits) {
        const uint32_t feedback = ((checksum >> 14) ^ bit) & 1;
        checksum = static_cast<uint16_t>((checksum << 1) & 0x7FFF);
        if (feedback) checksum ^= 0x4599;
    }
    return checksum;
}

std::optional<CanFrame> decodeWireBits(std::span<const uint8_t> wire,
                                       int64_t start,
                                       std::optional<uint32_t> identifier) {
    std::vector<uint8_t> decoded;
    decoded.reserve(128);
    size_t position = 0;
    int last = -1;
    int run = 0;
    size_t required = 19;
    while (decoded.size() < required) {
        if (position >= wire.size()) return std::nullopt;
        const int bit = wire[position++];
        run = (bit == last) ? run + 1 : 1;
        decoded.push_back(static_cast<uint8_t>(bit));
        last = bit;
        if (run == 5) {
            if (position >= wire.size() || wire[position] == bit)
                return std::nullopt;
            last = wire[position];
            run = 1;
            ++position;
        }
        if (decoded.size() == 19) {
            if (decoded[0] != 0 || decoded[12] || decoded[13] || decoded[14])
                return std::nullopt;
            const uint32_t frameId = bitValue(
                std::span(decoded).subspan(1, 11));
            if (identifier && frameId != *identifier) return std::nullopt;
            const uint32_t dlc = bitValue(
                std::span(decoded).subspan(15, 4));
            if (dlc > 8) return std::nullopt;
            required = 19 + 8 * dlc + 15;
        }
    }
    if (wire.size() < position + 10 || wire[position] != 1)
        return std::nullopt;
    const uint16_t receivedCrc = static_cast<uint16_t>(bitValue(
        std::span(decoded).subspan(decoded.size() - 15, 15)));
    if (receivedCrc !=
        crc15(std::span(decoded).first(decoded.size() - 15)))
        return std::nullopt;

    CanFrame frame;
    frame.start = start;
    frame.identifier = bitValue(std::span(decoded).subspan(1, 11));
    frame.dlc = static_cast<uint8_t>(bitValue(
        std::span(decoded).subspan(15, 4)));
    frame.crc = receivedCrc;
    for (int i = 0; i < frame.dlc * 8; i += 8)
        frame.data[i / 8] = static_cast<uint8_t>(bitValue(
            std::span(decoded).subspan(19 + i, 8)));
    frame.crcDelimiterBit = static_cast<int>(position);
    frame.ackBit = static_cast<int>(position) + 1;
    frame.ack = wire[frame.ackBit] == 0;
    frame.ackDelimiterOk = wire[frame.ackBit + 1] == 1;
    frame.eofOk = true;
    for (int i = frame.ackBit + 2; i < frame.ackBit + 9; ++i)
        frame.eofOk &= wire[i] != 0;
    frame.dominantErrorAfterAck = true;
    for (int i = frame.ackBit + 1; i < frame.ackBit + 7; ++i)
        frame.dominantErrorAfterAck &= wire[i] == 0;
    return frame;
}

std::vector<CanFrame> decodeCapture(std::span<const float> differential,
                                    double rateHz, uint32_t bitrate,
                                    float threshold, bool afterIdle,
                                    std::optional<uint32_t> identifier) {
    const double samplesPerBit = rateHz / bitrate;
    std::array<int, 140> bitSamples{};
    for (int i = 0; i < 140; ++i)
        bitSamples[i] = static_cast<int>((i + 0.65) * samplesPerBit);
    const size_t n = differential.size();
    std::vector<uint8_t> dominant(n);
    for (size_t i = 0; i < n; ++i) dominant[i] = differential[i] >= threshold;
    std::vector<int64_t> changes;
    for (size_t i = 1; i < n; ++i)
        if (dominant[i] != dominant[i - 1]) changes.push_back(i);
    std::vector<int64_t> edges;
    for (int64_t c : changes)
        if (dominant[c]) edges.push_back(c);
    if (afterIdle) {
        std::vector<int64_t> filtered;
        int64_t previous = 0;
        for (int64_t c : changes) {
            if (dominant[c] && c - previous >= 7 * samplesPerBit)
                filtered.push_back(c);
            previous = c;
        }
        edges = std::move(filtered);
    }
    std::vector<CanFrame> frames;
    int64_t nextStart = 0;
    std::vector<uint8_t> wire;
    wire.reserve(160);
    for (int64_t start : edges) {
        if (start < nextStart) continue;
        wire.clear();
        for (int offset : bitSamples) {
            const int64_t index = start + offset;
            if (index >= static_cast<int64_t>(n)) break;
            wire.push_back(dominant[index] ? 0 : 1);
        }
        if (wire.size() < 44) break;
        auto frame = decodeWireBits(wire, start, identifier);
        if (frame) {
            nextStart = start +
                        static_cast<int64_t>((frame->ackBit + 2) *
                                             samplesPerBit);
            frames.push_back(*frame);
        }
    }
    return frames;
}

std::vector<std::pair<int64_t, int64_t>>
longDominantRuns(std::span<const float> differential, float threshold,
                 int64_t minimum, bool includeEdges) {
    const size_t n = differential.size();
    std::vector<std::pair<int64_t, int64_t>> runs;
    size_t i = 0;
    while (i < n) {
        if (differential[i] < threshold) { ++i; continue; }
        const size_t begin = i;
        while (i < n && differential[i] >= threshold) ++i;
        const size_t end = i;
        if (static_cast<int64_t>(end - begin) >= minimum &&
            (includeEdges || (begin > 0 && end < n)))
            runs.emplace_back(static_cast<int64_t>(begin),
                              static_cast<int64_t>(end));
    }
    return runs;
}

} // namespace ucan::can
