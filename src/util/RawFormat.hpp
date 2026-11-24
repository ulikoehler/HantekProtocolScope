#pragma once

#include <cstdint>
#include <cstring>

namespace ucan {

/// Header of the raw capture format written by `--log-raw` and replayed
/// by `ulican view`. Directly followed by `sampleCount` interleaved
/// uint8 ADC-code pairs (CH1 byte, CH2 byte per sample).
struct RawFileHeader {
    char magic[8];         // "ULICAN01"
    uint32_t version;      // 1
    uint32_t channels;     // 2
    double rateHz;         // samples per second per channel
    double voltsPerCode[2];  // V per ADC code, per channel
    double offsetCodes[2];   // ADC-code offset, per channel
    uint64_t sampleCount;  // valid sample pairs (patched on close;
                           // 0 = unknown, derive from file size)
};
static_assert(sizeof(RawFileHeader) == 64);

inline constexpr char kRawMagic[8] = {'U', 'L', 'I', 'C', 'A', 'N',
                                      '0', '1'};
inline constexpr uint32_t kRawVersion = 1;

[[nodiscard]] inline bool rawHeaderValid(const RawFileHeader& h) noexcept {
    return std::memcmp(h.magic, kRawMagic, sizeof(kRawMagic)) == 0 &&
           h.version == kRawVersion && h.channels == 2 &&
           h.rateHz > 0.0;
}

} // namespace ucan
