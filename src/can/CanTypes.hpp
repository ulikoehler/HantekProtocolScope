#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace ucan::can {

/// Classic CAN 2.0A data frame decoded from the analog waveform.
/// Mirrors Frame in classic_can.py.
struct CanFrame {
    int64_t start = 0;  // dominant SOF edge, sample index in the window
    uint32_t identifier = 0;
    std::array<uint8_t, 8> data{};
    uint8_t dlc = 0;
    uint16_t crc = 0;
    int crcDelimiterBit = 0;
    int ackBit = 0;
    bool ack = false;                 // ACK slot dominant
    bool ackDelimiterOk = false;      // bit after ACK recessive
    bool eofOk = false;               // 7 recessive EOF bits
    bool dominantErrorAfterAck = false;
};

/// A decoded frame plus absolute position and signal margins.
struct FrameEvent {
    CanFrame frame;
    int64_t position = 0;      // absolute sample index of frame.start
    int64_t end = 0;           // absolute sample index past frame end
    double timeUs = 0;
    float recessiveMarginV = 0;
    float dominantMarginV = 0;
    /// Protocol-layer decode attached by the UI (e.g. Klipper message
    /// blocks completed by this frame's payload); empty if none.
    std::string decoded;
};

/// Trigger event produced by the virtual trigger (holdoff applied).
struct TriggerEvent {
    int64_t position = 0;      // absolute sample index
    double timeUs = 0;
    enum class Reason : uint8_t { LongDominant, MissingAck } reason =
        Reason::LongDominant;
    [[nodiscard]] const char* reasonText() const noexcept {
        return reason == Reason::LongDominant ? "long_dominant"
                                              : "missing_ack";
    }
};

/// CRC-15 over a bit sequence (MSB-first, polynomial 0x4599).
uint16_t crc15(std::span<const uint8_t> bits);

/// Decode one CRC-valid standard data frame from wire bits
/// (1 = recessive, 0 = dominant), honouring bit stuffing.
/// Returns nullopt on any protocol violation.
std::optional<CanFrame> decodeWireBits(std::span<const uint8_t> wire,
                                       int64_t start = 0,
                                       std::optional<uint32_t> identifier =
                                           std::nullopt);

/// Scan a differential-voltage window for complete frames.
/// afterIdle restricts candidates to frames preceded by >= 7 recessive
/// bits (bus idle), matching the Python live path.
std::vector<CanFrame> decodeCapture(std::span<const float> differential,
                                    double rateHz, uint32_t bitrate,
                                    float threshold, bool afterIdle = false,
                                    std::optional<uint32_t> identifier =
                                        std::nullopt);

/// Runs of `differential >= threshold` with length >= minimum samples.
/// includeEdges=false drops runs touching the window boundary.
std::vector<std::pair<int64_t, int64_t>>
longDominantRuns(std::span<const float> differential, float threshold,
                 int64_t minimum, bool includeEdges = false);

} // namespace ucan::can
