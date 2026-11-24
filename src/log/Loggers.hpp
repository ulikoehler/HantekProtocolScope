#pragma once

#include "../can/CanTypes.hpp"
#include "../util/RawFormat.hpp"

#include <cstdint>
#include <cstdio>
#include <string>

namespace ucan {

/// JSON-lines logger for decoded CAN frames and trigger events.
/// One object per line; safe to `tail -f` (line buffered).
/// Analyzer thread only.
class CanLogger {
public:
    /// Throws std::runtime_error when the file cannot be opened.
    explicit CanLogger(const std::string& path);
    ~CanLogger();

    CanLogger(const CanLogger&) = delete;
    CanLogger& operator=(const CanLogger&) = delete;

    void frame(const can::FrameEvent& ev);
    void trigger(const can::TriggerEvent& ev);

    [[nodiscard]] uint64_t frames() const noexcept { return frames_; }
    [[nodiscard]] uint64_t triggers() const noexcept { return triggers_; }

private:
    std::FILE* f_ = nullptr;
    uint64_t frames_ = 0;
    uint64_t triggers_ = 0;
};

/// Writes the raw ADC byte stream (interleaved CH1/CH2 uint8 pairs)
/// with a RawFileHeader, replayable via `ulican view`.
/// Analyzer thread only.
class RawLogger {
public:
    /// voltsPerCode/offsetCodes: per-channel calibration from the device
    /// EEPROM (or the source file header when re-logging a view).
    RawLogger(const std::string& path, double rateHz,
              const double* voltsPerCode, const double* offsetCodes);
    ~RawLogger();

    RawLogger(const RawLogger&) = delete;
    RawLogger& operator=(const RawLogger&) = delete;

    /// bytes must be a whole number of sample pairs (even).
    void write(const uint8_t* data, size_t bytes);

    [[nodiscard]] uint64_t samples() const noexcept { return samples_; }

private:
    void patchHeader();

    std::FILE* f_ = nullptr;
    RawFileHeader header_{};
    uint64_t samples_ = 0;
};

} // namespace ucan
