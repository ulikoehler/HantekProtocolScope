#include "Loggers.hpp"

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace ucan {

namespace {

std::FILE* openWrite(const std::string& path) {
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f)
        throw std::runtime_error("Log-Datei öffnen: " + path + ": " +
                                 std::strerror(errno));
    return f;
}

} // namespace

// ---------------------------------------------------------------------
// CanLogger
// ---------------------------------------------------------------------

CanLogger::CanLogger(const std::string& path) {
    f_ = openWrite(path);
    std::setvbuf(f_, nullptr, _IOLBF, 0); // line buffered for tail -f
}

CanLogger::~CanLogger() {
    if (f_) std::fclose(f_);
}

void CanLogger::frame(const can::FrameEvent& ev) {
    if (!f_) return;
    const auto& fr = ev.frame;
    char data[17]{};
    for (int i = 0; i < fr.dlc; ++i)
        std::snprintf(data + 2 * i, 3, "%02x", fr.data[i]);
    std::fprintf(
        f_,
        "{\"type\":\"frame\",\"t_ms\":%.3f,\"id\":%u,\"id_hex\":\"0x%03X\""
        ",\"dlc\":%u,\"data\":\"%s\",\"crc\":%u,\"ack\":%s"
        ",\"ack_delim\":%s,\"eof\":%s,\"r_v\":%.4f,\"d_v\":%.4f}\n",
        ev.timeUs / 1000.0, fr.identifier, fr.identifier, fr.dlc, data,
        fr.crc, fr.ack ? "true" : "false",
        fr.ackDelimiterOk ? "true" : "false",
        fr.eofOk ? "true" : "false", ev.recessiveMarginV,
        ev.dominantMarginV);
    ++frames_;
}

void CanLogger::trigger(const can::TriggerEvent& ev) {
    if (!f_) return;
    std::fprintf(f_,
                 "{\"type\":\"trigger\",\"t_ms\":%.3f,\"pos\":%lld"
                 ",\"reason\":\"%s\"}\n",
                 ev.timeUs / 1000.0,
                 static_cast<long long>(ev.position), ev.reasonText());
    ++triggers_;
}

// ---------------------------------------------------------------------
// RawLogger
// ---------------------------------------------------------------------

RawLogger::RawLogger(const std::string& path, double rateHz,
                     const double* voltsPerCode,
                     const double* offsetCodes) {
    f_ = openWrite(path);
    std::memcpy(header_.magic, kRawMagic, sizeof(kRawMagic));
    header_.version = kRawVersion;
    header_.channels = 2;
    header_.rateHz = rateHz;
    for (int i = 0; i < 2; ++i) {
        header_.voltsPerCode[i] = voltsPerCode[i];
        header_.offsetCodes[i] = offsetCodes[i];
    }
    header_.sampleCount = 0; // patched in the destructor
    std::fwrite(&header_, 1, sizeof(header_), f_);
}

RawLogger::~RawLogger() {
    if (!f_) return;
    patchHeader();
    std::fclose(f_);
}

void RawLogger::patchHeader() {
    header_.sampleCount = samples_;
    if (std::fseek(f_, 0, SEEK_SET) != 0) return;
    std::fwrite(&header_, 1, sizeof(header_), f_);
}

void RawLogger::write(const uint8_t* data, size_t bytes) {
    if (!f_ || !bytes) return;
    std::fwrite(data, 1, bytes, f_);
    samples_ += bytes / 2;
}

} // namespace ucan
