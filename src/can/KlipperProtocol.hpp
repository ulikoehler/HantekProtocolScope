#pragma once

#include <cstdint>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace ucan::can {

/// Klipper mcu-protocol data dictionary: msgid -> format string
/// (e.g. "queue_step oid=%c interval=%u count=%hu add=%hi").
/// Loaded from a JSON dict, plain or zlib-compressed (as transmitted
/// in identify_response chunks).
struct KlipperDict {
    std::map<int64_t, std::string> commands;
    std::map<int64_t, std::string> responses;
    std::map<int64_t, std::string> outputs;
    std::string version, buildVersions;

    bool load(const std::string& path, std::string* error);
    /// Format lookup across all maps.
    const std::string* format(int64_t msgid, bool* isCommand) const;
};

/// One decoded Klipper message block from a reassembled byte stream.
struct KlipperBlock {
    uint8_t seq = 0;
    bool crcOk = false;
    std::string text;  // decoded commands joined with "; "
};

/// Reassembles one direction's byte stream (CAN payloads appended) and
/// splits it into Klipper message blocks:
///   <len 5..64><seq><content><crc16 CCITT><0x7e>
/// CRC/length/sync violations resync by dropping bytes.
class KlipperStream {
public:
    explicit KlipperStream(const KlipperDict* dict = nullptr)
        : dict_(dict) {}
    void setDict(const KlipperDict* d) { dict_ = d; }

    /// Append CAN payload; returns blocks completed by this feed.
    std::vector<KlipperBlock> feed(std::span<const uint8_t> data);
    void reset() { buf_.clear(); }
    [[nodiscard]] size_t buffered() const { return buf_.size(); }

private:
    std::vector<uint8_t> buf_;
    const KlipperDict* dict_;
};

} // namespace ucan::can
