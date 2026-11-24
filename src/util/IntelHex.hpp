#pragma once

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace ucan {

/// Parse Intel-HEX firmware into contiguous RAM blocks (max 1024 bytes
/// each), mirroring firmware_blocks() in the Python implementation.
/// Throws std::runtime_error on malformed input.
std::vector<std::pair<uint32_t, std::vector<uint8_t>>>
parseIntelHex(const std::string& text);

/// Load and parse a firmware file.
std::vector<std::pair<uint32_t, std::vector<uint8_t>>>
loadFirmware(const std::string& path);

} // namespace ucan
