#include "IntelHex.hpp"

#include <fstream>
#include <sstream>
#include <stdexcept>

namespace ucan {

namespace {

int hexNibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    throw std::runtime_error("Ungültiges Intel-HEX-Zeichen");
}

} // namespace

std::vector<std::pair<uint32_t, std::vector<uint8_t>>>
parseIntelHex(const std::string& text) {
    std::vector<std::pair<uint32_t, std::vector<uint8_t>>> blocks;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        if (line.front() != ':')
            throw std::runtime_error("Ungültiges Intel-HEX-Format");
        if (line.size() % 2 != 0 || line.size() < 11)
            throw std::runtime_error("Ungültige Intel-HEX-Länge");
        auto byteAt = [&](size_t i) {
            return uint8_t((hexNibble(line[1 + 2 * i]) << 4) |
                           hexNibble(line[2 + 2 * i]));
        };
        const uint8_t count = byteAt(0);
        if (count == 0 || count > 255 || line.size() != 2 * (count + 5) + 1)
            throw std::runtime_error("Ungültige Intel-HEX-Prüfsumme oder Länge");
        uint32_t sum = 0;
        for (size_t i = 0; i <= count + 4; ++i) sum += byteAt(i);
        if (sum & 0xFF)
            throw std::runtime_error("Ungültige Intel-HEX-Prüfsumme");
        const uint16_t address = uint16_t(byteAt(1)) << 8 | byteAt(2);
        const uint8_t type = byteAt(3);
        if (type == 1) { // EOF
            if (blocks.empty())
                throw std::runtime_error("Firmware ohne gültige Daten");
            return blocks;
        }
        if (type != 0 || address + count > 0x4000)
            throw std::runtime_error("Firmware enthält Daten außerhalb des FX2LP-RAM");
        std::vector<uint8_t> data(count);
        for (uint8_t i = 0; i < count; ++i) data[i] = byteAt(4 + i);
        if (!blocks.empty() &&
            blocks.back().first + blocks.back().second.size() == address &&
            blocks.back().second.size() + data.size() <= 1024) {
            auto& prev = blocks.back().second;
            prev.insert(prev.end(), data.begin(), data.end());
            continue;
        }
        blocks.emplace_back(address, std::move(data));
    }
    throw std::runtime_error("Firmware ohne gültigen Intel-HEX-Abschluss");
}

std::vector<std::pair<uint32_t, std::vector<uint8_t>>>
loadFirmware(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file)
        throw std::runtime_error("Firmware-Datei nicht lesbar: " + path);
    std::ostringstream buffer;
    buffer << file.rdbuf();
    return parseIntelHex(buffer.str());
}

} // namespace ucan
