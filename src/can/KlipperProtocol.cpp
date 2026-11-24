#include "KlipperProtocol.hpp"

#include <zlib.h>

#include <algorithm>
#include <fstream>
#include <format>

namespace ucan::can {

namespace {

// CRC-16-CCITT exactly as klippy/msgproto.py crc16_ccitt.
uint16_t crc16Ccitt(std::span<const uint8_t> s) {
    uint32_t crc = 0xffff;
    for (uint8_t b : s) {
        uint32_t data = b;
        data ^= crc & 0xff;
        data ^= (data & 0x0f) << 4;
        crc = ((data << 8) | (crc >> 8)) ^ (data >> 4) ^ (data << 3);
        crc &= 0xffff;
    }
    return crc;
}

// Klipper VLQ (msgproto.py PT_uint32.parse): MSB continues, second byte
// of first octet encodes sign via bits 5..6.
std::optional<int64_t> parseVlq(std::span<const uint8_t> s, size_t& pos) {
    if (pos >= s.size()) return std::nullopt;
    uint8_t c = s[pos++];
    int64_t v = c & 0x7f;
    if ((c & 0x60) == 0x60) v |= -0x20;
    while (c & 0x80) {
        if (pos >= s.size()) return std::nullopt;
        c = s[pos++];
        v = (v << 7) | (c & 0x7f);
    }
    return v;
}

enum class ParamType { U32, I32, U16, I16, Byte, Str };

// Split "name p=%u q=%c" into name + (pname, type) pairs.
struct MsgFormat {
    std::string name;
    std::vector<std::pair<std::string, ParamType>> params;
};

MsgFormat parseFormat(const std::string& fmt) {
    MsgFormat mf;
    size_t i = 0, n = fmt.size();
    auto token = [&](size_t& i) {
        size_t b = i;
        while (i < n && fmt[i] != ' ') ++i;
        return fmt.substr(b, i - b);
    };
    while (i < n && fmt[i] == ' ') ++i;
    mf.name = token(i);
    while (i < n) {
        while (i < n && fmt[i] == ' ') ++i;
        if (i >= n) break;
        std::string tok = token(i);
        auto eq = tok.find('=');
        if (eq == std::string::npos) continue;
        std::string pname = tok.substr(0, eq);
        std::string spec = tok.substr(eq + 1);
        ParamType t;
        if (spec == "%u") t = ParamType::U32;
        else if (spec == "%i") t = ParamType::I32;
        else if (spec == "%hu") t = ParamType::U16;
        else if (spec == "%hi") t = ParamType::I16;
        else if (spec == "%c") t = ParamType::Byte;
        else if (spec == "%s" || spec == "%.*s" || spec == "%*s")
            t = ParamType::Str;
        else continue;
        mf.params.emplace_back(std::move(pname), t);
    }
    return mf;
}

std::string hexBytes(std::span<const uint8_t> s) {
    std::string out;
    for (uint8_t b : s) out += std::format(" {:02X}", b);
    return out;
}

std::string decodeContent(std::span<const uint8_t> content,
                          const KlipperDict* dict) {
    std::string out;
    size_t pos = 0;
    while (pos < content.size()) {
        auto msgid = parseVlq(content, pos);
        if (!msgid) break;
        const std::string* fmt = nullptr;
        bool isCmd = false;
        if (dict) fmt = dict->format(*msgid, &isCmd);
        // Hard-coded defaults (msgproto.py DefaultMessages).
        static const std::string idResp =
            "identify_response offset=%u data=%.*s";
        static const std::string idCmd = "identify offset=%u count=%c";
        if (!fmt) {
            if (*msgid == 0) fmt = &idResp;
            else if (*msgid == 1) fmt = &idCmd;
        }
        if (!fmt) {
            out += std::format("{}#unknown id={:#x} ", out.empty() ? "" : "; ",
                               *msgid);
            break;  // cannot know trailing args of unknown msgid
        }
        auto mf = parseFormat(*fmt);
        out += std::format("{}{}", out.empty() ? "" : "; ", mf.name);
        for (auto& [pname, t] : mf.params) {
            if (t == ParamType::Str) {
                if (pos >= content.size()) return out;
                size_t l = content[pos++];
                size_t avail = std::min(l, content.size() - pos);
                std::string sv;
                for (size_t k = 0; k < avail; ++k) {
                    uint8_t ch = content[pos + k];
                    sv += (ch >= 32 && ch < 127) ? char(ch)
                                                 : char('.');
                }
                pos += avail;
                out += std::format(" {}=\"{}\"", pname, sv);
                continue;
            }
            auto v = parseVlq(content, pos);
            if (!v) return out;
            if (t == ParamType::U32 || t == ParamType::U16 ||
                t == ParamType::Byte)
                *v &= 0xffffffff;
            out += std::format(" {}={}", pname, *v);
        }
    }
    return out;
}

// ---- minimal JSON extraction for {"key": {"fmt": id, ...}} ---------

std::optional<std::map<int64_t, std::string>>
extractIdMap(const std::string& json, const std::string& key) {
    auto kp = json.find(std::format("\"{}\"", key));
    if (kp == std::string::npos) return std::nullopt;
    auto ob = json.find('{', kp);
    if (ob == std::string::npos) return std::nullopt;
    int depth = 0;
    size_t i = ob;
    std::map<int64_t, std::string> out;
    bool inStr = false;
    size_t objEnd = std::string::npos;
    for (; i < json.size(); ++i) {
        char c = json[i];
        if (inStr) {
            if (c == '\\') { ++i; continue; }
            if (c == '"') inStr = false;
            continue;
        }
        if (c == '"') { inStr = true; continue; }
        if (c == '{') ++depth;
        else if (c == '}' && --depth == 0) { objEnd = i; break; }
    }
    if (objEnd == std::string::npos) return std::nullopt;
    // Parse "fmt" : id pairs inside [ob, objEnd).
    i = ob + 1;
    while (i < objEnd) {
        while (i < objEnd && json[i] != '"') ++i;
        if (i >= objEnd) break;
        size_t sb = ++i;
        std::string key2;
        while (i < objEnd) {
            if (json[i] == '\\') { key2 += json[++i]; ++i; continue; }
            if (json[i] == '"') break;
            key2 += json[i++];
        }
        ++i;  // past closing quote
        while (i < objEnd && (json[i] == ' ' || json[i] == ':')) ++i;
        // value: integer
        size_t vb = i;
        while (i < objEnd && (isdigit(json[i]) || json[i] == '-')) ++i;
        if (i > vb)
            out[std::stoll(json.substr(vb, i - vb))] = key2;
        ++i;
    }
    return out;
}

std::string extractString(const std::string& json, const std::string& key) {
    auto kp = json.find(std::format("\"{}\"", key));
    if (kp == std::string::npos) return {};
    auto q1 = json.find('"', json.find(':', kp) + 1);
    if (q1 == std::string::npos) return {};
    auto q2 = json.find('"', q1 + 1);
    if (q2 == std::string::npos) return {};
    return json.substr(q1 + 1, q2 - q1 - 1);
}

} // namespace

const std::string* KlipperDict::format(int64_t msgid, bool* isCmd) const {
    if (auto it = commands.find(msgid); it != commands.end()) {
        if (isCmd) *isCmd = true;
        return &it->second;
    }
    if (auto it = responses.find(msgid); it != responses.end())
        return &it->second;
    if (auto it = outputs.find(msgid); it != outputs.end())
        return &it->second;
    return nullptr;
}

bool KlipperDict::load(const std::string& path, std::string* error) {
    std::ifstream f(path, std::ios::binary);
    if (!f) {
        if (error) *error = "cannot open " + path;
        return false;
    }
    std::string raw{std::istreambuf_iterator<char>(f), {}};
    // Try zlib first (identify chunks / .dict), else treat as plain JSON.
    std::string json;
    if (raw.size() > 2 && (raw[0] == 'x' || raw[0] == char(0x78))) {
        z_stream zs{};
        inflateInit(&zs);
        json.resize(raw.size() * 8 + 4096);
        zs.next_in = reinterpret_cast<Bytef*>(raw.data());
        zs.avail_in = static_cast<uInt>(raw.size());
        zs.next_out = reinterpret_cast<Bytef*>(json.data());
        zs.avail_out = static_cast<uInt>(json.size());
        int ret = inflate(&zs, Z_FINISH);
        inflateEnd(&zs);
        if (ret == Z_STREAM_END || ret == Z_OK)
            json.resize(zs.total_out);
        else
            json.clear();
    }
    if (json.empty()) json = std::move(raw);
    auto c = extractIdMap(json, "commands");
    auto r = extractIdMap(json, "responses");
    auto o = extractIdMap(json, "output");
    if (!c || !r) {
        if (error) *error = "no commands/responses map in " + path;
        return false;
    }
    commands = std::move(*c);
    responses = std::move(*r);
    if (o) outputs = std::move(*o);
    version = extractString(json, "version");
    buildVersions = extractString(json, "build_versions");
    return true;
}

std::vector<KlipperBlock> KlipperStream::feed(
    std::span<const uint8_t> data) {
    buf_.insert(buf_.end(), data.begin(), data.end());
    std::vector<KlipperBlock> out;
    while (buf_.size() >= 5) {
        uint8_t len = buf_[0];
        if (len < 5 || len > 64) {           // resync: drop one byte
            buf_.erase(buf_.begin());
            continue;
        }
        if (buf_.size() < len) break;        // wait for more data
        if (buf_[len - 1] != 0x7e) {
            buf_.erase(buf_.begin());
            continue;
        }
        uint16_t want = (uint16_t(buf_[len - 3]) << 8) | buf_[len - 2];
        uint16_t got = crc16Ccitt(
            std::span{buf_.data(), size_t(len - 3)});
        KlipperBlock b;
        b.seq = buf_[1] & 0x0f;
        b.crcOk = (want == got);
        if (b.crcOk)
            b.text = decodeContent(
                std::span{buf_.data() + 2, size_t(len - 5)}, dict_);
        else
            b.text = std::format("!crc seq={:02x}{}", b.seq,
                                 hexBytes(std::span{buf_.data() + 2,
                                                    size_t(len - 5)}));
        out.push_back(std::move(b));
        buf_.erase(buf_.begin(), buf_.begin() + len);
    }
    // Bound memory on permanent garbage.
    if (buf_.size() > 4096) buf_.erase(buf_.begin(), buf_.end() - 512);
    return out;
}

} // namespace ucan::can
