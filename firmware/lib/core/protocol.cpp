#include "protocol.h"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace cyc {

uint8_t checksum(const char* body, size_t len) {
    uint8_t c = 0;
    for (size_t i = 0; i < len; ++i) c ^= static_cast<uint8_t>(body[i]);
    return c;
}

size_t makeFrame(char* out, size_t cap, const char* body) {
    size_t len = std::strlen(body);
    // $ + body + * + 2 hex + \n + \0
    if (len + 6 > cap) return 0;
    int w = std::snprintf(out, cap, "$%s*%02X\n", body, checksum(body, len));
    return (w > 0 && static_cast<size_t>(w) < cap) ? static_cast<size_t>(w) : 0;
}

static int hexVal(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

ParseResult parseLine(const char* line, Message& out, bool allowNoChecksum) {
    out.n = 0;
    out.hadChecksum = false;
    // пропустить пробелы и \r
    while (*line == ' ' || *line == '\r' || *line == '\t') ++line;
    size_t len = std::strlen(line);
    while (len > 0 && (line[len - 1] == '\r' || line[len - 1] == ' ' || line[len - 1] == '\n')) --len;
    if (len == 0) return ParseResult::Empty;
    if (line[0] != '$') return ParseResult::NoDollar;
    if (len >= kMaxLine) return ParseResult::TooLong;

    const char* body = line + 1;
    size_t bodyLen = len - 1;
    const char* star = static_cast<const char*>(std::memchr(body, '*', bodyLen));
    if (star) {
        size_t bl = static_cast<size_t>(star - body);
        if (bodyLen - bl != 3) return ParseResult::BadChecksum;
        int h = hexVal(star[1]), l = hexVal(star[2]);
        if (h < 0 || l < 0) return ParseResult::BadChecksum;
        if (checksum(body, bl) != static_cast<uint8_t>(h * 16 + l)) return ParseResult::BadChecksum;
        bodyLen = bl;
        out.hadChecksum = true;
    } else if (!allowNoChecksum) {
        return ParseResult::BadChecksum;
    }
    std::memcpy(out.buf, body, bodyLen);
    out.buf[bodyLen] = '\0';

    char* p = out.buf;
    out.f[out.n++] = p;
    for (; *p; ++p) {
        if (*p == ',') {
            *p = '\0';
            if (out.n >= kMaxFields) return ParseResult::TooManyFields;
            out.f[out.n++] = p + 1;
        }
    }
    return ParseResult::Ok;
}

bool LineReader::push(char c) {
    if (c == '\n') {
        buf_[len_] = '\0';
        bool ok = !overflow_ && len_ > 0;
        len_ = 0;
        overflow_ = false;
        return ok;
    }
    if (c == '$') len_ = 0, overflow_ = false;  // начало нового кадра — сбрасываем мусор
    if (len_ < kMaxLine) {
        buf_[len_++] = c;
    } else {
        overflow_ = true;
    }
    return false;
}

bool toFloat(const char* s, float& v) {
    if (!s || !*s) return false;
    char* end = nullptr;
    errno = 0;
    double d = std::strtod(s, &end);
    if (errno != 0 || end == s || *end != '\0') return false;
    v = static_cast<float>(d);
    return v == v;  // не NaN
}

bool toInt(const char* s, long& v) {
    if (!s || !*s) return false;
    char* end = nullptr;
    errno = 0;
    long d = std::strtol(s, &end, 10);
    if (errno != 0 || end == s || *end != '\0') return false;
    v = d;
    return true;
}

}  // namespace cyc
