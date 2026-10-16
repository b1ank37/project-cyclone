// Текстовый протокол Pi ⇄ ESP32 в стиле NMEA:  $ТЕЛО*CS\n
// CS — XOR всех байтов ТЕЛА, две шестнадцатеричные цифры. Поля разделены запятыми.
// Такой формат можно читать глазами в мониторе порта и набирать руками (CS можно
// не указывать: строка без «*CS» принимается, если в конфиге разрешён «ручной» режим).
#pragma once
#include <cstddef>
#include <cstdint>

namespace cyc {

constexpr size_t kMaxLine = 120;
constexpr int kMaxFields = 16;

uint8_t checksum(const char* body, size_t len);

// Собрать кадр: body без $ и *. Возвращает длину или 0, если не влезло.
size_t makeFrame(char* out, size_t cap, const char* body);

struct Message {
    char buf[kMaxLine];
    const char* f[kMaxFields];
    int n = 0;
    bool hadChecksum = false;
    const char* type() const { return n > 0 ? f[0] : ""; }
};

enum class ParseResult { Ok, Empty, NoDollar, BadChecksum, TooLong, TooManyFields };

// Разобрать одну строку (без \n). allowNoChecksum — принимать «$PNG,1» без «*CS».
ParseResult parseLine(const char* line, Message& out, bool allowNoChecksum);

// Накопитель байтов в строки (для UART).
class LineReader {
public:
    // Возвращает true, когда собрана строка; она доступна в line().
    bool push(char c);
    const char* line() const { return buf_; }
    bool overflowed() const { return overflow_; }

private:
    char buf_[kMaxLine + 1] = {0};
    size_t len_ = 0;
    bool overflow_ = false;
};

bool toFloat(const char* s, float& v);
bool toInt(const char* s, long& v);

}  // namespace cyc
