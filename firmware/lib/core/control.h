// Небольшие «кирпичики» управления: ПИ-регулятор, медианный фильтр, утилиты.
#pragma once
#include <algorithm>
#include <cmath>

namespace cyc {

inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

// Угол в диапазон [-180, 180)
inline float wrapDeg(float a) {
    a = std::fmod(a + 180.0f, 360.0f);
    if (a < 0) a += 360.0f;
    return a - 180.0f;
}

// ПИ-регулятор с прямой связью и защитой от «накопления» интеграла при насыщении.
class PI {
public:
    void reset() { integ_ = 0; }
    float update(float target, float measured, float ff, float kp, float ki, float dt, float limit) {
        float e = target - measured;
        float out = ff + kp * e + ki * integ_;
        bool satHi = out > limit && e > 0;
        bool satLo = out < -limit && e < 0;
        if (!satHi && !satLo) integ_ += e * dt;   // интегрируем, только если это не усилит насыщение
        out = ff + kp * e + ki * integ_;
        return clampf(out, -limit, limit);
    }

private:
    float integ_ = 0;
};

// Медиана трёх последних значений — убирает одиночные выбросы УЗ-датчика (снежинки, эхо).
class Median3 {
public:
    float push(float v) {
        a_[i_] = v;
        i_ = (i_ + 1) % 3;
        if (n_ < 3) ++n_;
        if (n_ < 3) return v;
        float x = a_[0], y = a_[1], z = a_[2];
        return std::max(std::min(x, y), std::min(std::max(x, y), z));
    }
    void reset() { n_ = 0; i_ = 0; }

private:
    float a_[3] = {0, 0, 0};
    int i_ = 0, n_ = 0;
};

}  // namespace cyc
