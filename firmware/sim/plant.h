// Простая физическая модель робота для симулятора и тестов ядра.
// Не претендует на точность: нужна, чтобы проверить логику (регуляторы, безопасность,
// протокол) без железа. Параметры — см. поля PlantParams.
#pragma once
#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "../lib/core/hal.h"

namespace cyc {

struct Obstacle {
    float x, y, r;          // м
    float t0 = 0, t1 = 1e9; // когда существует, с
    float hold = 0;         // >0: «прохожий» — исчезает через hold с после того, как робот подъехал ближе 1 м
};

struct PlantParams {
    float v0 = 0.37f;            // м/с при ШИМ 100 % и 12 В
    float tau = 0.08f;           // постоянная времени борта, с
    float loadDrop = 0.06f;      // падение скорости под нагрузкой (снег), м/с
    float track = 0.28f;
    float skid = 1.3f;           // бортовой привод: эффективная колея больше геометрической
    float wheelD = 0.100f;
    float cpr = 3960.0f;
    float gyroBias = 0.8f;       // °/с
    float gyroNoise = 0.05f;
    float leftGain = 1.0f;       // разница бортов (износ, давление) — проверка удержания курса
    float vbat = 15.2f;
    float stuckX = 1e9f;         // за этой координатой робот «вязнет» (колёса стоят)
    float sensorX = 0.20f;       // УЗ-датчики на 20 см впереди центра
    float sensorSpread = 30.0f;  // боковые датчики повёрнуты на ±30°
    float bumperX = 0.24f;
    uint32_t seed = 1;
};

class Plant {
public:
    explicit Plant(PlantParams p = {}) : p_(p), rng_(p.seed), noise_(0.0f, 1.0f) {}

    void step(float dt) {
        t_ += dt;
        float vb = p_.vbat / 12.0f;
        float sL = (dutyL * vb * p_.v0 * p_.leftGain) - p_.loadDrop * sgn(dutyL);
        float sR = (dutyR * vb * p_.v0) - p_.loadDrop * sgn(dutyR);
        if (std::fabs(dutyL) < 0.15f) sL = 0;  // мёртвая зона: мотор не трогается
        if (std::fabs(dutyR) < 0.15f) sR = 0;
        bool stuck = x >= p_.stuckX && (sL + sR) > 0;
        if (stuck) sL = sR = 0;
        wl += (sL - wl) * dt / p_.tau;
        wr += (sR - wr) * dt / p_.tau;
        float v = 0.5f * (wl + wr);
        omega = (wr - wl) / (p_.track * p_.skid);
        th += omega * dt;
        x += v * std::cos(th) * dt;
        y += v * std::sin(th) * dt;
        for (auto& o : obstacles)
            if (o.hold > 0 && o.t1 > 1e8f && std::hypot(o.x - x, o.y - y) < 1.0f + o.r) o.t1 = t_ + o.hold;
        float cpm = p_.cpr / (3.14159265f * p_.wheelD);
        encL += wl * dt * cpm;
        encR += wr * dt * cpm;
    }

    float gyro() { return omega * 57.29578f + p_.gyroBias + p_.gyroNoise * noise_(rng_); }

    // Дальность датчика i: 0 — левый (+30°), 1 — центр, 2 — правый (−30°); < 0 — нет эха
    float range(int i) const {
        float ang = th + (i == 0 ? 1 : (i == 2 ? -1 : 0)) * p_.sensorSpread / 57.29578f;
        float sx = x + p_.sensorX * std::cos(th), sy = y + p_.sensorX * std::sin(th);
        float best = -1;
        for (auto& o : obstacles) {
            if (t_ < o.t0 || t_ > o.t1) continue;
            float dx = o.x - sx, dy = o.y - sy;
            float d = std::sqrt(dx * dx + dy * dy);
            float a = std::atan2(dy, dx) - ang;
            while (a > 3.14159f) a -= 6.28318f;
            while (a < -3.14159f) a += 6.28318f;
            float half = 15.0f / 57.29578f + std::atan2(o.r, std::max(d, 0.01f));
            if (std::fabs(a) < half) {
                float r = std::max(0.02f, d - o.r) * 100.0f;
                if (r < 400 && (best < 0 || r < best)) best = r;
            }
        }
        return best;
    }

    bool bumper() const {
        float bx = x + p_.bumperX * std::cos(th), by = y + p_.bumperX * std::sin(th);
        for (auto& o : obstacles) {
            if (t_ < o.t0 || t_ > o.t1) continue;
            if (std::hypot(o.x - bx, o.y - by) < o.r + 0.02f) return true;
        }
        return false;
    }

    PlantParams p_;
    std::vector<Obstacle> obstacles;
    float x = 0, y = 0, th = 0, wl = 0, wr = 0, omega = 0, t_ = 0;
    double encL = 0, encR = 0;
    float dutyL = 0, dutyR = 0;
    uint16_t servoUs = 0;
    bool forceBumper = false;

private:
    static float sgn(float v) { return v > 0 ? 1.0f : (v < 0 ? -1.0f : 0.0f); }
    std::mt19937 rng_;
    std::normal_distribution<float> noise_;
};

class SimHal : public Hal {
public:
    explicit SimHal(Plant& pl) : pl_(pl) {}
    uint32_t millis() override { return static_cast<uint32_t>(pl_.t_ * 1000.0f + 0.5f); }
    void setMotors(float l, float r) override { pl_.dutyL = l; pl_.dutyR = r; }
    int32_t encoderLeft() override { return static_cast<int32_t>(pl_.encL); }
    int32_t encoderRight() override { return static_cast<int32_t>(pl_.encR); }
    float gyroZ() override { return pl_.gyro(); }
    bool gyroOk() override { return gyroOk_; }
    float rangeCm(int i) override { return pl_.range(i); }
    float batteryVolts() override { return pl_.p_.vbat; }
    bool bumper() override { return pl_.forceBumper || pl_.bumper(); }
    void setServoUs(uint16_t us) override { pl_.servoUs = us; }
    void send(const char* line) override { out.emplace_back(line); }

    std::vector<std::string> out;
    bool gyroOk_ = true;

private:
    Plant& pl_;
};

}  // namespace cyc
