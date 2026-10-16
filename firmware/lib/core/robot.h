// Ядро робота: команды, движение, безопасность, одометрия, телеметрия.
// Не зависит от Arduino — тестируется и симулируется на ПК.
#pragma once
#include <cstdint>

#include "config.h"
#include "control.h"
#include "hal.h"
#include "protocol.h"

namespace cyc {

constexpr const char* kFirmwareVersion = "cyclone-fw 0.1";

enum class State : uint8_t { Idle, Drive, Turn, Vel, Calib, Fault };
enum class Result : uint8_t { Ok, Blocked, Stall, Bumper, Abort, LowBat, Link, Fail };

const char* stateName(State s);
const char* resultName(Result r);

class Robot {
public:
    Robot(Hal& hal, Config& cfg) : hal_(hal), cfg_(cfg) {}

    void begin();                  // после включения: EVT BOOT и автокалибровка гироскопа
    void onLine(const char* line); // строка из UART (без \n)
    void tick();                   // вызывать каждые cfg.tickMs

    // --- для тестов и отладки ---
    State state() const { return state_; }
    float x() const { return x_; }
    float y() const { return y_; }
    float heading() const { return heading_; }
    bool paused() const { return paused_; }
    float speedLeft() const { return vL_; }
    float speedRight() const { return vR_; }
    float dutyLeft() const { return dutyL_; }
    float dutyRight() const { return dutyR_; }
    float gyroBias() const { return gyroBias_; }
    bool bladeDown() const { return bladeDown_; }
    float range(int i) const { return range_[i]; }

private:
    // команды
    void handle(Message& m);
    void reply(const char* fmt, ...);
    bool busyForMotion() const { return state_ == State::Drive || state_ == State::Turn || state_ == State::Calib; }
    bool applySet(const char* key, float v);

    // цикл
    void sense(float dt);
    void safety(uint32_t now);
    void motion(uint32_t now, float dt);
    void wheels(float dt);
    void telemetry(uint32_t now);
    void finish(Result r, const char* info = "");
    void startCalib(long seq);

    Hal& hal_;
    Config& cfg_;

    State state_ = State::Idle;
    long activeSeq_ = -1;      // команда, по которой ждут DONE
    long lastSeq_ = -1;        // последняя принятая (для повторов)
    uint32_t lastTick_ = 0, lastRx_ = 0, lastTel_ = 0;
    bool started_ = false;

    // одометрия
    int32_t encL0_ = 0, encR0_ = 0;
    float x_ = 0, y_ = 0, heading_ = 0;
    float vL_ = 0, vR_ = 0, gyroRate_ = 0, gyroBias_ = 0;
    bool gyroFail_ = false;

    // датчики
    Median3 med_[3];
    float range_[3] = {999, 999, 999};
    float vbat_ = 0;

    // движение
    float dist_ = 0, speedCmd_ = 0, traveled_ = 0, vcur_ = 0;
    float headingTarget_ = 0;
    float velV_ = 0, velW_ = 0;
    uint32_t velStamp_ = 0, cmdStart_ = 0, settleSince_ = 0;
    float tL_ = 0, tR_ = 0;
    PI piL_, piR_;
    float dutyL_ = 0, dutyR_ = 0;

    // пауза из-за препятствия
    bool paused_ = false;
    uint32_t pauseSince_ = 0, clearSince_ = 0;

    // безопасность
    uint32_t stallSince_ = 0, lowSince_ = 0, critSince_ = 0;
    bool lowReported_ = false, lowbatLatched_ = false, linkLost_ = false;

    // калибровка
    double calSum_ = 0, calSumSq_ = 0;
    long calN_ = 0;
    uint32_t calStart_ = 0;

    bool bladeDown_ = false;
};

}  // namespace cyc
