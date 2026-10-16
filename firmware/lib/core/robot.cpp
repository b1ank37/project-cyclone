#include "robot.h"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace cyc {

namespace {
constexpr float kPi = 3.14159265f;
constexpr float kDeg = 180.0f / kPi;
constexpr float kFar = 999.0f;

float sgn(float v) { return v < 0 ? -1.0f : 1.0f; }
}  // namespace

const char* stateName(State s) {
    switch (s) {
        case State::Idle: return "IDLE";
        case State::Drive: return "DRIVE";
        case State::Turn: return "TURN";
        case State::Vel: return "VEL";
        case State::Calib: return "CALIB";
        case State::Fault: return "FAULT";
    }
    return "?";
}

const char* resultName(Result r) {
    switch (r) {
        case Result::Ok: return "OK";
        case Result::Blocked: return "BLOCKED";
        case Result::Stall: return "STALL";
        case Result::Bumper: return "BUMPER";
        case Result::Abort: return "ABORT";
        case Result::LowBat: return "LOWBAT";
        case Result::Link: return "LINK";
        case Result::Fail: return "FAIL";
    }
    return "?";
}

// ---------------------------------------------------------------- вывод
void Robot::reply(const char* fmt, ...) {
    char body[kMaxLine];
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(body, sizeof(body), fmt, ap);
    va_end(ap);
    char frame[kMaxLine + 8];
    if (makeFrame(frame, sizeof(frame), body)) hal_.send(frame);
}

// ---------------------------------------------------------------- запуск
void Robot::begin() {
    uint32_t now = hal_.millis();
    lastTick_ = lastRx_ = lastTel_ = now;
    encL0_ = hal_.encoderLeft();
    encR0_ = hal_.encoderRight();
    vbat_ = hal_.batteryVolts();
    hal_.setMotors(0, 0);
    hal_.setServoUs(cfg_.bladeUpUs);
    started_ = true;
    reply("EVT,BOOT,%s", kFirmwareVersion);
    startCalib(-1);  // робот должен стоять неподвижно первые 2 секунды после включения
}

void Robot::startCalib(long seq) {
    state_ = State::Calib;
    activeSeq_ = seq;
    calSum_ = calSumSq_ = 0;
    calN_ = 0;
    calStart_ = hal_.millis();
    cmdStart_ = calStart_;
}

// ---------------------------------------------------------------- команды
void Robot::onLine(const char* line) {
    Message m;
    ParseResult r = parseLine(line, m, /*allowNoChecksum=*/true);
    if (r == ParseResult::Empty || r == ParseResult::NoDollar) return;
    if (r != ParseResult::Ok) {
        reply("NAK,-,%s", r == ParseResult::BadChecksum ? "CHECKSUM" : "FORMAT");
        return;
    }
    lastRx_ = hal_.millis();
    linkLost_ = false;
    handle(m);
}

bool Robot::applySet(const char* k, float v) {
    struct Item { const char* key; float* f; float lo, hi; };
    Item items[] = {
        {"obst_stop", &cfg_.obstStopCm, 10, 200},   {"obst_resume", &cfg_.obstResumeCm, 10, 250},
        {"obst_slow", &cfg_.obstSlowCm, 10, 400},   {"max_speed", &cfg_.maxSpeedMps, 0.02f, 0.6f},
        {"accel", &cfg_.accelMps2, 0.02f, 2.0f},    {"turn_inner", &cfg_.turnInnerRatio, -1.0f, 0.8f},
        {"heading_kp", &cfg_.headingKp, 0, 1},      {"speed_kp", &cfg_.speedKp, 0, 20},
        {"speed_ki", &cfg_.speedKi, 0, 50},         {"turn_kp", &cfg_.turnKp, 0, 20},
        {"turn_rate", &cfg_.turnRateDps, 5, 180},   {"cpr", &cfg_.countsPerRev, 10, 100000},
        {"wheel_d", &cfg_.wheelDiameterM, 0.03f, 0.3f}, {"track", &cfg_.trackWidthM, 0.1f, 1.0f},
        {"stall_duty", &cfg_.stallDuty, 0.05f, 1.0f},
    };
    for (auto& it : items) {
        if (std::strcmp(k, it.key) == 0) {
            if (v < it.lo || v > it.hi) return false;
            *it.f = v;
            return true;
        }
    }
    struct ItemMs { const char* key; uint32_t* f; };
    ItemMs ms[] = {{"obst_wait_ms", &cfg_.obstWaitMs}, {"hb_ms", &cfg_.heartbeatMs},
                   {"stall_ms", &cfg_.stallMs}, {"obst_resume_ms", &cfg_.obstResumeMs}};
    for (auto& it : ms) {
        if (std::strcmp(k, it.key) == 0) {
            if (v < 50 || v > 600000) return false;
            *it.f = static_cast<uint32_t>(v);
            return true;
        }
    }
    return false;
}

void Robot::handle(Message& m) {
    const char* t = m.type();
    if (std::strcmp(t, "HB") == 0) return;

    long seq = -1;
    if (m.n < 2 || !toInt(m.f[1], seq) || seq < 0) {
        reply("NAK,-,BADARG");
        return;
    }
    // Повтор той же команды (Pi не получил ACK) — просто подтверждаем ещё раз.
    if (seq == lastSeq_) {
        reply("ACK,%ld", seq);
        return;
    }
    auto nak = [&](const char* why) { reply("NAK,%ld,%s", seq, why); };
    auto accept = [&]() { lastSeq_ = seq; reply("ACK,%ld", seq); };
    auto doneNow = [&](const char* info) { reply("DONE,%ld,OK,%s", seq, info); };
    uint32_t now = hal_.millis();

    if (std::strcmp(t, "PNG") == 0) {
        accept();
        doneNow(kFirmwareVersion);
    } else if (std::strcmp(t, "STP") == 0) {
        if (state_ != State::Idle && state_ != State::Fault) finish(Result::Abort, "STOP");
        accept();
        doneNow("STOPPED");
    } else if (std::strcmp(t, "DRV") == 0 || std::strcmp(t, "TRN") == 0) {
        bool drive = t[0] == 'D';
        float a = 0, sp = 0, hdg = 0;
        if (m.n < 3 || !toFloat(m.f[2], a)) return nak("BADARG");
        if (m.n >= 4 && !toFloat(m.f[3], sp)) return nak("BADARG");
        // DRV: 5-е поле — абсолютный курс, который надо держать (Pi знает курс маршрута,
        // так ошибки поворотов не накапливаются). TRN: 5-е поле — 1, если угол абсолютный.
        bool hasHdg = m.n >= 5 && toFloat(m.f[4], hdg);
        if (m.n >= 5 && !hasHdg) return nak("BADARG");
        if (busyForMotion()) return nak("BUSY");
        if (lowbatLatched_) return nak("LOWBAT");
        if (drive) {
            if (std::fabs(a) > 50.0f || std::fabs(a) < 0.005f) return nak("BADARG");
            if (a > 0 && hal_.bumper()) return nak("BUMPER");
            float lim = a > 0 ? cfg_.maxSpeedMps : cfg_.backSpeedMps;
            speedCmd_ = sp > 0 ? clampf(sp, 0.02f, lim) : lim;
            dist_ = a;
            traveled_ = 0;
            vcur_ = 0;
            paused_ = false;
            headingTarget_ = hasHdg ? wrapDeg(hdg) : heading_;
            state_ = State::Drive;
        } else {
            if (std::fabs(a) > 720.0f) return nak("BADARG");
            speedCmd_ = sp > 0 ? clampf(sp, 5.0f, cfg_.turnRateDps) : cfg_.turnRateDps;
            bool absolute = hasHdg && hdg == 1.0f;
            headingTarget_ = wrapDeg(absolute ? a : heading_ + a);
            dist_ = absolute ? wrapDeg(a - heading_) : a;  // для таймаута
            settleSince_ = 0;
            state_ = State::Turn;
        }
        piL_.reset();
        piR_.reset();
        activeSeq_ = seq;
        cmdStart_ = now;
        stallSince_ = 0;
        accept();
    } else if (std::strcmp(t, "VEL") == 0) {
        float v = 0, w = 0;
        if (m.n < 4 || !toFloat(m.f[2], v) || !toFloat(m.f[3], w)) return nak("BADARG");
        if (busyForMotion()) return nak("BUSY");
        if (lowbatLatched_) return nak("LOWBAT");
        velV_ = clampf(v, -cfg_.backSpeedMps, cfg_.maxSpeedMps);
        velW_ = clampf(w, -cfg_.turnRateDps, cfg_.turnRateDps);
        velStamp_ = now;
        if (state_ != State::Vel) {
            piL_.reset();
            piR_.reset();
            stallSince_ = 0;
        }
        state_ = State::Vel;
        activeSeq_ = -1;  // VEL не ждёт DONE
        accept();
    } else if (std::strcmp(t, "BLD") == 0) {
        long d = 0;
        if (m.n < 3 || !toInt(m.f[2], d) || (d != 0 && d != 1)) return nak("BADARG");
        bladeDown_ = d == 1;
        hal_.setServoUs(bladeDown_ ? cfg_.bladeDownUs : cfg_.bladeUpUs);
        accept();
        doneNow(bladeDown_ ? "DOWN" : "UP");
    } else if (std::strcmp(t, "CAL") == 0) {
        if (state_ != State::Idle) return nak("BUSY");
        accept();
        startCalib(seq);
    } else if (std::strcmp(t, "RST") == 0) {
        if (busyForMotion()) return nak("BUSY");
        x_ = y_ = heading_ = 0;
        accept();
        doneNow("ODOM_RESET");
    } else if (std::strcmp(t, "SET") == 0) {
        float v = 0;
        if (m.n < 4 || !toFloat(m.f[3], v) || !applySet(m.f[2], v)) return nak("BADARG");
        accept();
        doneNow(m.f[2]);
    } else {
        nak("UNKNOWN");
    }
}

// ---------------------------------------------------------------- цикл
void Robot::tick() {
    if (!started_) return;
    uint32_t now = hal_.millis();
    uint32_t dms = now - lastTick_;
    if (dms == 0) return;
    float dt = dms / 1000.0f;
    if (dt > 0.2f) dt = 0.2f;  // после долгой паузы не «прыгаем»
    lastTick_ = now;

    sense(dt);
    safety(now);
    motion(now, dt);
    wheels(dt);
    telemetry(now);
}

void Robot::sense(float dt) {
    int32_t eL = hal_.encoderLeft(), eR = hal_.encoderRight();
    float mPerCount = kPi * cfg_.wheelDiameterM / cfg_.countsPerRev;
    float dl = (eL - encL0_) * mPerCount * cfg_.encLeftSign;
    float dr = (eR - encR0_) * mPerCount * cfg_.encRightSign;
    encL0_ = eL;
    encR0_ = eR;
    vL_ = 0.6f * vL_ + 0.4f * (dl / dt);
    vR_ = 0.6f * vR_ + 0.4f * (dr / dt);

    if (hal_.gyroOk()) {
        gyroRate_ = hal_.gyroZ() - gyroBias_;
        if (state_ != State::Calib) heading_ = wrapDeg(heading_ + gyroRate_ * dt);
    } else {
        if (!gyroFail_) reply("EVT,GYRO_FAIL");
        gyroFail_ = true;
        gyroRate_ = (dr - dl) / cfg_.trackWidthM / dt * kDeg;  // запасной вариант: по энкодерам
        heading_ = wrapDeg(heading_ + gyroRate_ * dt);
    }
    float d = 0.5f * (dl + dr);
    float hr = heading_ / kDeg;
    x_ += d * std::cos(hr);
    y_ += d * std::sin(hr);
    if (state_ == State::Drive) traveled_ += d * sgn(dist_);

    for (int i = 0; i < 3; ++i) {
        float r = hal_.rangeCm(i);
        range_[i] = med_[i].push(r < 0 ? kFar : r);
    }
    float vb = hal_.batteryVolts();
    vbat_ = vbat_ <= 0.1f ? vb : 0.9f * vbat_ + 0.1f * vb;
}

void Robot::safety(uint32_t now) {
    bool moving = state_ == State::Drive || state_ == State::Turn || state_ == State::Vel;

    // 1. связь
    if (moving && now - lastRx_ > cfg_.heartbeatMs) {
        if (!linkLost_) reply("EVT,LINK_LOST");
        linkLost_ = true;
        finish(Result::Link, "NO_HEARTBEAT");
        return;
    }
    // 2. бампер — только если едем вперёд или поворачиваем
    bool forward = (state_ == State::Drive && dist_ > 0) || state_ == State::Turn || (state_ == State::Vel && velV_ > 0);
    if (forward && hal_.bumper()) {
        reply("EVT,BUMP");
        finish(Result::Bumper, "CONTACT");
        return;
    }
    // 3. батарея (vbat <= 1 В — датчик не подключён, пропускаем)
    if (vbat_ > 1.0f) {
        if (vbat_ < cfg_.vbatCrit) {
            if (!critSince_) critSince_ = now;
            if (now - critSince_ > cfg_.vbatCritMs && !lowbatLatched_) {
                lowbatLatched_ = true;
                reply("EVT,CRITBAT,%.2f", vbat_);
                if (moving) finish(Result::LowBat, "CRITICAL");
                state_ = State::Fault;
            }
        } else {
            critSince_ = 0;
        }
        if (vbat_ < cfg_.vbatLow) {
            if (!lowSince_) lowSince_ = now;
            if (now - lowSince_ > cfg_.vbatLowMs && !lowReported_) {
                lowReported_ = true;
                reply("EVT,LOWBAT,%.2f", vbat_);
            }
        } else if (vbat_ > cfg_.vbatLow + 0.3f) {
            lowSince_ = 0;
            lowReported_ = false;
        }
    }
    // 4. застревание: ШИМ большой, а колесо стоит
    if (moving && !paused_) {
        bool stuckL = std::fabs(dutyL_) > cfg_.stallDuty && std::fabs(vL_) < cfg_.stallSpeedMps;
        bool stuckR = std::fabs(dutyR_) > cfg_.stallDuty && std::fabs(vR_) < cfg_.stallSpeedMps;
        if (stuckL || stuckR) {
            if (!stallSince_) stallSince_ = now;
            if (now - stallSince_ > cfg_.stallMs) {
                reply("EVT,STALL,%s", stuckL && stuckR ? "BOTH" : (stuckL ? "LEFT" : "RIGHT"));
                finish(Result::Stall, stuckL && stuckR ? "BOTH" : (stuckL ? "LEFT" : "RIGHT"));
                return;
            }
        } else {
            stallSince_ = 0;
        }
    }
}

void Robot::motion(uint32_t now, float dt) {
    tL_ = tR_ = 0;
    float halfTrack = cfg_.trackWidthM / 2;

    switch (state_) {
        case State::Drive: {
            bool fwd = dist_ > 0;
            float remaining = std::fabs(dist_) - traveled_;
            if (remaining <= cfg_.distTolM) {
                finish(Result::Ok, "");
                return;
            }
            // препятствия — только при движении вперёд
            float slow = 1.0f;
            if (fwd) {
                float c = range_[1], side = std::min(range_[0], range_[2]);
                bool blocked = c < cfg_.obstStopCm || side < cfg_.obstStopCm * 0.6f;
                bool clear = c > cfg_.obstResumeCm && side > cfg_.obstResumeCm * 0.6f;
                if (!paused_ && blocked) {
                    paused_ = true;
                    pauseSince_ = now;
                    clearSince_ = 0;
                    reply("EVT,OBST,%.0f", std::min(c, side));
                }
                if (paused_) {
                    if (clear) {
                        if (!clearSince_) clearSince_ = now;
                        if (now - clearSince_ >= cfg_.obstResumeMs) {
                            paused_ = false;
                            stallSince_ = 0;
                            reply("EVT,CLEAR");
                        }
                    } else {
                        clearSince_ = 0;
                    }
                    if (paused_ && now - pauseSince_ > cfg_.obstWaitMs) {
                        finish(Result::Blocked, "OBSTACLE");
                        return;
                    }
                }
                if (c < cfg_.obstSlowCm)
                    slow = clampf((c - cfg_.obstStopCm) / (cfg_.obstSlowCm - cfg_.obstStopCm), 0.3f, 1.0f);
            }
            if (paused_) {
                vcur_ = 0;
                piL_.reset();
                piR_.reset();
                return;
            }
            float vmax = speedCmd_ * slow;
            float vprof = std::sqrt(2 * cfg_.accelMps2 * remaining);
            float target = std::min(vmax, vprof);
            vcur_ = std::min(target, vcur_ + cfg_.accelMps2 * dt);
            if (vcur_ < cfg_.minCreepMps) vcur_ = std::min(cfg_.minCreepMps, vmax);
            float err = wrapDeg(headingTarget_ - heading_);
            float dv = clampf(cfg_.headingKp * err, -0.1f, 0.1f);
            float s = fwd ? 1.0f : -1.0f;
            tL_ = s * vcur_ - dv;
            tR_ = s * vcur_ + dv;
            break;
        }
        case State::Turn: {
            float e = wrapDeg(headingTarget_ - heading_);
            if (std::fabs(e) < cfg_.turnTolDeg && std::fabs(gyroRate_) < 10.0f) {
                if (!settleSince_) settleSince_ = now;
                if (now - settleSince_ > 200) {
                    finish(Result::Ok, "");
                    return;
                }
            } else {
                settleSince_ = 0;
            }
            uint32_t limitMs = static_cast<uint32_t>(std::fabs(dist_) / speedCmd_ * 3000.0f) + 4000;
            if (now - cmdStart_ > limitMs) {
                finish(Result::Fail, "TIMEOUT");
                return;
            }
            float w = clampf(cfg_.turnKp * e, -speedCmd_, speedCmd_);
            if (std::fabs(e) >= cfg_.turnTolDeg && std::fabs(w) < 10.0f) w = 10.0f * sgn(e);
            if (std::fabs(e) < cfg_.turnTolDeg) w = 0;
            float vs = (w / kDeg) * halfTrack;  // скорость борта при повороте на месте
            float r = cfg_.turnInnerRatio;
            if (r <= -0.999f) {
                tL_ = -vs;
                tR_ = vs;
            } else {
                float outer = 2 * std::fabs(vs) / (1 - r);
                float inner = r * outer;
                if (w > 0) { tR_ = outer; tL_ = inner; }
                else { tL_ = outer; tR_ = inner; }
            }
            break;
        }
        case State::Vel: {
            if (now - velStamp_ > cfg_.velTimeoutMs) {
                state_ = State::Idle;
                return;
            }
            float v = velV_;
            if (v > 0) {
                float c = std::min(range_[1], std::min(range_[0], range_[2]) / 0.6f);
                if (c < cfg_.obstStopCm) v = 0;
                else if (c < cfg_.obstSlowCm)
                    v *= clampf((c - cfg_.obstStopCm) / (cfg_.obstSlowCm - cfg_.obstStopCm), 0.3f, 1.0f);
            }
            float dv = (velW_ / kDeg) * halfTrack;
            tL_ = v - dv;
            tR_ = v + dv;
            break;
        }
        case State::Calib: {
            if (hal_.gyroOk()) {
                float g = hal_.gyroZ();
                calSum_ += g;
                calSumSq_ += double(g) * g;
                ++calN_;
            }
            if (now - calStart_ >= cfg_.calibrateMs) {
                bool ok = false;
                float mean = 0, sd = 0;
                if (calN_ > 10) {
                    mean = static_cast<float>(calSum_ / calN_);
                    double var = calSumSq_ / calN_ - double(mean) * mean;
                    sd = static_cast<float>(std::sqrt(var > 0 ? var : 0));
                    bool still = std::fabs(vL_) < 0.01f && std::fabs(vR_) < 0.01f;
                    ok = sd < 1.0f && std::fabs(mean) < 10.0f && still;
                }
                if (ok) gyroBias_ = mean;
                reply("EVT,CAL,%s,%.3f,%.3f", ok ? "OK" : "FAIL", ok ? mean : gyroBias_, sd);
                finish(ok ? Result::Ok : Result::Fail, ok ? "GYRO" : "MOVING");
                if (lowbatLatched_) state_ = State::Fault;
            }
            break;
        }
        case State::Idle:
        case State::Fault:
            break;
    }
}

void Robot::wheels(float dt) {
    bool active = state_ == State::Drive || state_ == State::Turn || state_ == State::Vel;
    if (!active || (state_ == State::Drive && paused_)) {
        dutyL_ = dutyR_ = 0;
        piL_.reset();
        piR_.reset();
    } else {
        float limit = 1.0f;
        if (vbat_ > cfg_.motorVoltMax) limit = cfg_.motorVoltMax / vbat_;  // не подаём больше 12 В на моторы
        float k = 1.0f / cfg_.noLoadSpeedMps;  // прямая связь: ШИМ ≈ v / v_max
        dutyL_ = piL_.update(tL_, vL_, tL_ * k, cfg_.speedKp, cfg_.speedKi, dt, limit);
        dutyR_ = piR_.update(tR_, vR_, tR_ * k, cfg_.speedKp, cfg_.speedKi, dt, limit);
        if (tL_ == 0 && tR_ == 0 && std::fabs(vL_) < 0.01f && std::fabs(vR_) < 0.01f) dutyL_ = dutyR_ = 0;
    }
    hal_.setMotors(dutyL_ * cfg_.motorLeftSign, dutyR_ * cfg_.motorRightSign);
}

void Robot::telemetry(uint32_t now) {
    if (now - lastTel_ < cfg_.telemetryMs) return;
    lastTel_ = now;
    int flags = (paused_ ? 1 : 0) | (hal_.bumper() ? 2 : 0) | (lowReported_ ? 4 : 0) |
                (lowbatLatched_ ? 8 : 0) | (gyroFail_ ? 16 : 0) | (linkLost_ ? 32 : 0);
    reply("TEL,%lu,%.3f,%.3f,%.1f,%.3f,%.3f,%s,%.0f,%.0f,%.0f,%.2f,%d,%d",
          static_cast<unsigned long>(now), x_, y_, heading_, vL_, vR_, stateName(state_),
          range_[0], range_[1], range_[2], vbat_, bladeDown_ ? 1 : 0, flags);
}

void Robot::finish(Result r, const char* info) {
    if (activeSeq_ >= 0) reply("DONE,%ld,%s,%s", activeSeq_, resultName(r), info);
    activeSeq_ = -1;
    state_ = lowbatLatched_ ? State::Fault : State::Idle;
    paused_ = false;
    tL_ = tR_ = 0;
    vcur_ = 0;
    dutyL_ = dutyR_ = 0;
    piL_.reset();
    piR_.reset();
    stallSince_ = 0;
    hal_.setMotors(0, 0);
}

}  // namespace cyc
