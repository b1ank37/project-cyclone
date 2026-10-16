// Тесты ядра прошивки на ПК: make -C firmware/test
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include "../lib/core/protocol.h"
#include "../lib/core/robot.h"
#include "../sim/plant.h"

using namespace cyc;

static int g_fail = 0, g_checks = 0;
#define CHECK(cond)                                                                 \
    do {                                                                            \
        ++g_checks;                                                                 \
        if (!(cond)) {                                                              \
            ++g_fail;                                                               \
            std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);           \
        }                                                                           \
    } while (0)
#define CHECK_NEAR(a, b, tol)                                                       \
    do {                                                                            \
        ++g_checks;                                                                 \
        double _a = (a), _b = (b);                                                  \
        if (std::fabs(_a - _b) > (tol)) {                                           \
            ++g_fail;                                                               \
            std::printf("  FAIL %s:%d  %s = %.4f, ожидали %.4f ± %.4f\n", __FILE__, \
                        __LINE__, #a, _a, _b, (double)(tol));                       \
        }                                                                           \
    } while (0)

// Мир: модель + HAL + ядро. Шаг модели 1 мс, ядро — каждые 10 мс, heartbeat — каждые 200 мс.
struct World {
    Plant plant;
    SimHal hal;
    Config cfg;
    Robot robot;
    bool heartbeat = true;
    uint32_t lastHb = 0;
    long seq = 100;

    explicit World(PlantParams p = {}) : plant(p), hal(plant), robot(hal, cfg) {
        robot.begin();
        run(2.3f);  // автокалибровка гироскопа
    }
    void run(float sec, std::function<bool()> stop = nullptr) {
        int steps = static_cast<int>(sec * 1000);
        for (int i = 0; i < steps; ++i) {
            plant.step(0.001f);
            uint32_t ms = hal.millis();
            if (ms % 10 == 0) robot.tick();
            if (heartbeat && ms - lastHb >= 200) {
                lastHb = ms;
                robot.onLine("$HB*0A");
            }
            if (stop && stop()) return;
        }
    }
    long send(const char* fmt, double a = 0, double b = 0) {
        char body[160], tmp[96], frame[200];
        std::snprintf(tmp, sizeof(tmp), fmt, a, b);
        ++seq;
        // вставляем seq вторым полем: "DRV,%g,%g" → "DRV,<seq>,..."
        const char* comma = std::strchr(tmp, ',');
        if (comma) std::snprintf(body, sizeof(body), "%.*s,%ld%s", int(comma - tmp), tmp, seq, comma);
        else std::snprintf(body, sizeof(body), "%s,%ld", tmp, seq);
        makeFrame(frame, sizeof(frame), body);
        frame[std::strlen(frame) - 1] = '\0';  // убираем \n
        robot.onLine(frame);
        return seq;
    }
    bool has(const std::string& prefix) const {
        for (auto& l : hal.out)
            if (l.compare(0, prefix.size(), prefix) == 0) return true;
        return false;
    }
    std::string find(const std::string& prefix) const {
        for (auto& l : hal.out)
            if (l.compare(0, prefix.size(), prefix) == 0) return l;
        return "";
    }
    bool done(long s) const { return has("$DONE," + std::to_string(s) + ","); }
    std::string doneLine(long s) const { return find("$DONE," + std::to_string(s) + ","); }
    bool runUntilDone(long s, float maxSec) {
        run(maxSec, [&] { return done(s); });
        return done(s);
    }
};

static void test_protocol() {
    std::printf("protocol\n");
    char f[64];
    CHECK(makeFrame(f, sizeof(f), "PNG,1") > 0);
    // XOR "PNG,1" = 0x50^0x4E^0x47^0x2C^0x31
    char expect[32];
    std::snprintf(expect, sizeof(expect), "$PNG,1*%02X\n", 0x50 ^ 0x4E ^ 0x47 ^ 0x2C ^ 0x31);
    CHECK(std::strcmp(f, expect) == 0);

    Message m;
    CHECK(parseLine("$DRV,5,1.50,0.25", m, true) == ParseResult::Ok);
    CHECK(m.n == 4 && std::strcmp(m.f[0], "DRV") == 0 && std::strcmp(m.f[3], "0.25") == 0);
    CHECK(parseLine("$DRV,5,1.50,0.25", m, false) == ParseResult::BadChecksum);
    CHECK(parseLine("$PNG,1*00", m, true) == ParseResult::BadChecksum);
    f[std::strlen(f) - 1] = '\0';
    CHECK(parseLine(f, m, false) == ParseResult::Ok && m.hadChecksum);
    CHECK(parseLine("garbage", m, true) == ParseResult::NoDollar);
    CHECK(parseLine("  \r", m, true) == ParseResult::Empty);

    LineReader lr;
    const char* stream = "xx$G\x01\x02$PNG,7*";
    for (const char* p = stream; *p; ++p) CHECK(!lr.push(*p));
    char cs[4];
    std::snprintf(cs, sizeof(cs), "%02X", checksum("PNG,7", 5));
    lr.push(cs[0]);
    lr.push(cs[1]);
    CHECK(lr.push('\n'));
    CHECK(parseLine(lr.line(), m, false) == ParseResult::Ok && std::strcmp(m.f[1], "7") == 0);

    float v;
    long iv;
    CHECK(toFloat("-1.25", v) && v == -1.25f);
    CHECK(!toFloat("1.2x", v) && !toFloat("", v) && !toFloat("nan", v));
    CHECK(toInt("42", iv) && iv == 42 && !toInt("4.2", iv));
}

static void test_boot_and_calibration() {
    std::printf("boot + calibration\n");
    World w;
    CHECK(w.has("$EVT,BOOT"));
    CHECK(w.has("$EVT,CAL,OK"));
    CHECK_NEAR(w.robot.gyroBias(), 0.8, 0.05);
    CHECK(w.robot.state() == State::Idle);
    CHECK(w.has("$TEL,"));
}

static void test_drive_straight() {
    std::printf("drive 2 m with uneven sides\n");
    PlantParams p;
    p.leftGain = 0.85f;  // левый борт слабее — регуляторы должны выровнять
    World w(p);
    long s = w.send("DRV,%.2f,%.2f", 2.0, 0.25);
    CHECK(w.has("$ACK," + std::to_string(s)));
    CHECK(w.runUntilDone(s, 20));
    CHECK(w.doneLine(s).find(",OK,") != std::string::npos);
    CHECK_NEAR(w.plant.x, 2.0, 0.06);       // настоящее положение
    CHECK_NEAR(w.plant.y, 0.0, 0.08);
    CHECK_NEAR(w.plant.th * 57.3, 0.0, 3.0);
    CHECK_NEAR(w.robot.x(), 2.0, 0.03);     // оценка робота
    w.run(0.5);
    CHECK(std::fabs(w.plant.wl) < 0.01 && std::fabs(w.plant.wr) < 0.01);
}

static void test_busy_and_duplicate() {
    std::printf("busy + duplicate seq\n");
    World w;
    long s = w.send("DRV,%.2f,%.2f", 1.0, 0.2);
    w.run(0.5);
    long s2 = w.send("DRV,%.2f,%.2f", 1.0, 0.2);
    CHECK(w.has("$NAK," + std::to_string(s2) + ",BUSY"));
    // повтор первой команды с тем же seq: только ACK, движение не перезапускается
    size_t acks = 0;
    char body[64], frame[96];
    std::snprintf(body, sizeof(body), "DRV,%ld,1.00,0.20", s);
    makeFrame(frame, sizeof(frame), body);
    w.robot.onLine(frame);
    for (auto& l : w.hal.out)
        if (l.rfind("$ACK," + std::to_string(s) + "*", 0) == 0) ++acks;
    CHECK(acks == 2);
    CHECK(w.runUntilDone(s, 15));
    CHECK_NEAR(w.plant.x, 1.0, 0.05);
}

static void test_turn() {
    std::printf("turn 90 / -45\n");
    World w;
    long s = w.send("TRN,%.1f", 90.0);
    CHECK(w.runUntilDone(s, 10));
    CHECK(w.doneLine(s).find(",OK,") != std::string::npos);
    CHECK_NEAR(w.plant.th * 57.2958, 90.0, 3.0);
    s = w.send("TRN,%.1f", -45.0);
    CHECK(w.runUntilDone(s, 10));
    CHECK_NEAR(w.plant.th * 57.2958, 45.0, 3.0);
}

static void test_turn_arc() {
    std::printf("turn as arc (turn_inner 0.3)\n");
    World w;
    long s = w.send("SET,turn_inner,%.2f", 0.3);
    CHECK(w.done(s));
    s = w.send("TRN,%.1f", 90.0);
    CHECK(w.runUntilDone(s, 15));
    CHECK_NEAR(w.plant.th * 57.2958, 90.0, 3.5);
    CHECK(w.plant.x > 0.05);  // ехали по дуге, а не крутились на месте
}

static void test_obstacle_pause_resume() {
    std::printf("obstacle: pause, then resume\n");
    PlantParams p;
    World w(p);
    float t = w.plant.t_;
    w.plant.obstacles.push_back({1.0f, 0.0f, 0.15f, t + 1.0f, t + 5.0f});  // человек прошёл и ушёл
    long s = w.send("DRV,%.2f,%.2f", 2.0, 0.25);
    CHECK(w.runUntilDone(s, 30));
    CHECK(w.has("$EVT,OBST"));
    CHECK(w.has("$EVT,CLEAR"));
    CHECK(w.doneLine(s).find(",OK,") != std::string::npos);
    CHECK_NEAR(w.plant.x, 2.0, 0.06);
    CHECK(!w.plant.bumper());
}

static void test_obstacle_blocked() {
    std::printf("obstacle: stays → BLOCKED\n");
    World w;
    w.send("SET,obst_wait_ms,%.0f", 3000);
    w.plant.obstacles.push_back({1.2f, 0.0f, 0.2f});
    long s = w.send("DRV,%.2f,%.2f", 3.0, 0.25);
    CHECK(w.runUntilDone(s, 30));
    CHECK(w.doneLine(s).find(",BLOCKED,") != std::string::npos);
    CHECK(w.plant.x < 1.2 - 0.2 - 0.2);  // остановились до препятствия, не коснувшись
    CHECK(!w.has("$EVT,BUMP"));
}

static void test_stall() {
    std::printf("stuck in snow → STALL\n");
    PlantParams p;
    p.stuckX = 0.5f;
    World w(p);
    long s = w.send("DRV,%.2f,%.2f", 2.0, 0.25);
    CHECK(w.runUntilDone(s, 20));
    CHECK(w.doneLine(s).find(",STALL,") != std::string::npos);
    CHECK(w.has("$EVT,STALL"));
    w.run(0.3);
    CHECK(w.plant.dutyL == 0 && w.plant.dutyR == 0);
}

static void test_link_lost() {
    std::printf("heartbeat lost → LINK\n");
    World w;
    long s = w.send("DRV,%.2f,%.2f", 5.0, 0.25);
    w.run(1.0);
    w.heartbeat = false;
    CHECK(w.runUntilDone(s, 5));
    CHECK(w.doneLine(s).find(",LINK,") != std::string::npos);
    CHECK(w.has("$EVT,LINK_LOST"));
    CHECK(w.plant.x < 1.0);
}

static void test_bumper() {
    std::printf("bumper → BUMPER, then refuse forward\n");
    World w;
    long s = w.send("DRV,%.2f,%.2f", 3.0, 0.25);
    w.run(1.0);
    w.plant.forceBumper = true;
    CHECK(w.runUntilDone(s, 2));
    CHECK(w.doneLine(s).find(",BUMPER,") != std::string::npos);
    long s2 = w.send("DRV,%.2f,%.2f", 1.0, 0.25);
    CHECK(w.has("$NAK," + std::to_string(s2) + ",BUMPER"));
    long s3 = w.send("DRV,%.2f,%.2f", -0.2, 0.1);  // назад — можно
    CHECK(w.has("$ACK," + std::to_string(s3)));
}

static void test_low_battery() {
    std::printf("battery critical → LOWBAT, refuse motion\n");
    World w;
    long s = w.send("DRV,%.2f,%.2f", 5.0, 0.25);
    w.run(1.0);
    w.plant.p_.vbat = 12.4f;
    CHECK(w.runUntilDone(s, 10));
    CHECK(w.doneLine(s).find(",LOWBAT,") != std::string::npos);
    CHECK(w.robot.state() == State::Fault);
    long s2 = w.send("DRV,%.2f,%.2f", 1.0, 0.25);
    CHECK(w.has("$NAK," + std::to_string(s2) + ",LOWBAT"));
}

static void test_duty_limit_and_backward() {
    std::printf("PWM limit at 16.8 V, slow backward\n");
    PlantParams p;
    p.vbat = 16.8f;
    p.loadDrop = 0.25f;  // тяжёлый снег — регулятор захочет много ШИМ
    World w(p);
    long s = w.send("DRV,%.2f,%.2f", 1.0, 0.3);
    float maxDuty = 0;
    w.run(4.0, [&] {
        maxDuty = std::max(maxDuty, std::fabs(w.plant.dutyL));
        return w.done(s);
    });
    CHECK(maxDuty <= 12.0f / 16.8f + 0.01f);
    World b;
    long s2 = b.send("DRV,%.2f,%.2f", -0.5, 0.3);
    float vmin = 0;
    b.run(8.0, [&] {
        vmin = std::min(vmin, 0.5f * (b.plant.wl + b.plant.wr));
        return b.done(s2);
    });
    CHECK(b.done(s2));
    CHECK(vmin > -b.cfg.backSpeedMps - 0.03f);
    CHECK_NEAR(b.plant.x, -0.5, 0.05);
}

static void test_vel_timeout_and_stop() {
    std::printf("VEL timeout, STP abort\n");
    World w;
    w.send("VEL,%.2f,%.1f", 0.2, 0.0);
    w.run(0.4);
    CHECK(w.robot.state() == State::Vel);
    w.run(0.6);
    CHECK(w.robot.state() == State::Idle);
    long s = w.send("DRV,%.2f,%.2f", 3.0, 0.25);
    w.run(1.0);
    long st = w.send("STP");
    CHECK(w.doneLine(s).find(",ABORT,") != std::string::npos);
    CHECK(w.done(st));
    CHECK(w.robot.state() == State::Idle);
}

static void test_bad_input() {
    std::printf("bad input\n");
    World w;
    long s = w.send("SET,no_such_key,%.1f", 1.0);
    CHECK(w.has("$NAK," + std::to_string(s) + ",BADARG"));
    s = w.send("DRV,%.1f", 100.0);
    CHECK(w.has("$NAK," + std::to_string(s) + ",BADARG"));
    s = w.send("FLY");
    CHECK(w.has("$NAK," + std::to_string(s) + ",UNKNOWN"));
    w.robot.onLine("$PNG,1*00");
    CHECK(w.has("$NAK,-,CHECKSUM"));
    s = w.send("BLD,%.0f", 1);
    CHECK(w.robot.bladeDown() && w.plant.servoUs == w.cfg.bladeDownUs);
}

static void test_gyro_fail_fallback() {
    std::printf("gyro failure → heading from encoders\n");
    World w;
    w.hal.gyroOk_ = false;
    long s = w.send("TRN,%.1f", 90.0);
    CHECK(w.runUntilDone(s, 15));
    CHECK(w.has("$EVT,GYRO_FAIL"));
    // по энкодерам курс завышается/занижается из-за проскальзывания — проверяем грубо
    CHECK(std::fabs(w.plant.th * 57.2958 - 90.0) < 35.0);
}

static void test_absolute_heading() {
    std::printf("DRV with absolute heading, TRN absolute\n");
    World w;
    long s = w.send("TRN,%.1f,45,1", 30.0);   // абсолютный курс 30°
    CHECK(w.runUntilDone(s, 10));
    CHECK_NEAR(w.plant.th * 57.2958, 30.0, 1.5);
    s = w.send("TRN,%.1f,45,1", 30.0);         // уже на курсе — сразу OK
    CHECK(w.runUntilDone(s, 3));
    s = w.send("DRV,%.2f,0.25,%.1f", 1.0, 40.0);  // ехать, удерживая 40°
    CHECK(w.runUntilDone(s, 15));
    CHECK_NEAR(w.plant.th * 57.2958, 40.0, 2.5);
}

int main() {
    test_protocol();
    test_boot_and_calibration();
    test_drive_straight();
    test_busy_and_duplicate();
    test_turn();
    test_turn_arc();
    test_obstacle_pause_resume();
    test_obstacle_blocked();
    test_stall();
    test_link_lost();
    test_bumper();
    test_low_battery();
    test_duty_limit_and_backward();
    test_vel_timeout_and_stop();
    test_bad_input();
    test_gyro_fail_fallback();
    test_absolute_heading();
    std::printf("\n%d проверок, ошибок: %d\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
