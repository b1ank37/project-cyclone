// Прошивка ESP32 «Циклон»: реализация Hal для реального железа + главный цикл.
// Вся логика — в lib/core (Robot). Здесь только работа с выводами.
//
// ВНИМАНИЕ: этот файл не компилировался под ESP32 (не было доступа к тулчейну);
// ядро lib/core проверено тестами на ПК. При первой сборке возможны мелкие правки API.
#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>
#include <soc/gpio_reg.h>

#include "config.h"
#include "pins.h"
#include "protocol.h"
#include "robot.h"

// ------------------------------------------------ ШИМ: совместимость Arduino-ESP32 2.x и 3.x
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static void pwmInit(int pin, int /*ch*/, int freq, int bits) { ledcAttach(pin, freq, bits); }
static void pwmWrite(int pin, int /*ch*/, uint32_t duty) { ledcWrite(pin, duty); }
#else
static void pwmInit(int pin, int ch, int freq, int bits) {
    ledcSetup(ch, freq, bits);
    ledcAttachPin(pin, ch);
}
static void pwmWrite(int /*pin*/, int ch, uint32_t duty) { ledcWrite(ch, duty); }
#endif

// ------------------------------------------------ энкодеры: квадратура ×4 на прерываниях
static inline int IRAM_ATTR fastRead(int pin) {
    return pin < 32 ? (REG_READ(GPIO_IN_REG) >> pin) & 1 : (REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1;
}
// таблица переходов: индекс = (старое состояние << 2) | новое
static const DRAM_ATTR int8_t QEM[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
static volatile int32_t encL = 0, encR = 0;
static volatile uint8_t stL = 0, stR = 0;

static void IRAM_ATTR isrEncL() {
    uint8_t s = (fastRead(PIN_ENC_L_A) << 1) | fastRead(PIN_ENC_L_B);
    encL += QEM[(stL << 2) | s];
    stL = s;
}
static void IRAM_ATTR isrEncR() {
    uint8_t s = (fastRead(PIN_ENC_R_A) << 1) | fastRead(PIN_ENC_R_B);
    encR += QEM[(stR << 2) | s];
    stR = s;
}

// ------------------------------------------------ УЗ-датчики: неблокирующее измерение
static volatile int64_t usRise[3] = {0, 0, 0};
static volatile int32_t usEchoUs[3] = {-1, -1, -1};
static volatile bool usFresh[3] = {false, false, false};

// PIN — параметр шаблона, чтобы в прерывании не читать массив из flash
template <int I, int PIN>
static void IRAM_ATTR isrEcho() {
    int64_t t = esp_timer_get_time();
    if (fastRead(PIN)) {
        usRise[I] = t;
    } else if (usRise[I] > 0) {
        usEchoUs[I] = static_cast<int32_t>(t - usRise[I]);
        usFresh[I] = true;
        usRise[I] = 0;
    }
}

class Esp32Hal : public cyc::Hal {
public:
    void begin() {
        pinMode(PIN_DRV_EN, OUTPUT);
        digitalWrite(PIN_DRV_EN, LOW);
        const int pwmPins[4] = {PIN_L_RPWM, PIN_L_LPWM, PIN_R_RPWM, PIN_R_LPWM};
        for (int i = 0; i < 4; ++i) {
            pwmInit(pwmPins[i], i, PWM_FREQ, PWM_BITS);
            pwmWrite(pwmPins[i], i, 0);
        }
        pwmInit(PIN_SERVO, 4, 50, 16);

        pinMode(PIN_ENC_L_A, INPUT);  // 34/35: подтяжки только внешние
        pinMode(PIN_ENC_L_B, INPUT);
        pinMode(PIN_ENC_R_A, INPUT_PULLUP);
        pinMode(PIN_ENC_R_B, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_A), isrEncL, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PIN_ENC_L_B), isrEncL, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_A), isrEncR, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PIN_ENC_R_B), isrEncR, CHANGE);

        for (int i = 0; i < 3; ++i) {
            pinMode(PIN_US_TRIG[i], OUTPUT);
            digitalWrite(PIN_US_TRIG[i], LOW);
            pinMode(PIN_US_ECHO[i], INPUT);
        }
        attachInterrupt(digitalPinToInterrupt(PIN_US_ECHO[0]), isrEcho<0, PIN_US_ECHO[0]>, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PIN_US_ECHO[1]), isrEcho<1, PIN_US_ECHO[1]>, CHANGE);
        attachInterrupt(digitalPinToInterrupt(PIN_US_ECHO[2]), isrEcho<2, PIN_US_ECHO[2]>, CHANGE);

        pinMode(PIN_BUMPER, INPUT);
        analogReadResolution(12);

        Wire.begin(PIN_SDA, PIN_SCL, 400000);
        initGyro();

        digitalWrite(PIN_DRV_EN, HIGH);  // драйверы включаем последними
    }

    // Опрос УЗ: каждые 30 мс запускаем следующий датчик по кругу (чтобы они не слышали друг друга).
    void pollUltrasonic() {
        uint32_t now = ::millis();
        if (now - usLastTrig_ < 30) return;
        usLastTrig_ = now;
        int i = usNext_;
        // ответа на прошлый запуск этого датчика не было → «далеко/нет эха»
        if (!usFresh[i]) range_[i] = -1.0f;
        else range_[i] = usEchoUs[i] / 58.0f;
        usFresh[i] = false;
        int k = (i + 1) % 3;
        usNext_ = k;
        usRise[k] = 0;
        digitalWrite(PIN_US_TRIG[k], HIGH);
        delayMicroseconds(10);
        digitalWrite(PIN_US_TRIG[k], LOW);
    }

    // --- cyc::Hal ---
    uint32_t millis() override { return ::millis(); }

    void setMotors(float l, float r) override {
        writeSide(PIN_L_RPWM, 0, PIN_L_LPWM, 1, l);
        writeSide(PIN_R_RPWM, 2, PIN_R_LPWM, 3, r);
    }
    int32_t encoderLeft() override {
        noInterrupts();
        int32_t v = encL;
        interrupts();
        return v;
    }
    int32_t encoderRight() override {
        noInterrupts();
        int32_t v = encR;
        interrupts();
        return v;
    }
    float gyroZ() override {
        Wire.beginTransmission(0x68);
        Wire.write(0x47);  // GYRO_ZOUT_H
        if (Wire.endTransmission(false) != 0 || Wire.requestFrom(0x68, 2) != 2) {
            if (++gyroErrors_ > 20) gyroOk_ = false;
            return lastGyro_;
        }
        int16_t raw = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
        gyroErrors_ = 0;
        lastGyro_ = GYRO_SIGN * raw / 65.5f;  // ±500 °/с
        return lastGyro_;
    }
    bool gyroOk() override { return gyroOk_; }
    float rangeCm(int i) override { return range_[i]; }
    float batteryVolts() override { return analogReadMilliVolts(PIN_VBAT) / 1000.0f * VBAT_DIVIDER * VBAT_CAL; }
    bool bumper() override { return digitalRead(PIN_BUMPER) == LOW; }
    void setServoUs(uint16_t us) override { pwmWrite(PIN_SERVO, 4, static_cast<uint32_t>(us) * 65535UL / 20000UL); }
    void send(const char* line) override {
        Serial.print(line);   // USB — для отладки
        Serial2.print(line);  // Raspberry Pi
    }

private:
    void initGyro() {
        auto wr = [](uint8_t reg, uint8_t val) {
            Wire.beginTransmission(0x68);
            Wire.write(reg);
            Wire.write(val);
            return Wire.endTransmission() == 0;
        };
        gyroOk_ = wr(0x6B, 0x00)      // PWR_MGMT_1: проснуться
                  && wr(0x1A, 0x03)   // CONFIG: фильтр ~44 Гц
                  && wr(0x1B, 0x08);  // GYRO_CONFIG: ±500 °/с
    }
    static void writeSide(int pinF, int chF, int pinB, int chB, float d) {
        const uint32_t maxDuty = (1u << PWM_BITS) - 1;
        uint32_t v = static_cast<uint32_t>(fabsf(d) * maxDuty + 0.5f);
        if (v > maxDuty) v = maxDuty;
        pwmWrite(pinF, chF, d > 0 ? v : 0);
        pwmWrite(pinB, chB, d < 0 ? v : 0);
    }

    bool gyroOk_ = false;
    int gyroErrors_ = 0;
    float lastGyro_ = 0;
    float range_[3] = {-1, -1, -1};
    uint32_t usLastTrig_ = 0;
    int usNext_ = 0;
};

static cyc::Config cfg;
static Esp32Hal hal;
static cyc::Robot robot(hal, cfg);
static cyc::LineReader rdUsb, rdPi;
static uint32_t lastTick = 0;

static void pump(Stream& s, cyc::LineReader& rd) {
    while (s.available()) {
        if (rd.push(static_cast<char>(s.read()))) robot.onLine(rd.line());
    }
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, PIN_PI_RX, PIN_PI_TX);
    hal.begin();
    robot.begin();
    lastTick = millis();
}

void loop() {
    pump(Serial, rdUsb);
    pump(Serial2, rdPi);
    hal.pollUltrasonic();
    uint32_t now = millis();
    if (now - lastTick >= cfg.tickMs) {
        lastTick = now;
        robot.tick();
    }
}
