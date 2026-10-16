// Все настраиваемые параметры прошивки. Значения — стартовые, подбираются на стенде.
// Часть можно менять на ходу командой SET (см. Robot::applySet).
#pragma once
#include <cstdint>

namespace cyc {

struct Config {
    // --- геометрия и энкодеры ---
    float wheelDiameterM = 0.100f;
    float trackWidthM    = 0.28f;    // колея: нужна для перевода угловой скорости в скорости бортов
    float countsPerRev   = 3960.0f;  // 11 имп × 90 (редуктор) × 4 (квадратура). ПРОВЕРИТЬ на вашем моторе!
    int8_t encLeftSign   = 1;        // если при движении вперёд счётчик убывает — поставить -1
    int8_t encRightSign  = 1;
    int8_t motorLeftSign = 1;        // если борт крутится назад при положительной команде — -1
    int8_t motorRightSign = 1;

    // --- скорости и разгон ---
    float maxSpeedMps    = 0.30f;    // ограничение для любых команд
    float noLoadSpeedMps = 0.37f;    // скорость при ШИМ 100 % от 12 В без нагрузки (для прямой связи)
    float accelMps2      = 0.20f;
    float turnRateDps    = 45.0f;    // макс. скорость поворота
    float minCreepMps    = 0.04f;    // минимальная скорость в конце отрезка
    float turnInnerRatio = -1.0f;    // -1 — поворот на месте; 0 — вокруг стоящего борта; 0.3 — по дуге

    // --- регуляторы ---
    float speedKp = 1.2f, speedKi = 3.0f;      // ПИ скорости борта (вход — м/с, выход — доля ШИМ)
    float headingKp = 0.03f;                   // удержание курса: м/с разницы бортов на 1° ошибки
    float turnKp = 2.0f;                       // поворот: °/с на 1° ошибки
    float turnTolDeg = 1.0f;
    float distTolM = 0.01f;
    float motorVoltMax = 12.0f;                // моторы 12 В, АКБ до 16,8 В → ограничиваем ШИМ

    // --- безопасность ---
    float obstStopCm   = 35.0f;
    float obstResumeCm = 45.0f;
    float obstSlowCm   = 90.0f;
    uint32_t obstResumeMs = 1000;    // столько препятствие должно отсутствовать, чтобы ехать дальше
    uint32_t obstWaitMs   = 15000;   // столько ждём, потом DONE BLOCKED
    uint32_t stallMs      = 1500;
    float stallDuty       = 0.45f;   // застревание: ШИМ выше этого, а колёса стоят
    float stallSpeedMps   = 0.02f;
    float vbatLow  = 13.6f;          // 3,4 В/банку — предупреждение
    float vbatCrit = 12.8f;          // 3,2 В/банку — остановка
    uint32_t vbatLowMs  = 5000;
    uint32_t vbatCritMs = 3000;
    uint32_t heartbeatMs = 1000;     // нет кадров от Pi дольше — остановка
    uint32_t velTimeoutMs = 600;     // ручное управление VEL надо обновлять
    float backSpeedMps = 0.12f;      // назад едем медленно: сзади датчиков нет

    // --- отвал (серво, мкс) ---
    uint16_t bladeUpUs = 1100;
    uint16_t bladeDownUs = 1900;

    // --- служебное ---
    uint32_t tickMs = 10;
    uint32_t telemetryMs = 100;
    uint32_t calibrateMs = 2000;
};

}  // namespace cyc
