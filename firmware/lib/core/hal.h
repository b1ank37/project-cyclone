// Интерфейс к «железу». Ядро (Robot) знает только его, поэтому одно и то же ядро
// работает на ESP32 (src/main.cpp) и в симуляторе на ПК (sim/).
#pragma once
#include <cstdint>

namespace cyc {

class Hal {
public:
    virtual ~Hal() = default;
    virtual uint32_t millis() = 0;
    // ШИМ бортов: -1…+1 (знак — направление). Ядро само ограничивает по напряжению АКБ.
    virtual void setMotors(float left, float right) = 0;
    // Накопленные отсчёты энкодеров (квадратура), растут при движении вперёд.
    virtual int32_t encoderLeft() = 0;
    virtual int32_t encoderRight() = 0;
    // Угловая скорость вокруг вертикали, °/с (против часовой — плюс). Без вычета смещения.
    virtual float gyroZ() = 0;
    virtual bool gyroOk() = 0;
    // Дальность УЗ-датчика i (0 — левый, 1 — центр, 2 — правый), см; < 0 — эха нет (далеко).
    virtual float rangeCm(int i) = 0;
    virtual float batteryVolts() = 0;
    virtual bool bumper() = 0;
    virtual void setServoUs(uint16_t us) = 0;
    // Отправить строку (уже с \n) в канал, откуда пришла последняя команда, или во все.
    virtual void send(const char* line) = 0;
};

}  // namespace cyc
