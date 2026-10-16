// Распиновка ESP32 DevKit V1 (30 pin). Таблица с пояснениями — в документе, раздел «Электрика».
// Ограничения ESP32, которые учтены:
//  * GPIO 34, 35, 36, 39 — только вход и БЕЗ внутренних подтяжек → внешние 10 кОм к 3,3 В;
//  * GPIO 0, 2, 5, 12, 15 — «strapping»: влияют на загрузку; 12 не трогаем совсем;
//  * GPIO 6–11 заняты flash — не использовать;
//  * АЦП2 не работает при включённом Wi-Fi → напряжение АКБ на GPIO36 (АЦП1).
#pragma once

// --- драйверы моторов IBT-2 (через буфер 74HCT245: 3,3 → 5 В) ---
constexpr int PIN_L_RPWM = 25;   // левый борт вперёд
constexpr int PIN_L_LPWM = 26;   // левый борт назад
constexpr int PIN_R_RPWM = 32;
constexpr int PIN_R_LPWM = 33;
constexpr int PIN_DRV_EN = 27;   // R_EN+L_EN обоих драйверов; внешний 10 кОм к GND — при загрузке моторы выключены

// --- энкодеры (питание 3,3 В!) ---
constexpr int PIN_ENC_L_A = 34;  // внешняя подтяжка 10 кОм
constexpr int PIN_ENC_L_B = 35;  // внешняя подтяжка 10 кОм
constexpr int PIN_ENC_R_A = 18;
constexpr int PIN_ENC_R_B = 19;

// --- I2C: MPU6050 ---
constexpr int PIN_SDA = 21;
constexpr int PIN_SCL = 22;

// --- UART к Raspberry Pi (3,3 В с обеих сторон, общий GND) ---
constexpr int PIN_PI_RX = 16;    // ← GPIO14 (TXD) Raspberry Pi
constexpr int PIN_PI_TX = 17;    // → GPIO15 (RXD) Raspberry Pi

// --- УЗ-датчики HC-SR04 (питание 5 В, ECHO через делитель 10к/20к) ---
constexpr int PIN_US_TRIG[3] = {5, 23, 2};    // левый, центр, правый
constexpr int PIN_US_ECHO[3] = {14, 4, 15};

// --- прочее ---
constexpr int PIN_SERVO = 13;    // сигнал сервопривода отвала
constexpr int PIN_VBAT = 36;     // делитель 100к/22к + 100 нФ
constexpr int PIN_BUMPER = 39;   // кнопки бампера на GND, внешняя подтяжка 10 кОм

// --- калибровочные константы ---
constexpr float VBAT_DIVIDER = (100.0f + 22.0f) / 22.0f;
constexpr float VBAT_CAL = 1.00f;     // подогнать по мультиметру
constexpr float GYRO_SIGN = 1.0f;     // если поворот против часовой даёт «минус» — поставить -1
constexpr int PWM_FREQ = 20000;       // 20 кГц — моторы не пищат
constexpr int PWM_BITS = 10;
