// Заглушки Arduino API — только для проверки синтаксиса main.cpp на ПК (make syntax).
#pragma once
#include <cmath>
#include <cstdint>
#include <cstddef>
#define IRAM_ATTR
#define DRAM_ATTR
#define ESP_ARDUINO_VERSION_MAJOR 2
enum { LOW = 0, HIGH = 1, INPUT = 1, OUTPUT = 3, INPUT_PULLUP = 5, CHANGE = 3 };
#define SERIAL_8N1 0x800001c
class Stream { public: int available(); int read(); size_t print(const char*); };
class HardwareSerial : public Stream { public: void begin(unsigned long, uint32_t = SERIAL_8N1, int8_t = -1, int8_t = -1); };
extern HardwareSerial Serial, Serial2;
void pinMode(uint8_t, uint8_t); void digitalWrite(uint8_t, uint8_t); int digitalRead(uint8_t);
void attachInterrupt(uint8_t, void (*)(void), int); inline uint8_t digitalPinToInterrupt(uint8_t p) { return p; }
unsigned long millis(); unsigned long micros(); void delayMicroseconds(uint32_t);
void noInterrupts(); void interrupts();
void analogReadResolution(uint8_t); uint32_t analogReadMilliVolts(uint8_t);
uint32_t ledcSetup(uint8_t, uint32_t, uint8_t); void ledcAttachPin(uint8_t, uint8_t); void ledcWrite(uint8_t, uint32_t);
