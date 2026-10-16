#pragma once
#include <cstdint>
#define GPIO_IN_REG 0x3FF4403C
#define GPIO_IN1_REG 0x3FF44040
#define REG_READ(r) (*(volatile uint32_t*)(uintptr_t)(r))
