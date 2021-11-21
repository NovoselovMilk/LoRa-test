#ifndef __LORA_PACKET_CONFIG_H__
#define __LORA_PACKET_CONFIG_H__

#if defined( linux )
#include "../../Arduino.h"
constexpr size_t SIZE_LORA_PACKET_MAX_LEN = 250; // - 254 max
constexpr size_t SIZE_LORA_PACKET_BUFFER = 100;

#elif defined( ESP32 )
#include <Arduino.h>
constexpr size_t SIZE_LORA_PACKET_MAX_LEN = 250; // - 254 max
constexpr size_t SIZE_LORA_PACKET_BUFFER = 100;

#else // STM32
#include <main.h>
constexpr size_t SIZE_LORA_PACKET_MAX_LEN = 50; // - 254 max
constexpr size_t SIZE_LORA_PACKET_BUFFER = 80;

#endif // use (linux/ESP32/STM32)

// Использование вектора или статического массива
//#define USE_VECTOR
#if defined( USE_VECTOR )
#include <vector>
#define USE_STANDARD_ARRAY
#else
#define USE_STANDARD_ARRAY
#include <array>
#include <algorithm>
#endif

#endif // __LORA_PACKET_CONFIG_H__
