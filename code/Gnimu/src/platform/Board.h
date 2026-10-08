#pragma once

// The selected board's BoardConfig.h, for code that isn't itself per-board
// (App). Same guards as the per-board files: the ESP32's two-layer chip check,
// and the four XIAO nRF52840 boards whose pins nrf52840/BoardConfig.h names.
#if defined(ARDUINO_ARCH_ESP32)
#include "sdkconfig.h"
#if CONFIG_IDF_TARGET_ESP32
#include "esp32/BoardConfig.h"
#else
#error "No BoardConfig for this ESP32 variant"
#endif
#elif defined(ARDUINO_Seeed_XIAO_nRF52840) || defined(ARDUINO_Seeed_XIAO_nRF52840_Sense) || \
    defined(ARDUINO_Seeed_XIAO_nRF52840_Plus) || defined(ARDUINO_Seeed_XIAO_nRF52840_Sense_Plus)
#include "nrf52840/BoardConfig.h"
#else
#error "No BoardConfig for this board"
#endif
