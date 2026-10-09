// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// Two-layer guard: see Clock.cpp for why.
#if defined(ARDUINO_ARCH_ESP32)
#include "sdkconfig.h"
#if CONFIG_IDF_TARGET_ESP32

#include <Arduino.h>
#include <esp_attr.h>
#include <esp_system.h>

#include "../ResetInfo.h"

namespace gnimu {

namespace {

// RTC slow memory the startup code never initialises: it survives every reset
// but power loss, and holds garbage after power-on (makeResetRecord ignores
// it then).
RTC_NOINIT_ATTR uint8_t keptRestartCode;

}  // namespace

ResetCause ResetInfo::cause() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:
      return ResetCause::PowerOn;
    case ESP_RST_BROWNOUT:
      return ResetCause::Brownout;
    case ESP_RST_TASK_WDT:
    case ESP_RST_INT_WDT:
    case ESP_RST_WDT:
      return ResetCause::Watchdog;
    case ESP_RST_SW:
      return ResetCause::Software;
    case ESP_RST_EXT:
      return ResetCause::Pin;
    case ESP_RST_PANIC:
      return ResetCause::Panic;
    case ESP_RST_DEEPSLEEP:
      return ResetCause::WakeDeepSleep;
    default:
      return ResetCause::Unknown;
  }
}

void ResetInfo::storeRestartCode(uint8_t code) { keptRestartCode = code; }

uint8_t ResetInfo::takeRestartCode() {
  const uint8_t code = keptRestartCode;
  keptRestartCode = 0;
  return code;
}

}  // namespace gnimu

#endif  // CONFIG_IDF_TARGET_ESP32
#endif  // ARDUINO_ARCH_ESP32
