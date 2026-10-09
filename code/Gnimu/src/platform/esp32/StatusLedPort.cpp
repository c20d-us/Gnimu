// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// Two-layer guard: see Clock.cpp for why.
#if defined(ARDUINO_ARCH_ESP32)
#include "sdkconfig.h"
#if CONFIG_IDF_TARGET_ESP32

#include <Arduino.h>

#include "../StatusLedPort.h"
#include "BoardConfig.h"

namespace gnimu {

// One single-color (blue) LED, active-high: the pattern shows, color can't.
void StatusLedPort::begin() {
  pinMode(board::kStatusLedPin, OUTPUT);
  digitalWrite(board::kStatusLedPin, LOW);
}

void StatusLedPort::write(bool on, LedColor) { digitalWrite(board::kStatusLedPin, on ? HIGH : LOW); }

}  // namespace gnimu

#endif  // CONFIG_IDF_TARGET_ESP32
#endif  // ARDUINO_ARCH_ESP32
