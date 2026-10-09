// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// Two-layer guard: see Clock.cpp for why.
#if defined(ARDUINO_ARCH_ESP32)
#include "sdkconfig.h"
#if CONFIG_IDF_TARGET_ESP32

#include <Arduino.h>

#include "../ConsolePort.h"

namespace gnimu {

// Through the USB-UART bridge Serial is true as soon as begin() returns, and
// anything sent before the host reopens the port after a reset is lost. A fixed
// pause is the only option; waiting on Serial doesn't help.
void consoleOpen() {
  // The core's default is no TX buffer, only the UART's 128-byte FIFO, so any
  // longer print blocks the loop until it's on the wire (~15 ms for a status
  // line, ~95 ms for D at 115200). With a buffer, writes return at once and
  // the UART interrupt drains it. Must be set before begin().
  Serial.setTxBufferSize(2048);
  Serial.begin(115200);
  delay(750);
}

}  // namespace gnimu

#endif  // CONFIG_IDF_TARGET_ESP32
#endif  // ARDUINO_ARCH_ESP32
