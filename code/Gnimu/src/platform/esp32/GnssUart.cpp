// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// Two-layer guard: see Clock.cpp for why.
#if defined(ARDUINO_ARCH_ESP32)
#include "sdkconfig.h"
#if CONFIG_IDF_TARGET_ESP32

#include "../GnssUart.h"
#include "BoardConfig.h"

namespace gnimu {

static HardwareSerial uart(2);

Stream& GnssUart::stream() { return uart; }

void GnssUart::begin(uint32_t baud) {
  uart.setRxBufferSize(board::kGnssRxBufferBytes);
  uart.begin(baud, SERIAL_8N1, board::kGnssRxPin, board::kGnssTxPin);
}

void GnssUart::end() { uart.end(); }

}  // namespace gnimu

#endif  // CONFIG_IDF_TARGET_ESP32
#endif  // ARDUINO_ARCH_ESP32
