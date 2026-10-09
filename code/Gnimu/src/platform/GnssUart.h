// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <Arduino.h>

namespace gnimu {

// The UART wired to the GNSS receiver. Implemented once per platform.
struct GnssUart {
  static Stream& stream();
  static void begin(uint32_t baud);
  // Safe when the port isn't open (never started, or already ended).
  static void end();
};

}  // namespace gnimu
