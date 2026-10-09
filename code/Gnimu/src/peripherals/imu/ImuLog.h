// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdarg.h>
#include <stdio.h>

#include "../../app/Console.h"

namespace gnimu {

// One bring-up step's log line, "IMU: <what> ✅" or "... ❌", shared by the
// chip drivers.
inline void imuStep(Console& log, bool ok, const char* fmt, ...) __attribute__((format(printf, 3, 4)));
inline void imuStep(Console& log, bool ok, const char* fmt, ...) {
  char what[96];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(what, sizeof(what), fmt, ap);
  va_end(ap);
  log.log("IMU: %s %s", what, ok ? "\xE2\x9C\x85" : "\xE2\x9D\x8C");
}

}  // namespace gnimu
