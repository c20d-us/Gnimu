// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

// Test helpers: physical-unit inputs that the formatters display as exactly
// the integers given. Each is half a display unit away from zero, so the
// formatters' truncation lands on the intended value regardless of float
// rounding in the conversion factor.

#include "../code/Gnimu/src/core/ImuSample.h"

namespace gnimu {
namespace test {

inline float halfUnitOut(int32_t v) { return v + (v < 0 ? -0.5f : 0.5f); }

inline ImuSample sampleShowing(int32_t ax, int32_t ay, int32_t az, int32_t gx, int32_t gy, int32_t gz) {
  ImuSample s;
  const int32_t a[3] = {ax, ay, az};
  const int32_t g[3] = {gx, gy, gz};
  for (int i = 0; i < 3; i++) {
    s.accel[i] = halfUnitOut(a[i]) / kMps2ToMilliG;
    s.gyro[i] = halfUnitOut(g[i]) / kRadpsToCentiDeg;
  }
  return s;
}

// Tilt is never negative.
inline float degreesShowing(int32_t centiDeg) { return (centiDeg + 0.5f) / 100.0f; }

}  // namespace test
}  // namespace gnimu
