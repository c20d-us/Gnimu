// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

namespace gnimu {

// Physical units: m/s^2 for accel, rad/s for gyro. Protocol quantization
// (milli-g, centi-deg/s, ...) is each protocol's own job, done from this.
struct ImuSample {
  float accel[3] = {0, 0, 0};
  float gyro[3] = {0, 0, 0};
};

// All three accel axes exactly zero is not a measurement: a working IMU
// always measures gravity. It's what a chip reads back after resetting
// itself, so drivers treat it as a failed read.
inline bool isZeroAccel(const ImuSample& s) {
  return s.accel[0] == 0.0f && s.accel[1] == 0.0f && s.accel[2] == 0.0f;
}

// Physical units to the milli-g / centi-deg/s scale shared by the RaceBox
// packet and the console.
constexpr float kMps2ToMilliG = 1000.0f / 9.80665f;
constexpr float kRadpsToCentiDeg = (180.0f / 3.14159265f) * 100.0f;

// The console's integer display units, converted once where they're printed
// so the formatters keep to integer printf formats. Truncated toward zero
// (not rounded). No int16 saturation: these are debug prints, not a wire
// format (the RaceBox encoder has its own).
struct ImuDisplay {
  int32_t accelMilliG[3];
  int32_t gyroCentiDegS[3];
};

inline ImuDisplay toDisplayUnits(const ImuSample& s) {
  ImuDisplay d;
  for (int i = 0; i < 3; i++) {
    d.accelMilliG[i] = static_cast<int32_t>(s.accel[i] * kMps2ToMilliG);
    d.gyroCentiDegS[i] = static_cast<int32_t>(s.gyro[i] * kRadpsToCentiDeg);
  }
  return d;
}

// Degrees to hundredths, truncated; the formatters round that to tenths.
inline int32_t tiltCentiDeg(float degrees) { return static_cast<int32_t>(degrees * 100.0f); }

// Maps the sensor frame into the vehicle frame: axis i of the output is
// src[i] of the input, sign-corrected. Identity (src={0,1,2}, sign={+1,+1,+1})
// is the default until a mount's orientation says otherwise.
struct AxisMap {
  uint8_t src[3];
  float sign[3];
};

// A general permutation can't be done in place: writing out[0] could clobber
// an input a later axis still needs. Three floats of stack avoids that.
inline void remapAxes(float v[3], const AxisMap& m) {
  const float in[3] = {v[0], v[1], v[2]};
  v[0] = in[m.src[0]] * m.sign[0];
  v[1] = in[m.src[1]] * m.sign[1];
  v[2] = in[m.src[2]] * m.sign[2];
}

// The map as text, e.g. "X=+x Y=-z Z=+y": each vehicle axis, then the signed
// sensor axis that feeds it. Returns the length written.
inline size_t formatAxisMap(char* buf, size_t cap, const AxisMap& m) {
  const int n = snprintf(buf, cap, "X=%c%c Y=%c%c Z=%c%c",
                         m.sign[0] < 0 ? '-' : '+', "xyz"[m.src[0]],
                         m.sign[1] < 0 ? '-' : '+', "xyz"[m.src[1]],
                         m.sign[2] < 0 ? '-' : '+', "xyz"[m.src[2]]);
  if (n < 0 || cap == 0) return 0;
  return static_cast<size_t>(n) < cap ? static_cast<size_t>(n) : cap - 1;
}

}  // namespace gnimu
