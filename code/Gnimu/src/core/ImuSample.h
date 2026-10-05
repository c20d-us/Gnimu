#pragma once

#include <stdint.h>

namespace gnimu {

// Physical units: m/s^2 for accel, rad/s for gyro. Protocol quantization
// (milli-g, centi-deg/s, ...) is each protocol's own job, done from this.
struct ImuSample {
  float accel[3] = {0, 0, 0};
  float gyro[3] = {0, 0, 0};
};

// Physical units to the milli-g / centi-deg/s scale shared by the RaceBox
// packet and the console - Classic's exact factors.
constexpr float kMps2ToMilliG = 1000.0f / 9.80665f;
constexpr float kRadpsToCentiDeg = (180.0f / 3.14159265f) * 100.0f;

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

}  // namespace gnimu
