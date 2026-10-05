#pragma once

#include "Ema.h"
#include "ImuSample.h"

namespace gnimu {

// Six independent EMAs (3 accel + 3 gyro) advanced together as one sample.
class ImuFilter {
 public:
  ImuFilter(float accelAlpha, float gyroAlpha)
      : accel_{Ema(accelAlpha), Ema(accelAlpha), Ema(accelAlpha)},
        gyro_{Ema(gyroAlpha), Ema(gyroAlpha), Ema(gyroAlpha)} {}

  void reset(const ImuSample& s) {
    for (int i = 0; i < 3; i++) {
      accel_[i].reset(s.accel[i]);
      gyro_[i].reset(s.gyro[i]);
    }
  }

  void update(const ImuSample& s) {
    for (int i = 0; i < 3; i++) {
      accel_[i].update(s.accel[i]);
      gyro_[i].update(s.gyro[i]);
    }
  }

  ImuSample read() const {
    ImuSample out;
    for (int i = 0; i < 3; i++) {
      out.accel[i] = accel_[i].read();
      out.gyro[i] = gyro_[i].read();
    }
    return out;
  }

 private:
  Ema accel_[3];
  Ema gyro_[3];
};

}  // namespace gnimu
