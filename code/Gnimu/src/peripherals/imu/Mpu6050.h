#pragma once

#include "../../app/Console.h"
#include "../../core/ImuSample.h"

namespace gnimu {

// MPU-6050 over I2C: bring-up and sample reads only. ImuDevice runs the shared
// pipeline (health, trim, filter) around it.
class Mpu6050 {
 public:
  // Detects and configures the chip, logging each step to `log`, and returns
  // its first sample in `first`, once the gyro has settled. False if it isn't
  // found, won't take its configuration, or can't produce a sample.
  bool begin(Console& log, ImuSample& first);
  // One sample in physical units (m/s^2, rad/s), remapped to vehicle axes.
  // False on a failed or implausible read.
  bool read(ImuSample& out);
  // Wired straight to the supply, so there's nothing to power off.
  void end() {}
};

}  // namespace gnimu
