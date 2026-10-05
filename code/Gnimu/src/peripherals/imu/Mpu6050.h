#pragma once

#include "../../core/ImuFilter.h"
#include "../../core/ImuHealth.h"
#include "../../core/ImuSample.h"
#include "../../core/ImuTrim.h"
#include "../../app/Console.h"

namespace gnimu {

// MPU-6050 over I2C. Never halts if the chip is absent or misconfigured: GNSS
// is the primary sensor, and the rest of the device runs without IMU data,
// matching Classic's policy - it also means a board wired for a different (or
// no) IMU chip reports "not found" here rather than failing to boot.
class Mpu6050 {
 public:
  // Detects and configures the sensor, logging each step to `log`. Leaves the
  // IMU down (not up()) if it isn't found or won't take its configuration.
  void begin(Console& log);
  // Reads one sample, trims it, and advances the filter; call at the
  // configured period. speedMps/speedValid are the current GNSS ground speed
  // and whether it's still fresh enough to trust for the trim motion gate.
  void poll(float speedMps, bool speedValid);

  // Stops sampling for a power-state stop; reads go to zero.
  void end();

  bool up() const { return health_.up(); }
  // Filtered, axis-remapped, trim-corrected values in physical units
  // (m/s^2, rad/s). Zero while down.
  ImuSample read() const { return up() ? filter_.read() : ImuSample{}; }

  // Mount status, for the D report. See core/ImuTrim.h for what these mean.
  float trimTiltDegrees() const { return trim_.tiltDegrees(); }
  bool trimConverged() const { return trim_.converged(); }

 private:
  void step(bool ok, const char* fmt, ...) __attribute__((format(printf, 3, 4)));

  ImuFilter filter_{0, 0};  // re-constructed with real alphas in begin()
  ImuHealth health_{0};
  ImuTrim trim_{ImuTrimConfig{}};  // re-constructed with real config in begin()
  Console* log_ = nullptr;
};

}  // namespace gnimu
