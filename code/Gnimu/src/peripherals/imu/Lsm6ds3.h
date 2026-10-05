#pragma once

#include "../../core/ImuFilter.h"
#include "../../core/ImuHealth.h"
#include "../../core/ImuSample.h"
#include "../../core/ImuTrim.h"
#include "../../app/Console.h"

namespace gnimu {

// Onboard LSM6DS3TR-C on the XIAO nRF52840 Sense, on its own Wire1 bus (the
// Seeed_Arduino_LSM6DS3 library redirects to it internally, keyed off the
// board's own TARGET_SEEED_XIAO_NRF52840_SENSE macro - nothing here has to
// know which bus it is). Never halts if absent or misconfigured, matching
// Mpu6050's policy and Classic's own: a board without this chip reports "not
// found" rather than failing to boot.
class Lsm6ds3 {
 public:
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
