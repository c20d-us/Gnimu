// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "../../app/Console.h"
#include "../../core/ImuFilter.h"
#include "../../core/ImuHealth.h"
#include "../../core/ImuSample.h"
#include "../../core/ImuTrim.h"

// Which chip driver this board's ImuDevice runs, resolved at compile time - no
// virtual dispatch. ARDUINO_ARCH_ESP32 here (not the two-layer chip-specific
// guard some platform/esp32 files need): Mpu6050 is a builder-wired external
// sensor over the portable Wire API, not tied to one exact chip in the ESP32
// family. NRF52840_XXAA: the Sense's onboard chip is a fixed hardware fact of
// that chip, not a pinout choice, matching Clock/ConsolePort/Target's guard.
#if defined(ARDUINO_ARCH_ESP32)

#include "Mpu6050.h"
namespace gnimu {
using ImuChip = Mpu6050;
}

#elif defined(NRF52840_XXAA)

#include "Lsm6ds3.h"
namespace gnimu {
using ImuChip = Lsm6ds3;
}

#else
#error "no IMU driver selected for this board"
#endif

namespace gnimu {

// The IMU pipeline around this board's chip driver: health, mount trim and
// the smoothing filter, written once. The driver only brings its chip up and
// reads samples. Never halts if the chip is absent or misconfigured: GNSS is
// the primary sensor and the rest of the device runs without IMU data, so a
// board without the chip reports "not found" rather than failing to boot.
class ImuDevice {
 public:
  // Brings the chip up, logging each step to `log`. Leaves the IMU down (not
  // up()) if it isn't found or won't take its configuration.
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

  // The sensor-to-vehicle axis map this build applies.
  static const AxisMap& axisMap() { return ImuChip::axisMap(); }

 private:
  ImuChip chip_;
  ImuFilter filter_{0, 0};  // re-constructed with real alphas in begin()
  ImuHealth health_{0};
  ImuTrim trim_{ImuTrimConfig{}};  // re-constructed with real config in begin()
  Console* log_ = nullptr;
};

}  // namespace gnimu
