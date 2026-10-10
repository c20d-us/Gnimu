// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include "../../app/Console.h"
#include "../../core/ImuSample.h"

namespace gnimu {

// Onboard LSM6DS3TR-C on the XIAO nRF52840 Sense, on its own Wire1 bus (the
// Seeed_Arduino_LSM6DS3 library redirects to it internally, keyed off the
// board's own TARGET_SEEED_XIAO_NRF52840_SENSE macro - nothing here has to
// know which bus it is). Bring-up and sample reads only: ImuDevice runs the
// shared pipeline (health, trim, filter) around it.
class Lsm6ds3 {
 public:
  // Powers, detects and configures the chip, logging each step to `log`, and
  // returns its first sample in `first` (the read that also proves sampling
  // works at the configured bus speed). False if it isn't found, won't take
  // its configuration, or can't produce a sample.
  bool begin(Console& log, ImuSample& first);
  // One sample in physical units (m/s^2, rad/s), remapped to vehicle axes.
  // False on a failed or all-zero read.
  bool read(ImuSample& out);
  // Powers the chip back off, as at boot.
  void end();
};

}  // namespace gnimu
