// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

#include "BatteryMonitor.h"
#include "BleStats.h"
#include "GnssEpoch.h"
#include "ImuSample.h"
#include "ResetReason.h"

namespace gnimu {

// The device's state at one moment, as the D report and the status line show
// it. Built once by Diagnostics and formatted immediately on the same loop
// pass - nothing updates the pointed-to objects in between, so pointers are as
// consistent as copies would be.
//
// No default member values, so it stays a C++11 aggregate: `DeviceStatus s =
// {};` zeroes everything and callers (and tests) set fields by name.
struct DeviceStatus {
  uint64_t uptimeUs;
  ResetRecord reset;  // how this boot started; fixed for the whole run
  struct Gnss {
    const EpochTracker* epochs;  // always set
    const GnssEpoch* latest;     // nullptr until the first epoch
  } gnss;
  struct Imu {
    bool up;
    ImuSample sample;  // physical units; the formatters convert for display
    bool trimLocked;
    float tiltDegrees;
  } imu;
  struct Ble {
    bool connected;
    BleStats stats;
  } ble;
  struct Power {
    const BatteryStatus* battery;  // nullptr on boards without one (the ESP32)
    bool usbPresent;
    uint32_t switchNoise;  // switch readings rejected as noise (battery boards)
  } power;
  uint32_t consoleDropped;  // console bytes lost to a host that stopped reading
};

}  // namespace gnimu
