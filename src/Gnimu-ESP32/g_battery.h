// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once
#include <Arduino.h>

// Battery stub: this board is USB-powered with no battery sensing. Provides
// the interface g_telemetry uses, reporting a constant percent. See
// BATTERY_HAS_GAUGE.

// Same layout as the nRF52840 BatteryStatus.
struct BatteryStatus {
  float voltage;   // always 0.0f
  uint8_t percent; // always BATTERY_REPORT_PERCENT
  bool charging;   // always false
  bool warn;       // always false
  bool critical;   // always false
  bool full;       // always false
};

// Constant battery snapshot.
BatteryStatus batteryGetStatus();
