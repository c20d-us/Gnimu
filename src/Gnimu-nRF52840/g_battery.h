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

// Battery: VBAT sensing, state of charge, and the low-voltage cutoff request.
// Measurement only; g_state acts on it and g_power switches the rails.
// powerBegin() configures the shared ADC and must run first.

// Snapshot refreshed at the end of each sampler run.
struct BatteryStatus {
  float voltage;   // smoothed cell voltage, V
  uint8_t percent; // state of charge, 0-100
  bool charging;   // USB present and switch on
  bool warn;       // voltage <= BATTERY_WARN_V
  bool critical;   // voltage <= BATTERY_CRITICAL_V
  bool full;       // charging and voltage >= BATTERY_FULL_V
};

// Configure the divider and charge-current pins, then run one blocking sample
// (~50ms) so the status is valid on return. Call after powerBegin().
void batteryBegin();

// Advance the non-blocking sampler (at most one analogRead() per call).
// Call every loop().
void batteryPoll();

// The latest battery snapshot.
BatteryStatus batteryGetStatus();

// True once the unsmoothed voltage has stayed below BATTERY_CUTOFF_V for
// BATTERY_CUTOFF_DEBOUNCE_MS. Ignores USB; g_state applies that gate.
bool batteryCutoffRequested();
