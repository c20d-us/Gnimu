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

// State machine. Reads USB, switch, battery cutoff, and BLE subscription;
// drives gnssEnd(), the power holds, deep sleep, and reset.

enum SystemState {
  STATE_RUNNING,      // normal operation
  STATE_CHARGE_ONLY,  // USB in, switch on, STATE_CHARGE_ONLY_ON_USB 1:
                      // peripherals off for full charge current
  STATE_BATTERY_WAIT, // USB in, switch off
  STATE_DEEP_SLEEP,   // System OFF, on low voltage or idle timeout
};

// Classify and record the initial state. Enters deep sleep directly (no return)
// if that is the result; otherwise returns RUNNING or BATTERY_WAIT. Call after
// powerBegin(), batteryBegin(), and ledBegin().
SystemState stateBegin();

// Advance the state machine. Call every loop().
void stateUpdate();

// The current state.
SystemState stateCurrent();
