// Gnimu - RaceBox Mini-compatible GNSS+IMU streaming telemetry
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

// ============================================================================
// IMU module - onboard LSM6DS3TR-C accelerometer + gyroscope
//
// Owns all IMU state internally.
// ============================================================================

// The filtered IMU values converted to RaceBox protocol units.
struct ImuProtocolUnits {
  int16_t gX, gY, gZ; // acceleration, milli-g
  int16_t rX, rY, rZ; // rotation rate, centi-deg/sec
};

// Power the IMU, detect it, set ranges/ODR/bandwidth, and seed the filters
// with a first reading. Halts with a serial message if the chip isn't found.
void imuBegin();

// Poll the IMU and advance its filters. Self-throttles internally on two
// cadences, so it is safe to call every loop():
//   - IMU_SAMPLE_INTERVAL_MS: reads the sensor and updates each axis filter.
//   - IMU_TRANSMIT_INTERVAL_MS: decimates each axis down to the transmission
//     rate and caches the result, independent of BLE connection state.
void imuPoll();

// Retrieve the most recent transmission-rate IMU values in RaceBox protocol
// units. Cheap accessor with no side effects. Safe to call any number of
// times per frame.
ImuProtocolUnits imuReadProtocolUnits();

// LIGHT_SLEEP entry: drop to low-power ODR and arm the embedded omni-
// directional wake-up detector. Do not call imuPoll() while armed as normal
// ODR/ranges are not in effect.
void imuArmWake();

// Poll while armed (every loop()). Returns true once (edge-detected) when
// the wake-up detector fires, clearing the latch internally. False the rest
// of the time, and always false if imuArmWake() hasn't been called.
bool imuWakeTriggered();

// LIGHT_SLEEP exit: restore normal ODR/range/BDU and disarm the wake-up
// interrupt routing, re-seeding the axis filters (same as a fresh imuBegin())
// since they've been idle.
void imuDisarmWake();
