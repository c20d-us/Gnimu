// Gnimu - RaceBox Mini-compatible GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// Based on the Open-Source RaceBox Mini Emulator by Anchit Chandra Sekhar
// (https://github.com/anchit92/Open-Source-RaceBox-mini-Emulator)
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
// IMU module - accelerometer + gyroscope
//
// Owns all IMU state internally (see gc_imu.cpp); callers interact only through
// the small API below.
// ============================================================================

// The filtered IMU values converted to RaceBox protocol units.
struct ImuProtocolUnits {
  int16_t gX, gY, gZ; // acceleration, milli-g
  int16_t rX, rY, rZ; // rotation rate, centi-deg/sec
};

// Detect the IMU, set ranges/bandwidth, seed the filters with a first
// reading, and (when GYRO_CALIBRATION_ENABLED is defined) calibrate gyro bias.
// Keep the device still during startup. Halts with a serial message if the
// chip isn't found.
void imuBegin();

// Poll the IMU at a fixed interval and update the smoothed accel/gyro
// values. Self-throttles to ACCEL_SAMPLE_INTERVAL_MS, so it is safe to call
// every loop().
void imuPoll();

// Convert the current filtered IMU values into RaceBox protocol units.
ImuProtocolUnits imuReadProtocolUnits();
