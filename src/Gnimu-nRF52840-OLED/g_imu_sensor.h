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
#include <stdint.h>

// ============================================================================
// The seam between the IMU PIPELINE (g_imu.cpp - identical in every tree) and
// a SENSOR DRIVER (g_imu_<part>.cpp - one per sensor part).
//
// The pipeline owns everything that is the same whatever chip is fitted: the
// axis remap, runtime trim, the per-axis filters, epoch-locked decimation,
// conversion to protocol units, and what a failed or missing sensor means. A
// driver owns only what is specific to its part: bring-up, one read, and
// converting its library's units to the ones below. Drivers are named after
// the PART, not the board, because the part is what varies - nothing ties a
// sensor to an MCU family.
//
// ONE UNIT SYSTEM: every driver reports acceleration in g and rotation in
// deg/s. That is what lets every IMU constant in config.h be identical on every
// board - the ESP32's used to be in m/s^2 and rad/s, because that is what its
// library returns.
//
//   g_imu_mpu6050.cpp - InvenSense MPU-6050 via Adafruit (the ESP32 build)
//   g_imu_lsm6ds3.cpp - ST LSM6DS3TR-C on the XIAO nRF52840 Sense
//
// Deliberately Arduino-free (only <stdint.h>), so host tools - the IMU harness
// and the planned buildSample() harness - can use these types directly.
// ============================================================================

// The filtered IMU values converted to RaceBox protocol units.
struct ImuProtocolUnits {
  int16_t gX, gY, gZ; // acceleration, milli-g
  int16_t rX, rY, rZ; // rotation rate, centi-deg/sec
};

// One coherent six-axis sample, in the SENSOR's frame. Indexed [0]=X, [1]=Y,
// [2]=Z. The pipeline remaps it into the vehicle frame.
struct ImuRawSample {
  float accel[3]; // g
  float gyro[3];  // deg/s
};

// --- Implemented by the driver ----------------------------------------------

// Power the part if it needs it, configure ranges/rates, confirm it answers.
// Returns false if it did not answer. MUST NOT halt: the pipeline decides what
// a missing sensor means, and on the battery builds a halt here would stop the
// low-voltage cutoff ever running (the same defect ROB-1 fixed for the GNSS).
bool imuSensorBegin();

// Take one sample, in g and deg/s. Returns false if the read failed, leaving
// `out` unspecified - the pipeline holds its last good sample rather than
// filtering garbage.
bool imuSensorRead(ImuRawSample *out);
