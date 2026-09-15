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
#include <stdint.h>

// IMU sensor: the interface between the shared pipeline (g_imu.cpp) and one
// driver per sensor part. The pipeline owns remap, trim, filtering,
// decimation, unit conversion, and failure handling. A driver owns bring-up,
// reading, and conversion to g and deg/s.
//
//   g_imu_mpu6050.cpp  InvenSense MPU-6050 (ESP32)
//   g_imu_lsm6ds3.cpp  ST LSM6DS3TR-C (XIAO nRF52840 Sense)
//
// No Arduino dependency, so host tools can use these types.

// Filtered IMU values in TelemetrySample units (see g_protocol.h).
struct ImuProtocolUnits {
  int16_t gX, gY, gZ; // milli-g
  int16_t rX, rY, rZ; // centi-deg/s
};

// One six-axis sample in the sensor frame, [0]=X [1]=Y [2]=Z.
struct ImuRawSample {
  float accel[3]; // g
  float gyro[3];  // deg/s
};

// Implemented by the driver

// Power and configure the part. Returns false if it does not answer. Must not
// halt.
bool imuSensorBegin();

// Read one sample in g and deg/s. Returns false on failure, leaving `out`
// unspecified.
bool imuSensorRead(ImuRawSample *out);
