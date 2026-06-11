#pragma once
#include <Arduino.h>

// ============================================================================
// IMU module — MPU6050 accelerometer + gyroscope
//
// Owns all IMU state internally (see imu.cpp); callers interact only through
// the small API below.
// ============================================================================

// The filtered IMU values converted to RaceBox protocol units.
struct ImuProtocolUnits {
  int16_t gX, gY, gZ; // acceleration, milli-g
  int16_t rX, rY, rZ; // rotation rate, centi-deg/sec
};

// Detect the MPU6050, set ranges/bandwidth, and seed the filters with a first
// reading. Halts with a serial message if the chip isn't found. Call early in
// setup().
void imuBegin();

// Gyro-bias calibration — keep the device still during this. A no-op when
// GYRO_CALIBRATION_ENABLED is not defined. Call late in setup(), after the
// device has had time to settle.
void imuCalibrate();

// Read the MPU6050 at a fixed interval and update the smoothed accel/gyro
// values. Self-throttles to ACCEL_SAMPLE_INTERVAL_MS, so it is safe to call
// every loop().
void updateImuFilters();

// Convert the current filtered IMU values into RaceBox protocol units.
ImuProtocolUnits readImuProtocolUnits();
