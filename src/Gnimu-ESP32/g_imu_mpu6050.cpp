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

#include "g_imu_sensor.h"

#include "config.h"
#include "g_log.h"
#include <Arduino.h>

// IMU driver for the InvenSense MPU-6050 (ESP32). The Adafruit library handles
// bring-up; samples use this file's checked burst read. See g_imu_sensor.h.

#if IMU_ENABLED
// Inside the #if so IMU_ENABLED 0 builds don't need the library.
#include <Adafruit_MPU6050.h>
#include <Wire.h>

// Counts per g and per deg/s for each range, fixed at compile time.
// imuSensorBegin() verifies the chip took the configured range.
static constexpr float accelCountsPerG(mpu6050_accel_range_t r) {
  return r == MPU6050_RANGE_2_G    ? 16384.0f
         : r == MPU6050_RANGE_4_G  ? 8192.0f
         : r == MPU6050_RANGE_8_G  ? 4096.0f
         : r == MPU6050_RANGE_16_G ? 2048.0f
                                   : 0.0f;
}
static constexpr float gyroCountsPerDps(mpu6050_gyro_range_t r) {
  return r == MPU6050_RANGE_250_DEG    ? 131.0f
         : r == MPU6050_RANGE_500_DEG  ? 65.5f
         : r == MPU6050_RANGE_1000_DEG ? 32.8f
         : r == MPU6050_RANGE_2000_DEG ? 16.4f
                                       : 0.0f;
}
static constexpr float kAccelCountsPerG = accelCountsPerG(IMU_ACCEL_RANGE_G);
static constexpr float kGyroCountsPerDps = gyroCountsPerDps(IMU_GYRO_RANGE_DPS);
static_assert(kAccelCountsPerG > 0.0f,
              "ERROR: IMU_ACCEL_RANGE_G must be one of MPU6050_RANGE_2_G, "
              "_4_G, _8_G or _16_G.");
static_assert(kGyroCountsPerDps > 0.0f,
              "ERROR: IMU_GYRO_RANGE_DPS must be one of MPU6050_RANGE_250_DEG, "
              "_500_DEG, _1000_DEG or _2000_DEG.");

// Burst from MPU6050_ACCEL_OUT (0x3B): accel X/Y/Z, temperature, gyro X/Y/Z,
// each a big-endian pair.
static constexpr size_t kBurstBytes = 14;

// Bring-up only. getEvent() reports success regardless of bus errors.
static Adafruit_MPU6050 myIMU;

// Big-endian register pair, high byte first.
static inline int16_t rawPair(const uint8_t *p) {
  return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

bool imuSensorBegin() {
  if (!myIMU.begin(IMU_I2C_ADDRESS)) {
    return false;
  }

  // Must follow begin(), which resets the bus clock.
  Wire.setClock(IMU_I2C_CLOCK_HZ);

  myIMU.setAccelerometerRange(IMU_ACCEL_RANGE_G);
  myIMU.setGyroRange(IMU_GYRO_RANGE_DPS);
  myIMU.setFilterBandwidth(IMU_FILTER_BANDWIDTH_HZ);

  // Verify the settings took, since the setters are unchecked and the fixed
  // scaling depends on the range. A failed read also fails the check.
  if (myIMU.getAccelerometerRange() != IMU_ACCEL_RANGE_G ||
      myIMU.getGyroRange() != IMU_GYRO_RANGE_DPS ||
      myIMU.getFilterBandwidth() != IMU_FILTER_BANDWIDTH_HZ) {
    LOG_PRINTLN("❌ MPU-6050 answered but did not take its configuration.");
    return false;
  }
  return true;
}

bool imuSensorRead(ImuRawSample *out) {
  // One 14-byte burst. On this core endTransmission(false) sends nothing and
  // always returns 0; the transfer happens in requestFrom(), so its count is
  // the check.
  Wire.beginTransmission((uint8_t)IMU_I2C_ADDRESS);
  Wire.write((uint8_t)MPU6050_ACCEL_OUT);
  (void)Wire.endTransmission(false);
  if (Wire.requestFrom((uint8_t)IMU_I2C_ADDRESS, kBurstBytes, true) !=
      kBurstBytes) {
    return false;
  }
  uint8_t raw[kBurstBytes];
  for (size_t i = 0; i < kBurstBytes; i++) {
    raw[i] = (uint8_t)Wire.read();
  }

  const int16_t ax = rawPair(&raw[0]);
  const int16_t ay = rawPair(&raw[2]);
  const int16_t az = rawPair(&raw[4]);

  // All-zero accel is the chip's reset state (e.g. after a supply dropout),
  // not a measurement. Treat it as a failed read.
  if (ax == 0 && ay == 0 && az == 0) {
    return false;
  }

  out->accel[0] = ax / kAccelCountsPerG;
  out->accel[1] = ay / kAccelCountsPerG;
  out->accel[2] = az / kAccelCountsPerG;
  // raw[6..7] is temperature, unused.
  out->gyro[0] = rawPair(&raw[8]) / kGyroCountsPerDps;
  out->gyro[1] = rawPair(&raw[10]) / kGyroCountsPerDps;
  out->gyro[2] = rawPair(&raw[12]) / kGyroCountsPerDps;
  return true;
}

#else // IMU_ENABLED == 0

// No IMU fitted: bring-up fails, and the pipeline reports "not fitted".
bool imuSensorBegin() { return false; }
bool imuSensorRead(ImuRawSample *) { return false; }

#endif // IMU_ENABLED
