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

// ============================================================================
// IMU DRIVER - InvenSense MPU-6050 (the ESP32 build).
//
// The part-specific half of the IMU: bring-up and configuration through the
// Adafruit library, then this file's own checked read, reporting g and deg/s.
// Everything else is the shared pipeline in g_imu.cpp; the seam is
// g_imu_sensor.h.
// ============================================================================

#include <Arduino.h> // first: config.h and the calls below assume it
#include "config.h"
#include "g_imu_sensor.h"
#include "g_log.h"

#if IMU_ENABLED
// The library include lives inside the #if, so a build with IMU_ENABLED 0 does
// not need the Adafruit MPU6050 library installed at all.
#include <Adafruit_MPU6050.h>
#include <Wire.h>

// Bring-up and configuration only. Samples are NOT read through the library:
// its getEvent() reports success whatever happened on the bus, and makes three
// unchecked transactions per sample to do it (IMU-3 - see imuSensorRead()).
static Adafruit_MPU6050 myIMU;

// Raw counts per g and per deg/s for each full-scale range - the divisors the
// library's own _read() uses. Taken from config.h at compile time rather than
// read back from the chip on every sample as the library does: that read-back
// is two extra unchecked transactions per sample, and a failed one returns all
// ones, which decodes as +/-16 g / +/-2000 deg/s and scales the sample 4x - a
// plausible-looking event out of nothing. Scaling from the config is only
// correct if the chip really took it, which imuSensorBegin() verifies once.
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

// The data registers read in one burst from MPU6050_ACCEL_OUT (0x3B): accel
// X/Y/Z, temperature, gyro X/Y/Z, each a big-endian 16-bit pair.
static constexpr size_t kBurstBytes = 14;

bool imuSensorBegin() {
  if (!myIMU.begin(IMU_I2C_ADDRESS)) {
    return false; // the pipeline decides what that means - never halt here
  }

  // Raise the IMU bus above the core's 100kHz default. MUST come after
  // begin(): that call brings the bus up and would overwrite anything set
  // earlier.
  Wire.setClock(IMU_I2C_CLOCK_HZ);

  myIMU.setAccelerometerRange(IMU_ACCEL_RANGE_G);
  myIMU.setGyroRange(IMU_GYRO_RANGE_DPS);
  myIMU.setFilterBandwidth(IMU_FILTER_BANDWIDTH_HZ);

  // Confirm the chip took all three. The setters return nothing and check
  // nothing, and begin() leaves the chip at +/-2 g, 500 deg/s and a 260 Hz
  // filter - so a write that did not land would go unnoticed all session.
  // With the accel left at +/-2 g, imuSensorRead()'s fixed scaling reads every
  // acceleration at TWICE its true size; with the filter write lost, the
  // anti-alias filter is gone. A read that fails here returns all ones and
  // fails the comparison too, so this errs toward "not up".
  if (myIMU.getAccelerometerRange() != IMU_ACCEL_RANGE_G ||
      myIMU.getGyroRange() != IMU_GYRO_RANGE_DPS ||
      myIMU.getFilterBandwidth() != IMU_FILTER_BANDWIDTH_HZ) {
    LOG_PRINTLN("❌ MPU-6050 answered but did not take its configuration.");
    return false;
  }
  return true;
}

// Reassemble a big-endian register pair - high byte first, the MPU-6050's
// order (the LSM6DS3's is the opposite).
static inline int16_t rawPair(const uint8_t *p) {
  return (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
}

bool imuSensorRead(ImuRawSample *out) {
  // ONE burst for all fourteen data registers, with its result checked -
  // replacing getEvent(), which returns a constant true (IMU-3).
  //
  // The check is requestFrom()'s COUNT, not endTransmission(). On this core
  // endTransmission(false) sends nothing: it marks a repeated start and
  // returns 0 unconditionally, and the whole write-then-read happens inside
  // requestFrom(). Checking endTransmission() would be getEvent()'s trap one
  // layer down.
  Wire.beginTransmission((uint8_t)IMU_I2C_ADDRESS);
  Wire.write((uint8_t)MPU6050_ACCEL_OUT);
  (void)Wire.endTransmission(false); // queues only - see above
  if (Wire.requestFrom((uint8_t)IMU_I2C_ADDRESS, kBurstBytes, true) !=
      kBurstBytes) {
    return false; // the pipeline holds the last good sample, then goes down
  }
  uint8_t raw[kBurstBytes];
  for (size_t i = 0; i < kBurstBytes; i++) {
    raw[i] = (uint8_t)Wire.read();
  }

  const int16_t ax = rawPair(&raw[0]);
  const int16_t ay = rawPair(&raw[2]);
  const int16_t az = rawPair(&raw[4]);

  // All three accel axes EXACTLY zero is the chip's power-on state, not a
  // measurement: a working accelerometer always sees gravity, and noise alone
  // makes an exact zero on all three vanishingly unlikely. It is what a module
  // whose supply dropped out and came back returns - asleep, every data
  // register zero - and those reads succeed. Reported as a failed read, so it
  // ends in the IMU-down state rather than a sensor apparently reading 0 g.
  if (ax == 0 && ay == 0 && az == 0) {
    return false;
  }

  out->accel[0] = ax / kAccelCountsPerG;
  out->accel[1] = ay / kAccelCountsPerG;
  out->accel[2] = az / kAccelCountsPerG;
  // raw[6..7] is the die temperature - read only because it sits between the
  // accel and gyro registers, which is what keeps this one burst.
  out->gyro[0] = rawPair(&raw[8]) / kGyroCountsPerDps;
  out->gyro[1] = rawPair(&raw[10]) / kGyroCountsPerDps;
  out->gyro[2] = rawPair(&raw[12]) / kGyroCountsPerDps;
  return true;
}

#else // IMU_ENABLED == 0

// No IMU fitted (see IMU_ENABLED in config.h). The pipeline reads a failed
// bring-up as "no IMU", says "not fitted", and every IMU field reads zero - the
// same state as an IMU that has died, deliberately, so there is one code path.
bool imuSensorBegin() { return false; }
bool imuSensorRead(ImuRawSample *) { return false; }

#endif // IMU_ENABLED
