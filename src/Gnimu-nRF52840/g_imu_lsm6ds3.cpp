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

// IMU driver for the ST LSM6DS3TR-C on the XIAO nRF52840 Sense. The library
// reports g and deg/s directly. See g_imu_sensor.h.

#if IMU_ENABLED
// Inside the #if so IMU_ENABLED 0 builds don't need the library.
#include <LSM6DS3.h>
#include <Wire.h>

// Register codes for the configured settings, written and verified by
// imuSensorBegin().
static constexpr uint8_t odrCode(int hz) { // CTRL1_XL / CTRL2_G bits 7:4
  return hz == 13     ? 0x10
         : hz == 26   ? 0x20
         : hz == 52   ? 0x30
         : hz == 104  ? 0x40
         : hz == 208  ? 0x50
         : hz == 416  ? 0x60
         : hz == 833  ? 0x70
         : hz == 1660 ? 0x80
         : hz == 3330 ? 0x90
         : hz == 6660 ? 0xA0
                      : 0x00; // power-down, never valid
}
static constexpr uint8_t accelFsCode(int g) { // CTRL1_XL bits 3:2
  return g == 2 ? 0x00 : g == 16 ? 0x04 : g == 4 ? 0x08 : g == 8 ? 0x0C : 0xFF;
}
static constexpr uint8_t gyroFsCode(int dps) { // CTRL2_G bits 3:2, plus FS_125
  return dps == 125    ? 0x02                  // FS_125, with FS_G 00
         : dps == 245  ? 0x00
         : dps == 500  ? 0x04
         : dps == 1000 ? 0x08
         : dps == 2000 ? 0x0C
                       : 0xFF;
}
// CTRL1_XL bit 1, LPF1_BW_SEL: 0 = ODR/2, 1 = ODR/4. Bit 0 (BW0_XL) stays 0; it
// only applies at ODR >= 1.67kHz. The library's accelBandWidth encoding is for
// the original LSM6DS3, so this driver writes CTRL1_XL itself.
static constexpr uint8_t lpf1Code(int div) { return div == 4 ? 0x02 : 0x00; }
static constexpr uint8_t kCtrl1Xl = odrCode(IMU_ACCEL_ODR_HZ) |
                                    accelFsCode(IMU_ACCEL_RANGE_G) |
                                    lpf1Code(IMU_ACCEL_LPF1_ODR_DIV);
static constexpr uint8_t kCtrl2G =
    odrCode(IMU_GYRO_ODR_HZ) | gyroFsCode(IMU_GYRO_RANGE_DPS);
static constexpr uint8_t kCtrl3C =
    LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE | LSM6DS3_ACC_GYRO_IF_INC_ENABLED;
static_assert(odrCode(IMU_ACCEL_ODR_HZ) != 0x00 &&
                  odrCode(IMU_GYRO_ODR_HZ) != 0x00,
              "ERROR: IMU_ACCEL_ODR_HZ / IMU_GYRO_ODR_HZ is not a rate this "
              "driver can encode - keep it in step with config.h's list.");
static_assert(accelFsCode(IMU_ACCEL_RANGE_G) != 0xFF &&
                  gyroFsCode(IMU_GYRO_RANGE_DPS) != 0xFF,
              "ERROR: IMU_ACCEL_RANGE_G / IMU_GYRO_RANGE_DPS is not a range "
              "this driver can encode - keep it in step with config.h's list.");

static LSM6DS3 myIMU(I2C_MODE, IMU_I2C_ADDRESS);

// Read `len` registers, checking both the address write and the byte count.
// The library's readRegisterRegion() does not check the count.
static bool readRegs(uint8_t reg, uint8_t *out, uint8_t len) {
  Wire1.beginTransmission((uint8_t)IMU_I2C_ADDRESS);
  Wire1.write(reg);
  if (Wire1.endTransmission() != 0) {
    return false;
  }
  if (Wire1.requestFrom((uint8_t)IMU_I2C_ADDRESS, (size_t)len) != len) {
    return false;
  }
  for (uint8_t i = 0; i < len; i++) {
    out[i] = (uint8_t)Wire1.read();
  }
  return true;
}

// Little-endian register pair, low byte first.
static inline int16_t rawPair(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

bool imuSensorBegin() {
  // Power the IMU and let it boot.
  pinMode(IMU_POWER_PIN, OUTPUT);
  digitalWrite(IMU_POWER_PIN, HIGH);
  delay(300);

  // The library applies these in begin() and starts its own I2C bus.
  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = IMU_ACCEL_RANGE_G;
  myIMU.settings.accelSampleRate = IMU_ACCEL_ODR_HZ;
  myIMU.settings.accelBandWidth =
      400; // overwritten by the CTRL1_XL write below
  myIMU.settings.gyroEnabled = 1;
  myIMU.settings.gyroRange = IMU_GYRO_RANGE_DPS;
  myIMU.settings.gyroSampleRate = IMU_GYRO_ODR_HZ;
  myIMU.settings.tempEnabled = 0;

  if (myIMU.begin() != 0) {
    return false;
  }

  // Must follow begin(), which resets the bus clock. See IMU_I2C_CLOCK_HZ.
  Wire1.setClock(IMU_I2C_CLOCK_HZ);

  // Rate, range, and LPF1 in one write.
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, kCtrl1Xl);

  // Block data update (no torn 16-bit reads) and register auto-increment (for
  // the burst read).
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL3_C, kCtrl3C);

  // Verify the configuration took, since none of the writes are checked.
  // CTRL1_XL and CTRL2_G reset to power-down, so a lost write would read zeros.
  // CTRL1_XL is compared whole, CTRL2_G without its reserved bit, and CTRL3_C
  // only for BDU and IF_INC. A failed read leaves 0 and fails the check.
  uint8_t xl = 0, gy = 0, c3 = 0;
  if (!readRegs(LSM6DS3_ACC_GYRO_CTRL1_XL, &xl, 1) ||
      !readRegs(LSM6DS3_ACC_GYRO_CTRL2_G, &gy, 1) ||
      !readRegs(LSM6DS3_ACC_GYRO_CTRL3_C, &c3, 1) || xl != kCtrl1Xl ||
      (gy & 0xFE) != kCtrl2G || (c3 & kCtrl3C) != kCtrl3C) {
    LOG_PRINTF("❌ LSM6DS3 answered but did not take its configuration "
               "(CTRL1_XL 0x%02X want 0x%02X, CTRL2_G 0x%02X want 0x%02X, "
               "CTRL3_C 0x%02X want bits 0x%02X).\n",
               (unsigned int)xl, (unsigned int)kCtrl1Xl, (unsigned int)gy,
               (unsigned int)kCtrl2G, (unsigned int)c3, (unsigned int)kCtrl3C);
    return false;
  }
  return true;
}

bool imuSensorRead(ImuRawSample *out) {
  // One burst of 12 bytes from OUTX_L_G (0x22): gyro X/Y/Z then accel X/Y/Z,
  // so all six axes come from the same sample.
  uint8_t raw[12];
  if (!readRegs(LSM6DS3_ACC_GYRO_OUTX_L_G, raw, sizeof(raw))) {
    return false;
  }

  out->gyro[0] = myIMU.calcGyro(rawPair(&raw[0]));
  out->gyro[1] = myIMU.calcGyro(rawPair(&raw[2]));
  out->gyro[2] = myIMU.calcGyro(rawPair(&raw[4]));
  out->accel[0] = myIMU.calcAccel(rawPair(&raw[6]));
  out->accel[1] = myIMU.calcAccel(rawPair(&raw[8]));
  out->accel[2] = myIMU.calcAccel(rawPair(&raw[10]));
  return true;
}

#else // IMU_ENABLED == 0

// No IMU fitted: bring-up fails, and the pipeline reports "not fitted".
bool imuSensorBegin() { return false; }
bool imuSensorRead(ImuRawSample *) { return false; }

#endif // IMU_ENABLED
