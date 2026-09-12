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
// IMU DRIVER - ST LSM6DS3TR-C, onboard the Seeed XIAO nRF52840 Sense.
//
// The part-specific half of the IMU: bring-up and one coherent read (the
// library already reports g and deg/s, so nothing to convert). Everything else
// is the shared pipeline in g_imu.cpp; the seam is g_imu_sensor.h.
// ============================================================================

#include <Arduino.h> // first: config.h and the calls below assume it
#include "config.h"
#include "g_imu_sensor.h"
#include "g_log.h"

#if IMU_ENABLED
// The library include lives inside the #if, so a build for a board with no IMU
// (IMU_ENABLED 0 - the plain XIAO nRF52840) does not need the Seeed LSM6DS3
// library installed at all.
#include <LSM6DS3.h>
#include <Wire.h> // Wire1, for the setClock() below

// IMU driver. I2C at IMU_I2C_ADDRESS (0x6A).
static LSM6DS3 myIMU(I2C_MODE, IMU_I2C_ADDRESS);

// Checked register reads on the IMU's bus, used instead of the library's
// readRegisterRegion(), which checks the register-address write but NOT the
// byte count: a short or failed data phase leaves the tail of the caller's
// buffer as uninitialised stack and still reports success. Same transaction
// shape as the library - address write with a stop, then the read - because
// that shape is proven on this bus; only the checks are new. Both report
// honestly on this core: endTransmission() returns the TWIM's error, and
// requestFrom() returns the true byte count (RXD.AMOUNT).
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

// Register codes for the configured rates and ranges, so imuSensorBegin() can
// confirm the chip took them. Same tables the library's begin() switches on -
// written out rather than reached into, so a library that stopped matching the
// chip shows up as a mismatch instead of being believed.
//
// The filter bits are ours too, since 2026-09-11: see kCtrl1Xl below.
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
                      : 0x00; // 0x00 is POWER-DOWN: never a configured rate
}
static constexpr uint8_t accelFsCode(int g) { // CTRL1_XL bits 3:2
  return g == 2 ? 0x00 : g == 16 ? 0x04 : g == 4 ? 0x08 : g == 8 ? 0x0C : 0xFF;
}
static constexpr uint8_t gyroFsCode(int dps) { // CTRL2_G bits 3:2, plus FS_125
  return dps == 125    ? 0x02 // FS_125, with FS_G 00
         : dps == 245  ? 0x00
         : dps == 500  ? 0x04
         : dps == 1000 ? 0x08
         : dps == 2000 ? 0x0C
                       : 0xFF;
}
// CTRL1_XL bit 1, LPF1_BW_SEL: 0 = ODR/2, 1 = ODR/4.
//
// Bit 0 (BW0_XL, the analog chain) is deliberately left 0, the part's default:
// ST's driver documents it as "only for accelerometer ODR >= 1.67 kHz", and
// config.h static_asserts that we stay below that. The Seeed library would set
// both bits from its own accelBandWidth setting, which encodes the ORIGINAL
// LSM6DS3's analog anti-alias filter (400/200/100/50 Hz) and means something
// else entirely on this part - so this file writes CTRL1_XL itself, below,
// rather than letting that mapping through.
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

// Reassemble a little-endian register pair, matching the byte order the
// library's own readRegisterInt16() uses (low byte first).
static inline int16_t rawPair(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

bool imuSensorRead(ImuRawSample *out) {
  // ONE burst for all six axes rather than six separate two-transaction
  // reads. OUTX_L_G (0x22) through OUTZ_H_XL (0x2D) is 12 contiguous bytes -
  // gyro X/Y/Z then accel X/Y/Z - so this is 2 I2C transactions instead of 12.
  //
  // Speed is the lesser reason. The real one is SAMPLE COHERENCY: BDU (set in
  // imuSensorBegin()) freezes each register PAIR so a 16-bit read cannot
  // straddle a sample, but six separate transactions still leave gaps in which
  // a new sample can land - so X could come from one sample and Z from the
  // next. A single burst returns one coherent six-axis set, which matters when
  // the values feed an axis remap, a runtime trim, and a transient-peak
  // detector that is specifically looking for short events.
  uint8_t raw[12];
  if (!readRegs(LSM6DS3_ACC_GYRO_OUTX_L_G, raw, sizeof(raw))) {
    // Reported, not papered over: the pipeline holds its last good sample and
    // decides when a run of these means the part has gone (g_imu.cpp).
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

bool imuSensorBegin() {
  // Power the onboard LSM6DS3TR-C, then give it time to boot before I2C. Pin
  // 15 driven HIGH enables it.
  pinMode(IMU_POWER_PIN, OUTPUT);
  digitalWrite(IMU_POWER_PIN, HIGH);
  delay(300); // wait for IMU to boot and settle

  // Configure ranges / output data rates / bandwidth from config.h. The Seeed
  // library takes plain integers here and applies them in begin(); it also
  // brings up its own I2C bus internally - no manual Wire/Wire1 setup needed.
  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = IMU_ACCEL_RANGE_G;
  myIMU.settings.accelSampleRate = IMU_ACCEL_ODR_HZ;
  // Whatever is passed here, CTRL1_XL is rewritten below - the library's
  // bandwidth setting cannot express this part's filter (see kCtrl1Xl).
  myIMU.settings.accelBandWidth = 400;
  myIMU.settings.gyroEnabled = 1;
  myIMU.settings.gyroRange = IMU_GYRO_RANGE_DPS;
  myIMU.settings.gyroSampleRate = IMU_GYRO_ODR_HZ;
  myIMU.settings.tempEnabled = 0; // unused by this firmware

  // The Seeed library returns 0 (IMU_SUCCESS) on success. A part that does not
  // answer returns false - never a halt (see g_imu_sensor.h for why that
  // matters on this board).
  if (myIMU.begin() != 0) {
    return false;
  }

  // Raise the IMU bus above the core's 100kHz default. MUST come after
  // begin(): the library calls Wire1.begin() internally, which hardcodes the
  // TWIM FREQUENCY register, so anything set earlier is silently overwritten.
  // See IMU_I2C_CLOCK_HZ in config.h for why this matters to loop() latency.
  Wire1.setClock(IMU_I2C_CLOCK_HZ);

  // Enable Block Data Update on CTRL3_C: freezes each 16-bit output register
  // between its low- and high-byte reads, so a two-byte fetch can never
  // straddle a sample rollover (torn read). At our 104 Hz ODR samples refresh
  // every ~9.6 ms, well inside the window of any BLE/serial hiccup.
  // Rate, range and LPF1 in one explicit write, replacing whatever the
  // library's begin() put there from its LSM6DS3-shaped settings.
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, kCtrl1Xl);

  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL3_C, kCtrl3C);

  // Confirm the chip took all of it. Nothing above checks anything: the
  // library's begin() returns only its WHO_AM_I result, every settings write
  // inside it is unchecked, writeRegister() is unchecked here, and
  // calcAccel()/calcGyro() scale from the library's own copy of the settings
  // rather than the chip. The failure that matters is specific to this part:
  // CTRL1_XL and CTRL2_G reset to 0x00, which is POWERED DOWN, so a lost write
  // leaves that sensor off while reads keep succeeding and returning zeros.
  //
  // CTRL1_XL is compared WHOLE, because every bit in it is now ours - rate,
  // range and LPF1. CTRL2_G masks off its reserved bit; BDU and IF_INC are
  // checked as bits because the rest of CTRL3_C is not ours. IF_INC is what
  // makes the burst read above walk the registers rather than re-read one. A
  // failed read here leaves the value 0 and fails the comparison too, so this
  // errs toward "not up".
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

#else // IMU_ENABLED == 0

// No IMU fitted (see IMU_ENABLED in config.h). The pipeline reads a failed
// bring-up as "no IMU", says "not fitted", and every IMU field reads zero - the
// same state as an IMU that has died, deliberately, so there is one code path.
bool imuSensorBegin() { return false; }
bool imuSensorRead(ImuRawSample *) { return false; }

#endif // IMU_ENABLED
