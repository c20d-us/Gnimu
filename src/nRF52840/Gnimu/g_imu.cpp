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

#include "g_imu.h"
#include "ImuAxis.h"
#include "config.h"
#include "g_log.h"
#include <LSM6DS3.h>

// IMU driver. I2C at IMU_I2C_ADDRESS (0x6A).
static LSM6DS3 myIMU(I2C_MODE, IMU_I2C_ADDRESS);

// Three axes each for accelerometer and gyroscope, indexed [0]=X, [1]=Y,
// [2]=Z. Arrays let imuBegin()/imuPoll() drive all three axes of a sensor
// with one loop instead of one line per axis.
static ImuAxis accelAxes[3] = {
    ImuAxis(IMU_ACCEL_ALPHA, IMU_ACCEL_TRANSIENT_THRESHOLD_G),
    ImuAxis(IMU_ACCEL_ALPHA, IMU_ACCEL_TRANSIENT_THRESHOLD_G),
    ImuAxis(IMU_ACCEL_ALPHA, IMU_ACCEL_TRANSIENT_THRESHOLD_G),
};
static ImuAxis gyroAxes[3] = {
    ImuAxis(IMU_GYRO_ALPHA, IMU_GYRO_TRANSIENT_THRESHOLD_DPS),
    ImuAxis(IMU_GYRO_ALPHA, IMU_GYRO_TRANSIENT_THRESHOLD_DPS),
    ImuAxis(IMU_GYRO_ALPHA, IMU_GYRO_TRANSIENT_THRESHOLD_DPS),
};

// Latest values decimated to the transmission rate, refreshed by imuPoll().
// Kept as a plain cache so imuReadProtocolUnits() stays a cheap, side-effect
// free accessor safe to call from multiple places (BLE packet send and serial
// debug reporting both read it).
static ImuProtocolUnits latestUnits = {0, 0, 0, 0, 0, 0};

// A single raw IMU sample, split into per-axis arrays indexed [0]=X, [1]=Y,
// [2]=Z, matching accelAxes/gyroAxes.
struct ImuRawSample {
  float accel[3]; // g (LSM6DS3 native units)
  float gyro[3];  // deg/s (LSM6DS3 native units)
};

// ============================================================================
// Axis remap - installed orientation. Maps the sensor frame into the vehicle
// frame; the mounting correction itself (IMU_SWAP_XY, IMU_SIGN_X/Y/Z) lives
// in config.h - see the comment there for its flat-mount-only scope.
// ============================================================================
static void remapAxes(float triple[3]) {
  if (IMU_SWAP_XY) {
    float t = triple[0];
    triple[0] = triple[1];
    triple[1] = t;
  }
  triple[0] *= IMU_SIGN_X;
  triple[1] *= IMU_SIGN_Y;
  triple[2] *= IMU_SIGN_Z;
}

// Convert a scaled sensor value (gyro in centi-deg/sec, accel in milli-g) to
// the protocol's int16_t, saturating at the representable ±32767 limit rather
// than overflowing. A wrapped overflow would flip sign (i.e., reporting a hard
// left spin as a right one), so we clamp to the extreme. The gyro can exceed
// range at ±500°/s; the accelerometer stays well within it even at the max ±g
// but goes through here too so all six fields follow one consistent, safe
// pattern.
static int16_t toProtocolInt16(float value) {
  if (value > 32767.0f)
    return 32767;
  if (value < -32768.0f)
    return -32768;
  return (int16_t)value;
}

// Read the IMU and return the raw accel (g) / gyro (deg/s) values for this
// instant, zero-corrected and remapped into the vehicle frame.
//
// Order matters: per-chip zero-point offsets (IMU_*_OFFSET_*) are subtracted
// FIRST, in the sensor's raw axis frame. That keeps the offsets intrinsic to
// the chip so they don't need to change if the mounting orientation flips
// IMU_SWAP_XY / IMU_SIGN_*. The mounting remap runs AFTER the correction.
static ImuRawSample readImuRaw() {
  ImuRawSample s;
  s.accel[0] = myIMU.readFloatAccelX() - IMU_ACCEL_OFFSET_X_G;
  s.accel[1] = myIMU.readFloatAccelY() - IMU_ACCEL_OFFSET_Y_G;
  s.accel[2] = myIMU.readFloatAccelZ() - IMU_ACCEL_OFFSET_Z_G;
  s.gyro[0] = myIMU.readFloatGyroX() - IMU_GYRO_OFFSET_X_DPS;
  s.gyro[1] = myIMU.readFloatGyroY() - IMU_GYRO_OFFSET_Y_DPS;
  s.gyro[2] = myIMU.readFloatGyroZ() - IMU_GYRO_OFFSET_Z_DPS;
  remapAxes(s.accel);
  remapAxes(s.gyro);
  return s;
}

// Shared by imuBegin() and imuDisarmWake(): (re)apply normal-operation
// ranges/ODR/bandwidth, BDU, and re-seed the axis filters. Does not touch
// the power pin as callers that need the chip powered on do that themselves.
static void configureNormalMode() {
  // Configure ranges / output data rates / bandwidth from config.h. The Seeed
  // library takes plain integers here and applies them in begin(); it also
  // brings up its own I2C bus internally - no manual Wire/Wire1 setup needed.
  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = IMU_ACCEL_RANGE_G;
  myIMU.settings.accelSampleRate = IMU_ACCEL_ODR_HZ;
  myIMU.settings.accelBandWidth = IMU_ACCEL_BANDWIDTH_HZ; // anti-aliasing
  myIMU.settings.gyroEnabled = 1;
  myIMU.settings.gyroRange = IMU_GYRO_RANGE_DPS;
  myIMU.settings.gyroSampleRate = IMU_GYRO_ODR_HZ;
  myIMU.settings.tempEnabled = 0; // unused by this firmware

  // The Seeed library returns 0 (IMU_SUCCESS) on success.
  if (myIMU.begin() != 0) {
    LOG_PRINTLN("❌ Failed to find IMU module - halting");
    while (1)
      delay(100);
  }

  // Enable Block Data Update on CTRL3_C: freezes each 16-bit output register
  // between its low- and high-byte reads, so a two-byte fetch can never
  // straddle a sample rollover (torn read). At our 104 Hz ODR samples refresh
  // every ~9.6 ms, well inside the window of any BLE/serial hiccup.
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL3_C,
                      LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE |
                          LSM6DS3_ACC_GYRO_IF_INC_ENABLED);

  // Fully disable the wake-up detector, not just un-route it (harmless no-op
  // on the very first call, before it's ever been armed). Un-routing MD1_CFG
  // alone leaves TAP_CFG1's INTERRUPTS_ENABLE bit live, so the slope detector
  // keeps running continuously in the background for the entire RUNNING
  // period, and since LIR latches it, any real motion during that window
  // (device handled, table bumped) sets WAKE_UP_SRC well before the next
  // imuArmWake() ever runs.
  // Clearing TAP_CFG1 here gives every imuArmWake() the same clean 0x00 ->
  // 0x8F transition that worked on the first cycle.
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x00);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x00);

  // Seed each axis with a real first reading rather than leaving it at its
  // zero-baseline default. Otherwise the first update() would see a huge
  // artificial jump (e.g. gravity on the Z axis) that gets latched into the
  // window's peak deviation and misreported as a genuine transient in the
  // very first transmitted frame.
  ImuRawSample seed = readImuRaw();
  for (int i = 0; i < 3; i++) {
    accelAxes[i].reset(seed.accel[i]);
    gyroAxes[i].reset(seed.gyro[i]);
  }
}

void imuBegin() {
  // Power the onboard LSM6DS3TR-C, then give it time to boot before I2C. Pin
  // 15 driven HIGH enables it.
  pinMode(IMU_POWER_PIN, OUTPUT);
  digitalWrite(IMU_POWER_PIN, HIGH);
  delay(300); // wait for IMU to boot and settle

  configureNormalMode();
  LOG_PRINTLN("✅ IMU Accelerometer/Gyro enabled.");
}

// LIGHT_SLEEP wake-up detector (AN4650)
// Registers touched, beyond the ones configureNormalMode() already owns:
//   TAP_CFG (0x58)     - bit7 INTERRUPTS_ENABLE, bit0 LIR (latch), bits3:1
//                        enable X/Y/Z in the wake-up slope detector.
//   WAKE_UP_THS (0x5B) - 6-bit threshold (IMU_WAKE_THS).
//   WAKE_UP_DUR (0x5C) - debounce, in ODR cycles (IMU_WAKE_DUR).
//   MD1_CFG (0x5E)     - bit5 INT1_WU routes the wake-up interrupt to INT1.
//   WAKE_UP_SRC (0x1B) - reading this clears the latched interrupt.
static bool wakeArmed = false;

void imuArmWake() {
  pinMode(IMU_INT1_PIN, INPUT); // push-pull active-high chip output

  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, IMU_WAKE_CTRL1_XL);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1,
                      0x8F); // INTERRUPTS_ENABLE|LIR|X_EN|Y_EN|Z_EN
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, IMU_WAKE_THS & 0x3F);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_DUR, IMU_WAKE_DUR);
  // MD1_CFG intentionally NOT written yet - stay un-routed from INT1 until
  // WAKE_UP_SRC has been flushed below, so nothing stale (or freshly
  // generated by the CTRL1_XL switch itself) can reach the pin before we've
  // had a chance to clear it.

  // The CTRL1_XL write above is a live mode switch (104Hz/+-4g while RUNNING
  // -> 12.5Hz/+-2g here), not a cold power-on into this config - unlike
  // src/tools/imu_wake, which only ever powered on directly into the wake
  // config. Changing ODR/full-scale on an already-running chip produces a
  // settling transient in the accelerometer's internal filter chain that's
  // large enough to trip WAKE_UP_THS on its own. Give it a couple of ODR
  // periods (12.5Hz -> 80ms/sample) to settle, then discard whatever that
  // transient (or anything stale left over from the last cycle) latched
  // into WAKE_UP_SRC before routing to INT1 - a single read fully clears the
  // latch (it's not a FIFO), but only if nothing's already been routed out
  // to the pin first.
  delay(250);
  uint8_t discard = 0;
  myIMU.readRegister(&discard, LSM6DS3_ACC_GYRO_WAKE_UP_SRC);

  myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x20); // INT1_WU - now safe

  wakeArmed = true;
  LOG_PRINTLN("🌙 IMU wake-up detector armed (low-power ODR).");
}

bool imuWakeTriggered() {
  if (!wakeArmed)
    return false;
  if (digitalRead(IMU_INT1_PIN) != HIGH)
    return false;

  uint8_t src = 0;
  myIMU.readRegister(&src, LSM6DS3_ACC_GYRO_WAKE_UP_SRC); // clears the latch
  return true;
}

void imuDisarmWake() {
  wakeArmed = false;
  configureNormalMode(); // restores ODR/range and un-routes MD1_CFG
  LOG_PRINTLN("☀️ IMU wake-up detector disarmed, normal ODR restored.");
}

// Poll the IMU and update the axis filters at our configured sample rate.
// Both cadences below are deadline-anchored (the anchor advances by the
// interval, not to "now"), so per-loop latency doesn't stretch every period
// and quietly drop the real rates below their configured values. If the loop
// ever falls more than one full interval behind, the anchor resyncs to now
// rather than firing a rapid catch-up burst.
void imuPoll() {
  static unsigned long lastImuReadMs = 0;
  static unsigned long lastTransmitReadMs = 0;
  const unsigned long nowMs = millis();

  // Update all six axis filters if it's time to sample.
  if (nowMs - lastImuReadMs >= IMU_SAMPLE_INTERVAL_MS) {
    lastImuReadMs += IMU_SAMPLE_INTERVAL_MS;
    if (nowMs - lastImuReadMs >= IMU_SAMPLE_INTERVAL_MS) {
      lastImuReadMs = nowMs; // fell > 1 interval behind - resync
    }
    ImuRawSample raw = readImuRaw();
    for (int i = 0; i < 3; i++) {
      accelAxes[i].update(raw.accel[i]);
      gyroAxes[i].update(raw.gyro[i]);
    }
  }

  // Decimate to the transmission rate on a fixed cadence, regardless of BLE
  // connection state. This is what drains each axis's transient window; if it
  // only ran while connected, the window would silently accumulate deviations
  // for as long as the device stayed disconnected and dump a stale "peak"
  // into the first packet after reconnecting.
  if (nowMs - lastTransmitReadMs >= IMU_TRANSMIT_INTERVAL_MS) {
    lastTransmitReadMs += IMU_TRANSMIT_INTERVAL_MS;
    if (nowMs - lastTransmitReadMs >= IMU_TRANSMIT_INTERVAL_MS) {
      lastTransmitReadMs = nowMs; // fell > 1 interval behind - resync
    }

    float accelFiltered[3], gyroFiltered[3];
    for (int i = 0; i < 3; i++) {
      accelFiltered[i] = accelAxes[i].read();
      gyroFiltered[i] = gyroAxes[i].read();
    }

    // LSM6DS3 returns g and deg/s directly, so the protocol conversion is
    // just a scale into milli-g and centi-deg/sec respectively.
    latestUnits.gX = toProtocolInt16(accelFiltered[0] * 1000.0f);
    latestUnits.gY = toProtocolInt16(accelFiltered[1] * 1000.0f);
    latestUnits.gZ = toProtocolInt16(accelFiltered[2] * 1000.0f);
    latestUnits.rX = toProtocolInt16(gyroFiltered[0] * 100.0f);
    latestUnits.rY = toProtocolInt16(gyroFiltered[1] * 100.0f);
    latestUnits.rZ = toProtocolInt16(gyroFiltered[2] * 100.0f);
  }
}

ImuProtocolUnits imuReadProtocolUnits() { return latestUnits; }
