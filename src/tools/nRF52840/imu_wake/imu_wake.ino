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
// TOOL: LIGHT_SLEEP IMU wake-up detector validation
//
// Validates the LSM6DS3TR-C's embedded, omni-directional wake-up ("activity")
// detector for LIGHT_SLEEP's exit trigger (v1 uses omni-directional
// shake-to-wake, not directional tap). The Seeed Arduino LSM6DS3 library
// exposes no high-level API for this, so it's configured via
// direct register writes per ST AN4650 (LSM6DS3 always-on 3D accelerometer
// and 3D gyroscope):
//   https://cdn.sparkfun.com/assets/learn_tutorials/4/1/6/AN4650_DM00157511.pdf
//
// Registers touched (addresses from the Seeed library's LSM6DS3.h, values
// from the AN4650 wake-up-function description):
//   CTRL1_XL   (0x10) - accel ODR + full-scale. A low ODR (12.5 Hz) puts the
//                        accelerometer in the chip's automatic low-power mode.
//   TAP_CFG    (0x58) - bit7 INTERRUPTS_ENABLE, bit0 LIR (latch - so a brief
//                        motion isn't missed by our polling), bits3:1 enable
//                        X/Y/Z in the wake-up slope detector (omnidirectional).
//   WAKE_UP_THS(0x5B) - 6-bit wake-up threshold, LSB weight = FS_XL/64. THE
//                        key tunable - too low = false triggers from vibration/
//                        handling noise, too high = a real pickup/shake misses.
//   WAKE_UP_DUR(0x5C) - debounce duration (in ODR cycles) before the
//                        interrupt asserts. 0 = fire on the first sample over
//                        threshold.
//   MD1_CFG    (0x5E) - bit5 INT1_WU routes the wake-up interrupt to INT1
//                        (PIN_LSM6DS3TR_C_INT1, onboard, no wiring needed).
//   WAKE_UP_SRC(0x1B) - reading this clears the latched interrupt (LIR=1).
//
// CTRL1_XL_VAL's full-scale (FS_XL) should match the RUNNING-mode range
// (IMU_ACCEL_RANGE_G in config.h), not be dropped further for "extra
// low-power" - lesson from g_imu.cpp's imuArmWake(): since that's a live
// mode switch on an already-running chip (not a cold power-on, unlike this
// standalone sketch), changing FS_XL creates a persistent scale mismatch in
// the wake-up detector's raw-code threshold comparison that reliably
// false-triggers immediately - a settle delay does not fix it. Only ODR
// should change between RUNNING and the wake-detect config.
//
// All the tunables above are #defines at the top - start here, watch how
// sensitive it feels, and adjust before porting the final values into
// config.h / g_imu.cpp.
//
// Requires: XIAO nRF52840 Sense + USB only (onboard LSM6DS3TR-C + INT1, no
// wiring needed).
// ============================================================================

#include <Arduino.h>

// USB Serial needs TinyUSB explicitly linked; LSM6DS3.h doesn't pull it in
// transitively (same gotcha as led_check/imu_probe's sibling tools).
#include <Adafruit_TinyUSB.h>
#include <LSM6DS3.h>

static LSM6DS3 myIMU(I2C_MODE, 0x6A); // SA0 tied high on the XIAO Sense

// --- Tunable wake-up parameters ---
static const uint8_t CTRL1_XL_VAL = 0x18;   // ODR=12.5Hz (low-power), FS=+/-4g
static const uint8_t WAKE_UP_THS_VAL = 4;   // 6-bit threshold (0-63) - TUNE ME
static const uint8_t WAKE_UP_DUR_VAL = 0;   // debounce, in ODR cycles - TUNE ME

static const unsigned long STATUS_INTERVAL_MS = 2000; // "still idle" heartbeat
static unsigned long lastStatusMs = 0;

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
  }

  Serial.println("\n=== LIGHT_SLEEP IMU wake-up detector validation ===");

  pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);
  digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
  delay(300); // boot settle, bench-confirmed via imu_probe

  pinMode(PIN_LSM6DS3TR_C_INT1, INPUT); // push-pull active-high chip output

  // Bring the library up first (I2C, WHO_AM_I check) - gyro left disabled,
  // matching the real design (accel-only wake sensor; gyro stays off in
  // LIGHT_SLEEP). accelRange must match CTRL1_XL_VAL's FS_XL below so the
  // library's readFloatAccelX/Y/Z() scaling stays correct for our debug
  // prints.
  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = 4; // +/- 4g, matches CTRL1_XL_VAL FS_XL bits
  myIMU.settings.accelSampleRate = 104;
  myIMU.settings.gyroEnabled = 0;
  myIMU.settings.tempEnabled = 0;

  if (myIMU.begin() != 0) {
    Serial.println("FAILED to find IMU - halting.");
    while (1)
      delay(100);
  }

  // --- Configure the wake-up detector directly (AN4650) ---
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, CTRL1_XL_VAL);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1,
                      0x8F); // INTERRUPTS_ENABLE|LIR|X_EN|Y_EN|Z_EN
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, WAKE_UP_THS_VAL & 0x3F);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_DUR, WAKE_UP_DUR_VAL);
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x20); // INT1_WU

  Serial.printf("CTRL1_XL=0x%02X WAKE_UP_THS=%d WAKE_UP_DUR=%d\n",
                CTRL1_XL_VAL, WAKE_UP_THS_VAL, WAKE_UP_DUR_VAL);
  Serial.println("Wake-up detector armed. Let the board sit still, then "
                 "pick it up / tap / shake it - watch for WAKE EVENT below.");
}

void loop() {
  static bool lastInt1 = false;
  bool int1 = digitalRead(PIN_LSM6DS3TR_C_INT1);

  if (int1 && !lastInt1) {
    uint8_t src = 0;
    myIMU.readRegister(&src, LSM6DS3_ACC_GYRO_WAKE_UP_SRC); // clears latch
    // WAKE_UP_SRC bits: 0=Z_WU 1=Y_WU 2=X_WU 3=WU_IA (wake event flag)
    Serial.printf("[%lums] WAKE EVENT - WAKE_UP_SRC=0x%02X (WU_IA=%d X=%d "
                  "Y=%d Z=%d)\n",
                  millis(), src, (src >> 3) & 0x1, (src >> 2) & 0x1,
                  (src >> 1) & 0x1, src & 0x1);
    lastStatusMs = millis(); // don't immediately print "still idle" right after
  }
  lastInt1 = int1;

  unsigned long now = millis();
  if (now - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = now;
    Serial.printf("  [%lums] still idle (accel x=%.2f y=%.2f z=%.2f g)\n", now,
                  myIMU.readFloatAccelX(), myIMU.readFloatAccelY(),
                  myIMU.readFloatAccelZ());
  }

  delay(20); // light polling cadence - this is a tuning tool, not the real loop
}
