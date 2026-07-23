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
// DIAGNOSTIC: XIAO nRF52840 Sense onboard IMU probe
//
// Validates the assumptions imu.cpp makes about the onboard LSM6DS3TR-C:
//   1. That PIN_LSM6DS3TR_C_POWER (config.h IMU_POWER_PIN) enables the IMU when
//      driven HIGH.
//   2. That the stock Seeed LSM6DS3 library's begin() reaches the IMU on this
//      board out of the box (i.e. whether imu.cpp can use it as-is, or needs a
//      custom bus binding).
//   3. That the library returns accel in g and gyro in deg/s directly - the
//      basis for imu.cpp's simplified unit conversion.
//
// NOTE: this probe does NOT do a manual I2C bus scan. On the nRF52 an idle /
// floating TWI bus hangs endTransmission() on the first address, which stalled
// earlier versions of this sketch (on both Wire and Wire1). The Seeed library is
// matched to this board and knows the IMU's bus, so letting begin() run is the
// real, hang-free bus test.
//
// What to look for on the serial monitor (115200):
//   - "begin() OK" -> the stock library reaches the IMU; imu.cpp works as-is
//     (its bus init should match whatever the library uses).
//     "begin() FAILED (rc=N)" -> library doesn't reach it; we investigate a
//     Wire1 binding. (If it prints "Calling begin()" then hangs, same conclusion
//     - the stock library's default bus is wrong for this board.)
//   - With the board resting flat, one accel axis reads ~+1.0 g (that axis is
//     "up"); previews the axis frame (full remap decided in the final installed
//     orientation - DESIGN.md section 4).
//
// Requires: XIAO nRF52840 Sense + USB only.
// ============================================================================

#include <Arduino.h>

#include "LSM6DS3.h"

static const uint8_t IMU_ADDR = 0x6A; // SA0 tied high; independent of config.h

// Stock Seeed library instance (it brings up its own default bus in begin()).
static LSM6DS3 imu(I2C_MODE, IMU_ADDR);
static bool imuOk = false;

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
  }
  Serial.println("\n=== XIAO IMU probe ===");

#ifdef PIN_LSM6DS3TR_C_POWER
  Serial.printf("PIN_LSM6DS3TR_C_POWER = %d (driving HIGH to enable)\n",
                (int)PIN_LSM6DS3TR_C_POWER);
  pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);
  digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
  delay(300); // let the IMU boot before I2C
#else
  Serial.println("WARNING: PIN_LSM6DS3TR_C_POWER is NOT defined by this variant "
                 "- config.h IMU_POWER_PIN assumption is wrong.");
#endif
  Serial.flush();

  Serial.println("Calling LSM6DS3.begin() ...");
  Serial.flush(); // so this line survives even if begin() were to stall
  status_t rc = imu.begin();
  if (rc != 0) {
    Serial.printf("  begin() FAILED (rc=%d) - IMU not reachable on the "
                  "library's default bus.\n",
                  (int)rc);
  } else {
    imuOk = true;
    Serial.println("  begin() OK - the stock library reaches the IMU. "
                   "Streaming raw readings:");
  }
  Serial.flush();
}

void loop() {
  if (!imuOk) {
    delay(1000);
    return;
  }
  // The Seeed library returns g and deg/s directly - confirm the magnitudes.
  Serial.printf(
      "A[g]  X=%+.3f Y=%+.3f Z=%+.3f   G[dps]  X=%+.2f Y=%+.2f Z=%+.2f\n",
      imu.readFloatAccelX(), imu.readFloatAccelY(), imu.readFloatAccelZ(),
      imu.readFloatGyroX(), imu.readFloatGyroY(), imu.readFloatGyroZ());
  delay(500);
}
