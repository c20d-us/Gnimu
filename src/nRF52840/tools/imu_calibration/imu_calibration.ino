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
// DIAGNOSTIC: bench IMU calibration
//
// Measures per-axis zero-point offsets for the onboard LSM6DS3TR-C in the
// sensor's RAW axis frame (before any axis remap), and prints six
// paste-ready #define lines for config.h. Independent of IMU_SWAP_XY /
// IMU_SIGN_* - the offsets stay valid across mounting changes.
//
// Units match what gp_imu.cpp / config.h use:
//   * accel:  g       (LSM6DS3 native, x1000 to milli-g downstream)
//   * gyro:   deg/s   (LSM6DS3 native, x100 to centi-deg/s downstream)
// Accel Z is calibrated with the LED face pointing +Z UP on a level surface,
// so gravity (1.0 g) is subtracted from the average before reporting.
//
// Method:
//   1. Warm up the IMU under normal operating settings (thermal stabilization).
//   2. Wait for the user to type 'c' when the device is confirmed still + level.
//   3. Average NUM_SAMPLES raw reads per axis at IMU_SAMPLE_INTERVAL_MS pacing.
//   4. Print the six offsets. Copy them into config.h's IMU_*_OFFSET_* defines
//      and reflash the main sketch.
//
// Requires: XIAO nRF52840 Sense; level bench surface; USB for serial. The
// device must be still throughout the sample window (accidental knocks skew
// the accel offsets more than the average can smooth away).
// ============================================================================

#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // Serial (USB CDC) needs TinyUSB linked
#include <LSM6DS3.h>

// ----------------------------------------------------------------------------
// Keep IMU settings in sync with config.h so calibration matches operating
// conditions (offsets drift slightly with range/ODR/bandwidth).
// ----------------------------------------------------------------------------
#define IMU_POWER_PIN PIN_LSM6DS3TR_C_POWER
#define IMU_I2C_ADDRESS 0x6A
#define ACCEL_RANGE_G 4
#define GYRO_RANGE_DPS 500
#define IMU_ACCEL_ODR_HZ 104
#define IMU_GYRO_ODR_HZ 104
#define IMU_ACCEL_BANDWIDTH_HZ 50
#define IMU_SAMPLE_INTERVAL_MS 10 // 100 Hz pacing between samples

// ----------------------------------------------------------------------------
// Tunables.
// ----------------------------------------------------------------------------
// Warmup lets the die reach thermal steady state before we take the reference
// measurement. The MPU6050 was famously drifty and used ~20 min; the LSM6DS3
// is far better in that regard but a few minutes is still cheap insurance.
static const unsigned long WARMUP_MS = 5UL * 60UL * 1000UL; // 5 minutes
static const int NUM_SAMPLES = 5000; // ~50 s at 100 Hz - excellent averaging

// LSM6DS3 driver on the shared I2C bus.
static LSM6DS3 myIMU(I2C_MODE, IMU_I2C_ADDRESS);

// ----------------------------------------------------------------------------
// Helpers.
// ----------------------------------------------------------------------------
static void imuBringUp() {
  pinMode(IMU_POWER_PIN, OUTPUT);
  digitalWrite(IMU_POWER_PIN, HIGH);
  delay(300); // datasheet-safe boot delay before I2C

  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = ACCEL_RANGE_G;
  myIMU.settings.accelSampleRate = IMU_ACCEL_ODR_HZ;
  myIMU.settings.accelBandWidth = IMU_ACCEL_BANDWIDTH_HZ;
  myIMU.settings.gyroEnabled = 1;
  myIMU.settings.gyroRange = GYRO_RANGE_DPS;
  myIMU.settings.gyroSampleRate = IMU_GYRO_ODR_HZ;
  myIMU.settings.tempEnabled = 0;

  if (myIMU.begin() != 0) {
    Serial.println("❌ Failed to find IMU - halting");
    while (1) {
      delay(100);
    }
  }

  // BDU on: prevents torn 16-bit reads from skewing an offset that has to be
  // trusted to the last mV. Same setting as gp_imu.cpp uses in production.
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL3_C,
                      LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE |
                          LSM6DS3_ACC_GYRO_IF_INC_ENABLED);
}

static void performCalibration() {
  Serial.println("\nStarting calibration - keep device perfectly still...");

  double sumAx = 0, sumAy = 0, sumAz = 0;
  double sumGx = 0, sumGy = 0, sumGz = 0;

  unsigned long nextSampleMs = millis();
  for (int i = 0; i < NUM_SAMPLES; i++) {
    while ((int32_t)(millis() - nextSampleMs) < 0) {
      // wait for the next sample tick
    }
    nextSampleMs += IMU_SAMPLE_INTERVAL_MS;

    sumAx += myIMU.readFloatAccelX();
    sumAy += myIMU.readFloatAccelY();
    sumAz += myIMU.readFloatAccelZ();
    sumGx += myIMU.readFloatGyroX();
    sumGy += myIMU.readFloatGyroY();
    sumGz += myIMU.readFloatGyroZ();

    if (i > 0 && i % 500 == 0) {
      Serial.printf(" [%d/%d]", i, NUM_SAMPLES);
    }
  }

  // Averages. Accel Z has gravity subtracted (device is +Z-up on a level
  // surface, so its true value at rest is +1.0 g).
  const float offAx = (float)(sumAx / NUM_SAMPLES);
  const float offAy = (float)(sumAy / NUM_SAMPLES);
  const float offAz = (float)(sumAz / NUM_SAMPLES) - 1.0f;
  const float offGx = (float)(sumGx / NUM_SAMPLES);
  const float offGy = (float)(sumGy / NUM_SAMPLES);
  const float offGz = (float)(sumGz / NUM_SAMPLES);

  Serial.println("\n\n--- CALIBRATION RESULTS ---");
  Serial.println("Paste these lines into config.h's IMU-offsets section:");
  Serial.println("-------------------------------------------");
  Serial.printf("#define IMU_ACCEL_OFFSET_X_G  %+.6ff\n", offAx);
  Serial.printf("#define IMU_ACCEL_OFFSET_Y_G  %+.6ff\n", offAy);
  Serial.printf("#define IMU_ACCEL_OFFSET_Z_G  %+.6ff\n", offAz);
  Serial.printf("#define IMU_GYRO_OFFSET_X_DPS %+.6ff\n", offGx);
  Serial.printf("#define IMU_GYRO_OFFSET_Y_DPS %+.6ff\n", offGy);
  Serial.printf("#define IMU_GYRO_OFFSET_Z_DPS %+.6ff\n", offGz);
  Serial.println("-------------------------------------------");
  Serial.println("Sanity check: |accel| offsets are typically <0.1 g, gyro <5 deg/s.");
  Serial.println("Values much larger than that suggest the device wasn't level or still.");
  Serial.println("System halted. Power-cycle to re-run.");

  while (1) {
    delay(100);
  }
}

// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
  }

  Serial.println("=== Gnimu - IMU calibration ===");
  Serial.println("Goal: measure per-axis zero-point offsets (raw sensor frame).");
  Serial.println("Setup:");
  Serial.println("  1. Mount the device on a level surface, LED face UP (+Z up).");
  Serial.println("  2. Do not touch it once warmup starts.");
  Serial.printf("  3. Wait for the %lu-minute warmup to complete.\n",
                WARMUP_MS / 60000UL);
  Serial.println("  4. Confirm device is still + level, then type 'c'.");

  imuBringUp();
  Serial.println("✅ IMU up.");

  Serial.print("Warming up:");
  const unsigned long start = millis();
  unsigned long lastMinLogged = 0;
  while (millis() - start < WARMUP_MS) {
    const unsigned long minsElapsed = (millis() - start) / 60000UL;
    if (minsElapsed > lastMinLogged) {
      lastMinLogged = minsElapsed;
      Serial.printf(" %lum", minsElapsed);
    }
    delay(500);
  }
  Serial.println("\nWarmup complete.");
  Serial.println("Confirm the device is still + level, then type 'c' to start.");
}

void loop() {
  if (Serial.available() > 0) {
    const char c = Serial.read();
    if (c == 'c' || c == 'C') {
      performCalibration();
    }
  }
}
