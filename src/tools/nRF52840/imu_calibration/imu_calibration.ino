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
// Units match what g_imu.cpp / config.h use:
//   * accel:  g       (LSM6DS3 native, x1000 to milli-g downstream)
//   * gyro:   deg/s   (LSM6DS3 native, x100 to centi-deg/s downstream)
// Accel Z is calibrated with the LED face pointing +Z UP on a level surface,
// so gravity (1.0 g) is subtracted from the average before reporting.
//
// Method (fully hands-off - set the device down level before power-on):
//   1. Warm up the IMU under normal operating settings (thermal stabilization).
//   2. Run calibration sessions forever: average NUM_SAMPLES raw reads per
//      axis at IMU_SAMPLE_INTERVAL_MS pacing, print the six offsets (plus
//      deltas vs the previous session), wait SESSION_GAP_MS, repeat.
//   3. Once consecutive sessions agree, copy the latest offsets into
//      config.h's IMU_*_OFFSET_* defines and reflash the main sketch.
//
// Requires: XIAO nRF52840 Sense; level bench surface; USB for serial. The
// device must be still throughout each sample window (accidental knocks skew
// the accel offsets more than the average can smooth away).
// ============================================================================

#include <Adafruit_TinyUSB.h> // Serial (USB CDC) needs TinyUSB linked
#include <Arduino.h>
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
static const int NUM_SAMPLES = 10000; // ~100 s at 100 Hz - excellent averaging
static const unsigned long SESSION_GAP_MS = 60UL * 1000UL; // between sessions

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
  // trusted to the last mV. Same setting as g_imu.cpp uses in production.
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL3_C,
                      LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE |
                          LSM6DS3_ACC_GYRO_IF_INC_ENABLED);
}

// Session results live at file scope (not passed as parameters) because the
// Arduino preprocessor emits function prototypes above type definitions,
// which breaks user types in signatures.
struct CalOffsets {
  float ax, ay, az;
  float gx, gy, gz;
};
static CalOffsets curOff;
static CalOffsets prevOff;

static void performCalibration(int session) {
  Serial.printf(
      "\n=== Session %d: sampling - keep device perfectly still ===\n",
      session);

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

    if (i > 0 && i % 1000 == 0) {
      Serial.printf(" [%d/%d]", i, NUM_SAMPLES);
    }
  }

  // Averages. Accel Z has gravity subtracted (device is +Z-up on a level
  // surface, so its true value at rest is +1.0 g).
  curOff.ax = (float)(sumAx / NUM_SAMPLES);
  curOff.ay = (float)(sumAy / NUM_SAMPLES);
  curOff.az = (float)(sumAz / NUM_SAMPLES) - 1.0f;
  curOff.gx = (float)(sumGx / NUM_SAMPLES);
  curOff.gy = (float)(sumGy / NUM_SAMPLES);
  curOff.gz = (float)(sumGz / NUM_SAMPLES);
}

static void printResults(int session) {
  Serial.printf("\n\n--- SESSION %d RESULTS ---\n", session);
  Serial.println("Paste these lines into config.h's IMU-offsets section:");
  Serial.println("-------------------------------------------");
  Serial.printf("#define IMU_ACCEL_OFFSET_X_G  %+.6ff\n", curOff.ax);
  Serial.printf("#define IMU_ACCEL_OFFSET_Y_G  %+.6ff\n", curOff.ay);
  Serial.printf("#define IMU_ACCEL_OFFSET_Z_G  %+.6ff\n", curOff.az);
  Serial.printf("#define IMU_GYRO_OFFSET_X_DPS %+.6ff\n", curOff.gx);
  Serial.printf("#define IMU_GYRO_OFFSET_Y_DPS %+.6ff\n", curOff.gy);
  Serial.printf("#define IMU_GYRO_OFFSET_Z_DPS %+.6ff\n", curOff.gz);
  Serial.println("-------------------------------------------");
  if (session > 1) {
    Serial.printf("Delta vs session %d:  accel %+.6f %+.6f %+.6f g\n",
                  session - 1, curOff.ax - prevOff.ax, curOff.ay - prevOff.ay,
                  curOff.az - prevOff.az);
    Serial.printf("                      gyro  %+.6f %+.6f %+.6f dps\n",
                  curOff.gx - prevOff.gx, curOff.gy - prevOff.gy,
                  curOff.gz - prevOff.gz);
  }
  Serial.println(
      "Sanity check: |accel| offsets are typically <0.1 g, gyro <5 deg/s.");
  Serial.println(
      "Values much larger than that suggest the device wasn't level or still.");
}

// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
  }

  Serial.println("=== Gnimu - IMU calibration ===");
  Serial.println(
      "Goal: measure per-axis zero-point offsets (raw sensor frame).");
  Serial.println("Fully hands-off:");
  Serial.println(
      "  1. Device must already be on a level surface, LED face UP (+Z up).");
  Serial.println("  2. Do not touch it from here on.");
  Serial.printf(
      "  3. After a %lu-minute warmup, %d-sample sessions run forever\n",
      WARMUP_MS / 60000UL, NUM_SAMPLES);
  Serial.printf("     with %lu s between them. Power-cycle to stop.\n",
                SESSION_GAP_MS / 1000UL);

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
}

void loop() {
  static int session = 0;

  session++;
  performCalibration(session);
  printResults(session);
  prevOff = curOff;

  Serial.printf("Next session in %lu s...\n", SESSION_GAP_MS / 1000UL);
  delay(SESSION_GAP_MS);
}
