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
// DIAGNOSTIC: XIAO nRF52840 Sense IMU axis tilt-map (onboard IMU, USB only)
//
// Maps the LSM6DS3TR-C sensor axes to the physical board so the axis remap in
// config.h (IMU_AXIS_{X,Y,Z}_SRC / _SIGN) can be filled in. The sensor's
// orientation is fixed by the XIAO's design, so what you learn here carries to
// any XIAO nRF52840 Sense - no wiring or soldering, board stays as-is.
//
// USUALLY YOU DO NOT NEED THIS SKETCH. The production firmware already prints
// the 1 Hz serial milliG line, and the three static poses documented in
// config.h's Axis orientation section derive the whole map from it - which is
// how the shipped maps were actually settled. Reach for this sketch when a
// board's sensor orientation is unknown from scratch. Either way, derive
// against the RAW SERIAL NUMBERS, not the Gnimu Monitor readout: Monitor is a
// display layer that has masked a mirrored map here before.
//
// The accelerometer at rest reads the specific force (reaction to gravity), so
// the axis pointing UP reads about +1 g and the axis pointing DOWN reads -1 g.
// Hold the board still in a pose; this prints ONE line naming the up-axis. Move
// it and hold a new pose for the next line.
//
// How to use (results feed imu.cpp - DESIGN.md section 4):
//   1. Board FLAT, component/LED side UP (the installed under-the-lid pose).
//      Expect "UP = +Z" -> confirms IMU_AXIS_Z_SRC = 2, IMU_AXIS_Z_SIGN = +1.
//   2. Stand it on each edge in turn (USB-C edge down, opposite edge down, each
//      long edge down). The axis that reads +-1 g there is the in-plane axis
//      aligned with that edge - that's your X vs Y and their signs relative to
//      the board. (Which in-plane axis becomes vehicle-forward vs. lateral, and
//      the final sign, still needs the in-car drive test.)
//   Record: for each pose, the printed "UP = <axis>" and the raw X/Y/Z.
//
// Requires: XIAO nRF52840 Sense + USB only.
// ============================================================================

#include <Arduino.h>

#include "LSM6DS3.h"

static const uint8_t IMU_ADDR = 0x6A; // SA0 tied high; independent of config.h
static LSM6DS3 imu(I2C_MODE, IMU_ADDR);
static bool imuOk = false;

// Motion / stability detection (values in g).
static const float MOVE_THRESH = 0.04f; // per-axis change that counts as moving
static const int STABLE_SAMPLES = 6;    // ~0.6 s still (100 ms/sample) to latch

// Name the dominant (largest-magnitude) axis and its sign - the one pointing up.
static const char *upAxis(float x, float y, float z) {
  float ax = fabsf(x), ay = fabsf(y), az = fabsf(z);
  if (az >= ax && az >= ay)
    return z >= 0 ? "+Z" : "-Z";
  if (ay >= ax)
    return y >= 0 ? "+Y" : "-Y";
  return x >= 0 ? "+X" : "-X";
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
  }
  Serial.println("\n=== XIAO IMU axis tilt-map ===");

#ifdef PIN_LSM6DS3TR_C_POWER
  pinMode(PIN_LSM6DS3TR_C_POWER, OUTPUT);
  digitalWrite(PIN_LSM6DS3TR_C_POWER, HIGH);
  delay(300);
#endif

  if (imu.begin() != 0) {
    Serial.println("begin() FAILED - IMU not reachable.");
    return;
  }
  imuOk = true;
  Serial.println("Hold the board still in a pose; a line prints when it settles.");
  Serial.println("Start with it FLAT, component/LED side UP (expect UP = +Z).");
}

void loop() {
  if (!imuOk) {
    delay(1000);
    return;
  }

  static float px = 0, py = 0, pz = 0;
  static int stableCount = 0;
  static bool reported = false;

  float x = imu.readFloatAccelX();
  float y = imu.readFloatAccelY();
  float z = imu.readFloatAccelZ();

  float delta = fmaxf(fmaxf(fabsf(x - px), fabsf(y - py)), fabsf(z - pz));
  px = x;
  py = y;
  pz = z;

  if (delta > MOVE_THRESH) {
    stableCount = 0;   // still moving
    reported = false;  // arm a new report for the next pose
  } else if (stableCount < STABLE_SAMPLES) {
    stableCount++;
  }

  // Latch one line per settled pose.
  if (stableCount >= STABLE_SAMPLES && !reported) {
    float mag = sqrtf(x * x + y * y + z * z);
    Serial.printf("STABLE  ->  UP = %s   (X=%+.2f  Y=%+.2f  Z=%+.2f g,  |g|=%.2f)\n",
                  upAxis(x, y, z), x, y, z, mag);
    reported = true;
  }

  delay(100);
}
