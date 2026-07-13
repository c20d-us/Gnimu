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

#include "gc_imu.h"
#include "config.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- IMU ---
static Adafruit_MPU6050 myIMU;

// Storage for the filtered values
static float filteredAx = 0, filteredAy = 0, filteredAz = 0;
static float filteredGx = 0, filteredGy = 0, filteredGz = 0;

// Convert a scaled sensor value (gyro in centi-deg/sec, accel in milli-g) to
// the protocol's int16_t, saturating at the representable ±32767 limit rather
// than overflowing. A wrapped overflow would flip sign (i.e., reporting a hard
// left spin as a right one), so we clamp to the extreme. The gyro can exceed
// range at ±500 °/s; the accelerometer stays well within it even at the max ±g
// but goes through here too so all six fields follow one consistent, safe
// pattern.
static int16_t toProtocolInt16(double value) {
  if (value > 32767.0)
    return 32767;
  if (value < -32768.0)
    return -32768;
  return (int16_t)value;
}

void imuBegin() {
  if (!myIMU.begin()) {
    Serial.println("❌ Failed to find the IMU module - halting");
    while (1)
      delay(100);
  }

  // IMU started, proceed with configuration
  Serial.println("✅ IMU Accelerometer/Gyro enabled.");
  myIMU.setAccelerometerRange(ACCEL_RANGE);
  myIMU.setGyroRange(GYRO_RANGE);
  myIMU.setFilterBandwidth(FILTER_BANDWIDTH);
  sensors_event_t a, g, temp;
  myIMU.getEvent(&a, &g, &temp);

  // Initialize filters with the first real reading so they don't start at zero
  filteredAx = a.acceleration.x;
  filteredAy = a.acceleration.y;
  filteredAz = a.acceleration.z;
  filteredGx = g.gyro.x;
  filteredGy = g.gyro.y;
  filteredGz = g.gyro.z;
}

void imuPoll() {
  static unsigned long lastAccelReadMs = 0;
  // Update Accelerometer readings at fixed interval
  if (millis() - lastAccelReadMs >= ACCEL_SAMPLE_INTERVAL_MS) {
    lastAccelReadMs = millis();
    sensors_event_t a, g, temp;
    myIMU.getEvent(&a, &g, &temp);

    // Apply Exponential Moving Average filter
    filteredAx =
        (ACCEL_ALPHA * a.acceleration.x) + ((1.0 - ACCEL_ALPHA) * filteredAx);
    filteredAy =
        (ACCEL_ALPHA * a.acceleration.y) + ((1.0 - ACCEL_ALPHA) * filteredAy);
    filteredAz =
        (ACCEL_ALPHA * a.acceleration.z) + ((1.0 - ACCEL_ALPHA) * filteredAz);

    filteredGx = (GYRO_ALPHA * g.gyro.x) + ((1.0 - GYRO_ALPHA) * filteredGx);
    filteredGy = (GYRO_ALPHA * g.gyro.y) + ((1.0 - GYRO_ALPHA) * filteredGy);
    filteredGz = (GYRO_ALPHA * g.gyro.z) + ((1.0 - GYRO_ALPHA) * filteredGz);
  }
}

ImuProtocolUnits imuReadProtocolUnits() {
  ImuProtocolUnits u;
  // Convert accelerometer to milli-g (1g = 9.80665 m/s^2)
  u.gX = toProtocolInt16(filteredAx * 1000.0 / 9.80665);
  u.gY = toProtocolInt16(filteredAy * 1000.0 / 9.80665);
  u.gZ = toProtocolInt16(filteredAz * 1000.0 / 9.80665);
  // Convert gyro to centi-deg/sec
  u.rX = toProtocolInt16(filteredGx * 180.0 / M_PI * 100.0);
  u.rY = toProtocolInt16(filteredGy * 180.0 / M_PI * 100.0);
  u.rZ = toProtocolInt16(filteredGz * 180.0 / M_PI * 100.0);
  return u;
}
