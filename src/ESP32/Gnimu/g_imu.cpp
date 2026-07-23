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
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// IMU
static Adafruit_MPU6050 myIMU;

// Three axes each for accelerometer and gyroscope, indexed [0]=X, [1]=Y,
// [2]=Z. Arrays let imuBegin()/imuPoll() drive all three axes of a sensor
// with one loop instead of one line per axis.
static ImuAxis accelAxes[3] = {
    ImuAxis(IMU_ACCEL_ALPHA, IMU_ACCEL_TRANSIENT_THRESHOLD_MPS2),
    ImuAxis(IMU_ACCEL_ALPHA, IMU_ACCEL_TRANSIENT_THRESHOLD_MPS2),
    ImuAxis(IMU_ACCEL_ALPHA, IMU_ACCEL_TRANSIENT_THRESHOLD_MPS2),
};
static ImuAxis gyroAxes[3] = {
    ImuAxis(IMU_GYRO_ALPHA, IMU_GYRO_TRANSIENT_THRESHOLD_RADPS),
    ImuAxis(IMU_GYRO_ALPHA, IMU_GYRO_TRANSIENT_THRESHOLD_RADPS),
    ImuAxis(IMU_GYRO_ALPHA, IMU_GYRO_TRANSIENT_THRESHOLD_RADPS),
};

// Latest values decimated to the transmission rate, refreshed by imuPoll().
// Kept as a plain cache so imuReadProtocolUnits() stays a cheap, side-effect
// free accessor safe to call from multiple places (BLE packet send and serial
// debug reporting both read it today).
static ImuProtocolUnits latestUnits = {0, 0, 0, 0, 0, 0};

// A single raw IMU sample, split into per-axis arrays indexed [0]=X, [1]=Y,
// [2]=Z, matching accelAxes/gyroAxes.
struct ImuRawSample {
  float accel[3];
  float gyro[3];
};

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

// Read the IMU and return the raw accel (m/s^2) / gyro (rad/s) values for this
// instant, with each axis's per-chip zero-point offset (IMU_ACCEL/
// IMU_GYRO_OFFSET_*, from config.h) subtracted so the bias is removed before
// any smoothing or protocol scaling. Offsets are in the same native units as
// the readings.
static ImuRawSample readImuRaw() {
  sensors_event_t a, g, temp;
  myIMU.getEvent(&a, &g, &temp);
  return {
      {a.acceleration.x - IMU_ACCEL_OFFSET_X_MPS2,
       a.acceleration.y - IMU_ACCEL_OFFSET_Y_MPS2,
       a.acceleration.z - IMU_ACCEL_OFFSET_Z_MPS2},
      {g.gyro.x - IMU_GYRO_OFFSET_X_RADPS, g.gyro.y - IMU_GYRO_OFFSET_Y_RADPS,
       g.gyro.z - IMU_GYRO_OFFSET_Z_RADPS},
  };
}

// Initialize the IMU, including setting up the sensor ranges and seed values.
void imuBegin() {
  if (!myIMU.begin()) {
    Serial.println("❌ Failed to find the IMU module - halting");
    while (1)
      delay(100);
  }

  // IMU started, proceed with configuration
  Serial.println("✅ IMU Accelerometer/Gyro enabled.");
  myIMU.setAccelerometerRange(IMU_ACCEL_RANGE_G);
  myIMU.setGyroRange(IMU_GYRO_RANGE_DPS);
  myIMU.setFilterBandwidth(IMU_FILTER_BANDWIDTH_HZ);

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

// Poll the IMU and update the axis filters at our configured sample rate.
void imuPoll() {
  static unsigned long lastImuReadMs = 0;
  static unsigned long lastTransmitReadMs = 0;

  // Update all six axis filters if it's time to sample.
  if (millis() - lastImuReadMs >= IMU_SAMPLE_INTERVAL_MS) {
    lastImuReadMs = millis();
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
  if (millis() - lastTransmitReadMs >= IMU_TRANSMIT_INTERVAL_MS) {
    lastTransmitReadMs = millis();

    float accelFiltered[3], gyroFiltered[3];
    for (int i = 0; i < 3; i++) {
      accelFiltered[i] = accelAxes[i].read();
      gyroFiltered[i] = gyroAxes[i].read();
    }

    // Convert accelerometer to milli-g (1g = 9.80665 m/s^2)
    latestUnits.gX = toProtocolInt16(accelFiltered[0] * 1000.0 / 9.80665);
    latestUnits.gY = toProtocolInt16(accelFiltered[1] * 1000.0 / 9.80665);
    latestUnits.gZ = toProtocolInt16(accelFiltered[2] * 1000.0 / 9.80665);
    // Convert gyro to centi-deg/sec
    latestUnits.rX = toProtocolInt16(gyroFiltered[0] * 180.0 / M_PI * 100.0);
    latestUnits.rY = toProtocolInt16(gyroFiltered[1] * 180.0 / M_PI * 100.0);
    latestUnits.rZ = toProtocolInt16(gyroFiltered[2] * 180.0 / M_PI * 100.0);
  }
}

ImuProtocolUnits imuReadProtocolUnits() { return latestUnits; }
