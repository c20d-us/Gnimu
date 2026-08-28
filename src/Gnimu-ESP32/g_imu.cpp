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
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h> // for the setClock() in imuBegin()

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
  float accel[3]; // m/s^2 (Adafruit MPU6050 native units)
  float gyro[3];  // rad/s (Adafruit MPU6050 native units)
};

// ============================================================================
// Axis remap - installed orientation. Maps the sensor frame into the vehicle
// frame. The mounting correction itself (IMU_AXIS_*_SRC / _SIGN) lives in
// config.h, along with the order table, the derivation procedure, and the
// static_asserts that reject a physically impossible map.
//
// A general permutation cannot be done in place - writing triple[0] would
// clobber a value a later axis still needs to read - so work from a copy. Three
// floats of stack, and it keeps the mapping a plain declarative read of the
// config rather than a sequence of conditional swaps.
// ============================================================================
static void remapAxes(float triple[3]) {
  const float in[3] = {triple[0], triple[1], triple[2]};
  triple[0] = in[IMU_AXIS_X_SRC] * IMU_AXIS_X_SIGN;
  triple[1] = in[IMU_AXIS_Y_SRC] * IMU_AXIS_Y_SIGN;
  triple[2] = in[IMU_AXIS_Z_SRC] * IMU_AXIS_Z_SIGN;
}

// Convert a scaled sensor value (gyro in centi-deg/sec, accel in milli-g) to
// the protocol's int16_t, saturating at the representable ±32767 limit rather
// than overflowing. A wrapped overflow would flip sign (i.e., reporting a hard
// left spin as a right one), so we clamp to the extreme. The gyro can exceed
// range at ±500 °/s; the accelerometer stays well within it even at the max ±g
// but goes through here too so all six fields follow one consistent, safe
// pattern.
static int16_t toProtocolInt16(float value) {
  if (value > 32767.0f)
    return 32767;
  if (value < -32768.0f)
    return -32768;
  return (int16_t)value;
}

// Read the IMU and return the raw accel (m/s^2) / gyro (rad/s) values for this
// instant, zero-corrected and remapped into the vehicle frame.
//
// Order matters: per-chip zero-point offsets (IMU_*_OFFSET_*) are subtracted
// FIRST, in the sensor's raw axis frame. That keeps the offsets intrinsic to
// the chip so they don't need to change if the mounting orientation flips
// the IMU_AXIS_* mapping. The mounting remap runs AFTER the correction.
static ImuRawSample readImuRaw() {
  sensors_event_t a, g, temp;
  myIMU.getEvent(&a, &g, &temp);
  ImuRawSample s;
  s.accel[0] = a.acceleration.x - IMU_ACCEL_OFFSET_X_MPS2;
  s.accel[1] = a.acceleration.y - IMU_ACCEL_OFFSET_Y_MPS2;
  s.accel[2] = a.acceleration.z - IMU_ACCEL_OFFSET_Z_MPS2;
  s.gyro[0] = g.gyro.x - IMU_GYRO_OFFSET_X_RADPS;
  s.gyro[1] = g.gyro.y - IMU_GYRO_OFFSET_Y_RADPS;
  s.gyro[2] = g.gyro.z - IMU_GYRO_OFFSET_Z_RADPS;
  remapAxes(s.accel);
  remapAxes(s.gyro);
  return s;
}

// Initialize the IMU, including setting up the sensor ranges and seed values.
void imuBegin() {
  if (!myIMU.begin()) {
    LOG_PRINTLN("❌ Failed to find the IMU module - halting");
    while (1)
      delay(100);
  }

  // Raise the IMU bus above the core's 100kHz default. MUST come after
  // begin(): that call brings the bus up and would overwrite anything set
  // earlier.
  Wire.setClock(IMU_I2C_CLOCK_HZ);

  // IMU started, proceed with configuration
  LOG_PRINTLN("✅ IMU Accelerometer/Gyro enabled.");
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

    // Convert accelerometer to milli-g (1g = 9.80665 m/s^2) and gyro to
    // centi-deg/sec. All-float math: the constant factors fold at compile
    // time and the ESP32's FPU is single-precision only, so double math here
    // would fall back to (slow) software emulation.
    const float mps2ToMilliG = 1000.0f / 9.80665f;
    const float radpsToCentiDeg = (180.0f / (float)M_PI) * 100.0f;
    latestUnits.gX = toProtocolInt16(accelFiltered[0] * mps2ToMilliG);
    latestUnits.gY = toProtocolInt16(accelFiltered[1] * mps2ToMilliG);
    latestUnits.gZ = toProtocolInt16(accelFiltered[2] * mps2ToMilliG);
    latestUnits.rX = toProtocolInt16(gyroFiltered[0] * radpsToCentiDeg);
    latestUnits.rY = toProtocolInt16(gyroFiltered[1] * radpsToCentiDeg);
    latestUnits.rZ = toProtocolInt16(gyroFiltered[2] * radpsToCentiDeg);
  }
}

ImuProtocolUnits imuReadProtocolUnits() { return latestUnits; }
