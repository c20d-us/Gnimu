// Gnimu - GNSS+IMU streaming telemetry
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
#include "g_gnss.h"
#include "g_imu_sensor.h"
#include "g_imu_trim.h"
#include "g_log.h"

// IMU pipeline: axis remap, trim, filtering, epoch-locked decimation, unit
// conversion, and sensor failure handling. The driver is behind
// g_imu_sensor.h. Output is checked by test/run_imu_harness.sh.

// The axis map must be a permutation. config.h checks each value's range.
static_assert(IMU_AXIS_X_SRC != IMU_AXIS_Y_SRC &&
                  IMU_AXIS_Y_SRC != IMU_AXIS_Z_SRC &&
                  IMU_AXIS_X_SRC != IMU_AXIS_Z_SRC,
              "ERROR: IMU_AXIS_X_SRC / _Y_SRC / _Z_SRC must be three different "
              "sensor axes - the map is a permutation, never a duplication.");

// A run of failed reads lasting this long marks the IMU down.
static constexpr unsigned long kImuDownAfterMs = 100;
static constexpr unsigned int kImuFailedReadsToDown =
    (kImuDownAfterMs + IMU_SAMPLE_INTERVAL_MS - 1) / IMU_SAMPLE_INTERVAL_MS;

// [0]=X [1]=Y [2]=Z. Thresholds in g and deg/s.
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

// Last latched values, returned by imuReadProtocolUnits().
static ImuProtocolUnits latestUnits = {0, 0, 0, 0, 0, 0};

static bool imuUp = false;

static unsigned int consecutiveFailedReads = 0;

// All failed reads since boot, so an intermittent bus shows up.
// Stops counting once the IMU is down.
static uint32_t totalFailedReads = 0;

// Map the sensor frame into the vehicle frame using IMU_AXIS_*_SRC / _SIGN
// (config.h). Works from a copy because a permutation can't be done in place.
static void remapAxes(float triple[3]) {
  const float in[3] = {triple[0], triple[1], triple[2]};
  triple[0] = in[IMU_AXIS_X_SRC] * IMU_AXIS_X_SIGN;
  triple[1] = in[IMU_AXIS_Y_SRC] * IMU_AXIS_Y_SIGN;
  triple[2] = in[IMU_AXIS_Z_SRC] * IMU_AXIS_Z_SIGN;
}

// Round a scaled value (milli-g or centi-deg/s) to int16_t, saturating instead
// of wrapping. NaN returns 0 (casting NaN is undefined); infinities clamp.
// `value != value` avoids <math.h>.
static int16_t toProtocolInt16(float value) {
  if (value != value)
    return 0;
  if (value > 32767.0f)
    return 32767;
  if (value < -32768.0f)
    return -32768;
  // Round half away from zero.
  return (int16_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

// GNSS ground speed (m/s) for the trim gate. Returns false without a 3D fix
// with gnssFixOK, or while the receiver is stalled - gnssLatestPvt() returns
// the last epoch however old. Used because gnssConsumePvt() belongs to
// g_telemetry.
static bool trimSpeedMps(float *speedMps) {
  const UBX_NAV_PVT_data_t *pvt = gnssLatestPvt();
  if (pvt == nullptr || gnssStalled())
    return false;

  // Also pauses gyro refinement under a poor fix.
  if (pvt->fixType != 3 || !pvt->flags.bits.gnssFixOK)
    return false;

  *speedMps = (float)pvt->gSpeed * 0.001f; // mm/s -> m/s
  return true;
}

// Mark the IMU down until reboot: fields read zero. Logs once.
static void markDown(const char *why) {
  if (imuUp) {
    LOG_PRINTF("❌ IMU %s - IMU fields will read zero.\n", why);
  }
  imuUp = false;
  latestUnits = {0, 0, 0, 0, 0, 0};
}

// Read one sample, remapped to the vehicle frame and trimmed.
static ImuRawSample readProcessed() {
  // Returned when a read fails. Zeros until the first good read.
  static ImuRawSample lastGood = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};

  ImuRawSample s;
  if (!imuSensorRead(&s)) {
    // Reuse the last good sample so a bad read never enters the filters or the
    // trim. Not logged per failure; a run marks the IMU down.
    totalFailedReads++;
    if (++consecutiveFailedReads >= kImuFailedReadsToDown) {
      markDown("stopped responding");
    }
    return lastGood;
  }
  consecutiveFailedReads = 0;

  remapAxes(s.accel);
  remapAxes(s.gyro);

  // Update the trim with uncorrected values, then apply it.
  float speedMps = 0.0f;
  const bool speedValid = trimSpeedMps(&speedMps);
  imuTrimUpdate(s.accel, s.gyro, speedMps, speedValid);
  imuTrimApply(s.accel, s.gyro);

  lastGood = s;
  return s;
}

// Seed each axis with a real reading, so the first window doesn't see a false
// transient (e.g. gravity on Z).
static void seedFilters() {
  ImuRawSample seed = readProcessed();
  for (int i = 0; i < 3; i++) {
    accelAxes[i].reset(seed.accel[i]);
    gyroAxes[i].reset(seed.gyro[i]);
  }
}

void imuBegin() {
  // Configure trim before any read. Drivers report g, so gravity is 1.0.
  const ImuTrimConfig trimCfg = {
      1.0f,
      (float)IMU_SAMPLE_INTERVAL_MS,
      IMU_TRIM_QUALIFY_MS,
      IMU_TRIM_BLOCK_MS,
      IMU_TRIM_LOCK_BLOCKS,
      IMU_TRIM_SPEED_MAX_MPS,
      IMU_TRIM_ACCEL_SANITY_TOL,
      IMU_TRIM_ACCEL_VAR_MAX,
      IMU_TRIM_GYRO_VAR_MAX,
      IMU_TRIM_MAX_TILT_DEG,
      (bool)IMU_TRIM_REQUIRE_FIX,
  };
  imuTrimBegin(trimCfg);

  if (!imuSensorBegin()) {
    // Not markDown(), which only logs an up -> down transition.
    imuUp = false;
    latestUnits = {0, 0, 0, 0, 0, 0};
#if IMU_ENABLED
    LOG_PRINTLN("❌ IMU not found - continuing without it: IMU fields will "
                "read zero.");
#else
    LOG_PRINTLN("⏸️ IMU not fitted (IMU_ENABLED 0) - IMU fields will read "
                "zero.");
#endif
    return;
  }
  imuUp = true;
  seedFilters();
  LOG_PRINTLN("✅ IMU Accelerometer/Gyro enabled.");
}

bool imuIsUp() { return imuUp; }

uint32_t imuFailedReads() { return totalFailedReads; }

void imuPoll() {
  static unsigned long lastImuReadMs = 0;
  const unsigned long nowMs = millis();

  if (!imuUp)
    return;

  // Deadline-anchored: the anchor advances by one interval, and resyncs to now
  // if more than one interval behind.
  if (nowMs - lastImuReadMs >= IMU_SAMPLE_INTERVAL_MS) {
    lastImuReadMs += IMU_SAMPLE_INTERVAL_MS;
    if (nowMs - lastImuReadMs >= IMU_SAMPLE_INTERVAL_MS) {
      lastImuReadMs = nowMs;
    }
    ImuRawSample raw = readProcessed();
    if (!imuUp)
      return; // this read marked the IMU down
    for (int i = 0; i < 3; i++) {
      accelAxes[i].update(raw.accel[i]);
      gyroAxes[i].update(raw.gyro[i]);
    }
  }
}

ImuProtocolUnits imuLatchForEpoch() {
  if (!imuUp)
    return latestUnits; // zeros

  // read() also closes each axis's transient window.
  float accelFiltered[3], gyroFiltered[3];
  for (int i = 0; i < 3; i++) {
    accelFiltered[i] = accelAxes[i].read();
    gyroFiltered[i] = gyroAxes[i].read();
  }

  // g -> milli-g, deg/s -> centi-deg/s.
  latestUnits.gX = toProtocolInt16(accelFiltered[0] * 1000.0f);
  latestUnits.gY = toProtocolInt16(accelFiltered[1] * 1000.0f);
  latestUnits.gZ = toProtocolInt16(accelFiltered[2] * 1000.0f);
  latestUnits.rX = toProtocolInt16(gyroFiltered[0] * 100.0f);
  latestUnits.rY = toProtocolInt16(gyroFiltered[1] * 100.0f);
  latestUnits.rZ = toProtocolInt16(gyroFiltered[2] * 100.0f);
  return latestUnits;
}

ImuProtocolUnits imuReadProtocolUnits() { return latestUnits; }
