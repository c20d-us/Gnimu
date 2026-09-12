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
// IMU PIPELINE - identical in every tree (src/tools/check_common.sh).
//
// Everything that does not depend on which sensor is fitted: axis remap,
// runtime trim, the per-axis filters, epoch-locked decimation, protocol-unit
// conversion, and the policy for a failed or missing sensor. The part-specific
// half is the driver behind g_imu_sensor.h.
//
// This file used to exist once per tree with the sensor code mixed in, and
// five of its functions had become character-for-character copies in files no
// script could compare - the blind spot the review blamed for IMU-3. Its
// behaviour across the split is held to byte-identical output by
// test/run_imu_harness.sh.
// ============================================================================

#include "g_imu.h"
#include "ImuAxis.h"
#include "config.h"
#include "g_gnss.h"
#include "g_imu_sensor.h"
#include "g_imu_trim.h"
#include "g_log.h"

// The axis map must be a permutation. config.h checks each IMU_AXIS_*_SRC is
// 0..2 and each _SIGN is +/-1, but nothing stopped two output axes naming the
// same sensor axis: X_SRC 0 / Y_SRC 0 compiles, runs, and silently duplicates
// one axis while dropping another - the mapping mistake the OLED README warns
// Monitor has hidden before. Three distinct values in 0..2 must be all three,
// so pairwise-distinct is the whole check. Here, beside remapAxes(), so one
// copy guards every board.
static_assert(IMU_AXIS_X_SRC != IMU_AXIS_Y_SRC &&
                  IMU_AXIS_Y_SRC != IMU_AXIS_Z_SRC &&
                  IMU_AXIS_X_SRC != IMU_AXIS_Z_SRC,
              "ERROR: IMU_AXIS_X_SRC / _Y_SRC / _Z_SRC must be three different "
              "sensor axes - the map is a permutation, never a duplication.");

// Three axes each for accelerometer and gyroscope, indexed [0]=X, [1]=Y,
// [2]=Z. Arrays let imuBegin()/imuPoll() drive all three axes of a sensor
// with one loop instead of one line per axis.
//
// Thresholds in g and deg/s, like every sample (see g_imu_sensor.h).
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

// Whether the sensor is delivering. See imuIsUp() in g_imu.h.
static bool imuUp = false;

// How long a run of failed reads is ridden out before the IMU is declared
// down: long enough to absorb an I2C glitch (a single failure is invisible -
// the last good sample is simply repeated), short enough that a sensor that
// has actually gone away is reported within a few GNSS epochs rather than
// transmitting its last reading, frozen, for the rest of the session.
static constexpr unsigned long kImuDownAfterMs = 100;
static constexpr unsigned int kImuFailedReadsToDown =
    (kImuDownAfterMs + IMU_SAMPLE_INTERVAL_MS - 1) / IMU_SAMPLE_INTERVAL_MS;
static unsigned int consecutiveFailedReads = 0;

// Every failed read since boot, not just the current run. The run is what
// decides the IMU is gone; this is what makes an INTERMITTENT bus visible - a
// loose wire can fail thousands of reads without ever failing ten in a row,
// and until g_telemetry reported this, nothing said so. Stops moving once the
// IMU is down, because imuPoll() stops reading.
static uint32_t totalFailedReads = 0;

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
// range at ±500°/s; the accelerometer stays well within it even at the max ±g
// but goes through here too so all six fields follow one consistent, safe
// pattern.
static int16_t toProtocolInt16(float value) {
  // NaN first, because every comparison against it is false - it would slip
  // past both clamps below and reach the cast, where converting a NaN to an
  // integer type is undefined behaviour.
  //
  // Nothing in the pipeline produces one today: the sensor path is
  // integer-derived, and g_imu_trim guards each of its divisions (mag < 1e-6f,
  // mn < 1e-6f) and clamps acosf's argument. This is here because the function
  // is the last thing between the filters and the wire, and is written as
  // though it were total.
  //
  // `value != value` rather than isnan() so this needs no <math.h>, and
  // DELIBERATELY NOT isfinite(): infinities are already handled correctly by
  // the clamps below (+inf > 32767.0f is true), and mapping them to 0 instead
  // of the largest representable value would be a regression.
  //
  // 0 is the least-bad substitute - the protocol has no "invalid" encoding for
  // an IMU field, so any value is a lie, and a consistent 0 at least reads as
  // a stuck-sensor fault rather than noise. Silent by design: this is on the
  // per-sample path, and logging here would be its own latency problem.
  if (value != value)
    return 0;
  if (value > 32767.0f)
    return 32767;
  if (value < -32768.0f)
    return -32768;
  // ROUND, half away from zero - not the plain cast, which truncates toward
  // zero. Truncation opened a dead zone two units wide around zero (anything
  // in (-1, 1) became 0) and pulled every value half a unit toward zero: a
  // systematic error, if a small one (0.5 mG, 0.005 deg/s), at the one place
  // the pipeline's floats become the protocol's integers. By hand rather than
  // lroundf(), keeping this function free of <math.h>. Safe at the clamps: the
  // largest value reaching here is 32767.0, and 32767.5 truncates to 32767.
  return (int16_t)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

// Fetch GNSS ground speed for the trim gate, in m/s.
//
// gnssLatestPvt() rather than gnssConsumePvt(): the latter is consume-once and
// belongs to g_telemetry, so a second consumer here would race it and drop BLE
// packets. See g_gnss.h.
//
// Staleness is this caller's problem - gnssLatestPvt() hands back the last
// epoch however old, so a receiver that died mid-drive would otherwise freeze
// at a stale 0 m/s and let the stationary gate pass while moving. Watch iTOW
// and stop trusting the speed once it has stalled for IMU_TRIM_PVT_STALE_MS.
static bool trimSpeedMps(float *speedMps) {
  static uint32_t lastITOW = 0;
  static uint16_t staleSamples = 0;
  static const uint16_t kStaleMax =
      (uint16_t)(IMU_TRIM_PVT_STALE_MS / IMU_SAMPLE_INTERVAL_MS);

  const UBX_NAV_PVT_data_t *pvt = gnssLatestPvt();
  if (pvt == nullptr)
    return false;

  if (pvt->iTOW != lastITOW) {
    lastITOW = pvt->iTOW;
    staleSamples = 0;
  } else if (staleSamples < kStaleMax) {
    staleSamples++;
  }
  if (staleSamples >= kStaleMax)
    return false;

  // 3D fix only - the same strict read of "valid" that g_telemetry reserves for
  // lat/lon. This was briefly relaxed to accept a 2D fix, on the grounds that
  // requiring 3D suspended trim learning under marginal sky. Locking the
  // orientation after one window removed that argument: there is no ongoing
  // accelerometer learning left to protect, and the single window that matters
  // happens in the paddock under clear sky, where a 3D fix is a given. Strict
  // costs nothing there and buys a stronger validity guarantee.
  //
  // One gate serves both halves, so this also pauses GYRO refinement under a
  // poor fix. Accepted deliberately: the gyro only needs to land occasionally
  // to track thermal drift, and a second gate is not worth the extra constant
  // or the longer explanation.
  if (pvt->fixType != 3 || !pvt->flags.bits.gnssFixOK)
    return false;

  *speedMps = (float)pvt->gSpeed * 0.001f; // mm/s -> m/s
  return true;
}

// Mark the IMU down: for the rest of the power cycle, every IMU field reads
// zero and nothing reaches the filters or the trim. There is no restart path -
// a switch cycle or reboot brings it back. Logged once, never per sample.
static void markDown(const char *why) {
  if (imuUp) {
    LOG_PRINTF("❌ IMU %s - IMU fields will read zero.\n", why);
  }
  imuUp = false;
  latestUnits = {0, 0, 0, 0, 0, 0};
}

// One sample in the VEHICLE frame, runtime-trimmed, ready for the filters.
//
// There is no build-time zero correction here any more. Hand-measured per-chip
// offsets (IMU_*_OFFSET_*) were removed once g_imu_trim learned the same
// correction at runtime: the trim measures the total resting error and cannot
// separate chip bias from mounting tilt - and does not need to, since one
// correction removes both. That deleted the only per-chip data in the whole
// configuration, so the firmware image is now identical across boards.
static ImuRawSample readProcessed() {
  // Last good sample, reused if a read fails - see the error branch below.
  // Zero-initialised, so a failure before the very first successful read
  // yields zeros rather than stack garbage.
  static ImuRawSample lastGood = {{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}};

  ImuRawSample s;
  if (!imuSensorRead(&s)) {
    // Reuse the previous sample rather than feeding a failed read into the
    // filters: a bad transfer would otherwise go straight into the EMA and,
    // worse, into the transient-peak window, where one bogus value is latched
    // as a "peak" and reported as a real event. Deliberately silent per
    // failure - this runs at IMU_SAMPLE_INTERVAL_MS, and logging each one
    // would be its own latency problem on a flaky bus. A RUN of failures is
    // different: that is a sensor that has gone, and repeating its last
    // reading forever would transmit stale data as if it were live.
    totalFailedReads++;
    if (++consecutiveFailedReads >= kImuFailedReadsToDown) {
      markDown("stopped responding");
    }
    return lastGood;
  }
  consecutiveFailedReads = 0;

  remapAxes(s.accel);
  remapAxes(s.gyro);

  // Runtime trim, in the vehicle frame. Update BEFORE apply and on the
  // uncorrected values: feeding the estimator its own output would close the
  // loop and make imuTrimTiltDegrees() decay toward zero as it converged,
  // which is exactly wrong for a mounting guard that needs the absolute tilt.
  //
  // Both sit after the failed-read early return above, so a bad I2C transfer
  // never reaches the estimator. That matters more than it looks: a repeated
  // lastGood sample would read as zero variance and falsely satisfy the
  // stillness gate.
  float speedMps = 0.0f;
  const bool speedValid = trimSpeedMps(&speedMps);
  imuTrimUpdate(s.accel, s.gyro, speedMps, speedValid);
  imuTrimApply(s.accel, s.gyro);

  lastGood = s;
  return s;
}

// Seed each axis with a real first reading rather than leaving it at its
// zero-baseline default. Otherwise the first update() would see a huge
// artificial jump (e.g. gravity on the Z axis) that gets latched into the
// window's peak deviation and misreported as a genuine transient in the very
// first transmitted frame.
static void seedFilters() {
  ImuRawSample seed = readProcessed();
  for (int i = 0; i < 3; i++) {
    accelAxes[i].reset(seed.accel[i]);
    gyroAxes[i].reset(seed.gyro[i]);
  }
}

void imuBegin() {
  // Ahead of any read, which would otherwise reach the trim before it has a
  // config.
  //
  // The trim module is unit-agnostic by design and takes 1 g in whatever units
  // it is fed. Every driver reports g, so that is exactly 1.0 - kept as a
  // parameter rather than removed, so g_imu_trim stays a general module.
  const ImuTrimConfig trimCfg = {
      1.0f,                    (float)IMU_SAMPLE_INTERVAL_MS,
      IMU_TRIM_QUALIFY_MS,     IMU_TRIM_BLOCK_MS,
      IMU_TRIM_LOCK_BLOCKS,    IMU_TRIM_SPEED_MAX_MPS,
      IMU_TRIM_ACCEL_SANITY_TOL, IMU_TRIM_ACCEL_VAR_MAX,
      IMU_TRIM_GYRO_VAR_MAX,   IMU_TRIM_MAX_TILT_DEG,
      (bool)IMU_TRIM_REQUIRE_FIX,
  };
  imuTrimBegin(trimCfg);

  if (!imuSensorBegin()) {
    // Not markDown(): that logs only on an up->down transition, and at boot
    // the IMU was never up.
    imuUp = false;
    latestUnits = {0, 0, 0, 0, 0, 0};
#if IMU_ENABLED
    LOG_PRINTLN("❌ IMU not found - continuing without it: IMU fields will "
                "read zero.");
#else
    // Deliberate (IMU_ENABLED 0 in config.h), so not an error - but the same
    // state as a missing or dead IMU, on purpose: one code path, two causes.
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

// Poll the IMU and update the axis filters at our configured sample rate.
// The sample cadence is deadline-anchored (the anchor advances by the interval,
// not to "now"), so per-loop latency doesn't stretch every period and quietly
// drop the real rate below its configured value. If the loop ever falls more
// than one full interval behind, the anchor resyncs to now rather than firing
// a rapid catch-up burst.
void imuPoll() {
  static unsigned long lastImuReadMs = 0;
  const unsigned long nowMs = millis();

  if (!imuUp)
    return;

  // Update all six axis filters if it's time to sample.
  if (nowMs - lastImuReadMs >= IMU_SAMPLE_INTERVAL_MS) {
    lastImuReadMs += IMU_SAMPLE_INTERVAL_MS;
    if (nowMs - lastImuReadMs >= IMU_SAMPLE_INTERVAL_MS) {
      lastImuReadMs = nowMs; // fell > 1 interval behind - resync
    }
    ImuRawSample raw = readProcessed();
    if (!imuUp)
      return; // that read was the one that tipped it over
    for (int i = 0; i < 3; i++) {
      accelAxes[i].update(raw.accel[i]);
      gyroAxes[i].update(raw.gyro[i]);
    }
  }
}

// Driven by the GNSS epoch, not a timer - see g_imu.h for why that distinction
// is the entire point of this function existing separately from imuPoll().
//
// ImuAxis::read() is what drains each axis's transient window, so calling this
// once per epoch is also what keeps the window aligned with the packet it ends
// up in. It must therefore stay outside any BLE-connected test: the drain
// cannot depend on a client being attached, or the window would accumulate
// across a disconnect and dump a stale peak into the first packet after
// reconnecting.
ImuProtocolUnits imuLatchForEpoch() {
  if (!imuUp)
    return latestUnits; // zeros - see markDown()

  float accelFiltered[3], gyroFiltered[3];
  for (int i = 0; i < 3; i++) {
    accelFiltered[i] = accelAxes[i].read();
    gyroFiltered[i] = gyroAxes[i].read();
  }

  // g and deg/s (every driver's units) to the protocol's milli-g and
  // centi-deg/sec.
  latestUnits.gX = toProtocolInt16(accelFiltered[0] * 1000.0f);
  latestUnits.gY = toProtocolInt16(accelFiltered[1] * 1000.0f);
  latestUnits.gZ = toProtocolInt16(accelFiltered[2] * 1000.0f);
  latestUnits.rX = toProtocolInt16(gyroFiltered[0] * 100.0f);
  latestUnits.rY = toProtocolInt16(gyroFiltered[1] * 100.0f);
  latestUnits.rZ = toProtocolInt16(gyroFiltered[2] * 100.0f);
  return latestUnits;
}

ImuProtocolUnits imuReadProtocolUnits() { return latestUnits; }
