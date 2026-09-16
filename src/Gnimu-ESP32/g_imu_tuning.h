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

#pragma once

// IMU tuning: sample rate, filter, transient, and trim values shared by every
// board (all drivers report g and deg/s). Included at the top of each config.h
// and must not reference board settings. A change here must be copied to every
// tree; check_common.sh enforces it. Measurements behind these values are in
// docs/imu-trim-design.md.

// Sampling

// imuPoll() sample interval: 10 = 100Hz. The alphas below are tuned at this
// rate.
#define IMU_SAMPLE_INTERVAL_MS 10

// Per-axis filters (ImuAxis)

// EMA weights. The alpha is also the anti-alias filter for decimation to the
// send rate; 0.09 at 100Hz is a ~1.5Hz corner. Lowering alpha increases
// |raw - baseline|, so raise the transient threshold first.
#define IMU_ACCEL_ALPHA 0.09f // 1.0 = raw, 0.1 = heavy
#define IMU_GYRO_ALPHA 0.09f  // 1.0 = raw, 0.1 = heavy

// Threshold value that disables transient blending: unreachable at any range.
#define IMU_TRANSIENT_PARKED 1.0e6f

// Deviation (g, deg/s) above which a window's peak blends into the output.
// Parked until more research is done.
// Before un-parking, add a stall drain: while gnssStalled(), call read() on
// each axis in imuPoll() and discard it, or the first epoch after a stall
// carries a stale peak.
#define IMU_ACCEL_TRANSIENT_THRESHOLD_G IMU_TRANSIENT_PARKED
#define IMU_GYRO_TRANSIENT_THRESHOLD_DPS IMU_TRANSIENT_PARKED

// Runtime IMU trim (see g_imu_trim.h)

// How long all stillness criteria must hold before a window opens.
#define IMU_TRIM_QUALIFY_MS 30000

// Averaging block length once a window is open.
#define IMU_TRIM_BLOCK_MS 1000

// Blocks averaged before the orientation locks for the power cycle.
#define IMU_TRIM_LOCK_BLOCKS 5

// GNSS ground speed below which the vehicle may be stationary.
#define IMU_TRIM_SPEED_MAX_MPS 0.5f

// Allowed |a| deviation from 1g, as a fraction. Wide on purpose: it only
// catches gross faults (wrong units, dead axis), and a tight band would reject
// the bias the trim corrects.
#define IMU_TRIM_ACCEL_SANITY_TOL 0.25f

// Per-axis accel standard deviation ceiling.
#define IMU_TRIM_ACCEL_VAR_MAX 0.04f // g

// Per-axis gyro standard deviation ceiling. Admits an idling engine. If trim
// won't converge in a rougher car, raise this.
#define IMU_TRIM_GYRO_VAR_MAX 1.0f // deg/s

// Largest tilt corrected. Beyond it the rotation is refused, not clamped.
#define IMU_TRIM_MAX_TILT_DEG 15.0f

// Require a 3D GNSS fix before trimming, so steady cruising isn't mistaken for
// parked. Set 0 for indoor bench testing.
#define IMU_TRIM_REQUIRE_FIX 1


// Compile-time validation

// The GNSS epoch relation is checked in config.h.
static_assert(IMU_SAMPLE_INTERVAL_MS > 0,
              "ERROR: IMU_SAMPLE_INTERVAL_MS must be greater than 0.");

static_assert(IMU_ACCEL_ALPHA > 0.0f && IMU_ACCEL_ALPHA <= 1.0f,
              "ERROR: IMU_ACCEL_ALPHA must be in the range (0.0, 1.0]");
static_assert(IMU_GYRO_ALPHA > 0.0f && IMU_GYRO_ALPHA <= 1.0f,
              "ERROR: IMU_GYRO_ALPHA must be in the range (0.0, 1.0]");

static_assert(IMU_ACCEL_TRANSIENT_THRESHOLD_G > 0.0f,
              "ERROR: IMU_ACCEL_TRANSIENT_THRESHOLD_G must be greater than 0.");
static_assert(
    IMU_GYRO_TRANSIENT_THRESHOLD_DPS > 0.0f,
    "ERROR: IMU_GYRO_TRANSIENT_THRESHOLD_DPS must be greater than 0.");

static_assert(IMU_TRIM_QUALIFY_MS >= IMU_SAMPLE_INTERVAL_MS,
              "ERROR: IMU_TRIM_QUALIFY_MS must span at least one IMU sample.");
static_assert(IMU_TRIM_BLOCK_MS >= IMU_SAMPLE_INTERVAL_MS,
              "ERROR: IMU_TRIM_BLOCK_MS must span at least one IMU sample.");
static_assert(IMU_TRIM_LOCK_BLOCKS >= 1,
              "ERROR: IMU_TRIM_LOCK_BLOCKS must be at least 1.");
static_assert(IMU_TRIM_SPEED_MAX_MPS > 0.0f,
              "ERROR: IMU_TRIM_SPEED_MAX_MPS must be greater than 0.");
static_assert(IMU_TRIM_ACCEL_SANITY_TOL > 0.0f &&
                  IMU_TRIM_ACCEL_SANITY_TOL < 1.0f,
              "ERROR: IMU_TRIM_ACCEL_SANITY_TOL must be in (0.0, 1.0).");
static_assert(IMU_TRIM_ACCEL_VAR_MAX > 0.0f,
              "ERROR: IMU_TRIM_ACCEL_VAR_MAX must be greater than 0.");
static_assert(IMU_TRIM_GYRO_VAR_MAX > 0.0f,
              "ERROR: IMU_TRIM_GYRO_VAR_MAX must be greater than 0.");
// The rotation is small-angle by design and singular at 180 degrees.
static_assert(IMU_TRIM_MAX_TILT_DEG > 0.0f && IMU_TRIM_MAX_TILT_DEG < 60.0f,
              "ERROR: IMU_TRIM_MAX_TILT_DEG must be in the range (0, 60).");
