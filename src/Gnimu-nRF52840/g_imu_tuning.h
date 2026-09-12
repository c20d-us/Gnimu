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

#pragma once

// ============================================================================
// IMU TUNING - the same on every board, and checked to stay that way.
//
// The IMU values that describe the DESIGN rather than the hardware: how often
// the sensor is sampled, how hard the per-axis filters smooth, when a
// transient is blended in, and how runtime trim decides the vehicle is
// parked. Every IMU driver reports g and deg/s (g_imu_sensor.h), so none of
// these depends on which sensor is fitted.
//
// They used to sit in each tree's config.h, byte-identical but UNCHECKED:
// config.h cannot join check_common.sh's common set, because DEVICE_ID, pins
// and sensor settings legitimately differ. This file can, so the copies can no
// longer drift apart. Same reasoning as the protocol constants moving into
// g_proto_<name>.h (docs/multiprotocol-design.md section 7).
//
// What stays in each config.h is what really differs per board: whether an
// IMU is fitted, its address, pins, full-scale ranges, data rates and
// bandwidth, the nRF wake-up detector, and the mounting axis map.
//
// EDITING: a change here is a change on every board. Make it in one tree and
// copy the file to the others; check_common.sh fails until you do. If a board
// ever genuinely needs its own value - the transient thresholds are the
// likeliest, since they depend on how that board is mounted - move that one
// define back into every config.h deliberately, rather than letting the
// copies differ.
//
// Included at the top of every config.h, before any board setting, and
// depends on none: nothing here may refer to a per-board define. The one check
// that relates these values to a board setting - the GNSS epoch interval
// against IMU_SAMPLE_INTERVAL_MS - stays in config.h, where both are defined.
// ============================================================================

// ----------------------------------------------------------------------------
// --- Sampling ---
// ----------------------------------------------------------------------------

// How often imuPoll() reads the sensor: 10 == 100Hz. The filter alphas below
// are tuned AT this rate - an EMA's corner frequency scales with how often it
// is fed - so change them together.
#define IMU_SAMPLE_INTERVAL_MS 10

// ----------------------------------------------------------------------------
// --- Per-axis filters (ImuAxis) ---
// ----------------------------------------------------------------------------

// ImuAxis smoothing rates and transient thresholds. Each axis tracks an EMA
// baseline; the transient threshold is the deviation a sample window's peak
// must exceed before it gets blended into the transmitted value instead of the
// plain baseline. In g for accel, deg/s for gyro.
//
// The ACCEL values are tuned from 13 autocross runs (2018 M2, RE-71RS) on the
// nRF52840 build, checked against the GNSS solution: longitudinal against
// d(speed)/dt, lateral against v * yaw-rate. They have NOT been re-measured on
// the ESP32 build, which carries a different sensor (MPU-6050, not
// LSM6DS3TR-C). The GYRO values are still placeholders on every build - the
// logging app records no gyro channel, so nothing here has been tested against
// real data.
//
// The threshold must sit ABOVE the car's vibration floor. At the original 0.2g
// it sat below it: the blend fired on 50-100% of transmit windows, and since
// maxDeviation_ is a max over the window it can only push the output away from
// the baseline, never toward it. Logged lateral peaks ran up to +105% over the
// true value (2.57g against a real 1.16g) and correlation with the GNSS
// reference fell to 0.92.
//
// The blend is currently PARKED - the thresholds below are set to
// IMU_TRANSIENT_PARKED, so the output is the plain EMA baseline. The mount is
// known to be too springy: on the nRF52840 build the vibration floor alone
// drives |raw - smoothedValue_| to ~1.55g, and no threshold below that
// separates vibration from genuine events.
//
// 1.5g was the first holding value tried, on the reasoning that it sat just
// clear of the floor. IT DOES NOT - 1.5 is below the measured ~1.55, so the
// blend still fired on vibration alone. Nor is 1.5 out of reach in general:
// deviation is bounded only by roughly twice the full-scale range, about 8g at
// the +/-4g both boards use. That is why this is now an explicitly unreachable
// sentinel rather than a number picked to be "high enough": a near-miss
// threshold reads as tuned when it is not, and the failure is invisible in the
// output.
//
// Re-tune after the mount is stiffened, on EACH board - the floor depends on
// the mount, and the ESP32's has never been measured. Log the real floor, then
// replace the sentinel with roughly 1.2x the observed peak deviation
// (~0.6-0.8g on a rigid mount) to bring the blend back for the impacts and
// kerb strikes it exists to catch. If the boards come out needing different
// values, that is the moment to move these two defines into each config.h.
//
// The two settings are NOT independent. A lower alpha makes the EMA baseline
// lag further, which INCREASES |raw - smoothedValue_| and so makes the blend
// fire MORE. Lowering the alpha without raising the threshold first makes the
// output worse, not smoother.
//
// The alpha also serves as the anti-alias filter for the 100Hz -> transmit-rate
// decimation, so it cannot be raised freely: mount resonance above the
// transmit Nyquist folds into the signal band, where no later filter can
// remove it. At the 100Hz sample rate, 0.09 puts the corner at ~1.5Hz, which
// measured best against the GNSS reference (0.99 correlation) while leaving
// real cornering amplitude intact - below ~0.06 it starts eating genuine
// signal.
#define IMU_ACCEL_ALPHA 0.09f // EMA smoothing: 1.0=raw, 0.1=heavy. ~1.5Hz
#define IMU_GYRO_ALPHA 0.09f  // EMA smoothing: 1.0=raw, 0.1=heavy. ~1.5Hz

// Sentinel meaning "transient blending disabled".
//
// Deliberately far beyond any deviation the sensor can produce at ANY
// configurable full-scale range, so parking cannot silently un-park when
// IMU_ACCEL_RANGE_G or IMU_GYRO_RANGE_DPS is widened. It is also deliberately
// unit-agnostic, a choice made back when the trees worked in different units
// and a bare 99.0f was copied between them without its units.
//
// ImuAxis::read() blends only when maxDeviation_ exceeds the threshold, so an
// unreachable threshold makes the output the plain EMA baseline.
#define IMU_TRANSIENT_PARKED 1.0e6f

#define IMU_ACCEL_TRANSIENT_THRESHOLD_G IMU_TRANSIENT_PARKED
#define IMU_GYRO_TRANSIENT_THRESHOLD_DPS IMU_TRANSIENT_PARKED

// ----------------------------------------------------------------------------
// --- Runtime IMU trim (levelling + gyro de-bias) ---
// ----------------------------------------------------------------------------

// Learned while the vehicle is confirmed stationary, frozen while it moves.
// Corrects a slightly off-level mount and the gyro zero point without any
// per-board calibration step. Design record: docs/imu-trim-design.md.
//
// These are per-DESIGN values, not per-board: identical on every unit AND on
// every board.

// How long every stillness criterion must hold CONTINUOUSLY before a window
// opens.
//
// Sensor noise is NOT what sets this: at ~90ug/sqrt(Hz) and the LSM6DS3's 26Hz
// LPF1 cutoff (ODR/4 at 104Hz - the "50Hz" this used to cite was a misreading
// of the part, see IMU_ACCEL_LPF1_ODR_DIV), half a second of averaging lands
// well under 0.1mg, two orders finer than anything relevant.
//
// Nor is it what keeps a sloped staging lane out of the calibration - locking
// after the first qualifying window does that, because the first stop of a
// session is the paddock. Selection happens by ORDERING, not by duration.
//
// What the length actually buys is a backstop for when that ordering does not
// hold: the device is switched on as the car leaves the paddock, so the first
// qualifying stop is a staging lane or a red light on a cambered road. 30s
// clears a rolling pause or a stop sign. It cannot fix the case properly -
// only powering the device on where it is parked does that.
#define IMU_TRIM_QUALIFY_MS 30000

// Averaging block length once the window is open. A long stop yields a steady
// run of blocks rather than re-serving the qualification delay between each.
#define IMU_TRIM_BLOCK_MS 1000

// Blocks averaged into the orientation before it LOCKS for the rest of the
// power cycle. 5 blocks = 5s of data on top of the qualification wait.
//
// Not a noise requirement - the gate already rejects any block containing a
// disturbance, and one block is far more than enough for precision. This is
// margin against a sub-threshold disturbance (someone leaning on the car)
// biasing a measurement that is never revisited.
#define IMU_TRIM_LOCK_BLOCKS 5

// GNSS ground speed below which we may be stationary.
#define IMU_TRIM_SPEED_MAX_MPS 0.5f

// |a| PLAUSIBILITY band, as a fraction of gravity - unit-free.
//
// Deliberately WIDE, and deliberately not a tight "is |a| exactly 1 g" test.
// A tight band has the same circularity that made us gate the gyro on variance
// rather than magnitude: |a| at rest is contaminated by the chip's own zero-g
// bias, which is exactly what the trim exists to remove, so a tight band holds
// the gate shut against the very measurement that would fix it.
//
// That is not hypothetical. The MPU-6050 on the ESP32 build has -69 mg on Z;
// once its hand-measured offset was deleted it read 0.925 g at rest, failed a
// 4% band on every single sample, and could never converge (2026-09-09).
//
// This band's only job is catching something gross - a misconfigured
// driver reporting the wrong units (m/s^2 would read ~9.8 g), a dead axis,
// a failed read. Motion is
// caught by IMU_TRIM_ACCEL_VAR_MAX and the GNSS speed gate, not by this.
#define IMU_TRIM_ACCEL_SANITY_TOL 0.25f

// Per-axis accel STANDARD DEVIATION ceiling, in g - the real
// stillness test for the accelerometer, and bias-immune by construction in the
// same way the gyro's is.
//
// Measured: stationary engine-idling in a 2018 M2 gives per-axis sd of 6-10 mg;
// driving gives 70-118 mg. This sits ~4x above idle and ~2x below driving, and
// it is the secondary check anyway - GNSS speed is the primary motion gate.
#define IMU_TRIM_ACCEL_VAR_MAX 0.04f // g

// Per-axis gyro STANDARD DEVIATION ceiling, in deg/s. Variance, not
// magnitude: gating on |gyro| would be circular, since a chip whose resting
// bias exceeds the threshold would hold the gate shut against the very
// measurement that would correct it (one board here sits at -4 deg/s).
//
// MEASURED, not guessed. Raw stationary capture in a 2018 M2 (2026-09-08):
// cold idle 0.62 deg/s, warm idle 0.37 deg/s, driving 3.6-4.2 deg/s. The
// original 0.5 blocked a cold idle entirely - trim would never converge if the
// logger was switched on after starting the car, which is the natural order.
// 1.0 clears cold idle by 1.6x and still sits 4.2x under driving.
//
// Allowing an idling engine is safe, and that was checked rather than assumed:
// simulating the real 5-block capture on that data gives 0.019 deg
// repeatability at cold idle and 0.019 deg at warm idle - identical despite
// 68% more vibration, because the block mean removes a zero-mean signal.
// Against 3.2 deg of ground-slope difference measured between two ordinary
// parking spots, vibration is ~150x down and simply not in the error budget.
//
// n=1 vehicle, and a reasonably smooth six. A four-cylinder or a diesel could
// sit well above this; if trim will not converge in a rougher car, start here.
#define IMU_TRIM_GYRO_VAR_MAX 1.0f // deg/s

// Largest tilt this module will correct. Beyond it the rotation is REFUSED
// rather than clamped and imuTrimConverged() stays false, but the measured
// angle is still reported so a mounting guard can see it. Past ~15 degrees the
// user has most likely made a mistake rather than a choice.
#define IMU_TRIM_MAX_TILT_DEG 15.0f

// Demand a valid 3D GNSS fix before trimming at all. Closes the hole in
// the gate: constant-velocity cruise on smooth pavement reads ~1g magnitude
// with near-zero gyro variance and is otherwise indistinguishable from parked.
// The cold-start window this costs is not scarce - powering on parked gives
// minutes of stillness after first fix. SET TO 0 FOR BENCH TESTING, which
// never gets a fix indoors - and since this file is checked, check_common.sh
// will flag a tree left at 0, so a bench setting cannot quietly ship in one
// variant.
#define IMU_TRIM_REQUIRE_FIX 1

// How long the last PVT epoch may go without advancing before its speed stops
// counting as valid. gnssLatestPvt() returns the last epoch however stale, so
// a receiver that dies mid-drive would otherwise freeze at a stale 0 m/s and
// let the gate pass while moving.
#define IMU_TRIM_PVT_STALE_MS 1000

// ============================================================================
// --- COMPILE-TIME VALIDATION ---
// ============================================================================

// Enforce a positive sample interval. Its relation to the GNSS epoch interval
// is checked in config.h, where GNSS_NAV_RATE_HZ lives.
static_assert(IMU_SAMPLE_INTERVAL_MS > 0,
              "ERROR: IMU_SAMPLE_INTERVAL_MS must be greater than 0.");

// Enforce sane EMA alpha range
static_assert(IMU_ACCEL_ALPHA > 0.0f && IMU_ACCEL_ALPHA <= 1.0f,
              "ERROR: IMU_ACCEL_ALPHA must be in the range (0.0, 1.0]");
static_assert(IMU_GYRO_ALPHA > 0.0f && IMU_GYRO_ALPHA <= 1.0f,
              "ERROR: IMU_GYRO_ALPHA must be in the range (0.0, 1.0]");

// Enforce positive transient thresholds (a zero/negative threshold would
// disable transient blending entirely; see ImuAxis::read()).
static_assert(IMU_ACCEL_TRANSIENT_THRESHOLD_G > 0.0f,
              "ERROR: IMU_ACCEL_TRANSIENT_THRESHOLD_G must be greater than 0.");
static_assert(
    IMU_GYRO_TRANSIENT_THRESHOLD_DPS > 0.0f,
    "ERROR: IMU_GYRO_TRANSIENT_THRESHOLD_DPS must be greater than 0.");

// --- Runtime IMU trim ---
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
// Kept well under 90 degrees: the rotation build is singular at 180, and the
// small-angle regime is the whole design scope.
static_assert(IMU_TRIM_MAX_TILT_DEG > 0.0f && IMU_TRIM_MAX_TILT_DEG < 60.0f,
              "ERROR: IMU_TRIM_MAX_TILT_DEG must be in the range (0, 60).");
static_assert(IMU_TRIM_PVT_STALE_MS > 0,
              "ERROR: IMU_TRIM_PVT_STALE_MS must be greater than 0.");
