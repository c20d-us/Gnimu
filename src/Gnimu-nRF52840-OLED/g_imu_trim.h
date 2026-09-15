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
#include <stdint.h>

// IMU trim: runtime levelling and gyro de-biasing, learned while the vehicle is
// stationary and held while it moves.
//
// Accelerometer: a rotation that maps measured gravity onto vehicle-up, plus a
// residual along the vertical so resting magnitude is exactly 1g. This also
// absorbs the chip's zero-g bias. Measured once per power cycle, then locked.
//
// Gyro: per-axis zero offset, refined at every qualifying stop.

// Trim settings, built by g_imu.cpp from IMU_TRIM_*.
// Accel and gyro values use the same units as the samples.
struct ImuTrimConfig {
  float gravityNative;    // 1g in accel units
  float sampleIntervalMs; // imuTrimUpdate() call interval
  uint32_t qualifyMs;     // stillness required before a window opens
  uint32_t blockMs;       // averaging block length
  uint32_t lockBlocks;    // blocks averaged before orientation locks
  float speedMaxMps;      // GNSS speed below which the vehicle may be stopped
  float accelSanityTol;   // |a| band, fraction of gravity
  float accelVarMax;      // per-axis accel std-dev ceiling
  float gyroVarMax;       // per-axis gyro std-dev ceiling
  float maxTiltDeg;       // largest tilt corrected
  bool requireFix;        // require a valid GNSS fix to trim
};

// Reset to level (identity rotation, zero gyro bias) and adopt cfg. The only
// way to unlock the orientation; called from imuBegin(). Nothing is persisted.
void imuTrimBegin(const ImuTrimConfig &cfg);

// Feed one uncorrected vehicle-frame sample (after remap, before
// imuTrimApply()) at sampleIntervalMs. With cfg.requireFix set, nothing is
// learned while speedValid is false.
void imuTrimUpdate(const float accel[3], const float gyro[3], float speedMps,
                   bool speedValid);

// Rotate accel and subtract gyro bias in place. No-op before convergence.
void imuTrimApply(float accel[3], float gyro[3]);

// Measured tilt from level, in degrees. Reported even when above cfg.maxTiltDeg
// and refused. 0 before the first window.
float imuTrimTiltDegrees();

// True once the orientation is measured and locked. Stays false while the
// measured tilt exceeds cfg.maxTiltDeg.
bool imuTrimConverged();
