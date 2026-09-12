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
#include <stdint.h>

// ============================================================================
// IMU Trim module - runtime levelling and gyro de-biasing
//
// Learns two corrections while the vehicle is confirmed stationary, and holds
// them frozen while it moves:
//
//   * ACCELEROMETER: the rotation that carries the measured gravity vector back
//     onto vehicle-up, correcting a slightly off-level mount, PLUS a residual
//     along the corrected vertical so the resting magnitude comes out at
//     exactly 1 g. Both are needed: a rotation preserves length, so it can
//     straighten a tilted reading but can never fix one that is short. A
//     rotation alone leaves a chip with a real zero-g bias reading (say)
//     0.925 g forever - which is what an MPU-6050 with -69 mg on Z does.
//
//     Together these subsume the chip's own accelerometer zero-bias: it is
//     indistinguishable from mounting tilt at rest, and one correction removes
//     both. The horizontal part of a bias is absorbed as apparent tilt and the
//     vertical part as the residual - a slight conceptual mismatch, but it
//     produces the correct resting vector either way, and the error it leaves
//     on real accelerations is second order at the small angles in scope.
//
//     Measured ONCE per power cycle and then LOCKED. A resting accelerometer
//     cannot separate mount tilt from the slope of the ground under the car,
//     so every re-measurement is a chance to swap a good calibration for a
//     worse one - and the worst-placed stop of an autocross session, the
//     sloped staging lane, is also the last one before the run that matters.
//     Learning once from a long, deliberate stop and then refusing to move is
//     strictly safer than continuously chasing whatever ground came last.
//
//   * GYROSCOPE: the per-axis zero offset. At rest the true rotation rate is
//     exactly zero in every orientation, so whatever the gyro reads is bias.
//
//     Refined at EVERY qualifying stop, not locked. The slope problem above is
//     purely an accelerometer problem - ground attitude does not appear on a
//     rate gyro at all - so there is no reason to freeze it, and gyro bias
//     drifts with temperature over a session in a way tilt does not.
//
// Replaces the per-board imu_calibration -> config.h workflow, which was the
// only per-chip data in the configuration. With this module the firmware image
// is identical across boards.
//
// Design record, including the alternatives that were rejected and why:
// docs/imu-trim-design.md.
//
// ---------------------------------------------------------------------------
// DELIBERATELY CONFIG-FREE
//
// This module does not include config.h. Every tunable arrives through
// ImuTrimConfig, the same way ImuAxis takes its alpha and threshold as
// constructor arguments rather than reading the config directly. It also needs
// no Arduino.h and no millis() - it counts samples against sampleIntervalMs -
// and no GNSS dependency, since speed is passed in.
//
// That keeps the file byte-identical across all three variants despite their
// different sensors, different drivers, and different native units (g and
// deg/s on the nRF52840 boards; m/s^2 and rad/s on the ESP32), and it means
// the rotation math can be exercised against known inputs off-hardware.
// ============================================================================

// Everything the trim needs to know about the variant it is running on. Built
// by the shared IMU pipeline (g_imu.cpp, identical in every tree) from the
// variant's own config.h IMU_TRIM_* block.
//
// UNITS: the module is unit-agnostic. accel values, gravityNative, and
// gyroVarMax are all in whatever native units that variant's driver produces;
// they only ever get compared against each other. accelSanityTol is expressed
// as a FRACTION of gravity precisely so it needs no per-variant value.
struct ImuTrimConfig {
  float gravityNative;    // 1 g expressed in native accel units
  float sampleIntervalMs; // pacing of imuTrimUpdate() calls
  uint32_t qualifyMs;     // stillness required before a window opens
  uint32_t blockMs;       // averaging block length once open
  uint32_t lockBlocks;    // blocks averaged before the orientation locks
  float speedMaxMps;      // GNSS speed below which we may be stationary
  float accelSanityTol;   // |a| plausibility band, as a fraction of gravity
  float accelVarMax;      // per-axis accel std-dev ceiling, native units
  float gyroVarMax;       // per-axis gyro std-dev ceiling, native units
  float maxTiltDeg;       // largest tilt this module will correct
  bool requireFix;        // demand a valid GNSS fix before trimming at all
};

// Reset all state and adopt cfg. Starts from a level assumption (identity
// rotation, zero gyro bias) with imuTrimConverged() false, so imuTrimApply()
// is a no-op until the first stationary window closes.
//
// This is the ONLY thing that unlocks the orientation, and it runs from
// imuBegin() alone, so the calibration is held until the device is powered
// off.
//
// Nothing is persisted across boots. A stored correction would only help when
// powering on already in motion, but the case that makes a stored value
// accurate - a device left mounted between sessions - is also the case where
// you power on parked and get a stationary window immediately. See
// docs/imu-trim-design.md section 5.4.
void imuTrimBegin(const ImuTrimConfig &cfg);

// Feed one sample. Call at sampleIntervalMs pacing with UNCORRECTED values in
// the vehicle frame - after the axis remap, before imuTrimApply().
//
// The estimator must see uncorrected data. Feeding it its own output would
// close the loop and make imuTrimTiltDegrees() decay toward zero as the
// correction converged, which is exactly the wrong behaviour for a mounting
// guard that needs the ABSOLUTE tilt.
//
// speedMps / speedValid come from the GNSS solution. When speedValid is false
// and cfg.requireFix is set, the gate cannot pass and nothing is learned.
void imuTrimUpdate(const float accel[3], const float gyro[3], float speedMps,
                   bool speedValid);

// Apply the current corrections in place: rotate accel, subtract bias from
// gyro. Cheap and side-effect free - a 3x3 matrix-vector product and three
// subtractions. Safe to call every sample; a no-op before convergence.
void imuTrimApply(float accel[3], float gyro[3]);

// The MEASURED tilt of the device from vehicle-level, in degrees.
//
// Reported even when it exceeds cfg.maxTiltDeg and the rotation was therefore
// refused - that is the case a mounting guard exists to catch, so the value
// has to survive the refusal. Returns 0 before the first stationary window.
//
// Note this measures tilt PLUS ground slope and cannot separate them: a
// correctly mounted device calibrated on a ramp reads high.
float imuTrimTiltDegrees();

// True once the orientation has been measured and LOCKED. False at boot, and
// false as long as every measured tilt has exceeded cfg.maxTiltDeg.
//
// Sensor noise is not why this takes time - half a second of averaging already
// lands two orders of magnitude finer than anything that matters. Nor is the
// qualification window what keeps bad ground out of the calibration: locking
// after the FIRST qualifying window does that, since the first stop of a
// session is the paddock. The length is only a backstop for the case where
// that ordering fails, i.e. the device is switched on on the way out.
bool imuTrimConverged();
