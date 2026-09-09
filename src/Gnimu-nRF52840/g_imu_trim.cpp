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

#include "g_imu_trim.h"
#include <math.h>

// ============================================================================
// Internal state. All of it is reset by imuTrimBegin() and none of it is
// persisted - see the header for why there is no stored seed.
// ============================================================================

static ImuTrimConfig cfg_;

// The locked unit gravity estimate, in the vehicle frame. This - not the
// rotation matrix - is what the averaging happens on.
//
// Averaging rotation matrices element-wise does not produce a rotation (the
// result is not orthonormal), and doing it properly would mean carrying
// quaternions and a slerp. Averaging the gravity VECTOR and rebuilding the
// matrix from it sidesteps that entirely, and it is the more natural space to
// work in anyway: what is being measured is where gravity appears to point,
// not some abstract rotation.
static float gRef_[3];

// Orientation capture, accumulated across cfg_.lockBlocks blocks and then
// frozen. An equal-weight mean rather than a running blend: there is exactly
// one measurement event per power cycle, so there is nothing to weight against.
static float lockSum_[3];
static uint32_t lockBlocksSeen_;

// Accelerometer correction, derived from gRef_ whenever it changes. Cached so
// that imuTrimApply() is a plain matrix-vector product on the hot path.
static float R_[3][3];

static float gyroBias_[3];
static float tiltDeg_;
static bool converged_;

// --- Gate: rolling gyro variance -------------------------------------------
// The gate needs a variance estimate BEFORE a block exists to compute one
// from, so this runs continuously and independently of block accumulation.
//
// An EMA rather than a ring buffer: O(1) memory instead of 3*N floats, it
// matches the EMA idiom already used in ImuAxis, and a gate does not need an
// exact variance. varSamples_ exists because an unsettled EMA starts near zero
// and would otherwise PASS the stillness test on no information at all.
static const float GYRO_VAR_WINDOW_MS = 500.0f;
static float varMean_[3];
static float varSq_[3];
static float varAlpha_;
static uint32_t varSamples_;
static uint32_t varSamplesNeeded_;

// Squared gate bounds, precomputed in imuTrimBegin().
//
// Both stillness tests are naturally written against a magnitude, which would
// put four sqrtf() calls on the per-sample path. Comparing squares instead is
// exactly equivalent - all quantities involved are non-negative and
// accelMagTol is constrained below 1.0, so squaring is monotonic - and leaves
// the hot path with no transcendentals at all. The remaining sqrtf/acosf live
// in consumeBlock(), which runs at 1 Hz and only while stationary.
static float gyroVarMaxSq_;
static float accelMagLoSq_;
static float accelMagHiSq_;

// --- Gate: qualification and block accumulation ----------------------------
static uint32_t qualifySamples_;
static uint32_t qualifyNeeded_;
static float accelSum_[3];
static float gyroSum_[3];
static uint32_t blockSamples_;
static uint32_t blockNeeded_;

// ============================================================================
// Small vector helpers. Written out in general form rather than folded into
// the specialised expressions they feed, because the whole rotation derivation
// below has to be auditable by eye - there is no host test standing behind it.
// ============================================================================

static float dot3(const float a[3], const float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static void cross3(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

// Angle of a unit vector from vehicle-up, in degrees.
static float tiltOf(const float u[3]) {
  float c = u[2];
  if (c > 1.0f)
    c = 1.0f;
  if (c < -1.0f)
    c = -1.0f;
  return acosf(c) * 57.29577951f;
}

static void setIdentity(float m[3][3]) {
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++)
      m[r][c] = (r == c) ? 1.0f : 0.0f;
}

// ============================================================================
// Rebuild R_ from gRef_.
//
// We want the rotation R that carries the measured gravity direction u onto
// vehicle-up, z = (0, 0, 1) - that is, R * u = z. Applying that same R to
// every subsequent sample expresses it in the level vehicle frame.
//
// Rodrigues, in the trig-free form for "rotate unit vector a onto unit vector
// b". With v = a x b and c = a . b:
//
//     R = I + [v]x + [v]x^2 * 1/(1+c)
//
// where [v]x is the skew-symmetric cross-product matrix of v.
//
// SIGN CONVENTION - worked by hand, since nothing else checks it. Take a
// device pitched by theta so gravity acquires a +X component:
//
//     u = (sin0, 0, cos0)
//     v = u x z = (u_y, -u_x, 0) = (0, -sin0, 0)
//     c = u . z = cos0
//
// which expands to
//
//     R = [  cos0  0  -sin0 ]
//         [   0    1    0   ]
//         [  sin0  0   cos0 ]
//
// and R * u = (cos0*sin0 - sin0*cos0, 0, sin0*sin0 + cos0*cos0) = (0, 0, 1).
// Correct: the tilt is removed rather than doubled.
//
// The 1/(1+c) term is singular at c = -1 (device fully inverted). It cannot be
// reached here: callers reject any block whose tilt exceeds cfg_.maxTiltDeg,
// which is validated well under 90 degrees, so c stays close to +1.
// ============================================================================
static void rebuildRotation() {
  static const float kUp[3] = {0.0f, 0.0f, 1.0f};

  float v[3];
  cross3(gRef_, kUp, v);
  const float c = dot3(gRef_, kUp);

  // Perfectly level (or numerically indistinguishable from it): no rotation.
  const float denom = 1.0f + c;
  if (denom < 1e-6f) {
    setIdentity(R_);
    return;
  }

  // [v]x
  const float K[3][3] = {
      {0.0f, -v[2], v[1]},
      {v[2], 0.0f, -v[0]},
      {-v[1], v[0], 0.0f},
  };

  // [v]x^2
  float KK[3][3];
  for (int r = 0; r < 3; r++) {
    for (int col = 0; col < 3; col++) {
      KK[r][col] = K[r][0] * K[0][col] + K[r][1] * K[1][col] +
                   K[r][2] * K[2][col];
    }
  }

  const float s = 1.0f / denom;
  for (int r = 0; r < 3; r++) {
    for (int col = 0; col < 3; col++) {
      R_[r][col] = ((r == col) ? 1.0f : 0.0f) + K[r][col] + KK[r][col] * s;
    }
  }
}

// Discard any part-accumulated block and the stillness credit earned so far.
// Called on every gate failure: a window has to be continuous to mean
// anything, so a single moving sample invalidates everything before it.
static void resetWindow() {
  qualifySamples_ = 0;
  blockSamples_ = 0;
  for (int i = 0; i < 3; i++) {
    accelSum_[i] = 0.0f;
    gyroSum_[i] = 0.0f;
  }
  // Discard a part-finished orientation capture too. The capture has to come
  // from one unbroken stop or it averages across two different attitudes.
  lockSum_[0] = lockSum_[1] = lockSum_[2] = 0.0f;
  lockBlocksSeen_ = 0;
}

void imuTrimBegin(const ImuTrimConfig &cfg) {
  cfg_ = cfg;

  // Start from a level assumption with no correction applied. imuTrimApply()
  // is a no-op in this state, so pre-convergence data passes through
  // untouched rather than being silently altered by a guess.
  gRef_[0] = 0.0f;
  gRef_[1] = 0.0f;
  gRef_[2] = 1.0f;
  setIdentity(R_);
  for (int i = 0; i < 3; i++) {
    gyroBias_[i] = 0.0f;
    varMean_[i] = 0.0f;
    varSq_[i] = 0.0f;
  }
  lockSum_[0] = lockSum_[1] = lockSum_[2] = 0.0f;
  lockBlocksSeen_ = 0;
  tiltDeg_ = 0.0f;
  converged_ = false;

  // Sample counts derived from the configured pacing. Guard the division:
  // a zero interval would be a config error, and every window collapsing to
  // one sample would let the gate pass on a single still reading.
  const float interval =
      (cfg_.sampleIntervalMs > 0.0f) ? cfg_.sampleIntervalMs : 1.0f;
  qualifyNeeded_ = (uint32_t)(cfg_.qualifyMs / interval);
  blockNeeded_ = (uint32_t)(cfg_.blockMs / interval);
  varSamplesNeeded_ = (uint32_t)(GYRO_VAR_WINDOW_MS / interval);
  if (qualifyNeeded_ < 1)
    qualifyNeeded_ = 1;
  if (blockNeeded_ < 1)
    blockNeeded_ = 1;
  if (varSamplesNeeded_ < 1)
    varSamplesNeeded_ = 1;

  // Standard EMA-to-window-length equivalence.
  varAlpha_ = 2.0f / ((float)varSamplesNeeded_ + 1.0f);
  varSamples_ = 0;

  gyroVarMaxSq_ = cfg_.gyroVarMax * cfg_.gyroVarMax;
  const float lo = cfg_.gravityNative * (1.0f - cfg_.accelMagTol);
  const float hi = cfg_.gravityNative * (1.0f + cfg_.accelMagTol);
  accelMagLoSq_ = lo * lo;
  accelMagHiSq_ = hi * hi;

  resetWindow();
}

// Advance the rolling gyro variance, then report whether every stillness
// criterion currently holds.
//
// The gyro test is on VARIANCE, not magnitude. Gating on |gyro| would be
// circular: a chip whose resting bias exceeds the threshold would hold the
// gate shut against the very measurement that would correct it (the OLED
// board's Y axis sits at -4 deg/s). The mean is what we are trying to
// measure; the spread is what says whether we are actually rotating.
static bool gatePasses(const float accel[3], const float gyro[3],
                       float speedMps, bool speedValid) {
  for (int i = 0; i < 3; i++) {
    if (varSamples_ == 0) {
      varMean_[i] = gyro[i]; // seed, else the first sample reads as a spike
      varSq_[i] = 0.0f;
      continue;
    }
    const float d = gyro[i] - varMean_[i];
    varMean_[i] += varAlpha_ * d;
    varSq_[i] += varAlpha_ * (d * d - varSq_[i]);
  }
  if (varSamples_ < varSamplesNeeded_)
    varSamples_++;

  // GNSS speed. Requiring a fix closes the one real hole in the rest of the
  // gate: constant-velocity cruise on smooth pavement reads ~1 g magnitude
  // with near-zero gyro variance, and is otherwise indistinguishable from
  // parked. The escape hatch exists because bench testing indoors never gets
  // a fix - see IMU_TRIM_REQUIRE_FIX.
  if (speedValid) {
    if (speedMps > cfg_.speedMaxMps)
      return false;
  } else if (cfg_.requireFix) {
    return false;
  }

  // Accelerometer magnitude, compared squared. |a| is rotation-invariant, so
  // this criterion is immune to mounting orientation by construction - only
  // genuine linear acceleration, chip scale error, or a bad read can move it.
  const float magSq = dot3(accel, accel);
  if (magSq < accelMagLoSq_ || magSq > accelMagHiSq_)
    return false;

  // An unsettled variance estimate is not evidence of stillness.
  if (varSamples_ < varSamplesNeeded_)
    return false;
  for (int i = 0; i < 3; i++) {
    if (varSq_[i] > gyroVarMaxSq_)
      return false;
  }

  return true;
}

// Fold one completed stationary block into the estimates.
static void consumeBlock() {
  const float n = (float)blockSamples_;

  // --- Gyro: take the block mean outright, every time. --------------------
  // At rest the true rate is exactly zero in every orientation, so every block
  // is a clean direct measurement and there is nothing to average away. This
  // runs for the life of the session - unlike the orientation below, the gyro
  // is NOT locked. Ground slope does not appear on a rate gyro, so the reason
  // for locking simply does not apply, and gyro bias drifts with temperature
  // in a way that mount tilt does not.
  //
  // Accepted regardless of what the accelerometer half decides: gyro bias is
  // independent of mounting tilt, so even a steeply mounted device yields a
  // perfectly valid gyro zero.
  for (int i = 0; i < 3; i++)
    gyroBias_[i] = gyroSum_[i] / n;

  // --- Accel: measured once per power cycle, then locked. ------------------
  if (converged_)
    return;

  float u[3];
  for (int i = 0; i < 3; i++)
    u[i] = accelSum_[i] / n;
  const float mag = sqrtf(dot3(u, u));
  if (mag < 1e-6f)
    return; // degenerate; nothing usable in this block
  for (int i = 0; i < 3; i++)
    u[i] /= mag;

  // Report this block's tilt while the capture is still filling, so the
  // console shows a live angle rather than a flat 0 during the wait.
  tiltDeg_ = tiltOf(u);

  for (int i = 0; i < 3; i++)
    lockSum_[i] += u[i];
  lockBlocksSeen_++;
  if (lockBlocksSeen_ < cfg_.lockBlocks)
    return;

  // Capture complete. Average, re-normalise (summed unit vectors are not unit
  // length), and decide.
  float mean[3];
  for (int i = 0; i < 3; i++)
    mean[i] = lockSum_[i] / (float)lockBlocksSeen_;
  const float mn = sqrtf(dot3(mean, mean));
  if (mn < 1e-6f) {
    lockSum_[0] = lockSum_[1] = lockSum_[2] = 0.0f;
    lockBlocksSeen_ = 0;
    return;
  }
  for (int i = 0; i < 3; i++)
    mean[i] /= mn;

  // Measured tilt, recorded BEFORE the range test and kept even when that test
  // fails: an out-of-range mount is precisely what a guard exists to report, so
  // the number has to survive the refusal.
  tiltDeg_ = tiltOf(mean);

  if (tiltDeg_ > cfg_.maxTiltDeg) {
    // Out of scope. Refuse the rotation rather than applying a clamped, partly
    // corrected one, and stay unconverged so the console keeps saying so. Drop
    // the capture and let a later stop try again - the mount may yet be fixed
    // without a reboot.
    lockSum_[0] = lockSum_[1] = lockSum_[2] = 0.0f;
    lockBlocksSeen_ = 0;
    return;
  }

  for (int i = 0; i < 3; i++)
    gRef_[i] = mean[i];
  rebuildRotation();
  converged_ = true; // locked for the rest of this power cycle
}

void imuTrimUpdate(const float accel[3], const float gyro[3], float speedMps,
                   bool speedValid) {
  if (!gatePasses(accel, gyro, speedMps, speedValid)) {
    resetWindow();
    return;
  }

  // Serve out the stillness qualification before accumulating anything. The
  // window has to be continuous to mean anything, so this counter only ever
  // advances on an unbroken run of passing samples.
  if (qualifySamples_ < qualifyNeeded_) {
    qualifySamples_++;
    return;
  }

  for (int i = 0; i < 3; i++) {
    accelSum_[i] += accel[i];
    gyroSum_[i] += gyro[i];
  }
  blockSamples_++;

  if (blockSamples_ >= blockNeeded_) {
    consumeBlock();
    // Stay qualified - we are still stationary. Only the block accumulators
    // roll over, so a long stop yields a steady run of blocks rather than
    // re-serving the qualification delay between each.
    blockSamples_ = 0;
    for (int i = 0; i < 3; i++) {
      accelSum_[i] = 0.0f;
      gyroSum_[i] = 0.0f;
    }
  }
}

void imuTrimApply(float accel[3], float gyro[3]) {
  // A general rotation cannot be done in place - writing accel[0] would
  // clobber a value the later rows still need - so work from a copy. Same
  // reasoning as remapAxes() in g_imu.cpp.
  const float in[3] = {accel[0], accel[1], accel[2]};
  for (int r = 0; r < 3; r++)
    accel[r] = R_[r][0] * in[0] + R_[r][1] * in[1] + R_[r][2] * in[2];

  for (int i = 0; i < 3; i++)
    gyro[i] -= gyroBias_[i];
}

float imuTrimTiltDegrees() { return tiltDeg_; }

bool imuTrimConverged() { return converged_; }
