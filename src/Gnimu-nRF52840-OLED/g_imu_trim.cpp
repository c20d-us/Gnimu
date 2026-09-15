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

#include "g_imu_trim.h"
#include <math.h>

// Rolling variance window for the stillness gate.
static const float GYRO_VAR_WINDOW_MS = 500.0f;

// State, all reset by imuTrimBegin().

static ImuTrimConfig cfg_;

// Locked unit gravity vector in the vehicle frame. Averaging is done on this
// vector, not on rotation matrices.
static float gRef_[3];

// Orientation capture: equal-weight sum over cfg_.lockBlocks blocks.
static float lockSum_[3];
static uint32_t lockBlocksSeen_;

// Accel rotation, rebuilt from gRef_.
static float R_[3][3];

static float gyroBias_[3];

// Residual along the corrected vertical, so a resting reading is exactly 1g.
static float accelZBias_;

static float tiltDeg_;
static bool converged_;

// Gate: rolling variance (EMA). varSamples_ keeps an unsettled estimate from
// passing.
static float varMean_[3];
static float varSq_[3];
static float aVarMean_[3];
static float aVarSq_[3];
static float varAlpha_;
static uint32_t varSamples_;
static uint32_t varSamplesNeeded_;

// Squared gate bounds, so the per-sample path needs no sqrtf().
static float gyroVarMaxSq_;
static float accelVarMaxSq_;
static float accelMagLoSq_;
static float accelMagHiSq_;

// Gate: qualification and block accumulation
static uint32_t qualifySamples_;
static uint32_t qualifyNeeded_;
static float accelSum_[3];
static float gyroSum_[3];
static uint32_t blockSamples_;
static uint32_t blockNeeded_;

// Vector helpers

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

// Rebuild R_ so that R_ * gRef_ = (0, 0, 1). Rodrigues' rotation of unit vector
// a onto b, with v = a x b and c = a . b:
//
//     R = I + [v]x + [v]x^2 / (1 + c)
//
// Singular at c = -1 (inverted), which the max-tilt limit rules out.
static void rebuildRotation() {
  static const float kUp[3] = {0.0f, 0.0f, 1.0f};

  float v[3];
  cross3(gRef_, kUp, v);
  const float c = dot3(gRef_, kUp);

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

// Drop the qualification credit, the partial block, and any partial capture.
// Called on every gate failure, since a window must be one unbroken stop.
static void resetWindow() {
  qualifySamples_ = 0;
  blockSamples_ = 0;
  for (int i = 0; i < 3; i++) {
    accelSum_[i] = 0.0f;
    gyroSum_[i] = 0.0f;
  }
  lockSum_[0] = lockSum_[1] = lockSum_[2] = 0.0f;
  lockBlocksSeen_ = 0;
}

// Update the rolling variances and return whether every stillness test passes.
// Gyro and accel are gated on variance, not magnitude, so a large bias can't
// block its own correction.
static bool gatePasses(const float accel[3], const float gyro[3],
                       float speedMps, bool speedValid) {
  for (int i = 0; i < 3; i++) {
    if (varSamples_ == 0) {
      // Seed the mean so the first sample isn't a spike.
      varMean_[i] = gyro[i];
      varSq_[i] = 0.0f;
      aVarMean_[i] = accel[i];
      aVarSq_[i] = 0.0f;
      continue;
    }
    const float d = gyro[i] - varMean_[i];
    varMean_[i] += varAlpha_ * d;
    varSq_[i] += varAlpha_ * (d * d - varSq_[i]);
    const float ad = accel[i] - aVarMean_[i];
    aVarMean_[i] += varAlpha_ * ad;
    aVarSq_[i] += varAlpha_ * (ad * ad - aVarSq_[i]);
  }
  if (varSamples_ < varSamplesNeeded_)
    varSamples_++;

  // GNSS speed. Without a fix, pass only if requireFix is off.
  if (speedValid) {
    if (speedMps > cfg_.speedMaxMps)
      return false;
  } else if (cfg_.requireFix) {
    return false;
  }

  // Wide |a| plausibility band; catches gross faults only.
  const float magSq = dot3(accel, accel);
  if (magSq < accelMagLoSq_ || magSq > accelMagHiSq_)
    return false;

  if (varSamples_ < varSamplesNeeded_)
    return false;
  for (int i = 0; i < 3; i++) {
    if (varSq_[i] > gyroVarMaxSq_)
      return false;
    if (aVarSq_[i] > accelVarMaxSq_)
      return false;
  }

  return true;
}

// Fold a completed stationary block into the estimates.
static void consumeBlock() {
  const float n = (float)blockSamples_;

  // Gyro: the block mean is the bias. Updated every block, never locked, and
  // independent of the accel result.
  for (int i = 0; i < 3; i++)
    gyroBias_[i] = gyroSum_[i] / n;

  // Accel: captured once, then locked.
  if (converged_)
    return;

  // Unnormalized mean: direction gives the rotation, length gives the residual.
  float m[3];
  for (int i = 0; i < 3; i++)
    m[i] = accelSum_[i] / n;
  const float mag = sqrtf(dot3(m, m));
  if (mag < 1e-6f)
    return;

  // Live tilt while the capture fills.
  const float u[3] = {m[0] / mag, m[1] / mag, m[2] / mag};
  tiltDeg_ = tiltOf(u);

  for (int i = 0; i < 3; i++)
    lockSum_[i] += m[i];
  lockBlocksSeen_++;
  if (lockBlocksSeen_ < cfg_.lockBlocks)
    return;

  // Capture complete.
  float mean[3];
  for (int i = 0; i < 3; i++)
    mean[i] = lockSum_[i] / (float)lockBlocksSeen_;
  const float mn = sqrtf(dot3(mean, mean));
  if (mn < 1e-6f) {
    lockSum_[0] = lockSum_[1] = lockSum_[2] = 0.0f;
    lockBlocksSeen_ = 0;
    return;
  }
  const float dir[3] = {mean[0] / mn, mean[1] / mn, mean[2] / mn};

  // Recorded before the range check so an out-of-range tilt is still reported.
  tiltDeg_ = tiltOf(dir);

  if (tiltDeg_ > cfg_.maxTiltDeg) {
    // Refuse, stay unconverged, and let a later stop retry.
    lockSum_[0] = lockSum_[1] = lockSum_[2] = 0.0f;
    lockBlocksSeen_ = 0;
    return;
  }

  for (int i = 0; i < 3; i++)
    gRef_[i] = dir[i];
  rebuildRotation();

  // The rotated resting vector is (0, 0, mn); remove the difference from 1g.
  accelZBias_ = mn - cfg_.gravityNative;

  converged_ = true; // locked until the next imuTrimBegin()
}

void imuTrimBegin(const ImuTrimConfig &cfg) {
  cfg_ = cfg;

  // Level, uncorrected start.
  gRef_[0] = 0.0f;
  gRef_[1] = 0.0f;
  gRef_[2] = 1.0f;
  setIdentity(R_);
  for (int i = 0; i < 3; i++) {
    gyroBias_[i] = 0.0f;
    varMean_[i] = 0.0f;
    varSq_[i] = 0.0f;
    aVarMean_[i] = 0.0f;
    aVarSq_[i] = 0.0f;
  }
  accelZBias_ = 0.0f;
  lockSum_[0] = lockSum_[1] = lockSum_[2] = 0.0f;
  lockBlocksSeen_ = 0;
  tiltDeg_ = 0.0f;
  converged_ = false;

  // Convert durations to sample counts, each at least 1.
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

  // EMA alpha equivalent to an N-sample window.
  varAlpha_ = 2.0f / ((float)varSamplesNeeded_ + 1.0f);
  varSamples_ = 0;

  gyroVarMaxSq_ = cfg_.gyroVarMax * cfg_.gyroVarMax;
  accelVarMaxSq_ = cfg_.accelVarMax * cfg_.accelVarMax;
  const float lo = cfg_.gravityNative * (1.0f - cfg_.accelSanityTol);
  const float hi = cfg_.gravityNative * (1.0f + cfg_.accelSanityTol);
  accelMagLoSq_ = lo * lo;
  accelMagHiSq_ = hi * hi;

  resetWindow();
}

void imuTrimUpdate(const float accel[3], const float gyro[3], float speedMps,
                   bool speedValid) {
  if (!gatePasses(accel, gyro, speedMps, speedValid)) {
    resetWindow();
    return;
  }

  // Serve the qualification period before accumulating.
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
    // Still qualified: start the next block immediately.
    blockSamples_ = 0;
    for (int i = 0; i < 3; i++) {
      accelSum_[i] = 0.0f;
      gyroSum_[i] = 0.0f;
    }
  }
}

void imuTrimApply(float accel[3], float gyro[3]) {
  // Rotate from a copy; can't be done in place.
  const float in[3] = {accel[0], accel[1], accel[2]};
  for (int r = 0; r < 3; r++)
    accel[r] = R_[r][0] * in[0] + R_[r][1] * in[1] + R_[r][2] * in[2];

  // Zero until the orientation locks.
  accel[2] -= accelZBias_;

  for (int i = 0; i < 3; i++)
    gyro[i] -= gyroBias_[i];
}

float imuTrimTiltDegrees() { return tiltDeg_; }

bool imuTrimConverged() { return converged_; }
