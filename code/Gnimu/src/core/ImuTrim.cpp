#include "ImuTrim.h"

#include <math.h>

namespace gnimu {

namespace {

constexpr float kGyroVarWindowMs = 500.0f;

float dot3(const float a[3], const float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void cross3(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

// Angle of a unit vector from vehicle-up, in degrees.
float tiltOf(const float u[3]) {
  float c = u[2];
  if (c > 1.0f) c = 1.0f;
  if (c < -1.0f) c = -1.0f;
  return acosf(c) * 57.29577951f;
}

void setIdentity(float m[3][3]) {
  for (int r = 0; r < 3; r++)
    for (int c = 0; c < 3; c++) m[r][c] = (r == c) ? 1.0f : 0.0f;
}

}  // namespace

ImuTrim::ImuTrim(const ImuTrimConfig& cfg) : cfg_(cfg) {
  // Start from a level assumption with no correction applied. apply() is a
  // no-op in this state, so pre-convergence data passes through untouched
  // rather than being silently altered by a guess.
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

  // Sample counts derived from the configured pacing. Guard the division: a
  // zero interval would be a config error, and every window collapsing to one
  // sample would let the gate pass on a single still reading.
  const float interval = (cfg_.sampleIntervalMs > 0.0f) ? cfg_.sampleIntervalMs : 1.0f;
  qualifyNeeded_ = static_cast<uint32_t>(cfg_.qualifyMs / interval);
  blockNeeded_ = static_cast<uint32_t>(cfg_.blockMs / interval);
  varSamplesNeeded_ = static_cast<uint32_t>(kGyroVarWindowMs / interval);
  if (qualifyNeeded_ < 1) qualifyNeeded_ = 1;
  if (blockNeeded_ < 1) blockNeeded_ = 1;
  if (varSamplesNeeded_ < 1) varSamplesNeeded_ = 1;

  // Standard EMA-to-window-length equivalence.
  varAlpha_ = 2.0f / (static_cast<float>(varSamplesNeeded_) + 1.0f);
  varSamples_ = 0;

  gyroVarMaxSq_ = cfg_.gyroVarMax * cfg_.gyroVarMax;
  accelVarMaxSq_ = cfg_.accelVarMax * cfg_.accelVarMax;
  const float lo = cfg_.gravityNative * (1.0f - cfg_.accelSanityTol);
  const float hi = cfg_.gravityNative * (1.0f + cfg_.accelSanityTol);
  accelMagLoSq_ = lo * lo;
  accelMagHiSq_ = hi * hi;

  resetWindow();
}

// ============================================================================
// Rebuild R_ from gRef_.
//
// We want the rotation R that carries the measured gravity direction u onto
// vehicle-up, z = (0, 0, 1) - that is, R * u = z. Rodrigues, in the trig-free
// form for "rotate unit vector a onto unit vector b". With v = a x b and
// c = a . b:
//
//     R = I + [v]x + [v]x^2 * 1/(1+c)
//
// The 1/(1+c) term is singular at c = -1 (device fully inverted). Unreachable
// here: callers reject any block whose tilt exceeds cfg_.maxTiltDeg, which is
// validated well under 90 degrees, so c stays close to +1.
// ============================================================================
void ImuTrim::rebuildRotation() {
  static constexpr float kUp[3] = {0.0f, 0.0f, 1.0f};

  float v[3];
  cross3(gRef_, kUp, v);
  const float c = dot3(gRef_, kUp);

  // Fully inverted, where 1/(1+c) is singular. Unreachable - see above - so
  // fall back to no rotation rather than divide by zero.
  const float denom = 1.0f + c;
  if (denom < 1e-6f) {
    setIdentity(R_);
    return;
  }

  const float K[3][3] = {
      {0.0f, -v[2], v[1]},
      {v[2], 0.0f, -v[0]},
      {-v[1], v[0], 0.0f},
  };

  float KK[3][3];
  for (int r = 0; r < 3; r++) {
    for (int col = 0; col < 3; col++) {
      KK[r][col] = K[r][0] * K[0][col] + K[r][1] * K[1][col] + K[r][2] * K[2][col];
    }
  }

  const float s = 1.0f / denom;
  for (int r = 0; r < 3; r++) {
    for (int col = 0; col < 3; col++) {
      R_[r][col] = ((r == col) ? 1.0f : 0.0f) + K[r][col] + KK[r][col] * s;
    }
  }
}

// Discards any part-accumulated block and the stillness credit earned so far.
// Called on every gate failure: a window has to be continuous to mean
// anything, so a single moving sample invalidates everything before it.
void ImuTrim::resetWindow() {
  qualifySamples_ = 0;
  blockSamples_ = 0;
  for (int i = 0; i < 3; i++) {
    accelSum_[i] = 0.0f;
    gyroSum_[i] = 0.0f;
  }
  // Discard a part-finished orientation capture too: it has to come from one
  // unbroken stop or it averages across two different attitudes.
  lockSum_[0] = lockSum_[1] = lockSum_[2] = 0.0f;
  lockBlocksSeen_ = 0;
}

// Advances the rolling gyro/accel variance, then reports whether every
// stillness criterion currently holds.
//
// The gyro test is on VARIANCE, not magnitude: gating on |gyro| would be
// circular, since a chip whose resting bias exceeds the threshold would hold
// the gate shut against the very measurement that would correct it.
bool ImuTrim::gatePasses(const float accel[3], const float gyro[3], float speedMps,
                         bool speedValid) {
  for (int i = 0; i < 3; i++) {
    if (varSamples_ == 0) {
      // Seed, else the first sample reads as a spike against a zero mean.
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
  if (varSamples_ < varSamplesNeeded_) varSamples_++;

  // GNSS speed. Requiring a fix closes the one real hole in the rest of the
  // gate: constant-velocity cruise on smooth pavement reads ~1g magnitude
  // with near-zero gyro variance, otherwise indistinguishable from parked.
  if (speedValid) {
    if (speedMps > cfg_.speedMaxMps) return false;
  } else if (cfg_.requireFix) {
    return false;
  }

  // Accelerometer PLAUSIBILITY, not equality - deliberately a wide band. A
  // tight "is |a| exactly 1g" test has the same circularity as gating the
  // gyro on magnitude would: |a| at rest is contaminated by the chip's own
  // zero-g bias, which is exactly what trim exists to remove, so a tight band
  // holds the gate shut against the very measurement that would fix it. This
  // band's only job is catching something gross - a misconfigured
  // gravityNative, a dead axis, a failed read. Motion is caught by the
  // variance test below and by the GNSS speed gate, not by this.
  const float magSq = dot3(accel, accel);
  if (magSq < accelMagLoSq_ || magSq > accelMagHiSq_) return false;

  // An unsettled variance estimate is not evidence of stillness.
  if (varSamples_ < varSamplesNeeded_) return false;
  for (int i = 0; i < 3; i++) {
    if (varSq_[i] > gyroVarMaxSq_) return false;
    if (aVarSq_[i] > accelVarMaxSq_) return false;
  }

  return true;
}

// Folds one completed stationary block into the estimates.
void ImuTrim::consumeBlock() {
  const float n = static_cast<float>(blockSamples_);

  // Gyro: take the block mean outright, every time. At rest the true rate is
  // exactly zero in every orientation, so every block is a clean direct
  // measurement. Unlike the orientation below, this is never locked -
  // accepted regardless of what the accelerometer half decides, since gyro
  // bias is independent of mounting tilt.
  for (int i = 0; i < 3; i++) gyroBias_[i] = gyroSum_[i] / n;

  // Accel: measured once per power cycle, then locked.
  if (converged_) return;

  // Accumulate the block mean UNNORMALISED: the direction gives the rotation
  // and the length gives the residual, and both are needed.
  float m[3];
  for (int i = 0; i < 3; i++) m[i] = accelSum_[i] / n;
  const float mag = sqrtf(dot3(m, m));
  if (mag < 1e-6f) return;  // degenerate; nothing usable in this block

  // Report this block's tilt while the capture is still filling, so a live
  // angle shows rather than a flat 0 during the wait.
  const float u[3] = {m[0] / mag, m[1] / mag, m[2] / mag};
  tiltDeg_ = tiltOf(u);

  for (int i = 0; i < 3; i++) lockSum_[i] += m[i];
  lockBlocksSeen_++;
  if (lockBlocksSeen_ < cfg_.lockBlocks) return;

  // Capture complete. Split the mean into a direction and a length.
  float mean[3];
  for (int i = 0; i < 3; i++) mean[i] = lockSum_[i] / static_cast<float>(lockBlocksSeen_);
  const float mn = sqrtf(dot3(mean, mean));
  if (mn < 1e-6f) {
    lockSum_[0] = lockSum_[1] = lockSum_[2] = 0.0f;
    lockBlocksSeen_ = 0;
    return;
  }
  const float dir[3] = {mean[0] / mn, mean[1] / mn, mean[2] / mn};

  // Measured tilt, recorded BEFORE the range test and kept even when that
  // test fails: an out-of-range mount is precisely what a guard exists to
  // report, so the number has to survive the refusal.
  tiltDeg_ = tiltOf(dir);

  if (tiltDeg_ > cfg_.maxTiltDeg) {
    // Out of scope. Refuse the rotation rather than applying a clamped,
    // partly corrected one, and stay unconverged. Drop the capture and let a
    // later stop try again - the mount may yet be fixed without a reboot.
    lockSum_[0] = lockSum_[1] = lockSum_[2] = 0.0f;
    lockBlocksSeen_ = 0;
    return;
  }

  for (int i = 0; i < 3; i++) gRef_[i] = dir[i];
  rebuildRotation();

  // R carries the resting vector onto the vertical, giving (0, 0, mn) - the
  // right direction but still the measured length. Subtracting the shortfall
  // along that vertical makes a resting device read exactly (0, 0, 1 g).
  accelZBias_ = mn - cfg_.gravityNative;

  converged_ = true;  // locked for the rest of this power cycle
}

void ImuTrim::update(const float accel[3], const float gyro[3], float speedMps,
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

void ImuTrim::apply(float accel[3], float gyro[3]) const {
  // A general rotation cannot be done in place - writing accel[0] would
  // clobber a value the later rows still need - so work from a copy.
  const float in[3] = {accel[0], accel[1], accel[2]};
  for (int r = 0; r < 3; r++)
    accel[r] = R_[r][0] * in[0] + R_[r][1] * in[1] + R_[r][2] * in[2];

  // After the rotation, in the vehicle frame. Zero until the first lock.
  accel[2] -= accelZBias_;

  for (int i = 0; i < 3; i++) gyro[i] -= gyroBias_[i];
}

}  // namespace gnimu
