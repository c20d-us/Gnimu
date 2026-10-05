#pragma once

#include <stdint.h>

namespace gnimu {

// ============================================================================
// IMU trim - runtime levelling and gyro de-biasing.
//
// Learns two corrections while the vehicle is confirmed stationary, and holds
// them frozen while it moves:
//
//   * ACCELEROMETER: the rotation that carries the measured gravity vector
//     back onto vehicle-up, correcting a slightly off-level mount, PLUS a
//     residual along the corrected vertical so the resting magnitude comes
//     out at exactly 1 g. Both are needed: a rotation preserves length, so it
//     can straighten a tilted reading but can never fix one that is short. A
//     rotation alone leaves a chip with a real zero-g bias reading (say)
//     0.925 g forever.
//
//     Together these subsume the chip's own accelerometer zero-bias: it is
//     indistinguishable from mounting tilt at rest, and one correction
//     removes both.
//
//     Measured ONCE per power cycle and then LOCKED. A resting accelerometer
//     cannot separate mount tilt from the slope of the ground under the
//     vehicle, so every re-measurement is a chance to swap a good calibration
//     for a worse one - and the worst-placed stop of a session, a sloped
//     staging lane, is also the last one before the run that matters.
//
//   * GYROSCOPE: the per-axis zero offset. At rest the true rotation rate is
//     exactly zero in every orientation, so whatever the gyro reads is bias.
//
//     Refined at EVERY qualifying stop, not locked. The slope problem above
//     is purely an accelerometer problem - ground attitude does not appear on
//     a rate gyro at all - so there is no reason to freeze it, and gyro bias
//     drifts with temperature over a session in a way tilt does not.
//
// Ported from Gnimu Classic's g_imu_trim, unit-agnostic by design (no
// Arduino, no config include - every tunable arrives via ImuTrimConfig, and
// accel/gyro/speed are passed in by the caller) so it builds and can be
// exercised against known inputs on a host.
// ============================================================================

// UNITS: unit-agnostic. accel values, gravityNative, and the variance ceilings
// are all in whatever units the caller's accel/gyro samples use; they only
// ever get compared against each other. accelSanityTol is a FRACTION of
// gravity, so it needs no unit at all.
struct ImuTrimConfig {
  float gravityNative;     // 1 g expressed in the caller's accel units
  float sampleIntervalMs;  // pacing of update() calls
  uint32_t qualifyMs;      // stillness required before a window opens
  uint32_t blockMs;        // averaging block length once open
  uint32_t lockBlocks;     // blocks averaged before the orientation locks
  float speedMaxMps;       // GNSS speed below which we may be stationary
  float accelSanityTol;    // |a| plausibility band, as a fraction of gravity
  float accelVarMax;       // per-axis accel std-dev ceiling, caller's units
  float gyroVarMax;        // per-axis gyro std-dev ceiling, caller's units
  float maxTiltDeg;        // largest tilt this module will correct
  bool requireFix;         // demand a valid GNSS fix before trimming at all
};

class ImuTrim {
 public:
  // Starts from a level assumption (identity rotation, zero gyro bias) with
  // converged() false, so apply() is a no-op until the first stationary
  // window closes.
  explicit ImuTrim(const ImuTrimConfig& cfg);

  // Feed one sample. Call at cfg.sampleIntervalMs pacing with UNCORRECTED
  // values in the vehicle frame - after the axis remap, before apply().
  //
  // The estimator must see uncorrected data: feeding it its own output would
  // close the loop and make tiltDegrees() decay toward zero as the correction
  // converged, which is exactly wrong for a mounting guard that needs the
  // ABSOLUTE tilt.
  //
  // speedMps/speedValid come from the GNSS solution. When speedValid is false
  // and cfg.requireFix is set, the gate cannot pass and nothing is learned.
  void update(const float accel[3], const float gyro[3], float speedMps, bool speedValid);

  // Applies the current corrections in place: rotate accel, subtract bias
  // from gyro. A no-op before convergence.
  void apply(float accel[3], float gyro[3]) const;

  // The MEASURED tilt from vehicle-level, in degrees. Reported even when it
  // exceeds cfg.maxTiltDeg and the rotation was therefore refused - that is
  // the case a mounting guard exists to catch. 0 before the first stationary
  // window.
  //
  // This measures tilt PLUS ground slope and cannot separate them: a
  // correctly mounted device calibrated on a ramp reads high.
  float tiltDegrees() const { return tiltDeg_; }

  // True once the orientation has been measured and LOCKED. False at boot,
  // and false as long as every measured tilt has exceeded cfg.maxTiltDeg.
  bool converged() const { return converged_; }

 private:
  void rebuildRotation();
  void resetWindow();
  bool gatePasses(const float accel[3], const float gyro[3], float speedMps, bool speedValid);
  void consumeBlock();

  ImuTrimConfig cfg_;

  // The locked unit gravity estimate, in the vehicle frame - not the rotation
  // matrix - is what the averaging happens on. Averaging rotation matrices
  // element-wise does not produce a rotation; averaging the gravity vector
  // and rebuilding the matrix from it sidesteps that.
  float gRef_[3];
  float lockSum_[3];
  uint32_t lockBlocksSeen_;
  float R_[3][3];
  float gyroBias_[3];
  // Residual along the corrected vertical; see the class comment.
  float accelZBias_;
  float tiltDeg_;
  bool converged_;

  // Rolling gyro/accel variance, advanced every sample independent of block
  // accumulation, since the gate needs a variance estimate before a block
  // exists to compute one from. An EMA rather than a ring buffer for O(1)
  // memory; varSamples_ exists because an unsettled EMA starts near zero and
  // would otherwise pass the stillness test on no information at all.
  float varMean_[3];
  float varSq_[3];
  float aVarMean_[3];
  float aVarSq_[3];
  float varAlpha_;
  uint32_t varSamples_;
  uint32_t varSamplesNeeded_;

  // Squared gate bounds, precomputed in the constructor so the hot path (every
  // sample) has no transcendentals; the remaining sqrtf/acosf live in
  // consumeBlock(), which runs at ~1 Hz and only while stationary.
  float gyroVarMaxSq_;
  float accelVarMaxSq_;
  float accelMagLoSq_;
  float accelMagHiSq_;

  uint32_t qualifySamples_;
  uint32_t qualifyNeeded_;
  float accelSum_[3];
  float gyroSum_[3];
  uint32_t blockSamples_;
  uint32_t blockNeeded_;
};

}  // namespace gnimu
