#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "../code/Gnimu/src/core/ImuTrim.h"

using namespace gnimu;

namespace {

constexpr float kGravity = 9.80665f;
constexpr float kDegToRad = 0.017453293f;

ImuTrimConfig baseConfig() {
  // Small numbers relative to the 500ms internal variance window, so tests
  // converge in a handful of iterations rather than hand-tracing exact counts.
  ImuTrimConfig cfg;
  cfg.gravityNative = kGravity;
  cfg.sampleIntervalMs = 100.0f;  // -> variance settles after 5 samples
  cfg.qualifyMs = 200;            // -> 2 qualifying samples
  cfg.blockMs = 100;              // -> 1 sample per block
  cfg.lockBlocks = 3;             // -> 3 blocks to lock
  cfg.speedMaxMps = 0.5f;
  cfg.accelSanityTol = 0.25f;
  cfg.accelVarMax = 0.392f;   // 0.04g in m/s^2
  cfg.gyroVarMax = 0.017453f;  // 1 deg/s in rad/s
  cfg.maxTiltDeg = 15.0f;
  cfg.requireFix = false;
  return cfg;
}

bool near(float a, float b, float eps = 1e-3f) { return fabsf(a - b) < eps; }

// Feeds constant accel/gyro/speed until converged() or the iteration cap is
// hit (10 is the minimum this config needs; 200 leaves generous margin).
bool driveToConvergence(ImuTrim& trim, const float accel[3], const float gyro[3], float speedMps,
                        bool speedValid, int maxIters = 200) {
  for (int i = 0; i < maxIters; i++) {
    trim.update(accel, gyro, speedMps, speedValid);
    if (trim.converged()) return true;
  }
  return false;
}

}  // namespace

static void testInitialStateIsNoop() {
  ImuTrim trim(baseConfig());
  assert(!trim.converged());
  assert(near(trim.tiltDegrees(), 0.0f));

  float accel[3] = {1.0f, 2.0f, 3.0f};
  float gyro[3] = {0.1f, 0.2f, 0.3f};
  trim.apply(accel, gyro);
  assert(near(accel[0], 1.0f) && near(accel[1], 2.0f) && near(accel[2], 3.0f));
  assert(near(gyro[0], 0.1f) && near(gyro[1], 0.2f) && near(gyro[2], 0.3f));
}

// A device pitched 10 degrees forward: gravity acquires a +X component.
static void testConvergesAndCorrectsKnownTilt() {
  ImuTrim trim(baseConfig());
  const float theta = 10.0f * kDegToRad;
  const float accel[3] = {kGravity * sinf(theta), 0.0f, kGravity * cosf(theta)};
  const float gyro[3] = {0.05f, -0.03f, 0.02f};  // constant gyro bias

  assert(driveToConvergence(trim, accel, gyro, 0.0f, true));
  assert(near(trim.tiltDegrees(), 10.0f, 0.05f));

  float a[3] = {accel[0], accel[1], accel[2]};
  float g[3] = {gyro[0], gyro[1], gyro[2]};
  trim.apply(a, g);
  assert(near(a[0], 0.0f, 0.01f) && near(a[1], 0.0f, 0.01f) && near(a[2], kGravity, 0.01f));
  assert(near(g[0], 0.0f, 1e-4f) && near(g[1], 0.0f, 1e-4f) && near(g[2], 0.0f, 1e-4f));
}

// Beyond maxTiltDeg the rotation must be refused, not clamped - but the
// measured angle still has to be reported, since that's the case a mounting
// guard exists to catch.
static void testRefusesExcessiveTilt() {
  ImuTrimConfig cfg = baseConfig();
  ImuTrim trim(cfg);
  const float theta = 20.0f * kDegToRad;  // > 15 deg maxTiltDeg
  const float accel[3] = {kGravity * sinf(theta), 0.0f, kGravity * cosf(theta)};
  const float gyro[3] = {0.0f, 0.0f, 0.0f};

  assert(!driveToConvergence(trim, accel, gyro, 0.0f, true));
  assert(near(trim.tiltDegrees(), 20.0f, 0.05f));  // measured and reported anyway

  // Accel correction stays a no-op (refused), but the gyro side isn't gated on
  // accel convergence at all - it should already be reading a real bias here
  // (zero, since the input gyro was zero) rather than something stale.
  float a[3] = {accel[0], accel[1], accel[2]};
  float g[3] = {1.0f, 1.0f, 1.0f};
  trim.apply(a, g);
  assert(near(a[0], accel[0]) && near(a[2], accel[2]));  // untouched
  assert(near(g[0], 1.0f) && near(g[1], 1.0f) && near(g[2], 1.0f));  // bias ~0, untouched
}

// Gyro keeps refining every qualifying stop; accel orientation freezes at
// first lock and ignores a later, different-looking tilt.
static void testGyroKeepsRefiningWhileAccelStaysLocked() {
  ImuTrim trim(baseConfig());
  const float theta = 10.0f * kDegToRad;
  const float accel[3] = {kGravity * sinf(theta), 0.0f, kGravity * cosf(theta)};
  const float gyro1[3] = {0.05f, 0.0f, 0.0f};
  assert(driveToConvergence(trim, accel, gyro1, 0.0f, true));
  const float lockedTilt = trim.tiltDegrees();

  // A different tilt and a different gyro bias, as if the device were bumped
  // and the gyro had drifted with temperature.
  const float theta2 = 3.0f * kDegToRad;
  const float accel2[3] = {kGravity * sinf(theta2), 0.0f, kGravity * cosf(theta2)};
  const float gyro2[3] = {-0.02f, 0.01f, 0.0f};
  for (int i = 0; i < 20; i++) trim.update(accel2, gyro2, 0.0f, true);

  assert(near(trim.tiltDegrees(), lockedTilt, 0.05f));  // unchanged: still locked

  float g[3] = {gyro2[0], gyro2[1], gyro2[2]};
  float a[3] = {0, 0, 0};
  trim.apply(a, g);
  assert(near(g[0], 0.0f, 1e-4f) && near(g[1], 0.0f, 1e-4f));  // refined to the new bias
}

static void testGateRejectsExcessiveSpeed() {
  ImuTrim trim(baseConfig());
  const float accel[3] = {0.0f, 0.0f, kGravity};
  const float gyro[3] = {0.0f, 0.0f, 0.0f};
  assert(!driveToConvergence(trim, accel, gyro, 5.0f, true));  // well above speedMaxMps
  assert(near(trim.tiltDegrees(), 0.0f));  // never even opened a window
}

static void testRequireFixBlocksWithoutValidSpeed() {
  ImuTrimConfig cfg = baseConfig();
  cfg.requireFix = true;
  ImuTrim trim(cfg);
  const float accel[3] = {0.0f, 0.0f, kGravity};
  const float gyro[3] = {0.0f, 0.0f, 0.0f};
  assert(!driveToConvergence(trim, accel, gyro, 0.0f, /*speedValid=*/false));
}

int main() {
  testInitialStateIsNoop();
  testConvergesAndCorrectsKnownTilt();
  testRefusesExcessiveTilt();
  testGyroKeepsRefiningWhileAccelStaysLocked();
  testGateRejectsExcessiveSpeed();
  testRequireFixBlocksWithoutValidSpeed();
  puts("imutrim: all tests passed");
  return 0;
}
