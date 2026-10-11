// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../code/Gnimu/src/core/ImuFilter.h"
#include "../code/Gnimu/src/core/ImuHealth.h"
#include "../code/Gnimu/src/core/ImuSample.h"

using namespace gnimu;

static bool near(float a, float b) { return fabsf(a - b) < 1e-4f; }

static void testRemapIdentity() {
  AxisMap identity = {{0, 1, 2}, {1.0f, 1.0f, 1.0f}};
  float v[3] = {1.0f, 2.0f, 3.0f};
  remapAxes(v, identity);
  assert(near(v[0], 1.0f) && near(v[1], 2.0f) && near(v[2], 3.0f));
}

static void testRemapPermutationAndSign() {
  // Vehicle forward <- sensor Y, vehicle left <- sensor X (negated), vehicle
  // up <- sensor Z: a 90-degree yaw remap, the kind a rotated mount needs.
  AxisMap rotated = {{1, 0, 2}, {1.0f, -1.0f, 1.0f}};
  float v[3] = {10.0f, 20.0f, 30.0f};
  remapAxes(v, rotated);
  assert(near(v[0], 20.0f) && near(v[1], -10.0f) && near(v[2], 30.0f));
}

static void testRemapDoesNotClobberDuringPermutation() {
  // A cyclic permutation is the case a naive in-place remap gets wrong: by the
  // time axis 2 reads source 1, an in-place write would already have
  // overwritten it from an earlier step.
  AxisMap cyclic = {{2, 0, 1}, {1.0f, 1.0f, 1.0f}};
  float v[3] = {1.0f, 2.0f, 3.0f};
  remapAxes(v, cyclic);
  assert(near(v[0], 3.0f) && near(v[1], 1.0f) && near(v[2], 2.0f));
}

// Only all three accel axes at exactly zero is rejected. A negative zero from
// a sign-flipping remap counts too: the drivers check after the remap.
static void testFormatAxisMap() {
  char buf[24];
  const AxisMap identity = {{0, 1, 2}, {+1.0f, +1.0f, +1.0f}};
  assert(formatAxisMap(buf, sizeof(buf), identity) == 14);
  assert(strcmp(buf, "X=+x Y=+y Z=+z") == 0);
  const AxisMap flipped = {{0, 1, 2}, {-1.0f, -1.0f, +1.0f}};
  formatAxisMap(buf, sizeof(buf), flipped);
  assert(strcmp(buf, "X=-x Y=-y Z=+z") == 0);
  const AxisMap swapped = {{2, 0, 1}, {+1.0f, -1.0f, +1.0f}};
  formatAxisMap(buf, sizeof(buf), swapped);
  assert(strcmp(buf, "X=+z Y=-x Z=+y") == 0);
  // Too small a buffer truncates and still terminates.
  char tiny[6];
  assert(formatAxisMap(tiny, sizeof(tiny), identity) == 5);
  assert(strcmp(tiny, "X=+x ") == 0);
}

static void testZeroAccel() {
  ImuSample s;
  assert(isZeroAccel(s));
  s.accel[0] = -0.0f;
  assert(isZeroAccel(s));
  s.gyro[1] = 0.5f;  // gyro doesn't count: a still IMU reads zero rate
  assert(isZeroAccel(s));
  s.accel[2] = 9.80665f;  // level and still: one axis carries gravity
  assert(!isZeroAccel(s));
  ImuSample partial;  // two axes zero, one not
  partial.accel[0] = 0.01f;
  assert(!isZeroAccel(partial));
}

static void testFilterSeedThenConverge() {
  ImuFilter f(0.5f, 0.5f);
  ImuSample seed;
  seed.accel[2] = 9.8f;
  f.reset(seed);
  ImuSample read = f.read();
  assert(near(read.accel[2], 9.8f) && near(read.gyro[0], 0.0f));

  ImuSample raw;
  raw.accel[2] = 11.8f;
  f.update(raw);
  assert(near(f.read().accel[2], 10.8f));  // halfway from 9.8 to 11.8
}

static void testHealthStartsDownAndNeedsMarkUp() {
  ImuHealth h(3);
  assert(!h.up());
  assert(!h.feed(true));  // feed before markUp() is a no-op
  assert(!h.up());
}

static void testHealthLatchesDownAfterFailures() {
  ImuHealth h(3);
  h.markUp();
  assert(h.up());
  assert(!h.feed(false));
  assert(!h.feed(false));
  assert(h.up());       // 2 failures, threshold is 3
  assert(h.feed(false));  // the 3rd failure latches it down and returns true
  assert(!h.up());
  assert(!h.feed(true));  // no recovery without a reboot
  assert(!h.up());
}

static void testHealthMarkDown() {
  ImuHealth h(3);
  h.markUp();
  h.markDown();
  assert(!h.up());
}

static void testHealthSuccessResetsFailureCount() {
  ImuHealth h(3);
  h.markUp();
  h.feed(false);
  h.feed(false);
  assert(!h.feed(true));  // resets the counter
  assert(!h.feed(false));
  assert(!h.feed(false));
  assert(h.up());  // would have latched down without the reset
}

int main() {
  testRemapIdentity();
  testRemapPermutationAndSign();
  testRemapDoesNotClobberDuringPermutation();
  testFormatAxisMap();
  testZeroAccel();
  testFilterSeedThenConverge();
  testHealthStartsDownAndNeedsMarkUp();
  testHealthLatchesDownAfterFailures();
  testHealthMarkDown();
  testHealthSuccessResetsFailureCount();
  puts("imu: all tests passed");
  return 0;
}
