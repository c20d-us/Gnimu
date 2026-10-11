// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "../code/Gnimu/src/core/Ema.h"

using namespace gnimu;

static bool near(float a, float b) { return fabsf(a - b) < 1e-6f; }

static void testResetSeedsImmediately() {
  Ema e(0.1f);
  e.reset(42.0f);
  assert(near(e.read(), 42.0f));
}

static void testKnownAlphaSequence() {
  Ema e(0.5f);
  e.reset(0.0f);
  e.update(10.0f);
  assert(near(e.read(), 5.0f));
  e.update(10.0f);
  assert(near(e.read(), 7.5f));
}

static void testAlphaOneTracksRaw() {
  Ema e(1.0f);
  e.reset(0.0f);
  e.update(3.0f);
  assert(near(e.read(), 3.0f));
  e.update(-7.0f);
  assert(near(e.read(), -7.0f));
}

int main() {
  testResetSeedsImmediately();
  testKnownAlphaSequence();
  testAlphaOneTracksRaw();
  puts("ema: all tests passed");
  return 0;
}
