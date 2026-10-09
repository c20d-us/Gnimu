// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "../code/Gnimu/src/core/GnssSpeedGate.h"

using namespace gnimu;

namespace {
bool near(float a, float b) { return fabsf(a - b) < 1e-6f; }

constexpr uint64_t kStaleUs = 1000000;

GnssEpoch epochAt(uint32_t mmPerS, uint8_t fixType, bool fixOk, uint64_t arrivalUs = 5000000) {
  GnssEpoch e;
  e.arrivalUs = arrivalUs;
  e.gSpeedMmPerS = mmPerS;
  e.fixType = fixType;
  e.gnssFixOk = fixOk;
  return e;
}
}  // namespace

static void testInvalidUntilFirstEpoch() {
  GnssSpeedGate gate(kStaleUs);
  assert(!gate.valid(0));
  assert(!gate.valid(10000000));
}

static void testFeedMakesItValid() {
  GnssSpeedGate gate(kStaleUs);
  gate.feed(epochAt(1500, 3, true, 5000000));
  assert(gate.valid(5000000));
  assert(near(gate.speedMps(), 1.5f));
}

static void testGoesStaleAfterTheWindow() {
  GnssSpeedGate gate(kStaleUs);
  gate.feed(epochAt(0, 3, true, 5000000));
  assert(gate.valid(5000000 + 999999));
  assert(!gate.valid(5000000 + 1000000));
  assert(!gate.valid(5000000 + 60000000));
}

static void testFreshFeedResetsStaleness() {
  GnssSpeedGate gate(kStaleUs);
  gate.feed(epochAt(0, 3, true, 5000000));
  assert(!gate.valid(7000000));
  gate.feed(epochAt(2000, 3, true, 7000000));  // a fresh epoch restarts the window
  assert(gate.valid(7000000));
  assert(near(gate.speedMps(), 2.0f));
}

// The point of timing by arrival: asking often or rarely gives the same
// answer, unlike the old count of caller ticks.
static void testStalenessIndependentOfQueryRate() {
  GnssSpeedGate often(kStaleUs), rarely(kStaleUs);
  often.feed(epochAt(0, 3, true, 5000000));
  rarely.feed(epochAt(0, 3, true, 5000000));
  for (uint64_t t = 5000000; t < 5000000 + 999999; t += 1000) assert(often.valid(t));
  assert(often.valid(5999999) == rarely.valid(5999999));
  assert(often.valid(6000000) == rarely.valid(6000000));
  assert(!rarely.valid(6000000));
}

static void testFixTypeMustBeThreeD() {
  GnssSpeedGate gate(kStaleUs);
  gate.feed(epochAt(0, 2, true));  // 2D fix
  assert(!gate.valid(5000000));
}

static void testGnssFixOkIsSeparateFromFixType() {
  GnssSpeedGate gate(kStaleUs);
  gate.feed(epochAt(0, 3, false));  // fixType 3 but gnssFixOK clear
  assert(!gate.valid(5000000));
}

int main() {
  testInvalidUntilFirstEpoch();
  testFeedMakesItValid();
  testGoesStaleAfterTheWindow();
  testFreshFeedResetsStaleness();
  testStalenessIndependentOfQueryRate();
  testFixTypeMustBeThreeD();
  testGnssFixOkIsSeparateFromFixType();
  puts("gnssspeedgate: all tests passed");
  return 0;
}
