// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include <assert.h>
#include <stdio.h>

#include "../code/Gnimu/src/core/GnssEpoch.h"

using namespace gnimu;

static GnssEpoch at(uint32_t iTow, uint64_t arrivalUs) {
  GnssEpoch e;
  e.iTowMs = iTow;
  e.arrivalUs = arrivalUs;
  return e;
}

static void testSteadyStream() {
  EpochTracker t(50);
  for (int i = 0; i < 10; i++) t.feed(at(1000 + 50 * i, 5000 + 50000ull * i));
  assert(t.epochs() == 10 && t.missed() == 0);
  assert(t.maxGapUs() == 50000);
}

static void testMissedEpochs() {
  EpochTracker t(50);
  t.feed(at(1000, 0));
  t.feed(at(1050, 50000));
  t.feed(at(1200, 200000));  // 1100 and 1150 never arrived
  assert(t.epochs() == 3 && t.missed() == 2);
  assert(t.maxGapUs() == 150000);
}

static void testJitterIsNotMissing() {
  EpochTracker t(50);
  t.feed(at(1000, 0));
  t.feed(at(1049, 0));  // iTOW is on the grid; small offsets are not gaps
  t.feed(at(1101, 0));
  assert(t.missed() == 0);
}

static void testWeekRollover() {
  EpochTracker t(50);
  t.feed(at(604799950, 0));
  t.feed(at(0, 50000));
  assert(t.missed() == 0);
  t.feed(at(100, 150000));
  assert(t.missed() == 1);
}

static void testClear() {
  EpochTracker t(50);
  t.feed(at(1000, 0));
  t.feed(at(1500, 500000));
  t.clear();
  assert(t.epochs() == 0 && t.missed() == 0 && t.maxGapUs() == 0);
  t.feed(at(9000, 0));  // the first epoch after a clear is never a gap
  assert(t.missed() == 0);
}

static void testTimeStepIsNotMissedEpochs() {
  EpochTracker t(50);
  t.feed(at(150, 0));         // receiver has no time yet, iTOW counts from zero
  t.feed(at(200, 50000));
  t.feed(at(434700000, 100000));  // then it gets time and iTOW jumps days ahead
  t.feed(at(434700050, 150000));
  assert(t.missed() == 0 && t.timeSteps() == 1);
  t.feed(at(100, 200000));    // a backward step is a step too
  assert(t.missed() == 0 && t.timeSteps() == 2);
  t.clear();
  assert(t.timeSteps() == 0);
}

int main() {
  testTimeStepIsNotMissedEpochs();
  testSteadyStream();
  testMissedEpochs();
  testJitterIsNotMissing();
  testWeekRollover();
  testClear();
  puts("epoch: all tests passed");
  return 0;
}
