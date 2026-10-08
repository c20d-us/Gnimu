#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "../code/Gnimu/src/core/GnssFeed.h"

using namespace gnimu;

namespace {
GnssEpoch epochAt(uint32_t iTowMs, uint64_t arrivalUs, uint32_t speedMmPerS) {
  GnssEpoch e;
  e.iTowMs = iTowMs;
  e.arrivalUs = arrivalUs;
  e.gSpeedMmPerS = speedMmPerS;
  e.fixType = 3;
  e.gnssFixOk = true;
  return e;
}
}  // namespace

static void testNothingBeforeTheFirstEpoch() {
  GnssFeed f(50, 1000000, 2000000);
  assert(f.latest() == nullptr);
  assert(f.epochs().epochs() == 0);
  assert(!f.speedValid(0));
}

static void testAddUpdatesEverything() {
  GnssFeed f(50, 1000000, 2000000);
  f.add(epochAt(1000, 5000000, 1500));
  f.add(epochAt(1050, 5050000, 2500));
  assert(f.epochs().epochs() == 2);
  assert(f.latest() != nullptr && f.latest()->iTowMs == 1050);
  assert(fabsf(f.speedMps() - 2.5f) < 1e-6f);
  assert(f.speedValid(5050000 + 999999));
  assert(!f.speedValid(5050000 + 1000000));
}

static void testClearResetsStatisticsOnly() {
  GnssFeed f(50, 1000000, 2000000);
  f.add(epochAt(1000, 5000000, 1500));
  f.clear();
  assert(f.epochs().epochs() == 0);
  // The latest epoch and the speed survive a stats clear.
  assert(f.latest() != nullptr && f.latest()->iTowMs == 1000);
  assert(f.speedValid(5000000));
}

static void testNeverStartedCountsFromTheFirstCheck() {
  GnssFeed f(50, 1000000, 2000000);
  // Boot time before the job's first check doesn't count.
  assert(f.liveness(30000000) == GnssLiveness::Ok);
  assert(f.liveness(30000000 + 1999999) == GnssLiveness::Ok);
  assert(f.liveness(30000000 + 2000000) == GnssLiveness::NeverStarted);
}

static void testAnEpochKeepsItAliveThenSilenceStops() {
  GnssFeed f(50, 1000000, 2000000);
  assert(f.liveness(30000000) == GnssLiveness::Ok);
  f.add(epochAt(1000, 30500000, 0));
  assert(f.liveness(30500000 + 1999999) == GnssLiveness::Ok);
  assert(f.liveness(30500000 + 2000000) == GnssLiveness::Stopped);
  // A fresh epoch brings it straight back.
  f.add(epochAt(1050, 32600000, 0));
  assert(f.liveness(32600000 + 100) == GnssLiveness::Ok);
}

static void testClearDoesNotAffectLiveness() {
  GnssFeed f(50, 1000000, 2000000);
  f.liveness(0);
  f.add(epochAt(1000, 100000, 0));
  f.clear();
  // Still "had epochs": silence now means Stopped, not NeverStarted.
  assert(f.liveness(100000 + 2000000) == GnssLiveness::Stopped);
}

int main() {
  testNothingBeforeTheFirstEpoch();
  testAddUpdatesEverything();
  testClearResetsStatisticsOnly();
  testNeverStartedCountsFromTheFirstCheck();
  testAnEpochKeepsItAliveThenSilenceStops();
  testClearDoesNotAffectLiveness();
  puts("gnssfeed: all tests passed");
  return 0;
}
