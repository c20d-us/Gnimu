#include <assert.h>
#include <stdio.h>

#include "../code/Gnimu/src/core/RateTracker.h"

using namespace gnimu;

static void testBasicRate() {
  RateTracker t;
  t.reset(0, 0);
  // 20 epochs over exactly 1,000,000us == 20.00Hz == 2000 centiHz.
  assert(t.sampleCentiHz(20, 1000000) == 2000);
}

static void testDifferentIntervalSameRate() {
  RateTracker t;
  t.reset(0, 0);
  // 10 epochs over 500,000us is also 20Hz: the interval length shouldn't
  // change the computed rate.
  assert(t.sampleCentiHz(10, 500000) == 2000);
}

static void testChainedSamplesUseThePreviousBaseline() {
  RateTracker t;
  t.reset(100, 0);
  assert(t.sampleCentiHz(120, 1000000) == 2000);  // +20 epochs / 1s
  // Baseline rolled forward to (120, 1,000,000us); a stall (no new epochs)
  // over the next second must read as 0Hz, not carry the prior rate forward.
  assert(t.sampleCentiHz(120, 2000000) == 0);
  // Recovery: +5 epochs over the next 500ms == 10Hz == 1000 centiHz.
  assert(t.sampleCentiHz(125, 2500000) == 1000);
}

static void testZeroElapsedNeverDividesByZero() {
  RateTracker t;
  t.reset(5, 1000);
  assert(t.sampleCentiHz(5, 1000) == 0);
}

// A counter reset between samples (console C) re-baselines instead of
// wrapping into an enormous rate.
static void testCounterGoingBackwardsRebaselines() {
  RateTracker r;
  r.reset(2000, 0);
  assert(r.sampleCentiHz(20, 1000000) == 0);
  // The next window measures from the reset point.
  assert(r.sampleCentiHz(40, 2000000) == 2000);  // 20 events in 1 s
}

int main() {
  testCounterGoingBackwardsRebaselines();
  testBasicRate();
  testDifferentIntervalSameRate();
  testChainedSamplesUseThePreviousBaseline();
  testZeroElapsedNeverDividesByZero();
  puts("ratetracker: all tests passed");
  return 0;
}
