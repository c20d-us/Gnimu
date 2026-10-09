// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "../code/Gnimu/src/core/BatteryMonitor.h"

using namespace gnimu;

// The firmware's own battery settings.
static const BatteryConfig kCfg = {20, 250, 0.1f, 3.30f, 3.35f, 3.40f, 4.10f, 5000, 4.10f, 5000};

static bool near(float a, float b) { return fabsf(a - b) < 0.0005f; }

// One whole run at `t`, every reading `mv` except one deliberate sag.
static void run(BatteryMonitor& m, uint32_t t, uint16_t mv) {
  int taken = 0;
  while (m.sampling(t)) {
    m.addSample(taken == 7 ? static_cast<uint16_t>(mv - 300) : mv, t);
    taken++;
  }
  assert(taken == 20);
}

static void testFirstRunStartsImmediatelyAndUsesThePeak() {
  BatteryMonitor m(kCfg);
  assert(!m.status().valid);
  run(m, 10, 3900);  // well inside the 250 ms interval: the first run can't wait
  assert(m.status().valid);
  assert(near(m.status().freshVoltage, 3.900f));  // the sag didn't pull it down
  assert(near(m.status().voltage, 3.900f));        // first run seeds the average
}

static void testRunsArePacedByTheInterval() {
  BatteryMonitor m(kCfg);
  run(m, 0, 3900);
  assert(!m.sampling(249));  // 250 ms from the previous run's START
  assert(m.sampling(250));
}

static void testSmoothingIsDisplayOnly() {
  BatteryMonitor m(kCfg);
  run(m, 0, 4000);
  run(m, 250, 3000);
  assert(near(m.status().freshVoltage, 3.000f));          // cutoff sees the raw drop
  assert(near(m.status().voltage, 4.000f - 0.1f * 1.0f));  // display moves 10%
}

static void testPercentIsAStraightLine() {
  BatteryMonitor m(kCfg);
  run(m, 0, 3700);  // halfway between 3.30 and 4.10
  assert(m.status().percent == 50);
  BatteryMonitor lo(kCfg), hi(kCfg);
  run(lo, 0, 3200);
  run(hi, 0, 4200);
  assert(lo.status().percent == 0 && hi.status().percent == 100);
}

static void testCutoffNeedsFiveSecondsOfLowPeaks() {
  BatteryMonitor m(kCfg);
  uint32_t t = 0;
  for (; t < 5000; t += 250) {
    run(m, t, 3250);
    assert(!m.status().cutoffRequested);
  }
  run(m, t, 3250);  // 5000 ms after the first low run
  assert(m.status().cutoffRequested);
  run(m, t + 250, 3400);  // one good run clears it, and the debounce restarts
  assert(!m.status().cutoffRequested);
  run(m, t + 500, 3250);
  assert(!m.status().cutoffRequested);
}

static void testChargeFullNeedsInactiveChargerAndPlausibleVoltage() {
  BatteryMonitor m(kCfg);
  run(m, 0, 4150);
  m.updateCharger(true, 0);
  assert(m.status().chargerActive && !m.status().chargeFull);
  m.updateCharger(false, 1000);
  m.updateCharger(false, 5999);
  assert(!m.status().chargeFull);
  m.updateCharger(false, 6000);
  assert(m.status().chargeFull);
  m.updateCharger(true, 6100);  // charging again
  assert(!m.status().chargeFull);

  BatteryMonitor low(kCfg);  // an idle charger on a half-empty cell isn't "full"
  run(low, 0, 3800);
  low.updateCharger(false, 0);
  low.updateCharger(false, 60000);
  assert(!low.status().chargeFull);
}

// "Charging" is the CHG pin, and only with USB present.
static void testIsChargingNeedsThePinAndUsb() {
  BatteryStatus b;
  assert(!isCharging(b, true));
  b.chargerActive = true;
  assert(isCharging(b, true));
  assert(!isCharging(b, false));
}

int main() {
  testFirstRunStartsImmediatelyAndUsesThePeak();
  testRunsArePacedByTheInterval();
  testSmoothingIsDisplayOnly();
  testPercentIsAStraightLine();
  testCutoffNeedsFiveSecondsOfLowPeaks();
  testChargeFullNeedsInactiveChargerAndPlausibleVoltage();
  testIsChargingNeedsThePinAndUsb();
  puts("batterymonitor: all tests passed");
  return 0;
}
