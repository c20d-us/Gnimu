// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include <assert.h>
#include <stdio.h>

#include <initializer_list>

#include "../code/Gnimu/src/core/PowerPolicy.h"

using namespace gnimu;

static const PowerPolicyConfig kCfg = {500};

// Polls every 50 ms (the power job's rate) from `from` to `to`, holding the
// inputs steady; returns the first non-None action, or None.
static PowerAction hold(PowerPolicy& p, uint32_t from, uint32_t to, bool sw, bool usb,
                        bool cutoff = false, uint32_t* at = nullptr) {
  for (uint32_t t = from; t <= to; t += 50) {
    const PowerAction a = p.update(t, sw, usb, cutoff);
    if (a != PowerAction::None) {
      if (at) *at = t;
      return a;
    }
  }
  return PowerAction::None;
}

static void testBootClassification() {
  PowerPolicy p(kCfg);
  assert(p.begin(true, true, false) == PowerAction::None && p.running());
  assert(p.begin(false, true, false) == PowerAction::StopForSwitch && p.waitingForSwitch());
  assert(p.begin(true, false, true) == PowerAction::EnterSystemOff);
  // With USB the battery is being charged, so a low reading doesn't stop the boot.
  assert(p.begin(true, true, true) == PowerAction::None);
}

// On battery the switch can't be off - the MCU would have no power - so an
// OFF reading at boot is noise: a normal boot, or System OFF below cutoff.
static void testBootOffWithoutUsbIsNoise() {
  PowerPolicy p(kCfg);
  assert(p.begin(false, false, false) == PowerAction::None && p.running());
  assert(p.switchNoise() == 1);
  PowerPolicy q(kCfg);
  assert(q.begin(false, false, true) == PowerAction::EnterSystemOff);
}

static void testShortFlickIsIgnored() {
  PowerPolicy p(kCfg);
  p.begin(true, true, false);
  assert(hold(p, 0, 450, false, true) == PowerAction::None);  // off < 500 ms
  assert(hold(p, 500, 3000, true, true) == PowerAction::None);
  assert(p.running());
  assert(p.switchNoise() == 1);  // one glitch
}

// However long, OFF without USB never stops the run; each run of it counts
// once as noise.
static void testOffWithoutUsbIsIgnored() {
  PowerPolicy p(kCfg);
  p.begin(true, false, false);
  assert(hold(p, 0, 5000, false, false) == PowerAction::None);
  assert(p.running());
  assert(p.switchNoise() == 1);
  hold(p, 5050, 5100, true, false);
  hold(p, 5150, 6000, false, false);
  assert(p.switchNoise() == 2);
  p.clearSwitchNoise();
  assert(p.switchNoise() == 0);
}

// Running on USB, switch off then on - however long it was off: CHARGE_ONLY.
static void testOffThenOnFromRunningCharges() {
  for (uint32_t offFor : {500u, 1950u, 10000u}) {
    PowerPolicy p(kCfg);
    p.begin(true, true, false);
    uint32_t at = 0;
    assert(hold(p, 1000, 1000 + offFor, false, true, false, &at) == PowerAction::StopForSwitch);
    assert(at == 1500);  // 500 ms after the first OFF reading
    const uint32_t onAt = 1050 + offFor;
    assert(hold(p, onAt, onAt + 1000, true, true, false, &at) == PowerAction::EnterChargeOnly);
    assert(at == onAt + 500);
    assert(p.chargingOnly());
  }
}

// Plugged in with the switch off (boot into the wait), then switched on:
// CHARGE_ONLY - a device that's off and plugged in is on a charger.
static void testBootIntoWaitThenOnCharges() {
  PowerPolicy p(kCfg);
  assert(p.begin(false, true, false) == PowerAction::StopForSwitch);
  uint32_t at = 0;
  assert(hold(p, 100, 1000, true, true, false, &at) == PowerAction::EnterChargeOnly);
  assert(at == 600);
}

// A single ON reading in the wait (a ghost) does nothing; ON has to hold.
static void testOnGhostInWaitIsIgnored() {
  PowerPolicy p(kCfg);
  assert(p.begin(false, true, false) == PowerAction::StopForSwitch);
  assert(p.update(100, true, true, false) == PowerAction::None);
  assert(p.update(150, false, true, false) == PowerAction::None);
  assert(p.waitingForSwitch());
  assert(p.switchNoise() == 1);
  assert(hold(p, 200, 1000, true, true) == PowerAction::EnterChargeOnly);
}

// Off/on from CHARGE_ONLY - quick or not - is the way back to RUNNING; and
// a further off/on from the RUNNING it reboots into charges again.
static void testLeavingChargeOnly() {
  PowerPolicy p(kCfg);
  p.begin(false, true, false);
  assert(hold(p, 0, 600, true, true) == PowerAction::EnterChargeOnly);       // at 500
  assert(hold(p, 700, 1300, false, true) == PowerAction::StopForSwitch);      // at 1200
  assert(hold(p, 5000, 6000, true, true) == PowerAction::ResetForSwitchOn);   // at 5500

  PowerPolicy q(kCfg);
  q.begin(false, true, false);
  hold(q, 0, 600, true, true);
  assert(q.chargingOnly());
  // USB removal needs no debounce: it's a digital VBUS detect.
  assert(q.update(650, true, false, false) == PowerAction::ResetForUsbRemoval);
}

static void testCutoffOnlyOnBattery() {
  PowerPolicy p(kCfg);
  p.begin(true, false, false);
  assert(p.update(0, true, false, true) == PowerAction::EnterSystemOff);
  PowerPolicy q(kCfg);
  q.begin(true, true, false);
  assert(hold(q, 0, 5000, true, true, true) == PowerAction::None);  // USB: never
}

static void testDebounced() {
  Debounced d;
  d.reset(false);
  assert(!d.update(true, 0, 100));
  assert(!d.update(true, 99, 100));
  assert(d.update(true, 100, 100));   // held 100 ms: taken
  assert(d.glitches() == 0);
  assert(d.update(false, 200, 100));  // starts a change...
  assert(d.update(true, 250, 100));   // ...that didn't last
  assert(d.glitches() == 1);
  assert(d.update(false, 300, 100));
  assert(!d.update(false, 400, 100));  // a fresh run, timed from 300
  assert(d.glitches() == 1);
}

int main() {
  testBootClassification();
  testBootOffWithoutUsbIsNoise();
  testShortFlickIsIgnored();
  testOffWithoutUsbIsIgnored();
  testOffThenOnFromRunningCharges();
  testBootIntoWaitThenOnCharges();
  testOnGhostInWaitIsIgnored();
  testLeavingChargeOnly();
  testCutoffOnlyOnBattery();
  testDebounced();
  puts("powerpolicy: all tests passed");
  return 0;
}
