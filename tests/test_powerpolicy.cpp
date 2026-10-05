#include <assert.h>
#include <stdio.h>

#include "../code/Gnimu/src/core/PowerPolicy.h"

using namespace gnimu;

static const PowerPolicyConfig kCfg = {500, 2000};

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

static void testShortFlickIsIgnored() {
  PowerPolicy p(kCfg);
  p.begin(true, true, false);
  assert(hold(p, 0, 450, false, true) == PowerAction::None);  // off < 500 ms
  assert(hold(p, 500, 3000, true, true) == PowerAction::None);
  assert(p.running());
}

static void testGestureEntersChargeOnly() {
  PowerPolicy p(kCfg);
  p.begin(true, true, false);
  uint32_t at = 0;
  assert(hold(p, 1000, 3000, false, true, false, &at) == PowerAction::StopForSwitch);
  assert(at == 1500);  // 500 ms after the first OFF reading
  // Back on at 2950: 1950 ms after first going off, inside the 2 s window.
  assert(p.update(2950, true, true, false) == PowerAction::EnterChargeOnly);
  assert(p.chargingOnly());
}

static void testLongOffRebootsIntoRunning() {
  PowerPolicy p(kCfg);
  p.begin(true, true, false);
  assert(hold(p, 1000, 3000, false, true) == PowerAction::StopForSwitch);
  assert(p.update(3000, true, true, false) == PowerAction::ResetForSwitchOn);  // 2000 ms: too late
}

static void testLeavingChargeOnly() {
  PowerPolicy p(kCfg);
  p.begin(true, true, false);
  hold(p, 0, 600, false, true);
  assert(p.update(800, true, true, false) == PowerAction::EnterChargeOnly);
  // Any off/on from CHARGE_ONLY - quick or not - reboots into RUNNING.
  assert(hold(p, 1000, 2000, false, true) == PowerAction::StopForSwitch);
  assert(p.update(2100, true, true, false) == PowerAction::ResetForSwitchOn);

  PowerPolicy q(kCfg);
  q.begin(true, true, false);
  hold(q, 0, 600, false, true);
  q.update(800, true, true, false);
  assert(q.update(900, true, false, false) == PowerAction::ResetForUsbRemoval);
}

static void testBootIntoWaitAlwaysReboots() {
  // Plugged in with the switch off, then switched on: a normal start, never
  // the gesture, however quick.
  PowerPolicy p(kCfg);
  assert(p.begin(false, true, false) == PowerAction::StopForSwitch);
  assert(p.update(100, true, true, false) == PowerAction::ResetForSwitchOn);
}

static void testCutoffOnlyOnBattery() {
  PowerPolicy p(kCfg);
  p.begin(true, false, false);
  assert(p.update(0, true, false, true) == PowerAction::EnterSystemOff);
  PowerPolicy q(kCfg);
  q.begin(true, true, false);
  assert(hold(q, 0, 5000, true, true, true) == PowerAction::None);  // USB: never
}

int main() {
  testBootClassification();
  testShortFlickIsIgnored();
  testGestureEntersChargeOnly();
  testLongOffRebootsIntoRunning();
  testLeavingChargeOnly();
  testBootIntoWaitAlwaysReboots();
  testCutoffOnlyOnBattery();
  puts("powerpolicy: all tests passed");
  return 0;
}
