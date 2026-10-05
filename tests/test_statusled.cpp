#include <assert.h>
#include <stdio.h>

#include "../code/Gnimu/src/core/StatusLed.h"

using namespace gnimu;

static const LedTiming kTiming = {1000, 50, 200};

static void testAdvertisingBlinksOneSecondStartingOn() {
  StatusLed led(kTiming);
  const uint32_t t0 = 5000;
  assert(led.update(LedState::Advertising, t0).on);
  assert(led.update(LedState::Advertising, t0 + 999).on);
  assert(!led.update(LedState::Advertising, t0 + 1000).on);
  assert(!led.update(LedState::Advertising, t0 + 1999).on);
  assert(led.update(LedState::Advertising, t0 + 2000).on);
  assert(led.update(LedState::Advertising, t0).color == LedColor::Blue);
}

static void testConnectedIsSolidBlue() {
  StatusLed led(kTiming);
  for (uint32_t t = 0; t < 5000; t += 37) {
    const LedOutput o = led.update(LedState::Connected, t);
    assert(o.on && o.color == LedColor::Blue);
  }
}

// 10 Hz red, phase counted from the moment the fault is entered (not from
// boot), and still blinking long after - it never goes dark on its own.
static void testFaultBlinksRedIndefinitely() {
  StatusLed led(kTiming);
  led.update(LedState::Off, 0);
  const uint32_t t0 = 7000;  // fault entered after some time in Off
  LedOutput o = led.update(LedState::Fault, t0);
  assert(o.on && o.color == LedColor::Red);
  assert(!led.update(LedState::Fault, t0 + 50).on);
  assert(led.update(LedState::Fault, t0 + 100).on);
  assert(led.update(LedState::Fault, t0 + 30000).on);
  assert(!led.update(LedState::Fault, t0 + 30050).on);
  assert(led.update(LedState::Fault, t0 + 600000).on);
  assert(!led.update(LedState::Fault, t0 + 600050).on);
}

// A reconnect-then-disconnect restarts the advertising blink on "on", so the
// LED reacts to the disconnect immediately instead of mid-way through a phase.
static void testPatternRestartsOnStateChange() {
  StatusLed led(kTiming);
  led.update(LedState::Advertising, 0);
  assert(!led.update(LedState::Advertising, 1500).on);
  led.update(LedState::Connected, 1600);
  assert(led.update(LedState::Advertising, 1700).on);
}

static void testOffIsOff() {
  StatusLed led(kTiming);
  assert(!led.update(LedState::Off, 0).on);
  assert(!led.update(LedState::Off, 123456).on);
}

// Amber both ways, told apart by rate: BATTERY_WAIT's 200 ms vs the battery
// warning's 1 s.
static void testBatteryWaitIsFastAmber() {
  StatusLed led(kTiming);
  LedOutput o = led.update(LedState::BatteryWait, 0);
  assert(o.on && o.color == LedColor::Amber);
  assert(led.update(LedState::BatteryWait, 199).on);
  assert(!led.update(LedState::BatteryWait, 200).on);
  assert(led.update(LedState::BatteryWait, 400).on);

  StatusLed warn(kTiming);
  assert(warn.update(LedState::BatteryWarning, 0).color == LedColor::Amber);
  assert(warn.update(LedState::BatteryWarning, 400).on);  // still in its 1 s "on"
  assert(!warn.update(LedState::BatteryWarning, 1000).on);
}

static void testBatteryCriticalIsRedBlink() {
  StatusLed led(kTiming);
  LedOutput o = led.update(LedState::BatteryCritical, 0);
  assert(o.on && o.color == LedColor::Red);
  assert(!led.update(LedState::BatteryCritical, 1000).on);
}

static void testChargingBlinksGreenAndChargedIsSolid() {
  StatusLed led(kTiming);
  LedOutput o = led.update(LedState::Charging, 0);
  assert(o.on && o.color == LedColor::Green);
  assert(!led.update(LedState::Charging, 1000).on);
  for (uint32_t t = 1500; t < 6000; t += 37) {
    o = led.update(LedState::Charged, t);
    assert(o.on && o.color == LedColor::Green);
  }
}

int main() {
  testBatteryWaitIsFastAmber();
  testBatteryCriticalIsRedBlink();
  testChargingBlinksGreenAndChargedIsSolid();
  testAdvertisingBlinksOneSecondStartingOn();
  testConnectedIsSolidBlue();
  testFaultBlinksRedIndefinitely();
  testPatternRestartsOnStateChange();
  testOffIsOff();
  puts("statusled: all tests passed");
  return 0;
}
