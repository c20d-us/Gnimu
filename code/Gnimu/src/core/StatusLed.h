// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

namespace gnimu {

// What the status LED shows, decided here once for every board; each board's
// driver only renders it. A single-color LED ignores `color`.
enum class LedColor : uint8_t { Blue, Red, Green, Amber };
enum class LedState : uint8_t {
  Off,
  Advertising,      // RUNNING, no client: blue blink
  Connected,        // RUNNING, client connected: solid blue
  BatteryWarning,   // RUNNING, low battery: amber blink, ahead of the BLE states
  BatteryCritical,  // RUNNING, critical battery: red blink, ahead of everything
  Fault,            // any fault: fast red, for as long as the fault lasts
  BatteryWait,      // switch off on USB: fast amber
  Charging,         // CHARGE_ONLY: green blink
  Charged,          // CHARGE_ONLY, full: solid green
};

struct LedOutput {
  bool on;
  LedColor color;
};

struct LedTiming {
  uint32_t blinkHalfMs;  // the standard blink (advertising, battery, charging): on, then off
  uint32_t faultHalfMs;
  uint32_t batteryWaitHalfMs;
};

// Every pattern restarts from "on" when its state is entered, so a disconnect
// lights the LED immediately.
class StatusLed {
 public:
  explicit StatusLed(const LedTiming& t) : t_(t) {}

  LedOutput update(LedState s, uint32_t nowMs) {
    if (s != state_) {
      state_ = s;
      enteredMs_ = nowMs;
    }
    const uint32_t ms = nowMs - enteredMs_;
    switch (s) {
      case LedState::Off:
        return {false, LedColor::Blue};
      case LedState::Advertising:
        return {blink(ms, t_.blinkHalfMs), LedColor::Blue};
      case LedState::Connected:
        return {true, LedColor::Blue};
      case LedState::BatteryWarning:
        return {blink(ms, t_.blinkHalfMs), LedColor::Amber};
      case LedState::BatteryCritical:
        return {blink(ms, t_.blinkHalfMs), LedColor::Red};
      case LedState::Fault:
        return {blink(ms, t_.faultHalfMs), LedColor::Red};
      case LedState::BatteryWait:
        return {blink(ms, t_.batteryWaitHalfMs), LedColor::Amber};
      case LedState::Charging:
        return {blink(ms, t_.blinkHalfMs), LedColor::Green};
      case LedState::Charged:
        return {true, LedColor::Green};
    }
    return {false, LedColor::Blue};
  }

 private:
  static bool blink(uint32_t ms, uint32_t halfMs) { return (ms / halfMs) % 2 == 0; }

  LedTiming t_;
  LedState state_ = LedState::Off;
  uint32_t enteredMs_ = 0;
};

}  // namespace gnimu
