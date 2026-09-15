// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "g_led.h"
#include "config.h"
#include "g_battery.h"
#include "g_ble.h"
#include "g_state.h"
#include <Arduino.h>

// XIAO onboard RGB LED. Priority, highest first:
//   1. BATTERY_WAIT    -> rapid red blink
//   2. RUNNING:
//      a. charging     -> green blink (steady when full)
//      b. critical bat -> red blink
//      c. warn bat     -> amber blink
//      d. connected    -> steady blue
//      e. advertising  -> blue blink

// Set the RGB LED (active-low).
static void setLed(bool r, bool g, bool b) {
  digitalWrite(LED_RED_PIN, r ? LOW : HIGH);
  digitalWrite(LED_GREEN_PIN, g ? LOW : HIGH);
  digitalWrite(LED_BLUE_PIN, b ? LOW : HIGH);
}

void ledBegin() {
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  setLed(false, false, false);
}

void ledUpdate() {
  const SystemState st = stateCurrent();

  if (st == STATE_BATTERY_WAIT) {
    const bool on = (millis() / LED_BATTERY_WAIT_BLINK_MS) % 2 == 0;
    setLed(on, false, false);
    return;
  }

  const BatteryStatus bat = batteryGetStatus();
  const bool blinkOn = (millis() / LED_BLINK_INTERVAL_MS) % 2 == 0;

  if (bat.charging) {
    const bool on = bat.full ? true : blinkOn;
    setLed(false, on, false); // green
  } else if (bat.critical) {
    setLed(blinkOn, false, false); // red
  } else if (bat.warn) {
    setLed(blinkOn, blinkOn, false); // amber
  } else if (bleIsConnected()) {
    setLed(false, false, true); // steady blue
  } else {
    setLed(false, false, blinkOn); // blue blink
  }
}
