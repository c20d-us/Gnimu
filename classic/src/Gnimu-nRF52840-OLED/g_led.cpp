// Gnimu - RaceBox Mini-compatible GNSS+IMU streaming telemetry
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
#include "g_display.h"
#include "g_ble.h"
#include "g_state.h"

// Drive the RGB LED, honoring the active-LOW wiring.
static void setLed(bool r, bool g, bool b) {
  digitalWrite(LED_RED_PIN, r ? LOW : HIGH);
  digitalWrite(LED_GREEN_PIN, g ? LOW : HIGH);
  digitalWrite(LED_BLUE_PIN, b ? LOW : HIGH);
}

// Configure the RGB LED pins and turn the LED off.
void ledBegin() {
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  setLed(false, false, false); // off
}

// State-first priority. BATTERY_WAIT overrides everything else because a
// held-off device doesn't have meaningful battery/BLE state to reflect and
// we need the "check the switch" signal to be unmissable.
void ledUpdate() {
  // Stay dark while the display is doing the job. Deliberately a RUNTIME check
  // rather than a compile-time #if: the whole point is to come back when the
  // panel is missing, which is only knowable at boot. See LED_ENABLED.
#if !LED_ENABLED
  if (displayIsPresent()) {
    setLed(false, false, false);
    return;
  }
#endif

  const SystemState st = stateCurrent();

  if (st == STATE_BATTERY_WAIT) {
    // Rapid red blink - draws the eye without hunting for other status.
    const bool on = (millis() / LED_BATTERY_WAIT_BLINK_MS) % 2 == 0;
    setLed(on, false, false);
    return;
  }

  // RUNNING / LIGHT_SLEEP: existing priority stack, with LIGHT_SLEEP's slow
  // blue blink taking the place of the connected/advertising tier. A
  // battery/charge condition still always wins regardless of state.
  // Priority: charge state (while on USB) > critical > warn > LIGHT_SLEEP blink
  // > connected > advertising.
  // Colors: blue = BLE, green = charge, amber = warn, red = critical.
  const BatteryStatus bat = batteryGetStatus();
  const bool blinkOn = (millis() / LED_BLINK_INTERVAL_MS) % 2 == 0;

  if (bat.charging) {
    const bool on = bat.full ? true : blinkOn; // steady green when full
    setLed(false, on, false);
  } else if (bat.critical) {
    setLed(blinkOn, false, false); // red blink - critical battery
  } else if (bat.warn) {
    setLed(blinkOn, blinkOn, false); // amber blink - low-battery warning
  } else if (st == STATE_LIGHT_SLEEP) {
    // Short on, long off
    static const unsigned long lightSleepCycleMs =
        LED_LIGHT_SLEEP_BLINK_ON_MS + LED_LIGHT_SLEEP_BLINK_OFF_MS;
    const bool lightSleepBlinkOn =
        (millis() % lightSleepCycleMs) < LED_LIGHT_SLEEP_BLINK_ON_MS;
    setLed(false, false, lightSleepBlinkOn);
  } else if (bleIsConnected()) {
    setLed(false, false, true); // blue steady - connected
  } else {
    setLed(false, false, blinkOn); // blue blink - advertising
  }
}
