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
#include "g_ble.h"
#include <Arduino.h>

// The ESP32 dev board's single onboard LED (LED_ONBOARD_PIN). Consumes
// bleIsConnected().
//
// Priority, highest first:
//   1. connected    -> solid
//   2. advertising  -> blink at LED_BLINK_INTERVAL_MS

void ledBegin() {
  pinMode(LED_ONBOARD_PIN, OUTPUT);
  digitalWrite(LED_ONBOARD_PIN, LOW);
}

void ledUpdate() {
  // Blink phase is tracked here rather than read back off the pin: reading an
  // output pin to decide what to drive it to makes the LED's state live in the
  // pin rather than in us, and read-back does not reflect the driven value on
  // every pin configuration. blinkOn is kept in step in the connected branch
  // too - a cache written on one path only is how these drift (LAT-4).
  static bool blinkOn = false;

  if (!bleIsConnected()) {
    static unsigned long lastBlinkMs = 0;
    if (millis() - lastBlinkMs > LED_BLINK_INTERVAL_MS) {
      lastBlinkMs = millis();
      blinkOn = !blinkOn;
      digitalWrite(LED_ONBOARD_PIN, blinkOn ? HIGH : LOW);
    }
  } else {
    digitalWrite(LED_ONBOARD_PIN, HIGH); // solid while a client is attached
    blinkOn = true;
  }
}
