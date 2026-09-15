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

// ============================================================================
// DIAGNOSTIC: XIAO nRF52840 Sense RGB status LED check
//
// Validates the assumptions g_led.cpp makes for the status LED:
//   - The LED_RED / LED_GREEN / LED_BLUE pin macros exist in the variant
//     (config.h maps LED_*_PIN to these).
//   - The LED is active-LOW (g_led.cpp's setLed() drives LOW for on): driving a
//     pin LOW lights that color. If the color you see does not match the name
//     printed over serial, the polarity assumption is inverted - fix setLed().
//   - The specific colors g_led.cpp uses as status states look right:
//       green = charging / full        blue = BLE advertising / connected
//       amber = low-battery warning    red  = critical battery / BATTERY_WAIT
//
// What to look for on the serial monitor (115200): each announced color should
// match what the onboard LED actually shows.
//
// Requires: XIAO nRF52840 Sense + USB only.
// ============================================================================

#include <Arduino.h>

// USB Serial on the Adafruit/Seeed nRF52 core (Adafruit_USBD_CDC) is provided by
// the TinyUSB library. A sketch that uses Serial but pulls in no other library
// that already includes TinyUSB must include it explicitly - otherwise linking
// fails with "undefined reference to Adafruit_USBD_CDC / Serial". led_check has
// no other library, so it needs this; imu_probe/ble_mtu get it transitively via
// LSM6DS3.h / bluefruit.h.
#include <Adafruit_TinyUSB.h>

// Active-LOW: LOW lights the color. Mirrors g_led.cpp's setLed().
static void show(const char *label, bool r, bool g, bool b) {
  digitalWrite(LED_RED, r ? LOW : HIGH);
  digitalWrite(LED_GREEN, g ? LOW : HIGH);
  digitalWrite(LED_BLUE, b ? LOW : HIGH);
  Serial.printf("LED = %-18s (R=%d G=%d B=%d, active-LOW)\n", label, r, g, b);
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
  }
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  Serial.println("\n=== XIAO RGB LED check (active-LOW expected) ===");
  Serial.println("Each color is announced over serial - confirm the LED matches.");
}

void loop() {
  show("RED (critical battery / switch off)", 1, 0, 0);
  delay(1200);
  show("GREEN (charging)", 0, 1, 0);
  delay(1200);
  show("BLUE (BLE)", 0, 0, 1);
  delay(1200);
  show("AMBER (low-battery warning)", 1, 1, 0);
  delay(1200);
  show("WHITE (all on)", 1, 1, 1);
  delay(1200);
  show("OFF", 0, 0, 0);
  delay(1200);
}
