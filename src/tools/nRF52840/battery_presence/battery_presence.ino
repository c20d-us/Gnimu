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

// ============================================================================
// DIAGNOSTIC: battery presence (switch-sense)
//
// Reads the slide switch's spare-pole divider tap on A4, the authoritative
// battery-present signal shipped in g_power.cpp. The 510k/510k divider taps
// ~2 V (half of the ~4.2 V node) when the switch is OFF, and is pulled to
// ~0 V when ON.
//
// Requires: XIAO nRF52840 Sense; a LiPo; USB for serial.
// ============================================================================

#include <Arduino.h>

// Serial (USB CDC) needs the TinyUSB library linked.
#include <Adafruit_TinyUSB.h>

#define SWITCH_SENSE_PIN A4
static const int SWITCH_OFF_THRESHOLD_MV = 800; // tap above this = switch OFF

// SAADC acquisition time (TACQ). The core default is 3 us, valid only up to a
// ~40k source; the switch-sense divider's ~255k source (510k || 510k) needs
// the same long-TACQ treatment as the VBAT divider. 40 us covers up to ~800k.
static const uint8_t SWITCH_SENSE_TACQ_US = 40;

static inline int readSwitchMv() {
  return (int)lroundf((analogRead(SWITCH_SENSE_PIN) / 4095.0f) * 3000.0f);
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
  }
  Serial.println("=== switch-sense (A4) check ===");

  analogReadResolution(12);
  analogReference(AR_INTERNAL_3_0); // pairs with the 3000 mV reference above
  analogSampleTime(SWITCH_SENSE_TACQ_US);
}

void loop() {
  const int mv = readSwitchMv();
  Serial.printf("A4=%d mV -> %s\n", mv,
                mv > SWITCH_OFF_THRESHOLD_MV ? "OFF (disconnected)"
                                              : "ON (connected)");
  delay(500);
}
