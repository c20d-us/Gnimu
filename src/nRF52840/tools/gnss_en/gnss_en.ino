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
// DIAGNOSTIC: GNSS rail EN-pad cutoff check (+ RX back-feed test)
//
// Answers the open hardware question that gp_battery.cpp's low-voltage cutoff
// and the planned GNSS idle-cutoff both rely on: does driving GNSS_EN_PIN (D8)
// LOW actually DISCONNECT the TPS63020 output rail?
//
// Two readings per state:
//   1) [meter]  Probe the TPS63020 VOUT: ~3.3 V = rail up, ~0 V = rail cut.
//   2) [serial] Bytes arriving on Serial1 from the GNSS: a powered M100 streams
//               continuously, so bytes > 0 = powered, bytes == 0 = not running.
//               (Presence is what matters, so it holds even at a mismatched
//               baud.)
//
// States (config.h GNSS_EN_ACTIVE_HIGH == 1 -> HIGH/hi-Z on, LOW off):
//   - hi-Z (INPUT): board pullup should hold EN high -> rail ON (also confirms
//                   the onboard pullup, DESIGN.md section 3).
//   - HIGH:         driven high -> rail ON.
//   - LOW:          driven low  -> rail should be CUT.
//
// RX BACK-FEED TEST (in the LOW state): with EN low, the GNSS can be phantom-
// powered through its RX pin - the XIAO's idle TX (D6) sits at 3.3 V and leaks
// through the GNSS RX ESD diode into VCC, enough to light the module's POWER
// LED even though the receiver is dead (UART silent, PPS LED off). The LOW
// phase therefore runs in two parts:
//   [part 1] TX left idling HIGH  -> back-feed path intact  (POWER LED may stay
//   on) [part 2] TX (D6) forced LOW   -> back-feed cut          (POWER LED
//   should go OUT)
// If the POWER LED goes out only in part 2, the rail is genuinely cut and the
// glow was back-feed via RX. Firmware takeaway: a real cutoff must idle TX low
// too, not just drop EN.
//
// Requires: XIAO nRF52840 Sense with the TPS63020 + GNSS wired per DESIGN.md
//           (EN on D8, GNSS TX -> D7), USB for serial. A meter on VOUT is ideal
//           but not required - the M100 POWER/PPS LEDs tell the story here.
// ============================================================================

#include <Arduino.h>

// Serial (USB CDC) needs the TinyUSB library linked; this sketch pulls in no
// other library that would include it transitively (same gotcha as led_check).
#include <Adafruit_TinyUSB.h>

// Keep these in sync with config.h (GNSS_EN_PIN, GNSS_TX_PIN, GNSS_BAUD).
#define EN_PIN D9
#define GNSS_TX_PIN D6
#define GNSS_UART_BAUD 460800

static const uint32_t SETTLE_MS = 2000; // let the rail settle / GNSS cold-boot
static const uint32_t COUNT_MS = 4000; // GNSS-byte measurement window per state
static const uint32_t HOLD_MS = 5000;  // hold time to watch the M100 LED

enum EnMode { EN_HIZ, EN_HIGH, EN_LOW };

static void applyEn(EnMode mode) {
  switch (mode) {
  case EN_HIZ:
    pinMode(EN_PIN,
            INPUT); // release; rely on the board's EN pullup (no MCU pull)
    break;
  case EN_HIGH:
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, HIGH);
    break;
  case EN_LOW:
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);
    break;
  }
}

// Let the rail settle, discard buffered bytes, then count how many arrive from
// the GNSS over COUNT_MS.
static unsigned long countGnssBytes() {
  delay(SETTLE_MS);
  while (Serial1.available())
    Serial1.read();

  unsigned long count = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < COUNT_MS) {
    while (Serial1.available()) {
      Serial1.read();
      count++;
    }
  }
  return count;
}

// Enable phases (hi-Z, HIGH): expect the GNSS to power up and stream.
static void runEnablePhase(const char *label, EnMode mode) {
  applyEn(mode);
  Serial.printf("\nEN -> %s\n", label);
  Serial.println("  probe TPS63020 VOUT now (~3.3 V = up, ~0 V = cut)");
  unsigned long bytes = countGnssBytes();
  Serial.printf("  GNSS UART: %lu bytes in ~%lu s -> %s\n", bytes,
                (unsigned long)(COUNT_MS / 1000),
                bytes ? "POWERED - OK"
                      : "SILENT - unexpected (check GNSS wiring / baud)");
}

// LOW phase with the two-part RX back-feed test.
static void runCutoffPhase() {
  applyEn(EN_LOW);
  Serial.println("\nEN -> LOW  (the cutoff -> rail should be OFF)");

  // Part 1: TX still idling high -> the RX back-feed path is intact.
  Serial.println(
      "  [part 1] TX (D6) idling HIGH. If the M100 POWER LED is still");
  Serial.println(
      "           lit here, VCC is back-fed through RX, not the rail.");
  unsigned long bytes = countGnssBytes();
  Serial.printf("  GNSS UART: %lu bytes -> %s\n", bytes,
                bytes ? "still talking (rail likely still up)"
                      : "silent (receiver not running)");

  // Part 2: force TX (D6) low to remove the back-feed path.
  Serial.println(
      "  [part 2] forcing TX (D6) LOW to cut the back-feed - WATCH the");
  Serial.println(
      "           M100 POWER LED: if it now goes OUT, the rail is truly");
  Serial.println("           cut and the earlier glow was back-feed via RX.");
  Serial1.end(); // release D6/D7 from the UART so D6 can be driven as GPIO
  pinMode(GNSS_TX_PIN, OUTPUT);
  digitalWrite(GNSS_TX_PIN, LOW);
  delay(HOLD_MS);

  // Restore the UART for the next cycle.
  pinMode(GNSS_TX_PIN, INPUT);
  Serial1.begin(GNSS_UART_BAUD);
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
  }
  Serial1.begin(
      GNSS_UART_BAUD); // Serial1 = D6 (TX) / D7 (RX), fixed on the nRF52

  Serial.println("\n=== GNSS EN-pad cutoff check (+ RX back-feed test) ===");
  Serial.println(
      "Watch the M100's POWER and PPS LEDs (a meter on VOUT is a bonus).");
  Serial.println(
      "Goal: in the LOW state part 2 (TX forced low) the POWER LED goes");
  Serial.println(
      "out - confirming EN cuts the rail and the earlier glow was RX");
  Serial.println(
      "back-feed. A real firmware cutoff must then idle TX low too.");
}

void loop() {
  runEnablePhase("hi-Z (released; board pullup should hold it ON)", EN_HIZ);
  runEnablePhase("HIGH (active-high enable -> ON)", EN_HIGH);
  runCutoffPhase();
  Serial.println("\n---- cycle complete; repeating ----");
}
