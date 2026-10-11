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
// DIAGNOSTIC: SSD1306 update-cost benchmark + partial-update validation
//
// Settles the display-library decision with numbers instead of argument.
// Two questions:
//
//   1. DOES updateDisplayArea() WORK on this panel? It is the only reason to
//      prefer u8g2 over Adafruit_SSD1306, which has no partial-update path.
//      Mode 'p' animates a counter inside one small region while the rest of
//      the screen holds a static pattern - any tearing, drift, or corruption
//      outside the updated region means partial updates are not usable and the
//      library choice collapses to a coin flip on familiarity.
//
//   2. WHAT DOES AN UPDATE COST, and can the GNSS survive it? This is a
//      correctness question, not an efficiency one. Per the base design, the
//      Serial1 RX buffer (~64 bytes) fills in roughly 5.5 ms at GNSS_BAUD
//      115200, and exceeding that drops NAV-PVT bytes and craters the observed
//      rate. A full 1024-byte frame at 400 kHz is ~24 ms - about 4x over
//      budget. So the shipped g_display MUST either update small regions, chunk
//      across loop iterations, or run the bus faster. Mode 'b' measures all of
//      it and prints each result against the RX budget.
//
// I2C CLOCK: keys '1'/'4'/'8' select 100 k / 400 k / 1 M. Note 400 kHz is the
// SSD1306 datasheet maximum - 1 MHz is out of spec and offered only to see
// whether these modules tolerate it, since it would roughly halve every number
// below. If 1 MHz shows corruption, that is the answer and it should not ship.
//
// SERIAL COMMANDS (115200):
//   b run the benchmark          p partial-update stress (visual check)
//   f full-frame animation, for comparison against 'p'
//   1 100kHz   4 400kHz   8 1MHz (out of spec)      ? help
//
// Requires: XIAO nRF52840 Sense + SSD1306 128x64 I2C module, u8g2 library.
// ============================================================================

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

static const uint8_t OLED_ADDR = 0x3C;

// The ~64-byte Serial1 RX buffer at 115200 8N1 gives about 5.5 ms before
// bytes are lost. Any blocking display write longer than this costs GNSS data.
static const uint32_t GNSS_RX_BUDGET_US = 5500;

static uint32_t busHz = 400000;

// Static backdrop so partial-update artefacts are obvious: a full-screen frame
// plus a hatch pattern. Anything that changes outside the updated region during
// mode 'p' is corruption.
static void drawBackdrop() {
  display.clearBuffer();
  display.drawFrame(0, 0, 128, 64);
  for (int x = 4; x < 124; x += 6)
    display.drawVLine(x, 4, 56);
  display.setFont(u8g2_font_5x7_tf);
  display.drawStr(6, 12, "static backdrop");
  display.sendBuffer();
}

static void verdict(const char *what, uint32_t us, int bytes) {
  Serial.printf("  %-26s %6lu us  (%4d B)  %s\n", what, (unsigned long)us,
                bytes,
                us <= GNSS_RX_BUDGET_US ? "OK - within GNSS RX budget"
                                        : "OVER BUDGET - would drop GNSS bytes");
}

// Tiles are 8x8 px, so tw/th are in tiles: 16 x 8 covers the whole panel.
static uint32_t timeArea(uint8_t tx, uint8_t ty, uint8_t tw, uint8_t th) {
  uint32_t t0 = micros();
  display.updateDisplayArea(tx, ty, tw, th);
  return micros() - t0;
}

static void benchmark() {
  Serial.printf("\n--- update cost @ %lu Hz (GNSS RX budget %lu us) ---\n",
                (unsigned long)busHz, (unsigned long)GNSS_RX_BUDGET_US);

  display.clearBuffer();
  display.drawBox(0, 0, 128, 64); // worst case: every byte non-zero
  uint32_t t0 = micros();
  display.sendBuffer();
  verdict("sendBuffer() full frame", micros() - t0, 1024);

  verdict("half screen  16x4 tiles", timeArea(0, 0, 16, 4), 16 * 4 * 8);
  verdict("one text row 16x2 tiles", timeArea(0, 2, 16, 2), 16 * 2 * 8);
  verdict("number field  4x2 tiles", timeArea(4, 2, 4, 2), 4 * 2 * 8);
  verdict("one page      16x1 tiles", timeArea(0, 3, 16, 1), 16 * 1 * 8);
  verdict("small field   2x1 tiles", timeArea(6, 3, 2, 1), 2 * 1 * 8);

  Serial.println("  (a chunked redraw would pay the one-page cost per loop)");
  drawBackdrop();
}

// Animate a counter inside ONE region. The backdrop must stay pristine.
static void partialStress() {
  Serial.println("\n[p] partial updates, 400 frames into a 4x2-tile region.");
  Serial.println("    Watch the backdrop - any change outside the box is a bug.");
  drawBackdrop();
  uint32_t total = 0;
  for (int i = 0; i < 400; i++) {
    // Redraw only the box contents in the buffer, then push only those tiles.
    display.setDrawColor(0);
    display.drawBox(32, 16, 32, 16);
    display.setDrawColor(1);
    display.drawFrame(32, 16, 32, 16);
    char buf[8];
    snprintf(buf, sizeof(buf), "%03d", i % 1000);
    display.setFont(u8g2_font_6x12_tf);
    display.drawStr(36, 28, buf);
    uint32_t t0 = micros();
    display.updateDisplayArea(4, 2, 4, 2);
    total += micros() - t0;
    delay(5);
  }
  Serial.printf("    mean partial update: %lu us over 400 frames\n",
                (unsigned long)(total / 400));
  Serial.println("    If the backdrop is intact, partial updates are usable.");
}

// Same animation but pushing the whole frame each time - the Adafruit-equivalent
// cost, for comparison.
static void fullStress() {
  Serial.println("\n[f] full-frame updates, 200 frames. Compare cost with 'p'.");
  uint32_t total = 0;
  for (int i = 0; i < 200; i++) {
    drawBackdrop(); // includes its own sendBuffer, so time the pair below
    display.setDrawColor(0);
    display.drawBox(32, 16, 32, 16);
    display.setDrawColor(1);
    display.drawFrame(32, 16, 32, 16);
    char buf[8];
    snprintf(buf, sizeof(buf), "%03d", i % 1000);
    display.setFont(u8g2_font_6x12_tf);
    display.drawStr(36, 28, buf);
    uint32_t t0 = micros();
    display.sendBuffer();
    total += micros() - t0;
    delay(5);
  }
  Serial.printf("    mean full frame: %lu us over 200 frames\n",
                (unsigned long)(total / 200));
}

static void setBus(uint32_t hz) {
  busHz = hz;
  display.setBusClock(hz);
  Serial.printf("\nI2C clock = %lu Hz%s\n", (unsigned long)hz,
                hz > 400000 ? "  (OUT OF SPEC - watch for corruption)" : "");
  drawBackdrop();
}

static void help() {
  Serial.println("\n--- commands ---");
  Serial.println("  b  benchmark update costs at the current bus clock");
  Serial.println("  p  partial-update stress (validates updateDisplayArea)");
  Serial.println("  f  full-frame stress, for cost comparison");
  Serial.println("  1  100 kHz    4  400 kHz (datasheet max)    8  1 MHz");
  Serial.println("  ?  help\n");
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0) < 3000)
    delay(10);

  Serial.println("\n=== Gnimu OLED update benchmark ===");
  Wire.begin();
  display.setI2CAddress(OLED_ADDR << 1);
  display.begin();
  display.setContrast(255);
  setBus(busHz);
  help();
}

void loop() {
  if (!Serial.available())
    return;
  switch ((char)Serial.read()) {
  case 'b': benchmark(); break;
  case 'p': partialStress(); break;
  case 'f': fullStress(); break;
  case '1': setBus(100000); break;
  case '4': setBus(400000); break;
  case '8': setBus(1000000); break;
  case '?': help(); break;
  default: break;
  }
}
