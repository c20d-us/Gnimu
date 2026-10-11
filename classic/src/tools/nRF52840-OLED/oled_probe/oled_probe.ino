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
// DIAGNOSTIC: SSD1306 128x64 OLED bring-up + power characterization
//
// First-light test for the OLED added in the nRF52840-OLED variant. Answers the
// open items that had to be settled on hardware before any g_display code was
// worth writing:
//
//   1. I2C ADDRESS + BUS. Raw Wire scan BEFORE any display library loads, so a
//      wiring/power fault is distinguishable from a library problem. These
//      modules are 0x3C or 0x3D depending on an onboard jumper/resistor.
//   2. GEOMETRY. A border + corner ticks + center crosshair confirms all
//      128x64 pixels are addressable and the controller variant is right (a
//      128x32 panel or a wrong "NONAME" variant shows up immediately here).
//   3. ICON / FONT capability - draws the Open Iconic Bluetooth glyph and a
//      proportionally-filled battery bar, the two rendering approaches
//      the variant settled on (icon font for static marks, drawn primitives
//      for anything with a continuous value).
//   4. POWER. Holds discrete states on command so a meter inline with the
//      module's VCC can be read at each: all-pixels-on (worst case), sparse
//      content (realistic), all-pixels-off-but-active (controller baseline),
//      and DISPLAYOFF (sleep). The DISPLAYOFF number is the one that decides
//      whether a load switch on the panel's supply is worth its parts.
//   5. SUNLIGHT READABILITY - mode 'r' parks a high-contrast screen so the
//      unit can be carried outdoors without a host attached.
//
// WIRING (breadboard, matching the variant's as-built wiring):
//   XIAO 3V3 -> OLED VCC        XIAO D4 (SDA) -> OLED SDA
//   XIAO GND -> OLED GND        XIAO D5 (SCL) -> OLED SCL
//
// SERIAL COMMANDS (115200) - each mode HOLDS until the next key, so there is
// time to read a meter:
//   1 geometry    2 text     3 icons+battery    4 all pixels ON
//   5 all pixels OFF (active)  6 DISPLAYOFF (sleep)  7 contrast sweep
//   r readability (outdoor)    s re-scan I2C bus     ? help
//
// Requires: XIAO nRF52840 Sense + one SSD1306 128x64 I2C module.
// Library: u8g2 (olikraus) via Library Manager. u8g2 is used rather than
// Adafruit_SSD1306 because the library choice was still open and
// it hinges on u8g2's updateDisplayArea() partial-update support - so the
// bring-up sketch may as well exercise the candidate. Nothing here depends on
// u8g2 specifically except the icon font in mode 3.
// ============================================================================

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

// Full-buffer (F) hardware-I2C constructor: 1KB RAM for the framebuffer, which
// is nothing against the nRF52840's 256KB, and it is the mode that supports
// updateDisplayArea() partial updates. U8X8_PIN_NONE = no reset pin broken out
// on a 4-pin module.
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

// Candidate 7-bit addresses for these modules.
static const uint8_t ADDR_CANDIDATES[] = {0x3C, 0x3D};

static uint8_t foundAddr = 0;   // 7-bit, 0 = none found
static char currentMode = '1';  // so 'redraw' can repaint after a contrast change
static uint8_t contrast = 255;

// ---------------------------------------------------------------------------
// I2C scan - deliberately raw Wire, no display library involved. If this finds
// nothing, the fault is power or wiring; there is no point debugging u8g2.
// ---------------------------------------------------------------------------
static uint8_t scanBus() {
  Serial.println("\n--- I2C scan (raw Wire, no display library) ---");
  uint8_t hits = 0, first = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      bool expected = false;
      for (uint8_t c : ADDR_CANDIDATES)
        if (addr == c)
          expected = true;
      Serial.printf("  0x%02X  ACK%s\n", addr,
                    expected ? "   <- SSD1306 candidate" : "");
      if (!first)
        first = addr;
      hits++;
    }
  }
  if (!hits) {
    Serial.println("  (nothing responded)");
    Serial.println("  Check: VCC on 3V3, GND common, SDA=D4, SCL=D5, and that");
    Serial.println("  the module is the 4-pin I2C variant (not jumpered SPI).");
  } else {
    Serial.printf("  %u device(s) found.\n", hits);
  }
  return first;
}

// ---------------------------------------------------------------------------
// Test patterns. Each leaves a static image on the panel and returns.
// ---------------------------------------------------------------------------

// Geometry: outline + corner ticks + crosshair. Any missing edge, wrapped row,
// or half-height image means the wrong controller variant is selected.
static void drawGeometry() {
  display.clearBuffer();
  display.drawFrame(0, 0, 128, 64);
  // Corner ticks - if a corner pixel is missing the addressing is off by one.
  display.drawPixel(0, 0);
  display.drawPixel(127, 0);
  display.drawPixel(0, 63);
  display.drawPixel(127, 63);
  display.drawLine(0, 32, 127, 32);
  display.drawLine(64, 0, 64, 63);
  display.setFont(u8g2_font_5x7_tf);
  display.drawStr(4, 12, "128x64");
  display.drawStr(88, 60, "EDGES");
  display.sendBuffer();
}

// Font sizes, to judge what is legible at arm's length in a car.
static void drawText() {
  display.clearBuffer();
  display.setFont(u8g2_font_5x7_tf);
  display.drawStr(0, 8, "5x7  SV 11  pDOP 1.42");
  display.setFont(u8g2_font_6x12_tf);
  display.drawStr(0, 22, "6x12 hAcc 248mm");
  display.setFont(u8g2_font_7x14B_tf);
  display.drawStr(0, 38, "7x14B 25.0Hz");
  display.setFont(u8g2_font_10x20_tf);
  display.drawStr(0, 58, "10x20 3D");
  display.sendBuffer();
}

// A proportionally-filled battery bar - preferred over an icon glyph, since
// g_battery produces a continuous 0-100%.
static void drawBatteryBar(int x, int y, int w, int h, uint8_t pct) {
  display.drawFrame(x, y, w, h);                    // body
  display.drawBox(x + w, y + (h / 4), 2, h / 2);    // terminal nub
  int inner = w - 4;
  int fill = (inner * pct) / 100;
  if (fill > 0)
    display.drawBox(x + 2, y + 2, fill, h - 4);
}

// Icon font + drawn battery, side by side. Answers "can I get a Bluetooth
// symbol" concretely: glyph 74 of open_iconic_embedded_2x is the BT mark.
static void drawIconsAndBattery() {
  display.clearBuffer();

  display.setFont(u8g2_font_open_iconic_embedded_2x_t);
  display.drawGlyph(4, 20, 74); // Bluetooth

  display.setFont(u8g2_font_6x12_tf);
  display.drawStr(28, 18, "BLE connected");

  // Three fill levels to confirm the bar renders sensibly across its range.
  drawBatteryBar(4, 30, 40, 12, 100);
  drawBatteryBar(4, 48, 40, 12, 45);
  display.setFont(u8g2_font_5x7_tf);
  display.drawStr(52, 40, "100%");
  display.drawStr(52, 58, "45%");
  display.sendBuffer();
}

// Worst-case current: every pixel lit.
static void drawAllOn() {
  display.clearBuffer();
  display.drawBox(0, 0, 128, 64);
  display.sendBuffer();
}

// Controller active, zero pixels lit - the baseline the panel itself costs.
static void drawAllOff() {
  display.clearBuffer();
  display.sendBuffer();
}

// Large, sparse, maximum contrast - for carrying outdoors.
static void drawReadability() {
  display.clearBuffer();
  display.setFont(u8g2_font_10x20_tf);
  display.drawStr(2, 20, "SV 11  3D");
  display.drawStr(2, 42, "1.42 pDOP");
  display.setFont(u8g2_font_6x12_tf);
  display.drawStr(2, 60, "248mm  25.0Hz  87%");
  display.sendBuffer();
}

// Steps contrast so its effect on both legibility and current is visible.
static void contrastSweep() {
  Serial.println("Contrast sweep 0->255, 3s per step. Watch current + legibility.");
  const uint8_t steps[] = {0, 32, 64, 128, 192, 255};
  for (uint8_t s : steps) {
    display.setContrast(s);
    Serial.printf("  contrast = %3u\n", s);
    delay(3000);
  }
  contrast = 255;
  display.setContrast(contrast);
  Serial.println("  restored to 255");
}

static void help() {
  Serial.println("\n--- commands ---");
  Serial.println("  1  geometry: frame, corner ticks, crosshair");
  Serial.println("  2  text: four font sizes");
  Serial.println("  3  icons (Bluetooth glyph) + drawn battery bars");
  Serial.println("  4  ALL PIXELS ON      <- worst-case current");
  Serial.println("  5  all pixels off, controller active  <- baseline current");
  Serial.println("  6  DISPLAYOFF (sleep) <- the load-switch decision number");
  Serial.println("  7  contrast sweep 0..255");
  Serial.println("  r  readability screen (take it outside)");
  Serial.println("  s  re-scan the I2C bus");
  Serial.println("  ?  this help");
  Serial.println("Modes hold until the next key - read the meter at leisure.\n");
}

static void runMode(char c) {
  switch (c) {
  case '1':
    Serial.println("[1] geometry");
    display.setPowerSave(0);
    drawGeometry();
    break;
  case '2':
    Serial.println("[2] text");
    display.setPowerSave(0);
    drawText();
    break;
  case '3':
    Serial.println("[3] icons + battery bars");
    display.setPowerSave(0);
    drawIconsAndBattery();
    break;
  case '4':
    Serial.println("[4] ALL PIXELS ON - measure now (worst case)");
    display.setPowerSave(0);
    drawAllOn();
    break;
  case '5':
    Serial.println("[5] all pixels off, controller ACTIVE - measure now");
    display.setPowerSave(0);
    drawAllOff();
    break;
  case '6':
    Serial.println("[6] DISPLAYOFF / sleep - measure now (expect ~10uA, but");
    Serial.println("    the module's own regulator may dominate: that is the");
    Serial.println("    number the load-switch decision turns on)");
    display.setPowerSave(1); // u8g2 wrapper for SSD1306 DISPLAYOFF (0xAE)
    break;
  case '7':
    contrastSweep();
    runMode(currentMode); // repaint whatever was showing
    return;
  case 'r':
    Serial.println("[r] readability screen - unplug and take it into the sun");
    display.setPowerSave(0);
    drawReadability();
    break;
  case 's':
    foundAddr = scanBus();
    return;
  case '?':
    help();
    return;
  default:
    return; // ignore newlines and stray characters
  }
  currentMode = c;
}

void setup() {
  Serial.begin(115200);
  // Battery-less bench sketch, USB always present: a bounded wait is fine here
  // (the shipped firmware must never gate startup on Serial).
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0) < 3000)
    delay(10);

  Serial.println("\n=== Gnimu OLED probe (SSD1306 128x64 I2C) ===");

  Wire.begin();
  foundAddr = scanBus();
  if (!foundAddr) {
    Serial.println("\nNo I2C device - fix wiring/power before continuing.");
    Serial.println("Press 's' to re-scan once corrected.");
  } else {
    // u8g2 wants the 8-bit (left-shifted) form here. Passing the 7-bit value
    // is the classic silent failure: init returns, nothing appears.
    display.setI2CAddress(foundAddr << 1);
    display.begin();
    display.setContrast(contrast);
    Serial.printf("\nDisplay initialized at 0x%02X.\n", foundAddr);
    help();
    runMode('1');
  }
}

void loop() {
  if (Serial.available())
    runMode((char)Serial.read());
}
