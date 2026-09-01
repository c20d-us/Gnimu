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
// DIAGNOSTIC: g_display screen-layout mockup
//
// Renders the proposed per-state screens with FAKE data so the layout can be
// judged on real glass before any g_display code exists. Layout decisions at
// 128x64 do not survive being reasoned about on paper - font legibility and
// density have to be looked at.
//
// Structure under test:
//   - A persistent STATUS BAR (rows 0..11): BLE icon, status label, battery
//     percent + drawn bar. Same in every state that has a battery in circuit.
//     In RUNNING the label is the BLE link state ("Connected"/"Advertising"),
//     not the state name - see statusBar() for why.
//   - A per-state BODY (rows 14..63) showing only what that state can know.
//     RUNNING has GNSS detail; CHARGE_ONLY has no fix data at all because the
//     GNSS is held off; BATTERY_WAIT drops the status bar entirely since the
//     cell is switched out of circuit and its percentage is meaningless.
//
// THREE THINGS TO LOOK FOR:
//   1. LEGIBILITY at arm's length, and in bright light.
//   2. WORST-CASE WIDTHS ('w', 2nd position): maximum-width values (2-digit
//      SV, 2-digit pDOP, 4-digit hAcc, 3-digit percent, long runtime). Layouts
//      look fine on typical data and break on the extremes.
//   3. NO FIX ('w', 3rd position): the widest fix string ("No Fix" is 60px at
//      10x20, against "11 SV" ending at 52), AND the semantic case - pDOP and
//      hAcc render as "--" because without a solution they are meaningless.
//
// BURN-IN / PIXEL SHIFT: every draw routes through ox()/oy(), so the whole
// layout can be nudged a few pixels on a slow cycle to avoid differential
// aging over an 8-hour run. Press 'j' to step the offset manually and confirm
// nothing clips at the extremes. The shift is positive-only into a 2px gutter
// at the right and bottom - see the SHIFT_X/SHIFT_Y comment for why.
// Note the interaction with partial updates: a shift dirties the whole frame,
// so production should shift rarely and do one full redraw at each shift.
//
// SERIAL COMMANDS (115200):
//   1 RUNNING (connected)   2 RUNNING (advertising)   3 CHARGE_ONLY
//   4 LIGHT_SLEEP           5 BATTERY_WAIT            6 DEEP_SLEEP (blank)
//   w cycle dataset: typical / worst-case widths / no fix
//   c toggle charging bolt          j step pixel-shift offset
//   a auto-cycle states             ? help
//
// Requires: XIAO nRF52840 Sense + SSD1306 128x64 I2C module, u8g2 library.
// Wiring: 3V3/GND, D4=SDA, D5=SCL.
// ============================================================================

#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

static const uint8_t OLED_ADDR = 0x3C; // bench-confirmed on both modules

// Open Iconic "embedded" subset glyph encodings (verified against the upstream
// open_iconic_embedded_1x.bdf, not guessed - a wrong encoding fails silently by
// drawing nothing rather than erroring).
static const uint16_t ICON_BLUETOOTH = 74;

// Charging bolt, hand-drawn rather than Open Iconic's "flash" glyph (enc 67).
// That glyph is a downscale of a vector icon and a bolt is precisely the shape
// that does not survive it - the diagonal strokes turn to mush at 8x8. At this
// size every pixel has to be placed deliberately.
//
// 6 wide x 8 tall, XBM row order, LSB = leftmost pixel. The middle row is the
// jog that makes it read as a bolt rather than a plain zigzag: the upper half
// slants down-left, the bar carries across, and the lower half restarts from
// the right and slants down-left again.
//
//     . . . # # .
//     . . # # . .
//     . # # . . .
//     # # # # # .   <- jog
//     . . . # # .
//     . . # # . .
//     . # # . . .
//     # # . . . .
static const uint8_t BOLT_XBM[] = {0x18, 0x0C, 0x06, 0x1F,
                                   0x18, 0x0C, 0x06, 0x03};
static const int BOLT_W = 6, BOLT_H = 8;

// Charging indicator, toggled with 'c'. CHARGE_ONLY forces it on since that
// state is charging by definition; every other screen follows the toggle.
//
// This is not only a bench-mode case. With the shipped CHARGE_ONLY_ON_USB=1,
// plugging in during RUNNING diverts to CHARGE_ONLY - but LIGHT_SLEEP has no
// USB guard in its transition table, so plugging in while asleep leaves the
// device in LIGHT_SLEEP with the charger running. There the bolt is the only
// on-screen indication that anything is happening.
static bool charging = false;

// --- Pixel-shift (burn-in mitigation) -------------------------------------
// Production would step this on a timer. Here it is manual so the extremes
// can be checked deliberately.
//
// The offset is POSITIVE-ONLY (0..SHIFT_MAX on both axes). The layout is
// designed for a 126x62 content area and the shift slides it into a 2px
// gutter along the right and bottom edges. A symmetric +/- shift was tried
// first and clipped anything sitting at x=0 or y=0 - notably the status bar's
// Bluetooth icon - because it pushed content off the top-left, where there is
// no gutter to give. Keeping the shift one-directional means layout code only
// has to reserve slack on two edges instead of four.
//
// Walks the perimeter of a 3x3 grid so every element visits 8 distinct pixel
// positions rather than just sliding along one diagonal.
#define SHIFT_MAX 2
#define LAYOUT_W (128 - SHIFT_MAX)
#define LAYOUT_H (64 - SHIFT_MAX)
static int8_t shiftIdx = 0;
static const int8_t SHIFT_X[] = {0, 1, 2, 2, 2, 1, 0, 0};
static const int8_t SHIFT_Y[] = {0, 0, 0, 1, 2, 2, 2, 1};
static inline int ox(int x) { return x + SHIFT_X[shiftIdx]; }
static inline int oy(int y) { return y + SHIFT_Y[shiftIdx]; }

// --- Fake telemetry -------------------------------------------------------
// Three datasets, cycled with 'w':
//   0 typical    - representative good fix
//   1 worst case - maximum field widths, to catch collisions
//   2 no fix     - not just a width case. Without a solution the receiver's
//                  pDOP and hAcc are meaningless (u-blox reports a sentinel
//                  hAcc and a nonsense pDOP), so they must render as "--"
//                  rather than showing garbage dressed up as data. The PVT
//                  rate stays valid - NAV-PVT keeps arriving without a fix.
static uint8_t dataSet = 0;
struct Fake {
  uint8_t sv;
  const char *fix;
  bool hasFix;
  float pdop;
  uint16_t hAccMm;
  float rateHz;
  uint8_t battPct;
  float battV;
  const char *runtime;
};
static Fake typical = {11, "3D", true, 1.42f, 248, 25.0f, 87, 4.02f, "1:23:45"};
static Fake extreme = {24, "3D", true, 19.99f, 9999, 25.0f, 100, 4.20f, "23:59:59"};
static Fake nofix = {0, "No Fix", false, 0.0f, 0, 25.0f, 87, 4.02f, "0:00:12"};

// Deliberately NOT a `static Fake &data()` accessor: the Arduino IDE
// auto-generates function prototypes and inserts them ABOVE the type
// definitions in a .ino, so any function returning a user-defined type fails
// with "'Fake' does not name a type". Callers take the reference locally
// instead - a local declaration inside a function body is never hoisted.
#define FAKE_DATA (dataSet == 0 ? typical : dataSet == 1 ? extreme : nofix)

// --- Drawing helpers ------------------------------------------------------
static void strAt(int x, int y, const char *s) { display.drawStr(ox(x), oy(y), s); }

// Right-aligned to a given right edge, using real glyph metrics rather than
// assumed advance widths (fonts are not all fixed-pitch).
static void strRight(int rightX, int y, const char *s) {
  display.drawStr(ox(rightX - display.getStrWidth(s)), oy(y), s);
}

// Centred within the layout area (not the panel), so text stays centred as the
// pixel shift moves the whole layout. Set the font before calling.
static void strCenter(int y, const char *s) {
  display.drawStr(ox((LAYOUT_W - display.getStrWidth(s)) / 2), oy(y), s);
}

// Proportionally-filled battery: preferred over an icon glyph, since g_battery
// produces a continuous 0-100%.
static void batteryBar(int x, int y, int w, int h, uint8_t pct) {
  display.drawFrame(ox(x), oy(y), w, h);
  display.drawBox(ox(x + w), oy(y + h / 4), 2, h / 2); // terminal nub
  int inner = w - 4;
  int fill = (int)((inner * (uint16_t)pct) / 100);
  if (fill > 0)
    display.drawBox(ox(x + 2), oy(y + 2), fill, h - 4);
}

// Status bar: BLE icon + status label on the left, percent + bar on the right.
//
// In RUNNING the label carries the BLE link state ("Connected"/"Advertising")
// rather than the state name - RUNNING is the implicit default and spending
// scarce bar width restating it is wasteful, whereas the link state is the
// thing that actually changes and that the user is checking for.
static void statusBar(const char *label, bool bleUp, bool isCharging) {
  Fake &d = FAKE_DATA;
  display.setFont(u8g2_font_open_iconic_embedded_1x_t);
  if (bleUp)
    display.drawGlyph(ox(0), oy(8), ICON_BLUETOOTH);
  // Bolt sits immediately left of the percent, at x=71..76.
  //
  // It is suppressed at 100%: a topped-up cell is not charging in any sense
  // the user cares about, and CHARGE_ONLY's label already says "Full". That
  // gate also buys back layout width - the bolt can never coexist with the
  // widest percentage ("100%"), so the worst case it has to clear is "87%"
  // starting at x=79. The battery bar had been narrowed 26px -> 22px to open
  // an 8px slot for it; with the gate in place that is unnecessary and the bar
  // is back to its full 26px.
  //
  // drawXBM's y is the bitmap TOP, unlike drawGlyph's baseline. y=1 puts the
  // bolt on the same rows as the 8px Bluetooth icon.
  //
  // Production note: gate on g_battery's voltage-based `full` flag rather than
  // percent, for the same reason the "Full" label does.
  if (isCharging && d.battPct < 100)
    display.drawXBM(ox(71), oy(1), BOLT_W, BOLT_H, BOLT_XBM);

  display.setFont(u8g2_font_5x7_tf);
  strAt(11, 8, label);

  char pct[8];
  snprintf(pct, sizeof(pct), "%u%%", d.battPct);
  strRight(94, 8, pct);
  batteryBar(98, 1, 26, 9, d.battPct);

  display.drawHLine(ox(0), oy(12), LAYOUT_W);
}

// --- Per-state bodies -----------------------------------------------------

static void bodyRunning() {
  Fake &d = FAKE_DATA;
  char buf[24];

  display.setFont(u8g2_font_10x20_tf);
  snprintf(buf, sizeof(buf), "%u SV", d.sv);
  strAt(2, 32, buf);
  strRight(126, 32, d.fix);

  // Quality metrics are only meaningful with a solution - see the dataSet
  // comment. Without one they show "--" rather than the receiver's sentinel
  // values, which would otherwise read as real (very bad) numbers.
  display.setFont(u8g2_font_6x12_tf);
  if (d.hasFix)
    snprintf(buf, sizeof(buf), "pDOP %.2f", d.pdop);
  else
    snprintf(buf, sizeof(buf), "pDOP --");
  strAt(2, 46, buf);
  snprintf(buf, sizeof(buf), "%.1fHz", d.rateHz);
  strRight(126, 46, buf);

  if (d.hasFix)
    snprintf(buf, sizeof(buf), "hAcc %umm", d.hAccMm);
  else
    snprintf(buf, sizeof(buf), "hAcc --");
  strAt(2, 60, buf);
  strRight(126, 60, d.runtime);
}

// GNSS is held off in CHARGE_ONLY, so there is deliberately no fix data here.
// Just the cell voltage, large and centred: the percentage and the charging
// state both already live in the status bar, so repeating them in the body
// would spend the whole screen restating one fact.
//
// Big numeric font for the value, small font for the unit - a "_tn" font is a
// numerals-only subset (digits and punctuation, no letters), which keeps the
// flash cost down but means the "V" has to come from a text font. If
// logisoso24 is unavailable in your u8g2 build, u8g2_font_fub25_tn and
// u8g2_font_inb24_mn are drop-in alternates.
static void bodyChargeOnly() {
  Fake &d = FAKE_DATA;
  char buf[16];
  snprintf(buf, sizeof(buf), "%.2f", d.battV);

  display.setFont(u8g2_font_logisoso24_tn);
  int wNum = display.getStrWidth(buf);
  display.setFont(u8g2_font_7x14B_tf);
  int wUnit = display.getStrWidth("V");

  const int gap = 4;
  int x = (LAYOUT_W - (wNum + gap + wUnit)) / 2;
  const int baseline = 48; // centres the block in the body area (rows 14..61)

  display.setFont(u8g2_font_logisoso24_tn);
  strAt(x, baseline, buf);
  display.setFont(u8g2_font_7x14B_tf);
  strAt(x + wNum + gap, baseline, "V");
}

// Deliberately sparse: fewer lit pixels for power and burn-in during a long idle.
static void bodyLightSleep() {
  display.setFont(u8g2_font_7x14B_tf);
  strAt(2, 34, "IDLE - Sleeping");
  display.setFont(u8g2_font_5x7_tf);
  strAt(2, 52, "shake or connect to wake");
}

// Full-screen alert, no status bar: the cell is switched out of circuit here,
// so a battery percentage would be meaningless.
static void screenBatteryWait() {
  display.clearBuffer();
  display.drawBox(ox(0), oy(0), LAYOUT_W, 26);
  display.setDrawColor(0); // knock out of the filled block
  // 9x18B rather than 10x20: "Switch is OFF" is 13 chars, which needs 130px at
  // 10x20 against a 126px layout width. 9x18B fits it in 117px and stays bold.
  // The wording is deliberately an observation of state, not an imperative -
  // "SWITCH OFF" reads as an instruction to switch something off, which is the
  // opposite of what the user needs to do here. Fall back to 7x14B (91px) if
  // 9x18B is unavailable in your u8g2 build.
  display.setFont(u8g2_font_9x18B_tf);
  strCenter(20, "Switch is OFF");
  display.setDrawColor(1);
  display.setFont(u8g2_font_6x12_tf);
  strCenter(44, "Slide switch to ON");
  strCenter(58, "to run or charge");
  display.sendBuffer();
}

// --- Screen dispatch ------------------------------------------------------
static char mode = '1';

static void render() {
  if (mode == '5') {
    screenBatteryWait();
    return;
  }
  if (mode == '6') {
    display.setPowerSave(1); // SSD1306 DISPLAYOFF
    return;
  }
  display.setPowerSave(0);
  display.clearBuffer();
  switch (mode) {
  case '1':
    statusBar("Connected", true, charging);
    bodyRunning();
    break;
  case '2':
    statusBar("Advertising", true, charging);
    bodyRunning();
    break;
  case '3':
    // Mockup proxies "full" off the percentage so 'w' demonstrates both
    // labels. Production should use g_battery's own `full` flag instead - it
    // is voltage-based (BATTERY_FULL_V) rather than derived from percent.
    statusBar(FAKE_DATA.battPct >= 100 ? "Full" : "Charging", false, true);
    bodyChargeOnly();
    break;
  case '4':
    statusBar("Advertising", true, charging);
    bodyLightSleep();
    break;
  }
  display.sendBuffer();
}

static void help() {
  Serial.println("\n--- layout mockup ---");
  Serial.println("  1 RUNNING (BLE connected)   2 RUNNING (advertising)");
  Serial.println("  3 CHARGE_ONLY              4 LIGHT_SLEEP");
  Serial.println("  5 BATTERY_WAIT (alert)     6 DEEP_SLEEP (display off)");
  Serial.println("  w cycle dataset: typical / worst-case widths / no fix");
  Serial.println("  c toggle charging (lightning bolt in status bar)");
  Serial.println("  j step pixel-shift offset (burn-in mitigation)");
  Serial.println("  a auto-cycle states 1-5");
  Serial.println("  ? help\n");
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0) < 3000)
    delay(10);

  Serial.println("\n=== Gnimu OLED layout mockup ===");
  Wire.begin();
  display.setI2CAddress(OLED_ADDR << 1); // u8g2 wants the 8-bit form
  display.begin();
  display.setContrast(255);
  help();
  render();
}

void loop() {
  if (!Serial.available())
    return;
  char c = (char)Serial.read();
  switch (c) {
  case 'w': {
    dataSet = (dataSet + 1) % 3;
    static const char *names[] = {"typical", "WORST CASE (max widths)",
                                  "NO FIX"};
    Serial.printf("values: %s\n", names[dataSet]);
    break;
  }
  case 'c':
    charging = !charging;
    Serial.printf("charging: %s (CHARGE_ONLY always shows it)\n",
                  charging ? "YES" : "no");
    break;
  case 'j':
    shiftIdx = (shiftIdx + 1) % (int8_t)(sizeof(SHIFT_X));
    Serial.printf("pixel shift: dx=%d dy=%d\n", SHIFT_X[shiftIdx],
                  SHIFT_Y[shiftIdx]);
    break;
  case 'a':
    Serial.println("auto-cycling 1-5, 3s each");
    for (char m = '1'; m <= '5'; m++) {
      mode = m;
      render();
      delay(3000);
    }
    break;
  case '?':
    help();
    return;
  default:
    if (c >= '1' && c <= '6')
      mode = c;
    else
      return;
  }
  render();
}
