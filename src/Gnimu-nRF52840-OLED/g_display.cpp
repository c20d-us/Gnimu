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

#include "g_display.h"
#include "config.h"

// See DISPLAY_ENABLED in config.h. When 0, none of this module's real
// implementation is compiled in.
#if DISPLAY_ENABLED

#include "g_battery.h"
#include "g_ble.h"
#include "g_gnss.h"
#include "g_log.h"
#include "g_state.h"
#include "g_telemetry.h"
#include <U8g2lib.h>
#include <Wire.h>

// Full-buffer (F) hardware-I2C driver.
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

static bool present = false; // panel answered at begin(); false disables all
static bool asleep = false;  // DISPLAYOFF sent
static unsigned long lastRenderMs = 0;
static unsigned long lastShiftMs = 0;
static unsigned long lastSliceMs = 0;

// Push cursor: -1 = nothing pending, otherwise the next slice index. A frame is
// DISPLAY_TILES_W / DISPLAY_CHUNK_TILES_W slices per tile row.
static const int SLICES_PER_ROW = DISPLAY_TILES_W / DISPLAY_CHUNK_TILES_W;
static const int SLICE_COUNT = SLICES_PER_ROW * DISPLAY_TILES_H;
static int pushCursor = -1;

// --- Burn-in pixel shift ----------------------------------------------------
// Walks the perimeter of a 3x3 grid so every element visits 8 distinct pixel
// positions rather than sliding along one diagonal.
static uint8_t shiftIdx = 0;
static const int8_t SHIFT_X[8] = {0, 1, 2, 2, 2, 1, 0, 0};
static const int8_t SHIFT_Y[8] = {0, 0, 0, 1, 2, 2, 2, 1};
static inline int ox(int x) { return x + SHIFT_X[shiftIdx]; }
static inline int oy(int y) { return y + SHIFT_Y[shiftIdx]; }

// --- USB plug icon ----------------------------------------------------------
// Hand-drawn, like everything small on this panel: Open Iconic's "embedded"
// subset has no USB glyph.
//
//     . # # . # # .
//     . # # . # # .
//     # # # # # # #
//     # # # # # # #
//     # # # # # # #
//     . # # # # # .
//     . . # # # . .
//     . . # # # . .
static const uint8_t USB_XBM[] = {0x36, 0x36, 0x7F, 0x7F,
                                  0x7F, 0x3E, 0x1C, 0x1C};
static const int USB_W = 7, USB_H = 8;

static const uint16_t ICON_BLUETOOTH = 74; // open_iconic_embedded encoding

// --- Draw helpers -----------------------------------------------------------
static void strAt(int x, int y, const char *s) {
  oled.drawStr(ox(x), oy(y), s);
}

// Right-aligned and centred variants measure with getStrWidth() rather than
// assuming a fixed advance - not every u8g2 "_tf" font is fixed-pitch. Centring
// is relative to the LAYOUT area, not the panel, so text stays centred as the
// pixel shift moves everything.
static void strRight(int rightX, int y, const char *s) {
  oled.drawStr(ox(rightX - (int)oled.getStrWidth(s)), oy(y), s);
}
static void strCenter(int y, const char *s) {
  oled.drawStr(ox((DISPLAY_LAYOUT_W - (int)oled.getStrWidth(s)) / 2), oy(y), s);
}

// Proportionally-filled battery. Drawn rather than taken from an icon font:
// g_battery produces a continuous 0-100%, while icon glyphs offer only
// empty/half/full.
static void batteryBar(int x, int y, int w, int h, uint8_t pct) {
  oled.drawFrame(ox(x), oy(y), w, h);
  oled.drawBox(ox(x + w), oy(y + h / 4), 2, h / 2); // terminal nub
  const int inner = w - 4;
  const int fill = (int)((inner * (uint16_t)pct) / 100);
  if (fill > 0) {
    oled.drawBox(ox(x + 2), oy(y + 2), fill, h - 4);
  }
}

// --- Status bar -------------------------------------------------------------
// BLE icon + label on the left; USB icon + battery gauge on the right.
//
// In RUNNING the label carries the BLE link state rather than the state name:
// RUNNING is the implicit default, and spending scarce bar width restating it
// is wasteful when the link state is the thing that changes and the thing the
// user is checking for.
//
// The battery shows as a gauge only - no percentage. The bar conveys the level
// well enough at a glance, and a number that close to the noise floor of a
// voltage-derived SoC estimate implied more precision than exists. Dropping it
// also freed the width the USB icon now uses.
static void drawStatusBar(const char *label, bool bleUp,
                          const BatteryStatus &bat) {
  if (bleUp) {
    oled.setFont(u8g2_font_open_iconic_embedded_1x_t);
    oled.drawGlyph(ox(0), oy(8), ICON_BLUETOOTH);
  }

  oled.setFont(u8g2_font_5x7_tf);
  strAt(11, 8, label);

  // bat.charging is `usbPresent && switchOn`. Every state that draws a status
  // bar already has the switch on (switch-off routes to BATTERY_WAIT, which
  // draws no bar), so within this function it is exactly "USB plugged in".
  if (bat.charging) {
    oled.drawXBM(ox(88), oy(1), USB_W, USB_H, USB_XBM);
  }

  batteryBar(98, 1, 26, 9, bat.percent);

  oled.drawHLine(ox(0), oy(12), DISPLAY_LAYOUT_W);
}

// --- Bodies -----------------------------------------------------------------

// Map u-blox fixType to something short enough for the headline. The protocol
// packer clamps this to {0,2,3} for the wire format; that clamp belongs to the
// wire and must not leak here.
static const char *fixLabel(uint8_t fixType) {
  switch (fixType) {
  case 2:
    return "2D";
  case 3:
    return "3D";
  case 4:
    return "3D"; // GNSS + dead reckoning; no DR sensors on the M100
  default:
    return "No Fix";
  }
}

static void drawRunningBody() {
  const UBX_NAV_PVT_data_t *pvt = gnssLatestPvt();
  const uint8_t fixType = pvt ? pvt->fixType : 0;

  // Position-valid predicate, deliberately identical to the one g_telemetry
  // uses to flag Lat/Lon invalid in the packet, so screen and packet can never
  // disagree about whether a position exists. Note a 2D fix IS valid: hAcc and
  // pDOP are real numbers there, merely worse.
  const bool posValid = pvt && fixType >= 2;

  char buf[24];
  oled.setFont(u8g2_font_10x20_tf);
  snprintf(buf, sizeof(buf), "%u SV", pvt ? pvt->numSV : 0);
  strAt(2, 32, buf);
  strRight(126, 32, fixLabel(fixType));

  oled.setFont(u8g2_font_6x12_tf);
  // Without a solution the receiver reports a sentinel hAcc and a meaningless
  // pDOP. Rendering them raw would put plausible-looking numbers on screen that
  // read as "the fix is poor" rather than "there is no fix".
  if (posValid) {
    snprintf(buf, sizeof(buf), "pDOP %.2f", pvt->pDOP / 100.0f);
  } else {
    snprintf(buf, sizeof(buf), "pDOP --");
  }
  strAt(2, 46, buf);

  // The PVT rate stays meaningful without a fix - NAV-PVT keeps arriving at the
  // configured rate regardless of solution status.
  snprintf(buf, sizeof(buf), "%.1fHz", telemetryGnssRateHz());
  strRight(126, 46, buf);

  if (posValid) {
    snprintf(buf, sizeof(buf), "hAcc %lumm", (unsigned long)pvt->hAcc);
  } else {
    snprintf(buf, sizeof(buf), "hAcc --");
  }
  strAt(2, 60, buf);

  const unsigned long secs = millis() / 1000UL;
  snprintf(buf, sizeof(buf), "%lu:%02lu:%02lu", secs / 3600UL,
           (secs / 60UL) % 60UL, secs % 60UL);
  strRight(126, 60, buf);
}

// GNSS is held off in CHARGE_ONLY, so there is deliberately no fix data here.
// Percentage and charge state already live in the status bar, so the body
// carries only the voltage - large and centred.
static void drawChargeOnlyBody(const BatteryStatus &bat) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%.2f", bat.voltage);

  // "_tn" is a numerals-only font subset (digits and punctuation, no letters),
  // which keeps the flash cost down but means the unit needs a text font.
  oled.setFont(u8g2_font_logisoso24_tn);
  const int wNum = (int)oled.getStrWidth(buf);
  oled.setFont(u8g2_font_7x14B_tf);
  const int wUnit = (int)oled.getStrWidth("V");

  const int gap = 4;
  const int x = (DISPLAY_LAYOUT_W - (wNum + gap + wUnit)) / 2;
  const int baseline = 48; // centres the block in the body area (rows 14..61)

  oled.setFont(u8g2_font_logisoso24_tn);
  strAt(x, baseline, buf);
  oled.setFont(u8g2_font_7x14B_tf);
  strAt(x + wNum + gap, baseline, "V");
}

// Deliberately sparse: fewer lit pixels for power and burn-in during what may
// be a long idle.
static void drawLightSleepBody() {
  oled.setFont(u8g2_font_7x14B_tf);
  strAt(2, 34, "IDLE - Sleeping");
  oled.setFont(u8g2_font_5x7_tf);
  strAt(2, 52, "shake or connect to wake");
}

// Full-screen alert, no status bar: the slide switch has taken the cell out of
// circuit, so a battery percentage would be meaningless. Wording is an
// observation of state - an imperative ("SWITCH OFF") reads as an instruction
// to switch something off, the opposite of the required action.
static void drawBatteryWaitScreen() {
  oled.drawBox(ox(0), oy(0), DISPLAY_LAYOUT_W, 26);
  oled.setDrawColor(0); // knock the text out of the filled block
  // 9x18B rather than 10x20: "Switch is OFF" needs 130 px at 10x20 against a
  // 126 px layout width. 9x18B fits it in 117 and stays bold.
  oled.setFont(u8g2_font_9x18B_tf);
  strCenter(20, "Switch is OFF");
  oled.setDrawColor(1);
  oled.setFont(u8g2_font_6x12_tf);
  strCenter(44, "Slide switch to ON");
  strCenter(58, "to run or charge");
}

// --- Frame assembly ---------------------------------------------------------
// Renders the whole screen into the RAM buffer. No I2C happens here, so this is
// cheap and can run in one go; only the push is metered.
static void renderFrame() {
  const BatteryStatus bat = batteryGetStatus();
  oled.clearBuffer();

  switch (stateCurrent()) {
  case STATE_BATTERY_WAIT:
    drawBatteryWaitScreen();
    break;

  case STATE_CHARGE_ONLY:
    // No BLE icon: bleStop() has run, so the radio is genuinely down.
    drawStatusBar(bat.full ? "Full" : "Charging", false, bat);
    drawChargeOnlyBody(bat);
    break;

  case STATE_LIGHT_SLEEP:
    // Still advertising and connectable - that is the whole point of the state.
    drawStatusBar("Advertising", true, bat);
    drawLightSleepBody();
    break;

  case STATE_RUNNING:
  default:
    drawStatusBar(bleIsConnected() ? "Connected" : "Advertising", true, bat);
    drawRunningBody();
    break;
  }
}

// --- Public API -------------------------------------------------------------

void displayBegin() {
  Wire.begin();
  // u8g2 wants the 8-bit left-shifted address here. Passing the 7-bit value is
  // the classic silent failure: begin() returns and nothing ever appears.
  oled.setI2CAddress(DISPLAY_I2C_ADDRESS << 1);

  // Probe before committing. A missing or mis-wired panel disables this module
  // rather than halting - the device's actual job is streaming telemetry, and
  // it can do that perfectly well with a dead screen.
  Wire.beginTransmission(DISPLAY_I2C_ADDRESS);
  if (Wire.endTransmission() != 0) {
    LOG_PRINTF("⚠️ No OLED at 0x%02X - display disabled\n", DISPLAY_I2C_ADDRESS);
    present = false;
    return;
  }

  oled.begin();
  oled.setContrast(DISPLAY_CONTRAST);
  present = true;
  asleep = false;
  lastShiftMs = lastRenderMs = millis();

  // First frame goes out in one blocking write. Startup is the one moment the
  // ~31 ms cost is free: the GNSS UART is not streaming yet.
  renderFrame();
  oled.sendBuffer();
  LOG_PRINTLN("✅ OLED display enabled.");
}

bool displayIsPresent() { return present; }

void displayUpdate() {
  if (!present || asleep) {
    return;
  }

  const unsigned long now = millis();

  // Finish any frame already in flight before starting another.
  //
  // Slices are SPACED, not pushed back to back. Total I2C time was never the
  // constraint - a whole frame is ~31 ms spread across a second. The constraint
  // was density: consecutive slices left the GNSS UART without a clear stretch
  // in which to be drained, and NAV-PVT bytes were lost. The gap leaves roughly
  // four RX-buffer fill windows between hits. See DISPLAY_SLICE_INTERVAL_MS.
  if (pushCursor >= 0) {
    if ((now - lastSliceMs) < DISPLAY_SLICE_INTERVAL_MS) {
      return; // not yet - let the UART breathe
    }
    lastSliceMs = now;
    const uint8_t tx = (pushCursor % SLICES_PER_ROW) * DISPLAY_CHUNK_TILES_W;
    const uint8_t ty = pushCursor / SLICES_PER_ROW;
    oled.updateDisplayArea(tx, ty, DISPLAY_CHUNK_TILES_W, 1);
    if (++pushCursor >= SLICE_COUNT) {
      pushCursor = -1;
    }
    return;
  }

  if ((now - lastRenderMs) < DISPLAY_REFRESH_INTERVAL_MS) {
    return;
  }
  lastRenderMs = now;

  // Advance the burn-in offset on its own much slower schedule. It only ever
  // changes between frames, so a shift can never tear a frame in flight.
  if ((now - lastShiftMs) >= DISPLAY_SHIFT_INTERVAL_MS) {
    lastShiftMs = now;
    shiftIdx = (shiftIdx + 1) % 8;
  }

  renderFrame();
  pushCursor = 0;    // hand the frame to the metered push above
  lastSliceMs = now; // first slice waits one full interval, like the rest
}

void displaySleep() {
  if (!present || asleep) {
    return;
  }
  pushCursor = -1;      // abandon any frame mid-push; it is about to be blanked
  oled.setPowerSave(1); // SSD1306 DISPLAYOFF
  asleep = true;
}

void displayWake() {
  if (!present || !asleep) {
    return;
  }
  oled.setPowerSave(0);
  asleep = false;
  lastRenderMs = 0; // force a render on the next poll rather than waiting out
                    // the refresh interval
}

#else // !DISPLAY_ENABLED

#include "g_log.h"

// Stubs matching g_display.h exactly. displayIsPresent() returns false, so
// LED_ENABLED's fallback (g_led lights up when no display is present) engages
// automatically - a side effect of reusing that mechanism, not the point of
// this flag.
void displayBegin() {
  LOG_PRINTLN("⏸️ Display disabled at compile time (DISPLAY_ENABLED=0).");
}
void displayUpdate() {}
bool displayIsPresent() { return false; }
void displaySleep() {}
void displayWake() {}

#endif // DISPLAY_ENABLED
