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

#include "g_display.h"
#include "config.h"

#if DISPLAY_ENABLED

#include "g_battery.h"
#include "g_ble.h"
#include "g_gnss.h"
#include "g_imu.h"
#include "g_imu_trim.h"
#include "g_log.h"
#include "g_state.h"
#include "g_telemetry.h"
#include <U8g2lib.h>
#include <Wire.h>

// Full-buffer hardware-I2C driver.
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);

static bool present = false; // panel answered; false disables the module
static bool asleep = false;  // DISPLAYOFF sent

// Set by the first displayUpdate(). The boot frame is drawn before gnssBegin(),
// so it must not show "No GNSS".
static bool setupDone = false;
static unsigned long lastRenderMs = 0;
static unsigned long lastShiftMs = 0;
static unsigned long lastSliceMs = 0;

// Next slice to push, or -1 when idle.
static const int SLICES_PER_ROW = DISPLAY_TILES_W / DISPLAY_CHUNK_TILES_W;
static const int SLICE_COUNT = SLICES_PER_ROW * DISPLAY_TILES_H;
static int pushCursor = -1;

// Epoch phase lock: iTOW of the last epoch seen and when. iTOW never reaches
// ITOW_NONE.
static const uint32_t ITOW_NONE = 0xFFFFFFFFUL;
static uint32_t lastSeenITOW = ITOW_NONE;
static unsigned long lastEpochSeenMs = 0;

// Burn-in shift: walks the 8 perimeter positions of a 3x3 grid.
static uint8_t shiftIdx = 0;
static const int8_t SHIFT_X[8] = {0, 1, 2, 2, 2, 1, 0, 0};
static const int8_t SHIFT_Y[8] = {0, 0, 0, 1, 2, 2, 2, 1};
static inline int ox(int x) { return x + SHIFT_X[shiftIdx]; }
static inline int oy(int y) { return y + SHIFT_Y[shiftIdx]; }

// USB plug icon, 7x8:
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

static const uint16_t ICON_BLUETOOTH = 74; // open_iconic_embedded

// open_iconic_check glyphs.
static const uint16_t ICON_TRIM_OK = 64;  // check
static const uint16_t ICON_TRIM_BAD = 68; // X

// Trim icon x, between the longest label (ends at 66) and the USB icon (88).
static const int TRIM_X = 78;

// Draw helpers

static void strAt(int x, int y, const char *s) {
  oled.drawStr(ox(x), oy(y), s);
}

// Measured with getStrWidth(), since not every font is fixed-pitch. Centered
// within the layout width.
static void strRight(int rightX, int y, const char *s) {
  oled.drawStr(ox(rightX - (int)oled.getStrWidth(s)), oy(y), s);
}
static void strCenter(int y, const char *s) {
  oled.drawStr(ox((DISPLAY_LAYOUT_W - (int)oled.getStrWidth(s)) / 2), oy(y), s);
}

// Battery outline filled to pct.
static void batteryBar(int x, int y, int w, int h, uint8_t pct) {
  oled.drawFrame(ox(x), oy(y), w, h);
  oled.drawBox(ox(x + w), oy(y + h / 4), 2, h / 2); // terminal nub
  const int inner = w - 4;
  const int fill = (int)((inner * (uint16_t)pct) / 100);
  if (fill > 0) {
    oled.drawBox(ox(x + 2), oy(y + 2), fill, h - 4);
  }
}

// Status bar: BLE icon and label on the left, trim icon, then USB icon and
// battery gauge on the right. showTrim is true only in RUNNING with the IMU up.
static void drawStatusBar(const char *label, bool bleUp, bool showTrim,
                          const BatteryStatus &bat) {
  if (bleUp) {
    oled.setFont(u8g2_font_open_iconic_embedded_1x_t);
    oled.drawGlyph(ox(0), oy(8), ICON_BLUETOOTH);
  }

  oled.setFont(u8g2_font_5x7_tf);
  strAt(11, 8, label);

  // Check when locked, X when tilt exceeds the limit, blank while deciding.
  if (showTrim) {
    oled.setFont(u8g2_font_open_iconic_check_1x_t);
    if (imuTrimConverged()) {
      oled.drawGlyph(ox(TRIM_X), oy(8), ICON_TRIM_OK);
    } else if (imuTrimTiltDegrees() > IMU_TRIM_MAX_TILT_DEG) {
      oled.drawGlyph(ox(TRIM_X), oy(8), ICON_TRIM_BAD);
    }
  }

  // The switch is always on when a bar is drawn, so this means USB present.
  if (bat.charging) {
    oled.drawXBM(ox(88), oy(1), USB_W, USB_H, USB_XBM);
  }

  batteryBar(98, 1, 26, 9, bat.percent);

  oled.drawHLine(ox(0), oy(12), DISPLAY_LAYOUT_W);
}

// Bodies

// Short fix label from the raw fixType.
static const char *fixLabel(uint8_t fixType) {
  switch (fixType) {
  case 2:
    return "2D";
  case 3:
    return "3D";
  case 4:
    return "3D"; // GNSS + dead reckoning
  default:
    return "No Fix";
  }
}

static void drawUptime() {
  char buf[16];
  const unsigned long secs = millis() / 1000UL;
  snprintf(buf, sizeof(buf), "%lu:%02lu:%02lu", secs / 3600UL,
           (secs / 60UL) % 60UL, secs % 60UL);
  strRight(126, 60, buf);
}

static void drawRunningBody() {
  // GNSS absent or stalled: say so instead of showing zeros or frozen values.
  // Uptime keeps ticking to show the loop is alive.
  if (setupDone && (!gnssIsUp() || gnssStalled())) {
    const bool stalled = gnssIsUp();
    oled.setFont(u8g2_font_10x20_tf);
    strCenter(32, stalled ? "GNSS stalled" : "No GNSS");
    oled.setFont(u8g2_font_6x12_tf);
    strCenter(46, stalled ? "Receiver not sending" : "Power-cycle to retry");
    drawUptime();
    return;
  }

  const UBX_NAV_PVT_data_t *pvt = gnssLatestPvt();
  const uint8_t fixType = pvt ? pvt->fixType : 0;

  // Same rule as the RaceBox latLonFlags.
  const bool posValid = pvt && fixType >= 2;

  char buf[24];
  oled.setFont(u8g2_font_10x20_tf);
  snprintf(buf, sizeof(buf), "%u SV", pvt ? pvt->numSV : 0);
  strAt(2, 32, buf);
  strRight(126, 32, fixLabel(fixType));

  oled.setFont(u8g2_font_6x12_tf);
  // hAcc and pDOP are meaningless without a fix.
  if (posValid) {
    snprintf(buf, sizeof(buf), "pDOP %.2f", pvt->pDOP / 100.0f);
  } else {
    snprintf(buf, sizeof(buf), "pDOP --");
  }
  strAt(2, 46, buf);

  // PVT rate, valid with or without a fix.
  snprintf(buf, sizeof(buf), "%.0fHz", telemetryGnssRateHz());
  strRight(126, 46, buf);

  if (posValid) {
    snprintf(buf, sizeof(buf), "hAcc %lumm", (unsigned long)pvt->hAcc);
  } else {
    snprintf(buf, sizeof(buf), "hAcc --");
  }
  strAt(2, 60, buf);

  drawUptime();
}

// Cell voltage, large and centered.
static void drawChargeOnlyBody(const BatteryStatus &bat) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%.2f", bat.voltage);

  // The _tn font has no letters, so the unit uses a text font.
  oled.setFont(u8g2_font_logisoso24_tn);
  const int wNum = (int)oled.getStrWidth(buf);
  oled.setFont(u8g2_font_7x14B_tf);
  const int wUnit = (int)oled.getStrWidth("V");

  const int gap = 4;
  const int x = (DISPLAY_LAYOUT_W - (wNum + gap + wUnit)) / 2;
  const int baseline = 48; // centers within body rows 14-61

  oled.setFont(u8g2_font_logisoso24_tn);
  strAt(x, baseline, buf);
  oled.setFont(u8g2_font_7x14B_tf);
  strAt(x + wNum + gap, baseline, "V");
}

// Full-screen switch-off alert, no status bar.
static void drawBatteryWaitScreen() {
  oled.drawBox(ox(0), oy(0), DISPLAY_LAYOUT_W, 26);
  oled.setDrawColor(0); // inverted text
  // 9x18B fits "Switch is OFF" in the layout width; 10x20 doesn't.
  oled.setFont(u8g2_font_9x18B_tf);
  strCenter(20, "Switch is OFF");
  oled.setDrawColor(1);
  oled.setFont(u8g2_font_6x12_tf);
  strCenter(44, "Slide switch to ON");
  strCenter(58, "to run or charge");
}

// Frame assembly

// Render the full screen into RAM. No I2C.
static void renderFrame() {
  const BatteryStatus bat = batteryGetStatus();
  oled.clearBuffer();

  switch (stateCurrent()) {
  case STATE_BATTERY_WAIT:
    drawBatteryWaitScreen();
    break;

  case STATE_CHARGE_ONLY:
    // No BLE icon; BLE is stopped.
    drawStatusBar(bat.full ? "Full" : "Charging", false, false, bat);
    drawChargeOnlyBody(bat);
    break;

  case STATE_RUNNING:
  default:
    drawStatusBar(bleIsConnected() ? "Connected" : "Advertising", true,
                  imuIsUp(), bat);
    drawRunningBody();
    break;
  }
}

// Public API

void displayBegin() {
  Wire.begin();
  // u8g2 takes the 8-bit (shifted) address.
  oled.setI2CAddress(DISPLAY_I2C_ADDRESS << 1);

  // Probe first; a missing panel disables the module.
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

  // The first frame is sent in one blocking write, before GNSS is streaming.
  renderFrame();
  oled.sendBuffer();
  LOG_PRINTLN("✅ OLED display enabled.");
}

bool displayIsPresent() { return present; }

// Push the next slice; clear the cursor after the last.
static void pushSlice() {
  const uint8_t tx = (pushCursor % SLICES_PER_ROW) * DISPLAY_CHUNK_TILES_W;
  const uint8_t ty = pushCursor / SLICES_PER_ROW;
  oled.updateDisplayArea(tx, ty, DISPLAY_CHUNK_TILES_W, 1);
  if (++pushCursor >= SLICE_COUNT) {
    pushCursor = -1;
  }
}

void displayUpdate() {
  setupDone = true;
  if (!present || asleep) {
    return;
  }

  const unsigned long now = millis();

  // A changed iTOW means gnssPoll() just parsed an epoch, so the UART is idle
  // until the next one. Uses the non-consuming reader.
  bool epochJustLanded = false;
  const UBX_NAV_PVT_data_t *pvt = gnssLatestPvt();
  if (pvt != nullptr && pvt->iTOW != lastSeenITOW) {
    lastSeenITOW = pvt->iTOW;
    lastEpochSeenMs = now;
    epochJustLanded = true;
  }

  // Without epochs (GNSS off or silent), fall back to timed slices.
  const bool epochsFlowing = (lastSeenITOW != ITOW_NONE) &&
                             ((now - lastEpochSeenMs) < DISPLAY_EPOCH_STALE_MS);

  // Finish the frame in flight first.
  if (pushCursor >= 0) {
    if (epochsFlowing) {
      if (!epochJustLanded) {
        return; // wait for the next epoch
      }
      for (int i = 0; i < DISPLAY_SLICES_PER_EPOCH && pushCursor >= 0; i++) {
        pushSlice();
      }
      lastSliceMs = now;
    } else {
      if ((now - lastSliceMs) < DISPLAY_SLICE_INTERVAL_MS) {
        return;
      }
      lastSliceMs = now;
      pushSlice();
    }
    return;
  }

  if ((now - lastRenderMs) < DISPLAY_REFRESH_INTERVAL_MS) {
    return;
  }

  // Rendering also blocks the loop, so it waits for an epoch too.
  if (epochsFlowing && !epochJustLanded) {
    return;
  }
  lastRenderMs = now;

  // Shift only between frames.
  if ((now - lastShiftMs) >= DISPLAY_SHIFT_INTERVAL_MS) {
    lastShiftMs = now;
    shiftIdx = (shiftIdx + 1) % 8;
  }

  renderFrame();
  pushCursor = 0;
  lastSliceMs = now;
}

void displaySleep() {
  if (!present || asleep) {
    return;
  }
  pushCursor = -1;      // abandon any frame in flight
  oled.setPowerSave(1); // SSD1306 DISPLAYOFF
  asleep = true;
}

#else // !DISPLAY_ENABLED

#include "g_log.h"

// Stubs. displayIsPresent() is false, so g_led falls back to the LED.
void displayBegin() {
  LOG_PRINTLN("⏸️ Display disabled at compile time (DISPLAY_ENABLED=0).");
}
void displayUpdate() {}
bool displayIsPresent() { return false; }
void displaySleep() {}

#endif // DISPLAY_ENABLED
