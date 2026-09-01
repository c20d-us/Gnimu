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
// DIAGNOSTIC: battery charge/rest voltage logger
//
// Captures the LiPo's TRUE resting voltage at full charge - something you
// can't read directly from BAT+/- while USB is connected, since the charger's
// own CC/CV output dominates that node. This sketch logs VBAT continuously
// through a plug-in -> full-charge -> unplug -> settle cycle, to internal
// flash (not Serial), since the whole point is to keep logging after USB
// (and Serial) is physically disconnected. The device stays powered the
// whole time off the LiPo directly (same as production - USB is only power
// AND data, the slide switch/battery keeps the MCU alive regardless), so one
// continuous run covers the entire cycle without a reset.
//
// Auto-detects two events, both via a simple "has it stopped moving"
// heuristic (no current sensing available - VBUS presence + voltage only):
//   - CHARGE_PLATEAU: while charging, VBAT has stayed flat for a while -
//     approximates end of the CV taper (not exact - the charger gives no
//     complete pin - but a reasonable proxy).
//   - SETTLED: after unplug, VBAT has stopped decaying - the point where the
//     "surface charge" bump has fully bled off and the reading reflects the
//     cell's true resting voltage. THIS is the number you want for
//     `BATTERY_DISCHARGE_CURVE`'s 100% anchor.
//
// Mirrors g_battery.cpp's actual sampling method (SAADC config, divider
// enable/disable, peak-of-N per reading) so results are directly comparable
// to what the real firmware would report - not a different measurement.
//
// Usage: flash, then let it run through a full plug-in/charge/unplug/settle
// cycle (hours). Reconnect USB + open Serial at any point (mid-test, after
// the whole cycle, whenever) and type 'd' to dump the log over Serial or 'e'
// to erase it and start fresh - both work at any time, not just at boot.
// Logging continues in the background regardless; existing data is appended
// to, never overwritten silently. The onboard LED glows GREEN whenever USB
// is detected present, live and independent of the log - a quick visual
// check (no Serial needed) that unplugging the cable actually registers.
//
// Requires: XIAO nRF52840 Sense; a LiPo; USB for charging + retrieval.
// ============================================================================

#include <Arduino.h>
#include <Adafruit_LittleFS.h>
#include <InternalFileSystem.h>

// Serial (USB CDC) needs the TinyUSB library linked.
#include <Adafruit_TinyUSB.h>

using namespace Adafruit_LittleFS_Namespace;

// --- GNSS rail hold-off (mirrors g_power.cpp's powerHoldPeripheralsOff()) ---
// The TPS63020's EN pad has its own onboard pullup, so the GNSS rail defaults
// to ENABLED when this pin is left floating - without this, the ~30 mA GNSS
// load would run the whole test and contaminate every VBAT reading. D6 must
// also be idled low, or the GNSS phantom-powers itself through its RX ESD
// diode off D6's floating/high idle level even with EN cut (same gotcha as
// production - see g_power.cpp's powerHoldPeripheralsOff()). The IMU needs no
// equivalent handling here: unlike GNSS_EN, its power pin defaults OFF
// (nothing drives it HIGH), so it simply never turns on in this sketch.
static const uint8_t GNSS_EN_PIN = D9;
static const uint8_t GNSS_TX_PIN = D6;

static void gnssRailOff() {
  pinMode(GNSS_EN_PIN, OUTPUT);
  digitalWrite(GNSS_EN_PIN, LOW); // active-high enable, so LOW disables
  pinMode(GNSS_TX_PIN, OUTPUT);
  digitalWrite(GNSS_TX_PIN, LOW);
}

// --- Live USB-present indicator (green LED, active-low like the rest of the
// XIAO's RGB LED) - a quick, no-flash-writes sanity check for whether
// usbPresent() actually flips the instant the cable is pulled. Independent
// of the CSV log entirely; updated every loop() iteration so there's no
// delay in seeing it change.
static const uint8_t USB_INDICATOR_LED_PIN = LED_GREEN;

// --- VBAT divider + SAADC config (mirrors config.h / g_battery.cpp) ---
#ifdef PIN_VBAT
#define BATTERY_ADC_PIN PIN_VBAT
#else
#define BATTERY_ADC_PIN 32 // Arduino D32 -> P0.31 / AIN7 (VBAT sense)
#endif
#ifdef VBAT_ENABLE
#define BATTERY_ADC_ENABLE_PIN VBAT_ENABLE
#else
#define BATTERY_ADC_ENABLE_PIN 14 // Arduino D14 -> P0.14 (LOW = divider on)
#endif

static const uint8_t SAADC_RESOLUTION_BITS = 12;
static const float SAADC_REFERENCE_MV = 3000.0f;
static const uint8_t SAADC_TACQ_US = 40; // long TACQ for the high-Z divider
static const float BATTERY_DIVIDER_R_TOP_OHM = 1000000.0f;
static const float BATTERY_DIVIDER_R_BOTTOM_OHM = 510000.0f;
static const float BATTERY_DIVIDER_RATIO =
    (BATTERY_DIVIDER_R_TOP_OHM + BATTERY_DIVIDER_R_BOTTOM_OHM) /
    BATTERY_DIVIDER_R_BOTTOM_OHM;
// This board's ADC reading is off from a multimeter on BAT+/- by a DIFFERENT
// amount depending on whether USB/VBUS is present - confirmed on hardware:
// readings jump ~250-270mV the instant USB is unplugged and drop right back
// the instant it's replugged, with the underlying real battery voltage
// barely moving in between. Most likely the nRF52840's internal ADC
// reference behaves slightly differently depending on which rail is
// powering the chip - a real per-power-state effect, not something a single
// calibration constant can correct for.
//   USB present:  two readings 4.05V/4.10V and 4.06V/4.10V ->
//                 factor = average(4.10/4.05, 4.10/4.06) = 1.0111
//   USB absent:   logged avg 4335.8mV (already at the USB factor) vs a
//                 steady 4090mV on the meter -> raw = 4335.8/1.0111 =
//                 4288.7, factor = 4090/4288.7 = 0.9537
// Both are per-board - re-derive if this specific XIAO is ever swapped.
static const float BATTERY_CALIBRATION_FACTOR_USB = 1.0111f;
static const float BATTERY_CALIBRATION_FACTOR_BATTERY = 0.9537f;
static const uint8_t BATTERY_SAMPLE_COUNT = 20; // paced reads per logged point
static const unsigned long BATTERY_SAMPLE_SPACING_US = 2500; // ~50ms window

// --- Logging cadence + storage ---
// The internal-flash filesystem region is a fixed 28 KiB on this chip
// (LFS_FLASH_ADDR/LFS_FLASH_TOTAL_SIZE in the core's InternalFileSystem.cpp -
// 7 flash pages), usable space a bit less after LittleFS's own metadata
// overhead - call it ~24-26 KB.
//
// 1 minute everywhere (charging and settling both) - the earlier "unexplained
// multi-hour climb" that motivated splitting cadence by phase turned out to
// be the USB-vs-battery ADC calibration difference (see
// BATTERY_CALIBRATION_FACTOR_USB/_BATTERY above), not a cadence artifact, so
// there's no need to keep charging-phase sampling coarse anymore. 1 minute
// gives ~29h of headroom before the ~24-26 KB budget fills (vs. ~14-15h at
// 30s), comfortably covering an overnight-plus-next-day run, while still
// giving both detectors plenty of samples per window (~30 for the 30-min
// CHARGE_PLATEAU window, ~15 for the 15-min settle grace+window - both well
// above MIN_SAMPLES_FOR_FLAT_CHECK). SETTLE_FINE_DURATION_MS is set past any
// realistic single test so it won't drop to COARSE mid-run; COARSE remains
// as a safety net if the device is ever left running much longer than
// planned.
static const unsigned long CHARGE_SAMPLE_INTERVAL_MS = 60000UL;         // 1m
static const unsigned long SETTLE_FINE_INTERVAL_MS = 60000UL;           // 1m
static const unsigned long SETTLE_FINE_DURATION_MS = 24UL * 60 * 60 * 1000; // 24h
static const unsigned long COARSE_SAMPLE_INTERVAL_MS = 5UL * 60 * 1000; // 5m
static const char *LOG_FILE_PATH = "/battery_log.csv";

// --- Event-detection tunables (placeholder starting points - watch a real
// run and adjust if these fire too early/late/never) ---
// 30m, not 15m: at the 5-minute charging cadence above, a 15m window would
// only span ~3 samples - widen it so the "is it flat" check still has
// enough points to be meaningful.
static const unsigned long CHARGE_PLATEAU_WINDOW_MS = 30UL * 60 * 1000;
static const uint16_t CHARGE_PLATEAU_THRESHOLD_MV = 5;
static const unsigned long SETTLE_GRACE_MS = 5UL * 60 * 1000;  // 5m
static const unsigned long SETTLE_WINDOW_MS = 10UL * 60 * 1000; // 10m
static const uint16_t SETTLE_THRESHOLD_MV = 3;

// Both flatness checks require at least this many samples in their window,
// not just the time span - a time-only window silently degrades to "is it
// flat across 2-3 points" if the cadence ever changes, which is exactly how
// the coarse (5-min) cadence's SETTLE_WINDOW_MS produced a false SETTLED
// flag on an overnight run: 3 samples happened to be close together by
// chance in the middle of a real, ongoing multi-hour climb.
static const uint8_t MIN_SAMPLES_FOR_FLAT_CHECK = 5;

// --- Rolling history of recent readings, for the plateau/settle checks ---
// 80 entries = ~80 min at the 1-min cadence above, comfortably covering the
// 30-min plateau window and the 15-min settle grace+window with margin.
static const uint8_t HISTORY_CAPACITY = 80;
static uint16_t historyMv[HISTORY_CAPACITY];
static unsigned long historyMs[HISTORY_CAPACITY];
static uint8_t historyHead = 0;
static uint8_t historyCount = 0;

static void historyPush(unsigned long nowMs, uint16_t mv) {
  historyMs[historyHead] = nowMs;
  historyMv[historyHead] = mv;
  historyHead = (historyHead + 1) % HISTORY_CAPACITY;
  if (historyCount < HISTORY_CAPACITY) {
    historyCount++;
  }
}

// Min/max among stored readings with timestamp >= sinceMs, plus how many
// samples that covers. Returns false if no samples fall in that window yet.
static bool historyMinMaxSince(unsigned long sinceMs, uint16_t *minMv,
                                uint16_t *maxMv, uint8_t *count) {
  uint16_t mn = 0xFFFF, mx = 0;
  uint8_t n = 0;
  for (uint8_t i = 0; i < historyCount; i++) {
    if (historyMs[i] >= sinceMs) {
      if (historyMv[i] < mn) {
        mn = historyMv[i];
      }
      if (historyMv[i] > mx) {
        mx = historyMv[i];
      }
      n++;
    }
  }
  if (n > 0) {
    *minMv = mn;
    *maxMv = mx;
    *count = n;
  }
  return n > 0;
}

// --- VBAT sampler: enable divider, take BATTERY_SAMPLE_COUNT paced reads,
// keep the peak (matches g_battery.cpp's peak-for-SoC rationale), disable
// divider. Blocking (~60ms) - negligible next to any of this tool's sample
// intervals. ---
static uint16_t readVbatMv(bool usb) {
  digitalWrite(BATTERY_ADC_ENABLE_PIN, LOW); // active-low: connect divider

  // g_battery.cpp never needs an explicit settle delay here because its
  // poll cadence (250ms) never leaves the divider disabled for more than a
  // fraction of a second. This tool disables it for up to 5 MINUTES between
  // reads during the charging phase - after that long floating, the divider
  // node needs real time to settle to the true voltage, not just the SAADC's
  // own short acquisition time. 500us was nowhere near enough: charging-phase
  // readings came in 300-900mV too high (a single LiPo cell physically can't
  // exceed ~4.24V, and readings hit 4560mV), while the 30s-cadence
  // post-unplug readings (divider only ever off briefly) were fine all
  // along. Fix: a real settle delay, plus a throwaway dummy read discarded
  // before the readings that count toward the peak.
  delay(10);
  analogRead(BATTERY_ADC_PIN); // dummy - discarded

  int peakAdc = 0;
  for (uint8_t i = 0; i < BATTERY_SAMPLE_COUNT; i++) {
    const int raw = analogRead(BATTERY_ADC_PIN);
    if (raw > peakAdc) {
      peakAdc = raw;
    }
    delayMicroseconds(BATTERY_SAMPLE_SPACING_US);
  }
  digitalWrite(BATTERY_ADC_ENABLE_PIN, HIGH); // disconnect divider

  const float adcMax = (float)((1UL << SAADC_RESOLUTION_BITS) - 1);
  const float adcVolts = ((float)peakAdc / adcMax) * (SAADC_REFERENCE_MV / 1000.0f);
  const float calibration =
      usb ? BATTERY_CALIBRATION_FACTOR_USB : BATTERY_CALIBRATION_FACTOR_BATTERY;
  return (uint16_t)lroundf(adcVolts * BATTERY_DIVIDER_RATIO * calibration *
                            1000.0f);
}

static bool usbPresent() {
  return (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
}

static void updateUsbIndicatorLed() {
  digitalWrite(USB_INDICATOR_LED_PIN, usbPresent() ? LOW : HIGH); // active-low
}

// --- Log file helpers ---
static void logLine(unsigned long nowMs, uint16_t mv, bool usb,
                    const char *event) {
  char line[80];
  snprintf(line, sizeof(line), "%lu,%u,%d,%s\n", nowMs / 1000, mv, usb ? 1 : 0,
           event);

  File f = InternalFS.open(LOG_FILE_PATH, FILE_O_WRITE);
  if (f) {
    f.write((const uint8_t *)line, strlen(line));
    f.close();
  } else {
    Serial.println("❌ Failed to open log file for append.");
  }

  if (Serial) {
    Serial.print(line);
  }
}

static void dumpLog() {
  File f = InternalFS.open(LOG_FILE_PATH, FILE_O_READ);
  if (!f) {
    Serial.println("(no log file yet)");
    return;
  }
  Serial.println("elapsed_s,vbat_mV,usb_present,event");
  uint8_t buf[64];
  int n;
  while ((n = f.read(buf, sizeof(buf))) > 0) {
    Serial.write(buf, n);
  }
  f.close();
}

static void eraseLog() {
  InternalFS.remove(LOG_FILE_PATH);
  Serial.println("Log erased - starting fresh.");
}

// --- Event-detection state ---
static bool lastUsbPresent = false;
static bool haveLastUsb = false;
static unsigned long usbSinceMs = 0;   // millis() of the last USB transition
static bool chargePlateauFlagged = false;
static bool settleFlagged = false;

void setup() {
  // Unconditionally hold the GNSS rail off first, before anything else -
  // same ordering as production's setup() prologue, so no window exists
  // where the rail could come up (e.g. while waiting on Serial below).
  gnssRailOff();

  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
  }

  if (!InternalFS.begin()) {
    // First-time use on this board (or a corrupt filesystem) - format once.
    // Safe here (unlike g_storage's firmware policy): this is a dedicated
    // bench tool, not production code that must never silently destroy data.
    if (Serial) {
      Serial.println("InternalFS mount failed - formatting...");
    }
    InternalFS.format();
    InternalFS.begin();
  }

  if (Serial) {
    Serial.println("\n=== battery charge/rest voltage logger ===");
    Serial.println("Type d (dump log) or e (erase log) anytime - not just "
                    "at boot.");
  }

  pinMode(BATTERY_ADC_ENABLE_PIN, OUTPUT);
  digitalWrite(BATTERY_ADC_ENABLE_PIN, HIGH); // divider off until sampling
  analogReadResolution(SAADC_RESOLUTION_BITS);
  analogReference(AR_INTERNAL_3_0); // pairs with SAADC_REFERENCE_MV
  analogSampleTime(SAADC_TACQ_US);

  pinMode(USB_INDICATOR_LED_PIN, OUTPUT);
  updateUsbIndicatorLed();

  if (Serial) {
    Serial.println("Logging (cadence varies by phase - see comments). Plug "
                    "in, let it charge fully, then unplug and let it sit - "
                    "CHARGE_PLATEAU and SETTLED events log automatically.");
  }
}

// Sample cadence for the CURRENT phase, based on the last known USB state -
// a phase change is picked up at the next sample point regardless (worst
// case a few minutes late while in a coarse phase), which is fine here since
// nothing about the cadence itself is time-critical to the second.
static unsigned long currentIntervalMs(unsigned long nowMs) {
  if (!haveLastUsb || lastUsbPresent) {
    return CHARGE_SAMPLE_INTERVAL_MS;
  }
  const unsigned long sinceUnplugMs = nowMs - usbSinceMs;
  return (sinceUnplugMs < SETTLE_FINE_DURATION_MS) ? SETTLE_FINE_INTERVAL_MS
                                                    : COARSE_SAMPLE_INTERVAL_MS;
}

// Live command handling - checked every loop() iteration (not just at boot),
// so reconnecting Serial at any point (mid-test, after the whole cycle,
// whenever) still works. The original design only listened for a few
// seconds right at boot, which was easy to miss entirely if the Serial
// Monitor took a moment to enumerate/reconnect after a reset - typed input
// then just sat unread with no feedback at all.
static void handleSerialCommands() {
  if (!Serial || !Serial.available()) {
    return;
  }
  const char c = Serial.read();
  if (c == 'd' || c == 'D') {
    dumpLog();
  } else if (c == 'e' || c == 'E') {
    eraseLog();
  }
}

void loop() {
  handleSerialCommands();
  updateUsbIndicatorLed(); // every iteration - no delay in seeing it change

  static unsigned long lastSampleMs = 0;
  const unsigned long nowMs = millis();
  if (nowMs - lastSampleMs < currentIntervalMs(nowMs)) {
    return;
  }
  lastSampleMs = nowMs;

  const bool usb = usbPresent();
  const uint16_t mv = readVbatMv(usb);
  historyPush(nowMs, mv);

  // USB transition - mark it, reset the relevant detector, and start timing
  // the new phase from right now.
  if (!haveLastUsb || usb != lastUsbPresent) {
    haveLastUsb = true;
    lastUsbPresent = usb;
    usbSinceMs = nowMs;
    chargePlateauFlagged = false;
    settleFlagged = false;
    logLine(nowMs, mv, usb, usb ? "USB_CONNECTED" : "USB_DISCONNECTED");
    return; // don't also run a detector check on the transition sample itself
  }

  logLine(nowMs, mv, usb, "");

  if (usb && !chargePlateauFlagged &&
      (nowMs - usbSinceMs) >= CHARGE_PLATEAU_WINDOW_MS) {
    uint16_t mn, mx;
    uint8_t n;
    if (historyMinMaxSince(nowMs - CHARGE_PLATEAU_WINDOW_MS, &mn, &mx, &n) &&
        n >= MIN_SAMPLES_FOR_FLAT_CHECK && (mx - mn) <= CHARGE_PLATEAU_THRESHOLD_MV) {
      chargePlateauFlagged = true;
      logLine(nowMs, mv, usb, "CHARGE_PLATEAU");
    }
  }

  if (!usb && !settleFlagged &&
      (nowMs - usbSinceMs) >= (SETTLE_GRACE_MS + SETTLE_WINDOW_MS)) {
    uint16_t mn, mx;
    uint8_t n;
    if (historyMinMaxSince(nowMs - SETTLE_WINDOW_MS, &mn, &mx, &n) &&
        n >= MIN_SAMPLES_FOR_FLAT_CHECK && (mx - mn) <= SETTLE_THRESHOLD_MV) {
      settleFlagged = true;
      logLine(nowMs, mv, usb, "SETTLED");
    }
  }
}
