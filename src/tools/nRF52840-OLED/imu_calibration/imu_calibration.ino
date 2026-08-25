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
// DIAGNOSTIC: unattended bench IMU calibration
//
// Measures per-axis zero-point offsets for the onboard LSM6DS3TR-C in the
// sensor's RAW axis frame (before any axis remap), logs every session to
// internal flash, and aggregates them into six paste-ready #define lines for
// config.h. Independent of the IMU_AXIS_* remap - the offsets stay valid
// across mounting changes.
//
// Units match what g_imu.cpp / config.h use:
//   * accel:  g       (LSM6DS3 native, x1000 to milli-g downstream)
//   * gyro:   deg/s   (LSM6DS3 native, x100 to centi-deg/s downstream)
// Accel Z is calibrated with the LED face pointing +Z UP on a level surface,
// so gravity (1.0 g) is subtracted from the average before recording.
//
// UNATTENDED BY DESIGN. Nothing waits on Serial, and every result goes to
// flash rather than the wire, so the intended workflow is: flash the sketch,
// slide the switch on, set the device down on a flat surface, walk away.
// Come back hours later, plug in USB, press a key, and aggregate.
//
// Flow:
//   1. Bring up the IMU, the GNSS rail, and BLE advertising (see "thermal
//      realism" below), then WARM UP until the die temperature stops moving
//      (WARMUP_MIN_MS floor, WARMUP_MAX_MS ceiling).
//   2. STABILITY GATE, re-run before every session: sample for
//      GATE_WINDOW_MS and require low MOTION (peak-to-peak). A failed gate
//      prints a full per-axis breakdown, waits GATE_RETRY_MS and tries
//      again, so a bumped bench costs one retry instead of a poisoned
//      session. Tilt is measured and recorded but does NOT block - see the
//      gate constants for why, and step 4 for what it does instead.
//   3. SESSION: average NUM_SAMPLES raw reads per axis at
//      IMU_SAMPLE_INTERVAL_MS pacing, append one CSV row to flash, wait
//      SESSION_GAP_MS, repeat forever.
//   4. Press any key over Serial at any point to halt to a menu:
//        d - dump the raw CSV log
//        a - aggregate the most recent run into paste-ready #define lines
//        e - erase the log
//        b - reboot (starts a fresh run without touching the device)
//        ? - reprint this menu
//      Halting is terminal for that run: to resume calibrating, reboot. `b`
//      exists so a reboot doesn't require physically power-cycling the
//      device, which would disturb the very stillness being measured.
//
// Runs accumulate. Each boot appends to the same file under a new run
// number, so a reboot never destroys earlier data; `a` aggregates the
// highest run number present, which is the current run once it has produced
// a record and the previous boot's run before that. Use `e` to start clean.
//
// THERMAL REALISM: the GNSS rail, BLE advertising and the OLED panel are
// deliberately left ON. IMU offsets drift with die temperature, and production
// runs with all three active, so calibrating on a cold board would
// characterize a thermal state the firmware never actually operates in. GNSS
// is by far the larger effect (~30 mA searching vs. ~1 mA advertising); BLE is
// included for completeness rather than for the heat. Nothing is configured
// beyond bring-up - GNSS bytes are read and discarded, BLE advertises with no
// services and needs no connection, and no sky view is required.
//
// The panel is continuously lit in this variant's production firmware and is
// therefore part of the load the die settles against. It doubles as a USB-free
// readout of phase, temperature and the latest offsets, which is what makes a
// genuinely unattended run retrievable without plugging anything in.
//
// LIMITATION - accel X/Y is a levelness measurement as much as a bias one.
// One degree of tilt leaks ~17 mg of gravity into the horizontal axes, which
// is several times larger than the offsets being measured. Averaging cannot
// remove it: a consistently tilted bench produces a consistent, wrong
// answer. The levelness gate below only catches gross errors (device on a
// book, cable propping up one corner), it does not certify accuracy.
// Gyro offsets are unaffected by this and are trustworthy at any pose.
// Truly separating accel X/Y bias from gravity leakage requires a
// multi-position (tumble) calibration, which is not hands-off and is out of
// scope for this tool. Rather than block on tilt, aggregation compares the
// run's median tilt against ACCEL_TRUST_TILT_DEG and comments the accel
// defines out when the pose was too far off level, while still emitting the
// gyro defines - a sloped bench costs you the numbers it compromised, not
// the whole run.
//
// This is the nRF52840-OLED tree's copy. The base tree has its own at
// tools/nRF52840/imu_calibration - same measurement approach, without the
// panel. Keeping them separate is what lets each one hardcode its own
// variant's settings instead of carrying a build flag.
//
// Requires: XIAO nRF52840 Sense with the SSD1306 panel; level bench surface.
// USB is never needed - results are readable on the panel, and the aggregated
// #define block is retrieved over Serial only when you want to paste it.
// ============================================================================

#include <Adafruit_LittleFS.h>
#include <Adafruit_TinyUSB.h> // Serial (USB CDC) needs TinyUSB linked
#include <Arduino.h>
#include <InternalFileSystem.h>
#include <LSM6DS3.h>
#include <bluefruit.h>

using namespace Adafruit_LittleFS_Namespace;

// ----------------------------------------------------------------------------
// Keep IMU settings in sync with config.h so calibration matches operating
// conditions (offsets drift slightly with range/ODR/bandwidth).
// ----------------------------------------------------------------------------
#define IMU_POWER_PIN PIN_LSM6DS3TR_C_POWER
#define IMU_I2C_ADDRESS 0x6A
#define ACCEL_RANGE_G 4
#define GYRO_RANGE_DPS 500
#define IMU_ACCEL_ODR_HZ 104
#define IMU_GYRO_ODR_HZ 104
#define IMU_ACCEL_BANDWIDTH_HZ 50
#define IMU_SAMPLE_INTERVAL_MS 10 // 100 Hz pacing between samples

// ----------------------------------------------------------------------------
// Thermal-load peripherals. Mirrors config.h's GNSS_EN_PIN / GNSS_BAUD and
// BLE_TX_POWER_ADV_DBM. The advertised name deliberately does NOT mirror
// config.h's "RaceBox Mini <id>" - a bench unit sitting here for hours
// should not look like a real device to any phone app that happens to scan.
// ----------------------------------------------------------------------------
#define GNSS_EN_PIN D9
#define GNSS_BAUD 57600
#define BLE_ADV_NAME "Gnimu cal"
// Mirrors this tree's config.h, which advertises quieter than the base tree's
// -16. Matching production is the whole point of bringing the radio up here.
#define BLE_TX_POWER_ADV_DBM -20

// ----------------------------------------------------------------------------
// OLED panel. Always on in this tree - the panel is lit continuously in this
// variant's production firmware, so it is part of the load the die settles
// against, and leaving it dark would calibrate a cooler board than the
// firmware ever runs on.
//
// The IMU library drives Wire1 internally, so the panel's Wire is a separate
// bus and the two never contend. A4/A5 are SDA/SCL on this board; the base
// tree, where A4 is instead an analog divider tap, has its own copy of this
// sketch with no display code at all rather than a flag to get this wrong.
//
// displayBringUp() still probes before committing, so a cracked or unplugged
// panel costs a warning and a Serial-only run rather than a hang.
// ----------------------------------------------------------------------------
#define DISPLAY_I2C_ADDRESS 0x3C
#define DISPLAY_CONTRAST 255
#define DISPLAY_REFRESH_MS 1000

#include <U8g2lib.h>
#include <Wire.h>
static U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
static bool displayPresent = false;

// ----------------------------------------------------------------------------
// Warmup: run until the die temperature plateaus, not for a fixed span. With
// GNSS and BLE up the board dissipates real power, so "how long to thermal
// steady state" is a property of the hardware rather than a number worth
// guessing. Same "has it stopped moving" shape battery_log uses for its
// SETTLED detector: a max-min check over a trailing window, guarded by a
// minimum sample count so it can't pass on two coincidentally-close reads.
// The floor keeps a fast-plateauing board from starting too early; the
// ceiling keeps a slow one (or a wandering ambient) from stalling forever.
// ----------------------------------------------------------------------------
static const unsigned long WARMUP_MIN_MS = 5UL * 60UL * 1000UL;  // 5 minutes
static const unsigned long WARMUP_MAX_MS = 20UL * 60UL * 1000UL; // 20 minutes
static const unsigned long TEMP_SAMPLE_INTERVAL_MS = 10UL * 1000UL;
static const unsigned long TEMP_PLATEAU_WINDOW_MS = 120UL * 1000UL;
static const float TEMP_PLATEAU_THRESHOLD_C = 0.25f;
static const uint8_t TEMP_MIN_SAMPLES_FOR_PLATEAU = 6;
static const uint8_t TEMP_HISTORY_CAPACITY = 24; // 4 min at the cadence above

// ----------------------------------------------------------------------------
// Stability gate. Thresholds are PEAK-TO-PEAK, not absolute magnitude: bias
// is exactly what this tool measures, so a perfectly still device reads a
// non-zero gyro rate by definition (config.h currently carries a -1.46 dps
// gyro Y offset) and an absolute-magnitude test would reject it.
//
// These are deliberately loose starting points - roughly 5x the sensor's own
// noise floor over this window - so the gate rejects real disturbance rather
// than normal quiet. Every session logs its measured peak-to-peak and tilt,
// so tighten these from real data rather than from theory.
//
// TILT DOES NOT BLOCK. Only motion does. Tilt is measured, printed and
// recorded in every row, but it never stops a session, because:
//   * waiting cannot fix tilt - only physically moving the device can, and
//     an unattended tool that blocks on it just produces nothing all night;
//   * gyro offsets are completely immune to tilt, and they are the most
//     valuable output here (accel offsets are small and partly a levelness
//     artifact anyway - see the header). Blocking the whole run over tilt
//     sacrifices good gyro data to protect accel data that was already
//     compromised.
// Instead, aggregation checks the run's median tilt against
// ACCEL_TRUST_TILT_DEG and marks the accel defines untrustworthy while
// emitting the gyro ones normally. Set GATE_TILT_BLOCKS true to go back to
// refusing to calibrate on a tilted bench.
// ----------------------------------------------------------------------------
static const unsigned long GATE_WINDOW_MS = 5000;
static const float GATE_GYRO_PP_LIMIT_DPS = 3.0f;
static const float GATE_ACCEL_PP_LIMIT_G = 0.02f;
static const float GATE_TILT_LIMIT_DEG = 1.0f;
static const bool GATE_TILT_BLOCKS = false;
static const unsigned long GATE_RETRY_MS = 10UL * 1000UL;
// While the gate is failing, serial reports every retry (free, and the point
// is to watch it converge while you level or steady the device) but the log
// gets a note at most this often, so an all-night wait can't eat the budget.
static const unsigned long GATE_NOTE_INTERVAL_MS = 5UL * 60UL * 1000UL;

// ----------------------------------------------------------------------------
// Sessions. A disturbance partway through a session can't be caught by the
// gate that preceded it, so each session tracks its own peak-to-peak and
// flags the row SUSPECT rather than discarding it - aggregation skips
// suspect rows, but they stay in the dump where they can be inspected. The
// limits are slightly looser than the gate's because a 10000-sample window
// naturally reaches further into the noise tails than a 500-sample one.
// ----------------------------------------------------------------------------
static const int NUM_SAMPLES = 10000; // ~100 s at 100 Hz - excellent averaging
static const unsigned long SESSION_GAP_MS = 60UL * 1000UL; // between sessions
static const float SESSION_GYRO_PP_LIMIT_DPS = 4.0f;
static const float SESSION_ACCEL_PP_LIMIT_G = 0.03f;

// ----------------------------------------------------------------------------
// Storage. The internal-flash filesystem region is a fixed 28 KiB on this
// chip (LFS_FLASH_ADDR / LFS_FLASH_TOTAL_SIZE in the core's
// InternalFileSystem.cpp - 7 flash pages), a bit less after LittleFS's own
// metadata. The budget below is that usable space with margin.
//
// At ~100 bytes per row and one row per (100 s session + 60 s gap), this
// fills in roughly 10-11 hours - well before a ~1000 mAh cell runs down, so
// FLASH is the binding constraint on run length, not battery. Raise
// SESSION_GAP_MS for longer wall-clock coverage at the same row count.
//
// The budget is tracked by counting bytes rather than asking the filesystem,
// so a full log stops appending cleanly (logging LOG_FULL once) instead of
// discovering the problem as a failed write. Sessions keep running after
// that and stay visible over Serial.
// ----------------------------------------------------------------------------
static const char *LOG_FILE_PATH = "/imu_cal.csv";
static const uint32_t LOG_BUDGET_BYTES = 22528; // 22 KiB

// ----------------------------------------------------------------------------
// Aggregation. Reports mean, median and a trimmed mean side by side because
// they answer different questions: the median ignores everything but rank
// (immune to outliers, but discards the information in every other session),
// while the trimmed mean drops the extreme TRIM_FRACTION from each tail and
// then averages the rest (nearly as robust, and tighter on clean data). The
// paste-ready block is emitted from the MEDIAN, which is the safe choice at
// any sample count. If the two disagree by more than the spread, the run
// hasn't converged - that disagreement is the useful signal, and it's why
// both are printed rather than one being chosen up front.
//
// AGGREGATE_SKIP_FIRST is 0 because the temperature plateau already gates
// warmup; raise it if a run's early sessions still look like outliers.
// ----------------------------------------------------------------------------
#define MAX_RECORDS 300
static const int AGGREGATE_SKIP_FIRST = 0;
static const float TRIM_FRACTION = 0.10f; // each tail -> 20% trimmed mean
// Above this median tilt, the accel defines are labelled untrustworthy in the
// paste-ready block. One degree already leaks ~17 mg into the horizontal
// axes, which is several times the bias being measured.
static const float ACCEL_TRUST_TILT_DEG = 1.0f;

// LSM6DS3 driver on the shared I2C bus.
static LSM6DS3 myIMU(I2C_MODE, IMU_I2C_ADDRESS);

// ----------------------------------------------------------------------------
// State. Session results and parsed records live at file scope (not passed as
// parameters) because the Arduino preprocessor emits function prototypes
// above type definitions, which breaks user types in signatures.
// ----------------------------------------------------------------------------
enum Phase {
  PHASE_WARMUP,
  PHASE_GATE,
  PHASE_GAP,
  PHASE_HALTED,
};
static Phase phase = PHASE_WARMUP;

// Most recent session's results, in the CSV's units.
struct CalOffsets {
  float ax, ay, az;
  float gx, gy, gz;
  float gyroPp, accelPp, tiltDeg, tempC;
};
static CalOffsets cur;
static CalOffsets prev;
static bool havePrev = false;

// One parsed CSV row, for aggregation.
struct CalRecord {
  uint16_t run;
  uint16_t session;
  uint32_t elapsedS;
  float tempC;
  float ax, ay, az;
  float gx, gy, gz;
  float tiltDeg;
  uint8_t suspect;
};
static CalRecord records[MAX_RECORDS];
static int recordCount = 0;
static float work[MAX_RECORDS]; // scratch for per-axis sorting

static uint16_t runNumber = 1;
static uint16_t sessionNumber = 0;
static uint32_t logBytesUsed = 0;
static bool logFullReported = false;
static bool fsReady = false;

// Set by whichever loop notices a keypress, so the halt handler can act on
// the key that caused the halt instead of making it a wasted keystroke.
static char pendingCmd = 0;

// Temperature history for the warmup plateau check.
static float tempHistC[TEMP_HISTORY_CAPACITY];
static unsigned long tempHistMs[TEMP_HISTORY_CAPACITY];
static uint8_t tempHistHead = 0;
static uint8_t tempHistCount = 0;

// Gate + phase timers.
static unsigned long warmupStartMs = 0;
static unsigned long gapStartMs = 0;
static unsigned long gateRetryAtMs = 0;
static bool gateWaiting = false;
static bool gateWaitReported = false;
static unsigned long gateWaitStartMs = 0;
static unsigned long gateLastNoteMs = 0;
static uint32_t gateAttempts = 0;

// ----------------------------------------------------------------------------
// Peripheral bring-up.
// ----------------------------------------------------------------------------
static void imuBringUp() {
  pinMode(IMU_POWER_PIN, OUTPUT);
  digitalWrite(IMU_POWER_PIN, HIGH);
  delay(300); // datasheet-safe boot delay before I2C

  myIMU.settings.accelEnabled = 1;
  myIMU.settings.accelRange = ACCEL_RANGE_G;
  myIMU.settings.accelSampleRate = IMU_ACCEL_ODR_HZ;
  myIMU.settings.accelBandWidth = IMU_ACCEL_BANDWIDTH_HZ;
  myIMU.settings.gyroEnabled = 1;
  myIMU.settings.gyroRange = GYRO_RANGE_DPS;
  myIMU.settings.gyroSampleRate = IMU_GYRO_ODR_HZ;
  // Must be set BEFORE begin(): the library only populates
  // settings.tempSensitivity (256 LSB/degC for the TR-C part) inside begin(),
  // and only when this flag is already 1. Enabling it afterwards leaves
  // readTempC() dividing by an unset sensitivity.
  myIMU.settings.tempEnabled = 1;

  if (myIMU.begin() != 0) {
    if (Serial) {
      Serial.println("❌ Failed to find IMU - halting");
    }
    while (1) {
      delay(100);
    }
  }

  // BDU on: prevents torn 16-bit reads from skewing an offset that has to be
  // trusted to the last mV. Same setting as g_imu.cpp uses in production.
  myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL3_C,
                      LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE |
                          LSM6DS3_ACC_GYRO_IF_INC_ENABLED);
}

// Bring the GNSS rail up purely as a thermal load - no UBX configuration, no
// fix required, no sky view needed. A searching receiver draws near its peak
// anyway, which is the point. If the module is still at its factory baud the
// bytes arrive as framing garbage; that's fine, they're discarded either way,
// and keeping the UART open matches production's pin state.
static void gnssWarmLoadOn() {
  pinMode(GNSS_EN_PIN, OUTPUT);
  digitalWrite(GNSS_EN_PIN, HIGH); // active-high enable
  Serial1.begin(GNSS_BAUD);
}

static void gnssDrain() {
  while (Serial1.available()) {
    (void)Serial1.read();
  }
}

// BLE advertising, also purely as load. No services, no connection handling -
// nothing here needs to be talked to. autoConnLed(false) keeps the stack from
// blinking the onboard LED, which would otherwise add a light source and a
// small periodic current draw to a measurement that wants neither.
static void bleWarmLoadOn() {
  Bluefruit.begin();
  Bluefruit.setName(BLE_ADV_NAME);
  Bluefruit.setTxPower(BLE_TX_POWER_ADV_DBM);
  Bluefruit.autoConnLed(false);
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);
}

// ----------------------------------------------------------------------------
// OLED status panel. Two jobs: reproduce the panel's share of production's
// thermal load, and make an unattended run legible without USB.
//
// Deliberately simpler than g_display.cpp - no burn-in pixel shift, no sliced
// frame pushes. Production slices frames so the GNSS UART is never starved of
// drain windows; here GNSS bytes are discarded, so a plain blocking
// sendBuffer is fine. The one place it would NOT be fine is inside the
// sampling loop, which is why displayTick() is never called from there - see
// performCalibration().
// ----------------------------------------------------------------------------
static float latestTempC = 0.0f;
static bool haveResult = false;

static void displayBringUp() {
  Wire.begin();
  // u8g2 wants the 8-bit left-shifted address. Passing the 7-bit value is the
  // classic silent failure: begin() returns and nothing ever appears.
  oled.setI2CAddress(DISPLAY_I2C_ADDRESS << 1);

  // Probe before committing, so a cracked, unplugged or failed panel costs a
  // warning rather than a hang.
  Wire.beginTransmission(DISPLAY_I2C_ADDRESS);
  if (Wire.endTransmission() != 0) {
    if (Serial) {
      Serial.printf("⚠️  No OLED at 0x%02X - display disabled (thermal load "
                    "will be light for this variant).\n",
                    DISPLAY_I2C_ADDRESS);
    }
    displayPresent = false;
    return;
  }

  oled.begin();
  oled.setContrast(DISPLAY_CONTRAST);
  oled.setFont(u8g2_font_5x8_tf); // 25 cols x 7 rows at this panel size
  displayPresent = true;
  if (Serial) {
    Serial.println("✅ OLED panel up (thermal load + standalone readout).");
  }
}

// phaseText is the one-line "what is happening now"; force bypasses the 1 Hz
// throttle for state changes worth showing immediately.
static void displayTick(const char *phaseText, bool force) {
  if (!displayPresent) {
    return;
  }
  static unsigned long lastMs = 0;
  const unsigned long nowMs = millis();
  if (!force && (nowMs - lastMs) < DISPLAY_REFRESH_MS) {
    return;
  }
  lastMs = nowMs;

  char l[32];
  oled.clearBuffer();

  snprintf(l, sizeof(l), "Gnimu IMU cal   run %u", (unsigned)runNumber);
  oled.drawStr(0, 8, l);
  oled.drawStr(0, 17, phaseText);
  snprintf(l, sizeof(l), "die %.2fC  sess %u", latestTempC,
           (unsigned)sessionNumber);
  oled.drawStr(0, 26, l);
  snprintf(l, sizeof(l), "log %lu/%lu B", (unsigned long)logBytesUsed,
           (unsigned long)LOG_BUDGET_BYTES);
  oled.drawStr(0, 35, l);

  if (haveResult) {
    snprintf(l, sizeof(l), "a %+.3f %+.3f %+.3f", cur.ax, cur.ay, cur.az);
    oled.drawStr(0, 49, l);
    snprintf(l, sizeof(l), "g %+.3f %+.3f %+.3f", cur.gx, cur.gy, cur.gz);
    oled.drawStr(0, 58, l);
  } else {
    oled.drawStr(0, 49, "no session yet");
  }

  oled.sendBuffer();
}

// ----------------------------------------------------------------------------
// Log file.
//
// Every row carries its own run number, so aggregation needs no state beyond
// the file itself. Lines beginning with '#' are commentary (the header and
// the per-boot marker) and are skipped by the parser.
// ----------------------------------------------------------------------------
static void logAppend(const char *line) {
  if (!fsReady) {
    return;
  }
  const uint32_t len = (uint32_t)strlen(line);
  if (logBytesUsed + len > LOG_BUDGET_BYTES) {
    if (!logFullReported) {
      logFullReported = true;
      if (Serial) {
        Serial.printf("\n⚠️  Log budget full (%lu bytes) - no longer "
                      "appending. Sessions continue; aggregate or erase.\n",
                      (unsigned long)LOG_BUDGET_BYTES);
      }
    }
    return;
  }

  File f = InternalFS.open(LOG_FILE_PATH, FILE_O_WRITE);
  if (!f) {
    if (Serial) {
      Serial.println("❌ Failed to open log file for append.");
    }
    return;
  }
  f.write((const uint8_t *)line, len);
  f.close();
  logBytesUsed += len;
}

// Scan the existing log to size it and to find the highest run number, so
// this boot starts a new run that appends rather than collides.
static void logScanExisting() {
  logBytesUsed = 0;
  uint16_t maxRun = 0;

  File f = InternalFS.open(LOG_FILE_PATH, FILE_O_READ);
  if (!f) {
    return; // no log yet - run 1, zero bytes used
  }
  logBytesUsed = (uint32_t)f.size();

  char line[160];
  size_t n = 0;
  int c;
  while ((c = f.read()) >= 0) {
    if (c == '\n') {
      line[n] = '\0';
      if (n > 0 && line[0] != '#') {
        const unsigned long r = strtoul(line, NULL, 10);
        if (r > maxRun && r < 65535UL) {
          maxRun = (uint16_t)r;
        }
      }
      n = 0;
    } else if (n < sizeof(line) - 1) {
      line[n++] = (char)c;
    }
  }
  f.close();
  runNumber = (uint16_t)(maxRun + 1);
}

static void logWriteHeaderIfNew() {
  if (logBytesUsed > 0) {
    return;
  }
  logAppend("# run,session,elapsed_s,temp_c,ax_g,ay_g,az_g,gx_dps,gy_dps,"
            "gz_dps,gyro_pp_dps,accel_pp_g,tilt_deg,flag\n");
}

static void logBootMarker() {
  char line[64];
  snprintf(line, sizeof(line), "# --- RUN %u boot ---\n", (unsigned)runNumber);
  logAppend(line);
}

static void logSession() {
  char line[176];
  snprintf(line, sizeof(line),
           "%u,%u,%lu,%.2f,%+.6f,%+.6f,%+.6f,%+.6f,%+.6f,%+.6f,%.3f,%.4f,"
           "%.2f,%s\n",
           (unsigned)runNumber, (unsigned)sessionNumber,
           (unsigned long)(millis() / 1000UL), cur.tempC, cur.ax, cur.ay,
           cur.az, cur.gx, cur.gy, cur.gz, cur.gyroPp, cur.accelPp, cur.tiltDeg,
           (cur.gyroPp > SESSION_GYRO_PP_LIMIT_DPS ||
            cur.accelPp > SESSION_ACCEL_PP_LIMIT_G)
               ? "SUSPECT"
               : "OK");
  logAppend(line);
}

static void logNote(const char *note) {
  // Sized to hold the prefix plus the longest caller-supplied note without
  // snprintf having to truncate it.
  char line[160];
  snprintf(line, sizeof(line), "# run %u t=%lus %s\n", (unsigned)runNumber,
           (unsigned long)(millis() / 1000UL), note);
  logAppend(line);
}

static void dumpLog() {
  File f = InternalFS.open(LOG_FILE_PATH, FILE_O_READ);
  if (!f) {
    Serial.println("(no log file yet)");
    return;
  }
  Serial.printf("\n--- %s (%lu bytes) ---\n", LOG_FILE_PATH,
                (unsigned long)f.size());
  uint8_t buf[64];
  int n;
  while ((n = f.read(buf, sizeof(buf))) > 0) {
    Serial.write(buf, n);
  }
  f.close();
  Serial.println("--- end of log ---");
}

static void eraseLog() {
  InternalFS.remove(LOG_FILE_PATH);
  logBytesUsed = 0;
  logFullReported = false;
  Serial.println("Log erased. Reboot to start a fresh run 1.");
}

// ----------------------------------------------------------------------------
// Temperature history (warmup plateau detection).
// ----------------------------------------------------------------------------
static void tempHistPush(unsigned long nowMs, float c) {
  tempHistMs[tempHistHead] = nowMs;
  tempHistC[tempHistHead] = c;
  tempHistHead = (uint8_t)((tempHistHead + 1) % TEMP_HISTORY_CAPACITY);
  if (tempHistCount < TEMP_HISTORY_CAPACITY) {
    tempHistCount++;
  }
}

// True once the trailing TEMP_PLATEAU_WINDOW_MS of readings spans less than
// TEMP_PLATEAU_THRESHOLD_C, given enough samples to make that meaningful.
static bool tempPlateaued(unsigned long nowMs) {
  if (nowMs < TEMP_PLATEAU_WINDOW_MS) {
    return false;
  }
  const unsigned long sinceMs = nowMs - TEMP_PLATEAU_WINDOW_MS;
  float mn = 1e9f, mx = -1e9f;
  uint8_t n = 0;
  for (uint8_t i = 0; i < tempHistCount; i++) {
    if (tempHistMs[i] >= sinceMs) {
      if (tempHistC[i] < mn) {
        mn = tempHistC[i];
      }
      if (tempHistC[i] > mx) {
        mx = tempHistC[i];
      }
      n++;
    }
  }
  return n >= TEMP_MIN_SAMPLES_FOR_PLATEAU &&
         (mx - mn) <= TEMP_PLATEAU_THRESHOLD_C;
}

// ----------------------------------------------------------------------------
// Serial command plumbing.
//
// Checked from inside every phase (including partway through a session) so a
// keypress is honored within milliseconds rather than up to 100 s later.
// The key that caused the halt is kept and executed by the halt handler.
// ----------------------------------------------------------------------------
static void printMenu() {
  Serial.println("\n=== halted - calibration stopped ===");
  Serial.println("  d - dump raw CSV log");
  Serial.println("  a - aggregate most recent run -> config.h defines");
  Serial.println("  e - erase log");
  Serial.println("  b - reboot (fresh run, no need to touch the device)");
  Serial.println("  ? - this menu");
}

// Returns true if a keypress arrived; the caller must abandon what it is
// doing and return so loop() can enter PHASE_HALTED.
static bool checkHaltRequest() {
  if (!Serial || !Serial.available()) {
    return false;
  }
  pendingCmd = (char)Serial.read();
  phase = PHASE_HALTED;
  return true;
}

// ----------------------------------------------------------------------------
// Stability gate: GATE_WINDOW_MS of motion measurement, plus a (non-blocking)
// levelness reading. Returns true if the device is quiet enough to calibrate.
//
// Per-axis mean, peak-to-peak AND standard deviation are all kept, because
// p-p on its own can't distinguish real disturbance from a few bad samples.
// One torn 16-bit read produces a single absurd value that blows up max-min
// while barely moving sd; genuine vibration raises both together. Seeing the
// pair side by side is what makes a failing gate diagnosable rather than
// just discouraging.
// ----------------------------------------------------------------------------
// Indices into the per-axis arrays below.
#define AX 0
#define AY 1
#define AZ 2
#define GX 3
#define GY 4
#define GZ 5

static float gateMean[6];
static float gatePp[6];
static float gateSd[6];
static float gateGyroPp = 0.0f;  // worst of the three gyro axes
static float gateAccelPp = 0.0f; // worst of the three accel axes
static float gateTiltDeg = 0.0f;

static bool stabilityGate() {
  double sum[6] = {0, 0, 0, 0, 0, 0};
  double sumSq[6] = {0, 0, 0, 0, 0, 0};
  float mn[6], mx[6];
  for (int i = 0; i < 6; i++) {
    mn[i] = 1e9f;
    mx[i] = -1e9f;
  }
  uint32_t n = 0;

  const unsigned long start = millis();
  unsigned long nextSampleMs = start;
  while (millis() - start < GATE_WINDOW_MS) {
    if (checkHaltRequest()) {
      return false;
    }
    gnssDrain();
    while ((int32_t)(millis() - nextSampleMs) < 0) {
      // wait for the next sample tick
    }
    nextSampleMs += IMU_SAMPLE_INTERVAL_MS;

    float s[6];
    s[AX] = myIMU.readFloatAccelX();
    s[AY] = myIMU.readFloatAccelY();
    s[AZ] = myIMU.readFloatAccelZ();
    s[GX] = myIMU.readFloatGyroX();
    s[GY] = myIMU.readFloatGyroY();
    s[GZ] = myIMU.readFloatGyroZ();

    for (int i = 0; i < 6; i++) {
      sum[i] += s[i];
      sumSq[i] += (double)s[i] * s[i];
      if (s[i] < mn[i])
        mn[i] = s[i];
      if (s[i] > mx[i])
        mx[i] = s[i];
    }
    n++;
  }

  if (n < 2) {
    return false;
  }

  for (int i = 0; i < 6; i++) {
    gateMean[i] = (float)(sum[i] / n);
    gatePp[i] = mx[i] - mn[i];
    const double var =
        (sumSq[i] - (double)n * gateMean[i] * gateMean[i]) / (double)(n - 1);
    gateSd[i] = (var > 0.0) ? (float)sqrt(var) : 0.0f;
  }

  gateAccelPp = gatePp[AX];
  if (gatePp[AY] > gateAccelPp)
    gateAccelPp = gatePp[AY];
  if (gatePp[AZ] > gateAccelPp)
    gateAccelPp = gatePp[AZ];
  gateGyroPp = gatePp[GX];
  if (gatePp[GY] > gateGyroPp)
    gateGyroPp = gatePp[GY];
  if (gatePp[GZ] > gateGyroPp)
    gateGyroPp = gatePp[GZ];

  // Angle between the measured gravity vector and +Z. A device that is
  // upside down or on its side lands past 90 degrees.
  gateTiltDeg = degrees(
      atan2f(sqrtf(gateMean[AX] * gateMean[AX] + gateMean[AY] * gateMean[AY]),
             gateMean[AZ]));

  const bool motionOk = gateGyroPp <= GATE_GYRO_PP_LIMIT_DPS &&
                        gateAccelPp <= GATE_ACCEL_PP_LIMIT_G;
  const bool tiltOk = !GATE_TILT_BLOCKS || gateTiltDeg <= GATE_TILT_LIMIT_DEG;
  return motionOk && tiltOk;
}

// Full per-axis picture, printed on every failed gate so a wait is something
// you can act on: which axis is moving, and whether p-p and sd agree.
static void printGateDetail() {
  if (!Serial) {
    return;
  }
  Serial.printf("   accel mean %+.4f %+.4f %+.4f g   p-p %.4f %.4f %.4f   "
                "sd %.4f %.4f %.4f\n",
                gateMean[AX], gateMean[AY], gateMean[AZ], gatePp[AX],
                gatePp[AY], gatePp[AZ], gateSd[AX], gateSd[AY], gateSd[AZ]);
  Serial.printf("   gyro  mean %+.3f %+.3f %+.3f dps p-p %.3f %.3f %.3f   "
                "sd %.3f %.3f %.3f\n",
                gateMean[GX], gateMean[GY], gateMean[GZ], gatePp[GX],
                gatePp[GY], gatePp[GZ], gateSd[GX], gateSd[GY], gateSd[GZ]);
  Serial.printf("   limits: gyro p-p %.3f, accel p-p %.4f   tilt %.2f deg "
                "(%s)\n",
                GATE_GYRO_PP_LIMIT_DPS, GATE_ACCEL_PP_LIMIT_G, gateTiltDeg,
                GATE_TILT_BLOCKS ? "blocking" : "recorded, not blocking");
  // A high p-p next to a low sd means a handful of wild samples, not a
  // moving bench - averaging would have absorbed them, so the gate is the
  // only place they are visible at all.
  if (gateGyroPp > GATE_GYRO_PP_LIMIT_DPS && gateGyroPp > 20.0f * gateSd[GX] &&
      gateGyroPp > 20.0f * gateSd[GY] && gateGyroPp > 20.0f * gateSd[GZ]) {
    Serial.println("   note: gyro p-p is huge relative to sd - looks like a "
                   "few outlier samples rather than real motion.");
  }
}

// ----------------------------------------------------------------------------
// Calibration session. Returns false if a keypress aborted it partway, in
// which case the partial average is discarded - a half-length session isn't
// worth the special case in the record format, and the next boot re-measures
// it in 100 seconds.
// ----------------------------------------------------------------------------
static bool performCalibration() {
  if (Serial) {
    Serial.printf("\n=== Run %u session %u: sampling %d @ %d ms ===\n",
                  (unsigned)runNumber, (unsigned)sessionNumber, NUM_SAMPLES,
                  IMU_SAMPLE_INTERVAL_MS);
  }

  double sumAx = 0, sumAy = 0, sumAz = 0;
  double sumGx = 0, sumGy = 0, sumGz = 0;
  float gxMin = 1e9f, gxMax = -1e9f, gyMin = 1e9f, gyMax = -1e9f;
  float gzMin = 1e9f, gzMax = -1e9f;
  float axMin = 1e9f, axMax = -1e9f, ayMin = 1e9f, ayMax = -1e9f;
  float azMin = 1e9f, azMax = -1e9f;

  unsigned long nextSampleMs = millis();
  for (int i = 0; i < NUM_SAMPLES; i++) {
    if (checkHaltRequest()) {
      if (Serial) {
        Serial.printf("\n(session aborted at %d/%d samples - partial average "
                      "discarded)\n",
                      i, NUM_SAMPLES);
      }
      return false;
    }
    gnssDrain();
    while ((int32_t)(millis() - nextSampleMs) < 0) {
      // wait for the next sample tick
    }
    nextSampleMs += IMU_SAMPLE_INTERVAL_MS;

    const float ax = myIMU.readFloatAccelX();
    const float ay = myIMU.readFloatAccelY();
    const float az = myIMU.readFloatAccelZ();
    const float gx = myIMU.readFloatGyroX();
    const float gy = myIMU.readFloatGyroY();
    const float gz = myIMU.readFloatGyroZ();

    sumAx += ax;
    sumAy += ay;
    sumAz += az;
    sumGx += gx;
    sumGy += gy;
    sumGz += gz;

    if (ax < axMin)
      axMin = ax;
    if (ax > axMax)
      axMax = ax;
    if (ay < ayMin)
      ayMin = ay;
    if (ay > ayMax)
      ayMax = ay;
    if (az < azMin)
      azMin = az;
    if (az > azMax)
      azMax = az;
    if (gx < gxMin)
      gxMin = gx;
    if (gx > gxMax)
      gxMax = gx;
    if (gy < gyMin)
      gyMin = gy;
    if (gy > gyMax)
      gyMax = gy;
    if (gz < gzMin)
      gzMin = gz;
    if (gz > gzMax)
      gzMax = gz;

    if (Serial && i > 0 && i % 1000 == 0) {
      Serial.printf(" [%d/%d]", i, NUM_SAMPLES);
    }
  }

  const float mx = (float)(sumAx / NUM_SAMPLES);
  const float my = (float)(sumAy / NUM_SAMPLES);
  const float mz = (float)(sumAz / NUM_SAMPLES);

  // Averages. Accel Z has gravity subtracted (device is +Z-up on a level
  // surface, so its true value at rest is +1.0 g).
  cur.ax = mx;
  cur.ay = my;
  cur.az = mz - 1.0f;
  cur.gx = (float)(sumGx / NUM_SAMPLES);
  cur.gy = (float)(sumGy / NUM_SAMPLES);
  cur.gz = (float)(sumGz / NUM_SAMPLES);

  float gpp = gxMax - gxMin;
  if ((gyMax - gyMin) > gpp)
    gpp = gyMax - gyMin;
  if ((gzMax - gzMin) > gpp)
    gpp = gzMax - gzMin;
  float app = axMax - axMin;
  if ((ayMax - ayMin) > app)
    app = ayMax - ayMin;
  if ((azMax - azMin) > app)
    app = azMax - azMin;
  cur.gyroPp = gpp;
  cur.accelPp = app;
  // Tilt from the session's own 10000-sample mean - a better estimate than
  // the gate's 500-sample one, and the number worth recording.
  cur.tiltDeg = degrees(atan2f(sqrtf(mx * mx + my * my), mz));
  cur.tempC = myIMU.readTempC();

  return true;
}

static void printSession() {
  if (!Serial) {
    return;
  }
  Serial.printf("\naccel %+.6f %+.6f %+.6f g   gyro %+.6f %+.6f %+.6f dps\n",
                cur.ax, cur.ay, cur.az, cur.gx, cur.gy, cur.gz);
  Serial.printf("temp %.2f C   p-p gyro %.3f dps / accel %.4f g   tilt %.2f "
                "deg   %s\n",
                cur.tempC, cur.gyroPp, cur.accelPp, cur.tiltDeg,
                (cur.gyroPp > SESSION_GYRO_PP_LIMIT_DPS ||
                 cur.accelPp > SESSION_ACCEL_PP_LIMIT_G)
                    ? "SUSPECT"
                    : "OK");
  if (havePrev) {
    Serial.printf("delta vs previous:    accel %+.6f %+.6f %+.6f g\n",
                  cur.ax - prev.ax, cur.ay - prev.ay, cur.az - prev.az);
    Serial.printf("                      gyro  %+.6f %+.6f %+.6f dps\n",
                  cur.gx - prev.gx, cur.gy - prev.gy, cur.gz - prev.gz);
  }
}

// ----------------------------------------------------------------------------
// Aggregation.
// ----------------------------------------------------------------------------

// Split a CSV line in place into up to maxFields pointers. Returns the count.
static int splitCsv(char *line, char **fields, int maxFields) {
  int n = 0;
  char *p = line;
  fields[n++] = p;
  while (*p && n < maxFields) {
    if (*p == ',') {
      *p = '\0';
      fields[n++] = p + 1;
    }
    p++;
  }
  return n;
}

// Load every row belonging to the highest run number present in the file.
// Rows arrive in chronological order and run numbers only increase, so a
// single pass works: a higher run number invalidates everything gathered so
// far. This is what makes "aggregate the most recent set" do the right thing
// after a fresh boot, where the current run has no rows yet and the previous
// boot's rows are the newest available.
static void loadLatestRun() {
  recordCount = 0;
  uint16_t targetRun = 0;

  File f = InternalFS.open(LOG_FILE_PATH, FILE_O_READ);
  if (!f) {
    return;
  }

  char line[176];
  char *fields[16];
  size_t n = 0;
  int c;
  while ((c = f.read()) >= 0) {
    if (c != '\n') {
      if (n < sizeof(line) - 1) {
        line[n++] = (char)c;
      }
      continue;
    }
    line[n] = '\0';
    const size_t len = n;
    n = 0;
    if (len == 0 || line[0] == '#') {
      continue;
    }

    const int nf = splitCsv(line, fields, 16);
    if (nf < 14) {
      continue; // truncated final row (power loss mid-append) - skip it
    }
    const uint16_t r = (uint16_t)strtoul(fields[0], NULL, 10);
    if (r > targetRun) {
      targetRun = r;
      recordCount = 0; // newer run supersedes everything collected so far
    }
    if (r != targetRun || recordCount >= MAX_RECORDS) {
      continue;
    }

    CalRecord *rec = &records[recordCount++];
    rec->run = r;
    rec->session = (uint16_t)strtoul(fields[1], NULL, 10);
    rec->elapsedS = (uint32_t)strtoul(fields[2], NULL, 10);
    rec->tempC = strtof(fields[3], NULL);
    rec->ax = strtof(fields[4], NULL);
    rec->ay = strtof(fields[5], NULL);
    rec->az = strtof(fields[6], NULL);
    rec->gx = strtof(fields[7], NULL);
    rec->gy = strtof(fields[8], NULL);
    rec->gz = strtof(fields[9], NULL);
    rec->tiltDeg = strtof(fields[12], NULL);
    rec->suspect = (strncmp(fields[13], "SUSPECT", 7) == 0) ? 1 : 0;
  }
  f.close();
}

static void sortFloats(float *a, int n) {
  for (int i = 1; i < n; i++) {
    const float v = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > v) {
      a[j + 1] = a[j];
      j--;
    }
    a[j + 1] = v;
  }
}

// Prints one axis's statistics and returns its median (the value the
// paste-ready block is built from).
static float summarizeAxis(const char *name, const char *unit, int n) {
  sortFloats(work, n);

  double sum = 0;
  for (int i = 0; i < n; i++) {
    sum += work[i];
  }
  const float mean = (float)(sum / n);

  const float median =
      (n % 2) ? work[n / 2] : (work[n / 2 - 1] + work[n / 2]) * 0.5f;

  int trim = (int)(n * TRIM_FRACTION);
  if (trim * 2 >= n) {
    trim = 0; // too few samples to trim anything meaningful
  }
  double tsum = 0;
  for (int i = trim; i < n - trim; i++) {
    tsum += work[i];
  }
  const float trimmed = (float)(tsum / (n - 2 * trim));

  double var = 0;
  for (int i = 0; i < n; i++) {
    const double d = work[i] - mean;
    var += d * d;
  }
  const float sd = (n > 1) ? (float)sqrt(var / (n - 1)) : 0.0f;

  Serial.printf("  %-9s mean %+.6f  median %+.6f  trim20 %+.6f  sd %.6f  "
                "min %+.6f  max %+.6f  %s\n",
                name, mean, median, trimmed, sd, work[0], work[n - 1], unit);

  // The median and the trimmed mean answer the same question two ways; when
  // they disagree by more than the session-to-session spread, the run is
  // still drifting and more sessions are worth more than either number.
  if (sd > 0.0f && fabsf(median - trimmed) > sd) {
    Serial.printf("            ^ median and trim20 differ by more than 1 sd - "
                  "run not converged\n");
  }
  return median;
}

static void aggregate() {
  loadLatestRun();
  if (recordCount == 0) {
    Serial.println("\n(no calibration rows in the log yet - nothing to "
                   "aggregate)");
    return;
  }

  // Compact in place: drop suspect rows and the first AGGREGATE_SKIP_FIRST.
  int used = 0, dropped = 0, skipped = 0;
  for (int i = 0; i < recordCount; i++) {
    if (i < AGGREGATE_SKIP_FIRST) {
      skipped++;
      continue;
    }
    if (records[i].suspect) {
      dropped++;
      continue;
    }
    records[used++] = records[i];
  }

  Serial.printf("\n=== AGGREGATE - run %u ===\n", (unsigned)records[0].run);
  Serial.printf("rows: %d usable (%d suspect dropped, %d skipped as early)\n",
                used, dropped, skipped);
  if (used == 0) {
    Serial.println("No usable rows - every session was flagged SUSPECT. The "
                   "surface was not stable.");
    return;
  }

  float tMin = records[0].tempC, tMax = records[0].tempC;
  for (int i = 1; i < used; i++) {
    if (records[i].tempC < tMin)
      tMin = records[i].tempC;
    if (records[i].tempC > tMax)
      tMax = records[i].tempC;
  }
  Serial.printf(
      "span: %lu s of run time, die temp %.2f - %.2f C\n",
      (unsigned long)(records[used - 1].elapsedS - records[0].elapsedS), tMin,
      tMax);

  // Median tilt across the run decides whether the accel numbers are worth
  // pasting. Median rather than mean so one disturbed session can't condemn
  // (or rescue) an otherwise consistent pose.
  for (int i = 0; i < used; i++) {
    work[i] = records[i].tiltDeg;
  }
  sortFloats(work, used);
  const float medianTilt = (used % 2)
                               ? work[used / 2]
                               : (work[used / 2 - 1] + work[used / 2]) * 0.5f;
  const bool accelTrusted = medianTilt <= ACCEL_TRUST_TILT_DEG;
  Serial.printf("pose: median tilt %.2f deg -> accel offsets %s\n", medianTilt,
                accelTrusted ? "usable" : "NOT TRUSTWORTHY (see below)");
  Serial.println();

  CalOffsets agg;
  for (int i = 0; i < used; i++)
    work[i] = records[i].ax;
  agg.ax = summarizeAxis("accel X", "g", used);
  for (int i = 0; i < used; i++)
    work[i] = records[i].ay;
  agg.ay = summarizeAxis("accel Y", "g", used);
  for (int i = 0; i < used; i++)
    work[i] = records[i].az;
  agg.az = summarizeAxis("accel Z", "g", used);
  for (int i = 0; i < used; i++)
    work[i] = records[i].gx;
  agg.gx = summarizeAxis("gyro X", "dps", used);
  for (int i = 0; i < used; i++)
    work[i] = records[i].gy;
  agg.gy = summarizeAxis("gyro Y", "dps", used);
  for (int i = 0; i < used; i++)
    work[i] = records[i].gz;
  agg.gz = summarizeAxis("gyro Z", "dps", used);

  Serial.println("\nPaste these lines into config.h's IMU-offsets section:");
  Serial.println("-------------------------------------------");
  // The gyro block is emitted plainly whatever the pose was - tilt cannot
  // affect a rate measurement, so these stay good even on a sloped bench.
  Serial.printf("#define IMU_GYRO_OFFSET_X_DPS %+.6ff\n", agg.gx);
  Serial.printf("#define IMU_GYRO_OFFSET_Y_DPS %+.6ff\n", agg.gy);
  Serial.printf("#define IMU_GYRO_OFFSET_Z_DPS %+.6ff\n", agg.gz);
  if (accelTrusted) {
    Serial.printf("#define IMU_ACCEL_OFFSET_X_G  %+.6ff\n", agg.ax);
    Serial.printf("#define IMU_ACCEL_OFFSET_Y_G  %+.6ff\n", agg.ay);
    Serial.printf("#define IMU_ACCEL_OFFSET_Z_G  %+.6ff\n", agg.az);
  } else {
    // Commented out deliberately: at this tilt these are mostly a
    // measurement of the bench, and pasting them would bake that slope into
    // the firmware as if it were sensor bias.
    Serial.printf("// DO NOT PASTE - measured at %.2f deg tilt, so these are\n",
                  medianTilt);
    Serial.printf("// mostly leaked gravity (~17 mg/deg), not sensor bias:\n");
    Serial.printf("// #define IMU_ACCEL_OFFSET_X_G  %+.6ff\n", agg.ax);
    Serial.printf("// #define IMU_ACCEL_OFFSET_Y_G  %+.6ff\n", agg.ay);
    Serial.printf("// #define IMU_ACCEL_OFFSET_Z_G  %+.6ff\n", agg.az);
  }
  Serial.println("-------------------------------------------");
  Serial.println("Values above are per-axis MEDIANS. Sanity check: |accel| "
                 "offsets are typically <0.1 g, gyro <5 dps.");
  if (!accelTrusted) {
    Serial.printf("Accel withheld: median tilt %.2f deg exceeds %.2f deg. "
                  "Re-level the device and rerun for accel; the gyro values "
                  "above are unaffected and ready to use.\n",
                  medianTilt, ACCEL_TRUST_TILT_DEG);
  } else {
    Serial.println("Note: accel X/Y also absorb bench tilt (~17 mg per "
                   "degree) - see this sketch's header before trusting them.");
  }
}

// ----------------------------------------------------------------------------
// Phases.
// ----------------------------------------------------------------------------
static void warmupTick() {
  static unsigned long lastTempMs = 0;
  static unsigned long lastLogMs = 0;

  if (checkHaltRequest()) {
    return;
  }
  gnssDrain();

  const unsigned long nowMs = millis();
  const unsigned long elapsed = nowMs - warmupStartMs;

  if (nowMs - lastTempMs >= TEMP_SAMPLE_INTERVAL_MS || tempHistCount == 0) {
    lastTempMs = nowMs;
    latestTempC = myIMU.readTempC();
    tempHistPush(nowMs, latestTempC);
  }

  char phaseText[32];
  snprintf(phaseText, sizeof(phaseText), "WARMUP %lus", elapsed / 1000UL);
  displayTick(phaseText, false);

  if (Serial && nowMs - lastLogMs >= 30000UL) {
    lastLogMs = nowMs;
    Serial.printf("warmup %lus  temp %.2f C\n", elapsed / 1000UL,
                  tempHistC[(tempHistHead + TEMP_HISTORY_CAPACITY - 1) %
                            TEMP_HISTORY_CAPACITY]);
  }

  const bool floorReached = elapsed >= WARMUP_MIN_MS;
  const bool ceilingReached = elapsed >= WARMUP_MAX_MS;
  const bool plateaued = floorReached && tempPlateaued(nowMs);
  if (plateaued || ceilingReached) {
    // Which of the two ended warmup is worth recording: repeatedly hitting
    // the ceiling means the board never reached steady state, and every
    // session after it is measuring a still-drifting die.
    if (Serial) {
      Serial.printf("\nWarmup complete after %lus (%s).\n", elapsed / 1000UL,
                    plateaued ? "die temp plateaued"
                              : "ceiling reached, temp still moving");
    }
    logNote(plateaued ? "WARMUP_END plateau" : "WARMUP_END ceiling");
    phase = PHASE_GATE;
  }

  delay(50);
}

static void gateTick() {
  if (checkHaltRequest()) {
    return;
  }
  const unsigned long nowMs = millis();
  if (gateWaiting && (int32_t)(nowMs - gateRetryAtMs) < 0) {
    gnssDrain();
    char phaseText[32];
    snprintf(phaseText, sizeof(phaseText), "NOT STABLE - retry %lu",
             (unsigned long)gateAttempts);
    displayTick(phaseText, false);
    delay(50);
    return;
  }
  gateWaiting = false;

  if (!stabilityGate()) {
    if (phase == PHASE_HALTED) {
      return; // gate was interrupted, not failed
    }
    // Report EVERY retry, not just the first. A silent retry loop is
    // indistinguishable from a hang, and these numbers are exactly what you
    // want in front of you while steadying or levelling the device.
    if (!gateWaitReported) {
      gateWaitReported = true;
      gateWaitStartMs = millis();
      gateLastNoteMs = 0;
    }
    gateAttempts++;
    if (Serial) {
      Serial.printf("\n⏸  Not stable - attempt %lu, waiting %lus. Retrying "
                    "every %lus; press any key to halt.\n",
                    (unsigned long)gateAttempts,
                    (unsigned long)((millis() - gateWaitStartMs) / 1000UL),
                    GATE_RETRY_MS / 1000UL);
      printGateDetail();
    }
    // The log gets a note far less often than serial does - an unattended
    // all-night wait must not fill the budget with retry lines.
    const unsigned long nowNoteMs = millis();
    if (gateLastNoteMs == 0 ||
        (nowNoteMs - gateLastNoteMs) >= GATE_NOTE_INTERVAL_MS) {
      gateLastNoteMs = nowNoteMs;
      char note[112];
      snprintf(note, sizeof(note),
               "GATE_WAIT n=%lu pp_gyro=%.3f pp_accel=%.4f tilt=%.2f",
               (unsigned long)gateAttempts, gateGyroPp, gateAccelPp,
               gateTiltDeg);
      logNote(note);
    }
    gateRetryAtMs = millis() + GATE_RETRY_MS;
    gateWaiting = true;
    return;
  }

  if (gateWaitReported) {
    gateWaitReported = false;
    char note[80];
    snprintf(note, sizeof(note), "GATE_OK after %lu attempts, %lus",
             (unsigned long)gateAttempts,
             (unsigned long)((millis() - gateWaitStartMs) / 1000UL));
    logNote(note);
    if (Serial) {
      Serial.printf("▶️  Stable again after %lus - resuming.\n",
                    (unsigned long)((millis() - gateWaitStartMs) / 1000UL));
    }
    gateAttempts = 0;
  }

  sessionNumber++;
  // Painted once here and not touched again until the session ends: a
  // blocking ~31 ms sendBuffer inside the sampling loop would swallow three
  // 10 ms sample ticks, and the deadline-anchored pacing would then take
  // three reads back to back off the same 104 Hz sensor sample - duplicated
  // values, slightly over-weighting one instant in the average.
  displayTick("SAMPLING...", true);
  if (!performCalibration()) {
    return; // aborted into PHASE_HALTED
  }
  latestTempC = cur.tempC;
  haveResult = true;
  printSession();
  logSession();
  prev = cur;
  havePrev = true;

  gapStartMs = millis();
  phase = PHASE_GAP;
  if (Serial) {
    Serial.printf("Next session in %lus (press any key to halt)...\n",
                  SESSION_GAP_MS / 1000UL);
  }
}

static void gapTick() {
  if (checkHaltRequest()) {
    return;
  }
  gnssDrain();
  const unsigned long gapElapsed = millis() - gapStartMs;
  if (gapElapsed >= SESSION_GAP_MS) {
    phase = PHASE_GATE;
    return;
  }
  char phaseText[32];
  snprintf(phaseText, sizeof(phaseText), "GAP %lus",
           (SESSION_GAP_MS - gapElapsed) / 1000UL);
  displayTick(phaseText, false);
  delay(50);
}

static void haltedTick() {
  static bool menuShown = false;
  if (!menuShown) {
    menuShown = true;
    displayTick("HALTED - see serial", true);
    if (Serial) {
      printMenu();
    }
  }

  char c = pendingCmd;
  pendingCmd = 0;
  if (c == 0) {
    if (!Serial || !Serial.available()) {
      delay(50);
      return;
    }
    c = (char)Serial.read();
  }

  switch (c) {
  case 'd':
  case 'D':
    dumpLog();
    break;
  case 'a':
  case 'A':
    aggregate();
    break;
  case 'e':
  case 'E':
    eraseLog();
    break;
  case 'b':
  case 'B':
    Serial.println("Rebooting...");
    Serial.flush();
    delay(100);
    NVIC_SystemReset();
    break;
  case '?':
    printMenu();
    break;
  case '\r':
  case '\n':
    break; // stray line ending from the halting keypress
  default:
    Serial.printf("(unknown command '%c' - press ? for the menu)\n", c);
    break;
  }
}

// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  // Bounded, never blocking: the whole point of this revision is that the
  // device runs standalone. Three seconds is only enough for CDC to
  // enumerate when USB happens to be attached at boot.
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
  }

  // Internal flash first, before the SoftDevice is enabled. Bluefruit shares
  // this filesystem (bonding data lives there), and once the SoftDevice is
  // up every flash write has to go through it, where radio activity can
  // defer an erase/program. The core handles that, but it's the first thing
  // to suspect if an append ever fails on the bench.
  fsReady = InternalFS.begin();
  if (!fsReady) {
    // First-time use on this board (or a corrupt filesystem) - format once.
    // Safe here (unlike the main firmware's storage policy): this is a
    // dedicated bench tool, not production code that must never silently
    // destroy data.
    if (Serial) {
      Serial.println("InternalFS mount failed - formatting...");
    }
    InternalFS.format();
    fsReady = InternalFS.begin();
  }
  if (fsReady) {
    logScanExisting();
  }

  if (Serial) {
    Serial.println("\n=== Gnimu - unattended IMU calibration ===");
    Serial.printf("Run %u. Log %s: %lu / %lu bytes used.\n",
                  (unsigned)runNumber, LOG_FILE_PATH,
                  (unsigned long)logBytesUsed, (unsigned long)LOG_BUDGET_BYTES);
    Serial.println("Device must be on a level surface, LED face UP (+Z up), "
                   "and untouched from here on.");
    Serial.printf("Warmup %lu-%lu min (until die temp plateaus), then "
                  "%d-sample sessions %lus apart, forever.\n",
                  WARMUP_MIN_MS / 60000UL, WARMUP_MAX_MS / 60000UL, NUM_SAMPLES,
                  SESSION_GAP_MS / 1000UL);
    Serial.println("Press any key at any time to halt and dump/aggregate.");
  }

  imuBringUp();
  // GNSS and BLE are thermal load only - see the header. Brought up after the
  // IMU so an IMU failure halts before anything starts drawing current.
  gnssWarmLoadOn();
  bleWarmLoadOn();
  displayBringUp();
  if (Serial) {
    Serial.println("✅ IMU up; GNSS rail and BLE advertising on (thermal "
                   "load).");
  }
  latestTempC = myIMU.readTempC();
  displayTick("starting warmup", true);

  if (fsReady) {
    logWriteHeaderIfNew();
    logBootMarker();
  } else if (Serial) {
    Serial.println("⚠️  No filesystem - sessions will print but NOT be "
                   "logged.");
  }

  warmupStartMs = millis();
  phase = PHASE_WARMUP;
}

void loop() {
  switch (phase) {
  case PHASE_WARMUP:
    warmupTick();
    break;
  case PHASE_GATE:
    gateTick();
    break;
  case PHASE_GAP:
    gapTick();
    break;
  case PHASE_HALTED:
    haltedTick();
    break;
  }
}
