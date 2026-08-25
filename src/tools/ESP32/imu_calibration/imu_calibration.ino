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
// DIAGNOSTIC: long-run bench IMU calibration
//
// Measures per-axis zero-point offsets for the external MPU-6050 in the
// sensor's RAW axis frame (before any axis remap), logs every session to
// LittleFS, and aggregates them into six paste-ready #define lines for
// config.h. Independent of the IMU_AXIS_* remap - the offsets stay valid
// across mounting changes.
//
// This is the ESP32 counterpart of tools/nRF52840/imu_calibration and follows
// the same approach deliberately: plateau-detected warmup, a stability gate
// before every session, repeating logged sessions, and a robust aggregation
// over the whole run. Read that sketch's header for the reasoning behind the
// shape; the differences that are real on this hardware are called out below.
//
// Units match what g_imu.cpp / config.h use on THIS variant:
//   * accel:  m/s^2   (Adafruit MPU6050 native, ->milli-g downstream)
//   * gyro:   rad/s   (Adafruit MPU6050 native, ->centi-deg/s downstream)
// This is the main difference from the nRF52840 tool, whose LSM6DS3 reports g
// and deg/s directly. Every threshold below is therefore written in native
// units with the familiar g / dps equivalent in a comment, the same way
// config.h writes IMU_ACCEL_TRANSIENT_THRESHOLD_MPS2.
//
// Accel Z is calibrated with the sensor's +Z face pointing UP on a level
// surface, so gravity (9.80665 m/s^2) is subtracted from the average before
// recording.
//
// NOT "unattended" in the nRF sense - this board is USB-powered, so USB is
// necessarily present the whole time. Flash logging still earns its place for
// different reasons: it survives a Serial Monitor disconnect and the board's
// auto-reset on monitor attach, and it lets a multi-hour run be aggregated
// afterwards rather than requiring someone to watch the output scroll past.
//
// Flow:
//   1. Bring up the IMU, the GNSS UART, and BLE advertising (see "thermal
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
// THERMAL REALISM, and how it differs here. IMU offsets drift with die
// temperature, so calibrating a cold board characterizes a thermal state the
// firmware never operates in. BLE advertising is brought up for the same
// reason as on the nRF tool. GNSS is the part that does NOT carry over: this
// variant has no power gate (no GNSS_EN_PIN - the receiver runs off the USB
// rail whenever the board is powered), so unlike the nRF build there is no
// rail to switch on and the receiver's heat is present either way. The UART is
// opened and drained anyway, because that reproduces production's pin state
// and UART ISR activity - not because it adds the receiver's power draw, which
// this sketch has no control over.
//
// LIMITATION - accel X/Y is a levelness measurement as much as a bias one.
// One degree of tilt leaks ~0.17 m/s^2 (~17 mg) of gravity into the horizontal
// axes, which is several times larger than the offsets being measured.
// Averaging cannot remove it: a consistently tilted bench produces a
// consistent, wrong answer. The levelness gate below only catches gross errors
// (device on a book, cable propping up one corner), it does not certify
// accuracy. Gyro offsets are unaffected by this and are trustworthy at any
// pose. Truly separating accel X/Y bias from gravity leakage requires a
// multi-position (tumble) calibration, which is not hands-off and is out of
// scope for this tool. Rather than block on tilt, aggregation compares the
// run's median tilt against ACCEL_TRUST_TILT_DEG and comments the accel
// defines out when the pose was too far off level, while still emitting the
// gyro defines - a sloped bench costs you the numbers it compromised, not
// the whole run.
//
// Requires: the ESP32 build's MPU-6050 wired as in config.h; level bench
// surface; USB (which this board needs for power regardless).
// ============================================================================

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <LittleFS.h>
#include <Wire.h>

// ----------------------------------------------------------------------------
// Keep IMU settings in sync with config.h so calibration matches operating
// conditions (offsets drift slightly with range/bandwidth).
// ----------------------------------------------------------------------------
#define IMU_ACCEL_RANGE_G MPU6050_RANGE_4_G
#define IMU_GYRO_RANGE_DPS MPU6050_RANGE_500_DEG
#define IMU_FILTER_BANDWIDTH_HZ MPU6050_BAND_21_HZ
#define IMU_I2C_CLOCK_HZ 400000
#define IMU_SAMPLE_INTERVAL_MS 10 // 100 Hz pacing between samples

// Standard gravity, subtracted from the accel Z average. Matches the constant
// g_imu.cpp uses for its milli-g conversion.
static const float GRAVITY_MPS2 = 9.80665f;

// ----------------------------------------------------------------------------
// Thermal-load peripherals. Mirrors config.h's GNSS_RX_PIN / GNSS_TX_PIN /
// GNSS_BAUD and BLE_TX_POWER. The advertised name deliberately does NOT mirror
// config.h's "RaceBox Mini <id>" - a bench unit sitting here for hours should
// not look like a real device to any phone app that happens to scan.
//
// There is no GNSS_EN_PIN on this variant: the receiver is powered from the
// board's rail with no firmware gate, so all this does is open and drain the
// UART. See the header's thermal-realism note.
// ----------------------------------------------------------------------------
#define GNSS_RX_PIN 16
#define GNSS_TX_PIN 17
#define GNSS_BAUD 115200
#define BLE_ADV_NAME "Gnimu cal"
// ESP_PWR_LVL_N12 (-12 dBm) is this part's floor and is what config.h ships.
// The nRF tool drops to -16 to be a quieter bench neighbour; that level does
// not exist here, so -12 is as quiet as this radio goes.
#define BLE_TX_POWER_ADV ESP_PWR_LVL_N12

static HardwareSerial gnssSerial(2);

// ----------------------------------------------------------------------------
// Warmup: run until the die temperature plateaus, not for a fixed span. The
// original version of this sketch warmed up for a flat 5 minutes; "how long to
// thermal steady state" is a property of the hardware rather than a number
// worth guessing, and the MPU-6050's own temperature sensor can just be asked.
// A max-min check over a trailing window, guarded by a minimum sample count so
// it can't pass on two coincidentally-close reads. The floor keeps a
// fast-plateauing board from starting too early; the ceiling keeps a slow one
// (or a wandering ambient) from stalling forever.
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
// non-zero gyro rate by definition (config.h currently carries a -0.080 rad/s
// gyro X offset) and an absolute-magnitude test would reject it.
//
// These are deliberately loose starting points - roughly 5x the sensor's own
// noise floor over this window - so the gate rejects real disturbance rather
// than normal quiet. Every session logs its measured peak-to-peak and tilt,
// so tighten these from real data rather than from theory.
//
// TILT DOES NOT BLOCK. Only motion does. Tilt is measured, printed and
// recorded in every row, but it never stops a session, because:
//   * waiting cannot fix tilt - only physically moving the device can;
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
static const float GATE_GYRO_PP_LIMIT_RADPS = 0.052f;  // ~3.0 dps
static const float GATE_ACCEL_PP_LIMIT_MPS2 = 0.196f;  // ~0.02 g
static const float GATE_TILT_LIMIT_DEG = 1.0f;
static const bool GATE_TILT_BLOCKS = false;
static const unsigned long GATE_RETRY_MS = 10UL * 1000UL;
// While the gate is failing, serial reports every retry (free, and the point
// is to watch it converge while you level or steady the device) but the log
// gets a note at most this often, so a long wait can't eat the budget.
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
static const float SESSION_GYRO_PP_LIMIT_RADPS = 0.070f;   // ~4.0 dps
static const float SESSION_ACCEL_PP_LIMIT_MPS2 = 0.294f;   // ~0.03 g

// ----------------------------------------------------------------------------
// Storage. Unlike the nRF52840 tool - whose 28 KiB internal-flash region is
// the binding constraint on run length - LittleFS here has megabytes to spare
// and the board is mains-powered, so neither flash nor battery ends a run.
// MAX_RECORDS is what actually binds, since aggregation only reads that many
// rows of a run.
//
// The budget below is therefore set to hold just under MAX_RECORDS rows
// (~110 bytes each) ON PURPOSE, preserving the nRF tool's useful property that
// you can never log more than you can aggregate. Raising one without the other
// silently produces rows that `a` will ignore.
//
// At one row per (100 s session + 60 s gap), this covers roughly 13 hours.
// Raise SESSION_GAP_MS for longer wall-clock coverage at the same row count.
// ----------------------------------------------------------------------------
static const char *LOG_FILE_PATH = "/imu_cal.csv";
static const uint32_t LOG_BUDGET_BYTES = 32768; // 32 KiB ~= 298 rows

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
// paste-ready block. One degree already leaks ~0.17 m/s^2 into the horizontal
// axes, which is several times the bias being measured.
static const float ACCEL_TRUST_TILT_DEG = 1.0f;

static Adafruit_MPU6050 mpu;

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
static float latestTempC = 0.0f;

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
// Sampling helpers.
//
// One getEvent() fetches all six axes plus temperature as a single 14-byte
// burst, so unlike the nRF tool (which reads six separate registers) there is
// nothing to batch here - Adafruit already does it.
// ----------------------------------------------------------------------------
static float lastSampleTempC = 0.0f;

static void readSample(float *accel3, float *gyro3) {
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  accel3[0] = a.acceleration.x;
  accel3[1] = a.acceleration.y;
  accel3[2] = a.acceleration.z;
  gyro3[0] = g.gyro.x;
  gyro3[1] = g.gyro.y;
  gyro3[2] = g.gyro.z;
  lastSampleTempC = t.temperature;
}

// Wait until the next sample tick. Uses delay(1) rather than the nRF tool's
// busy spin: on the ESP32 a tight multi-second spin starves the idle task and
// can trip the task watchdog. At 10 ms pacing the 1 ms granularity is far
// below the sensor's own timing jitter.
static void waitForTick(unsigned long *nextSampleMs) {
  while ((int32_t)(millis() - *nextSampleMs) < 0) {
    delay(1);
  }
  *nextSampleMs += IMU_SAMPLE_INTERVAL_MS;
}

// ----------------------------------------------------------------------------
// Peripheral bring-up.
// ----------------------------------------------------------------------------
static void imuBringUp() {
  if (!mpu.begin()) {
    // Reprint continuously: on this board the first line is easily missed in
    // the gap before the monitor re-attaches after reset.
    while (1) {
      Serial.println("❌ Failed to find MPU6050 chip - halting");
      delay(1000);
    }
  }

  mpu.setAccelerometerRange(IMU_ACCEL_RANGE_G);
  mpu.setGyroRange(IMU_GYRO_RANGE_DPS);
  mpu.setFilterBandwidth(IMU_FILTER_BANDWIDTH_HZ);

  // MUST come after begin(): that call brings the bus up and would overwrite
  // anything set earlier. Same ordering constraint g_imu.cpp documents.
  Wire.setClock(IMU_I2C_CLOCK_HZ);
}

// Open the GNSS UART purely to reproduce production's pin and ISR activity.
// No UBX configuration, no fix required, no sky view needed. If the module is
// still at its factory baud the bytes arrive as framing garbage; that's fine,
// they're discarded either way. Note this does NOT gate the receiver's power -
// see the header.
static void gnssWarmLoadOn() {
  gnssSerial.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN);
}

static void gnssDrain() {
  while (gnssSerial.available()) {
    (void)gnssSerial.read();
  }
}

// BLE advertising, purely as load. No services, no connection handling -
// nothing here needs to be talked to.
static void bleWarmLoadOn() {
  BLEDevice::init(BLE_ADV_NAME);
  BLEDevice::setPower(BLE_TX_POWER_ADV);
  // A server is created but left bare: on this stack advertising is started
  // through the server's advertising object, and creating one is the
  // documented path even when no service is registered.
  BLEDevice::createServer();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->setMinInterval(32); // units of 0.625 ms
  adv->setMaxInterval(244);
  adv->start();
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
      Serial.printf("\n⚠️  Log budget full (%lu bytes) - no longer "
                    "appending. Sessions continue; aggregate or erase.\n",
                    (unsigned long)LOG_BUDGET_BYTES);
    }
    return;
  }

  // FILE_APPEND, not FILE_WRITE: on this core FILE_WRITE truncates, which
  // would silently discard the run so far on every single append.
  File f = LittleFS.open(LOG_FILE_PATH, FILE_APPEND);
  if (!f) {
    Serial.println("❌ Failed to open log file for append.");
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

  File f = LittleFS.open(LOG_FILE_PATH, FILE_READ);
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
  logAppend("# run,session,elapsed_s,temp_c,ax_mps2,ay_mps2,az_mps2,gx_radps,"
            "gy_radps,gz_radps,gyro_pp_radps,accel_pp_mps2,tilt_deg,flag\n");
}

static void logBootMarker() {
  char line[64];
  snprintf(line, sizeof(line), "# --- RUN %u boot ---\n", (unsigned)runNumber);
  logAppend(line);
}

static void logSession() {
  char line[176];
  snprintf(line, sizeof(line),
           "%u,%u,%lu,%.2f,%+.6f,%+.6f,%+.6f,%+.6f,%+.6f,%+.6f,%.4f,%.4f,"
           "%.2f,%s\n",
           (unsigned)runNumber, (unsigned)sessionNumber,
           (unsigned long)(millis() / 1000UL), cur.tempC, cur.ax, cur.ay,
           cur.az, cur.gx, cur.gy, cur.gz, cur.gyroPp, cur.accelPp, cur.tiltDeg,
           (cur.gyroPp > SESSION_GYRO_PP_LIMIT_RADPS ||
            cur.accelPp > SESSION_ACCEL_PP_LIMIT_MPS2)
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
  File f = LittleFS.open(LOG_FILE_PATH, FILE_READ);
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
  LittleFS.remove(LOG_FILE_PATH);
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
  if (!Serial.available()) {
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
// One glitched read produces a single absurd value that blows up max-min
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
    waitForTick(&nextSampleMs);

    // One flat array so the stats loop below can treat all six axes
    // uniformly. Relies on AX..AZ being 0..2 and GX..GZ being 3..5, i.e. the
    // accel and gyro triples sitting contiguously in that order.
    float s[6];
    readSample(&s[AX], &s[GX]);

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

  const bool motionOk = gateGyroPp <= GATE_GYRO_PP_LIMIT_RADPS &&
                        gateAccelPp <= GATE_ACCEL_PP_LIMIT_MPS2;
  const bool tiltOk = !GATE_TILT_BLOCKS || gateTiltDeg <= GATE_TILT_LIMIT_DEG;
  return motionOk && tiltOk;
}

// Full per-axis picture, printed on every failed gate so a wait is something
// you can act on: which axis is moving, and whether p-p and sd agree.
static void printGateDetail() {
  Serial.printf("   accel mean %+.4f %+.4f %+.4f m/s^2  p-p %.4f %.4f %.4f   "
                "sd %.4f %.4f %.4f\n",
                gateMean[AX], gateMean[AY], gateMean[AZ], gatePp[AX],
                gatePp[AY], gatePp[AZ], gateSd[AX], gateSd[AY], gateSd[AZ]);
  Serial.printf("   gyro  mean %+.4f %+.4f %+.4f rad/s  p-p %.4f %.4f %.4f   "
                "sd %.4f %.4f %.4f\n",
                gateMean[GX], gateMean[GY], gateMean[GZ], gatePp[GX],
                gatePp[GY], gatePp[GZ], gateSd[GX], gateSd[GY], gateSd[GZ]);
  Serial.printf("   limits: gyro p-p %.4f, accel p-p %.4f   tilt %.2f deg "
                "(%s)\n",
                GATE_GYRO_PP_LIMIT_RADPS, GATE_ACCEL_PP_LIMIT_MPS2, gateTiltDeg,
                GATE_TILT_BLOCKS ? "blocking" : "recorded, not blocking");
  // A high p-p next to a low sd means a handful of wild samples, not a
  // moving bench - averaging would have absorbed them, so the gate is the
  // only place they are visible at all.
  if (gateGyroPp > GATE_GYRO_PP_LIMIT_RADPS &&
      gateGyroPp > 20.0f * gateSd[GX] && gateGyroPp > 20.0f * gateSd[GY] &&
      gateGyroPp > 20.0f * gateSd[GZ]) {
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
  Serial.printf("\n=== Run %u session %u: sampling %d @ %d ms ===\n",
                (unsigned)runNumber, (unsigned)sessionNumber, NUM_SAMPLES,
                IMU_SAMPLE_INTERVAL_MS);

  double sumA[3] = {0, 0, 0};
  double sumG[3] = {0, 0, 0};
  float aMin[3] = {1e9f, 1e9f, 1e9f}, aMax[3] = {-1e9f, -1e9f, -1e9f};
  float gMin[3] = {1e9f, 1e9f, 1e9f}, gMax[3] = {-1e9f, -1e9f, -1e9f};

  unsigned long nextSampleMs = millis();
  for (int i = 0; i < NUM_SAMPLES; i++) {
    if (checkHaltRequest()) {
      Serial.printf("\n(session aborted at %d/%d samples - partial average "
                    "discarded)\n",
                    i, NUM_SAMPLES);
      return false;
    }
    gnssDrain();
    waitForTick(&nextSampleMs);

    float a[3], g[3];
    readSample(a, g);

    for (int k = 0; k < 3; k++) {
      sumA[k] += a[k];
      sumG[k] += g[k];
      if (a[k] < aMin[k])
        aMin[k] = a[k];
      if (a[k] > aMax[k])
        aMax[k] = a[k];
      if (g[k] < gMin[k])
        gMin[k] = g[k];
      if (g[k] > gMax[k])
        gMax[k] = g[k];
    }

    if (i > 0 && i % 1000 == 0) {
      Serial.printf(" [%d/%d]", i, NUM_SAMPLES);
    }
  }

  const float mx = (float)(sumA[0] / NUM_SAMPLES);
  const float my = (float)(sumA[1] / NUM_SAMPLES);
  const float mz = (float)(sumA[2] / NUM_SAMPLES);

  // Averages. Accel Z has gravity subtracted (device is +Z-up on a level
  // surface, so its true value at rest is +9.80665 m/s^2).
  cur.ax = mx;
  cur.ay = my;
  cur.az = mz - GRAVITY_MPS2;
  cur.gx = (float)(sumG[0] / NUM_SAMPLES);
  cur.gy = (float)(sumG[1] / NUM_SAMPLES);
  cur.gz = (float)(sumG[2] / NUM_SAMPLES);

  float gpp = 0.0f, app = 0.0f;
  for (int k = 0; k < 3; k++) {
    if ((gMax[k] - gMin[k]) > gpp)
      gpp = gMax[k] - gMin[k];
    if ((aMax[k] - aMin[k]) > app)
      app = aMax[k] - aMin[k];
  }
  cur.gyroPp = gpp;
  cur.accelPp = app;
  // Tilt from the session's own 10000-sample mean - a better estimate than
  // the gate's 500-sample one, and the number worth recording.
  cur.tiltDeg = degrees(atan2f(sqrtf(mx * mx + my * my), mz));
  cur.tempC = lastSampleTempC;

  return true;
}

static void printSession() {
  Serial.printf("\naccel %+.6f %+.6f %+.6f m/s^2   gyro %+.6f %+.6f %+.6f "
                "rad/s\n",
                cur.ax, cur.ay, cur.az, cur.gx, cur.gy, cur.gz);
  Serial.printf("temp %.2f C   p-p gyro %.4f rad/s / accel %.4f m/s^2   tilt "
                "%.2f deg   %s\n",
                cur.tempC, cur.gyroPp, cur.accelPp, cur.tiltDeg,
                (cur.gyroPp > SESSION_GYRO_PP_LIMIT_RADPS ||
                 cur.accelPp > SESSION_ACCEL_PP_LIMIT_MPS2)
                    ? "SUSPECT"
                    : "OK");
  if (havePrev) {
    Serial.printf("delta vs previous:    accel %+.6f %+.6f %+.6f m/s^2\n",
                  cur.ax - prev.ax, cur.ay - prev.ay, cur.az - prev.az);
    Serial.printf("                      gyro  %+.6f %+.6f %+.6f rad/s\n",
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

  File f = LittleFS.open(LOG_FILE_PATH, FILE_READ);
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
  agg.ax = summarizeAxis("accel X", "m/s^2", used);
  for (int i = 0; i < used; i++)
    work[i] = records[i].ay;
  agg.ay = summarizeAxis("accel Y", "m/s^2", used);
  for (int i = 0; i < used; i++)
    work[i] = records[i].az;
  agg.az = summarizeAxis("accel Z", "m/s^2", used);
  for (int i = 0; i < used; i++)
    work[i] = records[i].gx;
  agg.gx = summarizeAxis("gyro X", "rad/s", used);
  for (int i = 0; i < used; i++)
    work[i] = records[i].gy;
  agg.gy = summarizeAxis("gyro Y", "rad/s", used);
  for (int i = 0; i < used; i++)
    work[i] = records[i].gz;
  agg.gz = summarizeAxis("gyro Z", "rad/s", used);

  Serial.println("\nPaste these lines into config.h's IMU-offsets section:");
  Serial.println("-------------------------------------------");
  // The gyro block is emitted plainly whatever the pose was - tilt cannot
  // affect a rate measurement, so these stay good even on a sloped bench.
  Serial.printf("#define IMU_GYRO_OFFSET_X_RADPS %+.6ff\n", agg.gx);
  Serial.printf("#define IMU_GYRO_OFFSET_Y_RADPS %+.6ff\n", agg.gy);
  Serial.printf("#define IMU_GYRO_OFFSET_Z_RADPS %+.6ff\n", agg.gz);
  if (accelTrusted) {
    Serial.printf("#define IMU_ACCEL_OFFSET_X_MPS2 %+.6ff\n", agg.ax);
    Serial.printf("#define IMU_ACCEL_OFFSET_Y_MPS2 %+.6ff\n", agg.ay);
    Serial.printf("#define IMU_ACCEL_OFFSET_Z_MPS2 %+.6ff\n", agg.az);
  } else {
    // Commented out deliberately: at this tilt these are mostly a
    // measurement of the bench, and pasting them would bake that slope into
    // the firmware as if it were sensor bias.
    Serial.printf("// DO NOT PASTE - measured at %.2f deg tilt, so these are\n",
                  medianTilt);
    Serial.printf("// mostly leaked gravity (~0.17 m/s^2/deg), not bias:\n");
    Serial.printf("// #define IMU_ACCEL_OFFSET_X_MPS2 %+.6ff\n", agg.ax);
    Serial.printf("// #define IMU_ACCEL_OFFSET_Y_MPS2 %+.6ff\n", agg.ay);
    Serial.printf("// #define IMU_ACCEL_OFFSET_Z_MPS2 %+.6ff\n", agg.az);
  }
  Serial.println("-------------------------------------------");
  Serial.println("Values above are per-axis MEDIANS. Sanity check: |accel| "
                 "offsets are typically <1.0 m/s^2, gyro <0.09 rad/s.");
  if (!accelTrusted) {
    Serial.printf("Accel withheld: median tilt %.2f deg exceeds %.2f deg. "
                  "Re-level the device and rerun for accel; the gyro values "
                  "above are unaffected and ready to use.\n",
                  medianTilt, ACCEL_TRUST_TILT_DEG);
  } else {
    Serial.println("Note: accel X/Y also absorb bench tilt (~0.17 m/s^2 per "
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
    float a[3], g[3];
    readSample(a, g); // refreshes lastSampleTempC
    latestTempC = lastSampleTempC;
    tempHistPush(nowMs, latestTempC);
  }

  if (nowMs - lastLogMs >= 30000UL) {
    lastLogMs = nowMs;
    Serial.printf("warmup %lus  temp %.2f C\n", elapsed / 1000UL, latestTempC);
  }

  const bool floorReached = elapsed >= WARMUP_MIN_MS;
  const bool ceilingReached = elapsed >= WARMUP_MAX_MS;
  const bool plateaued = floorReached && tempPlateaued(nowMs);
  if (plateaued || ceilingReached) {
    // Which of the two ended warmup is worth recording: repeatedly hitting
    // the ceiling means the board never reached steady state, and every
    // session after it is measuring a still-drifting die.
    Serial.printf("\nWarmup complete after %lus (%s).\n", elapsed / 1000UL,
                  plateaued ? "die temp plateaued"
                            : "ceiling reached, temp still moving");
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
    Serial.printf("\n⏸  Not stable - attempt %lu, waiting %lus. Retrying "
                  "every %lus; press any key to halt.\n",
                  (unsigned long)gateAttempts,
                  (unsigned long)((millis() - gateWaitStartMs) / 1000UL),
                  GATE_RETRY_MS / 1000UL);
    printGateDetail();
    // The log gets a note far less often than serial does - a long wait must
    // not fill the budget with retry lines.
    const unsigned long nowNoteMs = millis();
    if (gateLastNoteMs == 0 ||
        (nowNoteMs - gateLastNoteMs) >= GATE_NOTE_INTERVAL_MS) {
      gateLastNoteMs = nowNoteMs;
      char note[112];
      snprintf(note, sizeof(note),
               "GATE_WAIT n=%lu pp_gyro=%.4f pp_accel=%.4f tilt=%.2f",
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
    Serial.printf("▶️  Stable again after %lus - resuming.\n",
                  (unsigned long)((millis() - gateWaitStartMs) / 1000UL));
    gateAttempts = 0;
  }

  sessionNumber++;
  if (!performCalibration()) {
    return; // aborted into PHASE_HALTED
  }
  latestTempC = cur.tempC;
  printSession();
  logSession();
  prev = cur;
  havePrev = true;

  gapStartMs = millis();
  phase = PHASE_GAP;
  Serial.printf("Next session in %lus (press any key to halt)...\n",
                SESSION_GAP_MS / 1000UL);
}

static void gapTick() {
  if (checkHaltRequest()) {
    return;
  }
  gnssDrain();
  if (millis() - gapStartMs >= SESSION_GAP_MS) {
    phase = PHASE_GATE;
    return;
  }
  delay(50);
}

static void haltedTick() {
  static bool menuShown = false;
  if (!menuShown) {
    menuShown = true;
    printMenu();
  }

  char c = pendingCmd;
  pendingCmd = 0;
  if (c == 0) {
    if (!Serial.available()) {
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
    ESP.restart();
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
  // Harmless on UART-bridge boards (Serial is always truthy there); gives a
  // native-USB monitor a brief chance to attach.
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
  }

  // true = format on failed mount. Safe here (unlike the main firmware's
  // storage policy): this is a dedicated bench tool, not production code that
  // must never silently destroy data.
  fsReady = LittleFS.begin(true);
  if (fsReady) {
    logScanExisting();
  }

  Serial.println("\n=== Gnimu - long-run IMU calibration ===");
  Serial.printf("Run %u. Log %s: %lu / %lu bytes used.\n", (unsigned)runNumber,
                LOG_FILE_PATH, (unsigned long)logBytesUsed,
                (unsigned long)LOG_BUDGET_BYTES);
  Serial.println("Device must be on a level surface, sensor +Z face UP, "
                 "and untouched from here on.");
  Serial.printf("Warmup %lu-%lu min (until die temp plateaus), then "
                "%d-sample sessions %lus apart, forever.\n",
                WARMUP_MIN_MS / 60000UL, WARMUP_MAX_MS / 60000UL, NUM_SAMPLES,
                SESSION_GAP_MS / 1000UL);
  Serial.println("Press any key at any time to halt and dump/aggregate.");

  imuBringUp();
  // GNSS and BLE are load only - see the header. Brought up after the IMU so
  // an IMU failure halts before anything starts drawing current.
  gnssWarmLoadOn();
  bleWarmLoadOn();
  Serial.println("✅ IMU up; GNSS UART draining and BLE advertising.");

  if (fsReady) {
    logWriteHeaderIfNew();
    logBootMarker();
  } else {
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
