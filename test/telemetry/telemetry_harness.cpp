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
// Telemetry harness - runs the REAL g_telemetry.cpp, unmodified, on the host.
//
// test/harness.cpp proves the encoder turns a TelemetrySample into the right
// 88 bytes. This covers the step before it and the code around it: the copy
// from the receiver's UBX_NAV_PVT_data_t into the sample (buildSample()), the
// connected gate, and the hand-off through the protocol descriptor. It works
// the way test/imu does for the IMU pipeline - the shipping file is compiled
// against fakes of the modules it calls. Nothing in src/ was changed for it.
//
// Four modes, one per process so g_telemetry.cpp's statics start fresh:
//
//   vectors     every golden vector through telemetrySendIfReady(), the
//               emitted frame compared byte for byte (pass/fail)
//   invariants  one frame per epoch while connected, none while not, and the
//               IMU latched once per epoch EITHER WAY (pass/fail)
//   rates       the epoch-to-epoch rate measurement (NEW-3) against a drifting,
//               jittered receiver clock (golden output plus assertions)
//   stats       the 1 Hz serial stats line, through the REAL g_log.h (golden
//               output plus assertions: never clipped, width budget held)
//
// HOW A VECTOR GOES IN: the harness fills the real struct FIELD BY NAME from
// the vector, over a background byte pattern, so buildSample() reading the
// wrong member - or one it should never touch, like headVeh - produces a
// wrong frame. The vectors were decoded from captures of this same code, but a
// mis-copied field still shows: it was applied once when the capture was taken
// and is applied again here, and the two do not cancel.
//
// NOT COVERED: the SparkFun library filling the struct from UART bytes (vendor
// code), and the BLE transport itself.
//
// BUILD & RUN: ./test/run_telemetry_harness.sh   (--save rewrites the goldens)
// ============================================================================

#include "gc1.h" // the vector format and loader, shared with test/harness.cpp

#include "config.h"
#include "g_battery.h"
#include "g_ble.h"
#include "g_gnss.h"
#include "g_imu.h"
#include "g_imu_trim.h"
#include "g_log.h"
#include "g_telemetry.h"

// ----------------------------------------------------------------------------
// Fakes: one per function g_telemetry.cpp calls. If it ever calls something
// new, this stops linking - loud, and a one-line fix here.
// ----------------------------------------------------------------------------

static unsigned long nowMs = 0;
unsigned long millis() { return nowMs; }

// Serial: everything written is kept, and every write is checked for being a
// whole line. g_telemetry.cpp only ever logs whole lines, so a write that does
// not end in '\n' is one LOG_PRINT / LOG_PRINTF clipped at LOG_LINE_MAX.
FakeSerial Serial;
static std::string serialOut;
static int serialClipped = 0;
static size_t serialLongest = 0;
static void serialChunk(const char *data, size_t len) {
  serialOut.append(data, len);
  if (len > serialLongest) {
    serialLongest = len;
  }
  if (len == 0 || data[len - 1] != '\n') {
    serialClipped++;
  }
}
size_t FakeSerial::print(const char *s) {
  serialChunk(s, strlen(s));
  return strlen(s);
}
size_t FakeSerial::println(const char *s) {
  std::string line = std::string(s) + "\n";
  serialChunk(line.data(), line.size());
  return line.size();
}
size_t FakeSerial::write(const uint8_t *data, size_t len) {
  serialChunk((const char *)data, len);
  return len;
}

// GNSS. The struct persists, as the library's does: telemetry keeps a pointer
// to the latest epoch for the stats line.
static UBX_NAV_PVT_data_t pvt;
static bool pvtPending = false;
static bool gnssUp = true;
const UBX_NAV_PVT_data_t *gnssConsumePvt() {
  if (!pvtPending) {
    return nullptr;
  }
  pvtPending = false;
  return &pvt;
}
bool gnssIsUp() { return gnssUp; }

// BLE. A dropped frame is recorded by the transport and reported as not
// accepted, which is what the real one does.
static bool connected = true;
static bool dropNextFrame = false;
static uint32_t droppedFrames = 0, droppedWrites = 0;
static int frames = 0;
static uint8_t frameChannel = 0xFF;
static size_t frameLen = 0;
static uint8_t frame[RACEBOX_PACKET_LEN];
bool bleIsConnected() { return connected; }
bool bleEmitFrame(uint8_t channel, const uint8_t *data, size_t len) {
  if (dropNextFrame) {
    dropNextFrame = false;
    droppedFrames++;
    return false;
  }
  frames++;
  frameChannel = channel;
  frameLen = len;
  memcpy(frame, data, len <= sizeof(frame) ? len : sizeof(frame));
  return true;
}
uint32_t bleDroppedFrames() { return droppedFrames; }
uint32_t bleDroppedWrites() { return droppedWrites; }

// Battery: this variant's REAL BatteryStatus, with whatever the test sets.
static BatteryStatus battery = {};
BatteryStatus batteryGetStatus() { return battery; }

// IMU and trim.
static ImuProtocolUnits imuLatch = {}, imuDisplay = {};
static int latches = 0;
static bool imuUp = true, trimConverged = false;
static float trimTilt = 0.0f;
static uint32_t imuFails = 0;
ImuProtocolUnits imuLatchForEpoch() {
  latches++;
  return imuLatch;
}
ImuProtocolUnits imuReadProtocolUnits() { return imuDisplay; }
bool imuIsUp() { return imuUp; }
uint32_t imuFailedReads() { return imuFails; }
bool imuTrimConverged() { return trimConverged; }
float imuTrimTiltDegrees() { return trimTilt; }

// ----------------------------------------------------------------------------

static bool check(bool ok, const char *what) {
  printf("%s %s\n", ok ? "✅" : "❌", what);
  return ok;
}

// The harness's OWN mapping, written independently of buildSample(): each
// member set by NAME from the matching sample field. The background pattern
// gives every member buildSample() should not read - headVeh, velN/E/D,
// flags2, the other bits sharing a byte with gnssFixOK - a value that would
// show up in the frame if it did.
static void loadPvt(const TelemetrySample &s) {
  memset(&pvt, 0xA5, sizeof(pvt));
  pvt.iTOW = s.iTOW;
  pvt.year = s.year;
  pvt.month = s.month;
  pvt.day = s.day;
  pvt.hour = s.hour;
  pvt.min = s.min;
  pvt.sec = s.sec;
  pvt.valid.bits.validDate = s.validDate;
  pvt.valid.bits.validTime = s.validTime;
  pvt.valid.bits.fullyResolved = s.fullyResolved;
  pvt.valid.bits.validMag = s.validMag;
  pvt.tAcc = s.tAcc;
  pvt.nano = s.nano;
  pvt.fixType = s.fixType;
  pvt.flags.bits.gnssFixOK = s.gnssFixOK;
  pvt.flags.bits.headVehValid = s.headVehValid;
  pvt.numSV = s.numSV;
  pvt.lon = s.lon;
  pvt.lat = s.lat;
  pvt.height = s.height;
  pvt.hMSL = s.hMSL;
  pvt.hAcc = s.hAcc;
  pvt.vAcc = s.vAcc;
  pvt.gSpeed = s.gSpeed;
  pvt.headMot = s.headMot;
  pvt.sAcc = s.sAcc;
  pvt.headAcc = s.headAcc;
  pvt.pDOP = s.pDOP;
}

// --- vectors -----------------------------------------------------------------

static int runVectors(int argc, char **argv) {
  std::vector<Vector> vectors;
  if (!loadVectors(argc, argv, 2, vectors)) {
    return 2;
  }
  telemetryBegin(); // the clock stays at 0, so no stats window closes here
  connected = true;

  int failures = 0, shown = 0;
  for (const Vector &v : vectors) {
    const TelemetrySample &s = v.sample;
    loadPvt(s);
    imuLatch = {s.accelX, s.accelY, s.accelZ, s.gyroX, s.gyroY, s.gyroZ};
    battery.percent = s.batteryPercent;
    battery.charging = s.batteryCharging;
    pvtPending = true;
    frames = 0;
    frameLen = 0;
    frameChannel = 0xFF;
    memset(frame, 0, sizeof(frame));
    const int latchesBefore = latches;

    telemetrySendIfReady();

    const char *problem = nullptr;
    size_t diffAt = 0;
    if (latches != latchesBefore + 1) {
      problem = "IMU not latched exactly once for the epoch";
    } else if (frames != 1) {
      problem = "wrong number of frames emitted";
    } else if (frameChannel != TELEMETRY_CHANNEL_PRIMARY) {
      problem = "wrong channel";
    } else if (frameLen != RACEBOX_PACKET_LEN) {
      problem = "wrong frame length";
    } else {
      for (size_t i = 0; i < RACEBOX_PACKET_LEN; i++) {
        if (frame[i] != v.expected[i]) {
          problem = "byte mismatch";
          diffAt = i;
          break;
        }
      }
    }
    if (!problem) {
      continue;
    }
    failures++;
    if (shown++ < 10) {
      printf("\n❌ %s\n", v.source.c_str());
      if (!v.label.empty()) {
        printf("   testing: %s\n", v.label.c_str());
      }
      if (strcmp(problem, "byte mismatch") == 0) {
        printf("   first diff at byte %zu", diffAt);
        if (diffAt >= 6) {
          printf(" (payload offset %zu)", diffAt - 6);
        }
        printf(" - got 0x%02X, expected 0x%02X\n", frame[diffAt],
               v.expected[diffAt]);
      } else {
        printf("   %s (frames=%d channel=%u len=%zu)\n", problem, frames,
               frameChannel, frameLen);
      }
    }
  }
  printf("\n%zu vectors, %d failures\n", vectors.size(), failures);
  if (failures > shown) {
    printf("(%d further failures not shown)\n", failures - shown);
  }
  if (failures == 0) {
    printf("✅ the real telemetry path reproduces every golden vector exactly\n");
  }
  return failures == 0 ? 0 : 1;
}

// --- invariants --------------------------------------------------------------

static int runInvariants() {
  telemetryBegin(); // clock at 0 throughout: no stats window closes
  memset(&pvt, 0, sizeof(pvt));
  bool ok = true;

  connected = false;
  for (int i = 0; i < 50; i++) {
    pvtPending = true;
    telemetrySendIfReady();
  }
  ok &= check(frames == 0, "disconnected: 50 epochs, no frames sent");
  ok &= check(latches == 50,
              "disconnected: IMU still latched once per epoch (50) - no stale "
              "transient peak waiting for the next connection");

  connected = true;
  for (int i = 0; i < 50; i++) {
    pvtPending = true;
    telemetrySendIfReady();
  }
  ok &= check(frames == 50, "connected: exactly one frame per epoch (50)");
  ok &= check(latches == 100, "connected: IMU latched once per epoch (100)");

  for (int i = 0; i < 50; i++) {
    telemetrySendIfReady();
  }
  ok &= check(frames == 50 && latches == 100,
              "no epoch: nothing latched, nothing sent");
  if (ok) {
    printf("✅ all 5 invariants hold\n");
  }
  return ok ? 0 : 1;
}

// --- rates -------------------------------------------------------------------

// Deterministic PRNG - the same stream on every run and every machine.
static uint32_t rng = 12345u;
static unsigned long jitterMs() { // loop() picking an epoch up 0-2 ms late
  rng = rng * 1664525u + 1013904223u;
  return (rng >> 8) % 3;
}

static int runRates() {
  // Receiver epochs are timed on the RECEIVER's clock: nominally 50 ms, 100
  // ppm fast against the MCU's, each picked up by loop() 0-2 ms late. They
  // start phase-aligned to the 1 s stats windows, which is the worst case for
  // the pre-NEW-3 method of dividing by the clock window: an epoch sitting on
  // a boundary lands in whichever window the jitter picks, so a perfect 20 Hz
  // stream reads 19 and 21. The "old" column shows what that method would
  // have printed, and the run asserts it really would have gone wrong - a
  // scenario the old code also passes would prove nothing.
  //
  // Events: one epoch lost (window 31), one BLE frame dropped (window 41), and
  // a 3 s receiver stall (windows 43-45).
  const double kPeriodMs = 50.0 * (1.0 - 100e-6);
  const unsigned long kEnd = 50000;
  const int kLost = 610, kDropped = 805;

  telemetryBegin();
  connected = true;
  memset(&pvt, 0, sizeof(pvt));

  int k = 0;
  unsigned long due = jitterMs();
  int pickedThisWindow = 0;
  float gnss[64] = {}, ble[64] = {};
  int old[64] = {};

  for (nowMs = 1; nowMs <= kEnd; nowMs++) {
    while (due <= nowMs) {
      const bool stalled = due >= 42000 && due < 45000;
      if (!stalled && k != kLost) {
        if (k == kDropped) {
          dropNextFrame = true;
        }
        pvtPending = true;
        pickedThisWindow++;
      }
      k++;
      due = (unsigned long)(k * kPeriodMs) + jitterMs();
    }
    telemetrySendIfReady();
    if (nowMs % 1000 == 0) { // a window closed on this call (begin() was at 0)
      const int w = (int)(nowMs / 1000);
      gnss[w] = telemetryGnssRateHz();
      ble[w] = telemetryBleRateHz();
      old[w] = pickedThisWindow; // epochs per 1 s clock window = old reading
      pickedThisWindow = 0;
      printf("W %2d  gnss=%.1f  ble=%.1f  old=%d\n", w, gnss[w], ble[w],
             old[w]);
    }
  }

  auto steady = [](float r) { return r >= 19.85f && r <= 20.15f; };
  auto oneShort = [](float r) { return r >= 18.85f && r <= 19.15f; };
  bool ok = true, allSteady = true, bleMatches = true, oldWrong = false;
  int notExact = 0;
  const int kWindows = (int)(kEnd / 1000);
  for (int w = 1; w <= kWindows; w++) {
    if (w == 31 || (w >= 43 && w <= 45)) {
      continue;
    }
    allSteady &= steady(gnss[w]);
    notExact += (gnss[w] < 19.95f || gnss[w] >= 20.05f);
    if (w != 41) {
      bleMatches &= (ble[w] == gnss[w]);
    }
    if (w != 1 && w != 46) { // windows that open a span count one edge fewer
      oldWrong |= (old[w] != 20);
    }
  }
  printf("\n");
  ok &= check(oldWrong, "scenario is adversarial: the clock-window method "
                        "would have misread a steady stream");
  ok &= check(allSteady, "steady windows read 20.0 +/- 0.1, including the one "
                         "after the stall (no stretched span)");
  printf("   %d of %d steady windows read 19.9 or 20.1\n", notExact,
         kWindows - 4);
  ok &= check(oneShort(gnss[31]), "lost epoch: that window alone reads 19.0");
  ok &= check(oneShort(ble[41]) && steady(gnss[41]),
              "dropped BLE frame: BLE 19.0 while GNSS stays 20.0");
  ok &= check(bleMatches, "every other window: BLE rate equals GNSS rate");
  ok &= check(gnss[43] == 0.0f && gnss[44] == 0.0f && gnss[45] == 0.0f &&
                  ble[43] == 0.0f && ble[44] == 0.0f && ble[45] == 0.0f,
              "receiver stall: windows with no epoch read 0.0");
  return ok ? 0 : 1;
}

// --- stats -------------------------------------------------------------------

// One stats window of steady 20 Hz epochs (or none), ending in its report.
// Everything the report printed is echoed under a label.
static void window(const char *label, bool epochs = true) {
  serialOut.clear();
  const unsigned long end = nowMs + 1000;
  while (nowMs < end) {
    nowMs++;
    if (epochs && nowMs % 50 == 0) {
      pvtPending = true;
    }
    telemetrySendIfReady();
  }
  printf("## %s\n%s", label, serialOut.c_str());
}

static bool printed(const char *text) {
  return strstr(serialOut.c_str(), text) != nullptr;
}

static int runStats() {
  // All positions here are synthetic: goldens are committed, and real ones
  // never are (see README - privacy).
  Serial.enabled = true;
  telemetryBegin();
  connected = true;
  bool ok = true;

  memset(&pvt, 0, sizeof(pvt));
  pvt.numSV = 14;
  pvt.fixType = 3;
  pvt.tAcc = 25;
  pvt.hAcc = 1200;
  pvt.lat = 12345678;  //  1.2345678
  pvt.lon = -23456789; // -2.3456789
  imuDisplay = {12, -34, 998, 5, -7, 120};
  trimConverged = true;
  trimTilt = 2.34f;
  battery.voltage = 3.87f;
  battery.charging = false;
  window("fix, IMU up, trim locked"); // the first window opens the span
  window("fix, IMU up, trim locked (steady)");

  pvt.tAcc = 0xFFFFFFFFu;
  pvt.hAcc = 0xFFFFFFFFu;
  pvt.fixType = 0;
  pvt.numSV = 0;
  pvt.lat = pvt.lon = 0;
  trimConverged = false;
  trimTilt = 0.0f;
  battery.charging = true;
  window("no fix yet: accuracy sentinels, trim waiting, charging");
  ok &= check(printed("tA: -|hA: -|"), "sentinel accuracies print as '-'");

  pvt.lat = INT32_MAX; // 214.7 degrees: impossible, so '-'
  pvt.lon = INT32_MIN;
  trimTilt = 20.0f; // past IMU_TRIM_MAX_TILT_DEG: refused
  window("coordinates out of range, mount too far off level");
  ok &= check(printed("Lat: -|Lon: -|"), "impossible coordinates print as '-'");
  ok &= check(printed("Trim: 20.0° ❌"), "refused tilt still reported, with ❌");

  pvt.lat = 12345678;
  pvt.lon = -23456789;
  imuUp = false;
  window("IMU down (not fitted, not found, or stopped answering)");
  ok &= check(printed("mG: -|c°/s: -|Trim: -"), "IMU down prints dashes");
  imuUp = true;

  droppedFrames = 3;
  droppedWrites = 2;
  window("BLE dropped 3 frames and 2 inbound writes this window");
  ok &= check(printed("BLE dropped 3 frame(s)") &&
                  printed("BLE dropped 2 inbound write(s)"),
              "drops get their own lines");
  window("no new drops: no drop lines");
  ok &= check(!printed("dropped"), "drop lines only when the count moves");

  // A flaky IMU bus: held reads, never ten in a row, so the IMU stays up and
  // only this line says anything is wrong.
  imuFails = 7;
  window("7 failed IMU reads this window (IMU still up)");
  ok &= check(printed("IMU: 7 failed read(s) this window (7 total)"),
              "failed IMU reads get their own line");
  imuFails = 9;
  window("2 more failed IMU reads: the delta, and the running total");
  ok &= check(printed("IMU: 2 failed read(s) this window (9 total)"),
              "the line reports the window's delta and the total");
  window("no new failed reads: no line");
  ok &= check(!printed("failed read"), "the line only appears when it moves");

  gnssUp = false;
  window("GNSS not responding", false);
  ok &= check(printed("GNSS not responding"), "no receiver says so");
  gnssUp = true;

  // Widest realistic line: runtime at the 32-bit millis() ceiling, every
  // bounded field at its bound, every IMU axis at -32768.
  nowMs = 4294966000UL;
  pvt.numSV = 255;
  pvt.fixType = 255;
  pvt.tAcc = 999999;
  pvt.hAcc = 999999;
  pvt.lat = -900000000;
  pvt.lon = -1800000000;
  imuDisplay = {-32768, -32768, -32768, -32768, -32768, -32768};
  trimTilt = 179.9f;
  battery.voltage = 4.20f;
  battery.charging = true;
  window("clock jumps to the millis() ceiling: the jump closes an empty window");
  serialLongest = 0;
  window("widest line");
  printf("\nlongest line: %zu bytes (ceiling %d)\n", serialLongest,
         LOG_LINE_MAX - 1);
  ok &= check(serialLongest <= LOG_LINE_MAX - 1, "widest line fits the ceiling");
  ok &= check(serialClipped == 0, "no line was ever clipped");
  return ok ? 0 : 1;
}

int main(int argc, char **argv) {
  // Line-buffered even into a file, so the output reads in the order it ran.
  setvbuf(stdout, nullptr, _IOLBF, 0);
  const char *mode = argc > 1 ? argv[1] : "vectors";
  if (!strcmp(mode, "vectors")) return runVectors(argc, argv);
  if (!strcmp(mode, "invariants")) return runInvariants();
  if (!strcmp(mode, "rates")) return runRates();
  if (!strcmp(mode, "stats")) return runStats();
  fprintf(stderr, "unknown mode: %s\n", mode);
  return 2;
}
