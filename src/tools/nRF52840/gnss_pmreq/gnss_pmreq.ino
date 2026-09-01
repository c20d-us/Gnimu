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
// TOOL: UBX-RXM-PMREQ ("backup mode") validation
//
// Validates our assumptions about the planned Phase 2 LIGHT_SLEEP GNSS power
// ladder's middle rung: RXM-PMREQ backup mode (receiver stays powered but
// nearly idle, versus a full EN-rail cut). Three questions:
//
//   1. Does it actually stop UART traffic while "asleep"?
//   2. Does it self-wake at the requested duration (myGNSS.powerOff()), and
//      separately, does UART RX activity wake it early
//      (myGNSS.powerOffWithInterrupt(..., VAL_RXM_PMREQ_WAKEUPSOURCE_UARTRX))?
//      UART RX is the only wake source available to us - no EXTINT pin is
//      wired to the M100 - and it's also the operationally relevant one:
//      LIGHT_SLEEP would wake the GNSS itself by sending it a command the
//      moment BLE connects or the IMU fires a tap/shake wake.
//   3. Does the receiver retain usable ephemeris across the sleep, or does it
//      fall back to a cold-start-style reacquisition?
//
// Question 3 is answered via UBX-NAV-STATUS's `ttff` field (Time To First
// Fix, reported natively by the receiver) plus our own wall-clock timing from
// "wake detected" to "first 3D fix". Ephemeris freshness is close to binary,
// not a smooth dial: a receiver with valid ephemeris (<~4 hr old) gets a HOT
// start (ttff ~1-3 s); once ephemeris is stale/gone it falls back to
// re-decoding ephemeris off the air (~20-30 s), barely faster than a true
// cold start. So the result here should be unambiguous either way.
//
// Requires: XIAO nRF52840 Sense wired GNSS TX -> D7, GNSS RX <- D6, USB for
// serial, clear sky view (a fix is required to measure TTFF), and the
// SparkFun_u-blox_GNSS_v3 library installed. Optional: a multimeter on
// the M100's supply rail to observe current draw during the "asleep" phases,
// and watching the M100's POWER/PPS LEDs.
// ============================================================================

#include <Arduino.h>

// Serial (USB CDC) needs the TinyUSB library linked; this sketch pulls in no
// other library that would include it transitively (same gotcha as
// led_check/gnss_en/gnss_reset).
#include <Adafruit_TinyUSB.h>

#include <SparkFun_u-blox_GNSS_v3.h>

static SFE_UBLOX_GNSS_SERIAL myGNSS;
// Serial1 = D6 (TX) / D7 (RX), fixed on the nRF52 - same wiring as g_gnss.cpp.
static Uart &gnssSerial = Serial1;
static const uint8_t GNSS_TX_PIN = D6; // XIAO TX -> GNSS RX

// Common u-blox baud rates, target rate first for the fastest normal case.
static const uint32_t BAUD_RATES[] = {115200, 9600,   38400,
                                      57600,  230400, 460800};
static const int NUM_BAUD_RATES = sizeof(BAUD_RATES) / sizeof(BAUD_RATES[0]);

// Test tuning.
static const uint32_t TEST1_SLEEP_MS = 10000;        // timed self-wake duration
static const uint32_t BYTE_POLL_WINDOW_MS = 1000;    // granularity while asleep
static const uint32_t FIX_WAIT_TIMEOUT_MS = 60000;   // max wait for a 3D fix
static const uint32_t TEST3_LONG_SLEEP_MS = 60000;   // 1 min - watch the LED
static const uint32_t TEST3_REPORT_EVERY_MS = 15000; // print every 15s

// --- Latest parsed data (via library callbacks) ---
static UBX_NAV_PVT_data_t latestPvt;
static UBX_NAV_STATUS_data_t latestStatus;
static volatile bool newPvt = false;
static volatile bool newStatus = false;

static void pvtCallback(UBX_NAV_PVT_data_t *d) {
  memcpy(&latestPvt, d, sizeof(latestPvt));
  newPvt = true;
}

static void statusCallback(UBX_NAV_STATUS_data_t *d) {
  memcpy(&latestStatus, d, sizeof(latestStatus));
  newStatus = true;
}

static bool connectAtAnyBaud() {
  for (int i = 0; i < NUM_BAUD_RATES; i++) {
    uint32_t baud = BAUD_RATES[i];
    Serial.printf("Trying GNSS at %lu baud...\n", (unsigned long)baud);

    gnssSerial.begin(baud);
    delay(100);

    if (myGNSS.begin(gnssSerial)) {
      Serial.printf("Connected at %lu baud.\n", (unsigned long)baud);
      return true;
    }

    gnssSerial.end();
    delay(100);
  }
  return false;
}

// Count raw bytes arriving on Serial1 over windowMs, bypassing the library
// parser entirely (used only to detect asleep/awake transitions - once we
// need parsed data we stop calling this and switch to checkUblox()).
static unsigned long countRawBytes(uint32_t windowMs) {
  unsigned long count = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < windowMs) {
    while (gnssSerial.available()) {
      gnssSerial.read();
      count++;
    }
  }
  return count;
}

// Poll in BYTE_POLL_WINDOW_MS windows, printing a running count, until either
// traffic resumes (returns the elapsed ms since call) or maxWaitMs elapses
// (returns maxWaitMs).
static unsigned long waitForTrafficResume(uint32_t maxWaitMs) {
  unsigned long waited = 0;
  while (waited < maxWaitMs) {
    unsigned long bytes = countRawBytes(BYTE_POLL_WINDOW_MS);
    waited += BYTE_POLL_WINDOW_MS;
    Serial.printf("  [+%lus] %lu bytes\n", waited / 1000, bytes);
    if (bytes > 0) {
      return waited;
    }
  }
  return waited;
}

// Like waitForTrafficResume(), but only prints a running "still quiet" line
// every reportIntervalMs instead of every BYTE_POLL_WINDOW_MS - for long
// observation windows where per-second output would be unreadable spam.
static unsigned long waitForTrafficResumeCoarse(uint32_t maxWaitMs,
                                                uint32_t reportIntervalMs) {
  unsigned long waited = 0;
  unsigned long sinceReport = 0;
  while (waited < maxWaitMs) {
    unsigned long bytes = countRawBytes(BYTE_POLL_WINDOW_MS);
    waited += BYTE_POLL_WINDOW_MS;
    sinceReport += BYTE_POLL_WINDOW_MS;
    if (bytes > 0) {
      Serial.printf("  [+%lus] %lu bytes - traffic resumed on its own!\n",
                    waited / 1000, bytes);
      return waited;
    }
    if (sinceReport >= reportIntervalMs) {
      Serial.printf("  [+%lus] still quiet\n", waited / 1000);
      Serial.flush();
      sinceReport = 0;
    }
  }
  return waited;
}

// After wake, drain any partial garbage, register callbacks fresh, and poll
// with checkUblox()/checkCallbacks() (the library parser) until a 3D fix
// arrives or we time out. Prints ttff/fixType/numSV/hAcc/pDOP plus our own
// wall-clock-since-wake measurement, then a verdict.
static void measureFixAfterWake() {
  while (gnssSerial.available())
    gnssSerial.read();

  newPvt = false;
  newStatus = false;
  latestStatus.ttff = 0;

  unsigned long t0 = millis();
  bool got3dFix = false;

  while (millis() - t0 < FIX_WAIT_TIMEOUT_MS) {
    myGNSS.checkUblox();
    myGNSS.checkCallbacks();

    if (newPvt) {
      newPvt = false;
      Serial.printf("  [+%lums] fixType=%d numSV=%d hAcc=%lumm pDOP=%.2f\n",
                    millis() - t0, latestPvt.fixType, latestPvt.numSV,
                    (unsigned long)latestPvt.hAcc, latestPvt.pDOP * 0.01f);
      if (latestPvt.fixType == 3 && !got3dFix) {
        got3dFix = true;
        Serial.printf("  -> 3D fix acquired %lums after wake.\n",
                      millis() - t0);
      }
    }

    if (newStatus) {
      newStatus = false;
      Serial.printf("  [+%lums] NAV-STATUS gpsFix=%d ttff=%lums msss=%lums\n",
                    millis() - t0, latestStatus.gpsFix,
                    (unsigned long)latestStatus.ttff,
                    (unsigned long)latestStatus.msss);
    }

    if (got3dFix)
      break;
  }

  if (!got3dFix) {
    Serial.println("  No 3D fix within timeout - inconclusive (check sky "
                   "view) or a genuine cold-start reacquisition in "
                   "progress beyond this test's window.");
    return;
  }

  // Ephemeris freshness is close to binary: HOT (valid ephemeris) resolves
  // in a few seconds; anything else falls back to re-decoding ephemeris off
  // the air (~20-30 s). Judge on our own wall-clock-since-wake measurement,
  // since it's not yet confirmed whether the receiver's own ttff/msss
  // counters reset across a PMREQ wake the same way they would across a
  // true reset - that's part of what this test is checking.
  unsigned long sinceWakeMs = millis() - t0;
  if (sinceWakeMs < 8000) {
    Serial.printf("VERDICT: fix in %lums after wake -> ephemeris was "
                  "RETAINED (hot start).\n",
                  sinceWakeMs);
  } else {
    Serial.printf("VERDICT: fix took %lums after wake -> ephemeris was "
                  "LOST/stale (cold/warm reacquisition).\n",
                  sinceWakeMs);
  }
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
  }

  Serial.println("\n=== UBX-RXM-PMREQ (backup mode) validation ===\n");

  if (!connectAtAnyBaud()) {
    Serial.println("FAILED: u-blox GNSS not detected at any standard baud "
                   "rate. Check your wiring.");
    while (1)
      delay(100);
  }

  delay(100);
  bool acked = false;

  // Minimal config: UBX only, PVT + STATUS via callback. VAL_LAYER_RAM only -
  // this is a throwaway diagnostic, nothing here should persist.
  myGNSS.setUART1Output(COM_TYPE_UBX, VAL_LAYER_RAM);
  myGNSS.setAutoPVTcallbackPtr(&pvtCallback, VAL_LAYER_RAM);
  myGNSS.setAutoNAVSTATUScallbackPtr(&statusCallback, VAL_LAYER_RAM);

  // --- Baseline: confirm the receiver is chatting normally ---
  Serial.println("\n--- Baseline: confirming normal traffic ---");
  unsigned long baseline = countRawBytes(3000);
  Serial.printf("  %lu bytes in 3s -> %s\n", baseline,
                baseline ? "alive" : "SILENT (unexpected before any sleep)");

  // --- Test 1: timed self-wake (myGNSS.powerOff) ---
  // Serial.println("\n--- Test 1: powerOff() timed self-wake ---");
  // Serial.printf("Issuing powerOff(%lu ms)...\n", (unsigned
  // long)TEST1_SLEEP_MS); acked = myGNSS.powerOff(TEST1_SLEEP_MS);
  // Serial.printf("  command %s\n", acked ? "acknowledged" : "NOT
  // acknowledged"); Serial.println("  watching for traffic to stop, then resume
  // near the "
  //                "requested duration:");
  // unsigned long resumedAfter = waitForTrafficResume(TEST1_SLEEP_MS + 5000);
  // Serial.printf("  traffic resumed after ~%lums (requested %lums)\n",
  //               resumedAfter, (unsigned long)TEST1_SLEEP_MS);
  // measureFixAfterWake();

  // delay(TEST1_SLEEP_MS);

  // --- Test 2: powerOffWithInterrupt(), finite duration, no wake nudge ---
  // Isolating one variable at a time: Test 2 previously ran with an infinite
  // duration and two different flag combos (force=true, then force=false) -
  // both produced an identical result (fully-dark PPS LED, never woke, even
  // to a deliberate wake nudge), ruling out `force` as the cause. The
  // remaining candidate is duration itself: a receiver told to sleep with no
  // timer backstop may commit to a genuinely deeper power state than one
  // that knows it has to wake itself on schedule, independent of message
  // format. This run uses a FINITE duration (matching Test 1's 10 s) with
  // the same new-format message and UARTRX wake source, and deliberately
  // sends no wake nudge at all - a nudge here would be ambiguous, since any
  // resumed traffic could be credited to either the nudge or the duration
  // timer expiring around the same time. Passive self-wake observation only,
  // same pattern as Test 1: if the LED goes solid and it self-wakes near 10s
  // like Test 1 did, the trigger was duration, not message format.
  // Serial.println("\n--- Test 2: powerOffWithInterrupt(), finite duration
  // ---"); Serial.printf("Issuing powerOffWithInterrupt(%lu ms, UARTRX wake "
  //               "source)...\n",
  //               (unsigned long)TEST1_SLEEP_MS);
  // acked = myGNSS.powerOffWithInterrupt(
  //     TEST1_SLEEP_MS, VAL_RXM_PMREQ_WAKEUPSOURCE_UARTRX, false);
  // Serial.printf("  command %s\n", acked ? "acknowledged" : "NOT
  // acknowledged"); Serial.println("  watching for traffic to stop, then resume
  // near the "
  //                "requested duration:");
  // unsigned long resumedAfter2 = waitForTrafficResume(TEST1_SLEEP_MS + 5000);
  // Serial.printf("  traffic resumed after ~%lums (requested %lums)\n",
  //               resumedAfter2, (unsigned long)TEST1_SLEEP_MS);
  // measureFixAfterWake();

  delay(TEST1_SLEEP_MS);

  // --- Test 3: infinite duration, manual GPIO pulse wake on the UART line ---
  // The SparkFun library's own reference example (Example22_PowerOff.ino)
  // wakes the module by manually pulsing a GPIO wired to the module's
  // separate EXTINT0 pin - LOW, delay, HIGH, delay, LOW - not by sending
  // framed UART data. We have no EXTINT pin wired, only the UART lines, but
  // it's worth testing whether the UARTRX wake source responds to that same
  // kind of deliberate slow level change rather than real baud-rate framing
  // (our two earlier attempts - a framed poll, and a plain peripheral
  // restart - did not wake it). Duration is infinite here (matching the
  // earlier UARTRX tests), so any resumed traffic can only be credited to
  // this pulse, not a timer.
  Serial.println("\n--- Test 3: infinite duration, manual pulse wake ---");
  Serial.println("Issuing powerOffWithInterrupt(0, UARTRX wake "
                 "source)...");
  acked = myGNSS.powerOffWithInterrupt(0, VAL_RXM_PMREQ_WAKEUPSOURCE_UARTRX);
  Serial.printf("  command %s\n", acked ? "acknowledged" : "NOT acknowledged");

  Serial.printf("  watching for %lus - keep an eye on the PPS LED...\n",
                (unsigned long)(TEST3_LONG_SLEEP_MS / 1000));
  unsigned long quietFor =
      waitForTrafficResumeCoarse(TEST3_LONG_SLEEP_MS, TEST3_REPORT_EVERY_MS);
  if (quietFor < TEST3_LONG_SLEEP_MS) {
    Serial.println("  traffic resumed on its own before the pulse - "
                   "unexpected, see above.");
  } else {
    Serial.printf("  stayed quiet the full %lus.\n",
                  (unsigned long)(TEST3_LONG_SLEEP_MS / 1000));
  }

  int pin_pulse = 10;

  Serial.printf("  releasing UART, pulsing D6 (GNSS RX) LOW -> HIGH -> "
                "LOW (%ums each)...\n",
                pin_pulse);
  gnssSerial.end();
  pinMode(GNSS_TX_PIN, OUTPUT);
  digitalWrite(GNSS_TX_PIN, LOW);
  delay(pin_pulse);
  digitalWrite(GNSS_TX_PIN, HIGH);
  delay(pin_pulse);
  digitalWrite(GNSS_TX_PIN, LOW);
  delay(pin_pulse);
  gnssSerial.begin(115200); // resumes UART duty on D6/D7
  Serial.flush();

  unsigned long resumedAfter3 = waitForTrafficResume(15000);
  Serial.printf("  traffic resumed after ~%lums\n", resumedAfter3);
  if (resumedAfter3 >= 15000) {
    Serial.println("  no response to the manual pulse either - UART-RX "
                   "wake does not appear reachable without an EXTINT wire.");
  }
  Serial.flush();
  measureFixAfterWake();
  Serial.flush();

  Serial.println("\nDone. Halting.");
  Serial.flush();

  while (1)
    delay(100); // Halt
}

void loop() { delay(1000); }
