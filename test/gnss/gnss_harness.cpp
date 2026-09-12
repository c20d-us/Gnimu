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
// GNSS harness - runs the REAL g_gnss.cpp on the host, against a fake receiver.
//
// It exists because nothing covered that file: the telemetry harness fakes the
// GNSS module out entirely, so the baud sweep, the config sequence and ROB-1's
// non-halting failure path were tested only by flashing a board. It is built
// BEFORE the shared-core split (see the deferred entry in
// docs/code-review-remediation.md) so the split can be held to "the receiver
// sees exactly the same calls, in the same order".
//
// WHAT THE GOLDEN IS: the ordered log of every port and library call, with its
// arguments, plus the driver's own state at each step. Serial log WORDING is
// deliberately NOT captured - converging it across the two trees is part of the
// split, and locking it here would report that convergence as a regression.
//
// The fake receiver answers only when the port is open at ITS baud, which is
// what makes the sweep a real test rather than a walk through a list.
//
// BUILD & RUN: ./test/run_gnss_harness.sh   (--save rewrites the goldens)
// ============================================================================

#include "g_gnss.h"

#include "config.h"
#include "fake_gnss.h"

// GNSS_BAUD as text, to match the logged "gnss.begin(port@<baud>)".
#define GNSS_BAUD_STR2(x) #x
#define GNSS_BAUD_STR1(x) GNSS_BAUD_STR2(x)
#define GNSS_BAUD_STR GNSS_BAUD_STR1(GNSS_BAUD)

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>

// ----------------------------------------------------------------------------
// Fakes
// ----------------------------------------------------------------------------

static unsigned long nowMs = 0;
unsigned long millis() { return nowMs; }
void delay(unsigned long ms) { nowMs += ms; } // the driver's settling waits

FakeSerial Serial;
size_t FakeSerial::print(const char *s) { return strlen(s); }
size_t FakeSerial::println(const char *s) { return strlen(s); }
size_t FakeSerial::write(const uint8_t *, size_t n) { return n; }

static std::vector<std::string> g_log;
static void logf(const char *fmt, ...) {
  char line[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  g_log.push_back(line);
}
void fakeGnssLogReset() { g_log.clear(); }
void fakeGnssLogPrint() {
  for (const std::string &l : g_log) {
    printf("  %s\n", l.c_str());
  }
}

// The receiver.
static unsigned long g_receiverBaud = 0;
static bool g_failVerify = false;
static bool g_verifySeen = false;
static std::string g_reject;
static void (*g_pvtCallback)(UBX_NAV_PVT_data_t *) = nullptr;
void fakeGnssReceiverBaud(unsigned long baud) { g_receiverBaud = baud; }
void fakeGnssFailVerify(bool fail) {
  g_failVerify = fail;
  g_verifySeen = false;
}
void fakeGnssRejectCall(const char *name) { g_reject = name ? name : ""; }

// Every configuration call runs through here, so the log and the rejection
// knob cannot get out of step with each other.
static bool call(const char *name, const char *fmt, ...) {
  char args[128] = "";
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(args, sizeof(args), fmt, ap);
  va_end(ap);
  const bool ok = g_reject != name;
  logf("%s %s-> %d", name, args, (int)ok);
  return ok;
}

void FakeSerialPort::openAt(unsigned long baud) {
  baud_ = baud;
  open_ = true;
}
void FakeSerialPort::end() {
  logf("port.end");
  open_ = false;
  baud_ = 0;
}
int FakeSerialPort::available() { return 0; } // nothing buffered in these runs
int FakeSerialPort::read() { return -1; }

HardwareSerial::HardwareSerial(int uartNum) { logf("HardwareSerial(%d)", uartNum); }
void HardwareSerial::begin(unsigned long baud, uint32_t, int rxPin, int txPin) {
  openAt(baud);
  logf("port.begin %lu rx=%d tx=%d", baud, rxPin, txPin);
}
size_t HardwareSerial::setRxBufferSize(size_t bytes) {
  // The real one returns 0 (and only log_e()s) when the driver is already
  // running - the silent failure LAT-5's warning exists for.
  const size_t got = open() ? 0 : bytes;
  logf("port.setRxBufferSize %zu -> %zu", bytes, got);
  return got;
}
void Uart::begin(unsigned long baud) {
  openAt(baud);
  logf("port.begin %lu", baud);
}
Uart Serial1;

bool SFE_UBLOX_GNSS_SERIAL::begin(Stream &stream) {
  // The driver hands us whatever the port returned; the fake receiver only
  // answers if that port is open at its baud.
  FakeSerialPort &port = static_cast<FakeSerialPort &>(stream);
  bool ok = port.open() && g_receiverBaud != 0 && port.baud() == g_receiverBaud;
  if (ok && g_failVerify && g_verifySeen) {
    ok = false; // the post-switch verification, and only that one
  }
  if (ok) {
    g_verifySeen = true;
  }
  logf("gnss.begin(port@%lu) -> %d", port.baud(), (int)ok);
  return ok;
}
bool SFE_UBLOX_GNSS_SERIAL::setSerialRate(unsigned long baud, uint8_t layer) {
  const bool ok = call("setSerialRate", "%lu layer=%u ", baud, layer);
  if (ok) {
    g_receiverBaud = baud; // the receiver really does move
  }
  return ok;
}
bool SFE_UBLOX_GNSS_SERIAL::saveConfigSelective(uint32_t subsection) {
  return call("saveConfigSelective", "0x%02X ", subsection);
}
bool SFE_UBLOX_GNSS_SERIAL::enableGNSS(bool enable, sfe_ublox_gnss_ids_e id,
                                       uint8_t layer) {
  return call("enableGNSS", "id=%d en=%d layer=%u ", (int)id, (int)enable, layer);
}
bool SFE_UBLOX_GNSS_SERIAL::setAopCfg(uint8_t aopCfg, uint16_t maxWait,
                                      uint8_t layer) {
  return call("setAopCfg", "%u maxWait=%u layer=%u ", aopCfg, maxWait, layer);
}
bool SFE_UBLOX_GNSS_SERIAL::setDynamicModel(dynModel model, uint8_t layer) {
  return call("setDynamicModel", "%d layer=%u ", (int)model, layer);
}
bool SFE_UBLOX_GNSS_SERIAL::setUART1Output(uint8_t comType, uint8_t layer) {
  return call("setUART1Output", "0x%02X layer=%u ", comType, layer);
}
bool SFE_UBLOX_GNSS_SERIAL::setVal8(uint32_t key, uint8_t value, uint8_t layer) {
  return call("setVal8", "key=0x%08X val=%u layer=%u ", key, value, layer);
}
bool SFE_UBLOX_GNSS_SERIAL::setNavigationFrequency(uint8_t hz, uint8_t layer) {
  return call("setNavigationFrequency", "%u layer=%u ", hz, layer);
}
bool SFE_UBLOX_GNSS_SERIAL::setAutoPVTcallbackPtr(
    void (*cb)(UBX_NAV_PVT_data_t *), uint8_t layer) {
  const bool ok = call("setAutoPVTcallbackPtr", "layer=%u ", layer);
  if (ok) {
    g_pvtCallback = cb;
  }
  return ok;
}
void SFE_UBLOX_GNSS_SERIAL::checkUblox() { logf("checkUblox"); }
void SFE_UBLOX_GNSS_SERIAL::checkCallbacks() { logf("checkCallbacks"); }

void fakeGnssDeliverEpoch(uint32_t iTOW, uint8_t fixType, int32_t gSpeedMmS) {
  if (g_pvtCallback == nullptr) {
    logf("epoch dropped: no callback registered");
    return;
  }
  UBX_NAV_PVT_data_t pvt;
  memset(&pvt, 0, sizeof(pvt));
  pvt.iTOW = iTOW;
  pvt.fixType = fixType;
  pvt.gSpeed = gSpeedMmS;
  pvt.flags.bits.gnssFixOK = fixType >= 2;
  g_pvtCallback(&pvt);
  logf("epoch delivered iTOW=%u", iTOW);
}

// ----------------------------------------------------------------------------
// Scenarios. Each prints the call log the receiver saw, then the driver state.
// ----------------------------------------------------------------------------

static int failures = 0;
static void check(bool ok, const char *what) {
  printf("%s %s\n", ok ? "✅" : "❌", what);
  failures += !ok;
}

// Log queries, so the scenarios can assert on the SEQUENCE rather than leaving
// it as something only a human reading the golden would notice.
static int logCount(const char *prefix) {
  int n = 0;
  for (const std::string &l : g_log) {
    n += l.compare(0, strlen(prefix), prefix) == 0;
  }
  return n;
}
static int logFirst(const char *prefix) { // index, or -1
  for (size_t i = 0; i < g_log.size(); i++) {
    if (g_log[i].compare(0, strlen(prefix), prefix) == 0) {
      return (int)i;
    }
  }
  return -1;
}
// Every begin() is preceded by an end(), except the first: the sweep must not
// leave a port open behind it.
static bool portsBalanced() {
  int open = 0;
  for (const std::string &l : g_log) {
    if (l.compare(0, 11, "port.begin ") == 0) {
      if (open != 0) {
        return false;
      }
      open = 1;
    } else if (l == "port.end") {
      open = 0;
    }
  }
  return true;
}

static void showState(const char *label) {
  printf("  state: %s up=%d consume=%s latest=%s\n", label, (int)gnssIsUp(),
         gnssConsumePvt() ? "epoch" : "null",
         gnssLatestPvt() ? "epoch" : "null");
}

// The normal boot: the receiver is already at GNSS_BAUD, so the sweep stops on
// its first try and every configuration call follows.
static int runAtTarget() {
  printf("S at-target\n");
  fakeGnssReceiverBaud(GNSS_BAUD);
  const bool up = gnssBegin();
  fakeGnssLogPrint();
  check(up && gnssIsUp(), "at-target: bring-up succeeded");
  check(logFirst("gnss.begin") == logFirst("gnss.begin(port@" GNSS_BAUD_STR ")"),
        "at-target: the sweep tries GNSS_BAUD first");
  // LAT-5, where the call exists at all: sized ONCE, and before the first
  // begin() - the driver silently ignores it afterwards.
  check(logCount("port.setRxBufferSize") <= 1 &&
            (logFirst("port.setRxBufferSize") < 0 ||
             logFirst("port.setRxBufferSize") < logFirst("port.begin ")),
        "at-target: RX ring sized at most once, before the first begin()");
  return 0;
}

// A receiver left at the factory 9600: the sweep must find it, move it, cycle
// the MCU's port, verify, and save ONLY the I/O-port subsection.
static int runAt9600() {
  printf("S at-9600\n");
  fakeGnssReceiverBaud(9600);
  const bool up = gnssBegin();
  fakeGnssLogPrint();
  check(up && gnssIsUp(), "at-9600: found, switched and verified");
  return 0;
}

// Nothing answers anywhere. ROB-1: report and return, never halt - and the
// module must stay inert afterwards rather than half-configured.
static int runAbsent() {
  printf("S absent\n");
  fakeGnssReceiverBaud(0);
  const bool up = gnssBegin();
  fakeGnssLogPrint();
  check(!up && !gnssIsUp(), "absent: returns false, not up (no halt)");
  check(logCount("gnss.begin") == 7, "absent: sweeps all seven rates, then stops");
  check(portsBalanced(), "absent: every attempt closes its port before the next");
  check(logCount("port.setRxBufferSize") <= 1,
        "absent: the RX ring is sized once for the whole sweep, not per attempt");
  fakeGnssLogReset();
  gnssPoll();
  fakeGnssDeliverEpoch(1000, 3, 0);
  fakeGnssLogPrint();
  showState("after poll");
  check(gnssLatestPvt() == nullptr, "absent: no epoch can arrive");
  return 0;
}

// The receiver answers at 9600 and takes the new baud, but does not answer
// after the switch. The branch that exists for "something went deeply wrong".
static int runVerifyFails() {
  printf("S verify-fails\n");
  fakeGnssReceiverBaud(9600);
  fakeGnssFailVerify(true);
  const bool up = gnssBegin();
  fakeGnssLogPrint();
  check(!up && !gnssIsUp(), "verify-fails: bring-up abandoned, not up");
  return 0;
}

// The receiver answers but rejects a configuration key - an older firmware, or
// a message this build does not have. Bring-up must carry on and still come up.
static int runConfigRejects() {
  printf("S config-rejects (setVal8)\n");
  fakeGnssReceiverBaud(GNSS_BAUD);
  fakeGnssRejectCall("setVal8");
  const bool up = gnssBegin();
  fakeGnssLogPrint();
  check(up && gnssIsUp(), "config-rejects: a rejected key does not stop bring-up");
  return 0;
}

// The epoch path: consume-once, and latest surviving the consume.
static int runEpochs() {
  printf("S epochs\n");
  fakeGnssReceiverBaud(GNSS_BAUD);
  gnssBegin();
  fakeGnssLogReset();

  gnssPoll();
  fakeGnssDeliverEpoch(100000, 3, 8000);
  const UBX_NAV_PVT_data_t *first = gnssConsumePvt();
  check(first != nullptr && first->iTOW == 100000, "epoch reaches the driver");
  check(gnssConsumePvt() == nullptr, "consume-once: the second call sees nothing");
  check(gnssLatestPvt() != nullptr && gnssLatestPvt()->iTOW == 100000,
        "latest survives the consume");

  fakeGnssDeliverEpoch(100050, 3, 8100);
  const UBX_NAV_PVT_data_t *second = gnssConsumePvt();
  check(second != nullptr && second->iTOW == 100050, "the next epoch replaces it");
  fakeGnssLogPrint();

  fakeGnssLogReset();
  gnssEnd();
  gnssPoll(); // must be inert now
  fakeGnssDeliverEpoch(100100, 3, 8200);
  fakeGnssLogPrint();
  check(!gnssIsUp(), "gnssEnd: releases the port and marks it down");
  return 0;
}

int main(int argc, char **argv) {
  setvbuf(stdout, nullptr, _IOLBF, 0);
  const char *which = argc > 1 ? argv[1] : "at-target";
  if (!strcmp(which, "at-target")) runAtTarget();
  else if (!strcmp(which, "at-9600")) runAt9600();
  else if (!strcmp(which, "absent")) runAbsent();
  else if (!strcmp(which, "verify-fails")) runVerifyFails();
  else if (!strcmp(which, "config-rejects")) runConfigRejects();
  else if (!strcmp(which, "epochs")) runEpochs();
  else {
    fprintf(stderr, "unknown scenario: %s\n", which);
    return 2;
  }
  return failures == 0 ? 0 : 1;
}
