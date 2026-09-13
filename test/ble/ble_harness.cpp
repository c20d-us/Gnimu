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
// BLE harness - runs the REAL shared g_ble.cpp on the host, against a scripted
// fake port (the functions g_ble_port.h declares) and a test protocol with two
// notify channels (see g_protocol_active.h beside this file).
//
// The driver owns every transport decision, so this is where they are tested:
// the emit sequence and each refusal's counters and one-shot log lines, the
// multi-channel counter movements the telemetry "sent" rule depends on, the
// session lifecycle including reconnects that fall between two polls, and the
// inbound write queue. The stacks themselves are not covered - that is what
// the hardware baseline in docs/code-review-remediation.md is for.
//
// WHAT THE GOLDEN IS: everything each scenario prints, in order - the port
// calls that matter (begin, send, stop), the driver's real log lines through
// the real g_log.h, and each check. One process per scenario, so the driver's
// statics start fresh.
//
// BUILD & RUN: ./test/run_ble_harness.sh   (--save rewrites the goldens)
// ============================================================================

#include "g_ble.h"
#include "g_ble_port.h"

#include "config.h"
#include "g_log.h"
#include "g_protocol_active.h"

#include <cstdio>
#include <cstring>
#include <utility>

// ----------------------------------------------------------------------------
// Fakes
// ----------------------------------------------------------------------------

// Serial goes straight to stdout, so driver log lines interleave in order with
// the harness's own output.
FakeSerial Serial;
size_t FakeSerial::print(const char *s) { return (size_t)printf("%s", s); }
size_t FakeSerial::println(const char *s) { return (size_t)printf("%s\n", s); }
size_t FakeSerial::write(const uint8_t *data, size_t len) {
  return fwrite(data, 1, len, stdout);
}

// The port.
static bool g_beginResult = true;
static bool g_up = false;
static uint32_t g_sessions = 0;
static uint8_t g_reason = 0;
static bool g_sub[PROTOCOL_CHANNEL_COUNT] = {};
static bool g_fragments = false; // a stack that splits notifies itself
static uint16_t g_mtu = 247;
static int g_shortNext = 0; // the next N sends come back short

bool blePortBegin(const BleIdentity &id, const ProtocolDescriptor *proto) {
  printf("port.begin name=\"%s\" model=\"%s\" serial=\"%s\" manufacturer=%s "
         "fw=\"%s\" hw=\"%s\" channels=%u -> %d\n",
         id.name, id.model, id.serial,
         id.manufacturer ? id.manufacturer : "(none)", id.fwRev, id.hwRev,
         (unsigned)proto->channelCount, (int)g_beginResult);
  return g_beginResult;
}
bool blePortConnected() { return g_up; }
uint32_t blePortSessionCount() { return g_sessions; }
uint8_t blePortDisconnectReason() { return g_reason; }
bool blePortSubscribed(uint8_t channel) {
  return channel < PROTOCOL_CHANNEL_COUNT && g_sub[channel];
}
size_t blePortMaxFrame(uint8_t) {
  return g_fragments ? (size_t)-1 : (g_mtu > 3 ? (size_t)(g_mtu - 3) : 0);
}
uint16_t blePortMtu() { return g_mtu; }
size_t blePortSend(uint8_t channel, const uint8_t *, size_t len) {
  const bool shortSend = g_shortNext > 0;
  if (shortSend) {
    g_shortNext--;
  }
  printf("port.send ch=%u len=%zu%s\n", (unsigned)channel, len,
         shortSend ? " (short)" : "");
  return shortSend ? len / 2 : len;
}
void blePortUpdate() {}
void blePortStop() { printf("port.stop\n"); }

// The test protocol's handlers.
static int g_writes = 0;
static size_t g_writeBytes = 0;
void harnessEncode(const TelemetrySample &, TelemetryEmit) {}
void harnessOnWrite(uint8_t channel, const uint8_t *data, size_t len) {
  g_writes++;
  g_writeBytes += len;
  printf("proto.onWrite ch=%u len=%zu first=0x%02X\n", (unsigned)channel, len,
         (unsigned)data[0]);
}

// ----------------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------------

static int failures = 0;
static void check(bool ok, const char *what) {
  printf("%s %s\n", ok ? "✅" : "❌", what);
  failures += !ok;
}

static const uint8_t kFrame[PROTOCOL_MAX_FRAME_LEN] = {0xAB};

struct Counts {
  uint32_t sent, dropped, unsub;
  uint32_t failed() const { return dropped - unsub; }
};
static Counts counts() {
  return {bleSentFrames(), bleDroppedFrames(), bleUnsubscribedFrames()};
}
static void showCounts(const char *label) {
  const Counts c = counts();
  printf("  counts after %s: sent=%u dropped=%u unsubscribed=%u failed=%u\n",
         label, c.sent, c.dropped, c.unsub, c.failed());
}

static void connectNow(uint16_t mtu) {
  g_mtu = mtu;
  g_up = true;
  g_sessions++;
}

// ----------------------------------------------------------------------------
// Scenarios
// ----------------------------------------------------------------------------

// Identity from the descriptor and DEVICE_ID, handed to the port; the
// advertising line only after the port succeeded; bleStop() reaching the port.
static void runBegin() {
  printf("S begin\n");
  bleBegin();
  bleStop();
}

// A port that could not come up: no advertising line, and bleStop() must not
// reach a stack that was never started.
static void runBeginFails() {
  printf("S begin-fails\n");
  g_beginResult = false;
  bleBegin();
  bleStop();
  check(true, "begin-fails: no advertising line and no port.stop above");
}

// The emit sequence, one refusal kind at a time.
static void runEmit() {
  printf("S emit\n");
  connectNow(247);
  bleUpdate();

  check(!bleEmitFrame(3, kFrame, sizeof(kFrame)) &&
            !bleEmitFrame(1, kFrame, sizeof(kFrame)),
        "emit: a channel the protocol lacks, and a write-only one, are refused");
  showCounts("invalid channels");

  for (int i = 0; i < 3; i++) {
    bleEmitFrame(0, kFrame, sizeof(kFrame));
  }
  showCounts("3 unsubscribed");
  check(counts().unsub == 3 && counts().failed() == 2,
        "emit: unsubscribed refusals are the subset; the invalid channels are "
        "the failures");

  g_sub[0] = true;
  check(bleEmitFrame(0, kFrame, sizeof(kFrame)), "emit: subscribed frame sent");
  showCounts("subscribing");

  g_mtu = 13; // room for 10 bytes, against a 20-byte frame
  for (int i = 0; i < 2; i++) {
    bleEmitFrame(0, kFrame, sizeof(kFrame));
  }
  g_mtu = 247;
  check(bleEmitFrame(0, kFrame, sizeof(kFrame)),
        "emit: the MTU refusal ends once the frame fits");
  showCounts("2 MTU refusals");
  g_mtu = 13;
  g_fragments = true; // a fragmenting stack reports no limit at any MTU
  check(bleEmitFrame(0, kFrame, sizeof(kFrame)),
        "emit: a stack that fragments is never refused on MTU");
  g_fragments = false;
  g_mtu = 247;
  check(counts().failed() == 4 && counts().unsub == 3,
        "emit: MTU refusals are failures, not unsubscribed");

  g_shortNext = 1;
  check(!bleEmitFrame(0, kFrame, sizeof(kFrame)),
        "emit: a short send is a failure");
  showCounts("a short send");
  check(counts().sent == 3 && counts().failed() == 5,
        "emit: sent counts only frames the port accepted whole");
}

// The counter movements g_telemetry's "sent" rule reads, for each row of
// docs/multiprotocol-design.md 6.3's table: an epoch counts only if sent moved
// and failed did not.
static void runMultiChannel() {
  printf("S multichannel\n");
  connectNow(247);
  bleUpdate();
  auto epoch = [](const char *label) {
    const Counts b = counts();
    bleEmitFrame(0, kFrame, sizeof(kFrame));
    bleEmitFrame(2, kFrame, sizeof(kFrame));
    const Counts a = counts();
    printf("  %s: sent +%u, failed +%u, unsubscribed +%u\n", label,
           a.sent - b.sent, a.failed() - b.failed(), a.unsub - b.unsub);
    return std::make_pair(a.sent != b.sent, a.failed() != b.failed());
  };

  g_sub[0] = g_sub[2] = true;
  auto r = epoch("both subscribed");
  check(r.first && !r.second, "multichannel: both subscribed - counts");

  g_sub[2] = false;
  r = epoch("one subscribed");
  check(r.first && !r.second,
        "multichannel: one of two subscribed - counts (the old rule read 0 Hz)");
  epoch("one subscribed, again");
  epoch("one subscribed, again");
  check(true, "multichannel: a partly subscribed client logs no "
              "not-subscribed or resumed lines, epoch after epoch");

  g_sub[0] = false;
  r = epoch("none subscribed");
  check(!r.first && !r.second, "multichannel: none subscribed - does not count");

  g_sub[0] = g_sub[2] = true;
  g_shortNext = 1;
  r = epoch("one accepted, one short");
  check(r.first && r.second,
        "multichannel: one accepted and one failed - does not count");

  check(bleIsSubscribed(), "multichannel: bleIsSubscribed with a notify "
                           "channel subscribed");
  g_sub[0] = g_sub[2] = false;
  check(!bleIsSubscribed(), "multichannel: bleIsSubscribed with none");
}

// Session lifecycle, including the two reconnect shapes a polled flag misses.
static void runSession() {
  printf("S session\n");
  connectNow(23);
  bleUpdate();
  g_mtu = 247;
  bleUpdate();
  bleUpdate(); // unchanged MTU: no second line

  bleEmitFrame(0, kFrame, sizeof(kFrame)); // latch the not-subscribed episode
  g_reason = 0x13;
  g_up = false;
  bleUpdate();

  printf("  -- reconnect after a visible disconnect\n");
  connectNow(23);
  bleUpdate();
  bleEmitFrame(0, kFrame, sizeof(kFrame));
  check(true, "session: the not-subscribed line above fired afresh");

  printf("  -- disconnect and reconnect both between two polls\n");
  g_reason = 0x08;
  connectNow(23); // still up, count moved
  bleUpdate();
  bleEmitFrame(0, kFrame, sizeof(kFrame));
  check(true, "session: one disconnect, one connect, and the latch reset");

  printf("  -- connect and disconnect both between two polls\n");
  g_up = false;
  bleUpdate();
  g_reason = 0x3E;
  g_sessions++; // a whole session came and went unseen
  bleUpdate();
  check(!bleIsConnected(), "session: a session seen only by its count is "
                           "still reported, and ends not connected");
}

// The inbound write queue.
static void runInbound() {
  printf("S inbound\n");
  uint8_t buf[200];
  for (size_t i = 0; i < sizeof(buf); i++) {
    buf[i] = (uint8_t)i;
  }

  bleRxFromCallback(1, buf, 5, true);
  bleUpdate();
  check(g_writes == 1 && bleDroppedWrites() == 0,
        "inbound: one write, dispatched on the next update");

  bleRxFromCallback(1, buf, TELEMETRY_MAX_WRITE_LEN + 1, true);
  check(bleDroppedWrites() == 1, "inbound: an oversized discrete write is "
                                 "dropped whole and counted");

  bleRxFromCallback(1, buf, sizeof(buf), false);
  for (int i = 0; i < 5; i++) {
    bleUpdate();
  }
  check(g_writes == 5 && g_writeBytes == 5 + sizeof(buf),
        "inbound: a 200-byte stream slice is split 64/64/64/8, one dispatch "
        "per update");

  for (int i = 0; i < 9; i++) {
    bleRxFromCallback(1, buf + i, 1, true);
  }
  check(bleDroppedWrites() == 3,
        "inbound: a burst of 9 fills the 7 usable slots and drops 2");
  bleUpdate();
  check(g_writes == 6, "inbound: exactly one dispatch per update");
  for (int i = 0; i < 10; i++) {
    bleUpdate();
  }
  check(g_writes == 12, "inbound: the queue drains in order, then is empty");
}

int main(int argc, char **argv) {
  setvbuf(stdout, nullptr, _IOLBF, 0);
  Serial.enabled = true;
  const char *which = argc > 1 ? argv[1] : "begin";
  if (!strcmp(which, "begin")) runBegin();
  else if (!strcmp(which, "begin-fails")) runBeginFails();
  else if (!strcmp(which, "emit")) runEmit();
  else if (!strcmp(which, "multichannel")) runMultiChannel();
  else if (!strcmp(which, "session")) runSession();
  else if (!strcmp(which, "inbound")) runInbound();
  else {
    fprintf(stderr, "unknown scenario: %s\n", which);
    return 2;
  }
  return failures == 0 ? 0 : 1;
}
