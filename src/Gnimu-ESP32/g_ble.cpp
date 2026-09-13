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
// BLE driver - identical in every tree.
//
// Every DECISION about the transport lives here, once, for both stacks: the
// identity advertised, whether a frame may go out and what each refusal means,
// the counters and their one-shot log lines, the inbound write queue, and the
// session lifecycle. The stack itself sits behind g_ble_port.h. This is the
// same split as g_imu.cpp / g_imu_sensor.h and g_gnss.cpp / g_gnss_port.h, and
// for the same reason: a policy that lives in one file cannot land in one tree
// only. See docs/multiprotocol-design.md section 6.3.
// ============================================================================

#include "g_ble.h"
#include "config.h"
#include "g_ble_port.h"
#include "g_log.h"
#include "g_protocol_active.h"
#include <atomic>
#include <stdio.h>
#include <string.h>

// The protocol selected by TELEMETRY_PROTOCOL in config.h.
static const ProtocolDescriptor *proto = ACTIVE_PROTOCOL;

// A notify carries at most MTU-3 bytes and the ATT MTU tops out at 517, so a
// protocol whose largest frame needs more cannot be served on any stack.
static_assert(PROTOCOL_MAX_FRAME_LEN + 3 <= 517,
              "ERROR: the active protocol's largest frame plus 3 exceeds the "
              "maximum ATT MTU of 517 bytes.");

// Advertised name, "<modelName> <DEVICE_ID>". Static storage: the stacks keep
// the pointer.
static char deviceName[40];

// Whether blePortBegin() succeeded - bleStop() is a no-op otherwise.
static bool started = false;

// ----------------------------------------------------------------------------
// Outbound accounting
// ----------------------------------------------------------------------------
//
// Not logged per frame: that would put a LOG_PRINTF on the transmit path at the
// nav rate. Each refusal KIND logs once when it starts and once when it ends;
// g_telemetry reports the per-window counts.

static uint32_t sentFrames = 0;
static uint32_t droppedFrames = 0;
// The subset of droppedFrames refused because the channel was not subscribed,
// and its value when the current not-subscribed episode began - so the resume
// line can say how many that episode refused. A subset rather than a separate
// count: see bleUnsubscribedFrames() in g_ble.h.
static uint32_t unsubscribedFrames = 0;
static uint32_t unsubscribedEpisodeStart = 0;

// One-shot latches, cleared on the recovery edge and at session end.
static bool notSubscribed = false;
static bool mtuRefusing = false;

// ----------------------------------------------------------------------------
// Inbound write queue
// ----------------------------------------------------------------------------
//
// Writes arrive on the stack's own callback context - never the loop - so
// they are copied here and dispatched from bleUpdate(). That is what lets
// ProtocolDescriptor::onWrite promise it runs on the loop.
//
// SINGLE PRODUCER (the stack callback), SINGLE CONSUMER (the loop), with a ring
// so a burst survives. A configuration sequence is exactly a burst - RaceChrono
// sets a CAN filter with a deny-all followed by one allow-PID per channel - and
// delivering only the first would leave a half-configured device blaming us.
//
// std::atomic with release/acquire, NOT volatile. volatile stops the compiler
// reordering and does nothing about cross-core visibility: on ESP32 the
// Bluedroid task and the Arduino loop task run on DIFFERENT CORES, so a plain
// index could become visible before the bytes it guards. The release on the
// producer's tail store pairs with the acquire on the consumer's load. Costs
// nothing on the single-core nRF.
//
// This does not widen the codebase's cooperative-polled model (ARC-1): BLE
// callbacks were already the one breach, and this is the first data crossing
// that boundary wider than a single byte - precisely where volatile stops being
// adequate.
//
// Sized from the nRF's BLEUart receive FIFO, which is 256 bytes
// (BLE_UART_DEFAULT_FIFO_DEPTH). A single drain can therefore yield four
// 64-byte chunks, so a smaller ring would lose the tail of a large drain - as
// an early version did, dropping 8 of 200 bytes. Seven usable slots (one is
// reserved to distinguish full from empty) covers 448 bytes, comfortably past
// one full FIFO, with slack for a second drain arriving before the loop
// catches up. 512 bytes of RAM on parts with 256KB.
static constexpr uint8_t kRxSlots = 8;
static uint8_t rxBuf[kRxSlots][TELEMETRY_MAX_WRITE_LEN];
static size_t rxLen[kRxSlots];
static uint8_t rxChannel[kRxSlots];
static std::atomic<uint8_t> rxHead{0}; // consumer index, loop only
static std::atomic<uint8_t> rxTail{0}; // producer index, callback only
// Written only by the producer and read by the loop; a 32-bit aligned read is
// atomic on both parts, and a read racing an increment is off by one at most.
static uint32_t droppedWrites = 0;

// Producer. Returns false when the ring is full, so the caller can count it.
static bool rxPush(uint8_t channel, const uint8_t *data, size_t len) {
  const uint8_t tail = rxTail.load(std::memory_order_relaxed);
  const uint8_t next = (uint8_t)((tail + 1) % kRxSlots);
  if (next == rxHead.load(std::memory_order_acquire)) {
    return false; // full - the consumer has not caught up
  }
  memcpy(rxBuf[tail], data, len);
  rxLen[tail] = len;
  rxChannel[tail] = channel;
  // Release: everything above is visible before the consumer sees the index.
  rxTail.store(next, std::memory_order_release);
  return true;
}

void bleRxFromCallback(uint8_t channel, const uint8_t *data, size_t len,
                       bool wholeMessage) {
  if (len == 0) {
    return;
  }
  if (wholeMessage) {
    // One client write: dropped WHOLE if it cannot be delivered whole.
    if (len > TELEMETRY_MAX_WRITE_LEN || !rxPush(channel, data, len)) {
      droppedWrites++;
    }
    return;
  }
  // A stream slice: split across slots. A slot that cannot be queued is
  // counted and the rest still tried - lost, never silently truncated.
  while (len > 0) {
    const size_t n =
        len < TELEMETRY_MAX_WRITE_LEN ? len : TELEMETRY_MAX_WRITE_LEN;
    if (!rxPush(channel, data, n)) {
      droppedWrites++;
    }
    data += n;
    len -= n;
  }
}

// Consumer. ONE per call, deliberately: the loop runs far faster than writes
// arrive (>1000Hz against one per 15-30ms connection interval), so a burst
// still clears in microseconds while per-iteration work stays bounded by a
// single handler call. Given onWrite carries a 5.5ms UART deadline, bounding
// the worst case matters more than draining promptly.
static void rxDispatchOne() {
  const uint8_t head = rxHead.load(std::memory_order_relaxed);
  if (head == rxTail.load(std::memory_order_acquire)) {
    return; // empty
  }
  LOG_PRINTF("📨 BLE write: %u byte(s) on channel %u\n",
             (unsigned int)rxLen[head], (unsigned int)rxChannel[head]);
  if (proto->onWrite != nullptr) {
    proto->onWrite(rxChannel[head], rxBuf[head], rxLen[head]);
  }
  rxHead.store((uint8_t)((head + 1) % kRxSlots), std::memory_order_release);
}

// ----------------------------------------------------------------------------
// Session lifecycle - loop-side
// ----------------------------------------------------------------------------
//
// Followed from blePortSessionCount() rather than by watching a connected flag
// for edges: a disconnect and reconnect that both land between two polls leave
// the flag unchanged, but not the count. Whatever a session END must do - reset
// the latches today; R2-2's queue discard and ARC-5's protocol hook later -
// therefore happens for every session, however quickly the next one arrives.
//
// Logged here, on the loop, so the stack callbacks never log.

static uint32_t seenSessions = 0;
static bool inSession = false;
static uint16_t lastLoggedMtu = 0;

static void sessionBegan() {
  inSession = true;
  lastLoggedMtu = blePortMtu();
  LOG_PRINTF("✅ BLE client connected (MTU %u).\n",
             (unsigned int)lastLoggedMtu);
}

static void sessionEnded() {
  inSession = false;
  // A new central starts from a clean slate: its own not-subscribed and MTU
  // episodes log afresh, whatever the last one left latched.
  notSubscribed = false;
  mtuRefusing = false;
  LOG_PRINTF("❌ BLE client disconnected (reason 0x%02X).\n",
             (unsigned int)blePortDisconnectReason());
}

static void followSession() {
  const uint32_t sessions = blePortSessionCount();
  const bool up = blePortConnected();
  if (sessions != seenSessions) {
    seenSessions = sessions;
    if (inSession) {
      sessionEnded(); // the previous session ended between polls
    }
    sessionBegan();
    if (!up) {
      sessionEnded(); // and this one has already ended too
    }
  } else if (inSession && !up) {
    sessionEnded();
  }

  // Report the MTU the central negotiates after connecting (23 -> 247 on nRF;
  // the requested value -> 517 on ESP32), rather than leaving an MTU stuck at
  // 23 to show up only as refused or chunked frames.
  if (inSession) {
    const uint16_t mtu = blePortMtu();
    if (mtu != lastLoggedMtu) {
      LOG_PRINTF("🔧 BLE MTU changed: %u -> %u\n", (unsigned int)lastLoggedMtu,
                 (unsigned int)mtu);
      lastLoggedMtu = mtu;
    }
  }
}

// ----------------------------------------------------------------------------
// Public interface
// ----------------------------------------------------------------------------

void bleBegin() {
  snprintf(deviceName, sizeof(deviceName), "%s %s", proto->modelName,
           DEVICE_ID);
  // A protocol that declares no manufacturer (RaceChrono needs no DIS at all)
  // gets no Device Information Service rather than one advertising empty
  // strings. BleIdentity carries that as manufacturer == nullptr.
  const BleIdentity id = {deviceName,          proto->modelName, DEVICE_ID,
                          proto->manufacturer, proto->fwRev,     proto->hwRev};
  if (!blePortBegin(id, proto)) {
    return; // the port has said why
  }
  started = true;
  LOG_PRINTLN("📡 BLE advertising started.");
}

bool bleIsConnected() { return blePortConnected(); }

// Whether the central receives anything at all: notifications enabled on at
// least one of the protocol's notify channels.
static bool anyChannelSubscribed() {
  for (uint8_t i = 0; i < proto->channelCount; i++) {
    if ((proto->channels[i].props & PROP_NOTIFY) && blePortSubscribed(i)) {
      return true;
    }
  }
  return false;
}

bool bleIsSubscribed() { return blePortConnected() && anyChannelSubscribed(); }

bool bleEmitFrame(uint8_t channel, const uint8_t *data, size_t len) {
  // A channel the protocol does not have, or one a central cannot be notified
  // on, is a programming error in an encoder. Refused, never routed elsewhere:
  // wrong data on a valid-looking stream is the hardest bug to notice.
  if (channel >= proto->channelCount ||
      !(proto->channels[channel].props & PROP_NOTIFY)) {
    droppedFrames++;
    return false;
  }

  // Nobody listening on this channel. Checked first because it separates
  // "nobody is listening" from "the link cannot carry this", which otherwise
  // look identical - and only the second is a device problem. Observed with nRF
  // Connect, which connects without subscribing: every frame refused for the
  // life of the connection while the drop counter climbed past 1400 (NEW-1).
  //
  // The one-shot pair means what it says - NOTHING is being sent - so it
  // latches only when no notify channel is subscribed. A client subscribed to
  // some channels and not others has chosen what it receives: those refusals
  // are counted in the subset, silently. Otherwise the latch would flip on
  // every epoch of a partly subscribed multi-channel client, two log lines at
  // the nav rate.
  if (!blePortSubscribed(channel)) {
    droppedFrames++;
    if (!notSubscribed && !anyChannelSubscribed()) {
      notSubscribed = true;
      unsubscribedEpisodeStart = unsubscribedFrames;
      LOG_PRINTLN("❌ BLE: client connected but has not subscribed to "
                  "notifications - nothing is being sent.");
    }
    unsubscribedFrames++;
    return false;
  }
  if (notSubscribed) {
    notSubscribed = false;
    LOG_PRINTF("✅ BLE: notifications enabled - sending resumed (%u frame(s) "
               "refused while unsubscribed).\n",
               (unsigned int)(unsubscribedFrames - unsubscribedEpisodeStart));
  }

  // A frame the negotiated MTU cannot carry is REFUSED, not sent truncated: on
  // a stack that does not fragment, a notify larger than MTU-3 is silently cut,
  // and a client receives a fragment of every packet forever with no
  // diagnostic. A stack that fragments reports no limit here.
  if (len > blePortMaxFrame(channel)) {
    droppedFrames++;
    if (!mtuRefusing) {
      mtuRefusing = true;
      LOG_PRINTF("❌ BLE: peer MTU %u too small for a %u-byte frame (need %u). "
                 "Refusing to send - a truncated packet is worse than none.\n",
                 (unsigned int)blePortMtu(), (unsigned int)len,
                 (unsigned int)(len + 3));
    }
    return false;
  }
  if (mtuRefusing) {
    mtuRefusing = false;
    LOG_PRINTF("✅ BLE: peer MTU now %u - sending resumed.\n",
               (unsigned int)blePortMtu());
  }

  // A short count is a send the stack gave up part way through - on nRF, a
  // notify queue exhausted mid-frame. What went out stays out; the frame is
  // counted as lost, since the client now has to resynchronise.
  if (blePortSend(channel, data, len) != len) {
    droppedFrames++;
    return false;
  }
  sentFrames++;
  return true;
}

uint32_t bleSentFrames() { return sentFrames; }

uint32_t bleDroppedFrames() { return droppedFrames; }

uint32_t bleUnsubscribedFrames() { return unsubscribedFrames; }

uint32_t bleDroppedWrites() { return droppedWrites; }

void bleUpdate() {
  // Inbound writes first: this is what makes onWrite a loop-thread call.
  rxDispatchOne();
  followSession();
  blePortUpdate();
}

void bleStop() {
  if (!started) {
    return; // never brought up (a boot that went straight to a quiet state)
  }
  blePortStop();
  LOG_PRINTLN("📴 BLE stopped (disconnected + advertising off).");
}
