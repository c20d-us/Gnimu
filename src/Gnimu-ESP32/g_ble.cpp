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

#include "g_ble.h"
#include "config.h"
#include "g_ble_port.h"
#include "g_log.h"
#include "g_protocol_active.h"
#include <atomic>
#include <stdio.h>
#include <string.h>

// BLE driver: the transport decisions shared by both stacks (identity, send
// policy, counters, write queue, sessions). The stack is behind g_ble_port.h.

static const ProtocolDescriptor *proto = ACTIVE_PROTOCOL;

// A notify carries at most MTU - 3 bytes, and the ATT MTU maxes out at 517.
static_assert(PROTOCOL_MAX_FRAME_LEN + 3 <= 517,
              "ERROR: the active protocol's largest frame plus 3 exceeds the "
              "maximum ATT MTU of 517 bytes.");

// "<modelName> <DEVICE_ID>". Static because the stacks keep the pointer.
static char deviceName[40];

// Whether blePortBegin() succeeded.
static bool started = false;

// Outbound accounting
//
// Each refusal kind logs once when it starts and once when it ends.
// g_telemetry reports the counts.

static uint32_t sentFrames = 0;
static uint32_t droppedFrames = 0;
// Subset of droppedFrames refused while unsubscribed, and its value when the
// current episode began.
static uint32_t unsubscribedFrames = 0;
static uint32_t unsubscribedEpisodeStart = 0;

// One-shot log latches, cleared on recovery and at session end.
static bool notSubscribed = false;
static bool mtuRefusing = false;

// Inbound write queue
//
// Writes arrive on the stack's callback context and are dispatched from
// bleUpdate() on the loop. Single-producer, single-consumer ring.
//
// std::atomic, not volatile: on ESP32 the Bluedroid task and the loop run on
// different cores.
//
// 8 slots (7 usable) x 64 bytes holds more than one full 256-byte BLEUart FIFO
// drain.
static constexpr uint8_t kRxSlots = 8;
static uint8_t rxBuf[kRxSlots][TELEMETRY_MAX_WRITE_LEN];
static size_t rxLen[kRxSlots];
static uint8_t rxChannel[kRxSlots];
static std::atomic<uint8_t> rxHead{0}; // consumer index, loop only
static std::atomic<uint8_t> rxTail{0}; // producer index, callback only
// Producer writes, loop reads. An aligned 32-bit read is atomic on both parts.
static uint32_t droppedWrites = 0;

// Producer. Returns false when the ring is full.
static bool rxPush(uint8_t channel, const uint8_t *data, size_t len) {
  const uint8_t tail = rxTail.load(std::memory_order_relaxed);
  const uint8_t next = (uint8_t)((tail + 1) % kRxSlots);
  if (next == rxHead.load(std::memory_order_acquire)) {
    return false; // full
  }
  memcpy(rxBuf[tail], data, len);
  rxLen[tail] = len;
  rxChannel[tail] = channel;
  // Publish the index only after the slot is written.
  rxTail.store(next, std::memory_order_release);
  return true;
}

// Consumer. Dispatches one write per call to bound per-loop work; the loop runs
// far faster than writes arrive.
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

void bleRxFromCallback(uint8_t channel, const uint8_t *data, size_t len,
                       bool wholeMessage) {
  if (len == 0) {
    return;
  }
  if (wholeMessage) {
    // One client write: queued whole or dropped whole.
    if (len > TELEMETRY_MAX_WRITE_LEN || !rxPush(channel, data, len)) {
      droppedWrites++;
    }
    return;
  }
  // Stream slice: split across slots, counting any slot that doesn't fit.
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

// Session lifecycle
//
// Tracked by blePortSessionCount() rather than a connected flag, so a
// disconnect and reconnect between polls still ends the old session. Logged
// here so the stack callbacks never log.

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
  // The next central logs its own episodes.
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
      sessionEnded(); // previous session ended between polls
    }
    sessionBegan();
    if (!up) {
      sessionEnded(); // this one has already ended too
    }
  } else if (inSession && !up) {
    sessionEnded();
  }

  // Log MTU renegotiation after connect.
  if (inSession) {
    const uint16_t mtu = blePortMtu();
    if (mtu != lastLoggedMtu) {
      LOG_PRINTF("🔧 BLE MTU changed: %u -> %u\n", (unsigned int)lastLoggedMtu,
                 (unsigned int)mtu);
      lastLoggedMtu = mtu;
    }
  }
}

// True if notifications are enabled on any notify channel.
static bool anyChannelSubscribed() {
  for (uint8_t i = 0; i < proto->channelCount; i++) {
    if ((proto->channels[i].props & PROP_NOTIFY) && blePortSubscribed(i)) {
      return true;
    }
  }
  return false;
}

// Public interface

void bleBegin() {
  snprintf(deviceName, sizeof(deviceName), "%s %s", proto->modelName,
           DEVICE_ID);
  // A null manufacturer omits the Device Information Service.
  const BleIdentity id = {deviceName,          proto->modelName, DEVICE_ID,
                          proto->manufacturer, proto->fwRev,     proto->hwRev};
  if (!blePortBegin(id, proto)) {
    return; // the port logs the reason
  }
  started = true;
  LOG_PRINTLN("📡 BLE advertising started.");
}

bool bleIsConnected() { return blePortConnected(); }

bool bleIsSubscribed() { return blePortConnected() && anyChannelSubscribed(); }

bool bleEmitFrame(uint8_t channel, const uint8_t *data, size_t len) {
  // Invalid or non-notify channel: an encoder bug. Refuse.
  if (channel >= proto->channelCount ||
      !(proto->channels[channel].props & PROP_NOTIFY)) {
    droppedFrames++;
    return false;
  }

  // Not subscribed. Checked before the MTU so the two cases log separately.
  // The latch logs only when no notify channel is subscribed; a partial
  // subscription is counted silently.
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

  // Refuse a frame the MTU can't carry; a non-fragmenting stack would silently
  // truncate it.
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

  // A short count means the stack stopped mid-frame; count the frame as lost.
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
  rxDispatchOne();
  followSession();
  blePortUpdate();
}

void bleStop() {
  if (!started) {
    return;
  }
  blePortStop();
  LOG_PRINTLN("📴 BLE stopped (disconnected + advertising off).");
}
