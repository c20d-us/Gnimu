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

#include "g_ble.h"
#include "config.h"
#include "g_battery.h"
#include "g_log.h"
#include <atomic>
#include <string.h>
#include "g_protocol_active.h"
#include <bluefruit.h>

// The protocol selected by TELEMETRY_PROTOCOL in config.h. The transport names
// no concrete protocol: identity, service topology and channel UUIDs all come
// from the descriptor.
static const ProtocolDescriptor *proto = ACTIVE_PROTOCOL;

// Advertised name, composed at runtime as "<modelName> <DEVICE_ID>".
//
// It used to be a compile-time literal concatenation, which required g_ble to
// name a protocol constant directly. Building it here instead is what lets the
// transport stay ignorant of which protocol it is serving: modelName comes from
// the descriptor, DEVICE_ID from per-variant config, and neither module has to
// know about the other. Static storage because Bluefruit is given the pointer
// during setup.
static char deviceName[40];

// Nordic UART Service. Its UUIDs are exactly the RaceBox service/Tx/Rx UUIDs
// (see g_proto_racebox.h), so BLEUart implements that transport natively -
// which is the whole reason TRANSPORT_NORDIC_UART exists as a distinct kind.
// Tx = notify, Rx = write.
static BLEUart bleuart;
static BLEDis bledis; // Device Information Service (0x180A)
static BLEBas blebas; // Battery Service (0x180F)

static volatile bool deviceConnected = false;

// Frames the transport refused or truncated, cumulative since boot. Counted
// here rather than in g_telemetry because that module hands bleEmitFrame
// straight to the encoder and never sees individual frame results - and
// because the transport is what knows why a send failed.
//
// Deliberately not logged at the point of failure: that would put a LOG_PRINTF
// on the transmit path at the nav rate, the same latency problem the IMU's
// failed-read guard documents. g_telemetry reports the delta once per stats
// window instead.
static uint32_t droppedFrames = 0;

// Set while refusing to send because the client has not subscribed, so the two
// log lines fire once per episode rather than per frame at the nav rate.
// Cleared in bleUpdate() when the link drops - loop-side, not in the disconnect
// callback, which runs on the SoftDevice thread.
static bool notSubscribed = false;

// ----------------------------------------------------------------------------
// Inbound write queue
// ----------------------------------------------------------------------------
//
// Writes arrive on the transport's own callback context - never the loop - so
// they are copied here and dispatched from bleUpdate(). That is what lets
// ProtocolDescriptor::onWrite promise it runs on the loop.
//
// SINGLE PRODUCER (the BLE callback), SINGLE CONSUMER (the loop), with a ring
// so a burst survives. A configuration sequence is exactly a burst - RaceChrono
// sets a CAN filter with a deny-all followed by one allow-PID per channel - and
// delivering only the first would leave a half-configured device blaming us.
//
// std::atomic with release/acquire, NOT volatile. volatile stops the compiler
// reordering and does nothing about cross-core visibility: on ESP32 the
// Bluedroid task and the Arduino loop task run on DIFFERENT CORES, so a plain
// flag could become visible before the bytes it guards. The release on the
// producer's tail store pairs with the acquire on the consumer's load. Costs
// nothing on the single-core nRF.
//
// This does not widen the codebase's cooperative-polled model (the review's
// ARC-1): BLE callbacks were already the one breach, and this is the first data
// crossing that boundary wider than a single byte - precisely where volatile
// stops being adequate.
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
static uint32_t droppedWrites = 0;

// Producer. Returns false when the ring is full, so the caller can count it.
static bool rxPush(uint8_t channel, const uint8_t *data, size_t len) {
  const uint8_t tail = rxTail.load(std::memory_order_relaxed);
  const uint8_t next = (uint8_t)((tail + 1) % kRxSlots);
  if (next == rxHead.load(std::memory_order_acquire)) {
    return false; // full - the consumer has not caught up
  }
  if (len > TELEMETRY_MAX_WRITE_LEN) {
    len = TELEMETRY_MAX_WRITE_LEN;
  }
  memcpy(rxBuf[tail], data, len);
  rxLen[tail] = len;
  rxChannel[tail] = channel;
  // Release: everything above is visible before the consumer sees the index.
  rxTail.store(next, std::memory_order_release);
  return true;
}

// Consumer. ONE per call, deliberately: the loop runs far faster than writes
// arrive (>1000Hz against one per 15-30ms connection interval), so a burst
// still clears in microseconds while per-iteration work stays bounded by a
// single handler call. Given onWrite now carries a 5.5ms UART deadline,
// bounding the worst case matters more than draining promptly.
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

static uint8_t lastBasPercent = 0xFF; // force a first Battery-service write

// Last MTU value we logged, so bleUpdate() only prints on an actual change
// rather than every loop. 0 = "nothing logged yet this connection" - reset
// on every connect/disconnect so a new connection's negotiation is reported
// fresh rather than silently comparing against a stale value from the
// previous client.
static uint16_t lastLoggedMtu = 0;

// Tracks whether bleBegin() has completed successfully. bleStop() is a no-op
// unless this is true - guards against calling Bluefruit APIs on an
// uninitialized stack (e.g. from a boot-classified quiet state where
// bleBegin was never invoked).
static bool bleInitialized = false;

// Connection callbacks
//
// OFF-LOOP CODE. These run in Bluefruit's callback task, not in loop(). On this
// single-core part that means preemption rather than true parallelism, but it
// is still the only code in the firmware that runs outside the cooperative
// loop, and what it may touch is an explicit, short list - see "Concurrency" in
// docs/architecture-runtime.md before adding to it. Inbound data goes through
// the rxPush() ring, never straight into shared state.
//
// What crosses today, and why each is acceptable:
//   deviceConnected - a lone volatile flag that publishes nothing else, so the
//                     ordering problem ESP32 had to fix (its flag guards a
//                     timestamp) does not arise here.
//   droppedWrites   - one writer (rxCallback), read by the loop; a 32-bit
//                     aligned access is atomic on Cortex-M.
//   lastLoggedMtu   - written here AND in bleUpdate(). A race costs at most one
//                     duplicated or missed "MTU changed" log line. Known,
//                     recorded (ARC-1), deliberately not synchronised.
static void connectCallback(uint16_t conn_handle) {
  deviceConnected = true;
  // Switch to the connected TX power level (see BLE_TX_POWER_* in config.h).
  Bluefruit.setTxPower(BLE_TX_POWER_CONN_DBM);
  BLEConnection *conn = Bluefruit.Connection(conn_handle);
  // Let the CENTRAL drive the MTU exchange. The ceiling is already raised by
  // configPrphBandwidth(BANDWIDTH_MAX). MTU here always reads the BLE default
  // (23) since negotiation hasn't happened yet at connect time. bleUpdate()
  // polls and logs the follow-up change once the central actually raises it.
  const uint16_t mtu = conn->getMtu();
  LOG_PRINTF("✅ BLE client connected (MTU %u).\n", mtu);
  lastLoggedMtu = mtu; // baseline - bleUpdate() only logs a genuine change
}

static void disconnectCallback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  deviceConnected = false;
  // Restore advertising TX power; advertising auto-restarts.
  Bluefruit.setTxPower(BLE_TX_POWER_ADV_DBM);
  LOG_PRINTF("❌ BLE client disconnected (reason 0x%02X).\n", reason);
  lastLoggedMtu = 0; // next connection's negotiation reported fresh
}

// Bytes written by the client on the Rx characteristic. Drained here, logged
// as before, then handed to the protocol - command PARSING is protocol
// business, not transport business, even though RaceBox currently does nothing
// with it.
static void rxCallback(uint16_t conn_handle) {
  (void)conn_handle;

  // Runs on the Bluefruit/SoftDevice callback thread. It does exactly two
  // things: drain the UART (which it MUST, or the buffer stays non-empty and
  // the callback refires forever) and hand the bytes to the ring. No protocol
  // code and no logging run here - both move to rxDispatchOne() on the loop.
  //
  // BLEUart is a BYTE STREAM, so a large drain is SPLIT across slots rather
  // than dropped. Splitting costs nothing here: this transport never promised
  // message boundaries, and onWrite documents that. A discrete transport must
  // behave differently - see the ESP32 copy.
  //
  // The ring is sized to hold one full 256-byte BLEUart FIFO, so a drain is
  // not truncated in practice. Beyond that capacity rxPush() fails and the
  // remainder is counted as a dropped write - lost, not silently truncated.
  uint8_t buf[TELEMETRY_MAX_WRITE_LEN];
  size_t n = 0;
  while (bleuart.available()) {
    buf[n++] = (uint8_t)bleuart.read();
    if (n == sizeof(buf)) {
      if (!rxPush(TELEMETRY_CHANNEL_NORDIC_RX, buf, n)) {
        droppedWrites++;
      }
      n = 0;
    }
  }
  if (n > 0 && !rxPush(TELEMETRY_CHANNEL_NORDIC_RX, buf, n)) {
    droppedWrites++;
  }
}

static void startAdvertising() {
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  // Advertise the Nordic UART (RaceBox) service so apps can discover us by it.
  Bluefruit.Advertising.addService(bleuart);
  // Full device name goes in the scan response (the 128-bit UUID above nearly
  // fills the 31-byte advertising packet). DIS is found after connect.
  Bluefruit.ScanResponse.addName();

  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // fast, slow (units of 0.625 ms)
  Bluefruit.Advertising.setFastTimeout(30);   // seconds in fast mode
  Bluefruit.Advertising.start(0);             // 0 = advertise without timeout
  LOG_PRINTLN("📡 BLE advertising started.");
}

void bleBegin() {
  // Raise the ATT MTU ceiling so an 88-byte notify fits in one packet.
  // Must be called BEFORE Bluefruit.begin() to take effect.
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin(); // 1 peripheral, 0 central (defaults)
  snprintf(deviceName, sizeof(deviceName), "%s %s", proto->modelName,
           DEVICE_ID);
  Bluefruit.setName(deviceName);
  Bluefruit.setTxPower(BLE_TX_POWER_ADV_DBM);
  LOG_PRINTF("✅ BLE TX power set to %d dBm (advertising).\n",
             BLE_TX_POWER_ADV_DBM);
  // g_led owns the RGB LED; stop Bluefruit toggling the onboard LED itself.
  Bluefruit.autoConnLed(false);
  Bluefruit.Periph.setConnectCallback(connectCallback);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

  // Device Information Service, entirely from the descriptor. A protocol that
  // declares no manufacturer (RaceChrono needs no DIS at all) omits the
  // service rather than advertising empty strings.
  if (proto->manufacturer != nullptr) {
    bledis.setManufacturer(proto->manufacturer);
    bledis.setModel(proto->modelName);
    bledis.setSerialNum(DEVICE_ID);
    bledis.setFirmwareRev(proto->fwRev);
    bledis.setHardwareRev(proto->hwRev);
    bledis.begin();
  }

  // Battery Service (reflects the real cell state of charge)
  blebas.begin();
  blebas.write(batteryGetStatus().percent);

  // Transport. Phase C implements TRANSPORT_NORDIC_UART only, which is what
  // the RaceBox descriptor declares; the GATT-channels builder arrives in
  // phase G with RaceChrono. Refusing loudly beats standing up a device that
  // advertises but serves no data.
  if (proto->transport != TRANSPORT_NORDIC_UART) {
    LOG_PRINTLN("❌ BLE: this build implements TRANSPORT_NORDIC_UART only.");
    return;
  }
  bleuart.begin();
  bleuart.setRxCallback(rxCallback);

  startAdvertising();

  bleInitialized = true;
}

bool bleIsConnected() { return deviceConnected; }

bool bleIsSubscribed() { return deviceConnected && bleuart.notifyEnabled(); }

bool bleEmitFrame(uint8_t channel, const uint8_t *data, size_t len) {
  // TRANSPORT_NORDIC_UART is a single byte stream: index 0 is the Tx notify,
  // index 1 the Rx write (TELEMETRY_CHANNEL_NORDIC_RX). Only channel 0 is
  // sendable.
  //
  // Without this guard a protocol that emitted on any other index would have
  // its frame silently routed out the Tx characteristic anyway - wrong data on
  // a valid-looking stream, which is the hardest kind of bug to notice. A
  // protocol genuinely needing more than one outbound channel wants
  // TRANSPORT_GATT_CHANNELS (phase G), not this transport.
  if (channel != TELEMETRY_CHANNEL_PRIMARY) {
    droppedFrames++;
    return false;
  }

  // Notifies the Tx characteristic. If the MTU is smaller than len, BLEUart
  // splits it across notifications; the RaceBox app reassembles the UBX stream.
  // Keeping that behavior is precisely why BLEUart was not replaced here.
  //
  // The return value is a BYTE COUNT and it matters. With _tx_buffered off
  // (the default), write() forwards to a notify that chunks to MTU-3 in a loop
  // and bails mid-loop if the SoftDevice's notify queue is exhausted - so
  // whatever chunks already went out stay out, and a partial UBX packet enters
  // a stream the app then has to resynchronise from. A short count is the only
  // signal that happened.
  // A client that has connected but never written the CCCD cannot receive
  // anything: BLECharacteristic::notify() gates its entire send loop on
  // notifyEnabled() and falls through to false otherwise. Checking it here
  // separates "nobody is listening" from "the link is congested" - both look
  // identical as a short write, and only one of them is a device problem.
  //
  // Observed with nRF Connect, which connects without subscribing: every frame
  // failed for the life of the connection while the drop counter climbed past
  // 1400. The counter was right; it just could not say why.
  if (!bleuart.notifyEnabled()) {
    droppedFrames++;
    if (!notSubscribed) {
      notSubscribed = true;
      LOG_PRINTLN("❌ BLE: client connected but has not subscribed to "
                  "notifications - nothing is being sent.");
    }
    return false;
  }
  if (notSubscribed) {
    notSubscribed = false;
    LOG_PRINTLN("✅ BLE: notifications enabled - sending resumed.");
  }

  const size_t written = bleuart.write(data, len);
  if (written != len) {
    droppedFrames++;
    return false;
  }
  return true;
}

uint32_t bleDroppedFrames() { return droppedFrames; }

uint32_t bleDroppedWrites() { return droppedWrites; }

void bleUpdate() {
  // Inbound writes first: this is what makes onWrite a loop-thread call.
  rxDispatchOne();

  // Reset the subscription latch per connection, loop-side.
  if (!deviceConnected) {
    notSubscribed = false;
  }

  // Keep the Battery service in step with the cell, but only on a real change
  // (avoids a needless notify every loop). LED lives in g_led now.
  const uint8_t pct = batteryGetStatus().percent;
  if (pct != lastBasPercent) {
    blebas.write(pct);
    lastBasPercent = pct;
  }

  // Detect and log MTU growth after connect. See connectCallback()'s comment
  // for why this has to be polled rather than event-driven. Confirms the
  // central actually raised the MTU (expected: 23 -> 247) rather than
  // silently staying stuck at 23, which would otherwise only show up as an
  // unexplained "chunked" 88-byte notify.
  if (deviceConnected) {
    BLEConnection *conn = Bluefruit.Connection(0);
    if (conn) {
      const uint16_t mtu = conn->getMtu();
      if (mtu != lastLoggedMtu) {
        LOG_PRINTF("🔧 BLE MTU changed: %u -> %u\n", lastLoggedMtu, mtu);
        lastLoggedMtu = mtu;
      }
    }
  }
}

void bleStop() {
  // No-op if the stack was never brought up (e.g. boot classifier landed in
  // a quiet state and bleBegin never ran).
  if (!bleInitialized) {
    return;
  }
  // Turn OFF restart-on-disconnect FIRST. Otherwise Bluefruit's internal
  // disconnect handler (triggered by our disconnect() calls below) fires
  // Advertising.start() before - or racing with - our own Advertising.stop(),
  // and the device stays advertising even though we asked it to hush.
  Bluefruit.Advertising.restartOnDisconnect(false);

  // Disconnect any active peripheral connection so the client sees a clean
  // link end rather than a silent, indefinitely-hanging one. Bluefruit's
  // BLE_MAX_CONNECTION is the compile-time upper bound; only handle 0 is
  // ever populated in our peripheral-only config, but iterating costs
  // nothing and future-proofs against a bandwidth-config change.
  for (uint16_t h = 0; h < BLE_MAX_CONNECTION; h++) {
    BLEConnection *c = Bluefruit.Connection(h);
    if (c && c->connected()) {
      c->disconnect();
    }
  }
  // Vanish from BLE scans.
  Bluefruit.Advertising.stop();
  LOG_PRINTLN("📴 BLE stopped (disconnected + advertising off).");
}
