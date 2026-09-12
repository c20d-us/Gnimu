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
#include "g_log.h"
#include <atomic>
#include <string.h>
#include "g_protocol_active.h"
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// BLE state
//
// The protocol selected by TELEMETRY_PROTOCOL in config.h. The transport names
// no concrete protocol: identity, service topology and channel UUIDs all come
// from the descriptor.
static const ProtocolDescriptor *proto = ACTIVE_PROTOCOL;

// Advertised name, composed at runtime as "<modelName> <DEVICE_ID>" so this
// module never names a protocol constant directly.
static char deviceName[40];

// Characteristics, parallel to proto->channels - index IS the channel number.
// Sized generously; the descriptor's channelCount is what actually bounds it.
static const uint8_t MAX_CHANNELS = 8;
static BLECharacteristic *channelChr[MAX_CHANNELS] = {nullptr};

// Frames the transport refused, cumulative since boot. See the nRF copy for
// why this lives in the transport rather than g_telemetry.
//
// EXPECT THIS TO READ ZERO until BLE-2 lands. notify() returns void on this
// stack, so the only thing reachable below is an invalid channel - a
// programming error, not a runtime condition. A quiet counter here is not
// evidence that nothing is being dropped; it is evidence that this stack
// cannot see drops yet.
static uint32_t droppedFrames = 0;

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


// The ATT MTU this build asks the central for, derived rather than configured.
//
// There is exactly one correct class of value - large enough for the biggest
// frame the active protocol emits - so it was never a tunable, and having it in
// config.h as BLE_MTU_BYTES only created something to get wrong. Deriving it
// also means it tracks the protocol: select a different one and the request
// follows without anyone remembering to change it.
//
// +3 is the ATT notify header; the usable payload is MTU-3. This is an exact
// fit by construction, deliberately with no arbitrary headroom - a margin would
// mask an off-by-one here rather than expose it, and an inadequate NEGOTIATED
// MTU is caught loudly at send time below.
//
// TO EXERCISE THE REFUSAL PATH: temporarily set this to 23 and reconnect.
//
// It does NOT simulate a client that declines the raise, despite the obvious
// reading. Measured 2026-09-10: an iOS central negotiated 517 regardless of
// what was requested, because the CENTRAL drives the MTU exchange. What the
// knob actually exposes is the window BEFORE that exchange completes, where
// getPeerMTU() still reports the 23-byte default - about 650ms, 13 frames at
// 20Hz, which is long enough to watch the refusal, the one-shot warning, the
// recovery line and the drop counter fire in sequence.
//
// With the derived value there is no refusal at connect at all - verified on
// the same hardware - so this check costs nothing in normal operation.
//
// Genuinely simulating a central that refuses to raise the MTU needs a BLE
// client under your control, not a firmware knob. That path remains untested.
static constexpr uint16_t kRequestedMtu = PROTOCOL_MAX_FRAME_LEN + 3;
static_assert(kRequestedMtu <= 517,
              "ERROR: the active protocol's largest frame plus 3 exceeds the "
              "maximum ATT MTU of 517 bytes.");

// Set while refusing to send, so the two log lines below fire once per episode
// rather than per frame at the nav rate. Cleared in bleUpdate() when the link
// drops, NOT in the disconnect callback: that runs on the Bluedroid BTC task
// and this is read from the loop. Without the reset a second bad client after
// a good one would be silent, leaving only a climbing drop counter - most of
// the way back to the failure this exists to fix.
static bool mtuRefusing = false;

// CCCD per channel, kept so bleEmitFrame() can ask whether the client actually
// subscribed. Parallel to channelChr[]; null for channels with no notify
// property.
static BLE2902 *channelCccd[8] = {nullptr};

// Set while refusing to send because nothing has subscribed - same one-shot
// latch as mtuRefusing, cleared per connection in bleUpdate().
static bool notSubscribed = false;

static BLEServer *pServer = NULL;
// connectTimeMs is written by onConnect() on the Bluedroid task and read by
// bleIsConnected() on the loop - on the OTHER core. deviceConnected is the flag
// that publishes it, so it is a std::atomic with a release store / acquire load:
// a loop that sees true is then guaranteed to see the timestamp stored before
// it. A volatile flag gives no such ordering against a plain store (see the
// batch 3 SPSC ring, same reasoning). This is the one flag-guards-data crossing
// on this stack; the concurrency model is written down in
// docs/architecture-runtime.md.
static unsigned long connectTimeMs = 0;
static std::atomic<bool> deviceConnected{false};
// Loop-only - read and written solely in bleUpdate(), so it needs no
// synchronisation at all. The volatile is vestigial and harmless (ARC-1).
static volatile bool oldDeviceConnected = false;

// Drive the onboard LED: solid when connected, blink when disconnected
static void updateLed() {
  // Blink phase is tracked here rather than read back off the pin.
  //
  // The old form was digitalWrite(pin, !digitalRead(pin)). That works on this
  // board - GPIO2 is a plain output and reads back the level it is driving -
  // so this is a robustness change, not a bug fix. But reading an output pin to
  // decide what to drive it to makes the LED's state live in the pin rather
  // than in us, and read-back does not reflect the driven value on every pin
  // configuration. One bool is cheaper than that dependency.
  //
  // blinkOn is kept in step in the connected branch too. A cache that is only
  // written on one path is exactly how these drift - the same coherence trap
  // that made caching the nRF RGB writes a bad trade (see LAT-4, declined).
  static bool blinkOn = false;

  if (!deviceConnected) {
    static unsigned long lastBlinkMs = 0;
    if (millis() - lastBlinkMs > LED_BLINK_INTERVAL_MS) {
      lastBlinkMs = millis();
      blinkOn = !blinkOn;
      digitalWrite(LED_ONBOARD_PIN, blinkOn ? HIGH : LOW);
    }
  } else {
    digitalWrite(LED_ONBOARD_PIN, HIGH); // solid while a client is attached
    blinkOn = true;
  }
}

// BLE Callbacks
//
// OFF-LOOP CODE. Everything in these callbacks runs on the Bluedroid task, on a
// different core from loop(), truly in parallel with it. This is the only code
// in the firmware that does, and what it may touch is an explicit, short list -
// see "Concurrency" in docs/architecture-runtime.md before adding to it. Rule of
// thumb: a lone flag may cross as a plain/volatile write; anything that a flag
// PUBLISHES needs std::atomic release/acquire, and inbound data goes through the
// rxPush() ring, never straight into shared state.
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    // ORDER MATTERS, and it used to be wrong (ARC-1). The flag was set first and
    // the timestamp after an updatePeerMTU() call into the stack, so a loop on
    // the other core could see "connected" beside the PREVIOUS connection's
    // timestamp - or 0 on first connect - and bleIsConnected() would skip the
    // BLE_CONNECT_SETTLE_MS window entirely. Timestamp first, then publish.
    connectTimeMs = millis();
    deviceConnected.store(true, std::memory_order_release);
    // Request a larger MTU to fit an 88-byte packet + headers in one go
    pServer->updatePeerMTU(pServer->getConnId(), kRequestedMtu);
    LOG_PRINTLN("✅ BLE Client connected & MTU update requested");
  }
  void onDisconnect(BLEServer *pServer) {
    deviceConnected.store(false, std::memory_order_release);
    LOG_PRINTLN("❌ BLE Client disconnected");
  }
};

// One instance per writable channel, each carrying its own channel index -
// which is how a write is attributed to a channel without searching for the
// characteristic pointer.
class ChannelWriteCallbacks : public BLECharacteristicCallbacks {
public:
  explicit ChannelWriteCallbacks(uint8_t channel) : channel_(channel) {}
  void onWrite(BLECharacteristic *pCharacteristic) override {
    // Runs on the Bluedroid BTC task - a DIFFERENT CORE from the loop. It only
    // copies into the ring; protocol code and logging happen in
    // rxDispatchOne() on the loop.
    //
    // getData() points into the stack's own live characteristic buffer, so the
    // copy is also what gives onWrite a defined buffer lifetime.
    const size_t len = pCharacteristic->getLength();
    if (len == 0) {
      return;
    }
    // Discrete transport: each call is exactly one client write, so an
    // oversized one is dropped WHOLE. Truncating would hand the handler part
    // of a message that looks complete, which is worse than losing it. The
    // stream transport splits instead - see the nRF copy.
    if (len > TELEMETRY_MAX_WRITE_LEN ||
        !rxPush(channel_, pCharacteristic->getData(), len)) {
      droppedWrites++;
    }
  }

private:
  uint8_t channel_;
};

// A descriptor names a service or characteristic by 16-bit UUID when it has
// one, and by 128-bit string otherwise. Both stacks need that translated into
// their own UUID type; this is the ESP32 half.
static BLEUUID uuidFor(uint16_t uuid16, const char *uuid128) {
  return uuid16 != 0 ? BLEUUID(uuid16) : BLEUUID(uuid128);
}

void bleBegin() {
  pinMode(LED_ONBOARD_PIN, OUTPUT);
  snprintf(deviceName, sizeof(deviceName), "%s %s", proto->modelName,
           DEVICE_ID);
  BLEDevice::init(deviceName);
  BLEDevice::setPower(BLE_TX_POWER);
  {
    int requestedDbm = (BLE_TX_POWER * 3) - 12;
    int actualDbm = BLEDevice::getPower();
    if (actualDbm == requestedDbm) {
      LOG_PRINTF("✅ BLE TX power set to %d dBm.\n", actualDbm);
    } else {
      LOG_PRINTF("⚠️  BLE TX power mismatch - requested %d dBm, got %d dBm.\n",
                 requestedDbm, actualDbm);
    }
  }
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Build the protocol's service and characteristics from its channel table.
  //
  // TransportKind is deliberately NOT consulted here. On nRF the distinction
  // matters because TRANSPORT_NORDIC_UART hands the job to Bluefruit's BLEUart
  // (which chunks an oversized notify and manages TX backpressure). This stack
  // has no BLEUart and does no chunking - bleEmitFrame is setValue + notify,
  // and a frame the negotiated MTU cannot carry is REFUSED there rather than
  // silently truncated - and builds discrete characteristics for everything.
  // Both kinds are the same code here, so one builder serves every protocol.
  if (proto->channelCount > MAX_CHANNELS) {
    LOG_PRINTF("❌ BLE: protocol declares %u channels, max is %u.\n",
               proto->channelCount, MAX_CHANNELS);
    return;
  }

  // createService()'s handle count must cover every characteristic AND its
  // descriptors, and an undersized value makes the extra characteristics fail
  // to register SILENTLY. Budget the worst case: 1 for the service, then 2 per
  // characteristic (declaration + value) plus 1 for a possible CCCD.
  const uint32_t numHandles = 1 + (uint32_t)proto->channelCount * 3;
  BLEService *pService = pServer->createService(
      uuidFor(proto->serviceUuid16, proto->serviceUuid128), numHandles);

  for (uint8_t i = 0; i < proto->channelCount; i++) {
    const ProtocolChannel &ch = proto->channels[i];
    uint32_t props = 0;
    if (ch.props & PROP_READ)
      props |= BLECharacteristic::PROPERTY_READ;
    if (ch.props & PROP_NOTIFY)
      props |= BLECharacteristic::PROPERTY_NOTIFY;
    if (ch.props & PROP_WRITE)
      props |= BLECharacteristic::PROPERTY_WRITE;
    if (ch.props & PROP_WRITE_NR)
      props |= BLECharacteristic::PROPERTY_WRITE_NR;

    channelChr[i] =
        pService->createCharacteristic(uuidFor(ch.uuid16, ch.uuid128), props);

    // This stack needs the CCCD added explicitly; Bluefruit adds it itself.
    // That asymmetry is exactly what PROP_NOTIFY abstracts away.
    if (ch.props & PROP_NOTIFY) {
      channelCccd[i] = new BLE2902();
      channelChr[i]->addDescriptor(channelCccd[i]);
    }
    if (ch.props & (PROP_WRITE | PROP_WRITE_NR)) {
      channelChr[i]->setCallbacks(new ChannelWriteCallbacks(i));
    }
  }
  pService->start();

  // Device Information Service, values from the descriptor. A protocol that
  // declares no manufacturer (RaceChrono needs no DIS at all) omits the
  // service rather than advertising empty strings.
  BLEService *pDeviceInfo = nullptr;
  if (proto->manufacturer != nullptr) {
    pDeviceInfo =
        pServer->createService("0000180a-0000-1000-8000-00805f9b34fb");

    struct DeviceInfoField {
      const char *uuid;
      const char *value;
    };
    const DeviceInfoField deviceInfoFields[] = {
        {"00002a24-0000-1000-8000-00805f9b34fb", proto->modelName},
        {"00002a25-0000-1000-8000-00805f9b34fb", DEVICE_ID}, // Serial number
        {"00002a29-0000-1000-8000-00805f9b34fb", proto->manufacturer},
        {"00002a26-0000-1000-8000-00805f9b34fb", proto->fwRev},
        {"00002a27-0000-1000-8000-00805f9b34fb", proto->hwRev},
    };
    for (const auto &field : deviceInfoFields) {
      pDeviceInfo
          ->createCharacteristic(field.uuid, BLECharacteristic::PROPERTY_READ)
          ->setValue(field.value);
    }
    pDeviceInfo->start();
  }

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(
      uuidFor(proto->serviceUuid16, proto->serviceUuid128));
  // Advertise Device Information Service so apps can discover the device
  if (pDeviceInfo != nullptr) {
    pAdvertising->addServiceUUID("0000180a-0000-1000-8000-00805f9b34fb");
  }
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  LOG_PRINTLN("📡 BLE advertising started.");
}

bool bleIsConnected() {
  // Give the connection a short pause to ensure that MTU negotiation completes.
  // The acquire pairs with onConnect()'s release store, and the && short-
  // circuits, so connectTimeMs is only ever read AFTER a load has seen the flag
  // that published it - never while a callback could still be writing it.
  return deviceConnected.load(std::memory_order_acquire) &&
         (millis() - connectTimeMs > BLE_CONNECT_SETTLE_MS);
}

bool bleEmitFrame(uint8_t channel, const uint8_t *data, size_t len) {
  if (channel >= proto->channelCount || channelChr[channel] == nullptr) {
    droppedFrames++;
    return false;
  }

  // The const_cast is retained for older cores. As of esp32 3.x the signature
  // is setValue(const uint8_t *, size_t), so the cast is a no-op here - but it
  // was non-const in earlier cores and casting to a const-accepting parameter
  // costs nothing.
  channelChr[channel]->setValue(const_cast<uint8_t *>(data), len);

  // An MTU too small to carry this frame is THE failure mode of this stack, and
  // it is silent without this check: BLECharacteristic::notify() logs
  // "Truncating to N bytes" at a debug level nobody enables and then hands the
  // controller the full length anyway, which cuts at MTU-3. A client that never
  // raises the MTU therefore receives a 20-byte fragment of every packet,
  // forever, with no diagnostic.
  //
  // Polled here rather than cached from bleUpdate(): this runs once per epoch
  // (20/sec) while bleUpdate() runs every loop iteration (>1000/sec), so the
  // apparently-cheaper place is fifty times more expensive. Reading it at the
  // point of use also removes any staleness - notably the spurious refusal
  // that a cache would produce on the first frame after every connection.
  //
  // Written as len + 3 > mtu rather than len > mtu - 3: both operands are
  // unsigned, and the subtraction form underflows for any mtu below 3.
  // A client that connected without writing the CCCD receives nothing. This
  // stack's notify() returns void and cannot report that, so ask the
  // descriptor directly - otherwise the failure is entirely silent here,
  // unlike the nRF where the write at least comes back short.
  if (channelCccd[channel] != nullptr &&
      !channelCccd[channel]->getNotifications()) {
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

  const uint16_t mtu = pServer->getPeerMTU(pServer->getConnId());
  if (len + 3 > mtu) {
    droppedFrames++;
    if (!mtuRefusing) {
      mtuRefusing = true;
      LOG_PRINTF("❌ BLE: peer MTU %u too small for a %u-byte frame (need %u). "
                 "Refusing to send - a truncated packet is worse than none.\n",
                 (unsigned int)mtu, (unsigned int)len, (unsigned int)(len + 3));
    }
    return false;
  }
  if (mtuRefusing) {
    mtuRefusing = false;
    LOG_PRINTF("✅ BLE: peer MTU now %u - sending resumed.\n",
               (unsigned int)mtu);
  }

  // notify() returns void: THIS STACK CANNOT SEE A FAILED SEND. Returning true
  // below therefore means "handed to the stack", not "transmitted" - the
  // weaker of the two guarantees TelemetryEmit documents. Everything this
  // transport can genuinely detect is checked before this point.
  channelChr[channel]->notify();
  return true;
}

uint32_t bleDroppedFrames() { return droppedFrames; }

uint32_t bleDroppedWrites() { return droppedWrites; }

void bleUpdate() {
  // Inbound writes first: this is what makes onWrite a loop-thread call.
  rxDispatchOne();

  // Reset the MTU-refusal latch per connection. Loop-side and semaphore-free:
  // just a read of the volatile flag the callbacks set.
  if (!deviceConnected) {
    mtuRefusing = false;
    notSubscribed = false;
  }

  // BLE connection state management.
  // The re-advertise after a disconnect is deferred  so that it's non-blocking.
  // We record when the disconnect happened and restart advertising on a later
  // loop() pass once the settle delay has elapsed.
  static bool reAdvertisePending = false;
  static unsigned long disconnectMs = 0;

  // Disconnect edge - schedule a re-advertise after the settle delay.
  if (!deviceConnected && oldDeviceConnected) {
    disconnectMs = millis();
    reAdvertisePending = true;
    oldDeviceConnected = deviceConnected;
  }
  // Connect edge - a client is back; cancel any pending re-advertise.
  if (deviceConnected && !oldDeviceConnected) {
    reAdvertisePending = false;
    oldDeviceConnected = deviceConnected;
  }
  // Settle delay has elapsed, restart advertising.
  // Only clear the pending flag on success. On failure, leave it set and reset
  // the timer so we retry after another interval rather than sitting
  // unconnectable until reboot.
  if (reAdvertisePending &&
      millis() - disconnectMs >= BLE_READVERTISE_DELAY_MS) {
    if (BLEDevice::getAdvertising()->start()) {
      LOG_PRINTLN("📡 BLE re-advertising started.");
      reAdvertisePending = false;
    } else {
      LOG_PRINTLN("⚠️  BLE re-advertising failed to start - will retry.");
      disconnectMs = millis();
    }
  }
  updateLed();
}
