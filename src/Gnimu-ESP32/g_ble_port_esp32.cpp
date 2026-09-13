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
// BLE port for the esp32 Arduino core (Bluedroid). Mechanism only - every
// decision is g_ble.cpp's. See g_ble_port.h for the contract.
// ============================================================================

#include "g_ble_port.h"
#include "config.h"
#include <Arduino.h>
#include "g_log.h"
#include "g_protocol_active.h"
#include <atomic>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// Characteristics and their CCCDs, parallel to proto->channels - index IS the
// channel number. The table size is this port's limit, checked at compile time
// against the active protocol rather than discovered at boot.
static constexpr uint8_t kMaxChannels = 8;
static_assert(PROTOCOL_CHANNEL_COUNT <= kMaxChannels,
              "ERROR: the active protocol declares more channels than the "
              "ESP32 BLE port's table holds (kMaxChannels).");
static BLECharacteristic *channelChr[kMaxChannels] = {nullptr};
static BLE2902 *channelCccd[kMaxChannels] = {nullptr};

static BLEServer *pServer = nullptr;

// ----------------------------------------------------------------------------
// TX power
// ----------------------------------------------------------------------------
//
// config.h states it in dBm, as on nRF, and asserts the value is one this part
// accepts. Mapped here to the NAMED enum constants rather than computed, so the
// build does not depend on the enum's numbering - and esp_bt.h's
// backward-compatibility aliases (ESP_PWR_LVL_N14 is really -12 dBm, _P7 really
// +9) can never be how a value gets in.
static constexpr esp_power_level_t powerLevelFor(int dBm) {
  return dBm == -12  ? ESP_PWR_LVL_N12
         : dBm == -9 ? ESP_PWR_LVL_N9
         : dBm == -6 ? ESP_PWR_LVL_N6
         : dBm == -3 ? ESP_PWR_LVL_N3
         : dBm == 0  ? ESP_PWR_LVL_N0
         : dBm == 3  ? ESP_PWR_LVL_P3
         : dBm == 6  ? ESP_PWR_LVL_P6
                     : ESP_PWR_LVL_P9;
}

// ----------------------------------------------------------------------------
// MTU
// ----------------------------------------------------------------------------
//
// The ATT MTU this port asks the central for, derived rather than configured:
// there is exactly one correct class of value - large enough for the biggest
// frame the active protocol emits - so it was never a tunable. +3 is the ATT
// notify header; an exact fit by construction, with no headroom that would
// mask an off-by-one.
//
// It also does more than request. onConnect() records it as the peer's MTU
// straight away, so frames are not refused against the 23-byte default while
// the central's exchange is in flight. Measured 2026-09-13: with this value,
// five fast reconnects from Gnimu Monitor produced no refusal, which is why
// the old BLE_CONNECT_SETTLE_MS delay was deleted.
//
// TO EXERCISE THE REFUSAL PATH: temporarily set this to 23. A client that
// subscribes before the central's exchange completes (Gnimu Monitor's
// reconnects) then sees g_ble's MTU refusal and recovery lines fire. It does
// NOT simulate a central that declines the raise - the CENTRAL drives the
// exchange, and an iOS central negotiated 517 regardless. That path remains
// untested.
static constexpr uint16_t kRequestedMtu = PROTOCOL_MAX_FRAME_LEN + 3;

// ----------------------------------------------------------------------------
// State the callbacks write
// ----------------------------------------------------------------------------
//
// OFF-LOOP. The callbacks below run on the Bluedroid task, on the OTHER core
// from loop(), truly in parallel with it. They only ever set these, and push
// inbound bytes through bleRxFromCallback(). All std::atomic: each is read on
// the loop, on a different core. Orders that matter are commented where the
// stores happen.
static std::atomic<bool> connected{false};
static std::atomic<uint32_t> sessionCount{0};
static std::atomic<uint8_t> disconnectReason{0};
static std::atomic<uint16_t> peerMtu{23};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server, esp_ble_gatts_cb_param_t *param) override {
    // The MTU is recorded BEFORE the connection is published, so a loop that
    // sees the new session never reads the previous one's MTU.
    server->updatePeerMTU(param->connect.conn_id, kRequestedMtu);
    peerMtu.store(server->getPeerMTU(param->connect.conn_id),
                  std::memory_order_release);
    connected.store(true, std::memory_order_release);
    sessionCount.fetch_add(1, std::memory_order_release);
  }
  void onDisconnect(BLEServer *server,
                    esp_ble_gatts_cb_param_t *param) override {
    (void)server;
    // Reason first, so a loop that sees the drop reads this disconnect's code.
    disconnectReason.store(param->disconnect.reason, std::memory_order_release);
    connected.store(false, std::memory_order_release);
  }
  void onMtuChanged(BLEServer *server,
                    esp_ble_gatts_cb_param_t *param) override {
    (void)server;
    // The stack has already written this into its peer map; caching it here
    // keeps the per-frame MTU check off that map's semaphore.
    peerMtu.store(param->mtu.mtu, std::memory_order_release);
  }
};

// One instance per writable channel, carrying its own channel index - which is
// how a write is attributed to a channel without searching for the pointer.
class ChannelWriteCallbacks : public BLECharacteristicCallbacks {
public:
  explicit ChannelWriteCallbacks(uint8_t channel) : channel_(channel) {}
  void onWrite(BLECharacteristic *pCharacteristic) override {
    // getData() points into the stack's live characteristic buffer; the
    // driver copies it. A discrete characteristic: one call is one client
    // write, so it goes in whole or not at all.
    bleRxFromCallback(channel_, pCharacteristic->getData(),
                      pCharacteristic->getLength(), true);
  }

private:
  uint8_t channel_;
};

// A descriptor names a service or characteristic by 16-bit UUID when it has
// one, and by 128-bit string otherwise.
static BLEUUID uuidFor(uint16_t uuid16, const char *uuid128) {
  return uuid16 != 0 ? BLEUUID(uuid16) : BLEUUID(uuid128);
}

// ----------------------------------------------------------------------------
// Port interface
// ----------------------------------------------------------------------------

bool blePortBegin(const BleIdentity &id, const ProtocolDescriptor *proto) {
  BLEDevice::init(id.name);

  // Advertising and connections separately, as on nRF. DEFAULT covers
  // whichever connection handle the central is given.
  BLEDevice::setPower(powerLevelFor(BLE_TX_POWER_ADV_DBM), ESP_BLE_PWR_TYPE_ADV);
  BLEDevice::setPower(powerLevelFor(BLE_TX_POWER_CONN_DBM),
                      ESP_BLE_PWR_TYPE_DEFAULT);
  const int advDbm = BLEDevice::getPower(ESP_BLE_PWR_TYPE_ADV);
  const int connDbm = BLEDevice::getPower(ESP_BLE_PWR_TYPE_DEFAULT);
  if (advDbm == BLE_TX_POWER_ADV_DBM && connDbm == BLE_TX_POWER_CONN_DBM) {
    LOG_PRINTF("✅ BLE TX power: advertising %d dBm, connected %d dBm.\n",
               advDbm, connDbm);
  } else {
    if (advDbm != BLE_TX_POWER_ADV_DBM) {
      LOG_PRINTF("⚠️  BLE TX power mismatch - requested %d dBm, got %d dBm.\n",
                 BLE_TX_POWER_ADV_DBM, advDbm);
    }
    if (connDbm != BLE_TX_POWER_CONN_DBM) {
      LOG_PRINTF("⚠️  BLE TX power mismatch - requested %d dBm, got %d dBm.\n",
                 BLE_TX_POWER_CONN_DBM, connDbm);
    }
  }

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // The protocol's service, built from its channel table. TransportKind is
  // deliberately not consulted: this stack has no BLEUart and no chunking, so
  // TRANSPORT_NORDIC_UART and TRANSPORT_GATT_CHANNELS are the same discrete
  // characteristics here, and one builder serves both.
  //
  // createService()'s handle count must cover every characteristic AND its
  // descriptors; an undersized value makes the extra characteristics fail to
  // register SILENTLY. Budget the worst case: 1 for the service, then 2 per
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

  // Device Information Service, from the identity the driver built.
  BLEService *pDeviceInfo = nullptr;
  if (id.manufacturer != nullptr) {
    pDeviceInfo = pServer->createService("0000180a-0000-1000-8000-00805f9b34fb");
    struct DeviceInfoField {
      const char *uuid;
      const char *value;
    };
    const DeviceInfoField deviceInfoFields[] = {
        {"00002a24-0000-1000-8000-00805f9b34fb", id.model},
        {"00002a25-0000-1000-8000-00805f9b34fb", id.serial},
        {"00002a29-0000-1000-8000-00805f9b34fb", id.manufacturer},
        {"00002a26-0000-1000-8000-00805f9b34fb", id.fwRev},
        {"00002a27-0000-1000-8000-00805f9b34fb", id.hwRev},
    };
    for (const auto &field : deviceInfoFields) {
      pDeviceInfo
          ->createCharacteristic(field.uuid, BLECharacteristic::PROPERTY_READ)
          ->setValue(field.value);
    }
    pDeviceInfo->start();
  }

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(
      uuidFor(proto->serviceUuid16, proto->serviceUuid128));
  if (pDeviceInfo != nullptr) {
    pAdvertising->addServiceUUID("0000180a-0000-1000-8000-00805f9b34fb");
  }
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  return true;
}

bool blePortConnected() { return connected.load(std::memory_order_acquire); }

uint32_t blePortSessionCount() {
  return sessionCount.load(std::memory_order_acquire);
}

uint8_t blePortDisconnectReason() {
  return disconnectReason.load(std::memory_order_acquire);
}

bool blePortSubscribed(uint8_t channel) {
  // This stack's notify() returns void and cannot report an unsubscribed
  // client, so ask the CCCD directly.
  return channel < kMaxChannels && channelCccd[channel] != nullptr &&
         channelCccd[channel]->getNotifications();
}

size_t blePortMaxFrame(uint8_t channel) {
  (void)channel;
  // No fragmentation on this stack: a notify larger than MTU-3 is cut. Guarded
  // against an MTU below 3 rather than letting the subtraction underflow.
  const uint16_t mtu = peerMtu.load(std::memory_order_acquire);
  return mtu > 3 ? (size_t)(mtu - 3) : 0;
}

uint16_t blePortMtu() { return peerMtu.load(std::memory_order_acquire); }

size_t blePortSend(uint8_t channel, const uint8_t *data, size_t len) {
  // notify() returns void: THIS STACK CANNOT SEE A FAILED SEND. "Accepted"
  // therefore means handed to the stack - the weaker of the two guarantees
  // TelemetryEmit documents.
  channelChr[channel]->setValue(data, len);
  channelChr[channel]->notify();
  return len;
}

// Re-advertising after a disconnect, deferred so the loop never blocks on it.
// The restart is retried rather than abandoned, so a failed start does not
// leave the device unconnectable until reboot.
static bool stopped = false;

void blePortUpdate() {
  static bool wasConnected = false;
  static bool reAdvertisePending = false;
  static unsigned long disconnectMs = 0;
  const bool now = connected.load(std::memory_order_acquire);

  if (!now && wasConnected && !stopped) {
    disconnectMs = millis();
    reAdvertisePending = true;
  }
  if (now) {
    reAdvertisePending = false;
  }
  wasConnected = now;

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
}

void blePortStop() {
  stopped = true;
  if (connected.load(std::memory_order_acquire)) {
    pServer->disconnect(pServer->getConnId());
  }
  BLEDevice::stopAdvertising();
}
