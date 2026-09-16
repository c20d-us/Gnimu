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

#include "g_ble_port.h"

#include "config.h"
#include "g_log.h"
#include "g_protocol_active.h"
#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <atomic>

// BLE port for the ESP32 core (Bluedroid). See g_ble_port.h.

// ATT MTU at connect. Only the central can start the exchange, so frames that
// don't fit are refused until it raises the MTU.
static constexpr uint16_t kDefaultMtu = 23;

// Characteristics and CCCDs, indexed by channel.
static constexpr uint8_t kMaxChannels = 8;
static_assert(PROTOCOL_CHANNEL_COUNT <= kMaxChannels,
              "ERROR: the active protocol declares more channels than the "
              "ESP32 BLE port's table holds (kMaxChannels).");
static BLECharacteristic *channelChr[kMaxChannels] = {nullptr};
static BLE2902 *channelCccd[kMaxChannels] = {nullptr};

static BLEServer *pServer = nullptr;

// Callback state
//
// Written by Bluedroid callbacks on the other core. Callbacks only set these
// and call bleRxFromCallback().
static std::atomic<bool> connected{false};
static std::atomic<uint32_t> sessionCount{0};
static std::atomic<uint8_t> disconnectReason{0};
static std::atomic<uint16_t> peerMtu{kDefaultMtu};

// Set by blePortStop() to suppress re-advertising.
static bool stopped = false;

// Map dBm to the named power level. Named constants avoid esp_bt.h's
// misleading compatibility aliases.
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

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    (void)server;
    // Store the MTU before publishing the connection.
    peerMtu.store(kDefaultMtu, std::memory_order_release);
    connected.store(true, std::memory_order_release);
    sessionCount.fetch_add(1, std::memory_order_release);
  }
  void onDisconnect(BLEServer *server,
                    esp_ble_gatts_cb_param_t *param) override {
    (void)server;
    // Store the reason before clearing connected.
    disconnectReason.store(param->disconnect.reason, std::memory_order_release);
    connected.store(false, std::memory_order_release);
  }
  void onMtuChanged(BLEServer *server,
                    esp_ble_gatts_cb_param_t *param) override {
    (void)server;
    // Cached to keep the per-frame check off the stack's peer map.
    peerMtu.store(param->mtu.mtu, std::memory_order_release);
  }
};

// Write callback for one channel.
class ChannelWriteCallbacks : public BLECharacteristicCallbacks {
public:
  explicit ChannelWriteCallbacks(uint8_t channel) : channel_(channel) {}
  void onWrite(BLECharacteristic *pCharacteristic) override {
    // One call per client write; the driver copies the data.
    bleRxFromCallback(channel_, pCharacteristic->getData(),
                      pCharacteristic->getLength(), true);
  }

private:
  uint8_t channel_;
};

// 16-bit UUID if set, else the 128-bit string.
static BLEUUID uuidFor(uint16_t uuid16, const char *uuid128) {
  return uuid16 != 0 ? BLEUUID(uuid16) : BLEUUID(uuid128);
}

// Port interface

bool blePortBegin(const BleIdentity &id, const ProtocolDescriptor *proto) {
  BLEDevice::init(id.name);

  // DEFAULT applies to connections.
  BLEDevice::setPower(powerLevelFor(BLE_TX_POWER_ADV_DBM),
                      ESP_BLE_PWR_TYPE_ADV);
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

  // Build the service from the channel table; both transport kinds are built
  // the same way here.
  //
  // Handles: 1 for the service plus 3 per characteristic (declaration, value,
  // CCCD). Too few makes later characteristics fail silently.
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

    // Bluedroid needs the CCCD added explicitly.
    if (ch.props & PROP_NOTIFY) {
      channelCccd[i] = new BLE2902();
      channelChr[i]->addDescriptor(channelCccd[i]);
    }
    if (ch.props & (PROP_WRITE | PROP_WRITE_NR)) {
      channelChr[i]->setCallbacks(new ChannelWriteCallbacks(i));
    }
  }
  pService->start();

  // Device Information Service.
  BLEService *pDeviceInfo = nullptr;
  if (id.manufacturer != nullptr) {
    pDeviceInfo =
        pServer->createService("0000180a-0000-1000-8000-00805f9b34fb");
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
  return channel < kMaxChannels && channelCccd[channel] != nullptr &&
         channelCccd[channel]->getNotifications();
}

size_t blePortMaxFrame(uint8_t channel) {
  (void)channel;
  // No fragmentation: MTU - 3, guarded against underflow.
  const uint16_t mtu = peerMtu.load(std::memory_order_acquire);
  return mtu > 3 ? (size_t)(mtu - 3) : 0;
}

uint16_t blePortMtu() { return peerMtu.load(std::memory_order_acquire); }

size_t blePortSend(uint8_t channel, const uint8_t *data, size_t len) {
  // notify() returns void, so a failed send can't be detected.
  channelChr[channel]->setValue(data, len);
  channelChr[channel]->notify();
  return len;
}

// Restart advertising BLE_READVERTISE_DELAY_MS after a disconnect, retrying on
// failure.
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
