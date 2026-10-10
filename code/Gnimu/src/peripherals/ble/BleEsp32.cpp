// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#if defined(ARDUINO_ARCH_ESP32)

#include "BleEsp32.h"

#include <Arduino.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>

#include <atomic>

#include "../../app/SystemConfig.h"
#include "../../core/ByteRing.h"
#include "../../platform/Clock.h"

namespace gnimu {

namespace {

// ESP_PWR_LVL_N12 (0) through ESP_PWR_LVL_P9 (7) in 3 dB steps.
static_assert(config::kBleTxPowerDbm >= -12 && config::kBleTxPowerDbm <= 9 &&
                  (config::kBleTxPowerDbm + 12) % 3 == 0,
              "ESP32 BLE TX power must be -12..+9 dBm in 3 dB steps");
constexpr esp_power_level_t kTxPower =
    static_cast<esp_power_level_t>((config::kBleTxPowerDbm + 12) / 3);

constexpr char kDeviceInfoUuid[] = "0000180a-0000-1000-8000-00805f9b34fb";

BLEServer* server = nullptr;
BLECharacteristic* tx = nullptr;
std::atomic<bool> linkUp{false};
std::atomic<uint16_t> peerMtu{23};  // ATT default until the exchange raises it
ByteRing<256> rxRing;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    // Reset the MTU before publishing the link, so send() never pairs a new
    // connection with the previous peer's MTU.
    peerMtu.store(23, std::memory_order_release);
    linkUp.store(true, std::memory_order_release);
  }
  void onDisconnect(BLEServer*) override { linkUp.store(false, std::memory_order_release); }
  void onMtuChanged(BLEServer*, esp_ble_gatts_cb_param_t* param) override {
    peerMtu.store(param->mtu.mtu, std::memory_order_release);
  }
};

// notify() returns nothing; it reports what it did through onStatus(), called
// synchronously from inside notify() on the caller's (loop) task.
using NotifyStatus = BLECharacteristicCallbacks::Status;
NotifyStatus lastNotifyStatus = NotifyStatus::ERROR_NO_CLIENT;

class TxCallbacks : public BLECharacteristicCallbacks {
  void onStatus(BLECharacteristic*, Status s, uint32_t) override { lastNotifyStatus = s; }
};

class RxCallbacks : public BLECharacteristicCallbacks {
  // Copy only: parsing and acting on commands happen on the loop.
  void onWrite(BLECharacteristic* c) override { rxRing.push(c->getData(), c->getLength()); }
};

}  // namespace

bool BleEsp32::begin(const BleProfile& p, const char* name, const char* serial, Console& log) {
  link_.setLog(log);
  if (started_) return true;
  if (!built_) {
    // Nothing below works on a stack that didn't start.
    if (!BLEDevice::init(name)) {
      log.log("BLE: stack failed to start \xE2\x9D\x8C");
      return false;
    }
    // Set both: ESP_BLE_PWR_TYPE_DEFAULT reaches connections but not
    // advertising, which would otherwise stay at the controller's +9 dBm.
    BLEDevice::setPower(kTxPower, ESP_BLE_PWR_TYPE_ADV);
    BLEDevice::setPower(kTxPower, ESP_BLE_PWR_TYPE_DEFAULT);
    const int advDbm = BLEDevice::getPower(ESP_BLE_PWR_TYPE_ADV);
    const int connDbm = BLEDevice::getPower(ESP_BLE_PWR_TYPE_DEFAULT);
    if (advDbm == config::kBleTxPowerDbm && connDbm == config::kBleTxPowerDbm) {
      log.log("BLE: TX power %d dBm (advertising and connected) \xE2\x9C\x85", config::kBleTxPowerDbm);
    } else {
      log.log("BLE: TX power requested %d dBm, got advertising %d / connected %d \xE2\x9D\x8C",
              config::kBleTxPowerDbm, advDbm, connDbm);
    }

    server = BLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());

    BLEService* service = server->createService(p.serviceUuid);
    tx = service->createCharacteristic(p.txUuid, BLECharacteristic::PROPERTY_NOTIFY);
    tx->addDescriptor(new BLE2902());
    tx->setCallbacks(new TxCallbacks());
    BLECharacteristic* rx = service->createCharacteristic(
        p.rxUuid, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    rx->setCallbacks(new RxCallbacks());
    service->start();

    struct Field {
      const char* uuid;
      const char* value;
    };
    const Field fields[] = {
        {"00002a24-0000-1000-8000-00805f9b34fb", p.model},
        {"00002a25-0000-1000-8000-00805f9b34fb", serial},
        {"00002a29-0000-1000-8000-00805f9b34fb", p.manufacturer},
        {"00002a26-0000-1000-8000-00805f9b34fb", p.firmwareRevision},
        {"00002a27-0000-1000-8000-00805f9b34fb", p.hardwareRevision},
    };
    BLEService* info = server->createService(kDeviceInfoUuid);
    for (const Field& f : fields)
      info->createCharacteristic(f.uuid, BLECharacteristic::PROPERTY_READ)->setValue(f.value);
    info->start();

    BLEAdvertising* adv = BLEDevice::getAdvertising();
    adv->addServiceUUID(p.serviceUuid);
    adv->addServiceUUID(kDeviceInfoUuid);
    adv->setScanResponse(true);
    built_ = true;
  }
  started_ = true;
  // The library restarts advertising from its own disconnect event, so a
  // connect and disconnect between two update() passes can't strand it.
  server->advertiseOnDisconnect(true);
  const bool advertising = BLEDevice::getAdvertising()->start();
  log.log("BLE: advertising as \"%s\" %s", name, advertising ? "\xE2\x9C\x85" : "\xE2\x9D\x8C");
  return advertising;
}

void BleEsp32::end() {
  if (!started_) return;
  started_ = false;
  // Off before the disconnect below, or its event would restart advertising.
  server->advertiseOnDisconnect(false);
  BLEDevice::getAdvertising()->stop();
  if (server->getConnectedCount() > 0) server->disconnect(server->getConnId());
}

void BleEsp32::update() {
  if (!built_) return;
  link_.edges(linkUp.load(std::memory_order_acquire));
}

bool BleEsp32::connected() const { return started_ && linkUp.load(std::memory_order_acquire); }

bool BleEsp32::send(const uint8_t* data, size_t len) {
  if (!connected()) return false;
  // A notification carries MTU - 3 bytes; anything longer would arrive cut short.
  if (!link_.admit(len, peerMtu.load(std::memory_order_acquire))) return false;
  // Anything but SUCCESS_NOTIFY - notifications off, no peer, the stack
  // refusing the frame - counts as failed. Congestion isn't visible here: it
  // arrives later as a separate stack event.
  const uint64_t start = PlatformClock::nowUs();
  lastNotifyStatus = NotifyStatus::ERROR_NO_CLIENT;
  tx->setValue(data, len);
  tx->notify();
  return link_.sent(lastNotifyStatus == NotifyStatus::SUCCESS_NOTIFY, start);
}

size_t BleEsp32::read(uint8_t* out, size_t max) { return rxRing.pop(out, max); }
BleStats BleEsp32::stats() const {
  BleStats s = link_.stats();
  s.rxBytes = rxRing.pushed();
  s.rxDropped = rxRing.dropped();
  return s;
}

}  // namespace gnimu

#endif  // ARDUINO_ARCH_ESP32
