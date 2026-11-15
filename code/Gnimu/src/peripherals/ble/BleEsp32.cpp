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

// Room for the 140-byte Gnimu frame and more; iOS asks for 185.
constexpr uint16_t kLocalMtu = 247;

constexpr char kDeviceInfoUuid[] = "0000180a-0000-1000-8000-00805f9b34fb";

// Room for every characteristic of a profile in one service: the declaration
// and value handles, plus a CCCD for a notify one. Bluedroid's default of 15
// is too few for six.
constexpr uint32_t kServiceHandles = 1 + 3 * kBleMaxCharacteristics;

BLEServer* server = nullptr;
uint8_t charCount = 0;
BLECharacteristic* chars[kBleMaxCharacteristics] = {};
// Each writable characteristic's inbound queue, by its position in the list;
// nullptr for the others.
ByteRing<256> rxRings[kBleMaxWritable];
ByteRing<256>* ringOf[kBleMaxCharacteristics] = {};
std::atomic<bool> linkUp{false};
std::atomic<uint16_t> peerMtu{23};  // ATT default until the exchange raises it

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

// One object per characteristic, for both directions: the library keeps a
// single callbacks pointer, so a characteristic that notifies and is written
// to needs both handled by the same object.
class CharCallbacks : public BLECharacteristicCallbacks {
 public:
  // `ring` receives the client's writes; nullptr for one that isn't writable.
  explicit CharCallbacks(ByteRing<256>* ring) : ring_(ring) {}

 private:
  void onStatus(BLECharacteristic*, Status s, uint32_t) override { lastNotifyStatus = s; }
  // Copy only: parsing and acting on commands happen on the loop.
  void onWrite(BLECharacteristic* c) override {
    if (ring_) ring_->push(c->getData(), c->getLength());
  }
  ByteRing<256>* ring_;
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

    // Lets a client that asks for it carry a whole frame in one notification;
    // without it the ESP32 keeps the 23-byte default.
    if (BLEDevice::setMTU(kLocalMtu) != ESP_OK) {
      log.log("BLE: MTU %u not accepted \xE2\x9D\x8C", static_cast<unsigned>(kLocalMtu));
    }

    server = BLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());

    // The profile lists a service's characteristics together, so each service
    // is started once its last one has been added.
    BLEService* service = nullptr;
    size_t rings = 0;
    charCount = p.charCount;
    for (uint8_t i = 0; i < p.charCount; i++) {
      const BleCharacteristic& d = p.chars[i];
      if (i == 0 || strcmp(d.serviceUuid, p.chars[i - 1].serviceUuid) != 0) {
        if (service) service->start();
        service = server->createService(BLEUUID(d.serviceUuid), kServiceHandles);
      }
      uint32_t props = 0;
      if (d.props & kBleNotify) props |= BLECharacteristic::PROPERTY_NOTIFY;
      if (d.props & kBleRead) props |= BLECharacteristic::PROPERTY_READ;
      if (d.props & kBleWrite) {
        props |= BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR;
      }
      BLECharacteristic* c = service->createCharacteristic(d.uuid, props);
      if (d.props & kBleNotify) c->addDescriptor(new BLE2902());
      if (d.props & kBleWrite) ringOf[i] = &rxRings[rings++];
      if (d.props & (kBleNotify | kBleWrite)) c->setCallbacks(new CharCallbacks(ringOf[i]));
      if (d.value) c->setValue(reinterpret_cast<uint8_t*>(const_cast<char*>(d.value)), strlen(d.value));
      chars[i] = c;
    }
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
    adv->addServiceUUID(p.advertisedServiceUuid);
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

bool BleEsp32::send(uint8_t index, const uint8_t* data, size_t len) {
  if (!connected()) return false;
  // A notification carries MTU - 3 bytes; anything longer would arrive cut short.
  if (!link_.admit(len, peerMtu.load(std::memory_order_acquire))) return false;
  // Anything but SUCCESS_NOTIFY - notifications off, no peer, the stack
  // refusing the frame - counts as failed. Congestion isn't visible here: it
  // arrives later as a separate stack event.
  const uint64_t start = PlatformClock::nowUs();
  lastNotifyStatus = NotifyStatus::ERROR_NO_CLIENT;
  chars[index]->setValue(data, len);
  chars[index]->notify();
  return link_.sent(lastNotifyStatus == NotifyStatus::SUCCESS_NOTIFY, start);
}

size_t BleEsp32::read(uint8_t index, uint8_t* out, size_t max) {
  return index < charCount && ringOf[index] ? ringOf[index]->pop(out, max) : 0;
}

BleStats BleEsp32::stats() const {
  BleStats s = link_.stats();
  for (const ByteRing<256>& r : rxRings) {
    s.rxBytes += r.pushed();
    s.rxDropped += r.dropped();
  }
  s.rxBytes -= rxBytesBase_;
  s.rxDropped -= rxDroppedBase_;
  return s;
}

void BleEsp32::clearStats() {
  link_.clear();
  rxBytesBase_ = rxDroppedBase_ = 0;
  for (const ByteRing<256>& r : rxRings) {
    rxBytesBase_ += r.pushed();
    rxDroppedBase_ += r.dropped();
  }
}

}  // namespace gnimu

#endif  // ARDUINO_ARCH_ESP32
