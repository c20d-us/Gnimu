// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// Chip-generic guard, like Clock.cpp: Bluefruit ships with the core for every
// nRF52840 board, and nothing here depends on the XIAO's pinout.
#if defined(NRF52840_XXAA)

#include "BleNrf.h"

#include <Arduino.h>
#include <bluefruit.h>

#include <atomic>

#include "../../app/SystemConfig.h"
#include "../../core/ByteRing.h"
#include "../../platform/Clock.h"

namespace gnimu {

namespace {

// The TX power levels the nRF52840 radio supports.
constexpr bool isNrfTxPower(int dbm) {
  return dbm == -40 || dbm == -20 || dbm == -16 || dbm == -12 || dbm == -8 || dbm == -4 ||
         dbm == 0 || (dbm >= 2 && dbm <= 8);
}
static_assert(isNrfTxPower(config::kBleTxPowerDbm),
              "nRF52840 BLE TX power must be -40, -20, -16, -12, -8, -4, 0 or 2..8 dBm");

// Advertising cadence, in 0.625 ms units: fast (20 ms) for the first
// 30 s so a client finds it quickly, then slow (152.5 ms) to save power.
constexpr uint16_t kAdvFastInterval = 32;
constexpr uint16_t kAdvSlowInterval = 244;
constexpr uint16_t kAdvFastTimeoutS = 30;

uint8_t charCount = 0;
BLECharacteristic* chars[kBleMaxCharacteristics] = {};
// Each writable characteristic's inbound queue, by its position in the list;
// nullptr for the others.
ByteRing<256> rxRings[kBleMaxWritable];
ByteRing<256>* ringOf[kBleMaxCharacteristics] = {};
BLEDis deviceInfo;
BLEBas batteryService;
std::atomic<bool> linkUp{false};
std::atomic<uint16_t> connHandle{BLE_CONN_HANDLE_INVALID};

void onConnect(uint16_t handle) {
  connHandle.store(handle, std::memory_order_relaxed);
  linkUp.store(true, std::memory_order_release);
}

void onDisconnect(uint16_t, uint8_t) { linkUp.store(false, std::memory_order_release); }

// Copy only: parsing and acting on commands happen on the loop. Registered
// only on writable characteristics, so the one written always has a queue.
void onRxWrite(uint16_t, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  for (uint8_t i = 0; i < charCount; i++) {
    if (chars[i] == chr) {
      ringOf[i]->push(data, len);
      return;
    }
  }
}

}  // namespace

bool BleNrf::begin(const BleProfile& p, const char* name, const char* serial, Console& log) {
  link_.setLog(log);
  if (started_) return true;
  if (!built_) {
    // Must precede Bluefruit.begin(): raises the ATT MTU ceiling to 247, so a
    // client that asks for room gets a whole frame in one notification.
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    if (!Bluefruit.begin()) {
      log.log("BLE: stack failed to start \xE2\x9D\x8C");
      return false;
    }
    Bluefruit.setName(name);
    if (Bluefruit.setTxPower(config::kBleTxPowerDbm)) {
      log.log("BLE: TX power %d dBm \xE2\x9C\x85", config::kBleTxPowerDbm);
    } else {
      log.log("BLE: TX power %d dBm rejected \xE2\x9D\x8C", config::kBleTxPowerDbm);
    }
    // StatusLedPort owns the RGB LED; stop Bluefruit lighting blue on its own.
    Bluefruit.autoConnLed(false);
    Bluefruit.Periph.setConnectCallback(onConnect);
    Bluefruit.Periph.setDisconnectCallback(onDisconnect);

    deviceInfo.setManufacturer(p.manufacturer);
    deviceInfo.setModel(p.model);
    deviceInfo.setSerialNum(serial);
    deviceInfo.setFirmwareRev(p.firmwareRevision);
    deviceInfo.setHardwareRev(p.hardwareRevision);
    deviceInfo.begin();
    batteryService.begin();

    // Built from the profile rather than Bluefruit's BLEUart, whose UUIDs are
    // fixed to the Nordic UART service - which only happens to be RaceBox's.
    // Properties, permissions and lengths match BLEUart::begin()'s own setup.
    // A text longer than the MTU is read in pieces by the stack.
    const uint16_t maxMtu = Bluefruit.getMaxMtu(BLE_GAP_ROLE_PERIPH);
    size_t rings = 0;
    charCount = p.charCount;
    for (uint8_t i = 0; i < p.charCount; i++) {
      const BleCharacteristic& d = p.chars[i];
      // The profile lists a service's characteristics together, and Bluefruit
      // adds each characteristic to the service begun most recently.
      if (i == 0 || strcmp(d.serviceUuid, p.chars[i - 1].serviceUuid) != 0) {
        (new BLEService(BLEUuid(d.serviceUuid)))->begin();
      }
      BLECharacteristic* c = new BLECharacteristic(BLEUuid(d.uuid));
      uint8_t props = 0;
      if (d.props & kBleNotify) props |= CHR_PROPS_NOTIFY;
      if (d.props & kBleRead) props |= CHR_PROPS_READ;
      if (d.props & kBleWrite) props |= CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP;
      c->setProperties(props);
      c->setPermission((d.props & (kBleNotify | kBleRead)) ? SECMODE_OPEN : SECMODE_NO_ACCESS,
                       (d.props & kBleWrite) ? SECMODE_OPEN : SECMODE_NO_ACCESS);
      c->setMaxLen(d.text == BleText::None ? maxMtu : kBleTextMaxBytes);
      if (d.props & kBleWrite) {
        ringOf[i] = &rxRings[rings++];
        c->setWriteCallback(onRxWrite, true);
      }
      c->begin();
      if (d.value) c->write(d.value, static_cast<uint16_t>(strlen(d.value)));
      chars[i] = c;
    }

    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addUuid(BLEUuid(p.advertisedServiceUuid));
    // A 128-bit service UUID nearly fills the 31-byte advertising packet, so
    // the full name goes in the scan response.
    Bluefruit.ScanResponse.addName();
    Bluefruit.Advertising.setInterval(kAdvFastInterval, kAdvSlowInterval);
    Bluefruit.Advertising.setFastTimeout(kAdvFastTimeoutS);
    built_ = true;
  }
  started_ = true;
  Bluefruit.Advertising.restartOnDisconnect(true);
  const bool advertising = Bluefruit.Advertising.start(0);  // 0 = no timeout
  log.log("BLE: advertising as \"%s\" %s", name, advertising ? "\xE2\x9C\x85" : "\xE2\x9D\x8C");
  return advertising;
}

void BleNrf::end() {
  if (!started_) return;
  started_ = false;
  // Restart-on-disconnect goes off first, or the disconnect below makes
  // Bluefruit restart advertising, racing the stop().
  Bluefruit.Advertising.restartOnDisconnect(false);
  for (uint16_t h = 0; h < BLE_MAX_CONNECTION; h++) {
    BLEConnection* c = Bluefruit.Connection(h);
    if (c && c->connected()) c->disconnect();
  }
  Bluefruit.Advertising.stop();
}

void BleNrf::update() {
  if (!built_) return;
  link_.edges(linkUp.load(std::memory_order_acquire));
}

bool BleNrf::connected() const { return started_ && linkUp.load(std::memory_order_acquire); }

bool BleNrf::send(uint8_t index, const uint8_t* data, size_t len) {
  if (!connected()) return false;
  const uint16_t handle = connHandle.load(std::memory_order_relaxed);
  BLEConnection* c = Bluefruit.Connection(handle);
  if (!c) return false;
  // Refused rather than split: Bluefruit's notify() would otherwise break a
  // frame across several notifications.
  if (!link_.admit(len, c->getMtu())) return false;
  // False when the peer hasn't enabled notifications, or after waiting up to
  // 100 ms (BLE_GENERIC_TIMEOUT) for a free slot in the send queue.
  const uint64_t start = PlatformClock::nowUs();
  return link_.sent(chars[index]->notify(handle, data, static_cast<uint16_t>(len)), start);
}

void BleNrf::setBatteryPercent(uint8_t percent) {
  if (!built_ || percent == batteryPercent_) return;
  // notify(), not write(): it notifies a subscribed client and also updates
  // the stored value, and falls back to write() when nobody is subscribed.
  batteryService.notify(percent);
  batteryPercent_ = percent;
}

size_t BleNrf::read(uint8_t index, uint8_t* out, size_t max) {
  return index < charCount && ringOf[index] ? ringOf[index]->pop(out, max) : 0;
}

BleStats BleNrf::stats() const {
  BleStats s = link_.stats();
  for (const ByteRing<256>& r : rxRings) {
    s.rxBytes += r.pushed();
    s.rxDropped += r.dropped();
  }
  s.rxBytes -= rxBytesBase_;
  s.rxDropped -= rxDroppedBase_;
  return s;
}

void BleNrf::clearStats() {
  link_.clear();
  rxBytesBase_ = rxDroppedBase_ = 0;
  for (const ByteRing<256>& r : rxRings) {
    rxBytesBase_ += r.pushed();
    rxDroppedBase_ += r.dropped();
  }
}

}  // namespace gnimu

#endif  // NRF52840_XXAA
