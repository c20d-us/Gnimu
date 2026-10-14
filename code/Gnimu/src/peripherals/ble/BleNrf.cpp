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

BLEService* service = nullptr;
BLECharacteristic* tx = nullptr;
BLECharacteristic* rx = nullptr;
BLEDis deviceInfo;
BLEBas batteryService;
std::atomic<bool> linkUp{false};
std::atomic<uint16_t> connHandle{BLE_CONN_HANDLE_INVALID};
ByteRing<256> rxRing;

void onConnect(uint16_t handle) {
  connHandle.store(handle, std::memory_order_relaxed);
  linkUp.store(true, std::memory_order_release);
}

void onDisconnect(uint16_t, uint8_t) { linkUp.store(false, std::memory_order_release); }

// Copy only: parsing and acting on commands happen on the loop.
void onRxWrite(uint16_t, BLECharacteristic*, uint8_t* data, uint16_t len) { rxRing.push(data, len); }

}  // namespace

bool BleNrf::begin(const BleProfile& p, const char* name, const char* serial, Console& log) {
  link_.setLog(log);
  if (started_) return true;
  if (!built_) {
    // Must precede Bluefruit.begin(): raises the ATT MTU ceiling to 247, so a
    // client that asks for room gets an 88-byte frame in one notification.
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
    const uint16_t maxMtu = Bluefruit.getMaxMtu(BLE_GAP_ROLE_PERIPH);
    service = new BLEService(BLEUuid(p.serviceUuid));
    service->begin();
    tx = new BLECharacteristic(BLEUuid(p.txUuid));
    tx->setProperties(CHR_PROPS_NOTIFY);
    tx->setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    tx->setMaxLen(maxMtu);
    tx->begin();
    rx = new BLECharacteristic(BLEUuid(p.rxUuid));
    rx->setProperties(CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
    rx->setPermission(SECMODE_NO_ACCESS, SECMODE_OPEN);
    rx->setMaxLen(maxMtu);
    rx->setWriteCallback(onRxWrite, true);
    rx->begin();

    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addService(*service);
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

bool BleNrf::send(const uint8_t* data, size_t len) {
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
  return link_.sent(tx->notify(handle, data, static_cast<uint16_t>(len)), start);
}

void BleNrf::setBatteryPercent(uint8_t percent) {
  if (!built_ || percent == batteryPercent_) return;
  // notify(), not write(): it notifies a subscribed client and also updates
  // the stored value, and falls back to write() when nobody is subscribed.
  batteryService.notify(percent);
  batteryPercent_ = percent;
}

size_t BleNrf::read(uint8_t* out, size_t max) { return rxRing.pop(out, max); }
BleStats BleNrf::stats() const {
  BleStats s = link_.stats();
  s.rxBytes = rxRing.pushed();
  s.rxDropped = rxRing.dropped();
  return s;
}

}  // namespace gnimu

#endif  // NRF52840_XXAA
