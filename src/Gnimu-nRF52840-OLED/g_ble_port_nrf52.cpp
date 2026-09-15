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

#include "config.h"
#include "g_battery.h"
#include "g_ble_port.h"
#include "g_log.h"
#include "g_protocol_active.h"
#include <atomic>
#include <bluefruit.h>

// BLE port for the nRF52 core (Bluefruit). See g_ble_port.h.

// Only TRANSPORT_NORDIC_UART is implemented, via BLEUart.
static_assert(PROTOCOL_TRANSPORT == TRANSPORT_NORDIC_UART,
              "ERROR: the nRF52840 BLE port implements TRANSPORT_NORDIC_UART "
              "only; the GATT-channels builder is phase G.");

static BLEUart bleuart;
static BLEDis bledis; // Device Information Service (0x180A)
static BLEBas blebas; // Battery Service (0x180F)

// Callback state
//
// Written by Bluefruit callbacks outside the loop. Callbacks only set these and
// call bleRxFromCallback().
static std::atomic<bool> connected{false};
static std::atomic<uint32_t> sessionCount{0};
static std::atomic<uint8_t> disconnectReason{0};

static void connectCallback(uint16_t conn_handle) {
  (void)conn_handle;
  Bluefruit.setTxPower(BLE_TX_POWER_CONN_DBM);
  // The central starts the MTU exchange; the driver logs the result.
  connected.store(true, std::memory_order_release);
  sessionCount.fetch_add(1, std::memory_order_release);
}

static void disconnectCallback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  // Store the reason before clearing connected.
  disconnectReason.store(reason, std::memory_order_release);
  connected.store(false, std::memory_order_release);
  // Advertising restarts automatically.
  Bluefruit.setTxPower(BLE_TX_POWER_ADV_DBM);
}

// Drain every received byte (or the callback refires) and pass them on as
// stream slices.
static void rxCallback(uint16_t conn_handle) {
  (void)conn_handle;
  uint8_t buf[TELEMETRY_MAX_WRITE_LEN];
  size_t n = 0;
  while (bleuart.available()) {
    buf[n++] = (uint8_t)bleuart.read();
    if (n == sizeof(buf)) {
      bleRxFromCallback(TELEMETRY_CHANNEL_NORDIC_RX, buf, n, false);
      n = 0;
    }
  }
  bleRxFromCallback(TELEMETRY_CHANNEL_NORDIC_RX, buf, n, false);
}

// Port interface

bool blePortBegin(const BleIdentity &id, const ProtocolDescriptor *proto) {
  (void)proto; // BLEUart's GATT is fixed

  // Raise the MTU ceiling. Must precede Bluefruit.begin().
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin(); // 1 peripheral, 0 central
  Bluefruit.setName(id.name);

  // Advertising power here; connected power in connectCallback().
  const bool advOk = Bluefruit.setTxPower(BLE_TX_POWER_ADV_DBM);
  if (advOk) {
    LOG_PRINTF("✅ BLE TX power: advertising %d dBm, connected %d dBm.\n",
               BLE_TX_POWER_ADV_DBM, BLE_TX_POWER_CONN_DBM);
  } else {
    LOG_PRINTF("⚠️  BLE TX power mismatch - requested %d dBm, got %d dBm.\n",
               BLE_TX_POWER_ADV_DBM, (int)Bluefruit.getTxPower());
  }

  Bluefruit.autoConnLed(false); // g_led owns the LED
  Bluefruit.Periph.setConnectCallback(connectCallback);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

  if (id.manufacturer != nullptr) {
    bledis.setManufacturer(id.manufacturer);
    bledis.setModel(id.model);
    bledis.setSerialNum(id.serial);
    bledis.setFirmwareRev(id.fwRev);
    bledis.setHardwareRev(id.hwRev);
    bledis.begin();
  }

  blebas.begin();
  blebas.write(batteryGetStatus().percent);

  bleuart.begin();
  bleuart.setRxCallback(rxCallback);

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  // The name goes in the scan response; the 128-bit UUID nearly fills the
  // advertising packet.
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244); // fast, slow (0.625ms units)
  Bluefruit.Advertising.setFastTimeout(30);   // seconds in fast mode
  Bluefruit.Advertising.start(0);             // no timeout
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
  // Tx is the only notify channel.
  return channel == TELEMETRY_CHANNEL_PRIMARY && bleuart.notifyEnabled();
}

size_t blePortMaxFrame(uint8_t channel) {
  (void)channel;
  // BLEUart fragments, so there is no per-frame limit.
  return (size_t)-1;
}

uint16_t blePortMtu() {
  BLEConnection *conn = Bluefruit.Connection(0);
  return conn != nullptr ? conn->getMtu() : 0;
}

size_t blePortSend(uint8_t channel, const uint8_t *data, size_t len) {
  (void)channel;
  // Returns a byte count: write() stops mid-frame if the notify queue is full.
  return bleuart.write(data, len);
}

void blePortUpdate() {
  // Update the Battery Service only on change.
  static uint8_t lastBasPercent = 0xFF; // force a first write
  const uint8_t pct = batteryGetStatus().percent;
  if (pct != lastBasPercent) {
    blebas.write(pct);
    lastBasPercent = pct;
  }
}

void blePortStop() {
  // Disable restart first, or the disconnects below restart advertising.
  Bluefruit.Advertising.restartOnDisconnect(false);

  for (uint16_t h = 0; h < BLE_MAX_CONNECTION; h++) {
    BLEConnection *c = Bluefruit.Connection(h);
    if (c && c->connected()) {
      c->disconnect();
    }
  }
  Bluefruit.Advertising.stop();
}
