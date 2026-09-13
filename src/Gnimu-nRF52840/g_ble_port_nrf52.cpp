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
// BLE port for the Seeed/Adafruit nRF52 core (Bluefruit). Mechanism only -
// every decision is g_ble.cpp's. See g_ble_port.h for the contract.
// ============================================================================

#include "g_ble_port.h"
#include "config.h"
#include "g_battery.h"
#include "g_log.h"
#include "g_protocol_active.h"
#include <atomic>
#include <bluefruit.h>

// This port builds TRANSPORT_NORDIC_UART only, with Bluefruit's BLEUart, which
// fragments an oversized notify and manages TX backpressure itself - the
// reason that transport kind exists (docs/multiprotocol-design.md 6.1). The
// GATT-channels builder arrives in phase G; until then a protocol needing it is
// a compile error rather than a device that advertises but serves nothing.
static_assert(PROTOCOL_TRANSPORT == TRANSPORT_NORDIC_UART,
              "ERROR: the nRF52840 BLE port implements TRANSPORT_NORDIC_UART "
              "only; the GATT-channels builder is phase G.");

// Nordic UART Service. Its UUIDs are exactly the RaceBox service/Tx/Rx UUIDs,
// which API-4's nordicUartShapeOk() asserts at compile time.
static BLEUart bleuart;
static BLEDis bledis; // Device Information Service (0x180A)
static BLEBas blebas; // Battery Service (0x180F)

// ----------------------------------------------------------------------------
// State the callbacks write
// ----------------------------------------------------------------------------
//
// OFF-LOOP. The callbacks below run in Bluefruit's callback task - preemption
// rather than true parallelism on this single-core part, but still outside the
// cooperative loop. They only ever set these, and push inbound bytes through
// bleRxFromCallback(). std::atomic to the same pattern as the ESP32 port, where
// the reason matters more (other core); costs nothing here.
static std::atomic<bool> connected{false};
static std::atomic<uint32_t> sessionCount{0};
static std::atomic<uint8_t> disconnectReason{0};

static void connectCallback(uint16_t conn_handle) {
  (void)conn_handle;
  // Switch to the connected TX power level (see BLE_TX_POWER_* in config.h).
  Bluefruit.setTxPower(BLE_TX_POWER_CONN_DBM);
  // The CENTRAL drives the MTU exchange; the ceiling is already raised by
  // configPrphBandwidth(BANDWIDTH_MAX). The driver polls blePortMtu() and logs
  // the rise once the central makes it.
  connected.store(true, std::memory_order_release);
  sessionCount.fetch_add(1, std::memory_order_release);
}

static void disconnectCallback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  // Reason first, so a loop that sees the drop reads this disconnect's code.
  disconnectReason.store(reason, std::memory_order_release);
  connected.store(false, std::memory_order_release);
  // Restore advertising TX power; advertising auto-restarts.
  Bluefruit.setTxPower(BLE_TX_POWER_ADV_DBM);
}

// Bytes written by the client on the Rx characteristic.
//
// It MUST drain the UART, or the buffer stays non-empty and the callback
// refires forever. BLEUart is a BYTE STREAM, so the drain is handed over in
// slices and the driver splits and queues them (wholeMessage = false): this
// transport never promised message boundaries, and onWrite documents that.
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

// ----------------------------------------------------------------------------
// Port interface
// ----------------------------------------------------------------------------

bool blePortBegin(const BleIdentity &id, const ProtocolDescriptor *proto) {
  (void)proto; // BLEUart builds its own fixed GATT; the shape is asserted above

  // Raise the ATT MTU ceiling so an 88-byte notify fits in one packet.
  // Must be called BEFORE Bluefruit.begin() to take effect.
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
  Bluefruit.begin(); // 1 peripheral, 0 central (defaults)
  Bluefruit.setName(id.name);

  // Advertising power now, connected power in connectCallback(). config.h
  // asserts both are levels this part implements; setTxPower() still reports
  // whether it took each, and this stack cannot read the level back.
  const bool advOk = Bluefruit.setTxPower(BLE_TX_POWER_ADV_DBM);
  if (advOk) {
    LOG_PRINTF("✅ BLE TX power: advertising %d dBm, connected %d dBm.\n",
               BLE_TX_POWER_ADV_DBM, BLE_TX_POWER_CONN_DBM);
  } else {
    LOG_PRINTF("⚠️  BLE TX power mismatch - requested %d dBm, got %d dBm.\n",
               BLE_TX_POWER_ADV_DBM, (int)Bluefruit.getTxPower());
  }

  // g_led owns the RGB LED; stop Bluefruit toggling the onboard LED itself.
  Bluefruit.autoConnLed(false);
  Bluefruit.Periph.setConnectCallback(connectCallback);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

  // Device Information Service, from the identity the driver built.
  if (id.manufacturer != nullptr) {
    bledis.setManufacturer(id.manufacturer);
    bledis.setModel(id.model);
    bledis.setSerialNum(id.serial);
    bledis.setFirmwareRev(id.fwRev);
    bledis.setHardwareRev(id.hwRev);
    bledis.begin();
  }

  // Battery Service (reflects the real cell state of charge).
  blebas.begin();
  blebas.write(batteryGetStatus().percent);

  bleuart.begin();
  bleuart.setRxCallback(rxCallback);

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
  // One stream: the Tx characteristic is the only notify channel. A client
  // that never wrote its CCCD receives nothing - BLECharacteristic::notify()
  // gates its whole send loop on this.
  return channel == TELEMETRY_CHANNEL_PRIMARY && bleuart.notifyEnabled();
}

size_t blePortMaxFrame(uint8_t channel) {
  (void)channel;
  // BLEUart splits a frame larger than MTU-3 across notifies, and the RaceBox
  // app reassembles the UBX stream. Keeping that is why BLEUart was not
  // replaced here - so this stack imposes no per-frame limit.
  return (size_t)-1;
}

uint16_t blePortMtu() {
  BLEConnection *conn = Bluefruit.Connection(0);
  return conn != nullptr ? conn->getMtu() : 0;
}

size_t blePortSend(uint8_t channel, const uint8_t *data, size_t len) {
  (void)channel;
  // The return value is a BYTE COUNT and it matters. With _tx_buffered off (the
  // default), write() forwards to a notify that chunks to MTU-3 in a loop and
  // bails mid-loop if the SoftDevice's notify queue is exhausted - so whatever
  // chunks already went out stay out, and a partial UBX packet enters a stream
  // the app then has to resynchronise from. A short count is the only signal.
  return bleuart.write(data, len);
}

void blePortUpdate() {
  // Keep the Battery Service in step with the cell, but only on a real change
  // (avoids a needless notify every loop).
  static uint8_t lastBasPercent = 0xFF; // force a first write
  const uint8_t pct = batteryGetStatus().percent;
  if (pct != lastBasPercent) {
    blebas.write(pct);
    lastBasPercent = pct;
  }
}

void blePortStop() {
  // Turn OFF restart-on-disconnect FIRST. Otherwise Bluefruit's internal
  // disconnect handler (triggered by the disconnect() calls below) fires
  // Advertising.start() before - or racing with - our own Advertising.stop(),
  // and the device stays advertising even though we asked it to hush.
  Bluefruit.Advertising.restartOnDisconnect(false);

  // Disconnect any active connection so the client sees a clean link end
  // rather than a silent, indefinitely-hanging one. Only handle 0 is ever
  // populated in this peripheral-only, single-central configuration, but
  // iterating to BLE_MAX_CONNECTION costs nothing.
  for (uint16_t h = 0; h < BLE_MAX_CONNECTION; h++) {
    BLEConnection *c = Bluefruit.Connection(h);
    if (c && c->connected()) {
      c->disconnect();
    }
  }
  // Vanish from BLE scans.
  Bluefruit.Advertising.stop();
}
