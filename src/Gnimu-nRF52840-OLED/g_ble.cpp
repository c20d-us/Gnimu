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
#include <bluefruit.h>

// BLE state - private to this file
// Adjacent string literals concatenate at compile time - no heap, no
// static-init-order concerns.
static const char deviceName[] = RACEBOX_MODEL " " DEVICE_ID;

// Nordic UART Service. Its UUIDs are exactly the RaceBox service/Tx/Rx UUIDs
// (RACEBOX_*_UUID in config.h), so BLEUart implements the RaceBox transport
// natively. Tx = notify, Rx = write.
static BLEUart bleuart;
static BLEDis bledis; // Device Information Service (0x180A)
static BLEBas blebas; // Battery Service (0x180F)

static volatile bool deviceConnected = false;
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

// Debug: print any bytes the client writes to the Rx characteristic.
static void rxCallback(uint16_t conn_handle) {
  (void)conn_handle;
  if (!bleuart.available())
    return;
  LOG_PRINT("📨 Received BLE command: ");
  while (bleuart.available()) {
    // The read must live OUTSIDE the LOG macro: in silent builds the macro
    // (and its arguments) vanish at preprocessor level, and losing the
    // read() side effect here would leave the buffer forever non-empty -
    // an infinite loop inside a BLE callback.
    const uint8_t b = (uint8_t)bleuart.read();
    (void)b; // silence the unused warning in silent builds
    LOG_PRINTF("0x%02X ", b);
  }
  LOG_PRINTLN();
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
  Bluefruit.setName(deviceName);
  Bluefruit.setTxPower(BLE_TX_POWER_ADV_DBM);
  LOG_PRINTF("✅ BLE TX power set to %d dBm (advertising).\n",
             BLE_TX_POWER_ADV_DBM);
  // g_led owns the RGB LED; stop Bluefruit toggling the onboard LED itself.
  Bluefruit.autoConnLed(false);
  Bluefruit.Periph.setConnectCallback(connectCallback);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

  // Device Information Service
  bledis.setManufacturer(RACEBOX_MANUFACTURER);
  bledis.setModel(RACEBOX_MODEL);
  bledis.setSerialNum(DEVICE_ID);
  bledis.setFirmwareRev(RACEBOX_FIRMWARE_VERSION);
  bledis.setHardwareRev(RACEBOX_HARDWARE_VERSION);
  bledis.begin();

  // Battery Service (reflects the real cell state of charge)
  blebas.begin();
  blebas.write(batteryGetStatus().percent);

  // Nordic UART / RaceBox service
  bleuart.begin();
  bleuart.setRxCallback(rxCallback);

  startAdvertising();

  bleInitialized = true;
}

bool bleIsConnected() { return deviceConnected; }

void bleSendPacket(uint8_t *data, size_t len) {
  // Notifies the Tx characteristic. If the MTU is smaller than len, BLEUart
  // splits it across notifications; the RaceBox app reassembles the UBX stream.
  bleuart.write(data, len);
}

void bleUpdate() {
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
