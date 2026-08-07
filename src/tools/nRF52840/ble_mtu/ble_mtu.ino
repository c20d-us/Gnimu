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
// DIAGNOSTIC: XIAO nRF52840 Sense BLE + MTU bring-up
//
// A minimal Bluefruit peripheral that mirrors ble.cpp's setup, to validate:
//   - Bluefruit advertises under the device name (MODEL + " " + DEVICE_ID).
//   - Bluefruit.setTxPower() accepts the config.h BLE_TX_POWER level.
//   - configPrphBandwidth(BANDWIDTH_MAX) raises the MTU *ceiling* (to 247) AND
//     the connect callback calls requestMtuExchange(247) to actually initiate
//     the negotiation from the peripheral side - required because some centrals
//     (iOS / nRF Connect) never start it, leaving the MTU at the 23 default.
//     Together they let an 88-byte notify ride in a single packet.
//   - BLEUart.write() of an 88-byte buffer works (BLEUart implements the Nordic
//     UART service, whose UUIDs equal the RaceBox service/Tx/Rx UUIDs).
//   - restartOnDisconnect(true) resumes advertising with no manual bookkeeping.
//
// How to run:
//   1. Flash, open serial monitor (115200).
//   2. Connect with nRF Connect (or similar) on a phone.
//   3. Watch the "MTU is now N" lines (reported from loop() as the negotiation
//      settles - not just at connect, which is too early to read the final
//      value). N must reach >= 91. iOS centrals usually auto-negotiate ~185; on
//      Android/nRF Connect you may need to request it: menu (three dots) ->
//      Request MTU -> 247.
//   4. Subscribe to the Tx characteristic to see the 88-byte test notify.
//
// The constants below mirror config.h; keep them in sync if you change config.
//
// Requires: XIAO nRF52840 Sense + USB + a phone running nRF Connect.
// ============================================================================

#include <bluefruit.h>

#define TEST_DEVICE_NAME "RaceBox Mini 0123456789" // MODEL + " " + DEVICE_ID
#define TEST_TX_POWER -12 // config.h BLE_TX_POWER (dBm)

static BLEUart bleuart;
static uint16_t g_connHandle = BLE_CONN_HANDLE_INVALID;

static void connectCallback(uint16_t conn_handle) {
  g_connHandle = conn_handle;
  BLEConnection *conn = Bluefruit.Connection(conn_handle);

  // WHO connected? A valid 6-byte peer address = a real central really
  // connected (most likely this Mac if its Bluetooth / LightBlue is on).
  // addr_type: 0 = public, 1/2 = random (Apple devices rotate these).
  // If instead nothing/garbage shows here with all radios truly off, that would
  // point at the board rather than a stray central.
  ble_gap_addr_t peer = conn->getPeerAddr();
  Serial.printf(
      "Connected. peer = %02X:%02X:%02X:%02X:%02X:%02X (addr_type %u), "
      "initial MTU = %u.\n",
      peer.addr[5], peer.addr[4], peer.addr[3], peer.addr[2], peer.addr[1],
      peer.addr[0], peer.addr_type, conn->getMtu());

  // NOTE: we deliberately do NOT call requestMtuExchange() here. Testing showed
  // a peripheral-initiated MTU exchange in the connect callback collides with
  // the central's own connection setup (its MTU exchange + service discovery),
  // which deadlocks the ATT bearer - macOS/LightBlue then abandons the link
  // after ~10s. The MTU ceiling is already raised by
  // configPrphBandwidth(BANDWIDTH_MAX); let the central drive the negotiation.
  // loop() reports the settled value.
}

static void disconnectCallback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  g_connHandle = BLE_CONN_HANDLE_INVALID;
  Serial.printf("Disconnected (reason 0x%02X). Advertising will resume.\n",
                reason);
}

void setup() {
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
  }
  Serial.println("\n=== XIAO BLE + MTU bring-up [build: BANDWIDTH_MAX] ===");

  // Must precede Bluefruit.begin(): raise the ATT MTU ceiling above the 23-byte
  // default so an 88-byte notify can fit in one packet. Use the library preset
  // (as every Bluefruit peripheral example does) rather than hand-rolled
  // configPrphConn params - BANDWIDTH_MAX = configPrphConn(247, 100, 3, ...),
  // a matched set. The earlier hand-rolled call left the MTU stuck at 23.
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setName(TEST_DEVICE_NAME);
  Bluefruit.setTxPower(TEST_TX_POWER);
  Bluefruit.autoConnLed(false);
  Bluefruit.Periph.setConnectCallback(connectCallback);
  Bluefruit.Periph.setDisconnectCallback(disconnectCallback);

  bleuart.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addService(bleuart);
  Bluefruit.ScanResponse.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0);

  Serial.printf("Advertising as \"%s\" at %d dBm.\n", TEST_DEVICE_NAME,
                TEST_TX_POWER);
}

void loop() {
  static uint16_t lastMtu = 0;
  if (g_connHandle == BLE_CONN_HANDLE_INVALID) {
    lastMtu = 0;
    return;
  }
  BLEConnection *conn = Bluefruit.Connection(g_connHandle);
  if (conn == NULL) {
    return;
  }

  // Report the MTU whenever it changes. The exchange completes shortly after
  // connect, so the final negotiated value shows up here, not in the callback.
  const uint16_t mtu = conn->getMtu();
  if (mtu != lastMtu) {
    lastMtu = mtu;
    Serial.printf("MTU is now %u  %s\n", mtu,
                  mtu >= 91
                      ? "-> OK (>= 91): an 88-byte packet fits in one notify"
                      : "-> still low");
  }

  // Every 2 s, send an 88-byte test notify (0..87) to exercise BLEUart.write.
  static uint32_t lastMs = 0;
  if (millis() - lastMs > 2000) {
    lastMs = millis();
    uint8_t buf[88];
    for (int i = 0; i < 88; i++) {
      buf[i] = (uint8_t)i;
    }
    bleuart.write(buf, sizeof(buf));
    Serial.printf("Sent 88-byte notify (MTU %u: %s).\n", mtu,
                  mtu >= 91 ? "one packet" : "chunked");
  }
}
