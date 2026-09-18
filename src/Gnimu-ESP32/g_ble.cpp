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
#include "g_log.h"
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <atomic>

// BLE state
static const char deviceName[] = RACEBOX_MODEL " " DEVICE_ID;
static BLEServer *pServer = NULL;
static BLECharacteristic *pCharacteristicTx = NULL;
static volatile bool deviceConnected = false;
static volatile bool oldDeviceConnected = false;

// The peer's ATT MTU, 23 until the phone's MTU exchange raises it. Written by
// Bluedroid callbacks on the other core, read by the loop.
static std::atomic<uint16_t> peerMtu{23};

// Frames are refused until the MTU can carry them. A few refusals on a fast
// reconnect are normal, before the exchange completes; only a refusal lasting
// kMtuRefusalLogMs is logged, once per session.
static constexpr unsigned long kMtuRefusalLogMs = 1000;
static bool mtuRefusing = false;
static bool mtuRefusalLogged = false;
static unsigned long mtuRefusingSinceMs = 0;

// Blink the onboard LED while disconnected. The connect edge in bleUpdate()
// turns it solid once, rather than rewriting it on every pass.
static void updateLed() {
  if (!deviceConnected) {
    static unsigned long lastBlinkMs = 0;
    if (millis() - lastBlinkMs > LED_BLINK_INTERVAL_MS) {
      lastBlinkMs = millis();
      digitalWrite(LED_ONBOARD_PIN, !digitalRead(LED_ONBOARD_PIN));
    }
  }
}

// BLE Callbacks. They run on Bluedroid's task on the other core, so they only
// set state; bleUpdate() logs the edges from the loop.
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    (void)pServer;
    // Store the MTU before publishing the connection.
    peerMtu.store(23, std::memory_order_release);
    deviceConnected = true;
  }
  void onMtuChanged(BLEServer *pServer,
                    esp_ble_gatts_cb_param_t *param) override {
    (void)pServer;
    peerMtu.store(param->mtu.mtu, std::memory_order_release);
  }
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
  }
};

void bleBegin() {
  pinMode(LED_ONBOARD_PIN, OUTPUT);
  BLEDevice::init(deviceName);
  // One level for both states. ESP_BLE_PWR_TYPE_DEFAULT reaches connections
  // but not advertising, which otherwise stays at the controller's +9 dBm.
  BLEDevice::setPower(BLE_TX_POWER, ESP_BLE_PWR_TYPE_ADV);
  BLEDevice::setPower(BLE_TX_POWER, ESP_BLE_PWR_TYPE_DEFAULT);
  {
    const int requestedDbm = (BLE_TX_POWER * 3) - 12;
    const int advDbm = BLEDevice::getPower(ESP_BLE_PWR_TYPE_ADV);
    const int connDbm = BLEDevice::getPower(ESP_BLE_PWR_TYPE_DEFAULT);
    if (advDbm == requestedDbm && connDbm == requestedDbm) {
      LOG_PRINTF("✅ BLE TX power set to %d dBm (advertising and connected).\n",
                 requestedDbm);
    } else {
      LOG_PRINTF("⚠️  BLE TX power mismatch - requested %d dBm, got "
                 "advertising %d dBm, connected %d dBm.\n",
                 requestedDbm, advDbm, connDbm);
    }
  }
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Create the RaceBox Service
  BLEService *pService = pServer->createService(RACEBOX_SERVICE_UUID);
  pCharacteristicTx = pService->createCharacteristic(
      RACEBOX_CHARACTERISTIC_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicTx->addDescriptor(new BLE2902());
  // Rx exists for the RaceBox GATT layout; writes to it are accepted and
  // ignored.
  pService->createCharacteristic(
      RACEBOX_CHARACTERISTIC_RX_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pService->start();

  // Device Information Service
  BLEService *pDeviceInfo =
      pServer->createService("0000180a-0000-1000-8000-00805f9b34fb");

  struct DeviceInfoField {
    const char *uuid;
    const char *value;
  };
  const DeviceInfoField deviceInfoFields[] = {
      {"00002a24-0000-1000-8000-00805f9b34fb", RACEBOX_MODEL},
      {"00002a25-0000-1000-8000-00805f9b34fb", DEVICE_ID}, // Serial number
      {"00002a29-0000-1000-8000-00805f9b34fb", RACEBOX_MANUFACTURER},
      {"00002a26-0000-1000-8000-00805f9b34fb", RACEBOX_FIRMWARE_VERSION},
      {"00002a27-0000-1000-8000-00805f9b34fb", RACEBOX_HARDWARE_VERSION},
  };
  for (const auto &field : deviceInfoFields) {
    pDeviceInfo
        ->createCharacteristic(field.uuid, BLECharacteristic::PROPERTY_READ)
        ->setValue(field.value);
  }
  pDeviceInfo->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(RACEBOX_SERVICE_UUID);
  // Advertise Device Information Service so apps can discover the device
  pAdvertising->addServiceUUID("0000180a-0000-1000-8000-00805f9b34fb");
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  LOG_PRINTLN("📡 BLE advertising started.");
}

bool bleIsConnected() { return deviceConnected; }

bool bleSendPacket(uint8_t *data, size_t len) {
  // A notify carries MTU - 3 bytes; a longer one would be cut short.
  const uint16_t mtu = peerMtu.load(std::memory_order_acquire);
  if (len + 3 > mtu) {
    if (!mtuRefusing) {
      mtuRefusing = true;
      mtuRefusingSinceMs = millis();
    } else if (!mtuRefusalLogged &&
               millis() - mtuRefusingSinceMs >= kMtuRefusalLogMs) {
      mtuRefusalLogged = true;
      LOG_PRINTF("❌ BLE: peer MTU %u too small for %u-byte frames - not "
                 "sending.\n",
                 (unsigned int)mtu, (unsigned int)len);
    }
    return false;
  }
  mtuRefusing = false;
  pCharacteristicTx->setValue(data, len);
  pCharacteristicTx->notify();
  return true;
}

void bleUpdate() {
  // BLE connection state management.
  // The re-advertise after a disconnect is deferred  so that it's non-blocking.
  // We record when the disconnect happened and restart advertising on a later
  // loop() pass once the settle delay has elapsed.
  static bool reAdvertisePending = false;
  static unsigned long disconnectMs = 0;

  // Disconnect edge - schedule a re-advertise after the settle delay.
  if (!deviceConnected && oldDeviceConnected) {
    LOG_PRINTLN("👋 BLE Client disconnected");
    disconnectMs = millis();
    reAdvertisePending = true;
    oldDeviceConnected = deviceConnected;
  }
  // Connect edge - a client is back; cancel any pending re-advertise.
  if (deviceConnected && !oldDeviceConnected) {
    LOG_PRINTLN("🤝 BLE Client connected");
    digitalWrite(LED_ONBOARD_PIN, HIGH);
    reAdvertisePending = false;
    mtuRefusing = false;
    mtuRefusalLogged = false;
    oldDeviceConnected = deviceConnected;
  }
  // Settle delay has elapsed, restart advertising.
  // Only clear the pending flag on success. On failure, leave it set and reset
  // the timer so we retry after another interval rather than sitting
  // unconnectable until reboot.
  if (reAdvertisePending &&
      millis() - disconnectMs >= BLE_READVERTISE_DELAY_MS) {
    if (BLEDevice::getAdvertising()->start()) {
      reAdvertisePending = false;
    } else {
      LOG_PRINTLN("⚠️  BLE re-advertising failed to start - will retry.");
      disconnectMs = millis();
    }
  }
  updateLed();
}
