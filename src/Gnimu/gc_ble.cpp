// Gnimu - RaceBox Mini-compatible GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// Based on the Open-Source RaceBox Mini Emulator by Anchit Chandra Sekhar
// (https://github.com/anchit92/Open-Source-RaceBox-mini-Emulator)
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

#include "gc_ble.h"
#include "config.h"
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// --- BLE state - private to this file ---
static const String deviceName = String(MODEL) + " " + DEVICE_ID;

static BLEServer *pServer = NULL;
static BLECharacteristic *pCharacteristicTx = NULL;
static BLECharacteristic *pCharacteristicRx = NULL;
static volatile bool deviceConnected = false;
static volatile bool oldDeviceConnected = false;

// --- BLE Callbacks ---
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    // Request a larger MTU to fit an 88-byte packet + headers in one go
    pServer->updatePeerMTU(pServer->getConnId(), BLE_MTU_SIZE);
    Serial.println("✅ BLE Client connected & MTU update requested");
  }
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("❌ BLE Client disconnected");
  }
};

class RxCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0) {
      Serial.print("📨 Received BLE command: ");
      for (size_t i = 0; i < rxValue.length(); i++) {
        Serial.printf("0x%02X ", (uint8_t)rxValue[i]);
      }
      Serial.println();
    }
  }
};

void bleBegin() {
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  BLEDevice::init(deviceName.c_str());
  BLEDevice::setPower(BLE_TX_POWER);
  {
    int requestedDbm = (BLE_TX_POWER * 3) - 12;
    int actualDbm = BLEDevice::getPower();
    if (actualDbm == requestedDbm) {
      Serial.printf("✅ BLE TX power set to %d dBm.\n", actualDbm);
    } else {
      Serial.printf(
          "⚠️  BLE TX power mismatch - requested %d dBm, got %d dBm.\n",
          requestedDbm, actualDbm);
    }
  }
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(RACEBOX_SERVICE_UUID);
  pCharacteristicTx = pService->createCharacteristic(
      RACEBOX_CHARACTERISTIC_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicTx->addDescriptor(new BLE2902());
  pCharacteristicRx = pService->createCharacteristic(
      RACEBOX_CHARACTERISTIC_RX_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pCharacteristicRx->setCallbacks(new RxCharacteristicCallbacks());
  pService->start();
  // --- Device Information Service ---
  BLEService *pDeviceInfo =
      pServer->createService("0000180a-0000-1000-8000-00805f9b34fb");
  // Model
  BLECharacteristic *pModel = pDeviceInfo->createCharacteristic(
      "00002a24-0000-1000-8000-00805f9b34fb", BLECharacteristic::PROPERTY_READ);
  pModel->setValue(MODEL);
  // Serial number - the 10-digit device ID from config.h
  BLECharacteristic *pSerial = pDeviceInfo->createCharacteristic(
      "00002a25-0000-1000-8000-00805f9b34fb", BLECharacteristic::PROPERTY_READ);
  pSerial->setValue(DEVICE_ID);
  // Firmware revision
  BLECharacteristic *pFirm = pDeviceInfo->createCharacteristic(
      "00002a26-0000-1000-8000-00805f9b34fb", BLECharacteristic::PROPERTY_READ);
  pFirm->setValue(FIRMWARE_VERSION);
  // Hardware revision
  BLECharacteristic *pHardware = pDeviceInfo->createCharacteristic(
      "00002a27-0000-1000-8000-00805f9b34fb", BLECharacteristic::PROPERTY_READ);
  pHardware->setValue(HARDWARE_VERSION);
  // Manufacturer
  BLECharacteristic *pManufacturer = pDeviceInfo->createCharacteristic(
      "00002a29-0000-1000-8000-00805f9b34fb", BLECharacteristic::PROPERTY_READ);
  pManufacturer->setValue(MANUFACTURER);
  pDeviceInfo->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(RACEBOX_SERVICE_UUID);
  // Advertise Device Information Service to help official apps discover the
  // device
  pAdvertising->addServiceUUID("0000180a-0000-1000-8000-00805f9b34fb");
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("📡 BLE advertising started.");
}

bool bleIsConnected() { return deviceConnected; }

void bleSendPacket(uint8_t *data, size_t len) {
  pCharacteristicTx->setValue(data, len);
  pCharacteristicTx->notify();
}

// --- Drive the onboard LED: solid when connected, blink when disconnected ---
static void updateLed() {
  if (!deviceConnected) {
    static unsigned long lastBlinkMs = 0;
    if (millis() - lastBlinkMs > LED_BLINK_INTERVAL_MS) {
      lastBlinkMs = millis();
      digitalWrite(ONBOARD_LED_PIN, !digitalRead(ONBOARD_LED_PIN));
    }
  } else {
    digitalWrite(ONBOARD_LED_PIN, HIGH);
  }
}

void bleUpdate() {
  // BLE connection state management - runs regardless of GNSS state. The
  // re-advertise after a disconnect is deferred (non-blocking): we record when
  // the disconnect happened and restart advertising on a later loop() pass once
  // the settle delay has elapsed, rather than blocking the whole loop.
  static bool reAdvertisePending = false;
  static unsigned long disconnectMs = 0;

  // Disconnect edge - schedule a re-advertise after the settle delay.
  if (!deviceConnected && oldDeviceConnected) {
    disconnectMs = millis();
    reAdvertisePending = true;
    oldDeviceConnected = deviceConnected;
  }
  // Connect edge - a client is back; cancel any pending re-advertise.
  if (deviceConnected && !oldDeviceConnected) {
    reAdvertisePending = false;
    oldDeviceConnected = deviceConnected;
  }
  // Settle delay elapsed - restart advertising. Only clear the pending flag on
  // success; on failure, leave it set and reset the timer so we retry after
  // another interval rather than sitting unconnectable until reboot.
  if (reAdvertisePending &&
      millis() - disconnectMs >= BLE_READVERTISE_DELAY_MS) {
    if (BLEDevice::getAdvertising()->start()) {
      Serial.println("📡 BLE re-advertising started.");
      reAdvertisePending = false;
    } else {
      Serial.println("⚠️  BLE re-advertising failed to start - will retry.");
      disconnectMs = millis();
    }
  }
  updateLed();
}
