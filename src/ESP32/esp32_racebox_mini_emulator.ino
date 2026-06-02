// This is a private fork of the work done by Anchit Chandra Sekhar (https://github.com/anchit92)
//
#include "config.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

SFE_UBLOX_GNSS myGNSS;
HardwareSerial GPS_Serial(2);

const String deviceName = String(MODEL) + " " + String(DEVICE_ID);

Adafruit_MPU6050 mpu;
// Storage for the filtered values
float filtered_ax = 0, filtered_ay = 0, filtered_az = 0;
float filtered_gx = 0, filtered_gy = 0, filtered_gz = 0;

BLEServer *pServer = NULL;
BLECharacteristic *pCharacteristicTx = NULL;
BLECharacteristic *pCharacteristicRx = NULL;
volatile bool deviceConnected = false;
volatile bool oldDeviceConnected = false;

// --- Packet Timing ---
unsigned long lastGpsRateCheckTime = 0;
volatile unsigned int gpsUpdateCount = 0;
volatile unsigned int gnssUpdateCount = 0;

// --- BLE Callbacks ---
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    // Request a larger MTU to fit an 88-byte packet + headers in one go
    pServer->updatePeerMTU(pServer->getConnId(), BLE_MTU_SIZE);
    digitalWrite(ONBOARD_LED_PIN, HIGH);
    Serial.println("✅ BLE Client connected & MTU update requested");
  }
  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    digitalWrite(ONBOARD_LED_PIN, LOW);
    Serial.println("❌ BLE Client disconnected");
  }
};

class MyCharacteristicCallbacks : public BLECharacteristicCallbacks {
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

// --- UBX Packet Construction Helpers ---
void writeLittleEndian(uint8_t* buffer, int offset, uint32_t value) { memcpy(buffer + offset, &value, 4); }
void writeLittleEndian(uint8_t* buffer, int offset, int32_t value)  { memcpy(buffer + offset, &value, 4); }
void writeLittleEndian(uint8_t* buffer, int offset, uint16_t value) { memcpy(buffer + offset, &value, 2); }
void writeLittleEndian(uint8_t* buffer, int offset, int16_t value)  { memcpy(buffer + offset, &value, 2); }
void writeLittleEndian(uint8_t* buffer, int offset, uint8_t value)  { buffer[offset] = value; }
void writeLittleEndian(uint8_t* buffer, int offset, int8_t value)   { buffer[offset] = (uint8_t)value; }

void calculateChecksum(uint8_t* payload, uint16_t len, uint8_t cls, uint8_t id, uint8_t* ckA, uint8_t* ckB) {
  *ckA = *ckB = 0;
  *ckA += cls; *ckB += *ckA;
  *ckA += id; *ckB += *ckA;
  *ckA += len & 0xFF; *ckB += *ckA;
  *ckA += len >> 8; *ckB += *ckA;
  for (uint16_t i = 0; i < len; i++) {
    *ckA += payload[i];
    *ckB += *ckA;
  }
}

void resetGpsBaudRate() {
  Serial.println("Attempting to set Correct Baud Rate");
  GPS_Serial.begin(FACTORY_GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(500);

  if (!myGNSS.begin(GPS_Serial)) {
    Serial.print("u-blox GNSS not detected at ");
    Serial.print(FACTORY_GPS_BAUD);
    Serial.println(" baud.");
    Serial.print("u-blox GNSS not detected, Check documentation for factory baud rate and/or check your wiring");
    while(1) delay(100);
  } else {
    Serial.print("GNSS detected at ");
    Serial.print(FACTORY_GPS_BAUD);
    Serial.println(" baud!");
  }
  delay(500);

  // Now switch baud rate
  Serial.print("Setting baud rate to ");
  Serial.print(GPS_BAUD);
  Serial.println("...");
  myGNSS.setSerialRate(GPS_BAUD);
  Serial.print("Baud rate changed to ");
  Serial.println(GPS_BAUD);

  GPS_Serial.end();
  delay(100);
  // Re-initialize the serial port at the new baud rate
  GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(500);

  if (!myGNSS.begin(GPS_Serial)) {
    Serial.print("GNSS not detected at ");
    Serial.print(GPS_BAUD);
    Serial.println(" baud.");
    Serial.print("u-blox GNSS not detected, Check documentation for factory baud rate and/or check your wiring");
    while (1) delay(100);
  }
  Serial.print("GNSS detected at ");
  Serial.print(GPS_BAUD);
  Serial.println(" baud! Saving to Flash");
  myGNSS.saveConfiguration(); // Save to flash
  GPS_Serial.end();
}

void setup() {
  Serial.begin(115200);
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  if (!mpu.begin()) {
    Serial.println("❌ Failed to find MPU6050 chip");
    while (1) delay(100);
  }
  mpu.setAccelerometerRange(ACCEL_RANGE);
  mpu.setGyroRange(GYRO_RANGE);
  mpu.setFilterBandwidth(FILTER_BANDWIDTH);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Initialize filters with the first real reading so they don't start at zero
  filtered_ax = a.acceleration.x;
  filtered_ay = a.acceleration.y;
  filtered_az = a.acceleration.z;

  GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  if (!myGNSS.begin(GPS_Serial)) {
    Serial.println("❌ GNSS not detected. Attempting to configure.");
    GPS_Serial.end();
    resetGpsBaudRate();
    GPS_Serial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  }

  // Set GNSS output to PVT only
  myGNSS.setAutoPVT(true);
  myGNSS.setDynamicModel(GPS_DYNAMIC_MODEL);
    // --- Configure GPS update rate to MAX_NAVIGATION_RATE Hz ---
  if (myGNSS.setNavigationFrequency(MAX_NAVIGATION_RATE)) {
  Serial.printf("✅ GPS update rate set to %d Hz.\n",MAX_NAVIGATION_RATE );
  } else {
    Serial.println("❌ Failed to set GPS update rate.");
  }

  // --- GNSS Constellation Setup ---

  // GPS
  #ifdef ENABLE_GNSS_GPS
    if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS)) {
      Serial.println("✅ GPS enabled.");
    } else {
      Serial.println("❌ Failed to enable GPS.");
    }
  #else
    myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GPS);
    Serial.println("🚫 GPS disabled.");
  #endif

  // Galileo
  #ifdef ENABLE_GNSS_GALILEO
    if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO)) {
      Serial.println("✅ Galileo enabled.");
    } else {
      Serial.println("❌ Failed to enable Galileo.");
    }
  #else
    myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GALILEO);
    Serial.println("🚫 Galileo disabled.");
  #endif

  // GLONASS
  #ifdef ENABLE_GNSS_GLONASS
    if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS)) {
      Serial.println("✅ GLONASS enabled.");
    } else {
      Serial.println("❌ Failed to enable GLONASS.");
    }
  #else
    myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_GLONASS);
    Serial.println("🚫 GLONASS disabled.");
  #endif

  // BeiDou
  #ifdef ENABLE_GNSS_BEIDOU
    if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_BEIDOU)) {
      Serial.println("✅ BEIDOU enabled.");
    } else {
      Serial.println("❌ Failed to enable BEIDOU.");
    }
  #else
    myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_BEIDOU);
    Serial.println("🚫 BEIDOU disabled.");
  #endif

  // Optional: QZSS
  #ifdef ENABLE_GNSS_QZSS
    if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_QZSS)) {
      Serial.println("✅ QZSS enabled.");
    } else {
      Serial.println("❌ Failed to enable QZSS.");
    }
  #else
    myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_QZSS);
    Serial.println("🚫 QZSS disabled.");
  #endif

  // Optional: SBAS (satellite-based augmentation)
  #ifdef ENABLE_GNSS_SBAS
    if (myGNSS.enableGNSS(true, SFE_UBLOX_GNSS_ID_SBAS)) {
      Serial.println("✅ SBAS enabled.");
    } else {
      Serial.println("❌ Failed to enable SBAS.");
    }
  #else
    myGNSS.enableGNSS(false, SFE_UBLOX_GNSS_ID_SBAS);
    Serial.println("🚫 SBAS disabled.");
  #endif

  // --- BLE Setup ---
  BLEDevice::init(deviceName.c_str());
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(RACEBOX_SERVICE_UUID);
  pCharacteristicTx = pService->createCharacteristic(RACEBOX_CHARACTERISTIC_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicTx->addDescriptor(new BLE2902());
  pCharacteristicRx = pService->createCharacteristic(RACEBOX_CHARACTERISTIC_RX_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  pCharacteristicRx->setCallbacks(new MyCharacteristicCallbacks());
  pService->start();
  // --- Device Information Service ---
  BLEService *pDeviceInfo = pServer->createService("0000180a-0000-1000-8000-00805f9b34fb");
  // Model
  BLECharacteristic *pModel = pDeviceInfo->createCharacteristic("00002a24-0000-1000-8000-00805f9b34fb", BLECharacteristic::PROPERTY_READ);
  pModel->setValue(MODEL);
  // Serial number — the 10-digit device ID from config.h
  BLECharacteristic *pSerial = pDeviceInfo->createCharacteristic("00002a25-0000-1000-8000-00805f9b34fb", BLECharacteristic::PROPERTY_READ);
  pSerial->setValue(String(DEVICE_ID).c_str());
  // Firmware revision
  BLECharacteristic *pFirm = pDeviceInfo->createCharacteristic("00002a26-0000-1000-8000-00805f9b34fb", BLECharacteristic::PROPERTY_READ);
  pFirm->setValue(FIRMWARE_VERSION);
  // Hardware revision
  BLECharacteristic *pHardware = pDeviceInfo->createCharacteristic("00002a27-0000-1000-8000-00805f9b34fb", BLECharacteristic::PROPERTY_READ);
  pHardware->setValue(HARDWARE_VERSION);
  // Manufacturer
  BLECharacteristic *pManufacturer = pDeviceInfo->createCharacteristic("00002a29-0000-1000-8000-00805f9b34fb", BLECharacteristic::PROPERTY_READ);
  pManufacturer->setValue(MANUFACTURER);
  pDeviceInfo->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(RACEBOX_SERVICE_UUID);
  // Advertise Device Information Service to help official apps discover the device
  pAdvertising->addServiceUUID("0000180a-0000-1000-8000-00805f9b34fb");
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("📡 BLE advertising started.");

  lastGpsRateCheckTime = millis();
}

void loop() {
  myGNSS.checkUblox(); // Required to keep GNSS data flowing
  static unsigned long lastAccelReadMs = 0;
  // Update Accelerometer readings at fixed interval
  if (millis() - lastAccelReadMs >= ACCEL_SAMPLE_INTERVAL_MS) {
    lastAccelReadMs = millis();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Apply Exponential Moving Average (Complementary Filter logic)
    filtered_ax = (ACCEL_ALPHA * a.acceleration.x) + ((1.0 - ACCEL_ALPHA) * filtered_ax);
    filtered_ay = (ACCEL_ALPHA * a.acceleration.y) + ((1.0 - ACCEL_ALPHA) * filtered_ay);
    filtered_az = (ACCEL_ALPHA * a.acceleration.z) + ((1.0 - ACCEL_ALPHA) * filtered_az);

    filtered_gx = (GYRO_ALPHA * g.gyro.x) + ((1.0 - GYRO_ALPHA) * filtered_gx);
    filtered_gy = (GYRO_ALPHA * g.gyro.y) + ((1.0 - GYRO_ALPHA) * filtered_gy);
    filtered_gz = (GYRO_ALPHA * g.gyro.z) + ((1.0 - GYRO_ALPHA) * filtered_gz);
  }
  // LED Blink Logic
  if (!deviceConnected) {
    static unsigned long lastBlinkMs = 0;
    if (millis() - lastBlinkMs > LED_BLINK_INTERVAL_MS) {
      lastBlinkMs = millis();
      digitalWrite(ONBOARD_LED_PIN, !digitalRead(ONBOARD_LED_PIN));
    }
  } else {
    digitalWrite(ONBOARD_LED_PIN, HIGH);
  }
  if (myGNSS.getPVT() && myGNSS.packetUBXNAVPVT != NULL) {
    static uint32_t lastITOW = 0;
    uint32_t currentITOW = myGNSS.packetUBXNAVPVT->data.iTOW;

    if (currentITOW != lastITOW) {
      lastITOW = currentITOW;
      gnssUpdateCount++;

      if (deviceConnected) {
        gpsUpdateCount++;

        // Convert accelerometer to milli-g (1g = 9.80665 m/s^2)
        int16_t gX = filtered_ax * 1000.0 / 9.80665;
        int16_t gY = filtered_ay * 1000.0 / 9.80665;
        int16_t gZ = filtered_az * 1000.0 / 9.80665;

        // Convert gyro to centi-deg/sec
        int16_t rX = filtered_gx * 180.0 / M_PI * 100.0;
        int16_t rY = filtered_gy * 180.0 / M_PI * 100.0;
        int16_t rZ = filtered_gz * 180.0 / M_PI * 100.0;

        uint8_t payload[80] = {0};
        uint8_t packet[88] = {0};

        // Access data directly from myGNSS.packetUBXNAVPVT->data
        writeLittleEndian(payload, 0, myGNSS.packetUBXNAVPVT->data.iTOW);
        writeLittleEndian(payload, 4, myGNSS.packetUBXNAVPVT->data.year);
        writeLittleEndian(payload, 6, myGNSS.packetUBXNAVPVT->data.month);
        writeLittleEndian(payload, 7, myGNSS.packetUBXNAVPVT->data.day);
        writeLittleEndian(payload, 8, myGNSS.packetUBXNAVPVT->data.hour);
        writeLittleEndian(payload, 9, myGNSS.packetUBXNAVPVT->data.min);
        writeLittleEndian(payload, 10, myGNSS.packetUBXNAVPVT->data.sec);

        // Offset 11: Validity Flags (RaceBox Protocol)
        uint8_t raceboxValidityFlags = 0;
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.validDate)    raceboxValidityFlags |= (1 << 0); // Bit 0: valid date
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.validTime)    raceboxValidityFlags |= (1 << 1); // Bit 1: valid time
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.fullyResolved) raceboxValidityFlags |= (1 << 2); // Bit 2: fully resolved
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.validMagDec)  raceboxValidityFlags |= (1 << 3); // Bit 3: valid magnetic declination
        writeLittleEndian(payload, 11, raceboxValidityFlags);

        // Offset 12: Time Accuracy (RaceBox Protocol)
        writeLittleEndian(payload, 12, myGNSS.packetUBXNAVPVT->data.tAcc);

        // Offset 16: Nanoseconds (RaceBox Protocol)
        writeLittleEndian(payload, 16, myGNSS.packetUBXNAVPVT->data.nano);

        // Offset 20: Fix Status (RaceBox Protocol)
        // Protocol only defines 0 (no fix), 2 (2D fix), 3 (3D fix) — clamp any
        // other u-blox fix types (e.g. 1=DR only, 4=GNSS+DR) to 0 (no fix).
        uint8_t safeFixType = (myGNSS.packetUBXNAVPVT->data.fixType == 2 || myGNSS.packetUBXNAVPVT->data.fixType == 3)
                              ? myGNSS.packetUBXNAVPVT->data.fixType : 0;
        writeLittleEndian(payload, 20, safeFixType);

        // Offset 21: Fix Status Flags (RaceBox Protocol)
        uint8_t fixStatusFlagsRacebox = 0;

        if (myGNSS.packetUBXNAVPVT->data.fixType == 3) {
            fixStatusFlagsRacebox |= (1 << 0); // Bit 0: valid fix
        }

        if (myGNSS.getHeadVehValid()) { // Use the confirmed function to check for valid heading
            fixStatusFlagsRacebox |= (1 << 5); // Bit 5: valid heading (as per RaceBox Protocol)
        }
        writeLittleEndian(payload, 21, fixStatusFlagsRacebox);

        // Offset 22: Date/Time Flags (RaceBox Protocol)
        uint8_t raceboxDateTimeFlags = 0;
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.validTime) raceboxDateTimeFlags |= (1 << 5); // Available confirmation of Date/Time Validity
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.validDate) raceboxDateTimeFlags |= (1 << 6); // Confirmed UTC Date Validity
        if (myGNSS.packetUBXNAVPVT->data.valid.bits.validTime && myGNSS.packetUBXNAVPVT->data.valid.bits.fullyResolved) raceboxDateTimeFlags |= (1 << 7); // Confirmed UTC Time Validity
        writeLittleEndian(payload, 22, raceboxDateTimeFlags);

        // Offset 23: Number of SVs (RaceBox Protocol)
        writeLittleEndian(payload, 23, myGNSS.packetUBXNAVPVT->data.numSV);

        // Remaining fields, mostly direct mappings from u-blox data
        writeLittleEndian(payload, 24, myGNSS.packetUBXNAVPVT->data.lon);
        writeLittleEndian(payload, 28, myGNSS.packetUBXNAVPVT->data.lat);
        writeLittleEndian(payload, 32, myGNSS.packetUBXNAVPVT->data.height);
        writeLittleEndian(payload, 36, myGNSS.packetUBXNAVPVT->data.hMSL);

        writeLittleEndian(payload, 40, myGNSS.packetUBXNAVPVT->data.hAcc);
        writeLittleEndian(payload, 44, myGNSS.packetUBXNAVPVT->data.vAcc);
        writeLittleEndian(payload, 48, myGNSS.packetUBXNAVPVT->data.gSpeed);
        writeLittleEndian(payload, 52, myGNSS.packetUBXNAVPVT->data.headMot);
        writeLittleEndian(payload, 56, myGNSS.packetUBXNAVPVT->data.sAcc);
        writeLittleEndian(payload, 60, myGNSS.packetUBXNAVPVT->data.headAcc);

        writeLittleEndian(payload, 64, myGNSS.packetUBXNAVPVT->data.pDOP);

        // Offset 66: Lat/Lon Flags (RaceBox Protocol)
        uint8_t latLonFlags = 0;
        if (myGNSS.packetUBXNAVPVT->data.fixType < 2) { // If no 2D/3D fix, then coordinates are considered invalid
            latLonFlags |= (1 << 0); // Bit 0: Invalid Latitude, Longitude, WGS Altitude, and MSL Altitude
        }
        writeLittleEndian(payload, 66, latLonFlags);

        // Offset 67: Battery status (1 byte) - report 100% to avoid low battery warnings
        writeLittleEndian(payload, 67, (uint8_t)BATTERY_REPORT_PERCENT);

        writeLittleEndian(payload, 68, gX);
        writeLittleEndian(payload, 70, gY);
        writeLittleEndian(payload, 72, gZ);
        writeLittleEndian(payload, 74, rX);
        writeLittleEndian(payload, 76, rY);
        writeLittleEndian(payload, 78, rZ);

        // Wrap in UBX (standard RaceBox header and checksum)
        packet[0] = 0xB5;
        packet[1] = 0x62;
        packet[2] = 0xFF; // Message Class: RaceBox Data Message
        packet[3] = 0x01; // Message ID: RaceBox Data Message
        packet[4] = 80;   // Payload size
        packet[5] = 0;
        memcpy(packet + 6, payload, 80);
        uint8_t ckA, ckB;
        calculateChecksum(payload, 80, 0xFF, 0x01, &ckA, &ckB);
        packet[86] = ckA;
        packet[87] = ckB;

        pCharacteristicTx->setValue(packet, 88);
        pCharacteristicTx->notify();
      }
    }

  }

  // Report packet send rate — runs regardless of GPS state
  const unsigned long now = millis();
  if ((now - lastGpsRateCheckTime) >= GPS_RATE_REPORT_INTERVAL_MS) {
    float elapsed = (now - lastGpsRateCheckTime) / 1000.0;
    float bleRate = gpsUpdateCount / elapsed;
    float gnssRate = gnssUpdateCount / elapsed;
    // Additional satellite info for debugging: number of satellites, fix type, horizontal accuracy, and lat/lon
    uint8_t sats = 0;
    uint8_t fix = 0;
    uint32_t hAcc = 0;
    double lat = 0.0, lon = 0.0;
    if (myGNSS.packetUBXNAVPVT != NULL) {
      sats = myGNSS.packetUBXNAVPVT->data.numSV;
      fix = myGNSS.packetUBXNAVPVT->data.fixType;
      hAcc = myGNSS.packetUBXNAVPVT->data.hAcc;
      lat = myGNSS.packetUBXNAVPVT->data.lat * 1e-7;
      lon = myGNSS.packetUBXNAVPVT->data.lon * 1e-7;
    }
    Serial.printf("BLE Packet Rate: %.2f Hz | GNSS Update Rate: %.2f Hz | SVs: %u | Fix: %u | HAcc: %u mm | Lat: %.7f Lon: %.7f\n",
                  bleRate, gnssRate, sats, fix, hAcc, lat, lon);
    gpsUpdateCount = 0;
    gnssUpdateCount = 0;
    lastGpsRateCheckTime = now;
  }

  // BLE connection state management — runs regardless of GPS state
  if (!deviceConnected && oldDeviceConnected) {
    delay(BLE_READVERTISE_DELAY_MS);
    pServer->startAdvertising();
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }
}
