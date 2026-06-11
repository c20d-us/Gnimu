#include "telemetry.h"
#include "ble.h"
#include "config.h"
#include "gnss.h"
#include "imu.h"
#include "ubx_helpers.h"
#include <string.h> // for memcpy

// --- Packet timing / stats counters — private to this file ---
static unsigned long lastGpsRateCheckTime = 0;
static volatile unsigned int bleSentPacketCount = 0;
static volatile unsigned int gnssUpdateCount = 0;

void telemetryBegin() {
  pinMode(ONBOARD_LED_PIN, OUTPUT);
  lastGpsRateCheckTime = millis();
}

// --- Blink the onboard LED when disconnected, solid when connected ---
void updateLedStatus() {
  if (!bleIsConnected()) {
    static unsigned long lastBlinkMs = 0;
    if (millis() - lastBlinkMs > LED_BLINK_INTERVAL_MS) {
      lastBlinkMs = millis();
      digitalWrite(ONBOARD_LED_PIN, !digitalRead(ONBOARD_LED_PIN));
    }
  } else {
    digitalWrite(ONBOARD_LED_PIN, HIGH);
  }
}

// --- Assemble an 88-byte RaceBox Data Message from the latest GNSS + IMU data.
// Fills packet[0..87] with the UBX header, 80-byte payload, and checksum. ---
static void buildRaceBoxPacket(uint8_t *packet) {
  ImuProtocolUnits imu = readImuProtocolUnits();

  uint8_t payload[80] = {0};

  // Read fields from the latest PVT solution — a read-only view owned by the
  // GNSS module. Non-null here because we only build after gnssHasNewEpoch().
  const UBX_NAV_PVT_data_t *pvt = gnssLatestPvt();
  writeLittleEndian(payload, 0, pvt->iTOW);
  writeLittleEndian(payload, 4, pvt->year);
  writeLittleEndian(payload, 6, pvt->month);
  writeLittleEndian(payload, 7, pvt->day);
  writeLittleEndian(payload, 8, pvt->hour);
  writeLittleEndian(payload, 9, pvt->min);
  writeLittleEndian(payload, 10, pvt->sec);

  // Offset 11: Validity Flags (RaceBox Protocol)
  uint8_t raceboxValidityFlags = 0;
  if (pvt->valid.bits.validDate)
    raceboxValidityFlags |= (1 << 0); // Bit 0: valid date
  if (pvt->valid.bits.validTime)
    raceboxValidityFlags |= (1 << 1); // Bit 1: valid time
  if (pvt->valid.bits.fullyResolved)
    raceboxValidityFlags |= (1 << 2); // Bit 2: fully resolved
  if (pvt->valid.bits.validMag)
    raceboxValidityFlags |= (1 << 3); // Bit 3: valid magnetic declination
  writeLittleEndian(payload, 11, raceboxValidityFlags);

  // Offset 12: Time Accuracy (RaceBox Protocol)
  writeLittleEndian(payload, 12, pvt->tAcc);

  // Offset 16: Nanoseconds (RaceBox Protocol)
  writeLittleEndian(payload, 16, pvt->nano);

  // Offset 20: Fix Status (RaceBox Protocol)
  // Protocol only defines 0 (no fix), 2 (2D fix), 3 (3D fix) — clamp any
  // other u-blox fix types (e.g. 1=DR only, 4=GNSS+DR) to 0 (no fix).
  uint8_t safeFixType =
      (pvt->fixType == 2 || pvt->fixType == 3) ? pvt->fixType : 0;
  writeLittleEndian(payload, 20, safeFixType);

  // Offset 21: Fix Status Flags (RaceBox Protocol)
  uint8_t fixStatusFlagsRacebox = 0;

  if (pvt->fixType == 3) {
    fixStatusFlagsRacebox |= (1 << 0); // Bit 0: valid fix
  }

  if (gnssHeadingValid()) { // Use the confirmed function to check
                            // for valid heading
    fixStatusFlagsRacebox |=
        (1 << 5); // Bit 5: valid heading (as per RaceBox Protocol)
  }
  writeLittleEndian(payload, 21, fixStatusFlagsRacebox);

  // Offset 22: Date/Time Flags (RaceBox Protocol)
  uint8_t raceboxDateTimeFlags = 0;
  if (pvt->valid.bits.validTime)
    raceboxDateTimeFlags |=
        (1 << 5); // Available confirmation of Date/Time Validity
  if (pvt->valid.bits.validDate)
    raceboxDateTimeFlags |= (1 << 6); // Confirmed UTC Date Validity
  if (pvt->valid.bits.validTime && pvt->valid.bits.fullyResolved)
    raceboxDateTimeFlags |= (1 << 7); // Confirmed UTC Time Validity
  writeLittleEndian(payload, 22, raceboxDateTimeFlags);

  // Offset 23: Number of SVs (RaceBox Protocol)
  writeLittleEndian(payload, 23, pvt->numSV);

  // Remaining fields, mostly direct mappings from u-blox data
  writeLittleEndian(payload, 24, pvt->lon);
  writeLittleEndian(payload, 28, pvt->lat);
  writeLittleEndian(payload, 32, pvt->height);
  writeLittleEndian(payload, 36, pvt->hMSL);

  writeLittleEndian(payload, 40, pvt->hAcc);
  writeLittleEndian(payload, 44, pvt->vAcc);
  writeLittleEndian(payload, 48, pvt->gSpeed);
  writeLittleEndian(payload, 52, pvt->headMot);
  writeLittleEndian(payload, 56, pvt->sAcc);
  writeLittleEndian(payload, 60, pvt->headAcc);

  writeLittleEndian(payload, 64, pvt->pDOP);

  // Offset 66: Lat/Lon Flags (RaceBox Protocol)
  uint8_t latLonFlags = 0;
  if (pvt->fixType <
      2) { // If no 2D/3D fix, then coordinates are considered invalid
    latLonFlags |= (1 << 0); // Bit 0: Invalid Latitude, Longitude, WGS
                             // Altitude, and MSL Altitude
  }
  writeLittleEndian(payload, 66, latLonFlags);

  // Offset 67: Battery status (1 byte) - report 100% to avoid low battery
  // warnings
  writeLittleEndian(payload, 67, (uint8_t)BATTERY_REPORT_PERCENT);

  writeLittleEndian(payload, 68, imu.gX);
  writeLittleEndian(payload, 70, imu.gY);
  writeLittleEndian(payload, 72, imu.gZ);
  writeLittleEndian(payload, 74, imu.rX);
  writeLittleEndian(payload, 76, imu.rY);
  writeLittleEndian(payload, 78, imu.rZ);

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
}

// --- On each new GNSS epoch, count it and (when connected) send a packet ---
void sendRaceBoxPacketIfReady() {
  if (!gnssHasNewEpoch())
    return;
  gnssUpdateCount++;

  if (!bleIsConnected())
    return;
  bleSentPacketCount++;

  uint8_t packet[88] = {0};
  buildRaceBoxPacket(packet);

  bleSendPacket(packet, 88);
}

// --- Periodically print packet rate and GNSS/IMU debug stats over serial ---
void reportStatus() {
  // Report packet send rate — runs regardless of GPS state
  const unsigned long now = millis();
  if ((now - lastGpsRateCheckTime) >= GPS_RATE_REPORT_INTERVAL_MS) {
    float elapsed = (now - lastGpsRateCheckTime) / 1000.0;
    float bleRate = bleSentPacketCount / elapsed;
    float gnssRate = gnssUpdateCount / elapsed;
    // Additional satellite info for debugging: number of satellites, fix type,
    // horizontal accuracy, and lat/lon
    uint8_t sats = 0;
    uint8_t fix = 0;
    uint32_t hAcc = 0;
    double lat = 0.0, lon = 0.0;
    const UBX_NAV_PVT_data_t *pvt = gnssLatestPvt();
    if (pvt != nullptr) {
      sats = pvt->numSV;
      fix = pvt->fixType;
      hAcc = pvt->hAcc;
      lat = pvt->lat * 1e-7;
      lon = pvt->lon * 1e-7;
    }
    // Convert filtered IMU values to protocol units for display
    ImuProtocolUnits imu = readImuProtocolUnits();
    Serial.printf("BLE Rate: %.2f Hz | GNSS Rate: %.2f Hz | SV: %u | Fix: %u | "
                  "HAcc: %u mm | Lat: %.7f Lon: %.7f | milliG: X=%d Y=%d Z=%d "
                  "| centiDeg/s: X=%d Y=%d Z=%d\n",
                  bleRate, gnssRate, sats, fix, hAcc, lat, lon, imu.gX, imu.gY,
                  imu.gZ, imu.rX, imu.rY, imu.rZ);
    bleSentPacketCount = 0;
    gnssUpdateCount = 0;
    lastGpsRateCheckTime = now;
  }
}
