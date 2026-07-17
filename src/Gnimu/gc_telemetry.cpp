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

#include "gc_telemetry.h"
#include "config.h"
#include "gc_ble.h"
#include "gc_gnss.h"
#include "gc_imu.h"
#include "gc_ubx_helpers.h"
#include <string.h>

// --- Internal counters and a private pointer to the latest PVT data ---
static unsigned long lastReportMs = 0;
static unsigned int bleSentPacketCount = 0;
static unsigned int gnssEpochCount = 0;
static const UBX_NAV_PVT_data_t *pvt = nullptr;

// Assemble an 88-byte RaceBox Data Message from the latest GNSS + IMU data.
// Fills packet[0..87] with the UBX header, 80-byte payload, and checksum.
// Once packet is built, hand off to BLE to send it.
static void sendPacket() {
  // Defensive check: if we somehow reach here with no data, abort immediately.
  if (pvt == nullptr) {
    return;
  }

  uint8_t packet[88] = {0};
  uint8_t payload[80] = {0};

  ImuProtocolUnits imu = imuReadProtocolUnits();

  // Casts pin each field to its RaceBox protocol wire width (U1/U2/U4/I4),
  // so the writeLittleEndian overload is correct regardless of the u-blox
  // library's field types.
  writeLittleEndian(payload, 0, (uint32_t)pvt->iTOW); // U4
  writeLittleEndian(payload, 4, (uint16_t)pvt->year); // U2
  writeLittleEndian(payload, 6, (uint8_t)pvt->month); // U1
  writeLittleEndian(payload, 7, (uint8_t)pvt->day);   // U1
  writeLittleEndian(payload, 8, (uint8_t)pvt->hour);  // U1
  writeLittleEndian(payload, 9, (uint8_t)pvt->min);   // U1
  writeLittleEndian(payload, 10, (uint8_t)pvt->sec);  // U1

  // Offset 11: Validity Flags
  uint8_t validityFlags = 0;
  if (pvt->valid.bits.validDate)
    validityFlags |= (1 << 0); // Bit 0: valid date
  if (pvt->valid.bits.validTime)
    validityFlags |= (1 << 1); // Bit 1: valid time
  if (pvt->valid.bits.fullyResolved)
    validityFlags |= (1 << 2); // Bit 2: fully resolved
  if (pvt->valid.bits.validMag)
    validityFlags |= (1 << 3); // Bit 3: valid magnetic declination
  writeLittleEndian(payload, 11, validityFlags);

  // Offset 12: Time Accuracy
  writeLittleEndian(payload, 12, (uint32_t)pvt->tAcc); // U4

  // Offset 16: Nanoseconds
  writeLittleEndian(payload, 16, (int32_t)pvt->nano); // I4

  // Offset 20: Fix Status
  // Protocol only defines 0 (no fix), 2 (2D fix), 3 (3D fix).
  // Clamp any other u-blox fix types (e.g. 1=DR only, 4=GNSS+DR) to 0 (no fix).
  uint8_t safeFixType =
      (pvt->fixType == 2 || pvt->fixType == 3) ? pvt->fixType : 0;
  writeLittleEndian(payload, 20, safeFixType);

  // Offset 21: Fix Status Flags
  uint8_t fixStatusFlags = 0;

  if (pvt->fixType == 3) {
    fixStatusFlags |= (1 << 0); // Bit 0: valid fix
  }

  if (gnssHeadingValid()) {
    fixStatusFlags |= (1 << 5); // Bit 5: valid heading
  }
  writeLittleEndian(payload, 21, fixStatusFlags);

  // Offset 22: Date/Time Flags
  uint8_t dateTimeFlags = 0;
  if (pvt->valid.bits.validTime)
    dateTimeFlags |= (1 << 5); // Available confirmation of Date/Time Validity
  if (pvt->valid.bits.validDate)
    dateTimeFlags |= (1 << 6); // Confirmed UTC Date Validity
  if (pvt->valid.bits.validTime && pvt->valid.bits.fullyResolved)
    dateTimeFlags |= (1 << 7); // Confirmed UTC Time Validity
  writeLittleEndian(payload, 22, dateTimeFlags);

  // Offset 23: Number of SVs
  writeLittleEndian(payload, 23, (uint8_t)pvt->numSV); // U1

  // Remaining fields, mostly direct mappings from u-blox data
  writeLittleEndian(payload, 24, (int32_t)pvt->lon);      // I4
  writeLittleEndian(payload, 28, (int32_t)pvt->lat);      // I4
  writeLittleEndian(payload, 32, (int32_t)pvt->height);   // I4
  writeLittleEndian(payload, 36, (int32_t)pvt->hMSL);     // I4
  writeLittleEndian(payload, 40, (uint32_t)pvt->hAcc);    // U4
  writeLittleEndian(payload, 44, (uint32_t)pvt->vAcc);    // U4
  writeLittleEndian(payload, 48, (int32_t)pvt->gSpeed);   // I4
  writeLittleEndian(payload, 52, (int32_t)pvt->headMot);  // I4
  writeLittleEndian(payload, 56, (uint32_t)pvt->sAcc);    // U4
  writeLittleEndian(payload, 60, (uint32_t)pvt->headAcc); // U4
  writeLittleEndian(payload, 64, (uint16_t)pvt->pDOP);    // U2

  // Offset 66: Lat/Lon Flags
  uint8_t latLonFlags = 0;
  if (pvt->fixType <
      2) { // If no 2D/3D fix, then coordinates are considered invalid
    latLonFlags |= (1 << 0); // Bit 0: Invalid Latitude, Longitude, WGS
                             // Altitude, and MSL Altitude
  }
  writeLittleEndian(payload, 66, latLonFlags);

  // Offset 67: Battery status (1 byte)
  // Report 100% to avoid low battery warnings.
  writeLittleEndian(payload, 67, (uint8_t)BATTERY_REPORT_PERCENT);

  // Offset 68-78: IMU data
  writeLittleEndian(payload, 68, imu.gX);
  writeLittleEndian(payload, 70, imu.gY);
  writeLittleEndian(payload, 72, imu.gZ);
  writeLittleEndian(payload, 74, imu.rX);
  writeLittleEndian(payload, 76, imu.rY);
  writeLittleEndian(payload, 78, imu.rZ);

  // Add RaceBox protocol header
  packet[0] = 0xB5;
  packet[1] = 0x62;
  packet[2] = 0xFF; // Message Class: RaceBox Data Message
  packet[3] = 0x01; // Message ID: RaceBox Data Message
  packet[4] = 80;   // Payload size
  packet[5] = 0;
  memcpy(packet + 6, payload, 80);

  // Calculate payload checksum and add to packet
  UbxChecksum checksum = calculateChecksum(payload, 80, 0xFF, 0x01);
  packet[86] = checksum.ckA;
  packet[87] = checksum.ckB;

  // Hand the packet off to BLE
  bleSendPacket(packet, 88);
  bleSentPacketCount++;
}

// Periodically print packet rate and GNSS/IMU debug stats over serial
static void telemetrySerialReport() {
  const unsigned long now = millis();

  if ((now - lastReportMs) >= STATS_REPORT_INTERVAL_MS) {
    float elapsed = (now - lastReportMs) / 1000.0;
    float bleRate = bleSentPacketCount / elapsed;
    float gnssRate = gnssEpochCount / elapsed;
    // Additional satellite info for debugging: number of satellites, fix type,
    // horizontal accuracy, and lat/lon
    uint8_t sats = 0, fix = 0;
    uint32_t hAcc = 0, tAcc = 0;
    double lat = 0.0, lon = 0.0;
    if (pvt != nullptr) {
      sats = pvt->numSV;
      fix = pvt->fixType;
      hAcc = pvt->hAcc;
      tAcc = pvt->tAcc;
      lat = pvt->lat * 1e-7;
      lon = pvt->lon * 1e-7;
    }
    // Convert filtered IMU values to protocol units for display
    ImuProtocolUnits imu = imuReadProtocolUnits();
    // Print out the informational report
    Serial.printf(
        "BLE: %.2fHz | GNSS: %.2fHz | SV: %u | Fix: %u | TAcc: %uns | HAcc: "
        "%umm | Lat: %.7f | Lon: %.7f | milliG: X=%d Y=%d "
        "Z=%d | centiDeg/s: X=%d Y=%d Z=%d\n",
        bleRate, gnssRate, sats, fix, tAcc, hAcc, lat, lon, imu.gX, imu.gY,
        imu.gZ, imu.rX, imu.rY, imu.rZ);

    // Reset packet and epoch counts for the next report
    bleSentPacketCount = 0;
    gnssEpochCount = 0;

    // Update the last report timestamp
    lastReportMs = now;
  }
}

// --- Simple startup - nothing really to do here ---
void telemetryBegin() { return; }

// On each new GNSS epoch, count it and (when connected) send a packet.
// Always try to send informational report over serial.
void telemetrySendIfReady() {
  if (const UBX_NAV_PVT_data_t *newPvt = gnssConsumePvt()) {
    pvt = newPvt;
    gnssEpochCount++;
    if (bleIsConnected()) {
      sendPacket();
    }
  }
  telemetrySerialReport();
}
