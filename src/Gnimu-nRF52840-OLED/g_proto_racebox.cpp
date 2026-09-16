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

#include "g_proto_racebox.h"
#include "g_ubx_helpers.h"

void raceboxEncode(const TelemetrySample &s, TelemetryEmit emit) {
  // The payload is built in place at bytes 6-85.
  uint8_t packet[RACEBOX_PACKET_LEN] = {0};
  uint8_t *payload = packet + 6;

  // Casts set each field's wire width.
  writeLittleEndian(payload, 0, (uint32_t)s.iTOW); // U4
  writeLittleEndian(payload, 4, (uint16_t)s.year); // U2
  writeLittleEndian(payload, 6, (uint8_t)s.month); // U1
  writeLittleEndian(payload, 7, (uint8_t)s.day);   // U1
  writeLittleEndian(payload, 8, (uint8_t)s.hour);  // U1
  writeLittleEndian(payload, 9, (uint8_t)s.min);   // U1
  writeLittleEndian(payload, 10, (uint8_t)s.sec);  // U1

  // Offset 11: validity flags
  uint8_t validityFlags = 0;
  if (s.validDate)
    validityFlags |= (1 << 0); // valid date
  if (s.validTime)
    validityFlags |= (1 << 1); // valid time
  if (s.fullyResolved)
    validityFlags |= (1 << 2); // fully resolved
  if (s.validMag)
    validityFlags |= (1 << 3); // valid magnetic declination
  writeLittleEndian(payload, 11, validityFlags);

  // Offset 12: time accuracy
  writeLittleEndian(payload, 12, (uint32_t)s.tAcc); // U4

  // Offset 16: nanoseconds
  writeLittleEndian(payload, 16, (int32_t)s.nano); // I4

  // Offset 20: fix status. RaceBox defines only 0, 2, and 3; anything else is
  // sent as 0.
  uint8_t safeFixType = (s.fixType == 2 || s.fixType == 3) ? s.fixType : 0;
  writeLittleEndian(payload, 20, safeFixType);

  // Offset 21: fix status flags
  uint8_t fixStatusFlags = 0;

  // Bit 0: valid fix, a 3D fix with gnssFixOK.
  if (s.fixType == 3 && s.gnssFixOK) {
    fixStatusFlags |= (1 << 0);
  }

  // Bit 5: valid heading. Always 0 on M10 receivers.
  if (s.headVehValid) {
    fixStatusFlags |= (1 << 5);
  }
  writeLittleEndian(payload, 21, fixStatusFlags);

  // Offset 22: date/time flags
  uint8_t dateTimeFlags = 0;
  if (s.validTime)
    dateTimeFlags |= (1 << 5); // date/time validity confirmation available
  if (s.validDate)
    dateTimeFlags |= (1 << 6); // UTC date confirmed
  if (s.validTime && s.fullyResolved)
    dateTimeFlags |= (1 << 7); // UTC time confirmed
  writeLittleEndian(payload, 22, dateTimeFlags);

  // Offset 23: satellites used
  writeLittleEndian(payload, 23, (uint8_t)s.numSV); // U1

  writeLittleEndian(payload, 24, (int32_t)s.lon);      // I4
  writeLittleEndian(payload, 28, (int32_t)s.lat);      // I4
  writeLittleEndian(payload, 32, (int32_t)s.height);   // I4
  writeLittleEndian(payload, 36, (int32_t)s.hMSL);     // I4
  writeLittleEndian(payload, 40, (uint32_t)s.hAcc);    // U4
  writeLittleEndian(payload, 44, (uint32_t)s.vAcc);    // U4
  writeLittleEndian(payload, 48, (int32_t)s.gSpeed);   // I4
  writeLittleEndian(payload, 52, (int32_t)s.headMot);  // I4
  writeLittleEndian(payload, 56, (uint32_t)s.sAcc);    // U4
  writeLittleEndian(payload, 60, (uint32_t)s.headAcc); // U4
  writeLittleEndian(payload, 64, (uint16_t)s.pDOP);    // U2

  // Offset 66: lat/lon flags. Tests the raw fixType, so fixType 4 or 5 reports
  // "no fix" at offset 20 but valid coordinates here.
  uint8_t latLonFlags = 0;
  if (s.fixType < 2) {
    latLonFlags |= (1 << 0); // lat/lon and altitudes invalid
  }
  writeLittleEndian(payload, 66, latLonFlags);

  // Offset 67: battery. Bit 7 = charging, bits 0-6 = percent.
  const uint8_t percent = s.batteryPercent > 100 ? 100 : s.batteryPercent;
  const uint8_t batteryByte =
      (uint8_t)((s.batteryCharging ? 0x80 : 0x00) | (percent & 0x7F));
  writeLittleEndian(payload, 67, batteryByte);

  // Offsets 68-79: IMU
  writeLittleEndian(payload, 68, s.accelX);
  writeLittleEndian(payload, 70, s.accelY);
  writeLittleEndian(payload, 72, s.accelZ);
  writeLittleEndian(payload, 74, s.gyroX);
  writeLittleEndian(payload, 76, s.gyroY);
  writeLittleEndian(payload, 78, s.gyroZ);

  // UBX header
  packet[0] = 0xB5;
  packet[1] = 0x62;
  packet[2] = 0xFF; // class: RaceBox Data Message
  packet[3] = 0x01; // id: RaceBox Data Message
  packet[4] = 80;   // payload length
  packet[5] = 0;

  UbxChecksum checksum = calculateChecksum(payload, 80, 0xFF, 0x01);
  packet[86] = checksum.ckA;
  packet[87] = checksum.ckB;

  emit(TELEMETRY_CHANNEL_PRIMARY, packet, RACEBOX_PACKET_LEN);
}

// Descriptor

// Order sets RACEBOX_CHANNEL_TX / _RX. The nRF port ignores this table (BLEUart
// uses the same UUIDs); the ESP32 port builds its GATT from it.
static constexpr ProtocolChannel raceboxChannels[] = {
    {0, RACEBOX_CHARACTERISTIC_TX_UUID, PROP_NOTIFY},
    {0, RACEBOX_CHARACTERISTIC_RX_UUID, PROP_WRITE | PROP_WRITE_NR},
};

// RaceBox commands are not currently implemented.
// g_ble logs each write before calling this.
static void raceboxOnWrite(uint8_t channel, const uint8_t *data, size_t len) {
  (void)channel;
  (void)data;
  (void)len;
}

// constexpr so the asserts below can read it.
constexpr ProtocolDescriptor RACEBOX_PROTOCOL = {
    RACEBOX_MODEL,
    RACEBOX_MANUFACTURER,
    RACEBOX_HARDWARE_VERSION,
    RACEBOX_FIRMWARE_VERSION,
    0, // no 16-bit service UUID
    RACEBOX_SERVICE_UUID,
    raceboxChannels,
    (uint8_t)(sizeof(raceboxChannels) / sizeof(raceboxChannels[0])),
    raceboxEncode,
    raceboxOnWrite,
};

static_assert(RACEBOX_PROTOCOL.channelCount == PROTOCOL_CHANNEL_COUNT,
              "ERROR: PROTOCOL_CHANNEL_COUNT in g_proto_racebox.h disagrees "
              "with RACEBOX_PROTOCOL.");

// Keeps the ESP32 GATT identical to BLEUart's.
static_assert(PROTOCOL_TRANSPORT != TRANSPORT_NORDIC_UART ||
                  nordicUartShapeOk(RACEBOX_PROTOCOL),
              "ERROR: g_proto_racebox.h declares TRANSPORT_NORDIC_UART, but "
              "RACEBOX_PROTOCOL's service UUID or channel table does not "
              "match the Nordic UART service BLEUart serves on nRF (Tx notify "
              "at index 0, Rx write at index 1, 6E40000x UUIDs) - the two "
              "families would present different GATTs.");
