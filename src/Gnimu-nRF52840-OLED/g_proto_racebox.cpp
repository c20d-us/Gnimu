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

#include "g_proto_racebox.h"
#include "g_ubx_helpers.h"

void raceboxEncode(const TelemetrySample &s, TelemetryEmit emit) {
  // The 80-byte payload is built in place inside the packet buffer (it
  // occupies bytes 6..85), so no separate staging buffer or copy is needed.
  uint8_t packet[RACEBOX_PACKET_LEN] = {0};
  uint8_t *payload = packet + 6;

  // Casts pin each field to its RaceBox protocol wire width (U1/U2/U4/I4),
  // so the writeLittleEndian overload is correct regardless of the source
  // field types.
  writeLittleEndian(payload, 0, (uint32_t)s.iTOW); // U4
  writeLittleEndian(payload, 4, (uint16_t)s.year); // U2
  writeLittleEndian(payload, 6, (uint8_t)s.month); // U1
  writeLittleEndian(payload, 7, (uint8_t)s.day);   // U1
  writeLittleEndian(payload, 8, (uint8_t)s.hour);  // U1
  writeLittleEndian(payload, 9, (uint8_t)s.min);   // U1
  writeLittleEndian(payload, 10, (uint8_t)s.sec);  // U1

  // Offset 11: Validity Flags
  uint8_t validityFlags = 0;
  if (s.validDate)
    validityFlags |= (1 << 0); // Bit 0: valid date
  if (s.validTime)
    validityFlags |= (1 << 1); // Bit 1: valid time
  if (s.fullyResolved)
    validityFlags |= (1 << 2); // Bit 2: fully resolved
  if (s.validMag)
    validityFlags |= (1 << 3); // Bit 3: valid magnetic declination
  writeLittleEndian(payload, 11, validityFlags);

  // Offset 12: Time Accuracy
  writeLittleEndian(payload, 12, (uint32_t)s.tAcc); // U4

  // Offset 16: Nanoseconds
  writeLittleEndian(payload, 16, (int32_t)s.nano); // I4

  // Offset 20: Fix Status
  // Protocol only defines 0 (no fix), 2 (2D fix), 3 (3D fix).
  // Clamp any other u-blox fix types (e.g. 1=DR only, 4=GNSS+DR) to 0 (no fix).
  //
  // PROTOCOL POLICY, not data - which is exactly why TelemetrySample carries
  // the raw fixType and this clamp lives here. RaceChrono's fix field uses
  // NMEA GGA semantics and needs a different derivation from the same input.
  uint8_t safeFixType = (s.fixType == 2 || s.fixType == 3) ? s.fixType : 0;
  writeLittleEndian(payload, 20, safeFixType);

  // Offset 21: Fix Status Flags
  uint8_t fixStatusFlags = 0;

  // Bit 0: valid fix - a 3D fix that the receiver also reports as within its
  // DOP/accuracy masks (gnssFixOK), the strictest read of "valid".
  if (s.fixType == 3 && s.gnssFixOK) {
    fixStatusFlags |= (1 << 0);
  }

  // Bit 5: valid heading. Read from the SAME sample being transmitted, so it
  // can neither block nor describe a different epoch than the rest of this
  // packet. Measured to be permanently zero on M10 hardware - see
  // docs/multiprotocol-design.md section 14.1. Preserved as-is regardless:
  // phase B changes no behavior.
  if (s.headVehValid) {
    fixStatusFlags |= (1 << 5);
  }
  writeLittleEndian(payload, 21, fixStatusFlags);

  // Offset 22: Date/Time Flags
  uint8_t dateTimeFlags = 0;
  if (s.validTime)
    dateTimeFlags |= (1 << 5); // Available confirmation of Date/Time Validity
  if (s.validDate)
    dateTimeFlags |= (1 << 6); // Confirmed UTC Date Validity
  if (s.validTime && s.fullyResolved)
    dateTimeFlags |= (1 << 7); // Confirmed UTC Time Validity
  writeLittleEndian(payload, 22, dateTimeFlags);

  // Offset 23: Number of SVs
  writeLittleEndian(payload, 23, (uint8_t)s.numSV); // U1

  // Remaining fields, mostly direct mappings from u-blox data
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

  // Offset 66: Lat/Lon Flags
  //
  // NOTE the asymmetry with offset 20: this tests the RAW fixType while the
  // fix-status byte above reports the CLAMPED one. For a raw fixType of 4 or
  // 5 the packet therefore says "no fix" and "coordinates valid" at the same
  // time. That is pre-existing behavior, preserved deliberately - see
  // docs/multiprotocol-design.md section 14.2. Changing it is a separate,
  // visible commit, not something to smuggle into a refactor.
  uint8_t latLonFlags = 0;
  if (s.fixType < 2) {       // If no 2D/3D fix, coordinates are invalid
    latLonFlags |= (1 << 0); // Bit 0: Invalid Latitude, Longitude, WGS
                             // Altitude, and MSL Altitude
  }
  writeLittleEndian(payload, 66, latLonFlags);

  // Offset 67: Battery status (1 byte) - bit 7 = charging, bits 0-6 = percent.
  //
  // PROTOCOL POLICY. This packing used to live in batteryProtocolByte() in
  // g_battery, which put a RaceBox wire format inside a hardware module. The
  // > 100 clamp is defensive (voltageToPercent already caps at 100) and is
  // kept verbatim so the output stays byte-identical.
  const uint8_t percent = s.batteryPercent > 100 ? 100 : s.batteryPercent;
  const uint8_t batteryByte =
      (uint8_t)((s.batteryCharging ? 0x80 : 0x00) | (percent & 0x7F));
  writeLittleEndian(payload, 67, batteryByte);

  // Offset 68-78: IMU data
  writeLittleEndian(payload, 68, s.accelX);
  writeLittleEndian(payload, 70, s.accelY);
  writeLittleEndian(payload, 72, s.accelZ);
  writeLittleEndian(payload, 74, s.gyroX);
  writeLittleEndian(payload, 76, s.gyroY);
  writeLittleEndian(payload, 78, s.gyroZ);

  // Add RaceBox protocol header
  packet[0] = 0xB5;
  packet[1] = 0x62;
  packet[2] = 0xFF; // Message Class: RaceBox Data Message
  packet[3] = 0x01; // Message ID: RaceBox Data Message
  packet[4] = 80;   // Payload size
  packet[5] = 0;

  // Calculate payload checksum and add to packet
  UbxChecksum checksum = calculateChecksum(payload, 80, 0xFF, 0x01);
  packet[86] = checksum.ckA;
  packet[87] = checksum.ckB;

  emit(TELEMETRY_CHANNEL_PRIMARY, packet, RACEBOX_PACKET_LEN);
}

// ----------------------------------------------------------------------------
// Descriptor
// ----------------------------------------------------------------------------

// Channel table. Order defines the channel indices (RACEBOX_CHANNEL_TX / _RX),
// and encode() emits on TELEMETRY_CHANNEL_PRIMARY, which is index 0 - the Tx
// characteristic.
//
// On the nRF build these UUIDs are not read: TRANSPORT_NORDIC_UART hands the
// job to Bluefruit's BLEUart, whose UUIDs ARE these values. On ESP32, which has
// no BLEUart, this table is what the characteristics are built from. Both paths
// must therefore produce the same GATT - which they do only because the Nordic
// UART UUIDs and the RaceBox UUIDs are the same three values.
static constexpr ProtocolChannel raceboxChannels[] = {
    {0, RACEBOX_CHARACTERISTIC_TX_UUID, PROP_NOTIFY},
    {0, RACEBOX_CHARACTERISTIC_RX_UUID, PROP_WRITE | PROP_WRITE_NR},
};

// Client writes on the Rx characteristic. RaceBox's command set is not
// implemented - the previous transport-level handler only logged the bytes, and
// preserving that exactly is the point of this phase. It lives here rather than
// in g_ble so that a protocol which DOES need command parsing (RaceChrono's CAN
// filter characteristic, for one) has somewhere protocol-specific to put it.
static void raceboxOnWrite(uint8_t channel, const uint8_t *data, size_t len) {
  (void)channel;
  (void)data;
  (void)len;
  // Deliberately empty. g_ble logs the bytes it received before calling this,
  // which is all the old rxCallback did.
}

// constexpr, not just const, so the assert below can read the descriptor
// itself rather than a copy of its fields. The extern declaration in
// g_proto_racebox.h still gives it external linkage.
constexpr ProtocolDescriptor RACEBOX_PROTOCOL = {
    RACEBOX_MODEL,
    RACEBOX_MANUFACTURER,
    RACEBOX_HARDWARE_VERSION,
    RACEBOX_FIRMWARE_VERSION,
    0, // no 16-bit service UUID; RaceBox uses the 128-bit Nordic one
    RACEBOX_SERVICE_UUID,
    TRANSPORT_NORDIC_UART,
    raceboxChannels,
    (uint8_t)(sizeof(raceboxChannels) / sizeof(raceboxChannels[0])),
    raceboxEncode,
    raceboxOnWrite,
};

// The descriptor against the transport it declares (API-4). On nRF the Nordic
// UART transport is BLEUart, which ignores the table above; on ESP32 the table
// IS the GATT. This is what keeps the two identical - see nordicUartShapeOk()
// in g_protocol.h.
static_assert(RACEBOX_PROTOCOL.transport != TRANSPORT_NORDIC_UART ||
                  nordicUartShapeOk(RACEBOX_PROTOCOL),
              "ERROR: RACEBOX_PROTOCOL declares TRANSPORT_NORDIC_UART, but its "
              "service UUID or channel table does not match the Nordic UART "
              "service BLEUart serves on nRF (Tx notify at index 0, Rx write "
              "at index 1, 6E40000x UUIDs) - the two families would present "
              "different GATTs.");
