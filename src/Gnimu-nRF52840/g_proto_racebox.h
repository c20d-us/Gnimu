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

#pragma once
#include "g_protocol.h"

// RaceBox encoder: serializes a TelemetrySample into the 88-byte RaceBox Data
// Message, a UBX envelope (class 0xFF, id 0x01) around an 80-byte little-endian
// payload. Owns the fixType clamp, the valid-fix rule, and battery byte packing
// (see g_protocol.h, Rule 2).
//
// Includes only g_protocol.h, so it builds on a host for test/harness.cpp.

// Protocol identity and topology

// Required for app compatibility; do not change.
#define RACEBOX_MODEL "RaceBox Mini"
#define RACEBOX_MANUFACTURER "RaceBox"
#define RACEBOX_HARDWARE_VERSION "1"
#define RACEBOX_FIRMWARE_VERSION "3.3"

// The RaceBox UUIDs are the Nordic UART UUIDs, so Bluefruit's BLEUart serves
// them natively. The ESP32 builds them from the channel table below;
// g_proto_racebox.cpp checks the two match.
#define RACEBOX_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RACEBOX_CHARACTERISTIC_RX_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define RACEBOX_CHARACTERISTIC_TX_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Channel indices into RACEBOX_PROTOCOL.channels.
constexpr uint8_t RACEBOX_CHANNEL_TX = 0; // notify: telemetry out
constexpr uint8_t RACEBOX_CHANNEL_RX = 1; // write: commands in

// 6-byte UBX header + 80-byte payload + 2-byte checksum.
constexpr size_t RACEBOX_PACKET_LEN = 88;

// Required of every protocol header (see g_protocol.h).
constexpr size_t PROTOCOL_MAX_FRAME_LEN = RACEBOX_PACKET_LEN;
constexpr uint8_t PROTOCOL_CHANNEL_COUNT = 2;
constexpr TransportKind PROTOCOL_TRANSPORT = TRANSPORT_NORDIC_UART;

// UUID format validation

namespace uuid_format {
constexpr bool isHexDigit(char c) {
  return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') ||
         (c >= 'A' && c <= 'F');
}
// 36 characters: hyphens at 8, 13, 18, and 23, hex digits elsewhere.
constexpr bool isValid(const char *s, int i = 0) {
  return i == 36 ? s[i] == '\0'
         : (i == 8 || i == 13 || i == 18 || i == 23)
             ? (s[i] == '-' && isValid(s, i + 1))
             : (isHexDigit(s[i]) && isValid(s, i + 1));
}
} // namespace uuid_format

static_assert(uuid_format::isValid(RACEBOX_SERVICE_UUID),
              "ERROR: RACEBOX_SERVICE_UUID must be a standard 8-4-4-4-12 hex "
              "UUID string.");
static_assert(uuid_format::isValid(RACEBOX_CHARACTERISTIC_TX_UUID),
              "ERROR: RACEBOX_CHARACTERISTIC_TX_UUID must be a standard "
              "8-4-4-4-12 hex UUID string.");
static_assert(uuid_format::isValid(RACEBOX_CHARACTERISTIC_RX_UUID),
              "ERROR: RACEBOX_CHARACTERISTIC_RX_UUID must be a standard "
              "8-4-4-4-12 hex UUID string.");

// Encode one packet and pass it to `emit` on TELEMETRY_CHANNEL_PRIMARY. The
// buffer is valid only during the call.
void raceboxEncode(const TelemetrySample &s, TelemetryEmit emit);

// Everything g_ble needs to serve this protocol.
extern const ProtocolDescriptor RACEBOX_PROTOCOL;
