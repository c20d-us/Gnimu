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

#pragma once
#include "g_protocol.h"

// ============================================================================
// RaceBox Data Message encoder.
//
// Serializes a TelemetrySample into the 88-byte RaceBox Data Message: a UBX
// envelope (class 0xFF, id 0x01) around an 80-byte little-endian payload.
//
// This module holds every RaceBox-specific decision about the wire format,
// which is the point of it existing. Notably it OWNS the three policies that
// used to be inline in sendPacket() and must not live in TelemetrySample:
// the fixType clamp, the definition of "valid fix", and the battery byte
// packing. See g_protocol.h's Rule 2.
//
// Includes only g_protocol.h - no config.h, no Arduino.h - so it compiles
// unchanged against both BLE stacks and can be exercised on a host by
// test/harness.cpp.
// ============================================================================

// ----------------------------------------------------------------------------
// Protocol identity and topology
// ----------------------------------------------------------------------------
//
// These moved out of every variant's config.h, where they were byte-identical
// but UNCHECKED - config.h cannot join the common set (DEVICE_ID, pins and IMU
// ranges legitimately differ per variant), so nothing prevented them drifting
// apart and silently breaking app compatibility. Here they sit in an
// all-variant file that check_common.sh enforces.
//
// They are compatibility requirements, not tunables. Changing any of them stops
// RaceBox-compatible apps recognizing the device.
#define RACEBOX_MODEL "RaceBox Mini"
#define RACEBOX_MANUFACTURER "RaceBox"
#define RACEBOX_HARDWARE_VERSION "1"
#define RACEBOX_FIRMWARE_VERSION "3.3"

// The Nordic UART UUIDs, which are exactly the RaceBox service/Tx/Rx UUIDs.
// That coincidence is why Bluefruit's BLEUart implements this transport
// natively - see TRANSPORT_NORDIC_UART in g_protocol.h. The ESP32 stack has no
// BLEUart and builds these characteristics explicitly from the channel table
// below, so both must stay in step - which g_proto_racebox.cpp asserts at
// compile time with nordicUartShapeOk() (g_protocol.h).
#define RACEBOX_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RACEBOX_CHARACTERISTIC_RX_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define RACEBOX_CHARACTERISTIC_TX_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Channel indices into RACEBOX_PROTOCOL.channels.
constexpr uint8_t RACEBOX_CHANNEL_TX = 0; // notify - telemetry out
constexpr uint8_t RACEBOX_CHANNEL_RX = 1; // write - commands in

// Total frame size: 6-byte UBX header + 80-byte payload + 2-byte checksum.
constexpr size_t RACEBOX_PACKET_LEN = 88;

// The largest frame this protocol will ever emit. Every protocol header must
// define this - see g_protocol.h. It exists as a constexpr rather than a
// ProtocolDescriptor field because a struct member cannot be used in a
// static_assert or to derive a compile-time MTU request, which are the two
// things a transport needs it for.
//
// RaceBox emits exactly one frame size, so this is simply the packet length.
constexpr size_t PROTOCOL_MAX_FRAME_LEN = RACEBOX_PACKET_LEN;

// ----------------------------------------------------------------------------
// UUID format validation
// ----------------------------------------------------------------------------
//
// Moved here with the constants, and now protocol-scoped rather than firing
// unconditionally: a build selecting a protocol with no 128-bit UUIDs should
// not have to keep dead RaceBox constants alive just to satisfy an assert in
// config.h.
namespace uuid_format {
constexpr bool isHexDigit(char c) {
  return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') ||
         (c >= 'A' && c <= 'F');
}
// Hyphens are required at positions 8, 13, 18, and 23; every other position
// (of the required 36 total) must be a hex digit.
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

// Build the packet and hand it to `emit` on TELEMETRY_CHANNEL_PRIMARY.
// Emits exactly one frame per call. The buffer passed to `emit` is stack-local
// and valid only for the duration of that call.
void raceboxEncode(const TelemetrySample &s, TelemetryEmit emit);

// Everything g_ble needs to stand up this protocol.
extern const ProtocolDescriptor RACEBOX_PROTOCOL;
