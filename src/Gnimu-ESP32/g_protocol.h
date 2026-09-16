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
#include <stddef.h>
#include <stdint.h>

// Protocol contract: the hand-off between data acquisition and wire encoding.
// No config.h or Arduino.h, so encoders build on a host (test/harness.cpp) and
// one encoder serves both BLE stacks.

// The canonical sample
//
// Rule 1: u-blox native scaling, never converted (lat/lon deg x 1e7, speed
// mm/s, heading deg x 1e-5).
//
// Rule 2: raw flags, never derived. fixType is unclamped, validity flags are
// separate, and battery is percent + charging.
struct TelemetrySample {
  // GNSS, from UBX-NAV-PVT
  uint32_t iTOW; // GNSS epoch time of week, ms
  uint16_t year;
  uint8_t month, day, hour, min, sec;
  bool validDate, validTime, fullyResolved, validMag;
  uint32_t tAcc;        // time accuracy, ns
  int32_t nano;         // sub-second correction, ns, signed (about +/-5e8)
  uint8_t fixType;      // 0 none, 1 DR, 2 2D, 3 3D, 4 GNSS+DR, 5 time
  bool gnssFixOK;       // within the receiver's DOP/accuracy masks
  bool headVehValid;    // fused vehicle heading valid
  uint8_t numSV;        // satellites used
  int32_t lon, lat;     // deg x 1e7
  int32_t height, hMSL; // ellipsoid / mean-sea-level altitude, mm
  uint32_t hAcc, vAcc;  // horizontal / vertical accuracy, mm
  int32_t gSpeed;       // ground speed, mm/s
  int32_t headMot;      // heading of motion, deg x 1e-5
  uint32_t sAcc;        // speed accuracy, mm/s
  uint32_t headAcc;     // heading accuracy, deg x 1e-5
  uint16_t pDOP;        // position DOP x 0.01 (not hDOP)

  // IMU, trim-corrected, one sample per epoch
  int16_t accelX, accelY, accelZ; // milli-g
  int16_t gyroX, gyroY, gyroZ;    // centi-deg/s

  // Power
  uint8_t batteryPercent; // 0-100
  bool batteryCharging;   // USB present and able to charge
};

// Frame emission

// Sink for one frame on one of the protocol's channels; an encode call may emit
// several. Called synchronously, so `data` may be a stack buffer.
//
// Returns whether the transport accepted the frame, never whether it was
// delivered. An encoder emitting several frames stops on false. What false
// catches depends on the stack:
//
//   nRF (Bluefruit)  write() returns a byte count, so an exhausted HVN queue is
//                    caught.
//   ESP32            notify() returns void, so false means refused before
//                    sending (bad channel or MTU too small).
typedef bool (*TelemetryEmit)(uint8_t channel, const uint8_t *data, size_t len);

// The protocol's main data stream. Other channels follow in descriptor order.
constexpr uint8_t TELEMETRY_CHANNEL_PRIMARY = 0;

// TRANSPORT_NORDIC_UART is fixed: channel 0 is Tx (notify), channel 1 is Rx
// (write). Checked by nordicUartShapeOk() below.
constexpr uint8_t TELEMETRY_CHANNEL_NORDIC_RX = 1;

// Largest inbound write buffered as one piece. A discrete transport drops a
// larger write whole; a stream transport splits it across slots. Losses are
// counted.
constexpr size_t TELEMETRY_MAX_WRITE_LEN = 64;

// Protocol selection

// Values for TELEMETRY_PROTOCOL in config.h, resolved in g_protocol_active.h.
// Numbered from 1 so an undefined macro (which evaluates to 0) matches nothing.
#define PROTO_RACEBOX 1

// Required of every protocol header
//
// Each g_proto_<name>.h defines, alongside its descriptor:
//
//   constexpr size_t PROTOCOL_MAX_FRAME_LEN        largest frame emitted
//   constexpr uint8_t PROTOCOL_CHANNEL_COUNT       descriptor's channelCount
//   constexpr TransportKind PROTOCOL_TRANSPORT     how frames reach the client
//
// Transports use these at compile time to size the MTU request and reject a
// protocol they cannot serve. The protocol's .cpp asserts the channel count
// matches its descriptor; the transport has no descriptor field, so that the
// ports read one value rather than two kept equal.

// Transport description
//
// Plain data with no BLE library types. Each g_ble port translates it.

// Characteristic properties bitmask.
enum ProtocolProps : uint8_t {
  PROP_READ = 1 << 0,
  PROP_NOTIFY = 1 << 1,
  PROP_WRITE = 1 << 2,    // write with response
  PROP_WRITE_NR = 1 << 3, // write without response
};

// How frames reach the client. The nRF port serves TRANSPORT_NORDIC_UART with
// BLEUart, which fragments and handles backpressure. The ESP32 port builds
// discrete characteristics for both kinds.
enum TransportKind : uint8_t {
  TRANSPORT_NORDIC_UART,   // byte stream over the Nordic UART service
  TRANSPORT_GATT_CHANNELS, // one characteristic per channel
};

// One characteristic, addressed by its index in the descriptor's array.
struct ProtocolChannel {
  uint16_t uuid16;     // 16-bit UUID, or 0 to use uuid128
  const char *uuid128; // 8-4-4-4-12 string, used when uuid16 == 0
  uint8_t props;       // ProtocolProps bitmask
};

// Everything g_ble needs to serve a protocol.
struct ProtocolDescriptor {
  // Identity. g_ble advertises "<modelName> <DEVICE_ID>".
  const char *modelName;
  const char *manufacturer; // nullptr omits the Device Information Service
  const char *hwRev;
  const char *fwRev;

  // Primary service, same uuid16/uuid128 convention as ProtocolChannel.
  uint16_t serviceUuid16;
  const char *serviceUuid128;

  const ProtocolChannel *channels;
  uint8_t channelCount;

  // Encode one sample into one or more frames.
  void (*encode)(const TelemetrySample &, TelemetryEmit);

  // Handle a client write, or nullptr if the protocol takes none.
  //
  // Runs on the loop, dispatched from bleUpdate(). Keep it short: gnssPoll()
  // must run about every 5.5ms during a NAV-PVT message or GNSS bytes are
  // lost. Defer blocking work to a flag.
  //
  // `data` is valid only during the call.
  //
  // Writes are untrusted: the GATT is open to any central in range. Validate
  // every length, index, and value, and drop malformed commands.
  //
  // Message boundaries:
  //
  //   TRANSPORT_GATT_CHANNELS  one call per client write
  //   TRANSPORT_NORDIC_UART    byte stream; writes may merge or split
  //
  // Delivery is best-effort; overflowing writes are dropped and counted.
  void (*onWrite)(uint8_t channel, const uint8_t *data, size_t len);
};

// TRANSPORT_NORDIC_UART shape check
//
// On nRF, BLEUart hard-codes this GATT and ignores the channel table; the ESP32
// builds its GATT from the table. A protocol declaring this transport asserts
// the two match in its own .cpp:
//
//   static_assert(PROTOCOL_TRANSPORT != TRANSPORT_NORDIC_UART ||
//                     nordicUartShapeOk(D),
//                 "...");
//
// Single-return constexpr functions: the nRF core builds C++11.
namespace nordic_uart {
constexpr const char *kServiceUuid = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr const char *kRxUuid = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr const char *kTxUuid = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

constexpr char lower(char c) {
  return (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c;
}

// Case-insensitive. A null string never matches.
constexpr bool sameUuid(const char *a, const char *b) {
  return (a == nullptr || b == nullptr) ? false
         : (*a == '\0' || *b == '\0')
             ? (*a == *b)
             : (lower(*a) == lower(*b) && sameUuid(a + 1, b + 1));
}

constexpr bool channelIs(const ProtocolChannel &c, const char *uuid,
                         uint8_t props) {
  return c.uuid16 == 0 && sameUuid(c.uuid128, uuid) && c.props == props;
}
} // namespace nordic_uart

// True if `d` matches BLEUart exactly: the 128-bit Nordic UART service, two
// channels, Tx notify-only on TELEMETRY_CHANNEL_PRIMARY and Rx write +
// write-without-response on TELEMETRY_CHANNEL_NORDIC_RX. Properties must match
// exactly.
constexpr bool nordicUartShapeOk(const ProtocolDescriptor &d) {
  return d.serviceUuid16 == 0 &&
         nordic_uart::sameUuid(d.serviceUuid128, nordic_uart::kServiceUuid) &&
         d.channelCount == 2 && d.channels != nullptr &&
         nordic_uart::channelIs(d.channels[TELEMETRY_CHANNEL_PRIMARY],
                                nordic_uart::kTxUuid, PROP_NOTIFY) &&
         nordic_uart::channelIs(d.channels[TELEMETRY_CHANNEL_NORDIC_RX],
                                nordic_uart::kRxUuid,
                                PROP_WRITE | PROP_WRITE_NR);
}
