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
#include <stddef.h>
#include <stdint.h>

// ============================================================================
// Protocol plug-in contract - the single hand-off between data acquisition and
// wire serialization. See docs/multiprotocol-design.md.
//
// DELIBERATELY DEPENDENCY-FREE. No config.h, no Arduino.h - only fixed-width
// integer types. Two reasons, both load-bearing:
//
//   1. It lets an encoder be compiled and exercised against known inputs on a
//      host (test/harness.cpp), which is the only way to validate wire-format
//      correctness without owning every target device. Same precedent as
//      g_imu_trim - see docs/imu-trim-design.md section 5.9.
//   2. It lets one encoder serve two completely different BLE stacks. Nothing
//      here knows what Bluefruit or the ESP32 BLE library are, so a single
//      copy of the packing logic compiles for both.
// ============================================================================

// ----------------------------------------------------------------------------
// The canonical sample
// ----------------------------------------------------------------------------
//
// TWO RULES GOVERN WHAT GOES IN HERE, and both exist to stop one protocol's
// opinions leaking into another's:
//
// RULE 1 - u-blox native scaling, never reinterpreted. Latitude stays
//   deg x 1e7, speed stays mm/s, heading stays deg x 1e-5. RaceBox is largely
//   a re-frame of UBX-NAV-PVT so its encoder becomes a copy, and RaceChrono
//   happens to want deg x 1e7 for lat/lon as well. No value takes a rounding
//   trip it did not need.
//
// RULE 2 - RAW flag bits, never derived ones. This is the one that bites.
//   The RaceBox encoder clamps fixType, defines "valid fix" as
//   (fixType == 3 && gnssFixOK), and packs the battery byte. All three are
//   RaceBox POLICY, not data. If any of them were resolved here, every future
//   protocol would silently inherit those choices - and RaceChrono, whose fix
//   field is NMEA GGA quality rather than u-blox fixType, needs a DIFFERENT
//   derivation from the same raw inputs. A pre-clamped fixType would produce a
//   wrong answer that still looked plausible.
//
// So: fixType arrives raw (0,1,2,3,4,5 - not clamped), the validity booleans
// arrive individually, and battery arrives as percent + charging rather than a
// packed byte.
struct TelemetrySample {
  // --- GNSS, straight from UBX-NAV-PVT ---
  uint32_t iTOW; // GPS time of week, ms
  uint16_t year;
  uint8_t month, day, hour, min, sec;
  bool validDate, validTime, fullyResolved, validMag;
  uint32_t tAcc; // time accuracy estimate, ns
  int32_t nano;  // sub-second correction, ns. SIGNED, roughly +/-5e8.
  uint8_t fixType;      // RAW u-blox: 0 none, 1 DR, 2 2D, 3 3D, 4 GNSS+DR, 5 time
  bool gnssFixOK;       // receiver's own "within DOP/accuracy masks"
  bool headVehValid;    // fused VEHICLE heading valid (see design doc 14.1)
  uint8_t numSV;        // satellites used in the solution
  int32_t lon, lat;     // deg x 1e7
  int32_t height, hMSL; // ellipsoid / mean-sea-level altitude, mm
  uint32_t hAcc, vAcc;  // horizontal / vertical accuracy, mm
  int32_t gSpeed;       // ground speed, mm/s
  int32_t headMot;      // heading of MOTION (course over ground), deg x 1e-5
  uint32_t sAcc;        // speed accuracy, mm/s
  uint32_t headAcc;     // heading accuracy, deg x 1e-5
  uint16_t pDOP;        // position DOP, x 0.01 (note: NOT hDOP)

  // --- IMU, trim-corrected and decimated to the transmit rate ---
  int16_t accelX, accelY, accelZ; // milli-g
  int16_t gyroX, gyroY, gyroZ;    // centi-deg/s

  // --- Power ---
  uint8_t batteryPercent; // 0..100
  bool batteryCharging;   // USB present and able to charge the cell
};

// ----------------------------------------------------------------------------
// Frame emission
// ----------------------------------------------------------------------------

// Emits one frame on one of the active protocol's channels.
//
// A SINK rather than a return value, because one sample does not always mean
// one frame: RaceBox sends a single 88-byte packet, but RaceChrono sends 20
// bytes on its GPS characteristic and 3 more on GPS Time. One encode call, N
// frames, no allocation.
//
// Called SYNCHRONOUSLY, so an encoder may hand over a stack buffer without
// copying. An implementation that needs the bytes later must copy them.
//
// RETURNS whether the transport ACCEPTED the frame - never whether the peer
// received it, which BLE notify cannot tell anyone. An encoder emitting more
// than one frame per sample should stop on a false rather than ship a partial
// set: RaceChrono, for instance, sends a position on one characteristic and
// its timestamp on another, and a position with no timestamp beside it is
// worse than neither.
//
// WHAT false DETECTS IS PLATFORM-DEPENDENT, and deliberately so - each
// transport reports what its stack can actually observe:
//
//   nRF (Bluefruit)  BLEUart::write() returns a byte count, so a frame lost to
//                    an exhausted HVN queue - the real truncation risk - is
//                    caught.
//   ESP32            BLECharacteristic::notify() returns void; that stack
//                    cannot see a failed send at all. false there means the
//                    frame was refused before transmission (bad channel, or an
//                    MTU too small to carry it), which is that board's actual
//                    failure mode.
//
// So a true is weaker on some transports than others. It is never a delivery
// receipt anywhere, and code that treats it as one is wrong on every platform.
typedef bool (*TelemetryEmit)(uint8_t channel, const uint8_t *data, size_t len);

// Channel 0 is whatever the protocol considers its main data stream. Protocols
// with more channels number them from there, in the order their descriptor
// declares - which keeps channel identity out of a per-protocol enum that
// would have to grow with every protocol added.
constexpr uint8_t TELEMETRY_CHANNEL_PRIMARY = 0;

// TRANSPORT_NORDIC_UART has a FIXED two-channel shape: index 0 is the Tx
// (notify) stream and index 1 the Rx (write) stream, because that is what the
// Nordic UART service is. A protocol declaring that transport must lay its
// channel table out that way - and asserts that it does, with
// nordicUartShapeOk() at the end of this file. Naming the index here rather
// than in a protocol header is what stops the transport having to reach into
// one.
constexpr uint8_t TELEMETRY_CHANNEL_NORDIC_RX = 1;

// Largest inbound write a transport will buffer in one piece.
//
// On a DISCRETE transport a larger write is dropped whole and counted - handing
// a handler part of a message that looks complete is the failure mode this
// guards. On a STREAM transport the bytes are split across slots instead,
// which changes nothing because that transport never promised boundaries; only
// a drain exceeding the transport's whole queue is lost, and that is counted
// too.
constexpr size_t TELEMETRY_MAX_WRITE_LEN = 64;

// ----------------------------------------------------------------------------
// Protocol selection
// ----------------------------------------------------------------------------

// Identifiers for TELEMETRY_PROTOCOL in config.h. The selection is resolved by
// g_protocol_active.h, which is the ONE place a new protocol has to be wired
// in - see the note there.
//
// NUMBERED FROM 1 DELIBERATELY. An undefined macro evaluates to 0 in a
// preprocessor #if, so a build where these are somehow not visible would
// silently compare 0 == 0 and pick the first protocol. Starting at 1 makes
// that case match nothing and trip the #error instead.
#define PROTO_RACEBOX 1
// PROTO_RACECHRONO 2 - added in phase F, with the encoder that implements it.

// ----------------------------------------------------------------------------
// Required of every protocol header
// ----------------------------------------------------------------------------
//
// Alongside its descriptor, each g_proto_<name>.h must define:
//
//   constexpr size_t PROTOCOL_MAX_FRAME_LEN   largest frame the protocol emits
//
// A transport uses it to size the MTU it requests and to reject a link that
// cannot carry the protocol's frames. It is a constexpr rather than a
// descriptor field because both of those uses are compile-time. Omitting it is
// caught by g_protocol_active.h.

// ----------------------------------------------------------------------------
// Transport description
// ----------------------------------------------------------------------------
//
// PURE DATA, deliberately. Nothing here names a BLE library type, which is what
// lets one descriptor drive both Bluefruit and the ESP32 BLE stack from a
// single copy of the protocol definition. Each g_ble translates these fields
// into whatever its own library wants.

// Characteristic properties, as a protocol-neutral bitmask. Each stack honors
// them its own way - notably, ESP32 needs an explicit BLE2902 descriptor on a
// notify characteristic while Bluefruit adds the CCCD itself. That asymmetry
// is exactly what this abstracts.
enum ProtocolProps : uint8_t {
  PROP_READ = 1 << 0,
  PROP_NOTIFY = 1 << 1,
  PROP_WRITE = 1 << 2,     // write with response
  PROP_WRITE_NR = 1 << 3,  // write without response
};

// How a protocol's frames reach the client.
//
// This distinction exists for ONE reason: Bluefruit's BLEUart is doing real
// work on the RaceBox path - it splits an 88-byte notify when the MTU is small
// and manages TX backpressure internally - and re-implementing that on a path
// that already works is not a risk worth taking for the sake of a uniform
// abstraction.
//
// It is honored by the nRF builder and IGNORED by the ESP32 one, which has no
// BLEUart, does no chunking, and already builds discrete characteristics for
// everything. There, both kinds are the same code.
enum TransportKind : uint8_t {
  TRANSPORT_NORDIC_UART,   // a byte stream over the Nordic UART service
  TRANSPORT_GATT_CHANNELS, // discrete characteristics, one per channel
};

// One characteristic. Channels are addressed by their INDEX in the descriptor's
// array, which is what keeps channel identity out of a per-protocol enum that
// would have to grow every time a protocol is added.
struct ProtocolChannel {
  uint16_t uuid16;     // 16-bit UUID, or 0 to use uuid128 instead
  const char *uuid128; // 8-4-4-4-12 string; ignored unless uuid16 == 0
  uint8_t props;       // ProtocolProps bitmask
};

// Everything g_ble needs to stand up a protocol without knowing what it is.
struct ProtocolDescriptor {
  // Identity. The advertised device name is composed by g_ble as
  // "<modelName> <DEVICE_ID>" - DEVICE_ID is per-variant config, so the
  // concatenation happens there rather than here, which is what keeps this
  // header free of config.h.
  const char *modelName;
  const char *manufacturer; // null omits the Device Information Service
  const char *hwRev;
  const char *fwRev;

  // Primary service. Same uuid16-or-uuid128 convention as ProtocolChannel.
  uint16_t serviceUuid16;
  const char *serviceUuid128;

  TransportKind transport;
  const ProtocolChannel *channels;
  uint8_t channelCount;

  // Serialize one sample into one or more frames.
  void (*encode)(const TelemetrySample &, TelemetryEmit);

  // Handle a client write on a writable channel. Null if the protocol has
  // nothing to receive. Keeps command parsing out of the transport.
  //
  // RUNS ON THE LOOP THREAD, always. Transports receive on their own callback
  // context - Bluefruit's SoftDevice callback on nRF, the Bluedroid BTC task on
  // ESP32, neither of them the loop - so they buffer the bytes and dispatch
  // them from bleUpdate(). A handler therefore never races the code reading
  // whatever state it mutates.
  //
  // THAT SAFETY COMES WITH A DEADLINE. Being on the loop means a slow handler
  // is a CORRECTNESS problem, not a latency one: Serial1's receive ring is 64
  // bytes against a 100-byte NAV-PVT, so gnssPoll() must be reached roughly
  // every 5.5ms during a message or GNSS bytes are lost outright. Parse and
  // return. Anything that blocks - flash writes, waits, long loops - belongs
  // behind a flag this sets and the loop picks up later.
  //
  // `data` is valid ONLY for the duration of the call. Copy what you need.
  //
  // EVERY WRITE IS UNTRUSTED. The GATT is open by necessity - the apps expect
  // no pairing, no encryption, no allow-list (SEC-1) - so a write can come from
  // any central in radio range, with any content, at any rate, including
  // write-without-response floods. Validate everything before acting on it:
  // never trust a length, an index, a count or a value to be in range, and
  // treat a malformed command as something to drop, not something to repair.
  // Today raceboxOnWrite() is empty, so this is inert; the first protocol with
  // a real command set (phase F's CAN-filter channel) is where it starts to
  // matter, and where an unchecked field becomes the device's attack surface.
  // The transport already bounds the damage it can do upstream - 8-slot ring,
  // overflow counted and dropped - so the handler only has to be correct, not
  // defensive about volume.
  //
  // MESSAGE BOUNDARIES ARE NOT GUARANTEED, and cannot be on every transport:
  //
  //   TRANSPORT_GATT_CHANNELS  each call is one client write; boundaries hold.
  //   TRANSPORT_NORDIC_UART    a BYTE STREAM. Two client writes can arrive as
  //                            one call and one write can split across two.
  //                            Nothing reassembles them.
  //
  // A protocol needing framing must implement it. Fixed-length, self-delimiting
  // commands (RaceChrono's CAN filter, for one) survive either transport as-is.
  //
  // Delivery is best-effort. A transport buffers a small number of pending
  // writes; beyond that it drops and counts them, and a dropped command is
  // reported rather than silently swallowed. A protocol whose configuration
  // must not half-apply should be designed to be re-drivable.
  void (*onWrite)(uint8_t channel, const uint8_t *data, size_t len);
};

// ----------------------------------------------------------------------------
// TRANSPORT_NORDIC_UART's fixed shape, as a compile-time check (API-4)
// ----------------------------------------------------------------------------
//
// On nRF this transport IS Bluefruit's BLEUart, which builds its GATT with the
// three UUIDs below hard-coded - Tx notify-only, Rx write and write-without-
// response - and never reads a descriptor's channel table. The ESP32 has no
// BLEUart and builds its GATT FROM that table. So a descriptor declaring this
// transport gets the same GATT on both families only if its table matches
// BLEUart exactly, and a mismatch is invisible on the nRF, which never looks.
// Until this existed, prose was the only thing saying so.
//
// A protocol declaring the transport asserts this on its OWN descriptor, in its
// own .cpp, so the check follows whatever transport the descriptor declares:
//
//   static_assert(D.transport != TRANSPORT_NORDIC_UART || nordicUartShapeOk(D),
//                 "...");
//
// Single-return constexpr functions throughout: the nRF core builds C++11.
namespace nordic_uart {
constexpr const char *kServiceUuid = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr const char *kRxUuid = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
constexpr const char *kTxUuid = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

constexpr char lower(char c) {
  return (c >= 'A' && c <= 'Z') ? (char)(c - 'A' + 'a') : c;
}

// Case-insensitive, because BLEUart holds these as bytes: letter case is not
// part of the GATT. A null string never matches.
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

// True if `d` is laid out exactly as BLEUart serves it: the Nordic UART service
// by its 128-bit UUID, and exactly two channels - TELEMETRY_CHANNEL_PRIMARY the
// Tx characteristic (notify only), TELEMETRY_CHANNEL_NORDIC_RX the Rx one
// (write, with and without response). Properties must match EXACTLY, not
// merely include these: an extra read or notify would be a characteristic the
// ESP32 serves and the nRF does not.
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
