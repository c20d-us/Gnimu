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
#include <stddef.h>
#include <stdint.h>

// ============================================================================
// The seam between the BLE DRIVER (g_ble.cpp - identical in every tree) and
// the MCU's BLE STACK (g_ble_port_<mcu>.cpp - one per core).
//
// The driver owns every decision: what the device advertises as, whether a
// frame may be sent and what a refusal means, the counters and one-shot log
// lines, the inbound write queue, and the connection-session lifecycle. The
// port owns the stack's mechanism: bring-up, TX power, the GATT it builds from
// the descriptor, advertising, the stack callbacks, and the few primitives
// below that answer "is it connected / subscribed / how big a frame fits /
// send this".
//
//   g_ble_port_esp32.cpp  - Bluedroid (esp32 Arduino core): a discrete GATT
//                           built from the channel table for every transport
//   g_ble_port_nrf52.cpp  - Bluefruit (Adafruit nRF52 core): BLEUart for
//                           TRANSPORT_NORDIC_UART
//
// Ports are named after the CORE, like g_gnss_port: what varies is the stack.
//
// SINGLE CENTRAL is a contract, not an accident. Both ports serve one
// connection at a time (the ESP32's getConnId(), Bluefruit's Connection(0)),
// and the session logic in g_ble.cpp assumes it. The README's privacy note
// describes the consequence: the first client to connect holds the device.
//
// CONCURRENCY. Everything below is called from the loop, EXCEPT
// bleRxFromCallback(), which is the one entry from a stack callback. A port's
// callbacks may otherwise only set its own flags (std::atomic where a flag
// publishes other data, or where the callback runs on another core) - never
// log, never call into the driver's state. See docs/architecture-runtime.md.
//
// Arduino-free, like g_imu_sensor.h, so the driver compiles on a host against a
// fake port (test/ble).
// ============================================================================

// What the device presents itself as, built once by the driver from the active
// descriptor and DEVICE_ID so every stack advertises exactly the same identity.
// manufacturer == nullptr means "no Device Information Service".
struct BleIdentity {
  const char *name; // advertised: "<modelName> <DEVICE_ID>"
  const char *model;
  const char *serial;
  const char *manufacturer;
  const char *fwRev;
  const char *hwRev;
};

// --- Implemented by the port ------------------------------------------------

// Bring the stack up: TX power (reporting it), the service and characteristics
// from `proto`, the Device Information Service when id.manufacturer is set, and
// advertising. Returns false if the stack could not be brought up; the driver
// then leaves BLE off for the session.
bool blePortBegin(const BleIdentity &id, const ProtocolDescriptor *proto);

// True while a central is connected.
bool blePortConnected();

// Incremented once per connection, in the connect callback. The driver treats
// any change as "a new session began", which is what lets it see a disconnect
// and reconnect that both fall between two loop passes.
uint32_t blePortSessionCount();

// The stack's reason code for the most recent disconnect.
uint8_t blePortDisconnectReason();

// Whether the central has enabled notifications on `channel`. Only asked about
// channels with PROP_NOTIFY.
bool blePortSubscribed(uint8_t channel);

// The largest frame `channel` can carry in one notify right now: the ATT MTU
// minus the 3-byte header, or SIZE_MAX where the stack fragments on its own.
size_t blePortMaxFrame(uint8_t channel);

// The current ATT MTU, for logging.
uint16_t blePortMtu();

// Hand one frame to the stack. Returns the bytes it ACCEPTED - not a delivery
// receipt, and on a stack whose notify returns void, simply `len`. See
// TelemetryEmit in g_protocol.h.
size_t blePortSend(uint8_t channel, const uint8_t *data, size_t len);

// Stack housekeeping, every loop: whatever this stack needs that the driver
// does not decide (re-advertising, the Battery Service).
void blePortUpdate();

// Disconnect any central and stop advertising, for a power state that wants
// BLE silent. The driver only calls this after a successful blePortBegin().
void blePortStop();

// --- Implemented by the driver ----------------------------------------------

// Queue bytes a central wrote on `channel`, for dispatch to the protocol's
// onWrite on the loop. THE ONLY DRIVER FUNCTION A CALLBACK MAY CALL.
//
// `wholeMessage` says what one call represents. true: exactly one client
// write, so an oversized one is dropped whole - truncating would hand the
// protocol part of a message that looks complete. false: a slice of a byte
// stream, which is split across queue slots, since the stream never promised
// boundaries. Either way a byte that cannot be queued is counted in
// bleDroppedWrites().
void bleRxFromCallback(uint8_t channel, const uint8_t *data, size_t len,
                       bool wholeMessage);
