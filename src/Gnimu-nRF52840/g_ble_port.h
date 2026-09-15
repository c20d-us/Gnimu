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
#include <stddef.h>
#include <stdint.h>

// BLE port: the interface between the shared driver (g_ble.cpp) and each core's
// BLE stack.
//
//   g_ble_port_esp32.cpp  Bluedroid: a discrete GATT built from the channel
//                         table
//   g_ble_port_nrf52.cpp  Bluefruit: BLEUart for TRANSPORT_NORDIC_UART
//
// The driver makes all decisions (identity, send policy, counters, logging,
// write queue, sessions). The port provides the stack mechanism.
//
// One central at a time; g_ble.cpp assumes it.
//
// All functions are called from the loop except bleRxFromCallback(). Port
// callbacks may only set the port's own flags (std::atomic where needed): no
// logging, no calls into the driver. See docs/architecture-runtime.md.
//
// No Arduino dependency, so the driver builds on a host against test/ble.

// Device identity, built by the driver from the descriptor and DEVICE_ID.
// manufacturer == nullptr omits the Device Information Service.
struct BleIdentity {
  const char *name; // "<modelName> <DEVICE_ID>"
  const char *model;
  const char *serial;
  const char *manufacturer;
  const char *fwRev;
  const char *hwRev;
};

// Implemented by the port

// Bring up the stack: TX power, the service and characteristics from `proto`,
// the Device Information Service if id.manufacturer is set, and advertising.
// Returns false on failure; BLE then stays off.
bool blePortBegin(const BleIdentity &id, const ProtocolDescriptor *proto);

// True while a central is connected.
bool blePortConnected();

// Incremented in the connect callback. Any change means a new session, even if
// a disconnect and reconnect both happened between loop passes.
uint32_t blePortSessionCount();

// The stack's reason code for the last disconnect.
uint8_t blePortDisconnectReason();

// Whether notifications are enabled on `channel` (a PROP_NOTIFY channel).
bool blePortSubscribed(uint8_t channel);

// Largest frame one notify on `channel` can carry: ATT MTU - 3, or SIZE_MAX if
// the stack fragments.
size_t blePortMaxFrame(uint8_t channel);

// Current ATT MTU, for logging.
uint16_t blePortMtu();

// Hand one frame to the stack. Returns bytes accepted (`len` if the stack's
// notify returns void), not bytes delivered.
size_t blePortSend(uint8_t channel, const uint8_t *data, size_t len);

// Stack housekeeping (re-advertising, Battery Service). Called every loop.
void blePortUpdate();

// Disconnect any central and stop advertising. Only called after a successful
// blePortBegin().
void blePortStop();

// Implemented by the driver

// Queue bytes written on `channel` for the protocol's onWrite. The only driver
// function a callback may call.
//
// wholeMessage true: one client write; an oversized one is dropped whole.
// wholeMessage false: a byte-stream slice, split across queue slots as needed.
// Bytes that cannot be queued count in bleDroppedWrites().
void bleRxFromCallback(uint8_t channel, const uint8_t *data, size_t len,
                       bool wholeMessage);
