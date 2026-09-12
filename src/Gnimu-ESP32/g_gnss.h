// Gnimu - RaceBox Mini-compatible GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// Based on the Open-Source RaceBox Mini Emulator by Anchit Chandra Sekhar
// (https://github.com/anchit92/Open-Source-RaceBox-mini-Emulator)
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
#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>

// ============================================================================
// GNSS module - the u-blox receiver, identical on every board.
//
// Owns the receiver object, its configuration and the epoch cache. It does NOT
// own the serial port: that is g_gnss_port.h, with one implementation per core
// (g_gnss_port_esp32.cpp / g_gnss_port_nrf52.cpp). Callers interact only
// through the small read-only interface below.
//
// THE UART DRAIN CONSTRAINT lives with the port, because it differs in KIND
// between the cores - a phase-sensitive deadline on the nRF's 63-byte ring, a
// duration budget on the ESP32's 512-byte one. Read the port file for your
// board before adding work to loop() or to a BLE/display callback.
//
// What is common to both, and the reason either matters: RING OVERFLOW IS
// SILENT. The byte is dropped with no flag and no counter; it surfaces one step
// removed as a checksum failure costing a whole epoch, and one step further as
// a GNSS rate below GNSS_NAV_RATE_HZ on the 1Hz stats line. A sagging rate with
// no other explanation is the signature. g_protocol.h's onWrite contract states
// the same constraint for inbound BLE writes.
// ============================================================================

// Bring up the receiver and configure the GNSS.
//
// RETURNS false if the receiver never answered the baud sweep. It does NOT
// halt: a halt in setup() stops batteryPoll() and stateUpdate() from ever
// running, so on a battery build the low-voltage cutoff could never fire and
// the cell would discharge to damage. The device continues without telemetry
// instead, saying so once per second, and a power cycle is the recovery.
//
// There is deliberately no automatic retry - see the implementation.
bool gnssBegin();

// Whether the receiver is currently configured and usable.
//
// False before the first successful gnssBegin(), after a failed one, and after
// gnssEnd() releases the UART.
//
// Callers use this to avoid reporting meaningless data - with no receiver every
// GNSS field is a sentinel or zero, which reads like a device searching for a
// fix rather than one that has none.
bool gnssIsUp();

// Stop talking to the receiver and release the UART, leaving gnssIsUp() false.
//
// What that buys depends on the board, and is documented in its port file: on
// the nRF it is what lets powerHoldPeripheralsOff() idle the TX pin low, so it
// MUST be called before the state machine enters BATTERY_WAIT or DEEP_SLEEP
// from RUNNING - a UART that still owns the pin idles it HIGH and
// phantom-powers the receiver through its RX ESD diode even with the rail cut.
// The ESP32 build has no rail to cut and never calls it today; it exists on
// every board so this interface does not fork.
void gnssEnd();

// Pumps the GNSS UART and triggers a callback if a new epoch has arrived.
//
// That is ALL it does: checkUblox() to parse incoming bytes, checkCallbacks()
// to fire the PVT handler, and an early return when gnssIsUp() is false.
//
// It does NOT touch the navigation rate, whatever an earlier version of this
// comment said. setNavigationFrequency() is called exactly once, in
// gnssBegin(), at the fixed GNSS_NAV_RATE_HZ; nothing varies it at runtime -
// not the system state, not the BLE connection. Said explicitly rather than
// just deleted, because two separate versions of this header have now claimed
// a dynamic rate that has never existed.
//
// Call every loop(). See the drain-deadline note above for how often "every
// loop()" actually has to be.
void gnssPoll();

// Fetches a pointer to the most recent PVT data.
// Returns a valid pointer to the PVT if a new epoch has arrived since the last
// call, otherwise returns nullptr.
const UBX_NAV_PVT_data_t *gnssConsumePvt();

// Peek at the most recent PVT WITHOUT consuming it. Returns nullptr until the
// first epoch has ever arrived; afterwards it always returns the last one,
// however stale.
//
// gnssConsumePvt() is consume-once by design - it clears the new-epoch flag on
// the way out, so exactly one caller can ever see a given epoch, and that
// caller is g_telemetry. Any second consumer calling it would race: whichever
// ran first in a loop iteration takes the epoch and the other sees nullptr,
// producing dropped BLE packets or a half-rate reader depending on call order.
// Read-only observers (a display, a health check) must use this instead.
//
// Staleness is the caller's problem: this says nothing about how old the data
// is, or whether the receiver is still answering - if it stops, the last epoch
// simply stops advancing.
const UBX_NAV_PVT_data_t *gnssLatestPvt();
