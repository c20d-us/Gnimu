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
// GNSS module - u-blox receiver on Serial1 (XIAO D6=TX / D7=RX)
//
// Owns the receiver object and its serial port internally.
// Callers interact only through the small read-only interface below.
// ============================================================================

// Bring up the receiver and configure the GNSS.
// Call once in setup(). Halts if the module is not detected.
void gnssBegin();

// Release Serial1 so its pins (D6/D7) revert to plain GPIO and can be idled
// low by powerHoldPeripheralsOff(). MUST be called before the state machine
// enters BATTERY_WAIT or DEEP_SLEEP from RUNNING, otherwise the UART
// peripheral keeps owning D6 and idles it HIGH, phantom-powering the GNSS
// through its RX ESD diode even with the TPS rail cut.
void gnssEnd();

// Pumps the GNSS UART and triggers a callback if a new epoch has arrived.
// Sets the appropriate navigation frequency based on the current state.
// Call every loop().
void gnssPoll();

// Fetches a pointer to the most recent PVT data.
// Returns a valid pointer to the PVT if a new epoch has arrived since the last
// call, otherwise returns nullptr.
const UBX_NAV_PVT_data_t *gnssConsumePvt();

// True if the receiver currently reports a valid vehicle heading.
bool gnssHeadingValid();

// Put the receiver into RXM-PMREQ backup mode (LIGHT_SLEEP entry): rail
// stays on, ~uA draw, BBR/ephemeris retained.
void gnssSleep();

// Wake the receiver from gnssSleep() via a manual GPIO pulse on the shared
// UART line. The receiver resumes its prior config (AutoPVT etc.) on its
// own. Only valid if the receiver is still in backup mode (i.e. not after
// a full EN-cut - that needs powerGnssRailOn() + gnssBegin() instead).
void gnssWake();
