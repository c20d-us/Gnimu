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
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // for UBX_NAV_PVT_data_t

// ============================================================================
// GNSS module - u-blox receiver on Serial2
//
// Owns the receiver object and its serial port internally (see gc_gnss.cpp).
// Callers interact only through the small read-only interface below; the live
// receiver is never exposed, so nothing outside this module can reconfigure it.
// ============================================================================

// Bring up the receiver: open serial, detect the module (auto-recovering its
// baud rate if needed), and configure PVT output, dynamic model, navigation
// rate, and enabled constellations. Call once in setup().
void gnssBegin();

// Poll the GNSS link - feeds the parser from the UART. Call every loop().
void gnssPoll();

// True exactly once per new navigation epoch. Driven by an internal callback
// that verifies complete packet reception. Safe to call every loop().
bool gnssHasNewEpoch();

// Read-only view of the most recent PVT solution from the callback cache.
const UBX_NAV_PVT_data_t *gnssLatestPvt();

// True if the receiver currently reports a valid vehicle heading.
bool gnssHeadingValid();
