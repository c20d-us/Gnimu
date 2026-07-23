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
#include <SparkFun_u-blox_GNSS_v3.h> // for UBX_NAV_PVT_data_t

// ============================================================================
// GNSS module - u-blox receiver on Serial2
//
// Owns the receiver object and its serial port internally.
// Callers interact only through the small read-only interface below.
// ============================================================================

// Bring up the receiver and configure the GNSS.
// Call once in setup().
void gnssBegin();

// Pump the GNSS UART and trigger a callback if a new epoch has arrived.
// Call every loop().
void gnssPoll();

// Fetches a pointer to the most recent PVT data.
// Returns a valid pointer to the PVT if a new epoch has arrived since the last
// call, otherwise returns nullptr.
const UBX_NAV_PVT_data_t *gnssConsumePvt();

// True if the receiver currently reports a valid vehicle heading.
bool gnssHeadingValid();
