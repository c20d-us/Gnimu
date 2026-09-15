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
#include <Arduino.h>

// Telemetry: on each GNSS epoch, builds a TelemetrySample (g_protocol.h),
// passes it to the active encoder, sends the frames over BLE, and prints serial
// stats. Knows nothing about the wire format.

// Call once in setup() after the other modules are up.
void telemetryBegin();

// Consume a new GNSS epoch if one arrived; encode and send it when a client is
// connected. Prints stats every LOG_STATS_INTERVAL_MS. Call every loop().
void telemetrySendIfReady();

// GNSS epoch rate (Hz) over the last stats window, measured epoch to epoch.
// 0.0f before the first window closes or if no epoch arrived.
float telemetryGnssRateHz();

// BLE send rate (Hz) over the same span as telemetryGnssRateHz(). The two differ
// only when frames were dropped.
float telemetryBleRateHz();
