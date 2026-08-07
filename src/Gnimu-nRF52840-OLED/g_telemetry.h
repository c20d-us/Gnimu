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
#include <Arduino.h>

// ============================================================================
// Telemetry module
//
// Packs the latest GNSS + IMU data into a RaceBox packet, hands the packet off
// to the BLE module for sending, and prints periodic serial stats. Consumes the
// imu / gnss / ble / ubx_helpers module interfaces; owns no hardware itself.
// ============================================================================

// Call once in setup() after the other modules are up.
void telemetryBegin();

// When a new GNSS epoch is available, retrieve it and count it.
// When a BLE client is connected, build an 88-byte RaceBox Data Message and
// hand it over to the BLE module for sending.
// Always print Serial stats on the frequency defined in config.h.
// Call every loop() iteration.
void telemetrySendIfReady();

// Observed GNSS epoch rate in Hz, averaged over the most recently completed
// LOG_STATS_INTERVAL_MS window. Returns 0.0f until the first window closes.
//
// This is NOT compiled out in silent builds. The rate used to be derived
// inline inside the LOG_ENABLED-guarded serial report, which meant it simply
// did not exist when GNIMU_SERIAL_LOG_ENABLED was 0 - the configuration
// intended for shipping. Any non-logging consumer (a display, a health check)
// would have silently read nothing there.
float telemetryGnssRateHz();

// Observed BLE packet send rate in Hz, same window and the same rationale.
float telemetryBleRateHz();
