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
