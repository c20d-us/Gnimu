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

// ============================================================================
// Telemetry module - application glue
//
// Turns the latest GNSS + IMU data into RaceBox packets, sends them over BLE,
// and prints periodic serial stats. Consumes the imu / gnss / ble / ubx_helpers
// module interfaces; owns no hardware itself.
// ============================================================================

// Start the stats-report timer.
// Call once in setup(), after the other modules are up.
void telemetryBegin();

// On each new GNSS epoch, count it and - when a client is connected - build and
// send an 88-byte RaceBox Data Message over BLE.
void telemetrySendPacketIfReady();

// Periodically print packet-rate and GNSS/IMU debug stats over serial.
void telemetryReport();
