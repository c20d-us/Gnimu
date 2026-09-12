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
// Owns CADENCE, not wire format. On each new GNSS epoch it assembles the
// canonical TelemetrySample (see g_protocol.h), hands it to the active
// protocol encoder, forwards the emitted frames to the BLE module, and prints
// periodic serial stats. Consumes the imu / gnss / ble / battery / protocol
// module interfaces; owns no hardware itself.
//
// The packet layout itself lives in g_proto_racebox - nothing here knows what
// a RaceBox packet looks like.
// ============================================================================

// Call once in setup() after the other modules are up.
void telemetryBegin();

// When a new GNSS epoch is available, retrieve it and count it.
// When a BLE client is connected, build a TelemetrySample from it and pass it
// to the active protocol encoder, whose emitted frames go to the BLE module.
// Always print Serial stats on the frequency defined in config.h.
// Call every loop() iteration.
void telemetrySendIfReady();

// Observed GNSS epoch rate in Hz for the most recently completed stats window.
// Windows close every LOG_STATS_INTERVAL_MS on the clock, but the rate is
// measured epoch to epoch - from the last epoch of the previous window to the
// last of this one - so it does not alias against the receiver's clock: a
// steady stream reads its true rate (to within a tenth), and a lost epoch
// lowers exactly one window's reading. Returns 0.0f until the first window
// closes, and for any window in which no epoch arrived.
float telemetryGnssRateHz();

// Observed BLE packet send rate in Hz: counted on the same epochs over the
// same span as telemetryGnssRateHz(), so the two differ only when frames were
// actually dropped in that window.
float telemetryBleRateHz();
