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
// Battery module - VBAT voltage sense, state-of-charge fuel gauge, and the
// voltage-only side of the low-voltage cutoff (batteryCutoffRequested()).
// Pure measurement, contains no policy. The state machine (gp_state) decides
// when to act on the cutoff request; the rail actions (System OFF etc.) live
// in gp_power.
//
// The ADC configuration (resolution + reference + TACQ) is shared with the
// switch-sense read in gp_power. It is applied by powerBegin() and must be
// called before batteryBegin().
// ============================================================================

// A snapshot of the battery state, refreshed when a sampler run completes in
// batteryPoll(). `charging` and `full` are composed from powerUsbPresent() +
// powerSwitchOn() so there is one canonical definition here for all display
// consumers (LED + telemetry).
struct BatteryStatus {
  float voltage;   // cell voltage in volts (EMA-smoothed peak)
  uint8_t percent; // state of charge, 0-100
  bool charging;   // USB present AND switch on (cell can actually charge)
  bool warn;       // voltage <= BATTERY_WARN_V (amber-blink LED)
  bool critical;   // voltage <= BATTERY_CRITICAL_V (red-blink LED)
  bool full;       // charging AND voltage >= BATTERY_FULL_V (steady green LED)
};

// Configure the VBAT-divider enable pin and the charge-current select pin
// (fast charge when BATTERY_FAST_CHARGE is defined), then prime the sampler
// with one blocking run (~50 ms) so the cached status is valid on return.
// Call in setup() AFTER powerBegin() (which configures the shared ADC).
void batteryBegin();

// Advance the non-blocking VBAT sampler and refresh the cached status when a
// run completes. Takes at most one analogRead() per call and never blocks
// longer than a single ADC conversion, so it is safe to call every loop().
// Also updates the debounce anchor read by batteryCutoffRequested().
void batteryPoll();

// The most recent battery snapshot from the last completed sampler run.
BatteryStatus batteryGetStatus();

// True when the fresh sampler peak has been below BATTERY_CUTOFF_V for at
// least BATTERY_CUTOFF_DEBOUNCE_MS. Voltage-only, no knowledge of USB.//
// gp_state applies the USB gate before entering DEEP_SLEEP.
bool batteryCutoffRequested();

// The RaceBox protocol battery byte for payload offset 67:
// bit 7 = charging, bits 0-6 = percent.
uint8_t batteryProtocolByte();
