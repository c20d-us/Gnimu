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
// Power module - rail mechanisms and rail/switch sensing.
//
// Owns the primitives the state machine (g_state) actuates:
//   - USB/VBUS presence
//   - Slide-switch position via the A4 divider (load-independent presence)
//   - GNSS EN + TX drive (rail on/off + phantom back-feed prevention)
//   - IMU power pin drive
//   - LED pin drive when g_led isn't yet initialized (boot / halt)
//   - powerHoldPeripheralsOff() - unconditional held-off entry action for
//     boot, BATTERY_WAIT, and DEEP_SLEEP; drives peripheral controls directly
//     so it can run before any other module has begun.
//   - powerEnterDeepSleep() - final System OFF sequence, does not return.
//
// Contains no policy - the state machine decides when to call these.
// ============================================================================

// Configure the ADC (resolution + reference + TACQ) shared by battery voltage
// and switch-sense reads, and the switch-sense pin as INPUT. Call after
// Serial.begin() but before anything reads powerSwitchOn() or the battery
// sampler.
void powerBegin();

// True when VBUS is present.
bool powerUsbPresent();

// True when the slide switch is ON (battery is physically in the circuit).
// Reads the A4 divider tap; below POWER_SWITCH_OFF_THRESHOLD_MV = ON.
// Internally throttled + denoised (dummy read + averaged burst, refreshed at
// most every ~50 ms) to guard against SAADC channel-switch "ghost" readings
// from the shared VBAT pin - safe to call every loop().
bool powerSwitchOn();

// Drive every peripheral control pin to its safe held-off state without
// touching any peripheral library. Safe before Serial.begin() and before any
// other module has initialized. Idempotent. Used as: boot prologue
// (unconditional), BATTERY_WAIT entry, DEEP_SLEEP entry.
void powerHoldPeripheralsOff();

// Release the GNSS EN pin so the TPS63020's own pullup enables the rail (the
// pre-cutoff-fix "normal" state). Undoes the EN-low drive done by
// powerHoldPeripheralsOff(). Call before gnssBegin() when entering RUNNING.
// imuBegin() and Serial1.begin() reclaim their own pins.
void powerGnssRailOn();

// Cut the GNSS rail (EN low) and idle TX low, WITHOUT touching IMU/LED -
// unlike powerHoldPeripheralsOff(), which is a blanket held-off state for
// BATTERY_WAIT/DEEP_SLEEP. Used for LIGHT_SLEEP's GNSS-only backup->EN-cut
// escalation (STATE_LIGHT_SLEEP_GNSS_CUTOFF_MIN), where IMU wake-detect and
// BLE must stay alive. Caller must gnssEnd() first (releases Serial1's
// ownership of TX) - same UART-vs-GPIO ordering rule as
// powerHoldPeripheralsOff().
void powerGnssRailOff();

// Enter System OFF (single-digit uA deep sleep): print, hold peripherals off,
// then sd_power_system_off(). Does not return; recovery is a hard reset
// triggered by USB plug-in or a slide-switch off->on cycle.
void powerEnterDeepSleep();
