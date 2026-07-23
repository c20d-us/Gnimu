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
// Gnimu state machine
//
// Owns the SystemState enum + the transition table. Consumes:
//   - powerUsbPresent()                (g_power)
//   - powerSwitchOn()                  (g_power)
//   - batteryCutoffRequested()         (g_battery)
//   - bleIsConnected()                 (g_ble)
//   - imuWakeTriggered()               (g_imu)
// Actuates:
//   - gnssEnd()/gnssSleep()/gnssWake() (g_gnss)
//   - imuArmWake()/imuDisarmWake()     (g_imu)
//   - powerHoldPeripheralsOff()        (g_power)
//   - powerGnssRailOn()/Off()          (g_power)
//   - powerEnterDeepSleep()            (g_power)
//   - NVIC_SystemReset()
// ============================================================================

// The State Machine Definition
enum SystemState {
  STATE_RUNNING,      // Normal operation
  STATE_CHARGE_ONLY,  // USB in + switch on + STATE_CHARGE_ONLY_ON_USB=1:
                      // peripherals held off so the charge IC gets max current
                      // to the cell. Exit is unplug or switch off.
  STATE_BATTERY_WAIT, // USB in but switch off; waits for switch on or unplug
  STATE_DEEP_SLEEP,   // Entry action only; MCU halts in System OFF
  STATE_LIGHT_SLEEP,  // GNSS backup + IMU wake-detect armed +
                      // LED blinks blue slowly. Reversible without a reset.
};

// Classify + record the initial state (RUNNING / BATTERY_WAIT / DEEP_SLEEP).
// If the classification lands in DEEP_SLEEP, calls powerEnterDeepSleep()
// directly and does NOT return. Otherwise records the state and returns it,
// so setup() can decide whether to bring up GNSS/IMU/BLE (RUNNING only).
//
// Call after powerBegin() + batteryBegin() + ledBegin() have run. Reads
// powerSwitchOn(), powerUsbPresent(), and batteryGetStatus().voltage.
SystemState stateBegin();

// Advance the state machine. Call every loop().
void stateUpdate();

// The current live state (for g_led + diagnostics to observe).
SystemState stateCurrent();
