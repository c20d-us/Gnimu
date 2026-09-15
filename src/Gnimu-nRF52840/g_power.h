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

// Power: USB and switch sensing, and the rail and pin controls g_state drives.
// No policy.

// Configure the shared ADC and the switch-sense pin. Call after Serial.begin()
// and before powerSwitchOn() or the battery sampler.
void powerBegin();

// True when VBUS is present.
bool powerUsbPresent();

// True when the slide switch is on (sense tap below
// POWER_SWITCH_OFF_THRESHOLD_MV). Refreshed with one analogRead() every
// POWER_SWITCH_POLL_INTERVAL_MS; cached in between. Reliability comes from the
// threshold margin (checked in config.h), SAADC_TACQ_US, and
// STATE_SWITCH_OFF_DEBOUNCE_MS.
bool powerSwitchOn();

// Drive every peripheral control pin to its off state. No library calls, safe
// before Serial.begin(), idempotent. Used at boot and on entering BATTERY_WAIT
// and DEEP_SLEEP.
void powerHoldPeripheralsOff();

// Release GNSS EN so the regulator's pullup turns the rail on. Call before
// gnssBegin().
void powerGnssRailOn();

// Enter System OFF. Does not return; USB plug-in or a switch off -> on resets.
void powerEnterDeepSleep();
