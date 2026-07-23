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
// Status LED module - drives the XIAO's onboard RGB LED to reflect system
// state. Consumes stateCurrent() (gp_state) plus the observable battery and
// connection state (batteryGetStatus() / bleIsConnected()).
//
// Priority, highest first:
//   1. BATTERY_WAIT    -> rapid red blink ("check the switch")
//   2. RUNNING/LIGHT_SLEEP:
//      a. charging     -> green blink (or steady green if full)
//      b. critical bat -> red blink
//      c. warn bat     -> amber blink
//      d. LIGHT_SLEEP  -> breathing blue (non-linear, exponential ramp)
//      e. connected    -> steady blue
//      f. advertising  -> blue blink
// ============================================================================

// Configure the RGB LED pins and turn the LED off. Call once in setup().
void ledBegin();

// Reflect the current state, with blink timing. Safe to call every loop().
void ledUpdate();
