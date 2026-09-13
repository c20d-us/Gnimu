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

// ============================================================================
// Status LED module - drives the board's status LED from the observable system
// state (connection, and on battery boards the cell and power state).
//
// The interface is the same on every board; what the LED shows is not, because
// the hardware is not - a single onboard LED on the ESP32, an RGB LED on the
// XIAO. Each board's g_led.cpp documents its own priority table.
// ============================================================================

// Configure the LED pin(s) and turn the LED off. Call once in setup().
void ledBegin();

// Reflect the current state, with blink timing. Safe to call every loop().
void ledUpdate();
