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

// Display: the SSD1306 OLED that replaces the RGB LED on this board. Renders
// state from g_state, g_battery, g_ble, g_gnss, g_telemetry, and g_imu_trim;
// owns no state of its own.
//
// A status bar sits above a per-state body:
//
//   RUNNING       bar: connection, trim, battery
//                 body: SV count and fix type, pDOP, PVT rate, hAcc, runtime
//   CHARGE_ONLY   bar: charging/full, battery
//                 body: cell voltage (GNSS is off)
//   BATTERY_WAIT  no bar; full-screen alert
//   DEEP_SLEEP    display off
//
// A full frame push is ~31ms of I2C at 400kHz, longer than the GNSS UART can
// wait to receive a portion of a PVT (~5.5ms). Frames render to RAM, then go
// out in DISPLAY_CHUNK_TILES_W-tile slices at most every
// DISPLAY_SLICE_INTERVAL_MS.

// Bring up the panel and draw the first frame. Call once in setup(). With no
// panel it logs and disables itself.
void displayBegin();

// Advance the render/push state machine one step. Call every loop().
void displayUpdate();

// True if a panel answered at displayBegin(). g_led falls back to the LED when
// false.
bool displayIsPresent();

// Turn the panel off (SSD1306 DISPLAYOFF). Called before System OFF, which
// doesn't cut the panel's rail. There is no wake; the next boot reinitializes
// it.
void displaySleep();
