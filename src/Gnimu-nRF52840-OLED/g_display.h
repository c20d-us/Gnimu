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
// Display module - drives the SSD1306 OLED that replaces this variant's RGB
// status LED. Like g_led it owns no state of its own: it observes
// stateCurrent() (g_state), batteryGetStatus(), bleIsConnected(),
// gnssLatestPvt() and telemetryGnssRateHz(), and renders what it finds.
//
// A persistent STATUS BAR sits over a per-state BODY, so each screen shows only
// what that state can actually know:
//
//   RUNNING       "Connected"/"Advertising" + battery | SV count and fix type
//                 large, with pDOP, PVT rate, hAcc and runtime below
//   CHARGE_ONLY   "Charging"/"Full" + battery         | cell voltage only - the
//                 GNSS is held off in this state, so there is no fix data and
//                 showing stale numbers would be actively misleading
//   LIGHT_SLEEP   "Advertising" + battery             | sparse idle notice
//   BATTERY_WAIT  (no status bar - the cell is switched out of circuit, so its
//                 charge state is meaningless)        | full-screen alert
//   DEEP_SLEEP    display off
//
// UPDATE COST is the constraint that shapes this module, and it is a matter of
// DENSITY rather than volume. A full 1024-byte frame costs ~31 ms at 400 kHz -
// affordable in total, spread across a second. What is not affordable is
// pushing it back to back: the GNSS UART's RX buffer fills in ~5.5 ms, and a
// burst of consecutive transfers never leaves it a clear stretch in which to be
// drained, so NAV-PVT bytes are lost and the 25 Hz rate sags.
//
// So rendering (into a RAM buffer, free) is separated from pushing (over I2C,
// expensive), and the pushes are SPACED: each frame goes out as slices of
// DISPLAY_CHUNK_TILES_W tiles, at most one every DISPLAY_SLICE_INTERVAL_MS.
// Bench-validated 2026-08-05 - smaller slices alone were not enough.
// ============================================================================

// Bring up the panel and paint the first frame. Call once in setup(), after
// the modules it observes are up. Safe to call when no panel is attached: it
// logs and disables itself rather than halting, since a missing display should
// not stop the device streaming telemetry.
void displayBegin();

// Reflect the current state on the panel, advancing the render/push state
// machine by one step. Safe to call every loop() - named to match ledUpdate(),
// its counterpart: both are pure observers that recompute an output from state
// rather than pumping a data source the way the *Poll() modules do.
void displayUpdate();

// True if a panel answered at displayBegin(). False means the module has
// disabled itself and nothing is being shown - which is why g_led uses this to
// decide whether to act as a fallback indicator. See LED_ENABLED in config.h.
bool displayIsPresent();

// Blank the panel (SSD1306 DISPLAYOFF, ~10 uA) without cutting power. Call
// before entering a state where the MCU stops servicing the display -
// specifically before powerEnterDeepSleep(), which never returns.
void displaySleep();

// Undo displaySleep() and force a full repaint on the next poll.
void displayWake();
