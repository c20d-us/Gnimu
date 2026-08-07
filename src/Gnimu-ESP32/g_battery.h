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
// Battery module (stub) - this build has no battery hardware; the USB rail
// powers everything. The stub provides the same interface surface that the
// shared telemetry module consumes on the nRF52840 variant, reporting a
// constant full charge, so g_telemetry.cpp stays byte-identical across
// variants. See BATTERY_HAS_GAUGE in config.h.
// ============================================================================

// Mirrors the nRF52840 variant's BatteryStatus so the shared telemetry code
// sees one interface. Only `percent` carries real meaning on this build.
struct BatteryStatus {
  float voltage;   // always 0.0f - no VBAT sense on this build
  uint8_t percent; // always BATTERY_REPORT_PERCENT
  bool charging;   // always false
  bool warn;       // always false
  bool critical;   // always false
  bool full;       // always false
};

// The (constant) battery snapshot.
BatteryStatus batteryGetStatus();

// The RaceBox protocol battery byte for payload offset 67:
// bit 7 = charging (never set on this build), bits 0-6 = percent.
uint8_t batteryProtocolByte();
