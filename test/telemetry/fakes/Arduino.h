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

// The IMU harness's Arduino stand-in (millis, pin calls, the board pin names
// the real config.h files reference), plus the one thing the telemetry module
// needs that the IMU code does not: Serial. This harness runs the REAL g_log.h
// rather than a stderr stand-in, because the bounded LOG_PRINTF and the stats
// line it carries are part of what is under test.
#include "../../imu/fakes/Arduino.h"

// The nRF core's UART ring size. The nRF g_gnss.h static_asserts on it (its
// hard real-time note); nothing here depends on the value beyond that.
#define SERIAL_BUFFER_SIZE 64

// Only what g_log.h calls. Output is captured by telemetry_harness.cpp.
struct FakeSerial {
  bool enabled = false; // Serial's bool conversion: "is anything attached"
  explicit operator bool() const { return enabled; }
  size_t print(const char *s);
  size_t println(const char *s);
  size_t write(const uint8_t *data, size_t len);
  void flush() {}
};
extern FakeSerial Serial;
