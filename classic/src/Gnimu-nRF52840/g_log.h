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
#include "config.h"
#include <Arduino.h>

// ============================================================================
// Logging shim. One-for-one macro replacements for Serial.print/println/
// printf/flush, gated by the single LOG_ENABLED flag (config.h).
//
// Preprocessor-level, not function calls: with logging disabled, each macro
// expands to nothing at all - the call AND its arguments vanish before the
// compiler ever sees them.
//
// RUNTIME guard, layered on top of the compile-time one: every macro also
// checks `Serial` (its bool conversion - the same check setup()'s boot wait
// already uses) before doing anything, so a LOG_ENABLED=1 build that happens
// to be running with nothing attached pays no cost either, not just a build
// with logging compiled out entirely.
//
// This matters specifically for LOG_PRINTF. On the nRF52 boards (native USB
// CDC via TinyUSB), Serial's own write() already no-ops instantly when
// nothing is connected - confirmed in Adafruit_USBD_CDC.cpp, its transmit
// loop is itself gated on tud_cdc_n_connected(). But that only covers the
// WRITE. The printf-style FORMATTING (vsnprintf building the string, real
// cost for something like the once-a-second stats line with several %f
// fields) happens before write() is ever reached, unconditionally - exactly
// the class of cost LOG_ENABLED exists to strip at compile time.
// Checking `Serial` here skips the formatting too, not just the write.
//
// LOG_FLUSH() needed this even more directly: unlike write(), the library's
// flush() has no internal connected-check of its own (only confirms the CDC
// interface itself is valid) - so this guard is closing a real gap, not just
// adding a redundant one.
//
// On ESP32, Serial's bool conversion just reflects whether the UART driver
// has been installed (true from Serial.begin() onward, forever) - so this
// guard always takes the "connected" branch there and is a no-op, identical
// to today's behavior. Safe to share verbatim across all three variants: real
// benefit on the nRF52 boards, neutral elsewhere.
// ============================================================================

#if LOG_ENABLED

#define LOG_PRINT(...)                                                         \
  do {                                                                         \
    if (Serial)                                                                \
      Serial.print(__VA_ARGS__);                                               \
  } while (0)
#define LOG_PRINTLN(...)                                                       \
  do {                                                                         \
    if (Serial)                                                                \
      Serial.println(__VA_ARGS__);                                             \
  } while (0)
// ##__VA_ARGS__ swallows the preceding comma when fmt is the only argument.
#define LOG_PRINTF(fmt, ...)                                                   \
  do {                                                                         \
    if (Serial)                                                                \
      Serial.printf(fmt, ##__VA_ARGS__);                                       \
  } while (0)
#define LOG_FLUSH()                                                            \
  do {                                                                         \
    if (Serial)                                                                \
      Serial.flush();                                                          \
  } while (0)

#else

#define LOG_PRINT(...)
#define LOG_PRINTLN(...)
#define LOG_PRINTF(...)
#define LOG_FLUSH()

#endif // LOG_ENABLED
