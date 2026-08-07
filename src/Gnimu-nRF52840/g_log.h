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
// ============================================================================

#if LOG_ENABLED

#define LOG_PRINT(...) Serial.print(__VA_ARGS__)
#define LOG_PRINTLN(...) Serial.println(__VA_ARGS__)
// ##__VA_ARGS__ swallows the preceding comma when fmt is the only argument.
#define LOG_PRINTF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#define LOG_FLUSH() Serial.flush()

#else

#define LOG_PRINT(...)
#define LOG_PRINTLN(...)
#define LOG_PRINTF(...)
#define LOG_FLUSH()

#endif // LOG_ENABLED
