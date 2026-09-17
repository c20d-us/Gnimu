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
#include "config.h"
#include <Arduino.h>
#include <stdio.h>

// Logging: Serial.print/println/printf/flush replacements. With LOG_ENABLED 0
// each macro and its arguments compile away. At runtime every call first checks
// Serial, so an unattached console skips formatting as well as writing.
// LOG_PRINTF's arguments are still evaluated, because it is a function call.
//
// LOG_PRINTF formats into its own LOG_LINE_MAX buffer and clips long lines. The
// nRF core's Print::printf can transmit stack memory past 255 bytes.
//
// snprintf's format attribute checks LOG_PRINTF arguments. uint32_t is
// `long unsigned int` on both toolchains: cast to (unsigned int) for %u.

// Longest LOG_PRINTF line, including the NUL.
#define LOG_LINE_MAX 256

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
// Formats into its own buffer and clips; the attribute keeps snprintf's
// argument checking at every call site. Arguments are evaluated even with no
// console attached, so they must have no side effects.
void logPrintf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
#define LOG_PRINTF(...) logPrintf(__VA_ARGS__)
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
