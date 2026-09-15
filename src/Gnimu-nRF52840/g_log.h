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
// each macro and its arguments compile away. At runtime every macro first
// checks Serial, so an unattached console skips formatting as well as writing.
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
// ##__VA_ARGS__ drops the comma when fmt is the only argument.
#define LOG_PRINTF(fmt, ...)                                                   \
  do {                                                                         \
    if (Serial) {                                                              \
      char logLine_[LOG_LINE_MAX];                                             \
      const int logLen_ =                                                      \
          snprintf(logLine_, sizeof(logLine_), fmt, ##__VA_ARGS__);            \
      if (logLen_ > 0)                                                         \
        Serial.write((const uint8_t *)logLine_,                                \
                     logLen_ < (int)sizeof(logLine_) ? (size_t)logLen_         \
                                                     : sizeof(logLine_) - 1);  \
    }                                                                          \
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
