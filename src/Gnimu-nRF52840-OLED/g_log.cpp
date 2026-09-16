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

#include "g_log.h"
#include <stdarg.h>

#if LOG_ENABLED

void logPrintf(const char *fmt, ...) {
  if (!Serial) {
    return;
  }
  char line[LOG_LINE_MAX];
  va_list args;
  va_start(args, fmt);
  const int len = vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);
  if (len > 0) {
    Serial.write((const uint8_t *)line,
                 len < (int)sizeof(line) ? (size_t)len : sizeof(line) - 1);
  }
}

#endif // LOG_ENABLED
