// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "../core/ConsoleEntry.h"
#include "../core/Report.h"
#include "../core/StatusLine.h"
#include "SystemConfig.h"

namespace gnimu {

static_assert(config::kConsoleTxBufferBytes >= 2 * kReportMaxBytes,
              "the ESP32 console buffer must hold a whole report plus its per-write overhead");

// One poll() result: a command letter, a status count, or neither.
struct ConsoleCommand {
  CommandKind kind = CommandKind::None;
  bool hasCount = false;
  uint32_t count = 0;    // valid only when hasCount
};

// Serial commands in, diagnostics report out. Nothing prints unless asked.
class Console {
 public:
  void begin();
  // Reads waiting input as entries ended by Enter: 'D' report, 'C' clear,
  // 'V' version, 'L' toggles status reporting (at most kStatusMaxCount
  // lines), 'T' re-trims the IMU, 'R' restarts, 'O' enters System OFF, or a number 0..kStatusMaxCount arms exactly that many status
  // reports. Anything else is rejected whole, with a message. Each call
  // handles at most one entry and reads at most the longest valid entry's
  // bytes, so a paste can't hold up the loop; the rest waits for later calls.
  ConsoleCommand poll();
  void printReport(const JobTable& jobs, const DeviceStatus& status);
  void printStatus(const DeviceStatus& status, const StatusRates& rates);
  // Text of several '\n'-separated lines, sent a line at a time like a report.
  void print(const char* text, size_t n) { emit(text, n); }
  // Boot-time and exception messages; one line each.
  void log(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
  // Lets queued output reach the host before a reset or System OFF.
  void flush();
  // Bytes thrown away because the host stopped reading; never cleared.
  uint32_t dropped() const { return dropped_; }

 private:
  void emit(const char* text, size_t n);
  void sendLine(const char* line, size_t len);

  ConsoleEntry<config::kStatusMaxCount> entry_;
  uint32_t dropped_ = 0;
};

}  // namespace gnimu
