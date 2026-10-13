// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include "Console.h"

#include <Arduino.h>

#include "../platform/Clock.h"
#include "../platform/ConsolePort.h"
#include "SystemConfig.h"
#include <stdarg.h>
#include <stdio.h>

namespace gnimu {

void Console::begin() { consoleOpen(); }

ConsoleCommand Console::poll() {
  // The longest valid entry plus CR+LF: the most one call reads. An entry
  // longer than that is collected over several calls, and rejected whole when
  // it ends.
  constexpr size_t kMaxReadBytes = decltype(entry_)::kCapacity + 2;
  ConsoleCommand result;
  for (size_t i = 0; i < kMaxReadBytes && Serial.available(); i++) {
    const EntryResult r = entry_.feed(static_cast<char>(Serial.read()));
    switch (r.kind) {
      case EntryKind::None:
        continue;
      case EntryKind::Command:
        result.kind = r.cmd;
        break;
      case EntryKind::Count:
        result.hasCount = true;
        result.count = r.count;
        break;
      case EntryKind::TooLarge:
        log("Console: count must be 0-%lu", static_cast<unsigned long>(config::kStatusMaxCount));
        break;
      case EntryKind::Unrecognised:
        log("Console: unrecognised input '%s%s'", entry_.text(), entry_.truncated() ? "\xE2\x80\xA6" : "");
        break;
    }
    // One entry per call: whatever follows it waits for the next poll, so
    // commands sent together all run, in order.
    return result;
  }
  return result;
}

void Console::printReport(const JobTable& jobs, const DeviceStatus& status) {
  // Static rather than on the loop task's stack, which is only a few KB on
  // the nRF.
  static char buf[kReportMaxBytes];
  emit(buf, formatReport(buf, sizeof(buf), jobs, status));
}

void Console::printStatus(const DeviceStatus& status, const StatusRates& rates) {
  char buf[kStatusLineMaxBytes];
  emit(buf, formatStatus(buf, sizeof(buf), status, rates));
}

void Console::flush() {
  Serial.flush();
  // flush() hands the bytes to USB/UART; the short wait lets them actually
  // leave before a reset cuts the link.
  delay(50);
}

void Console::log(const char* fmt, ...) {
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  const int r = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  // vsnprintf returns the untruncated length; the buffer holds one less than
  // its size.
  const size_t n = r < 0 ? 0 : static_cast<size_t>(r);
  emit(buf, n < sizeof(buf) ? n : sizeof(buf) - 1);
}

// Sends text to the console a line at a time, each ending in CR+LF whatever
// the text used: '\n' separates lines, and a last line without one is sent
// the same way. The whole report is bigger than the nRF's 256-byte USB buffer,
// which is why output goes a line at a time. With no host listening (no DTR on
// native USB) output is discarded uncounted; the ESP32's UART is always
// "listening".
void Console::emit(const char* text, size_t n) {
  if (!Serial) return;
  size_t start = 0;
  for (size_t i = 0; i < n; i++) {
    if (text[i] != '\n') continue;
    sendLine(text + start, i - start);
    start = i + 1;
  }
  if (start < n) sendLine(text + start, n - start);
}

// Sends one line and its CR+LF whole, or drops both, waiting at most
// kConsoleWaitUs for room, so a host that holds the port open without reading
// can't stall the loop. Every line with its CR+LF fits the nRF's USB buffer,
// so it's always sent once the host reads.
void Console::sendLine(const char* line, size_t len) {
  const size_t wire = len + 2;
  // PlatformClock, not micros(): the nRF core's micros() only advances once
  // per 1 ms RTOS tick, too coarse to time a 2 ms wait.
  const uint64_t start = PlatformClock::nowUs();
  while (static_cast<size_t>(Serial.availableForWrite()) < wire) {
    if (PlatformClock::nowUs() - start >= config::kConsoleWaitUs) {
      dropped_ += wire;
      return;
    }
  }
  Serial.write(reinterpret_cast<const uint8_t*>(line), len);
  Serial.write(reinterpret_cast<const uint8_t*>("\r\n"), 2);
}

}  // namespace gnimu
