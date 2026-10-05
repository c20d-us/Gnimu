#pragma once

#include <stddef.h>
#include <stdint.h>

#include "../core/ConsoleEntry.h"
#include "../core/Report.h"
#include "../core/StatusLine.h"
#include "Config.h"

namespace gnimu {

// One poll() result: at most one command letter and/or one status count.
// Normally one or neither; both only if two entries end in the same pass.
struct ConsoleCommand {
  char cmd = 0;          // 'D', 'C', 'V', 'L', or 0 if none
  bool hasCount = false;
  uint32_t count = 0;    // valid only when hasCount
};

// Serial commands in, diagnostics report out. Nothing prints unless asked.
class Console {
 public:
  void begin();
  // Reads waiting input as entries ended by Enter: 'D' report, 'C' clear,
  // 'V' version, 'L' toggles continuous status reporting, or a number
  // 0..kStatusMaxCount arms exactly that many status reports. Anything else
  // is rejected with a message. Reads only as many bytes per call as the
  // longest valid entry needs and discards the rest, so a paste can't hold up
  // the loop or leave stray command letters behind.
  ConsoleCommand poll();
  void printReport(const ReportView& view);
  void printStatus(const StatusView& view);
  // Boot-time and exception messages; one line each.
  void log(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
  // Lets queued output reach the host before a reset or System OFF.
  void flush();
  // Bytes thrown away because the host stopped reading; never cleared.
  uint32_t dropped() const { return dropped_; }

 private:
  void emit(const char* p, size_t n);

  ConsoleEntry<config::kStatusMaxCount> entry_;
  uint32_t dropped_ = 0;
};

}  // namespace gnimu
