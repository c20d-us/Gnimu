#pragma once

#include "../core/Report.h"

namespace gnimu {

// Serial commands in, diagnostics report out. Nothing prints unless asked.
class Console {
 public:
  void begin();
  // Returns the next command character, or 0 if none is waiting.
  // 'D' report, 'C' clear, 'V' version.
  char poll();
  void printReport(const ReportView& view);
  // Boot-time and exception messages; one line each.
  void log(const char* fmt, ...) __attribute__((format(printf, 2, 3)));
};

}  // namespace gnimu
