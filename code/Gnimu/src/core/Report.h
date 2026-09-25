#pragma once

#include <stdarg.h>
#include <stdio.h>

#include "GnssEpoch.h"
#include "Scheduler.h"

namespace gnimu {

// Text of the on-request diagnostics report, formatted into a caller buffer so
// it can be tested on a host. Output is truncated, never overrun.
struct ReportView {
  const Job* jobs;
  const JobStats* stats;
  size_t jobCount;
  uint32_t loops;
  const EpochTracker* epochs;
  const GnssEpoch* latest;  // nullptr until the first epoch
};

namespace detail {
struct Out {
  char* buf;
  size_t cap;
  size_t len;
  void put(const char* fmt, ...) __attribute__((format(printf, 2, 3))) {
    if (len + 1 >= cap) return;
    va_list ap;
    va_start(ap, fmt);
    const int n = vsnprintf(buf + len, cap - len, fmt, ap);
    va_end(ap);
    if (n > 0) len += (static_cast<size_t>(n) < cap - len) ? static_cast<size_t>(n) : cap - len - 1;
  }
};
}  // namespace detail

inline size_t formatReport(char* buf, size_t cap, const ReportView& v) {
  detail::Out o{buf, cap, 0};
  if (cap) buf[0] = '\0';
  o.put("job runs skipped overruns max_us mean_us\n");
  for (size_t i = 0; i < v.jobCount; i++) {
    const JobStats& s = v.stats[i];
    o.put("%s %lu %lu %lu %lu %lu\n", v.jobs[i].name, static_cast<unsigned long>(s.runs),
          static_cast<unsigned long>(s.skipped), static_cast<unsigned long>(s.overruns),
          static_cast<unsigned long>(s.maxUs),
          static_cast<unsigned long>(s.runs ? s.totalUs / s.runs : 0));
  }
  o.put("loops %lu\n", static_cast<unsigned long>(v.loops));
  o.put("epochs %lu missed %lu max_gap_us %lu time_steps %lu\n",
        static_cast<unsigned long>(v.epochs->epochs()),
        static_cast<unsigned long>(v.epochs->missed()),
        static_cast<unsigned long>(v.epochs->maxGapUs()),
        static_cast<unsigned long>(v.epochs->timeSteps()));
  if (v.latest) {
    o.put("fix %u sv %u hacc_mm %lu pdop %u.%02u\n", static_cast<unsigned>(v.latest->fixType),
          static_cast<unsigned>(v.latest->numSv), static_cast<unsigned long>(v.latest->hAccMm),
          static_cast<unsigned>(v.latest->pDop / 100), static_cast<unsigned>(v.latest->pDop % 100));
  } else {
    o.put("fix none\n");
  }
  return o.len;
}

}  // namespace gnimu
