#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <initializer_list>

#include "../code/Gnimu/src/core/Report.h"

using namespace gnimu;

static void noop(void*) {}

static void testReport() {
  const Job jobs[] = {{"gnss", 1000, 1000, noop, nullptr}};
  JobStats stats[1];
  stats[0].runs = 4;
  stats[0].skipped = 1;
  stats[0].overruns = 2;
  stats[0].maxUs = 900;
  stats[0].totalUs = 2000;
  EpochTracker epochs(50);
  GnssEpoch latest;
  latest.fixType = 3;
  latest.numSv = 17;
  latest.hAccMm = 210;
  latest.pDop = 105;

  char buf[512];
  ReportView v{jobs, stats, 1, 12345, &epochs, &latest};
  const size_t n = formatReport(buf, sizeof(buf), v);
  assert(n == strlen(buf));
  assert(strstr(buf, "gnss 4 1 2 900 500\n"));
  assert(strstr(buf, "loops 12345\n"));
  assert(strstr(buf, "epochs 0 missed 0 max_gap_us 0 time_steps 0\n"));
  assert(strstr(buf, "fix 3 sv 17 hacc_mm 210 pdop 1.05\n"));

  v.latest = nullptr;
  formatReport(buf, sizeof(buf), v);
  assert(strstr(buf, "fix none\n"));
}

static void testTruncationNeverOverruns() {
  const Job jobs[] = {{"gnss", 1000, 1000, noop, nullptr}};
  JobStats stats[1];
  EpochTracker epochs(50);
  ReportView v{jobs, stats, 1, 0, &epochs, nullptr};
  for (size_t cap : {size_t(0), size_t(1), size_t(8), size_t(40)}) {
    char buf[64];
    memset(buf, 'x', sizeof(buf));
    const size_t n = formatReport(buf, cap, v);
    assert(n < (cap ? cap : 1));
    if (cap) assert(buf[n] == '\0');
    for (size_t i = cap; i < sizeof(buf); i++) assert(buf[i] == 'x');
  }
}

int main() {
  testReport();
  testTruncationNeverOverruns();
  puts("report: all tests passed");
  return 0;
}
