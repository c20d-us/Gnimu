#pragma once

#include <stdio.h>

namespace gnimu {

struct BuildInfo {
  const char* version;
  const char* revision;
  const char* dirty;
  const char* buildUtc;
  const char* target;
};

// One machine-readable line identifying what was flashed.
inline size_t formatBuildLine(char* buf, size_t cap, const BuildInfo& b) {
  const int n = snprintf(buf, cap, "BUILD,version=%s,revision=%s,dirty=%s,utc=%s,target=%s",
                         b.version, b.revision, b.dirty, b.buildUtc, b.target);
  if (n < 0 || cap == 0) return 0;
  return static_cast<size_t>(n) < cap ? static_cast<size_t>(n) : cap - 1;
}

}  // namespace gnimu
