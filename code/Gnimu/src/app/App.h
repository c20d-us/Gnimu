#pragma once

#include "../core/Scheduler.h"
#include "../platform/Clock.h"

namespace gnimu {

class App {
 public:
  App();
  void begin();
  void tick();

 private:
  static void serialCommands(void* self);
  void printStats();

  static const size_t kJobs = 1;
  Job jobs_[kJobs];
  Scheduler<kJobs, PlatformClock> scheduler_;
};

}  // namespace gnimu
