#pragma once

#include "../core/GnssEpoch.h"
#include "../core/Scheduler.h"
#include "../platform/Clock.h"
#include "Config.h"
#include "Console.h"
#include "Gnss.h"

namespace gnimu {

class App {
 public:
  App();
  void begin();
  void tick();

 private:
  static void gnssJob(void* self);
  static void consoleJob(void* self);
  void clearStats();
  void printBuild();

  static const size_t kJobs = 2;
  Job jobs_[kJobs];
  Scheduler<kJobs, PlatformClock> scheduler_;
  Gnss gnss_;
  Console console_;
  EpochTracker epochs_;
  GnssEpoch latest_;
  bool haveEpoch_ = false;
  uint32_t loops_ = 0;
};

}  // namespace gnimu
