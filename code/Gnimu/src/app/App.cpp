#include "App.h"

#include <Arduino.h>

#include "BuildInfo.h"

namespace gnimu {

App::App()
    : jobs_{{"gnss", config::kGnssDrainPeriodUs, config::kGnssDrainBudgetUs, &App::gnssJob, this},
            {"console", config::kConsolePeriodUs, config::kConsoleBudgetUs, &App::consoleJob,
             this}},
      scheduler_(jobs_),
      epochs_(1000 / config::kGnssNavRateHz) {}

void App::begin() {
  console_.begin();
  printBuild();
  console_.log("Gnimu starting");

  PlatformClock::begin();
  console_.log("Clock started");

  if (!gnss_.begin(console_)) {
    console_.log("GNSS not detected at any standard baud rate. Check wiring.");
    for (;;) delay(100);
  }

  for (size_t i = 0; i < kJobs; i++)
    console_.log("Job %s: every %lu us, budget %lu us", jobs_[i].name,
                 static_cast<unsigned long>(jobs_[i].periodUs),
                 static_cast<unsigned long>(jobs_[i].budgetUs));
  scheduler_.start();
  console_.log("Running. D = report, C = clear, V = version.");
}

void App::tick() {
  loops_++;
  PlatformClock::sleepUntilUs(scheduler_.tick());
}

void App::gnssJob(void* self) {
  App* app = static_cast<App*>(self);
  app->gnss_.poll();
  GnssEpoch epoch;
  if (app->gnss_.consumeEpoch(epoch)) {
    app->epochs_.feed(epoch);
    app->latest_ = epoch;
    app->haveEpoch_ = true;
  }
}

void App::consoleJob(void* self) {
  App* app = static_cast<App*>(self);
  const char cmd = app->console_.poll();
  if (cmd == 'D') {
    app->console_.printReport({app->scheduler_.jobs(), app->scheduler_.allStats(), kJobs,
                               app->loops_, &app->epochs_,
                               app->haveEpoch_ ? &app->latest_ : nullptr});
  } else if (cmd == 'C') {
    app->clearStats();
  } else if (cmd == 'V') {
    app->printBuild();
  }
}

void App::printBuild() {
  char line[192];
  formatBuildLine(line, sizeof(line), buildInfo());
  console_.log("%s", line);
}

void App::clearStats() {
  scheduler_.clearStats();
  epochs_.clear();
  loops_ = 0;
}

}  // namespace gnimu
