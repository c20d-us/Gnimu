#include "App.h"

#include <Arduino.h>

namespace gnimu {

App::App()
    : jobs_{{"serial", 10000, 2000, &App::serialCommands, this}},
      scheduler_(jobs_) {}

void App::begin() {
  Serial.begin(115200);
  scheduler_.start();
}

void App::tick() {
  const uint64_t next = scheduler_.tick();
  PlatformClock::sleepUntilUs(next);
}

void App::serialCommands(void* self) {
  App* app = static_cast<App*>(self);
  while (Serial.available()) {
    const int c = Serial.read();
    if (c == 'D') app->printStats();
    if (c == 'C') app->scheduler_.clearStats();
  }
}

void App::printStats() {
  Serial.println("job runs skipped overruns max_us mean_us");
  for (size_t i = 0; i < scheduler_.count(); i++) {
    const JobStats& s = scheduler_.stats(i);
    Serial.print(scheduler_.job(i).name);
    Serial.print(' ');
    Serial.print(s.runs);
    Serial.print(' ');
    Serial.print(s.skipped);
    Serial.print(' ');
    Serial.print(s.overruns);
    Serial.print(' ');
    Serial.print(s.maxUs);
    Serial.print(' ');
    Serial.println(s.runs ? static_cast<uint32_t>(s.totalUs / s.runs) : 0);
  }
}

}  // namespace gnimu
