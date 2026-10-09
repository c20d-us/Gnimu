// Two-layer guard: see Clock.cpp for why.
#if defined(ARDUINO_ARCH_ESP32)
#include "sdkconfig.h"
#if CONFIG_IDF_TARGET_ESP32

#include <Arduino.h>
#include <esp_task_wdt.h>

#include "../../app/SystemConfig.h"
#include "../Watchdog.h"

namespace gnimu {

namespace {

// The same idle-task checks the core configured.
#ifdef CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU0
constexpr uint32_t kIdleCpu0 = 1u << 0;
#else
constexpr uint32_t kIdleCpu0 = 0;
#endif
#ifdef CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1
constexpr uint32_t kIdleCpu1 = 1u << 1;
#else
constexpr uint32_t kIdleCpu1 = 0;
#endif

}  // namespace

// The core starts the task watchdog but only has it watch the idle tasks; the
// loop task (where setup() runs too) isn't on it. Retimed to ours, keeping the
// panic, which prints the hung task's backtrace on the console before
// resetting.
bool Watchdog::start() {
  const esp_task_wdt_config_t cfg = {config::kWatchdogMs, kIdleCpu0 | kIdleCpu1, true};
  return esp_task_wdt_reconfigure(&cfg) == ESP_OK &&
         esp_task_wdt_add(nullptr) == ESP_OK;  // the calling task: the loop task
}

void Watchdog::feed() { (void)esp_task_wdt_reset(); }

}  // namespace gnimu

#endif  // CONFIG_IDF_TARGET_ESP32
#endif  // ARDUINO_ARCH_ESP32
