// Two-layer guard, same reasoning as GnssUart.cpp: ARDUINO_ARCH_ESP32 is a raw
// -D flag; CONFIG_IDF_TARGET_ESP32 only exists once sdkconfig.h is included.
#if defined(ARDUINO_ARCH_ESP32)
#include "sdkconfig.h"
#if CONFIG_IDF_TARGET_ESP32

#include <Arduino.h>
#include <esp_sleep.h>

#include "../PeripheralPower.h"

namespace gnimu {

// The GNSS module and MPU-6050 run straight off the dev board's 3.3 V, and
// it's USB-powered: there's nothing to switch.
bool PeripheralPower::holdAllOff() { return false; }
void PeripheralPower::gnssRailOn() {}
void PeripheralPower::gnssRailOff() {}

void PeripheralPower::reset() {
  ESP.restart();
  for (;;) {
  }
}

// No battery, so no cutoff ever calls this; deep sleep is the nearest
// equivalent should one ever be wired.
void PeripheralPower::systemOff() { esp_deep_sleep_start(); }

}  // namespace gnimu

#endif  // CONFIG_IDF_TARGET_ESP32
#endif  // ARDUINO_ARCH_ESP32
