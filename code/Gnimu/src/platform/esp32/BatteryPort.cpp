// Two-layer guard, same reasoning as GnssUart.cpp: ARDUINO_ARCH_ESP32 is a raw
// -D flag; CONFIG_IDF_TARGET_ESP32 only exists once sdkconfig.h is included.
#if defined(ARDUINO_ARCH_ESP32)
#include "sdkconfig.h"
#if CONFIG_IDF_TARGET_ESP32

#include "../BatteryPort.h"

namespace gnimu {

// USB-powered dev board, no battery.
bool BatteryPort::begin() { return false; }
uint16_t BatteryPort::readCellMv() { return 0; }
bool BatteryPort::chargerActive() { return false; }
bool BatteryPort::usbPresent() { return true; }
bool BatteryPort::switchOn() { return true; }
void BatteryPort::disableVoltageSense() {}

}  // namespace gnimu

#endif  // CONFIG_IDF_TARGET_ESP32
#endif  // ARDUINO_ARCH_ESP32
