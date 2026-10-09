// Two-layer guard: see Clock.cpp for why.
#if defined(ARDUINO_ARCH_ESP32)
#include "sdkconfig.h"
#if CONFIG_IDF_TARGET_ESP32

#include <Arduino.h>
#include <Wire.h>

#include "../../app/Config.h"
#include "../ImuBus.h"
#include "BoardConfig.h"

namespace gnimu {

void imuBusBegin(uint32_t clockHz) {
  Wire.begin(board::kImuSdaPin, board::kImuSclPin);
  Wire.setClock(clockHz);
  // After begin(), which applies the core's 50 ms default.
  Wire.setTimeOut(config::kImuI2cTimeoutMs);
}

}  // namespace gnimu

#endif  // CONFIG_IDF_TARGET_ESP32
#endif  // ARDUINO_ARCH_ESP32
