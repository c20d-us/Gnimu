#pragma once

// The board's BLE transport, chosen at compile time. Named BleTransport rather
// than BleDevice to stay clear of the ESP32 library's <BLEDevice.h> on
// case-insensitive filesystems.
#if defined(ARDUINO_ARCH_ESP32)
#include "BleEsp32.h"
namespace gnimu {
using BleTransport = BleEsp32;
}
#elif defined(NRF52840_XXAA)
#include "BleNrf.h"
namespace gnimu {
using BleTransport = BleNrf;
}
#else
#include "BleStub.h"
namespace gnimu {
using BleTransport = BleStub;
}
#endif
