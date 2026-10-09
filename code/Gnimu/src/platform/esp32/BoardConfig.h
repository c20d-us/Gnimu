#pragma once

#include <stddef.h>

namespace gnimu {
namespace board {

// What this board has, fixed at compile time: USB-powered, peripherals wired
// straight to 3.3 V.
constexpr bool kHasBattery = false;
constexpr bool kHasSwitchedPower = false;

constexpr int kGnssRxPin = 16;
constexpr int kGnssTxPin = 17;
// The stock 256-byte ring fills in ~22 ms at 115200 baud. 512 doubles that.
constexpr size_t kGnssRxBufferBytes = 512;

// The core's I2C defaults; an external MPU-6050 is wired here.
constexpr int kImuSdaPin = 21;
constexpr int kImuSclPin = 22;

// The dev board's onboard LED: single color, blue.
constexpr int kStatusLedPin = 2;

}  // namespace board
}  // namespace gnimu
