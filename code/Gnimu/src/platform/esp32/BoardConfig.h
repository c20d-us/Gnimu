#pragma once

#include <stddef.h>

namespace gnimu {
namespace board {

constexpr int kGnssRxPin = 16;
constexpr int kGnssTxPin = 17;
// The stock 256-byte ring fills in ~22 ms at 115200 baud. 512 doubles that.
constexpr size_t kGnssRxBufferBytes = 512;

// The core's I2C defaults; an external MPU-6050 is wired here.
constexpr int kImuSdaPin = 21;
constexpr int kImuSclPin = 22;

// The dev board's onboard LED (Classic's LED_ONBOARD_PIN): single color, blue.
constexpr int kStatusLedPin = 2;

}  // namespace board
}  // namespace gnimu
