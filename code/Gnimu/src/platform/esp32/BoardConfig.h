#pragma once

#include <stddef.h>

namespace gnimu {
namespace board {

constexpr int kGnssRxPin = 16;
constexpr int kGnssTxPin = 17;
// The stock 256-byte ring fills in ~22 ms at 115200 baud. 512 doubles that.
constexpr size_t kGnssRxBufferBytes = 512;

}  // namespace board
}  // namespace gnimu
