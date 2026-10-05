#pragma once

#include <stdint.h>

namespace gnimu {

// A factory-programmed, per-chip identifier: not the BLE MAC, and not
// something firmware can set. Implemented once per platform.
uint64_t hardwareId();

}  // namespace gnimu
