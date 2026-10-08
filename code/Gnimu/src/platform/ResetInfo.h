#pragma once

#include <stdint.h>

#include "../core/ResetReason.h"

namespace gnimu {

// Why the chip last reset, and one byte kept across our own restarts.
// Implemented once per platform.
struct ResetInfo {
  // The hardware's reason for this boot.
  static ResetCause cause();
  // Kept across software, watchdog and pin resets, but not power loss. Set
  // just before a restart we request.
  static void storeRestartCode(uint8_t code);
  // Reads the kept byte and clears it, so it never outlives the boot that
  // read it. Call once, at boot.
  static uint8_t takeRestartCode();
};

}  // namespace gnimu
