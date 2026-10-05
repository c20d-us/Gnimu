#pragma once

#include <stdint.h>

namespace gnimu {

// One BLE transport's counters, as the D report shows them.
struct BleStats {
  uint32_t sent = 0;       // frames the stack accepted
  uint32_t failed = 0;     // frames the stack rejected (notifications off, queue timeout, stack error)
  uint32_t refused = 0;    // frames we didn't send because the peer's MTU can't carry them
  uint32_t maxSendUs = 0;  // longest single send; C resets it
  uint32_t rxBytes = 0;    // bytes the peer wrote, queued for the loop
  uint32_t rxDropped = 0;  // bytes lost to a full inbound queue
};

}  // namespace gnimu
