#pragma once

#include <stdint.h>

namespace gnimu {

// The board's battery hardware: cell voltage, charger status, USB presence,
// and the slide switch that connects the battery.
// Implemented once per platform. core/BatteryMonitor decides what the
// readings mean; this only reads them.
struct BatteryPort {
  // Configures the ADC and the charger pins. Where the board has no battery
  // (board::kHasBattery false) everything here is a stub; begin() is a no-op.
  static void begin();
  static uint16_t readCellMv();
  static bool chargerActive();
  static bool usbPresent();
  // True when the switch is on (the battery is in the circuit). One ADC read;
  // the caller paces it.
  static bool switchOn();
  // Turns the VBAT divider off before System OFF, where it would otherwise
  // keep drawing. Only for System OFF entry, which happens below the cutoff:
  // on the nRF that low cell voltage is what makes it safe (see its port).
  static void disableVoltageSense();
};

}  // namespace gnimu
