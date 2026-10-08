#pragma once

namespace gnimu {

// Switched power for the peripherals that have it. Implemented once per
// platform; a board whose peripherals are wired straight to its supply (the
// ESP32) implements these as no-ops.
struct PeripheralPower {
  // Every peripheral power control to its safe off state, through plain pin
  // drives only - no peripheral library - so it's safe as the very first
  // thing at boot. A no-op where nothing is switchable
  // (board::kHasSwitchedPower false).
  static void holdAllOff();
  // GNSS rail on, ahead of detection.
  static void gnssRailOn();
  // GNSS rail off. The GNSS UART must already be closed: its TX pin is taken
  // back as a GPIO and idled low.
  static void gnssRailOff();
  // Restarts the MCU through a fresh boot.
  [[noreturn]] static void reset();
  // Low-voltage cutoff: everything held off, then the chip's deepest sleep.
  // Only a fresh boot leaves it (USB plugged in, or the switch cycled).
  [[noreturn]] static void systemOff();
};

}  // namespace gnimu
