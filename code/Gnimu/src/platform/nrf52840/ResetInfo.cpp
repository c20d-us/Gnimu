// NRF52840_XXAA, not ARDUINO_ARCH_NRF52: same reasoning as Clock.cpp.
#if defined(NRF52840_XXAA)

#include <Arduino.h>

#include "../ResetInfo.h"

namespace gnimu {

namespace {

// GPREGRET2: GPREGRET itself carries the bootloader's DFU requests. With the
// SoftDevice (BLE) running, POWER registers go through it, as in
// PeripheralPower::systemOff().
bool softDeviceEnabled() {
  uint8_t enabled = 0;
  (void)sd_softdevice_is_enabled(&enabled);
  return enabled != 0;
}

void writeKept(uint8_t code) {
  if (softDeviceEnabled()) {
    // Set only ORs bits in, so clear first.
    (void)sd_power_gpregret_clr(1, 0xFF);
    (void)sd_power_gpregret_set(1, code);
  } else {
    NRF_POWER->GPREGRET2 = code;
  }
}

}  // namespace

// The core reads RESETREAS and clears it before setup(); readResetReason()
// returns that copy. Normally a single bit; none set means power-on, which
// the nRF can't tell from a brownout.
ResetCause ResetInfo::cause() {
  const uint32_t r = readResetReason();
  if (r & POWER_RESETREAS_DOG_Msk) return ResetCause::Watchdog;
  if (r & POWER_RESETREAS_LOCKUP_Msk) return ResetCause::Lockup;
  if (r & POWER_RESETREAS_SREQ_Msk) return ResetCause::Software;
  if (r & POWER_RESETREAS_RESETPIN_Msk) return ResetCause::Pin;
  if (r & POWER_RESETREAS_VBUS_Msk) return ResetCause::WakeUsb;
  if (r & POWER_RESETREAS_OFF_Msk) return ResetCause::WakePin;
  if (r == 0) return ResetCause::PowerOnOrBrownout;
  return ResetCause::Unknown;
}

void ResetInfo::storeRestartCode(uint8_t code) { writeKept(code); }

uint8_t ResetInfo::takeRestartCode() {
  uint32_t code = 0;
  if (softDeviceEnabled()) {
    (void)sd_power_gpregret_get(1, &code);
  } else {
    code = NRF_POWER->GPREGRET2;
  }
  writeKept(0);
  return static_cast<uint8_t>(code);
}

}  // namespace gnimu

#endif
