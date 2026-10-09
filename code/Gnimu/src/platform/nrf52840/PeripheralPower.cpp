// Same four-board guard as GnssUart.cpp: the pins below are the four Seeed XIAO
// nRF52840 boards' shared pinout.
#if defined(ARDUINO_Seeed_XIAO_nRF52840) || defined(ARDUINO_Seeed_XIAO_nRF52840_Sense) || \
    defined(ARDUINO_Seeed_XIAO_nRF52840_Plus) || defined(ARDUINO_Seeed_XIAO_nRF52840_Sense_Plus)

#include <Arduino.h>

#include "../PeripheralPower.h"
#include "BoardConfig.h"

namespace gnimu {

namespace {

// Must be an OUTPUT for the LOW to win; as an input, the regulator's own EN
// pull-up enables the rail.
void gnssRailDisable() {
  pinMode(board::kGnssEnablePin, OUTPUT);
  digitalWrite(board::kGnssEnablePin, LOW);
}

// With the GNSS unpowered, a UART TX idling HIGH leaks through the receiver's
// RX protection diode into its supply, half-powering it. Idle it LOW as a
// plain GPIO instead.
void gnssTxIdleLow() {
  pinMode(board::kGnssTxPin, OUTPUT);
  digitalWrite(board::kGnssTxPin, LOW);
}

}  // namespace

void PeripheralPower::holdAllOff() {
  gnssRailDisable();
  gnssTxIdleLow();
  pinMode(board::kImuPowerPin, OUTPUT);
  digitalWrite(board::kImuPowerPin, LOW);
}

// Released, not driven HIGH: the regulator's pull-up is what holds it enabled
// whenever the MCU isn't driving it.
// GnssUart::begin() reclaims the TX pin as a UART pin.
void PeripheralPower::gnssRailOn() { pinMode(board::kGnssEnablePin, INPUT); }

void PeripheralPower::gnssRailOff() {
  gnssRailDisable();
  gnssTxIdleLow();
}

void PeripheralPower::reset() { NVIC_SystemReset(); }

// GPIO state persists through System OFF, so everything
// is held off first: anything left driven keeps drawing until the next boot.
void PeripheralPower::systemOff() {
  holdAllOff();
  // With the SoftDevice (BLE) running, System OFF must go through it; without
  // it, that call fails and the register is written directly.
  uint8_t sdEnabled = 0;
  (void)sd_softdevice_is_enabled(&sdEnabled);
  if (sdEnabled) {
    sd_power_system_off();
  } else {
    NRF_POWER->SYSTEMOFF = 1;
  }
  for (;;) {
  }  // only reached under a debugger
}

}  // namespace gnimu

#endif
