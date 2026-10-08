#pragma once

#include <Arduino.h>

namespace gnimu {
namespace board {

// What this board has, fixed at compile time: a LiPo with charger and slide
// switch, and switched power for the GNSS rail and the IMU.
constexpr bool kHasBattery = true;
constexpr bool kHasSwitchedPower = true;

// Drives HIGH to power the onboard LSM6DS3TR-C. Defined identically in every
// XIAO nRF52840 variant.h (even ones without the chip populated), so this
// compiles the same everywhere; only Sense/Sense Plus actually have the chip.
constexpr int kImuPowerPin = PIN_LSM6DS3TR_C_POWER;

// Which wiring this build targets. Breadboard: the current development unit.
// Classic: units wired for Gnimu Classic. Polarity is the same on both; only
// the pins differ. Change this to build for a Classic-wired unit.
enum class Wiring : uint8_t { Breadboard, Classic };
constexpr Wiring kWiring = Wiring::Breadboard;

// EN of the regulator that powers only the GNSS module (TPS63070 on the
// breadboard, TPS63020 in Classic). LOW disables it; released to an input,
// the regulator's own pull-up enables it.
constexpr int kGnssEnablePin = kWiring == Wiring::Breadboard ? D1 : D9;

// The slide switch's spare throw through a 510k/510k divider (same circuit on
// both wirings): ~2 V switch OFF, ~0 V switch ON, read through the ADC.
constexpr int kSwitchSensePin = kWiring == Wiring::Breadboard ? A0 : A4;
constexpr float kSwitchOffThresholdMv = 800.0f;  // above this = switch OFF

// Serial1 TX, the GNSS receiver's RX: idled low as a GPIO while the GNSS is
// unpowered (see PeripheralPower.cpp).
constexpr int kGnssTxPin = PIN_SERIAL1_TX;

}  // namespace board
}  // namespace gnimu
