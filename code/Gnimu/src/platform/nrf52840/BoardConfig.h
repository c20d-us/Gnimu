// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

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

// Which wiring this build targets: V2, the current hardware revision, or
// Classic, the earlier revision's PCB. Polarity is the same on both; only the
// pins differ.
enum class Wiring : uint8_t { V2, Classic };
constexpr Wiring kWiring = Wiring::V2;
// constexpr Wiring kWiring = Wiring::Classic;
// As the console's V command shows it.
constexpr const char* kWiringName = kWiring == Wiring::V2 ? "V2" : "Classic";

// EN of the regulator that powers only the GNSS module (TPS63070 on V2,
// TPS63020 on Classic). LOW disables it; released to an input, the
// regulator's own pull-up enables it.
constexpr int kGnssEnablePin = kWiring == Wiring::V2 ? D1 : D9;

// The slide switch's spare throw through a 510k/510k divider (same circuit on
// both wirings): ~2 V switch OFF, ~0 V switch ON, read through the ADC.
constexpr int kSwitchSensePin = kWiring == Wiring::V2 ? A0 : A4;
constexpr float kSwitchOffThresholdMv = 800.0f; // above this = switch OFF

// Serial1 TX, the GNSS receiver's RX: idled low as a GPIO while the GNSS is
// unpowered (see PeripheralPower.cpp).
constexpr int kGnssTxPin = PIN_SERIAL1_TX;

} // namespace board
} // namespace gnimu
