// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// Same four-board guard as GnssUart.cpp: the battery pins below are the four
// Seeed XIAO nRF52840 boards' own (PIN_VBAT is 32 on base/Sense and 35 on the
// Plus boards; the macro covers both).
#if defined(ARDUINO_Seeed_XIAO_nRF52840) || defined(ARDUINO_Seeed_XIAO_nRF52840_Sense) || \
    defined(ARDUINO_Seeed_XIAO_nRF52840_Plus) || defined(ARDUINO_Seeed_XIAO_nRF52840_Sense_Plus)

#include <Arduino.h>

#include "../BatteryPort.h"
#include "BoardConfig.h"

namespace gnimu {

namespace {

// The charger's ~CHG output, P0.17: Arduino pin 23 on all four variants
// (checked in each variant.cpp pin map). Open-drain, active low.
constexpr int kChargeStatusPin = 23;

constexpr float kAdcReferenceMv = 3000.0f;  // AR_INTERNAL_3_0
constexpr float kAdcMax = 4095.0f;          // 12-bit
// The XIAO's VBAT divider: 1 M over 510 k.
constexpr float kDividerRatio = (1000000.0f + 510000.0f) / 510000.0f;

}  // namespace

void BatteryPort::begin() {
  analogReadResolution(12);
  analogReference(AR_INTERNAL_3_0);
  // Long acquisition time for the high-impedance divider.
  analogSampleTime(40);
  // LOW enables the divider. Left on: it draws only microamps, and Seeed
  // advises keeping it enabled while charging. Switched off before System OFF.
  pinMode(VBAT_ENABLE, OUTPUT);
  digitalWrite(VBAT_ENABLE, LOW);
  // HICHG LOW selects the ~100 mA charge rate rather than ~50 mA.
  pinMode(PIN_CHARGING_CURRENT, OUTPUT);
  digitalWrite(PIN_CHARGING_CURRENT, LOW);
  pinMode(kChargeStatusPin, INPUT_PULLUP);
  pinMode(board::kSwitchSensePin, INPUT);
}

uint16_t BatteryPort::readCellMv() {
  return static_cast<uint16_t>(analogRead(PIN_VBAT) * (kAdcReferenceMv / kAdcMax) * kDividerRatio +
                               0.5f);
}

bool BatteryPort::chargerActive() { return digitalRead(kChargeStatusPin) == LOW; }

// The divider through the switch's spare throw reads ~2 V with the
// switch OFF and ~0 V with it ON; analog, not digital, so the threshold sits
// well clear of both.
bool BatteryPort::switchOn() {
  // The ADC alternates between this high-impedance divider and VBAT; the
  // first conversion after a channel switch can carry charge from the last
  // one ("ghost" readings), so it's thrown away. The boot
  // classification acts on a single reading with no debounce behind it.
  (void)analogRead(board::kSwitchSensePin);
  const float mv = analogRead(board::kSwitchSensePin) * (kAdcReferenceMv / kAdcMax);
  return mv < board::kSwitchOffThresholdMv;
}

// HIGH lifts the divider's 510 k leg to 3.3 V instead of grounding it, which
// saves its ~2.3 uA in System OFF (Seeed's sleep advice) but puts P0.31 at
// about 3.3 V + (VBAT - 3.3 V) / 3. That passes the pin's 3.6 V limit only
// with a cell near 4.2 V, i.e. while charging - which is why Seeed says never
// HIGH while charging. It's safe here because the only caller is System OFF
// entry, below the 3.30 V cutoff, and plugging in USB wakes the chip into a
// boot that drives it LOW again before the cell can charge.
void BatteryPort::disableVoltageSense() { digitalWrite(VBAT_ENABLE, HIGH); }

bool BatteryPort::usbPresent() {
  return (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
}

}  // namespace gnimu

#endif
