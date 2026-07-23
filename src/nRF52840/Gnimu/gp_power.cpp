// Gnimu - RaceBox Mini-compatible GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "gp_power.h"
#include "config.h"
#include "gp_log.h"

// Drive the GNSS TPS63020 EN pin to its "disabled" level. Must be OUTPUT for
// the write to take effect. An INPUT pin lets the TPS's own EN pullup win.
static void gnssEnDisable() {
  pinMode(GNSS_EN_PIN, OUTPUT);
  digitalWrite(GNSS_EN_PIN, LOW); // active-high enable, so LOW disables
}

// Idle GNSS TX (D6) low as a GPIO. If Serial1 is left running, D6 idles HIGH
// and leaks through the GNSS RX ESD diode into VCC. We don't own Serial1 here,
// so we just claim the pin as GPIO. Callers that had Serial1 up must
// Serial1.end() before us.
static void gnssTxIdleLow() {
  pinMode(GNSS_TX_PIN, OUTPUT);
  digitalWrite(GNSS_TX_PIN, LOW);
}

// Turn the RGB LED off via direct pin drives (active-low: HIGH = off). Used
// before gp_led has initialized, and as the final word before DEEP_SLEEP's
// permanent halt.
static void ledPinsOff() {
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, HIGH);
  digitalWrite(LED_BLUE_PIN, HIGH);
}

// Switch-sense settings and state.
static const unsigned long SWITCH_POLL_INTERVAL_MS = 50;
static bool switchOnCached = true; // optimistic default until powerBegin runs
static unsigned long switchLastPollMs = 0;

// Switch-sense: one analogRead per throttled call.
static bool switchReadOnce() {
  const float adcMax = (float)((1UL << SAADC_RESOLUTION_BITS) - 1);
  const int mv = (int)(((float)analogRead(POWER_SWITCH_SENSE_PIN) / adcMax) *
                           SAADC_REFERENCE_MV +
                       0.5f);
  return mv < POWER_SWITCH_OFF_THRESHOLD_MV;
}

// Initialize power management: analogRead resolution, reference, sample time,
// and switch-sense pin mode. Also primes the switch state cache.
void powerBegin() {
  analogReadResolution(SAADC_RESOLUTION_BITS);
  analogReference(AR_INTERNAL_3_0); // pairs with SAADC_REFERENCE_MV == 3000
  analogSampleTime(SAADC_TACQ_US);  // long TACQ for the high-Z divider

  pinMode(POWER_SWITCH_SENSE_PIN, INPUT);

  // Prime the cached reading synchronously so stateBegin() gets an
  // authoritative switch position on its very first powerSwitchOn() call.
  // One-time cost.
  switchOnCached = switchReadOnce();
  switchLastPollMs = millis();
}

// Detect whether we're plugged into USB.
bool powerUsbPresent() {
  return (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
}

// Return true if the power switch is on (battery is physically in the circuit).
bool powerSwitchOn() {
  const unsigned long nowMs = millis();
  if ((nowMs - switchLastPollMs) >= SWITCH_POLL_INTERVAL_MS) {
    switchLastPollMs = nowMs;
    switchOnCached = switchReadOnce();
  }
  return switchOnCached;
}

// Disable all peripherals.
void powerHoldPeripheralsOff() {
  gnssEnDisable();
  gnssTxIdleLow();
  pinMode(IMU_POWER_PIN, OUTPUT);
  digitalWrite(IMU_POWER_PIN, LOW);
  ledPinsOff();
}

// Restore power to the GNSS rail.
void powerGnssRailOn() {
  // Release EN to INPUT (hi-Z). The TPS63020's own EN pullup then drives it
  // high and the rail comes up.
  pinMode(GNSS_EN_PIN, INPUT);
}

// Disable power to the GNSS rail.
void powerGnssRailOff() {
  // Drive EN low to disable the TPS63020 rail.
  gnssEnDisable();
  gnssTxIdleLow();
}

// Enter deep sleep mode, disabling all peripherals and powering down the
// system. Does not return.
void powerEnterDeepSleep() {
  LOG_PRINTLN("💤 Entering deep sleep (System OFF).");
  LOG_FLUSH();
  // Hold everything off - GPIO state persists through System OFF, so anything
  // left driven will keep drawing until the reset.
  powerHoldPeripheralsOff();
  // With the SoftDevice enabled (Bluefruit) this must go through the SD call.
  sd_power_system_off();
  while (1) {
    delay(100); // in case a debugger is attached and the call returns
  }
}
