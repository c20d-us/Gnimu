// Gnimu - GNSS+IMU streaming telemetry
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

#include "g_power.h"
#include "config.h"
#include "g_log.h"

// Drive GNSS EN low. The pin must be an output to override the regulator's
// pullup.
static void gnssEnDisable() {
  pinMode(GNSS_EN_PIN, OUTPUT);
  digitalWrite(GNSS_EN_PIN, LOW);
}

// Hold GNSS TX low so it can't back-power the receiver through its RX pin.
// Serial1 must already be closed.
static void gnssTxIdleLow() {
  pinMode(GNSS_TX_PIN, OUTPUT);
  digitalWrite(GNSS_TX_PIN, LOW);
}

// RGB LED off by direct pin drive (active-low).
static void ledPinsOff() {
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, HIGH);
  digitalWrite(LED_BLUE_PIN, HIGH);
}

static bool switchOnCached = true;
static unsigned long switchLastPollMs = 0;

static bool switchReadOnce() {
  const float adcMax = (float)((1UL << SAADC_RESOLUTION_BITS) - 1);
  const int mv = (int)(((float)analogRead(POWER_SWITCH_SENSE_PIN) / adcMax) *
                           SAADC_REFERENCE_MV +
                       0.5f);
  return mv < POWER_SWITCH_OFF_THRESHOLD_MV;
}

void powerBegin() {
  analogReadResolution(SAADC_RESOLUTION_BITS);
  analogReference(AR_INTERNAL_3_0); // matches SAADC_REFERENCE_MV
  analogSampleTime(SAADC_TACQ_US);  // long acquisition for the high-Z divider

  pinMode(POWER_SWITCH_SENSE_PIN, INPUT);

  // Prime the cache for stateBegin().
  switchOnCached = switchReadOnce();
  switchLastPollMs = millis();
}

bool powerUsbPresent() {
  return (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) != 0;
}

bool powerSwitchOn() {
  const unsigned long nowMs = millis();
  if ((nowMs - switchLastPollMs) >= POWER_SWITCH_POLL_INTERVAL_MS) {
    switchLastPollMs = nowMs;
    switchOnCached = switchReadOnce();
  }
  return switchOnCached;
}

void powerHoldPeripheralsOff() {
  gnssEnDisable();
  gnssTxIdleLow();
  pinMode(IMU_POWER_PIN, OUTPUT);
  digitalWrite(IMU_POWER_PIN, LOW);
  ledPinsOff();
}

void powerGnssRailOn() {
  pinMode(GNSS_EN_PIN, INPUT); // hi-Z: the regulator's pullup enables the rail
}

void powerEnterDeepSleep() {
  LOG_PRINTLN("💤 Entering deep sleep (System OFF).");
  LOG_FLUSH();
  // GPIO state persists through System OFF.
  powerHoldPeripheralsOff();
  // Must go through the SoftDevice while Bluefruit is enabled.
  sd_power_system_off();
  while (1) {
    delay(100); // only reached under a debugger
  }
}
