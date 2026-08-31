// Gnimu - RaceBox Mini-compatible GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// Based on the Open-Source RaceBox Mini Emulator by Anchit Chandra Sekhar
// (https://github.com/anchit92/Open-Source-RaceBox-mini-Emulator)
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

// Settings live in config.h. Hardware & protocol logic lives in the
// battery/power/state/ble/gnss/imu/led/telemetry modules. This top-level
// sketch orchestrates lifecycles per the state machine (DESIGN §5).
#include "config.h"
#include "g_battery.h"
#include "g_display.h"
#include "g_ble.h"
#include "g_gnss.h"
#include "g_imu.h"
#include "g_led.h"
#include "g_log.h"
#include "g_power.h"
#include "g_state.h"
#include "g_telemetry.h"

void setup() {
  // Prologue: unconditionally hold every peripheral control pin in its safe
  // off state so a switch-off or low-voltage boot never lights the GNSS or
  // IMU. Runs before Serial.begin() so nothing can leak power before we know
  // what state to enter.
  powerHoldPeripheralsOff();

#if LOG_ENABLED
  // Wait up to 3s for the USB CDC port to enumerate on the host, so the
  // startup lines aren't dropped into the void while the terminal is still
  // re-attaching.
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
  }
#endif
  LOG_PRINTF("🚀 Gnimu [%s] starting up...\n", GNIMU_VARIANT);

  // powerBegin configures the shared ADC.
  // batteryBegin primes the sampler.
  // ledBegin sets LED to the initial off-state.
  powerBegin();
  batteryBegin();
  ledBegin();
  // Display comes up before the state classifier so BATTERY_WAIT and
  // CHARGE_ONLY boots have a screen to draw on.
  displayBegin();

  // Classify our initial state.
  // If the classifier lands in DEEP_SLEEP, this call enters System OFF directly
  // and does not return.
  const SystemState initial = stateBegin();

  // GNSS / IMU / BLE / telemetry only come up for the RUNNING branch.
  if (initial == STATE_RUNNING) {
    // Power the GNSS rail before talking to the receiver.
    // Serial1.begin inside gnssBegin claims/reclaims D6/D7 as UART pins.
    // imuBegin drives its own power pin high.
    powerGnssRailOn();
    gnssBegin();
    imuBegin();
    bleBegin();
    telemetryBegin();
  }
}

void loop() {
  // The state machine runs first so a switch-off while plugged into USB or a
  // low-voltage cutoff request is caught *before* the peripheral loops do work
  // in a state that's about to change.
  stateUpdate();

  // Battery + LED are safe calls in every state. batteryPoll advances the
  // voltage sampler, ledUpdate reflects the current state.
  batteryPoll();
  ledUpdate();
  // Peripheral polls are gated by the live state.
  if (stateCurrent() == STATE_RUNNING) {
    gnssPoll();
    imuPoll();
    telemetrySendIfReady();
    bleUpdate();
  }

  // Safe in every live state; it renders whatever stateCurrent() reports and
  // meters its own I2C cost across iterations.
  //
  // Order matters: this MUST follow gnssPoll(). displayUpdate() phase-locks its
  // I2C pushes to the navigation epoch by watching for a changed iTOW, and
  // gnssPoll() is what parses the message that changes it. Called before, it
  // would see the previous iteration's epoch and lose the alignment that keeps
  // it off the UART while a NAV-PVT is arriving.
  displayUpdate();
}
