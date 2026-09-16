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

// Settings are in config.h.
// This sketch sequences module lifecycles by state (see g_state.h).
#include "config.h"
#include "g_battery.h"
#include "g_ble.h"
#include "g_gnss.h"
#include "g_imu.h"
#include "g_led.h"
#include "g_log.h"
#include "g_power.h"
#include "g_state.h"
#include "g_telemetry.h"

void setup() {
  // Hold all peripherals off until the boot state is known.
  powerHoldPeripheralsOff();

#if LOG_ENABLED
  // Wait up to 3s for USB CDC to enumerate so boot messages aren't lost. Only
  // with VBUS: on battery no host can attach, and the wait would delay the
  // low-voltage check in stateBegin() by its full length.
  Serial.begin(115200);
  if (powerUsbPresent()) {
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 3000) {
    }
  }
#endif
  LOG_PRINTF("🚀 Gnimu [%s] starting up...\n", GNIMU_VARIANT);

  powerBegin(); // configures the shared ADC
  batteryBegin();
  ledBegin();

  // Enters System OFF directly if the result is DEEP_SLEEP.
  const SystemState initial = stateBegin();

  if (initial == STATE_RUNNING) {
    powerGnssRailOn();
    // Continue even if GNSS is absent; the loop must keep running for battery
    // protection.
    (void)gnssBegin();
    imuBegin();
    bleBegin();
    telemetryBegin();
  }
}

void loop() {
  // State first, so a pending transition happens before peripheral work.
  stateUpdate();

  batteryPoll();
  ledUpdate();
  if (stateCurrent() == STATE_RUNNING) {
    gnssPoll();
    imuPoll();
    telemetrySendIfReady();
    bleUpdate();
  }
}
