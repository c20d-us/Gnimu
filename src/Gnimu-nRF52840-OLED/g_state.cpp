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

#include "g_state.h"
#include "config.h"
#include "g_battery.h"
#include "g_ble.h"
#include "g_display.h"
#include "g_gnss.h"
#include "g_log.h"
#include "g_power.h"

static SystemState current = STATE_RUNNING;

// millis() when the switch first read off, or 0. See
// STATE_SWITCH_OFF_DEBOUNCE_MS.
static unsigned long switchOffSinceMs = 0;

// True unless the switch has read off continuously for the debounce period.
// Switch-on is immediate.
static bool switchOnDebounced(unsigned long nowMs) {
  if (powerSwitchOn()) {
    switchOffSinceMs = 0;
    return true;
  }
  if (switchOffSinceMs == 0)
    switchOffSinceMs = nowMs;
  return (nowMs - switchOffSinceMs) < STATE_SWITCH_OFF_DEBOUNCE_MS;
}

// millis() when idling began, or 0. See STATE_IDLE_TIMEOUT_MIN.
static unsigned long idleSinceMs = 0;

// Shut down from RUNNING and enter System OFF. Does not return.
static void enterDeepSleepFrom(const char *reasonLog) {
  LOG_PRINTLN(reasonLog);
  bleStop();
  gnssEnd();
  // System OFF doesn't cut the panel's rail.
  displaySleep();
  LOG_FLUSH();
  powerEnterDeepSleep(); // does not return
}

// Enter BATTERY_WAIT from RUNNING.
static void enterBatteryWait() {
  LOG_PRINTLN("🪫 -> BATTERY_WAIT (switch off).");
  // gnssEnd() must precede powerHoldPeripheralsOff(): the UART holds TX high
  // until released.
  bleStop();
  gnssEnd();
  powerHoldPeripheralsOff();
  // Flush now; power may drop soon after a switch-off on battery.
  LOG_FLUSH();
  current = STATE_BATTERY_WAIT;
}

// Enter CHARGE_ONLY from RUNNING. Guarded so it isn't an unused function when
// the flag is 0.
#if STATE_CHARGE_ONLY_ON_USB
static void enterChargeOnly() {
  LOG_PRINTLN("🔌 -> CHARGE_ONLY (USB in, peripherals held off).");
  bleStop();
  gnssEnd();
  powerHoldPeripheralsOff();
  LOG_FLUSH();
  current = STATE_CHARGE_ONLY;
}
#endif // STATE_CHARGE_ONLY_ON_USB

SystemState stateBegin() {
  const bool switchOn = powerSwitchOn(); // first call is a fresh read
  const bool usb = powerUsbPresent();
  const float peak = batteryGetStatus().voltage;

  if (!switchOn) {
    current = STATE_BATTERY_WAIT;
    // With the switch off the MCU should be on USB; log it rather than assume.
    LOG_PRINTF("Boot -> BATTERY_WAIT (switch off, USB %s).\n",
               usb ? "in" : "absent");
    return current;
  }

  if (!usb && peak < BATTERY_CUTOFF_V) {
    LOG_PRINTF("Boot -> DEEP_SLEEP (VBAT %.2f V < cutoff, no USB).\n", peak);
    displaySleep(); // harmless before displayBegin()
    LOG_FLUSH();
    current = STATE_DEEP_SLEEP;
    powerEnterDeepSleep(); // does not return
  }

#if STATE_CHARGE_ONLY_ON_USB
  if (usb) {
    current = STATE_CHARGE_ONLY;
    LOG_PRINTLN("Boot -> CHARGE_ONLY (USB in, switch on).");
    return current;
  }
#endif

  current = STATE_RUNNING;
  LOG_PRINTLN("Boot -> RUNNING.");
  return current;
}

void stateUpdate() {
  const unsigned long nowMs = millis();

  switch (current) {
  case STATE_RUNNING: {
    // 1. Low voltage on battery. On USB the charger holds VBAT up.
    if (batteryCutoffRequested() && !powerUsbPresent()) {
      current = STATE_DEEP_SLEEP;
      enterDeepSleepFrom(
          "RUNNING -> DEEP_SLEEP (voltage cutoff)."); // no return
    }
    // 2. Switch off (debounced).
    if (!switchOnDebounced(nowMs)) {
      enterBatteryWait();
      break;
    }
#if STATE_CHARGE_ONLY_ON_USB
    // 3. USB plugged in.
    if (powerUsbPresent()) {
      enterChargeOnly();
      break;
    }
#endif
    // 4. Idle: no subscribed client and no USB for STATE_IDLE_TIMEOUT_MIN.
    if (bleIsSubscribed() || powerUsbPresent()) {
      idleSinceMs = 0;
    } else {
      if (idleSinceMs == 0)
        idleSinceMs = nowMs;
      if (nowMs - idleSinceMs >=
          (unsigned long)STATE_IDLE_TIMEOUT_MIN * 60000UL) {
        current = STATE_DEEP_SLEEP;
        enterDeepSleepFrom("RUNNING -> DEEP_SLEEP (idle: no subscribed client, "
                           "no USB)."); // no return
      }
    }
    break;
  }

  case STATE_CHARGE_ONLY: {
    // Unplug or switch off resets, and stateBegin() picks the next state.
    if (!powerUsbPresent()) {
      LOG_PRINTLN("CHARGE_ONLY: USB removed - resetting.");
      LOG_FLUSH();
      delay(50);
      NVIC_SystemReset(); // does not return
    }
    if (!switchOnDebounced(nowMs)) {
      LOG_PRINTLN("CHARGE_ONLY: switch off - resetting.");
      LOG_FLUSH();
      delay(50);
      NVIC_SystemReset(); // does not return
    }
    break;
  }

  case STATE_BATTERY_WAIT: {
    // Switch on resets. Unplugging USB simply removes power.
    if (powerSwitchOn()) {
      LOG_PRINTLN("BATTERY_WAIT: switch on - resetting.");
      LOG_FLUSH();
      delay(50);          // let Serial drain
      NVIC_SystemReset(); // does not return
    }
    break;
  }

  case STATE_DEEP_SLEEP:
    // Unreachable; listed for switch exhaustiveness.
    break;
  }
}

SystemState stateCurrent() { return current; }
