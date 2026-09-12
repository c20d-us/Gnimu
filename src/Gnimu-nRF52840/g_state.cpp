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

#include "g_state.h"
#include "config.h"
#include "g_battery.h"
#include "g_ble.h"
#include "g_gnss.h"
#include "g_log.h"
#include "g_power.h"

// Current state
static SystemState current = STATE_RUNNING;

// Switch-off debounce anchor - see STATE_SWITCH_OFF_DEBOUNCE_MS in config.h
// for the rationale. Switch-ON is instant.
static unsigned long switchOffSinceMs = 0;

// True unless the switch has been read OFF continuously for >= debounce.
// switchOffSinceMs tracks the earliest OFF; any ON resets it.
static bool switchOnDebounced(unsigned long nowMs) {
  if (powerSwitchOn()) {
    switchOffSinceMs = 0;
    return true;
  }
  if (switchOffSinceMs == 0)
    switchOffSinceMs = nowMs;
  return (nowMs - switchOffSinceMs) < STATE_SWITCH_OFF_DEBOUNCE_MS;
}

// Idle-cutoff anchor - see STATE_IDLE_TIMEOUT_MIN in config.h. 0 is the
// sentinel for "clock not running" (mirrors switchOffSinceMs above).
static unsigned long idleSinceMs = 0;

// Shared by every RUNNING -> DEEP_SLEEP path: kick BLE clients,
// release Serial1 (same UART-vs-GPIO ordering rule as enterBatteryWait()),
// flush, then hand off to powerEnterDeepSleep() (which itself calls
// powerHoldPeripheralsOff() before sd_power_system_off()). Does not return.
static void enterDeepSleepFrom(const char *reasonLog) {
  LOG_PRINTLN(reasonLog);
  bleStop();
  gnssEnd();
  LOG_FLUSH();
  powerEnterDeepSleep(); // does not return; also forces the LED off
}

// Entry action for BATTERY_WAIT. Boot-classified entry doesn't need it:
// setup()'s prologue already called powerHoldPeripheralsOff() before
// batteryBegin() ran, and RUNNING bring-up is skipped for BATTERY_WAIT boots.
static void enterBatteryWait() {
  LOG_PRINTLN("🪫 -> BATTERY_WAIT (switch off).");
  // Kick BLE clients and stop advertising while Serial is still up (so the
  // disconnect callback's log lands cleanly). Then release Serial1 BEFORE
  // powerHoldPeripheralsOff() drives D6 low - while the UART peripheral
  // owns D6 it keeps idling HIGH and pinMode/digitalWrite are silently
  // ignored, phantom-powering the GNSS through its RX ESD diode.
  bleStop();
  gnssEnd();
  powerHoldPeripheralsOff();
  // Drain all queued log output NOW, after every helper above has had a
  // chance to log - a battery-only switch-off may cut MCU power very
  // shortly after this returns.
  LOG_FLUSH();
  current = STATE_BATTERY_WAIT;
}

// Runtime entry to CHARGE_ONLY from RUNNING (boot-classified entries have
// already skipped peripheral bring-up in setup(), no teardown needed there).
// Same UART-ownership rule as BATTERY_WAIT: bleStop() + gnssEnd() first,
// then hold-off.
//
// Guarded by the same flag as its only call site below, because a static
// function that is defined and never referenced is a -Wunused-function warning
// - and with STATE_CHARGE_ONLY_ON_USB at 0 it was the one warning standing
// between these builds and a clean one. The point is not the warning itself,
// it is that the NEXT one should be visible.
//
// Deliberately #if rather than [[maybe_unused]]: this ties the code's existence
// to the flag that controls it, keeps all three CHARGE_ONLY guards greppable
// together, and does not permanently suppress the signal on the day this goes
// unused for some other reason.
//
// NOTE the asymmetry with `case STATE_CHARGE_ONLY:` in stateUpdate(), which is
// equally unreachable at 0 and is deliberately NOT guarded. That switch has no
// default:, so it is exhaustive over SystemState - dropping a case would just
// trade -Wunused-function for -Wswitch. Case labels raise no unused warning,
// so there is nothing to fix there.
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
  const bool switchOn =
      powerSwitchOn(); // denoised read; first call is always fresh
  const bool usb = powerUsbPresent();
  const float peak = batteryGetStatus().voltage;

  if (!switchOn) {
    current = STATE_BATTERY_WAIT;
    // Reports `usb` rather than asserting it. The claim is sound - the slide
    // switch is 3-pole and takes the cell physically out of circuit, so an MCU
    // executing at all with the switch off must be running on USB - but nothing
    // in this branch established it. Printing the observation costs nothing, and
    // an "absent" here would mean something genuinely surprising about the
    // hardware rather than passing unnoticed.
    LOG_PRINTF("Boot -> BATTERY_WAIT (switch off, USB %s).\n",
               usb ? "in" : "absent");
    return current;
  }

  if (!usb && peak < BATTERY_CUTOFF_V) {
    // Voltage grossly low with no USB - skip GNSS cold-start and go straight
    // to System OFF.
    LOG_PRINTF("Boot -> DEEP_SLEEP (VBAT %.2f V < cutoff, no USB).\n", peak);
    LOG_FLUSH();
    current = STATE_DEEP_SLEEP;
    powerEnterDeepSleep(); // does not return
  }

#if STATE_CHARGE_ONLY_ON_USB
  if (usb) {
    // Plugged in at boot -> park in CHARGE_ONLY so the charger gets max
    // current to the cell. Peripheral bring-up in setup() is skipped for any
    // state other than RUNNING, so nothing spins up here.
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
    // Priority 1: voltage cutoff on battery. USB present clamps VBAT via the
    // charger, so we let the cell recover instead of cutting.
    if (batteryCutoffRequested() && !powerUsbPresent()) {
      current = STATE_DEEP_SLEEP;
      enterDeepSleepFrom(
          "RUNNING -> DEEP_SLEEP (voltage cutoff)."); // no return
    }
    // Priority 2: switch flipped off. Debounced against noise spikes.
    // Break after the transition so no lower-priority check can overwrite the
    // new state on the same pass.
    if (!switchOnDebounced(nowMs)) {
      enterBatteryWait();
      break;
    }
#if STATE_CHARGE_ONLY_ON_USB
    // Priority 3: USB plugged in -> drop to CHARGE_ONLY so the charger gets
    // max current. No edge detection needed: the boot classifier already
    // sends USB-at-boot straight to CHARGE_ONLY, so if we're in RUNNING with
    // the flag on, USB was absent at boot; the very first loop after
    // plug-in transitions us.
    if (powerUsbPresent()) {
      enterChargeOnly();
      break;
    }
#endif
    // Priority 4: nobody using it for STATE_IDLE_TIMEOUT_MIN -> DEEP_SLEEP.
    // "Using" is a SUBSCRIBED client, not merely a connected one - see the
    // config.h note - and the clock stands still on USB power, where there is
    // no cell to protect. Same debounce-anchor idiom as switchOffSinceMs.
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
    // Exit is via reset -> boot classifier picks the right next state:
    //   USB removed  -> RUNNING (usb absent, switch on)
    //   Switch off   -> BATTERY_WAIT (switch off wins in the classifier)
    // Either edge is exclusive - a single sample is enough for USB
    // (register bit, not analog); the switch uses its debounce as usual.
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
    // Switch back on -> reset back to boot classification. User-intentional
    // action; a single confirmed ON reading is enough.
    if (powerSwitchOn()) {
      LOG_PRINTLN("BATTERY_WAIT: switch on - resetting.");
      LOG_FLUSH();
      delay(50);          // let Serial drain before the reset kills it
      NVIC_SystemReset(); // does not return
    }
    // USB removal is not our concern - the rail vanishes and the MCU dies.
    break;
  }

  case STATE_DEEP_SLEEP:
    // Unreachable in loop() - powerEnterDeepSleep() never returned. Present
    // only for enum exhaustiveness.
    break;
  }
}

SystemState stateCurrent() { return current; }
