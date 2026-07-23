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
#include "g_imu.h"
#include "g_log.h"
#include "g_power.h"

// Current state
static SystemState current = STATE_RUNNING;

// Switch-off debounce anchor. A floating/unwired switch-sense pin, and even a
// wired divider under EMI, can produce transient OFF readings; require the
// OFF to be sustained for SWITCH_OFF_DEBOUNCE_MS before entering BATTERY_WAIT
// so a noise spike can't reset a happily-running device. Switch-ON is instant.
static const unsigned long SWITCH_OFF_DEBOUNCE_MS = 500;
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
  return (nowMs - switchOffSinceMs) < SWITCH_OFF_DEBOUNCE_MS;
}

// LIGHT_SLEEP tracking
// 0 is the sentinel for "clock not running" (mirrors switchOffSinceMs above).
static unsigned long bleDisconnectedSinceMs = 0;
static unsigned long lightSleepEnteredMs = 0;
// True from LIGHT_SLEEP entry until the GNSS-cutoff escalation fires; tells
// exitLightSleep() whether to wake GNSS from backup (gnssWake()) or from a
// full EN-cut (powerGnssRailOn() + gnssBegin() cold-start).
static bool gnssInBackup = false;
// Heartbeat so LIGHT_SLEEP isn't completely silent - nothing else logs while
// asleep, so without this there's no way to tell how long it's been asleep
// until it wakes back up.
static unsigned long lightSleepLastReportMs = 0;

// Shared by every RUNNING/LIGHT_SLEEP -> DEEP_SLEEP path: kick BLE clients,
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
static void enterChargeOnly() {
  LOG_PRINTLN("🔌 -> CHARGE_ONLY (USB in, peripherals held off).");
  bleStop();
  gnssEnd();
  powerHoldPeripheralsOff();
  LOG_FLUSH();
  current = STATE_CHARGE_ONLY;
}

// Entry action for LIGHT_SLEEP from RUNNING. Reversible without a reset -
// BLE stays advertising/connectable throughout (no bleStop()), only GNSS and
// IMU change. gnssInBackup starts true; the GNSS-cutoff escalation flips it.
static void enterLightSleep(unsigned long nowMs) {
  LOG_PRINTLN("🌙 RUNNING -> LIGHT_SLEEP (idle timeout).");
  gnssSleep();
  imuArmWake();
  gnssInBackup = true;
  lightSleepEnteredMs = nowMs;
  lightSleepLastReportMs = nowMs;
  current = STATE_LIGHT_SLEEP;
}

// Exit action back to RUNNING, from either wake trigger (BLE connect or IMU
// wake). Resets the idle clock so the device gets a full fresh
// STATE_IDLE_TIMEOUT_MIN window in RUNNING before it can re-enter
// LIGHT_SLEEP - without this, waking via IMU motion (not a BLE connect)
// would leave bleDisconnectedSinceMs pointing at a stale pre-sleep timestamp
// and immediately re-trigger entry on the very next stateUpdate().
static void exitLightSleep(const char *reasonLog) {
  LOG_PRINTLN(reasonLog);
  if (gnssInBackup) {
    gnssWake();
  } else {
    // Escalated to a full EN-cut already - cold-start it back up, same as a
    // normal RUNNING boot.
    powerGnssRailOn();
    gnssBegin();
  }
  imuDisarmWake();
  bleDisconnectedSinceMs = 0;
  current = STATE_RUNNING;
}

SystemState stateBegin() {
  const bool switchOn =
      powerSwitchOn(); // denoised read; first call is always fresh
  const bool usb = powerUsbPresent();
  const float peak = batteryGetStatus().voltage;

  if (!switchOn) {
    current = STATE_BATTERY_WAIT;
    LOG_PRINTLN("Boot -> BATTERY_WAIT (switch off, USB in).");
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
    if (!switchOnDebounced(nowMs)) {
      enterBatteryWait();
    }
#if STATE_CHARGE_ONLY_ON_USB
    // Priority 3: USB plugged in -> drop to CHARGE_ONLY so the charger gets
    // max current. No edge detection needed: the boot classifier already
    // sends USB-at-boot straight to CHARGE_ONLY, so if we're in RUNNING with
    // the flag on, USB was absent at boot; the very first loop after
    // plug-in transitions us.
    if (powerUsbPresent()) {
      enterChargeOnly();
    }
#endif
    // Priority 4: idleElapsed -> LIGHT_SLEEP. bleDisconnectedSinceMs tracks
    // the earliest continuous disconnect, same debounce-anchor idiom as
    // switchOffSinceMs; any connect resets it.
    if (bleIsConnected()) {
      bleDisconnectedSinceMs = 0;
    } else {
      if (bleDisconnectedSinceMs == 0)
        bleDisconnectedSinceMs = nowMs;
      if (nowMs - bleDisconnectedSinceMs >=
          (unsigned long)STATE_IDLE_TIMEOUT_MIN * 60000UL) {
        enterLightSleep(nowMs);
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

  case STATE_LIGHT_SLEEP: {
    // Heartbeat - nothing else logs while asleep, so without this there's no
    // way to tell it's still alive/how long it's been asleep until it wakes.
    if (nowMs - lightSleepLastReportMs >= LOG_LIGHT_SLEEP_INTERVAL_MS) {
      lightSleepLastReportMs = nowMs;
      LOG_PRINTF("🌙 LIGHT_SLEEP: %lus elapsed (GNSS %s)\n",
                 (nowMs - lightSleepEnteredMs) / 1000,
                 gnssInBackup ? "backup" : "EN-cut");
    }

    // Priority 1: same voltage-cutoff safety edge as RUNNING.
    if (batteryCutoffRequested() && !powerUsbPresent()) {
      current = STATE_DEEP_SLEEP;
      enterDeepSleepFrom(
          "LIGHT_SLEEP -> DEEP_SLEEP (voltage cutoff)."); // no return
    }
    // Priority 2: switch flipped off - same reusable entry as from RUNNING.
    // enterBatteryWait() handles bleStop()/gnssEnd()/hold-off unconditionally,
    // which is correct regardless of whether GNSS is currently in backup
    // mode or already EN-cut.
    if (!switchOnDebounced(nowMs)) {
      enterBatteryWait();
      break;
    }
    // Priority 3: wake triggers - BLE connect or IMU motion. Either exits
    // instantly back to RUNNING; GNSS reacquisition happens asynchronously
    // in the background (validated sub-2s typical, see gnss-idle-cutoff-
    // enhancement memory).
    if (bleIsConnected()) {
      exitLightSleep("☀️ LIGHT_SLEEP -> RUNNING (BLE connect).");
      break;
    }
    if (imuWakeTriggered()) {
      exitLightSleep("☀️ LIGHT_SLEEP -> RUNNING (IMU wake).");
      break;
    }
    // Priority 4: GNSS backup -> EN-cut escalation. Ephemeris would be stale
    // by this point anyway, so there's no reacquisition-speed cost to
    // cutting harder. Only fires once (gnssInBackup guards it) and only
    // affects GNSS - IMU wake-detect and BLE stay exactly as they are.
    if (gnssInBackup &&
        (nowMs - lightSleepEnteredMs) >=
            (unsigned long)STATE_LIGHT_SLEEP_GNSS_CUTOFF_MIN * 60000UL) {
      LOG_PRINTLN("🔌 LIGHT_SLEEP: GNSS backup -> EN-cut (ephemeris stale).");
      gnssEnd();
      powerGnssRailOff();
      gnssInBackup = false;
    }
    // Priority 5: give up entirely -> DEEP_SLEEP. Ultimate backstop for a
    // truly-forgotten device; past this point BLE/shake wake no longer
    // work, same as any other DEEP_SLEEP entry.
    if ((nowMs - lightSleepEnteredMs) >=
        (unsigned long)STATE_LIGHT_SLEEP_TIMEOUT_MIN * 60000UL) {
      current = STATE_DEEP_SLEEP;
      enterDeepSleepFrom(
          "LIGHT_SLEEP -> DEEP_SLEEP (idle timeout)."); // no return
    }
    break;
  }
  }
}

SystemState stateCurrent() { return current; }
