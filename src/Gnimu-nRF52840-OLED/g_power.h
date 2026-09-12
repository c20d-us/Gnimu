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

#pragma once
#include <Arduino.h>

// ============================================================================
// Power module - rail mechanisms and rail/switch sensing.
//
// Owns the primitives the state machine (g_state) actuates:
//   - USB/VBUS presence
//   - Slide-switch position via the POWER_SWITCH_SENSE_PIN divider
//     (load-independent presence; the pin differs per board - see config.h)
//   - GNSS EN + TX drive (rail on/off + phantom back-feed prevention)
//   - IMU power pin drive
//   - LED pin drive when g_led isn't yet initialized (boot / halt)
//   - powerHoldPeripheralsOff() - unconditional held-off entry action for
//     boot, BATTERY_WAIT, and DEEP_SLEEP; drives peripheral controls directly
//     so it can run before any other module has begun.
//   - powerEnterDeepSleep() - final System OFF sequence, does not return.
//
// Contains no policy - the state machine decides when to call these.
// ============================================================================

// Configure the ADC (resolution + reference + TACQ) shared by battery voltage
// and switch-sense reads, and the switch-sense pin as INPUT. Call after
// Serial.begin() but before anything reads powerSwitchOn() or the battery
// sampler.
void powerBegin();

// True when VBUS is present.
bool powerUsbPresent();

// True when the slide switch is ON (battery is physically in the circuit).
// Reads the POWER_SWITCH_SENSE_PIN divider tap; below
// POWER_SWITCH_OFF_THRESHOLD_MV = ON.
//
// Throttled to POWER_SWITCH_POLL_INTERVAL_MS - reads in between return the
// cache, so the per-loop cost is a compare. Safe to call every loop().
//
// The refresh is ONE unaveraged analogRead(), deliberately. An earlier version
// of this comment promised a dummy read plus an averaged burst to guard against
// SAADC channel-switch "ghost" readings from the shared VBAT pin; no such code
// ever existed here, and none is needed. Three things carry it instead:
//
//   1. MARGIN, which does nearly all the work. The tap reads ~0mV with the
//      switch ON and >=1675mV OFF (510k/510k at the 3.35V discharge floor),
//      against an 800mV threshold - 800mV and 875mV of headroom. A ghost would
//      have to move the reading by ~28% of the 3000mV reference to flip the
//      decision. config.h static_asserts this margin so the argument cannot
//      rot the way the old comment did.
//   2. TACQ, which is the REAL high-Z mitigation and is genuinely implemented:
//      powerBegin() calls analogSampleTime(SAADC_TACQ_US) at 40us for the
//      divider's ~255k source impedance. Settling is handled by acquisition
//      time, not by discarding reads.
//   3. STATE_SWITCH_OFF_DEBOUNCE_MS, the second line, covering any single bad
//      read before it can route the device to BATTERY_WAIT.
//
// Worth knowing about (2) and (3): BATTERY_POLL_INTERVAL_MS and
// POWER_SWITCH_POLL_INTERVAL_MS are exactly commensurate (250 / 50), both off
// millis(), so the two channel reads CAN phase-lock. A systematic offset would
// not be averaged away by a debounce that only covers uncorrelated bad reads.
// That is the strongest form of the ghost argument, and margin still answers
// it - but it is why the margin, not the debounce, is the load-bearing part.
bool powerSwitchOn();

// Drive every peripheral control pin to its safe held-off state without
// touching any peripheral library. Safe before Serial.begin() and before any
// other module has initialized. Idempotent. Used as: boot prologue
// (unconditional), BATTERY_WAIT entry, DEEP_SLEEP entry.
void powerHoldPeripheralsOff();

// Release the GNSS EN pin so the TPS63020's own pullup enables the rail (the
// pre-cutoff-fix "normal" state). Undoes the EN-low drive done by
// powerHoldPeripheralsOff(). Call before gnssBegin() when entering RUNNING.
// imuBegin() and Serial1.begin() reclaim their own pins.
void powerGnssRailOn();

// Enter System OFF (single-digit uA deep sleep): print, hold peripherals off,
// then sd_power_system_off(). Does not return; recovery is a hard reset
// triggered by USB plug-in or a slide-switch off->on cycle.
void powerEnterDeepSleep();
