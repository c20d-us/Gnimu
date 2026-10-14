// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

#include "GnssFeed.h"
#include "PowerPolicy.h"
#include "StatusLed.h"

namespace gnimu {

// Starting: peripheral bring-up at boot, before the scheduler runs. Running:
// normal operation. Fault: fatal (no receiver, its settings not applied, no
// data, no BLE) - everything else stops, the console stays up. BatteryWait:
// switch off while USB holds the MCU up. ChargeOnly: charging with everything
// else off (switch on with USB). A reset or System OFF leaves the run
// altogether rather than being a state.
enum class RunState : uint8_t { Starting, Running, Fault, BatteryWait, ChargeOnly };
enum class FaultReason : uint8_t { None, GnssMissing, GnssConfigFailed, GnssNoData, BleFailed };
enum class RestartReason : uint8_t { SwitchOn, UsbRemoved, GnssSilent, Requested };
// One past the last RestartReason; update it when adding one. Bounds the
// reason kept across a restart (core/ResetReason.h).
constexpr uint8_t kRestartReasonCount = static_cast<uint8_t>(RestartReason::Requested) + 1;

// What an event leads to. App carries it out: entering a state runs that
// state's fixed entry effects; Restart and SystemOff never return.
struct Transition {
  enum class Kind : uint8_t { Stay, Enter, Restart, SystemOff };
  Kind kind;
  RunState to;         // Enter only
  RestartReason why;   // Restart only
  bool requested;      // SystemOff only: asked for, rather than a battery cutoff

  static Transition stay() { return {Kind::Stay, RunState::Starting, RestartReason::SwitchOn, false}; }
  static Transition enter(RunState s) { return {Kind::Enter, s, RestartReason::SwitchOn, false}; }
  static Transition restart(RestartReason r) { return {Kind::Restart, RunState::Starting, r, false}; }
  static Transition systemOff(bool requested = false) {
    return {Kind::SystemOff, RunState::Starting, RestartReason::SwitchOn, requested};
  }
};

struct RunConfig {
  bool hasBattery;       // battery boards run the power policy and the battery/power jobs
  bool protocolUsesImu;  // otherwise the IMU never starts
  PowerPolicyConfig power;
};

// Run-state decisions only, no hardware: App applies each Transition. Owns
// PowerPolicy, so every transition comes through one place. An event that
// doesn't apply in the current state returns Stay.
class RunStateMachine {
 public:
  explicit RunStateMachine(const RunConfig& c) : c_(c), policy_(c.power) {}

  // Boot classification. Without a battery the inputs are ignored.
  Transition boot(bool switchOn, bool usbPresent, bool belowCutoff) {
    if (!c_.hasBattery) return enter(RunState::Starting);
    switch (policy_.begin(switchOn, usbPresent, belowCutoff)) {
      case PowerAction::StopForSwitch:
        return enter(RunState::BatteryWait);
      case PowerAction::EnterSystemOff:
        return Transition::systemOff();
      default:
        return enter(RunState::Starting);
    }
  }

  // The result of peripheral bring-up: `failure` is the first thing that
  // failed, or None if everything started. The only way into Running.
  Transition started(FaultReason failure) {
    if (state_ != RunState::Starting) return Transition::stay();
    if (failure == FaultReason::None) return enter(RunState::Running);
    fault_ = failure;
    return enter(RunState::Fault);
  }

  // Epoch flow while running. Stopped mid-run restarts: a fresh boot re-runs
  // detection and, on the nRF, power-cycles the GNSS rail. Never started is a
  // fault: restarting can't help a receiver that answers but sends nothing,
  // and a receiver that stays silent after a restart lands here, so a failure
  // costs at most one restart.
  Transition gnssLiveness(GnssLiveness l) {
    if (state_ != RunState::Running) return Transition::stay();
    if (l == GnssLiveness::Stopped) return Transition::restart(RestartReason::GnssSilent);
    if (l == GnssLiveness::NeverStarted) {
      fault_ = FaultReason::GnssNoData;
      return enter(RunState::Fault);
    }
    return Transition::stay();
  }

  // The power job's inputs. The policy is updated on every call, whatever it
  // returns, so its debounce keeps running.
  Transition power(uint32_t nowMs, bool switchOn, bool usbPresent, bool cutoffRequested) {
    if (!c_.hasBattery) return Transition::stay();
    const PowerAction a = policy_.update(nowMs, switchOn, usbPresent, cutoffRequested);
    switch (state_) {
      case RunState::Running:
      case RunState::Fault:
        if (a == PowerAction::StopForSwitch) return enter(RunState::BatteryWait);
        if (a == PowerAction::EnterSystemOff) return Transition::systemOff();
        break;
      case RunState::BatteryWait:
        if (a == PowerAction::EnterChargeOnly) return enter(RunState::ChargeOnly);
        if (a == PowerAction::ResetForSwitchOn) return Transition::restart(RestartReason::SwitchOn);
        break;
      case RunState::ChargeOnly:
        if (a == PowerAction::StopForSwitch) return enter(RunState::BatteryWait);
        if (a == PowerAction::ResetForSwitchOn) return Transition::restart(RestartReason::SwitchOn);
        if (a == PowerAction::ResetForUsbRemoval) return Transition::restart(RestartReason::UsbRemoved);
        break;
      case RunState::Starting:
        break;
    }
    return Transition::stay();
  }

  // A restart or System OFF someone asked for, from the console or BLE. Both
  // are taken in every state.
  Transition restartRequested() const { return Transition::restart(RestartReason::Requested); }
  // Only for boards with a battery and switch: elsewhere nothing could wake
  // it, so it stays.
  Transition systemOffRequested() const {
    return c_.hasBattery ? Transition::systemOff(true) : Transition::stay();
  }

  RunState state() const { return state_; }
  FaultReason fault() const { return fault_; }
  // Switch readings the power policy rejected as noise (D report); C clears.
  uint32_t switchNoise() const { return policy_.switchNoise(); }
  void clearSwitchNoise() { policy_.clearSwitchNoise(); }

 private:
  Transition enter(RunState s) {
    state_ = s;
    return Transition::enter(s);
  }

  RunConfig c_;
  PowerPolicy policy_;
  RunState state_ = RunState::Starting;
  FaultReason fault_ = FaultReason::None;
};

// Which jobs run in each state. Starting is boot-only, before the scheduler
// starts, so its set is never run.
struct JobSet {
  bool gnss, imu, battery, power, ble, led, console;
};

inline JobSet jobsFor(RunState s, const RunConfig& c) {
  const bool running = s == RunState::Running;
  JobSet j;
  j.gnss = running;
  j.imu = running && c.protocolUsesImu;
  j.battery = c.hasBattery;  // every state: the cutoff keeps watching
  j.power = c.hasBattery;
  j.ble = running;
  j.led = true;
  j.console = true;
  return j;
}

// What the LED needs besides the state, already gated on a valid battery
// reading (all false without one).
struct LedInputs {
  bool batteryCritical;
  bool batteryWarning;
  bool chargeFull;
  bool bleConnected;
};

inline LedState ledStateFor(RunState s, const LedInputs& in) {
  switch (s) {
    case RunState::Running:
      // A battery problem outranks the BLE status: it matters whatever the
      // client is doing. Charging isn't shown here - only in CHARGE_ONLY.
      if (in.batteryCritical) return LedState::BatteryCritical;
      if (in.batteryWarning) return LedState::BatteryWarning;
      return in.bleConnected ? LedState::Connected : LedState::Advertising;
    case RunState::Fault:
      return LedState::Fault;
    case RunState::BatteryWait:
      return LedState::BatteryWait;
    case RunState::ChargeOnly:
      return in.chargeFull ? LedState::Charged : LedState::Charging;
    case RunState::Starting:
      return LedState::Off;
  }
  return LedState::Off;
}

// The final boot line's state name.
inline const char* stateName(RunState s) {
  switch (s) {
    case RunState::Starting:
      return "Starting";
    case RunState::Running:
      return "Running";
    case RunState::Fault:
      return "Fault - console only";
    case RunState::BatteryWait:
      return "BATTERY_WAIT - waiting for switch on";
    case RunState::ChargeOnly:
      return "CHARGE_ONLY - charging, everything else off";
  }
  return "?";
}

inline const char* faultMessage(FaultReason r) {
  switch (r) {
    case FaultReason::GnssMissing:
      return "GNSS not detected at any standard baud rate. Check wiring. Nothing else will start.";
    case FaultReason::GnssConfigFailed:
      return "GNSS settings could not be applied. Power-cycle to retry. Everything else stopped.";
    case FaultReason::GnssNoData:
      return "GNSS detected but sending no data. Check its power supply. Everything else stopped.";
    case FaultReason::BleFailed:
      return "BLE failed to start. Power-cycle to retry. Everything else stopped.";
    case FaultReason::None:
      break;
  }
  return "Fault";
}

// Logged on entering a waiting state; nullptr for the others.
inline const char* entryMessage(RunState s) {
  switch (s) {
    case RunState::BatteryWait:
      return "Power: switch off on USB - BATTERY_WAIT, waiting for switch on";
    case RunState::ChargeOnly:
      return "Power: switch on with USB - CHARGE_ONLY, charging with everything else off";
    case RunState::Starting:
    case RunState::Running:
    case RunState::Fault:
      break;
  }
  return nullptr;
}

// The first half of the restart log line ("<this> - restarting into RUNNING").
inline const char* restartReasonText(RestartReason r) {
  switch (r) {
    case RestartReason::SwitchOn:
      return "Power: switch on";
    case RestartReason::UsbRemoved:
      return "Power: USB removed";
    case RestartReason::GnssSilent:
      return "GNSS: epochs stopped";
    case RestartReason::Requested:
      return "Restart requested";
  }
  return "?";
}

}  // namespace gnimu
