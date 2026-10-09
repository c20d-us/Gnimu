#pragma once

#include <stdint.h>

namespace gnimu {

enum class PowerAction : uint8_t {
  None,
  StopForSwitch,       // switch off with USB holding the MCU up: stop and wait
  EnterChargeOnly,     // switch on with USB from BATTERY_WAIT: charge with everything else off
  ResetForSwitchOn,    // back to RUNNING, through a fresh boot
  ResetForUsbRemoval,  // CHARGE_ONLY lost USB: reboot, onto battery
  EnterSystemOff,      // low-voltage cutoff
};

struct PowerPolicyConfig {
  uint32_t switchDebounceMs;  // a new switch reading must hold this long to count, either way
};

// A two-state input that only takes a new value once readings of it have held
// for `holdMs`. A run of differing readings that ends sooner is a glitch, and
// counted.
class Debounced {
 public:
  void reset(bool v) {
    stable_ = v;
    pending_ = false;
  }

  // The stable value after this reading.
  bool update(bool reading, uint32_t nowMs, uint32_t holdMs) {
    if (reading == stable_) {
      if (pending_) glitches_++;
      pending_ = false;
      return stable_;
    }
    if (!pending_) {
      pending_ = true;
      sinceMs_ = nowMs;
    }
    if (nowMs - sinceMs_ >= holdMs) {
      stable_ = reading;
      pending_ = false;
    }
    return stable_;
  }

  uint32_t glitches() const { return glitches_; }
  void clearGlitches() { glitches_ = 0; }

 private:
  bool stable_ = true;
  bool pending_ = false;
  uint32_t sinceMs_ = 0;
  uint32_t glitches_ = 0;
};

// Power-state decisions only: the app applies each action in hardware-safe
// order.
//
// The switch is a high-impedance analog reading, so it's cleaned up twice
// before anything acts on it. Without USB, switching off removes the battery
// physically: an OFF reading then can only be noise, and reads as ON. And
// both edges are debounced by the same delay.
//
// On USB, the switch picks between running and charging. Switch OFF stops
// everything (BATTERY_WAIT: the battery is disconnected, the MCU is on USB).
// Switch ON from there charges with everything else off (CHARGE_ONLY), since
// a plugged-in device is on a charger - except straight after CHARGE_ONLY,
// where off/on is the way back to RUNNING. Unplugging USB in CHARGE_ONLY also
// reboots into RUNNING, onto the battery. On battery (the normal case, in the
// car), switch ON is the only state the MCU ever sees.
class PowerPolicy {
 public:
  explicit PowerPolicy(const PowerPolicyConfig& c) : c_(c) {}

  // Boot classification, on a single reading. `belowCutoff` is a fresh
  // battery reading under the cutoff voltage: on battery that means don't
  // even start.
  PowerAction begin(bool switchOn, bool usbPresent, bool belowCutoff) {
    const bool on = clean(switchOn, usbPresent);
    switch_.reset(on);
    running_ = on;
    chargingOnly_ = false;
    onEntersChargeOnly_ = true;  // plugged in with the switch off
    if (!on) return PowerAction::StopForSwitch;
    if (!usbPresent && belowCutoff) return PowerAction::EnterSystemOff;
    return PowerAction::None;
  }

  PowerAction update(uint32_t nowMs, bool switchOn, bool usbPresent, bool cutoffRequested) {
    const bool on = switch_.update(clean(switchOn, usbPresent), nowMs, c_.switchDebounceMs);
    if (waitingForSwitch()) {
      if (!on) return PowerAction::None;
      if (onEntersChargeOnly_ && usbPresent) {
        chargingOnly_ = true;
        return PowerAction::EnterChargeOnly;
      }
      return PowerAction::ResetForSwitchOn;
    }
    if (chargingOnly_ && !usbPresent) return PowerAction::ResetForUsbRemoval;
    if (running_ && cutoffRequested && !usbPresent) return PowerAction::EnterSystemOff;
    if (on) return PowerAction::None;
    // Off from RUNNING waits to charge; off from CHARGE_ONLY waits to run.
    onEntersChargeOnly_ = !chargingOnly_;
    running_ = false;
    chargingOnly_ = false;
    return PowerAction::StopForSwitch;
  }

  bool running() const { return running_; }
  bool chargingOnly() const { return chargingOnly_; }
  bool waitingForSwitch() const { return !running_ && !chargingOnly_; }

  // Switch readings rejected as noise: glitches shorter than the debounce,
  // and runs of OFF readings without USB. Shown in the D report, so a noisy
  // sense line in the car is visible.
  uint32_t switchNoise() const { return switch_.glitches() + offWithoutUsb_; }
  void clearSwitchNoise() {
    switch_.clearGlitches();
    offWithoutUsb_ = 0;
  }

 private:
  // The USB rule, counting each run of impossible OFF readings once.
  bool clean(bool switchOn, bool usbPresent) {
    const bool impossible = !switchOn && !usbPresent;
    if (impossible && !lastImpossible_) offWithoutUsb_++;
    lastImpossible_ = impossible;
    return switchOn || !usbPresent;
  }

  PowerPolicyConfig c_;
  Debounced switch_;
  bool running_ = false;
  bool chargingOnly_ = false;
  bool onEntersChargeOnly_ = true;  // in BATTERY_WAIT: what switch ON leads to
  bool lastImpossible_ = false;
  uint32_t offWithoutUsb_ = 0;
};

}  // namespace gnimu
