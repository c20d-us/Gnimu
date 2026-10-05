#pragma once

#include <stdint.h>

namespace gnimu {

enum class PowerAction : uint8_t {
  None,
  StopForSwitch,       // switch off with USB holding the MCU up: stop and wait
  EnterChargeOnly,     // the quick off/on gesture: charge with everything else off
  ResetForSwitchOn,    // back to RUNNING, through a fresh boot
  ResetForUsbRemoval,  // CHARGE_ONLY lost USB: reboot, onto battery
  EnterSystemOff,      // low-voltage cutoff
};

struct PowerPolicyConfig {
  uint32_t switchOffDebounceMs;  // an OFF must last this long to count
  uint32_t chargeOnlyWindowMs;   // back ON within this of first going OFF = gesture
};

// NG's PowerState, decisions only: the app applies each action in hardware-safe
// order. Without USB, switching off removes the battery physically, so every
// switch-off transition here happens with USB present.
//
// The CHARGE_ONLY gesture: running with USB, switch OFF for at least the
// debounce, then back ON within the window (counted from when it first went
// OFF). A shorter flick is ignored; a longer off reboots into RUNNING. Any
// off/on from CHARGE_ONLY, or unplugging USB there, reboots into RUNNING.
class PowerPolicy {
 public:
  explicit PowerPolicy(const PowerPolicyConfig& c) : c_(c) {}

  // Boot classification. `belowCutoff` is a fresh battery reading under the
  // cutoff voltage: on battery that means don't even start.
  PowerAction begin(bool switchOn, bool usbPresent, bool belowCutoff) {
    running_ = switchOn;
    chargingOnly_ = false;
    quickReturnEntersChargeOnly_ = false;
    switchOffPending_ = false;
    if (!switchOn) return PowerAction::StopForSwitch;
    if (!usbPresent && belowCutoff) return PowerAction::EnterSystemOff;
    return PowerAction::None;
  }

  PowerAction update(uint32_t nowMs, bool switchOn, bool usbPresent, bool cutoffRequested) {
    if (waitingForSwitch()) {
      if (!switchOn) return PowerAction::None;
      if (quickReturnEntersChargeOnly_ && usbPresent &&
          nowMs - switchOffSinceMs_ < c_.chargeOnlyWindowMs) {
        chargingOnly_ = true;
        quickReturnEntersChargeOnly_ = false;
        return PowerAction::EnterChargeOnly;
      }
      return PowerAction::ResetForSwitchOn;
    }
    if (chargingOnly_ && !usbPresent) return PowerAction::ResetForUsbRemoval;
    if (running_ && cutoffRequested && !usbPresent) return PowerAction::EnterSystemOff;
    if (switchOn) {
      switchOffPending_ = false;
      return PowerAction::None;
    }
    if (!switchOffPending_) {
      switchOffPending_ = true;
      switchOffSinceMs_ = nowMs;
    }
    if (nowMs - switchOffSinceMs_ >= c_.switchOffDebounceMs) {
      // Only a stop from RUNNING with USB arms the gesture; a stop from
      // CHARGE_ONLY can't lead back into it.
      quickReturnEntersChargeOnly_ = running_ && usbPresent;
      running_ = false;
      chargingOnly_ = false;
      switchOffPending_ = false;
      return PowerAction::StopForSwitch;
    }
    return PowerAction::None;
  }

  bool running() const { return running_; }
  bool chargingOnly() const { return chargingOnly_; }
  bool waitingForSwitch() const { return !running_ && !chargingOnly_; }

 private:
  PowerPolicyConfig c_;
  bool running_ = false;
  bool chargingOnly_ = false;
  bool quickReturnEntersChargeOnly_ = false;
  bool switchOffPending_ = false;
  uint32_t switchOffSinceMs_ = 0;
};

}  // namespace gnimu
