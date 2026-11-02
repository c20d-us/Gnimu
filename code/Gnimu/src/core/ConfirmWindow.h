// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stdint.h>

namespace gnimu {

// Two presses for a command that erases something. The first press arms it,
// the same command pressed again inside the window confirms it, and any other
// command in between, or running out the window, drops it. Commands that only
// interrupt, like a restart, don't use it.
class ConfirmWindow {
 public:
  explicit ConfirmWindow(uint32_t windowMs) : windowMs_(windowMs) {}

  // True when `command` confirms an earlier press of itself. Otherwise it arms
  // `command`, replacing whatever was armed, and returns false.
  bool press(uint8_t command, uint32_t nowMs) {
    if (armed_ && command == command_ && static_cast<int32_t>(nowMs - deadlineMs_) < 0) {
      armed_ = false;
      return true;
    }
    armed_ = true;
    command_ = command;
    deadlineMs_ = nowMs + windowMs_;
    return false;
  }

  // Another command was entered.
  void cancel() { armed_ = false; }

 private:
  uint32_t windowMs_;
  uint32_t deadlineMs_ = 0;
  uint8_t command_ = 0;
  bool armed_ = false;
};

}  // namespace gnimu
