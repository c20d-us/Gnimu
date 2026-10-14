// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace gnimu {

// What a client can ask of the device, from the console (T, R, O) or over BLE.
// Both routes end in the same handler, so each means one thing.
enum class DeviceCommand : uint8_t { Retrim, Restart, SystemOff };

// Finds device commands in the bytes a BLE client writes. The RaceBox app
// writes UBX frames to the same characteristic, so each command is a complete
// UBX frame of our own class (0x47) with no payload; the three are fixed
// bytes, listed in docs/device-commands.md. Matching the last eight bytes
// against them needs no state across frames: a write can end anywhere, other
// traffic passes by, and a checksum error simply never matches.
class CommandFrameParser {
 public:
  static constexpr size_t kFrameBytes = 8;

  // One byte in; true when it completed a command frame, set in `out`.
  bool feed(uint8_t b, DeviceCommand& out) {
    memmove(last_, last_ + 1, kFrameBytes - 1);
    last_[kFrameBytes - 1] = b;
    for (uint8_t i = 0; i < 3; i++) {
      if (memcmp(last_, frame(i), kFrameBytes) == 0) {
        out = static_cast<DeviceCommand>(i);
        return true;
      }
    }
    return false;
  }

 private:
  // Indexed by DeviceCommand: B5 62, class 0x47, id (command + 1), length 0,
  // then the Fletcher checksum over class through length.
  static const uint8_t* frame(uint8_t command) {
    static const uint8_t kFrames[3][kFrameBytes] = {
        {0xB5, 0x62, 0x47, 0x01, 0x00, 0x00, 0x48, 0x1F},
        {0xB5, 0x62, 0x47, 0x02, 0x00, 0x00, 0x49, 0x22},
        {0xB5, 0x62, 0x47, 0x03, 0x00, 0x00, 0x4A, 0x25},
    };
    return kFrames[command];
  }

  uint8_t last_[kFrameBytes] = {};
};

}  // namespace gnimu
