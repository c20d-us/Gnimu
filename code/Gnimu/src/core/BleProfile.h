// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

namespace gnimu {

// What a protocol needs the BLE transport to expose: one notify (TX) and one
// write (RX) characteristic in a single service, plus the Device Information
// Service strings. The transport stays protocol-agnostic; RaceChrono's
// different service layout will extend this when it lands.
struct BleProfile {
  const char *model;  // advertised-name prefix and DIS model string
  const char *manufacturer;
  const char *firmwareRevision;
  const char *hardwareRevision;
  const char *serviceUuid;
  const char *txUuid;  // notify: device -> phone
  const char *rxUuid;  // write / write-without-response: phone -> device
};

// The RaceBox data protocol's BLE layout. The model and manufacturer are
// left empty: App fills them from Config.h, the one place the device's
// identity is set.
constexpr BleProfile kRaceBoxProfile = {
    "",
    "",
    "3.3",
    "1",
    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E",
    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E",
    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E",
};

}  // namespace gnimu
