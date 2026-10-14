// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>

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
  const char *txUuid;  // notify: device -> client
  const char *rxUuid;  // write / write-without-response: client -> device
  // Text a client can read from the Gnimu config characteristic; nullptr
  // leaves that service out. Set by App from what V prints, not by the
  // protocol. The transport copies it, so it need not outlive begin().
  const char *configReport;
};

// The Gnimu service, separate from the protocol's: one read-only characteristic
// holding the device's configuration as text (docs/device-commands.md). It is
// not advertised, so a client finds it by discovering services once connected.
constexpr char kGnimuServiceUuid[] = "20D21CDE-F59E-41F1-9BCA-5BDE69876C8B";
constexpr char kGnimuConfigUuid[] = "689E54CD-60B0-473A-B049-A5E02A2716E0";
// The most a BLE attribute can hold; the report is cut off here, never beyond.
constexpr size_t kConfigReportMaxBytes = 512;

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
    nullptr,
};

}  // namespace gnimu
