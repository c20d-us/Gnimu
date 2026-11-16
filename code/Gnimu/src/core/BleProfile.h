// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>

namespace gnimu {

// What a protocol needs the BLE transport to expose: its characteristics, each
// in a named service, plus the Device Information Service strings. The
// transport builds whatever the list says and stays protocol-agnostic; a
// characteristic is addressed by its position in the list.

// Characteristic properties, combined as a bitmask. Write accepts both write
// and write-without-response.
enum : uint8_t { kBleNotify = 1, kBleRead = 2, kBleWrite = 4 };

// The text a read-only characteristic holds. App fills each one from its role
// when BLE starts, so it never needs to know which protocol declared it.
enum class BleText : uint8_t { None, ConfigReport, Schema, DeviceInfo };

struct BleCharacteristic {
  const char* serviceUuid;
  const char* uuid;
  uint8_t props;
  BleText text;
  const char* value;  // the role's text, set by App; the transport copies it
};

constexpr size_t kBleMaxCharacteristics = 6;
// Each writable characteristic gets its own inbound queue.
constexpr size_t kBleMaxWritable = 2;
// The most a BLE attribute can hold; a text value is cut off here, never beyond.
constexpr size_t kBleTextMaxBytes = 512;
// A protocol's characteristic index when it has no such characteristic.
constexpr uint8_t kBleNoChar = 0xFF;

struct BleProfile {
  const char* model;  // advertised-name prefix and DIS model string
  const char* manufacturer;
  // nullptr: App fills in the device's own build version and hardware.
  const char* firmwareRevision;
  const char* hardwareRevision;
  const char* advertisedServiceUuid;
  uint8_t charCount;
  // A service's characteristics are listed together, in the order they're built.
  BleCharacteristic chars[kBleMaxCharacteristics];
};

// Compile-time checks on a profile. Recursive rather than loops: the nRF core
// builds as C++11, where a constexpr function is a single return.
namespace ble_check {

constexpr bool sameUuid(const char* a, const char* b) {
  return *a == *b && (*a == '\0' || sameUuid(a + 1, b + 1));
}

constexpr size_t writable(const BleProfile& p, size_t i = 0) {
  return i >= p.charCount ? 0 : ((p.chars[i].props & kBleWrite) ? 1 : 0) + writable(p, i + 1);
}

// Characteristic i's service appears among chars[0, end).
constexpr bool serviceSeen(const BleProfile& p, size_t i, size_t end, size_t k = 0) {
  return k < end &&
         (sameUuid(p.chars[k].serviceUuid, p.chars[i].serviceUuid) || serviceSeen(p, i, end, k + 1));
}

// Each characteristic either continues the previous one's service or starts
// a service not seen before.
constexpr bool servicesTogether(const BleProfile& p, size_t i = 1) {
  return i >= p.charCount ||
         ((sameUuid(p.chars[i].serviceUuid, p.chars[i - 1].serviceUuid) || !serviceSeen(p, i, i - 1)) &&
          servicesTogether(p, i + 1));
}

// A text characteristic is read and nothing else.
constexpr bool textsReadOnly(const BleProfile& p, size_t i = 0) {
  return i >= p.charCount ||
         ((p.chars[i].text == BleText::None || p.chars[i].props == kBleRead) && textsReadOnly(p, i + 1));
}

constexpr bool advertisedServiceListed(const BleProfile& p, size_t i = 0) {
  return i < p.charCount &&
         (sameUuid(p.chars[i].serviceUuid, p.advertisedServiceUuid) || advertisedServiceListed(p, i + 1));
}

}  // namespace ble_check

constexpr bool bleProfileValid(const BleProfile& p) {
  return p.charCount >= 1 && p.charCount <= kBleMaxCharacteristics &&
         ble_check::writable(p) <= kBleMaxWritable && ble_check::servicesTogether(p) &&
         ble_check::textsReadOnly(p) && ble_check::advertisedServiceListed(p);
}

constexpr bool bleNotifies(const BleProfile& p, uint8_t i) {
  return i < p.charCount && (p.chars[i].props & kBleNotify) != 0;
}

// kBleNoChar counts: a protocol needn't accept writes.
constexpr bool bleWritableOrNone(const BleProfile& p, uint8_t i) {
  return i == kBleNoChar || (i < p.charCount && (p.chars[i].props & kBleWrite) != 0);
}

}  // namespace gnimu
