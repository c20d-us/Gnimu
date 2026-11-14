// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include <assert.h>
#include <stdio.h>

#include "../code/Gnimu/src/core/BleProfile.h"

using namespace gnimu;

namespace {

constexpr char kA[] = "AAAA";
constexpr char kA2[] = "AAAA";  // the same UUID at a different address
constexpr char kB[] = "BBBB";

constexpr BleCharacteristic notifyIn(const char* service) {
  return {service, "N", kBleNotify, BleText::None, nullptr};
}
constexpr BleCharacteristic writeIn(const char* service) {
  return {service, "W", kBleWrite, BleText::None, nullptr};
}
constexpr BleCharacteristic textIn(const char* service, uint8_t props = kBleRead) {
  return {service, "T", props, BleText::ConfigReport, nullptr};
}

constexpr BleProfile kGood = {"", "", "", "", kA, 4, {notifyIn(kA), writeIn(kA), textIn(kB), writeIn(kB)}};
static_assert(bleProfileValid(kGood), "two services, two writable, one text");
static_assert(bleNotifies(kGood, 0) && !bleNotifies(kGood, 1) && !bleNotifies(kGood, 4), "notify by index");
static_assert(bleWritableOrNone(kGood, 1) && bleWritableOrNone(kGood, kBleNoChar), "writable by index");
static_assert(!bleWritableOrNone(kGood, 0) && !bleWritableOrNone(kGood, 4), "not writable, or past the end");

// Services are compared by UUID text, not by pointer.
constexpr BleProfile kSameUuidText = {"", "", "", "", kA, 3, {notifyIn(kA), textIn(kB), notifyIn(kA2)}};
static_assert(!bleProfileValid(kSameUuidText), "service A split around service B");

constexpr BleProfile kEmpty = {"", "", "", "", kA, 0, {}};
static_assert(!bleProfileValid(kEmpty), "no characteristics");

constexpr BleProfile kTooMany = {"", "", "", "", kA, 7, {}};
static_assert(!bleProfileValid(kTooMany), "more than the list holds");

constexpr BleProfile kThreeWritable = {"", "", "", "", kA, 3, {writeIn(kA), writeIn(kA), writeIn(kA)}};
static_assert(!bleProfileValid(kThreeWritable), "more writable than there are queues");

constexpr BleProfile kNotifyingText = {"", "", "", "", kA, 1, {textIn(kA, kBleRead | kBleNotify)}};
static_assert(!bleProfileValid(kNotifyingText), "text that also notifies");

constexpr BleProfile kUnlistedAdvert = {"", "", "", "", kB, 1, {notifyIn(kA)}};
static_assert(!bleProfileValid(kUnlistedAdvert), "advertising a service that isn't built");

}  // namespace

int main() {
  // The same checks at run time, so a failure names its line.
  assert(bleProfileValid(kGood));
  assert(!bleProfileValid(kSameUuidText));
  assert(!bleProfileValid(kThreeWritable));
  puts("bleprofile: all tests passed");
  return 0;
}
