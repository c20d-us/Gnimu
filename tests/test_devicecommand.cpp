// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../code/Gnimu/src/core/DeviceCommand.h"

using namespace gnimu;

namespace {

const uint8_t kRetrim[] = {0xB5, 0x62, 0x47, 0x01, 0x00, 0x00, 0x48, 0x1F};
const uint8_t kRestart[] = {0xB5, 0x62, 0x47, 0x02, 0x00, 0x00, 0x49, 0x22};
const uint8_t kSystemOff[] = {0xB5, 0x62, 0x47, 0x03, 0x00, 0x00, 0x4A, 0x25};
// The RaceBox app's own frames, as captured from it.
const uint8_t kRaceBoxPoll[] = {0xB5, 0x62, 0xFF, 0x22, 0x00, 0x00, 0x21, 0x62};
const uint8_t kRaceBoxWrite[] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00,
                                 0x00, 0x21, 0x00, 0x11, 0x20, 0x04, 0xF0, 0x4D};

// Feeds every byte; counts and records the commands found.
struct Run {
  int found = 0;
  DeviceCommand last = DeviceCommand::Retrim;
};
Run feed(CommandFrameParser& p, const uint8_t* b, size_t n, Run r = Run()) {
  for (size_t i = 0; i < n; i++) {
    DeviceCommand c;
    if (p.feed(b[i], c)) {
      r.found++;
      r.last = c;
    }
  }
  return r;
}

}  // namespace

static void testEachCommand() {
  CommandFrameParser p;
  Run r = feed(p, kRetrim, sizeof(kRetrim));
  assert(r.found == 1 && r.last == DeviceCommand::Retrim);
  r = feed(p, kRestart, sizeof(kRestart));
  assert(r.found == 1 && r.last == DeviceCommand::Restart);
  r = feed(p, kSystemOff, sizeof(kSystemOff));
  assert(r.found == 1 && r.last == DeviceCommand::SystemOff);
}

// A write can end anywhere in the frame, and the stream can begin mid-frame.
static void testSplitAndLeadingNoise() {
  CommandFrameParser p;
  Run r = feed(p, kRestart, 3);
  assert(r.found == 0);
  r = feed(p, kRestart + 3, sizeof(kRestart) - 3);
  assert(r.found == 1 && r.last == DeviceCommand::Restart);

  const uint8_t noise[] = {0x00, 0xB5, 0xB5, 0x13, 0x62};
  r = feed(p, noise, sizeof(noise));
  r = feed(p, kRetrim, sizeof(kRetrim), r);
  assert(r.found == 1 && r.last == DeviceCommand::Retrim);
}

// The RaceBox app's traffic is skipped, and a command after it still lands.
static void testForeignFramesPass() {
  CommandFrameParser p;
  Run r = feed(p, kRaceBoxPoll, sizeof(kRaceBoxPoll));
  r = feed(p, kRaceBoxWrite, sizeof(kRaceBoxWrite), r);
  assert(r.found == 0);
  r = feed(p, kSystemOff, sizeof(kSystemOff), r);
  assert(r.found == 1 && r.last == DeviceCommand::SystemOff);
}

static void testRejects() {
  CommandFrameParser p;
  uint8_t bad[sizeof(kRestart)];

  for (size_t i = 2; i < sizeof(bad); i++) {  // any single wrong byte after the sync
    memcpy(bad, kRestart, sizeof(bad));
    bad[i] ^= 0x01;
    assert(feed(p, bad, sizeof(bad)).found == 0);
  }

  // A payload on one of our ids, with a correct checksum, isn't a command.
  const uint8_t payload[] = {0xB5, 0x62, 0x47, 0x02, 0x01, 0x00, 0x00, 0x4A, 0x6E};
  assert(feed(p, payload, sizeof(payload)).found == 0);

  // Unknown id, with a correct checksum.
  const uint8_t id4[] = {0xB5, 0x62, 0x47, 0x04, 0x00, 0x00, 0x4B, 0x28};
  assert(feed(p, id4, sizeof(id4)).found == 0);

  // Still matching afterwards.
  assert(feed(p, kRetrim, sizeof(kRetrim)).found == 1);
}

// A frame cut off mid-way, then a whole one: only the whole one counts.
static void testTruncatedThenWhole() {
  CommandFrameParser p;
  Run r = feed(p, kSystemOff, 5);
  r = feed(p, kRestart, sizeof(kRestart), r);
  assert(r.found == 1 && r.last == DeviceCommand::Restart);
}

int main() {
  testEachCommand();
  testSplitAndLeadingNoise();
  testForeignFramesPass();
  testRejects();
  testTruncatedThenWhole();
  puts("devicecommand: all tests passed");
  return 0;
}
