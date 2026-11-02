// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include <assert.h>
#include <stdint.h>

#include "../code/Gnimu/src/core/ConfirmWindow.h"

using namespace gnimu;

constexpr uint32_t kWindow = 10000;

static void testSecondPressConfirms() {
  ConfirmWindow w(kWindow);
  assert(!w.press(1, 1000));
  assert(w.press(1, 5000));
}

// A confirmed command is spent: pressing again starts over.
static void testConfirmIsSpent() {
  ConfirmWindow w(kWindow);
  w.press(1, 0);
  assert(w.press(1, 1));
  assert(!w.press(1, 2));
}

static void testWindowExpires() {
  ConfirmWindow w(kWindow);
  w.press(1, 0);
  assert(!w.press(1, kWindow));  // at the deadline: too late, and it re-arms
  assert(w.press(1, kWindow + 1));
}

static void testJustInsideWindow() {
  ConfirmWindow w(kWindow);
  w.press(1, 0);
  assert(w.press(1, kWindow - 1));
}

static void testOtherCommandCancels() {
  ConfirmWindow w(kWindow);
  w.press(1, 0);
  w.cancel();
  assert(!w.press(1, 100));
}

// A different command pressed second arms itself instead.
static void testDifferentCommandDoesNotConfirm() {
  ConfirmWindow w(kWindow);
  w.press(1, 0);
  assert(!w.press(2, 100));
  assert(w.press(2, 200));
}

// Nothing armed: a lone press never confirms.
static void testFirstPressNeverConfirms() {
  ConfirmWindow w(kWindow);
  assert(!w.press(0, 0));
}

// The millisecond count wraps after 49 days.
static void testClockWrap() {
  ConfirmWindow w(kWindow);
  const uint32_t t = 0xFFFFFFFFu - 2000;
  w.press(1, t);
  assert(w.press(1, t + 5000));
  w.press(1, t);
  assert(!w.press(1, t + 20000));
}

int main() {
  testSecondPressConfirms();
  testConfirmIsSpent();
  testWindowExpires();
  testJustInsideWindow();
  testOtherCommandCancels();
  testDifferentCommandDoesNotConfirm();
  testFirstPressNeverConfirms();
  testClockWrap();
  return 0;
}
