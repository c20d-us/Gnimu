// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// Same four-board guard as GnssUart.cpp: the four Seeed XIAO nRF52840 boards
// share the RGB LED on 11 (red), 13 (green), 12 (blue), confirmed in each
// variant.h; the trackers built on the same chip don't.
#if defined(ARDUINO_Seeed_XIAO_nRF52840) || defined(ARDUINO_Seeed_XIAO_nRF52840_Sense) || \
    defined(ARDUINO_Seeed_XIAO_nRF52840_Plus) || defined(ARDUINO_Seeed_XIAO_nRF52840_Sense_Plus)

#include <Arduino.h>

#include "../StatusLedPort.h"

namespace gnimu {

namespace {
// Active-low: HIGH is off. The variant's LED_STATE_ON (1) says otherwise, but
// these LEDs light when driven LOW.
void drive(int pin, bool on) { digitalWrite(pin, on ? LOW : HIGH); }
}  // namespace

void StatusLedPort::begin() {
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  write(false, LedColor::Blue);
}

// Amber is red and green together.
void StatusLedPort::write(bool on, LedColor color) {
  drive(LED_RED, on && (color == LedColor::Red || color == LedColor::Amber));
  drive(LED_GREEN, on && (color == LedColor::Green || color == LedColor::Amber));
  drive(LED_BLUE, on && color == LedColor::Blue);
}

}  // namespace gnimu

#endif
