// Gnimu - RaceBox Mini-compatible GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.
#pragma once

// Arduino stand-in for the GNSS harness: the telemetry harness's (millis,
// delay, Serial, board pin names) plus the two serial-port classes g_gnss.cpp
// opens its UART with - HardwareSerial(2) on the ESP32, Serial1 on the nRF.
#include "../../telemetry/fakes/Arduino.h"

#define SERIAL_8N1 0x800001c

// What a port does here is carry bytes to and from the fake receiver, and
// remember what baud it was opened at - which is what decides whether the
// receiver answers at all (see fake_gnss.h).
class FakeSerialPort : public Stream {
public:
  void end();
  int available() override;
  int read() override;
  unsigned long baud() const { return baud_; }
  bool open() const { return open_; }

protected:
  void openAt(unsigned long baud);

private:
  unsigned long baud_ = 0;
  bool open_ = false;
};

// ESP32: constructed with a UART number, begun with an explicit pin pair, and
// the only one with a resizable RX ring (LAT-5).
class HardwareSerial : public FakeSerialPort {
public:
  explicit HardwareSerial(int uartNum);
  void begin(unsigned long baud, uint32_t config, int rxPin, int txPin);
  size_t setRxBufferSize(size_t bytes);
};

// nRF: fixed pins, plain begin(baud). The core exposes it as `Uart`.
class Uart : public FakeSerialPort {
public:
  void begin(unsigned long baud);
};
extern Uart Serial1;
