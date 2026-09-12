#pragma once
#include <stddef.h>
#include <stdint.h>

// Wire for the MPU-6050 driver's own burst read, mirroring the ESP32 core
// (3.3.11) where it matters: endTransmission(false) only QUEUES a repeated
// start and returns 0 unconditionally, and the real write-then-read happens in
// requestFrom(), which returns the byte count (0 on failure). A driver that
// checks endTransmission() instead of the count therefore misses every
// injected failure here, exactly as it would on the device.
//
// The nRF builds use only setClock() on Wire1; the LSM6DS3 fake reads through
// its own library stand-in.
struct FakeWire {
  void setClock(unsigned long) {}
  void beginTransmission(uint8_t addr);
  size_t write(uint8_t b);
  uint8_t endTransmission(bool sendStop = true);
  size_t requestFrom(uint8_t addr, size_t len, bool sendStop = true);
  int available();
  int read();
};
extern FakeWire Wire, Wire1;
