// Controls the test scenario drives: the clock, the GNSS epoch, the physical
// motion the fake sensors report, and injected failures.
// Shared by every fake, so the "before" and "after" builds see identical inputs.
#pragma once
#include <stdint.h>

void fakeAdvanceMs(unsigned long ms);
unsigned long fakeNowMs();

// GNSS: present=false makes gnssLatestPvt() return nullptr.
void fakeSetPvt(bool present, uint8_t fixType, bool fixOK, int32_t gSpeedMmS,
                uint32_t iTOW);

// Physical motion in g and deg/s; each fake sensor converts to its own API.
void fakeSetMotion(const float accelG[3], const float gyroDps[3]);

void fakeFailNextReads(int n);     // next n sensor reads report failure
void fakeSetBeginResult(bool ok);  // what the sensor's begin() returns
int fakeReadCount();               // sensor reads attempted so far

void fakeSetConfigTakes(bool takes);   // false: configuration writes don't land
void fakeSetSensorResetState(bool reset); // MPU-6050: chip in its power-on state
void fakeSetBduWriteLands(bool lands);    // LSM6DS3: the driver's own BDU write
void fakeSetLpf1WriteLands(bool lands);   // LSM6DS3: the driver's CTRL1_XL write
