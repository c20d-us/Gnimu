#include "fake_control.h"
#include "Arduino.h"
#include <math.h>

static unsigned long g_ms = 0;
void fakeAdvanceMs(unsigned long ms) { g_ms += ms; }
unsigned long fakeNowMs() { return g_ms; }
unsigned long millis() { return g_ms; }
void delay(unsigned long ms) { g_ms += ms; }
void pinMode(int, int) {}
void digitalWrite(int, int) {}
int digitalRead(int) { return LOW; }

float g_motionG[3] = {0, 0, 1}, g_motionDps[3] = {0, 0, 0};
void fakeSetMotion(const float a[3], const float g[3]) {
  for (int i = 0; i < 3; i++) { g_motionG[i] = a[i]; g_motionDps[i] = g[i]; }
}
int g_failNext = 0;
void fakeFailNextReads(int n) { g_failNext = n; }
bool g_beginOk = true;
void fakeSetBeginResult(bool ok) { g_beginOk = ok; }
int g_reads = 0;
int fakeReadCount() { return g_reads; }

#include "u-blox_structs.h"
static UBX_NAV_PVT_data_t g_pvt;
static bool g_pvtPresent = false;
void fakeSetPvt(bool present, uint8_t fixType, bool fixOK, int32_t gSpeedMmS,
                uint32_t iTOW) {
  g_pvtPresent = present;
  g_pvt.fixType = fixType;
  g_pvt.flags.bits.gnssFixOK = fixOK;
  g_pvt.gSpeed = gSpeedMmS;
  g_pvt.iTOW = iTOW;
}
const UBX_NAV_PVT_data_t *gnssLatestPvt() { return g_pvtPresent ? &g_pvt : nullptr; }

#include "Wire.h"
#include <string.h>
FakeWire Wire, Wire1;

// Scenario controls shared by both sensor models.
bool g_configTakes = true;
static bool g_mpuResetState = false;
static bool g_bduWriteLands = true;
static bool g_lpf1WriteLands = true;
void fakeSetConfigTakes(bool takes) { g_configTakes = takes; }
void fakeSetBduWriteLands(bool lands) { g_bduWriteLands = lands; }
void fakeSetLpf1WriteLands(bool lands) { g_lpf1WriteLands = lands; }
void fakeSetSensorResetState(bool reset) { g_mpuResetState = reset; }

// The I2C bus both drivers read through. One transaction shape each: point the
// register pointer with a write, then read. An injected failure becomes a SHORT
// READ - the case the Seeed library mishandles (it ignores the byte count) and
// the one a driver must reject rather than parse.
static uint8_t g_txAddr = 0, g_regPtr = 0;
static uint8_t g_rx[16];
static size_t g_rxLen = 0, g_rxPos = 0;

static int16_t counts(float v, float perUnit) {
  const long c = lroundf(v * perUnit);
  return (int16_t)(c > 32767 ? 32767 : c < -32768 ? -32768 : c);
}

void FakeWire::beginTransmission(uint8_t addr) { g_txAddr = addr; }
size_t FakeWire::write(uint8_t b) {
  g_regPtr = b;
  return 1;
}
uint8_t FakeWire::endTransmission(bool sendStop) {
  if (!sendStop) {
    return 0; // the ESP32 core only queues a repeated start: always 0
  }
  return (g_txAddr == 0x68 || g_txAddr == 0x6A) ? 0 : 2;
}

#if __has_include("Adafruit_MPU6050.h")
#include "Adafruit_MPU6050.h"

// What the chip is ACTUALLY configured to, which the data registers below are
// scaled by - so a configuration write that did not land shows up in the data
// exactly as it would on the device.
mpu6050_accel_range_t g_mpuAccelRange = MPU6050_RANGE_2_G;
mpu6050_gyro_range_t g_mpuGyroRange = MPU6050_RANGE_500_DEG;
mpu6050_bandwidth_t g_mpuBandwidth = MPU6050_BAND_260_HZ;

// MPU-6050: 14 big-endian bytes from 0x3B - accel, temperature, gyro.
static size_t mpuBurst(uint8_t *out) {
  memset(out, 0, 14);
  if (g_mpuResetState) { // after a power-on reset every data register is 0
    return 14;
  }
  static const float kCountsPerG[] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
  static const float kCountsPerDps[] = {131.0f, 65.5f, 32.8f, 16.4f};
  for (int i = 0; i < 3; i++) {
    const int16_t a = counts(g_motionG[i], kCountsPerG[g_mpuAccelRange]);
    const int16_t g = counts(g_motionDps[i], kCountsPerDps[g_mpuGyroRange]);
    out[2 * i] = (uint8_t)((uint16_t)a >> 8);
    out[2 * i + 1] = (uint8_t)((uint16_t)a & 0xFF);
    out[8 + 2 * i] = (uint8_t)((uint16_t)g >> 8);
    out[9 + 2 * i] = (uint8_t)((uint16_t)g & 0xFF);
  }
  out[6] = 0x04; // die temperature: never read
  out[7] = 0xD2;
  return 14;
}
#endif

#if __has_include("LSM6DS3.h")
#include "LSM6DS3.h"

// The LSM6DS3's control registers, at their power-on values: both sensors
// POWERED DOWN, IF_INC set.
static uint8_t g_lsmCtrl1 = 0x00, g_lsmCtrl2 = 0x00, g_lsmCtrl3 = 0x04;

// The library's own rate/range tables, written out here independently of the
// driver's copy - two mappings that must agree, not one shared constant.
static uint8_t lsmOdr(int hz) {
  return hz == 13     ? 0x10
         : hz == 26   ? 0x20
         : hz == 52   ? 0x30
         : hz == 104  ? 0x40
         : hz == 208  ? 0x50
         : hz == 416  ? 0x60
         : hz == 833  ? 0x70
         : hz == 1660 ? 0x80
         : hz == 3330 ? 0x90
         : hz == 6660 ? 0xA0
                      : 0x40; // the library's `default:` - silently 104 Hz
}
void fakeLsmConfigure(int accelOdrHz, int accelRangeG, int accelBwHz,
                      int gyroOdrHz, int gyroRangeDps) {
  if (!g_configTakes) {
    return; // writes never land: the chip stays powered down
  }
  const uint8_t aFs = accelRangeG == 2    ? 0x00
                      : accelRangeG == 16 ? 0x04
                      : accelRangeG == 4  ? 0x08
                                          : 0x0C;
  const uint8_t bw = accelBwHz == 400   ? 0x00
                     : accelBwHz == 200 ? 0x01
                     : accelBwHz == 100 ? 0x02
                                        : 0x03;
  const uint8_t gFs = gyroRangeDps == 125    ? 0x02
                      : gyroRangeDps == 245  ? 0x00
                      : gyroRangeDps == 500  ? 0x04
                      : gyroRangeDps == 1000 ? 0x08
                                             : 0x0C;
  g_lsmCtrl1 = (uint8_t)(lsmOdr(accelOdrHz) | aFs | bw);
  g_lsmCtrl2 = (uint8_t)(lsmOdr(gyroOdrHz) | gFs);
}
void fakeLsmWrite(uint8_t reg, uint8_t value) {
  if (!g_configTakes) {
    return;
  }
  if (g_bduWriteLands && reg == LSM6DS3_ACC_GYRO_CTRL3_C) {
    g_lsmCtrl3 = value;
  }
  // The driver rewrites CTRL1_XL after begin(), because the library's
  // bandwidth setting means something else on the TR-C. A driver that stopped
  // doing that would leave the library's value here, which no longer matches
  // what it checks for.
  if (g_lpf1WriteLands && reg == LSM6DS3_ACC_GYRO_CTRL1_XL) {
    g_lsmCtrl1 = value;
  }
}

// LSM6DS3: 12 little-endian bytes from 0x22 - gyro then accel. A sensor whose
// ODR bits are 0 is powered down and reads back zeros.
static size_t lsmBurst(uint8_t *out) {
  memset(out, 0, 12);
  for (int i = 0; i < 3; i++) {
    const int16_t g = (g_lsmCtrl2 & 0xF0) ? counts(g_motionDps[i], 1.0f / 0.0175f) : 0;
    const int16_t a = (g_lsmCtrl1 & 0xF0) ? counts(g_motionG[i], 1.0f / 0.000122f) : 0;
    out[2 * i] = (uint8_t)((uint16_t)g & 0xFF);
    out[2 * i + 1] = (uint8_t)((uint16_t)g >> 8);
    out[6 + 2 * i] = (uint8_t)((uint16_t)a & 0xFF);
    out[7 + 2 * i] = (uint8_t)((uint16_t)a >> 8);
  }
  return 12;
}
#endif

size_t FakeWire::requestFrom(uint8_t addr, size_t len, bool) {
  g_rxLen = g_rxPos = 0;
  if (addr != g_txAddr) {
    return 0;
  }
#if __has_include("LSM6DS3.h")
  if (addr == 0x6A && len == 1) { // a control register read-back
    g_rx[0] = g_regPtr == LSM6DS3_ACC_GYRO_CTRL1_XL   ? g_lsmCtrl1
              : g_regPtr == LSM6DS3_ACC_GYRO_CTRL2_G  ? g_lsmCtrl2
              : g_regPtr == LSM6DS3_ACC_GYRO_CTRL3_C  ? g_lsmCtrl3
                                                      : 0x00;
    g_rxLen = 1;
    return 1;
  }
#endif
  // A DATA burst: the only read the scenarios count or fail.
  size_t n = 0;
#if __has_include("Adafruit_MPU6050.h")
  if (addr == 0x68 && g_regPtr == MPU6050_ACCEL_OUT && len == 14) n = mpuBurst(g_rx);
#endif
#if __has_include("LSM6DS3.h")
  if (addr == 0x6A && g_regPtr == LSM6DS3_ACC_GYRO_OUTX_L_G && len == 12) n = lsmBurst(g_rx);
#endif
  if (n == 0) {
    return 0;
  }
  g_reads++;
  if (g_failNext > 0) { // injected failure: a SHORT read, not a clean refusal
    g_failNext--;
    n /= 2;
  }
  g_rxLen = n;
  return n;
}
int FakeWire::available() { return (int)(g_rxLen - g_rxPos); }
int FakeWire::read() { return g_rxPos < g_rxLen ? g_rx[g_rxPos++] : -1; }
