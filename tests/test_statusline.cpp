#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <initializer_list>

#include "../code/Gnimu/src/core/StatusLine.h"
#include "display_units.h"

using namespace gnimu;
using gnimu::test::degreesShowing;
using gnimu::test::sampleShowing;

static void testWithFix() {
  char buf[256];
  GnssEpoch e;
  e.fixType = 3;
  e.numSv = 10;
  e.hAccMm = 529;
  e.tAccNs = 22;
  e.latDeg1e7 = 480000000;    // 48.0000000
  e.lonDeg1e7 = -1280000000;  // -128.0000000 (west)
  // 19.50Hz deliberately exercises the round-half-up boundary -> 20Hz; BLE
  // 19.49Hz checks the other side of it -> 19Hz.
  DeviceStatus v = {};
  v.uptimeUs = 42500000ULL;
  v.gnss.latest = &e;
  v.imu.sample = sampleShowing(-12, 3, 998, 5, -2, 1);
  v.imu.trimLocked = true;
  v.imu.tiltDegrees = degreesShowing(1234);
  const StatusRates r = {1949, 1950};
  const size_t n = formatStatus(buf, sizeof(buf), v, r);
  assert(n == strlen(buf));
  assert(strcmp(buf,
                "RT: 42s | BLE: 19Hz | GNSS: 20Hz | Fix: 3D | SV: 10 | tAcc: 22ns | hAcc: 529mm | "
                "Lat: 48.0000000 | Long: -128.0000000 | "
                "Accel mG: -12 3 998 | Gyro c\xC2\xB0/s: 5 -2 1 | "
                "Trim: 12.3\xC2\xB0 \xF0\x9F\x94\x92\n") == 0);
  e.fixType = 0;
  formatStatus(buf, sizeof(buf), v, r);
  assert(strstr(buf, "| Fix: none | SV: 10 |"));
  e.fixType = 2;
  formatStatus(buf, sizeof(buf), v, r);
  assert(strstr(buf, "| Fix: 2D | SV: 10 |"));
  e.fixType = 3;
  e.latDeg1e7 = INT32_MIN;  // the one value whose signed negation overflows
  formatStatus(buf, sizeof(buf), v, r);
  assert(strstr(buf, "Lat: -214.7483648 | "));
}

static void testNoFixOmitsFixFields() {
  char buf[256];
  DeviceStatus v = {};
  v.uptimeUs = 5000000ULL;
  formatStatus(buf, sizeof(buf), v, StatusRates{0, 0});
  assert(strstr(buf, "Fix: none |"));
  assert(!strstr(buf, "SV:"));
  assert(!strstr(buf, "tAcc:"));
  assert(!strstr(buf, "hAcc:"));
  assert(!strstr(buf, "Lat:"));
  assert(!strstr(buf, "Long:"));
  assert(strstr(buf, "BLE: 0Hz | GNSS: 0Hz |"));
  assert(strstr(buf, "Trim: 0.0\xC2\xB0 \xE2\x8F\xB3\n"));
}

static void testTruncationNeverOverruns() {
  GnssEpoch e;
  e.fixType = 3;
  e.numSv = 10;
  e.hAccMm = 529;
  e.tAccNs = 22;
  DeviceStatus v = {};
  v.gnss.latest = &e;
  v.imu.sample = sampleShowing(-12, 3, 998, 5, -2, 1);
  v.imu.trimLocked = true;
  v.imu.tiltDegrees = degreesShowing(1234);
  const StatusRates r = {2000, 2000};
  for (size_t cap : {size_t(0), size_t(1), size_t(8), size_t(40)}) {
    char buf[256];
    memset(buf, 'x', sizeof(buf));
    const size_t n = formatStatus(buf, cap, v, r);
    assert(n < (cap ? cap : 1));
    if (cap) assert(buf[n] == '\0');
    for (size_t i = cap; i < sizeof(buf); i++) assert(buf[i] == 'x');
  }
}

// The Batt cell only exists where there's a battery; the bolt needs both USB
// and an active charger.
static void testBatteryCell() {
  char buf[256];
  BatteryStatus b;
  b.valid = true;
  b.voltage = 3.906f;  // rounds to 3.91
  b.percent = 76;
  b.chargerActive = true;
  DeviceStatus v = {};
  v.power.battery = &b;
  v.power.usbPresent = true;
  const StatusRates r = {0, 2000};
  formatStatus(buf, sizeof(buf), v, r);
  assert(strstr(buf, "| GNSS: 20Hz | Batt: 3.91V 76% \xE2\x9A\xA1 | Fix: none |"));

  v.power.usbPresent = false;
  formatStatus(buf, sizeof(buf), v, r);
  assert(strstr(buf, "| Batt: 3.91V 76% | Fix: none |"));

  b.valid = false;  // no reading yet: say nothing rather than 0.00V
  formatStatus(buf, sizeof(buf), v, r);
  assert(!strstr(buf, "Batt:"));
}

// The display conversion itself, away from truncation boundaries: 1 g a
// little over, and 1 rad/s.
static void testDisplayUnits() {
  ImuSample s;
  s.accel[2] = 9.81f;   // 1000.3 mG
  s.accel[0] = -9.81f;  // truncates toward zero
  s.gyro[1] = 1.0f;     // 5729.6 c°/s
  const ImuDisplay d = toDisplayUnits(s);
  assert(d.accelMilliG[2] == 1000 && d.accelMilliG[0] == -1000 && d.accelMilliG[1] == 0);
  assert(d.gyroCentiDegS[1] == 5729);
  assert(tiltCentiDeg(12.349f) == 1234);
}

// The longest line the device can produce must fit kStatusLineMaxBytes whole,
// newline included: the nRF can't send a longer one at all. Both accuracy
// forms are checked, since "unknown" is shorter than the longest number.
static void testWorstCaseFits() {
  BatteryStatus b;
  b.valid = true;
  b.voltage = 4.2f;
  b.percent = 100;
  b.chargerActive = true;
  GnssEpoch e;
  e.fixType = 4;  // "GNSS+DR", the longest fix name
  e.numSv = 99;   // two digits: more than any receiver tracks
  e.latDeg1e7 = -899999999;
  e.lonDeg1e7 = -1799999999;
  DeviceStatus v = {};
  v.uptimeUs = 99999999ULL * 1000000ULL;  // 8-digit RT, about 3 years
  v.power.battery = &b;
  v.power.usbPresent = true;
  v.gnss.latest = &e;
  // Beyond full scale (4 g, 500 deg/s), with signs, for margin.
  v.imu.sample = sampleShowing(-4999, -4999, -4999, -59999, -59999, -59999);
  v.imu.tiltDegrees = degreesShowing(18000);
  const StatusRates r = {9900, 9900};  // 99Hz: epochs and packets top out at the 25 Hz nav limit
  for (uint32_t acc : {UINT32_MAX, UINT32_MAX - 1}) {
    e.tAccNs = acc;
    e.hAccMm = acc;
    char buf[2 * kStatusLineMaxBytes];
    const size_t n = formatStatus(buf, sizeof(buf), v, r);
    assert(n < kStatusLineMaxBytes);
    assert(buf[n - 1] == '\n');
  }
}

static void testUnknownAccuracy() {
  char buf[kStatusLineMaxBytes];
  GnssEpoch e;
  e.tAccNs = UINT32_MAX;
  e.hAccMm = UINT32_MAX;
  DeviceStatus v = {};
  v.gnss.latest = &e;
  formatStatus(buf, sizeof(buf), v, StatusRates{0, 0});
  assert(strstr(buf, "| tAcc: unknown | hAcc: unknown |"));
}

int main() {
  testDisplayUnits();
  testWorstCaseFits();
  testUnknownAccuracy();
  testBatteryCell();
  testWithFix();
  testNoFixOmitsFixFields();
  testTruncationNeverOverruns();
  puts("statusline: all tests passed");
  return 0;
}
