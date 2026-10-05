#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <initializer_list>

#include "../code/Gnimu/src/core/StatusLine.h"

using namespace gnimu;

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
  StatusView v{42500000ULL, 1949, 1950, nullptr, false, &e, {-12, 3, 998}, {5, -2, 1}, true, 1234};
  const size_t n = formatStatus(buf, sizeof(buf), v);
  assert(n == strlen(buf));
  assert(strcmp(buf,
                "RT: 42s | BLE: 19Hz | GNSS: 20Hz | Fix: 3 | SV: 10 | tAcc: 22ns | hAcc: 529mm | "
                "Lat: 48.0000000 | Long: -128.0000000 | "
                "Accel mG: -12 3 998 | Gyro c\xC2\xB0/s: 5 -2 1 | "
                "Trim: 12.3\xC2\xB0 \xF0\x9F\x94\x92\n") == 0);
  e.latDeg1e7 = INT32_MIN;  // the one value whose signed negation overflows
  formatStatus(buf, sizeof(buf), v);
  assert(strstr(buf, "Lat: -214.7483648 | "));
}

static void testNoFixOmitsFixFields() {
  char buf[256];
  StatusView v{5000000ULL, 0, 0, nullptr, false, nullptr, {0, 0, 0}, {0, 0, 0}, false, 0};
  formatStatus(buf, sizeof(buf), v);
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
  StatusView v{0, 2000, 2000, nullptr, false, &e, {-12, 3, 998}, {5, -2, 1}, true, 1234};
  for (size_t cap : {size_t(0), size_t(1), size_t(8), size_t(40)}) {
    char buf[256];
    memset(buf, 'x', sizeof(buf));
    const size_t n = formatStatus(buf, cap, v);
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
  StatusView v{0, 0, 2000, &b, true, nullptr, {0, 0, 0}, {0, 0, 0}, false, 0};
  formatStatus(buf, sizeof(buf), v);
  assert(strstr(buf, "| GNSS: 20Hz | Batt: 3.91V 76% \xE2\x9A\xA1 | Fix: none |"));

  v.usbPresent = false;
  formatStatus(buf, sizeof(buf), v);
  assert(strstr(buf, "| Batt: 3.91V 76% | Fix: none |"));

  b.valid = false;  // no reading yet: say nothing rather than 0.00V
  formatStatus(buf, sizeof(buf), v);
  assert(!strstr(buf, "Batt:"));
}

int main() {
  testBatteryCell();
  testWithFix();
  testNoFixOmitsFixFields();
  testTruncationNeverOverruns();
  puts("statusline: all tests passed");
  return 0;
}
