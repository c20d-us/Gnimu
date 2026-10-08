#include <assert.h>
#include <stdio.h>
#include <string.h>

#include <initializer_list>

#include "../code/Gnimu/src/core/Report.h"
#include "display_units.h"

using namespace gnimu;
using gnimu::test::degreesShowing;
using gnimu::test::sampleShowing;

static void noop(void*) {}

static void testReport() {
  const Job jobs[] = {{"gnss", 1000, 1000, noop, nullptr}};
  JobStats stats[1];
  stats[0].runs = 4;
  stats[0].skipped = 1;
  stats[0].overruns = 2;
  stats[0].maxUs = 900;
  stats[0].totalUs = 2000;
  EpochTracker epochs(50);
  GnssEpoch latest;
  latest.fixType = 3;
  latest.numSv = 17;
  latest.hAccMm = 210;
  latest.tAccNs = 42;
  latest.pDop = 105;

  char buf[1024];
  const JobTable t = {jobs, stats, 1, 12345};
  DeviceStatus v = {};
  v.uptimeUs = 355400000;
  v.gnss.epochs = &epochs;
  v.gnss.latest = &latest;
  v.imu.up = true;
  v.imu.sample = sampleShowing(-12, 3, 998, 5, -2, 1);
  v.imu.trimLocked = true;
  v.imu.tiltDegrees = degreesShowing(1234);
  v.ble.connected = true;
  v.ble.stats.sent = 42;
  v.ble.stats.failed = 5;
  v.ble.stats.refused = 2;
  v.ble.stats.maxSendUs = 850;
  v.ble.stats.rxBytes = 7;
  v.ble.stats.rxDropped = 3;
  const size_t n = formatReport(buf, sizeof(buf), t, v);
  assert(n == strlen(buf));
  assert(strstr(buf, "| job  | runs | skipped | overruns | max \xCE\xBCs | mean \xCE\xBCs |\n"));
  assert(strstr(buf, "| gnss |    4 |       1 |        2 |    900 |     500 |\n"));
  assert(strstr(buf, "| RT:                 355s | Loops:             12345 |\n"));
  assert(strstr(buf, "GNSS: epochs 0, missed 0, max gap 0\xCE\xBCs, time steps 0\n"));
  assert(strstr(buf, "GNSS: fix type 3, sv count 17, hAcc 210mm, tAcc 42ns, pDOP 1.05\n"));
  assert(strstr(buf, "IMU: up, Accel XYZ mG -12 3 998, Gyro XYZ c\xC2\xB0/s 5 -2 1\n"));
  assert(strstr(buf, "IMU: trim locked, tilt 12.3\xC2\xB0\n"));
  assert(strstr(buf, "BLE: connected, sent 42, failed 5, refused 2, max send 850\xCE\xBCs, rx 7 bytes, "
                     "rx dropped 3\n"));
  assert(!strstr(buf, "Battery:"));  // no battery on this board
  assert(strstr(buf, "Console: dropped 0 bytes\n"));

  BatteryStatus b;
  b.valid = true;
  b.voltage = 3.904f;
  b.freshVoltage = 3.927f;
  b.percent = 75;
  b.chargerActive = true;
  v.power.battery = &b;
  v.power.usbPresent = true;
  formatReport(buf, sizeof(buf), t, v);
  assert(strstr(buf, "Battery: 3.90V (last peak 3.93V), 75%, USB 1, charging 1, full 0, "
                     "warning 0, critical 0, cutoff 0\n"));
  v.power.battery = nullptr;
  v.power.usbPresent = false;

  v.gnss.latest = nullptr;
  v.imu.up = false;
  v.imu.trimLocked = false;
  v.ble.connected = false;
  formatReport(buf, sizeof(buf), t, v);
  assert(strstr(buf, "BLE: disconnected, sent 42,"));
  assert(strstr(buf, "GNSS: fix none\n"));
  assert(strstr(buf, "IMU: down, Accel XYZ mG -12 3 998, Gyro XYZ c\xC2\xB0/s 5 -2 1\n"));
  assert(strstr(buf, "IMU: trim settling, tilt 12.3\xC2\xB0\n"));
}

// Tilt is rounded to tenths, not truncated, so a case that carries into the
// next whole degree (9.99 -> 10.0) needs its own check beyond testReport()'s
// single 12.34 -> 12.3 case, which doesn't exercise the carry.
static void testTiltRounding() {
  const Job jobs[] = {{"gnss", 1000, 1000, noop, nullptr}};
  JobStats stats[1] = {};
  EpochTracker epochs(50);
  char buf[1024];

  const JobTable t = {jobs, stats, 1, 0};
  DeviceStatus v = {};
  v.gnss.epochs = &epochs;
  v.imu.tiltDegrees = degreesShowing(999);
  formatReport(buf, sizeof(buf), t, v);
  assert(strstr(buf, "tilt 10.0\xC2\xB0\n"));

  v.imu.tiltDegrees = degreesShowing(1236);
  formatReport(buf, sizeof(buf), t, v);
  assert(strstr(buf, "tilt 12.4\xC2\xB0\n"));

  v.imu.tiltDegrees = degreesShowing(0);
  formatReport(buf, sizeof(buf), t, v);
  assert(strstr(buf, "tilt 0.0\xC2\xB0\n"));
}

// Every row - header, job rows and the split RT/Loops row - comes out the
// same display width as the top border, both at the minimum column widths
// and once max/mean values outgrow their "max \xCE\xBCs"/"mean \xCE\xBCs" headers (the
// header then needs padding, which byte-counting got wrong because \xCE\xBC is
// two bytes).
static size_t lineWidth(const char* line) {
  size_t w = 0;
  for (; *line && *line != '\n'; line++)
    if ((static_cast<unsigned char>(*line) & 0xC0) != 0x80) w++;
  return w;
}

static void checkAligned(const JobTable& t, const DeviceStatus& v) {
  char buf[1024];
  formatReport(buf, sizeof(buf), t, v);
  const size_t width = lineWidth(buf);
  assert(width > 0);
  for (const char* needle : {"| job", "| gnss", "| console", "| RT:"}) {
    const char* line = strstr(buf, needle);
    assert(line);
    assert(lineWidth(line) == width);
  }
}

static void testTableStaysAligned() {
  const Job jobs[] = {{"gnss", 1000, 1000, noop, nullptr}, {"console", 1000, 1000, noop, nullptr}};
  JobStats stats[2] = {};
  EpochTracker epochs(50);
  const JobTable t = {jobs, stats, 2, 1};
  DeviceStatus v = {};
  v.gnss.epochs = &epochs;
  checkAligned(t, v);

  stats[0].runs = 1;
  stats[0].maxUs = 1234567;     // 7 digits: wider than "max \xCE\xBCs"
  stats[0].totalUs = 12345678;  // mean 8 digits: wider than "mean \xCE\xBCs"
  checkAligned(t, v);
}

static void testTruncationNeverOverruns() {
  const Job jobs[] = {{"gnss", 1000, 1000, noop, nullptr}};
  JobStats stats[1];
  EpochTracker epochs(50);
  const JobTable t = {jobs, stats, 1, 0};
  DeviceStatus v = {};
  v.gnss.epochs = &epochs;
  for (size_t cap : {size_t(0), size_t(1), size_t(8), size_t(40)}) {
    char buf[64];
    memset(buf, 'x', sizeof(buf));
    const size_t n = formatReport(buf, cap, t, v);
    assert(n < (cap ? cap : 1));
    if (cap) assert(buf[n] == '\0');
    for (size_t i = cap; i < sizeof(buf); i++) assert(buf[i] == 'x');
  }
}

int main() {
  testReport();
  testTiltRounding();
  testTableStaysAligned();
  testTruncationNeverOverruns();
  puts("report: all tests passed");
  return 0;
}
