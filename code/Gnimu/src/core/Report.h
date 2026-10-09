#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "BatteryMonitor.h"
#include "BleStats.h"
#include "DeviceStatus.h"
#include "GnssEpoch.h"
#include "GnssNames.h"
#include "ResetReason.h"
#include "Scheduler.h"
#include "TextOut.h"

namespace gnimu {

// The scheduler's side of the D report: per-job stats and the loop count.
struct JobTable {
  const Job *jobs;
  const JobStats *stats;
  size_t count;
  uint32_t loops;
};

// The console's D-report buffer. test_report checks that the worst case
// (every counter at 10 digits) fits, so a report that grows fails a test
// rather than losing its last lines on the device.
constexpr size_t kReportMaxBytes = 2048;

// Text of the on-request diagnostics report, formatted into a caller buffer so
// it can be tested on a host. Output is truncated, never overrun.
inline size_t formatReport(char *buf, size_t cap, const JobTable &t, const DeviceStatus &d) {
  detail::Out o{buf, cap, 0};
  if (cap)
    buf[0] = '\0';

  // Column widths: at least wide enough for the header label, widened to fit
  // whatever value is actually present so the table stays aligned regardless
  // of job name length or how large a count has grown.
  size_t wJob = 3, wRuns = 4, wSkipped = 7, wOverruns = 8, wMaxUs = 6,
         wMeanUs = 7, wLateUs = 11;
  for (size_t i = 0; i < t.count; i++) {
    const JobStats &s = t.stats[i];
    const unsigned long meanUs = s.runs ? s.totalUs / s.runs : 0;
    const size_t nameLen = strlen(t.jobs[i].name);
    if (nameLen > wJob)
      wJob = nameLen;
    if (detail::decimalWidth(s.runs) > wRuns)
      wRuns = detail::decimalWidth(s.runs);
    if (detail::decimalWidth(s.skipped) > wSkipped)
      wSkipped = detail::decimalWidth(s.skipped);
    if (detail::decimalWidth(s.overruns) > wOverruns)
      wOverruns = detail::decimalWidth(s.overruns);
    if (detail::decimalWidth(s.maxUs) > wMaxUs)
      wMaxUs = detail::decimalWidth(s.maxUs);
    if (detail::decimalWidth(meanUs) > wMeanUs)
      wMeanUs = detail::decimalWidth(meanUs);
    if (detail::decimalWidth(s.maxLateUs) > wLateUs)
      wLateUs = detail::decimalWidth(s.maxLateUs);
  }
  // 7 columns, each framed as " | <cell>" plus the table's opening "| " and
  // closing " |": 2 + 2 + 6*3 = 22 characters of furniture around the widths.
  const size_t width =
      22 + wJob + wRuns + wSkipped + wOverruns + wMaxUs + wMeanUs + wLateUs;

  o.fill('=', width);
  o.put("\n| ");
  o.left("job", wJob);
  o.put(" | ");
  o.right("runs", wRuns);
  o.put(" | ");
  o.right("skipped", wSkipped);
  o.put(" | ");
  o.right("overruns", wOverruns);
  o.put(" | ");
  o.right("max μs", wMaxUs);
  o.put(" | ");
  o.right("mean μs", wMeanUs);
  o.put(" | ");
  o.right("max late μs", wLateUs);
  o.put(" |\n");
  o.fill('-', width);
  o.put("\n");

  for (size_t i = 0; i < t.count; i++) {
    const JobStats &s = t.stats[i];
    const unsigned long meanUs = s.runs ? s.totalUs / s.runs : 0;
    o.put("| ");
    o.left(t.jobs[i].name, wJob);
    o.put(" | ");
    o.right(static_cast<unsigned long>(s.runs), wRuns);
    o.put(" | ");
    o.right(static_cast<unsigned long>(s.skipped), wSkipped);
    o.put(" | ");
    o.right(static_cast<unsigned long>(s.overruns), wOverruns);
    o.put(" | ");
    o.right(static_cast<unsigned long>(s.maxUs), wMaxUs);
    o.put(" | ");
    o.right(meanUs, wMeanUs);
    o.put(" | ");
    o.right(static_cast<unsigned long>(s.maxLateUs), wLateUs);
    o.put(" |\n");
  }
  o.fill('=', width);
  o.put("\n");

  // RT and Loops split the bottom row in half, label left and value right in
  // each: "| " + left + " | " + right + " |" is 7 characters of furniture.
  // The narrowest table (68) leaves 30 per half, room for any 10-digit value.
  char rt[16];
  snprintf(rt, sizeof(rt), "%lus", static_cast<unsigned long>(d.uptimeUs / 1000000ULL));
  const size_t wLeft = (width - 7) / 2, wRight = width - 7 - wLeft;
  o.put("| RT:");
  o.right(rt, wLeft - 3);
  o.put(" | Loops:");
  o.right(static_cast<unsigned long>(t.loops), wRight - 6);
  o.put(" |\n");
  o.fill('=', width);
  o.put("\n");

  char reset[64];
  formatReset(reset, sizeof(reset), d.reset);
  o.put("Reset: %s\n", reset);
  if (d.gnss.settings)
    o.put("GNSS: %s\n", d.gnss.settings);
  o.put("GNSS: epochs %lu, missed %lu, max gap %luμs, time steps %lu\n",
        static_cast<unsigned long>(d.gnss.epochs->epochs()),
        static_cast<unsigned long>(d.gnss.epochs->missed()),
        static_cast<unsigned long>(d.gnss.epochs->maxGapUs()),
        static_cast<unsigned long>(d.gnss.epochs->timeSteps()));
  if (d.gnss.latest) {
    const GnssEpoch &e = *d.gnss.latest;
    char hAcc[16];
    char tAcc[16];
    formatAccuracy(hAcc, sizeof(hAcc), e.hAccMm, "mm");
    formatAccuracy(tAcc, sizeof(tAcc), e.tAccNs, "ns");
    // gnssFixOK is false while a fix is outside the receiver's accuracy
    // limits; a 3D fix that isn't OK yet is easy to misread without it.
    o.put("GNSS: %s%s, %u SVs, hAcc %s, tAcc %s, pDOP %u.%02u\n", fixTypeName(e.fixType),
          e.fixType != 0 && !e.gnssFixOk ? " (not yet OK)" : "",
          static_cast<unsigned>(e.numSv), hAcc, tAcc, static_cast<unsigned>(e.pDop / 100),
          static_cast<unsigned>(e.pDop % 100));
  } else {
    o.put("GNSS: no epoch yet\n");
  }
  const ImuDisplay imu = toDisplayUnits(d.imu.sample);
  o.put("IMU: %s, Accel XYZ mG %ld %ld %ld, Gyro XYZ c°/s %ld %ld %ld\n",
        d.imu.up ? "up" : "down", static_cast<long>(imu.accelMilliG[0]),
        static_cast<long>(imu.accelMilliG[1]),
        static_cast<long>(imu.accelMilliG[2]),
        static_cast<long>(imu.gyroCentiDegS[0]),
        static_cast<long>(imu.gyroCentiDegS[1]),
        static_cast<long>(imu.gyroCentiDegS[2]));
  // tiltDegrees() is an arccos-based angle magnitude, always >= 0, so it
  // needs no sign handling here. Rounded to tenths (not truncated) and split
  // into whole/fraction, the same integer-only approach already used for
  // pDOP above - no float format specifier required.
  const unsigned long tiltTenths =
      (static_cast<unsigned long>(tiltCentiDeg(d.imu.tiltDegrees)) + 5) / 10;
  o.put("IMU: trim %s, tilt %lu.%lu\xC2\xB0\n", d.imu.trimLocked ? "locked" : "settling",
        tiltTenths / 10, tiltTenths % 10);
  o.put("BLE: %s, sent %lu, failed %lu, refused %lu, max send %lu\xCE\xBCs, rx %lu bytes, "
        "rx dropped %lu\n",
        d.ble.connected ? "connected" : "disconnected", static_cast<unsigned long>(d.ble.stats.sent),
        static_cast<unsigned long>(d.ble.stats.failed), static_cast<unsigned long>(d.ble.stats.refused),
        static_cast<unsigned long>(d.ble.stats.maxSendUs), static_cast<unsigned long>(d.ble.stats.rxBytes),
        static_cast<unsigned long>(d.ble.stats.rxDropped));
  if (d.power.battery && d.power.battery->valid) {
    const BatteryStatus &b = *d.power.battery;
    const unsigned long centiV = static_cast<unsigned long>(b.voltage * 100.0f + 0.5f);
    const unsigned long peakCentiV = static_cast<unsigned long>(b.freshVoltage * 100.0f + 0.5f);
    o.put("Battery: %lu.%02luV (last peak %lu.%02luV), %u%%", centiV / 100, centiV % 100,
          peakCentiV / 100, peakCentiV % 100, static_cast<unsigned>(b.percent));
    // Only the flags that are true. Charging is isCharging(), the same as the
    // boot line, the status line and the RaceBox bit use.
    if (d.power.usbPresent)
      o.put(", USB");
    if (isCharging(b, d.power.usbPresent))
      o.put(", charging");
    if (b.chargeFull)
      o.put(", full");
    if (b.warning)
      o.put(", warning");
    if (b.critical)
      o.put(", critical");
    if (b.cutoffRequested)
      o.put(", cutoff requested");
    o.put(", switch noise %lu\n", static_cast<unsigned long>(d.power.switchNoise));
  } else if (d.power.battery) {
    o.put("Battery: no reading yet\n");
  }
  o.put("Console: dropped %lu bytes\n", static_cast<unsigned long>(d.consoleDropped));
  return o.len;
}

} // namespace gnimu
