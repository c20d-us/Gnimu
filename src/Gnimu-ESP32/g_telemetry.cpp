// Gnimu - GNSS+IMU streaming telemetry
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

#include "g_telemetry.h"
#include "config.h"
#include "g_battery.h"
#include "g_ble.h"
#include "g_gnss.h"
#include "g_imu.h"
#include "g_imu_trim.h"
#include "g_log.h"
#include "g_protocol_active.h"

#include <stdio.h>

static unsigned long bootTimeMs = 0;
static unsigned long lastReportMs = 0;
static unsigned int bleSentPacketCount = 0;
static unsigned int gnssEpochCount = 0;
// Published by updateRates(); compiled in even without logging.
static float gnssRateHz = 0.0f;
static float bleRateHz = 0.0f;
// Epoch-to-epoch rate span (see updateRates()).
static unsigned long lastEpochMs = 0; // millis() of the latest epoch
static unsigned long spanStartMs = 0; // epoch that opens the current span
static bool spanStarted = false;      // false until an epoch opens a span
static const UBX_NAV_PVT_data_t *pvt = nullptr;

static const ProtocolDescriptor *proto = ACTIVE_PROTOCOL;

// Copy a GNSS epoch, the latched IMU values, and battery state into a sample.
// Field copies only; interpretation belongs to the encoder.
static TelemetrySample buildSample(const UBX_NAV_PVT_data_t &p,
                                   const ImuProtocolUnits &imu) {
  TelemetrySample s = {};

  s.iTOW = p.iTOW;
  s.year = p.year;
  s.month = p.month;
  s.day = p.day;
  s.hour = p.hour;
  s.min = p.min;
  s.sec = p.sec;

  s.validDate = p.valid.bits.validDate;
  s.validTime = p.valid.bits.validTime;
  s.fullyResolved = p.valid.bits.fullyResolved;
  s.validMag = p.valid.bits.validMag;

  s.tAcc = p.tAcc;
  s.nano = p.nano;

  s.fixType = p.fixType; // unclamped
  s.gnssFixOK = p.flags.bits.gnssFixOK;
  s.headVehValid = p.flags.bits.headVehValid;
  s.numSV = p.numSV;

  s.lon = p.lon;
  s.lat = p.lat;
  s.height = p.height;
  s.hMSL = p.hMSL;
  s.hAcc = p.hAcc;
  s.vAcc = p.vAcc;
  s.gSpeed = p.gSpeed;
  s.headMot = p.headMot;
  s.sAcc = p.sAcc;
  s.headAcc = p.headAcc;
  s.pDOP = p.pDOP;

  // Passed in so imuLatchForEpoch() must run first.
  s.accelX = imu.gX;
  s.accelY = imu.gY;
  s.accelZ = imu.gZ;
  s.gyroX = imu.rX;
  s.gyroY = imu.rY;
  s.gyroZ = imu.rZ;

  const BatteryStatus bat = batteryGetStatus();
  s.batteryPercent = bat.percent;
  s.batteryCharging = bat.charging;

  return s;
}

// Close the stats window: compute rates and reset the counters.
//
// The window closes on the MCU clock, but rates are measured over the span from
// the previous window's last epoch to this window's last epoch. This avoids
// aliasing between the MCU and receiver clocks; a lost epoch lowers exactly one
// window's reading. Loop pickup jitter still moves about 1 window in 25 by
// 0.1Hz, so displays round to whole numbers.
static void updateRates(unsigned long now) {
  if (gnssEpochCount > 0 && lastEpochMs != spanStartMs) {
    const float span = (lastEpochMs - spanStartMs) / 1000.0f;
    gnssRateHz = gnssEpochCount / span;
    // Epochs fully accepted by the transport, over the same span.
    bleRateHz = bleSentPacketCount / span;
    spanStartMs = lastEpochMs; // this span's end opens the next
  } else {
    // No epoch this window: report zero and let the next epoch open a new span.
    gnssRateHz = 0.0f;
    bleRateHz = 0.0f;
    spanStarted = false;
  }
  gnssEpochCount = 0;
  bleSentPacketCount = 0;
  lastReportMs = now;
}

#if LOG_ENABLED
// Stats line rendering
//
// Every field is width-bounded: at type maxima the line is 216 bytes, under
// LOG_LINE_MAX. Give any new field a width bound too.

// Values above this render as "-". Also caps the field width.
static constexpr uint32_t kStatsMeasMax = 999999;

// "<value><unit>", or "-" past kStatsMeasMax.
static const char *fmtMeas(char *buf, size_t n, uint32_t v, const char *unit) {
  if (v > kStatsMeasMax) {
    return "-";
  }
  snprintf(buf, n, "%u%s", (unsigned int)v, unit);
  return buf;
}

// A coordinate, or "-" if outside +/-limit. Written so NaN also renders "-".
static const char *fmtCoord(char *buf, size_t n, double deg, double limit) {
  if (!(deg >= -limit && deg <= limit)) {
    return "-";
  }
  snprintf(buf, n, "%.7f", deg);
  return buf;
}
#endif // LOG_ENABLED

// Print the stats line and any drop/failure lines for the closed window.
static void telemetrySerialReport(unsigned long now) {
  (void)now; // unused when logging is compiled out
#if LOG_ENABLED
  {
    const float bleRate = telemetryBleRateHz();
    const float gnssRate = telemetryGnssRateHz();
    uint8_t sats = 0, fix = 0;
    uint32_t hAcc = 0, tAcc = 0;
    double lat = 0.0, lon = 0.0;
    if (pvt != nullptr) {
      sats = pvt->numSV;
      fix = pvt->fixType;
      hAcc = pvt->hAcc;
      tAcc = pvt->tAcc;
      lat = pvt->lat * 1e-7;
      lon = pvt->lon * 1e-7;
    }
    // GNSS down or stalled: print a short status instead of sentinel or frozen
    // fields. Battery voltage stays to show the cutoff is still running.
    const bool gnssUp = gnssIsUp();
    if (!gnssUp || gnssStalled()) {
      const char *why =
          gnssUp ? "stalled - no data from receiver" : "not responding";
#if BATTERY_HAS_GAUGE
      const BatteryStatus nb = batteryGetStatus();
      LOG_PRINTF("RT: %us | ❌ GNSS %s | Batt: %.2fV%s\n",
                 (unsigned int)((now - bootTimeMs) / 1000), why, nb.voltage,
                 nb.charging ? "⚡" : "");
#else
      LOG_PRINTF("RT: %us | ❌ GNSS %s\n",
                 (unsigned int)((now - bootTimeMs) / 1000), why);
#endif
      // The drop reports below still run.
    } else {

      ImuProtocolUnits imu = imuReadProtocolUnits();

      // Trim state (matches the OLED indicator):
      //   ✅  locked
      //   ❌  tilt too large to correct
      //   ⏳  not yet converged (tilt reads 0 until the first block)
      const float trimTilt = imuTrimTiltDegrees();
      const char *trimState = imuTrimConverged()                   ? "✅"
                              : (trimTilt > IMU_TRIM_MAX_TILT_DEG) ? "❌"
                                                                   : "⏳";

      // "999999mm" is 9 bytes, "-179.1234567" is 13, both rounded up.
      char tb[12], hb[12], latb[16], lonb[16];
      const char *tAccStr = fmtMeas(tb, sizeof(tb), tAcc, "ns");
      const char *hAccStr = fmtMeas(hb, sizeof(hb), hAcc, "mm");
      const char *latStr = fmtCoord(latb, sizeof(latb), lat, 90.0);
      const char *lonStr = fmtCoord(lonb, sizeof(lonb), lon, 180.0);

      // IMU segment, or dashes when the IMU is not up.
      char imub[96];
      if (imuIsUp()) {
        snprintf(imub, sizeof(imub),
                 "mG: X=%d Y=%d Z=%d|c°/s: X=%d Y=%d Z=%d|Trim: %.1f° %s",
                 imu.gX, imu.gY, imu.gZ, imu.rX, imu.rY, imu.rZ, trimTilt,
                 trimState);
      } else {
        snprintf(imub, sizeof(imub), "mG: -|c°/s: -|Trim: -");
      }

      char line[256];

#if BATTERY_HAS_GAUGE
      // Voltage only: percent is unreliable while charging.
      const BatteryStatus bat = batteryGetStatus();

      snprintf(line, sizeof(line),
               "RT: %us|BLE: %.0fHz|GNSS: %.0fHz|SV: %u|Fix: %u|tA: %s|hA: %s|"
               "Lat: %s|Lon: %s|%s|Batt: %.2fV%s\n",
               (unsigned int)((now - bootTimeMs) / 1000), bleRate, gnssRate,
               sats, fix, tAccStr, hAccStr, latStr, lonStr, imub, bat.voltage,
               bat.charging ? "⚡" : "");
      LOG_PRINT(line);
#else
      snprintf(line, sizeof(line),
               "RT: %us|BLE: %.0fHz|GNSS: %.0fHz|SV: %u|Fix: %u|tA: %s|hA: %s|"
               "Lat: %s|Lon: %s|%s\n",
               (unsigned int)((now - bootTimeMs) / 1000), bleRate, gnssRate,
               sats, fix, tAccStr, hAccStr, latStr, lonStr, imub);
      LOG_PRINT(line);
#endif
    } // GNSS up

    // Outbound frame failures, on their own line and only when the count moves.
    // Unsubscribed refusals are excluded.
    static uint32_t lastDropTotal = 0;
    const uint32_t drops = bleDroppedFrames() - bleUnsubscribedFrames();
    if (drops != lastDropTotal) {
      LOG_PRINTF("⚠️  BLE dropped %u frame(s) this window (%u total)\n",
                 (unsigned int)(drops - lastDropTotal), (unsigned int)drops);
      lastDropTotal = drops;
    }

    // Dropped inbound writes.
    static uint32_t lastWriteDropTotal = 0;
    const uint32_t wdrops = bleDroppedWrites();
    if (wdrops != lastWriteDropTotal) {
      LOG_PRINTF("⚠️  BLE dropped %u inbound write(s) this window (%u total) - "
                 "commands may have been lost\n",
                 (unsigned int)(wdrops - lastWriteDropTotal),
                 (unsigned int)wdrops);
      lastWriteDropTotal = wdrops;
    }

    // Failed IMU reads.
    static uint32_t lastImuFailTotal = 0;
    const uint32_t imuFails = imuFailedReads();
    if (imuFails != lastImuFailTotal) {
      LOG_PRINTF("⚠️  IMU: %u failed read(s) this window (%u total)\n",
                 (unsigned int)(imuFails - lastImuFailTotal),
                 (unsigned int)imuFails);
      lastImuFailTotal = imuFails;
    }
  }
#endif // LOG_ENABLED
}

void telemetryBegin() { bootTimeMs = lastReportMs = millis(); }

void telemetrySendIfReady() {
  if (const UBX_NAV_PVT_data_t *newPvt = gnssConsumePvt()) {
    pvt = newPvt;

    // The epoch that opens a span is its edge and is not counted. It is still
    // sent.
    const unsigned long epochMs = millis();
    const bool counted = spanStarted;
    if (counted) {
      gnssEpochCount++;
    } else {
      spanStartMs = epochMs;
      spanStarted = true;
    }
    lastEpochMs = epochMs;

    // Latch on every epoch, connected or not, so the transient window never
    // spans a disconnect.
    const ImuProtocolUnits imu = imuLatchForEpoch();

    if (bleIsConnected()) {
      // An epoch counts as sent if at least one frame was accepted and none
      // failed. Unsubscribed refusals count neither way.
      const uint32_t sentBefore = bleSentFrames();
      const uint32_t failedBefore =
          bleDroppedFrames() - bleUnsubscribedFrames();
      proto->encode(buildSample(*pvt, imu), bleEmitFrame);
      if (counted && bleSentFrames() != sentBefore &&
          bleDroppedFrames() - bleUnsubscribedFrames() == failedBefore) {
        bleSentPacketCount++;
      }
    }
  }

  // Close the stats window on the clock, independent of epochs.
  const unsigned long now = millis();
  if ((now - lastReportMs) >= LOG_STATS_INTERVAL_MS) {
    updateRates(now);
    telemetrySerialReport(now);
  }
}

float telemetryGnssRateHz() { return gnssRateHz; }
float telemetryBleRateHz() { return bleRateHz; }
