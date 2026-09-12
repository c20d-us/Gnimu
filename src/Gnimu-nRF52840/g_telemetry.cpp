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

#include "g_telemetry.h"
#include "config.h"
#include "g_battery.h"
#include "g_ble.h"
#include "g_gnss.h"
#include "g_imu.h"
#include "g_imu_trim.h"
#include "g_log.h"
#include "g_protocol_active.h"

#include <stdio.h> // snprintf, for the stats-line renderer below

// Internal counters and a private pointer to the latest PVT data
static unsigned long bootTimeMs = 0;
static unsigned long lastReportMs = 0;
static unsigned int bleSentPacketCount = 0;
static unsigned int gnssEpochCount = 0;
// Rates published by updateRates() at each window close. Always compiled in -
// see telemetryGnssRateHz() in the header for why.
static float gnssRateHz = 0.0f;
static float bleRateHz = 0.0f;
// Epoch-to-epoch span the rates are measured over - see updateRates().
static unsigned long lastEpochMs = 0; // millis() when the latest epoch was used
static unsigned long spanStartMs = 0; // the epoch the current span opens on
static bool spanStarted = false;      // false until an epoch opens a span
static const UBX_NAV_PVT_data_t *pvt = nullptr;

// The protocol selected by TELEMETRY_PROTOCOL in config.h. This module names
// no concrete protocol - adding one touches g_protocol_active.h and nothing
// here.
static const ProtocolDescriptor *proto = ACTIVE_PROTOCOL;

// Copy one GNSS epoch, plus the current IMU and battery state, into the
// canonical sample.
//
// DELIBERATELY MECHANICAL - pure field copies, no decisions. Every judgement
// about what a value MEANS belongs to the encoder (see g_protocol.h's Rule 2),
// which is also the half the host harness can test. Keeping this function
// dumb is what concentrates the risk somewhere it can be measured.
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

  // Raw, unclamped. The RaceBox encoder does its own clamping.
  s.fixType = p.fixType;
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

  // Taken as a PARAMETER, not fetched here. imuLatchForEpoch() must run first
  // to phase-lock the sample to this epoch, and passing its result in makes
  // that a data dependency the compiler enforces rather than an ordering
  // comment a later edit could reorder past.
  s.accelX = imu.gX;
  s.accelY = imu.gY;
  s.accelZ = imu.gZ;
  s.gyroX = imu.rX;
  s.gyroY = imu.rY;
  s.gyroZ = imu.rZ;

  // Percent and charging separately, NOT a packed protocol byte - the packing
  // is RaceBox-specific and now lives in the encoder. Both nRF and ESP32
  // expose batteryGetStatus(), the latter as a constant-reporting stub, so no
  // per-variant branch is needed here.
  const BatteryStatus bat = batteryGetStatus();
  s.batteryPercent = bat.percent;
  s.batteryCharging = bat.charging;

  return s;
}

// Close the stats window: convert the epoch/packet counts accumulated since
// the last window into rates, publish them, and reset the counters.
//
// THE RATE IS MEASURED EPOCH TO EPOCH, not over the clock window. The window
// still closes on the clock, once a second, so the report cadence is exactly
// what it was - but the span divided into is from the last epoch of the
// previous window to the last epoch of this one.
//
// Why: the window is timed on the MCU's clock and the epochs arrive on the
// receiver's. Dividing by the clock window made an epoch that landed near a
// boundary count in whichever window jitter chose, so a perfect 20Hz stream
// read as balanced 21/19 pairs, in clusters, as the two clocks drifted. That
// is exactly the "GNSS rate below 20" signature g_gnss.h documents for UART
// loss - a false alarm on the one diagnostic that loss has. Measured epoch to
// epoch, a window's edges ARE epochs, so there is nothing to be ambiguous
// about, and each span begins where the last ended, so every gap between
// epochs is counted exactly once. A genuinely lost epoch, anywhere - including
// right at a boundary - lowers exactly one window's reading. Chosen over
// closing the window on an epoch, which fixed the rate too but moved the
// aliasing into the RT counter; see NEW-3 in docs/code-review-remediation.md
// for the simulation that decided it.
//
// What remains is a tenth: each edge carries the loop's pickup jitter, so
// about 1 window in 25 reads 19.9 or 20.1. A lost epoch reads 19.0 or lower.
//
// The published value keeps that tenth - telemetryGnssRateHz() returns the
// float, and the harness asserts on it. Only the DISPLAYS round to whole
// numbers (the stats line here, the OLED's rate field): at an integer nav rate
// the real resolution is whole epochs per second, so the tenth shows pickup
// jitter and nothing else. A lost epoch still reads 19.
static void updateRates(unsigned long now) {
  if (gnssEpochCount > 0 && lastEpochMs != spanStartMs) {
    const float span = (lastEpochMs - spanStartMs) / 1000.0f;
    gnssRateHz = gnssEpochCount / span;
    // Complete packets accepted by the transport, NOT encode attempts - see
    // the drop bracket in telemetrySendIfReady(). Counted on the same epochs
    // over the same span, so a BLE rate below the GNSS rate means frames were
    // lost in THIS window, which is the whole point.
    bleRateHz = bleSentPacketCount / span;
    spanStartMs = lastEpochMs; // this span's closing edge opens the next
  } else {
    // No epoch arrived this window (receiver down, stalled, or asleep): report
    // zero, and let the next epoch open a fresh span rather than stretching
    // one across the gap.
    gnssRateHz = 0.0f;
    bleRateHz = 0.0f;
    spanStarted = false;
  }
  gnssEpochCount = 0;
  bleSentPacketCount = 0;
  lastReportMs = now;
}

float telemetryGnssRateHz() { return gnssRateHz; }
float telemetryBleRateHz() { return bleRateHz; }

// Periodically print packet rate and GNSS/IMU debug stats over serial.
// Called only when a stats window has just closed, so it does no timing of its
// own - it reads the rates updateRates() has already published.
#if LOG_ENABLED
// ---------------------------------------------------------------------------
// 1Hz stats line rendering.
//
// THE LINE HAS A HARD CEILING, and the failure past it is not truncation. On
// the nRF cores Print::printf formats into a 256-byte STACK buffer and then
// calls write(buf, len) with vsnprintf's return - which is the length it WOULD
// have written. Over 255 bytes that reads past the buffer and transmits
// adjacent stack memory to the console. ESP32's Print::vprintf reallocates
// correctly, so this is nRF-only - and both nRF variants are the ones with
// BATTERY_HAS_GAUGE, i.e. the ones printing the LONGEST line.
//
// Two independent defences, deliberately not merged into one:
//
//   1. Every field below is width-bounded, so the maximum is arithmetic rather
//      than hope. All fields at their type maxima gives 216 bytes against the
//      255 ceiling - 39 spare, roughly three more fields' worth. The widths:
//      uint32 -> 10 digits, int32/1e7 -> 12, int16 -> 6, uint8 -> 3, and the
//      capped fields here -> 6 digits + unit.
//
//   2. telemetrySerialReport() formats into its own buffer with snprintf and
//      prints the result, so Print::printf never sees a long format at all.
//      snprintf has the bounded semantics Print::printf only appears to have:
//      an over-long line becomes a CLIPPED line, not a stack over-read.
//
// (2) is what stops (1) being load-bearing. Anything added to this line should
// state its width bound above; the buffer keeps the mistake survivable if it
// does not.
// ---------------------------------------------------------------------------

// Past this bound a measurement carries no information, so it renders as "-".
//
// A THRESHOLD, not a comparison against 0xFFFFFFFF: the receiver is not
// contractually bound to any particular sentinel, and a value past the bound
// is uninformative however it got there. It also does real work for the
// reader, since "4294967295" spends ten characters saying "no data" on a line
// meant to be scanned at a glance.
//
// 999999 is deliberately generous - tAcc converges to 10-50ns and hAcc to a
// few metres, so a receiver walking down toward a fix shows real numbers the
// whole way and reads "-" only while genuinely meaningless. The bound doubles
// as the field's width cap, which is what makes the budget above provable.
static constexpr uint32_t kStatsMeasMax = 999999;

// Renders "<value><unit>", or "-" past the bound. The unit travels with the
// value so the empty case reads "tA: -" rather than "tA: -ns".
static const char *fmtMeas(char *buf, size_t n, uint32_t v, const char *unit) {
  if (v > kStatsMeasMax) {
    return "-";
  }
  snprintf(buf, n, "%u%s", (unsigned int)v, unit);
  return buf;
}

// Same idea for a coordinate, bounded by what the quantity can physically be.
//
// Written as !(deg >= -limit && deg <= limit) rather than fabs(deg) > limit so
// that a NaN takes the "-" path too: every comparison against NaN is false, so
// the negation is true. A fabs() test would pass it through to print "nan".
static const char *fmtCoord(char *buf, size_t n, double deg, double limit) {
  if (!(deg >= -limit && deg <= limit)) {
    return "-";
  }
  snprintf(buf, n, "%.7f", deg);
  return buf;
}
#endif // LOG_ENABLED

static void telemetrySerialReport(unsigned long now) {
  (void)now; // only read by the report body, which silent builds compile out
#if LOG_ENABLED
  {
    const float bleRate = telemetryBleRateHz();
    const float gnssRate = telemetryGnssRateHz();
    // Additional satellite info for debugging: number of satellites, fix type,
    // horizontal accuracy, and lat/lon
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
    // With no receiver every GNSS field below is a sentinel or a zero -
    // SV: 0, Fix: 0, tAcc: 4294967295, Lat: 0.0000000 - which reads like a
    // device searching for a fix rather than one that has no GNSS at all.
    // Say the true thing instead, and keep the battery segment: the whole
    // point of not halting is that the low-voltage cutoff still runs, and a
    // voltage ticking over is the proof of it.
    if (!gnssIsUp()) {
#if BATTERY_HAS_GAUGE
      const BatteryStatus nb = batteryGetStatus();
      LOG_PRINTF("RT: %us | ❌ GNSS not responding | Batt: %.2fV%s\n",
                 (unsigned int)((now - bootTimeMs) / 1000), nb.voltage,
                 nb.charging ? "⚡" : "");
#else
      LOG_PRINTF("RT: %us | ❌ GNSS not responding\n",
                 (unsigned int)((now - bootTimeMs) / 1000));
#endif
      // Drop reporting below still runs: inbound writes can arrive with no
      // GNSS, and skipping would only defer the delta rather than drop it.
    } else {

    // Convert filtered IMU values to protocol units for display
    ImuProtocolUnits imu = imuReadProtocolUnits();

    // Runtime trim state, three-way. The angle is the MEASURED tilt and is
    // reported even when it exceeded IMU_TRIM_MAX_TILT_DEG and the rotation was
    // refused - that is the case worth seeing on the console, so it must
    // survive the refusal.
    //
    //   ✅  locked
    //   ❌  measured a mount too far off level to correct; never converges
    //   ⏳  still deciding - either no stationary window has closed yet
    //       (tiltDegrees() reads 0 until the first block, so a badly mounted
    //       device shows ⏳ for ~31s before the ❌), or a correctable mount
    //       that has not converged
    //
    // Same policy as the OLED status-bar indicator, and the same collapse of
    // docs/imu-trim-design.md section 9's tiers onto one threshold. On the two
    // variants with no panel this line is the ONLY trim indicator.
    const float trimTilt = imuTrimTiltDegrees();
    const char *trimState = imuTrimConverged() ? "✅"
                            : (trimTilt > IMU_TRIM_MAX_TILT_DEG) ? "❌"
                                                                 : "⏳";

    // Buffers sized from the bounds above: "999999mm" is 8 + NUL, a coordinate
    // "-179.1234567" is 12 + NUL. Both rounded up.
    char tb[12], hb[12], latb[16], lonb[16];
    const char *tAccStr = fmtMeas(tb, sizeof(tb), tAcc, "ns");
    const char *hAccStr = fmtMeas(hb, sizeof(hb), hAcc, "mm");
    const char *latStr = fmtCoord(latb, sizeof(latb), lat, 90.0);
    const char *lonStr = fmtCoord(lonb, sizeof(lonb), lon, 180.0);

    // The IMU segment, pre-rendered like the fields above so it can say "-"
    // when there is no IMU to report - not fitted (IMU_ENABLED 0), not found
    // at boot, or stopped answering. Zeros and a ⏳ that never resolves would
    // read like a sensor at rest waiting for trim, which is not what is
    // happening. Same width budget as before: the live form below is exactly
    // what used to sit inline in the line, and the dashed form is shorter.
    char imub[96];
    if (imuIsUp()) {
      snprintf(imub, sizeof(imub),
               "mG: X=%d Y=%d Z=%d|c°/s: X=%d Y=%d Z=%d|Trim: %.1f° %s",
               imu.gX, imu.gY, imu.gZ, imu.rX, imu.rY, imu.rZ, trimTilt,
               trimState);
    } else {
      snprintf(imub, sizeof(imub), "mG: -|c°/s: -|Trim: -");
    }

    // Built here and printed once - see the ceiling note above the renderer.
    char line[256];

#if BATTERY_HAS_GAUGE
    // Battery voltage/percent/charging for debugging
    const BatteryStatus bat = batteryGetStatus();

    // Print out the informational report. Voltage is the honest measurement;
    // SoC is deliberately omitted from serial output because the voltage->%
    // lookup is masked by charge/TPS draw and can swing 10+% between plugged
    // and unplugged - misleading in the console. The percent still drives the
    // BLE Battery Service + LED thresholds where consumers expect a 0-100 %.
    snprintf(line, sizeof(line),
        "RT: %us|BLE: %.0fHz|GNSS: %.0fHz|SV: %u|Fix: %u|tA: %s|hA: %s|"
        "Lat: %s|Lon: %s|%s|Batt: %.2fV%s\n",
        (unsigned int)((now - bootTimeMs) / 1000), bleRate, gnssRate, sats, fix,
        tAccStr, hAccStr, latStr, lonStr, imub, bat.voltage,
        bat.charging ? "⚡" : "");
    LOG_PRINT(line);
#else
    // No battery gauge on this build - the same report minus the Batt segment
    // (the battery byte in the packet itself still carries the constant
    // percent that batteryGetStatus() reports on this variant).
    snprintf(line, sizeof(line),
        "RT: %us|BLE: %.0fHz|GNSS: %.0fHz|SV: %u|Fix: %u|tA: %s|hA: %s|"
        "Lat: %s|Lon: %s|%s\n",
        (unsigned int)((now - bootTimeMs) / 1000), bleRate, gnssRate, sats, fix,
        tAccStr, hAccStr, latStr, lonStr, imub);
    LOG_PRINT(line);
#endif
    } // end of the GNSS-up branch

    // Dropped frames get their OWN line rather than a field on the one above,
    // and only when the count has moved.
    //
    // The line above works to a hard 255-byte ceiling (see the renderer note):
    // 216 bytes with every field at its bound, and 210 measured at the widest
    // real values by test/telemetry's stats mode (214 before the rates went to
    // whole numbers). A drop field would spend that
    // margin exactly when frames are being lost, which is when the line most
    // needs to survive intact - and its own line costs nothing in the windows
    // with no drops.
    static uint32_t lastDropTotal = 0;
    const uint32_t drops = bleDroppedFrames();
    if (drops != lastDropTotal) {
      LOG_PRINTF("⚠️  BLE dropped %u frame(s) this window (%u total)\n",
                 (unsigned int)(drops - lastDropTotal), (unsigned int)drops);
      lastDropTotal = drops;
    }

    // Inbound losses are reported the same way, and say what they mean: a
    // dropped write is a command the protocol never saw, which for a
    // configuration channel means the device may be half-configured.
    static uint32_t lastWriteDropTotal = 0;
    const uint32_t wdrops = bleDroppedWrites();
    if (wdrops != lastWriteDropTotal) {
      LOG_PRINTF("⚠️  BLE dropped %u inbound write(s) this window (%u total) - "
                 "commands may have been lost\n",
                 (unsigned int)(wdrops - lastWriteDropTotal),
                 (unsigned int)wdrops);
      lastWriteDropTotal = wdrops;
    }

    // Failed IMU reads, reported the same way and for the same reason: single
    // failures are held silently (see imuFailedReads()), so without this an
    // intermittent bus is invisible until it finally takes the IMU down - at
    // which point the line above says "mG: -" and this one has already been
    // saying why. Stops by itself once the IMU is down, since nothing reads it
    // any more.
    static uint32_t lastImuFailTotal = 0;
    const uint32_t imuFails = imuFailedReads();
    if (imuFails != lastImuFailTotal) {
      LOG_PRINTF("⚠️  IMU: %u failed read(s) this window (%u total)\n",
                 (unsigned int)(imuFails - lastImuFailTotal),
                 (unsigned int)imuFails);
      lastImuFailTotal = imuFails;
    }

    // Counters and the window timestamp are reset by updateRates(), which runs
    // whether or not this report is compiled in.
  }
#endif // LOG_ENABLED
}

// Simple startup - just prime the boot time and report timer.
void telemetryBegin() { bootTimeMs = lastReportMs = millis(); }

// On each new GNSS epoch, count it and (when connected) send a packet.
// Always try to send informational report over serial.
void telemetrySendIfReady() {
  if (const UBX_NAV_PVT_data_t *newPvt = gnssConsumePvt()) {
    pvt = newPvt;

    // Rate bookkeeping - see updateRates(). The epoch that OPENS a span is its
    // edge, not one of the epochs inside it, so it is not counted. It is still
    // latched, encoded and sent below exactly like every other epoch; only the
    // counting differs.
    const unsigned long epochMs = millis();
    const bool counted = spanStarted;
    if (counted) {
      gnssEpochCount++;
    } else {
      spanStartMs = epochMs;
      spanStarted = true;
    }
    lastEpochMs = epochMs;

    // Latch the IMU here, phase-locked to the epoch, and DELIBERATELY OUTSIDE
    // the connected test below: ImuAxis::read() is what drains each axis's
    // transient window, and a drain that only happened while a client was
    // attached would let the window accumulate across a disconnect and dump a
    // stale peak into the first packet after reconnecting. See g_imu.h.
    const ImuProtocolUnits imu = imuLatchForEpoch();

    if (bleIsConnected()) {
      // Dispatch through the descriptor, not the concrete encoder: this is
      // the plug-in seam, and bleEmitFrame matches TelemetryEmit exactly, so
      // no adapter sits in between.
      //
      // Bracketing the call with the transport's drop counter is what lets
      // bleSentPacketCount mean "this epoch went out WHOLE" rather than "we
      // tried". encode() itself stays void: an encoder emitting several frames
      // knows to stop on a false (see TelemetryEmit), and the transport has
      // already recorded which ones failed, so there is nothing for a return
      // value here to add. It also stays correct when a protocol emits more
      // than one frame per sample.
      const uint32_t dropsBefore = bleDroppedFrames();
      proto->encode(buildSample(*pvt, imu), bleEmitFrame);
      if (counted && bleDroppedFrames() == dropsBefore) {
        bleSentPacketCount++;
      }
    }
  }

  // Close the stats window on schedule - on the CLOCK, deliberately, so the
  // report cadence never depends on epochs arriving. The rates inside it are
  // measured epoch to epoch (see updateRates()). updateRates() is
  // unconditional so the rates stay available in silent builds; only the
  // printing is compiled out.
  const unsigned long now = millis();
  if ((now - lastReportMs) >= LOG_STATS_INTERVAL_MS) {
    updateRates(now);
    telemetrySerialReport(now);
  }
}
