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

// ============================================================================
// TOOL: GNSS identity + high-rate capability report
//
// Connects to the receiver (sweeping common baud rates) and prints what it
// actually is, whether it is clocked for the nav rates config.h asks for, and
// what fix rate it is really delivering at its current settings.
//
// Phases 1-5 are strictly read-only. Phase 6 makes the sketch's only write:
// enabling UBX-NAV-PVT output on the RAM layer so there is something to count.
// That is the same call g_gnss.cpp makes, it is RAM-only so a power cycle
// clears it, and it touches neither the rate nor the constellations - phase 6
// measures the receiver exactly as phase 4 found it. Nothing here writes OTP
// or issues a reset. Set MEASURE_SECONDS to 0 to skip phase 6 and keep the
// whole run read-only.
//
// WHY THIS EXISTS
//
// The Gnimu READMEs and config.h only ever say "u-blox M10 chipset", because
// the HGLRC M100-5883 / M100 Mini boards don't publish which M10 part they
// carry. That matters for nav rate. u-blox ships M10 silicon at a reduced CPU
// clock to save power, and at that default clock the maximum nav rate is much
// lower than the headline figures:
//
//   Constellations enabled        1       2       3       4
//   Default CPU clock          18 Hz   10 Hz   10 Hz    5 Hz
//   High CPU clock             25 Hz   20 Hz   16 Hz   10 Hz
//
// The high-clock row is the one every M10 spec sheet quotes, and it is the row
// the datasheets footnote as "Configuration required" - the configuration
// being a one-time write of a higher CPU clock into the receiver's OTP memory
// (MAX-M10S Integration manual UBX-20053088 sec 2.1.7). Selecting
// constellations and setting CFG-RATE-MEAS does NOT get you there on its own.
//
// Gnimu ships GNSS_NAV_RATE_HZ 20 with GPS + Galileo - two constellations,
// which is a high-clock number. If a board were at the default clock it would
// cap at 10 Hz and simply drop fixes when asked for 20, rather than refuse.
// Phase 3 below reads the clock back so that question is answered by the
// hardware instead of inferred from how smooth the stream looks.
//
// As measured 2026-08-29. Every M100 board checked so far is a bare
// UBX-M10050-KB, ROM SPG 5.10, PROTVER 34.10, and every unprogrammed one reads
// 128/128/128/64 MHz on RAM and Default with no OTP reply at all - so the
// shipped firmware is asking 20 Hz of a receiver rated for 10 Hz on two
// constellations. The manual permits that ("the navigation update rate can be
// increased beyond the maximum value stated in the datasheet. However, this may
// result in a reduced fix rate"), which is why phase 6 exists: the rating says
// nothing about what you are actually getting, and only counting epochs does.
//
// The nRF52840-OLED board (M100 Mini) and the nRF52840 board (M100-5883) have
// since been through gnss_otp_clock - the procedure behaves identically on both
// GNSS board types. Each now reads 192/192/192/96 on the Default and OTP layers
// and returns the manual's expected OTP frame - but RAM still reports
// 128/128/128/64. That is the layer distinction described under phase 3, and
// the reason this sketch judges on the OTP poll. The ESP32 board is
// deliberately left unprogrammed as a stock-clock control.
//
// WHAT IT PRINTS
//
//   Phase 1  Raw UBX-MON-VER: sw/hw version + every extension string. This is
//            where FWVER=SPG 5.xx, PROTVER=xx.xx, MOD=<part number> and the
//            supported-GNSS list live. PROTVER 34.10 = ROM SPG 5.10 (the
//            generation that needs the OTP write); 34.30 = SPG 5.30, where
//            25 Hz / 20 Hz are the native full-power figures.
//   Phase 2  The same identity as the SparkFun library parses it, for
//            comparison - some M10 parts leave fields the library expects
//            empty.
//   Phase 3  CPU clock. Reads the four undocumented CFG keys in group 0x40A4
//            from the RAM and Default layers, then runs the integration
//            manual's own verification poll on VALGET layer 4 - the OTP layer.
//            High clock = 192/192/192/96 MHz, and the OTP poll is the
//            authoritative one: after a successful OTP write these keys read
//            192/192/192/96 on Default and OTP while RAM still reports the
//            stock 128/128/128/64. RAM is not the answer for these keys.
//   Phase 4  Current rate + constellation config, read from RAM, with the
//            nav rate this combination is actually rated for.
//   Phase 5  Message output inventory: every CFG-MSGOUT-*-UART1 key the M10
//            interface description documents, listing the ones with a
//            non-zero rate, plus the port's UBX/NMEA protocol filter. The
//            filter and the per-message rates are independent, so a message
//            can be configured and still never reach the wire - which is how
//            Gnimu runs, since gnssBegin() clears the NMEA protocol bit but
//            leaves the NMEA message rates alone. Marked [protocol off]
//            where that applies.
//
//            This phase also self-checks: its own replies arrive as UBX
//            frames, so a receiver reporting UBX output disabled is telling
//            an impossible story and the reads are lagging rather than the
//            config being unusual. That case is called out loudly, because a
//            shifted-by-one inventory otherwise reads as a real answer.
//   Phase 6  Measured fix rate. Counts NAV-PVT epochs for MEASURE_SECONDS and
//            histograms the gaps in iTOW, which advances by exactly one
//            measurement period per healthy epoch. A skipped epoch shows up as
//            a delta of 2x the period, two skipped as 3x, and so on. This is
//            the number that decides whether an out-of-rating configuration is
//            actually costing you anything: u-blox specs its rate figures for
//            a >=98% fix rate, and a receiver asked for more than it can
//            deliver silently drops epochs rather than refusing the setting.
//            Needs sky view. Also reports the satellite count, since the
//            manual attributes rate loss to "a very large number of
//            satellites" - a poor sky and an overclocked config look nothing
//            alike in the SV stats.
//
//            The UART is not a plausible bottleneck here and doesn't need
//            ruling out: NAV-PVT is 100 bytes on the wire, so 20 Hz is 2 KB/s,
//            about 17% of 115200 baud. Gaps are the receiver, not the link -
//            as long as the port is UBX-only. If a variant ever re-enables
//            NMEA, revisit that before trusting a low number here.
//
// ADAPTING IT
//
// If phase 3 comes back at the default clock and you decide to program the
// high clock, that is a separate tool: tools/common/gnss_otp_clock. It is kept
// separate on purpose - this sketch is a diagnostic you can run against a
// working board any time, and the OTP write is PERMANENT and cannot be
// reverted. Do not fold the write in here.
//
// PASS CRITERIA
//
// Phase 1 identifies the part. Phase 3 passes when the OTP-layer poll returns
// the manual's frame byte-for-byte - that is the manual's own criterion, and it
// holds even when the RAM column still shows the stock clock. Phase 4's rated
// ceiling should then be >= GNSS_NAV_RATE_HZ.
//
// Phase 6 is the one that matters when phase 4's ceiling comes up short: a
// measured fix rate at or above 98% means the receiver is keeping up with what
// config.h asks of it regardless of what it is rated for, and the out-of-rating
// setting is costing nothing measurable. Below that, the loss is real and the
// gap histogram says how it is distributed - a few isolated skips are a very
// different thing from a steady every-other-epoch pattern.
//
// But phase 6 only passes or fails when the run actually loaded the receiver.
// The nav solution is what the clock limits, so a window with no fix, or with
// only a handful of satellites, is reported INCONCLUSIVE however clean the
// count was: an empty epoch costs nothing to emit and will read 100% on any
// clock. A verdict you can act on needs a 3D fix and a MEDIAN satellite count
// of MEASURE_MIN_SV_FOR_VERDICT or more - median rather than mean so the
// acquisition ramp at the start of the window doesn't fail an otherwise good
// run.
//
// Requires: the GNSS wired as the variant's README describes, USB for serial,
// the SparkFun_u-blox_GNSS_v3 library installed, and - for phase 6 - a clear
// enough sky to hold a fix for the duration. Open the serial monitor at 115200.
// ============================================================================

// UBX-MON-VER can return up to ~348 bytes. This must be defined before the
// library header so the config packet buffer is sized to hold a full reply.
#define MAX_PAYLOAD_SIZE 384

#include <Arduino.h>

// --- Platform wiring -------------------------------------------------------
// The only architecture-dependent part of this sketch: which UART object the
// receiver hangs off, and whether opening it needs pin arguments.
#if defined(ARDUINO_ARCH_ESP32)

// ESP32 variant: UART2 on the GPIOs from Gnimu-ESP32/config.h. Change these to
// match your wiring if you routed the receiver elsewhere.
static const int GNSS_RX_PIN = 16; // ESP32 RX <- GNSS TX
static const int GNSS_TX_PIN = 17; // ESP32 TX -> GNSS RX
static HardwareSerial gnssSerial(2);
#define GNSS_SERIAL_BEGIN(baud)                                                \
  gnssSerial.begin((baud), SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN)

#else

// nRF52840 (XIAO) variants: Serial (USB CDC) needs the TinyUSB library linked,
// and this sketch pulls in nothing else that would include it transitively.
#include <Adafruit_TinyUSB.h>

// Serial1 = D6 (TX) / D7 (RX), fixed on the nRF52 - same wiring as g_gnss.cpp.
static Uart &gnssSerial = Serial1;
#define GNSS_SERIAL_BEGIN(baud) gnssSerial.begin(baud)

// TPS63020 EN pad (config.h's GNSS_EN_PIN). Left alone deliberately: the TPS's
// own EN pullup holds the rail on, and this pin is an input at reset, so an
// untouched D9 means a powered receiver. Driving it here would only matter if
// a prior firmware run had left it low, which a reset already undoes.

#endif

#include <SparkFun_u-blox_GNSS_v3.h>

static SFE_UBLOX_GNSS_SERIAL myGNSS;

// Common u-blox baud rates. 115200 first - GNSS_BAUD in every variant's
// config.h - then the factory default and the rest.
static const uint32_t BAUD_RATES[] = {115200, 38400, 9600,
                                      57600,  230400, 460800};
static const int NUM_BAUD_RATES = sizeof(BAUD_RATES) / sizeof(BAUD_RATES[0]);

// The four CPU clock configuration keys the MAX-M10S integration manual polls
// in its own post-OTP verification step (sec 2.1.7, step 5). Undocumented in
// the interface description - group 0xA4, U4 values, in Hz. A receiver running
// the high-performance clock reads back 192/192/192/96 MHz.
struct ClockKey {
  uint32_t key;
  uint32_t highClockHz;
};
static const ClockKey CLOCK_KEYS[] = {
    {0x40A40001UL, 192000000UL},
    {0x40A40003UL, 192000000UL},
    {0x40A40005UL, 192000000UL},
    {0x40A4000AUL, 96000000UL},
};
static const int NUM_CLOCK_KEYS = sizeof(CLOCK_KEYS) / sizeof(CLOCK_KEYS[0]);

// Max nav rate by concurrent constellation count, per u-blox information note
// UBX-23006557. Index 0 is unused so the count indexes directly.
static const uint8_t RATE_DEFAULT_CLOCK[] = {0, 18, 10, 10, 5};
static const uint8_t RATE_HIGH_CLOCK[] = {0, 25, 20, 16, 10};

// Constellations, in the order config.h lists them. QZSS and SBAS are
// augmentation rather than independent constellations and don't count toward
// the concurrent-GNSS figure the rate table is indexed by.
struct Constellation {
  const char *name;
  sfe_ublox_gnss_ids_e id;
  bool countsTowardRate;
};
static const Constellation CONSTELLATIONS[] = {
    {"GPS", SFE_UBLOX_GNSS_ID_GPS, true},
    {"Galileo", SFE_UBLOX_GNSS_ID_GALILEO, true},
    {"GLONASS", SFE_UBLOX_GNSS_ID_GLONASS, true},
    {"BeiDou", SFE_UBLOX_GNSS_ID_BEIDOU, true},
    {"QZSS", SFE_UBLOX_GNSS_ID_QZSS, false},
    {"SBAS", SFE_UBLOX_GNSS_ID_SBAS, false},
};
static const int NUM_CONSTELLATIONS =
    sizeof(CONSTELLATIONS) / sizeof(CONSTELLATIONS[0]);

// --- Message output inventory (phase 5) ------------------------------------
// Every CFG-MSGOUT-*-UART1 key the u-blox M10 SPG interface description
// documents (58 of them), so "nothing else is enabled" is a readout rather
// than an assumption. Generated from the interface description's own key list
// and cross-checked against the SparkFun library's constants, so it covers
// what this receiver can actually emit without carrying the ~80 further keys
// that only exist on high-precision and dead-reckoning products.
//
// Note what this does NOT tell you: a non-zero rate here is the message's
// configured rate, not proof it reaches the wire. The port's protocol filter
// (CFG-UART1OUTPROT-NMEA / -UBX) gates output independently, which is exactly
// how Gnimu runs - gnssBegin() clears the NMEA protocol bit but leaves the
// individual NMEA message rates untouched. Both are reported below.
struct MessageOut {
  uint32_t key;
  const char *name;
};
static const MessageOut MESSAGE_OUT_KEYS[] = {
    {UBLOX_CFG_MSGOUT_NMEA_ID_DTM_UART1, "NMEA-DTM"},
    {UBLOX_CFG_MSGOUT_NMEA_ID_GBS_UART1, "NMEA-GBS"},
    {UBLOX_CFG_MSGOUT_NMEA_ID_GGA_UART1, "NMEA-GGA"},
    {UBLOX_CFG_MSGOUT_NMEA_ID_GLL_UART1, "NMEA-GLL"},
    {UBLOX_CFG_MSGOUT_NMEA_ID_GNS_UART1, "NMEA-GNS"},
    {UBLOX_CFG_MSGOUT_NMEA_ID_GRS_UART1, "NMEA-GRS"},
    {UBLOX_CFG_MSGOUT_NMEA_ID_GSA_UART1, "NMEA-GSA"},
    {UBLOX_CFG_MSGOUT_NMEA_ID_GST_UART1, "NMEA-GST"},
    {UBLOX_CFG_MSGOUT_NMEA_ID_GSV_UART1, "NMEA-GSV"},
    {UBLOX_CFG_MSGOUT_NMEA_ID_RLM_UART1, "NMEA-RLM"},
    {UBLOX_CFG_MSGOUT_NMEA_ID_RMC_UART1, "NMEA-RMC"},
    {UBLOX_CFG_MSGOUT_NMEA_ID_VLW_UART1, "NMEA-VLW"},
    {UBLOX_CFG_MSGOUT_NMEA_ID_VTG_UART1, "NMEA-VTG"},
    {UBLOX_CFG_MSGOUT_NMEA_ID_ZDA_UART1, "NMEA-ZDA"},
    {UBLOX_CFG_MSGOUT_PUBX_ID_POLYP_UART1, "PUBX-POLYP"},
    {UBLOX_CFG_MSGOUT_PUBX_ID_POLYS_UART1, "PUBX-POLYS"},
    {UBLOX_CFG_MSGOUT_PUBX_ID_POLYT_UART1, "PUBX-POLYT"},
    {UBLOX_CFG_MSGOUT_UBX_MON_COMMS_UART1, "UBX-MON-COMMS"},
    {UBLOX_CFG_MSGOUT_UBX_MON_HW3_UART1, "UBX-MON-HW3"},
    {UBLOX_CFG_MSGOUT_UBX_MON_IO_UART1, "UBX-MON-IO"},
    {UBLOX_CFG_MSGOUT_UBX_MON_MSGPP_UART1, "UBX-MON-MSGPP"},
    {UBLOX_CFG_MSGOUT_UBX_MON_RF_UART1, "UBX-MON-RF"},
    {UBLOX_CFG_MSGOUT_UBX_MON_RXBUF_UART1, "UBX-MON-RXBUF"},
    {UBLOX_CFG_MSGOUT_UBX_MON_RXR_UART1, "UBX-MON-RXR"},
    {UBLOX_CFG_MSGOUT_UBX_MON_SPAN_UART1, "UBX-MON-SPAN"},
    {UBLOX_CFG_MSGOUT_UBX_MON_SYS_UART1, "UBX-MON-SYS"},
    {UBLOX_CFG_MSGOUT_UBX_MON_TXBUF_UART1, "UBX-MON-TXBUF"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_AOPSTATUS_UART1, "UBX-NAV-AOPSTATUS"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_CLOCK_UART1, "UBX-NAV-CLOCK"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_COV_UART1, "UBX-NAV-COV"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_DOP_UART1, "UBX-NAV-DOP"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_EOE_UART1, "UBX-NAV-EOE"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_ODO_UART1, "UBX-NAV-ODO"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_ORB_UART1, "UBX-NAV-ORB"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_POSECEF_UART1, "UBX-NAV-POSECEF"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_POSLLH_UART1, "UBX-NAV-POSLLH"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_PVT_UART1, "UBX-NAV-PVT"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_SAT_UART1, "UBX-NAV-SAT"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_SBAS_UART1, "UBX-NAV-SBAS"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_SIG_UART1, "UBX-NAV-SIG"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_SLAS_UART1, "UBX-NAV-SLAS"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_STATUS_UART1, "UBX-NAV-STATUS"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_TIMEBDS_UART1, "UBX-NAV-TIMEBDS"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_TIMEGAL_UART1, "UBX-NAV-TIMEGAL"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_TIMEGLO_UART1, "UBX-NAV-TIMEGLO"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_TIMEGPS_UART1, "UBX-NAV-TIMEGPS"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_TIMELS_UART1, "UBX-NAV-TIMELS"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_TIMEQZSS_UART1, "UBX-NAV-TIMEQZSS"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_TIMEUTC_UART1, "UBX-NAV-TIMEUTC"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_VELECEF_UART1, "UBX-NAV-VELECEF"},
    {UBLOX_CFG_MSGOUT_UBX_NAV_VELNED_UART1, "UBX-NAV-VELNED"},
    {UBLOX_CFG_MSGOUT_UBX_RXM_RLM_UART1, "UBX-RXM-RLM"},
    {UBLOX_CFG_MSGOUT_UBX_RXM_SFRBX_UART1, "UBX-RXM-SFRBX"},
    {UBLOX_CFG_MSGOUT_UBX_SEC_SIG_UART1, "UBX-SEC-SIG"},
    {UBLOX_CFG_MSGOUT_UBX_SEC_SIGLOG_UART1, "UBX-SEC-SIGLOG"},
    {UBLOX_CFG_MSGOUT_UBX_TIM_TM2_UART1, "UBX-TIM-TM2"},
    {UBLOX_CFG_MSGOUT_UBX_TIM_TP_UART1, "UBX-TIM-TP"},
    {UBLOX_CFG_MSGOUT_UBX_TIM_VRFY_UART1, "UBX-TIM-VRFY"},
};
static const int NUM_MESSAGE_OUT_KEYS =
    sizeof(MESSAGE_OUT_KEYS) / sizeof(MESSAGE_OUT_KEYS[0]);

// Phase 6 window. 60 s at 20 Hz is 1200 expected epochs - enough that a 1%
// loss is several counts rather than statistical noise. Set to 0 to skip
// phase 6 entirely and keep the whole run read-only.
static const uint32_t MEASURE_SECONDS = 60;

// Progress line cadence during the measurement, so a 60 s wait doesn't look
// like a hang.
static const uint32_t MEASURE_REPORT_EVERY_MS = 10000;

// Below this median satellite count the run is reported as inconclusive rather
// than passed. Not a spec number - a judgement call. The navigation solution is
// what the CPU clock limits, and the integration manual blames rate loss on "a
// very large number of satellites", so a handful of SVs is the easy case and a
// clean result there does not generalise to an open sky. Twelve is roughly
// where a two-constellation fix stops being trivial.
static const uint8_t MEASURE_MIN_SV_FOR_VERDICT = 12;

// --- Phase 6 measurement state ---------------------------------------------
// Written from the library's NAV-PVT callback, which fires inside
// checkUblox() on the same thread as the measurement loop.
static uint32_t epochCount = 0;
static uint32_t prevITOW = 0;
static uint32_t firstITOW = 0;
static uint32_t lastITOW = 0;
static bool haveFirstEpoch = false;
static uint32_t skippedEpochs = 0;
// Gap histogram, indexed by how many epoch periods the iTOW step covered:
// [0] unused, [1] clean, [2] one skipped, [3] two skipped, [4] three or more.
static uint32_t gapHistogram[5] = {0, 0, 0, 0, 0};
// Steps that weren't a whole multiple of the epoch period - a rate change
// mid-run, a time jump, or a period that isn't what phase 4 read back.
static uint32_t irregularGaps = 0;
static uint8_t minNumSV = 255;
static uint8_t maxNumSV = 0;
static uint32_t sumNumSV = 0;
static uint8_t lastNumSV = 0;
static uint8_t lastFixType = 0;

// Satellite-count distribution, so the verdict can use a median rather than a
// mean. The mean is dragged down by the acquisition ramp at the start of the
// window - a receiver that climbs 7 -> 12 SVs over the first several seconds
// and then holds 12 for the rest of the minute reports a mean well below 12,
// failing a 12-SV gate on a run that was genuinely at 12 nearly throughout.
// The median ignores a minority of low samples, which is exactly the shape of
// that ramp. 64 bins is far above anything an L1-only two-constellation fix
// reaches; counts above it are clamped into the top bin.
static const int SV_HISTOGRAM_BINS = 64;
static uint16_t svHistogram[SV_HISTOGRAM_BINS];

// Satellite count at the midpoint of the distribution.
static uint8_t medianNumSV() {
  uint32_t half = epochCount / 2;
  uint32_t cumulative = 0;
  for (int i = 0; i < SV_HISTOGRAM_BINS; i++) {
    cumulative += svHistogram[i];
    if (cumulative > half) {
      return (uint8_t)i;
    }
  }
  return 0;
}

// Epoch period in ms, from phase 4. Zero means phase 4 couldn't read it, in
// which case phase 6 has no yardstick and is skipped.
static uint32_t epochPeriodMs = 0;

static void measurePvtCallback(UBX_NAV_PVT_data_t *pvt) {
  epochCount++;
  lastITOW = pvt->iTOW;
  lastFixType = pvt->fixType;

  if (pvt->numSV < minNumSV) {
    minNumSV = pvt->numSV;
  }
  if (pvt->numSV > maxNumSV) {
    maxNumSV = pvt->numSV;
  }
  sumNumSV += pvt->numSV;
  lastNumSV = pvt->numSV;
  svHistogram[pvt->numSV < SV_HISTOGRAM_BINS ? pvt->numSV
                                             : SV_HISTOGRAM_BINS - 1]++;

  if (!haveFirstEpoch) {
    haveFirstEpoch = true;
    firstITOW = pvt->iTOW;
    prevITOW = pvt->iTOW;
    return;
  }

  // iTOW is GPS time of week in ms and wraps at the end of the week. An
  // apparent backwards step means a wrap or a time resync, not a gap - treat
  // it as a resync point rather than letting it poison the histogram.
  if (pvt->iTOW <= prevITOW) {
    irregularGaps++;
    prevITOW = pvt->iTOW;
    return;
  }

  uint32_t delta = pvt->iTOW - prevITOW;
  prevITOW = pvt->iTOW;

  if (epochPeriodMs == 0 || (delta % epochPeriodMs) != 0) {
    irregularGaps++;
    return;
  }

  uint32_t periods = delta / epochPeriodMs;
  skippedEpochs += periods - 1;
  if (periods >= 4) {
    gapHistogram[4]++;
  } else {
    gapHistogram[periods]++;
  }
}

// Sweep common baud rates until the receiver responds. Returns true and leaves
// gnssSerial/myGNSS connected at the working baud on success.
static bool connectAtAnyBaud() {
  for (int i = 0; i < NUM_BAUD_RATES; i++) {
    uint32_t baud = BAUD_RATES[i];
    Serial.printf("Trying GNSS at %lu baud...\n", (unsigned long)baud);

    GNSS_SERIAL_BEGIN(baud);
    delay(100); // let the serial port stabilize

    if (myGNSS.begin(gnssSerial)) {
      Serial.printf("Connected at %lu baud.\n\n", (unsigned long)baud);
      return true;
    }

    gnssSerial.end();
    delay(100);
  }
  return false;
}

// --- OTP layer poll (phase 3) ----------------------------------------------
// The integration manual verifies the clock on VALGET layer 4 - the OTP layer -
// not on RAM. That distinction turned out to matter: after a successful OTP
// write these keys read back 192/192/192/96 on the Default and OTP layers while
// RAM still reports the stock 128/128/128/64. RAM is therefore NOT the
// authoritative answer for these keys, and the manual's layer-4 poll is.
//
// Layer 4 isn't in the public interface description and the SparkFun library
// has no constant for it, so the poll and its expected reply go out and get
// compared as raw bytes, exactly as the manual prints them.
static const uint8_t OTP_VERIFY_POLL[] = {
    0xB5, 0x62, 0x06, 0x8B, 0x14, 0x00, 0x00, 0x04, 0x00, 0x00,
    0x01, 0x00, 0xA4, 0x40, 0x03, 0x00, 0xA4, 0x40, 0x05, 0x00,
    0xA4, 0x40, 0x0A, 0x00, 0xA4, 0x40, 0x4C, 0x15};

static const uint8_t OTP_VERIFY_EXPECTED[] = {
    0xB5, 0x62, 0x06, 0x8B, 0x24, 0x00, 0x01, 0x04, 0x00, 0x00, 0x01,
    0x00, 0xA4, 0x40, 0x00, 0xB0, 0x71, 0x0B, 0x03, 0x00, 0xA4, 0x40,
    0x00, 0xB0, 0x71, 0x0B, 0x05, 0x00, 0xA4, 0x40, 0x00, 0xB0, 0x71,
    0x0B, 0x0A, 0x00, 0xA4, 0x40, 0x00, 0xD8, 0xB8, 0x05, 0x76, 0x81};

static void drainSerial() {
  while (gnssSerial.available()) {
    gnssSerial.read();
  }
}

// Discard everything arriving for windowMs, then clear the buffer.
//
// This exists because of a specific hazard. A UBX-CFG-VALGET poll is answered
// by the reply frame AND a UBX-ACK-ACK for class/id 0x06 0x8B. The SparkFun
// library's own getVal() is *also* UBX-CFG-VALGET - the same 0x06 0x8B - so a
// leftover ACK from a hand-rolled poll looks exactly like the acknowledgement
// of the library's next request. The library accepts it, returns the response
// already sitting in its buffer, and from then on every read comes back one
// key behind: phase 4 reports a failed CFG-RATE read, constellations and
// message rates all shift by one entry, and the numbers look plausible enough
// to be believed. Consume the ACK before handing the port back.
static void settleAndDrain(uint32_t windowMs) {
  uint32_t deadline = millis() + windowMs;
  while ((int32_t)(millis() - deadline) < 0) {
    while (gnssSerial.available()) {
      gnssSerial.read();
    }
    delay(1);
  }
}

// Read UBX frames until one matches cls/id or the timeout expires. Syncs on a
// rolling 0xB5 0x62 pair (a payload byte can be 0xB5), verifies each checksum,
// and consumes oversized frames rather than resyncing inside their payloads.
static bool waitForUbxFrame(uint8_t cls, uint8_t id, uint8_t *out,
                            size_t maxLen, size_t *outLen, uint32_t timeoutMs) {
  uint32_t deadline = millis() + timeoutMs;
  uint8_t prev = 0;

  while ((int32_t)(millis() - deadline) < 0) {
    if (!gnssSerial.available()) {
      delay(1);
      continue;
    }

    uint8_t current = (uint8_t)gnssSerial.read();
    if (!(prev == 0xB5 && current == 0x62)) {
      prev = current;
      continue;
    }
    prev = 0;

    uint8_t header[4];
    for (int i = 0; i < 4; i++) {
      while (!gnssSerial.available() && (int32_t)(millis() - deadline) < 0) {
        delay(1);
      }
      if (!gnssSerial.available()) {
        return false;
      }
      header[i] = gnssSerial.read();
    }

    uint16_t payloadLen = (uint16_t)header[2] | ((uint16_t)header[3] << 8);
    size_t frameLen = 8 + payloadLen;
    bool tooBig = frameLen > maxLen;

    if (!tooBig) {
      out[0] = 0xB5;
      out[1] = 0x62;
      memcpy(out + 2, header, 4);
    }

    for (size_t i = 0; i < (size_t)payloadLen + 2; i++) {
      while (!gnssSerial.available() && (int32_t)(millis() - deadline) < 0) {
        delay(1);
      }
      if (!gnssSerial.available()) {
        return false;
      }
      uint8_t b = gnssSerial.read();
      if (!tooBig) {
        out[6 + i] = b;
      }
    }
    if (tooBig) {
      continue;
    }

    uint8_t ckA = 0;
    uint8_t ckB = 0;
    for (size_t i = 2; i < 6 + (size_t)payloadLen; i++) {
      ckA = (uint8_t)(ckA + out[i]);
      ckB = (uint8_t)(ckB + ckA);
    }
    if (ckA != out[6 + payloadLen] || ckB != out[7 + payloadLen]) {
      continue;
    }

    if (header[0] == cls && header[1] == id) {
      *outLen = frameLen;
      return true;
    }
  }

  return false;
}

// --- Phase 1: raw UBX-MON-VER ----------------------------------------------
// The library's getModuleInfo() keeps only the four fields it knows how to
// parse. Polling MON-VER ourselves gets the whole reply, including the
// extension strings that carry FWVER / PROTVER / MOD and the supported-GNSS
// list. Payload layout: 30-byte swVersion, 10-byte hwVersion, then N 30-byte
// extension strings.
static void printRawMonVer() {
  Serial.println("--- Phase 1: raw UBX-MON-VER ---");

  uint8_t customPayload[MAX_PAYLOAD_SIZE];
  ubxPacket customCfg = {0,
                         0,
                         0,
                         0,
                         0,
                         customPayload,
                         0,
                         0,
                         SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED,
                         SFE_UBLOX_PACKET_VALIDITY_NOT_DEFINED};

  customCfg.cls = UBX_CLASS_MON;
  customCfg.id = UBX_MON_VER;
  customCfg.len = 0; // zero length = poll the current value
  customCfg.startingSpot = 0;

  if (myGNSS.sendCommand(&customCfg, 1100) != SFE_UBLOX_STATUS_DATA_RECEIVED) {
    Serial.println("  MON-VER poll FAILED.");
    Serial.println();
    return;
  }

  // The strings are NULL-padded to their field width, but treat the last byte
  // as a terminator anyway rather than trusting the receiver to pad.
  char field[31];

  memcpy(field, customPayload, 30);
  field[30] = '\0';
  Serial.printf("  swVersion : %s\n", field);

  memcpy(field, customPayload + 30, 10);
  field[10] = '\0';
  Serial.printf("  hwVersion : %s\n", field);

  uint16_t position = 40;
  int extensionNo = 0;
  while (customCfg.len >= position + 30) {
    memcpy(field, customPayload + position, 30);
    field[30] = '\0';
    Serial.printf("  ext[%d]    : %s\n", extensionNo, field);
    position += 30;
    extensionNo++;
  }

  if (extensionNo == 0) {
    Serial.println("  (no extension strings)");
  }
  Serial.println();
}

// --- Phase 2: identity as the library parses it ----------------------------
static void printLibraryModuleInfo() {
  Serial.println("--- Phase 2: identity via the SparkFun library ---");

  if (!myGNSS.getModuleInfo()) {
    Serial.println("  getModuleInfo() FAILED.");
    Serial.println();
    return;
  }

  const char *name = myGNSS.getModuleName();
  const char *type = myGNSS.getFirmwareType();

  Serial.printf("  module    : %s\n",
                (name != nullptr && name[0] != '\0') ? name : "(empty)");
  Serial.printf("  firmware  : %s %u.%02u\n",
                (type != nullptr && type[0] != '\0') ? type : "(empty)",
                myGNSS.getFirmwareVersionHigh(),
                myGNSS.getFirmwareVersionLow());
  Serial.printf("  protver   : %u.%02u\n", myGNSS.getProtocolVersionHigh(),
                myGNSS.getProtocolVersionLow());
  Serial.println("  (protver 34.10 = ROM SPG 5.10, the generation whose");
  Serial.println("   high-rate figures need the OTP clock write; 34.30 =");
  Serial.println("   SPG 5.30, where they are the native figures)");
  Serial.println();
}

// --- Phase 3: CPU clock ----------------------------------------------------
// Returns true if every key read back its high-clock value on the RAM layer.
static bool printClockConfig() {
  Serial.println("--- Phase 3: CPU clock (VALGET, read-only) ---");
  Serial.println("  key         RAM          Default      expected (high)");

  bool allHigh = true;
  bool anyRamReadFailed = false;

  for (int i = 0; i < NUM_CLOCK_KEYS; i++) {
    uint32_t ramValue = 0;
    uint32_t defaultValue = 0;
    bool ramOk = myGNSS.getVal32(CLOCK_KEYS[i].key, &ramValue, VAL_LAYER_RAM);
    bool defaultOk =
        myGNSS.getVal32(CLOCK_KEYS[i].key, &defaultValue, VAL_LAYER_DEFAULT);

    char ramText[16];
    char defaultText[16];
    if (ramOk) {
      snprintf(ramText, sizeof(ramText), "%lu MHz",
               (unsigned long)(ramValue / 1000000UL));
    } else {
      snprintf(ramText, sizeof(ramText), "read failed");
    }
    if (defaultOk) {
      snprintf(defaultText, sizeof(defaultText), "%lu MHz",
               (unsigned long)(defaultValue / 1000000UL));
    } else {
      snprintf(defaultText, sizeof(defaultText), "read failed");
    }

    Serial.printf("  0x%08lX  %-12s %-12s %lu MHz\n",
                  (unsigned long)CLOCK_KEYS[i].key, ramText, defaultText,
                  (unsigned long)(CLOCK_KEYS[i].highClockHz / 1000000UL));

    if (!ramOk) {
      anyRamReadFailed = true;
      allHigh = false;
    } else if (ramValue != CLOCK_KEYS[i].highClockHz) {
      allHigh = false;
    }
  }

  // The authoritative check. A blank OTP has nothing to return, so an
  // unprogrammed receiver simply does not answer this poll; a programmed one
  // returns the manual's frame exactly.
  Serial.println();
  Serial.println("  OTP layer 4 (the manual's own verification poll):");

  drainSerial();
  gnssSerial.write(OTP_VERIFY_POLL, sizeof(OTP_VERIFY_POLL));
  gnssSerial.flush();

  uint8_t reply[64];
  size_t replyLen = 0;
  bool otpProgrammed = false;

  bool gotReply =
      waitForUbxFrame(0x06, 0x8B, reply, sizeof(reply), &replyLen, 2000);

  // Must happen before anything else touches the port - see settleAndDrain().
  settleAndDrain(300);

  if (!gotReply) {
    Serial.println("    no reply - OTP holds no clock configuration.");
  } else if (replyLen == sizeof(OTP_VERIFY_EXPECTED) &&
             memcmp(reply, OTP_VERIFY_EXPECTED, replyLen) == 0) {
    otpProgrammed = true;
    Serial.println("    MATCHES the manual's programmed reply byte-for-byte.");
  } else {
    Serial.print("    unexpected reply: ");
    for (size_t i = 0; i < replyLen; i++) {
      Serial.printf("%02X ", reply[i]);
    }
    Serial.println();
  }

  Serial.println();

  if (otpProgrammed) {
    Serial.println("  VERDICT: HIGH CPU CLOCK PROGRAMMED. The OTP layer returns");
    Serial.println("  the manual's expected frame, which is its own pass");
    Serial.println("  criterion. This receiver is clocked for 25 Hz single-GNSS");
    Serial.println("  / 20 Hz two-GNSS. Do not run gnss_otp_clock against it.");
    if (!allHigh) {
      Serial.println();
      Serial.println("  Note the RAM column above may still read the stock");
      Serial.println("  128/128/128/64. That is expected and is NOT a failure:");
      Serial.println("  for these keys the OTP value surfaces on the Default and");
      Serial.println("  OTP layers, not RAM. It is exactly why the manual");
      Serial.println("  verifies on layer 4 rather than layer 0.");
    }
  } else if (allHigh) {
    Serial.println("  VERDICT: HIGH CPU CLOCK. RAM reads 192/192/192/96 even");
    Serial.println("  though OTP did not answer - unusual, but the receiver is");
    Serial.println("  running the high clock. No OTP write is needed.");
  } else if (anyRamReadFailed) {
    Serial.println("  VERDICT: at least one key would not read. These keys are");
    Serial.println("  undocumented, so a receiver that isn't M10 silicon (or a");
    Serial.println("  firmware that doesn't carry them) will reject the poll.");
    Serial.println("  Check phase 1 before reading anything into this.");
  } else {
    Serial.println("  VERDICT: NOT at the high clock. Max nav rate is the");
    Serial.println("  default row: 18 Hz single-GNSS, 10 Hz two-GNSS. Asking");
    Serial.println("  for more will drop fixes rather than error. Raising it");
    Serial.println("  means the PERMANENT OTP write in integration manual");
    Serial.println("  sec 2.1.7 - see tools/common/gnss_otp_clock.");
  }
  Serial.println();

  return otpProgrammed || (allHigh && !anyRamReadFailed);
}

// --- Phase 4: current rate + constellation config --------------------------
static void printRateConfig(bool highClock) {
  Serial.println("--- Phase 4: current rate + constellations (RAM layer) ---");

  uint16_t measRate = 0;
  uint16_t navRate = 0;
  bool measOk = myGNSS.getMeasurementRate(&measRate, VAL_LAYER_RAM);
  bool navOk = myGNSS.getNavigationRate(&navRate, VAL_LAYER_RAM);

  if (measOk && navOk && measRate > 0 && navRate > 0) {
    // Stash the epoch period for phase 6 to measure gaps against.
    epochPeriodMs = (uint32_t)measRate * (uint32_t)navRate;
    Serial.printf("  CFG-RATE-MEAS : %u ms\n", measRate);
    Serial.printf("  CFG-RATE-NAV  : %u\n", navRate);
    Serial.printf("  nav epoch     : %lu ms (%.2f Hz)\n",
                  (unsigned long)epochPeriodMs, 1000.0f / (float)epochPeriodMs);
  } else {
    Serial.println("  CFG-RATE read FAILED.");
  }

  Serial.println();

  int concurrent = 0;
  for (int i = 0; i < NUM_CONSTELLATIONS; i++) {
    bool enabled = false;
    if (!myGNSS.isGNSSenabled(CONSTELLATIONS[i].id, &enabled, VAL_LAYER_RAM)) {
      Serial.printf("  %-8s : read failed / unsupported\n",
                    CONSTELLATIONS[i].name);
      continue;
    }
    Serial.printf("  %-8s : %s%s\n", CONSTELLATIONS[i].name,
                  enabled ? "enabled" : "disabled",
                  CONSTELLATIONS[i].countsTowardRate ? "" : " (augmentation)");
    if (enabled && CONSTELLATIONS[i].countsTowardRate) {
      concurrent++;
    }
  }

  Serial.println();
  Serial.printf("  concurrent GNSS: %d\n", concurrent);

  if (concurrent >= 1 && concurrent <= 4) {
    uint8_t rated = highClock ? RATE_HIGH_CLOCK[concurrent]
                              : RATE_DEFAULT_CLOCK[concurrent];
    Serial.printf("  rated max nav rate at this clock: %u Hz\n", rated);
    Serial.println("  (u-blox UBX-23006557; spec'd for a >=98% fix rate, and");
    Serial.println("   a sky full of BeiDou can still pull it down)");
  } else if (concurrent == 0) {
    Serial.println("  No constellations enabled - nothing to rate.");
  } else {
    Serial.println("  More than 4 concurrent GNSS - off the published table.");
  }
  Serial.println();
}

// --- Phase 6: measured fix rate --------------------------------------------
static void printMeasuredFixRate() {
  Serial.println("--- Phase 6: measured fix rate ---");

  if (MEASURE_SECONDS == 0) {
    Serial.println("  MEASURE_SECONDS is 0 - skipped, run stays read-only.");
    Serial.println();
    return;
  }

  if (epochPeriodMs == 0) {
    Serial.println("  Skipped: phase 4 could not read the epoch period, so");
    Serial.println("  there is nothing to measure gaps against.");
    Serial.println();
    return;
  }

  Serial.printf("  Enabling UBX-NAV-PVT on the RAM layer (the only write this "
                "sketch makes;\n  a power cycle clears it), then counting for "
                "%lu s.\n",
                (unsigned long)MEASURE_SECONDS);

  if (!myGNSS.setAutoPVTcallbackPtr(&measurePvtCallback, VAL_LAYER_RAM)) {
    Serial.println("  FAILED to enable NAV-PVT output. Nothing to count.");
    Serial.println();
    return;
  }

  Serial.println("  Needs sky view - a receiver that never fixes still emits");
  Serial.println("  epochs, but the SV stats below are what tell you whether a");
  Serial.println("  low number is the sky or the clock.");
  Serial.println();

  uint32_t startMs = millis();
  uint32_t windowMs = MEASURE_SECONDS * 1000UL;
  uint32_t nextReportMs = startMs + MEASURE_REPORT_EVERY_MS;

  while (millis() - startMs < windowMs) {
    myGNSS.checkUblox();     // parse whatever has arrived on the UART
    myGNSS.checkCallbacks(); // dispatch measurePvtCallback for each new epoch
    delay(1);                // yield; the UART buffer holds several epochs

    if ((int32_t)(millis() - nextReportMs) >= 0) {
      uint32_t elapsedS = (millis() - startMs) / 1000UL;
      Serial.printf(
          "  ... %lu s: %lu epochs, %lu skipped, %u SVs (max %u), fixType %u\n",
          (unsigned long)elapsedS, (unsigned long)epochCount,
          (unsigned long)skippedEpochs, lastNumSV, maxNumSV, lastFixType);
      nextReportMs += MEASURE_REPORT_EVERY_MS;
    }
  }

  Serial.println();

  if (epochCount == 0) {
    Serial.println("  No NAV-PVT epochs received at all. Either the receiver");
    Serial.println("  isn't emitting them or the port isn't carrying UBX.");
    Serial.println();
    return;
  }

  // Measure against the iTOW span rather than wall-clock millis(): the span is
  // the receiver's own view of how much time the epochs covered, and it isn't
  // affected by where in an epoch the window happened to open and close.
  uint32_t spanMs = (lastITOW > firstITOW) ? (lastITOW - firstITOW) : 0;

  if (epochCount < 10 || spanMs == 0) {
    Serial.printf("  Only %lu epoch(s) over a %lu ms span - too few to draw a\n",
                  (unsigned long)epochCount, (unsigned long)spanMs);
    Serial.println("  fix rate from. Check the receiver is emitting NAV-PVT and");
    Serial.println("  that MEASURE_SECONDS is long enough for the epoch period.");
    Serial.println();
    return;
  }

  // Both divisors are non-zero by the guard above. Note that epochs dropped at
  // the very end of the window shrink the span with them, so this is a mildly
  // optimistic estimate rather than a pessimistic one.
  uint32_t expectedEpochs = (spanMs / epochPeriodMs) + 1;
  float fixRatePct = 100.0f * (float)epochCount / (float)expectedEpochs;
  float measuredHz = 1000.0f * (float)(epochCount - 1) / (float)spanMs;

  Serial.printf("  window (iTOW span) : %.1f s\n", (float)spanMs / 1000.0f);
  Serial.printf("  epochs received    : %lu\n", (unsigned long)epochCount);
  Serial.printf("  epochs expected    : %lu\n", (unsigned long)expectedEpochs);
  Serial.printf("  epochs skipped     : %lu\n", (unsigned long)skippedEpochs);
  Serial.printf("  fix rate           : %.2f%%\n", fixRatePct);
  Serial.printf("  effective rate     : %.2f Hz (configured %.2f Hz)\n",
                measuredHz, 1000.0f / (float)epochPeriodMs);
  Serial.println();

  Serial.println("  iTOW gap histogram:");
  Serial.printf("    %lu ms  (clean)        : %lu\n",
                (unsigned long)epochPeriodMs, (unsigned long)gapHistogram[1]);
  Serial.printf("    %lu ms  (1 skipped)    : %lu\n",
                (unsigned long)(epochPeriodMs * 2),
                (unsigned long)gapHistogram[2]);
  Serial.printf("    %lu ms  (2 skipped)    : %lu\n",
                (unsigned long)(epochPeriodMs * 3),
                (unsigned long)gapHistogram[3]);
  Serial.printf("    >=%lu ms (3+ skipped)  : %lu\n",
                (unsigned long)(epochPeriodMs * 4),
                (unsigned long)gapHistogram[4]);
  Serial.printf("    irregular / resync     : %lu\n",
                (unsigned long)irregularGaps);
  Serial.println();

  Serial.printf("  satellites   : min %u, max %u, mean %.1f, median %u\n",
                minNumSV, maxNumSV, (float)sumNumSV / (float)epochCount,
                medianNumSV());
  Serial.printf("  last fixType : %u (0 no fix, 2 = 2D, 3 = 3D)\n",
                lastFixType);
  Serial.println();

  // A fix rate means nothing without knowing what load produced it. The
  // navigation solution is what the CPU clock limits, and with no fix there is
  // no solution being computed - a receiver emitting empty epochs at 20 Hz will
  // hit 100% on any clock, which says nothing about whether it could hold that
  // rate under a real sky. Gate the verdict on the load actually being there.
  bool haveFix = lastFixType >= 2;
  uint8_t medianSV = medianNumSV();

  if (!haveFix) {
    Serial.println("  INCONCLUSIVE: no position fix during the window, so the");
    Serial.println("  receiver was emitting empty epochs and never ran a real");
    Serial.println("  navigation solution. 100% here is the cheapest possible");
    Serial.println("  case and proves nothing about the rate limit. Get the");
    Serial.println("  antenna a clear view of the sky and run it again.");
  } else if (medianSV < MEASURE_MIN_SV_FOR_VERDICT) {
    Serial.printf("  INCONCLUSIVE: fixed, but on a median of only %u\n",
                  medianSV);
    Serial.printf("  satellites. The manual attributes rate loss to \"a very\n"
                  "  large number of satellites\", so a thin sky is the easy\n"
                  "  case and a clean result here does not generalise. Re-run\n"
                  "  with a median of at least %d SVs to exercise the limit.\n",
                  MEASURE_MIN_SV_FOR_VERDICT);
  } else if (fixRatePct >= 98.0f) {
    Serial.println("  VERDICT: >=98% fix rate - the receiver is keeping up with");
    Serial.println("  what config.h asks of it. That is u-blox's own criterion");
    Serial.println("  for a rated figure, so even an out-of-rating setting is");
    Serial.println("  costing nothing measurable at this satellite count.");
  } else if (fixRatePct >= 90.0f) {
    Serial.println("  VERDICT: measurable loss. Under 98% but over 90% - the");
    Serial.println("  configured rate is more than this receiver reliably");
    Serial.println("  delivers here. Check the histogram: scattered single");
    Serial.println("  skips degrade gracefully, a dominant 2x bucket means it");
    Serial.println("  is effectively running at half the configured rate.");
  } else {
    Serial.println("  VERDICT: substantial loss. The configured rate is well");
    Serial.println("  beyond what this receiver is delivering. If the SV counts");
    Serial.println("  above are healthy, this is the clock rather than the sky -");
    Serial.println("  compare against phase 3's verdict and phase 4's rating.");
  }
  Serial.println();
}

// --- Phase 5: message output inventory -------------------------------------
static void printMessageOutputs() {
  Serial.println("--- Phase 5: enabled message output (RAM layer) ---");

  // The port's protocol filter gates output independently of the per-message
  // rates, so report it first - it is what makes a non-zero NMEA rate below
  // harmless rather than alarming.
  uint8_t nmeaOut = 0;
  uint8_t ubxOut = 0;
  bool nmeaOk =
      myGNSS.getVal8(UBLOX_CFG_UART1OUTPROT_NMEA, &nmeaOut, VAL_LAYER_RAM);
  bool ubxOk =
      myGNSS.getVal8(UBLOX_CFG_UART1OUTPROT_UBX, &ubxOut, VAL_LAYER_RAM);

  Serial.printf("  UART1 output protocols: UBX %s, NMEA %s\n",
                ubxOk ? (ubxOut ? "on" : "off") : "read failed",
                nmeaOk ? (nmeaOut ? "on" : "off") : "read failed");

  // Self-consistency check. Every value in this phase arrives as a UBX frame -
  // including the read that just claimed UBX output is off. That is a
  // contradiction the receiver cannot actually be in, so it is proof the reads
  // themselves are wrong rather than a real configuration.
  //
  // The way this happens is a response lag: a hand-rolled UBX-CFG-VALGET poll
  // leaves a trailing UBX-ACK-ACK for 0x06 0x8B in the buffer, the library's
  // own getVal() is also 0x06 0x8B, and it accepts the stale ACK as its own -
  // returning the previous response to every later request. See
  // settleAndDrain(). The values stay individually plausible, which is exactly
  // why this check is worth making: without it, a shifted-by-one inventory
  // reads as a real answer.
  bool ubxContradiction = ubxOk && ubxOut == 0;

  if (ubxContradiction) {
    Serial.println();
    Serial.println("  *** INCONSISTENT READ - DO NOT TRUST THIS PHASE ***");
    Serial.println("  UBX output reads as off, but this reply arrived over UBX,");
    Serial.println("  so it cannot be. The reads are lagging behind the");
    Serial.println("  requests - every value below is probably the previous");
    Serial.println("  key's. Phases 3 and 4 are suspect for the same reason.");
    Serial.println("  Re-run; if it persists, the port is desynchronised and");
    Serial.println("  something polled it outside the library without draining");
    Serial.println("  the trailing ACK.");
  }

  // Gate on what is demonstrably true rather than on the suspect read, so a
  // desync doesn't also mislabel every UBX message as [protocol off].
  bool ubxActuallyOn = ubxContradiction || (ubxOk && ubxOut != 0);

  Serial.println();

  Serial.printf("  Polling %d CFG-MSGOUT-*-UART1 keys...\n",
                NUM_MESSAGE_OUT_KEYS);

  int enabled = 0;
  int readFailed = 0;

  for (int i = 0; i < NUM_MESSAGE_OUT_KEYS; i++) {
    uint8_t rate = 0;
    if (!myGNSS.getVal8(MESSAGE_OUT_KEYS[i].key, &rate, VAL_LAYER_RAM)) {
      readFailed++;
      continue;
    }
    if (rate == 0) {
      continue;
    }

    bool gated = false;
    if (strncmp(MESSAGE_OUT_KEYS[i].name, "NMEA", 4) == 0 ||
        strncmp(MESSAGE_OUT_KEYS[i].name, "PUBX", 4) == 0) {
      gated = nmeaOk && nmeaOut == 0;
    } else {
      gated = !ubxActuallyOn;
    }

    Serial.printf("    %-18s rate %u%s\n", MESSAGE_OUT_KEYS[i].name, rate,
                  gated ? "   [protocol off]" : "");
    enabled++;
  }

  Serial.println();
  Serial.println("  (rate is a divisor on the navigation epoch, not a "
                 "frequency:");
  Serial.println("   1 = every epoch, 5 = every fifth)");
  Serial.println();
  Serial.printf("  %d message(s) with a non-zero rate", enabled);
  if (readFailed > 0) {
    Serial.printf(", %d key(s) not supported by this firmware", readFailed);
  }
  Serial.println(".");

  if (enabled == 0) {
    Serial.println("  Nothing periodic is configured on UART1.");
  } else {
    Serial.println("  Anything marked [protocol off] is configured but not");
    Serial.println("  reaching the wire - the port filter is suppressing it.");
    Serial.println("  Whether the receiver still composes those messages before");
    Serial.println("  discarding them is not documented; if you care about the");
    Serial.println("  power cost, zero their rates rather than relying on the");
    Serial.println("  filter alone.");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(5000); // give USB CDC time to enumerate before the first print

  Serial.println("\n=== GNSS identity + high-rate capability report ===");
  Serial.println("Read-only: this sketch writes nothing to the receiver.\n");

  myGNSS.setPacketCfgPayloadSize(MAX_PAYLOAD_SIZE);

  if (!connectAtAnyBaud()) {
    Serial.println("\nFAILED: u-blox GNSS not detected at any standard baud "
                   "rate. Check your wiring.");
    while (1)
      delay(100); // Halt
  }

  // Let the receiver settle, then drain any backlog before polling - same
  // lesson as g_gnss.cpp's gnssBegin().
  delay(500);
  while (gnssSerial.available()) {
    gnssSerial.read();
  }

  printRawMonVer();
  printLibraryModuleInfo();
  bool highClock = printClockConfig();
  printRateConfig(highClock); // also stashes epochPeriodMs for phase 6
  printMessageOutputs();
  printMeasuredFixRate();

  Serial.println("Done. Halting.");
}

void loop() {
  delay(1000); // Nothing to do - setup() already halted logically.
}
