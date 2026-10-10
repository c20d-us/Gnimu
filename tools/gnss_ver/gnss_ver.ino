// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

// ============================================================================
// TOOL: GNSS identity + high-rate capability report
//
// Connects to the receiver (sweeping common baud rates) and prints what it
// actually is, whether it is clocked for the nav rate Config.h asks for, and
// how it is currently configured.
//
// Strictly read-only: nothing here writes a setting, writes OTP or issues a
// reset.
//
// WHY THIS EXISTS
//
// The Gnimu documentation only ever says "u-blox M10 chipset", because
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
// Gnimu defaults to kGnssNavRateHz 20 with GPS + Galileo - two constellations,
// which is a high-clock number. If a board were at the default clock it would
// cap at 10 Hz and simply drop fixes when asked for 20, rather than refuse.
// Phase 3 below reads the clock back so that question is answered by the
// hardware instead of inferred from how smooth the stream looks.
//
// Measured on hardware: every M100 board checked is a bare UBX-M10050-KB,
// ROM SPG 5.10, PROTVER 34.10, and every unprogrammed one reads
// 128/128/128/64 MHz on RAM and Default with no OTP reply at all - so the
// default configuration asks 20 Hz of a receiver rated for 10 Hz on two
// constellations. The manual permits that ("the navigation update rate can be
// increased beyond the maximum value stated in the datasheet. However, this may
// result in a reduced fix rate"), so the receiver accepts the setting and the
// rating is the only warning that it may not keep up.
//
// After gnss_otp_clock, which behaves identically on the M100 Mini and the
// M100-5883, a receiver reads 192/192/192/96 on the Default and OTP layers and
// returns the manual's expected OTP frame - but RAM still reports
// 128/128/128/64. That is the layer distinction described under phase 3, and
// the reason this sketch judges on the OTP poll.
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
//            can be configured and still never reach the wire. Marked
//            [protocol off] where that applies.
//
//            This phase also self-checks: its own replies arrive as UBX
//            frames, so a receiver reporting UBX output disabled is telling
//            an impossible story and the reads are lagging rather than the
//            config being unusual. That case is called out loudly, because a
//            shifted-by-one inventory otherwise reads as a real answer.
//
// ADAPTING IT
//
// If phase 3 comes back at the default clock and you decide to program the
// high clock, that is a separate tool: tools/gnss_otp_clock. It is kept
// separate on purpose - this sketch is a diagnostic you can run against a
// working board any time, and the OTP write is PERMANENT and cannot be
// reverted. Do not fold the write in here.
//
// PASS CRITERIA
//
// Phase 1 identifies the part. Phase 3 passes when the OTP-layer poll returns
// the manual's frame byte-for-byte - that is the manual's own criterion, and it
// holds even when the RAM column still shows the stock clock. Phase 4's rated
// ceiling should then be >= kGnssNavRateHz.
//
// Requires: the GNSS wired as the board's build guide describes, USB for
// serial, and the SparkFun_u-blox_GNSS_v3 library installed. Open the serial
// monitor at 115200.
// ============================================================================

// UBX-MON-VER can return up to ~348 bytes. This must be defined before the
// library header so the config packet buffer is sized to hold a full reply.
#define MAX_PAYLOAD_SIZE 384

#include <Arduino.h>

// --- Platform wiring -------------------------------------------------------
// The only architecture-dependent part of this sketch: which UART object the
// receiver hangs off, and whether opening it needs pin arguments.
#if defined(ARDUINO_ARCH_ESP32)

// ESP32: UART2 on the GPIOs the Gnimu firmware uses. Change these to match
// your wiring if you routed the receiver elsewhere.
static const int GNSS_RX_PIN = 16; // ESP32 RX <- GNSS TX
static const int GNSS_TX_PIN = 17; // ESP32 TX -> GNSS RX
static HardwareSerial gnssSerial(2);
#define GNSS_SERIAL_BEGIN(baud)                                                \
  gnssSerial.begin((baud), SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN)

#else

// XIAO nRF52840: Serial (USB CDC) needs the TinyUSB library linked,
// and this sketch pulls in nothing else that would include it transitively.
#include <Adafruit_TinyUSB.h>

// Serial1 = D6 (TX) / D7 (RX), fixed on the XIAO - the pins the Gnimu firmware
// uses.
static Uart &gnssSerial = Serial1;
#define GNSS_SERIAL_BEGIN(baud) gnssSerial.begin(baud)

// GNSS regulator enable pin (kGnssEnablePin in the firmware's board config).
// Left alone deliberately: the regulator's own EN pull-up holds the rail on,
// and the pin is an input at reset, so an untouched pin means a powered
// receiver. Driving it here would only matter if a prior firmware run had left
// it low, which a reset already undoes.

#endif

#include <SparkFun_u-blox_GNSS_v3.h>

static SFE_UBLOX_GNSS_SERIAL myGNSS;

// Common u-blox baud rates. 115200 first - the Gnimu firmware's default,
// kGnssBaud in Config.h - then the factory default and the rest.
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

// Constellations. QZSS and SBAS are
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
// (CFG-UART1OUTPROT-NMEA / -UBX) gates output independently. Both are
// reported below.
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
    Serial.println("  sec 2.1.7 - see tools/gnss_otp_clock.");
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
    uint32_t epochPeriodMs = (uint32_t)measRate * (uint32_t)navRate;
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

  // Let the receiver settle, then drain any backlog before polling, so stale
  // bytes can't be taken for a reply.
  delay(500);
  while (gnssSerial.available()) {
    gnssSerial.read();
  }

  printRawMonVer();
  printLibraryModuleInfo();
  bool highClock = printClockConfig();
  printRateConfig(highClock);
  printMessageOutputs();

  Serial.println("Done. Halting.");
}

void loop() {
  delay(1000); // Nothing to do - setup() already halted logically.
}
