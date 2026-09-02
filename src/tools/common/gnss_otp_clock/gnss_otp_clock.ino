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
// TOOL: program the u-blox M10 high-performance CPU clock into OTP
//
// *** THIS MAKES A PERMANENT, IRREVERSIBLE CHANGE TO THE RECEIVER. ***
//
// u-blox ships M10 silicon at a reduced CPU clock to save power. At that stock
// clock the maximum navigation update rate is the "Default" row below; the row
// every M10 spec sheet quotes is the "High" one, and the datasheets footnote it
// as "Configuration required":
//
//   Constellations enabled        1       2       3       4
//   ---------------------------------------------------------
//   Default CPU clock          18 Hz   10 Hz   10 Hz    5 Hz
//   High CPU clock             25 Hz   20 Hz   16 Hz   10 Hz
//
// The configuration in question is a one-time write of a higher CPU clock rate
// into the receiver's OTP (one-time programmable) memory, per MAX-M10S
// Integration manual UBX-20053088 sec 2.1.7. It moves three clocks from 128 to
// 192 MHz and one from 64 to 96 MHz, costs 18 of the receiver's 64 bytes of OTP
// space, and CANNOT BE UNDONE. Read `gnss_ver`'s output before running this,
// and test burn a spare module first if you have one.
//
// This sketch follows the manual's procedure exactly:
//
//   1. Connect, silence the receiver's periodic output, and poll UBX-MON-VER
//      (the manual's own interface check).
//   2. Read the four clock keys back - from RAM, from the firmware Default
//      layer, and from OTP layer 4, which is what the manual's verification
//      step polls.
//   3. If the high clock is ALREADY programmed: report and halt. No write.
//      If the receiver doesn't look like an M10 at the stock clock: report and
//      halt. No write. The write only happens on the exact case it is for.
//   4. Otherwise, wait for the operator to type the confirmation word. Any
//      other input aborts.
//   5. Send the two UBX-CFG (0x06 0x41) frames from Table 8 and check for the
//      two ACK-ACKs the manual specifies.
//   6. Hardware-reset the receiver so the new clock is applied at startup.
//   7. Re-connect and re-read all three layers, comparing the OTP-layer reply
//      byte-for-byte against the frame the manual says to expect.
//
// Every byte string below was checked against the manual and its UBX checksums
// recomputed independently before being pasted here.
//
// A NOTE ON QUIETING THE RECEIVER
//
// The sketch silences periodic output before doing anything else. The reason is
// narrow: there is exactly one window where it cannot drain the UART, and that
// is the blocking wait for the operator to type the confirmation word, which
// lasts as long as they take to read the warning. A Gnimu-configured receiver
// pushes ~2 KB/s, so a 256-byte RX ring (the nRF52 core's compile-time size)
// overflows in about 130 ms of wait. The overflow is not itself fatal,
// since the buffer is drained before the write, but a UART left in an overrun
// state is not what anyone wants immediately before an irreversible operation.
//
// Quieting also means the ACK and VALGET replies arrive in an otherwise empty
// stream, so the frame scanner never has to resync past unrelated traffic.
//
// See quietReceiver() for what is turned off and why UBX output has to stay on.
//
// A NOTE ON THE RESET
//
// The manual says to power-cycle or send UBX-CFG-RST. This sketch sends its own
// CFG-RST with navBbrMask = 0x0000 (hot start) rather than calling the SparkFun
// library's hardReset(), which uses 0xFFFF and would additionally wipe BBR,
// discarding ephemeris for a cold-start TTFF and dropping the saved baud rate.
// Neither is needed to apply an OTP setting: OTP is read at startup regardless.
// resetMode is 0x00, the hardware (watchdog) reset the manual asks for.
//
// The receiver does not ACK CFG-RST, but reset immediately, so the sketch
// re-sweeps the baud rates afterwards rather than assuming the link survived.
//
// PASS CRITERIA
//
// The OTP-layer reply matches the manual's expected frame byte-for-byte. That
// is the manual's own verification step and the authoritative result. The
// Default layer follows it to 192/192/192/96; the RAM layer does NOT, and keeps
// reporting the stock 128/128/128/64 even after a fully successful write, so
// RAM must not be used to judge the outcome.
//
// Requires: the GNSS wired as the variant's README describes, USB for serial,
// and the SparkFun_u-blox_GNSS_v3 library installed. Open the serial monitor at
// 115200 with a newline-sending line ending.
// ============================================================================

// UBX-MON-VER can return up to ~348 bytes. Must be defined before the library
// header so the config packet buffer is sized to hold a full reply.
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
// A larger RX ring buys slack for the one place this sketch can't drain the
// port: the blocking wait for the operator's confirmation. Must be called
// before begin(). The nRF52 core has no equivalent - its buffer is a
// compile-time constant - which is the other reason the receiver gets quieted
// rather than this being relied on.
#define GNSS_SERIAL_PREPARE() gnssSerial.setRxBufferSize(1024)
#define GNSS_SERIAL_BEGIN(baud)                                                \
  gnssSerial.begin((baud), SERIAL_8N1, GNSS_RX_PIN, GNSS_TX_PIN)

#else

// nRF52840 (XIAO) variants: Serial (USB CDC) needs the TinyUSB library linked,
// and this sketch pulls in nothing else that would include it transitively.
#include <Adafruit_TinyUSB.h>

// Serial1 = D6 (TX) / D7 (RX), fixed on the nRF52.
static Uart &gnssSerial = Serial1;
#define GNSS_SERIAL_PREPARE() ((void)0)
#define GNSS_SERIAL_BEGIN(baud) gnssSerial.begin(baud)

#endif

#include <SparkFun_u-blox_GNSS_v3.h>

static SFE_UBLOX_GNSS_SERIAL myGNSS;

// The word the operator must type to authorize the burn. Anything else aborts.
static const char CONFIRM_WORD[] = "BURN";

// Common u-blox baud rates. 115200 first (GNSS_BAUD in every variant's
// config.h), then the factory default, then the rest.
static const uint32_t BAUD_RATES[] = {115200, 38400,  9600,
                                      57600,  230400, 460800};
static const int NUM_BAUD_RATES = sizeof(BAUD_RATES) / sizeof(BAUD_RATES[0]);

// --- Byte strings from MAX-M10S Integration manual UBX-20053088 sec 2.1.7 ---

// Table 8, "High CPU clock". Two UBX-CFG frames, class 0x06 id 0x41 - an
// undocumented message the SparkFun library has no support for, so these go out
// as raw bytes.
static const uint8_t OTP_WRITE_FRAME_1[] = {
    0xB5, 0x62, 0x06, 0x41, 0x10, 0x00, 0x03, 0x00, 0x04, 0x1F, 0x54, 0x5E,
    0x79, 0xBF, 0x28, 0xEF, 0x12, 0x05, 0xFD, 0xFF, 0xFF, 0xFF, 0x8F, 0x0D};
static const uint8_t OTP_WRITE_FRAME_2[] = {
    0xB5, 0x62, 0x06, 0x41, 0x1C, 0x00, 0x04, 0x01, 0xA4, 0x10, 0xBD, 0x34,
    0xF9, 0x12, 0x28, 0xEF, 0x12, 0x05, 0x05, 0x00, 0xA4, 0x40, 0x00, 0xB0,
    0x71, 0x0B, 0x0A, 0x00, 0xA4, 0x40, 0x00, 0xD8, 0xB8, 0x05, 0xDE, 0xAE};

// Step 5's verification poll: UBX-CFG-VALGET, layer 4 (OTP), the four clock
// keys.
static const uint8_t OTP_VERIFY_POLL[] = {
    0xB5, 0x62, 0x06, 0x8B, 0x14, 0x00, 0x00, 0x04, 0x00, 0x00,
    0x01, 0x00, 0xA4, 0x40, 0x03, 0x00, 0xA4, 0x40, 0x05, 0x00,
    0xA4, 0x40, 0x0A, 0x00, 0xA4, 0x40, 0x4C, 0x15};

// The reply the manual says a correctly programmed receiver returns. Compared
// byte-for-byte after the burn.
static const uint8_t OTP_VERIFY_EXPECTED[] = {
    0xB5, 0x62, 0x06, 0x8B, 0x24, 0x00, 0x01, 0x04, 0x00, 0x00, 0x01,
    0x00, 0xA4, 0x40, 0x00, 0xB0, 0x71, 0x0B, 0x03, 0x00, 0xA4, 0x40,
    0x00, 0xB0, 0x71, 0x0B, 0x05, 0x00, 0xA4, 0x40, 0x00, 0xB0, 0x71,
    0x0B, 0x0A, 0x00, 0xA4, 0x40, 0x00, 0xD8, 0xB8, 0x05, 0x76, 0x81};

// UBX-CFG-RST, navBbrMask 0x0000 (hot start), resetMode 0x00 (hardware reset).
// See the header for why this rather than the library's hardReset().
static const uint8_t CFG_RST_HARDWARE[] = {0xB5, 0x62, 0x06, 0x04, 0x04, 0x00,
                                           0x00, 0x00, 0x00, 0x00, 0x0E, 0x64};

// --- The four clock keys ---------------------------------------------------
struct ClockKey {
  uint32_t key;
  uint32_t stockHz;
  uint32_t highHz;
};
static const ClockKey CLOCK_KEYS[] = {
    {0x40A40001UL, 128000000UL, 192000000UL},
    {0x40A40003UL, 128000000UL, 192000000UL},
    {0x40A40005UL, 128000000UL, 192000000UL},
    {0x40A4000AUL, 64000000UL, 96000000UL},
};
static const int NUM_CLOCK_KEYS = sizeof(CLOCK_KEYS) / sizeof(CLOCK_KEYS[0]);

// What the RAM layer says the receiver is running.
enum ClockState { CLOCK_HIGH, CLOCK_STOCK, CLOCK_UNKNOWN };

// --- Raw UBX helpers -------------------------------------------------------
// quietReceiver() silences the periodic stream first, but these still parse
// the byte stream into whole frames rather than pattern-matching a buffer:
// quieting is best-effort (an unexpected message may not be in the list), and
// the scanner has to be right whether or not it worked.

static void drainSerial() {
  while (gnssSerial.available()) {
    gnssSerial.read();
  }
}

static void sendRaw(const uint8_t *frame, size_t len) {
  gnssSerial.write(frame, len);
  gnssSerial.flush();
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

// Read UBX frames until one matches cls/id or the timeout expires. On a match,
// copies the whole frame (sync bytes through checksum) into out and sets
// outLen. Frames failing their checksum are discarded and the scan continues.
static bool waitForUbxFrame(uint8_t cls, uint8_t id, uint8_t *out,
                            size_t maxLen, size_t *outLen, uint32_t timeoutMs) {
  uint32_t deadline = millis() + timeoutMs;

  // Rolling sync byte, so a 0xB5 that turns out not to be followed by 0x62 can
  // itself be the start of the real frame on the next byte. A payload byte can
  // be 0xB5, and this is a one-shot irreversible tool - a scanner that resyncs
  // sloppily could miss the one reply that matters.
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
    prev = 0; // consumed the sync pair

    // Header: class, id, length.
    uint8_t header[4];
    for (int i = 0; i < 4; i++) {
      while (!gnssSerial.available() && (int32_t)(millis() - deadline) < 0) {
        delay(1);
      }
      if (!gnssSerial.available()) {
        return false; // timed out mid-frame
      }
      header[i] = gnssSerial.read();
    }

    uint16_t payloadLen = (uint16_t)header[2] | ((uint16_t)header[3] << 8);
    size_t frameLen = 8 + payloadLen;
    if (frameLen > maxLen) {
      // Too big to be the frame we want. Consume and discard it rather than
      // resyncing inside its payload, where a stray 0xB5 0x62 pair would look
      // like a frame header.
      for (size_t i = 0; i < (size_t)payloadLen + 2; i++) {
        while (!gnssSerial.available() && (int32_t)(millis() - deadline) < 0) {
          delay(1);
        }
        if (!gnssSerial.available()) {
          return false; // timed out mid-frame
        }
        gnssSerial.read();
      }
      continue;
    }

    out[0] = 0xB5;
    out[1] = 0x62;
    memcpy(out + 2, header, 4);

    for (size_t i = 0; i < (size_t)payloadLen + 2; i++) {
      while (!gnssSerial.available() && (int32_t)(millis() - deadline) < 0) {
        delay(1);
      }
      if (!gnssSerial.available()) {
        return false; // timed out mid-frame
      }
      out[6 + i] = gnssSerial.read();
    }

    // Verify the checksum over class..payload before trusting the frame.
    uint8_t ckA = 0;
    uint8_t ckB = 0;
    for (size_t i = 2; i < 6 + (size_t)payloadLen; i++) {
      ckA = (uint8_t)(ckA + out[i]);
      ckB = (uint8_t)(ckB + ckA);
    }
    if (ckA != out[6 + payloadLen] || ckB != out[7 + payloadLen]) {
      continue; // corrupt; keep scanning
    }

    if (header[0] == cls && header[1] == id) {
      *outLen = frameLen;
      return true;
    }
  }

  return false;
}

// Wait for UBX-ACK-ACK acknowledging the given class/id.
static bool waitForAck(uint8_t cls, uint8_t id, uint32_t timeoutMs) {
  uint8_t frame[16];
  size_t frameLen = 0;
  uint32_t deadline = millis() + timeoutMs;

  while ((int32_t)(millis() - deadline) < 0) {
    uint32_t remaining = (uint32_t)(deadline - millis());
    if (!waitForUbxFrame(0x05, 0x01, frame, sizeof(frame), &frameLen,
                         remaining)) {
      return false;
    }
    if (frameLen == 10 && frame[6] == cls && frame[7] == id) {
      return true;
    }
    // An ACK for something else - keep waiting.
  }
  return false;
}

static void printHex(const uint8_t *buf, size_t len) {
  for (size_t i = 0; i < len; i++) {
    Serial.printf("%02X", buf[i]);
    if (i + 1 < len) {
      Serial.print(' ');
    }
    if ((i + 1) % 16 == 0 && i + 1 < len) {
      Serial.print("\n    ");
    }
  }
  Serial.println();
}

// --- Connection ------------------------------------------------------------
static bool connectAtAnyBaud() {
  for (int i = 0; i < NUM_BAUD_RATES; i++) {
    uint32_t baud = BAUD_RATES[i];
    Serial.printf("  trying %lu baud...\n", (unsigned long)baud);

    GNSS_SERIAL_PREPARE();
    GNSS_SERIAL_BEGIN(baud);
    delay(100);

    if (myGNSS.begin(gnssSerial)) {
      Serial.printf("  connected at %lu baud.\n", (unsigned long)baud);
      return true;
    }

    gnssSerial.end();
    delay(100);
  }
  return false;
}

// --- Quieting the receiver -------------------------------------------------
// Periodic output is the only thing that can fill the UART buffer while this
// sketch isn't draining it, and there is exactly one such window: the blocking
// wait for the operator to type the confirmation word, which lasts as long as
// they take. Everything else here either reads continuously or has the port
// closed. Silencing the stream removes that window, and it also means the ACK
// and VALGET replies land in an otherwise empty stream, so the frame scanner
// has nothing to resync through.
//
// UBX output protocol must stay ON: ACK-ACK and the VALGET reply are UBX, so
// turning CFG-UART1OUTPROT-UBX off would silence the very frames this sketch
// waits for. What gets turned off is NMEA, plus the periodic NAV messages -
// none of which is a poll response.
//
// Dropping CFG-RATE-MEAS to 1 Hz is the catch-all: any periodic message not in
// the list below still gets 20x quieter on a receiver configured the way Gnimu
// configures one.
//
// All of it is written to the RAM layer, so a power cycle restores the
// receiver, the CFG-RST at the end of a successful burn does the same, and
// g_gnss.cpp's gnssBegin() reconfigures everything from config.h at boot
// regardless. Nothing here needs undoing.
static const uint32_t PERIODIC_NAV_MSGS[] = {
    UBLOX_CFG_MSGOUT_UBX_NAV_PVT_UART1,
    UBLOX_CFG_MSGOUT_UBX_NAV_SAT_UART1,
    UBLOX_CFG_MSGOUT_UBX_NAV_SIG_UART1,
    UBLOX_CFG_MSGOUT_UBX_NAV_STATUS_UART1,
    UBLOX_CFG_MSGOUT_UBX_NAV_DOP_UART1,
    UBLOX_CFG_MSGOUT_UBX_NAV_POSLLH_UART1,
    UBLOX_CFG_MSGOUT_UBX_NAV_TIMEGPS_UART1};
static const int NUM_PERIODIC_NAV_MSGS =
    sizeof(PERIODIC_NAV_MSGS) / sizeof(PERIODIC_NAV_MSGS[0]);

// Counts bytes arriving over windowMs without parsing them.
static uint32_t countIncomingBytes(uint32_t windowMs) {
  uint32_t count = 0;
  uint32_t deadline = millis() + windowMs;
  while ((int32_t)(millis() - deadline) < 0) {
    while (gnssSerial.available()) {
      gnssSerial.read();
      count++;
    }
    delay(1);
  }
  return count;
}

static void quietReceiver() {
  Serial.println("--- Quieting periodic output (RAM layer) ---");

  uint32_t before = countIncomingBytes(500);
  Serial.printf("  before: %lu bytes in 500 ms\n", (unsigned long)before);

  // NMEA off, UBX kept - the replies this sketch waits for are UBX.
  if (!myGNSS.setUART1Output(COM_TYPE_UBX, VAL_LAYER_RAM)) {
    Serial.println("  WARNING: could not disable NMEA output.");
  }

  int silenced = 0;
  for (int i = 0; i < NUM_PERIODIC_NAV_MSGS; i++) {
    if (myGNSS.setVal8(PERIODIC_NAV_MSGS[i], 0, VAL_LAYER_RAM)) {
      silenced++;
    }
  }
  Serial.printf("  periodic NAV messages set to 0: %d of %d\n", silenced,
                NUM_PERIODIC_NAV_MSGS);

  if (!myGNSS.setVal16(UBLOX_CFG_RATE_MEAS, 1000, VAL_LAYER_RAM)) {
    Serial.println("  WARNING: could not drop the measurement rate to 1 Hz.");
  }

  delay(200);
  drainSerial();

  uint32_t after = countIncomingBytes(500);
  Serial.printf("  after:  %lu bytes in 500 ms\n", (unsigned long)after);

  if (after > 200) {
    Serial.println("  Still chattier than expected. Not fatal - the frame");
    Serial.println("  scanner below copes with interleaved traffic - but the");
    Serial.println("  buffer has less slack during the confirmation wait.");
  }

  Serial.println();
}

// --- Identity --------------------------------------------------------------
// Returns true if this looks like M10 silicon. The burn is only offered on a
// part the manual's procedure is actually written for.
static bool printIdentity() {
  Serial.println("--- Identity (UBX-MON-VER) ---");

  if (!myGNSS.getModuleInfo()) {
    Serial.println("  getModuleInfo() FAILED - cannot identify the receiver.");
    Serial.println();
    return false;
  }

  const char *type = myGNSS.getFirmwareType();
  uint8_t protHigh = myGNSS.getProtocolVersionHigh();
  uint8_t protLow = myGNSS.getProtocolVersionLow();

  Serial.printf("  firmware : %s %u.%02u\n",
                (type != nullptr && type[0] != '\0') ? type : "(empty)",
                myGNSS.getFirmwareVersionHigh(),
                myGNSS.getFirmwareVersionLow());
  Serial.printf("  protver  : %u.%02u\n", protHigh, protLow);

  // PROTVER 34.x is the u-blox M10 generation. 34.10 is ROM SPG 5.10, which is
  // what the affected-products list in information note UBX-23006557 covers.
  bool looksLikeM10 = (protHigh == 34);
  if (!looksLikeM10) {
    Serial.println(
        "  This is not PROTVER 34.x, so it is not the M10 generation");
    Serial.println(
        "  this procedure is written for. No write will be offered.");
  } else if (protLow != 10) {
    Serial.println("  NOTE: PROTVER 34.10 (ROM SPG 5.10) is the firmware the");
    Serial.println(
        "  manual documents this procedure against. Yours differs -");
    Serial.println("  check your own product's integration manual first.");
  }

  Serial.println();
  return looksLikeM10;
}

// --- Clock readback --------------------------------------------------------
// Reads the four keys from RAM and Default via the library, then polls the OTP
// layer with the manual's own raw frame. Returns what RAM says.
static ClockState printClockState(bool *otpMatchedExpected) {
  Serial.println("--- Clock state ---");
  Serial.println("  key         RAM          Default      stock    high");

  int highCount = 0;
  int stockCount = 0;
  int failCount = 0;

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

    Serial.printf("  0x%08lX  %-12s %-12s %lu MHz  %lu MHz\n",
                  (unsigned long)CLOCK_KEYS[i].key, ramText, defaultText,
                  (unsigned long)(CLOCK_KEYS[i].stockHz / 1000000UL),
                  (unsigned long)(CLOCK_KEYS[i].highHz / 1000000UL));

    if (!ramOk) {
      failCount++;
    } else if (ramValue == CLOCK_KEYS[i].highHz) {
      highCount++;
    } else if (ramValue == CLOCK_KEYS[i].stockHz) {
      stockCount++;
    }
  }

  // Now the manual's own check: VALGET on OTP layer 4.
  Serial.println();
  Serial.println("  OTP layer 4 (the manual's verification poll):");

  drainSerial();
  sendRaw(OTP_VERIFY_POLL, sizeof(OTP_VERIFY_POLL));

  uint8_t reply[64];
  size_t replyLen = 0;
  *otpMatchedExpected = false;

  bool gotReply =
      waitForUbxFrame(0x06, 0x8B, reply, sizeof(reply), &replyLen, 2000);

  // Must happen before anything else touches the port - see settleAndDrain().
  settleAndDrain(300);

  if (!gotReply) {
    Serial.println("    no VALGET reply (the receiver may reject layer 4).");
  } else {
    Serial.print("    ");
    printHex(reply, replyLen);
    if (replyLen == sizeof(OTP_VERIFY_EXPECTED) &&
        memcmp(reply, OTP_VERIFY_EXPECTED, replyLen) == 0) {
      *otpMatchedExpected = true;
      Serial.println("    MATCHES the manual's expected programmed reply.");
    } else {
      Serial.println(
          "    does not match the programmed reply (expected if the");
      Serial.println("    high clock has not been burned yet).");
    }
  }

  Serial.println();

  if (failCount > 0) {
    return CLOCK_UNKNOWN;
  }
  if (highCount == NUM_CLOCK_KEYS) {
    return CLOCK_HIGH;
  }
  if (stockCount == NUM_CLOCK_KEYS) {
    return CLOCK_STOCK;
  }
  return CLOCK_UNKNOWN;
}

// --- Confirmation ----------------------------------------------------------
// Blocks until a line arrives. Returns true only on an exact match for
// CONFIRM_WORD - anything else, including an empty line, aborts.
static bool waitForConfirmation() {
  Serial.println(
      "############################################################");
  Serial.println(
      "#  PERMANENT CHANGE - THIS CANNOT BE UNDONE                #");
  Serial.println(
      "############################################################");
  Serial.println();
  Serial.println("  About to write the high CPU clock into this receiver's");
  Serial.println(
      "  one-time programmable memory. It will consume 18 of its 64");
  Serial.println(
      "  bytes of OTP space, apply at every startup from now on, and");
  Serial.println("  there is no way back. The receiver will then be rated for");
  Serial.println("  25 Hz single-GNSS / 20 Hz two-GNSS instead of 18 / 10.");
  Serial.println();
  Serial.printf("  Type %s and press enter to proceed. Anything else aborts.\n",
                CONFIRM_WORD);
  Serial.println();

  // Discard anything already sitting in the host buffer, so a stray newline
  // from opening the monitor can't be read as an answer.
  while (Serial.available()) {
    Serial.read();
  }

  char input[16];
  size_t length = 0;

  for (;;) {
    while (!Serial.available()) {
      delay(10);
    }

    char c = (char)Serial.read();
    if (c == '\r') {
      continue; // tolerate CRLF line endings
    }
    if (c == '\n') {
      break;
    }
    if (length + 1 < sizeof(input)) {
      input[length++] = c;
    } else {
      length = sizeof(input); // overlong: will fail the compare below
      break;
    }
  }

  if (length >= sizeof(input)) {
    Serial.println("  Input too long. ABORTED - nothing was written.");
    return false;
  }
  input[length] = '\0';

  if (strcmp(input, CONFIRM_WORD) != 0) {
    Serial.printf("  Got \"%s\", not %s. ABORTED - nothing was written.\n",
                  input, CONFIRM_WORD);
    return false;
  }

  Serial.println("  Confirmed.");
  Serial.println();
  return true;
}

// --- The burn --------------------------------------------------------------
static bool performOtpWrite() {
  Serial.println("--- Writing OTP (manual sec 2.1.7, Table 8) ---");

  drainSerial();

  Serial.println("  sending frame 1/2...");
  sendRaw(OTP_WRITE_FRAME_1, sizeof(OTP_WRITE_FRAME_1));
  bool ack1 = waitForAck(0x06, 0x41, 3000);
  Serial.printf("    ACK-ACK for 0x06 0x41: %s\n", ack1 ? "received" : "NONE");

  Serial.println("  sending frame 2/2...");
  sendRaw(OTP_WRITE_FRAME_2, sizeof(OTP_WRITE_FRAME_2));
  bool ack2 = waitForAck(0x06, 0x41, 3000);
  Serial.printf("    ACK-ACK for 0x06 0x41: %s\n", ack2 ? "received" : "NONE");

  Serial.println();

  if (!ack1 || !ack2) {
    Serial.println(
        "  The manual expects an ACK-ACK for each frame. At least one");
    Serial.println("  is missing, so the write may not have taken. The verify");
    Serial.println(
        "  step below is what settles it - do not simply re-run this");
    Serial.println(
        "  sketch, since OTP space is finite and already partly spent");
    Serial.println("  if the first frame did land.");
    Serial.println();
    return false;
  }

  return true;
}

// Returns true if the receiver came back and the link was re-established.
static bool resetReceiver() {
  Serial.println("--- Hardware reset (applies the new clock at startup) ---");
  Serial.println("  sending UBX-CFG-RST, hot start, resetMode 0x00...");

  sendRaw(CFG_RST_HARDWARE, sizeof(CFG_RST_HARDWARE));

  // CFG-RST is not acknowledged; the receiver resets immediately. Close the
  // port so the re-sweep starts from a clean UART on both platforms.
  delay(100);
  gnssSerial.end();
  delay(2000); // let the receiver boot

  Serial.println("  reconnecting...");
  if (!connectAtAnyBaud()) {
    Serial.println(
        "  Could not reconnect after the reset. Power-cycle the board");
    Serial.println("  and re-run this sketch: it will read the clock back and");
    Serial.println(
        "  tell you whether the write landed, without writing again.");
    Serial.println();
    return false;
  }

  delay(500);
  drainSerial();
  Serial.println();

  // The reset restored the receiver's normal output, so quiet it again before
  // the verification poll.
  quietReceiver();
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(5000); // give USB CDC time to enumerate before the first print

  Serial.println("\n=== u-blox M10 high-performance CPU clock (OTP) ===");
  Serial.println(
      "Reads the current clock; writes ONLY on explicit confirmation,");
  Serial.println(
      "and only when the receiver is an M10 still at the stock clock.");
  Serial.println();

  myGNSS.setPacketCfgPayloadSize(MAX_PAYLOAD_SIZE);

  Serial.println("--- Connecting ---");
  if (!connectAtAnyBaud()) {
    Serial.println("\nFAILED: u-blox GNSS not detected at any standard baud "
                   "rate. Check your wiring.");
    while (1)
      delay(100); // Halt
  }
  Serial.println();

  delay(500);
  drainSerial();

  quietReceiver();

  bool looksLikeM10 = printIdentity();

  bool otpMatched = false;
  ClockState state = printClockState(&otpMatched);

  if (state == CLOCK_HIGH) {
    Serial.println("ALREADY PROGRAMMED. All four keys read their high-clock");
    Serial.println("values, so this receiver is already rated for 25 Hz");
    Serial.println("single-GNSS / 20 Hz two-GNSS. Nothing to do.");
    Serial.println("\nDone. Halting.");
    return;
  }

  if (!looksLikeM10 || state == CLOCK_UNKNOWN) {
    Serial.println("NOT PROCEEDING. The receiver is either not the M10 this");
    Serial.println("procedure is written for, or its clock keys did not read");
    Serial.println(
        "back cleanly as the stock 128/128/128/64 MHz. The write is");
    Serial.println("permanent, so it is only offered on the exact case it is");
    Serial.println("for. Nothing was written.");
    Serial.println("\nDone. Halting.");
    return;
  }

  if (otpMatched) {
    // This is the state a correctly programmed receiver reports: OTP returns
    // the manual's frame while RAM still shows the stock values. It is a
    // completed burn, not a pending one - the RAM column is simply not where
    // these keys surface. Burning again would spend more of a finite 64 bytes
    // for nothing.
    Serial.println("ALREADY PROGRAMMED. The OTP layer returns the manual's");
    Serial.println(
        "expected frame, which is the manual's own pass criterion, so");
    Serial.println(
        "the high clock is written and in effect. RAM still reading");
    Serial.println("the stock values is expected for these keys - check the");
    Serial.println("Default column above, which should read 192/192/192/96.");
    Serial.println();
    Serial.println("Nothing to do. Writing again would only consume more OTP.");
    Serial.println("\nDone. Halting.");
    return;
  }

  Serial.println("STOCK CLOCK. This receiver is at 128/128/128/64 MHz, so two");
  Serial.println("constellations are rated for 10 Hz and one for 18 Hz.");
  Serial.println();

  if (!waitForConfirmation()) {
    Serial.println("\nDone. Halting.");
    return;
  }

  // The return value is informational: the write is attempted either way, and
  // the verify below is the authoritative answer on what actually landed.
  (void)performOtpWrite();

  if (!resetReceiver()) {
    Serial.println("Cannot verify without a link. Halting.");
    return;
  }

  Serial.println("--- Verifying ---");
  bool otpMatchedAfter = false;
  ClockState after = printClockState(&otpMatchedAfter);

  // The OTP-layer reply is the verdict. That is the manual's own pass
  // criterion, and it is the only layer that reliably reflects the write: on
  // real hardware the Default layer follows it, but RAM keeps reporting the
  // stock 128/128/128/64 even after a fully successful burn. Judging on RAM
  // produced a false FAILED on the first board through this tool.
  if (otpMatchedAfter) {
    Serial.println(
        "SUCCESS. The OTP-layer reply matches the manual's expected");
    Serial.println(
        "frame byte-for-byte, which is its own verification step and");
    Serial.println("the authoritative answer. This receiver is now rated for");
    Serial.println("25 Hz single-GNSS / 20 Hz two-GNSS.");
    if (after != CLOCK_HIGH) {
      Serial.println();
      Serial.println(
          "The RAM column above still reads the stock clock. That is");
      Serial.println(
          "expected for these keys and is NOT a failure - compare it");
      Serial.println("against the Default column, which should now read");
      Serial.println(
          "192/192/192/96. Nothing further is needed, and this sketch");
      Serial.println("must NOT be run against this receiver again.");
    }
    Serial.println();
    Serial.println(
        "Next: run gnss_ver under an open sky and compare phase 6's");
    Serial.println(
        "fix rate at a high satellite count against your pre-burn run.");
  } else if (after == CLOCK_HIGH) {
    Serial.println(
        "PARTIAL. RAM reads the high clock but the OTP layer did not");
    Serial.println("return the manual's expected frame. Compare the hex dump");
    Serial.println(
        "above against the manual before trusting this across power");
    Serial.println("cycles - re-run after a full power-down to be sure.");
  } else {
    Serial.println(
        "FAILED. Neither the OTP layer nor RAM came back at the high");
    Serial.println("values. Do NOT re-run this sketch blindly: OTP space is");
    Serial.println(
        "finite and a partial write may already have consumed some.");
    Serial.println("Compare the hex above with the manual and work out what");
    Serial.println("landed first.");
  }

  Serial.println("\nDone. Halting.");
}

void loop() {
  delay(1000); // Nothing to do - setup() already halted logically.
}
