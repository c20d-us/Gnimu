#include "Gnss.h"

#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_v3.h>

#include "../../app/Config.h"
#include "../../platform/Clock.h"
#include "../../platform/GnssUart.h"
#include "../../platform/Watchdog.h"

namespace gnimu {

namespace {

SFE_UBLOX_GNSS_SERIAL receiver;
GnssEpoch latestEpoch;
bool epochPending = false;

// Every library call that waits for a reply feeds the watchdog first, so even
// a receiver that answers nothing never lets it expire. The longest such call
// is detection on a silent port: begin() tries three times, each waiting out
// the full timeout, between tryBaud()'s 100 ms pauses.
static_assert(3 * kUBLOXGNSSDefaultMaxWait + 1000 <= config::kWatchdogMs,
              "watchdog shorter than one GNSS detection attempt plus margin");

// A setting the receiver must hold, and its human name for the log line.
struct KeyVal {
  uint32_t key;
  uint8_t desired;
  const char *name;
};

// Helper function to drain (and discard) any outstanding data in the UART.
void drainUart() {
  Stream &s = GnssUart::stream();
  while (s.available())
    s.read();
}

// Grab data from an newly received PVT.
void onPvt(UBX_NAV_PVT_data_t *pvt) {
  latestEpoch.arrivalUs = PlatformClock::nowUs();
  latestEpoch.iTowMs = pvt->iTOW;
  latestEpoch.hAccMm = pvt->hAcc;
  latestEpoch.tAccNs = pvt->tAcc;
  latestEpoch.latDeg1e7 = pvt->lat;
  latestEpoch.lonDeg1e7 = pvt->lon;
  latestEpoch.pDop = pvt->pDOP;
  latestEpoch.fixType = pvt->fixType;
  latestEpoch.numSv = pvt->numSV;
  latestEpoch.gSpeedMmPerS = static_cast<uint32_t>(pvt->gSpeed);
  latestEpoch.gnssFixOk = pvt->flags.bits.gnssFixOK;
  latestEpoch.year = pvt->year;
  latestEpoch.month = pvt->month;
  latestEpoch.day = pvt->day;
  latestEpoch.hour = pvt->hour;
  latestEpoch.min = pvt->min;
  latestEpoch.sec = pvt->sec;
  latestEpoch.validDate = pvt->valid.bits.validDate;
  latestEpoch.validTime = pvt->valid.bits.validTime;
  latestEpoch.fullyResolved = pvt->valid.bits.fullyResolved;
  latestEpoch.validMag = pvt->valid.bits.validMag;
  latestEpoch.nanoNs = pvt->nano;
  latestEpoch.heightMm = pvt->height;
  latestEpoch.hMslMm = pvt->hMSL;
  latestEpoch.vAccMm = pvt->vAcc;
  latestEpoch.headMotDeg1e5 = pvt->headMot;
  latestEpoch.sAccMmPerS = pvt->sAcc;
  latestEpoch.headAccDeg1e5 = pvt->headAcc;
  latestEpoch.headVehValid = pvt->flags.bits.headVehValid;
  epochPending = true;
}

// Overloaded on value width so ensureVal<T> can reach the right getVal/setVal
// pair through ordinary overload resolution, without an if-constexpr (not
// worth risking on a C++ standard we haven't confirmed either core compiles
// with).
bool getFlash(uint32_t key, uint8_t *val) {
  return receiver.getVal8(key, val, VAL_LAYER_FLASH);
}
bool getFlash(uint32_t key, uint16_t *val) {
  return receiver.getVal16(key, val, VAL_LAYER_FLASH);
}
bool setAll(uint32_t key, uint8_t val) {
  return receiver.setVal8(key, val, VAL_LAYER_ALL);
}
bool setAll(uint32_t key, uint16_t val) {
  return receiver.setVal16(key, val, VAL_LAYER_ALL);
}

// A lost or corrupted reply fails the same way as a key that's absent from
// flash, so a single failed read can't tell "never stored" from "reply got
// mangled" - and treating the latter as absent means a needless flash write
// plus a false failure line. Retrying sorts them apart: a mangled reply is
// transient, while an absent key NAKs every time, in milliseconds, so the
// retries cost almost nothing on a first-ever configuration. No pause between
// attempts: a lost reply already waits out the library's full timeout.
constexpr int kFlashReadAttempts = 3;

template <typename T> bool readFlash(uint32_t key, T *val) {
  for (int attempt = 0; attempt < kFlashReadAttempts; attempt++) {
    Watchdog::feed();
    if (getFlash(key, val))
      return true;
  }
  return false;
}

} // namespace

// Opens the port at `baud` and looks for the receiver. Leaves the port open on
// success and closed on failure.
bool Gnss::tryBaud(uint32_t baud) {
  log_->log("GNSS: trying %lu baud", static_cast<unsigned long>(baud));
  GnssUart::begin(baud);
  delay(100);
  Watchdog::feed();
  if (receiver.begin(GnssUart::stream())) {
    log_->log("GNSS: detected at %lu baud ✅",
              static_cast<unsigned long>(baud));
    return true;
  }
  GnssUart::end();
  delay(100);
  return false;
}

// Switches a receiver found at another rate to the configured one, then writes
// the baud to flash so that the rate persists.
bool Gnss::switchToTargetBaud() {
  log_->log("GNSS: switching to %lu baud",
            static_cast<unsigned long>(config::kGnssBaud));
  // The result is ignored on purpose: the receiver switches baud before its
  // ACK goes out, so the library can't report success (its own note). The
  // reconnect at the new rate below is the real check.
  Watchdog::feed();
  receiver.setSerialRate(config::kGnssBaud);
  delay(100);
  GnssUart::end();
  delay(100);
  GnssUart::begin(config::kGnssBaud);
  delay(100);
  Watchdog::feed();
  if (!receiver.begin(GnssUart::stream())) {
    log_->log("GNSS: could not verify the new baud rate ❌");
    return false;
  }
  Watchdog::feed();
  if (receiver.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT)) {
    log_->log("GNSS: new baud rate saved to flash ✅");
  } else {
    log_->log("GNSS: unable to save new baud rate to flash ❌");
  }
  return true;
}

// Try the target rate first, then the sweep rest if needed.
bool Gnss::connectAtBaud() {
  if (tryBaud(config::kGnssBaud))
    return true;
  for (uint32_t rate : config::kGnssBaudRates) {
    if (rate == config::kGnssBaud)
      continue;
    if (tryBaud(rate))
      return switchToTargetBaud();
  }
  return false;
}

// Reads `key` from the FLASH layer. Flash is the layer that answers "did this
// setting already get persisted" and is absent (NACKs) for a key that was never
// stored there. Every key logs a line, so the boot log is a complete record of
// what the receiver's flash held versus what had to be written: a simple check
// mark if it already matched, "written to flash" if it had to be pushed to
// VAL_LAYER_ALL (RAM+BBR+flash together) and read back from flash to confirm.
// Persisting to flash means a given key's write only ever has to succeed once
// per receiver: every later boot's flash read already reflects it, so there's
// nothing to write.
template <typename T>
bool Gnss::ensureVal(uint32_t key, T desired, const char *name) {
  T current = 0;
  if (readFlash(key, &current) && current == desired) {
    log_->log("GNSS: %s=%u ✅", name, static_cast<unsigned>(desired));
    return true;
  }
  // Judged by the flash readback alone: a write whose ACK got lost can still
  // have landed, and the readback (retried) is the real test either way.
  Watchdog::feed();
  setAll(key, desired);
  const bool verified = readFlash(key, &current) && current == desired;
  log_->log(verified ? "GNSS: %s=%u written to flash ⏬"
                      : "GNSS: %s=%u write to flash failed - value not set ❌",
            name, static_cast<unsigned>(desired));
  return verified;
}

bool Gnss::ensureVal8(uint32_t key, uint8_t desired, const char *name) {
  return ensureVal<uint8_t>(key, desired, name);
}

bool Gnss::ensureVal16(uint32_t key, uint16_t desired, const char *name) {
  return ensureVal<uint16_t>(key, desired, name);
}

bool Gnss::configure() {
  // Let the receiver settle and then clear any bytes left from the baud
  // handshake so the first configuration reads/ACKs parse cleanly.
  delay(500);
  drainUart();

  // NAV-PVT output is stored on in flash (below), so a receiver that resets
  // mid-session resumes on its own - but that also starts it at power-up.
  // Paused in RAM only for the reads and writes below, so they run on a quiet
  // line; the callback registration at the end turns it back on.
  // This first write is the one that competes with the traffic it stops, so
  // its ACK can be lost among PVT frames; retried, each try on a freshly
  // drained line. Repeating a 0 is harmless if an earlier try took.
  constexpr int kPauseAttempts = 3;
  int attempt = 1;
  bool paused = false;
  for (; attempt <= kPauseAttempts; attempt++) {
    if (attempt > 1) drainUart();
    Watchdog::feed();
    paused = receiver.setVal8(UBLOX_CFG_MSGOUT_UBX_NAV_PVT_UART1, 0, VAL_LAYER_RAM);
    if (paused) break;
  }
  if (paused && attempt > 1) {
    log_->log("GNSS: NAV-PVT output paused for configuration \xE2\x9C\x85 (attempt %d)", attempt);
  } else {
    log_->log("GNSS: NAV-PVT output paused for configuration %s", paused ? "\xE2\x9C\x85" : "\xE2\x9D\x8C");
  }
  drainUart();

  bool selected[4] = {};
  for (unsigned i = 0; i < config::kConstellationCount; i++)
    selected[static_cast<unsigned>(config::kConstellationPriority[i])] = true;
  const auto on = [&](config::GnssSystem s) -> uint8_t {
    return selected[static_cast<unsigned>(s)] ? 1 : 0;
  };

  // Every setting the receiver must hold. Read-verified-and-fixed-if-needed by
  // ensureVal8 above, so every boot logs one "✅" or "written" line per key.
  // A normal boot shows all "✅"; a first-ever configuration, or something
  // knocking a value loose, shows "written" instead for whichever keys needed
  // it.
  const KeyVal settings[] = {
      {UBLOX_CFG_NAVSPG_DYNMODEL, config::kGnssDynamicModel, "dynamic model"},
      {UBLOX_CFG_UART1OUTPROT_UBX, 1, "UBX output enable"},
      {UBLOX_CFG_UART1OUTPROT_NMEA, 0, "NMEA output enable"},
      {UBLOX_CFG_MSGOUT_NMEA_ID_DTM_UART1, 0, "NMEA DTM rate"},
      {UBLOX_CFG_MSGOUT_NMEA_ID_GBS_UART1, 0, "NMEA GBS rate"},
      {UBLOX_CFG_MSGOUT_NMEA_ID_GGA_UART1, 0, "NMEA GGA rate"},
      {UBLOX_CFG_MSGOUT_NMEA_ID_GLL_UART1, 0, "NMEA GLL rate"},
      {UBLOX_CFG_MSGOUT_NMEA_ID_GNS_UART1, 0, "NMEA GNS rate"},
      {UBLOX_CFG_MSGOUT_NMEA_ID_GRS_UART1, 0, "NMEA GRS rate"},
      {UBLOX_CFG_MSGOUT_NMEA_ID_GSA_UART1, 0, "NMEA GSA rate"},
      {UBLOX_CFG_MSGOUT_NMEA_ID_GST_UART1, 0, "NMEA GST rate"},
      {UBLOX_CFG_MSGOUT_NMEA_ID_GSV_UART1, 0, "NMEA GSV rate"},
      {UBLOX_CFG_MSGOUT_NMEA_ID_RLM_UART1, 0, "NMEA RLM rate"},
      {UBLOX_CFG_MSGOUT_NMEA_ID_RMC_UART1, 0, "NMEA RMC rate"},
      {UBLOX_CFG_MSGOUT_NMEA_ID_VLW_UART1, 0, "NMEA VLW rate"},
      {UBLOX_CFG_MSGOUT_NMEA_ID_VTG_UART1, 0, "NMEA VTG rate"},
      {UBLOX_CFG_MSGOUT_NMEA_ID_ZDA_UART1, 0, "NMEA ZDA rate"},
      {UBLOX_CFG_NAVSPG_INFIL_MINELEV, config::kGnssMinElevDeg,
       "minimum SV elevation"},
      // BeiDou B1I and B1C are mutually exclusive: B1I is always off, and B1C
      // follows BeiDou's selection. The other systems' signal keys follow
      // theirs.
      {UBLOX_CFG_SIGNAL_BDS_B1_ENA, 0, "BeiDou B1I signal"},
      {UBLOX_CFG_SIGNAL_BDS_B1C_ENA, on(config::GnssSystem::Beidou),
       "BeiDou B1C signal"},
      {UBLOX_CFG_SIGNAL_GPS_L1CA_ENA, on(config::GnssSystem::Gps),
       "GPS L1C/A signal"},
      {UBLOX_CFG_SIGNAL_GAL_E1_ENA, on(config::GnssSystem::Galileo),
       "Galileo E1 signal"},
      {UBLOX_CFG_SIGNAL_GLO_L1_ENA, on(config::GnssSystem::Glonass),
       "GLONASS L1 signal"},
      {UBLOX_CFG_SIGNAL_GPS_ENA, on(config::GnssSystem::Gps), "GPS enable"},
      {UBLOX_CFG_SIGNAL_GAL_ENA, on(config::GnssSystem::Galileo),
       "Galileo enable"},
      {UBLOX_CFG_SIGNAL_BDS_ENA, on(config::GnssSystem::Beidou),
       "BeiDou enable"},
      {UBLOX_CFG_SIGNAL_GLO_ENA, on(config::GnssSystem::Glonass),
       "GLONASS enable"},
      // QZSS and SBAS always disabled
      {UBLOX_CFG_SIGNAL_QZSS_ENA, 0, "QZSS enable"},
      {UBLOX_CFG_SIGNAL_SBAS_ENA, 0, "SBAS enable"},
  };

  bool allOk = true;
  for (const auto &s : settings)
    allOk &= ensureVal8(s.key, s.desired, s.name);
  // Output rate is measurement interval x nav ratio. There's no single "nav
  // Hz" key, so both values have to be pinned: RATE_MEAS controls the interval
  // in milliseconds, and RATE_NAV=1 ensures every measurement produces a PVT.
  log_->log("GNSS: setting Nav frequency to %uHz",
            static_cast<unsigned>(config::kGnssNavRateHz));
  allOk &= ensureVal16(UBLOX_CFG_RATE_NAV, 1, "nav ratio"); // always 1
  allOk &= ensureVal16(UBLOX_CFG_RATE_MEAS, 1000 / config::kGnssNavRateHz,
                       "nav rate");
  // Last: on a first-ever write this also turns output on in RAM (the write
  // goes to every layer), so nothing after it competes with the traffic.
  allOk &= ensureVal8(UBLOX_CFG_MSGOUT_UBX_NAV_PVT_UART1, 1, "NAV-PVT output rate");
  if (!allOk)
    log_->log(
        "GNSS: configuration did not complete successfully - performance may "
        "be degraded ❌");

  // Callback is registered last, once the receiver is fully configured. It
  // also turns PVT output back on - in RAM only, since flash already holds it
  // (above). The callback itself is local firmware state, so there's nothing
  // to read back and compare.
  log_->log("GNSS: Configuring PVT callback and auto PVT output");
  Watchdog::feed();
  const bool pvtOk = receiver.setAutoPVTcallbackPtr(&onPvt, VAL_LAYER_RAM);
  if (pvtOk) {
    log_->log("GNSS: PVT callback and auto PVT output enabled ✅");
  } else {
    log_->log("GNSS: PVT callback and auto PVT output not enabled ❌");
  }
  return allOk && pvtOk;
}

bool Gnss::begin(Console &log) {
  log_ = &log;
  configured_ = false;
  if (!connectAtBaud())
    return false;
  configured_ = configure();
  return true;
}

void Gnss::poll() {
  // Callbacks only become pending when bytes have arrived.
  if (receiver.checkUblox())
    receiver.checkCallbacks();
}

void Gnss::end() {
  GnssUart::end();
  epochPending = false;
}

bool Gnss::consumeEpoch(GnssEpoch &out) {
  if (!epochPending)
    return false;
  epochPending = false;
  out = latestEpoch;
  return true;
}

} // namespace gnimu
