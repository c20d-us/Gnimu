#include "Gnss.h"

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include <SparkFun_u-blox_GNSS_v3.h>

#include "../platform/Clock.h"
#include "../platform/GnssUart.h"
#include "Config.h"

namespace gnimu {

namespace {

SFE_UBLOX_GNSS_SERIAL receiver;
GnssEpoch latestEpoch;
bool epochPending = false;

// Every NMEA sentence the M10 defines for UART1. Disabling the UART's NMEA
// protocol only filters the output, so the per-sentence rates are zeroed too. A
// key the firmware rejects just doesn't exist there, which is not an error.
const uint32_t kNmeaRateKeys[] = {
    UBLOX_CFG_MSGOUT_NMEA_ID_DTM_UART1, UBLOX_CFG_MSGOUT_NMEA_ID_GBS_UART1,
    UBLOX_CFG_MSGOUT_NMEA_ID_GGA_UART1, UBLOX_CFG_MSGOUT_NMEA_ID_GLL_UART1,
    UBLOX_CFG_MSGOUT_NMEA_ID_GNS_UART1, UBLOX_CFG_MSGOUT_NMEA_ID_GRS_UART1,
    UBLOX_CFG_MSGOUT_NMEA_ID_GSA_UART1, UBLOX_CFG_MSGOUT_NMEA_ID_GST_UART1,
    UBLOX_CFG_MSGOUT_NMEA_ID_GSV_UART1, UBLOX_CFG_MSGOUT_NMEA_ID_RLM_UART1,
    UBLOX_CFG_MSGOUT_NMEA_ID_RMC_UART1, UBLOX_CFG_MSGOUT_NMEA_ID_VLW_UART1,
    UBLOX_CFG_MSGOUT_NMEA_ID_VTG_UART1, UBLOX_CFG_MSGOUT_NMEA_ID_ZDA_UART1,
};

void drainUart() {
  Stream& s = GnssUart::stream();
  while (s.available()) s.read();
}

void onPvt(UBX_NAV_PVT_data_t* pvt) {
  latestEpoch.arrivalUs = PlatformClock::nowUs();
  latestEpoch.iTowMs = pvt->iTOW;
  latestEpoch.hAccMm = pvt->hAcc;
  latestEpoch.pDop = pvt->pDOP;
  latestEpoch.fixType = pvt->fixType;
  latestEpoch.numSv = pvt->numSV;
  epochPending = true;
}

}  // namespace

void Gnss::step(bool ok, const char* fmt, ...) {
  char what[96];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(what, sizeof(what), fmt, ap);
  va_end(ap);
  log_->log("GNSS: %s %s", what, ok ? "ok" : "FAILED");
}

// Opens the port at `baud` and looks for the receiver. Leaves the port open on
// success and closed on failure. The library's default wait is deliberate: a
// shorter one has missed a receiver at 115200 on hardware, and the wait only
// costs time at rates where nothing answers.
bool Gnss::tryBaud(uint32_t baud) {
  log_->log("GNSS: trying %lu baud", static_cast<unsigned long>(baud));
  GnssUart::begin(baud);
  delay(100);
  if (receiver.begin(GnssUart::stream())) {
    log_->log("GNSS: detected at %lu baud", static_cast<unsigned long>(baud));
    return true;
  }
  GnssUart::end();
  delay(100);
  return false;
}

// Switches a receiver found at another rate to the configured one, then writes
// only the baud to flash. Everything else deliberately stays out of flash.
bool Gnss::switchToTargetBaud() {
  log_->log("GNSS: switching to %lu baud", static_cast<unsigned long>(config::kGnssBaud));
  receiver.setSerialRate(config::kGnssBaud);
  delay(100);
  GnssUart::end();
  delay(100);
  GnssUart::begin(config::kGnssBaud);
  delay(100);
  if (!receiver.begin(GnssUart::stream())) {
    log_->log("GNSS: could not verify the new baud rate");
    return false;
  }
  step(receiver.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT), "baud rate saved to flash");
  return true;
}

// The configured rate first for the fastest normal boot, then the rest of the
// sweep.
bool Gnss::connectAtBaud() {
  if (tryBaud(config::kGnssBaud)) return true;
  for (uint32_t rate : config::kGnssBaudRates) {
    if (rate == config::kGnssBaud) continue;
    if (tryBaud(rate)) return switchToTargetBaud();
  }
  return false;
}

void Gnss::configureConstellations() {
  bool selected[4] = {};
  for (unsigned i = 0; i < config::kConstellationCount; i++)
    selected[static_cast<unsigned>(config::kConstellationPriority[i])] = true;
  const auto on = [&](config::GnssSystem s) { return selected[static_cast<unsigned>(s)]; };

  // BeiDou B1I and B1C are mutually exclusive: B1I is always off, and B1C
  // follows BeiDou's selection. The other systems' signal keys follow theirs.
  step(receiver.setVal8(UBLOX_CFG_SIGNAL_BDS_B1_ENA, 0), "BeiDou B1I signal off");
  step(receiver.setVal8(UBLOX_CFG_SIGNAL_BDS_B1C_ENA, on(config::GnssSystem::Beidou)),
       "BeiDou B1C signal %s", on(config::GnssSystem::Beidou) ? "on" : "off");
  step(receiver.setVal8(UBLOX_CFG_SIGNAL_GPS_L1CA_ENA, on(config::GnssSystem::Gps)),
       "GPS L1C/A signal %s", on(config::GnssSystem::Gps) ? "on" : "off");
  step(receiver.setVal8(UBLOX_CFG_SIGNAL_GAL_E1_ENA, on(config::GnssSystem::Galileo)),
       "Galileo E1 signal %s", on(config::GnssSystem::Galileo) ? "on" : "off");
  step(receiver.setVal8(UBLOX_CFG_SIGNAL_GLO_L1_ENA, on(config::GnssSystem::Glonass)),
       "GLONASS L1 signal %s", on(config::GnssSystem::Glonass) ? "on" : "off");

  const struct {
    const char* name;
    bool enabled;
    sfe_ublox_gnss_ids_e id;
  } systems[] = {
      {"GPS", on(config::GnssSystem::Gps), SFE_UBLOX_GNSS_ID_GPS},
      {"Galileo", on(config::GnssSystem::Galileo), SFE_UBLOX_GNSS_ID_GALILEO},
      {"BeiDou", on(config::GnssSystem::Beidou), SFE_UBLOX_GNSS_ID_BEIDOU},
      {"GLONASS", on(config::GnssSystem::Glonass), SFE_UBLOX_GNSS_ID_GLONASS},
      {"QZSS", false, SFE_UBLOX_GNSS_ID_QZSS},
      {"SBAS", false, SFE_UBLOX_GNSS_ID_SBAS},
  };
  for (const auto& sys : systems)
    step(receiver.enableGNSS(sys.enabled, sys.id), "%s %s", sys.name,
         sys.enabled ? "enabled" : "disabled");
}

void Gnss::configure() {
  // Let the receiver settle, and clear bytes left from the baud handshake so
  // the first configuration ACKs parse cleanly.
  delay(500);
  drainUart();

  step(receiver.setDynamicModel(static_cast<dynModel>(config::kGnssDynamicModel)),
       "dynamic model %u", static_cast<unsigned>(config::kGnssDynamicModel));
  step(receiver.setUART1Output(COM_TYPE_UBX), "UBX-only output");

  size_t zeroed = 0;
  for (uint32_t key : kNmeaRateKeys)
    if (receiver.setVal8(key, 0)) zeroed++;
  // A rejected key is a sentence this firmware doesn't have, not an error.
  log_->log("GNSS: NMEA sentence rates zeroed (%u of %u)", static_cast<unsigned>(zeroed),
            static_cast<unsigned>(sizeof(kNmeaRateKeys) / sizeof(kNmeaRateKeys[0])));

  step(receiver.setVal8(UBLOX_CFG_NAVSPG_INFIL_MINELEV, config::kGnssMinElevDeg),
       "minimum SV elevation %u deg", static_cast<unsigned>(config::kGnssMinElevDeg));

  configureConstellations();

  step(receiver.setNavigationFrequency(config::kGnssNavRateHz), "nav rate %u Hz",
       static_cast<unsigned>(config::kGnssNavRateHz));
  // Registered last, once the receiver is fully configured. This also turns on
  // automatic PVT output.
  step(receiver.setAutoPVTcallbackPtr(&onPvt), "PVT callback and auto PVT output");
}

bool Gnss::begin(Console& log) {
  log_ = &log;
  if (!connectAtBaud()) return false;
  configure();
  return true;
}

void Gnss::poll() {
  // Callbacks can only become pending when bytes arrive, so skip the library's
  // walk of every message type on passes where nothing did.
  if (receiver.checkUblox()) receiver.checkCallbacks();
}

bool Gnss::consumeEpoch(GnssEpoch& out) {
  if (!epochPending) return false;
  epochPending = false;
  out = latestEpoch;
  return true;
}

}  // namespace gnimu
