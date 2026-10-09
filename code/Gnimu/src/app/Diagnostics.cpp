#include "Diagnostics.h"

#include "../core/GnssNames.h"
#include "../core/TextOut.h"
#include "../platform/BatteryPort.h"
#include "../platform/Clock.h"
#include "BuildInfo.h"
#include "Config.h"

namespace gnimu {

namespace {

// The next whole multiple of the status period in uptime. Lines land on the
// grid plus at most one console tick, so RT steps by exactly one per line
// instead of occasionally skipping or repeating a second.
uint64_t nextStatusGridUs(uint64_t nowUs) {
  return (nowUs / config::kStatusPeriodUs + 1) * config::kStatusPeriodUs;
}

// The first line after arming lands on the first grid point a full period
// away, so the first measurement window is 1-2 s long. Landing on the very
// next grid point could leave a window of a few milliseconds, where a single
// epoch more or less reads as tens of Hz (seen: "GNSS: 50Hz").
uint64_t firstStatusGridUs(uint64_t nowUs) {
  return nextStatusGridUs(nowUs + config::kStatusPeriodUs - 1);
}

}  // namespace

void Diagnostics::handle(const ConsoleCommand& cmd) {
  switch (cmd.kind) {
    case CommandKind::None:
      break;
    case CommandKind::Report:
      printReport();
      break;
    case CommandKind::Clear:
      // The counters were just cleared, so the old baselines would read as a
      // backwards jump; measure the next line from now instead.
      if (statusRemaining_ != 0) {
        const uint64_t now = PlatformClock::nowUs();
        gnssRate_.reset(feed_.epochs().epochs(), now);
        bleRate_.reset(ble_.stats().sent, now);
      }
      break;
    case CommandKind::Version:
      printBuild();
      printDeviceId();
      break;
    case CommandKind::ToggleStatus:
      // A second 'L' stops whatever's running; otherwise it starts the
      // longest run a typed count allows, so reporting left on by a host that
      // went away still ends by itself.
      armStatus(statusRemaining_ != 0 ? 0 : config::kStatusMaxCount);
      break;
  }
  if (cmd.hasCount) {
    // Typing a number always (re)arms to exactly that count, overriding
    // whatever was running before; 0 stops reporting, same as a second 'L'.
    armStatus(cmd.count);
  }
}

void Diagnostics::tick(uint64_t nowUs) {
  if (statusRemaining_ == 0 || nowUs < nextStatusDueUs_) return;
  printStatus(nowUs);
  // Fixed grid, not "now + period": scheduling from now let every line
  // land up to a console tick later than the last, slipping a whole
  // second of RT every ~100 lines. After a long stall, resync to the next
  // grid point rather than firing a burst of catch-up lines.
  nextStatusDueUs_ += config::kStatusPeriodUs;
  if (nextStatusDueUs_ <= nowUs) nextStatusDueUs_ = nextStatusGridUs(nowUs);
  statusRemaining_--;
}

void Diagnostics::armStatus(uint32_t count) {
  statusRemaining_ = count;
  if (count != 0) {
    const uint64_t now = PlatformClock::nowUs();
    gnssRate_.reset(feed_.epochs().epochs(), now);
    bleRate_.reset(ble_.stats().sent, now);
    nextStatusDueUs_ = firstStatusGridUs(now);
  }
}

// The constellations in priority order, the nav rate and the dynamic model:
// the targets the boot configuration verified, not a live readback.
void Diagnostics::formatGnssSettings() {
  detail::Out o{gnssSettings_, sizeof(gnssSettings_), 0};
  for (unsigned i = 0; i < config::kConstellationCount; i++)
    o.put("%s%s", i ? "+" : "", config::gnssSystemName(config::kConstellationPriority[i]));
  o.put(", %u Hz, %s", static_cast<unsigned>(config::kGnssNavRateHz),
        dynamicModelName(config::kGnssDynamicModel));
}

DeviceStatus Diagnostics::snapshot(uint64_t nowUs) const {
  DeviceStatus s = {};
  s.uptimeUs = nowUs;
  s.reset = reset_;
  s.gnss.settings = gnssSettings_;
  s.gnss.epochs = &feed_.epochs();
  s.gnss.latest = feed_.latest();
  s.imu.up = imu_.up();
  s.imu.sample = imu_.read();
  s.imu.trimLocked = imu_.trimConverged();
  s.imu.tiltDegrees = imu_.trimTiltDegrees();
  s.ble.connected = ble_.connected();
  s.ble.stats = ble_.stats();
  s.power.battery = battery_;
  s.power.usbPresent = BatteryPort::usbPresent();
  s.power.switchNoise = machine_.switchNoise();
  s.consoleDropped = console_.dropped();
  return s;
}

void Diagnostics::printReport() {
  const JobTable jobs = {jobs_, stats_, jobCount_, loops_};
  console_.printReport(jobs, snapshot(PlatformClock::nowUs()));
}

void Diagnostics::printStatus(uint64_t nowUs) {
  const StatusRates rates = {bleRate_.sampleCentiHz(ble_.stats().sent, nowUs),
                             gnssRate_.sampleCentiHz(feed_.epochs().epochs(), nowUs)};
  console_.printStatus(snapshot(nowUs), rates);
}

void Diagnostics::printBuild() {
  char line[192];
  formatBuildLine(line, sizeof(line), buildInfo());
  console_.log("%s", line);
}

void Diagnostics::printDeviceId() { console_.log("Device ID: %s", deviceId_); }

}  // namespace gnimu
