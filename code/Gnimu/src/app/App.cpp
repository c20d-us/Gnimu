#include "App.h"

#include <Arduino.h>
#include <stdio.h>

#include "../core/BleProfile.h"
#include "../core/DeviceId.h"
#include "../core/RaceBoxEncoder.h"
#include "../platform/BatteryPort.h"
#include "../platform/HardwareId.h"
#include "../platform/PeripheralPower.h"
#include "../platform/StatusLedPort.h"
#include "BuildInfo.h"

namespace gnimu {

namespace {

// Boards without a battery (the ESP32): 100%, not charging - Classic's value
// for its battery-less builds.
constexpr uint8_t kRaceBoxNoBattery = 100;

uint32_t nowMs() { return static_cast<uint32_t>(PlatformClock::nowUs() / 1000); }

// The selected protocol's BLE layout. -Wswitch flags a new Protocol value
// that isn't handled here.
const BleProfile& activeProfile() {
  switch (config::kProtocol) {
    case config::Protocol::RaceBox:
      return kRaceBoxProfile;
  }
  return kRaceBoxProfile;  // unreachable: every Protocol is handled above
}

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

App::App()
    : jobs_{{"gnss", config::kGnssDrainPeriodUs, config::kGnssDrainBudgetUs, &App::gnssJob, this},
            {"imu", config::kImuSamplePeriodUs, config::kImuSampleBudgetUs, &App::imuJob, this},
            {"battery", config::kBatteryPeriodUs, config::kBatteryBudgetUs, &App::batteryJob, this},
            {"power", config::kPowerPeriodUs, config::kPowerBudgetUs, &App::powerJob, this},
            {"ble", config::kBlePeriodUs, config::kBleBudgetUs, &App::bleJob, this},
            {"led", config::kLedPeriodUs, config::kLedBudgetUs, &App::ledJob, this},
            {"console", config::kConsolePeriodUs, config::kConsoleBudgetUs, &App::consoleJob,
             this}},
      scheduler_(jobs_),
      epochs_(1000 / config::kGnssNavRateHz),
      speedGate_(config::kImuTrimPvtStaleMs * 1000ULL),
      statusLed_({config::kLedBlinkHalfMs, config::kLedFaultHalfMs, config::kLedBatteryWaitHalfMs}),
      battery_({config::kBatterySamplesPerRun, config::kBatteryRunIntervalMs, config::kBatteryEmaAlpha,
                config::kBatteryCutoffV, config::kBatteryCriticalV, config::kBatteryWarningV,
                config::kBatteryFullScaleV, config::kBatteryCutoffDebounceMs,
                config::kBatteryChargeFullV, config::kBatteryChargeFullDebounceMs}),
      policy_({config::kSwitchOffDebounceMs, config::kChargeOnlyWindowMs}) {}

void App::begin() {
  // Before anything else, the console included: every switched peripheral
  // stays off until a state decides otherwise (the nRF's console can wait
  // seconds for USB, and nothing should draw power meanwhile).
  switchedPower_ = PeripheralPower::holdAllOff();
  StatusLedPort::begin();
  console_.begin();
  decimalDeviceId(hardwareId(), deviceId_);
  printBuild();
  printDeviceId();
  console_.log("Gnimu starting");
  if (switchedPower_) console_.log("Power: GNSS rail, IMU and LED held off \xE2\x9C\x85");

  PlatformClock::begin();
  console_.log("Clock started");

  // The power policy decides before anything starts: a switched-off or
  // flat-battery boot never powers the GNSS at all.
  RunState first = RunState::Running;
  haveBattery_ = BatteryPort::begin();
  if (haveBattery_) {
    primeBattery();
    const bool switchOn = BatteryPort::switchOn();
    const bool usb = BatteryPort::usbPresent();
    const BatteryStatus& b = battery_.status();
    console_.log("Power: switch %s, USB %s", switchOn ? "on" : "off", usb ? "present" : "absent");
    switch (policy_.begin(switchOn, usb, b.valid && b.freshVoltage < config::kBatteryCutoffV)) {
      case PowerAction::StopForSwitch:
        first = RunState::BatteryWait;
        break;
      case PowerAction::EnterSystemOff:
        enterSystemOff();
        break;
      default:
        break;
    }
  } else {
    scheduler_.setEnabled(kBatteryJob, false);
    scheduler_.setEnabled(kPowerJob, false);
  }

  enterState(first);

  for (size_t i = 0; i < kJobs; i++) {
    if (scheduler_.enabled(i)) {
      console_.log("Job %s: every %lu us, budget %lu us", jobs_[i].name,
                   static_cast<unsigned long>(jobs_[i].periodUs),
                   static_cast<unsigned long>(jobs_[i].budgetUs));
    } else {
      console_.log("Job %s: disabled", jobs_[i].name);
    }
  }
  scheduler_.start();
  console_.log("%s. Commands end with Enter: D = report, C = clear, V = version, L = toggle 1s status, "
               "N = N status reports (0-%lu).",
               stateName(), static_cast<unsigned long>(config::kStatusMaxCount));
}

void App::primeBattery() {
  // Nothing time-critical runs yet, so a short blocking run is fine; it gives
  // the first telemetry and the boot-time state decisions a real reading.
  while (battery_.sampling(nowMs())) {
    battery_.addSample(BatteryPort::readCellMv(), nowMs());
    delayMicroseconds(config::kBatteryPeriodUs);
  }
  battery_.updateCharger(BatteryPort::chargerActive(), nowMs());
  const BatteryStatus& b = battery_.status();
  const unsigned long centiV = static_cast<unsigned long>(b.voltage * 100.0f + 0.5f);
  console_.log("Battery: %lu.%02luV %u%%%s \xE2\x9C\x85", centiV / 100, centiV % 100,
               static_cast<unsigned>(b.percent),
               BatteryPort::usbPresent() && b.chargerActive ? " \xE2\x9A\xA1" : "");
}

void App::enterState(RunState s) {
  state_ = s;
  switch (s) {
    case RunState::Running:
      // Peripherals start only once RUNNING is decided (the nRF holds them
      // off until then), so detection lives here: no receiver means this
      // entry fails over to the fault state instead.
      PeripheralPower::gnssRailOn();
      if (switchedPower_) console_.log("Power: GNSS rail on");
      if (!gnss_.begin(console_)) {
        enterState(RunState::GnssFault);
        return;
      }
      if (config::kProtocolUsesImu) {
        imu_.begin(console_);
      } else {
        scheduler_.setEnabled(kImuJob, false);
      }
      {
        // Last: advertise only once there's something to stream.
        char name[32];
        snprintf(name, sizeof(name), "%s %s", activeProfile().model, deviceId_);
        ble_.begin(activeProfile(), name, deviceId_, console_);
      }
      break;
    case RunState::GnssFault:
      // Fatal: log it and start nothing else. The console job stays up so
      // the device still answers D/V; a switch cycle on USB retries.
      console_.log("GNSS not detected at any standard baud rate. Check wiring. Nothing else will start.");
      stopPeripherals();
      if (switchedPower_) console_.log("Power: GNSS rail off");
      break;
    case RunState::BatteryWait:
      stopPeripherals();
      console_.log("Power: switch off on USB - BATTERY_WAIT, waiting for switch on");
      break;
    case RunState::ChargeOnly:
      stopPeripherals();
      console_.log("Power: quick switch cycle - CHARGE_ONLY, charging with everything else off");
      break;
  }
}

void App::stopPeripherals() {
  ble_.end();
  gnss_.end();                     // the UART closes first...
  PeripheralPower::gnssRailOff();  // ...so TX can idle low as the rail goes off
  imu_.end();
  scheduler_.setEnabled(kGnssJob, false);
  scheduler_.setEnabled(kImuJob, false);
  scheduler_.setEnabled(kBleJob, false);
}

void App::restart(const char* why) {
  console_.log("Power: %s - restarting into RUNNING", why);
  console_.flush();
  PeripheralPower::reset();
}

void App::enterSystemOff() {
  console_.log("Power: battery below cutoff - System OFF (plug in USB or cycle the switch to restart)");
  stopPeripherals();
  StatusLedPort::write(false, LedColor::Blue);
  BatteryPort::disableVoltageSense();
  console_.flush();
  PeripheralPower::systemOff();
}

const char* App::stateName() const {
  switch (state_) {
    case RunState::Running:
      return "Running";
    case RunState::GnssFault:
      return "GNSS fault - console only";
    case RunState::BatteryWait:
      return "BATTERY_WAIT - waiting for switch on";
    case RunState::ChargeOnly:
      return "CHARGE_ONLY - charging, everything else off";
  }
  return "?";
}

void App::tick() {
  loops_++;
  PlatformClock::sleepUntilUs(scheduler_.tick());
}

void App::gnssJob(void* self) {
  App* app = static_cast<App*>(self);
  app->gnss_.poll();
  GnssEpoch epoch;
  if (app->gnss_.consumeEpoch(epoch)) {
    app->epochs_.feed(epoch);
    app->speedGate_.feed(epoch);
    app->latest_ = epoch;
    app->haveEpoch_ = true;
    // One packet per epoch, in the same pass that completed it.
    if (app->ble_.connected()) app->sendTelemetry(epoch);
  }
}

void App::sendTelemetry(const GnssEpoch& epoch) {
  switch (config::kProtocol) {
    case config::Protocol::RaceBox: {
      uint8_t packet[kRaceBoxPacketBytes];
      encodeRaceBoxData(epoch, imu_.read(), raceBoxBatteryByte(), packet);
      ble_.send(packet, sizeof(packet));
      break;
    }
  }
}

uint8_t App::raceBoxBatteryByte() const {
  if (!haveBattery_ || !battery_.status().valid) return kRaceBoxNoBattery;
  const BatteryStatus& b = battery_.status();
  const bool charging = BatteryPort::usbPresent() && b.chargerActive;
  return static_cast<uint8_t>((b.percent & 0x7F) | (charging ? 0x80 : 0));
}

// Runs in every state, GnssFault included: the low-voltage cutoff has to keep
// watching even when nothing else is allowed to start.
void App::batteryJob(void* self) {
  App* app = static_cast<App*>(self);
  const uint32_t now = nowMs();
  if (app->battery_.sampling(now)) app->battery_.addSample(BatteryPort::readCellMv(), now);
  app->battery_.updateCharger(BatteryPort::chargerActive(), now);
  app->ble_.setBatteryPercent(app->battery_.status().percent);
}

// Every 50 ms on battery boards: reads the switch, applies the policy's
// decision. Runs in every state.
void App::powerJob(void* self) {
  App* app = static_cast<App*>(self);
  switch (app->policy_.update(nowMs(), BatteryPort::switchOn(), BatteryPort::usbPresent(),
                              app->battery_.status().cutoffRequested)) {
    case PowerAction::None:
      break;
    case PowerAction::StopForSwitch:
      app->enterState(RunState::BatteryWait);
      break;
    case PowerAction::EnterChargeOnly:
      app->enterState(RunState::ChargeOnly);
      break;
    // restart() and enterSystemOff() never return; the breaks keep each case
    // self-contained anyway, so a future returning path can't fall through.
    case PowerAction::ResetForSwitchOn:
      app->restart("switch on");
      break;
    case PowerAction::ResetForUsbRemoval:
      app->restart("USB removed");
      break;
    case PowerAction::EnterSystemOff:
      app->enterSystemOff();
      break;
  }
}

void App::imuJob(void* self) {
  App* app = static_cast<App*>(self);
  app->imu_.poll(app->speedGate_.speedMps(), app->speedGate_.valid(PlatformClock::nowUs()));
}

void App::bleJob(void* self) {
  App* app = static_cast<App*>(self);
  app->ble_.update();
  // Inbound command bytes arrive here, already off the BLE stack's task. No
  // commands are handled yet, so they're drained and dropped; the protocol's
  // command decoder will consume them. The D report counts what arrived.
  uint8_t buf[32];
  while (app->ble_.read(buf, sizeof(buf)) > 0) {
  }
}

void App::ledJob(void* self) {
  App* app = static_cast<App*>(self);
  const BatteryStatus* b = app->batteryStatus();
  const bool batteryKnown = b && b->valid;
  LedState s = LedState::Off;
  switch (app->state_) {
    case RunState::Running:
      // A battery problem outranks the BLE status: it matters whatever the
      // phone is doing. Charging isn't shown here - only in CHARGE_ONLY.
      if (batteryKnown && b->critical) {
        s = LedState::BatteryCritical;
      } else if (batteryKnown && b->warning) {
        s = LedState::BatteryWarning;
      } else {
        s = app->ble_.connected() ? LedState::Connected : LedState::Advertising;
      }
      break;
    case RunState::GnssFault:
      s = LedState::Fault;
      break;
    case RunState::BatteryWait:
      s = LedState::BatteryWait;
      break;
    case RunState::ChargeOnly:
      s = batteryKnown && b->chargeFull ? LedState::Charged : LedState::Charging;
      break;
  }
  const LedOutput out = app->statusLed_.update(s, static_cast<uint32_t>(PlatformClock::nowUs() / 1000));
  StatusLedPort::write(out.on, out.color);
}

void App::consoleJob(void* self) {
  App* app = static_cast<App*>(self);
  const ConsoleCommand cmd = app->console_.poll();
  if (cmd.cmd == 'D') {
    int32_t accelMg[3], gyroCdegS[3];
    toDisplayUnits(app->imu_.read(), accelMg, gyroCdegS);
    app->console_.printReport({app->scheduler_.jobs(), app->scheduler_.allStats(), kJobs,
                               app->loops_, PlatformClock::nowUs(), &app->epochs_,
                               app->haveEpoch_ ? &app->latest_ : nullptr, app->imu_.up(),
                               {accelMg[0], accelMg[1], accelMg[2]},
                               {gyroCdegS[0], gyroCdegS[1], gyroCdegS[2]}, app->imu_.trimConverged(),
                               static_cast<int32_t>(app->imu_.trimTiltDegrees() * 100.0f),
                               app->ble_.connected(), app->ble_.stats(), app->batteryStatus(),
                               BatteryPort::usbPresent(), app->console_.dropped()});
  } else if (cmd.cmd == 'C') {
    app->clearStats();
  } else if (cmd.cmd == 'V') {
    app->printBuild();
    app->printDeviceId();
  } else if (cmd.cmd == 'L') {
    // A second 'L' stops whatever's running, continuous or counting down;
    // otherwise it starts continuous reporting.
    app->armStatus(app->statusRemaining_ != 0 ? 0 : -1);
  }
  if (cmd.hasCount) {
    // Typing a number always (re)arms to exactly that count, overriding
    // whatever was running before; 0 stops reporting, same as a second 'L'.
    app->armStatus(static_cast<int32_t>(cmd.count));
  }

  if (app->statusRemaining_ != 0) {
    const uint64_t now = PlatformClock::nowUs();
    if (now >= app->nextStatusDueUs_) {
      app->printStatus(now);
      // Fixed grid, not "now + period": scheduling from now let every line
      // land up to a console tick later than the last, slipping a whole
      // second of RT every ~100 lines. After a long stall, resync to the next
      // grid point rather than firing a burst of catch-up lines.
      app->nextStatusDueUs_ += config::kStatusPeriodUs;
      if (app->nextStatusDueUs_ <= now) app->nextStatusDueUs_ = nextStatusGridUs(now);
      if (app->statusRemaining_ > 0) app->statusRemaining_--;
    }
  }
}

void App::armStatus(int32_t count) {
  statusRemaining_ = count;
  if (count != 0) {
    const uint64_t now = PlatformClock::nowUs();
    gnssRate_.reset(epochs_.epochs(), now);
    bleRate_.reset(ble_.stats().sent, now);
    nextStatusDueUs_ = firstStatusGridUs(now);
  }
}

void App::printStatus(uint64_t nowUs) {
  const uint32_t bleCentiHz = bleRate_.sampleCentiHz(ble_.stats().sent, nowUs);
  const uint32_t gnssCentiHz = gnssRate_.sampleCentiHz(epochs_.epochs(), nowUs);
  int32_t accelMg[3], gyroCdegS[3];
  toDisplayUnits(imu_.read(), accelMg, gyroCdegS);
  console_.printStatus({nowUs, bleCentiHz, gnssCentiHz, batteryStatus(), BatteryPort::usbPresent(),
                        haveEpoch_ ? &latest_ : nullptr,
                        {accelMg[0], accelMg[1], accelMg[2]},
                        {gyroCdegS[0], gyroCdegS[1], gyroCdegS[2]}, imu_.trimConverged(),
                        static_cast<int32_t>(imu_.trimTiltDegrees() * 100.0f)});
}

void App::toDisplayUnits(const ImuSample& imu, int32_t accelMg[3], int32_t gyroCdegS[3]) {
  for (int i = 0; i < 3; i++) {
    accelMg[i] = static_cast<int32_t>(imu.accel[i] * kMps2ToMilliG);
    gyroCdegS[i] = static_cast<int32_t>(imu.gyro[i] * kRadpsToCentiDeg);
  }
}

void App::printBuild() {
  char line[192];
  formatBuildLine(line, sizeof(line), buildInfo());
  console_.log("%s", line);
}

void App::printDeviceId() { console_.log("Device ID: %s", deviceId_); }

void App::clearStats() {
  scheduler_.clearStats();
  epochs_.clear();
  loops_ = 0;
  ble_.clearMaxSend();
}

}  // namespace gnimu
