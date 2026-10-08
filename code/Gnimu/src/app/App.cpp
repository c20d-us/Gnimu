#include "App.h"

#include <Arduino.h>
#include <stdio.h>

#include "../core/DeviceId.h"
#include "../platform/BatteryPort.h"
#include "../platform/Board.h"
#include "../platform/HardwareId.h"
#include "../platform/PeripheralPower.h"
#include "../platform/StatusLedPort.h"
#include "BuildInfo.h"

namespace gnimu {

// C++11 needs a definition for a static constexpr array that's indexed at runtime.
constexpr JobSpec<App> App::kJobSpecs[App::kJobs];

namespace {

uint32_t nowMs() { return static_cast<uint32_t>(PlatformClock::nowUs() / 1000); }

// What this board and protocol can do, for the state machine and its job sets.
constexpr RunConfig kRunConfig = {board::kHasBattery, Protocol::kUsesImu,
                                  {config::kSwitchOffDebounceMs, config::kChargeOnlyWindowMs}};

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
    : jobs_(this, kJobSpecs),
      scheduler_(jobs_.table()),
      feed_(1000 / config::kGnssNavRateHz, config::kImuTrimPvtStaleMs * 1000ULL,
            config::kGnssSilenceMs * 1000ULL),
      statusLed_({config::kLedBlinkHalfMs, config::kLedFaultHalfMs, config::kLedBatteryWaitHalfMs}),
      battery_({config::kBatterySamplesPerRun, config::kBatteryRunIntervalMs, config::kBatteryEmaAlpha,
                config::kBatteryCutoffV, config::kBatteryCriticalV, config::kBatteryWarningV,
                config::kBatteryFullScaleV, config::kBatteryCutoffDebounceMs,
                config::kBatteryChargeFullV, config::kBatteryChargeFullDebounceMs}),
      machine_(kRunConfig) {
  // Here, not in the class: GCC only evaluates it once App is complete.
  static_assert(jobSpecsValid(kJobSpecs), "job specs out of id order, unnamed, or with a zero period");
}

void App::begin() {
  // Before anything else, the console included: every switched peripheral
  // stays off until a state decides otherwise (the nRF's console can wait
  // seconds for USB, and nothing should draw power meanwhile).
  PeripheralPower::holdAllOff();
  StatusLedPort::begin();
  console_.begin();
  decimalDeviceId(hardwareId(), deviceId_);
  printBuild();
  printDeviceId();
  console_.log("Gnimu starting");
  if (board::kHasSwitchedPower) console_.log("Power: GNSS rail, IMU and LED held off \xE2\x9C\x85");

  PlatformClock::begin();
  console_.log("Clock started");

  // The state machine decides before anything starts: a switched-off or
  // flat-battery boot never powers the GNSS at all.
  Transition first = Transition::stay();
  if (board::kHasBattery) {
    BatteryPort::begin();
    primeBattery();
    const bool switchOn = BatteryPort::switchOn();
    const bool usb = BatteryPort::usbPresent();
    const BatteryStatus& b = battery_.status();
    console_.log("Power: switch %s, USB %s", switchOn ? "on" : "off", usb ? "present" : "absent");
    first = machine_.boot(switchOn, usb, b.valid && b.freshVoltage < config::kBatteryCutoffV);
  } else {
    first = machine_.boot(true, true, false);
  }
  apply(first);

  for (size_t i = 0; i < kJobs; i++) {
    if (scheduler_.enabled(i)) {
      console_.log("Job %s: every %lu us, budget %lu us", kJobSpecs[i].name,
                   static_cast<unsigned long>(kJobSpecs[i].periodUs),
                   static_cast<unsigned long>(kJobSpecs[i].budgetUs));
    } else {
      console_.log("Job %s: disabled", kJobSpecs[i].name);
    }
  }
  scheduler_.start();
  console_.log("%s. Commands end with Enter: D = report, C = clear, V = version, L = toggle 1s status, "
               "N = N status reports (0-%lu).",
               stateName(machine_.state()), static_cast<unsigned long>(config::kStatusMaxCount));
}

const BatteryStatus* App::batteryStatus() const {
  return board::kHasBattery ? &battery_.status() : nullptr;
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
               isCharging(b, BatteryPort::usbPresent()) ? " \xE2\x9A\xA1" : "");
}

void App::apply(Transition t) {
  // Only Starting produces a follow-up, so this runs at most twice.
  while (t.kind == Transition::Kind::Enter) {
    setJobs(jobsFor(t.to, kRunConfig));
    t = enter(t.to);
  }
  if (t.kind == Transition::Kind::Restart) restart(restartReasonText(t.why));
  if (t.kind == Transition::Kind::SystemOff) enterSystemOff();
}

Transition App::enter(RunState s) {
  switch (s) {
    case RunState::Starting:
      // Peripherals start only once a run is decided (the nRF holds them off
      // until then). The detection result picks Running or Fault.
      PeripheralPower::gnssRailOn();
      if (board::kHasSwitchedPower) console_.log("Power: GNSS rail on");
      return machine_.gnssStarted(gnss_.begin(console_));
    case RunState::Running:
      if (Protocol::kUsesImu) imu_.begin(console_);
      {
        // Last: advertise only once there's something to stream.
        char name[32];
        snprintf(name, sizeof(name), "%s %s", Protocol::profile().model, deviceId_);
        ble_.begin(Protocol::profile(), name, deviceId_, console_);
      }
      break;
    case RunState::Fault:
      // Fatal: log it and start nothing else. The console job stays up so
      // the device still answers D/V; a switch cycle on USB retries.
      console_.log("%s", faultMessage(machine_.fault()));
      stopPeripherals();
      if (board::kHasSwitchedPower) console_.log("Power: GNSS rail off");
      break;
    case RunState::BatteryWait:
    case RunState::ChargeOnly:
      stopPeripherals();
      console_.log("%s", entryMessage(s));
      break;
  }
  return Transition::stay();
}

void App::setJobs(const JobSet& j) {
  scheduler_.setEnabled(kGnssJob, j.gnss);
  scheduler_.setEnabled(kImuJob, j.imu);
  scheduler_.setEnabled(kBatteryJob, j.battery);
  scheduler_.setEnabled(kPowerJob, j.power);
  scheduler_.setEnabled(kBleJob, j.ble);
  scheduler_.setEnabled(kLedJob, j.led);
  scheduler_.setEnabled(kConsoleJob, j.console);
}

void App::stopPeripherals() {
  ble_.end();
  gnss_.end();                     // the UART closes first...
  PeripheralPower::gnssRailOff();  // ...so TX can idle low as the rail goes off
  imu_.end();
}

void App::restart(const char* why) {
  console_.log("%s - restarting into RUNNING", why);
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

void App::tick() {
  loops_++;
  PlatformClock::sleepUntilUs(scheduler_.tick());
}

void App::gnssJob() {
  gnss_.poll();
  GnssEpoch epoch;
  if (gnss_.consumeEpoch(epoch)) {
    feed_.add(epoch);
    // One packet per epoch, in the same pass that completed it.
    if (ble_.connected()) {
      Protocol::Packet packet;
      protocol_.encode(epoch, imu_.read(), batteryStatus(), BatteryPort::usbPresent(), packet);
      ble_.send(packet, sizeof(packet));
    }
  }
  apply(machine_.gnssLiveness(feed_.liveness(PlatformClock::nowUs())));
}

// Runs in every state, Fault included: the low-voltage cutoff has to keep
// watching even when nothing else is allowed to start.
void App::batteryJob() {
  const uint32_t now = nowMs();
  if (battery_.sampling(now)) battery_.addSample(BatteryPort::readCellMv(), now);
  battery_.updateCharger(BatteryPort::chargerActive(), now);
  ble_.setBatteryPercent(battery_.status().percent);
}

// Every 50 ms on battery boards: reads the switch and applies whatever the
// state machine decides. Runs in every state.
void App::powerJob() {
  apply(machine_.power(nowMs(), BatteryPort::switchOn(), BatteryPort::usbPresent(),
                       battery_.status().cutoffRequested));
}

void App::imuJob() {
  imu_.poll(feed_.speedMps(), feed_.speedValid(PlatformClock::nowUs()));
}

void App::bleJob() {
  ble_.update();
  // Inbound command bytes arrive here, already off the BLE stack's task, and
  // go to the protocol. The D report counts what arrived.
  uint8_t buf[32];
  size_t n;
  while ((n = ble_.read(buf, sizeof(buf))) > 0) protocol_.onRx(buf, n);
}

void App::ledJob() {
  const BatteryStatus* b = batteryStatus();
  const bool known = b && b->valid;
  const LedInputs in = {known && b->critical, known && b->warning, known && b->chargeFull,
                        ble_.connected()};
  const LedOutput out = statusLed_.update(ledStateFor(machine_.state(), in), nowMs());
  StatusLedPort::write(out.on, out.color);
}

void App::consoleJob() {
  const ConsoleCommand cmd = console_.poll();
  if (cmd.cmd == 'D') {
    int32_t accelMg[3], gyroCdegS[3];
    toDisplayUnits(imu_.read(), accelMg, gyroCdegS);
    console_.printReport({scheduler_.jobs(), scheduler_.allStats(), kJobs,
                               loops_, PlatformClock::nowUs(), &feed_.epochs(), feed_.latest(),
                               imu_.up(),
                               {accelMg[0], accelMg[1], accelMg[2]},
                               {gyroCdegS[0], gyroCdegS[1], gyroCdegS[2]}, imu_.trimConverged(),
                               static_cast<int32_t>(imu_.trimTiltDegrees() * 100.0f),
                               ble_.connected(), ble_.stats(), batteryStatus(),
                               BatteryPort::usbPresent(), console_.dropped()});
  } else if (cmd.cmd == 'C') {
    clearStats();
  } else if (cmd.cmd == 'V') {
    printBuild();
    printDeviceId();
  } else if (cmd.cmd == 'L') {
    // A second 'L' stops whatever's running, continuous or counting down;
    // otherwise it starts continuous reporting.
    armStatus(statusRemaining_ != 0 ? 0 : -1);
  }
  if (cmd.hasCount) {
    // Typing a number always (re)arms to exactly that count, overriding
    // whatever was running before; 0 stops reporting, same as a second 'L'.
    armStatus(static_cast<int32_t>(cmd.count));
  }

  if (statusRemaining_ != 0) {
    const uint64_t now = PlatformClock::nowUs();
    if (now >= nextStatusDueUs_) {
      printStatus(now);
      // Fixed grid, not "now + period": scheduling from now let every line
      // land up to a console tick later than the last, slipping a whole
      // second of RT every ~100 lines. After a long stall, resync to the next
      // grid point rather than firing a burst of catch-up lines.
      nextStatusDueUs_ += config::kStatusPeriodUs;
      if (nextStatusDueUs_ <= now) nextStatusDueUs_ = nextStatusGridUs(now);
      if (statusRemaining_ > 0) statusRemaining_--;
    }
  }
}

void App::armStatus(int32_t count) {
  statusRemaining_ = count;
  if (count != 0) {
    const uint64_t now = PlatformClock::nowUs();
    gnssRate_.reset(feed_.epochs().epochs(), now);
    bleRate_.reset(ble_.stats().sent, now);
    nextStatusDueUs_ = firstStatusGridUs(now);
  }
}

void App::printStatus(uint64_t nowUs) {
  const uint32_t bleCentiHz = bleRate_.sampleCentiHz(ble_.stats().sent, nowUs);
  const uint32_t gnssCentiHz = gnssRate_.sampleCentiHz(feed_.epochs().epochs(), nowUs);
  int32_t accelMg[3], gyroCdegS[3];
  toDisplayUnits(imu_.read(), accelMg, gyroCdegS);
  console_.printStatus({nowUs, bleCentiHz, gnssCentiHz, batteryStatus(), BatteryPort::usbPresent(),
                        feed_.latest(),
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
  feed_.clear();
  loops_ = 0;
  ble_.clearMaxSend();
}

}  // namespace gnimu
