// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include "App.h"

#include <Arduino.h>
#include <stdio.h>

#include "../core/DeviceId.h"
#include "../platform/BatteryPort.h"
#include "../platform/Board.h"
#include "../platform/HardwareId.h"
#include "../platform/PeripheralPower.h"
#include "../platform/ResetInfo.h"
#include "../platform/StatusLedPort.h"
#include "../platform/Watchdog.h"

namespace gnimu {

// C++11 needs a definition for a static constexpr array that's indexed at runtime.
constexpr JobSpec<App> App::kJobSpecs[App::kJobs];

namespace {

uint32_t nowMs() { return static_cast<uint32_t>(PlatformClock::nowUs() / 1000); }

// What this board and protocol can do, for the state machine and its job sets.
constexpr RunConfig kRunConfig = {board::kHasBattery, Protocol::kUsesImu,
                                  {config::kSwitchDebounceMs}};

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
      machine_(kRunConfig),
      diagnostics_(console_, scheduler_.jobs(), scheduler_.allStats(), kJobs, loops_, feed_, imu_,
                   ble_, board::kHasBattery ? &battery_.status() : nullptr, machine_, deviceId_,
                   reset_) {
  // Here, not in the class: GCC only evaluates it once App is complete.
  static_assert(jobSpecsValid(kJobSpecs), "job specs out of id order, unnamed, or with a zero period");
}

void App::begin() {
  // Before anything else, the console included: every switched peripheral
  // stays off until a state decides otherwise (the nRF's console can wait
  // seconds for USB, and nothing should draw power meanwhile).
  PeripheralPower::holdAllOff();
  // Next, so boot is covered too: a hang anywhere from here on resets the
  // chip. Each boot step below feeds it.
  const bool watchdog = Watchdog::start();
  reset_ = makeResetRecord(ResetInfo::cause(), ResetInfo::takeRestartCode());
  // Before the console: its write wait is timed with nowUs(), which on the
  // nRF stands still until this runs - a host that stops reading would hold
  // that wait forever.
  PlatformClock::begin();
  StatusLedPort::begin();
  console_.begin();
  Watchdog::feed();
  decimalDeviceId(hardwareId(), deviceId_);
  diagnostics_.printBuild();
  diagnostics_.printDeviceId();
  console_.log("Gnimu starting");
  char reset[64];
  formatReset(reset, sizeof(reset), reset_);
  console_.log("Reset: %s", reset);
  if (watchdog) {
    console_.log("Watchdog: %lu ms \xE2\x9C\x85", static_cast<unsigned long>(config::kWatchdogMs));
  } else {
    console_.log("Watchdog: not started \xE2\x9D\x8C");
  }
  if (board::kHasSwitchedPower) console_.log("Power: GNSS rail, IMU and LED held off \xE2\x9C\x85");
  console_.log("Clock started");

  // The state machine decides before anything starts: a switched-off or
  // flat-battery boot never powers the GNSS at all.
  Transition first = Transition::stay();
  if (board::kHasBattery) {
    Watchdog::feed();
    BatteryPort::begin();
    primeBattery();
    const bool usb = BatteryPort::usbPresent();
    // An OFF reading on USB must hold for the switch debounce, as it must at
    // runtime: one ghost OFF would park a switched-on device in BATTERY_WAIT,
    // and the switch-on that follows would take it to CHARGE_ONLY. Without USB
    // an OFF reading counts as ON anyway (see PowerPolicy).
    bool switchOn = BatteryPort::switchOn();
    for (uint32_t waitedMs = 0; !switchOn && usb && waitedMs < config::kSwitchDebounceMs;
         waitedMs += config::kPowerPeriodUs / 1000) {
      delay(config::kPowerPeriodUs / 1000);
      switchOn = BatteryPort::switchOn();
    }
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
  // Two lines: together they'd pass Console::log's 160-character limit.
  console_.log("%s.", stateName(machine_.state()));
  console_.log("Commands end with Enter: D = report, C = clear statistics counters, V = version, "
               "L = 1s status on/off (up to %lu lines), N = N status reports (0-%lu).",
               static_cast<unsigned long>(config::kStatusMaxCount),
               static_cast<unsigned long>(config::kStatusMaxCount));
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
    // Entering a state can be a boot step: Starting brings up every
    // peripheral.
    Watchdog::feed();
    setJobs(jobsFor(t.to, kRunConfig));
    t = enter(t.to);
  }
  if (t.kind == Transition::Kind::Restart) restart(t.why);
  if (t.kind == Transition::Kind::SystemOff) enterSystemOff();
}

Transition App::enter(RunState s) {
  switch (s) {
    case RunState::Starting: {
      // Peripherals start only once a run is decided (the nRF holds them off
      // until then), and all of them start here: Running is only ever
      // entered from this boot-time bring-up. The first failure picks Fault.
      PeripheralPower::gnssRailOn();
      if (board::kHasSwitchedPower) console_.log("Power: GNSS rail on");
      if (!gnss_.begin(console_)) return machine_.started(FaultReason::GnssMissing);
      if (!gnss_.configured()) return machine_.started(FaultReason::GnssConfigFailed);
      Watchdog::feed();
      if (Protocol::kUsesImu) imu_.begin(console_);
      Watchdog::feed();
      // Last: advertise only once there's something to stream.
      char name[32];
      snprintf(name, sizeof(name), "%s %s", Protocol::profile().model, deviceId_);
      const bool advertising = ble_.begin(Protocol::profile(), name, deviceId_, console_);
      // After BLE: once the nRF's SoftDevice runs, the clock request has to
      // go through it. Boot until here runs on whatever clock is up.
      console_.log("Clock: crystal %s",
                   PlatformClock::useCrystal() ? "\xE2\x9C\x85" : "\xE2\x9D\x8C");
      return machine_.started(advertising ? FaultReason::None : FaultReason::BleFailed);
    }
    case RunState::Running:
      break;
    case RunState::Fault:
      // Fatal: log it and start nothing else. The console job stays up so
      // the device still answers D/V. The switch works as in RUNNING: on USB,
      // off/on leads to CHARGE_ONLY, and a second off/on restarts and retries.
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

void App::restart(RestartReason why) {
  console_.log("%s - restarting into RUNNING", restartReasonText(why));
  ResetInfo::storeRestartCode(restartCode(why));
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
  // Every pass, in every state: a job that never returns stops the feeding.
  Watchdog::feed();
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
  // Every command is routed here, to whoever owns it. App owns the counters
  // Clear resets; Diagnostics sees every command too and resets only its own
  // state, after App's.
  switch (cmd.kind) {
    case CommandKind::Clear:
      clearStats();
      break;
    case CommandKind::None:
    case CommandKind::Report:
    case CommandKind::Version:
    case CommandKind::ToggleStatus:
      break;
  }
  diagnostics_.handle(cmd);
  diagnostics_.tick(PlatformClock::nowUs());
}

void App::clearStats() {
  scheduler_.clearStats();
  feed_.clear();
  loops_ = 0;
  ble_.clearMaxSend();
  machine_.clearSwitchNoise();
}

}  // namespace gnimu
