// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../code/Gnimu/src/core/RunStateMachine.h"

using namespace gnimu;

namespace {

const RunConfig kBattery = {true, true, {500}};
const RunConfig kNoBattery = {false, true, {500}};
const RunConfig kNoImu = {true, false, {500}};

bool enters(const Transition& t, RunState s) { return t.kind == Transition::Kind::Enter && t.to == s; }
bool stays(const Transition& t) { return t.kind == Transition::Kind::Stay; }
bool restarts(const Transition& t, RestartReason r) {
  return t.kind == Transition::Kind::Restart && t.why == r;
}
bool systemOff(const Transition& t) { return t.kind == Transition::Kind::SystemOff; }

// Boots a battery board on USB with the switch on, through Starting, into Running.
RunStateMachine running() {
  RunStateMachine m(kBattery);
  assert(enters(m.boot(true, true, false), RunState::Starting));
  assert(enters(m.started(FaultReason::None), RunState::Running));
  return m;
}

// The switch held in one position for `ms` (500 ms debounce, 50 ms power
// ticks); returns the first non-Stay transition.
Transition switchFor(RunStateMachine& m, bool on, uint32_t startMs, uint32_t ms, bool usb = true) {
  for (uint32_t t = startMs; t <= startMs + ms; t += 50) {
    const Transition r = m.power(t, on, usb, false);
    if (!stays(r)) return r;
  }
  return Transition::stay();
}
Transition switchOffFor(RunStateMachine& m, uint32_t startMs, uint32_t ms, bool usb = true) {
  return switchFor(m, false, startMs, ms, usb);
}
Transition switchOnFor(RunStateMachine& m, uint32_t startMs, uint32_t ms, bool usb = true) {
  return switchFor(m, true, startMs, ms, usb);
}

}  // namespace

static void testBootClassification() {
  RunStateMachine noBattery(kNoBattery);
  // Without a battery the inputs are ignored: always Starting.
  assert(enters(noBattery.boot(false, false, true), RunState::Starting));

  RunStateMachine off(kBattery);
  assert(enters(off.boot(false, true, false), RunState::BatteryWait));

  RunStateMachine flat(kBattery);
  assert(systemOff(flat.boot(true, false, true)));

  RunStateMachine onUsbLow(kBattery);  // below cutoff but on USB: run
  assert(enters(onUsbLow.boot(true, true, true), RunState::Starting));
}

static void testRunningOnlyWhenEverythingStarted() {
  RunStateMachine m(kBattery);
  m.boot(true, true, false);
  assert(m.state() == RunState::Starting);
  assert(enters(m.started(FaultReason::GnssMissing), RunState::Fault));
  assert(m.fault() == FaultReason::GnssMissing);
  // Once faulted, a late success can't reach Running.
  assert(stays(m.started(FaultReason::None)));
  assert(m.state() == RunState::Fault);
}

// Each bring-up failure faults with its own reason, for its own message.
static void testBringUpFailureReasons() {
  RunStateMachine config(kBattery);
  config.boot(true, true, false);
  assert(enters(config.started(FaultReason::GnssConfigFailed), RunState::Fault));
  assert(config.fault() == FaultReason::GnssConfigFailed);

  RunStateMachine ble(kBattery);
  ble.boot(true, true, false);
  assert(enters(ble.started(FaultReason::BleFailed), RunState::Fault));
  assert(ble.fault() == FaultReason::BleFailed);

  RunStateMachine noBattery(kNoBattery);
  noBattery.boot(true, true, false);
  assert(enters(noBattery.started(FaultReason::BleFailed), RunState::Fault));
  assert(noBattery.fault() == FaultReason::BleFailed);
}

static void testStartedOutsideStartingStays() {
  RunStateMachine m = running();
  assert(stays(m.started(FaultReason::None)));
  assert(stays(m.started(FaultReason::GnssMissing)));
  assert(stays(m.started(FaultReason::BleFailed)));
  assert(m.state() == RunState::Running);
  assert(m.fault() == FaultReason::None);

  RunStateMachine waiting(kBattery);
  waiting.boot(false, true, false);
  assert(stays(waiting.started(FaultReason::None)));
  assert(waiting.state() == RunState::BatteryWait);
}

static void testSwitchOffOnUsbWaits() {
  RunStateMachine m = running();
  assert(stays(m.power(1000, false, true, false)));  // not yet debounced
  assert(enters(switchOffFor(m, 1000, 600), RunState::BatteryWait));
  assert(m.state() == RunState::BatteryWait);
}

static void testFaultCanStillWaitOrCutOff() {
  RunStateMachine m(kBattery);
  m.boot(true, true, false);
  m.started(FaultReason::GnssMissing);
  assert(enters(switchOffFor(m, 1000, 600), RunState::BatteryWait));

  RunStateMachine cut(kBattery);
  cut.boot(true, false, false);
  cut.started(FaultReason::GnssMissing);
  assert(systemOff(cut.power(1000, true, false, true)));
}

static void testCutoffOnBattery() {
  RunStateMachine m = running();
  assert(stays(m.power(1000, true, true, true)));  // on USB: no cutoff
  assert(systemOff(m.power(1050, true, false, true)));
}

// Plugged in with the switch off, then switched on: CHARGE_ONLY.
static void testBatteryWaitSwitchOnCharges() {
  RunStateMachine m(kBattery);
  m.boot(false, true, false);
  assert(stays(m.power(1000, false, true, false)));
  assert(stays(m.power(1050, true, true, false)));  // on has to hold first
  assert(enters(switchOnFor(m, 1100, 600), RunState::ChargeOnly));
}

static void testChargeOnlyEntryAndExits() {
  RunStateMachine m = running();
  assert(enters(switchOffFor(m, 1000, 600), RunState::BatteryWait));
  // Back on, however long after: CHARGE_ONLY.
  assert(enters(switchOnFor(m, 5000, 600), RunState::ChargeOnly));
  assert(m.state() == RunState::ChargeOnly);
  // Unplugging USB there reboots onto battery.
  assert(restarts(m.power(5600, true, false, false), RestartReason::UsbRemoved));

  RunStateMachine m2 = running();
  switchOffFor(m2, 1000, 600);
  switchOnFor(m2, 1700, 600);
  // Switch off again from CHARGE_ONLY waits; on again reboots into RUNNING.
  assert(enters(switchOffFor(m2, 2400, 600), RunState::BatteryWait));
  assert(restarts(switchOnFor(m2, 3000, 600), RestartReason::SwitchOn));
}

// On battery an OFF reading is noise: the run carries on, and it's counted.
static void testSwitchOffOnBatteryIsNoise() {
  RunStateMachine m(kBattery);
  assert(enters(m.boot(false, false, false), RunState::Starting));
  m.started(FaultReason::None);
  assert(stays(switchOffFor(m, 1000, 3000, false)));
  assert(m.state() == RunState::Running);
  assert(m.switchNoise() == 1);  // one unbroken run, from the boot reading on
  m.clearSwitchNoise();
  assert(m.switchNoise() == 0);
}

static void testNoPowerTransitionsWithoutBattery() {
  RunStateMachine m(kNoBattery);
  m.boot(true, true, false);
  m.started(FaultReason::None);
  assert(stays(m.power(1000, false, false, true)));
  assert(stays(switchOffFor(m, 1000, 2000, false)));
  assert(m.state() == RunState::Running);
}

static void testJobsPerState() {
  const JobSet run = jobsFor(RunState::Running, kBattery);
  assert(run.gnss && run.imu && run.battery && run.power && run.ble && run.led && run.console);

  const RunState stopped[] = {RunState::Fault, RunState::BatteryWait, RunState::ChargeOnly};
  for (RunState s : stopped) {
    const JobSet j = jobsFor(s, kBattery);
    assert(!j.gnss && !j.imu && !j.ble);
    assert(j.battery && j.power && j.led && j.console);
  }

  const JobSet esp = jobsFor(RunState::Running, kNoBattery);
  assert(esp.gnss && esp.imu && esp.ble && !esp.battery && !esp.power);
  const JobSet espFault = jobsFor(RunState::Fault, kNoBattery);
  assert(!espFault.battery && !espFault.power && espFault.led && espFault.console);

  assert(!jobsFor(RunState::Running, kNoImu).imu);
}

static void testLedMapping() {
  LedInputs in = {false, false, false, false};
  assert(ledStateFor(RunState::Running, in) == LedState::Advertising);
  in.bleConnected = true;
  assert(ledStateFor(RunState::Running, in) == LedState::Connected);
  in.batteryWarning = true;
  assert(ledStateFor(RunState::Running, in) == LedState::BatteryWarning);
  in.batteryCritical = true;  // critical outranks warning and BLE
  assert(ledStateFor(RunState::Running, in) == LedState::BatteryCritical);

  // Battery status doesn't change the other states' patterns.
  assert(ledStateFor(RunState::Fault, in) == LedState::Fault);
  assert(ledStateFor(RunState::BatteryWait, in) == LedState::BatteryWait);
  assert(ledStateFor(RunState::Starting, in) == LedState::Off);

  LedInputs charge = {false, false, false, false};
  assert(ledStateFor(RunState::ChargeOnly, charge) == LedState::Charging);
  charge.chargeFull = true;
  assert(ledStateFor(RunState::ChargeOnly, charge) == LedState::Charged);
}

static void testText() {
  assert(strcmp(stateName(RunState::Running), "Running") == 0);
  assert(strcmp(stateName(RunState::Fault), "Fault - console only") == 0);
  assert(strcmp(stateName(RunState::BatteryWait), "BATTERY_WAIT - waiting for switch on") == 0);
  assert(strcmp(stateName(RunState::ChargeOnly), "CHARGE_ONLY - charging, everything else off") == 0);
  assert(strcmp(faultMessage(FaultReason::GnssMissing),
                "GNSS not detected at any standard baud rate. Check wiring. Nothing else will start.") == 0);
  assert(strcmp(entryMessage(RunState::BatteryWait),
                "Power: switch off on USB - BATTERY_WAIT, waiting for switch on") == 0);
  assert(strcmp(entryMessage(RunState::ChargeOnly),
                "Power: switch on with USB - CHARGE_ONLY, charging with everything else off") == 0);
  assert(entryMessage(RunState::Running) == nullptr);
  assert(strcmp(restartReasonText(RestartReason::SwitchOn), "Power: switch on") == 0);
  assert(strcmp(restartReasonText(RestartReason::UsbRemoved), "Power: USB removed") == 0);
  assert(strcmp(restartReasonText(RestartReason::GnssSilent), "GNSS: epochs stopped") == 0);
  assert(strcmp(restartReasonText(RestartReason::Requested), "Restart requested") == 0);
  assert(strcmp(faultMessage(FaultReason::GnssNoData),
                "GNSS detected but sending no data. Check its power supply. Everything else stopped.") == 0);
  assert(strcmp(faultMessage(FaultReason::GnssConfigFailed),
                "GNSS settings could not be applied. Power-cycle to retry. Everything else stopped.") == 0);
  assert(strcmp(faultMessage(FaultReason::BleFailed),
                "BLE failed to start. Power-cycle to retry. Everything else stopped.") == 0);
}

static void testGnssLivenessInRunning() {
  RunStateMachine ok = running();
  assert(stays(ok.gnssLiveness(GnssLiveness::Ok)));

  RunStateMachine stopped = running();
  assert(restarts(stopped.gnssLiveness(GnssLiveness::Stopped), RestartReason::GnssSilent));

  RunStateMachine never = running();
  assert(enters(never.gnssLiveness(GnssLiveness::NeverStarted), RunState::Fault));
  assert(never.fault() == FaultReason::GnssNoData);
}

static void testGnssLivenessIgnoredOutsideRunning() {
  RunStateMachine starting(kBattery);
  starting.boot(true, true, false);
  assert(stays(starting.gnssLiveness(GnssLiveness::NeverStarted)));
  assert(starting.state() == RunState::Starting);

  RunStateMachine fault(kBattery);
  fault.boot(true, true, false);
  fault.started(FaultReason::GnssMissing);
  assert(stays(fault.gnssLiveness(GnssLiveness::Stopped)));
  assert(fault.fault() == FaultReason::GnssMissing);  // the first reason stands

  RunStateMachine waiting(kBattery);
  waiting.boot(false, true, false);
  assert(stays(waiting.gnssLiveness(GnssLiveness::NeverStarted)));
}

// A requested restart is taken in every state; System OFF only with a battery.
static void testRequests() {
  RunStateMachine m = running();
  assert(restarts(m.restartRequested(), RestartReason::Requested));
  const Transition off = m.systemOffRequested();
  assert(systemOff(off) && off.requested);
  assert(systemOff(Transition::systemOff()) && !Transition::systemOff().requested);

  RunStateMachine waiting(kBattery);
  waiting.boot(false, true, false);
  assert(restarts(waiting.restartRequested(), RestartReason::Requested));

  RunStateMachine bare(kNoBattery);
  assert(restarts(bare.restartRequested(), RestartReason::Requested));
  assert(stays(bare.systemOffRequested()));
}

int main() {
  testRequests();
  testBootClassification();
  testRunningOnlyWhenEverythingStarted();
  testBringUpFailureReasons();
  testStartedOutsideStartingStays();
  testSwitchOffOnUsbWaits();
  testFaultCanStillWaitOrCutOff();
  testCutoffOnBattery();
  testBatteryWaitSwitchOnCharges();
  testChargeOnlyEntryAndExits();
  testSwitchOffOnBatteryIsNoise();
  testNoPowerTransitionsWithoutBattery();
  testJobsPerState();
  testLedMapping();
  testGnssLivenessInRunning();
  testGnssLivenessIgnoredOutsideRunning();
  testText();
  puts("runstatemachine: all tests passed");
  return 0;
}
