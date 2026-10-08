#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../code/Gnimu/src/core/RunStateMachine.h"

using namespace gnimu;

namespace {

const RunConfig kBattery = {true, true, {500, 2000}};
const RunConfig kNoBattery = {false, true, {500, 2000}};
const RunConfig kNoImu = {true, false, {500, 2000}};

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
  assert(enters(m.gnssStarted(true), RunState::Running));
  return m;
}

// Switch held off long enough to count (500 ms debounce, 50 ms power ticks).
Transition switchOffFor(RunStateMachine& m, uint32_t startMs, uint32_t ms, bool usb = true) {
  Transition last = Transition::stay();
  for (uint32_t t = startMs; t <= startMs + ms; t += 50) {
    const Transition r = m.power(t, false, usb, false);
    if (!stays(r)) last = r;
  }
  return last;
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

static void testRunningOnlyThroughDetectedGnss() {
  RunStateMachine m(kBattery);
  m.boot(true, true, false);
  assert(m.state() == RunState::Starting);
  assert(enters(m.gnssStarted(false), RunState::Fault));
  assert(m.fault() == FaultReason::GnssMissing);
  // Once faulted, a late "detected" can't reach Running.
  assert(stays(m.gnssStarted(true)));
  assert(m.state() == RunState::Fault);
}

static void testGnssStartedOutsideStartingStays() {
  RunStateMachine m = running();
  assert(stays(m.gnssStarted(true)));
  assert(stays(m.gnssStarted(false)));
  assert(m.state() == RunState::Running);

  RunStateMachine waiting(kBattery);
  waiting.boot(false, true, false);
  assert(stays(waiting.gnssStarted(true)));
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
  m.gnssStarted(false);
  assert(enters(switchOffFor(m, 1000, 600), RunState::BatteryWait));

  RunStateMachine cut(kBattery);
  cut.boot(true, false, false);
  cut.gnssStarted(false);
  assert(systemOff(cut.power(1000, true, false, true)));
}

static void testCutoffOnBattery() {
  RunStateMachine m = running();
  assert(stays(m.power(1000, true, true, true)));  // on USB: no cutoff
  assert(systemOff(m.power(1050, true, false, true)));
}

static void testBatteryWaitSwitchOnRestarts() {
  RunStateMachine m(kBattery);
  m.boot(false, true, false);
  assert(stays(m.power(1000, false, true, false)));
  assert(restarts(m.power(1050, true, true, false), RestartReason::SwitchOn));
}

static void testChargeOnlyGestureAndExits() {
  RunStateMachine m = running();
  assert(enters(switchOffFor(m, 1000, 600), RunState::BatteryWait));
  // Back on within the 2 s window from first going off: CHARGE_ONLY.
  assert(enters(m.power(1700, true, true, false), RunState::ChargeOnly));
  assert(m.state() == RunState::ChargeOnly);
  // Unplugging USB there reboots onto battery.
  assert(restarts(m.power(1750, true, false, false), RestartReason::UsbRemoved));

  RunStateMachine m2 = running();
  switchOffFor(m2, 1000, 600);
  m2.power(1700, true, true, false);
  // Switch off again from CHARGE_ONLY waits; on again reboots.
  assert(enters(switchOffFor(m2, 2000, 600), RunState::BatteryWait));
  assert(restarts(m2.power(3000, true, true, false), RestartReason::SwitchOn));
}

static void testNoPowerTransitionsWithoutBattery() {
  RunStateMachine m(kNoBattery);
  m.boot(true, true, false);
  m.gnssStarted(true);
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
  assert(strcmp(stateName(RunState::Fault), "GNSS fault - console only") == 0);
  assert(strcmp(stateName(RunState::BatteryWait), "BATTERY_WAIT - waiting for switch on") == 0);
  assert(strcmp(stateName(RunState::ChargeOnly), "CHARGE_ONLY - charging, everything else off") == 0);
  assert(strcmp(faultMessage(FaultReason::GnssMissing),
                "GNSS not detected at any standard baud rate. Check wiring. Nothing else will start.") == 0);
  assert(strcmp(entryMessage(RunState::BatteryWait),
                "Power: switch off on USB - BATTERY_WAIT, waiting for switch on") == 0);
  assert(strcmp(entryMessage(RunState::ChargeOnly),
                "Power: quick switch cycle - CHARGE_ONLY, charging with everything else off") == 0);
  assert(entryMessage(RunState::Running) == nullptr);
  assert(strcmp(restartReasonText(RestartReason::SwitchOn), "Power: switch on") == 0);
  assert(strcmp(restartReasonText(RestartReason::UsbRemoved), "Power: USB removed") == 0);
  assert(strcmp(restartReasonText(RestartReason::GnssSilent), "GNSS: epochs stopped") == 0);
  assert(strcmp(faultMessage(FaultReason::GnssNoData),
                "GNSS detected but sending no data. Check its power supply. Everything else stopped.") == 0);
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
  fault.gnssStarted(false);
  assert(stays(fault.gnssLiveness(GnssLiveness::Stopped)));
  assert(fault.fault() == FaultReason::GnssMissing);  // the first reason stands

  RunStateMachine waiting(kBattery);
  waiting.boot(false, true, false);
  assert(stays(waiting.gnssLiveness(GnssLiveness::NeverStarted)));
}

int main() {
  testBootClassification();
  testRunningOnlyThroughDetectedGnss();
  testGnssStartedOutsideStartingStays();
  testSwitchOffOnUsbWaits();
  testFaultCanStillWaitOrCutOff();
  testCutoffOnBattery();
  testBatteryWaitSwitchOnRestarts();
  testChargeOnlyGestureAndExits();
  testNoPowerTransitionsWithoutBattery();
  testJobsPerState();
  testLedMapping();
  testGnssLivenessInRunning();
  testGnssLivenessIgnoredOutsideRunning();
  testText();
  puts("runstatemachine: all tests passed");
  return 0;
}
