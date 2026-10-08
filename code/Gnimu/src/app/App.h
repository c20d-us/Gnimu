#pragma once

#include "../core/BatteryMonitor.h"
#include "../core/GnssFeed.h"
#include "../core/RateTracker.h"
#include "../core/RunStateMachine.h"
#include "../core/Scheduler.h"
#include "../core/StatusLed.h"
#include "../platform/Clock.h"
#include "../peripherals/ble/BleTransport.h"
#include "../peripherals/gnss/Gnss.h"
#include "../peripherals/imu/ImuDevice.h"
#include "Config.h"
#include "Console.h"
#include "protocol/Protocol.h"

namespace gnimu {

class App {
 public:
  App();
  void begin();
  void tick();

 private:
  // Job ids: indices into kJobSpecs and the scheduler, in priority order.
  enum : size_t { kGnssJob, kImuJob, kBatteryJob, kPowerJob, kBleJob, kLedJob, kConsoleJob, kJobs };

  void gnssJob();
  void imuJob();
  void batteryJob();
  void powerJob();
  void bleJob();
  void ledJob();
  void consoleJob();
  // One line per job, each naming its id; the constructor checks the table at
  // compile time (id order, names, positive periods).
  static constexpr JobSpec<App> kJobSpecs[kJobs] = {
      {kGnssJob, "gnss", config::kGnssDrainPeriodUs, config::kGnssDrainBudgetUs, &App::gnssJob},
      {kImuJob, "imu", config::kImuSamplePeriodUs, config::kImuSampleBudgetUs, &App::imuJob},
      {kBatteryJob, "battery", config::kBatteryPeriodUs, config::kBatteryBudgetUs, &App::batteryJob},
      {kPowerJob, "power", config::kPowerPeriodUs, config::kPowerBudgetUs, &App::powerJob},
      {kBleJob, "ble", config::kBlePeriodUs, config::kBleBudgetUs, &App::bleJob},
      {kLedJob, "led", config::kLedPeriodUs, config::kLedBudgetUs, &App::ledJob},
      {kConsoleJob, "console", config::kConsolePeriodUs, config::kConsoleBudgetUs, &App::consoleJob},
  };
  // One blocking sampling run (~50 ms) at boot, before the GNSS starts.
  void primeBattery();
  // nullptr on boards without a battery.
  const BatteryStatus* batteryStatus() const;
  // Carries out a state-machine Transition: entering a state sets its jobs and
  // runs its entry effects, following Starting's GNSS result on into Running
  // or Fault. Restart and SystemOff don't return.
  void apply(Transition t);
  // One state's fixed entry effects - the only place peripherals start or
  // stop. Returns the follow-up transition (Starting reports the GNSS
  // result); Stay for the others.
  Transition enter(RunState s);
  void setJobs(const JobSet& j);
  // GNSS, IMU and BLE off in hardware-safe order. Idempotent: safe when
  // nothing was started (a boot straight into waiting).
  void stopPeripherals();
  // Back to RUNNING always goes through a fresh boot (NG's choice): it reruns
  // the verified bring-up instead of restarting peripherals in place.
  [[noreturn]] void restart(const char* why);
  [[noreturn]] void enterSystemOff();
  void clearStats();
  void printBuild();
  void printDeviceId();
  // Arms (or disarms, with count == 0) periodic status reporting and resets
  // the rate tracker's baseline so the first line's GNSS Hz reflects only the
  // interval after arming, not however long epochs have been accumulating.
  void armStatus(int32_t count);
  void printStatus(uint64_t nowUs);
  // Physical units (m/s^2, rad/s) to display units, same factors as Classic's
  // protocol conversion; unlike the protocol path there's no int16 saturation
  // here, since these are debug prints, not a wire format. Shared by the `D`
  // report and the status line so the conversion lives in exactly one place.
  static void toDisplayUnits(const ImuSample& imu, int32_t accelMg[3], int32_t gyroCdegS[3]);

  MemberJobs<App, kJobs> jobs_;
  Scheduler<kJobs, PlatformClock> scheduler_;
  Gnss gnss_;
  ImuDevice imu_;
  BleTransport ble_;
  Console console_;
  char deviceId_[11] = {};  // 10 digits; also the BLE name suffix and DIS serial
  GnssFeed feed_;
  Protocol protocol_;
  RateTracker gnssRate_;  // status line: epochs/s
  RateTracker bleRate_;   // status line: packets sent/s
  StatusLed statusLed_;
  BatteryMonitor battery_;
  RunStateMachine machine_;
  uint32_t loops_ = 0;
  // -1 = continuous (until 'L' or a 0-count stops it), 0 = off, >0 = reports
  // remaining before it auto-stops.
  int32_t statusRemaining_ = 0;
  uint64_t nextStatusDueUs_ = 0;
};

}  // namespace gnimu
