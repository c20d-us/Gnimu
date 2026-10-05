#pragma once

#include "../core/BatteryMonitor.h"
#include "../core/GnssEpoch.h"
#include "../core/GnssSpeedGate.h"
#include "../core/PowerPolicy.h"
#include "../core/RateTracker.h"
#include "../core/Scheduler.h"
#include "../core/StatusLed.h"
#include "../platform/Clock.h"
#include "../peripherals/ble/BleTransport.h"
#include "../peripherals/gnss/Gnss.h"
#include "../peripherals/imu/ImuDevice.h"
#include "Config.h"
#include "Console.h"

namespace gnimu {

class App {
 public:
  App();
  void begin();
  void tick();

 private:
  // Running: normal operation. GnssFault: no receiver at boot - fatal, so
  // nothing else starts. BatteryWait: switch off while USB holds the MCU up.
  // ChargeOnly: charging with everything else off (NG's switch gesture). In
  // all but Running only the console, battery and power jobs run. System OFF
  // is a one-way exit, not a state.
  enum class RunState : uint8_t { Running, GnssFault, BatteryWait, ChargeOnly };
  // Indices into jobs_, in table (priority) order.
  enum : size_t { kGnssJob, kImuJob, kBatteryJob, kPowerJob, kBleJob, kLedJob, kConsoleJob, kJobs };

  static void gnssJob(void* self);
  static void imuJob(void* self);
  static void batteryJob(void* self);
  static void powerJob(void* self);
  static void bleJob(void* self);
  static void ledJob(void* self);
  static void consoleJob(void* self);
  // Encodes one epoch in the selected protocol and hands it to BLE.
  void sendTelemetry(const GnssEpoch& epoch);
  // Bit 7 charging, bits 0-6 percent; Classic's fixed 100 without a battery.
  uint8_t raceBoxBatteryByte() const;
  // One blocking sampling run (~50 ms) at boot, before the GNSS starts.
  void primeBattery();
  const BatteryStatus* batteryStatus() const { return haveBattery_ ? &battery_.status() : nullptr; }
  // Applies a state's effects - which peripherals start, which jobs run. The
  // one place a state's side effects live, so later transitions extend this
  // rather than scattering begin/end calls.
  void enterState(RunState s);
  // GNSS, IMU and BLE off in hardware-safe order, their jobs disabled.
  // Idempotent: safe when nothing was started (a boot straight into waiting).
  void stopPeripherals();
  // Back to RUNNING always goes through a fresh boot (NG's choice): it reruns
  // the verified bring-up instead of restarting peripherals in place.
  [[noreturn]] void restart(const char* why);
  [[noreturn]] void enterSystemOff();
  const char* stateName() const;
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

  Job jobs_[kJobs];
  Scheduler<kJobs, PlatformClock> scheduler_;
  Gnss gnss_;
  ImuDevice imu_;
  BleTransport ble_;
  Console console_;
  char deviceId_[11] = {};  // 10 digits; also the BLE name suffix and DIS serial
  EpochTracker epochs_;
  GnssSpeedGate speedGate_;
  RateTracker gnssRate_;  // status line: epochs/s
  RateTracker bleRate_;   // status line: packets sent/s
  StatusLed statusLed_;
  BatteryMonitor battery_;
  PowerPolicy policy_;
  GnssEpoch latest_;
  RunState state_ = RunState::Running;
  bool switchedPower_ = false;  // this board has peripheral power to switch (nRF)
  bool haveBattery_ = false;    // this board has a battery to monitor (nRF)
  bool haveEpoch_ = false;
  uint32_t loops_ = 0;
  // -1 = continuous (until 'L' or a 0-count stops it), 0 = off, >0 = reports
  // remaining before it auto-stops.
  int32_t statusRemaining_ = 0;
  uint64_t nextStatusDueUs_ = 0;
};

}  // namespace gnimu
