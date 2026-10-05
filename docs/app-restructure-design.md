# App Restructure: Design for Review

**Status:** proposal only. Nothing here is implemented yet. Each step in [Order of work](#order-of-work) is reviewed and approved on its own before it is built.

**Context:** the items listed under "Between Phases 1 and 2" in [firmware-review-2026-10-03-response.md](firmware-review-2026-10-03-response.md). It addresses the review's TEST-1 point that the untested glue in `App` is where most P0/P1 findings live. It also prepares a tested home for Phase 2's new triggers: REL-2 liveness, REL-4, REL-7 and PWR-1.

---

## Goals

1. **`App.cpp` becomes wiring only.** It constructs the modules, holds the job table, runs the boot sequence, carries out state-machine actions, and ticks. Today it is 437 lines that mix all of that with policy, diagnostics and telemetry. The target is about 120–150 lines.
2. **Every run-state decision is pure and host-tested.** That covers boot classification, transitions, the jobs each state runs, and the LED pattern. It uses the same "decide in core, apply in App" seam as `core/PowerPolicy.h`.
3. **No behaviour change.** The boot log, D report, status line, LED patterns and BLE output stay byte-for-byte the same, except where a step says otherwise. Phase 2 then adds behaviour on top of this structure.
4. **Each step leaves the firmware building and working**, so it can be bench-checked before the next one.

## Non-goals

- Changing what any state *does*. That's Phase 2.
- Templating every module on its hardware types for host testing. Only the state machine, the LED mapping and the formatters get host tests; the other modules stay thin glue over hardware.
- Splitting `Config.h` by module (a separate effort).

---

## Target layout

```
app/
  App.{h,cpp}          wiring: modules, job table, boot, apply plans, tick
  Console.{h,cpp}      unchanged (line-based input, bounded output)
  Config.h             unchanged except constants this work removes
  Diagnostics.{h,cpp}  NEW: console commands, D report, status line, rates
  GnssFeed.{h,cpp}     NEW: epoch bookkeeping, speed gate, liveness (Phase 2)
  protocol/
    Protocol.h         NEW: compile-time alias for the selected protocol
    RaceBox.{h,cpp}    NEW: BLE profile, packet encoding, inbound commands
core/
  RunStateMachine.h    NEW: states, events, plans; owns PowerPolicy
  DeviceStatus.h       NEW: one snapshot struct for the D report and status line
  Report.h, StatusLine.h  format from DeviceStatus instead of their own views
  Scheduler.h          adds a member-function job helper
platform/*/BoardConfig.h  adds kHasBattery, kHasSwitchedPower
```

---

## 1. Run-state machine (`core/RunStateMachine.h`)

### Shape

```cpp
enum class RunState : uint8_t { Running, Fault, BatteryWait, ChargeOnly };
enum class FaultReason : uint8_t { None, GnssMissing /*, Phase 2: PvtEnableFailed, BleStackFailed */ };

enum class Action : uint8_t {
  GnssRailOn, StartGnss, StartImu, StartBle,
  StopPeripherals,   // App's existing hardware-safe sequence: BLE, GNSS UART, rail, IMU
  Restart,           // reason in Plan::restartWhy
  SystemOff,
};

struct Plan {
  RunState state;          // state after this plan
  bool entered;            // true when the state changed (App logs the entry line)
  FaultReason fault;
  JobSet jobs;             // which jobs run in `state` (bitmask by job id)
  Action actions[4];       // ordered; App applies them in sequence
  uint8_t actionCount;
  RestartReason restartWhy;
};

struct RunConfig { bool hasBattery; bool protocolUsesImu; uint32_t switchOffDebounceMs, chargeOnlyWindowMs; };

class RunStateMachine {
 public:
  explicit RunStateMachine(const RunConfig&);
  Plan boot(bool switchOn, bool usbPresent, bool belowCutoff);  // hasBattery=false ignores inputs
  Plan gnssStarted(bool detected);                               // result of Action::StartGnss
  Plan power(uint32_t nowMs, bool switchOn, bool usbPresent, bool cutoffRequested);
  // Phase 2 adds: pvtEnableFailed(), gnssSilent(), bleStackFailed()
  RunState state() const;
};
```

- **It owns `PowerPolicy`** and feeds it the raw switch, USB and cutoff inputs. `powerJob` becomes a single call, `apply(machine_.power(...))`. `PowerPolicy` stays as it is, with its existing tests, so PWR-1 in Phase 2 is still a change to `PowerPolicy`.
- **GNSS detection is an event, not a hidden branch.** Entering Running returns `[GnssRailOn, StartGnss]`. App runs `gnss_.begin()` and reports the result with `gnssStarted(ok)`. That returns either `[StartImu?, StartBle]` or a Fault plan. "A missing GNSS never starts BLE" then becomes a host test instead of an ordering detail inside `enterState()`.
- **`Fault` replaces `GnssFault`, with a reason.** Phase 2's PVT-enable failure (REL-4) and BLE stack failure (REL-7) are faults too. They need the same LED and stop behaviour, and only the log line differs. Today the only reason is `GnssMissing`, so behaviour is identical.

### Transitions (current behaviour, unchanged)

| From | Event | To | Actions | Notes |
|---|---|---|---|---|
| (boot) | no battery board | Running | GnssRailOn, StartGnss | ESP32 |
| (boot) | switch off | BatteryWait | StopPeripherals | |
| (boot) | on battery, below cutoff | (none) | SystemOff | never powers the GNSS |
| (boot) | otherwise | Running | GnssRailOn, StartGnss | |
| Running | `gnssStarted(true)` | Running | StartImu (if protocol uses IMU), StartBle | |
| Running | `gnssStarted(false)` | Fault(GnssMissing) | StopPeripherals | |
| Running / Fault | power: StopForSwitch | BatteryWait | StopPeripherals | switch off on USB |
| Running / Fault | power: EnterSystemOff | (none) | SystemOff | cutoff on battery |
| BatteryWait | power: EnterChargeOnly | ChargeOnly | (already stopped) | the on-off-on gesture |
| BatteryWait | power: ResetForSwitchOn | (none) | Restart("switch on") | |
| ChargeOnly | power: StopForSwitch | BatteryWait | | |
| ChargeOnly | power: ResetForSwitchOn | (none) | Restart("switch on") | |
| ChargeOnly | power: ResetForUsbRemoval | (none) | Restart("USB removed") | |

Phase 2 will add rows for `pvtEnableFailed` → Fault, `bleStackFailed` → Fault and `gnssSilent` → Restart, each with its own host test.

### Jobs per state

| State | gnss | imu | battery | power | ble | led | console |
|---|---|---|---|---|---|---|---|
| Running | ✓ | ✓ if protocol uses IMU | ✓ if battery | ✓ if battery | ✓ | ✓ | ✓ |
| Fault, BatteryWait, ChargeOnly | | | ✓ if battery | ✓ if battery | | ✓ | ✓ |

The table is a pure function `jobsFor(RunState, RunConfig)`. It replaces the scattered `setEnabled()` calls in `stopPeripherals()`, `begin()` and the Running branch. App applies `plan.jobs` with one loop.

### LED mapping

`ledStateFor(RunState, LedInputs)` is a pure function next to the machine, moved out of `ledJob`:

| State | LED |
|---|---|
| Running | battery critical → BatteryCritical; else warning → BatteryWarning; else connected → Connected, else Advertising |
| Fault | Fault |
| BatteryWait | BatteryWait |
| ChargeOnly | charge full → Charged, else Charging |

`StatusLed` keeps rendering the patterns, and `ledJob` becomes a single call.

### Text

The state-entry log lines and the final boot line's state name (`stateName()`) move into a pure lookup next to the machine, so tests can check them. App prints them when `plan.entered` is true.

### Tests (`tests/test_runstatemachine.cpp`)

- Every row of the transition table, for both a battery board and a battery-less board.
- `gnssStarted(false)` never produces StartImu or StartBle.
- Jobs per state, including the protocol-without-IMU and battery-less cases.
- LED mapping priorities.
- Hardware-safety ordering: StopPeripherals comes before SystemOff and Restart where both appear.

---

## 2. Board capabilities at compile time

```cpp
// platform/esp32/BoardConfig.h
constexpr bool kHasBattery = false;
constexpr bool kHasSwitchedPower = false;
// platform/nrf52840/BoardConfig.h
constexpr bool kHasBattery = true;
constexpr bool kHasSwitchedPower = true;
```

- These replace App's runtime `haveBattery_` (from `BatteryPort::begin()`) and `switchedPower_` (from `holdAllOff()`).
- `RunConfig.hasBattery` comes from `board::kHasBattery`, so the state machine never produces battery-only states on a board that can't have them.
- App's battery and power paths sit behind `if constexpr`. The ESP32's stub `BatteryPort` stays for the few shared callers, such as `usbPresent()` for the status line.
- `holdAllOff()` and `BatteryPort::begin()` keep their return values only where the boot log uses them.

---

## 3. One device-status snapshot (`core/DeviceStatus.h`)

```cpp
struct DeviceStatus {
  uint64_t uptimeUs;
  struct { const GnssEpoch* latest; const EpochTracker* epochs; uint32_t centiHz; } gnss;
  struct { bool up; int32_t accelMilliG[3]; int32_t gyroCentiDegS[3]; bool trimLocked; int32_t tiltCentiDeg; } imu;
  struct { bool connected; BleStats stats; uint32_t centiHz; } ble;
  struct { const BatteryStatus* battery; bool usbPresent; } power;
  uint32_t consoleDropped;
};
```

- `formatReport(buf, cap, jobs, status)` and `formatStatus(buf, cap, status)` format from it. `ReportView` and `StatusView` go away.
- Diagnostics fills it in one function, including the unit conversions now duplicated in `consoleJob` and `printStatus`.
- Tests build a `DeviceStatus` by named fields (`s.ble.connected = true;`), so adding a field no longer shifts every positional initialiser.
- The rates (`centiHz`) are only meaningful for the status line. The report ignores them, and Diagnostics fills them only when it prints a status line.

---

## 4. Diagnostics module (`app/Diagnostics.{h,cpp}`)

These move out of App unchanged: `consoleJob`'s command handling, `printReport` assembly, `armStatus`, the status-line grid and the rate trackers, `toDisplayUnits`, `printBuild`/`printDeviceId`, and `clearStats`.

- It is constructed with references to what it reports on: scheduler, GnssFeed, IMU, BLE, battery monitor, console, device ID. It only *reads* them, except for `C`, which clears the scheduler stats, epoch stats, loop count and BLE max-send.
- App's console job becomes one call, `diagnostics_.tick()`.

---

## 5. GnssFeed and the protocol module

### `app/GnssFeed.{h,cpp}`

Owns `EpochTracker`, the latest epoch and `GnssSpeedGate`, which App holds today.

- `gnssJob` becomes `if (feed_.poll(gnss_)) protocol_.onEpoch(feed_.latest(), ...)`.
- Phase 2's 2 s liveness check (REL-2) lives here, because it already has every arrival time. It reports `silent()`, and App turns that into `machine_.gnssSilent()`.

### `app/protocol/RaceBox.{h,cpp}` and `Protocol.h`

- One class per protocol. It owns its BLE profile, encodes a packet from epoch + IMU + battery and sends it, and handles inbound bytes.
- That replaces the `config::kProtocol` switches in `activeProfile()` and `sendTelemetry()`, `raceBoxBatteryByte()`, and `bleJob`'s drain loop.
- `Protocol.h` selects it at compile time (`using Protocol = RaceBoxProtocol;`), the same pattern as `BleTransport.h` and `ImuDevice.h`. There is no virtual dispatch.
- It is the home of the future RaceBox command decoder and on-demand calibration, and the place SEC-3's input rules apply: fixed frames, a length check before copying, checksum before acting, rate-limited side effects.
- `kProtocolUsesImu` becomes a static member of the protocol class.

---

## 6. Member-function jobs and a compile-time job table

```cpp
// Scheduler.h
template <class T, void (T::*M)()>
void memberJob(void* self) { (static_cast<T*>(self)->*M)(); }

// App.cpp
constexpr JobSpec kJobSpecs[] = {
  {"gnss",    config::kGnssDrainPeriodUs, config::kGnssDrainBudgetUs},
  {"imu",     config::kImuSamplePeriodUs, config::kImuSampleBudgetUs},
  ...
};
static_assert(allPeriodsPositive(kJobSpecs), "every job period must be positive");
```

- Jobs become ordinary member functions (`void App::gnssJob()`), with no `void* self` cast or `app->` on every line.
- Names, periods and budgets live in one compile-time table checked in one place. That replaces MIN-2's hand-maintained list in `Config.h`, and a job added to the table is checked automatically.
- The runtime `Job[]` is built from the specs plus `memberJob<App, &App::xJob>` and `this`.
- Job ids (`kGnssJob`…) stay the shared index for the scheduler and `JobSet`.

---

## What App looks like afterwards (outline)

```cpp
class App {
 public:
  void begin();   // hold off, console, IDs, clock, battery prime, apply(machine_.boot(...)), log jobs, start
  void tick();    // loops_++, sleep until next deadline

 private:
  void apply(const Plan& p);  // log entry line, run actions in order, set jobs
  bool startGnss();           // gnss_.begin(); then apply(machine_.gnssStarted(ok))
  void stopPeripherals();     // unchanged hardware-safe order
  [[noreturn]] void restart(RestartReason);
  [[noreturn]] void enterSystemOff();

  void gnssJob();     // feed_.poll → protocol_.onEpoch
  void imuJob();      // imu_.poll(feed_ speed gate)
  void batteryJob();  // if constexpr battery
  void powerJob();    // apply(machine_.power(...))
  void bleJob();      // ble_.update(); protocol_.onRx(ble_)
  void ledJob();      // StatusLedPort::write(statusLed_.update(ledStateFor(...)))
  void consoleJob();  // diagnostics_.tick()

  // modules: scheduler_, gnss_, imu_, ble_, console_, feed_, protocol_, battery_,
  // statusLed_, machine_, diagnostics_
};
```

---

## Order of work

Each step is one item: described, approved, built, host-tested, then compiled and bench-checked on both boards before the next starts. Steps 1–2 are mechanical and shrink what the later steps have to move.

| Step | Content | Behaviour change | Bench check |
|---|---|---|---|
| R1 | Member-function jobs + compile-time job specs (§6) | none | boot log job lines identical |
| R2 | Board capability constants (§2) | none | both boards boot as before |
| R3 | `RunStateMachine` + `jobsFor` + `ledStateFor` + entry text; App applies plans (§1) | none (`GnssFault` is renamed `Fault` internally; log text unchanged) | every power transition as in the nRF step 4 checks; GNSS-unplugged boot; LED states |
| R4 | `DeviceStatus` snapshot replaces `ReportView`/`StatusView` (§3) | none | D report and status line byte-identical |
| R5 | Diagnostics module (§4) | none | all console commands |
| R6 | GnssFeed + RaceBox protocol module (§5) | none | BLE streaming to the app; trim lock |

After R6, Phase 2 starts, adding its events to the state machine with tests first.

---

## Open questions (to settle one at a time, before or during the relevant step)

1. **`Fault` with a reason** instead of `GnssFault` (R3). Same LED and same stop behaviour; it prepares REL-4 and REL-7.
2. **The machine owns `PowerPolicy`** (R3), rather than App feeding it `PowerAction`s. One input path, with all transitions tested together.
3. **GNSS start as a two-step event** (`[GnssRailOn, StartGnss]`, then `gnssStarted(ok)`) (R3), rather than one "start peripherals" action that returns a result struct.
4. **Where the state-entry log text lives** (R3): a pure lookup in core, so it's testable, or kept in App.
5. **The `app/protocol/` subfolder** (R6), or flat in `app/`.
6. **Whether `DeviceStatus` holds pointers** to the tracker, epoch and battery as today (no copies), or copies the values it shows (R4). Pointers keep it small. Copies make a snapshot truly a moment in time.
