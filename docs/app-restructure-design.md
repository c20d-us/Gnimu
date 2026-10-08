# App Restructure: Design for Review

**Status:** proposal only, revised once after a self-review (see [Revision notes](#revision-notes)). Nothing here is implemented yet. Each step in [Order of work](#order-of-work) is reviewed and approved on its own before it is built.

**Context:** the items listed under "Between Phases 1 and 2" in [firmware-review-2026-10-03-response.md](firmware-review-2026-10-03-response.md). It addresses the review's TEST-1 point that the untested glue in `App` is where most P0/P1 findings live. It also prepares a tested home for Phase 2's new triggers: REL-2 liveness, REL-4, REL-7 and PWR-1.

---

## Goals

1. **`App.cpp` becomes wiring only.** It constructs the modules, holds the job table, runs the boot sequence, carries out state-machine actions, and ticks. Today it is 437 lines that mix all of that with policy, diagnostics and telemetry. The target is about 120–150 lines.
2. **Every run-state decision is pure and host-tested.** That covers boot classification, transitions, the jobs each state runs, and the LED pattern. It uses the same "decide in core, apply in App" seam as `core/PowerPolicy.h`.
3. **No behaviour change.** The boot log, D report, status line, LED patterns and BLE output stay byte-for-byte the same, except where a step says otherwise. Phase 2 then adds behaviour on top of this structure.
4. **Each step leaves the firmware building and working**, so it can be bench-checked before the next one.

## Language constraint

The nRF core compiles with **`-std=gnu++11`** (Seeed nRF52 1.1.13 `platform.txt`). The ESP32 core uses a much newer standard, but shared code must stay C++11, as the existing code already does. In practice that means:
- no `if constexpr`; use an ordinary `if` on a `constexpr` constant (the compiler drops the dead branch, but both branches must compile on both boards)
- `constexpr` functions are a single `return` (ternaries and recursion, no loops or `switch`)
- no `std::index_sequence` or fold expressions
- a `static constexpr` array member that's indexed at runtime needs an out-of-class definition

## Non-goals

- Changing what any state *does*. That's Phase 2.
- Templating every module on its hardware types for host testing. Only the state machine, the LED mapping and the formatters get host tests; the other modules stay thin glue over hardware.
- Splitting `Config.h` by module (a separate effort).

---

## Target layout

```
app/
  App.{h,cpp}          wiring: modules, job table, boot, apply transitions, tick
  Console.{h,cpp}      unchanged (line-based input, bounded output)
  Config.h             unchanged except constants this work removes
  Diagnostics.{h,cpp}  NEW: console commands, D report, status line, rates
  GnssFeed.{h,cpp}     NEW: epoch bookkeeping, speed gate, liveness (Phase 2)
  protocol/
    Protocol.h         NEW: compile-time alias for the selected protocol
    RaceBox.{h,cpp}    NEW: BLE profile, packet encoding, inbound commands
core/
  RunStateMachine.h    NEW: states, events, transitions; owns PowerPolicy
  DeviceStatus.h       NEW: one snapshot struct for the D report and status line
  Report.h, StatusLine.h  format from DeviceStatus instead of their own views
  Scheduler.h          adds a member-function job helper
platform/*/BoardConfig.h  adds kHasBattery, kHasSwitchedPower
```

---

## 1. Run-state machine (`core/RunStateMachine.h`)

### Shape

Every hardware effect is tied to *entering a state*, so the machine only has to decide which state comes next. There are no action lists.

```cpp
enum class RunState : uint8_t { Starting, Running, Fault, BatteryWait, ChargeOnly };
enum class FaultReason : uint8_t { None, GnssMissing /*, Phase 2: PvtEnableFailed, BleStackFailed */ };
enum class RestartReason : uint8_t { SwitchOn, UsbRemoved /*, Phase 2: GnssSilent */ };

// What an event leads to: nothing, a new state to enter, or leaving the run
// altogether (reset or System OFF, which App carries out and never returns from).
struct Transition {
  enum class Kind : uint8_t { Stay, Enter, Restart, SystemOff } kind = Kind::Stay;
  RunState to = RunState::Starting;  // Enter only
  FaultReason fault = FaultReason::None;
  RestartReason why = RestartReason::SwitchOn;  // Restart only
};

struct RunConfig { bool hasBattery; bool protocolUsesImu; uint32_t switchOffDebounceMs, chargeOnlyWindowMs; };

class RunStateMachine {
 public:
  explicit RunStateMachine(const RunConfig&);
  Transition boot(bool switchOn, bool usbPresent, bool belowCutoff);  // inputs ignored without a battery
  Transition gnssStarted(bool detected);                               // Stay outside Starting
  Transition power(uint32_t nowMs, bool switchOn, bool usbPresent, bool cutoffRequested);
  // Phase 2 adds: pvtEnableFailed(), gnssSilent(), bleStackFailed()
  RunState state() const;
  FaultReason fault() const;
};
```

**Entry effects, fixed per state, carried out by App:**

| Entering | App does |
|---|---|
| Starting | GNSS rail on, `gnss_.begin()`, then feeds the result to `gnssStarted()` |
| Running | IMU begin (if the protocol uses it), BLE begin |
| Fault | log the reason, stop peripherals |
| BatteryWait, ChargeOnly | log, stop peripherals |

App also sets each state's job set (below) on entry. Restart and SystemOff go to App's existing `restart()` and `enterSystemOff()`, which already stop peripherals in hardware-safe order before resetting or powering off.

- **Starting is a real, brief state.** It covers GNSS bring-up during boot, before the scheduler starts. "A missing GNSS never starts BLE" becomes structural: BLE starts only on *entering Running*, and the only way into Running is `gnssStarted(true)`. The tests check the transition, so there's no ordering to get wrong inside an entry function.
- **It owns `PowerPolicy`** and feeds it the raw switch, USB and cutoff inputs. `powerJob` becomes `apply(machine_.power(...))`. `PowerPolicy` itself is unchanged, with its existing tests, so PWR-1 in Phase 2 is still a change to `PowerPolicy`.
- **`Fault` replaces `GnssFault`, with a reason.** Phase 2's PVT-enable failure (REL-4) and BLE stack failure (REL-7) are faults too, with the same LED and stop behaviour; only the log line differs. Today the only reason is `GnssMissing`, so behaviour is identical.
- **App's `apply()` is a short loop:** enter the state, carry out its entry effects, and if that produced a follow-up transition (Starting's GNSS result), apply that too. That replaces today's `enterState()` calling itself recursively from the Running branch. Only Starting produces a follow-up, so the loop runs at most twice.
- **Every event is defined in every state.** An event that doesn't apply where it arrives returns Stay instead of being an unchecked precondition. Examples: `gnssStarted()` outside Starting, or a power event the policy maps to nothing. That's covered by a test, so a Phase 2 event wired up in the wrong place can't cause an unplanned transition.

### Transitions (current behaviour, unchanged)

| From | Event | Result | Notes |
|---|---|---|---|
| (boot) | no battery board | Enter Starting | ESP32 |
| (boot) | switch off | Enter BatteryWait | only possible with USB |
| (boot) | on battery, below cutoff | SystemOff | never powers the GNSS |
| (boot) | otherwise | Enter Starting | |
| Starting | `gnssStarted(true)` | Enter Running | |
| Starting | `gnssStarted(false)` | Enter Fault(GnssMissing) | |
| Running / Fault | power: StopForSwitch | Enter BatteryWait | switch off on USB |
| Running / Fault | power: EnterSystemOff | SystemOff | cutoff on battery |
| BatteryWait | power: EnterChargeOnly | Enter ChargeOnly | the on-off-on gesture |
| BatteryWait | power: ResetForSwitchOn | Restart(SwitchOn) | |
| ChargeOnly | power: StopForSwitch | Enter BatteryWait | |
| ChargeOnly | power: ResetForSwitchOn | Restart(SwitchOn) | |
| ChargeOnly | power: ResetForUsbRemoval | Restart(UsbRemoved) | |

Phase 2 will add rows for `pvtEnableFailed` → Fault, `bleStackFailed` → Fault and `gnssSilent` → Restart, each with its own host test.

### Jobs per state

| State | gnss | imu | battery | power | ble | led | console |
|---|---|---|---|---|---|---|---|
| Running | ✓ | ✓ if protocol uses IMU | ✓ if battery | ✓ if battery | ✓ | ✓ | ✓ |
| Fault, BatteryWait, ChargeOnly | | | ✓ if battery | ✓ if battery | | ✓ | ✓ |
| Starting | n/a: only at boot, before the scheduler starts | | | | | | |

The table is a pure function `jobsFor(RunState, RunConfig)`. It replaces the scattered `setEnabled()` calls in `stopPeripherals()`, `begin()` and the Running branch. App applies the entered state's set with one loop.

### LED mapping

`ledStateFor(RunState, LedInputs)` is a pure function next to the machine, moved out of `ledJob`:

| State | LED |
|---|---|
| Running | battery critical → BatteryCritical; else warning → BatteryWarning; else connected → Connected, else Advertising |
| Fault | Fault |
| BatteryWait | BatteryWait |
| ChargeOnly | charge full → Charged, else Charging |
| Starting | Off (the LED job isn't running yet; listed so every state is mapped) |

`StatusLed` keeps rendering the patterns, and `ledJob` becomes a single call.

### Text

The state-entry log lines, the fault reasons and the final boot line's state name (`stateName()`) move into a pure lookup next to the machine, so tests can check them. App prints them as part of each state's entry effects.

### Tests (`tests/test_runstatemachine.cpp`)

- Every row of the transition table, for both a battery board and a battery-less board.
- Events that don't apply in the current state return Stay (e.g. `gnssStarted()` in Running or Fault).
- Running is reachable only through `gnssStarted(true)`. From Starting, `gnssStarted(false)` leads to Fault, never Running.
- Jobs per state, including the protocol-without-IMU and battery-less cases.
- LED mapping priorities.
- Entry text and fault-reason text.

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
- App's battery and power paths sit behind ordinary `if (board::kHasBattery)` checks; the compiler removes them on the ESP32. The ESP32's stub `BatteryPort` stays for the few shared callers, such as `usbPresent()` for the status line.
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

- `gnssJob` becomes: if `feed_.poll(gnss_)` produced an epoch and BLE is connected, `ble_.send(protocol_.encode(...))`.
- Phase 2's 2 s liveness check (REL-2) lives here, because it already has every arrival time. It reports `silent()`, and App turns that into `machine_.gnssSilent()`.

### `app/protocol/RaceBox.{h,cpp}` and `Protocol.h`

- One class per protocol. It owns its BLE profile, encodes a packet from epoch + IMU + battery into a buffer, and handles inbound bytes. **It doesn't send.** App hands the encoded packet to the BLE transport, so the send path stays in one place. Phase 4 may move sending to a separate job or task (the 100 ms `notify()` question), and that change then never touches protocol code. Encoding also stays testable without a BLE fake, as `RaceBoxEncoder` is today.
- That replaces the `config::kProtocol` switches in `activeProfile()` and `sendTelemetry()`, `raceBoxBatteryByte()`, and `bleJob`'s drain loop.
- `Protocol.h` selects it at compile time (`using Protocol = RaceBoxProtocol;`), the same pattern as `BleTransport.h` and `ImuDevice.h`. There is no virtual dispatch.
- It is the home of the future RaceBox command decoder and on-demand calibration, and the place SEC-3's input rules apply: fixed frames, a length check before copying, checksum before acting, rate-limited side effects.
- `kProtocolUsesImu` becomes a static member of the protocol class.

---

## 6. Member-function jobs and a compile-time job table

```cpp
// Scheduler.h (generic, host-tested)
template <class T> struct JobSpec {
  size_t id;            // must equal the entry's position in its table
  const char* name;
  uint32_t periodUs;
  uint32_t budgetUs;
  void (T::*run)();
};
// True when every entry sits at its own id and has a name and a positive period.
// (`run` can't be checked: GCC 9 won't compare member-function pointers in a
// constant expression.)
template <class T, size_t N>
constexpr bool jobSpecsValid(const JobSpec<T> (&s)[N], size_t i = 0) {
  return i == N || (s[i].id == i && s[i].name && s[i].periodUs > 0 && jobSpecsValid(s, i + 1));
}
// Binds a spec table to an object: builds the scheduler's Job[] and dispatches
// each run to the right member function.
template <class T, size_t N> class MemberJobs { ... const Job (&table() const)[N]; };

// App.h (C++11: a static constexpr member is initialised in the class)
static constexpr JobSpec<App> kJobSpecs[kJobs] = {
  {kGnssJob, "gnss", config::kGnssDrainPeriodUs, config::kGnssDrainBudgetUs, &App::gnssJob},
  {kImuJob,  "imu",  config::kImuSamplePeriodUs, config::kImuSampleBudgetUs, &App::imuJob},
  ...
};
// App.cpp
constexpr JobSpec<App> App::kJobSpecs[App::kJobs];  // definition, as C++11 requires
App::App() : jobs_(this, kJobSpecs), scheduler_(jobs_.table()), ... {
  // Not in the class body: GCC only evaluates it once App is complete.
  static_assert(jobSpecsValid(kJobSpecs), "job specs out of id order, unnamed, or with a zero period");
}
```

- Jobs become ordinary member functions (`void App::gnssJob()`), with no `void* self` cast or `app->` on every line.
- **Names, periods, budgets and functions live in one compile-time table checked in one place.** That replaces MIN-2's hand-maintained list in `Config.h`, and a job added to the table is checked automatically.
- **The job ids are checked against the table at compile time.** Today the ids (`kGnssJob`…) and the table entries line up only by convention. A reordered or missing entry would silently swap jobs, so `setEnabled(kImuJob, …)` would switch off the battery job, and a missing entry would be zero-filled with a 0 period. Each entry carries its id, and the array is sized `[kJobs]`, so a missing entry, a reordered one, an unnamed one or a zero period fails the build. The function isn't checked at compile time (GCC 9 limitation); a missing one would be an entry visibly short a field.
- `MemberJobs` builds the runtime `Job[]` from the table in a loop. Each job's context points at a small binding (object plus member function), and one shared dispatcher calls it. That costs one extra indirect call per job run (about 1,100 a second), which is negligible.

---

## What App looks like afterwards (outline)

```cpp
class App {
 public:
  void begin();   // hold off, console, IDs, clock, battery prime, apply(machine_.boot(...)), log jobs, start scheduler
  void tick();    // loops_++, sleep until next deadline

 private:
  void apply(Transition t);   // loop: enter state, run its entry effects, set jobs,
                              // follow Starting's GNSS result; Restart/SystemOff never return
  Transition enter(RunState); // the per-state entry effects table above
  void stopPeripherals();     // unchanged hardware-safe order
  [[noreturn]] void restart(RestartReason);
  [[noreturn]] void enterSystemOff();

  void gnssJob();     // feed_.poll → ble_.send(protocol_.encode(...))
  void imuJob();      // imu_.poll(feed_ speed gate)
  void batteryJob();  // battery boards only (if board::kHasBattery)
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
| R1 | Member-function jobs + compile-time job specs with id checks (§6) | none | boot log job lines identical |
| R2 | Board capability constants (§2) | none | both boards boot as before |
| R3 | `RunStateMachine` + `jobsFor` + `ledStateFor` + entry text; App applies transitions (§1) | none (`GnssFault` is renamed `Fault`, and boot passes through `Starting`; log text unchanged) | every power transition as in the nRF step 4 checks; GNSS-unplugged boot; LED states |
| R4 | Diagnostics module (§4): a pure move of the console code out of App | none | all console commands |
| R5 | `DeviceStatus` snapshot replaces `ReportView`/`StatusView` (§3), changed in its final home in Diagnostics | none | D report and status line byte-identical |
| R6 | GnssFeed + RaceBox protocol module (§5) | none | BLE streaming to the app; trim lock |

After R6, Phase 2 starts, adding its events to the state machine with tests first.

---

## Open questions (to settle one at a time, before or during the relevant step)

1. **`Fault` with a reason** instead of `GnssFault` (R3). Same LED and same stop behaviour; it prepares REL-4 and REL-7.
2. **The machine owns `PowerPolicy`** (R3), rather than App feeding it `PowerAction`s. One input path, with all transitions tested together.
3. **Where the state-entry log text lives** (R3): a pure lookup in core, so it's testable, or kept in App.
4. **The `app/protocol/` subfolder** (R6), or flat in `app/`.
5. **Whether `DeviceStatus` holds pointers** to the tracker, epoch and battery as today (no copies), or copies the values it shows (R5). Pointers keep it small. Copies make a snapshot truly a moment in time.

---

## Implementation notes

- **R6:** GnssFeed was built as `core/GnssFeed.h`, pure and fed epochs through `add()`, rather than `app/GnssFeed` polling the receiver. That keeps the Phase 2 liveness logic host-testable. R6 also added `isCharging()` in `core/BatteryMonitor.h` as the single definition used by the boot line, the status line and the RaceBox charging bit.
- **Order changed after R3** (owner, 2026-10-05): R6, then Phase 2's GNSS items, then R4 and R5.

## Revision notes

Changes from the first draft, made after re-reading it against the code:

1. **Per-state entry effects plus a `Starting` state, instead of action lists in each plan.**
   - *Simpler:* the `Action` enum, the fixed `actions[4]` array, `actionCount` and the `entered` flag are gone. The machine returns a small `Transition`, and App's entry effects are one table with one row per state.
   - *More robust:* every effect is tied to a state, so an effect can't be left out of, or misordered in, one transition's list. "No BLE without GNSS" holds by construction rather than by list contents.
   - *Same test coverage:* the transition rows plus "Running only via `gnssStarted(true)`". Open question 3 (one-step or two-step GNSS start) goes away.
2. **Compile-time checks that job ids match the job table.** This closes an existing latent hazard where a reordered or missing entry silently swaps or zero-fills jobs. It's two `static_assert`s on the table R1 already makes `constexpr`.
3. **Diagnostics moves before the snapshot change (R4 ↔ R5).** In the first order, the snapshot step rewrote report assembly inside App, and the next step moved it out. Now the move is a pure move, and the snapshot change happens once, in its final home.

4. **Every event is defined in every state** (returns Stay where it doesn't apply), and the apply loop's bound is stated. The first draft left `gnssStarted()` outside Starting as an unchecked precondition. Phase 2 adds three more events, so undefined combinations would multiply.
5. **The protocol module encodes but doesn't send.** In the first draft it would also send. Keeping `ble_.send()` in App's GNSS job means Phase 4's likely send-path change (sender job or task, deeper queue) doesn't touch protocol code, and encoding stays host-testable without a BLE fake.
6. **`Starting` added to the LED mapping**, so every state maps to a pattern and `-Wswitch` stays meaningful.

7. **C++11 constraint recorded** (see [Language constraint](#language-constraint)). The first drafts used `if constexpr` and loop-based `constexpr` helpers, which the nRF core's `-std=gnu++11` would reject. §2 and §6 now use the C++11 forms, and the job table check is a recursive `constexpr` function over entries that carry their own id.

Considered and kept as is:
- **Performance:** no step touches a hot path materially. `ledStateFor` and `jobsFor` are a few comparisons; the snapshot is built only for `D` and the 1 s status line.
- **Pointers or copies in `DeviceStatus`** (open question 5): either works at this rate.
- **R1 and R2 first:** both are mechanical and shrink what R3 has to move.
