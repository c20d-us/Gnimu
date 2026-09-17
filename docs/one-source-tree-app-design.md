# One source tree and application coordinator design

## Status and purpose

This is a proposed target architecture for Gnimu. It replaces the three
near-duplicate firmware trees with one source tree, while preserving separate
compile-time firmware targets for the hardware that actually differs.

It is deliberately not a proposal for an RTOS, dynamic plug-ins, heap
allocation, or a universal firmware image. Each target remains a separate,
fully optimized firmware binary. The change is that those binaries are built
from the same source tree and a selected board profile.

The design has four goals:

1. Share GNSS, IMU, telemetry, protocol, state, and BLE policy code without
   byte-for-byte copy synchronization.
2. Select one output protocol at compile time while keeping protocol code
   independent of a particular MCU and BLE library.
3. Make the cooperative loop's ordering, data hand-offs, timing constraints,
   and lifecycle effects explicit.
4. Keep the existing low-latency, allocation-free embedded model intact.

This document describes the desired architecture. It does not require a
single disruptive rewrite; the migration section gives an incremental path.

## Terms

| Term | Meaning |
|---|---|
| **platform** | MCU/core family and its library backends: ESP32 or nRF52840. |
| **board profile** | A compile-time description of one physically wired device. It selects pins, peripherals, and defaults. |
| **target** | One build of one board profile and one selected protocol. A target produces one firmware binary. |
| **protocol** | The application-facing telemetry contract: identity, BLE profile, encoding, cadence policy, and optional inbound command parser. |
| **BLE profile** | The concrete GATT/service shape required by a protocol, such as Nordic UART or generic GATT characteristics. |
| **device view** | A read-only snapshot built by the application coordinator for presentation and diagnostics. |

## Non-goals

- Auto-detecting the target board at compile time or producing one binary for
  all boards.
- Runtime-loadable protocols. Protocol choice stays compile-time.
- Hiding real hardware differences behind false abstractions.
- Replacing the existing polling model with tasks, queues, or an RTOS.
- Removing timing protections such as GNSS-first service, incremental battery
  sampling, the BLE callback ring, or metered OLED transfers.

## Target and board selection

The build selects a board profile explicitly. The profile may be supplied by a
build flag or a small target header selected by the build system.

```cpp
// Example build definitions; exactly one of each category is required.
#define GNIMU_BOARD GNIMU_BOARD_NRF52840_OLED
#define GNIMU_PROTOCOL GNIMU_PROTOCOL_RACEBOX
```

The profile validates the selected Arduino core and declares compile-time
facts needed by the firmware:

```cpp
struct BoardTraits {
  static constexpr bool kHasBatteryGauge = true;
  static constexpr bool kHasDisplay = true;
  static constexpr bool kHasRgbLed = true;
  static constexpr int kGnssEnablePin = D9;
  static constexpr int kSwitchSensePin = A1;
  // Sensor, UART, I2C, power, and UI facts follow the same pattern.
};
```

Macros remain appropriate where a third-party Arduino library requires them;
the project should otherwise prefer named `constexpr` traits and small
configuration structs over a large global macro namespace.

### Current nRF board constraint

The current nRF52840 and nRF52840-OLED devices must remain distinct board
profiles. The non-OLED board uses `A4` for switch sensing; the OLED board uses
`A4` as I2C SDA and moves switch sensing to `A1`. Starting or probing the
external I2C display bus on the non-OLED wiring would interfere with that
switch-sense function.

The OLED code may therefore be shared and compiled only for the OLED profile
for now. If a later hardware revision standardizes switch sensing on `A1`, a
single nRF profile may safely compile display support and select the OLED at
boot by probing its I2C address. That is a future hardware simplification, not
a requirement of this source-tree design.

### Required validation

Compilation must fail for invalid combinations, including:

- ESP32 profile paired with an nRF-only power, battery, or IMU backend.
- OLED support paired with a board whose external I2C pins conflict with a
  required function.
- A protocol whose BLE profile is not implemented on the selected platform.
- A protocol whose worst-case frame cannot be carried or fragmented by its
  selected BLE profile.

## Proposed source layout

The exact filenames may change, but ownership should follow this shape:

```text
src/
  Gnimu.ino                         thin Arduino entry point
  app/
    app.cpp                         startup, tick ordering, lifecycle effects
    app.h
    device_view.h                   presentation/diagnostic snapshot
  board/
    target.h                        resolves selected profile
    esp32.h
    nrf52840.h
    nrf52840_oled.h
  core/
    state/                          transition decisions; no peripheral calls
    gnss/                           receiver behavior and epoch event
    imu/                            acquisition, frame transform, trim
    telemetry/                      sample assembly and statistics
    protocol/                       canonical contract and protocol adapters
    ble/                            stack-independent session/send policy
    logging/
  platform/
    esp32/                          ESP32 BLE, UART, and MPU6050 backends
    nrf52840/                       Bluefruit, UART, LSM6DS3, power, battery
  ui/
    led/
    oled/
```

`platform` sources are still built conditionally. Arduino commonly compiles
every `.cpp` in the sketch directory, so each backend must be gated by its
selected platform/profile, or the build system must filter its source files.
No target should have to install or link libraries used only by another target.

The result is one repository source tree, not one compilation unit and not one
binary. ESP32, nRF52840, and nRF52840-OLED remain separate supported targets.

## Runtime coordinator: `App::tick()`

The firmware remains a cooperative polling application. The change is that an
application coordinator owns the order of work and the hand-off between
modules:

```cpp
void setup() { app.begin(); }
void loop()  { app.tick(); }
```

`App` may be a normal C++ object, a file-local singleton, or a namespace-level
module. The design does not depend on object allocation, inheritance, virtual
functions, or an RTOS.

### Tick responsibilities

`App::tick()` is the authoritative place for:

1. advancing low-cost input samplers and collecting their latest snapshots;
2. asking the state machine whether a transition is needed;
3. applying the peripheral effects of an accepted transition;
4. giving the GNSS receive path priority while the device is running;
5. passing each new GNSS epoch to IMU/telemetry/protocol work explicitly;
6. updating BLE housekeeping after time-sensitive acquisition work; and
7. publishing a `DeviceView` to LED, OLED, and serial presentation.

A representative ordering is:

```cpp
void App::tick() {
  const uint32_t now = clock_.millis();

  battery_.poll(now);
  applyStateAction(state_.update(now, battery_.status(), power_.usbPresent(),
                                 power_.switchOn(), ble_.isSubscribed()));

  if (state_.isRunning()) {
    gnss_.poll(now);                     // preserve UART service priority
    imu_.poll(now, gnss_.motionStatus());

    if (const auto epoch = gnss_.takeEpoch()) {
      const TelemetrySample sample =
          sampleBuilder_.build(*epoch, imu_.latchForEpoch(), battery_.status());
      telemetry_.handleSample(sample, protocol_, ble_);
    }

    ble_.update(now);
  }

  const DeviceView view = makeDeviceView(now);
  led_.update(now, view);
  display_.update(now, view);
}
```

This is illustrative rather than a mandated API. In particular, the existing
measured GNSS deadline determines the final order and maximum work allowed in
each step. The coordinator must preserve that behavior as it is refactored.

### Explicit events and snapshots

Modules should not reach into one another for hidden state when an explicit
input or output will do.

| Current coupling | Desired hand-off |
|---|---|
| GNSS exposes a global latest PVT plus a consumable flag/pointer. | `gnss.takeEpoch()` yields one explicit epoch event; a separate snapshot API serves observers. |
| IMU trim queries GNSS directly for speed and fix state. | The coordinator passes a small `MotionStatus`/fix snapshot to IMU trim. |
| Telemetry consumes GNSS, latches IMU, reads battery, encodes, sends, and prints stats. | The coordinator creates a sample; telemetry tracks cadence/statistics; the selected protocol encodes; BLE emits. |
| LED and OLED each query several subsystems. | They consume one `DeviceView`. |
| State invokes GNSS/BLE/power/display operations itself. | State returns an action; the coordinator applies the action. |

The data is copied by value where practical. An epoch/sample is small enough
for predictable stack or static storage, and explicit ownership is preferable
to a cross-module raw pointer. No dynamic allocation is needed.

### State decisions versus state effects

The state module should decide policy, not directly operate peripherals.

```cpp
enum class StateAction {
  None,
  EnterBatteryWait,
  EnterChargeOnly,
  EnterDeepSleep,
  Restart,
};
```

`state.update(...)` returns an action based on USB, switch, battery, and BLE
subscription inputs. `App::applyStateAction(...)` then owns the ordered side
effects: stop advertising, close the GNSS UART, drive rails/pins safe, sleep an
OLED when one exists, flush logging, reset, or enter System OFF.

This prevents a display feature from changing the policy state machine merely
to add `displaySleep()`. It also makes transition policy easy to test without
hardware.

## Protocol architecture

### Compile-time protocol adapters

Protocols should be compile-time adapters, not runtime-loadable plug-ins. The
selected protocol is the only encoder and command parser linked into a target.
It receives a canonical `TelemetrySample` and may ignore fields it does not
need.

```text
GNSS epoch + IMU latch + battery snapshot
                 ↓
          TelemetrySample
                 ↓
   selected protocol encoder / command parser
                 ↓
        selected BLE profile and port
```

This supports a GNSS-only RaceChrono protocol without duplicating hardware
acquisition: it simply consumes the GNSS fields. RaceBox can consume its fuller
set. A future custom protocol can use any explicitly available fields.

### Protocol declaration

Each protocol should declare only the facts the coordinator and BLE layer need:

- identity and firmware/model metadata;
- required BLE profile;
- maximum outbound frame size and whether the profile fragments frames;
- outbound encoder(s) and their cadence policy;
- optional inbound parser and its stream/message framing expectations; and
- optional input requirements, if acquiring an otherwise unused sensor has a
  material power or timing cost.

The current canonical `TelemetrySample` is a suitable internal superset. It
should remain MCU- and BLE-library-free. Protocols perform wire-format choices
such as field clamping, validity mapping, scaling, packet layout, and checksum.

### Separate protocol semantics from BLE presentation

The selected protocol's GATT/service requirements must match what each
platform truly implements. A descriptor that describes arbitrary
characteristics while an nRF backend unconditionally serves Bluefruit
`BLEUart` creates two sources of truth.

Choose one of these supported paths for every protocol/platform combination:

1. **Nordic UART profile:** the protocol uses the exact BLEUart service and
   stream semantics; the nRF Bluefruit backend is appropriate.
2. **Generic GATT profile:** both ESP32 and nRF implement the declared service,
   characteristics, subscriptions, MTU behavior, and send semantics.
3. **Unsupported:** compilation fails until the required BLE profile is
   implemented for that platform.

RaceChrono should be introduced only after its required BLE profile is modeled
explicitly on each intended platform. This is the principal design checkpoint
for protocol two.

## Presentation and diagnostics

`DeviceView` is a read-only, platform-neutral value assembled once per tick.
It can include only presentation-safe values, for example:

- system state and transition-relevant status;
- BLE connection/subscription state;
- battery voltage, percentage, and charge state;
- current GNSS fix/quality summary and staleness;
- IMU/trim status;
- protocol-independent GNSS/BLE rate counters; and
- display availability, where appropriate.

The OLED still owns display-specific timing, framebuffer, partial transfers,
and burn-in shifting. The LED still owns its display rules. Neither owns or
queries acquisition state. Serial reporting may use the same view or a
diagnostic-specific snapshot.

## Expected simplifications

The design intentionally preserves physical complexity while removing
accidental complexity.

| Area | Simplification |
|---|---|
| Source sharing | One implementation of shared GNSS, IMU, telemetry, protocol, and BLE policy code; no copy-and-compare maintenance rule. |
| Board differences | Small explicit profiles instead of full forked trees. |
| Lifecycle | One transition-effect owner instead of state, power, GNSS, BLE, and OLED coordinating shutdown ad hoc. |
| Data flow | Explicit epoch, motion, sample, and view values replace cross-module global reads. |
| UI | LED/OLED dependencies collapse to `DeviceView`. |
| Telemetry | Sample assembly, stats, encoding, and transport become separable responsibilities. |
| Protocols | RaceChrono, RaceBox, and a future protocol share acquisition without pretending every BLE stack exposes the same service. |
| Testability | State decisions, protocol encoders, view construction, and tick ordering can be exercised with fakes without flashing hardware. |
| Configuration | Shared defaults plus profiles reduce repeated 500+ line configuration headers. |

The following mechanisms should remain unless measurements show otherwise:

- polling GNSS before discretionary work while its UART deadline is active;
- incremental/limited-work battery sampling;
- the BLE callback-to-loop ring and its atomic ordering;
- explicit MTU/backpressure handling; and
- OLED partial-update scheduling.

## Migration plan

The refactor should preserve behavior at each step and keep the existing test
harnesses runnable.

1. **Establish targets without moving behavior.** Create one shared tree and
   three board profiles. Initially retain the existing module APIs and the
   exact current loop order. Build each profile and compare existing harness
   output and device behavior.
2. **Introduce the coordinator.** Replace each sketch loop with `app.begin()`
   and `app.tick()`, initially forwarding to the old calls in the same order.
   This creates one scheduling owner with minimal risk.
3. **Move lifecycle effects.** Convert the nRF state machine to emit
   `StateAction` values. Put current stop/power/reset/deep-sleep sequences in
   the app's action executor. Verify every boot, USB, switch, cutoff, idle, and
   System OFF path.
4. **Make GNSS epochs explicit.** Replace consumable global PVT access with an
   epoch event and a separate observer snapshot. Keep the single-consumer rule
   while transitioning telemetry.
5. **Decouple IMU and presentation.** Pass GNSS motion status into trim; add
   `DeviceView`; convert LED and OLED to consume it.
6. **Split telemetry responsibility.** Preserve the canonical sample, then
   separate assembly, statistics, protocol encoding, and BLE emission.
7. **Define BLE profiles.** Document and implement the actual nRF and ESP32
   support for Nordic UART and generic GATT before adding RaceChrono to a
   target.
8. **Add protocol two.** Select it through the compile-time protocol selector,
   add golden-vector tests, and validate both supported board families.
9. **Retire duplicated trees and copy-check tooling.** Remove the old
   duplication guard only after all targets build from the new tree and test
   coverage has moved with them.

Every step should be reversible in version control and should preserve the
timing measurements and test fixtures that currently guard shipping behavior.

## Acceptance criteria

The migration is complete when all of the following are true:

- ESP32, nRF52840, and nRF52840-OLED build from one source tree using explicit
  board profiles.
- Existing board targets retain their current pin assignments and behavior.
- The two current nRF profiles remain distinct until their pin conflict is
  removed in hardware.
- `setup()` and `loop()` are thin wrappers around the application coordinator.
- No UI module imports GNSS, IMU, battery, BLE, telemetry, and state modules
  directly to construct its own view of the system.
- State policy can be tested independently of peripheral side effects.
- A protocol encoder can be built and tested on a host without Arduino or BLE
  headers.
- Unsupported board/protocol/BLE-profile combinations fail at compile time.
- RaceBox behavior is unchanged for the existing supported targets, as shown by
  existing vectors/harnesses and device checks.
- A GNSS-only RaceChrono protocol can be added without copying a board tree or
  changing GNSS/IMU/power implementation code.

## Decisions to make before implementation

1. Which build entry point will own target definitions: Arduino IDE sketches,
   Arduino CLI scripts, PlatformIO environments, or another build system?
2. Which BLE profile(s) does RaceChrono require on ESP32 and nRF52840?
3. Should IMU acquisition be skipped for protocols that do not use it, or kept
   running for diagnostics/UI consistency? This is a power/timing policy
   decision, not a protocol-encoder decision.
4. Which current configuration values belong to shared product defaults versus
   physical board traits versus user tunables?
5. Whether a future nRF hardware revision will standardize switch sensing on
   `A1`, allowing optional OLED support within one nRF profile.
