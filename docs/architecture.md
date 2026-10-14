# Architecture

This page describes how the firmware is put together: where things live, how the work is scheduled, and how the device decides what it should be doing. It's for anyone who wants to read or change the code. You don't need any of it to build and use a device.

The design goal behind most of it is **consistency**. A telemetry source is only useful if its data arrives at a steady rate, so the firmware is built to do small, bounded amounts of work at fixed intervals, and to make it visible when it doesn't.

---

## Layers

Everything is under [`code/Gnimu/src/`](../code/Gnimu/src/), in four layers. Each layer only depends on the ones above it in this table.

| Layer | What's in it | Rules |
|---|---|---|
| [`core/`](../code/Gnimu/src/core/) | Logic and math: the scheduler, the state machine, the power policy, the packet encoder, filters, the self-calibration, the report formatters. | Plain C++ with no Arduino and no hardware, so it runs in the [host tests](building.md#host-tests). |
| [`platform/`](../code/Gnimu/src/platform/) | What differs per board: the clock, the GNSS UART, the I2C bus, power control, the battery reading, the status LED, the watchdog. | One header per service, with an implementation under `esp32/` and `nrf52840/`. Each board's pins are in its `BoardConfig.h`. |
| [`peripherals/`](../code/Gnimu/src/peripherals/) | Drivers for the devices: the GNSS receiver, the two IMU chips, and BLE on each board's stack. | Built on `platform/` and the vendor libraries. |
| [`app/`](../code/Gnimu/src/app/) | The application: the job table, the states, the console, the diagnostics. | Ties the rest together. No board-specific code. |

The sketch itself, [`Gnimu.ino`](../code/Gnimu/Gnimu.ino), is three lines: it creates the `App` and calls `begin()` once and `tick()` forever.

### One sketch, two boards

Nothing is selected by editing the code. Each file under `platform/esp32/` and `platform/nrf52840/` is wrapped in a check for its own board, so the compiler builds one set and reduces the other to nothing. The same mechanism picks the IMU driver and the BLE transport: `ImuDevice` and `BleTransport` are compile-time aliases for the right class.

What a board *has* is stated once, in its `BoardConfig.h`: whether there's a battery, and whether the GNSS and IMU power can be switched. The rest of the code asks those questions rather than asking which board it is.

Shared code is written in C++11, because that's what the Seeed nRF52 board package compiles with.

---

## The scheduler

The firmware's work is a table of **jobs**, each with a period and a time budget. [`core/Scheduler.h`](../code/Gnimu/src/core/Scheduler.h) runs them:

| Job | Runs every | What it does |
|---|---|---|
| `gnss` | 1 ms | Drains the receiver's UART. When a position update completes, encodes and sends the packet. |
| `imu` | 10 ms | Reads the IMU, applies the self-calibration and the smoothing. |
| `battery` | 10 ms | Samples the battery voltage and the charger. Battery boards only. |
| `power` | 50 ms | Reads the power switch and USB, and applies any change of state. Battery boards only. |
| `ble` | 10 ms | BLE housekeeping, and collecting anything the app wrote to the device. |
| `led` | 25 ms | Updates the status LED. |
| `console` | 10 ms | Reads console commands and prints the status line. |

The scheduler is cooperative and fixed-rate:

- **Table order is priority.** On each pass, every job that's due runs once, in table order. `gnss` is first because losing receiver data is the one thing that can't be recovered.
- **Deadlines sit on a fixed grid.** A job's next run is scheduled from when it was *due*, not from when it ran, so a late run doesn't push every later one back.
- **Late is counted, not hidden.** If a job misses whole periods, they're counted as skipped rather than run in a burst to catch up. A run that exceeds its budget is counted as an overrun.
- **Between deadlines, the MCU sleeps** until the next one is due.

No job blocks. Each does a small piece of work and returns, and anything that could wait on the outside world has a bound on how long it will: a console line is dropped if the host isn't reading, an I2C read times out in 2 ms.

The counts the scheduler keeps are the job table in the [diagnostics report](console.md#the-diagnostics-report). That table is the main tool for checking that a change hasn't disturbed the timing.

### Timing

Each board supplies a microsecond clock through `platform/Clock.h`. On the nRF52840 that's a dedicated hardware timer, because the board package's own microsecond clock only advances once per millisecond.

A hardware watchdog is started at boot and fed on every pass of the main loop. If the firmware ever stalls for five seconds, the watchdog resets it, and the next boot log says so.

---

## The data path

A packet is built and sent in the same pass that completes the position update it carries:

1. The `gnss` job hands the UART's bytes to the u-blox library.
2. When the library completes a navigation message, the driver turns it into a `GnssEpoch`: one position update with everything the packet needs.
3. The job takes the latest smoothed IMU sample and the battery state, and the protocol encodes all three into a packet.
4. If an app is connected, the packet is sent as a BLE notification.

So there is exactly one packet per position update, and the packet rate is the receiver's navigation rate. The IMU runs on its own 100 Hz schedule; a packet carries whatever the latest sample was.

The receiver's speed also feeds back into the IMU job. The self-calibration only measures while the car is stationary, and the GNSS speed is its primary evidence of that. The reasoning is in [`imu-trim.md`](imu-trim.md).

### The protocol

What goes in a packet, and the BLE services that carry it, belong to a protocol class under [`app/protocol/`](../code/Gnimu/src/app/protocol/), chosen at compile time by `kProtocol` in `Config.h`. The protocol also decides whether the IMU is used at all.

RaceBox is the only protocol today. The encoder, [`core/RaceBoxEncoder.h`](../code/Gnimu/src/core/RaceBoxEncoder.h), is tested byte for byte against independently computed packets.

Bytes an app writes to the device are queued by the BLE stack's callback and collected by the `ble` job, which hands them to the protocol and also scans them for the [device commands](device-commands.md). The RaceBox protocol itself discards them. Separately, a small Gnimu service next to the protocol's holds the [configuration text](device-commands.md#reading-the-configuration) a client can read; it is built once at boot from the same code as the `V` command.

---

## Run states

What the device is doing at any moment is one of five states, decided in [`core/RunStateMachine.h`](../code/Gnimu/src/core/RunStateMachine.h):

| State | What's happening | Jobs running |
|---|---|---|
| **Starting** | Boot. Powers up and starts the GNSS, the IMU, and BLE, in that order. | None yet. |
| **Running** | Normal operation. | All of them. |
| **Fault** | Start-up failed, or the receiver never produced data. Everything else is stopped. | `led`, `console`, and on battery boards `battery` and `power`. |
| **BATTERY_WAIT** | The power switch is off while USB is plugged in. | Same as Fault. |
| **CHARGE_ONLY** | Charging, with everything else off. | Same as Fault. |

The state machine only makes decisions; it has no hardware in it, which is what makes it testable on the host. It is given events (boot, start-up result, GNSS data flowing or not, the power inputs) and answers with a transition. `App` carries the transition out: entering a state sets which jobs run and performs that state's fixed effects, such as stopping the peripherals and cutting the GNSS power.

Two rules keep it simple:

- **Running is only ever entered from a fresh boot.** Stopping happens in place, but going back to Running always goes through a restart, so the device only ever starts up one way.
- **A fault doesn't retry.** It stays put, with the LED blinking red and the console answering, until someone power-cycles it.

A receiver that was working and then goes silent for two seconds is handled differently: the firmware restarts, which on the nRF52840 also power-cycles the receiver. If it's still silent after that, it's a fault, so a dead receiver costs one restart and no more.

### Power, on battery boards

The XIAO nRF52840 build has a battery, a slide switch, and USB. [`core/PowerPolicy.h`](../code/Gnimu/src/core/PowerPolicy.h) turns those into the power-related transitions above.

**On battery**, which is the normal case in the car, the switch physically disconnects the battery. Switch on, the device runs. Switch off, it's unpowered.

**On USB**, the MCU stays powered no matter the switch state, so the switch chooses between running and charging:

| From | Event | Result |
|---|---|---|
| Off | Plug in USB, switch on | Running |
| Off | Plug in USB, switch off | BATTERY_WAIT |
| Running | Switch off | BATTERY_WAIT |
| BATTERY_WAIT | Switch on | CHARGE_ONLY |
| CHARGE_ONLY | Switch off, then on | Running, through a restart |
| CHARGE_ONLY | Unplug USB | Running on battery, through a restart |

CHARGE_ONLY exists because a running device draws enough that the charger never reports the cell as full.

Both switch edges are debounced for half a second. The switch is sensed through a high-impedance divider, so brief glitches are expected; the ones rejected are counted as `switch noise` in the diagnostics report.

**Low-voltage cutoff.** Running on battery below the cutoff voltage for five seconds puts the MCU into its lowest-power off state to protect the cell. The battery job keeps running in every state, Fault included, so the cutoff always works. Plugging in USB or cycling the switch starts the device again.

At boot on the nRF52840, the very first thing the firmware does is hold the GNSS power, the IMU power, and the LED off. They are powered only once the state machine has decided the device is going to run.

### The status LED

The LED shows the state. [`core/StatusLed.h`](../code/Gnimu/src/core/StatusLed.h) decides the pattern and color, and each board renders what it can: the XIAO has an RGB LED, the ESP32 dev board a single blue one that shows the pattern only.

| State | Pattern | Color |
|---|---|---|
| Running, waiting for an app | Slow blink, 1 s on, 1 s off | Blue |
| Running, app connected | Solid | Blue |
| Running, battery low | Slow blink | Amber |
| Running, battery critical | Slow blink | Red |
| Fault | Fast blink, 5 Hz | Red |
| BATTERY_WAIT | Quick blink, 2.5 Hz | Amber |
| CHARGE_ONLY, charging | Slow blink | Green |
| CHARGE_ONLY, full | Solid | Green |

A battery warning takes priority over the BLE indication.

---

## Diagnostics

The console is part of the design rather than an afterthought. The boot log reports every step, and the diagnostics report and status line read the same single snapshot of the device's state, [`core/DeviceStatus.h`](../code/Gnimu/src/core/DeviceStatus.h), so the two can't disagree. What they show is described in [Console and diagnostics](console.md).

All of the report formatting is in `core/` and covered by the host tests, including a check that the longest possible report and status line fit their buffers.

---

## Testing

- **Host tests** cover everything in `core/`: [`tests/`](../tests/), run with `tests/run.sh`.
- **Build checks** compile both boards with all warnings on and fail on any warning in the project's own code: `scripts/build.sh`.
- **CI** runs both on every push.

What can't be tested on the host is checked on hardware: timing under load, GNSS reception while BLE is streaming, and the power states. The job table in the diagnostics report exists to make those checks quick.

See [Building and flashing](building.md) for how to run each of them.
