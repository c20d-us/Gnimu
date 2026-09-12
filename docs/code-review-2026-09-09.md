# Gnimu Firmware Review

**Code review · branch `gnimu_multiproto` · 9 Sep 2026**

Three sketch trees, the new protocol plug-in layer, and the host encoder harness —
read against the stated priorities of consistent loop timing, measurement accuracy,
and a seam that can carry a second protocol.

| | |
|---|---|
| Variants compiled | 3 / 3 clean |
| Encoder vectors | 23,028 pass |
| Shared-file check | 24 identical |
| Sketch warnings | 5 |
| Findings | 27 |
| Architecture notes | 11 |

---

## Where this stands

Read fresh, with no history of the project beyond what the repo says.

The codebase is in unusually good shape for its size. Memory safety is genuinely
clean: no `strcpy`, no `sprintf`, one `memcpy` with a correct `sizeof`, every
`snprintf` bounded, the BLE receive path index-guarded, and `writeLittleEndian`
type-whitelisted so a wrong-width store cannot compile. The protocol split is the
right shape — `g_protocol.h`'s dependency-free constraint is what makes the host
harness possible, and 23,028 golden vectors reproducing byte-for-byte is real
proof, not a gesture at one.

What follows is what a second pair of eyes finds. Most of it is small. Four items
are worth acting on before phase F, and one of those — **IMU-1** — works directly
against the accuracy goal in a way the code's own comment says it doesn't.

### Act on these first

1. **IMU-1** — the IMU sample riding each packet is 0–50 ms old and the age
   wanders. Sharing a period with the GNSS epoch does not lock phase to it.
2. **BLE-1 / BLE-2** — both transports can put a truncated packet on the wire and
   neither reports it; the "BLE: 20.00Hz" stat counts encodes, not sends.
3. **API-1 / API-2** — `onWrite`'s thread context and buffer lifetime are
   unspecified, and the two stacks already differ. RaceChrono's CAN-filter
   characteristic is exactly the case that breaks on it.
4. **IMU-2** — the transient-peak blend is switched off in all three shipped
   configs while the comments beside it describe values that aren't there.

---

## Sensor accuracy

The two findings that most affect what the g-force channel actually reports.

### IMU-1 — High — IMU decimation runs on its own clock, not the GNSS epoch

`config.h:568` · `g_imu.cpp imuPoll()` · `g_telemetry.cpp telemetrySendIfReady()`

`IMU_TRANSMIT_INTERVAL_MS` is defined as `(1000 / GNSS_NAV_RATE_HZ)` with the
comment "so the two can't drift out of sync." Sharing a *period* does not lock
*phase*. One timer is anchored to `millis()`, the other to the receiver's own
oscillator, and they are never told about each other — so the accelerometer sample
attached to a packet is anywhere from 0 to 50 ms old, and that offset walks slowly
through the whole range as the two clocks beat against each other.

For a device whose output is read as "this g-force happened at this position,"
that is a real, unbounded-in-practice time-alignment error, and it is invisible in
any single log.

> **Fix.** Latch the decimation from `telemetrySendIfReady()` on epoch arrival. It
> already consumes every epoch regardless of BLE state, which is precisely the
> "always drain the transient window" requirement that put the drain on a timer in
> the first place. Keep the timer as the fallback for when epochs are not flowing.

### IMU-2 — High — The transient-peak blend is disabled in every shipped config, and the comments say otherwise

`config.h:104–106` in all three trees

`IMU_ACCEL_TRANSIENT_THRESHOLD_G 99.0f` sits beside a comment reading
`1.5g = ~14.7m/s^2`; `IMU_GYRO_TRANSIENT_THRESHOLD_DPS 9999.0f` beside
`28.6deg/s`. The nRF tree still carries the real value commented out one line
below. At those thresholds `ImuAxis::read()` never blends, so the output is a bare
α = 0.09 EMA at 100 Hz — roughly a 1.5 Hz low-pass, which flattens exactly the
short events an autocross trace exists to show.

Whatever the intended holding value, a reader of `config.h` currently comes away
believing the transient path is live and tuned to 1.5 g.

> **Fix.** Make the comments describe the values that are actually there, and say
> in one line that the blend is parked. The `static_assert` only checks `> 0.0f`,
> so nothing else catches this.

### IMU-3 — Medium — The ESP32 tree has no failed-read guard; the nRF tree does

`Gnimu-ESP32/g_imu.cpp readImuRaw()` vs `Gnimu-nRF52840/g_imu.cpp readImuRaw()`

The nRF version checks `readRegisterRegion()` and returns `lastGood` on failure,
with a comment explaining exactly why: a bad transfer fed into the filters gets
latched as a "peak" and reported as a genuine event, and — worse — a repeated
identical sample reads as zero variance and falsely satisfies `g_imu_trim`'s
stillness gate. The ESP32 version calls `myIMU.getEvent(&a, &g, &temp)` and
discards the result entirely.

Both trees feed the same byte-identical `g_imu_trim.cpp`, so the hazard is
identical and the mitigation landed on one side only. This is the kind of
divergence `check_common.sh` cannot see, because `g_imu.cpp` is legitimately
excluded from the shared set.

> **Fix.** Mirror the `lastGood` guard on the MPU-6050 path, keyed off
> `getEvent()`'s return.

### IMU-4 — Low — `toProtocolInt16()` passes NaN through to undefined behavior

`g_imu.cpp` (both variants)

A NaN fails both the `> 32767.0f` and `< -32768.0f` tests and falls into
`(int16_t)value`, which is UB. Nothing in the current pipeline produces one, but
the function is the last line of defence before the wire and is written as though
it were total.

> **Fix.** Add `if (!(value == value)) return 0;` as the first test, or use
> `isfinite()`.

---

## BLE transport

Both stacks can put a partial packet on the wire, and neither path reports it.

### BLE-1 — High — nRF discards `bleuart.write()`'s result — a congested HVN queue truncates the frame mid-chunk

`Gnimu-nRF52840/g_ble.cpp bleEmitFrame()`

With `_tx_buffered` off (the default), `BLEUart::write()` returns
`_txd.notify(...) ? len : 0`. Inside that notify, Bluefruit chunks to `MTU − 3` in
a loop and bails on `if (!conn->getHvnPacket()) return false;` — *inside* the loop.
When the SoftDevice's notify queue is exhausted, whatever chunks already went out
stay out and the rest is dropped, injecting a partial UBX packet into a stream the
app has to resynchronise from.

Nothing observes this. `bleEmitFrame()` returns `void`, and `bleSentPacketCount++`
in `telemetrySendIfReady()` counts the *encode*, not the send — so the console
reports a healthy `BLE: 20.00Hz` through a run that is dropping frames.

> **Fix.** Return the byte count from `bleEmitFrame()`, count short writes, and put
> a drop counter on the stats line. That single number turns an invisible failure
> into an obvious one.

### BLE-2 — High — ESP32 never verifies the MTU it asked for

`Gnimu-ESP32/g_ble.cpp` — `ServerCallbacks::onConnect()`, `bleEmitFrame()`

`updatePeerMTU()` is a request, and a central is free to decline it. The code
comments assert that "`BLE_MTU_BYTES` is negotiated up front to make that safe,"
but nothing checks the negotiated value afterwards. In esp32 core 3.3.11,
`BLECharacteristic::notify()` emits `log_w("- Truncating to %u bytes...")` —
invisible at the default core debug level — and hands the full length to the
controller anyway, which cuts at `MTU − 3`.

A client that stays at the 23-byte default therefore receives a 20-byte fragment of
an 88-byte packet, every epoch, forever, with no diagnostic. The nRF path chunks
correctly in the same situation, so the two variants fail in genuinely different
ways and neither behaviour has been exercised.

> **Fix.** Implement `onMtuChanged`, store the negotiated value, and refuse to send
> below `RACEBOX_PACKET_LEN + 3` with a loud log line — a device that says nothing
> is better than one that streams garbage.

### BLE-3 — Low — nRF `bleEmitFrame()` ignores `channel` entirely

`Gnimu-nRF52840/g_ble.cpp:bleEmitFrame`

Correct today — the Nordic UART transport has one stream. But
`TELEMETRY_CHANNEL_NORDIC_RX` exists as index 1, and a protocol that emitted on it
by mistake would have its frame silently routed out the Tx characteristic instead
of failing.

> **Fix.** One guard: `if (channel != TELEMETRY_CHANNEL_PRIMARY) return 0;`

### BLE-4 — Low — `lastLoggedMtu` is written from two threads

`Gnimu-nRF52840/g_ble.cpp` — `connectCallback()`, `disconnectCallback()`,
`bleUpdate()`

The connect and disconnect callbacks run on the Bluefruit event thread;
`bleUpdate()` runs on the loop. Both write the same non-atomic `uint16_t`. The
impact is confined to a log line, but it sits next to **API-1** as evidence that
the thread boundary here is not currently tracked anywhere.

> **Fix.** Worth noting in a comment even if left as-is, so the next person to add
> state here knows which side of the boundary they are on.

---

## Loop timing

One hard real-time constraint that is documented in the wrong file, and one
recurring stall that lands on the epoch by construction.

```
         0           10          20          30          40          50 ms
         |-----------|-----------|-----------|-----------|-----------|

UART     ##########                                                   NAV-PVT · 100 B · 8.7 ms
                ^ 5.5 ms - the 64-byte RX ring is full

IMU      |           :           :           |           |           100 Hz sample ticks
                                                                     ( : = dropped, not deferred )

Serial             ########################                          ESP32 stats line · 239 B
                                                                     up to 20 ms · once per second

                                    ONE EPOCH AT GNSS_NAV_RATE_HZ 20
```

Drawn to scale. The stats line does not delay the notify — that has already gone
out — but it blocks the loop task across two `imuPoll()` deadlines, and the
deadline-anchored catch-up resyncs rather than firing a burst, so those samples are
dropped from the EMA and from the trim's variance estimate rather than merely
delayed.

### LAT-1 — Medium — The once-per-second stats line lands on an epoch iteration by construction

`g_telemetry.cpp telemetrySendIfReady()` · `config.h LOG_STATS_INTERVAL_MS 1000`

1000 ms ÷ the 50 ms epoch is exactly 20, and the window check runs in the same
function, immediately after the epoch is consumed — so the print reliably shares an
iteration with a packet rather than occasionally colliding with one.

On ESP32 that matters: `Serial` is UART0 and `HardwareSerial` defaults to
`_txBufferSize == 0`, so `write()` blocks the loop task on the 128-byte hardware
FIFO. The line measures 239 bytes in normal operation — roughly 20 ms at 115200. On
the nRF boards `Serial` is USB CDC and no-ops when nothing is attached, so this
only bites with a console connected.

> **Fix.** `Serial.setTxBufferSize(512)` before `Serial.begin()` on ESP32 makes the
> write queue rather than block. Separately, defer the report by one iteration so
> it can never share a pass with an epoch.

### LAT-2 — Medium — The nRF UART's hard deadline is documented only inside `g_display.cpp`

`Seeeduino nRF52 core — cores/nRF5/RingBuffer.h: SERIAL_BUFFER_SIZE 64`

`Serial1`'s receive ring is 64 bytes — smaller than the 100-byte NAV-PVT it has to
hold. At 115200 it fills in about 5.5 ms, so `gnssPoll()` must be reached at least
that often *during* the 8.7 ms message or bytes are lost. That is a genuine hard
real-time constraint on the whole loop, and the only place it is written down is
the header comment of the OLED variant's display module — a file the other two
trees do not have.

The OLED build is carefully engineered around it (render separated from push,
slices phase-locked to the epoch). The other two satisfy it by having little else
to do, which is a property of today's loop rather than a guarantee.

> **Fix.** Move the constraint into `g_gnss.h`, where anyone adding work to the
> loop will see it. `SERIAL_BUFFER_SIZE` is `#ifndef`-guarded, so
> `-DSERIAL_BUFFER_SIZE=256` via `boards.local.txt` removes the cliff entirely if
> you want the margin.

### LAT-3 — Medium — The stats line already truncates on the nRF boards

`Seeeduino nRF52 core — cores/nRF5/Print.cpp:198 — vsnprintf(buf, 256, ...)`

`Print::printf` formats into a 256-byte stack buffer. The stats line measures
**239 bytes** in a normal fix and **257 bytes** when `hAcc` and `tAcc` sit at their
no-fix sentinels — which is exactly the state you watch the console during. It
fails safely (`vsnprintf` is bounded, so it clips rather than overflows), but it
clips silently, and any future field pushes the normal case over too. ESP32's
`Print::vprintf` heap-allocates instead, so this is nRF-only.

> **Fix.** Split the report across two `LOG_PRINTF` calls, or drop the redundant
> `tAcc` field. Worth a comment recording the 256-byte ceiling either way.

### LAT-4 — Low — Two per-iteration costs that only need to run on change

`g_led.cpp ledUpdate()` · `g_ble.cpp bleUpdate()`

`ledUpdate()` issues three `digitalWrite()` calls every pass regardless of whether
the colour changed, and `bleUpdate()` returns a 16-byte `BatteryStatus` by value
each pass to compare one byte of it. Both are cheap; both are also free to skip.

> **Fix.** Cache the last written LED triple; have `bleUpdate()` read a
> `batteryPercent()` accessor rather than the whole struct.

### LAT-5 — Low — ESP32 GNSS UART left at the 256-byte default receive buffer

`Gnimu-ESP32/g_gnss.cpp connectAndConfigureBaud()`

That is about 100 ms of NAV-PVT headroom at 20 Hz — comfortable, and comfortably
wider than the nRF's. Worth raising anyway since it costs one call and the ESP32
has the RAM.

> **Fix.** `gnssSerial.setRxBufferSize(512)` before `begin()`.

---

## The plug-in contract

Phases A–E are done and one protocol is live, so the seams have never carried a
second implementation. These are the ones that will bite in F–H, and all are
cheapest to fix now while nothing depends on the current behaviour.

### API-1 — High — `onWrite`'s execution context is unspecified, and the two stacks already differ

`g_protocol.h ProtocolDescriptor::onWrite` · `g_ble.cpp rxCallback()` /
`ChannelWriteCallbacks::onWrite()`

On nRF it is called from the Bluefruit / SoftDevice callback thread; on ESP32 from
the Bluedroid BTC task. Neither is the main loop. `g_protocol.h` is meticulous
about the *emit* side — "Called SYNCHRONOUSLY, so an encoder may hand over a stack
buffer without copying" — and says nothing at all about the receive side.

Today `raceboxOnWrite()` is empty, so it doesn't matter. RaceChrono's CAN-filter
characteristic is a command channel, which means the first real implementation will
mutate configuration state from an interrupt-priority thread while the loop reads
it, on a project where a data race would present as intermittent packet corruption
rather than a crash.

> **Fix.** Best: have `g_ble` buffer the write and dispatch it from `bleUpdate()`,
> so `onWrite` is defined to run on the loop. Cheaper: state the off-loop contract
> in `g_protocol.h` and say handlers must do nothing but enqueue.

### API-2 — High — `onWrite`'s buffer lifetime is also unspecified, and also differs

`Gnimu-nRF52840/g_ble.cpp rxCallback()` vs
`Gnimu-ESP32/g_ble.cpp ChannelWriteCallbacks::onWrite()`

nRF drains into a local `uint8_t buf[64]` and passes that — valid for the duration
of the call, and nothing beyond. ESP32 passes `pCharacteristic->getData()`, a
pointer into the stack's own live characteristic buffer. A handler written against
one platform's semantics and tested there will behave differently on the other, and
the failure mode is a stale read rather than a fault.

> **Fix.** Specify it the same way the emit side already is — "valid only for the
> duration of the call, copy if you need it later" — and make ESP32 copy so both
> paths actually honour it.

### API-3 — Medium — `TelemetryEmit` cannot report failure

`g_protocol.h` — `typedef void (*TelemetryEmit)(uint8_t, const uint8_t *, size_t)`

The header's own worked example is the multi-frame case: RaceChrono sends 20 bytes
on GPS and 3 more on GPS Time, one encode call, two frames. With a `void` sink an
encoder cannot learn that frame one landed and frame two did not, so it cannot
abort, and the client sees a position with no timestamp beside it.

> **Fix.** Make it `bool (*TelemetryEmit)(...)`. This is free today — one encoder,
> one call site, and the same change carries the drop counting that **BLE-1** needs.

### API-4 — Medium — Nothing validates a descriptor against the transport it declares

`g_protocol.h TransportKind` · `g_ble.cpp bleBegin()`

`TRANSPORT_NORDIC_UART` has a fixed shape — exactly two channels, index 0 notify,
index 1 write — stated only in prose. A descriptor that got it wrong would stand up
a plausible-looking GATT that quietly serves nothing, which is the same class of
silent failure the ESP32 handle-count comment already warns about.

> **Fix.** A `static_assert` in the protocol's own translation unit costs nothing
> and fires at build time: `channelCount == 2` and the expected props on each index.

### API-5 — Medium — nRF silently truncates a client write past 64 bytes

`Gnimu-nRF52840/g_ble.cpp rxCallback()`

The loop drains and logs every byte but only stores `n < sizeof(buf)` of them, so
`onWrite` receives an `n` that under-reports what actually arrived, with no signal
that anything was dropped. Correct and harmless for a no-op handler; wrong for a
command parser, which would silently act on a partial command.

> **Fix.** Pass an overflow flag, or drop the whole write and log it, rather than
> handing over a short buffer that looks complete.

### API-6 — Low — ESP32 is at 89% flash before a second protocol exists

`arduino-cli compile — 1,178,939 of 1,310,720 bytes`

Almost all of that is Bluedroid. Phase H adds a GATT-channels builder and a second
encoder to a tree with 131 KB of headroom. This is not a bug, but it is the
constraint most likely to stop phase H rather than any design question.

> **Fix.** Worth measuring NimBLE-Arduino before phase H — it typically halves this
> — but the port touches `g_ble.cpp` only, which is precisely what the descriptor
> split bought you.

---

## Robustness

Failure paths that halt where they should degrade, and code that no longer runs.

### ROB-1 — Medium — `gnssBegin()`'s `while(1)` halt is reachable at runtime, not just at boot

`g_gnss.cpp gnssBegin()` · `g_state.cpp exitLightSleep()`

After `STATE_LIGHT_SLEEP_GNSS_CUTOFF_MIN` the rail is cut, so waking calls
`powerGnssRailOn(); gnssBegin();` — and `gnssBegin()` ends in an infinite
`while (1) delay(100)` if the receiver does not answer the baud sweep. On a battery
build that bricks the device with BLE dead, the low-voltage cutoff dead and the LED
frozen, until someone notices the cell is flat.

Separately, that same call blocks the loop for anywhere from 0.5 to several seconds
inside `stateUpdate()` — and the thing that triggered the wake is usually a client
connecting, so the stall lands precisely when the app expects data.

> **Fix.** Return `bool` from `gnssBegin()` and give the state machine a fault state
> that keeps BLE and the battery cutoff alive. The boot-time halt is defensible; the
> runtime one is not.

### ROB-2 — Medium — `powerSwitchOn()` documents a mitigation it does not implement

`g_power.h:47` vs `g_power.cpp switchReadOnce()`

The header promises "dummy read + averaged burst, refreshed at most every ~50 ms…
to guard against SAADC channel-switch 'ghost' readings from the shared VBAT pin."
The implementation is one bare `analogRead()`. Only the throttling is real.

This matters because the switch read is what routes the device to `BATTERY_WAIT`,
and the battery sampler is hitting a different SAADC channel microseconds earlier —
which is the exact condition the comment describes. `STATE_SWITCH_OFF_DEBOUNCE_MS`
covers a single bad read, so this is more likely a stale comment than a live fault
— but one of the two needs to change.

> **Fix.** Either add the discard-first-read-and-average that the header describes,
> or correct the header to say a single read plus a 500 ms state-machine debounce is
> the whole mitigation.

### ROB-3 — Low — `displayWake()` is never called

`Gnimu-nRF52840-OLED/g_display.cpp:displayWake`

Both `displaySleep()` call sites (`enterDeepSleepFrom()` and the boot classifier)
lead to System OFF, which never returns — so `asleep` can never go back to false.
The function is fully implemented, including a `lastRenderMs = 0` forced repaint,
and nothing reaches it.

> **Fix.** Delete it, or note in the header that it exists for a wake path that does
> not exist yet. As written it reads like a live feature.

### ROB-4 — Low — `enterChargeOnly()` is dead under the shipped config

`g_state.cpp:94` (nRF) / `:98` (OLED)

The only warning either nRF build emits:
`'void enterChargeOnly()' defined but not used`, because `STATE_CHARGE_ONLY_ON_USB`
is 0. The single warning in an otherwise clean build is worth spending, so that the
next one that appears gets noticed.

> **Fix.** Wrap the definition in the same `#if STATE_CHARGE_ONLY_ON_USB` that
> guards its call sites.

### ROB-5 — Low — Two small mismatches between a comment and its code

`g_gnss.h:44` · `g_state.cpp stateBegin()`

`gnssPoll()`'s header says it "sets the appropriate navigation frequency based on
the current state" — it does not; it calls `checkUblox()` and `checkCallbacks()`.
And `stateBegin()`'s BATTERY_WAIT branch logs "switch off, USB in" without ever
reading `usb`.

> **Fix.** Both are one-line edits. Worth doing because the comments in this
> codebase are load-bearing everywhere else, which is what makes an inaccurate one
> costly.

### ROB-6 — Low — Two tooling gaps worth closing while the seam is fresh

`src/tools/check_common.sh` · `test/harness.cpp`

`check_common.sh` is a manual allowlist, and the README names the hole itself: a new
shared file is uncovered until someone remembers to add it. Nothing is missing today
— I checked all three trees against both lists — but the check can be inverted so it
cannot go stale: enumerate every `.h`/`.cpp` present in two or more trees, subtract
`COMMON_FILES`, `NRF_COMMON_FILES` and an explicit `EXCLUDED` list, and fail on the
remainder. The three existing exclusions are already documented with reasons, so the
list is written.

Second: the harness covers `raceboxEncode()` only, and its own header explains why —
`buildSample()` needs the SparkFun struct. That struct is a plain POD; a small host
stub would close the last untested gap between a captured vector and the wire.

> **Fix.** Also worth noting that `g_power.h` is excluded from the shared set over
> two comment lines (A4 vs A1). Rewording them to name `POWER_SWITCH_SENSE_PIN`
> would let the file join the checked set and remove an exclusion.

---

## Security posture

Assessed across the BLE attack surface, the GNSS serial input, and memory safety
generally. The short version is that the code is clean and the exposure is inherent
to the product.

### SEC-1 — Medium — The device is an unauthenticated real-time location beacon, by necessity

`g_ble.cpp bleBegin()` — no `setPermission`, no bonding, no whitelist

Any central in range can connect and stream position at 20 Hz. There is no pairing,
no encryption and no access control on any characteristic. That is forced — the
RaceBox app expects exactly this GATT — so it is not a defect. But it is a real
property of a device the README describes leaving mounted in a car, and it is not
mentioned anywhere in the documentation.

Two consequences worth naming alongside it: `Bluefruit.begin()` defaults to a single
peripheral connection, so a central that connects and idles locks the real app out
until it leaves or the link supervision times out; and the Rx characteristic accepts
writes from anyone at any rate. The latter is inert while `raceboxOnWrite()` is empty
and becomes the actual attack surface the moment a command set exists.

> **Fix.** Documentation, not code: a line in the README next to the mounting advice.
> If you ever want it optional, Bluefruit supports LESC pairing without disturbing the
> GATT layout — but it would break app compatibility, so treat it as a build flag,
> never a default.

### SEC-2 — Clean — Memory safety and untrusted input: nothing found

all three trees · `.gitignore` · `test/`

No `strcpy`, `strcat`, `sprintf` or `alloca` anywhere. One `memcpy`, in
`pvtCallback()`, with a correct `sizeof`. Every `snprintf` is bounded and every fixed
buffer index-guarded, including the BLE receive path. `writeLittleEndian`'s
`static_assert` whitelist means a wrong-width store into the packet buffer fails to
compile rather than running off the end of an 80-byte payload — that is the single
best safety decision in the codebase.

UBX parsing is entirely the SparkFun library's; the firmware never touches a raw
length field. `pvtCallback()` fires from `checkCallbacks()` on the loop thread, not an
ISR, so the `latestPVT` / `newEpochAvailable` pair is not racing anything despite
lacking `volatile`.

Capture hygiene is right too: `test/*.jsonl` and `test/vectors*.gc1` are ignored with
a comment explaining that they carry real coordinates, and `git check-ignore` confirms
the local 22,986-fix vector file and all five captures are covered.

> **Fix.** None. Noted so the clean result is on the record rather than absent from it.

### BLD-1 — Low — Four `-Wformat` warnings, ESP32 only

`Gnimu-ESP32/g_gnss.cpp:54,60` · `Gnimu-ESP32/g_telemetry.cpp:189`

`%d` and `%u` against `uint32_t`. Correct on nRF, where `uint32_t` is
`unsigned int`; wrong on xtensa, where it is `long unsigned int`. Same width, so the
output is right today — but `g_telemetry.cpp` is in the byte-identical shared set,
which means one file has to satisfy both ABIs.

> **Fix.** `PRIu32` from `<inttypes.h>`, which is correct on both.

---

## Architecture notes

These are **not defects** and are not counted in the 27 findings above. They are
observations about the overall shape of the codebase and the plug-in design —
judgment rather than fault — recorded because the modular structure and the plug-in
contract went into code without outside critique. `ARC-*` IDs are stable and citable
the same way the finding IDs are.

### ARC-1 — One concurrency model, and exactly one breach

The entire codebase contains **three concurrency primitives**: three `volatile bool`s,
all named `deviceConnected`, all in `g_ble.cpp`. No `attachInterrupt`, no critical
sections, no FreeRTOS primitives, no `onReceive`.

That is a strictly cooperative-polled system — no interrupts, therefore no shared
mutable state, therefore no races — and it is an uncommonly disciplined stance. Most
firmware at this level reaches for ISRs early and leaks races for years. It is why
`pvtCallback()` is safe without `volatile`, and why a full read of ~18k lines turned up
essentially no concurrency bugs.

The cost is that loop latency becomes a **correctness** property rather than a quality
one — the 5.5 ms UART deadline of LAT-2 is load-bearing. The code plainly knows this
(the whole OLED display module is architected around it) but never states it as an
invariant anywhere.

And there is exactly one thing outside the model: BLE callbacks, which the project does
not control. That is where all three `volatile`s live. The plug-in contract now routes
protocol authors directly into that one breach — `onWrite` is the only place a future
protocol's code runs off-loop, and the contract does not mention it. That is the
connective tissue behind API-1 and API-2.

> **Suggestion.** Write the stance down as an explicit invariant, precisely *because*
> the plug-in seam is an invitation for someone else to break it.

### ARC-2 — The plug-in contract: the three decisions worth protecting

Recorded so that a later refactor does not undo them by accident.

A POD descriptor carrying function pointers, rather than an abstract base class, is the
correct embedded C++ call — no vtables, lives in flash, and no virtual-destructor
question. Compile-time `#if` selection over a runtime table is right for the flash
budget, and §8.1 records why.

The best of the three is the dependency-free constraint on `g_protocol.h`, and
specifically because it is **enforced by something executable**: the harness fails to
compile if someone adds `#include "config.h"`. It is a design rule that cannot rot,
which is what separates it from most of the rules in this codebase (see ARC-10).

### ARC-3 — `TransportKind` is platform knowledge living in protocol data

A protocol declares a value whose real meaning is "which of two platform
implementations should serve me." §8.4's reasoning for keeping `BLEUart` is sound —
do not reimplement working chunking — but that argues for the *behaviour*, not for
where the field lives. `g_ble` could infer it instead: does this descriptor's service
and channel shape match the Nordic UART? A protocol would then declare only what it
*is*.

As it stands, phases G and H become two independent GATT builders with nothing checking
that they agree, and RaceBox works on both paths only because the Nordic UART UUIDs
coincidentally *are* the RaceBox UUIDs. The code says exactly that out loud, which is
admirable — but it means the two paths have never been proven equivalent, only never
diverged.

### ARC-4 — Outbound got a design; inbound got a function pointer

Outbound is fully specified: sample → encoder → sink → transport, with synchrony and
buffer lifetime documented in `g_protocol.h`. Inbound is `onWrite(channel, data, len)`
with no thread contract, no buffer contract, no framing and no reassembly. BLE permits
a command to arrive split across two writes; nothing reassembles it.

This asymmetry is the root that API-1, API-2 and API-5 each surface a symptom of.

### ARC-5 — The descriptor has no protocol lifecycle

There is no `begin` / `end` / `onConnect` / `onDisconnect` on `ProtocolDescriptor`.
RaceChrono's CAN filter is per-session state that ought to reset when a client drops,
and there is nowhere to hook that — it would become a file-static with no reset point.

This is the class of gap only found by writing protocol #2, which is why it is worth
settling before phase F rather than during it.

### ARC-6 — `TelemetrySample` is a widening union

One wide struct carrying everything any protocol might want, pushed to every encoder.
Correct for two protocols. It grows monotonically with each one added, and every
protocol pays the copy cost for fields it ignores — a magnetometer or wheel-speed
channel would sit in the struct that RaceBox never reads.

> **Suggestion.** Decide where the ceiling is now. At some point the answer stops being
> one wide push and becomes a per-protocol pull, and it is much cheaper to know which
> side of that line you intend to stay on before there are five encoders.

### ARC-7 — The verification methodology does not extend to protocol #2

This is the gap most worth having on the radar.

`capture.py` bootstraps golden vectors from real captured traffic, so the harness proves
*byte-identical to the previous implementation*. That is exactly the right instrument
for a refactor and §11 says so. But it means the vectors encode the old behaviour
including the deliberately preserved bugs of §14.2.

For RaceChrono there is no traffic to bootstrap from unless you own a device that speaks
it. Phase F's encoder can therefore only be checked against hand-written expectations —
a categorically weaker instrument than 22,986 captured frames. The project's strongest
quality asset silently does not extend to the thing it was built to enable, and §11 does
not say what phase F's evidence standard is instead.

> **Suggestion.** Settle phase F's evidence standard in the design record before writing
> the encoder, so the answer is a decision rather than whatever ends up being convenient.

### ARC-8 — `config.h` is the next drift hole, and it is the one already closed once

§7.2 correctly celebrates moving the RaceBox constants out of `config.h` because they
were "byte-identical but UNCHECKED."

Eight of the eleven `IMU_TRIM_*` values are in exactly that position today — identical
across all three trees, feeding byte-identical `g_imu_trim.cpp`, and checked by nothing:

| Constant | ESP32 | nRF52840 | nRF52840-OLED |
|---|---|---|---|
| `IMU_TRIM_QUALIFY_MS` | 30000 | 30000 | 30000 |
| `IMU_TRIM_BLOCK_MS` | 1000 | 1000 | 1000 |
| `IMU_TRIM_LOCK_BLOCKS` | 5 | 5 | 5 |
| `IMU_TRIM_SPEED_MAX_MPS` | 0.5f | 0.5f | 0.5f |
| `IMU_TRIM_ACCEL_SANITY_TOL` | 0.25f | 0.25f | 0.25f |
| `IMU_TRIM_MAX_TILT_DEG` | 15.0f | 15.0f | 15.0f |
| `IMU_TRIM_REQUIRE_FIX` | 1 | 1 | 1 |
| `IMU_TRIM_PVT_STALE_MS` | 1000 | 1000 | 1000 |
| `IMU_GRAVITY_NATIVE` | 9.80665f | 1.0f | 1.0f |
| `IMU_TRIM_ACCEL_VAR_MAX` | 0.392f | 0.04f | 0.04f |
| `IMU_TRIM_GYRO_VAR_MAX` | 0.017453f | 1.0f | 1.0f |

Only the last three are genuinely unit-dependent. The same migration and the same
argument apply: a shared tuning header in the checked common set, with the three
unit-dependent values staying in `config.h`.

### ARC-9 — The deadline-anchored cadence idiom is a one-off

`imuPoll()`'s anchored-with-resync pattern (`lastX += INTERVAL`, resync only if more
than one interval behind) appears in exactly four lines, all in `g_imu.cpp`. Every other
timer in the project — battery sampler, switch poll, stats window, light-sleep
heartbeat, display slices — uses `= now`, which stretches the effective period under
load and quietly drops the real rate below its configured value.

In a codebase this attentive to timing, the good idiom being a one-off is the most
surprising internal inconsistency I found.

> **Suggestion.** A small `Cadence` helper would make the correct behaviour the default,
> in the same spirit as `ImuAxis` being a class rather than six inline EMAs.

### ARC-10 — Where the risk concentrates: comments as design record

The comment density is the most striking property of this codebase, and mostly a
strength — rationale with numbered rejected alternatives is rare below the professional
level. But it has one specific failure mode: **when a comment is the only home for an
invariant, comment rot is design-record rot.**

Four of the 27 findings — IMU-2, LAT-1, ROB-2 and ROB-5 — are comment/code
disagreements rather than logic bugs. That is a real signal about where this project's
risk sits: not in the algorithms, in the prose.

The mitigation is the one already running here: keep migrating invariants out of prose
and into things that cannot go stale — `static_assert`s, the harness, the
`check_common.sh` set. The protocol-constants migration of §7.2 is exactly that move,
ARC-2's host-compilability constraint is exactly that move, and ARC-8 is the next
candidate. Worth treating as a standing pattern rather than a series of one-time fixes.

### ARC-11 — Correction: the shared-library rationale is checkable, and slightly wrong

Both READMEs state that the modules are duplicated because "a shared library doesn't fit
the Arduino sketch build model." Arduino *does* support a sketchbook `libraries/` folder,
and `#include <g_protocol.h>` from all three sketches would work.

The real cost of that route is distribution friction — clone-and-open versus
install-a-library-first — which for a build guide aimed at hobbyists is a perfectly good
reason to duplicate instead. The decision is right; the stated reason is not, and it is
the kind of claim a reader can check.

Worth noting alongside it that the duplication's real cost is visible in IMU-3: a design
decision that landed in one tree only, in a file legitimately excluded from the shared
set, where no script could see the divergence.

---

## What I ran

So you know which findings are measured and which are read.

- **arduino-cli compile ×3** — all three variants, `--warnings all`, against the
  installed cores (esp32 3.3.11, Seeeduino nrf52 1.1.13). All succeed. Five
  sketch-level warnings total.
- **test/run_harness.sh** — 23,028 vectors (42 synthetic plus 22,986 captured), zero
  failures. The encoder reproduces every golden vector exactly.
- **src/tools/check_common.sh** — 13 files identical across all three trees, 11 more
  across the two nRF trees. I also enumerated both trees independently: no shared file
  is missing from either list.
- **Library source** — Bluefruit `BLECharacteristic::notify()`, esp32
  `BLECharacteristic::notify()`, both `Print::printf` implementations, and
  `RingBuffer.h` read directly to confirm BLE-1, BLE-2, LAT-2 and LAT-3 rather than
  infer them.
- **Tree-wide greps** — the `IMU_TRIM_*` table in ARC-8, the three-`volatile` count in
  ARC-1, and the single cadence-idiom site in ARC-9 are counted from the source, not
  estimated.
- **Not run** — nothing was flashed. Anything about real BLE clients, real MTU
  negotiation or real GNSS behaviour is reasoning from source, not observation.
- **Out of scope** — items the design record already tracks as deferred
  (`IMU_ENABLED`, the `ImuProtocolUnits` rename, the §14.2 `fixType` asymmetry and the
  §14.1 `headVehValid` finding) are deliberately not repeated here.

---

Read against `docs/multiprotocol-design.md` phases A–E (done) and F–H (pending).
Finding and `ARC-*` IDs are stable and safe to cite. No source file was modified.
