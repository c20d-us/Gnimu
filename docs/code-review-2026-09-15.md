# Gnimu Third-Pass Review

**Code scan · branch `gnimu_multiproto` · 15 Sep 2026**

A narrower pass than the first two: the three firmware source trees only, looking
for suboptimal algorithms or code structures, obvious vulnerabilities, and
convoluted code. Every file was read — the 24 all-variant and 8 nRF-shared files
once each, plus every variant-specific file.

| | |
|---|---|
| Variants compiled | 3 / 3 clean |
| Sketch warnings | 0 |
| ESP32 flash | 90% (1,183,615 of 1,310,720 bytes) |
| Findings | 1 high, 3 medium, 1 low-medium, 4 low |

Findings use `R3-*` IDs, distinct from the first review's IDs, the second
review's `R2-*` and the remediation record's `NEW-*`. Items already dispositioned
in `docs/code-review-remediation.md` — R2-2 (open, folded into ARC-5 at phase F),
LAT-4 and the rest — are not repeated.

---

## Summary

1. **R3-1 (High)** — a low-voltage boot never reaches System OFF on either nRF
   build.
2. **R3-2 (Medium)** — the ESP32 MTU check compares against a value the firmware
   wrote itself, so it cannot catch the failure it exists for.
3. **R3-3 (Medium)** — the "Clean up comments" commit also disabled live GNSS
   configuration.
4. **R3-4 (Medium)** — the nRF IMU read does double-precision math in software on
   the hot path.
5. **R3-5 (Low-medium)** — the OLED resends the whole frame every second whether
   or not it changed.
6. **R3-6 … R3-9 (Low)** — a 3 s boot wait on battery, a ~35 s worst-case GNSS
   sweep, one outsider-rate log line, and a set of small simplifications.

---

## High

### R3-1 — High — A low-voltage boot never actually powers off

`src/Gnimu-nRF52840/g_state.cpp:98` → `src/Gnimu-nRF52840/g_power.cpp:99`
(and the same lines in the OLED tree)

**Cause.** At boot, `stateBegin()` sends a switch-on with a cell below
`BATTERY_CUTOFF_V` straight to `powerEnterDeepSleep()`. That happens before
`bleBegin()`, so the SoftDevice has not been enabled. `powerEnterDeepSleep()`
calls `sd_power_system_off()`, and the SoftDevice API header places
`SD_POWER_SYSTEM_OFF` in the `SOC_SVC_BASE_NOT_AVAILABLE` range — "SVCs that are
not available when the SoftDevice is disabled". The call returns an error, and
execution falls into the `while (1) delay(100)` the comment says only a debugger
reaches.

**Effect.** The device logs `💤 Entering deep sleep (System OFF).` and then stays
awake. A cell that was already below cutoff keeps draining, and plugging in USB
does not bring it back, because VBUS wake is a property of real System OFF.

**Scope.** Only the boot path. The runtime paths — `RUNNING -> DEEP_SLEEP` on
voltage cutoff or idle timeout — run after `bleBegin()`, with the SoftDevice
enabled, and work as intended.

**Reference.** The core's own `systemOff()` in `cores/nRF5/wiring.c` handles
exactly this case:

```c
uint8_t sd_en;
(void) sd_softdevice_is_enabled(&sd_en);
if (sd_en) {
  sd_power_system_off();
} else {
  NRF_POWER->SYSTEMOFF = 1;
}
```

> **Fix.** Apply the same check in `powerEnterDeepSleep()`. No host harness can
> see this; verify on the bench by temporarily raising `BATTERY_CUTOFF_V` above
> the cell voltage and confirming the current draw after boot.

---

## Medium

### R3-2 — Medium — The ESP32 MTU check compares against a number the firmware wrote itself

`src/Gnimu-ESP32/g_ble_port_esp32.cpp:74-76`

```cpp
server->updatePeerMTU(param->connect.conn_id, kRequestedMtu);
peerMtu.store(server->getPeerMTU(param->connect.conn_id), ...);
```

**Cause.** In esp32 core 3.3.11, `BLEServer::updatePeerMTU()` does not negotiate
anything. It writes the value into the server's own `m_connectedServersMap`
entry, which `addPeerDevice()` has just created, and `getPeerMTU()` reads that
same entry straight back. `peerMtu` therefore starts at 91 on every connection,
whatever the link is actually using. There is no request on air — a GATT server
cannot initiate an MTU exchange; only the central can.

**Effect.** If a central never runs the exchange, the driver's
`len > blePortMaxFrame()` check passes, `notify()` hands 88 bytes to the
controller, and the controller cuts them to 20. That is exactly the failure BLE-2
was written to catch, and it is why the remediation record notes that the
refusal path "remains untested": for a central that never exchanges, it cannot
fire. The same was true of the per-frame `getPeerMTU()` read before the port
split, since the seeded value was already in the peer map.

The remediation record describes the current code as recording "the requested
MTU at connect, as `getPeerMTU()` would report it". That is accurate, but it
reads as though `getPeerMTU()` reflected the link.

**Correction to the first review.** BLE-2 described `updatePeerMTU()` as "a
request, and a central is free to decline it". On this core it is not a request
at all.

> **Fix.** Either start `peerMtu` at 23 and accept the ~650 ms of refusals at
> connect (the one-shot refusal and recovery lines already handle that cleanly),
> or start optimistically but fall back to 23 if no `onMtuChanged` arrives within
> ~2 s. Either way, drop the `updatePeerMTU()` / `getPeerMTU()` pair; if the
> optimistic value is kept, `peerMtu.store(kRequestedMtu)` says the same thing
> honestly.

### R3-3 — Medium — The "Clean up comments" commit also disabled live GNSS configuration

`src/Gnimu-nRF52840/g_gnss.cpp:165-169` (all three trees) · commit `2c5fab6`

`2c5fab6` replaced a live call with a commented-out block and removed its reason
line:

```diff
-  // AssistNow Autonomous is explicitly DISABLED to save CPU cycles.
-  if (myGNSS.setAopCfg(0, 0, VAL_LAYER_RAM_BBR)) {
+  // if (myGNSS.setAopCfg(0, 0, VAL_LAYER_RAM_BBR)) {
```

Nothing in the docs records the change. The receiver now runs with whatever
AssistNow Autonomous setting it holds. If that default is on — I believe it is on
M10 firmware; `tools/common/gnss_ver` would confirm — the receiver is spending
CPU on orbit prediction, on a part the README already describes as pushed to its
nav-rate limit.

A behaviour change inside a comment-cleanup commit is also the kind of change a
reviewer reading that commit would not look for.

> **Fix.** If it was deliberate, delete the block and record the reason. If not,
> restore the call. The GNSS harness golden should change either way, which is a
> useful confirmation of which state the tree is in.

### R3-4 — Medium — The nRF IMU read does double-precision math in software

`src/Gnimu-nRF52840/g_imu_lsm6ds3.cpp:159-164`

**Cause.** `imuSensorRead()` scales each axis with the Seeed library's
`calcAccel()` and `calcGyro()`:

```cpp
float output = (float)input * 0.061 * (settings.accelRange >> 1) / 1000;
float output = (float)input * 4.375 * (gyroRangeDivisor) / 1000;
```

The literals are `double`. The Cortex-M4F FPU is single-precision only, so the
compiler emits software calls. Disassembly of the build's `LSM6DS3.cpp.o` shows,
per axis: `__aeabi_i2d`, `__aeabi_f2d`, two `__aeabi_dmul`, `__aeabi_d2f`.

**Effect.** Five libgcc soft-float calls per axis, 30 per sample, about 3,000 per
second, on the loop between `gnssPoll()` calls — to multiply by a constant that
never changes.

**Context.** The remediation deliberately kept these calls ("Keep the library's
`calcAccel()` / `calcGyro()`"), but in a correctness discussion about the checked
burst read, not a performance one. The driver's own comment already distrusts
them for a different reason: they scale from the library's copy of the settings,
not the chip.

> **Fix.** Precompute float scale factors at compile time, as the MPU-6050 driver
> already does with `kAccelCountsPerG` / `kGyroCountsPerDps`:
>
> - accel, g per LSB: `0.061e-3f × (IMU_ACCEL_RANGE_G / 2)`
> - gyro, deg/s per LSB: `4.375e-3f × (IMU_GYRO_RANGE_DPS / 125)`, with 245 dps
>   mapping to a multiplier of 2
>
> Float rounding differs from the library's double path in the last digit, so
> the IMU harness goldens may shift. Regenerate them as a deliberate step.

---

## Low-medium

### R3-5 — Low-medium — The OLED resends the whole frame every second

`src/Gnimu-nRF52840-OLED/g_display.cpp:316` (`displayUpdate()`), `:280`
(`pushSlice()`)

**Cause.** Every `DISPLAY_REFRESH_INTERVAL_MS` (1000) the frame is re-rendered and
all `SLICE_COUNT` slices are pushed: 16 tiles / 4 per slice × 8 tile rows = 32
slices, at `DISPLAY_SLICES_PER_EPOCH` (2) per epoch.

**Effect.** A full frame takes 16 of every 20 epochs, so the display is on the I2C
bus in ~80% of epochs, continuously — to redraw a screen where, on most refreshes,
only the uptime seconds changed. The frame also takes ~0.8 s to land, so the panel
lags the data by most of a second.

The phase-lock machinery (`epochJustLanded`, `DISPLAY_SLICES_PER_EPOCH`, the
fallback spacing) exists to spread this cost around the UART deadline. Reducing
the cost removes most of the pressure on it.

> **Fix.** Keep a 1,024-byte shadow of the last pushed frame (the size of U8g2's
> full buffer, from `getBufferPtr()`). After `renderFrame()`, compare each
> 32-byte slice against the shadow and queue only the slices that differ; copy
> each into the shadow as it is pushed. A burn-in shift every
> `DISPLAY_SHIFT_INTERVAL_MS` changes everything and forces a full push, which is
> correct.
>
> Typical traffic drops from 32 slices a second to a handful, and a frame lands
> in 1–3 epochs instead of ~16.

---

## Low

### R3-6 — Low — nRF battery boots wait 3 s for a USB host that cannot exist

`src/Gnimu-nRF52840/Gnimu-nRF52840.ino:38` ·
`src/Gnimu-nRF52840-OLED/Gnimu-nRF52840-OLED.ino:39`

```cpp
while (!Serial && millis() - t0 < 3000) {
}
```

This runs before `stateBegin()`, on every boot. On battery with no USB, no host
can be attached, so it always runs the full 3 s. `Adafruit_USBD_CDC::operator
bool()` calls `yield()` when not connected, but on this core that yields only to
equal- or higher-priority tasks; the idle task never runs, so the CPU spins awake
the whole time.

It delays reaching RUNNING by 3 s and, combined with R3-1, delays the low-voltage
decision on exactly the boot that most needs it.

> **Fix.** Gate the wait on `powerUsbPresent()`. It is a direct `NRF_POWER`
> register read, valid before `Serial.begin()` and before the SoftDevice is
> enabled. No VBUS, no host, no wait.

### R3-7 — Low — The GNSS baud sweep can block boot for about 35 s

`src/Gnimu-nRF52840/g_gnss.cpp:57-58` (all three trees)

```cpp
const uint32_t baudRates[] = {GNSS_BAUD, 4800,   9600,   19200,  38400,
                              57600,     115200, 230400, 460800, 921600};
```

**Duplicate.** `GNSS_BAUD` is 115200, so that rate is tried twice.

**Timeout.** `myGNSS.begin(stream)` uses the default `kUBLOXGNSSDefaultMaxWait`
of 1,100 ms, and `DevUBLOXGNSS::init()` calls `isConnected()` up to three times.
The library's own header says that default is sized for SerialUSB and that 250 ms
is enough otherwise. Each failed rate costs about 3.3 s plus the sweep's 200 ms of
delays — roughly 35 s across ten entries with no receiver.

All of that is inside `setup()`. On the battery builds the loop, and with it the
low-voltage cutoff, does not start until the sweep gives up.

> **Fix.** Skip the duplicate, and pass a shorter `maxWait` (around 250 ms) for
> every sweep rate after the first. Keep the default on the first `GNSS_BAUD`
> attempt, which covers a receiver still booting after power-on. A no-receiver
> boot drops to roughly 10 s.
>
> Related: the `static_assert` listing valid `GNSS_BAUD` values is duplicated in
> each `config.h` and has to match this array by hand. Since `g_gnss.cpp` is in
> the checked set, the check could live next to the array.

### R3-8 — Low — One log line still fires at a rate an outsider controls

`src/Gnimu-nRF52840/g_ble.cpp:99` (all three trees)

```cpp
LOG_PRINTF("📨 BLE write: %u byte(s) on channel %u\n", ...);
```

Every other per-event log in the firmware has become a counter reported once per
stats window. This one still prints on every dispatched inbound write, and the GATT
is open, so any central in range sets its rate. A write-without-response flood
makes `rxDispatchOne()` log once per loop iteration. With a console attached on
ESP32, that paces the loop to UART drain speed — about 3 ms per iteration once the
512-byte TX ring fills.

The impact is bounded: the ESP32's GNSS ring tolerates that pace, the queue drops
and counts excess writes, and on nRF the guard skips the log when no host is
attached. But it is the one exception to a pattern the codebase otherwise follows,
and for a protocol with a real command set it becomes noise on every command.

> **Fix.** Count dispatched writes and report the delta in the 1 Hz report,
> alongside the dropped-write line.

### R3-9 — Low — Small simplifications

Each of these keeps the same fact in two places, or keeps state nothing reads.

- **Duplicate staleness check.** `trimSpeedMps()`
  (`src/Gnimu-nRF52840/g_imu.cpp:90`) tracks PVT staleness with a static iTOW, a
  sample counter and `IMU_TRIM_PVT_STALE_MS`. `gnssStalled()` now exists in
  `g_gnss.cpp` with the same 1 s threshold at 20 Hz. Use it and drop the constant
  from `g_imu_tuning.h`.
- **A descriptor field read only by assertions.** `ProtocolDescriptor::transport`
  (`src/Gnimu-nRF52840/g_protocol.h:146`) has no runtime reader; the ports use
  `PROTOCOL_TRANSPORT`, and a `static_assert` in `g_proto_racebox.cpp` exists only
  to keep the two equal. Drop the field and key the Nordic-UART shape assert on
  `PROTOCOL_TRANSPORT`.
- **An unused variable.** `sMinAdc` (`src/Gnimu-nRF52840/g_battery.cpp:109`) is
  reset and updated in both the blocking and the polled sampler paths, and never
  read. Only `sMaxAdc` feeds `ingestPeak()`.
- **A redundant pointer.** The `pvt` static in
  `src/Gnimu-nRF52840/g_telemetry.cpp:40` is set to the epoch cache on every
  consume and read only by the stats report; `gnssLatestPvt()` returns the same
  object.
- **Inlined log formatting.** `LOG_PRINTF` (`g_log.h`) expands its `snprintf`,
  length clip and `Serial.write()` inline, at 30–34 sites per tree. One
  out-of-line `logPrintf(const char *fmt, ...)` with
  `__attribute__((format(printf, 1, 2)))` keeps the compile-time argument
  checking, holds one copy of the clip logic, and saves some flash on the ESP32,
  which is at 90%.
- **Near-identical LED files.** `g_led.cpp` in the nRF and OLED trees differ only
  by the 5-line `#if !LED_ENABLED` / `displayIsPresent()` block, which keeps the
  whole file out of the checked set. A default `displayIsPresent()` returning
  `false` on the base tree would let the two files match.

---

## Checked and fine

- **`fabs` on floats.** `ImuAxis::update()` calls `fabs()` on a float. The
  disassembly shows a single `vabs.f32` instruction — no double promotion.
- **The inbound write queue.** Single producer (the BLE callback), single consumer
  (the loop): the producer publishes `rxTail` only after the slot is written, and
  the consumer advances `rxHead` only after `onWrite` returns, with
  acquire/release ordering on both sides. Correct.
- **Session tracking.** Following `blePortSessionCount()` rather than a connected
  flag correctly ends the old session and starts the new one when a disconnect and
  reconnect both happen between two polls.
- **Stream slicing.** `bleRxFromCallback()` keeps trying later slices after one
  fails to queue, which could leave a gap in a byte stream. It is not reachable in
  practice: the nRF callback task outranks the loop, so the consumer cannot free a
  slot mid-slice, and the ESP32 port uses whole-message writes.
- **MPU-6050 configuration read-back.** A failed I2C read would return enum 0, which
  matches `MPU6050_RANGE_2_G` / `_250_DEG` / `_BAND_260_HZ`. The shipped
  configuration uses none of those, so the verification holds.
- **Memory safety.** Every `memcpy` is bounded (`sizeof` on the PVT copy; the
  inbound queue drops or splits anything over `TELEMETRY_MAX_WRITE_LEN` before
  copying). Every `snprintf` is bounded, `LOG_PRINTF` clips long lines, and the
  ESP32 port's channel table is bounded by a `static_assert`.

---

## What I ran

- **arduino-cli compile ×3** — all three variants with `--warnings all`, against
  the IDE's configuration (the sketchbook has moved, so the IDE's
  `arduino-cli.yaml` was passed explicitly). All succeed with zero sketch
  warnings.
- **check_common.sh** — 24 all-variant and 8 nRF-shared files identical; coverage
  sweep and concurrency tripwire green.
- **Disassembly** — `ImuAxis::update()` and the Seeed library's `calcAccel()` /
  `calcGyro()` from the nRF build objects, with the core's `arm-none-eabi-objdump`.
- **Library and core source** — the SoftDevice `nrf_soc.h` SVC ranges and the
  core's `systemOff()` (R3-1); esp32 `BLEServer::updatePeerMTU()`, `getPeerMTU()`
  and the connect handler (R3-2); the Seeed LSM6DS3 scaling functions (R3-4);
  `Adafruit_USBD_CDC::operator bool()` (R3-6); SparkFun `DevUBLOXGNSS::init()`
  and `kUBLOXGNSSDefaultMaxWait` (R3-7).
- **Git history** — `git log -S` to find the commit that commented out the
  AssistNow call (R3-3).
- **Not run** — the host harnesses (the scan was scoped to source), and nothing
  was flashed. Current draw in R3-1 and on-air MTU behaviour in R3-2 are reasoning
  from source, not measurement.

---

Read against `docs/code-review-remediation.md` so that decided items are not
re-raised. `R3-*` IDs are stable and safe to cite. No source file was modified.
