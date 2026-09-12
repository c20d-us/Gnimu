# Runtime path — one GNSS epoch

What happens each time the receiver produces a fix, from sensors to the app.
Companion to [`multiprotocol-design.md`](multiprotocol-design.md).

Driven by `telemetrySendIfReady()`, called every `loop()`. The work below runs
only when `gnssConsumePvt()` returns a new epoch **and** a BLE client is
connected.

```mermaid
flowchart LR
    PVT["gnssConsumePvt()<br/><i>UBX-NAV-PVT</i>"] --> BS
    IMUR["imuReadProtocolUnits()<br/><i>trim-corrected,<br/>milli-g · centi-deg/s</i>"] --> BS
    BAT["batteryGetStatus()<br/><i>percent + charging</i>"] --> BS

    BS["buildSample()<br/><i>field copies only —<br/>no decisions</i>"] --> S(["TelemetrySample<br/><i>u-blox scaling · RAW flags</i>"])

    subgraph seam["the only protocol-aware step"]
        ENC["encode() via ACTIVE_PROTOCOL<br/><i>g_proto_racebox:<br/>fixType clamp · valid-fix rule<br/>· battery packing · checksum</i>"]
    end

    S --> ENC
    ENC -->|"emit(channel, bytes)"| EMIT["bleEmitFrame()"]

    EMIT --> NRF["nRF · BLEUart<br/><i>splits an 88-byte notify<br/>if the MTU is small</i>"]
    EMIT --> ESP["ESP32 · characteristic[channel]<br/><i>setValue + notify,<br/>no chunking</i>"]

    NRF --> APP["RaceBox-compatible app"]
    ESP --> APP
```

## Why the split lands where it does

**`buildSample()` is deliberately boring.** It copies fields and makes no
judgements — `fixType` goes in raw, the validity booleans go in individually,
battery goes in as percent plus a charging flag rather than a packed byte. Every
interpretation happens one step later, inside the encoder.

That is not tidiness. `buildSample()` is the one link the host harness cannot
reach, because reconstructing a `UBX_NAV_PVT_data_t` would drag in the SparkFun
library. Keeping it trivial concentrates all the risk in `encode()`, which the
harness tests exhaustively. The copy itself is covered end-to-end instead, by
capturing from a real device and diffing against the pre-refactor reference.

**`encode()` emits rather than returns.** RaceBox sends one 88-byte frame, but
RaceChrono would send 20 bytes on its GPS characteristic and 3 more on GPS Time.
A sink handles one call producing N frames on different channels, with no
allocation — the buffer is stack-local and the call is synchronous.

**The transport split is real, not cosmetic.** `bleEmitFrame()` has the same
signature as `TelemetryEmit`, so it is handed straight to the encoder with no
adapter. Below it the two stacks genuinely differ: Bluefruit's `BLEUart` chunks
an oversized notify and manages TX backpressure, which is why the RaceBox path
was left on it rather than rebuilt. The ESP32 has no equivalent and does no
chunking: it requests an MTU derived from the largest frame any protocol emits
(`kRequestedMtu = PROTOCOL_MAX_FRAME_LEN + 3`) and refuses to send a frame that
does not fit rather than truncating it — so there, one channel builder serves
every protocol and `TransportKind` is ignored.

## What is not on this path

Cadence is one frame per GNSS epoch, so nothing here is timer-driven. The IMU
runs its own sampling and decimation in `g_imu`/`g_imu_trim` and this path only
reads the latest cached value. The serial stats line, the LED and the OLED all
observe state separately — the OLED uses `gnssLatestPvt()`, the non-consuming
reader, precisely so it cannot steal an epoch from this path.

## Concurrency — one model, one exception

**The firmware is cooperative-polled.** Every module is a `*Poll()` or
`*Update()` called from `loop()`, and everything on the path above runs there.
There are no interrupt handlers of ours (no `attachInterrupt`), no tasks we
create, no semaphores, no critical sections. Interrupts exist — the UART
receive ring, USB, the SoftDevice — but they live in the cores, and firmware
code consumes their results by polling, never by running inside them. That is
why `pvtCallback()` needs no `volatile`: the SparkFun library fires it from
`checkCallbacks()`, on the loop.

**The price is that loop latency is a correctness property.** Nothing may block
long enough to starve `gnssPoll()`. The budget is stated where the UART is
owned - since 2026-09-11 that is each `g_gnss_port_<mcu>.cpp`, not the shared
driver - and it is a different *kind* of constraint per MCU:
on the nRF52840 a 63-byte ring cannot hold one 100-byte NAV-PVT, so the loop
must drain *during* each message (a phase-sensitive 5.47 ms deadline); on the
ESP32 the ring holds 2.5 messages, leaving a duration budget of ~256 ms.

**The one exception is BLE stack callbacks** — the only code we own that runs
outside `loop()`, and the one thing the project does not schedule:

| stack | context | relation to `loop()` |
|---|---|---|
| nRF52840 (Bluefruit) | callback task | preemptive, single core |
| ESP32 (Bluedroid) | BTC task | **different core, truly parallel** |

**Rules for callback code:**

1. **A lone flag may cross as a plain or `volatile` write** — provided it
   publishes nothing else.
2. **Anything a flag publishes needs `std::atomic` release/acquire.** `volatile`
   orders only other volatile accesses, and on ESP32 the reader is on another
   core. Write the data, *then* release-store the flag; the reader
   acquire-loads the flag, *then* reads the data.
3. **Inbound data goes through the `rxPush()` single-producer/consumer ring**,
   never straight into shared state. That is what makes a protocol's `onWrite`
   a loop-thread call (see its contract in `g_protocol.h`).
4. **Do almost nothing.** The callback task also runs the stack.

**What crosses today, and the verdict on each:**

| state | writer → reader | verdict |
|---|---|---|
| `deviceConnected` (nRF) | callbacks → loop | lone `volatile` flag; publishes nothing — rule 1 |
| `deviceConnected` + `connectTimeMs` (ESP32) | `onConnect` → loop | flag publishes a timestamp — rule 2, `std::atomic<bool>` |
| RX ring indices | `onWrite` ↔ loop | `std::atomic`, release/acquire — rule 3 |
| `droppedWrites` | callbacks → loop | one writer; 32-bit aligned access is atomic on both MCUs — benign |
| `lastLoggedMtu` (nRF) | callbacks **and** loop | two writers; worst case one duplicated or missed log line — accepted |
| serial logging | both contexts | both write paths are mutex-guarded (ESP32 `UART_MUTEX_LOCK`; nRF TinyUSB with its FreeRTOS FIFO mutex) — lines may interleave, not corrupt |

The ESP32 row was wrong until 2026-09-10 (ARC-1): the flag was set *before* the
timestamp, with an `updatePeerMTU()` call between them, so the loop on the other
core could see "connected" beside the previous connection's timestamp and skip
the `BLE_CONNECT_SETTLE_MS` window. It went unnoticed because nothing listed
what crosses — which is what this table is for. **Adding to a callback means
adding a row here.**

**Enforced, not just stated.** `src/tools/check_common.sh` fails if an
interrupt-attach, task-creation, semaphore, queue or critical-section call
appears in any firmware tree — so the first line of this section is a checked
fact rather than a claim.

## See also

- [`architecture-modules.md`](architecture-modules.md) — what includes what, and the two forbidden edges
- [`architecture-verification.md`](architecture-verification.md) — how `encode()` is proven correct
