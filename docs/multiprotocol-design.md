# Multi-protocol telemetry — protocol plug-in architecture

Design record for separating Gnimu's RaceBox-specific output from its sensor
pipeline, so a second wire protocol can be added by writing one encoder rather
than editing the core.

Status: **phases A-E complete, verified on all three variants. RaceChrono (F-H) not started.** Branch
`gnimu_multiproto`. Started 2026-09-09.

The canonical sample, the RaceBox encoder and both transport builders exist.
The encoder is verified against 8,980 golden vectors (§10, §11) and the whole
chain is verified end-to-end on ESP32 hardware. The nRF variants build from the
same shared sources but have not been flashed since the refactor. The
GATT-channels builder exists only on ESP32; the nRF side gets it in phase G
with RaceChrono.

Diagrams (standalone, each answering one question):
[module dependencies and sharing scope](architecture-modules.md) ·
[the per-epoch runtime path](architecture-runtime.md) ·
[how the encoder is verified](architecture-verification.md).

Companion document: [`racechrono-ble-mapping.md`](racechrono-ble-mapping.md) —
the protocol reference and NAV-PVT field mapping for the intended second
protocol. Nothing there is implemented either.

---

## 1. Problem

Gnimu emulates the RaceBox Mini, and that decision is spread across the
firmware rather than isolated in it:

- `g_telemetry.cpp`'s `sendPacket()` builds an 88-byte RaceBox Data Message
  inline from the live PVT and IMU state — 130 lines mixing field copies with
  RaceBox-specific interpretation.
- `g_ble.cpp` advertises as `"RaceBox Mini <id>"`, exposes RaceBox's Device
  Information strings, and uses Bluefruit's `BLEUart` because its UUIDs happen
  to be exactly the RaceBox service UUIDs.
- `config.h` carries seven `RACEBOX_*` constants plus their validating
  `static_assert`s, in every variant.

Nothing is wrong with any of it, but there is no seam. Adding a second protocol
today means editing all three, in three sketch folders, with no way to prove the
first protocol still emits identical bytes.

The goal is a **plug-in seam**: a canonical sample struct, an encoder per
protocol, and a transport layer that builds whatever a protocol descriptor
tells it to build. Selection is a compile-time `#define`.

This is a fan-out serialization problem. The GNSS/IMU acquisition pipeline does
not change.

---

## 2. What changes and what doesn't

| | Today | After |
|---|---|---|
| GNSS / IMU / trim / battery / power / state | — | **untouched** |
| Cadence (one frame per GNSS epoch) | `g_telemetry` | `g_telemetry` |
| Field packing | inline in `sendPacket()` | `g_proto_<name>.cpp` |
| Protocol policy (fix clamping, validity rules) | inline in `sendPacket()` | `g_proto_<name>.cpp` |
| BLE identity + service topology | hardcoded in `g_ble.cpp` | protocol descriptor |
| BLE stack mechanics | `g_ble.cpp` | `g_ble.cpp` |
| Protocol constants | `config.h` × 3 | `g_proto_<name>.h` |

---

## 3. Scope

Settled before design:

| Decision | Outcome |
|---|---|
| Selection mechanism | Compile-time `#define TELEMETRY_PROTOCOL` |
| Second protocol | RaceChrono DIY BLE, **GPS-only** |
| Concurrency | One protocol active at a time; do not foreclose fan-out |
| Variant scope | **All three.** ESP32 + M100-5883 (~$30) is the cheapest possible RaceChrono DIY device, which is the whole point of supporting it there |

RaceChrono has **no IMU channel** — its GPS characteristic is 20 bytes and
fully accounted for. IMU data can only ride as synthetic CAN PIDs decoded by
user-written equations in the app's Vehicle profile. That is a separate
deliverable and is out of scope here; see the mapping document §1 and §6.

---

## 4. The canonical sample

`TelemetrySample` is the single hand-off between acquisition and serialization.
Two rules govern what goes in it.

**Rule 1 — u-blox native scaling, no reinterpretation.** Latitude stays
`deg × 10⁷`, speed stays mm/s, heading stays `deg × 10⁻⁵`. RaceBox is
essentially a re-frame of UBX-NAV-PVT, so its encoder becomes a copy; and
RaceChrono wants `deg × 10⁷` for lat/lon anyway, which is the same thing. No
value gets a rounding trip it did not need.

**Rule 2 — raw flag bits, never derived ones.** This is the important one.
Three things in today's `sendPacket()` look like data but are RaceBox
*opinions*:

- clamping u-blox `fixType` 1 (dead reckoning) and 4 (GNSS+DR) to 0, because
  the RaceBox protocol only defines 0 / 2 / 3
- defining "valid fix" as `fixType == 3 && gnssFixOK` — the strictest available
  reading
- packing battery as bit-7-charging plus a 7-bit percent

If any of those land in the sample, every future protocol silently inherits
RaceBox's editorial choices. The mapping document shows exactly why this
matters: RaceChrono's fix field is **NMEA GGA quality** (0 invalid, 1 GPS fix,
2 DGPS), not u-blox `fixType`, so it needs a *different* derivation from the
same raw inputs. A shared "fixType" that had already been clamped would produce
a wrong answer that still looks plausible.

So the sample carries `fixType`, `gnssFixOK`, `headVehValid`, `validDate`,
`validTime`, `fullyResolved` as-is, plus `batteryPercent` and
`batteryCharging` as separate fields. Each encoder derives its own.

`buildSample()` stays deliberately mechanical — pure field copies, no
decisions — so all judgment lives in the encoder, which is the part the host
harness can test.

The IMU fields carry the trim-corrected values `imuReadProtocolUnits()` already
returns (milli-g and centi-deg/s). Those units are protocol-neutral enough to
serve as canonical; only the field *names* change, since `ImuProtocolUnits`
names accelerometer axes `gX/gY/gZ` and gyro axes `rX/rY/rZ`, which is
confusing once shared.

---

## 5. The plug-in contract

One new header, `g_protocol.h`, holds the whole interface. **Types only — no
`config.h`, no `Arduino.h`.** Per the precedent in
`imu-trim-design.md` §5.9, a shared module takes its configuration as
parameters rather than reading it, which is what makes it exercisable against
known inputs on a host. That rationale applies here with full force: host
testing is the only way to validate wire-format correctness without owning the
target hardware.

Sketch, not final:

```c
// g_protocol.h  —  <stdint.h> and <stddef.h> only

struct TelemetrySample {
  // GNSS, u-blox native scaling, raw flags
  uint32_t iTOW;
  uint16_t year;  uint8_t month, day, hour, min, sec;
  int32_t  nano;                    // SIGNED, roughly +/-5e8
  uint32_t tAcc;
  uint8_t  fixType;                 // RAW: 0,1,2,3,4,5 - not clamped
  bool     gnssFixOK, headVehValid;
  bool     validDate, validTime, fullyResolved, validMag;
  uint8_t  numSV;
  int32_t  lon, lat;                // deg * 1e7
  int32_t  height, hMSL;            // mm
  uint32_t hAcc, vAcc;              // mm
  int32_t  gSpeed;                  // mm/s
  int32_t  headMot;                 // deg * 1e5
  uint32_t sAcc, headAcc;
  uint16_t pDOP;                    // * 0.01

  // IMU, trim-corrected
  int16_t  accelX, accelY, accelZ;  // milli-g
  int16_t  gyroX,  gyroY,  gyroZ;   // centi-deg/s

  // Power
  uint8_t  batteryPercent;          // 0..100
  bool     batteryCharging;
};

// Emits one frame on one of the protocol's channels. Called synchronously, so
// the encoder may hand over a stack buffer without copying.
typedef void (*TelemetryEmit)(uint8_t channel, const uint8_t *data, size_t len);

enum ProtocolProps : uint8_t {
  PROP_READ = 1, PROP_NOTIFY = 2, PROP_WRITE = 4, PROP_WRITE_NR = 8
};

enum TransportKind : uint8_t { TRANSPORT_NORDIC_UART, TRANSPORT_GATT_CHANNELS };

struct ProtocolChannel {
  uint16_t    uuid16;      // 0 => use uuid128
  const char *uuid128;     // null unless uuid16 == 0
  uint8_t     props;
};

struct ProtocolDescriptor {
  const char *modelName;                        // g_ble appends DEVICE_ID
  const char *manufacturer, *hwRev, *fwRev;     // DIS; null = omit the service
  uint16_t    serviceUuid16;
  const char *serviceUuid128;
  TransportKind transport;
  const ProtocolChannel *channels;
  uint8_t     channelCount;
  void (*encode)(const TelemetrySample &, TelemetryEmit);
  void (*onWrite)(uint8_t channel, const uint8_t *data, size_t len);  // nullable
};
```

Three load-bearing properties:

**The descriptor is pure data.** No Bluefruit types, no ESP32 `BLEUUID`. That
is what lets one encoder compile unchanged against two completely different BLE
stacks and live in the all-variant common set. Each `g_ble.cpp` translates the
data into whatever its library wants.

**`encode()` emits rather than returns.** RaceBox sends one 88-byte frame;
RaceChrono sends 20 B on the GPS characteristic and 3 B on the GPS Time
characteristic. One call, N frames, zero allocation. This is the hedge that
keeps fan-out possible later without costing anything now.

**`onWrite` keeps protocol logic out of `g_ble`.** RaceBox's RX characteristic
currently just logs incoming bytes; RaceChrono's CAN filter characteristic
would need real command parsing. The transport routes the write and stays dumb
about what it means.

---

## 6. Transport: inverting `g_ble`

`g_ble.cpp` tangles three concerns today, and only one of them is
protocol-specific:

| Concern | Protocol-specific | Platform-specific | Destination |
|---|---|---|---|
| Stack mechanics — init, MTU ceiling, TX power, connect/disconnect callbacks, advertising lifecycle, `bleStop()`, Battery Service | no | **yes** | stays |
| Identity and topology — device name, DIS strings, service and characteristic UUIDs and properties | **yes** | no | descriptor |
| The byte pipe — `bleSendPacket()` | no | no | gains a channel argument |

So `g_ble` is not split out; it is **inverted**. It stops knowing what RaceBox
is and starts building whatever the active descriptor hands it.

`bleSendPacket(uint8_t*, size_t)` becomes
`bleEmitFrame(uint8_t channel, const uint8_t*, size_t)`, matching
`TelemetryEmit`. `g_telemetry` passes `bleEmitFrame` straight to `encode()`.
Channels are indices into the descriptor's array, so no per-protocol enum has
to grow.

### 6.1 `TransportKind`, and why it exists

The two BLE stacks are not symmetric, and the asymmetry is worth preserving
rather than papering over.

On **nRF (Bluefruit)**, the RaceBox path uses `BLEUart`, which is doing real
work: `g_ble.cpp` documents that it splits the 88-byte notify when the MTU is
small, and it manages TX backpressure internally. Replacing it with hand-rolled
characteristics to satisfy a uniform abstraction means re-implementing that, on
the one path that is known good, in a firmware whose stated design goal is
consistent latency.

On **ESP32**, none of that applies: `bleSendPacket()` is `setValue()` +
`notify()` with no chunking at all, relying on `BLE_MTU_BYTES` (128, with a
`static_assert` ≥ 91) being negotiated up front. Services and characteristics
are already built explicitly, and the Device Information Service is already
built by **iterating a data table**. RaceBox and RaceChrono are the same shape
there.

So `TransportKind` is honored by the nRF builder and collapses on ESP32, which
uses one uniform channel builder for both protocols. The enum declares what the
*protocol* is — a byte stream versus discrete channels — and each platform
implements it with whatever its stack offers. It exists for exactly one reason:
keeping `BLEUart` on the working RaceBox path. If RaceBox ever moves to the
channel builder, flip its descriptor value and delete the branch.

### 6.2 ESP32 specifics to handle

Three details, all known at design time:

- **CCCD descriptors.** ESP32 requires an explicit `BLE2902` on every notify
  characteristic; Bluefruit adds it automatically. This asymmetry is precisely
  what `PROP_NOTIFY` abstracts — each builder honors it its own way.
- **16-bit UUIDs.** RaceChrono uses `0x1FF8` and `0x0001`–`0x0004`, but the
  ESP32 code currently passes 128-bit strings everywhere. Hence both `uuid16`
  and `uuid128` in `ProtocolChannel`; the builder widens as needed.
- **Handle budget.** `createService()` takes a handle count covering every
  characteristic *and* its descriptors. Undersize it and the extra
  characteristics fail to register quietly. Size it from `channelCount` rather
  than accepting the default; verify the arithmetic at implementation time.

---

## 7. Configuration

### 7.1 Where each kind of setting lives

`config.h` already encodes the right distinction: Section 1 is "values you
should expect to change," Section 2 is "change these without changing the
physical part and the firmware breaks." Protocols map onto it directly.

| Kind | Example | Home |
|---|---|---|
| Protocol **selection** | `TELEMETRY_PROTOCOL` | `config.h` §1 — per-variant, must be visible before anything decides what to compile |
| Protocol **tunables** | RaceChrono's HDOP policy, advertised name, whether to send the time characteristic every epoch | `config.h` §1, Protocol subsection |
| Protocol **constants** | `RACEBOX_*` UUIDs and identity strings, and the `static_assert`s validating them | `g_proto_<name>.h` |

Every current `RACEBOX_*` entry is annotated "Compatibility requirement" — all
Section 2, all of it moves.

**A parallel `proto-<name>-config.h` was rejected.** The plug-in already gets
`g_proto_<name>.h`; a second file beside it holding seven defines is two files
where one suffices, and in this repo every file costs three copies plus a
`check_common.sh` entry. See §8.5.

### 7.2 This closes a real drift hole

The seven `RACEBOX_*` constants are byte-identical across all three variants
today — and they must stay that way, or app compatibility breaks. But they live
in three separate per-variant `config.h` files, and **`config.h` is not in the
common set** and cannot be, since `DEVICE_ID`, pins, and IMU ranges legitimately
differ per variant.

So nothing currently prevents them drifting. Moving them into
`g_proto_racebox.h`, which *is* an all-variant byte-identical file, puts them
under `check_common.sh` for the first time. This is a correctness gain, not
housekeeping — and it is the strongest argument for the move. The size argument
is not: RaceBox-specific content is roughly 50 lines out of 968, about 5%.

### 7.3 It also fixes validation that is already broken-in-waiting

`config.h`'s `uuid_format::isValid` asserts currently validate
`RACEBOX_SERVICE_UUID` unconditionally. On a RaceChrono build there are no such
UUIDs — those asserts would fail to compile, or force dead RaceBox constants to
be kept purely to satisfy them. Moving each protocol's asserts into its own
header makes validation self-contained: it fires only for the protocol actually
selected.

Same treatment for `BLE_MTU_BYTES >= 91`, which exists solely to carry an
88-byte RaceBox notify and is meaningless for RaceChrono's 20-byte frames. And
for `DEVICE_ID`'s range rule — the value stays in `config.h` (it is per-variant
and it is the device's identity regardless of protocol), but "exactly 10 digits,
first digit 0–3 so the value stays below 4,000,000,000" is a RaceBox *app*
constraint and moves with the rest.

### 7.4 Include direction

Getting this backwards creates a cycle, so it is stated explicitly:

```
config.h              includes <Arduino.h> only                        (leaf)
g_protocol.h          includes <stdint.h>, <stddef.h> only             (leaf)
g_proto_<name>.h      includes g_protocol.h only                       (leaf-ish)
g_proto_<name>.cpp    includes g_protocol.h, g_proto_<name>.h,
                      g_ubx_helpers.h   —   NOT config.h
g_telemetry.cpp       includes config.h + the protocol headers;
                      performs the #if TELEMETRY_PROTOCOL dispatch
g_ble.cpp             includes config.h + the active protocol header;
                      composes the advertised name from modelName + DEVICE_ID
```

`config.h` must **never** include a protocol header. Neither `g_protocol.h` nor
any encoder may include `config.h` — that is what keeps them host-compilable,
and it is why the descriptor carries `modelName` rather than a fully composed
device name: `DEVICE_ID` is per-variant config, so the concatenation happens in
`g_ble`, which already sees both.

**Small enabling change:** `g_ubx_helpers.h` includes `<Arduino.h>` but uses
only fixed-width integer types plus `<type_traits>`. Swapping that for
`<stdint.h>` / `<stddef.h>` would make the encoder translation units fully
host-compilable with **no Arduino shim at all**, matching the trim module's
"only `math.h`" precedent. Cheap, and it removes the one remaining piece of
scaffolding from the test harness.

---

## 8. Decisions and rejected alternatives

### 8.1 Compile-time selection — rejected: runtime switching

`#define TELEMETRY_PROTOCOL` in `config.h`. Unselected encoders cost zero flash,
no persistence is needed, and it fits the file's existing philosophy.

Runtime selection was rejected on a chicken-and-egg problem, not on effort:
no variant has a button, so the only switching channel is a BLE write from the
very client that changing protocol would disconnect. It would also need
flash-backed persistence, which the nRF52840 has no EEPROM for.

The descriptor is a struct of function pointers rather than a virtual class
specifically so this stays reversible: promoting to a runtime table later needs
no vtables and no heap.

### 8.2 Encoder and transport stay separate — rejected: one adapter owning both

The originating brief proposed a single adapter per protocol owning encoding
*and* advertising. Rejected because the two have different sharing scopes in
this repo: `g_telemetry.*` is byte-identical across all three variants, while
`g_ble.*` is shared only by the nRF pair and the ESP32 has its own. An adapter
owning both would inherit the narrower scope, and every protocol's field packing
would be written twice — once per BLE stack. Encoding is pure and portable;
transport is not.

### 8.3 Descriptor is pure data — rejected: virtual adapter classes

A C++ interface with virtual methods costs vtables and heap, and interacts
badly with compile-time selection. A plain struct of function pointers gives
the same extension property at zero runtime indirection, and — critically —
carries no BLE-stack types, so it crosses the Bluefruit/ESP32 boundary intact.

### 8.4 `BLEUart` stays on the RaceBox path — rejected: uniform channel builder

See §6.1. The purity gain does not justify re-implementing fragmentation and TX
backpressure on the one path that currently works.

### 8.5 No `proto-<name>-config.h` — rejected: parallel config files

The idea was sound in principle: protocol constants should not sit in a shared
config file. But the plug-in header already exists and is the natural home. A
parallel config file doubles the per-protocol file count, and each file here
means three copies plus a checker entry. The two-tier split in §7.1 achieves
the same separation using structure `config.h` already has.

### 8.6 Encoders do not include `config.h`

Direct application of `imu-trim-design.md` §5.9. Host-testability is the
mitigation for the risk that matters most here — silently wrong field packing —
so the encoder must be compilable and exercisable off-target.

### 8.7 RaceChrono is GPS-only; IMU over synthetic CAN is deferred

Its GPS API has no IMU channel. IMU would have to be synthesized as CAN PIDs
that each user decodes with hand-written equations in their Vehicle profile —
real work plus a per-user setup burden and documentation to support. Cleanly
separable, and RaceChrono users normally take IMU from the phone's own sensors,
so a GPS-only DIY device is the ordinary case rather than a degraded one.

### 8.8a Battery packing moves to the encoder — rejected: a pre-packed byte

`TelemetrySample` carries `batteryPercent` and `batteryCharging` as separate
fields, and the RaceBox encoder packs them:

```c
(charging ? 0x80 : 0x00) | (percent & 0x7F)
```

That packing currently lives in `batteryProtocolByte()` in `g_battery.cpp` —
non-protocol-specific code, whose own doc comment describes it as "the RaceBox
protocol battery byte for payload offset 67." It is protocol policy in a
hardware module, so it moves out and the function is deleted from all three
variants.

Carrying the pre-packed byte in the sample was the safer option and was
rejected: it puts a RaceBox-shaped field in the canonical model, which is
precisely what §4's Rule 2 exists to prevent, and RaceChrono has no battery
field to pack it for.

No `#if BATTERY_HAS_GAUGE` branch is needed in `buildSample()` — the ESP32 stub
already mirrors `batteryGetStatus()` and its `BatteryStatus` struct for exactly
this reason, so the accessor is uniform across variants.

This makes the packing new *encoder* policy, so it needs test coverage: the
capture supplies `0x64` and `0xE4` (both branches of the charging ternary) and
`test/synthetic.py` adds percent variation.

### 8.8 All three variants — rejected: nRF-only

Originally scoped to the nRF pair with the ESP32 frozen on RaceBox. Revised:
an ESP32 plus an M100-5883 is roughly $30 in parts and gains nothing from the
nRF hardware if the output is GPS-only RaceChrono, which makes it the cheapest
sensible RaceChrono DIY build and the one most worth supporting.

This also *simplifies* the design — `g_proto_racechrono.*` joins the
all-variant common set, so there is no asymmetry to encode and no
`#include`-inside-a-false-`#if` subtlety to comment.

---

## 9. File layout

| File | `check_common` group | Status |
|---|---|---|
| `g_protocol.h` | all-variant | **new** — sample + contract |
| `g_proto_racebox.h` / `.cpp` | all-variant | **new** — constants, descriptor, encoder |
| `g_proto_racechrono.h` / `.cpp` | all-variant | **new, later** |
| `g_telemetry.h` / `.cpp` | all-variant | slimmed to cadence, assembly, dispatch, stats |
| `g_ubx_helpers.h` | all-variant | `<Arduino.h>` → `<stdint.h>` (§7.4) |
| `g_ble.h` / `.cpp` (nRF) | nRF-only | inverted; two builders |
| `g_ble.h` / `.cpp` (ESP32) | ESP32-only | inverted; one builder |
| `config.h` × 3 | per-variant | `TELEMETRY_PROTOCOL`; protocol constants removed |

The all-variant common set grows from 9 files to 12 (14 once RaceChrono lands).
New files must be added to `check_common.sh` **in the same commit** that creates
them, or the duplication contract is briefly unenforced.

**Optional consolidation:** ESP32's `g_ble.h` is currently a strict subset of
the nRF one — it lacks `bleStop()`. If ESP32 gained a no-op `bleStop()`, the
header would be byte-identical everywhere and could join the all-variant group,
making the transport *interface* uniform across variants. Cost is one dead
function on a variant with no state machine to call it.

---

## 10. Implementation plan

Each phase ends with `check_common.sh` green and a working device.

**A — Golden capture. ✅ DONE (2026-09-09).** The baseline everything downstream
is measured against.

Firmware instrumentation turned out to be unnecessary. **Gnimu Monitor already
records raw RaceBox frames** to newline-delimited JSON, base64-encoded — a
byte-exact record captured *after* BLE fragmentation and reassembly, which is a
stronger reference than a firmware-side dump of what the firmware believed it
sent, and reusable as an end-to-end check in phase C. It also perturbs nothing.

`test/capture.py` reads those captures, validates each frame (sync, class,
declared length, UBX checksum), reconstructs the inputs `sendPacket()` must
have read, reports state coverage with named gaps, and emits `GC1,` vectors.
`test/synthetic.py` adds the states hardware cannot reach.

The one lossy field is `fixType`: it is clamped on the way out, so a captured 0
may have been a true 0, 1, 4 or 5. That is what the synthetic vectors cover.
`gnssFixOK` is unobservable when `fixType != 3` but cannot affect output there,
so it is immaterial.

| | |
|---|---|
| Captured vectors | 8,938 across two sessions |
| Synthetic vectors | 42 |
| Self-check | every captured vector re-encodes to its original bytes |
| Coverage | no gaps — all reachable states |

**The self-check is the load-bearing result.** `capture.py`'s `encode_packet()`
is a Python port of `sendPacket()`; every captured vector is re-encoded from
its reconstructed inputs and diffed against the real bytes. Passing on 8,938
real packets is what promotes that port from "a reading of the code" to a
reference implementation — and it is why generating the synthetic vectors with
the same function is not circular.

Captures and derived vectors are **gitignored**: they contain real GNSS fixes
at full resolution, and committing them would publish the precise coordinates
of wherever they were recorded. `test/synthetic.gc1` is fabricated and safe to
commit. If a captured fixture is ever needed in-repo, sanitize it — offset the
coordinates and re-encode, which keeps checksums and derived flags consistent.

**B — Extract sample and encoder. ✅ DONE (2026-09-09).** `g_protocol.h` and
`g_proto_racebox.*` created; `g_telemetry` slimmed to cadence, `buildSample()`,
dispatch and stats. Still calls the existing `bleSendPacket` through a small
adapter, so the transport is untouched.

`g_ubx_helpers.h` swapped `<Arduino.h>` for `<stdint.h>`/`<stddef.h>` — it only
ever needed fixed-width types, and that removes the last barrier to compiling
the encoder on a host. `batteryProtocolByte()` deleted from all three
`g_battery` modules per §8.8a; `buildSample()` reads `batteryGetStatus()`
instead, and the packing moved into the encoder.

*Gate met: the host harness compiles `g_proto_racebox.cpp` itself and
reproduces **all 8,980 golden vectors exactly** — 8,938 captured, 42
synthetic — with `-Wall -Wextra -Werror`.*

The harness found one defect on its first run, and it was in a **test vector**,
not the firmware: a synthetic "everything at maximum" case asked for battery
byte `0xFF`, which implies percent 127. No device can emit that (percent caps
at 100, so `0xE4` is the true maximum), and the encoder correctly clamped it.
`test/synthetic.py` now rejects any battery byte whose low 7 bits exceed 100,
so the invariant is enforced rather than remembered.

**Not yet verified: the Arduino build.** The harness proves the encoder is
correct and warning-clean under g++, but only compiling in the IDE proves it
builds for the targets.

**C — Invert `g_ble`. ✅ DONE (2026-09-09).**
`g_ble` now builds identity, service and characteristics from the active
`ProtocolDescriptor` and exposes `bleEmitFrame(channel, …)`, whose signature
matches `TelemetryEmit` exactly — so `g_telemetry` hands it straight to the
encoder with no adapter, and phase B's temporary `const_cast` is gone.

Two builders, as designed. The nRF path keeps `BLEUart` under
`TRANSPORT_NORDIC_UART`, so the fragmentation and backpressure behavior of the
working RaceBox path is untouched. The ESP32 path ignores `TransportKind`
entirely and builds discrete characteristics from the channel table — it has no
`BLEUart` and does no chunking, so both kinds are the same code there. Its
`createService()` handle count is now sized from `channelCount` rather than
left at the default, since an undersized value makes extra characteristics fail
to register silently.

Phase D's constant relocation came along, because it had to: the descriptor
needs the `RACEBOX_*` identity strings, and they cannot be read from `config.h`
without breaking the header's dependency-freedom. They and their UUID asserts
now live in `g_proto_racebox.h`, under `check_common.sh` for the first time.

**Deviation from §7:** `DEVICE_ID`'s format asserts stayed in `config.h`. They
are RaceBox app constraints and by rights belong with the protocol, but
`DEVICE_ID` is per-variant config and `g_proto_racebox.h` must not include
`config.h`. Validating the value where the value lives is the lesser
compromise. Same reasoning for `BLE_MTU_BYTES >= 91` on ESP32.

*Gate met on both stacks.* ESP32 and nRF52840 were each flashed, connected,
and captured from cold start. Both captures **round-trip exactly against phase
A's reference encoder** — the end-to-end proof, because it covers
`buildSample()`, the one link the host harness structurally cannot reach.

That exercises both transport builders: the ESP32 channel builder (`uuidFor()`,
`BLE2902` handling, the sized handle count, `ChannelWriteCallbacks`) and the
nRF `TRANSPORT_NORDIC_UART` path through `BLEUart`. The nRF run also confirmed
the descriptor-driven Device Information Service and both battery states, USB
plugged and unplugged.

All three variants were flashed and captured. Totals across five sessions:
**22,986 captured vectors, every one round-tripping exactly**, plus 42
synthetic — 23,028 in the harness, zero failures.

*Rate note, for the record:* epoch delivery is essentially complete on every
board wherever it is measurable (ESP32 1356/1356, nRF 651/651, OLED 2698/2699) — that is, over packets whose time is fully resolved,
the only window in which `iTOW` can be differenced meaningfully. Before time resolution the ESP32 delivered at 16.75 Hz, against 20.00 Hz on
the base nRF and 19.99 Hz on the OLED — so the shortfall is specific to the
ESP32 board, not to the shared code. Not
attributable to the refactor (both run the same shared encoder and sample code)
and not further diagnosable from app-side captures, which cannot distinguish a
lost epoch from one the receiver never emitted.

**D — Configuration. ✅ DONE (2026-09-09).** `TELEMETRY_PROTOCOL` now sits in
the Protocol slot Section 1's header comment always reserved, in all three
variants. The constants and their asserts had already moved during phase C,
because the descriptor needed them.

The selection is resolved in **one place**, `g_protocol_active.h`, which is the
only file that has to change when a protocol is added: write `g_proto_<name>.*`,
give it an id in `g_protocol.h`, add a branch. Neither `g_telemetry` nor
`g_ble` names a concrete protocol any more — both consume `ACTIVE_PROTOCOL`.

That header is deliberately separate from `g_protocol.h` because it includes
`config.h`, which `g_protocol.h` must never do or the encoders stop being
host-compilable. The harness includes the contract and the encoder but never
this file.

`PROTO_*` ids are numbered **from 1**: an undefined macro evaluates to 0 in a
preprocessor `#if`, so ids starting at 0 would let a build with the constants
somehow not visible silently compare `0 == 0` and select the first protocol.
Starting at 1 makes that trip the `#error` instead.

*Verified by compiling the real dispatch against stub configs:* a valid
selection resolves to the RaceBox descriptor, an undefined `TELEMETRY_PROTOCOL`
and an unimplemented id each fail with their intended message.

**E — Propagate. ✅ DONE (2026-09-09).** All shared files triplicated and
registered in `check_common.sh` (13 all-variant, 11 nRF-shared). Documentation
swept:

- `src/README.md`'s module inventory rebuilt — it still described
  `g_telemetry` as "packs GNSS+IMU into a RaceBox Data Message", and had never
  listed `g_imu_trim` either. Now names the four protocol files and says what
  each is for.
- Root README's repo layout gained `test/` and the two new `docs/` entries.
- The duplication note now warns about **both** ways to go unchecked: a new
  sketch folder missing from `VARIANTS`, and a new shared file missing from
  `COMMON_FILES` / `NRF_COMMON_FILES`. Only the first was mentioned before,
  and this refactor added four files that needed the second.
- `TELEMETRY_PROTOCOL` documented in the ESP32 and base-nRF config tables. The
  OLED README deliberately does not repeat it — that variant's table covers
  only what it adds, and it points at the base nRF README for the rest.

Nothing else was stale: the READMEs never tabled the `RACEBOX_*` constants, and
their claims about advertising a RaceBox identity and speaking the RaceBox Data
Message protocol all remain true.

**F — RaceChrono encoder.** All-variant, platform-neutral, validated on the host
harness against the mapping document before any radio is involved — gates 1 and
2 of §11.1. First task: establish whether the reference sketch's packing can be
isolated for gate 2.

**G — nRF GATT-channels builder** + bring-up against the real app — gate 3 of
§11.1, then gate 4 freezes the accepted encoder's golden vectors.

**H — ESP32 GATT-channels builder** + bring-up.

**B and C are deliberately separate.** B is a large mechanical move with an
offline test; C is the one step that touches a working radio. Collapsing them
would put the only risky change behind the same commit as several hundred lines
of code motion, and a regression would not bisect.

---

## 11. Verification

Two tests doing different jobs.

**Golden serial capture** (phase A) proves the refactor changed nothing. It
covers the whole chain — PVT → sample → encode — end to end, once.

**Host harness.** The encoders depend only on fixed-width integer types, so
`g++` on macOS can compile `g_proto_racebox.cpp` plus `g_ubx_helpers.cpp` and
diff output against the golden vectors. It does *not* cover the PVT → sample
copy, which needs the SparkFun struct — which is exactly why `buildSample()`
stays mechanically trivial and all judgment lives where the harness can see it.

The harness's real payoff is phase F: iterating on RaceChrono field layout
without a reflash cycle, against a protocol whose byte order and field order are
both inverted relative to RaceBox.

### 11.1 Phase F's evidence standard

Settled before any RaceChrono code exists (ARC-7, 2026-09-11), so that what
counts as "proven" is a decision rather than whatever turns out to be
convenient.

**Why RaceBox's instrument does not transfer as-is.** The golden vectors prove
*byte-identical to the previous implementation*. That is exactly right for a
refactor, and RaceChrono has no previous implementation — no device of ours
speaks it, so there is no traffic to capture. Hand-written expectations would
not be neutral either: RaceChrono's published protocol README and the protocol
author's own reference sketch already disagree in two places (byte 19, VDOP vs a
hard-coded `0xFF`; and the reference clamps negatives where the README is
silent — see `racechrono-ble-mapping.md`). Writing expectations means choosing
which source to believe.

**What does transfer.** Two things, and together they are most of the value:

- **The captured corpus, as input.** Its samples are raw u-blox values, so the
  RaceBox quirks deliberately preserved in §14 live in the RaceBox encoder and
  do not leak into anything built on the same samples.
- **Golden vectors, after acceptance.** Once a first RaceChrono encoder is
  accepted, its output becomes the "previous implementation". The weaker
  instruments below are needed exactly once.

**What the corpus covers** (measured over the 23,910 captured samples; no
positions inspected):

| RaceChrono case | covered by real data? |
|---|---|
| hourly sync-counter increment | yes — 5 real hour boundaries |
| signed `nano` | yes — 500 samples |
| negative altitude within the +500 m offset | yes — 3,094 samples |
| coarse altitude (> 2776.7 m) | only via pre-fix garbage values |
| no fix / 2D / 3D | yes — fix types 0, 2, 3 |
| coarse speed (> 327.67 km/h) | **no** — corpus maximum is 16 km/h (bench and walking) |
| the reference's clamp below −500 m | **no** |
| fix types 4 and 5 | **no** |
| `headVehValid = 1` | **no** — never set on this hardware (§14.1) |
| week rollover, day change | **no** |

Every **no** needs a committed synthetic vector, and so does the exact edge of
each dual-mode threshold.

**The standard: an encoder is accepted only when it passes all four gates.**

1. **Round-trip — phase F, host, continuous.** A decoder written *separately*
   from the encoder, from the protocol README, in Python alongside
   `capture.py`. Every frame produced from the corpus and the synthetic set is
   decoded and each field checked against its sample within the protocol's
   documented quantization. Catches layout, byte order (inverted relative to
   RaceBox), scaling and mode-switch errors, across real-world data.
   *Weakness, stated:* encoder and decoder share one reading of the spec, so a
   misreading passes both.

2. **Differential against the reference — phase F, host, once.** Isolate the
   reference sketch's packing, compile it on the host, feed it the same
   samples, compare bytes. A different author removes the shared-misreading
   weakness of gate 1. Every disagreement is explained in writing, the way §14
   records RaceBox findings.
   **Rule where README and reference disagree: follow the reference** — it is
   known to work with the app — and record the choice. The mapping document
   already applies this to byte 19.
   *Open:* whether the packing can be isolated from the sketch at all. Checking
   that is phase F's **first** task. If it cannot, record it, and gate 3 carries
   more of the weight.

3. **The app decides — phase G, hardware.** The RaceChrono app is the only
   authority on what the protocol *means* — the two written sources are proof
   of that. A bench session and a short drive; what the app records is compared
   with the device's own log within quantization. The run must span a real hour
   boundary so the sync counter is checked end to end rather than only in the
   encoder. Coarse speed cannot be exercised on a road, so for that case gates
   1 and 2 are the evidence, and this document says so rather than implying
   otherwise.

4. **Freeze — end of phase G.** Generate RaceChrono golden vectors from the
   accepted encoder, over the corpus and the synthetic set. From then on a
   change must be byte-identical, exactly as for RaceBox. Vectors derived from
   the captured corpus carry real positions and stay gitignored, as
   `vectors.gc1` does; synthetic vectors are committed.

**Not yet decided, deliberately.** The harness is hard-wired to
`raceboxEncode()` and will need generalising to run gates 1, 2 and 4 against a
second encoder; that is a phase F design question. And gates 1 and 2 start
*from* the sample, so the PVT → sample copy (`buildSample()`) stays covered only
by gate 3 until the deferred harness extension lands — noted in
`docs/code-review-remediation.md` (ROB-6 part 2).

---

## 12. What this does not touch

GNSS configuration and rate; the IMU pipeline, `ImuAxis` smoothing, and
`g_imu_trim`; battery sampling and the SoC curve; power gating and the state
machine; the LED and OLED modules; `imu_calibration` and the other diagnostic
sketches.

`bleIsConnected()` keeps its current meaning. *(Update 2026-09-11: the state
machine's idle cutoff now keys on `bleIsSubscribed()` - connected AND
notifications enabled - and LIGHT_SLEEP no longer exists; see
`code-review-remediation.md`.)* Both protocols in scope are BLE, so that
predicate serves either. A future serial or UDP protocol would need a
protocol-agnostic "is anyone listening" predicate; that is not designed here.

---

## 13. Open decisions

**`IMU_ENABLED`.** RaceChrono GPS-only needs no IMU at all, but `imuBegin()`
hard-halts in a `while(1)` if the chip is absent, so the minimal no-IMU ESP32
build — the cheapest RaceChrono device, and the reason §8.8 was revised — is
impossible today. A config flag gating `imuBegin`/`imuPoll` would enable it.
Recommended as its own phase *after* the protocol work, not folded into B.
Making IMU absence non-fatal at runtime instead was considered and rejected: it
turns a genuine wiring fault into a silent degradation on RaceBox builds, which
is what the halt exists to prevent.

**HDOP source for RaceChrono.** NAV-PVT carries `pDOP` only. Send `0xFF`
(invalid), substitute `pDOP`, or add a UBX-NAV-DOP poll. See the mapping
document §5. Decide during phase F.

**`ImuProtocolUnits` rename.** The type is named for RaceBox and its fields
(`gX`/`rX`) are confusing once shared. Cosmetic; touches `g_imu_sensor.h`, where the type
has lived since the IMU unification (2026-09-11), and its users. Deliberately not folded into phase B.

---

## 14. Findings from the phase A capture

Two things the capture and the synthetic vectors settled. Neither is fixed
here: a refactor that also fixes a bug cannot be verified, because every diff
then has two explanations. Both are recorded for their own commits later.

### 14.1 "Valid heading" appears to be permanently zero

RaceBox packet bit 5, "valid heading," is driven by `flags.bits.headVehValid` —
but `headVeh` is *fused vehicle heading*, reported only in sensor-fusion mode,
which a plain M10 without dead reckoning never enters. The value actually
transmitted at offset 52 is `headMot` (course over ground), which is unrelated.

**Measured: `headVehValid` was 0 in all 22,986 captured vectors** — five
sessions, all three variants, two GNSS module types, across cold boots, fix
acquisition and motion. Consistent with the bit being permanently
zero on this hardware. Not proof — a wider capture could still show it — but
the mechanism explains the observation, so the burden has shifted.

If it is permanently zero, bit 5 is dead weight in the packet, and a consumer
that trusts it is reading a field Gnimu never sets.

### 14.2 `fixType` 4 and 5 produce "no fix" with coordinates marked valid

`sendPacket()` derives two fields from `fixType` but reads it at different
points in the clamp:

```c
uint8_t safeFixType = (pvt->fixType == 2 || pvt->fixType == 3) ? pvt->fixType : 0;
writeLittleEndian(payload, 20, safeFixType);        // POST-clamp
...
if (pvt->fixType < 2)                                // PRE-clamp
  latLonFlags |= (1 << 0);                           // coordinates invalid
```

For a pre-clamp `fixType` of 4 (GNSS+DR) or 5 (time only), offset 20 reports
**0, "no fix"**, while offset 66 bit 0 stays **clear, "coordinates valid"** —
because 4 and 5 are not `< 2`. A consumer sees no fix and trustworthy
coordinates at the same time. `fixType` 1 does not have this problem, being
`< 2` and consistently flagged.

Reachability is the open part. 4 needs dead reckoning and is unreachable on an
M10. **5 (time only) is reachable in principle** — a receiver holding time
without enough satellites for position — and would ship whatever stale position
the receiver last reported, flagged valid. The capture cannot say whether it
occurred, because 5 is indistinguishable from 0 once clamped. That is exactly
why `test/synthetic.py` covers 1, 4 and 5 deliberately.

Whether this is worth changing is a separate question from whether phase B
preserves it. Phase B preserves it; the synthetic vectors pin the current
behavior so any later fix is a visible, intentional change to those vectors.
