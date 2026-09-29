# RaceChrono DIY BLE — protocol reference and NAV-PVT mapping

Reference material for a possible future RaceChrono DIY BLE adapter. **Nothing in
this document is implemented.** It exists so the research doesn't have to be
redone, and so the field mapping can be reviewed independently of any code.

Compiled 2026-09-07 from the sources listed at the bottom. Field layouts were
cross-checked between RaceChrono's published protocol README and the reference
implementation's actual source, which disagree in two places — both noted below.

**How an encoder built from this document gets accepted** — four gates, and the
rule for where the README and the reference disagree — is settled in
`multiprotocol-design.md` §11.1. Read that before writing the encoder.

---

## 1. Why this is a different shape from RaceBox

RaceBox is a single 88-byte UBX-framed packet carrying GNSS *and* IMU, streamed
over a Nordic UART service. RaceChrono DIY is a set of small, purpose-specific
GATT characteristics with **no IMU channel at all**.

Practical consequences:

- **A RaceChrono adapter is GPS-only unless IMU is synthesized as CAN PIDs.**
  RaceChrono's docs state the CAN-Bus feature accepts "CAN-Bus data, or pretty
  much any sensor data," but decoding happens app-side: the user adds CAN-Bus
  channels to their Vehicle profile and writes equations to unpack each PID.
  Gnimu cannot ship that profile. This is a real per-user setup burden and
  should be treated as a separate deliverable from the GPS adapter.
- **No device identity requirements.** No Device Information Service strings, no
  model-name matching, no serial-number range rule. The device name is free-form
  (the reference uses `"RC DIY #A5B2"`, from the low MAC bytes). Contrast with
  RaceBox, where the DIS strings and the `DEVICE_ID < 4000000000` rule are
  compatibility requirements.
- **No fragmentation concerns.** The largest frame is 20 bytes, which fits the
  default 23-byte ATT MTU. None of the MTU-raising machinery the RaceBox path
  needs applies here.
- **This is a published, sanctioned API** for third-party DIY devices — a
  categorically different posture from emulating a commercial product.

---

## 2. Transport

**Service UUID:** `0x1FF8` — full form
`00001ff8-0000-1000-8000-00805f9b34fb` (Bluetooth SIG base UUID, so the 16-bit
short form is what goes on the wire and it costs 4 advertising bytes, not 18).

| Characteristic | UUID | Properties | Purpose |
|---|---|---|---|
| CAN-Bus Main | `0x0001` | READ, NOTIFY | CAN frames, or arbitrary sensor data as synthetic PIDs |
| CAN-Bus Filter | `0x0002` | WRITE (with response) | App tells the device which PIDs to send, and how often |
| GPS Main | `0x0003` | READ, NOTIFY | 20-byte position/velocity frame |
| GPS Time | `0x0004` | READ, NOTIFY | 3-byte date/hour frame |
| Monitor Config | `0x0005` | INDICATE, WRITE | App→device display equations (RaceChrono 7.4+) |
| Monitor Values | `0x0006` | WRITE_WITHOUT_RESPONSE | App→device computed values (RaceChrono 7.4+) |

The Monitor characteristics are the reverse direction (feeding a remote display
on the device) and are not relevant to a telemetry source.

> **Gotcha in the reference source.** It declares
> `BLEService(0x00000001000000fd8933990d6f411ff8)` — a 32-hex-digit literal.
> Bluefruit has no 128-bit-integer constructor, so this implicitly narrows to
> `uint16_t`, keeping the low 16 bits: `0x1FF8`. It works *by truncation*.
> Write `BLEService(0x1FF8)` explicitly.

**Endianness: big-endian throughout, unsigned unless noted** — the opposite of
RaceBox's little-endian UBX payload. The one documented exception is the CAN
Main packet ID, which is little-endian.

---

## 3. GPS Main characteristic (`0x0003`) — 20 bytes

| Bytes | Field | Encoding |
|---|---|---|
| 0 | sync + time high | `(sync & 0x7) << 5 \| (time >> 16) & 0x1F` |
| 1–2 | time low 16 | see `timeSinceHourStart` below |
| 3 | fix + satellites | `(fixQuality & 0x3) << 6 \| (satellites & 0x3F)`; `0x3F` sats = invalid |
| 4–7 | **latitude** | signed 2's complement, degrees × 10,000,000; `0x7FFFFFFF` = invalid |
| 8–11 | **longitude** | signed 2's complement, degrees × 10,000,000; `0x7FFFFFFF` = invalid |
| 12–13 | altitude | dual-mode, see below; `0xFFFF` = invalid |
| 14–15 | speed | dual-mode, see below; `0xFFFF` = invalid |
| 16–17 | bearing | degrees × 100; `0xFFFF` = invalid |
| 18 | HDOP | dop × 10; `0xFF` = invalid |
| 19 | VDOP | dop × 10; `0xFF` = invalid |

> **Two easy ways to ship a plausible-but-wrong packet.** RaceChrono is
> big-endian where RaceBox is little-endian; and RaceChrono puts **latitude
> first** (bytes 4–7) where the RaceBox payload puts **longitude first**
> (offset 24). A transposed lat/lon produces coordinates that look like valid
> numbers.

**Doc vs. reference disagreement — byte 19.** The protocol README documents
byte 19 as VDOP. The reference implementation writes `0xFF` with the comment
`// Unimplemented`. Treat VDOP as optional; `0xFF` is a known-accepted value.

### Dual-mode altitude and speed

Both fields switch to a coarser encoding once the fine encoding overflows the
15 bits it is masked to. Bit 15 flags which mode is in use.

```
altitude_m > 2776.7   ->  ((round(m + 500))      & 0x7FFF) | 0x8000   coarse, 1 m
otherwise             ->  ((round((m + 500) * 10)) & 0x7FFF)          fine,   0.1 m

speed_kmh > 327.67    ->  ((round(kmh * 10))     & 0x7FFF) | 0x8000   coarse, 0.1 km/h
otherwise             ->  ((round(kmh * 100))    & 0x7FFF)            fine,   0.01 km/h
```

Switching any later than these thresholds leaves in-between values encoding in
fine mode, where they wrap and decode as a plausible but wrong number. The
reference clamps negatives to 0 before masking.

### Time field

```
timeSinceHourStart = (minute * 30000) + (second * 500) + (millisecond / 2)
```

21 bits, 2 ms resolution. Maximum value 1,799,999 — fits comfortably.

---

## 4. GPS Time characteristic (`0x0004`) — 3 bytes

```
dateAndHour = (year - 2000) * 8928 + (month - 1) * 744 + (day - 1) * 24 + hour

byte 0 = (sync & 0x7) << 5 | (dateAndHour >> 16) & 0x1F
byte 1 = dateAndHour >> 8
byte 2 = dateAndHour
```

**Sync bits** are a 3-bit counter, incremented **only when `dateAndHour`
changes** (i.e. hourly). Both GPS characteristics must carry the same sync value
so the app can pair a position frame with the correct hour. The reference sends
both characteristics on every fix; sending the time frame less often is an
untested optimization.

The 21-bit `dateAndHour` field overflows in year 2235. Not a concern.

---

## 5. Mapping from UBX-NAV-PVT

Gnimu's canonical sample is fed from `UBX_NAV_PVT_data_t`. Every GPS field maps
directly except the DOP pair.

| RaceChrono field | NAV-PVT source | Conversion |
|---|---|---|
| `timeSinceHourStart` | `min`, `sec`, `nano` | `min*30000 + sec*500 + ms/2` — see nano note |
| `dateAndHour` | `year`, `month`, `day`, `hour` | `year` is a **full** year; subtract 2000 |
| fix quality | `fixType`, `flags.gnssFixOK` | **not** `fixType` — see mapping below |
| satellites | `numSV` | direct, clamp to 0x3F |
| latitude | `lat` | **direct** — u-blox is already deg × 10⁷ |
| longitude | `lon` | **direct** — u-blox is already deg × 10⁷ |
| altitude | `hMSL` (mm) | fine: `(hMSL + 500000) / 100` |
| speed | `gSpeed` (mm/s) | fine: `gSpeed * 36 / 100`; coarse above 91,019 mm/s |
| bearing | `headMot` (deg × 10⁻⁵) | `headMot / 1000` |
| HDOP | — | **no source** — see below |
| VDOP | — | no source; send `0xFF` |

### `nano` is signed

`nano` carries the sub-second correction and ranges roughly ±0.5 s. A negative
value means the true instant is *before* the reported `sec`. Dividing a negative
`nano` straight into milliseconds yields a negative offset and corrupts the time
field. Normalize by borrowing from `sec` (and `min`) before packing.

### Fix quality is NMEA GGA semantics, not u-blox

The 2-bit field follows GGA fix quality (`0` = invalid, `1` = GPS fix,
`2` = DGPS), **not** u-blox `fixType` (`0` = none, `2` = 2D, `3` = 3D). A
straight copy would report a 3D fix as DGPS.

A reasonable mapping — and note this is protocol *policy*, so it belongs in the
adapter, not the canonical sample:

```
fixQuality = (gnssFixOK && fixType >= 2) ? 1 : 0
```

with `2` reserved for a differential/RTK solution if `flags.carrSoln` ever
indicates one. This is deliberately a different rule from the RaceBox encoder's
"valid fix" bit, which requires `fixType == 3 && gnssFixOK`.

### HDOP has no honest source

Gnimu polls **NAV-PVT**, which carries `pDOP` only. There is no hDOP in that
message. Three options, in the order I'd try them:

1. **Send `0xFF` (invalid).** Matches how the reference treats VDOP. RaceChrono
   does not require the field.
2. **Substitute `pDOP`** (rescale ×100 → ×10). PDOP is always ≥ HDOP, so the
   value is pessimistic rather than wrong-shaped — but it is mislabelled.
3. **Add a UBX-NAV-DOP poll** for the real hDOP. Correct, but adds a second
   message to the receiver's output budget at 20 Hz, which is exactly the kind
   of thing this firmware is careful about.

---

## 6. CAN-Bus characteristics — for a future IMU channel

Only relevant if IMU data is synthesized as PIDs. Documented here so the
research is complete.

**CAN Main (`0x0001`), notify:**

| Bytes | Field |
|---|---|
| 0–3 | Packet ID, 32-bit **little-endian** (documented exception to big-endian) |
| 4–19 | Payload, 1–16 bytes variable length |

**CAN Filter (`0x0002`), write with response** — the app writes these:

| Command | Bytes | Meaning |
|---|---|---|
| `0` Deny all | `[0]` | Stop sending |
| `1` Allow all | `[1][interval:2]` | Promiscuous, with notify interval in ms |
| `2` Allow one PID | `[2][interval:2][pid:4]` | Per-PID subscription |

An adapter must at minimum accept these writes without erroring. Reference
implementations commonly **ignore the requested interval** and self-pace
instead, sending every Nth message per PID.

---

## 7. Update rate

| Source | Observation |
|---|---|
| Reference sketch | GPS configured at 5 Hz |
| Community reports | ~20 Hz CAN-only; ~10 Hz with GPS added; ~30 Hz stable when characteristics are sent together; degradation by 40 Hz |
| RaceChrono guidance | Prefers rates on the 1 / 5 / 10 / 20 / 30 / 40 / 50 / 100 Hz ladder |

Gnimu at 20 Hz sending two notifies per epoch (20 B + 3 B) sits inside the
reported-stable envelope.

**Note that Gnimu's 20 Hz GPS+Galileo configuration lands on RaceChrono's
preferred ladder, and the 25 Hz single-constellation configuration does not.**
If a RaceChrono adapter is built, 20 Hz is the configuration to recommend.

Reported best practice is to send all characteristics together with no
programmed delay between them — GPS Main first, then GPS Time.

---

## 8. Sources

- [aollin/racechrono-ble-diy-device — protocol README](https://github.com/aollin/racechrono-ble-diy-device/blob/master/README.md)
- [aollin — canbus-gps-device reference sketch](https://github.com/aollin/racechrono-ble-diy-device/blob/master/examples/canbus-gps-device/main/main.ino) (Bluefruit on nRF52840 — the same BLE stack Gnimu's nRF variants use)
- [RaceChrono — Tutorial: DIY devices](https://racechrono.com/article/2572)
- [timurrrr/RaceChronoDiyBleDevice](https://github.com/timurrrr/RaceChronoDiyBleDevice) — CAN-only, no GPS

Protocols may be revised. Re-verify against these sources before implementing.
