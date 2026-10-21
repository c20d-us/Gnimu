# Gnimu telemetry protocol — design record

The decisions made so far for Gnimu's own BLE telemetry protocol, and why. **Nothing in this document is implemented**, and it is not the protocol specification: the specification gets written when the protocol is built, from the decisions here. Recorded 2026-10-20.

---

## Goal

An open, documented protocol good enough that telemetry apps choose to support it: Gnimu Monitor first, and apps such as AutoXDL if they decide to. The alternatives a client developer faces today are a pseudo-public protocol (RaceBox) and a convoluted one (RaceChrono DIY, where anything beyond GNSS has to be decoded by user-written equations). This one should be easier to implement than either, and carry more.

Devices that speak it advertise with the plain `Gnimu` name prefix (`Gnimu-RB` is RaceBox, `Gnimu-RC` RaceChrono).

---

## Decisions

### The specification is the product

- A standalone document with its own version number and rules for what a change may break, apart from the firmware.
- An open license on the document and on reference code, so no one has to ask permission to implement it.
- Every record carries its type and length, so a client skips what it doesn't understand: old clients keep working with new devices.
- Byte-exact test vectors and a small reference decoder ship with the spec.

### IMU is its own stream

IMU data streams at the sensor's rate (100 Hz today), batched several samples per notification, independent of GNSS. This is how professional loggers (AiM, MoTeC) and CAN work; consumer BLE devices (RaceBox, VBOX Sport) instead bundle the latest IMU values into each GNSS-rate packet. The client aligns the streams by timestamp, which is why the timebase below matters.

Expected load is about 2.5 kB/s and 35 notifications a second (IMU ~1.3 kB/s, GNSS at 25 Hz ~1.2 kB/s), the same order as the RaceBox build sends today. Phones and both boards handle that with margin; a client needs an MTU above the 23-byte default, as RaceBox's 88-byte packet already does. Batch size trades latency against notification count, and can be left to the client.

### Timebase: device clock on everything, with a published mapping

- Every sample carries the device's monotonic clock.
- GNSS records carry their GNSS time *and* the device-clock time they arrived at.
- The device periodically publishes its current mapping from device clock to GNSS time (offset, rate and a quality figure).

A simple client applies the published mapping and has absolute time with no math. A careful client ignores it and fits its own from the raw pairs. Data is timestamped and ordered from power-on, before any fix. The spec must signal a device-clock reset (a reboot).

Rejected: GNSS time only (nothing to stamp before a fix, timestamps that step as the mapping is corrected, and the device's latency estimate baked in for good), and device clock only (every client must fit clocks before it has absolute time).

Without the receiver's timepulse (PPS) wired, the mapping is only as good as the UART delivery jitter, a few milliseconds. A board with PPS could advertise better time quality through the same mapping record; the protocol must not depend on it.

### BLE layout: inside the Gnimu service

The protocol lives in the Gnimu service (`20D21CDE-F59E-41F1-9BCA-5BDE69876C8B`), which every build already carries:

| Characteristic | Properties | Role |
|---|---|---|
| Data (new) | notify | All streams, as typed, length-prefixed records |
| Command | write | Gnimu device commands today; the protocol's control channel |
| Config (`689E54CD-…`) | read | The human-readable configuration report, as now |

One data characteristic keeps setup to a single subscription, delivers every stream in one ordered sequence (which helps alignment), and adds record types without new UUIDs. Streams are switched on and off through the control channel rather than by subscription.

### Self-description

- **In-stream device information.** A device-info record goes out on the data characteristic when a client subscribes and on request: protocol version, firmware version, hardware profile, streams and rates. A capture of the data characteristic therefore describes itself, and a reboot shows up in order with the data.
- **Core records have fixed layouts.** GNSS, IMU, clock mapping and status are defined field by field in the spec.
- **One descriptor-based record type is reserved** for additions: each such channel announces its name, units, scale and rate, so a client can show channels it has never seen.

### CAN, if it ever comes

A long-shot hardware idea, recorded so the protocol doesn't rule it out:

- **Raw frames:** a fixed record type (timestamp, ID, length, data). The client decodes with a DBC file or equations; the device needs to know nothing about the car.
- **Decoded channels:** the descriptor-based record type, from per-car decoding rules on the device.
- **Filtering:** a busy bus carries far more than BLE can forward, so the control channel needs "these IDs, at most this often" commands.
- Hardware: the ESP32 has a CAN controller built in (TWAI) and needs only a transceiver; the nRF52840 would need an external controller such as an MCP2515.

---

## What the firmware must provide

These shape the generalised BLE transport and protocol interface built for RaceChrono. With them in place, adding this protocol later is purely new code.

1. **IMU samples as they're produced**, each with its device-clock timestamp, not just the latest sample at epoch time.
2. **Client writes routed by characteristic**, so the control channel, RaceChrono's CAN filter and the device commands each arrive separately.
3. **Sends on any characteristic at any time**, driven by GNSS epochs, IMU samples and a periodic tick.
4. **More than one service per profile**: RaceChrono exposes `0x1FF8` beside the Gnimu service; this protocol adds its data characteristic to the Gnimu service itself.
5. **Subscription events** passed to the protocol, so device information can be sent when a client subscribes.
6. **The negotiated MTU** passed to the protocol, so it can pack as many records as fit into each notification.

---

## Still open

To be settled when the protocol is built. None of these affect the transport work.

- Framing: the record header layout, a sequence number for spotting lost notifications, and how records are packed into a notification.
- The control model: commands, stream and rate selection, the CAN filter.
- The field layout of each core record.
- Versioning rules, where the specification lives, and its license.
