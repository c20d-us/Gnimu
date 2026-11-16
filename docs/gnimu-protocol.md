# The Gnimu protocol

The Gnimu protocol is the open telemetry format for the Gnimu family of devices: one fixed-layout BLE frame, the same on every device. It is specified separately, in [Gnimu-protocol](https://github.com/c20d-us/Gnimu-protocol): `docs/frame.md` there defines every field, the BLE UUIDs and the rules. This page covers only what that spec leaves to each device: what Gnimu fills, and how.

Frame v1 is still in progress, so the layout can change until it's published.

## Building it

The Gnimu protocol is a [protocol profile](configuration.md#build-variants), so a build speaks either it or RaceBox, not both:

```bash
scripts/flash.sh nrf52840 --build --protocol gnimu
```

The device then advertises as `Gnimu` followed by its 10-digit device ID.

If a phone has connected to the same board running the other protocol, turn its Bluetooth off and on before connecting again. The board keeps its BLE address but presents a different set of services, and iOS can hold on to the old ones.

## What Gnimu sends

**One frame per GNSS epoch,** sent as soon as the receiver's solution arrives, so the frame rate is the navigation rate (20 Hz by default). A receiver without a fix still produces a solution every period, so frames keep coming before the first fix; the fix type and the GNSS valid bits show that there isn't one yet.

| Header field | Gnimu sends |
|---|---|
| Device time | ms since boot |
| Capabilities | `11` (GNSS, IMU, power) on the XIAO nRF52840; `3` (GNSS, IMU) on the ESP32, which has no battery |
| State | Always `2` (running): BLE only runs in the Running state |

**Which fields are valid when:**

| Fields | Valid when |
|---|---|
| Latitude, longitude, altitude, ground speed, speed accuracy | Fix type 2, 3 or 4, and the receiver's gnssFixOK |
| Heading of motion | Those, and a ground speed of at least 5 km/h. The receiver's own heading-valid flag only applies in sensor-fusion mode, which the M10 doesn't run, and below walking pace the heading of motion wanders. The heading value is in the frame either way. |
| Accuracies, PDOP, fix type, satellites, GNSS epoch age | Every frame |
| UTC timestamp, time accuracy | The receiver reports the date and time valid and fully resolved |
| IMU accelerations and rates | Whenever the IMU is running, including before the [self-calibration](imu-trim.md) has locked. The axis map always applies; the trim refines it by a few degrees once the car has been parked. |
| Supply voltage, battery charge, charging | XIAO nRF52840 only, from the first battery reading. Supply voltage is the battery voltage. |

The vehicle block is always empty.

**GNSS epoch age** is normally a millisecond or less, because each frame is sent in the same pass that parsed its solution.

## Schema

The Schema characteristic's value, on the XIAO nRF52840:

```json
{"v":1,"d":"Gnimu","r":20,"fill":{"gnss":16383,"imu":63,"power":7}}
```

`r` is `kGnssNavRateHz`. The ESP32 build leaves out `power`.

## Device info

The Device info characteristic carries the build and the settings in effect, the same information as the console's `V` command:

```json
{"v":1,"build":{"rev":"…","utc":"…","target":"nrf52840"},
 "cfg":{"hardware":"V2","gnss":"GPS+Galileo","nav_hz":20,"dyn_model":"automotive","min_elev":5,
 "imu_axes":"X=+x Y=+y Z=+z","accel_alpha":0.33,"gyro_alpha":0.33,"tx_dbm":-12,
 "batt_cutoff_v":3.30,"batt_critical_v":3.35,"batt_warn_v":3.40,"batt_full_v":4.10,"batt_charged_v":4.10}}
```

| `cfg` key | Setting |
|---|---|
| `hardware` | The [hardware profile](configuration.md#hardware-profiles) |
| `gnss` | The enabled constellations, `+`-separated |
| `nav_hz` | `kGnssNavRateHz` |
| `dyn_model` | The receiver's dynamic model |
| `min_elev` | `kGnssMinElevDeg`, in degrees |
| `imu_axes` | The IMU axis map: each vehicle axis, then the sensor axis that feeds it |
| `accel_alpha`, `gyro_alpha` | The [IMU smoothing](configuration.md#smoothing) |
| `tx_dbm` | `kBleTxPowerDbm` |
| `batt_cutoff_v`, `batt_critical_v`, `batt_warn_v`, `batt_full_v`, `batt_charged_v` | The [battery thresholds](configuration.md#battery), XIAO nRF52840 only |

Every value is fixed when the firmware is built. A client should ignore keys it doesn't recognise, since later versions may add some.

## Device Information Service

| Characteristic | Gnimu sends |
|---|---|
| Manufacturer | `kBleManufacturer` (`Gnimu`) |
| Model | The advertised-name prefix (`Gnimu`) |
| Serial number | The 10-digit device ID |
| Firmware revision | The build's version |
| Hardware revision | The board and hardware profile, such as `nrf52840, V2` |

## Commands

The spec's Control characteristic isn't defined yet, so a Gnimu-protocol build takes no commands over BLE. Re-trim, restart and System OFF work from the [serial console](console.md#commands).

## The reference encoder

The frame is built by the spec's own C++ reference encoder, copied unchanged into [`code/Gnimu/src/core/vendor/gnimu_frame/`](../code/Gnimu/src/core/vendor/gnimu_frame/). The README there names the spec commit it came from. Gnimu's part, converting its units and deciding the valid bits, is [`app/protocol/GnimuFrame.h`](../code/Gnimu/src/app/protocol/GnimuFrame.h), and the host tests check it byte for byte against the spec's `gnimu-parked` test vector.
