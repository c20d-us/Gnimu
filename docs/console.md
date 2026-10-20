# Console and diagnostics

The firmware talks over the USB serial port at **115200 baud**. Open the Arduino IDE's Serial Monitor, or any serial terminal, and you get three things:

- a **boot log** that reports every start-up step;
- an on-demand **diagnostics report**;
- an optional **1Hz status line**.

Once it's running, the firmware prints nothing unless you ask or something goes wrong. The console is never needed for normal use; it's there for building, bench testing, and troubleshooting.

---

## Commands

Type a command and press **Enter**. Set the terminal to send a line ending (CR, LF, or both all work). Commands are single upper-case letters.

| Command | What it does |
|---|---|
| `D` | Prints the [diagnostics report](#the-diagnostics-report). |
| `C` | Clears the statistics counters, so the next report covers only what happens from now on. |
| `V` | Prints [what the device is](#what-the-device-is): the build line, the device ID, and the settings the firmware was built with. A BLE client can [read the same text](device-commands.md#reading-the-configuration). |
| `L` | Turns the [status line](#the-status-line) on or off. Left on, it stops by itself after 3600 lines (one hour). |
| `T` | Drops the locked [IMU trim](imu-trim.md) and measures a new one at the next stop. Does nothing if no trim has locked yet. |
| `R` | Restarts the device. The next boot's `Reset:` line says `Restart requested`. |
| `O` | Puts the device into System OFF, on battery boards. Cycle the switch or USB to start it again. Other boards say it isn't available. |
| a number | Prints exactly that many status lines, 1 to 3600, then stops. `0` stops the status line. |

Anything else is rejected whole and reported, so a stray keypress does nothing:

```
Console: unrecognised input 'x'
Console: count must be 0-3600
```

The firmware prints the command list at the end of the boot log as a reminder.

---

## The boot log

Every start-up step reports what it did. Lines that check something end in ✅ when it passed and ❌ when it failed; a GNSS setting that had to be written shows ⏬, and one that could be set but not saved shows ⚠️.

The log runs in this order:

1. **Identity and start.** The `BUILD` line, the device ID and the `Config:` lines (the same lines `V` prints), then `Gnimu starting`, the reason for the last reset, and the watchdog.
2. **Power** (XIAO nRF52840 only). The switch and USB state, and a first battery reading.
3. **GNSS.** Detecting the receiver, then one line for every receiver setting the firmware checks.
4. **IMU and BLE.** The IMU start-up, then BLE, which is started last and ends with advertising.
5. **Jobs.** One line per job with how often it runs and its time budget, or `disabled`.
6. **State and commands.** The state the firmware ended up in, and the command list.

Here is a complete boot of each board. Both GNSS receivers had been configured on an earlier boot, so every setting shows ✅.

<details>
<summary>ESP32, built and flashed with the scripts</summary>

```
BUILD,version=2.0.0-rc1,revision=7f7efe5f4d6a,dirty=0,utc=2026-10-11T17:27:43Z,target=esp32
Device ID: 0301821570
Config: board esp32, hardware Reference
Config: GNSS GPS+Galileo, 20 Hz, automotive, min elevation 5°
Config: IMU axes X=+x Y=+y Z=+z, smoothing accel 0.33 gyro 0.33
Config: BLE RaceBox, name "Gnimu-RB 0301821570", TX power -12 dBm
Gnimu starting
Reset: power-on
Watchdog: 5000 ms ✅
Clock started
GNSS: trying 115200 baud
GNSS: detected at 115200 baud ✅
GNSS: NAV-PVT output paused for configuration ✅
GNSS: dynamic model=4 ✅
GNSS: UBX output enable=1 ✅
GNSS: NMEA output enable=0 ✅
GNSS: NMEA DTM rate=0 ✅
GNSS: NMEA GBS rate=0 ✅
GNSS: NMEA GGA rate=0 ✅
GNSS: NMEA GLL rate=0 ✅
GNSS: NMEA GNS rate=0 ✅
GNSS: NMEA GRS rate=0 ✅
GNSS: NMEA GSA rate=0 ✅
GNSS: NMEA GST rate=0 ✅
GNSS: NMEA GSV rate=0 ✅
GNSS: NMEA RLM rate=0 ✅
GNSS: NMEA RMC rate=0 ✅
GNSS: NMEA VLW rate=0 ✅
GNSS: NMEA VTG rate=0 ✅
GNSS: NMEA ZDA rate=0 ✅
GNSS: minimum SV elevation=5 ✅
GNSS: BeiDou B1I signal=0 ✅
GNSS: BeiDou B1C signal=0 ✅
GNSS: GPS L1C/A signal=1 ✅
GNSS: Galileo E1 signal=1 ✅
GNSS: GLONASS L1 signal=0 ✅
GNSS: GPS enable=1 ✅
GNSS: Galileo enable=1 ✅
GNSS: BeiDou enable=0 ✅
GNSS: GLONASS enable=0 ✅
GNSS: QZSS enable=0 ✅
GNSS: SBAS enable=0 ✅
GNSS: setting Nav frequency to 20Hz
GNSS: nav ratio=1 ✅
GNSS: nav rate=50 ✅
GNSS: NAV-PVT output rate=1 ✅
GNSS: Configuring PVT callback and auto PVT output
GNSS: PVT callback and auto PVT output enabled ✅
IMU: MPU-6050 detected ✅
IMU: configured, 4g 500deg/s ✅
BLE: TX power -12 dBm (advertising and connected) ✅
BLE: advertising as "Gnimu-RB 0301821570" ✅
Clock: crystal ✅
Job gnss: every 1000 us, budget 4000 us
Job imu: every 10000 us, budget 1000 us
Job battery: disabled
Job power: disabled
Job ble: every 10000 us, budget 2000 us
Job led: every 25000 us, budget 500 us
Job console: every 10000 us, budget 2000 us
Running.
Commands end with Enter: D = report, C = clear statistics counters, V = version, T = re-trim IMU, R = restart.
L = 1s status on/off (up to 3600 lines), N = N status reports (0-3600).
```

</details>

<details>
<summary>XIAO nRF52840, switched on while on USB, built and flashed with the scripts</summary>

```
BUILD,version=2.0.0-rc1,revision=7f7efe5f4d6a,dirty=0,utc=2026-10-11T17:30:08Z,target=nrf52840
Device ID: 1120954722
Config: board nrf52840, hardware V2
Config: GNSS GPS+Galileo, 20 Hz, automotive, min elevation 5°
Config: IMU axes X=+x Y=+y Z=+z, smoothing accel 0.33 gyro 0.33
Config: BLE RaceBox, name "Gnimu-RB 1120954722", TX power -12 dBm
Config: battery cutoff 3.30V, critical 3.35V, warning 3.40V, 100% at 4.10V, charged at 4.10V
Gnimu starting
Reset: software (Power: switch on)
Watchdog: 5000 ms ✅
Power: GNSS rail, IMU and LED held off ✅
Clock started
Battery: 4.17V 100% ⚡ ✅
Power: switch on, USB present
Power: GNSS rail on
GNSS: trying 115200 baud
GNSS: detected at 115200 baud ✅
GNSS: NAV-PVT output paused for configuration ✅
GNSS: dynamic model=4 ✅
GNSS: UBX output enable=1 ✅
GNSS: NMEA output enable=0 ✅
GNSS: NMEA DTM rate=0 ✅
GNSS: NMEA GBS rate=0 ✅
GNSS: NMEA GGA rate=0 ✅
GNSS: NMEA GLL rate=0 ✅
GNSS: NMEA GNS rate=0 ✅
GNSS: NMEA GRS rate=0 ✅
GNSS: NMEA GSA rate=0 ✅
GNSS: NMEA GST rate=0 ✅
GNSS: NMEA GSV rate=0 ✅
GNSS: NMEA RLM rate=0 ✅
GNSS: NMEA RMC rate=0 ✅
GNSS: NMEA VLW rate=0 ✅
GNSS: NMEA VTG rate=0 ✅
GNSS: NMEA ZDA rate=0 ✅
GNSS: minimum SV elevation=5 ✅
GNSS: BeiDou B1I signal=0 ✅
GNSS: BeiDou B1C signal=0 ✅
GNSS: GPS L1C/A signal=1 ✅
GNSS: Galileo E1 signal=1 ✅
GNSS: GLONASS L1 signal=0 ✅
GNSS: GPS enable=1 ✅
GNSS: Galileo enable=1 ✅
GNSS: BeiDou enable=0 ✅
GNSS: GLONASS enable=0 ✅
GNSS: QZSS enable=0 ✅
GNSS: SBAS enable=0 ✅
GNSS: setting Nav frequency to 20Hz
GNSS: nav ratio=1 ✅
GNSS: nav rate=50 ✅
GNSS: NAV-PVT output rate=1 ✅
GNSS: Configuring PVT callback and auto PVT output
GNSS: PVT callback and auto PVT output enabled ✅
IMU: LSM6DS3TR-C detected ✅
IMU: configured, 4g 500deg/s ✅
BLE: TX power -12 dBm ✅
BLE: advertising as "Gnimu-RB 1120954722" ✅
Clock: crystal ✅
Job gnss: every 1000 us, budget 4000 us
Job imu: every 10000 us, budget 1000 us
Job battery: every 10000 us, budget 500 us
Job power: every 50000 us, budget 1000 us
Job ble: every 10000 us, budget 2000 us
Job led: every 25000 us, budget 500 us
Job console: every 10000 us, budget 2000 us
Running.
Commands end with Enter: D = report, C = clear statistics counters, V = version, T = re-trim IMU, R = restart, O = System OFF.
L = 1s status on/off (up to 3600 lines), N = N status reports (0-3600).
```

</details>

`Reset:` says why the firmware started: `power-on`, `watchdog`, `software`, `reset pin`, and so on. When the firmware restarted itself, the reason follows in brackets, for example `Reset: software (GNSS: epochs stopped)`.

The `BUILD` line identifies the firmware. See [the build stamp](building.md#the-build-stamp) for why an Arduino IDE build shows placeholders in it.

The **device ID** is a 10-digit number derived from the chip's own hardware ID. It is the same on every boot and different on every board, and it is the number in the BLE name the device advertises: `Gnimu-RB <device ID>`.

### GNSS settings

The receiver keeps its settings in its own flash, so the firmware checks each one at boot and only writes what differs. On a receiver the firmware has configured before, every line shows ✅ and the boot is quick. On a new or factory-reset receiver, the first boot shows a run of ⏬ as the settings are written and saved, and later boots are back to ✅.

### When a boot stops early

If start-up can't finish, the log says why and the firmware stays in a fault state with only the console and the status LED running:

| Message | What to check |
|---|---|
| `GNSS not detected at any standard baud rate. Check wiring. Nothing else will start.` | The receiver's power and its TX/RX wiring. |
| `GNSS settings could not be applied. Power-cycle to retry. Everything else stopped.` | Power-cycle. If it repeats, try [`tools/gnss_reset`](../tools/gnss_reset/gnss_reset.ino). |
| `GNSS detected but sending no data. Check its power supply. Everything else stopped.` | The receiver's supply voltage under load. |
| `BLE failed to start. Power-cycle to retry. Everything else stopped.` | Power-cycle. |

The firmware does not retry by itself in a fault state; power-cycle the device, or send `R`, once the cause is fixed. The commands still work, so `D` and `V` are available while you investigate.

If the receiver was working and then goes quiet for two seconds, the firmware restarts itself instead, which runs the whole start-up again.

---

## What the device is

`V` prints what the device is and how its firmware was built. The same lines open the boot log.

```
BUILD,version=2.0.0-rc1,revision=7f7efe5f4d6a,dirty=0,utc=2026-10-11T17:30:08Z,target=nrf52840
Device ID: 1120954722
Config: board nrf52840, hardware V2
Config: GNSS GPS+Galileo, 20 Hz, automotive, min elevation 5°
Config: IMU axes X=+x Y=+y Z=+z, smoothing accel 0.33 gyro 0.33
Config: BLE RaceBox, name "Gnimu-RB 1120954722", TX power -12 dBm
Config: battery cutoff 3.30V, critical 3.35V, warning 3.40V, 100% at 4.10V, charged at 4.10V
```

The `BUILD` line and the device ID are described under [the boot log](#the-boot-log). The `Config:` lines are the settings from [`Config.h`](configuration.md) as they were compiled into this firmware. They never change while the device runs; they are there so that a device can be asked what it was built with, instead of you having to remember.

| Line | What it shows |
|---|---|
| `Config: board` | The board the firmware was built for, and the [hardware profile](configuration.md#hardware-profiles) it was built with: the pins and IMU mounting it expects. |
| `Config: GNSS` | The constellations in use, the navigation rate, the receiver's dynamic model, and the minimum satellite elevation. |
| `Config: IMU` | The axis mapping and the smoothing. `X=+x` means the vehicle's X axis is the sensor's x axis, unchanged; `X=-x` means it is reversed. See [mounting orientation](configuration.md#mounting-orientation). |
| `Config: BLE` | The protocol, the advertised BLE name and the transmit power. |
| `Config: battery` | Battery boards only. The cutoff, critical and warning voltages, the voltage shown as 100%, and the voltage at which a finished charge counts as full. |

If the data looks wrong, check these first. An axis that reads backwards, a rate that isn't what you expected, or a device that won't power its GNSS are all settings you can see here.

---

## The diagnostics report

`D` prints a snapshot of everything the firmware is tracking. This report is from a XIAO nRF52840 on USB, about three minutes after start-up, with no app connected:

```
==========================================================================
| job     |   runs | skipped | overruns | max μs | mean μs | max late μs |
--------------------------------------------------------------------------
| gnss    | 163966 |      15 |        0 |   1156 |      46 |        5095 |
| imu     |  16399 |       0 |        0 |    549 |     422 |        1242 |
| battery |  16399 |       0 |        0 |    159 |      14 |        1692 |
| power   |   3280 |       0 |        0 |    240 |     110 |        1799 |
| ble     |  16399 |       0 |        0 |    139 |       6 |        1914 |
| led     |   6560 |       0 |        0 |     57 |      10 |        1922 |
| console |  16398 |       0 |        1 |   4862 |      17 |        1935 |
==========================================================================
| RT:                          168s | Loops:                      180511 |
==========================================================================
Reset: software
GNSS: epochs 3279, missed 0, max gap 60572μs, time steps 0
GNSS: 3D fix, 6 SVs, hAcc 661mm, tAcc 24ns, pDOP 3.45
IMU: up, Accel XYZ mG 0 0 1000, Gyro XYZ c°/s 4 0 -1
IMU: trim locked, tilt 1.2°
BLE: disconnected, sent 0, failed 0, refused 0, max send 0μs, rx 0 bytes, rx dropped 0
Battery: 4.16V (last peak 4.16V), 100%, USB, charging, switch noise 0
Console: dropped 0 bytes
```

### The job table

The firmware's work is split into jobs that each run at a fixed rate. One row per job:

| Column | Meaning |
|---|---|
| `runs` | How many times the job has run. |
| `skipped` | Whole periods the job missed because it couldn't start in time. |
| `overruns` | Runs that took longer than the job's time budget. |
| `max μs`, `mean μs` | The longest and the average run, in microseconds. |
| `max late μs` | The longest a run started after it was due. |

A healthy device shows few or no skips and overruns. Printing the report itself takes up to a few milliseconds, during which the other jobs wait. A print that runs past the `console` job's budget is counted as an overrun, and the `gnss` job, which is due every millisecond, counts the periods it waited as skips. That is the report measuring its own cost, not a problem. The report above shows it: console output printed earlier in the session ran over budget once, the `gnss` row's skips and its 5 ms `max late` are the GNSS job waiting behind that output, and no position updates were missed.

`RT` is the uptime in seconds, and `Loops` counts passes of the main loop. `C` clears the table's counters but not `RT`.

The ESP32 has no battery, so its `battery` and `power` jobs show as disabled in the boot log and never run.

### The lines below the table

| Line | What it shows |
|---|---|
| `Reset:` | Why the firmware started, as in the boot log. |
| `GNSS:` (epochs) | Position updates received, how many were missed, the longest gap between two, and how many times the receiver's clock stepped (normal once or twice as it first acquires time). |
| `GNSS:` (fix) | The current fix, satellites in use, horizontal and time accuracy, and pDOP. `(not yet OK)` means the receiver has a fix but doesn't yet consider it within its accuracy limits. `no epoch yet` means nothing has arrived since boot. |
| `IMU:` (sample) | `up` or `down`, then the latest acceleration in milli-g and rotation in hundredths of a degree per second. At rest on a level surface, expect roughly `0 0 1000`. |
| `IMU:` (trim) | `settling` until the self-calibration locks, then `locked`, with the mounting tilt it measured. |
| `BLE:` | Connection state, packets sent, packets the BLE stack rejected, packets not sent because the connection couldn't carry them whole, the longest single send, and bytes written to the device by the app. |
| `Battery:` | Battery boards only. Voltage, charge percentage, and any of `USB`, `charging`, `full`, `warning`, `critical`, `cutoff requested` that apply. `switch noise` counts power-switch readings rejected as noise. |
| `Console:` | Bytes of console output dropped because the host stopped reading. The firmware drops output rather than wait for a stalled host. |

A few `refused` BLE packets right after an app connects are normal: the receiver hasn't finished negotiating the packet size yet.

---

## The status line

`L`, or a number, starts a line once per second:

```
RT: 301s | BLE: 20Hz | GNSS: 20Hz | Batt: 3.91V 76% | Fix: 3D | SV: 17 | tAcc: 22ns | hAcc: 412mm | Lat: 40.1234567 | Long: -75.1234567 | Accel mG: -3 2 1000 | Gyro c°/s: 0 1 -1 | Trim: 1.3° 🔒
```

| Field | Meaning |
|---|---|
| `RT` | Uptime in seconds. |
| `BLE` | Packets sent to the connected app over the last second. `0Hz` when nothing is connected. |
| `GNSS` | Position updates received over the last second. This should match the configured navigation rate. |
| `Batt` | Battery boards only. Voltage and percentage, with ⚡ while charging. |
| `Fix` | `none`, `2D`, `3D`, `DR`, `GNSS+DR`, or `time`. |
| `SV` | Satellites in use. |
| `tAcc`, `hAcc` | The receiver's time and horizontal accuracy estimates. `unknown` until it has one. |
| `Lat`, `Long` | Position in degrees. |
| `Accel mG` | Acceleration on X, Y, Z in milli-g. |
| `Gyro c°/s` | Rotation rate on X, Y, Z in hundredths of a degree per second. |
| `Trim` | The measured mounting tilt, with ⏳ while the self-calibration is settling and 🔒 once it has locked. |

The `GNSS` rate is the quickest health check there is: a steady `20Hz` with a `3D` fix means the receiver is delivering what the firmware asked for. If it sags as the satellite count climbs, see the note on the receiver's clock setting in the [README](../README.md#three-notes-about-gnss-modules).

> [!NOTE]
> The status line prints your position. Keep that in mind before pasting console output somewhere public.
