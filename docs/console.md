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
| `V` | Prints the build line and the device ID. |
| `L` | Turns the [status line](#the-status-line) on or off. Left on, it stops by itself after 3600 lines (one hour). |
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

1. **Identity and start.** The `BUILD` line and the device ID, then `Gnimu starting`, the reason for the last reset, and the watchdog.
2. **Power** (XIAO nRF52840 only). The switch and USB state, and a first battery reading.
3. **GNSS.** Detecting the receiver, then one line for every receiver setting the firmware checks.
4. **IMU and BLE.** The IMU start-up, then BLE, which is started last and ends with advertising.
5. **Jobs.** One line per job with how often it runs and its time budget, or `disabled`.
6. **State and commands.** The state the firmware ended up in, and the command list.

`Reset:` says why the firmware started: `power-on`, `watchdog`, `software`, `reset pin`, and so on. When the firmware restarted itself, the reason follows in brackets, for example `Reset: software (GNSS: epochs stopped)`.

The `BUILD` line identifies the firmware. See [the build stamp](building.md#the-build-stamp) for why an Arduino IDE build shows placeholders in it.

The **device ID** is a 10-digit number derived from the chip's own hardware ID. It is the same on every boot and different on every board, and it is the number in the BLE name the device advertises: `RaceBox Mini <device ID>`.

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

The firmware does not retry by itself in a fault state; power-cycle the device once the cause is fixed. The commands still work, so `D` and `V` are available while you investigate.

If the receiver was working and then goes quiet for two seconds, the firmware restarts itself instead, which runs the whole start-up again.

---

## The diagnostics report

`D` prints a snapshot of everything the firmware is tracking. The numbers below are an example from a battery-powered XIAO nRF52840 five minutes after power-on:

```
==========================================================================
| job     |   runs | skipped | overruns | max μs | mean μs | max late μs |
--------------------------------------------------------------------------
| gnss    | 300214 |      12 |        0 |    663 |      41 |        1299 |
| imu     |  30021 |       0 |        0 |    512 |     318 |        1105 |
| battery |  30021 |       0 |        0 |    185 |      52 |        1210 |
| power   |   6004 |       0 |        0 |    131 |     107 |         980 |
| ble     |  30021 |       0 |        0 |    402 |      23 |        1150 |
| led     |  12008 |       0 |        0 |     88 |       9 |        1020 |
| console |  30021 |       0 |        1 |   3912 |      14 |        1180 |
==========================================================================
| RT:                          300s | Loops:                   262104311 |
==========================================================================
Reset: power-on
GNSS: GPS+Galileo, 20 Hz, automotive
GNSS: epochs 6004, missed 0, max gap 52110μs, time steps 0
GNSS: 3D fix, 17 SVs, hAcc 412mm, tAcc 22ns, pDOP 1.21
IMU: up, Accel XYZ mG -3 2 1000, Gyro XYZ c°/s 0 1 -1
IMU: trim locked, tilt 1.3°
BLE: connected, sent 5412, failed 0, refused 0, max send 398μs, rx 92 bytes, rx dropped 0
Battery: 3.91V (last peak 3.92V), 76%, switch noise 0
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

A healthy device shows few or no skips and overruns. Printing the report itself takes a few milliseconds, so the `console` job normally collects one overrun per `D`; that is the report measuring its own cost, not a problem.

`RT` is the uptime in seconds, and `Loops` counts passes of the main loop. `C` clears the table's counters but not `RT`.

The ESP32 has no battery, so its `battery` and `power` jobs show as disabled in the boot log and never run.

### The lines below the table

| Line | What it shows |
|---|---|
| `Reset:` | Why the firmware started, as in the boot log. |
| `GNSS:` (settings) | The constellations, navigation rate, and dynamic model the firmware configured. |
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
