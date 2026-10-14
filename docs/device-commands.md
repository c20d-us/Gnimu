# Device commands

A connected BLE client can send the same three actions as the serial console's `T`, `R` and `O`: re-trim the IMU, restart the device, and enter System OFF. What each does is in the [console commands](console.md#commands); both routes end in the same handler.

There is no reply over BLE. A restart or System OFF shows as the disconnect, and a re-trim shows in the console log.

## The BLE frame

The client writes to the RX characteristic of the service below. Write with or without response.

| | UUID |
|---|---|
| Service | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` |
| RX (write, client to device) | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` |
| TX (notify, device to client) | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` |

Each command is a single write of its eight bytes; a command split across writes also works. Each command is one fixed frame of class `0x47` with no payload, and the device acts when it sees those exact eight bytes. Anything else written to the device passes harmlessly.

| Action | Bytes |
|---|---|
| Re-trim | `B5 62 47 01 00 00 48 1F` |
| Restart | `B5 62 47 02 00 00 49 22` |
| System OFF | `B5 62 47 03 00 00 4A 25` |

The last two bytes are the 8-bit Fletcher checksum over class, id and length, as UBX defines it.

The device requires no pairing, so any client in range can connect and send these. That is acceptable for a device with no protected data, but worth knowing.

## Reading the configuration

A client can also read what the device is, as text: the same lines the console's `V` command prints. It lives in its own service, apart from the telemetry service, so clients that only stream telemetry never see it.

| | UUID |
|---|---|
| Gnimu service | `20D21CDE-F59E-41F1-9BCA-5BDE69876C8B` |
| Configuration (read) | `689E54CD-60B0-473A-B049-A5E02A2716E0` |

- **Not advertised.** Find it by discovering the device's services once connected. A client that cached the device's services before this was added needs to refresh them.
- **Read the whole value.** It is up to 512 bytes, longer than one BLE packet, so use a long read (most BLE tools do this automatically).
- **UTF-8 text, one line per `\n`.** The first line is the `BUILD` line, which carries the firmware version and revision; read the rest against that. Lines begin `Device ID:` or `Config:`. New lines may be added in later versions, and a client should ignore any it doesn't recognise.
- **Fixed at boot.** The values are what the firmware was built with. The device never accepts changes through this characteristic.

