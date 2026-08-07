# ESP32 tools

Standalone sketches for bringing up or bench-tuning parts of the ESP32 build
outside the main firmware. Each lives in its own folder so the Arduino IDE can
open it directly.

| Sketch | Purpose |
|---|---|
| [`imu_calibration/`](imu_calibration/imu_calibration.ino) | Measures per-axis accel + gyro zero-point offsets on the MPU6050 (5-minute warmup + 10000-sample averaging) and prints the offsets ready to paste into `config.h`. |
