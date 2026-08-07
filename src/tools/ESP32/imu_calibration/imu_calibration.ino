#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

const int NUM_SAMPLES = 10000;
const uint32_t WARMUP_MINS = 5; // Minutes to warm up before sampling
const uint32_t WARMUP_MS = WARMUP_MINS * 60UL * 1000UL;

// Phases of the utility. Everything runs from loop() as a small state machine
// so we never block for minutes at a time (a long blocking setup() on the ESP32
// invites watchdog/brownout resets) and so prompts stay visible no matter when
// the Serial Monitor attaches after the board's auto-reset.
enum Phase { PROMPT, WARMING, DONE };
Phase phase = PROMPT;

uint32_t warmupStart = 0;
uint32_t lastMinute = 0xFFFFFFFF; // sentinel: no minute printed yet
uint32_t lastPromptMs = 0;

// Drain the serial input; return true if a 'c' was seen.
bool sawStartKey() {
  bool found = false;
  while (Serial.available() > 0) {
    if (Serial.read() == 'c')
      found = true;
  }
  return found;
}

void setup() {
  Serial.begin(115200);
  // Harmless on UART-bridge boards (Serial is always truthy there); gives a
  // native-USB monitor a brief chance to attach.
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
  }

  if (!mpu.begin()) {
    // Reprint continuously: on this board the first line is easily missed in
    // the gap before the monitor re-attaches after reset.
    while (1) {
      Serial.println("Failed to find MPU6050 chip");
      delay(1000);
    }
  }
}

void loop() {
  switch (phase) {
  case PROMPT:
    // Reprint every 2 s so the instructions are guaranteed to be caught,
    // regardless of when the monitor attached after reset.
    if (millis() - lastPromptMs >= 2000) {
      lastPromptMs = millis();
      Serial.println();
      Serial.println("--- MPU-6050 Bench Calibration Utility ---");
      Serial.println("Goal: stable Accel and Gyro offsets.");
      Serial.println("1. Mount the device on a perfectly level surface.");
      Serial.printf("2. Press 'c' to warm up (%u min) and calibrate.\n",
                    WARMUP_MINS);
    }
    if (sawStartKey()) {
      phase = WARMING;
      warmupStart = millis();
      lastMinute = 0xFFFFFFFF;
      Serial.printf("\nWarming up for %u minutes... ", WARMUP_MINS);
    }
    break;

  case WARMING: {
    uint32_t elapsed = millis() - warmupStart;
    uint32_t minute = elapsed / 60000UL;
    if (minute != lastMinute) { // prints 0,1,2,... exactly once each
      lastMinute = minute;
      Serial.printf("%u ", minute);
    }
    if (elapsed >= WARMUP_MS) {
      Serial.println("\nWarmup complete.");
      performCalibration(); // blocks through sampling; that is fine
      phase = DONE;
    }
    delay(50); // keep the loop cool and yielding
    break;
  }

  case DONE:
    break;
  }
}

void performCalibration() {
  Serial.println("Starting calibration, keep device perfectly still...");

  double sum_ax = 0, sum_ay = 0, sum_az = 0;
  double sum_gx = 0, sum_gy = 0, sum_gz = 0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sum_ax += a.acceleration.x;
    sum_ay += a.acceleration.y;
    sum_az += a.acceleration.z;
    sum_gx += g.gyro.x;
    sum_gy += g.gyro.y;
    sum_gz += g.gyro.z;

    if (i % 500 == 0)
      Serial.print("."); // Progress indicator
  }

  // Calculate averages
  float off_ax = sum_ax / NUM_SAMPLES;
  float off_ay = sum_ay / NUM_SAMPLES;
  float off_az = (sum_az / NUM_SAMPLES) - 9.80665; // Remove gravity
  float off_gx = sum_gx / NUM_SAMPLES;
  float off_gy = sum_gy / NUM_SAMPLES;
  float off_gz = sum_gz / NUM_SAMPLES;

  Serial.println("\n\n--- CALIBRATION RESULTS ---");
  Serial.println(
      "Paste these #define lines over the matching ones in config.h:");
  Serial.println("-------------------------------------------");
  Serial.printf("#define IMU_ACCEL_OFFSET_X_MPS2 %+.6ff\n", off_ax);
  Serial.printf("#define IMU_ACCEL_OFFSET_Y_MPS2 %+.6ff\n", off_ay);
  Serial.printf("#define IMU_ACCEL_OFFSET_Z_MPS2 %+.6ff\n", off_az);
  Serial.printf("#define IMU_GYRO_OFFSET_X_RADPS %+.6ff\n", off_gx);
  Serial.printf("#define IMU_GYRO_OFFSET_Y_RADPS %+.6ff\n", off_gy);
  Serial.printf("#define IMU_GYRO_OFFSET_Z_RADPS %+.6ff\n", off_gz);
  Serial.println("-------------------------------------------");
  Serial.println("System halted. Power cycle to re-run.");
}
