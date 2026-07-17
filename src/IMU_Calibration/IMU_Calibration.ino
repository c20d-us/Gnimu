#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

const int NUM_SAMPLES = 5000;
const unsigned long WARMUP_MS = 20 * 60 * 1000; // 20 minutes in ms

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Serial.println("--- MPU-6050 Bench Calibration Utility ---");
  Serial.println("Goal: Calculate stable offsets for Accel and Gyro.");
  Serial.println("Instructions:");
  Serial.println("1. Mount device on a perfectly level surface.");
  Serial.println("2. Let the device warm up (system will wait 20 mins).");
  Serial.println("3. Type 'c' in the Serial Monitor to start calibration.");

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }

  // Warmup phase
  Serial.print("Warming up... ");
  unsigned long start = millis();
  while (millis() - start < WARMUP_MS) {
    if ((millis() - start) % 60000 < 100) { // Print every minute
      Serial.print((millis() - start) / 60000);
      Serial.print(" ");
    }
    delay(1000);
  }
  Serial.println("\nWarmup complete.");
  Serial.println("Press 'c' to begin 5,000-sample calibration...");
}

void loop() {
  if (Serial.available() > 0) {
    char input = Serial.read();
    if (input == 'c') {
      performCalibration();
    }
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

    if (i % 500 == 0) Serial.print("."); // Progress indicator
  }

  // Calculate averages
  float off_ax = sum_ax / NUM_SAMPLES;
  float off_ay = sum_ay / NUM_SAMPLES;
  float off_az = (sum_az / NUM_SAMPLES) - 9.80665; // Remove gravity
  float off_gx = sum_gx / NUM_SAMPLES;
  float off_gy = sum_gy / NUM_SAMPLES;
  float off_gz = sum_gz / NUM_SAMPLES;

  Serial.println("\n\n--- CALIBRATION RESULTS ---");
  Serial.println("Copy these constants into your main code:");
  Serial.println("-------------------------------------------");
  Serial.printf("const float ACCEL_OFFSET_X = %.6f;\n", off_ax);
  Serial.printf("const float ACCEL_OFFSET_Y = %.6f;\n", off_ay);
  Serial.printf("const float ACCEL_OFFSET_Z = %.6f;\n", off_az);
  Serial.printf("const float GYRO_OFFSET_X  = %.6f;\n", off_gx);
  Serial.printf("const float GYRO_OFFSET_Y  = %.6f;\n", off_gy);
  Serial.printf("const float GYRO_OFFSET_Z  = %.6f;\n", off_gz);
  Serial.println("-------------------------------------------");
  Serial.println("System halted. Power cycle to re-run.");

  while (1) { delay(100); } // Stop
}
