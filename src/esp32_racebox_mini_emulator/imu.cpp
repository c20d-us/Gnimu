#include "imu.h"
#include "config.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// --- IMU state — private to this file (file-scope `static`) ---
static Adafruit_MPU6050 mpu;

// Storage for the filtered values
static float filtered_ax = 0, filtered_ay = 0, filtered_az = 0;
static float filtered_gx = 0, filtered_gy = 0, filtered_gz = 0;

// Gyro bias offsets — measured at startup, subtracted from every reading.
// Initialized to 0.0 so they are safe no-ops when calibration is disabled.
static float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;

#ifdef GYRO_CALIBRATION_ENABLED
static void calibrateGyro() {
  Serial.println("⏳ Gyro calibration starting — keep device still...");
  double sumX = 0, sumY = 0, sumZ = 0;
  sensors_event_t a, g, temp;
  for (int i = 0; i < GYRO_CALIBRATION_SAMPLES; i++) {
    mpu.getEvent(&a, &g, &temp);
    sumX += g.gyro.x;
    sumY += g.gyro.y;
    sumZ += g.gyro.z;
    delay(ACCEL_SAMPLE_INTERVAL_MS);
  }
  gyro_bias_x = sumX / GYRO_CALIBRATION_SAMPLES;
  gyro_bias_y = sumY / GYRO_CALIBRATION_SAMPLES;
  gyro_bias_z = sumZ / GYRO_CALIBRATION_SAMPLES;
  Serial.printf(
      "✅ Gyro calibration complete. Bias: X=%.4f Y=%.4f Z=%.4f rad/s\n",
      gyro_bias_x, gyro_bias_y, gyro_bias_z);
}
#endif // GYRO_CALIBRATION_ENABLED

void imuBegin() {
  if (!mpu.begin()) {
    Serial.println("❌ Failed to find MPU6050 chip");
    while (1)
      delay(100);
  } else {
    Serial.println("✅ MPU6050 Acceleromter/Gyro enabled.");
  }
  mpu.setAccelerometerRange(ACCEL_RANGE);
  mpu.setGyroRange(GYRO_RANGE);
  mpu.setFilterBandwidth(FILTER_BANDWIDTH);
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Initialize filters with the first real reading so they don't start at zero
  filtered_ax = a.acceleration.x;
  filtered_ay = a.acceleration.y;
  filtered_az = a.acceleration.z;
}

void imuCalibrate() {
#ifdef GYRO_CALIBRATION_ENABLED
  // Do this late in setup() so that the device has time to be settled prior.
  // Doing it sooner might catch small movements from being plugged in.
  // To be safe, throw in an extra half-second of wait time for settling.
  delay(500);
  calibrateGyro();
#endif // GYRO_CALIBRATION_ENABLED
}

void updateImuFilters() {
  static unsigned long lastAccelReadMs = 0;
  // Update Accelerometer readings at fixed interval
  if (millis() - lastAccelReadMs >= ACCEL_SAMPLE_INTERVAL_MS) {
    lastAccelReadMs = millis();
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Apply Exponential Moving Average (Complementary Filter logic)
    filtered_ax =
        (ACCEL_ALPHA * a.acceleration.x) + ((1.0 - ACCEL_ALPHA) * filtered_ax);
    filtered_ay =
        (ACCEL_ALPHA * a.acceleration.y) + ((1.0 - ACCEL_ALPHA) * filtered_ay);
    filtered_az =
        (ACCEL_ALPHA * a.acceleration.z) + ((1.0 - ACCEL_ALPHA) * filtered_az);

    filtered_gx = (GYRO_ALPHA * (g.gyro.x - gyro_bias_x)) +
                  ((1.0 - GYRO_ALPHA) * filtered_gx);
    filtered_gy = (GYRO_ALPHA * (g.gyro.y - gyro_bias_y)) +
                  ((1.0 - GYRO_ALPHA) * filtered_gy);
    filtered_gz = (GYRO_ALPHA * (g.gyro.z - gyro_bias_z)) +
                  ((1.0 - GYRO_ALPHA) * filtered_gz);
  }
}

ImuProtocolUnits readImuProtocolUnits() {
  ImuProtocolUnits u;
  // Convert accelerometer to milli-g (1g = 9.80665 m/s^2)
  u.gX = filtered_ax * 1000.0 / 9.80665;
  u.gY = filtered_ay * 1000.0 / 9.80665;
  u.gZ = filtered_az * 1000.0 / 9.80665;
  // Convert gyro to centi-deg/sec
  u.rX = filtered_gx * 180.0 / M_PI * 100.0;
  u.rY = filtered_gy * 180.0 / M_PI * 100.0;
  u.rZ = filtered_gz * 180.0 / M_PI * 100.0;
  return u;
}
