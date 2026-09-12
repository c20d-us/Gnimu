// Fake Adafruit MPU6050: m/s^2 and rad/s events from the scripted motion,
// quantised through int16 counts the way the real part is. getEvent() cannot
// fail - exactly the real library's behaviour (IMU-3).
#pragma once
#include "Adafruit_Sensor.h"
#include <math.h>
#include <stdint.h>
enum mpu6050_accel_range_t { MPU6050_RANGE_2_G, MPU6050_RANGE_4_G, MPU6050_RANGE_8_G, MPU6050_RANGE_16_G };
enum mpu6050_gyro_range_t { MPU6050_RANGE_250_DEG, MPU6050_RANGE_500_DEG, MPU6050_RANGE_1000_DEG, MPU6050_RANGE_2000_DEG };
enum mpu6050_bandwidth_t { MPU6050_BAND_260_HZ, MPU6050_BAND_184_HZ, MPU6050_BAND_94_HZ, MPU6050_BAND_44_HZ,
                           MPU6050_BAND_21_HZ, MPU6050_BAND_10_HZ, MPU6050_BAND_5_HZ };
extern float g_motionG[3], g_motionDps[3];
extern bool g_beginOk;

#define MPU6050_ACCEL_OUT 0x3B // base address for the sensor data registers

// What the chip is ACTUALLY configured to, which the fake Wire scales the
// scripted motion by - so a configuration write that did not land shows up
// in the data exactly as it would on the device.
extern mpu6050_accel_range_t g_mpuAccelRange;
extern mpu6050_gyro_range_t g_mpuGyroRange;
extern mpu6050_bandwidth_t g_mpuBandwidth;
extern bool g_configTakes; // false: configuration writes do not land

class Adafruit_MPU6050 {
public:
  // Real signature: begin(uint8_t i2c_addr = 0x68, TwoWire *wire = &Wire, ...).
  // Answers only at 0x68 (AD0 low), like the soldered part - so a wrong
  // IMU_I2C_ADDRESS looks exactly like a missing IMU, as on hardware. Leaves
  // the chip where the real _init() does: +/-2 g, 500 deg/s, 260 Hz.
  bool begin(uint8_t addr = 0x68) {
    g_mpuAccelRange = MPU6050_RANGE_2_G;
    g_mpuGyroRange = MPU6050_RANGE_500_DEG;
    g_mpuBandwidth = MPU6050_BAND_260_HZ;
    return g_beginOk && addr == 0x68;
  }
  // Void and unchecked, like the real ones.
  void setAccelerometerRange(mpu6050_accel_range_t r) { if (g_configTakes) g_mpuAccelRange = r; }
  void setGyroRange(mpu6050_gyro_range_t r) { if (g_configTakes) g_mpuGyroRange = r; }
  void setFilterBandwidth(mpu6050_bandwidth_t b) { if (g_configTakes) g_mpuBandwidth = b; }
  mpu6050_accel_range_t getAccelerometerRange() { return g_mpuAccelRange; }
  mpu6050_gyro_range_t getGyroRange() { return g_mpuGyroRange; }
  mpu6050_bandwidth_t getFilterBandwidth() { return g_mpuBandwidth; }
  // No getEvent(): the driver no longer reads through the library, and a
  // driver that went back to it should fail to build here rather than pass.
};
