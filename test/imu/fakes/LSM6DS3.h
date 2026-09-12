// Fake Seeed LSM6DS3: raw register counts from the scripted motion.
#pragma once
#include <stdint.h>
#include <math.h>
#define I2C_MODE 1
#define IMU_SUCCESS 0
typedef int status_t;
#define LSM6DS3_ACC_GYRO_OUTX_L_G 0x22
#define LSM6DS3_ACC_GYRO_CTRL1_XL 0x10
#define LSM6DS3_ACC_GYRO_CTRL2_G 0x11
#define LSM6DS3_ACC_GYRO_CTRL3_C 0x12
#define LSM6DS3_ACC_GYRO_BDU_BLOCK_UPDATE 0x40
#define LSM6DS3_ACC_GYRO_IF_INC_ENABLED 0x04
extern float g_motionG[3], g_motionDps[3];
extern int g_failNext, g_reads;
extern bool g_beginOk;
// The fake chip's registers live in fakes.cpp, shared with the fake Wire1 the
// driver reads them back through.
void fakeLsmConfigure(int accelOdrHz, int accelRangeG, int accelBwHz,
                      int gyroOdrHz, int gyroRangeDps);
void fakeLsmWrite(uint8_t reg, uint8_t value);

struct SensorSettings {
  uint8_t accelEnabled; uint16_t accelRange, accelSampleRate, accelBandWidth;
  uint8_t gyroEnabled; uint16_t gyroRange, gyroSampleRate; uint8_t tempEnabled;
};
class LSM6DS3 {
public:
  SensorSettings settings;
  // Answers only at 0x6A (SA0 high, as wired on the XIAO Sense), so a wrong
  // IMU_I2C_ADDRESS looks exactly like a missing IMU, as on hardware.
  LSM6DS3(int, uint8_t addr) : addr_(addr) {}
  status_t begin() {
    if (!(g_beginOk && addr_ == 0x6A)) {
      return 1;
    }
    // The real begin() writes CTRL1_XL/CTRL2_G from `settings` and checks
    // nothing. The fake chip takes them unless the scenario says otherwise.
    fakeLsmConfigure(settings.accelSampleRate, settings.accelRange,
                     settings.accelBandWidth, settings.gyroSampleRate,
                     settings.gyroRange);
    return IMU_SUCCESS;
  }
  // No readRegisterRegion(): the driver does its own checked read over the
  // fake Wire1 (which is where a short read can be injected), so a driver that
  // went back to the library's unchecked one would fail to build here.
  float calcGyro(int16_t v) { return v * 0.0175f; }
  float calcAccel(int16_t v) { return v * 0.000122f; }
  status_t writeRegister(uint8_t reg, uint8_t val) {
    fakeLsmWrite(reg, val);
    return IMU_SUCCESS;
  }
private:
  uint8_t addr_;
};
