// This is a private fork of the work done by Anchit Chandra Sekhar
// Original source is at https://github.com/anchit92
// Some minor changes have been made:
//  - Fixed some bugs
//  - Externalized settings to config.h
//  - Added adjustability of BLE power levels
//  - Added gyro calibration
//  - Modularized the code into imu / gnss / ble / telemetry units
//
// Settings live in config.h. The hardware/protocol logic lives in the module
// files (imu, gnss, ble, telemetry); this sketch is just orchestration.
#include "imu.h"
#include "gnss.h"
#include "ble.h"
#include "telemetry.h"

void setup() {
  Serial.begin(115200);
  delay(500); // Allow USB serial to enumerate before sending any output
  Serial.println("🚀 RaceBox Mini Emulator starting up...");

  imuBegin();
  gnssBegin();
  imuCalibrate(); // Late, once the device has settled (see imu.cpp)
  bleBegin();
  telemetryBegin();
}

void loop() {
  gnssCheck(); // Service the GNSS link — keeps data flowing
  updateImuFilters();
  updateLedStatus();
  sendRaceBoxPacketIfReady();
  reportStatus();
  bleUpdate();
}
