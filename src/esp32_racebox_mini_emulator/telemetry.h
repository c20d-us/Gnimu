#pragma once
#include <Arduino.h>

// ============================================================================
// Telemetry module — application glue
//
// Turns the latest GNSS + IMU data into RaceBox packets, sends them over BLE,
// drives the status LED, and prints periodic serial stats. Consumes the
// imu / gnss / ble / ubx_helpers module interfaces; owns no hardware itself.
// ============================================================================

// Configure the status LED pin and start the stats-report timer. Call once in
// setup(), after the other modules are up.
void telemetryBegin();

// Drive the onboard LED: blink while disconnected, solid while connected.
void updateLedStatus();

// On each new GNSS epoch, count it and — when a client is connected — build and
// send an 88-byte RaceBox Data Message over BLE.
void sendRaceBoxPacketIfReady();

// Periodically print packet-rate and GNSS/IMU debug stats over serial.
void reportStatus();
