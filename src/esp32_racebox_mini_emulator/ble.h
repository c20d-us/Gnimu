#pragma once
#include <Arduino.h>

// ============================================================================
// BLE module — RaceBox-compatible Bluetooth Low Energy server
//
// Owns the BLE server, characteristics, and connection state internally (see
// ble.cpp). Callers interact only through the small interface below; the live
// server objects are never exposed.
// ============================================================================

// Initialize the BLE device: set TX power, create the RaceBox service and its
// Tx/Rx characteristics, publish the Device Information Service, and start
// advertising. Call once in setup(), after the device name is known.
void bleBegin();

// True while a client is connected.
bool bleIsConnected();

// Send a packet to the connected client via a notify on the Tx characteristic.
// Caller is responsible for checking bleIsConnected() first if it cares.
void bleSendPacket(uint8_t *data, size_t len);

// Service the connection lifecycle — re-advertise after a disconnect and track
// connect/disconnect edges. Call every loop().
void bleUpdate();
