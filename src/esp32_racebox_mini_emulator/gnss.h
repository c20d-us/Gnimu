#pragma once
#include <Arduino.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // for the UBX_NAV_PVT_data_t type

// ============================================================================
// GNSS module — u-blox receiver on Serial2
//
// Owns the receiver object and its serial port internally (see gnss.cpp).
// Callers interact only through the small read-only interface below; the live
// receiver is never exposed, so nothing outside this module can reconfigure it.
// ============================================================================

// Bring up the receiver: open serial, detect the module (auto-recovering its
// baud rate if needed), and configure PVT output, dynamic model, navigation
// rate, and enabled constellations. Call once in setup().
void gnssBegin();

// Service the GNSS link — feeds the parser from the UART. Call every loop().
void gnssCheck();

// True exactly once per new navigation epoch (when iTOW advances). Internally
// polls the receiver and de-duplicates, so it is safe to call every loop().
bool gnssHasNewEpoch();

// Read-only view of the most recent PVT solution, or nullptr if none yet.
const UBX_NAV_PVT_data_t *gnssLatestPvt();

// True if the receiver currently reports a valid vehicle heading.
bool gnssHeadingValid();
