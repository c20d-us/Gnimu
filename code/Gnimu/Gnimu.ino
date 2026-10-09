// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include "src/app/App.h"

static gnimu::App app;

void setup() { app.begin(); }
void loop() { app.tick(); }
