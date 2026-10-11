// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#include "BuildInfo.h"

#include "../platform/Target.h"

// scripts/build.sh stamps these as -D flags. A build that skips the script,
// such as one from the Arduino IDE, reports the placeholders below.
#ifndef GNIMU_VERSION
#define GNIMU_VERSION "0.0.0-dev"
#endif
#ifndef GNIMU_REVISION
#define GNIMU_REVISION "IDE-UNSTAMPED"
#endif
#ifndef GNIMU_DIRTY
#define GNIMU_DIRTY "unknown"
#endif
#ifndef GNIMU_BUILD_UTC
#define GNIMU_BUILD_UTC "unknown"
#endif

namespace gnimu {

const BuildInfo& buildInfo() {
  static const BuildInfo info = {GNIMU_VERSION, GNIMU_REVISION, GNIMU_DIRTY, GNIMU_BUILD_UTC,
                                 targetName()};
  return info;
}

}  // namespace gnimu
