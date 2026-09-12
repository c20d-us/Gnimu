#!/usr/bin/env bash
# Gnimu - IMU pipeline harness runner.
#
# Compiles each variant's REAL g_imu* / ImuAxis / g_imu_trim sources and its
# REAL config.h against test/imu/fakes, runs one deterministic scenario
# (test/imu/imu_harness.cpp), and compares stdout with test/imu/golden/.
#
#   ./test/run_imu_harness.sh          compare against the golden output
#   ./test/run_imu_harness.sh --save   (re)write the golden output
#
# Two profiles per variant:
#   parked - config.h and g_imu_tuning.h exactly as shipped (transient
#            thresholds parked, IMU-2)
#   live   - transient thresholds switched on, so ImuAxis's transient path is
#            exercised too rather than sitting idle behind the sentinel
#   faults - the IMU-down state: missing at boot, dying mid-run, a
#            configuration the part did not take, on the nRF a lost BDU write,
#            and on the ESP32 a sensor reset to its power-on state. Logs
#            included.
#   notfitted - IMU_ENABLED 0, built with no sensor library in reach. On the
#            nRF via the plain-XIAO board macro, testing the auto-detection.
#   wrongaddr - IMU_I2C_ADDRESS moved to where nothing answers: the hardware
#            rehearsal for a missing IMU. Must take the "not found" path.
#
# nRF builds get -DARDUINO_Seeed_XIAO_nRF52840_Sense, as the IDE defines for
# that board, so IMU_ENABLED resolves exactly as it does in the real build.
#
# Needs the SparkFun u-blox GNSS v3 library for u-blox_structs.h, which
# compiles standalone (see ROB-6). Override the path with SPARKFUN_UBLOX_SRC.
#
# Floating point is compiled with -ffp-contract=off so the output is a function
# of the source alone. Golden files are host output: if they mismatch on a
# different compiler, regenerate them from a known-good commit with --save.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SPARKFUN="${SPARKFUN_UBLOX_SRC:-$HOME/Documents/Arduino/libraries/SparkFun_u-blox_GNSS_v3/src}"
GOLD="$ROOT/test/imu/golden"
mkdir -p "$GOLD"
SAVE=0
[ "${1:-}" = "--save" ] && SAVE=1
status=0

for V in Gnimu-ESP32 Gnimu-nRF52840 Gnimu-nRF52840-OLED; do
  for PROFILE in parked live faults notfitted wrongaddr; do
    B="$(mktemp -d)"
    cp "$ROOT/src/$V"/g_imu*.cpp "$ROOT/src/$V"/g_imu*.h "$ROOT/src/$V"/ImuAxis.* \
       "$ROOT/src/$V/config.h" "$B/"
    cp "$ROOT/test/imu/fakes/"* "$B/"
    BOARD=""
    case "$V" in *nRF*) BOARD="-DARDUINO_Seeed_XIAO_nRF52840_Sense" ;; esac
    if [ "$PROFILE" = wrongaddr ]; then
      # The hardware rehearsal for a missing IMU: nothing answers there.
      perl -pi -e 's/^(#define IMU_I2C_ADDRESS\s+)0x68\b/${1}0x69/; s/^(#define IMU_I2C_ADDRESS\s+)0x6A\b/${1}0x6B/' "$B/config.h"
      grep -Eq '^#define IMU_I2C_ADDRESS +0x6(9|B)' "$B/config.h" || { echo "❌ $V: could not move IMU_I2C_ADDRESS"; status=1; rm -rf "$B"; continue; }
    fi
    if [ "$PROFILE" = notfitted ]; then
      # No sensor library in reach: if anything still includes one, the build
      # fails - which is the claim (IMU_ENABLED 0 needs no IMU library).
      rm -f "$B/LSM6DS3.h" "$B/Adafruit_MPU6050.h" "$B/Adafruit_Sensor.h"
      case "$V" in
        # nRF: the plain XIAO board, so IMU_ENABLED sets itself to 0 - this
        # tests the auto-detection as well as the stub.
        *nRF*) BOARD="-DARDUINO_Seeed_XIAO_nRF52840" ;;
        *) perl -pi -e 's/^#define IMU_ENABLED 1\b/#define IMU_ENABLED 0/' "$B/config.h"
           grep -q '^#define IMU_ENABLED 0' "$B/config.h" || { echo "❌ $V: could not set IMU_ENABLED 0"; status=1; rm -rf "$B"; continue; } ;;
      esac
    fi
    if [ "$PROFILE" = live ]; then
      # The thresholds live in the shared g_imu_tuning.h (ARC-8), which the
      # g_imu*.h copy above brought in. Checked POSITIVELY: if the edit misses,
      # the build must not quietly run parked under the "live" name.
      perl -pi -e '
        s/^(#define IMU_ACCEL_TRANSIENT_THRESHOLD_G\s+)IMU_TRANSIENT_PARKED/${1}0.06f/;
        s/^(#define IMU_GYRO_TRANSIENT_THRESHOLD_DPS\s+)IMU_TRANSIENT_PARKED/${1}3.0f/;
      ' "$B/g_imu_tuning.h"
      if [ "$(grep -Ec '^#define IMU_(ACCEL_TRANSIENT_THRESHOLD_G +0\.06f|GYRO_TRANSIENT_THRESHOLD_DPS +3\.0f)$' "$B/g_imu_tuning.h")" != 2 ]; then
        echo "❌ $V: could not switch the transient thresholds on - g_imu_tuning.h changed shape"
        status=1; rm -rf "$B"; continue
      fi
    fi
    PART="-DHARNESS_LSM6DS3"
    case "$V" in *ESP32*) PART="-DHARNESS_MPU6050" ;; esac
    if ! c++ -std=gnu++11 -ffp-contract=off -I"$B" -I"$SPARKFUN" $PART $BOARD \
         "$B"/*.cpp "$ROOT/test/imu/imu_harness.cpp" -o "$B/harness" 2>"$B/build.txt"; then
      echo "❌ $V $PROFILE: build failed"; sed 's/^/     /' "$B/build.txt" | head -30
      status=1; rm -rf "$B"; continue
    fi
    if [ "$PROFILE" = notfitted ]; then
      "$B/harness" not-fitted >"$B/out.txt" 2>&1
    elif [ "$PROFILE" = wrongaddr ]; then
      "$B/harness" wrong-address >"$B/out.txt" 2>&1
    elif [ "$PROFILE" = faults ]; then
      # Logs are part of the behaviour under test here ("once, never per
      # sample"), so stderr is kept, interleaved in order.
      : >"$B/out.txt"
      for S in boot-missing dies misconfigured bdu-lost lpf1-lost sensor-reset; do
        "$B/harness" "$S" >>"$B/out.txt" 2>&1
      done
    else
      "$B/harness" >"$B/out.txt" 2>"$B/log.txt"
    fi
    G="$GOLD/$V-$PROFILE.txt"
    N=$(wc -l <"$B/out.txt" | tr -d ' ')
    if [ $SAVE = 1 ]; then
      cp "$B/out.txt" "$G"
      echo "saved   $V $PROFILE  ($N lines, $(tail -1 "$G"))"
    elif cmp -s "$B/out.txt" "$G"; then
      echo "✅ $V $PROFILE  identical ($N lines)"
    else
      echo "❌ $V $PROFILE  DIFFERS from golden:"
      # `|| true`: diff exits 1 on a difference, which under pipefail + set -e
      # would abort the whole run at the first mismatch and hide the rest.
      diff "$G" "$B/out.txt" | head -12 | sed 's/^/     /' || true
      status=1
    fi
    rm -rf "$B"
  done
done
exit $status
