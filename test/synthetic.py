#!/usr/bin/env python3
# Gnimu - RaceBox Mini-compatible GNSS+IMU streaming telemetry
# Copyright (C) 2026 Chris Halstead
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

"""Generate the synthetic half of the golden vector set.

Phase A of the multi-protocol refactor - see docs/multiprotocol-design.md.

WHY THIS EXISTS
    Captured vectors cover every state the hardware can actually reach. This
    file covers the states it cannot:

      - fixType 1 (dead reckoning), 4 (GNSS+DR) and 5 (time only). sendPacket()
        clamps all three to 0, so they are invisible in a capture even if they
        occurred - and 1 and 4 are unreachable on an M10 regardless, which has
        no dead reckoning. The clamp is protocol POLICY that phase B moves into
        the encoder, so it needs deliberate coverage or a broken clamp would
        pass every captured vector.

      - Geography. A capture taken in one place only ever exercises one sign of
        latitude and longitude.

      - Numeric extremes. Field-width and two's-complement edges that ordinary
        driving never approaches.

WHY encode_packet() IS A TRUSTWORTHY ORACLE HERE
    Normally, generating expected output with the same code you are testing is
    circular. It is not circular here: capture.py's encode_packet() was
    validated against 8,938 real packets from the device, every one round-
    tripping exactly. That measurement is what promotes it from "my reading of
    sendPacket()" to a reference implementation. These vectors inherit that.

    The assertions below are the second line of defence: each one states the
    expected byte independently of what the encoder produced, so a wrong
    expectation fails here rather than silently becoming the golden answer.

NOTE ON THE BATTERY FIELD
    A GC1 line carries the PACKED battery byte, because that is what the
    firmware puts on the wire. Phase B splits it into percent + charging inside
    TelemetrySample (packing is RaceBox-specific and moves to the encoder), so
    the harness must split this byte to build a sample. One consequence: the
    defensive "percent > 100 -> 100" clamp cannot be expressed as a vector,
    since a packed byte can only show the already-clamped value. That clamp is
    unreachable anyway - voltageToPercent() caps at 100 - and is covered by
    inspection, not by test.

USAGE
    ./test/synthetic.py -o test/synthetic.gc1

    Unlike captured vectors, this output contains no real position data and is
    safe to commit.
"""

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from capture import (GC1_ORDER, PAYLOAD_OFF, encode_packet,  # noqa: E402
                     packet_problem)

# A deliberately round, obviously-fictional starting point: 45.0000000 N,
# 9.0000000 E. Round numbers make it visible at a glance that these vectors are
# fabricated rather than captured.
BASE = {
    "iTOW": 200000, "year": 2026, "month": 6, "day": 15,
    "hour": 12, "min": 30, "sec": 45,
    "validDate": 1, "validTime": 1, "fullyResolved": 1, "validMag": 0,
    "tAcc": 30, "nano": 250000,
    "fixType": 3, "gnssFixOK": 1, "headVehValid": 0, "numSV": 12,
    "lon": 90000000, "lat": 450000000, "height": 120000, "hMSL": 118000,
    "hAcc": 1200, "vAcc": 1800,
    "gSpeed": 25000, "headMot": 18000000, "sAcc": 250, "headAcc": 900,
    "pDOP": 110,
    "gX": 100, "gY": -100, "gZ": 1000, "rX": 50, "rY": -50, "rZ": 25,
    "battery": 85,
}

VECTORS = []


def add(label, expect=None, **overrides):
    """Register one vector. `expect` maps payload offset -> expected byte."""
    v = dict(BASE)
    v.update(overrides)
    VECTORS.append((label, v, expect or {}))


# ---------------------------------------------------------------------------
# The fixType clamp. sendPacket() passes 2 and 3 through and maps everything
# else to 0. Offset 20 is the clamped fix type; offset 66 bit 0 is the
# invalid-coordinates flag, set whenever the PRE-clamp fixType is < 2; offset
# 21 bit 0 is "valid fix", which requires fixType == 3 specifically.
# ---------------------------------------------------------------------------
add("clamp: fixType 1 (dead reckoning) -> 0, coords invalid",
    fixType=1, gnssFixOK=1, expect={20: 0, 21: 0, 66: 1})
add("clamp: fixType 4 (GNSS+DR) -> 0, but coords VALID (4 is not < 2)",
    fixType=4, gnssFixOK=1, expect={20: 0, 21: 0, 66: 0})
add("clamp: fixType 5 (time only) -> 0, coords valid",
    fixType=5, gnssFixOK=1, expect={20: 0, 21: 0, 66: 0})
add("clamp: fixType 0 (no fix) stays 0, coords invalid",
    fixType=0, gnssFixOK=0, expect={20: 0, 21: 0, 66: 1})
add("clamp: fixType 2 passes through; valid-fix bit stays CLEAR even with "
    "gnssFixOK (bit 0 requires fixType == 3)",
    fixType=2, gnssFixOK=1, expect={20: 2, 21: 0, 66: 0})
add("clamp: fixType 3 with gnssFixOK=0 -> valid-fix bit clear",
    fixType=3, gnssFixOK=0, expect={20: 3, 21: 0, 66: 0})
add("clamp: fixType 3 with gnssFixOK=1 -> valid-fix bit SET",
    fixType=3, gnssFixOK=1, expect={20: 3, 21: 0x01, 66: 0})
add("headVehValid sets bit 5 (never observed on real M10 hardware)",
    headVehValid=1, expect={21: 0x21})

# ---------------------------------------------------------------------------
# Geography a capture from one location cannot reach.
# ---------------------------------------------------------------------------
add("southern hemisphere, eastern longitude", lat=-338600000, lon=151200000)
add("southern hemisphere, western longitude", lat=-347000000, lon=-584000000)
add("equator and prime meridian (both exactly zero)", lat=0, lon=0)
add("max latitude +90, max longitude +180", lat=900000000, lon=1800000000)
add("min latitude -90, min longitude -180", lat=-900000000, lon=-1800000000)
add("below sea level (Dead Sea, ~-430m)", height=-430000, hMSL=-430000)

# ---------------------------------------------------------------------------
# Validity flag combinations. Offset 11 is validityFlags (bits 0-3 map 1:1 to
# the u-blox booleans); offset 22 is dateTimeFlags, which is DERIVED from three
# of the same booleans - that derivation is what these exercise.
# ---------------------------------------------------------------------------
add("no validity flags at all",
    validDate=0, validTime=0, fullyResolved=0, validMag=0,
    expect={11: 0x00, 22: 0x00})
add("all four validity flags",
    validDate=1, validTime=1, fullyResolved=1, validMag=1,
    expect={11: 0x0F, 22: 0xE0})
add("validTime without fullyResolved: bit 7 must stay clear",
    validDate=0, validTime=1, fullyResolved=0, validMag=0,
    expect={11: 0x02, 22: 0x20})
add("fullyResolved without validTime: neither bit 5 nor bit 7",
    validDate=0, validTime=0, fullyResolved=1, validMag=0,
    expect={11: 0x04, 22: 0x00})
add("validDate only",
    validDate=1, validTime=0, fullyResolved=0, validMag=0,
    expect={11: 0x01, 22: 0x40})
add("validMag only - reaches validityFlags but NOT dateTimeFlags",
    validDate=0, validTime=0, fullyResolved=0, validMag=1,
    expect={11: 0x08, 22: 0x00})

# ---------------------------------------------------------------------------
# Battery. The capture covered 0x64 and 0xE4 (percent 100, both charging
# states); these add percent variation, which the capture could not.
# ---------------------------------------------------------------------------
add("battery 0%, not charging", battery=0, expect={67: 0x00})
add("battery 0%, charging", battery=0x80, expect={67: 0x80})
add("battery 1%, charging", battery=0x81, expect={67: 0x81})
add("battery 99%, not charging", battery=99, expect={67: 0x63})
add("battery 100%, charging", battery=0xE4, expect={67: 0xE4})

# ---------------------------------------------------------------------------
# Numeric and width extremes. These are direct copies in the encoder, so what
# they exercise is field width and the two's-complement path in
# writeLittleEndian(), not any derivation.
# ---------------------------------------------------------------------------
add("IMU all at int16 max", gX=32767, gY=32767, gZ=32767,
    rX=32767, rY=32767, rZ=32767)
add("IMU all at int16 min", gX=-32768, gY=-32768, gZ=-32768,
    rX=-32768, rY=-32768, rZ=-32768)
add("IMU alternating extremes", gX=32767, gY=-32768, gZ=32767,
    rX=-32768, rY=32767, rZ=-32768)
add("nano at u-blox negative limit", nano=-500000000)
add("nano at u-blox positive limit", nano=500000000)
add("nano at int32 limits (beyond spec, tests field width)", nano=-2147483648)
add("iTOW at uint32 max", iTOW=4294967295)
add("accuracy fields at uint32 max",
    tAcc=4294967295, hAcc=4294967295, vAcc=4294967295,
    sAcc=4294967295, headAcc=4294967295)
add("pDOP at uint16 max", pDOP=65535)
add("numSV at uint8 max (well beyond any real constellation)", numSV=255)
add("headMot at 359.99999 deg", headMot=35999999)
add("headMot negative", headMot=-18000000)
add("gSpeed negative (signed field, though u-blox reports >= 0)", gSpeed=-25000)
add("leap second (sec = 60)", sec=60)
add("date/time at field maxima", year=65535, month=255, day=255,
    hour=255, min=255, sec=255)

# ---------------------------------------------------------------------------
# Whole-packet extremes.
# ---------------------------------------------------------------------------
add("everything zero",
    **{k: 0 for k in GC1_ORDER}, expect={20: 0, 21: 0, 66: 1})
add("every unsigned field at max, every signed field at min",
    iTOW=4294967295, year=65535, month=255, day=255, hour=255, min=255,
    sec=255, validDate=1, validTime=1, fullyResolved=1, validMag=1,
    tAcc=4294967295, nano=-2147483648, fixType=3, gnssFixOK=1,
    headVehValid=1, numSV=255, lon=-2147483648, lat=-2147483648,
    height=-2147483648, hMSL=-2147483648, hAcc=4294967295, vAcc=4294967295,
    gSpeed=-2147483648, headMot=-2147483648, sAcc=4294967295,
    headAcc=4294967295, pDOP=65535, gX=-32768, gY=-32768, gZ=-32768,
    rX=-32768, rY=-32768, rZ=-32768, battery=0xE4,
    expect={20: 3, 21: 0x21, 66: 0, 67: 0xE4})


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("-o", "--out", metavar="PATH", default="-",
                    help="write vectors here (default: stdout)")
    args = ap.parse_args()

    failures = []
    lines = []
    for label, v, expect in VECTORS:
        # A GC1 battery field is the PACKED byte, and the encoder splits it
        # into percent + charging before re-packing (clamping percent to 100).
        # So a byte whose low 7 bits exceed 100 cannot survive that round trip
        # - and no real device emits one, because voltageToPercent caps at 100.
        # Asking for one produces a vector the firmware can never satisfy.
        if (v["battery"] & 0x7F) > 100:
            failures.append(
                f"{label}: battery byte 0x{v['battery']:02X} implies percent "
                f"{v['battery'] & 0x7F}, which is above 100 and unreachable. "
                "Max emittable byte is 0xE4.")
        pkt = encode_packet(v)

        # Every synthetic packet must still be a structurally valid RaceBox
        # frame - right sync, class, declared length and checksum. A generator
        # bug that produced malformed frames would otherwise become the golden
        # answer the harness is measured against.
        problem = packet_problem(pkt)
        if problem:
            failures.append(f"{label}: malformed packet - {problem}")

        # Assertions state the expected byte independently of the encoder, so a
        # wrong expectation fails here instead of being silently blessed.
        for off, want in expect.items():
            got = pkt[PAYLOAD_OFF + off]
            if got != want:
                failures.append(
                    f"{label}: payload offset {off} = 0x{got:02X}, "
                    f"expected 0x{want:02X}")

        lines.append(f"# {label}")
        lines.append("GC1," + ",".join(str(v[k]) for k in GC1_ORDER)
                     + "," + pkt.hex().upper())

    if failures:
        print(f"❌ {len(failures)} expectation failure(s):", file=sys.stderr)
        for f in failures:
            print(f"   {f}", file=sys.stderr)
        return 1

    body = "\n".join([
        "# Synthetic GC1 golden vectors - generated by test/synthetic.py",
        f"# {len(VECTORS)} vectors covering states the hardware cannot reach.",
        "# No real position data; safe to commit.",
        f"# Fields: {','.join(GC1_ORDER)},packetHex",
        "#",
        "# fixType here is the TRUE pre-clamp value, unlike captured vectors",
        "# where it can only ever be the post-clamp 0, 2 or 3.",
        *lines,
    ]) + "\n"

    if args.out == "-":
        sys.stdout.write(body)
    else:
        with open(args.out, "w", encoding="utf-8") as out:
            out.write(body)
        print(f"✅ {len(VECTORS)} vectors, all expectations met -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
