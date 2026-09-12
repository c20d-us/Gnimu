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

"""Turn a Gnimu Monitor capture into golden test vectors, and report coverage.

Phase A of the multi-protocol refactor - see docs/multiprotocol-design.md
sections 10 and 11.

WHAT THIS IS FOR
    Phase B moves ~130 lines of field packing out of sendPacket() and inserts a
    canonical struct in the middle of the data path. The claim is that it emits
    byte-identical output. This tool produces the evidence for that claim.

    Gnimu Monitor records the raw RaceBox frames it receives, base64-encoded,
    one JSON object per line. That is a byte-exact record of the OUTPUT side,
    captured after BLE fragmentation and reassembly - a stronger reference than
    a firmware-side dump, which would only show what the firmware believed it
    sent.

THE CATCH, AND WHY IT IS SMALL
    Golden vectors need input/output PAIRS; a capture holds only outputs. So
    this tool reconstructs the inputs sendPacket() must have read. Almost all
    of them are recoverable, because RaceBox is largely a re-frame of
    UBX-NAV-PVT and most fields are direct copies.

    Exactly two are not:

    fixType  - CLAMPED on the way out. sendPacket() maps anything other than 2
               or 3 to 0, so a packet reading 0 could have come from a true 0,
               1 (dead reckoning), 4 (GNSS+DR) or 5 (time only). Reconstruction
               picks 0. Consequence: these vectors exercise the clamp only for
               values that survive it. Cover 1/4/5 with synthetic vectors -
               1 and 4 are unreachable on an M10 anyway, so they always needed
               synthesizing.

    gnssFixOK - observable only when fixType == 3, since it reaches the wire
               only through bit 0 of fixStatusFlags, which is
               (fixType == 3 && gnssFixOK). When fixType is not 3 the value
               cannot influence the output at all, so any choice reproduces the
               packet. Unobservable here means immaterial; reconstruction uses
               0 and loses nothing.

IS RECONSTRUCT-THEN-RE-ENCODE CIRCULAR?
    No. The decoder below is written from the packet layout, independently of
    the encoder under test. If phase B writes hAcc at the wrong offset, the
    reconstruction - reading the old, correct offset - yields an input that
    re-encodes differently, and the diff catches it. The test is meaningful for
    field placement, endianness, scaling, and every flag derivation. It is
    vacuous ONLY for the fixType clamp, where reconstruction has to invert a
    lossy transform.

SELF-VERIFICATION
    encode_packet() below is a faithful Python port of sendPacket(). Every
    vector is re-encoded from its reconstructed inputs and diffed against the
    original captured bytes. A mismatch means either the reconstruction is
    wrong or this port has drifted from the firmware - both worth knowing
    before the C++ harness is ever written, and both reported as failures
    rather than silently tolerated.

USAGE
    ./test/capture.py <capture.jsonl>                  # coverage report
    ./test/capture.py <capture.jsonl> -o vectors.gc1   # + emit test vectors

    Vectors are written in the "GC1," line format, one per line: 35 input
    fields then the 88 expected bytes as hex. The format is deliberately
    independent of where the vectors came from, so a firmware-side dump could
    feed the same harness if the pre-clamp fixType is ever needed.
"""

import argparse
import base64
import binascii
import json
import struct
import sys
from collections import Counter

PACKET_LEN = 88
PAYLOAD_OFF = 6
PAYLOAD_LEN = 80
UBX_SYNC = b"\xB5\x62"
RACEBOX_CLASS = 0xFF
RACEBOX_ID = 0x01

# Direct field copies: (name, payload offset, struct format). These are data -
# the encoder moves them without an opinion, so decode and encode can share one
# table. The derived fields (validityFlags, the fixType clamp, fixStatusFlags,
# dateTimeFlags, latLonFlags) are protocol POLICY and are handled explicitly
# below, which is the same split the refactor itself is built around.
DIRECT_FIELDS = [
    ("iTOW", 0, "<I"), ("year", 4, "<H"), ("month", 6, "<B"), ("day", 7, "<B"),
    ("hour", 8, "<B"), ("min", 9, "<B"), ("sec", 10, "<B"),
    ("tAcc", 12, "<I"), ("nano", 16, "<i"), ("numSV", 23, "<B"),
    ("lon", 24, "<i"), ("lat", 28, "<i"), ("height", 32, "<i"),
    ("hMSL", 36, "<i"), ("hAcc", 40, "<I"), ("vAcc", 44, "<I"),
    ("gSpeed", 48, "<i"), ("headMot", 52, "<i"), ("sAcc", 56, "<I"),
    ("headAcc", 60, "<I"), ("pDOP", 64, "<H"), ("battery", 67, "<B"),
    ("gX", 68, "<h"), ("gY", 70, "<h"), ("gZ", 72, "<h"),
    ("rX", 74, "<h"), ("rY", 76, "<h"), ("rZ", 78, "<h"),
]

# Order of the 35 input values on a GC1 line. Keep in step with DIRECT_FIELDS
# and the reconstruction below.
GC1_ORDER = [
    "iTOW", "year", "month", "day", "hour", "min", "sec",
    "validDate", "validTime", "fullyResolved", "validMag", "tAcc", "nano",
    "fixType", "gnssFixOK", "headVehValid", "numSV",
    "lon", "lat", "height", "hMSL", "hAcc", "vAcc",
    "gSpeed", "headMot", "sAcc", "headAcc", "pDOP",
    "gX", "gY", "gZ", "rX", "rY", "rZ", "battery",
]


def ubx_checksum(payload, cls, msg_id, length):
    """8-bit Fletcher over class, id, length and payload - the UBX algorithm.

    Ported from calculateChecksum() in g_ubx_helpers.cpp.
    """
    a = b = 0
    for byte in (cls, msg_id, length & 0xFF, length >> 8):
        a = (a + byte) & 0xFF
        b = (b + a) & 0xFF
    for byte in payload:
        a = (a + byte) & 0xFF
        b = (b + a) & 0xFF
    return a, b


def packet_problem(pkt):
    """Return a description of why pkt is not a valid RaceBox frame, or None."""
    if len(pkt) != PACKET_LEN:
        return f"length {len(pkt)}, expected {PACKET_LEN}"
    if pkt[0:2] != UBX_SYNC:
        return f"bad sync {pkt[0]:#04x} {pkt[1]:#04x}"
    if pkt[2] != RACEBOX_CLASS or pkt[3] != RACEBOX_ID:
        return f"bad class/id {pkt[2]:#04x}/{pkt[3]:#04x}"
    declared = pkt[4] | (pkt[5] << 8)
    if declared != PAYLOAD_LEN:
        return f"declared payload {declared}, expected {PAYLOAD_LEN}"
    payload = pkt[PAYLOAD_OFF:PAYLOAD_OFF + PAYLOAD_LEN]
    ck_a, ck_b = ubx_checksum(payload, RACEBOX_CLASS, RACEBOX_ID, PAYLOAD_LEN)
    if (ck_a, ck_b) != (pkt[86], pkt[87]):
        return (f"checksum {pkt[86]:#04x}{pkt[87]:#04x}, "
                f"computed {ck_a:#04x}{ck_b:#04x}")
    return None


def reconstruct(pkt):
    """Recover the inputs sendPacket() read, from the bytes it produced.

    See the module docstring for what is and is not recoverable.
    """
    p = pkt[PAYLOAD_OFF:PAYLOAD_OFF + PAYLOAD_LEN]
    v = {name: struct.unpack_from(fmt, p, off)[0]
         for name, off, fmt in DIRECT_FIELDS}

    # validityFlags bits 0-3 are 1:1 with the four u-blox booleans, so this
    # inverts exactly.
    validity = p[11]
    v["validDate"] = (validity >> 0) & 1
    v["validTime"] = (validity >> 1) & 1
    v["fullyResolved"] = (validity >> 2) & 1
    v["validMag"] = (validity >> 3) & 1

    # Post-clamp value; the true input may have been 1, 4 or 5 when this is 0.
    v["fixType"] = p[20]

    fix_status = p[21]
    # Bit 0 is (fixType == 3 && gnssFixOK). Setting it implies both. When it is
    # clear, gnssFixOK is either genuinely false or unobservable - and in the
    # unobservable case it cannot affect the output, so 0 is safe.
    v["gnssFixOK"] = 1 if (fix_status & 1) else 0
    v["headVehValid"] = (fix_status >> 5) & 1

    # dateTimeFlags (p[22]) and latLonFlags (p[66]) carry no independent input:
    # both are derived from fields already recovered above. They are not
    # reconstructed, they are RE-DERIVED by encode_packet() - which is precisely
    # what makes the round trip a real test of those derivations.
    return v


def encode_packet(v):
    """Faithful Python port of sendPacket(). Inputs in, 88 bytes out."""
    pkt = bytearray(PACKET_LEN)
    p = memoryview(pkt)[PAYLOAD_OFF:PAYLOAD_OFF + PAYLOAD_LEN]

    for name, off, fmt in DIRECT_FIELDS:
        struct.pack_into(fmt, p, off, v[name])

    validity = 0
    if v["validDate"]:
        validity |= 1 << 0
    if v["validTime"]:
        validity |= 1 << 1
    if v["fullyResolved"]:
        validity |= 1 << 2
    if v["validMag"]:
        validity |= 1 << 3
    p[11] = validity

    fix_type = v["fixType"]
    p[20] = fix_type if fix_type in (2, 3) else 0

    fix_status = 0
    if fix_type == 3 and v["gnssFixOK"]:
        fix_status |= 1 << 0
    if v["headVehValid"]:
        fix_status |= 1 << 5
    p[21] = fix_status

    date_time = 0
    if v["validTime"]:
        date_time |= 1 << 5
    if v["validDate"]:
        date_time |= 1 << 6
    if v["validTime"] and v["fullyResolved"]:
        date_time |= 1 << 7
    p[22] = date_time

    p[66] = 1 if fix_type < 2 else 0

    pkt[0:2] = UBX_SYNC
    pkt[2] = RACEBOX_CLASS
    pkt[3] = RACEBOX_ID
    pkt[4] = PAYLOAD_LEN
    pkt[5] = 0
    ck_a, ck_b = ubx_checksum(bytes(p), RACEBOX_CLASS, RACEBOX_ID, PAYLOAD_LEN)
    pkt[86] = ck_a
    pkt[87] = ck_b
    return bytes(pkt)


def read_capture(path):
    """Yield (t, packet_bytes) and return (header, trailer, complaints)."""
    header = trailer = None
    frames = []
    complaints = []
    with open(path, "r", encoding="utf-8") as handle:
        for lineno, line in enumerate(handle, 1):
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError as exc:
                # A capture with no trailer was interrupted rather than stopped,
                # so a truncated final line is expected, not corruption.
                complaints.append(f"line {lineno}: unparseable JSON ({exc})")
                continue
            kind = obj.get("type")
            if kind == "summary":
                trailer = obj
            elif "d" in obj:
                try:
                    frames.append((obj.get("t"), base64.b64decode(obj["d"],
                                                                 validate=True)))
                except (binascii.Error, ValueError) as exc:
                    complaints.append(f"line {lineno}: bad base64 ({exc})")
            elif header is None:
                header = obj
    return header, trailer, frames, complaints


def check_header(header):
    """Warn about a capture that is not what this tool expects."""
    notes = []
    if header is None:
        return ["no header line - is this a Gnimu Monitor capture?"]
    cap = header.get("capture", header)
    if cap.get("protocol") not in (None, "racebox"):
        notes.append(f"protocol is {cap.get('protocol')!r}, not 'racebox'")
    if cap.get("frameBytes") not in (None, PACKET_LEN):
        notes.append(f"frameBytes is {cap.get('frameBytes')}, not {PACKET_LEN}")
    if cap.get("encoding") not in (None, "base64"):
        notes.append(f"encoding is {cap.get('encoding')!r}, not 'base64'")
    return notes


def report(vectors, stats, files):
    n = len(vectors)
    print(f"captures: {len(files)}")
    for f in files:
        line = f"  {f['path']}   {f['vectors']} vectors"
        if f["duration"]:
            line += f"   {f['duration']:.1f}s"
        print(line)
        for note in check_header(f["header"]) + f["complaints"][:5]:
            print(f"      ⚠️  {note}")
        if len(f["complaints"]) > 5:
            print(f"      ⚠️  ...and {len(f['complaints']) - 5} more line "
                  "problems")
        if f["trailer"] is None:
            print("      ⚠️  no trailer - interrupted, not cleanly stopped "
                  "(harmless for vectors)")
        # A file that contributes no packet nobody else already had is almost
        # certainly the same capture passed twice, or an overlapping export.
        # Harmless, but it inflates the vector count without adding coverage,
        # which is the one number here that must not be trusted blindly.
        if f["vectors"] and f["new_distinct"] == 0:
            print("      ⚠️  contributed no new distinct packets - duplicate "
                  "of another file?")

    if n == 0:
        print("\n❌ no usable frames.")
        return 1

    print(f"\nvectors: {n}   invalid frames skipped: {stats['invalid']}"
          f"   distinct packets: {stats['distinct']}")
    if stats["duration"]:
        # Summed per-file: each capture's "t" is seconds since ITS OWN start,
        # so the timestamps are not comparable across files and a global
        # min/max would be meaningless.
        print(f"combined duration: {stats['duration']:.1f}s   "
              f"mean rate: {n / stats['duration']:.2f} Hz")
    if stats["reencode_fail"]:
        print(f"\n❌ RE-ENCODE MISMATCH on {stats['reencode_fail']} of {n} "
              "vectors.")
        print("   Either the reconstruction is wrong or encode_packet() has")
        print("   drifted from sendPacket(). Do NOT use these vectors until")
        print("   this is understood - resolving it is the point of the check.")
        for line in stats["reencode_examples"]:
            print(f"     {line}")
    else:
        print(f"re-encode self-check: ✅ all {n} vectors round-trip exactly")

    c = stats["counters"]
    print("\n-- state coverage --")
    print(f"fixType (post-clamp): "
          f"{dict(sorted(c['fixType'].items()))}")
    print(f"valid-fix bit       : {dict(sorted(c['validFix'].items()))}")
    print(f"headVehValid bit    : {dict(sorted(c['headVehValid'].items()))}")
    print(f"invalid-coords flag : {dict(sorted(c['latLonInvalid'].items()))}")
    print(f"numSV range         : {stats['svmin']}..{stats['svmax']}")
    print(f"max gSpeed          : {stats['spdmax']} mm/s "
          f"({stats['spdmax'] * 0.0036:.1f} km/h)")
    # Ranges for the signed direct-copy fields. Magnitude is irrelevant to the
    # encoder - it copies these verbatim - but SIGN is not: a negative value is
    # what exercises the two's-complement path in writeLittleEndian(). Without
    # this, "no gaps" was being asserted with nothing to back it.
    rng = stats["ranges"]
    print(f"accel milli-g       : " + "  ".join(
        f"{k} {rng[k][0]}..{rng[k][1]}" for k in ("gX", "gY", "gZ")))
    print(f"gyro centi-deg/s    : " + "  ".join(
        f"{k} {rng[k][0]}..{rng[k][1]}" for k in ("rX", "rY", "rZ")))
    print(f"headMot             : {rng['headMot'][0]}..{rng['headMot'][1]} "
          f"({rng['headMot'][0] / 1e5:.1f}..{rng['headMot'][1] / 1e5:.1f} deg)")
    print(f"battery percent     : {stats['battmin']}..{stats['battmax']}"
          f"   charging bit set: {c['charging'].get(1, 0)}")
    print(f"negative nano       : {stats['negnano']}")
    print(f"negative lon / lat  : {stats['neglon']} / {stats['neglat']}")
    print(f"negative hMSL       : {stats['neghmsl']}")

    print("\n-- gaps worth closing --")
    gaps = []
    if 0 not in c["fixType"]:
        gaps.append("no NO-FIX vectors - capture from a cold boot with the app\n"
                    "  connected early; the invalid-coordinates branch is only\n"
                    "  reachable there")
    if 2 not in c["fixType"]:
        gaps.append("no 2D-fix vectors - these appear briefly during acquisition")
    if 3 not in c["fixType"]:
        gaps.append("no 3D-fix vectors - let it acquire a full fix")
    if len(c["validFix"]) < 2:
        gaps.append("valid-fix bit never varied - a cold boot produces both")
    if stats["spdmax"] < 500:
        gaps.append("never moved (all speeds < 0.5 m/s) - drive it, or the IMU\n"
                    "  and heading fields are all bench-flat and prove little")
    if stats["negnano"] == 0:
        gaps.append("no negative nano - the sub-second edge case most likely to\n"
                    "  be mishandled; synthesize it if it never shows up")
    unsigned_fields = [k for k in ("gX", "gY", "gZ", "rX", "rY", "rZ")
                       if rng[k][0] >= 0]
    if unsigned_fields:
        gaps.append(f"never negative: {', '.join(unsigned_fields)} - the "
                    "two's-complement\n  path for those fields is untested; "
                    "rotate the device through all axes")
    if stats["battmin"] == stats["battmax"] and c["charging"].get(1, 0) == 0:
        # Worth calling out precisely, because whether it MATTERS depends on a
        # design decision: if TelemetrySample carries percent and charging
        # separately (docs/multiprotocol-design.md sections 4-5), the encoder
        # has to repack the byte, and that repack would be exercised at exactly
        # one input value by this capture.
        gaps.append(f"battery byte never varied (always percent "
                    f"{stats['battmin']}, not charging) - synthetic vectors "
                    "are the\n  practical fix; re-capturing at a different "
                    "charge state is not worth a trip")
    elif c["charging"].get(1, 0) == 0:
        gaps.append("battery charging bit never set - plug in USB mid-capture,\n"
                    "  or cover it synthetically (ESP32 reports a constant)")
    if gaps:
        for gap in gaps:
            print(f"• {gap}")
    else:
        print("none - this capture covers every reachable state.")

    print("\n-- still needs synthetic vectors --")
    print("fixType 1, 4 and 5: clamped to 0 on the wire, so they are invisible")
    print("here AND (for 1 and 4) unreachable on an M10. The clamp can only be")
    print("tested with hand-written vectors.")
    if stats["neglat"] == 0:
        print("Southern-hemisphere latitude: not reachable from where you are.")

    # The design doc flags this as an untested hypothesis; a real capture
    # settles it, so say so explicitly rather than burying it in the table.
    print("\n-- docs/multiprotocol-design.md section 14 --")
    if c["headVehValid"].get(1, 0) == 0:
        print(f"headVehValid was 0 in all {n} vectors. Consistent with the")
        print("hypothesis that RaceBox bit 5 is always zero on an M10 without")
        print("dead reckoning. Not proof - a wider capture could still show it.")
    else:
        print(f"headVehValid was 1 in {c['headVehValid'][1]} vectors - the")
        print("hypothesis is disproved; bit 5 does get set on this hardware.")
    return 0


def accumulate(frames, complaints, stats, vectors, seen):
    """Fold one capture's frames into the shared stats/vectors/seen state.

    Returns how many packets this file contributed that no earlier file had.
    """
    before = len(seen)
    for _, pkt in frames:
        problem = packet_problem(pkt)
        if problem:
            stats["invalid"] += 1
            if stats["invalid"] <= 3:
                complaints.append(f"invalid frame: {problem}")
            continue
        v = reconstruct(pkt)
        got = encode_packet(v)
        if got != pkt:
            stats["reencode_fail"] += 1
            if len(stats["reencode_examples"]) < 3:
                # Report the first differing BYTE OFFSET, not a hex prefix: the
                # difference is usually well past anything a truncated dump
                # would show, and the payload offset points straight at the
                # field in sendPacket().
                i = next(k for k in range(PACKET_LEN) if got[k] != pkt[k])
                where = (f"byte {i} (payload offset {i - PAYLOAD_OFF})"
                         if i >= PAYLOAD_OFF else f"header byte {i}")
                stats["reencode_examples"].append(
                    f"iTOW={v['iTOW']} fixType={v['fixType']}: first diff at "
                    f"{where} - got 0x{got[i]:02X}, want 0x{pkt[i]:02X}")
        seen.add(bytes(pkt))
        vectors.append((v, pkt))

        c = stats["counters"]
        c["fixType"][v["fixType"]] += 1
        c["validFix"][pkt[PAYLOAD_OFF + 21] & 1] += 1
        c["headVehValid"][v["headVehValid"]] += 1
        c["latLonInvalid"][pkt[PAYLOAD_OFF + 66] & 1] += 1
        c["charging"][(v["battery"] >> 7) & 1] += 1

        sv, spd = v["numSV"], v["gSpeed"]
        pct = v["battery"] & 0x7F
        stats["svmin"] = sv if stats["svmin"] is None else min(stats["svmin"], sv)
        stats["svmax"] = sv if stats["svmax"] is None else max(stats["svmax"], sv)
        stats["battmin"] = pct if stats["battmin"] is None else min(stats["battmin"], pct)
        stats["battmax"] = pct if stats["battmax"] is None else max(stats["battmax"], pct)
        stats["spdmax"] = max(stats["spdmax"], spd)
        for key in ("gX", "gY", "gZ", "rX", "rY", "rZ", "headMot"):
            lo, hi = stats["ranges"][key]
            val = v[key]
            stats["ranges"][key] = [val if lo is None else min(lo, val),
                                    val if hi is None else max(hi, val)]
        stats["negnano"] += v["nano"] < 0
        stats["neglon"] += v["lon"] < 0
        stats["neglat"] += v["lat"] < 0
        stats["neghmsl"] += v["hMSL"] < 0
    return len(seen) - before


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("capture", nargs="+",
                    help="one or more Gnimu Monitor .jsonl captures; vectors "
                         "and coverage are merged across all of them")
    ap.add_argument("-o", "--vectors", metavar="PATH",
                    help="write GC1 test vectors here")
    args = ap.parse_args()

    counters = {k: Counter() for k in
                ("fixType", "validFix", "headVehValid", "latLonInvalid",
                 "charging")}
    stats = {
        "invalid": 0, "reencode_fail": 0, "reencode_examples": [],
        "svmin": None, "svmax": None, "spdmax": 0, "negnano": 0,
        "neglon": 0, "neglat": 0, "neghmsl": 0,
        "battmin": None, "battmax": None, "counters": counters,
        "ranges": {k: [None, None] for k in
                   ("gX", "gY", "gZ", "rX", "rY", "rZ", "headMot")},
        "duration": None, "distinct": 0,
    }
    vectors = []
    seen = set()
    files = []
    total_duration = 0.0

    for path in args.capture:
        if path.endswith(".icloud") or "/." in path:
            print(f"{path} looks like an undownloaded iCloud placeholder. Open "
                  "it in Finder first so iCloud fetches it.", file=sys.stderr)
            return 2
        try:
            header, trailer, frames, complaints = read_capture(path)
        except OSError as exc:
            print(f"cannot read {path}: {exc}", file=sys.stderr)
            return 2

        times = [t for t, _ in frames if isinstance(t, (int, float))]
        duration = (max(times) - min(times)) if len(times) >= 2 else None
        if duration:
            total_duration += duration

        before_vectors = len(vectors)
        new_distinct = accumulate(frames, complaints, stats, vectors, seen)
        files.append({
            "path": path, "header": header, "trailer": trailer,
            "complaints": complaints, "duration": duration,
            "vectors": len(vectors) - before_vectors,
            "new_distinct": new_distinct,
        })

    stats["duration"] = total_duration or None
    stats["distinct"] = len(seen)
    rc = report(vectors, stats, files)

    if args.vectors and vectors:
        if stats["reencode_fail"]:
            # Deliberately on stdout, after the report. Sending it to stderr
            # made it appear BEFORE the report under normal buffering, which
            # reads as if the refusal had no explanation.
            print("\nrefusing to write vectors while the self-check fails.")
            return 1
        with open(args.vectors, "w", encoding="utf-8") as out:
            out.write("# GC1 golden vectors from:\n")
            for f in files:
                out.write(f"#   {f['path']}  ({f['vectors']} vectors)\n")
            out.write(f"# {len(vectors)} vectors. Fields: "
                      f"{','.join(GC1_ORDER)},packetHex\n")
            out.write("# fixType is POST-CLAMP (see test/capture.py): a 0 here "
                      "may have been a true\n# 0, 1, 4 or 5. Cover those with "
                      "synthetic vectors.\n")
            for v, pkt in vectors:
                fields = ",".join(str(v[k]) for k in GC1_ORDER)
                out.write(f"GC1,{fields},{pkt.hex().upper()}\n")
        print(f"\nwrote {len(vectors)} vectors to {args.vectors}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
