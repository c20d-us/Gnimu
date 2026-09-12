// Gnimu - RaceBox Mini-compatible GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

// ============================================================================
// Host harness - runs the REAL firmware encoder against the golden vectors.
//
// This is the only thing in the project that tests the C++ that actually
// ships. capture.py's encode_packet() is a Python MODEL of the old encoder;
// its job ended once it produced the vectors. This compiles
// g_proto_racebox.cpp itself and checks it against them.
//
// It is possible only because g_protocol.h and g_proto_racebox.* depend on
// nothing but fixed-width integer types - no config.h, no Arduino.h. That
// constraint is deliberate; see g_protocol.h.
//
// WHAT IT COVERS
//   raceboxEncode(TelemetrySample) -> 88 bytes, for every captured and
//   synthetic vector.
//
// WHAT IT DOES NOT
//   buildSample(), the UBX_NAV_PVT_data_t -> TelemetrySample copy. That is
//   test/telemetry/telemetry_harness.cpp, which runs these same vectors through
//   the real g_telemetry.cpp (./test/run_telemetry_harness.sh). This harness
//   stays the encoder-only check: no config.h, no fakes, one variant.
//
// BUILD & RUN
//   ./test/run_harness.sh
// ============================================================================

#include "gc1.h" // the vector format and loader, shared with the telemetry harness

namespace {

// The encoder emits through a callback, so capture what it produced.
uint8_t g_emitted[RACEBOX_PACKET_LEN];
size_t g_emittedLen = 0;
uint8_t g_emittedChannel = 0xFF;
int g_emitCount = 0;

// Always accepts, and that is honest rather than a placeholder: a test sink
// genuinely cannot fail. Returning a constant true from a real transport would
// be the anti-pattern TelemetryEmit warns about; here there is nothing to
// detect.
bool captureEmit(uint8_t channel, const uint8_t *data, size_t len) {
  g_emitCount++;
  g_emittedChannel = channel;
  g_emittedLen = len;
  if (len <= sizeof(g_emitted)) {
    memcpy(g_emitted, data, len);
  }
  return true;
}

} // namespace

int main(int argc, char **argv) {
  std::vector<Vector> vectors;
  if (!loadVectors(argc, argv, 1, vectors)) {
    return 2;
  }

  int failures = 0, shown = 0;
  for (const Vector &v : vectors) {
    g_emitCount = 0;
    g_emittedLen = 0;
    g_emittedChannel = 0xFF;
    memset(g_emitted, 0, sizeof(g_emitted));

    raceboxEncode(v.sample, captureEmit);

    const char *problem = nullptr;
    size_t diffAt = 0;
    if (g_emitCount != 1) {
      problem = "wrong number of frames emitted";
    } else if (g_emittedChannel != TELEMETRY_CHANNEL_PRIMARY) {
      problem = "wrong channel";
    } else if (g_emittedLen != RACEBOX_PACKET_LEN) {
      problem = "wrong frame length";
    } else {
      for (size_t i = 0; i < RACEBOX_PACKET_LEN; i++) {
        if (g_emitted[i] != v.expected[i]) {
          problem = "byte mismatch";
          diffAt = i;
          break;
        }
      }
    }
    if (!problem) {
      continue;
    }

    failures++;
    if (shown++ < 10) {
      printf("\n❌ %s\n", v.source.c_str());
      if (!v.label.empty()) {
        printf("   testing: %s\n", v.label.c_str());
      }
      if (strcmp(problem, "byte mismatch") == 0) {
        // Payload offset is what points straight at the field in
        // g_proto_racebox.cpp; raw byte index alone means a mental subtraction
        // on every failure.
        printf("   first diff at byte %zu", diffAt);
        if (diffAt >= 6) {
          printf(" (payload offset %zu)", diffAt - 6);
        }
        printf(" - got 0x%02X, expected 0x%02X\n", g_emitted[diffAt],
               v.expected[diffAt]);
      } else {
        printf("   %s (frames=%d channel=%u len=%zu)\n", problem, g_emitCount,
               g_emittedChannel, g_emittedLen);
      }
    }
  }

  printf("\n%zu vectors, %d failures\n", vectors.size(), failures);
  if (failures > shown) {
    printf("(%d further failures not shown)\n", failures - shown);
  }
  if (failures == 0) {
    printf("✅ the firmware encoder reproduces every golden vector exactly\n");
  }
  return failures == 0 ? 0 : 1;
}
