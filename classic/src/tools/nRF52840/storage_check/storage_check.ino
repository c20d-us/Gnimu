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
// PROVISIONING TOOL (also doubles as a diagnostic): QSPI flash + LittleFS
//
// REQUIRED ONE-TIME STEP before g_storage / the GNSS DB warm-start feature
// works on a given board. The main firmware (g_storage.cpp) deliberately
// NEVER formats flash on its own - a mount failure there just means
// persistent storage is unavailable for that boot (GNSS falls back to a
// cold start every time), not a prompt to silently erase whatever is on
// the chip. Formatting is this sketch's job, and only after you explicitly
// type FORMAT (Phase 2 below) - never automatic, never silent.
//
// Run this once on any new board, and again if the QSPI chip is ever
// swapped for a different part (the P25Q16H device table below would also
// need updating in that case - see g_storage.cpp's matching comment).
//
// Exercises the same Adafruit_SPIFlash + Adafruit_LittleFS stack that
// g_storage.cpp uses in the main firmware, on its own so we can validate
// the storage layer in isolation before trusting the whole GNSS DB
// persistence path to it.
//
// Adafruit_LittleFS ships two ready-made constructors: a no-arg one bound to
// INTERNAL MCU flash, and one that takes a raw `lfs_config*` for anything
// else. There's no built-in "just point me at Adafruit_SPIFlash" overload
// for EXTERNAL QSPI, so this sketch wires up that `lfs_config` itself -
// read/prog/erase/sync callbacks bound to Adafruit_SPIFlash's block-level
// API - sized correctly for the XIAO Sense's actual 2 MiB Puya P25Q16H chip.
// (Seeed's own `ExternalFileSystem` library uses the identical pattern, but
// is hard-coded for a different board's 4 MiB P25Q32H chip - copying it
// verbatim would mis-size the block count for our chip.)
//
// Adafruit's SPIFlash device database also doesn't include Puya parts, so
// flash.begin() is given an explicit device table for the P25Q16H.
//
// Phase 1 (auto-runs on boot, read-only, no side effects):
//   1. QSPI flash init  - detects the chip, reports capacity + JEDEC ID.
//   2. LittleFS mount   - READ-ONLY attempt, no format on failure. Reports
//                         whether an existing filesystem was found (and
//                         lists its contents) or not (blank chip, or a
//                         filesystem LittleFS can't read - e.g. a stale FAT
//                         experiment).
//
// Phase 2 (only after typing FORMAT + enter - destroys existing data):
//   3. Format + mount   - fresh LittleFS filesystem.
//   4. Small write/read - 100 B file, known pattern, byte-exact compare.
//   5. Overwrite        - same file rewritten with a different pattern;
//                         verifies our truncate-then-write behavior actually
//                         replaces content rather than appending.
//   6. Large write/read - 32 KB file (approximates real DBD size, catches
//                         chunking issues a 100 B test wouldn't).
//   7. Delete           - remove the test files, verify open() then fails.
//
// All test files use `/storage_test_*.bin` names to avoid any possible
// collision with firmware files that may already be on the QSPI.
//
// Requires: XIAO nRF52840 Sense; USB for serial.
// ============================================================================

#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // Serial (USB CDC) needs TinyUSB linked
#include <Adafruit_SPIFlash.h>
#include <Adafruit_LittleFS.h>

#include "storage_check_config.h"

using namespace Adafruit_LittleFS_Namespace;

// ----------------------------------------------------------------------------
// QSPI flash chip identity + LittleFS block geometry - both hardware
// descriptors, sourced from storage_check_config.h. See that file for the
// full rationale and the upstream source of these values.
// ----------------------------------------------------------------------------
static const SPIFlash_Device_t P25Q16H = {
    .total_size = STORAGE_FLASH_TOTAL_SIZE,
    .start_up_time_us = STORAGE_FLASH_START_UP_TIME_US,
    .manufacturer_id = STORAGE_FLASH_MANUFACTURER_ID,
    .memory_type = STORAGE_FLASH_MEMORY_TYPE,
    .capacity = STORAGE_FLASH_CAPACITY,
    .max_clock_speed_mhz = STORAGE_FLASH_MAX_CLOCK_SPEED_MHZ,
    .quad_enable_bit_mask = STORAGE_FLASH_QUAD_ENABLE_BIT_MASK,
    .has_sector_protection = STORAGE_FLASH_HAS_SECTOR_PROTECTION,
    .supports_fast_read = STORAGE_FLASH_SUPPORTS_FAST_READ,
    .supports_qspi = STORAGE_FLASH_SUPPORTS_QSPI,
    .supports_qspi_writes = STORAGE_FLASH_SUPPORTS_QSPI_WRITES,
    .write_status_register_split = STORAGE_FLASH_WRITE_STATUS_REGISTER_SPLIT,
    .single_status_byte = STORAGE_FLASH_SINGLE_STATUS_BYTE,
    .is_fram = STORAGE_FLASH_IS_FRAM,
};

static Adafruit_FlashTransport_QSPI flashTransport;
static Adafruit_SPIFlash flash(&flashTransport);

static __attribute__((aligned(4))) uint8_t lfsReadBuf[STORAGE_LFS_CACHE_SIZE];
static __attribute__((aligned(4))) uint8_t lfsProgBuf[STORAGE_LFS_CACHE_SIZE];
static __attribute__((
    aligned(4))) uint8_t lfsLookaheadBuf[STORAGE_LFS_LOOKAHEAD_SIZE];

static int lfsRead(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
                   void *buffer, lfs_size_t size) {
  (void)c;
  flash.readBuffer(block * STORAGE_LFS_BLOCK_SIZE + off, (uint8_t *)buffer,
                   size);
  return 0;
}

static int lfsProg(const struct lfs_config *c, lfs_block_t block, lfs_off_t off,
                   const void *buffer, lfs_size_t size) {
  (void)c;
  flash.writeBuffer(block * STORAGE_LFS_BLOCK_SIZE + off,
                    (const uint8_t *)buffer, size);
  return 0;
}

static int lfsErase(const struct lfs_config *c, lfs_block_t block) {
  (void)c;
  flash.eraseSector(block);
  flash.waitUntilReady();
  return 0;
}

static int lfsSync(const struct lfs_config *c) {
  (void)c;
  return 0;
}

static struct lfs_config lfsCfg = {
    .context = NULL,
    .read = lfsRead,
    .prog = lfsProg,
    .erase = lfsErase,
    .sync = lfsSync,
    .read_size = STORAGE_LFS_CACHE_SIZE,
    .prog_size = STORAGE_LFS_CACHE_SIZE,
    .block_size = STORAGE_LFS_BLOCK_SIZE,
    .block_count = STORAGE_LFS_BLOCK_COUNT,
    .lookahead = STORAGE_LFS_LOOKAHEAD_SIZE,
    .read_buffer = lfsReadBuf,
    .prog_buffer = lfsProgBuf,
    .lookahead_buffer = lfsLookaheadBuf,
    .file_buffer = NULL,
};

static Adafruit_LittleFS fs(&lfsCfg);

// ----------------------------------------------------------------------------
// Test-file names. Distinctive prefix + .bin so no plausible firmware
// artifact collides.
// ----------------------------------------------------------------------------
static const char *SMALL_NAME = "/storage_test_small.bin";
static const char *LARGE_NAME = "/storage_test_large.bin";
static const size_t SMALL_LEN = 100;
static const size_t LARGE_LEN = 32 * 1024;

// Running counts.
static int testsRun = 0;
static int testsPassed = 0;

// ----------------------------------------------------------------------------
// Helpers.
// ----------------------------------------------------------------------------

// Deterministic pattern generator - each byte a linear function of its index
// so a mismatch tells us exactly where corruption is.
static uint8_t patternByte(size_t i, uint8_t seed) {
  return (uint8_t)((i * 13u + (uint32_t)seed * 7u + 5u) & 0xFF);
}

static void fillPattern(uint8_t *buf, size_t len, uint8_t seed) {
  for (size_t i = 0; i < len; i++) {
    buf[i] = patternByte(i, seed);
  }
}

// Returns -1 on match, or the first mismatching offset.
static int findFirstMismatch(const uint8_t *buf, size_t len, uint8_t seed) {
  for (size_t i = 0; i < len; i++) {
    if (buf[i] != patternByte(i, seed)) {
      return (int)i;
    }
  }
  return -1;
}

static void result(const char *name, bool ok, const char *detail = nullptr) {
  testsRun++;
  if (ok) {
    testsPassed++;
    Serial.printf("✅ PASS  %s%s%s\n", name, detail ? " - " : "",
                  detail ? detail : "");
  } else {
    Serial.printf("❌ FAIL  %s%s%s\n", name, detail ? " - " : "",
                  detail ? detail : "");
  }
}

// ----------------------------------------------------------------------------
// Test bodies.
// ----------------------------------------------------------------------------

static bool testFlashInit() {
  if (!flash.begin(&P25Q16H, 1)) {
    result("flash.begin()", false, "QSPI transport failed");
    return false;
  }
  const uint32_t jedec = flash.getJEDECID();
  const uint32_t sizeBytes = flash.size();
  char detail[80];
  snprintf(detail, sizeof(detail), "JEDEC=0x%06lx  size=%lu bytes (%lu KB)",
           (unsigned long)jedec, (unsigned long)sizeBytes,
           (unsigned long)(sizeBytes / 1024));
  result("flash.begin()", true, detail);
  return true;
}

// Phase 1: READ-ONLY mount attempt. No format-on-failure here - Phase 1's
// whole point is to report what's actually on the chip without touching it.
// Returns true if an existing filesystem mounted; false means either blank
// flash or an incompatible/corrupt filesystem (both look the same from out
// here - lfs_mount() doesn't distinguish "never formatted" from "formatted
// as something else"), which Phase 2's format-and-retry will resolve.
static bool tryMountReadOnly() {
  if (fs.begin()) {
    result("fs.begin() [read-only attempt]", true,
           "existing filesystem mounted");
    return true;
  }
  result("fs.begin() [read-only attempt]", false,
         "no mountable filesystem (blank chip, or formatted as something "
         "else - e.g. a prior FAT/SdFat experiment)");
  return false;
}

// Phase 2: format-and-retry. Only reached after explicit user confirmation.
static bool mountWithFormat() {
  if (fs.begin()) {
    result("fs.begin()", true, "already mounted");
    return true;
  }
  Serial.println("ℹ️  Formatting...");
  if (!fs.format()) {
    result("fs.begin()", false, "format failed");
    return false;
  }
  if (!fs.begin()) {
    result("fs.begin()", false, "mount failed even after format");
    return false;
  }
  result("fs.begin()", true, "mounted after format");
  return true;
}

static void testListRoot() {
  Serial.println("📂 Root directory listing (informational):");
  Adafruit_LittleFS_Namespace::File root = fs.open("/", FILE_O_READ);
  if (!root || !root.isDirectory()) {
    Serial.println("     (could not open root)");
    return;
  }
  Adafruit_LittleFS_Namespace::File entry = root.openNextFile(FILE_O_READ);
  int count = 0;
  while (entry) {
    Serial.printf("     %-40s  %8lu bytes%s\n", entry.name(),
                  (unsigned long)entry.size(),
                  entry.isDirectory() ? "  <DIR>" : "");
    entry.close();
    count++;
    entry = root.openNextFile(FILE_O_READ);
  }
  if (count == 0) {
    Serial.println("     (empty)");
  }
  root.close();
}

// Write a file with the given seed, read it back, verify byte-exact.
static bool testWriteRead(const char *name, size_t len, uint8_t seed,
                          const char *label) {
  uint8_t *buf = (uint8_t *)malloc(len);
  if (!buf) {
    result(label, false, "malloc failed");
    return false;
  }
  fillPattern(buf, len, seed);

  fs.remove(name); // clean overwrite, not append
  Adafruit_LittleFS_Namespace::File f = fs.open(name, FILE_O_WRITE);
  if (!f) {
    result(label, false, "open for write failed");
    free(buf);
    return false;
  }
  // write(uint8_t const*, size_t) -> size_t (unsigned, 0 on failure).
  const size_t written = f.write(buf, len);
  f.close();
  if (written != len) {
    char detail[80];
    snprintf(detail, sizeof(detail), "wrote %u/%u bytes", (unsigned)written,
             (unsigned)len);
    result(label, false, detail);
    free(buf);
    return false;
  }

  memset(buf, 0, len); // ensure we're reading flash, not stale RAM

  Adafruit_LittleFS_Namespace::File fr = fs.open(name, FILE_O_READ);
  if (!fr) {
    result(label, false, "open for read failed");
    free(buf);
    return false;
  }
  // read(void*, uint16_t) -> int; nbyte capped at 65535, fine for our test
  // sizes (100 B / 512 B / 32 KB all fit under that in one call).
  const int readN = fr.read(buf, (uint16_t)len);
  fr.close();
  if (readN < 0 || (size_t)readN != len) {
    char detail[80];
    snprintf(detail, sizeof(detail), "read %d/%u bytes", readN,
             (unsigned)len);
    result(label, false, detail);
    free(buf);
    return false;
  }

  const int mismatch = findFirstMismatch(buf, len, seed);
  if (mismatch >= 0) {
    char detail[80];
    snprintf(detail, sizeof(detail), "byte %d differs (got 0x%02x, want 0x%02x)",
             mismatch, buf[mismatch], patternByte(mismatch, seed));
    result(label, false, detail);
    free(buf);
    return false;
  }

  char detail[64];
  snprintf(detail, sizeof(detail), "%u bytes verified", (unsigned)len);
  result(label, true, detail);
  free(buf);
  return true;
}

// Verifies overwrite semantics: after two writes with different seeds, only
// the second seed's pattern must remain.
static bool testOverwrite(const char *name) {
  const uint8_t seedA = 0x11;
  const uint8_t seedB = 0x99;
  const size_t len = 512;
  uint8_t *buf = (uint8_t *)malloc(len);
  if (!buf) {
    result("overwrite", false, "malloc failed");
    return false;
  }

  fillPattern(buf, len, seedA);
  fs.remove(name);
  Adafruit_LittleFS_Namespace::File f1 = fs.open(name, FILE_O_WRITE);
  if (!f1) {
    result("overwrite", false, "1st open failed");
    free(buf);
    return false;
  }
  f1.write(buf, len);
  f1.close();

  fillPattern(buf, len, seedB);
  fs.remove(name);
  Adafruit_LittleFS_Namespace::File f2 = fs.open(name, FILE_O_WRITE);
  if (!f2) {
    result("overwrite", false, "2nd open failed");
    free(buf);
    return false;
  }
  f2.write(buf, len);
  f2.close();

  memset(buf, 0, len);
  Adafruit_LittleFS_Namespace::File fr = fs.open(name, FILE_O_READ);
  if (!fr) {
    result("overwrite", false, "read open failed");
    free(buf);
    return false;
  }
  const int readN = fr.read(buf, (uint16_t)len);
  const uint32_t sizeOnDisk = fr.size();
  fr.close();

  if (readN < 0 || (size_t)readN != len || sizeOnDisk != len) {
    char detail[96];
    snprintf(detail, sizeof(detail), "file size %lu (want %u), read %d",
             (unsigned long)sizeOnDisk, (unsigned)len, readN);
    result("overwrite", false, detail);
    free(buf);
    return false;
  }
  const int mismatch = findFirstMismatch(buf, len, seedB);
  if (mismatch >= 0) {
    result("overwrite", false, "read pattern != seedB (append/stale?)");
    free(buf);
    return false;
  }

  result("overwrite", true, "seedB fully replaced seedA");
  free(buf);
  return true;
}

static bool testDelete(const char *name) {
  const bool removed = fs.remove(name);
  Adafruit_LittleFS_Namespace::File f = fs.open(name, FILE_O_READ);
  const bool stillThere = (bool)f;
  if (f) {
    f.close();
  }
  if (!removed || stillThere) {
    char detail[80];
    snprintf(detail, sizeof(detail), "remove()=%d, open-after=%d", removed,
             stillThere);
    result("delete", false, detail);
    return false;
  }
  result("delete", true, name);
  return true;
}

// True once Phase 1 has established a baseline (whether or not it mounted).
static bool phase1Done = false;
// True once an existing filesystem was found mounted during Phase 1 - drives
// the extra confirmation wording before Phase 2 (format would destroy it).
static bool hadExistingFs = false;

static void runPhase1() {
  Serial.println("=== PHASE 1: read-only reconnaissance ===");
  Serial.println("No writes, no format - just reporting what's on the chip.");
  Serial.println();

  if (!testFlashInit()) {
    Serial.println("\nPhase 1 stops here: no usable QSPI flash.");
    phase1Done = true;
    return;
  }

  hadExistingFs = tryMountReadOnly();
  if (hadExistingFs) {
    Serial.println();
    testListRoot();
  }

  Serial.println();
  Serial.printf("Phase 1 summary: flash detected, filesystem %s.\n",
                hadExistingFs ? "MOUNTED (contains the listing above)"
                              : "NOT MOUNTED (blank or incompatible)");
  Serial.println();
  if (hadExistingFs) {
    Serial.println("Type FORMAT and press enter to ERASE this filesystem and");
    Serial.println("run the read/write/overwrite/delete test suite (Phase 2).");
  } else {
    Serial.println("Type FORMAT and press enter to create a fresh filesystem");
    Serial.println("and run the read/write/overwrite/delete test suite (Phase 2).");
  }
  phase1Done = true;
}

static void runPhase2() {
  Serial.println();
  Serial.println("=== PHASE 2: format + read/write/overwrite/delete tests ===");
  Serial.println();

  if (!mountWithFormat()) {
    Serial.println("\nAborting: could not mount a filesystem.");
    return;
  }

  testWriteRead(SMALL_NAME, SMALL_LEN, 0x42, "small write/read (100 B)");
  testOverwrite(SMALL_NAME);
  testWriteRead(LARGE_NAME, LARGE_LEN, 0xA5, "large write/read (32 KB)");
  testDelete(SMALL_NAME);
  testDelete(LARGE_NAME);

  Serial.println();
  if (testsPassed == testsRun) {
    Serial.printf("🎉 ALL %d/%d TESTS PASSED\n", testsPassed, testsRun);
  } else {
    Serial.printf("⚠️  %d/%d TESTS PASSED  (%d failed)\n", testsPassed, testsRun,
                  testsRun - testsPassed);
  }
  Serial.println("Idle. Reset to re-run.");
}

// ----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {
  }

  Serial.println("\n=== Gnimu - storage (QSPI+LittleFS) diagnostic ===");
  Serial.println("Uses the same stack as g_storage.cpp in the main firmware.");
  Serial.println();

  runPhase1();
}

void loop() {
  if (!phase1Done || !Serial.available()) {
    delay(50);
    return;
  }
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line == "FORMAT") {
    runPhase2();
    phase1Done = false; // idle after Phase 2 completes; reset to re-run
  } else if (line.length() > 0) {
    Serial.printf("(ignored input \"%s\" - type FORMAT to proceed, or reset "
                  "to re-run Phase 1)\n",
                  line.c_str());
  }
}
