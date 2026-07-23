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

#pragma once

// ============================================================================
// storage_check hardware config - QSPI flash chip identity + LittleFS block
// geometry.
//
// NOT TUNABLE. This is a hardware descriptor for the specific QSPI flash
// chip on the XIAO Sense (Puya P25Q16H) - not a value to adjust for taste.
// Only change these if a future hardware revision uses a different flash
// chip; if you do, also update the matching section in the main firmware's
// src/nRF52840/Gnimu/config.h ("STORAGE (QSPI FLASH)"). This file intentionally
// duplicates rather than shares that config - diagnostics stay isolated
// from firmware by design (see gp_storage.cpp's module comment) - but the
// macro names match 1:1 so the two are easy to diff when updating for a
// new chip.
//
// Adafruit_LittleFS's no-arg constructor only targets internal MCU flash;
// external QSPI needs a hand-wired lfs_config bound to Adafruit_SPIFlash's
// block API (see storage_check.ino), which is what these values feed.
// Adafruit's built-in flash-device database doesn't include Puya parts
// (added upstream after the version of Adafruit_SPIFlash bundled with this
// Arduino core was cut), so the device table below is supplied explicitly
// to flash.begin(). Values transcribed verbatim from upstream's entry
// (adafruit/Adafruit_SPIFlash src/flash_devices.h, commit a131d04).
// ============================================================================

// --- Flash chip identity (Puya P25Q16H, JEDEC 0x85 0x60 0x15, 2 MiB) ---
#define STORAGE_FLASH_TOTAL_SIZE (1UL << 21) // 2 MiB
#define STORAGE_FLASH_START_UP_TIME_US 5000
#define STORAGE_FLASH_MANUFACTURER_ID 0x85
#define STORAGE_FLASH_MEMORY_TYPE 0x60
#define STORAGE_FLASH_CAPACITY 0x15
#define STORAGE_FLASH_MAX_CLOCK_SPEED_MHZ 104
#define STORAGE_FLASH_QUAD_ENABLE_BIT_MASK 0x02
#define STORAGE_FLASH_HAS_SECTOR_PROTECTION false
#define STORAGE_FLASH_SUPPORTS_FAST_READ true
#define STORAGE_FLASH_SUPPORTS_QSPI true
#define STORAGE_FLASH_SUPPORTS_QSPI_WRITES true
#define STORAGE_FLASH_WRITE_STATUS_REGISTER_SPLIT false
#define STORAGE_FLASH_SINGLE_STATUS_BYTE false
#define STORAGE_FLASH_IS_FRAM false

// --- LittleFS block-device geometry ---
// 4 KiB erase sectors are standard for this chip family. Cache/lookahead
// sizes match Seeed's own ExternalFileSystem library pattern (which is
// hard-coded for a different board's 4 MiB chip - block_count here is
// sized correctly for this chip's real 2 MiB instead).
#define STORAGE_LFS_BLOCK_SIZE 4096
#define STORAGE_LFS_BLOCK_COUNT (STORAGE_FLASH_TOTAL_SIZE / STORAGE_LFS_BLOCK_SIZE) // 512
#define STORAGE_LFS_CACHE_SIZE 256     // NOR page-program size
#define STORAGE_LFS_LOOKAHEAD_SIZE 128 // bytes; >= block_count/8
