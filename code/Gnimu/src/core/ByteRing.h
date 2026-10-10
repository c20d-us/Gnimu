// Gnimu - GNSS+IMU streaming telemetry
// Copyright (C) 2026 Chris Halstead
// SPDX-License-Identifier: GPL-3.0-or-later

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <atomic>

namespace gnimu {

// Single-producer/single-consumer byte queue for handing bytes from a callback
// context (e.g. a BLE stack task on the other core) to the loop without locks.
// Exactly one context may push and exactly one may pop. Bytes that don't fit
// are dropped (newest-loses) and counted - the consumer sees a gap, never a
// torn write. Indices run free and wrap; N must be a power of two.
template <size_t N>
class ByteRing {
  static_assert(N > 0 && (N & (N - 1)) == 0, "ByteRing size must be a power of two");

 public:
  // Producer side. Returns how many bytes were queued.
  size_t push(const uint8_t *data, size_t len) {
    const uint32_t head = head_.load(std::memory_order_relaxed);
    const uint32_t tail = tail_.load(std::memory_order_acquire);
    const size_t space = N - static_cast<size_t>(head - tail);
    const size_t n = len < space ? len : space;
    for (size_t i = 0; i < n; i++) buf_[(head + i) & (N - 1)] = data[i];
    head_.store(head + static_cast<uint32_t>(n), std::memory_order_release);
    if (n < len) dropped_.fetch_add(static_cast<uint32_t>(len - n), std::memory_order_relaxed);
    pushed_.fetch_add(static_cast<uint32_t>(n), std::memory_order_relaxed);
    return n;
  }

  // Consumer side. Returns how many bytes were copied into `out`.
  size_t pop(uint8_t *out, size_t max) {
    const uint32_t tail = tail_.load(std::memory_order_relaxed);
    const uint32_t head = head_.load(std::memory_order_acquire);
    const size_t avail = static_cast<size_t>(head - tail);
    const size_t n = max < avail ? max : avail;
    for (size_t i = 0; i < n; i++) out[i] = buf_[(tail + i) & (N - 1)];
    tail_.store(tail + static_cast<uint32_t>(n), std::memory_order_release);
    return n;
  }

  uint32_t pushed() const { return pushed_.load(std::memory_order_relaxed); }
  uint32_t dropped() const { return dropped_.load(std::memory_order_relaxed); }

 private:
  uint8_t buf_[N] = {};
  std::atomic<uint32_t> head_{0};
  std::atomic<uint32_t> tail_{0};
  std::atomic<uint32_t> pushed_{0};
  std::atomic<uint32_t> dropped_{0};
};

}  // namespace gnimu
