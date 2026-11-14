// Gnimu protocol - one telemetry frame for the Gnimu family
// Copyright 2026 Chris Halstead
// SPDX-License-Identifier: Apache-2.0

// Reference encoder and decoder for the Gnimu frame, v1. Plain C++11 with no
// Arduino or platform dependencies, so it builds in firmware and in host tests.
// The layout itself (offsets, types, scales) is in layout_v1.h, which is
// generated from spec/frame-v1.json.

#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "layout_v1.h"

namespace gnimu_frame {
namespace v1 {

inline const FieldInfo &info(Field f) { return kFields[static_cast<uint8_t>(f)]; }

// Capability bits for the header: the blocks the device fills, whatever it has
// right now (the blocks' valid masks say that).
constexpr uint8_t kCapGnss = 1;
constexpr uint8_t kCapImu = 2;
constexpr uint8_t kCapVehicle = 4;
constexpr uint8_t kCapPower = 8;

namespace detail {

inline void putLe(uint8_t *p, uint64_t v, size_t bytes) {
  for (size_t i = 0; i < bytes; i++) p[i] = static_cast<uint8_t>(v >> (8 * i));
}

inline uint64_t getLe(const uint8_t *p, size_t bytes) {
  uint64_t v = 0;
  for (size_t i = 0; i < bytes; i++) v |= static_cast<uint64_t>(p[i]) << (8 * i);
  return v;
}

inline size_t width(Type t) {
  switch (t) {
    case Type::U8: return 1;
    case Type::I16:
    case Type::U16: return 2;
    case Type::I32: return 4;
    case Type::F64: return 8;
  }
  return 0;
}

// A field's valid bit: bit n of its block's little-endian mask is bit n % 8
// of the mask's byte n / 8.
inline void setValid(uint8_t *frame, const FieldInfo &fi) {
  uint8_t &b = frame[fi.maskOffset + fi.bit / 8];
  b = static_cast<uint8_t>(b | (1u << (fi.bit % 8)));
}

inline bool isValid(const uint8_t *frame, const FieldInfo &fi) {
  return ((frame[fi.maskOffset + fi.bit / 8] >> (fi.bit % 8)) & 1u) != 0;
}

inline double minOf(Type t) {
  switch (t) {
    case Type::I16: return -32768.0;
    case Type::I32: return -2147483648.0;
    default: return 0.0;
  }
}

inline double maxOf(Type t) {
  switch (t) {
    case Type::U8: return 255.0;
    case Type::I16: return 32767.0;
    case Type::U16: return 65535.0;
    case Type::I32: return 2147483647.0;
    default: return 0.0;
  }
}

// Half away from zero, matching the Python reference encoder.
inline double roundHalfAway(double x) { return x >= 0 ? floor(x + 0.5) : -floor(-x + 0.5); }

}  // namespace detail

// Builds one frame. Call begin(), set the fields the device has, then finish().
// Fields that aren't set are sent as 0 with their valid bit clear.
class Encoder {
 public:
  void begin(int32_t deviceTimeMs, uint8_t sequence, uint8_t capabilities, uint8_t state) {
    memset(bytes_, 0, sizeof(bytes_));
    detail::putLe(bytes_ + kDeviceTimeMsOffset, static_cast<uint32_t>(deviceTimeMs), 4);
    bytes_[kSequenceOffset] = sequence;
    bytes_[kSpecVersionOffset] = kSpecVersion;
    bytes_[kCapabilitiesOffset] = capabilities;
    bytes_[kStateOffset] = state;
  }

  // Sets a field from a value in the units the spec lists: multiplied by the
  // field's scale, rounded half away from zero, and saturated to its type.
  // Aux fields have no fixed scale, so the value is taken as raw.
  void set(Field f, double value) {
    const FieldInfo &fi = info(f);
    if (fi.type == Type::F64) {
      uint64_t u;
      memcpy(&u, &value, sizeof(u));
      detail::putLe(bytes_ + fi.offset, u, 8);
    } else {
      double raw = detail::roundHalfAway(fi.scale ? value * fi.scale : value);
      if (raw < detail::minOf(fi.type)) raw = detail::minOf(fi.type);
      if (raw > detail::maxOf(fi.type)) raw = detail::maxOf(fi.type);
      const int64_t r = static_cast<int64_t>(raw);
      detail::putLe(bytes_ + fi.offset, static_cast<uint64_t>(r), detail::width(fi.type));
    }
    detail::setValid(bytes_, fi);
  }

  // Returns the finished frame, kFrameBytes long. The valid masks are written
  // as fields are set, so there's nothing left to do; this is the read point.
  const uint8_t *finish() const { return bytes_; }

 private:
  uint8_t bytes_[kFrameBytes];
};

// Reads fields from a received frame. The caller checks the length first.
class Decoder {
 public:
  explicit Decoder(const uint8_t *frame) : p_(frame) {}

  int32_t deviceTimeMs() const {
    return static_cast<int32_t>(static_cast<uint32_t>(detail::getLe(p_ + kDeviceTimeMsOffset, 4)));
  }
  uint8_t sequence() const { return p_[kSequenceOffset]; }
  uint8_t specVersion() const { return p_[kSpecVersionOffset]; }
  uint8_t capabilities() const { return p_[kCapabilitiesOffset]; }
  uint8_t state() const { return p_[kStateOffset]; }
  bool valid(Field f) const { return detail::isValid(p_, info(f)); }

  // The field in the spec's units (raw for aux fields). Only meaningful when
  // valid(f); an invalid field reads 0.
  double value(Field f) const {
    const FieldInfo &fi = info(f);
    const uint64_t u = detail::getLe(p_ + fi.offset, detail::width(fi.type));
    double raw;
    switch (fi.type) {
      case Type::F64: {
        double d;
        memcpy(&d, &u, sizeof(d));
        return d;
      }
      case Type::I16: raw = static_cast<int16_t>(static_cast<uint16_t>(u)); break;
      case Type::I32: raw = static_cast<int32_t>(static_cast<uint32_t>(u)); break;
      default: raw = static_cast<double>(u); break;
    }
    return fi.scale ? raw / fi.scale : raw;
  }

 private:
  const uint8_t *p_;
};

}  // namespace v1
}  // namespace gnimu_frame
