#pragma once

#include <stddef.h>
#include <stdint.h>

namespace gnimu {

constexpr size_t decimalDigits(uint32_t v) { return v < 10 ? 1 : 1 + decimalDigits(v / 10); }

enum class EntryKind : uint8_t { None, Command, Count, TooLarge, Unrecognised };

// The console's commands, named once here where the letters are recognised:
// D Report, C Clear, V Version, L ToggleStatus. Everything downstream
// switches on these, so a new command can't go unrouted without a warning.
enum class CommandKind : uint8_t { None, Report, Clear, Version, ToggleStatus };

struct EntryResult {
  EntryKind kind = EntryKind::None;
  CommandKind cmd = CommandKind::None;  // Command only
  uint32_t count = 0;                   // Count only
};

// Collects console input into entries ended by Enter (CR or LF). An entry is
// either exactly one command letter (D, C, V, L) or a number 0..MaxCount;
// anything else is rejected whole, so "Love" or "1g6" do nothing.
template <uint32_t MaxCount>
class ConsoleEntry {
 public:
  // The longest valid entry: MaxCount's digits (a command letter is one).
  static constexpr size_t kCapacity = decimalDigits(MaxCount);
  static_assert(kCapacity <= 9, "a 10-digit count could overflow uint32_t while parsing");

  // One byte in; a non-None result when it ends an entry. An empty entry (the
  // LF of a CR+LF pair) is None.
  EntryResult feed(char c) {
    if (ended_) reset();
    if (c != '\r' && c != '\n') {
      if (c < '0' || c > '9') allDigits_ = false;
      if (len_ < kCapacity) {
        buf_[len_++] = (c < 0x20 || c > 0x7E) ? '?' : c;
      } else {
        overflow_ = true;
      }
      return {};
    }
    ended_ = true;
    buf_[len_] = '\0';
    return classify();
  }

  // Abandons the entry in progress.
  void reset() {
    len_ = 0;
    overflow_ = false;
    allDigits_ = true;
    ended_ = false;
    buf_[0] = '\0';
  }

  // The entry just ended, as far as it fit, with non-printables as '?'. Valid
  // until the next feed().
  const char* text() const { return buf_; }
  bool truncated() const { return overflow_; }

 private:
  EntryResult classify() const {
    EntryResult r;
    if (len_ == 0) return r;
    if (allDigits_) {
      // Too many digits can only mean too large; otherwise parse and compare.
      uint32_t v = 0;
      for (size_t i = 0; i < len_; i++) v = v * 10 + static_cast<uint32_t>(buf_[i] - '0');
      if (overflow_ || v > MaxCount) {
        r.kind = EntryKind::TooLarge;
      } else {
        r.kind = EntryKind::Count;
        r.count = v;
      }
      return r;
    }
    const CommandKind k = len_ == 1 && !overflow_ ? commandFor(buf_[0]) : CommandKind::None;
    r.kind = k == CommandKind::None ? EntryKind::Unrecognised : EntryKind::Command;
    r.cmd = k;
    return r;
  }

  static CommandKind commandFor(char c) {
    switch (c) {
      case 'D':
        return CommandKind::Report;
      case 'C':
        return CommandKind::Clear;
      case 'V':
        return CommandKind::Version;
      case 'L':
        return CommandKind::ToggleStatus;
      default:
        return CommandKind::None;
    }
  }

  char buf_[kCapacity + 1] = {};
  size_t len_ = 0;
  bool overflow_ = false;
  bool allDigits_ = true;
  bool ended_ = false;
};

}  // namespace gnimu
