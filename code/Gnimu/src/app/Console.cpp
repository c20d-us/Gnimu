#include "Console.h"

#include <Arduino.h>

#include "../platform/Clock.h"
#include "../platform/ConsolePort.h"
#include "Config.h"
#include <stdarg.h>
#include <stdio.h>

namespace gnimu {

void Console::begin() { consoleOpen(); }

ConsoleCommand Console::poll() {
  // The longest valid entry plus CR+LF. More than that waiting at once can't
  // be typed input, so it's discarded rather than read byte by byte.
  constexpr size_t kMaxReadBytes = decltype(entry_)::kCapacity + 2;
  ConsoleCommand result;
  for (size_t i = 0; i < kMaxReadBytes && Serial.available(); i++) {
    const EntryResult r = entry_.feed(static_cast<char>(Serial.read()));
    switch (r.kind) {
      case EntryKind::None:
        break;
      case EntryKind::Command:
        result.cmd = r.cmd;
        break;
      case EntryKind::Count:
        result.hasCount = true;
        result.count = r.count;
        break;
      case EntryKind::TooLarge:
        log("Console: count must be 0-%lu", static_cast<unsigned long>(config::kStatusMaxCount));
        break;
      case EntryKind::Unrecognised:
        log("Console: unrecognised input '%s%s'", entry_.text(), entry_.truncated() ? "\xE2\x80\xA6" : "");
        break;
    }
  }
  if (Serial.available()) {
    consoleDiscardInput();
    entry_.reset();
    log("Console: input too long, discarded");
  }
  return result;
}

void Console::printReport(const ReportView& view) {
  // Worst case (7 jobs, every counter at 10 digits, battery line) is ~1420
  // bytes. Static rather than on the loop task's stack, which is only a few
  // KB on the nRF.
  static char buf[1536];
  const size_t n = formatReport(buf, sizeof(buf), view);
  // A line at a time: the whole report is bigger than the USB buffer, but
  // each line fits, so emit() can send or drop it whole.
  size_t start = 0;
  for (size_t i = 0; i < n; i++) {
    if (buf[i] != '\n') continue;
    emit(buf + start, i + 1 - start);
    start = i + 1;
  }
  if (start < n) emit(buf + start, n - start);
}

void Console::printStatus(const StatusView& view) {
  // A full line (Fix+SV+tAcc+hAcc+Lat+Long+Accel+Gyro+Trim) runs ~185 bytes;
  // 160 silently truncated it before the trailing newline - exactly the kind
  // of mistake Out's truncation guard hides instead of catching loudly.
  char buf[256];
  const size_t n = formatStatus(buf, sizeof(buf), view);
  emit(buf, n);
}

void Console::flush() {
  Serial.flush();
  // flush() hands the bytes to USB/UART; the short wait lets them actually
  // leave before a reset cuts the link (Classic's 50 ms).
  delay(50);
}

void Console::log(const char* fmt, ...) {
  // Formatted into the first 160 bytes, then the line ending println used to
  // add, so the line goes out in one emit().
  char buf[162];
  va_list ap;
  va_start(ap, fmt);
  const int r = vsnprintf(buf, 160, fmt, ap);
  va_end(ap);
  size_t n = r < 0 ? 0 : (r < 160 ? static_cast<size_t>(r) : 159);
  buf[n++] = '\r';
  buf[n++] = '\n';
  emit(buf, n);
}

// Sends a line whole or drops it whole, waiting at most kConsoleWaitUs for
// room, so a host that holds the port open without reading can't stall the
// loop. Every line is under the nRF's 256-byte USB buffer, so it always fits
// once the host reads. With no host listening (no DTR on native USB) output is
// discarded uncounted, as Serial.write already did; the ESP32's UART is
// always "listening".
void Console::emit(const char* p, size_t n) {
  if (n == 0 || !Serial) return;
  // PlatformClock, not micros(): the nRF core's micros() only advances once
  // per 1 ms RTOS tick, which made the wait anywhere from 1 to 2 ms.
  const uint64_t start = PlatformClock::nowUs();
  while (static_cast<size_t>(Serial.availableForWrite()) < n) {
    if (PlatformClock::nowUs() - start >= config::kConsoleWaitUs) {
      dropped_ += n;
      return;
    }
  }
  Serial.write(reinterpret_cast<const uint8_t*>(p), n);
}

}  // namespace gnimu
