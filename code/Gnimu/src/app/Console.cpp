#include "Console.h"

#include <Arduino.h>

#include "../platform/ConsolePort.h"
#include <stdarg.h>
#include <stdio.h>

namespace gnimu {

void Console::begin() { consoleOpen(); }

char Console::poll() {
  char cmd = 0;
  while (Serial.available()) {
    const int c = Serial.read();
    if (c == 'D' || c == 'C' || c == 'V') cmd = static_cast<char>(c);
  }
  return cmd;
}

void Console::printReport(const ReportView& view) {
  char buf[512];
  const size_t n = formatReport(buf, sizeof(buf), view);
  Serial.write(reinterpret_cast<const uint8_t*>(buf), n);
}

void Console::log(const char* fmt, ...) {
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.println(buf);
}

}  // namespace gnimu
