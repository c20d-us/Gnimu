#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../code/Gnimu/src/core/ResetReason.h"

using namespace gnimu;

static const char* text(const ResetRecord& r) {
  static char buf[64];
  formatReset(buf, sizeof(buf), r);
  return buf;
}

// A restart we requested shows its reason alongside the hardware's "software".
static void testOwnRestartShowsItsReason() {
  assert(!strcmp(text(makeResetRecord(ResetCause::Software, restartCode(RestartReason::SwitchOn))),
                 "software (Power: switch on)"));
  assert(!strcmp(text(makeResetRecord(ResetCause::Software, restartCode(RestartReason::UsbRemoved))),
                 "software (Power: USB removed)"));
  assert(!strcmp(text(makeResetRecord(ResetCause::Software, restartCode(RestartReason::GnssSilent))),
                 "software (GNSS: epochs stopped)"));
}

// Nothing kept (an upload's reset, or the ESP32 core's own restart): plain
// "software", no invented reason.
static void testSoftwareResetWithoutReason() {
  const ResetRecord r = makeResetRecord(ResetCause::Software, 0);
  assert(!r.restartKnown);
  assert(!strcmp(text(r), "software"));
}

// The kept byte survives watchdog and pin resets too, so a stale reason from
// an earlier restart must not attach itself to one of those.
static void testReasonIgnoredForOtherCauses() {
  const uint8_t stale = restartCode(RestartReason::GnssSilent);
  assert(!strcmp(text(makeResetRecord(ResetCause::Watchdog, stale)), "watchdog"));
  assert(!strcmp(text(makeResetRecord(ResetCause::Pin, stale)), "reset pin"));
  assert(!strcmp(text(makeResetRecord(ResetCause::PowerOn, stale)), "power-on"));
}

// The ESP32's no-init byte is garbage after power-on; out-of-range values
// mean nothing even after a software reset.
static void testOutOfRangeCodeIgnored() {
  assert(!makeResetRecord(ResetCause::Software, kRestartReasonCount + 1).restartKnown);
  assert(!makeResetRecord(ResetCause::Software, 0xFF).restartKnown);
  // Every in-range code names a real reason.
  for (uint8_t c = 1; c <= kRestartReasonCount; c++) {
    const ResetRecord r = makeResetRecord(ResetCause::Software, c);
    assert(r.restartKnown);
    assert(strcmp(restartReasonText(r.restart), "?") != 0);
  }
}

static void testCauseTexts() {
  assert(!strcmp(text(makeResetRecord(ResetCause::PowerOnOrBrownout, 0)), "power-on or brownout"));
  assert(!strcmp(text(makeResetRecord(ResetCause::WakeUsb, 0)), "wake from System OFF (USB)"));
  assert(!strcmp(text(makeResetRecord(ResetCause::Unknown, 0)), "unknown"));
}

// Truncated, never overrun.
static void testTruncates() {
  char buf[9];
  formatReset(buf, sizeof(buf), makeResetRecord(ResetCause::Software, restartCode(RestartReason::GnssSilent)));
  assert(!strcmp(buf, "software"));
}

int main() {
  testOwnRestartShowsItsReason();
  testSoftwareResetWithoutReason();
  testReasonIgnoredForOtherCauses();
  testOutOfRangeCodeIgnored();
  testCauseTexts();
  testTruncates();
  printf("resetreason: all tests passed\n");
  return 0;
}
