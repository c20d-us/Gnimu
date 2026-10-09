#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../code/Gnimu/src/core/GnssNames.h"

using namespace gnimu;

static void testFixTypes() {
  const char* names[] = {"no fix", "dead reckoning", "2D fix", "3D fix", "GNSS + dead reckoning",
                         "time only"};
  const char* shorts[] = {"none", "DR", "2D", "3D", "GNSS+DR", "time"};
  for (uint8_t i = 0; i < 6; i++) {
    assert(strcmp(fixTypeName(i), names[i]) == 0);
    assert(strcmp(fixTypeShortName(i), shorts[i]) == 0);
  }
  assert(strcmp(fixTypeName(6), "unknown fix") == 0);
  assert(strcmp(fixTypeShortName(255), "?") == 0);
}

static void testDynamicModels() {
  assert(strcmp(dynamicModelName(0), "portable") == 0);
  assert(strcmp(dynamicModelName(1), "unknown model") == 0);  // reserved
  assert(strcmp(dynamicModelName(4), "automotive") == 0);
  assert(strcmp(dynamicModelName(8), "airborne <4g") == 0);
  assert(strcmp(dynamicModelName(12), "e-scooter") == 0);
  assert(strcmp(dynamicModelName(13), "unknown model") == 0);
}

static void testAccuracy() {
  char buf[16];
  formatAccuracy(buf, sizeof(buf), 529, "mm");
  assert(strcmp(buf, "529mm") == 0);
  formatAccuracy(buf, sizeof(buf), UINT32_MAX - 1, "ns");
  assert(strcmp(buf, "4294967294ns") == 0);
  formatAccuracy(buf, sizeof(buf), UINT32_MAX, "ns");
  assert(strcmp(buf, "unknown") == 0);
}

int main() {
  testFixTypes();
  testDynamicModels();
  testAccuracy();
  puts("gnssnames: all tests passed");
  return 0;
}
