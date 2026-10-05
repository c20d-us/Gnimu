#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../code/Gnimu/src/core/ByteRing.h"

using namespace gnimu;

static void testPushPopInOrder() {
  ByteRing<8> r;
  const uint8_t in[] = {1, 2, 3, 4, 5};
  assert(r.push(in, 5) == 5);
  uint8_t out[8] = {};
  assert(r.pop(out, 3) == 3);
  assert(out[0] == 1 && out[1] == 2 && out[2] == 3);
  assert(r.pop(out, 8) == 2);
  assert(out[0] == 4 && out[1] == 5);
  assert(r.pop(out, 8) == 0);
}

static void testOverflowDropsNewestAndCounts() {
  ByteRing<4> r;
  const uint8_t in[] = {1, 2, 3, 4, 5, 6};
  assert(r.push(in, 6) == 4);
  assert(r.dropped() == 2 && r.pushed() == 4);
  uint8_t out[8] = {};
  assert(r.pop(out, 8) == 4);
  assert(out[0] == 1 && out[3] == 4);  // the oldest bytes survived intact
}

// Indices are free-running: many fill/drain cycles must keep wrapping the
// buffer correctly, with no off-by-one at the wrap point.
static void testWrapsAcrossManyCycles() {
  ByteRing<8> r;
  uint8_t next = 0, expect = 0;
  for (int cycle = 0; cycle < 1000; cycle++) {
    uint8_t in[5];
    for (int i = 0; i < 5; i++) in[i] = next++;
    assert(r.push(in, 5) == 5);
    uint8_t out[5];
    assert(r.pop(out, 5) == 5);
    for (int i = 0; i < 5; i++) assert(out[i] == expect++);
  }
  assert(r.dropped() == 0);
}

int main() {
  testPushPopInOrder();
  testOverflowDropsNewestAndCounts();
  testWrapsAcrossManyCycles();
  puts("bytering: all tests passed");
  return 0;
}
