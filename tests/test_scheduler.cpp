#include <assert.h>
#include <stdio.h>

#include "../code/Gnimu/src/core/Scheduler.h"

using namespace gnimu;

struct FakeClock {
  static uint64_t now;
  static uint64_t nowUs() { return now; }
};
uint64_t FakeClock::now = 0;

struct Counter {
  int calls = 0;
  uint64_t cost = 0;  // simulated run time, advances the fake clock
};

static void runCounter(void* ctx) {
  Counter* c = static_cast<Counter*>(ctx);
  c->calls++;
  FakeClock::now += c->cost;
}

static void reset() { FakeClock::now = 1000; }

static void testFixedRateNoDrift() {
  reset();
  Counter c;
  const Job jobs[] = {{"a", 10000, 0, runCounter, &c}};
  Scheduler<1, FakeClock> s(jobs);
  s.start();
  uint64_t next = s.tick();
  assert(c.calls == 1 && next == 1000 + 10000);
  // Wake 3 ms late each time: the schedule must stay on the 10 ms grid.
  for (int i = 1; i <= 5; i++) {
    FakeClock::now = next + 3000;
    next = s.tick();
    assert(c.calls == 1 + i);
    assert(next == 1000 + 10000ull * (i + 1));
  }
  assert(s.stats(0).skipped == 0);
}

static void testNotRunEarly() {
  reset();
  Counter c;
  const Job jobs[] = {{"a", 10000, 0, runCounter, &c}};
  Scheduler<1, FakeClock> s(jobs);
  s.start();
  s.tick();
  FakeClock::now += 9999;
  s.tick();
  assert(c.calls == 1);
}

static void testSkipAhead() {
  reset();
  Counter c;
  const Job jobs[] = {{"a", 10000, 0, runCounter, &c}};
  Scheduler<1, FakeClock> s(jobs);
  s.start();
  s.tick();
  FakeClock::now = 1000 + 10000 + 35000;  // 3 whole periods late
  const uint64_t next = s.tick();
  assert(c.calls == 2);                       // one catch-up run, not four
  assert(s.stats(0).skipped == 3);
  assert(next == 1000 + 10000 * 5);           // back on the grid
}

static void testBudgetAndStats() {
  reset();
  Counter fast, slow;
  fast.cost = 100;
  slow.cost = 5000;
  const Job jobs[] = {{"fast", 1000, 500, runCounter, &fast},
                      {"slow", 10000, 2000, runCounter, &slow}};
  Scheduler<2, FakeClock> s(jobs);
  s.start();
  s.tick();
  assert(s.stats(0).runs == 1 && s.stats(0).overruns == 0 && s.stats(0).maxUs == 100);
  assert(s.stats(1).runs == 1 && s.stats(1).overruns == 1 && s.stats(1).maxUs == 5000);
  assert(s.stats(1).totalUs == 5000);
  s.clearStats();
  assert(s.stats(0).runs == 0 && s.stats(1).maxUs == 0);
}

static void testReturnsEarliestDeadline() {
  reset();
  Counter a, b;
  const Job jobs[] = {{"a", 10000, 0, runCounter, &a}, {"b", 3000, 0, runCounter, &b}};
  Scheduler<2, FakeClock> s(jobs);
  s.start();
  assert(s.tick() == 1000 + 3000);
}

static void testDisabledJobNeverRuns() {
  reset();
  Counter a, b;
  const Job jobs[] = {{"a", 1000, 0, runCounter, &a}, {"b", 1000, 0, runCounter, &b}};
  Scheduler<2, FakeClock> s(jobs);
  s.setEnabled(1, false);
  s.start();
  for (int i = 0; i < 5; i++) {
    s.tick();
    FakeClock::now += 1000;
  }
  assert(a.calls == 5 && b.calls == 0);
  assert(s.stats(1).runs == 0 && s.stats(1).skipped == 0);
}

// The case that matters: a disabled job's deadline is frozen in the past, so
// if it counted toward the returned deadline the caller would never sleep.
static void testDisabledJobIgnoredForDeadline() {
  reset();
  Counter a, b;
  const Job jobs[] = {{"a", 10000, 0, runCounter, &a}, {"b", 1000, 0, runCounter, &b}};
  Scheduler<2, FakeClock> s(jobs);
  s.start();
  s.setEnabled(1, false);
  FakeClock::now += 50000;
  const uint64_t next = s.tick();
  assert(next > FakeClock::now);
  assert(next == 1000 + 10000 * 6);
}

static void testReenableRestartsGridWithoutSkips() {
  reset();
  Counter a, b;
  const Job jobs[] = {{"a", 1000, 0, runCounter, &a}, {"b", 1000, 0, runCounter, &b}};
  Scheduler<2, FakeClock> s(jobs);
  s.start();
  s.tick();
  s.setEnabled(1, false);
  FakeClock::now += 100000;  // 100 of b's periods pass while it's off
  s.setEnabled(1, true);
  s.tick();
  assert(b.calls == 2);
  assert(s.stats(1).skipped == 0);
  assert(s.enabled(1));
}

// Member-function jobs: a JobSpec table bound to one object.
struct Owner {
  int a = 0;
  int b = 0;
  void runA() { a++; }
  void runB() { b++; }
};

constexpr JobSpec<Owner> kGoodSpecs[] = {
    {0, "a", 1000, 0, &Owner::runA},
    {1, "b", 3000, 500, &Owner::runB},
};
static_assert(jobSpecsValid(kGoodSpecs), "a well-formed table passes");

constexpr JobSpec<Owner> kSwappedSpecs[] = {
    {1, "b", 3000, 0, &Owner::runB},
    {0, "a", 1000, 0, &Owner::runA},
};
static_assert(!jobSpecsValid(kSwappedSpecs), "entries out of id order fail");

constexpr JobSpec<Owner> kZeroPeriodSpecs[] = {
    {0, "a", 1000, 0, &Owner::runA},
    {1, "b", 0, 0, &Owner::runB},
};
static_assert(!jobSpecsValid(kZeroPeriodSpecs), "a zero period fails");

constexpr JobSpec<Owner> kUnnamedSpecs[] = {
    {0, nullptr, 1000, 0, &Owner::runA},
};
static_assert(!jobSpecsValid(kUnnamedSpecs), "a missing name fails");

static void testMemberJobsDispatchToTheRightMember() {
  reset();
  Owner o;
  MemberJobs<Owner, 2> jobs(&o, kGoodSpecs);
  // The built table carries each spec's name, period and budget.
  assert(jobs.table()[1].name == kGoodSpecs[1].name);
  assert(jobs.table()[1].periodUs == 3000 && jobs.table()[1].budgetUs == 500);

  Scheduler<2, FakeClock> s(jobs.table());
  s.start();
  s.tick();
  assert(o.a == 1 && o.b == 1);
  FakeClock::now += 1000;  // only "a" is due again
  s.tick();
  assert(o.a == 2 && o.b == 1);
  FakeClock::now += 2000;  // both due
  s.tick();
  assert(o.a == 3 && o.b == 2);
}

int main() {
  testFixedRateNoDrift();
  testNotRunEarly();
  testSkipAhead();
  testBudgetAndStats();
  testReturnsEarliestDeadline();
  testDisabledJobNeverRuns();
  testDisabledJobIgnoredForDeadline();
  testReenableRestartsGridWithoutSkips();
  testMemberJobsDispatchToTheRightMember();
  puts("scheduler: all tests passed");
  return 0;
}
