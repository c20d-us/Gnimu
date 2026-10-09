#pragma once

#include <stddef.h>
#include <stdint.h>

namespace gnimu {

struct JobStats {
  uint32_t runs = 0;
  uint32_t overruns = 0;   // runs whose duration exceeded the job's budget
  uint32_t skipped = 0;    // whole periods missed before a run started
  uint32_t maxUs = 0;
  uint64_t totalUs = 0;
  uint32_t maxLateUs = 0;  // longest a run started after it was due
};

struct Job {
  const char* name;
  uint32_t periodUs;
  uint32_t budgetUs;  // 0 = no budget check
  void (*run)(void* ctx);
  void* ctx;
};

// Fixed-rate cooperative scheduler. Jobs run in table order (order is
// priority). Clock supplies `static uint64_t nowUs()`. The scheduler never
// sleeps: tick() returns the next deadline and the caller decides how to wait.
// Every job starts enabled; the app disables the ones its current state
// doesn't run. At least one job must stay enabled. Every periodUs must be
// positive (the skipped count divides by it); jobSpecsValid() below checks a
// JobSpec table for that at compile time.
template <size_t N, class Clock>
class Scheduler {
 public:
  explicit Scheduler(const Job (&jobs)[N]) : jobs_(jobs) {
    for (size_t i = 0; i < N; i++) enabled_[i] = true;
  }

  void start() {
    const uint64_t now = Clock::nowUs();
    for (size_t i = 0; i < N; i++) nextDue_[i] = now;
  }

  // Re-enabling restarts the job's grid at "now", so the time it spent
  // disabled isn't counted as skipped periods.
  void setEnabled(size_t i, bool on) {
    if (on && !enabled_[i]) nextDue_[i] = Clock::nowUs();
    enabled_[i] = on;
  }
  bool enabled(size_t i) const { return enabled_[i]; }

  // Runs every enabled job that is due, at most once each, and returns the
  // earliest upcoming deadline among enabled jobs only - a disabled job's
  // stale deadline would otherwise read as permanently overdue and the
  // caller would never sleep.
  uint64_t tick() {
    for (size_t i = 0; i < N; i++) {
      if (!enabled_[i]) continue;
      const uint64_t begin = Clock::nowUs();
      if (begin < nextDue_[i]) continue;

      const Job& job = jobs_[i];
      JobStats& s = stats_[i];
      const uint64_t late = begin - nextDue_[i];
      if (late > s.maxLateUs) s.maxLateUs = late > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(late);
      const uint64_t missed = late / job.periodUs;
      s.skipped += static_cast<uint32_t>(missed);
      nextDue_[i] += (missed + 1) * job.periodUs;

      job.run(job.ctx);

      const uint64_t elapsed = Clock::nowUs() - begin;
      const uint32_t us = elapsed > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(elapsed);
      s.runs++;
      s.totalUs += us;
      if (us > s.maxUs) s.maxUs = us;
      if (job.budgetUs != 0 && us > job.budgetUs) s.overruns++;
    }

    uint64_t next = UINT64_MAX;
    for (size_t i = 0; i < N; i++)
      if (enabled_[i] && nextDue_[i] < next) next = nextDue_[i];
    return next;
  }

  const JobStats& stats(size_t i) const { return stats_[i]; }
  const Job* jobs() const { return jobs_; }
  const JobStats* allStats() const { return stats_; }
  void clearStats() {
    for (size_t i = 0; i < N; i++) stats_[i] = JobStats();
  }

 private:
  const Job (&jobs_)[N];
  uint64_t nextDue_[N] = {};
  bool enabled_[N];
  JobStats stats_[N] = {};
};

// A job declared against a member function of T. Entries carry their own id
// so a table can be checked, at compile time, to sit in id order.
template <class T>
struct JobSpec {
  size_t id;  // must equal the entry's position in its table
  const char* name;
  uint32_t periodUs;
  uint32_t budgetUs;
  void (T::*run)();
};

// True when every entry sits at its own id and has a name and a positive
// period (tick() divides by it). One return, recursive: the nRF core builds as
// C++11. `run` isn't checked: GCC 9 (the nRF toolchain) can't compare a
// member-function pointer in a constant expression.
template <class T, size_t N>
constexpr bool jobSpecsValid(const JobSpec<T> (&s)[N], size_t i = 0) {
  return i == N || (s[i].id == i && s[i].name != nullptr && s[i].periodUs > 0 &&
                    jobSpecsValid(s, i + 1));
}

// Binds a JobSpec table to one object: builds the scheduler's Job table and
// routes each run to the right member function. Not copyable - each Job's
// context points into this object.
template <class T, size_t N>
class MemberJobs {
 public:
  using Table = Job[N];

  MemberJobs(T* self, const JobSpec<T> (&specs)[N]) {
    for (size_t i = 0; i < N; i++) {
      bindings_[i] = {self, specs[i].run};
      jobs_[i] = {specs[i].name, specs[i].periodUs, specs[i].budgetUs, &dispatch, &bindings_[i]};
    }
  }
  MemberJobs(const MemberJobs&) = delete;
  MemberJobs& operator=(const MemberJobs&) = delete;

  const Table& table() const { return jobs_; }

 private:
  struct Binding {
    T* self;
    void (T::*run)();
  };
  static void dispatch(void* ctx) {
    Binding* b = static_cast<Binding*>(ctx);
    (b->self->*b->run)();
  }

  Binding bindings_[N];
  Job jobs_[N];
};

}  // namespace gnimu
