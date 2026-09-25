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
template <size_t N, class Clock>
class Scheduler {
 public:
  explicit Scheduler(const Job (&jobs)[N]) : jobs_(jobs) {}

  void start() {
    const uint64_t now = Clock::nowUs();
    for (size_t i = 0; i < N; i++) nextDue_[i] = now;
  }

  // Runs every job that is due, at most once each, and returns the earliest
  // upcoming deadline.
  uint64_t tick() {
    for (size_t i = 0; i < N; i++) {
      const uint64_t begin = Clock::nowUs();
      if (begin < nextDue_[i]) continue;

      const Job& job = jobs_[i];
      JobStats& s = stats_[i];
      const uint64_t missed = (begin - nextDue_[i]) / job.periodUs;
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

    uint64_t next = nextDue_[0];
    for (size_t i = 1; i < N; i++)
      if (nextDue_[i] < next) next = nextDue_[i];
    return next;
  }

  const Job& job(size_t i) const { return jobs_[i]; }
  const JobStats& stats(size_t i) const { return stats_[i]; }
  const Job* jobs() const { return jobs_; }
  const JobStats* allStats() const { return stats_; }
  void clearStats() {
    for (size_t i = 0; i < N; i++) stats_[i] = JobStats();
  }
  static constexpr size_t count() { return N; }

 private:
  const Job (&jobs_)[N];
  uint64_t nextDue_[N] = {};
  JobStats stats_[N] = {};
};

}  // namespace gnimu
