#pragma once

#include <stdint.h>

namespace gnimu {

// Tracks whether the IMU is answering. Starts down; markUp() moves it up once
// bring-up succeeds. Once up, enough consecutive failed reads latch it back
// down for good - matching Classic, there is no recovery without a reboot, so
// a flaky sensor can't flap the rest of the device between states all session.
class ImuHealth {
 public:
  explicit ImuHealth(uint8_t downAfterFailures) : downAfterFailures_(downAfterFailures) {}

  void markUp() { up_ = true; }
  // The IMU was deliberately powered off (a power-state stop), not failed.
  void markDown() { up_ = false; }

  // Feeds one read result. Returns true the instant this call is what latches
  // it down, so the caller can log the transition exactly once.
  bool feed(bool ok) {
    if (!up_) return false;
    if (ok) {
      failures_ = 0;
      return false;
    }
    if (++failures_ >= downAfterFailures_) {
      up_ = false;
      return true;
    }
    return false;
  }

  bool up() const { return up_; }

 private:
  uint8_t downAfterFailures_;
  uint8_t failures_ = 0;
  bool up_ = false;
};

}  // namespace gnimu
