#pragma once

#include <atomic>
#include <chrono>
#include <thread>

// Thread lifecycle and loop pacing for pipeline stages. Stages talk to each other through
// the channels in communication.hpp.
namespace path3d {

// One thread running run(). Derived classes MUST call stop() in their own destructor (the
// thread uses derived members). Override on_stop() to wake blocking waits, e.g. close() a queue.
class Stage {
 public:
  Stage() = default;
  virtual ~Stage();
  Stage(const Stage&) = delete;
  Stage& operator=(const Stage&) = delete;

  void start();  // no-op if already running
  void stop();   // signals, calls on_stop(), joins; safe to call repeatedly

 protected:
  virtual void run() = 0;  // loop while running()
  virtual void on_stop() {}
  bool running() const { return !m_stop_requested; }

 private:
  std::atomic<bool> m_stop_requested{false};
  std::thread m_thread;
};

// Fixed-period pacing. sleep() waits until the next absolute deadline, so the period does not
// drift with the work done in between. If the loop falls behind, the schedule resets instead of
// firing a burst of catch-up iterations.
class Rate {
 public:
  explicit Rate(double hz);
  void sleep();

 private:
  using Clock = std::chrono::steady_clock;
  Clock::duration m_period;
  Clock::time_point m_next;
};

}  // namespace path3d
