#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

/// Thread lifecycle and loop pacing for pipeline threads. Threads talk to each other through
/// the channels in communication.hpp.
namespace path3d {

/**
 * @brief One thread running run(). Derived classes MUST call stop() in their own destructor (the
 * thread uses derived members). Override on_stop() to wake blocking waits, e.g. close() a
 * queue. An exception escaping run() ends the thread instead of the process: it is logged,
 * failed() becomes true and error() returns the message, so the owner can shut the pipeline
 * down.
 */
class Thread {
public:
	Thread() = default;
	virtual ~Thread();
	Thread(const Thread&) = delete;
	Thread& operator=(const Thread&) = delete;

	void start();  ///< no-op if already running
	void stop();   ///< signals, calls on_stop(), joins; safe to call repeatedly

	/// True if run() ended with an exception.
	bool failed() const { return m_failed; }
	std::string error() const;

protected:
	virtual void run() = 0;  ///< loop while running()
	/// Called by stop() before joining; wake blocking waits here (e.g. close a queue).
	virtual void on_stop() {}
	/// False once stop() was requested; run() loops while this is true.
	bool running() const { return !m_stop_requested; }

private:
	/// Records the failure message and logs it.
	void fail(const std::string& message);

	std::atomic<bool> m_stop_requested{false};
	std::atomic<bool> m_failed{false};
	mutable std::mutex m_error_mutex;
	std::string m_error;
	std::thread m_thread;
};

/**
 * @brief Fixed-period pacing in wall-clock time. sleep() waits until the next absolute deadline, so
 * the period does not drift with the work done in between. If the loop falls behind, the
 * schedule resets instead of firing a burst of catch-up iterations. Wall time is not sim
 * time: with a real-time factor below 1 a loop runs more often per simulated second. Code
 * paced by a Rate must take time steps from message stamps, never from the clock.
 */
class Rate {
public:
	/**
	 * @param hz Loop frequency (a non-positive value means 1 Hz).
	 */
	explicit Rate(double hz);
	void sleep();

private:
	using Clock = std::chrono::steady_clock;
	Clock::duration m_period;
	/// Deadline of the next period.
	Clock::time_point m_next;
};

}
