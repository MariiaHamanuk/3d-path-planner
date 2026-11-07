#include "core/threading.hpp"

#include <exception>
#include <iostream>

namespace path3d {

Thread::~Thread() {
	/// A joinable thread here means a derived class forgot stop(); std::thread would terminate.
	if (m_thread.joinable()) {
		m_stop_requested = true;
		m_thread.join();
	}
}

void Thread::start() {
	if (m_thread.joinable()) return;
	m_stop_requested = false;
	m_thread = std::thread([this] {
		try {
			run();
		} catch (const std::exception& e) {
			fail(e.what());
		} catch (...) {
			fail("unknown exception");
		}
	});
}

void Thread::stop() {
	m_stop_requested = true;
	on_stop();
	if (m_thread.joinable()) m_thread.join();
}

std::string Thread::error() const {
	std::lock_guard<std::mutex> lock(m_error_mutex);
	return m_error;
}

void Thread::fail(const std::string& message) {
	{
		std::lock_guard<std::mutex> lock(m_error_mutex);
		m_error = message;
	}
	m_failed = true;
	std::cerr << "thread failed: " << message << std::endl;
}

Rate::Rate(double hz)
    : m_period(std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(hz > 0.0 ? 1.0 / hz : 1.0))),
      m_next(Clock::now() + m_period) {}

void Rate::sleep() {
	const auto now = Clock::now();
	/// fell behind: resync instead of bursting
	if (now > m_next + m_period) m_next = now;
	std::this_thread::sleep_until(m_next);
	m_next += m_period;
}

}
