#include "threading.hpp"

namespace path3d {

Stage::~Stage() {
  // A joinable thread here means a derived class forgot stop(); std::thread would terminate.
  if (m_thread.joinable()) {
    m_stop_requested = true;
    m_thread.join();
  }
}

void Stage::start() {
  if (m_thread.joinable()) return;
  m_stop_requested = false;
  m_thread = std::thread([this] { run(); });
}

void Stage::stop() {
  m_stop_requested = true;
  on_stop();
  if (m_thread.joinable()) m_thread.join();
}

Rate::Rate(double hz)
    : m_period(std::chrono::duration_cast<Clock::duration>(
          std::chrono::duration<double>(hz > 0.0 ? 1.0 / hz : 1.0))),
      m_next(Clock::now() + m_period) {}

void Rate::sleep() {
  const auto now = Clock::now();
  if (now > m_next + m_period) m_next = now;  // fell behind: resync instead of bursting
  std::this_thread::sleep_until(m_next);
  m_next += m_period;
}

}  // namespace path3d
