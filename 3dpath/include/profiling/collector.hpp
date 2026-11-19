#pragma once

#include "profiling/scoped_timer.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <iostream>
#include <memory>
#include <mutex>
#include <stop_token>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace path3d::profiling {

/// Per-function timing summary accumulated by Collector.
struct FuncStats {
	std::size_t count = 0;
	std::chrono::nanoseconds total{0};
	std::chrono::nanoseconds min{std::chrono::nanoseconds::max()};
	std::chrono::nanoseconds max{0};
};

/// Handle stored in Collector's registry so EventBuffer in scoped_timer.hpp stays untouched.
struct BufferEntry {
	std::shared_ptr<ThreadBuffer> buf = std::make_shared<ThreadBuffer>();
	std::atomic<bool> retired{false};
};

// Background collector that periodically drains all registered EventBuffers and aggregates per-function timing statistics.
class Collector {
public:
	explicit Collector(std::chrono::milliseconds drain_interval = std::chrono::milliseconds(100));
	~Collector();

	Collector(const Collector&) = delete;
	Collector& operator=(const Collector&) = delete;
	Collector(Collector&&) = delete;
	Collector& operator=(Collector&&) = delete;

	static Collector* active();

	void register_buffer(std::shared_ptr<BufferEntry> entry);
	std::unordered_map<std::string, FuncStats> stats() const;
	void print_summary(std::ostream& out = std::cout) const;

private:
	void run(std::stop_token stop);
	void drain_all();

	static std::atomic<Collector*> s_active;

	std::chrono::milliseconds m_drain_interval;

	mutable std::mutex m_registry_mutex;
	std::vector<std::shared_ptr<BufferEntry>> m_buffers;

	mutable std::mutex m_stats_mutex;
	std::unordered_map<std::string, FuncStats> m_stats;

	std::mutex m_wake_mutex;
	std::condition_variable_any m_cv;
	std::jthread m_thread;
};

}  // namespace path3d::profiling
