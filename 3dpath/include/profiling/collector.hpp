#pragma once

#include "profiling/scoped_timer.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <stop_token>
#include <thread>
#include <vector>

namespace path3d::profiling {

/// Handle stored in Collector's registry so EventBuffer in scoped_timer.hpp stays untouched.
struct BufferEntry {
	std::shared_ptr<ThreadBuffer> buf = std::make_shared<ThreadBuffer>();
	std::atomic<bool> retired{false};
};

/// Background collector that periodically drains all registered EventBuffers and streams events to a CSV file.
class Collector {
public:
	explicit Collector(const std::filesystem::path& output_path,
	                   std::chrono::milliseconds drain_interval = std::chrono::milliseconds(100));
	~Collector();

	Collector(const Collector&) = delete;
	Collector& operator=(const Collector&) = delete;
	Collector(Collector&&) = delete;
	Collector& operator=(Collector&&) = delete;

	static Collector* active();

	void register_buffer(std::shared_ptr<BufferEntry> entry);

private:
	void run(std::stop_token stop);
	void drain_all();

	static std::atomic<Collector*> s_active;

	std::chrono::milliseconds m_drain_interval;
	std::chrono::steady_clock::time_point m_start;
	std::ofstream m_out;

	mutable std::mutex m_registry_mutex;
	std::vector<std::shared_ptr<BufferEntry>> m_buffers;

	std::mutex m_wake_mutex;
	std::condition_variable_any m_cv;
	std::jthread m_thread;
};

}  // namespace path3d::profiling
