#include "profiling/collector.hpp"

#include <algorithm>

namespace path3d::profiling {

std::atomic<Collector*> Collector::s_active{nullptr};

Collector::Collector(std::chrono::milliseconds drain_interval)
    : m_drain_interval(drain_interval) {
	s_active.store(this, std::memory_order_release);
	m_thread = std::jthread([this](std::stop_token stop) { run(stop); });
}

Collector::~Collector() {
	Collector* expected = this;
	s_active.compare_exchange_strong(expected, nullptr, std::memory_order_acq_rel);

	m_thread.request_stop();
	m_cv.notify_all();
	if (m_thread.joinable()) m_thread.join();
}

Collector* Collector::active() {
	return s_active.load(std::memory_order_acquire);
}

void Collector::register_buffer(std::shared_ptr<BufferEntry> entry) {
	std::lock_guard<std::mutex> lock(m_registry_mutex);
	m_buffers.push_back(std::move(entry));
}

std::unordered_map<std::string, FuncStats> Collector::stats() const {
	std::lock_guard<std::mutex> lock(m_stats_mutex);
	return m_stats;
}

void Collector::print_summary(std::ostream& out) const {
	const auto snapshot = stats();
	out << "Profiler summary (" << snapshot.size() << " zones):\n";
	for (const auto& [name, s] : snapshot) {
		const double total_ms = std::chrono::duration<double, std::milli>(s.total).count();
		const double mean_us =
		    s.count > 0 ? std::chrono::duration<double, std::micro>(s.total).count() / s.count : 0.0;
		const double min_us = std::chrono::duration<double, std::micro>(s.min).count();
		const double max_us = std::chrono::duration<double, std::micro>(s.max).count();
		out << "  " << name << ": count=" << s.count << " total=" << total_ms << "ms"
		    << " mean=" << mean_us << "us min=" << min_us << "us max=" << max_us << "us\n";
	}
}

void Collector::run(std::stop_token stop) {
	std::unique_lock<std::mutex> lock(m_wake_mutex);
	while (!stop.stop_requested()) {
		lock.unlock();
		drain_all();
		lock.lock();
		m_cv.wait_for(lock, stop, m_drain_interval, [] { return false; });
	}
	lock.unlock();
	drain_all();
}

void Collector::drain_all() {
	std::vector<std::shared_ptr<BufferEntry>> buffers;
	{
		std::lock_guard<std::mutex> lock(m_registry_mutex);
		buffers = m_buffers;
	}

	Event event;
	for (const auto& entry : buffers) {
		while (entry->buf->get(event)) {
			const auto duration = std::chrono::duration_cast<std::chrono::nanoseconds>(event.end - event.start);
			const char* key = event.func_name ? event.func_name : "<unknown>";

			std::lock_guard<std::mutex> lock(m_stats_mutex);
			FuncStats& s = m_stats[key];
			++s.count;
			s.total += duration;
			s.min = std::min(s.min, duration);
			s.max = std::max(s.max, duration);
		}
	}

	std::lock_guard<std::mutex> lock(m_registry_mutex);
	std::erase_if(m_buffers, [](const std::shared_ptr<BufferEntry>& entry) {
		if (!entry->retired.load(std::memory_order_acquire)) return false;
		std::lock_guard<std::mutex> buf_lock(entry->buf->mutex);
		return entry->buf->buf.empty();
	});
}

namespace detail {

struct ThreadSlot {
	std::shared_ptr<BufferEntry> entry;
	Collector* registered_to = nullptr;

	~ThreadSlot() {
		if (entry) entry->retired.store(true, std::memory_order_release);
	}
};

ThreadBuffer* current_thread_buffer() {
	Collector* collector = Collector::active();
	if (!collector) return nullptr;

	thread_local ThreadSlot slot;
	if (slot.registered_to != collector) {
		if (slot.entry) slot.entry->retired.store(true, std::memory_order_release);
		slot.entry = std::make_shared<BufferEntry>();
		slot.registered_to = collector;
		collector->register_buffer(slot.entry);
	}
	return slot.entry->buf.get();
}

}  // namespace detail

}  // namespace path3d::profiling
