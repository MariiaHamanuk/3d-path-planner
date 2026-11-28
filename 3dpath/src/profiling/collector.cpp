#include "profiling/collector.hpp"

#include <stdexcept>
#include <string_view>

namespace path3d::profiling {

namespace {

void write_csv_field(std::ostream& out, std::string_view text) {
	out.put('"');
	for (char c : text) {
		if (c == '"') out.put('"');
		out.put(c);
	}
	out.put('"');
}

std::filesystem::path unique_output_path(const std::filesystem::path& path) {
	if (!std::filesystem::exists(path)) return path;
	const auto dir = path.parent_path();
	const std::string stem = path.stem().string();
	const std::string ext = path.extension().string();
	for (std::size_t n = 1;; ++n) {
		const auto candidate = dir / (stem + "_" + std::to_string(n) + ext);
		if (!std::filesystem::exists(candidate)) return candidate;
	}
}

}  // namespace

std::atomic<Collector*> Collector::s_active{nullptr};

Collector::Collector(const std::filesystem::path& output_path, std::chrono::milliseconds drain_interval)
    : m_drain_interval(drain_interval), m_start(std::chrono::steady_clock::now()) {
	if (output_path.has_parent_path() && !output_path.parent_path().empty()) {
		std::filesystem::create_directories(output_path.parent_path());
	}
	const std::filesystem::path actual_path = unique_output_path(output_path);
	m_out.open(actual_path, std::ios::out | std::ios::trunc);
	if (!m_out.is_open()) {
		throw std::runtime_error("Failed to open profiler output file: " + actual_path.string());
	}
	m_out << "func,duration_us,start_s\n";
	m_out.flush();

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

	bool wrote_any = false;
	Event event;
	for (const auto& entry : buffers) {
		while (entry->buf->get(event)) {
			const double duration_us = std::chrono::duration<double, std::micro>(event.end - event.start).count();
			const double start_s = std::chrono::duration<double>(event.start - m_start).count();
			const char* name = event.func_name ? event.func_name : "<unknown>";

			write_csv_field(m_out, name);
			m_out << ',' << duration_us << ',' << start_s << '\n';
			wrote_any = true;
		}
	}
	if (wrote_any) m_out.flush();

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
