#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <concepts>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

/// In-process publish/subscribe between pipeline stages. No ROS types here.
///
///   Topic<T>          publisher side; fans one message out to any number of subscribers
///   LatestSlot<T>     subscriber that only keeps the newest message (state: pose, map, path)
///   BoundedQueue<T>   subscriber that keeps a bounded stream, dropping the oldest when full
///   HistoryBuffer<T>  subscriber that keeps the last N messages sorted by timestamp and answers
///                     "which message is closest to time t?" (sensor / pose synchronisation)
///
/// Messages are immutable and shared: a publish costs one allocation, delivery is a pointer copy.
namespace path3d {

/// Every message in the pipeline carries a timestamp in seconds as a `stamp` member.
template <class T>
concept Stamped = requires(const T& value) {
	{ value.stamp } -> std::convertible_to<double>;
};

template <Stamped T>
using Msg = std::shared_ptr<const T>;

/**
 * @brief Newest value wins. Readers never block; latest() is null until the first publish.
 */
template <Stamped T>
class LatestSlot {
public:
	void publish(Msg<T> msg) {
		std::lock_guard<std::mutex> lock(m_mutex);
		m_msg = std::move(msg);
	}

	/// Newest message, or null before the first publish.
	Msg<T> latest() const {
		std::lock_guard<std::mutex> lock(m_mutex);
		return m_msg;
	}

private:
	mutable std::mutex m_mutex;
	Msg<T> m_msg;
};

/**
 * @brief FIFO with a fixed capacity (at least 1). push() never blocks: when full, the oldest item
 * is dropped and counted. pop() blocks up to a timeout.
 */
template <Stamped T>
class BoundedQueue {
public:
	/**
	 * @param capacity Messages held before the oldest is dropped (at least 1).
	 */
	explicit BoundedQueue(std::size_t capacity) : m_capacity(capacity > 0 ? capacity : 1) {}

	/// Adds a message, dropping the oldest if the queue is full. Ignored after close().
	void push(Msg<T> msg) {
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			if (m_closed) return;
			if (m_items.size() == m_capacity) {
				m_items.pop_front();
				++m_dropped_count;
			}
			m_items.push_back(std::move(msg));
		}
		m_cv.notify_one();
	}

	/// Null on timeout, or once the queue is closed and drained.
	Msg<T> pop(std::chrono::milliseconds timeout) {
		std::unique_lock<std::mutex> lock(m_mutex);
		m_cv.wait_for(lock, timeout, [this] { return !m_items.empty() || m_closed; });
		if (m_items.empty()) return nullptr;
		Msg<T> msg = std::move(m_items.front());
		m_items.pop_front();
		return msg;
	}

	/// Wakes every waiting pop(); later pushes are ignored. Used to stop a consumer promptly.
	void close() {
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			m_closed = true;
		}
		m_cv.notify_all();
	}

	std::size_t size() const {
		std::lock_guard<std::mutex> lock(m_mutex);
		return m_items.size();
	}

	std::size_t dropped_count() const {
		std::lock_guard<std::mutex> lock(m_mutex);
		return m_dropped_count;
	}

private:
	const std::size_t m_capacity;
	mutable std::mutex m_mutex;
	std::condition_variable m_cv;
	std::deque<Msg<T>> m_items;
	std::size_t m_dropped_count = 0;
	bool m_closed = false;
};

/**
 * @brief Keeps the last `capacity` messages sorted by their `stamp` (seconds) and answers time
 * queries. Out-of-order messages are inserted at their sorted position. When full, the
 * message with the oldest timestamp is evicted.
 */
template <Stamped T>
class HistoryBuffer {
public:
	/// Messages around a time; either side is null if t is outside the history.
	///   first:  last message with stamp <= t
	///   second: first message with stamp >= t (same as `first` on an exact match)
	using Bracket = std::pair<Msg<T>, Msg<T>>;

	/**
	 * @param capacity Messages kept; the one with the oldest stamp is evicted first (at least 1).
	 */
	explicit HistoryBuffer(std::size_t capacity) : m_capacity(capacity > 0 ? capacity : 1) {}

	void push(Msg<T> msg) {
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			const auto pos = std::upper_bound(
			    m_msgs.begin(), m_msgs.end(), msg->stamp,
			    [](double value, const Msg<T>& other) { return value < other->stamp; });
			m_msgs.insert(pos, std::move(msg));
			if (m_msgs.size() > m_capacity) m_msgs.pop_front();
		}
		m_cv.notify_all();
	}

	/// Nearest message in time (the earlier one on a tie); null if the buffer is empty.
	Msg<T> closest(double stamp) const {
		std::lock_guard<std::mutex> lock(m_mutex);
		return find_closest(stamp);
	}

	/// Same, but null if the nearest message is farther than `max_diff` seconds from `stamp`.
	Msg<T> closest(double stamp, double max_diff) const {
		std::lock_guard<std::mutex> lock(m_mutex);
		Msg<T> msg = find_closest(stamp);
		if (msg == nullptr || std::abs(msg->stamp - stamp) > max_diff) return nullptr;
		return msg;
	}

	/// The two messages surrounding `stamp`, e.g. for interpolating a pose.
	Bracket bracket(double stamp) const {
		std::lock_guard<std::mutex> lock(m_mutex);
		Bracket out;
		const auto after = lower_bound(stamp);
		if (after != m_msgs.end()) out.second = *after;
		if (after != m_msgs.end() && (*after)->stamp == stamp) {
			out.first = *after;
		} else if (after != m_msgs.begin()) {
			out.first = *std::prev(after);
		}
		return out;
	}

	/// Blocks until a message with stamp >= `stamp` has arrived (so closest()/bracket() can see
	/// data at or after that time). False on timeout or after close().
	bool wait_until_newest(double stamp, std::chrono::milliseconds timeout) const {
		std::unique_lock<std::mutex> lock(m_mutex);
		return m_cv.wait_for(lock, timeout, [&] {
			return m_closed || (!m_msgs.empty() && m_msgs.back()->stamp >= stamp);
		}) && !m_closed;
	}

	void close() {
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			m_closed = true;
		}
		m_cv.notify_all();
	}

	std::size_t size() const {
		std::lock_guard<std::mutex> lock(m_mutex);
		return m_msgs.size();
	}

	std::optional<double> oldest_stamp() const {
		std::lock_guard<std::mutex> lock(m_mutex);
		return m_msgs.empty() ? std::nullopt : std::optional<double>(m_msgs.front()->stamp);
	}

	std::optional<double> newest_stamp() const {
		std::lock_guard<std::mutex> lock(m_mutex);
		return m_msgs.empty() ? std::nullopt : std::optional<double>(m_msgs.back()->stamp);
	}

private:
	using Msgs = std::deque<Msg<T>>;

	typename Msgs::const_iterator lower_bound(double stamp) const {
		return std::lower_bound(m_msgs.begin(), m_msgs.end(), stamp,
		                        [](const Msg<T>& msg, double value) { return msg->stamp < value; });
	}

	/// caller holds the lock
	Msg<T> find_closest(double stamp) const {
		if (m_msgs.empty()) return nullptr;
		const auto after = lower_bound(stamp);
		if (after == m_msgs.begin()) return *after;
		if (after == m_msgs.end()) return m_msgs.back();
		const auto before = std::prev(after);
		return (stamp - (*before)->stamp <= (*after)->stamp - stamp) ? *before : *after;
	}

	const std::size_t m_capacity;
	mutable std::mutex m_mutex;
	mutable std::condition_variable m_cv;
	Msgs m_msgs;
	bool m_closed = false;
};

/**
 * @brief Publisher side. Subscribers choose their policy when they subscribe and keep the returned
 * handle; dropping the last handle unsubscribes (cleaned up on the next publish). Delivery
 * runs on the publisher's thread and never holds the topic lock. The subscriber list is
 * copy-on-write: subscribing copies it, publishing only copies one shared_ptr to it.
 */
template <Stamped T>
class Topic {
public:
	std::shared_ptr<LatestSlot<T>> subscribe_latest() {
		auto slot = std::make_shared<LatestSlot<T>>();
		add_sink(slot, [](LatestSlot<T>& s, const Msg<T>& msg) { s.publish(msg); });
		return slot;
	}

	std::shared_ptr<BoundedQueue<T>> subscribe_queue(std::size_t capacity) {
		auto queue = std::make_shared<BoundedQueue<T>>(capacity);
		add_sink(queue, [](BoundedQueue<T>& q, const Msg<T>& msg) { q.push(msg); });
		return queue;
	}

	std::shared_ptr<HistoryBuffer<T>> subscribe_history(std::size_t capacity) {
		auto history = std::make_shared<HistoryBuffer<T>>(capacity);
		add_sink(history, [](HistoryBuffer<T>& h, const Msg<T>& msg) { h.push(msg); });
		return history;
	}

	void publish(Msg<T> msg) {
		std::shared_ptr<const Entries> entries;
		{
			std::lock_guard<std::mutex> lock(m_mutex);
			entries = m_entries;
		}
		bool any_gone = false;
		for (const auto& entry : *entries) any_gone |= !entry.deliver(msg);
		if (any_gone) remove_gone();
	}

	void publish(T value) { publish(std::make_shared<const T>(std::move(value))); }

	std::size_t subscriber_count() const {
		std::lock_guard<std::mutex> lock(m_mutex);
		std::size_t count = 0;
		for (const auto& entry : *m_entries) count += entry.is_alive() ? 1 : 0;
		return count;
	}

private:
	/**
	 * @brief One subscriber: whether it is still alive, and how to deliver a message to it.
	 */
	struct Entry {
		std::function<bool()> is_alive;
		std::function<bool(const Msg<T>&)> deliver;
	};
	/// Subscriber list; replaced as a whole when it changes (copy-on-write).
	using Entries = std::vector<Entry>;

	template <class Sink, class Push>
	void add_sink(const std::shared_ptr<Sink>& sink, Push push) {
		std::weak_ptr<Sink> weak = sink;
		Entry entry;
		entry.is_alive = [weak] { return !weak.expired(); };
		entry.deliver = [weak, push](const Msg<T>& msg) {
			auto locked = weak.lock();
			if (!locked) return false;
			push(*locked, msg);
			return true;
		};
		std::lock_guard<std::mutex> lock(m_mutex);
		auto next = std::make_shared<Entries>(*m_entries);
		next->push_back(std::move(entry));
		m_entries = std::move(next);
	}

	void remove_gone() {
		std::lock_guard<std::mutex> lock(m_mutex);
		auto alive = std::make_shared<Entries>();
		for (const auto& entry : *m_entries) {
			if (entry.is_alive()) alive->push_back(entry);
		}
		m_entries = std::move(alive);
	}

	mutable std::mutex m_mutex;
	std::shared_ptr<const Entries> m_entries = std::make_shared<const Entries>();
};

/**
 * @brief Bidirectional link between two stages, one Topic per direction. `down` carries messages
 * from the first stage to the second (e.g. routes from the global planner to the local
 * planner), `up` carries messages back (e.g. replan requests). Each subscriber picks its own
 * delivery policy.
 */
template <Stamped Down, Stamped Up>
class Pipe {
public:
	Topic<Down>& down() { return m_down; }
	Topic<Up>& up() { return m_up; }

private:
	Topic<Down> m_down;
	Topic<Up> m_up;
};

}
