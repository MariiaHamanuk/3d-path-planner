#pragma once

#include <chrono>
#include <cstddef>
#include <deque>
#include <mutex>
#include <source_location>

namespace path3d::profiling {

// EventBuffer element, takes in starting, ending time of the func, and the func name
struct Event {
    std::chrono::steady_clock::time_point start, end;
    const char* func_name = nullptr;
};

template<std::size_t N>
struct EventBuffer {
    std::mutex mutex;
    std::deque<Event> buf;

    // ? TODO: make it return true/false and hault profiling when buf is full
    void push(const Event &event) {
        std::lock_guard<std::mutex> lock(mutex);
        if (buf.size() < N) {
            buf.push_back(event);
        }
    }
    
    // for collector
    bool get(Event &event) {
        std::lock_guard<std::mutex> lock(mutex);
        if (buf.empty()) {
            return false;
        }
        event = buf.front();
        buf.pop_front();

        return true;
    }
};

inline constexpr std::size_t kBufferCapacity = 4096;
using ThreadBuffer = EventBuffer<kBufferCapacity>;

namespace detail {
ThreadBuffer* current_thread_buffer();
}  // namespace detail

class ScopedTime {
public:
    explicit ScopedTime(const char* func_name = std::source_location::current().function_name())
        : m_buf(detail::current_thread_buffer())
    {
        if (!m_buf) return;
        m_event.func_name = func_name;
        m_event.start = std::chrono::steady_clock::now();
    }

    ~ScopedTime() noexcept {
        if (!m_buf) return;
        m_event.end = std::chrono::steady_clock::now();
        m_buf->push(m_event);
    }

    ScopedTime(const ScopedTime&) = delete;
    ScopedTime& operator=(const ScopedTime&) = delete;
    ScopedTime(ScopedTime&&) = delete;
    ScopedTime& operator=(ScopedTime&&) = delete;

private:
    ThreadBuffer* m_buf = nullptr;
    Event m_event;
};

} // path3d::profiling
