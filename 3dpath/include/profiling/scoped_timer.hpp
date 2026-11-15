#pragma once

#include <cstddef>
#include <chrono>
#include <deque>
#include <mutex>

namespace path3d::profiling {

// EventBuffer element, takes in starting, ending time of the func, and the func name
struct Event {
    std::chrono::steady_clock::time_point start, end;
    const char* func_name;
};

template<size_t N>
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

template <std::size_t N>
class ScopedTime {
public:
    ScopedTime(EventBuffer<N>& buf, const char* func_name)
        : m_buf(buf)
    {
        m_event.func_name = func_name;
        m_event.start = std::chrono::steady_clock::now();
    }

    ~ScopedTime() noexcept {
        m_event.end = std::chrono::steady_clock::now();
        m_buf.push(m_event);
    }

    ScopedTime(const ScopedTime&) = delete;
    ScopedTime& operator=(const ScopedTime&) = delete;
    ScopedTime(ScopedTime&&) = delete;
    ScopedTime& operator=(ScopedTime&&) = delete;

private:
    EventBuffer<N>& m_buf;
    Event m_event;
};

} // path3d::profiling