#pragma once

#include "communication.hpp"
#include "mapper.hpp"
#include "reader.hpp"
#include "threading.hpp"

#include <chrono>
#include <limits>
#include <memory>

// Pipeline stages: each one owns a thread and talks to its neighbours only through Topics.
//
//   Reader ──ReaderStage──► Topic<SensorFrame> ──MapperStage──► Mapper
namespace path3d {

// Polls a Reader at `rate_hz` and publishes a SensorFrame whenever a new depth image has arrived.
// Polling faster than the camera rate is intended: duplicates are filtered by the depth stamp.
class ReaderStage : public Stage {
 public:
  ReaderStage(const Reader& reader, Topic<SensorFrame>& out, double rate_hz = 30.0)
      : m_reader(reader), m_out(out), m_rate_hz(rate_hz) {}
  ~ReaderStage() override { stop(); }

  std::size_t published() const { return m_published; }

 private:
  void run() override {
    Rate rate(m_rate_hz);
    double last_stamp = -std::numeric_limits<double>::infinity();
    while (running()) {
      auto frame = read_sensor_frame(m_reader);
      if (frame && frame->stamp > last_stamp) {
        last_stamp = frame->stamp;
        m_out.publish(std::move(*frame));
        ++m_published;
      }
      rate.sleep();
    }
  }

  const Reader& m_reader;
  Topic<SensorFrame>& m_out;
  double m_rate_hz;
  std::atomic<std::size_t> m_published{0};
};

// Feeds every frame from the topic to a Mapper, in order. The queue is bounded: if the mapper is
// slower than the reader, the oldest frames are dropped (see dropped()) instead of lagging behind.
// Subscribes in the constructor, so create it before starting the ReaderStage.
class MapperStage : public Stage {
 public:
  MapperStage(Topic<SensorFrame>& in, Mapper& mapper, std::size_t queue_capacity = 2)
      : m_queue(in.subscribe_queue(queue_capacity)), m_mapper(mapper) {}
  ~MapperStage() override { stop(); }

  std::size_t integrated() const { return m_integrated; }
  std::size_t dropped() const { return m_queue->dropped(); }

 private:
  void run() override {
    using namespace std::chrono_literals;
    while (running()) {
      if (const Msg<SensorFrame> frame = m_queue->pop(100ms)) {
        m_mapper.integrate(*frame);
        ++m_integrated;
      }
    }
  }
  void on_stop() override { m_queue->close(); }

  std::shared_ptr<BoundedQueue<SensorFrame>> m_queue;
  Mapper& m_mapper;
  std::atomic<std::size_t> m_integrated{0};
};

}  // namespace path3d
