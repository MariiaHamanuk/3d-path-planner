#include "communication.hpp"
#include "fake_reader.hpp"
#include "mapper.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <type_traits>

using path3d::Mapper;
using path3d::SensorFrame;

static_assert(std::is_abstract_v<Mapper>);
static_assert(path3d::Stamped<SensorFrame>);

namespace {
// Minimal implementation: remembers the last frame it was given.
class RecordingMapper : public Mapper {
 public:
  void integrate(const SensorFrame& frame) override {
    m_last_stamp = frame.stamp;
    m_last_depth_value = frame.depth.image.at<float>(0, 0);
    ++m_version;
  }
  std::uint64_t version() const override { return m_version; }
  double last_stamp() const { return m_last_stamp; }
  float last_depth_value() const { return m_last_depth_value; }

 private:
  std::uint64_t m_version = 0;
  double m_last_stamp = 0.0;
  float m_last_depth_value = 0.0f;
};

// Pipeline code that only knows the interfaces.
void feed(const path3d::Reader& reader, Mapper& mapper) {
  if (const auto frame = path3d::read_sensor_frame(reader)) mapper.integrate(*frame);
}
}  // namespace

TEST(Mapper, ConsumesWhatTheReaderProvides) {
  RecordingMapper recording;
  Mapper& mapper = recording;
  EXPECT_EQ(mapper.version(), 0u);
  feed(path3d::test::FakeReader(), mapper);
  EXPECT_EQ(mapper.version(), 1u);
  EXPECT_DOUBLE_EQ(recording.last_stamp(), 1.45);
  EXPECT_FLOAT_EQ(recording.last_depth_value(), 2.5f);
}

TEST(Mapper, NothingIntegratedWhileReaderHasNoData) {
  RecordingMapper mapper;
  feed(path3d::test::FakeReader(false), mapper);
  EXPECT_EQ(mapper.version(), 0u);
}
