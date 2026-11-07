#include "io/reader.hpp"
#include "io/sim_reader.hpp"

#include <gtest/gtest.h>

#include <type_traits>

using namespace path3d;

static_assert(std::is_base_of_v<Reader, SimReader>);
static_assert(!std::is_copy_constructible_v<Reader>);

namespace {
/**
 * @brief Smallest possible reader: publishes one pose when constructed.
 */
class OnePoseReader : public Reader {
 public:
  explicit OnePoseReader(SensorTopics& out) : Reader(out) {
    StampedPose pose;
    pose.stamp = 2.0;
    m_out.pose.publish(pose);
  }
};
}

TEST(Reader, PublishesIntoTheTopicsItWasGiven) {
  SensorTopics sensors;
  /// subscribers before the reader
  const auto pose = sensors.pose.subscribe_latest();
  OnePoseReader reader(sensors);
  ASSERT_NE(pose->latest(), nullptr);
  EXPECT_DOUBLE_EQ(pose->latest()->stamp, 2.0);
}
