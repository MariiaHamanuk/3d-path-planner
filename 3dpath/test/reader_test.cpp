#include "fake_reader.hpp"
#include "sim_reader.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <type_traits>

using namespace std::chrono_literals;
using path3d::Reader;
using path3d::test::FakeReader;

static_assert(std::is_abstract_v<Reader>);
static_assert(std::is_base_of_v<Reader, path3d::SimReader>);

namespace {
// Consumer code that only knows the interface.
double drone_height(const Reader& reader) {
  const auto pose = reader.drone_pose();
  return pose ? pose->pose.position.z() : -1.0;
}
}  // namespace

TEST(Reader, ConsumersWorkThroughTheInterface) {
  std::unique_ptr<Reader> reader = std::make_unique<FakeReader>();
  EXPECT_TRUE(reader->wait_for_data(1ms));
  EXPECT_DOUBLE_EQ(drone_height(*reader), 3.0);
  EXPECT_TRUE(reader->color_image().has_value());
}

TEST(Reader, SensorFrameBundlesEverythingTheReaderProvides) {
  const auto frame = path3d::read_sensor_frame(FakeReader());
  ASSERT_TRUE(frame.has_value());
  EXPECT_DOUBLE_EQ(frame->stamp, 1.45);  // depth stamp
  EXPECT_DOUBLE_EQ(frame->drone_pose.pose.position.z(), 3.0);
  EXPECT_EQ(frame->color.image.cols, 6);
  EXPECT_EQ(frame->depth.image.at<float>(0, 0), 2.5f);
  EXPECT_EQ(frame->color_calibration.frame_id, path3d::FrameId::ColorCamera);
  EXPECT_EQ(frame->depth_calibration.frame_id, path3d::FrameId::DepthCamera);
  EXPECT_EQ(frame->base_to_depth.child_frame, path3d::FrameId::DepthCamera);
}

TEST(Reader, SensorFrameIsNullWhileDataIsMissing) {
  EXPECT_FALSE(path3d::read_sensor_frame(FakeReader(false)).has_value());
}
