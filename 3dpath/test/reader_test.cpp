#include "reader.hpp"
#include "sim_reader.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <type_traits>

using namespace std::chrono_literals;
using path3d::CameraId;
using path3d::Reader;

static_assert(std::is_abstract_v<Reader>);
static_assert(std::is_base_of_v<Reader, path3d::SimReader>);

namespace {
// A second implementation: canned data, no ROS, no simulation.
class FakeReader : public Reader {
 public:
  std::optional<path3d::StampedPose> drone_pose() const override {
    path3d::StampedPose pose;
    pose.stamp = 1.5;
    pose.parent_frame = path3d::FrameId::World;
    pose.child_frame = path3d::FrameId::Base;
    pose.pose.position = Eigen::Vector3d(1, 2, 3);
    return pose;
  }
  std::optional<path3d::ImageFrame> color_image() const override { return std::nullopt; }
  std::optional<path3d::ImageFrame> depth_image() const override { return std::nullopt; }
  std::optional<path3d::CameraCalibration> calibration(CameraId) const override {
    return std::nullopt;
  }
  std::optional<path3d::StampedPose> base_to_camera(CameraId) const override {
    return std::nullopt;
  }
  bool wait_for_data(std::chrono::milliseconds) const override { return true; }
};

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
  EXPECT_FALSE(reader->color_image().has_value());
}
