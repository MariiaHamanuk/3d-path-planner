#include "communication.hpp"
#include "types.hpp"

#include <gtest/gtest.h>

// The project's timestamped types qualify for Topic::subscribe_history(capacity).
static_assert(path3d::Stamped<path3d::StampedPose>);
static_assert(path3d::Stamped<path3d::ImageFrame>);
static_assert(path3d::Stamped<path3d::CameraCalibration>);
static_assert(!path3d::Stamped<path3d::Pose>);

TEST(Types, PoseToIsometryAppliesRotationAndTranslation) {
  path3d::Pose pose;
  pose.position = Eigen::Vector3d(1, 2, 3);
  pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  const Eigen::Vector3d p = pose.to_isometry() * Eigen::Vector3d(1, 0, 0);
  EXPECT_NEAR(p.x(), 1.0, 1e-9);
  EXPECT_NEAR(p.y(), 3.0, 1e-9);
  EXPECT_NEAR(p.z(), 3.0, 1e-9);
}

TEST(Types, StampedPoseHistoryLookupByStamp) {
  path3d::Topic<path3d::StampedPose> topic;
  auto history = topic.subscribe_history(8);
  path3d::StampedPose a, b;
  a.stamp = 1.0;
  b.stamp = 2.0;
  topic.publish(a);
  topic.publish(b);
  EXPECT_DOUBLE_EQ(history->closest(1.9)->stamp, 2.0);
}
