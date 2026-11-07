#include "core/communication.hpp"
#include "core/types.hpp"

#include <gtest/gtest.h>

/// The project's timestamped types qualify for Topic::subscribe_history(capacity).
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

/// Frames are a one-byte enum, not a string copied with every message.
static_assert(sizeof(path3d::FrameId) == 1);
static_assert(path3d::camera_frame(path3d::CameraId::Depth) == path3d::FrameId::DepthCamera);

TEST(Types, FrameIdHasReadableNames) {
  EXPECT_STREQ(path3d::to_string(path3d::FrameId::Base), "base");
  EXPECT_EQ(path3d::StampedPose{}.parent_frame, path3d::FrameId::World);
}

TEST(Types, InterpolateIsLinearInPositionAndSlerpInOrientation) {
  path3d::StampedPose a, b;
  a.stamp = 1.0;
  b.stamp = 3.0;
  b.pose.position = Eigen::Vector3d(2, 4, 0);
  b.pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
  const path3d::StampedPose mid = path3d::interpolate(a, b, 2.0);
  EXPECT_DOUBLE_EQ(mid.stamp, 2.0);
  EXPECT_TRUE(mid.pose.position.isApprox(Eigen::Vector3d(1, 2, 0)));
  EXPECT_NEAR(mid.pose.orientation.angularDistance(a.pose.orientation), M_PI / 4, 1e-9);
  EXPECT_TRUE(path3d::interpolate(a, a, 5.0).pose.position.isApprox(a.pose.position));
}

static_assert(path3d::Stamped<path3d::GlobalPath>);
static_assert(path3d::Stamped<path3d::ReplanRequest>);
