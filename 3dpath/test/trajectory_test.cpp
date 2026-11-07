#include "core/types.hpp"

#include <gtest/gtest.h>

#include <numbers>

using namespace path3d;

namespace {
Setpoint at(TimeSec stamp, double x, double yaw = 0.0) {
  Setpoint setpoint;
  setpoint.stamp = stamp;
  setpoint.position = Eigen::Vector3d(x, 0, 2);
  setpoint.velocity = Eigen::Vector3d(x, 0, 0);
  setpoint.yaw = yaw;
  return setpoint;
}
}

TEST(Trajectory, SampleInterpolatesBetweenPoints) {
  Trajectory trajectory;
  trajectory.points = {at(1.0, 0.0), at(2.0, 4.0)};
  const auto mid = trajectory.sample(1.25);
  ASSERT_TRUE(mid.has_value());
  EXPECT_DOUBLE_EQ(mid->stamp, 1.25);
  EXPECT_TRUE(mid->position.isApprox(Eigen::Vector3d(1, 0, 2)));
  EXPECT_TRUE(mid->velocity.isApprox(Eigen::Vector3d(1, 0, 0)));
}

TEST(Trajectory, SampleIsEmptyOutsideTheTrajectory) {
  Trajectory trajectory;
  EXPECT_FALSE(trajectory.sample(0.0).has_value());
  trajectory.points = {at(1.0, 0.0), at(2.0, 4.0)};
  EXPECT_FALSE(trajectory.sample(0.99).has_value());
  EXPECT_FALSE(trajectory.sample(2.01).has_value());
  EXPECT_TRUE(trajectory.sample(1.0)->position.isApprox(Eigen::Vector3d(0, 0, 2)));
  /// the end
  EXPECT_TRUE(trajectory.sample(2.0)->position.isApprox(Eigen::Vector3d(4, 0, 2)));
}

TEST(Trajectory, YawTakesTheShorterWayRound) {
  Trajectory trajectory;
  const double pi = std::numbers::pi;
  /// 0.2 pi apart via +-pi
  trajectory.points = {at(0.0, 0.0, 0.9 * pi), at(1.0, 0.0, -0.9 * pi)};
  const auto mid = trajectory.sample(0.5);
  ASSERT_TRUE(mid.has_value());
  EXPECT_NEAR(std::abs(mid->yaw), pi, 1e-9);
}

TEST(Trajectory, YawOfAQuaternion) {
  const double yaw = 0.7;
  EXPECT_NEAR(yaw_of(Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()))), yaw, 1e-12);
  EXPECT_NEAR(wrap_angle(3 * std::numbers::pi), std::numbers::pi, 1e-12);
}
