#include "core/ned.hpp"

#include <gtest/gtest.h>

#include <numbers>
#include <random>

using namespace path3d;

TEST(Ned, AxesSwapBothWays) {
	const Eigen::Vector3d east_north_up(1, 2, 3);
	EXPECT_TRUE(enu_to_ned(east_north_up).isApprox(Eigen::Vector3d(2, 1, -3)));
	EXPECT_TRUE(ned_to_enu(enu_to_ned(east_north_up)).isApprox(east_north_up));
}

TEST(Ned, YawConvertsBetweenEastCcwAndNorthCw) {
	const double pi = std::numbers::pi;
	/// facing east
	EXPECT_NEAR(enu_to_ned_yaw(0.0), pi / 2, 1e-12);
	/// facing north
	EXPECT_NEAR(enu_to_ned_yaw(pi / 2), 0.0, 1e-12);
	EXPECT_NEAR(ned_to_enu_yaw(enu_to_ned_yaw(1.2)), 1.2, 1e-12);
}

TEST(NedOrigin, RecoversTheOffsetFromNoisyPairs) {
	/// world position of the NED origin
	const Eigen::Vector3d origin(10, -5, 0.2);
	NedOrigin estimate(200);
	std::mt19937 generator(1);
	std::normal_distribution<double> noise(0.0, 0.01);
	for (int i = 0; i < 200; ++i) {
		EXPECT_FALSE(estimate.ready());
		/// the drone on the ground
		const Eigen::Vector3d world = origin + Eigen::Vector3d(1, 2, 0.05);
		const Eigen::Vector3d ned = enu_to_ned(world - origin) +
		                            Eigen::Vector3d(noise(generator), noise(generator), noise(generator));
		estimate.add(world, ned);
	}
	ASSERT_TRUE(estimate.ready());
	EXPECT_LT((estimate.offset() - origin).norm(), 0.01);
	const Eigen::Vector3d target(12, -3, 5);
	EXPECT_LT((estimate.to_world(estimate.to_ned(target)) - target).norm(), 1e-9);
	/// ignored once ready
	estimate.add(Eigen::Vector3d(100, 100, 100), Eigen::Vector3d::Zero());
	EXPECT_LT((estimate.offset() - origin).norm(), 0.01);
}
