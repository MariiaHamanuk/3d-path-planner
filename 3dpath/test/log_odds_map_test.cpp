#include "mapping/log_odds_mapper.hpp"

#include <gtest/gtest.h>

using namespace path3d;

static_assert(ReplicatedMap<LogOddsGridMap>);
static_assert(GridMap<LogOddsGridMap>);

namespace {

/// A drone at the origin looking along +x at a flat wall `distance` m away.
SensorFrame wall_frame(double distance, double stamp = 1.0) {
	SensorFrame frame;
	frame.stamp = stamp;
	frame.depth.stamp = stamp;
	frame.depth.frame_id = FrameId::DepthCamera;
	frame.depth.image = cv::Mat(12, 16, CV_32FC1, cv::Scalar(static_cast<float>(distance)));
	frame.depth_calibration.width = 16;
	frame.depth_calibration.height = 12;
	frame.depth_calibration.K = cv::Matx33d(10, 0, 8, 0, 10, 6, 0, 0, 1);
	return frame;
}

LogOddsMapperConfig test_config() {
	LogOddsMapperConfig config;
	config.pixel_stride = 1;
	return config;
}

}

TEST(LogOddsGridMap, IndexAndCenterRoundTrip) {
	const LogOddsGridMap map(0.5);
	EXPECT_EQ(map.index_of({0.1, -0.1, 1.2}), Eigen::Vector3i(0, -1, 2));
	EXPECT_TRUE(map.center_of({0, -1, 2}).isApprox(Eigen::Vector3d(0.25, -0.25, 1.25)));
	EXPECT_EQ(map.index_of(map.center_of({-7, 3, 40})), Eigen::Vector3i(-7, 3, 40));
}

TEST(LogOddsGridMap, KeyRoundTripsNegativeIndices) {
	for (const Eigen::Vector3i& index : {Eigen::Vector3i(0, 0, 0), Eigen::Vector3i(-1, 5, -300),
	                                     Eigen::Vector3i(1000000, -1000000, 7)}) {
		EXPECT_EQ(LogOddsGridMap::index_of_key(LogOddsGridMap::key_of(index)), index);
	}
}

TEST(LogOddsGridMap, UnobservedIsUnknown) {
	const LogOddsGridMap map;
	EXPECT_EQ(map.occupancy({1, 2, 3}), Occupancy::Unknown);
}

TEST(LogOddsGridMap, KeyframeReplacesAndDiffOverwrites) {
	LogOddsGridMap map;
	map.apply({.stamp = 1.0, .to_version = 1, .keyframe = true, .resolution = 0.5, .cells = {{0, 0, 0, 1.0f}, {1, 0, 0, -1.0f}}});
	EXPECT_DOUBLE_EQ(map.resolution(), 0.5);
	EXPECT_EQ(map.occupancy_at({0, 0, 0}), Occupancy::Occupied);
	EXPECT_EQ(map.occupancy_at({1, 0, 0}), Occupancy::Free);
	map.apply({.stamp = 2.0, .from_version = 1, .to_version = 2, .cells = {{0, 0, 0, -1.0f}}});
	EXPECT_EQ(map.occupancy_at({0, 0, 0}), Occupancy::Free);
	EXPECT_EQ(map.version, 2u);
	map.apply({.to_version = 3, .keyframe = true, .cells = {{5, 5, 5, 2.0f}}});
	EXPECT_EQ(map.known_count(), 1u);
	EXPECT_EQ(map.occupancy_at({0, 0, 0}), Occupancy::Unknown);
}

TEST(LogOddsMapper, WallIsOccupiedAndTheSpaceBeforeItIsFree) {
	LogOddsMapper mapper(test_config());
	EXPECT_EQ(mapper.version(), 0u);
	mapper.integrate(wall_frame(2.0));
	EXPECT_EQ(mapper.version(), 1u);
	const LogOddsGridMap& map = mapper.map();
	EXPECT_EQ(map.occupancy({2.1, 0.0, 0.0}), Occupancy::Occupied);
	EXPECT_EQ(map.occupancy({1.1, 0.0, 0.0}), Occupancy::Free);
	EXPECT_EQ(map.occupancy({0.1, 0.0, 0.0}), Occupancy::Free);
	EXPECT_EQ(map.occupancy({4.0, 0.0, 0.0}), Occupancy::Unknown);
	EXPECT_EQ(map.occupancy({-1.0, 0.0, 0.0}), Occupancy::Unknown);
}

TEST(LogOddsMapper, FollowsTheDronePoseAndTheCameraMount) {
	LogOddsMapper mapper(test_config());
	SensorFrame frame = wall_frame(2.0);
	frame.drone_pose.pose.position = Eigen::Vector3d(10, 5, 3);
	/// Yawed 90 degrees: the camera looks along +y.
	frame.drone_pose.pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()));
	mapper.integrate(frame);
	EXPECT_EQ(mapper.map().occupancy({10.0, 7.1, 3.0}), Occupancy::Occupied);
	EXPECT_EQ(mapper.map().occupancy({10.0, 6.0, 3.0}), Occupancy::Free);
}

TEST(LogOddsMapper, NoReturnOnlyCarvesFreeSpace) {
	LogOddsMapper mapper(test_config());
	SensorFrame frame = wall_frame(2.0);
	frame.depth.image.setTo(std::numeric_limits<float>::infinity());
	mapper.integrate(frame);
	EXPECT_EQ(mapper.map().occupied_count(), 0u);
	EXPECT_EQ(mapper.map().occupancy({5.1, 0.0, 0.0}), Occupancy::Free);
	EXPECT_EQ(mapper.map().occupancy({20.0, 0.0, 0.0}), Occupancy::Unknown);
}

TEST(LogOddsMapper, ReturnsCloserThanMinRangeAreIgnored) {
	LogOddsMapper mapper(test_config());
	mapper.integrate(wall_frame(0.1));
	EXPECT_EQ(mapper.version(), 0u);
}

TEST(LogOddsMapper, ValuesSaturateAndTheVersionStopsMoving) {
	LogOddsMapper mapper(test_config());
	for (int i = 0; i < 20; ++i) mapper.integrate(wall_frame(2.0, i));
	const auto version = mapper.version();
	const auto wall = mapper.map().index_of({2.1, 0.0, 0.0});
	EXPECT_FLOAT_EQ(mapper.map().log_odds_at(wall), LogOddsMapperConfig{}.max_log_odds);
	mapper.integrate(wall_frame(2.0, 30));
	EXPECT_EQ(mapper.version(), version);
	EXPECT_FALSE(mapper.make_update(false).value().cells.empty());
	EXPECT_FALSE(mapper.make_update(false).has_value());
}

TEST(LogOddsMapper, AMovedObstacleIsEventuallyFreed) {
	LogOddsMapper mapper(test_config());
	mapper.integrate(wall_frame(2.0));
	for (int i = 0; i < 10; ++i) mapper.integrate(wall_frame(4.0, 2 + i));
	EXPECT_EQ(mapper.map().occupancy({2.1, 0.0, 0.0}), Occupancy::Free);
	EXPECT_EQ(mapper.map().occupancy({4.1, 0.0, 0.0}), Occupancy::Occupied);
}

TEST(LogOddsMapper, ReplicasBuiltFromUpdatesMatchTheWorkingMap) {
	LogOddsMapper mapper(test_config());
	LogOddsGridMap replica;
	mapper.integrate(wall_frame(2.0, 1.0));
	replica.apply(*mapper.make_update(true));
	mapper.integrate(wall_frame(3.0, 2.0));
	mapper.integrate(wall_frame(2.5, 3.0));
	const auto diff = mapper.make_update(false);
	ASSERT_TRUE(diff.has_value());
	EXPECT_FALSE(diff->keyframe);
	EXPECT_EQ(diff->from_version, 1u);
	EXPECT_EQ(diff->to_version, 3u);
	replica.apply(*diff);
	EXPECT_EQ(replica.version, mapper.version());
	ASSERT_EQ(replica.known_count(), mapper.map().known_count());
	for (const auto& [key, value] : mapper.map().voxels()) {
		EXPECT_FLOAT_EQ(replica.log_odds(key), value);
	}
	LogOddsGridMap from_keyframe;
	from_keyframe.apply(*mapper.make_update(true));
	EXPECT_EQ(from_keyframe.known_count(), mapper.map().known_count());
}

TEST(LogOddsMapper, KeyframeOfAnEmptyMapExists) {
	LogOddsMapper mapper;
	const auto update = mapper.make_update(true);
	ASSERT_TRUE(update.has_value());
	EXPECT_TRUE(update->cells.empty());
}
