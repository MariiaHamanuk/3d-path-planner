#include "mapping/log_odds_grid_map.hpp"
#include "pipeline/threads.hpp"
#include "planning/line_planner.hpp"

#include <gtest/gtest.h>

#include <functional>

using namespace std::chrono_literals;
using namespace path3d;

namespace {

bool wait_for(const std::function<bool()>& done, std::chrono::milliseconds timeout) {
	const auto deadline = std::chrono::steady_clock::now() + timeout;
	while (!done()) {
		if (std::chrono::steady_clock::now() > deadline) return false;
		std::this_thread::sleep_for(5ms);
	}
	return true;
}

StampedPose pose_at(double x, double y, double z, double stamp, double yaw = 0.0) {
	StampedPose pose;
	pose.stamp = stamp;
	pose.pose.position = Eigen::Vector3d(x, y, z);
	pose.pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
	return pose;
}

/**
 * @brief Both planners with their threads, a map with the given occupied cells, and the topics.
 */
struct Fixture {
	explicit Fixture(Eigen::Vector3d goal, std::vector<LogOddsCell> occupied = {}, double speed = 2.0)
	    : map_client(map_updates),
	      global_map(map_updates),
	      uav(poses),
	      global_uav(poses),
	      global(global_map, global_uav, goal),
	      local(map_client, uav, speed),
	      trajectories_latest(trajectories.subscribe_latest()),
	      global_thread(global, pipe, std::nullopt),
	      local_thread(local, pipe, trajectories) {
		LogOddsUpdate update;
		update.keyframe = true;
		update.to_version = 1;
		update.cells = std::move(occupied);
		map_updates.publish(update);
		global_thread.start();
		local_thread.start();
	}

	Topic<LogOddsGridMap::Update> map_updates;
	Topic<StampedPose> poses;
	Topic<Trajectory> trajectories;
	Pipe<GlobalPath, ReplanRequest> pipe;
	MapClient<LogOddsGridMap> map_client;
	MapClient<LogOddsGridMap> global_map;
	UavClient uav;
	UavClient global_uav;
	LineGlobalPlanner global;
	LineLocalPlanner local;
	std::shared_ptr<LatestSlot<Trajectory>> trajectories_latest;
	GlobalPlanningThread<LogOddsGridMap, GlobalPath, ReplanRequest> global_thread;
	LocalPlanningThread<LogOddsGridMap, GlobalPath, ReplanRequest> local_thread;
};

}

TEST(LinePlanner, FliesFromTheDroneToTheGoal) {
	Fixture f(Eigen::Vector3d(10, 0, 5));
	f.poses.publish(pose_at(0, 0, 5, 1.0));
	ASSERT_TRUE(wait_for([&] { return f.trajectories_latest->latest() != nullptr; }, 5s));
	const auto trajectory = f.trajectories_latest->latest();
	ASSERT_FALSE(trajectory->points.empty());
	EXPECT_NEAR(trajectory->points.front().position.x(), 0.0, 1e-6);
	EXPECT_NEAR(trajectory->points.back().position.x(), 4.0, 1e-6);
	for (const Setpoint& point : trajectory->points) {
		EXPECT_NEAR(point.position.y(), 0.0, 1e-6);
		EXPECT_NEAR(point.position.z(), 5.0, 1e-6);
		EXPECT_NEAR(point.yaw, 0.0, 1e-6);
		EXPECT_NEAR(point.velocity.x(), 2.0, 1e-6);
	}
	/// stamps start at the pose stamp and increase
	EXPECT_NEAR(trajectory->points.front().stamp, 1.0, 1e-6);
	EXPECT_GT(trajectory->points.back().stamp, trajectory->points.front().stamp);
}

TEST(LinePlanner, PullsADriftedDroneBackOntoTheLine) {
	Fixture f(Eigen::Vector3d(10, 0, 0));
	f.poses.publish(pose_at(0, 0, 0, 1.0));
	ASSERT_TRUE(wait_for([&] { return f.trajectories_latest->latest() != nullptr; }, 5s));
	/// the drone is now 3 m to the side and 4 m along the line
	f.poses.publish(pose_at(4, 3, 0, 2.0));
	ASSERT_TRUE(wait_for([&] {
		const auto t = f.trajectories_latest->latest();
		return t && t->stamp == 2.0;
	},
	                     5s));
	const auto trajectory = f.trajectories_latest->latest();
	EXPECT_NEAR(trajectory->points.front().position.x(), 4.0, 1e-6);
	EXPECT_NEAR(trajectory->points.front().position.y(), 0.0, 1e-6);
}

TEST(LinePlanner, StopsAtTheGoal) {
	Fixture f(Eigen::Vector3d(10, 0, 0));
	f.poses.publish(pose_at(0, 0, 0, 1.0));
	ASSERT_TRUE(wait_for([&] { return f.trajectories_latest->latest() != nullptr; }, 5s));
	f.poses.publish(pose_at(9.5, 0, 0, 2.0));
	ASSERT_TRUE(wait_for([&] {
		const auto t = f.trajectories_latest->latest();
		return t && t->stamp == 2.0;
	},
	                     5s));
	const Setpoint& last = f.trajectories_latest->latest()->points.back();
	EXPECT_NEAR(last.position.x(), 10.0, 1e-6);
	EXPECT_NEAR(last.velocity.norm(), 0.0, 1e-6);
}

TEST(LinePlanner, FollowsALineAtAnAngle) {
	const double yaw = std::numbers::pi / 2;
	Fixture f(Eigen::Vector3d(0, 10, 0));
	f.poses.publish(pose_at(0, 0, 0, 1.0));
	ASSERT_TRUE(wait_for([&] { return f.trajectories_latest->latest() != nullptr; }, 5s));
	const Setpoint& last = f.trajectories_latest->latest()->points.back();
	EXPECT_NEAR(last.position.y(), 4.0, 1e-6);
	EXPECT_NEAR(last.yaw, yaw, 1e-6);
}

TEST(LinePlanner, DoesNotFlyIntoAnObstacleOnTheLine) {
	/// an occupied voxel at x = 2 m (voxel size 0.25 m: index 8)
	Fixture f(Eigen::Vector3d(10, 0, 0), {{8, 0, 0, 3.0f}});
	f.poses.publish(pose_at(0, 0, 0, 1.0));
	ASSERT_TRUE(wait_for([&] { return f.local_thread.sent_request_count() >= 1; }, 5s));
	std::this_thread::sleep_for(200ms);
	EXPECT_EQ(f.local_thread.published_trajectory_count(), 0u);
}

TEST(LinePlanner, RefusesAGoalInsideAnObstacle) {
	Fixture f(Eigen::Vector3d(10, 0, 0), {{40, 0, 0, 3.0f}});
	f.poses.publish(pose_at(0, 0, 0, 1.0));
	ASSERT_TRUE(wait_for([&] { return f.global_thread.plan_call_count() >= 1; }, 5s));
	std::this_thread::sleep_for(200ms);
	EXPECT_EQ(f.local_thread.published_trajectory_count(), 0u);
}
