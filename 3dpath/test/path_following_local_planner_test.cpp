#include "mapping/log_odds_grid_map.hpp"
#include "pipeline/threads.hpp"
#include "planning/benchmark_scenes.hpp"
#include "planning/path_following_local_planner.hpp"
#include "planning/search_planners.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <numbers>
#include <optional>
#include <vector>

using namespace std::chrono_literals;
using namespace path3d;

namespace {

using Planner = PathFollowingLocalPlanner<LogOddsGridMap>;

constexpr double kResolution = 0.5;  ///< voxel edge of the test maps, m
constexpr TimeSec kCycle = 0.1;      ///< sim time between two control cycles of a simulated flight, s

const PathFollowerConfig kConfig;  ///< the defaults, which every test uses

Eigen::Vector3d xyz(double x, double y, double z) { return Eigen::Vector3d(x, y, z); }

bool wait_for(const std::function<bool()>& done, std::chrono::milliseconds timeout) {
	const auto deadline = std::chrono::steady_clock::now() + timeout;
	while (!done()) {
		if (std::chrono::steady_clock::now() > deadline) return false;
		std::this_thread::sleep_for(5ms);
	}
	return true;
}

StampedPose pose_at(const Eigen::Vector3d& position, TimeSec stamp, double yaw = 0.0) {
	StampedPose pose;
	pose.stamp = stamp;
	pose.pose.position = position;
	pose.pose.orientation = Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
	return pose;
}

/// A keyframe of a map with `occupied` voxels; everything else is unknown.
LogOddsUpdate keyframe(std::vector<LogOddsCell> occupied, std::uint64_t version = 1) {
	LogOddsUpdate update;
	update.keyframe = true;
	update.to_version = version;
	update.resolution = kResolution;
	update.cells = std::move(occupied);
	return update;
}

/// Occupied voxels with indices from `lower` to `upper` (both included).
std::vector<LogOddsCell> voxels(const Eigen::Vector3i& lower, const Eigen::Vector3i& upper) {
	std::vector<LogOddsCell> cells;
	for (int x = lower.x(); x <= upper.x(); ++x) {
		for (int y = lower.y(); y <= upper.y(); ++y) {
			for (int z = lower.z(); z <= upper.z(); ++z) cells.push_back({.x = x, .y = y, .z = z, .log_odds = 3.0f});
		}
	}
	return cells;
}

GlobalPath route(std::vector<Eigen::Vector3d> waypoints) {
	GlobalPath path;
	path.waypoints = std::move(waypoints);
	return path;
}

GlobalPath failed() {
	GlobalPath path;
	path.status = PlanStatus::Failed;
	return path;
}

std::vector<Eigen::Vector3d> positions(const Trajectory& trajectory) {
	std::vector<Eigen::Vector3d> points;
	for (const Setpoint& point : trajectory.points) points.push_back(point.position);
	return points;
}

double distance_to_polyline(const std::vector<Eigen::Vector3d>& polyline, const Eigen::Vector3d& p) {
	double nearest = std::numeric_limits<double>::infinity();
	for (std::size_t i = 1; i < polyline.size(); ++i) {
		const Eigen::Vector3d segment = polyline[i] - polyline[i - 1];
		const double t = std::clamp((p - polyline[i - 1]).dot(segment) / segment.squaredNorm(), 0.0, 1.0);
		nearest = std::min(nearest, (polyline[i - 1] + t * segment - p).norm());
	}
	return nearest;
}

/// Whether the straight lines from point to point stay out of the occupied voxels (no margin).
bool clear_of_obstacles(const LogOddsGridMap& map, const std::vector<Eigen::Vector3d>& points) {
	SearchSpaceConfig config;
	config.margin = 0.0;
	const OccupancySpace<LogOddsGridMap> space(map, Eigen::AlignedBox3d(Eigen::Vector3d::Constant(-1000), Eigen::Vector3d::Constant(1000)), config);
	for (std::size_t i = 1; i < points.size(); ++i) {
		if (!space.segment_free(points[i - 1], points[i])) return false;
	}
	return true;
}

/// A drone in a map, with its planner called directly (no threads). The drone flies the trajectories
/// perfectly: after a cycle it is where the trajectory says it is one cycle later.
struct World {
	explicit World(std::vector<LogOddsCell> occupied = {}) : map(updates), uav(poses), planner(map, uav, kConfig) {
		updates.publish(keyframe(std::move(occupied)));
	}

	/// One control cycle at the drone's pose; then the drone moves on by one cycle.
	LocalStep<ReplanRequest> cycle(const GlobalPath* new_route = nullptr) {
		poses.publish(pose_at(position, stamp, yaw));
		EXPECT_TRUE(planner.prepare());
		LocalStep<ReplanRequest> step = planner.plan(new_route);
		if (step.trajectory) {
			if (const std::optional<Setpoint> next = step.trajectory->sample(stamp + kCycle)) position = next->position;
		}
		stamp += kCycle;
		return step;
	}

	/// Cycles while the planner follows its route (at most `max_cycles`); returns the trajectories.
	std::vector<Trajectory> fly(std::size_t max_cycles = 2000) {
		std::vector<Trajectory> flown;
		for (std::size_t i = 0; i < max_cycles && planner.state() == PathFollowerState::Following; ++i) {
			LocalStep<ReplanRequest> step = cycle();
			EXPECT_FALSE(step.request);
			flown.push_back(*step.trajectory);
		}
		return flown;
	}

	Topic<LogOddsGridMap::Update> updates;
	Topic<StampedPose> poses;
	MapClient<LogOddsGridMap> map;
	UavClient uav;
	Planner planner;
	Eigen::Vector3d position = xyz(0, 0, 2);
	TimeSec stamp = 1.0;
	double yaw = 0.0;
};

/// A hover trajectory: every point at `position` with zero velocity, from `stamp` over the whole horizon.
void expect_hover(const Trajectory& trajectory, const Eigen::Vector3d& position, TimeSec stamp) {
	ASSERT_FALSE(trajectory.points.empty());
	EXPECT_DOUBLE_EQ(trajectory.points.front().stamp, stamp);
	EXPECT_NEAR(trajectory.points.back().stamp - stamp, kConfig.horizon, 1e-9);
	for (const Setpoint& point : trajectory.points) {
		EXPECT_LT((point.position - position).norm(), 1e-9);
		EXPECT_LT(point.velocity.norm(), 1e-9);
	}
}

/// The speed changes by at most `max_acceleration` from point to point.
void expect_acceleration_limited(const Trajectory& trajectory) {
	for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
		const double change = trajectory.points[i].velocity.norm() - trajectory.points[i - 1].velocity.norm();
		EXPECT_LE(std::abs(change), kConfig.max_acceleration * kConfig.step + 1e-9) << "point " << i;
	}
}

/// Both planners with their threads, as in line_planner_test.cpp: A* and the path follower, on a map
/// with the given occupied voxels. Both use the default SearchSpaceConfig.
struct Fixture {
	explicit Fixture(Eigen::Vector3d goal, std::vector<LogOddsCell> occupied = {})
	    : local_map(map_updates),
	      global_map(map_updates),
	      uav(poses),
	      global_uav(poses),
	      global(global_map, global_uav, std::move(goal)),
	      local(local_map, uav, kConfig),
	      trajectories_latest(trajectories.subscribe_latest()),
	      global_thread(global, pipe, std::nullopt),
	      local_thread(local, pipe, trajectories) {
		map_updates.publish(keyframe(std::move(occupied)));
		global_thread.start();
		local_thread.start();
	}

	/// The newest trajectory goes somewhere (it is not a hover trajectory).
	bool moving() const {
		const auto trajectory = trajectories_latest->latest();
		return trajectory && (trajectory->points.back().position - trajectory->points.front().position).norm() > 1.0;
	}

	Topic<LogOddsGridMap::Update> map_updates;
	Topic<StampedPose> poses;
	Topic<Trajectory> trajectories;
	Pipe<GlobalPath, ReplanRequest> pipe;
	MapClient<LogOddsGridMap> local_map;
	MapClient<LogOddsGridMap> global_map;
	UavClient uav;
	UavClient global_uav;
	AStarGlobalPlanner<LogOddsGridMap> global;
	Planner local;
	std::shared_ptr<LatestSlot<Trajectory>> trajectories_latest;
	GlobalPlanningThread<LogOddsGridMap, GlobalPath, ReplanRequest> global_thread;
	LocalPlanningThread<LogOddsGridMap, GlobalPath, ReplanRequest> local_thread;
};

}

// --- Without a route ---

TEST(PathFollowingLocalPlanner, RequestsNoPathEveryCycleAndHovers) {
	World world;
	world.position = xyz(1, 2, 3);
	for (int i = 0; i < 3; ++i) {
		const TimeSec stamp = world.stamp;
		const LocalStep<ReplanRequest> step = world.cycle();
		ASSERT_TRUE(step.request);
		EXPECT_EQ(step.request->reason, ReplanReason::NoPath);
		EXPECT_DOUBLE_EQ(step.request->stamp, stamp);
		EXPECT_EQ(step.request->position, xyz(1, 2, 3));
		ASSERT_TRUE(step.trajectory);
		expect_hover(*step.trajectory, xyz(1, 2, 3), stamp);
	}
	EXPECT_EQ(world.planner.state(), PathFollowerState::Waiting);
}

TEST(PathFollowingLocalPlanner, HoverPointStaysWhereTheDroneStopped) {
	World world;
	world.position = xyz(1, 2, 3);
	world.cycle();
	/// the pose drifts: the request says where the drone is, the hover point stays
	world.position = xyz(1.5, 2, 3);
	const TimeSec stamp = world.stamp;
	const LocalStep<ReplanRequest> step = world.cycle();
	ASSERT_TRUE(step.request);
	EXPECT_EQ(step.request->position, xyz(1.5, 2, 3));
	expect_hover(*step.trajectory, xyz(1, 2, 3), stamp);
}

TEST(PathFollowingLocalPlanner, BacksOffAfterAFailedAnswerInSimTime) {
	World world;
	world.stamp = 9.9;
	ASSERT_TRUE(world.cycle().request);

	const GlobalPath answer = failed();
	world.stamp = 10.0;
	LocalStep<ReplanRequest> step = world.cycle(&answer);
	EXPECT_FALSE(step.request);
	expect_hover(*step.trajectory, xyz(0, 0, 2), 10.0);
	EXPECT_EQ(world.planner.state(), PathFollowerState::BackingOff);

	for (const TimeSec stamp : {10.5, 10.9}) {
		world.stamp = stamp;
		step = world.cycle();
		EXPECT_FALSE(step.request) << "at " << stamp;
		expect_hover(*step.trajectory, xyz(0, 0, 2), stamp);
	}

	world.stamp = 11.0;
	step = world.cycle();
	ASSERT_TRUE(step.request);
	EXPECT_EQ(step.request->reason, ReplanReason::NoPath);
	EXPECT_EQ(world.planner.state(), PathFollowerState::Waiting);
}

TEST(PathFollowingLocalPlanner, TreatsARouteWithoutTwoWaypointsAsFailed) {
	World world;
	world.cycle();
	const GlobalPath single = route({xyz(0, 0, 2)});
	LocalStep<ReplanRequest> step = world.cycle(&single);
	EXPECT_FALSE(step.request);
	EXPECT_EQ(world.planner.state(), PathFollowerState::BackingOff);

	World other;
	other.cycle();
	const GlobalPath repeated = route({xyz(0, 0, 2), xyz(0, 0, 2)});
	step = other.cycle(&repeated);
	EXPECT_FALSE(step.request);
	EXPECT_EQ(other.planner.state(), PathFollowerState::BackingOff);
}

// --- Following ---

TEST(PathFollowingLocalPlanner, TrajectoryStartsAtThePoseStampAndCoversTheHorizon) {
	World world;
	world.stamp = 5.0;
	const GlobalPath path = route({xyz(0, 0, 2), xyz(10, 0, 2)});
	const LocalStep<ReplanRequest> step = world.cycle(&path);
	EXPECT_FALSE(step.request);
	ASSERT_TRUE(step.trajectory);
	EXPECT_EQ(world.planner.state(), PathFollowerState::Following);

	const std::vector<Setpoint>& points = step.trajectory->points;
	EXPECT_DOUBLE_EQ(step.trajectory->stamp, 5.0);
	EXPECT_DOUBLE_EQ(points.front().stamp, 5.0);
	for (std::size_t i = 1; i < points.size(); ++i) EXPECT_GT(points[i].stamp, points[i - 1].stamp);
	EXPECT_GT(points.back().stamp - points.front().stamp, 0.5);
	EXPECT_NEAR(points.back().stamp - points.front().stamp, kConfig.horizon, 1e-9);
	EXPECT_EQ(points.front().position, xyz(0, 0, 2));
	EXPECT_GT(points.back().position.x(), 1.0);
}

TEST(PathFollowingLocalPlanner, FollowsAThreeDimensionalPathWithoutCuttingCorners) {
	World world;
	const std::vector<Eigen::Vector3d> waypoints{xyz(0, 0, 2), xyz(6, 0, 2), xyz(6, 6, 2), xyz(6, 6, 5), xyz(10, 6, 5)};
	const GlobalPath path = route(waypoints);
	std::vector<Trajectory> flown{*world.cycle(&path).trajectory};
	for (Trajectory& trajectory : world.fly()) flown.push_back(std::move(trajectory));

	EXPECT_EQ(world.planner.state(), PathFollowerState::Arrived);
	EXPECT_LE((world.position - waypoints.back()).norm(), kConfig.goal_tolerance);
	for (const Trajectory& trajectory : flown) {
		for (const Setpoint& point : trajectory.points) EXPECT_LT(distance_to_polyline(waypoints, point.position), 1e-6);
	}
}

TEST(PathFollowingLocalPlanner, PullsADriftedDroneBackOntoThePath) {
	World world;
	const GlobalPath path = route({xyz(0, 0, 2), xyz(10, 0, 2)});
	world.cycle(&path);
	/// 1 m to the side: the trajectory starts on the path, next to the drone
	world.position = xyz(1, 1, 2);
	const LocalStep<ReplanRequest> step = world.cycle();
	EXPECT_FALSE(step.request);
	EXPECT_LT((step.trajectory->points.front().position - xyz(1, 0, 2)).norm(), 1e-9);
	for (const Setpoint& point : step.trajectory->points) EXPECT_NEAR(point.position.y(), 0.0, 1e-9);
}

TEST(PathFollowingLocalPlanner, AsksForANewRouteWhenDeviated) {
	World world;
	const GlobalPath path = route({xyz(0, 0, 2), xyz(10, 0, 2)});
	world.cycle(&path);
	world.position = xyz(1, kConfig.deviation_distance + 0.5, 2);
	const Eigen::Vector3d deviated = world.position;
	for (int i = 0; i < 2; ++i) {
		const TimeSec stamp = world.stamp;
		const LocalStep<ReplanRequest> step = world.cycle();
		ASSERT_TRUE(step.request);
		EXPECT_EQ(step.request->reason, ReplanReason::Deviated);
		expect_hover(*step.trajectory, deviated, stamp);
	}
	EXPECT_EQ(world.planner.state(), PathFollowerState::Waiting);
}

TEST(PathFollowingLocalPlanner, ProgressDoesNotJumpToANearbyPartOfThePath) {
	World world;
	/// out along y = 0 and back along y = 1
	const GlobalPath path = route({xyz(0, 0, 2), xyz(10, 0, 2), xyz(10, 1, 2), xyz(0, 1, 2)});
	world.cycle(&path);
	for (double x = 0.5; x <= 5.0; x += 0.5) {
		world.position = xyz(x, 0, 2);
		world.cycle();
	}
	/// nearer to the way back, but that is 11 m further along the path
	world.position = xyz(5, 0.6, 2);
	const Trajectory trajectory = *world.cycle().trajectory;
	const Setpoint& first = trajectory.points.front();
	EXPECT_NEAR(first.position.x(), 5.0, 1e-9);
	EXPECT_NEAR(first.position.y(), 0.0, 1e-9);

	/// pushed back: the progress stays where it was
	world.position = xyz(3, 0, 2);
	const LocalStep<ReplanRequest> step = world.cycle();
	EXPECT_FALSE(step.request);
	EXPECT_NEAR(step.trajectory->points.front().position.x(), 5.0, 1e-9);
}

TEST(PathFollowingLocalPlanner, AcceleratesFromHoverUpToTheCruiseSpeed) {
	World world;
	const GlobalPath path = route({xyz(0, 0, 2), xyz(20, 0, 2)});
	const Trajectory trajectory = *world.cycle(&path).trajectory;
	EXPECT_NEAR(trajectory.points.front().velocity.norm(), 0.0, 1e-9);
	expect_acceleration_limited(trajectory);
	for (const Setpoint& point : trajectory.points) {
		EXPECT_LE(point.velocity.norm(), kConfig.cruise_speed + 1e-9);
		EXPECT_NEAR(point.velocity.normalized().y(), 0.0, 1e-9);
	}
	EXPECT_NEAR(trajectory.points.back().velocity.norm(), kConfig.cruise_speed, 1e-9);
}

TEST(PathFollowingLocalPlanner, SlowsDownBeforeASharpCorner) {
	World world;
	const GlobalPath path = route({xyz(0, 0, 2), xyz(12, 0, 2), xyz(12, 8, 2)});
	world.cycle(&path);
	const std::vector<Trajectory> flown = world.fly();
	ASSERT_EQ(world.planner.state(), PathFollowerState::Arrived);

	double fastest = 0.0;
	bool near_corner = false;
	for (const Trajectory& trajectory : flown) {
		expect_acceleration_limited(trajectory);
		/// the commanded state now, on the way to the corner
		const Setpoint& now = trajectory.points.front();
		if (now.position.y() > 1e-9) continue;
		const double to_corner = 12.0 - now.position.x();
		const double speed = now.velocity.norm();
		fastest = std::max(fastest, speed);
		if (to_corner > 0.9 && to_corner < 1.1) {
			EXPECT_LT(speed, 1.9);
		}
		if (to_corner > 0.0 && to_corner <= 0.1) {
			near_corner = true;
			EXPECT_LT(speed, 0.8);
			EXPECT_GT(speed, 0.0);
		}
	}
	EXPECT_GT(fastest, kConfig.cruise_speed - 0.01);
	EXPECT_TRUE(near_corner);
}

TEST(PathFollowingLocalPlanner, StopsWithZeroVelocityAtTheGoal) {
	World world;
	const Eigen::Vector3d goal = xyz(8, 0, 2);
	const GlobalPath path = route({xyz(0, 0, 2), goal});
	world.cycle(&path);
	const std::vector<Trajectory> flown = world.fly();
	ASSERT_EQ(world.planner.state(), PathFollowerState::Arrived);

	double last_speed = std::numeric_limits<double>::infinity();
	for (const Trajectory& trajectory : flown) {
		expect_acceleration_limited(trajectory);
		for (const Setpoint& point : trajectory.points) {
			if ((point.position - goal).norm() < 1e-9) {
				EXPECT_LT(point.velocity.norm(), 1e-9);
			}
		}
		/// on the last meter the commanded speed only goes down
		const Setpoint& now = trajectory.points.front();
		if (goal.x() - now.position.x() < 1.0) {
			EXPECT_LE(now.velocity.norm(), last_speed + 1e-9);
			last_speed = now.velocity.norm();
		}
	}
	/// 1 m before the goal, the trajectory already ends there, at rest
	const auto before = std::find_if(flown.begin(), flown.end(), [&](const Trajectory& t) { return goal.x() - t.points.front().position.x() < 1.0; });
	ASSERT_NE(before, flown.end());
	EXPECT_LT((before->points.back().position - goal).norm(), 1e-9);
	EXPECT_LT(before->points.back().velocity.norm(), 1e-9);
}

TEST(PathFollowingLocalPlanner, HoversAtTheGoalAfterArriving) {
	World world;
	const Eigen::Vector3d goal = xyz(5, 0, 2);
	const GlobalPath path = route({xyz(0, 0, 2), goal});
	world.cycle(&path);
	world.fly();
	ASSERT_EQ(world.planner.state(), PathFollowerState::Arrived);
	for (int i = 0; i < 3; ++i) {
		const TimeSec stamp = world.stamp;
		const LocalStep<ReplanRequest> step = world.cycle();
		EXPECT_FALSE(step.request);
		expect_hover(*step.trajectory, goal, stamp);
	}
	/// pushed away: it flies back to the goal and does not ask for a route
	world.position = xyz(5, 1, 2);
	const TimeSec stamp = world.stamp;
	const LocalStep<ReplanRequest> step = world.cycle();
	EXPECT_FALSE(step.request);
	expect_hover(*step.trajectory, goal, stamp);
}

TEST(PathFollowingLocalPlanner, YawFollowsThePathDirection) {
	World north;
	const GlobalPath to_north = route({xyz(0, 0, 2), xyz(0, 5, 2)});
	const Trajectory north_trajectory = *north.cycle(&to_north).trajectory;
	for (const Setpoint& point : north_trajectory.points) EXPECT_NEAR(point.yaw, std::numbers::pi / 2, 1e-9);

	World south_west;
	const GlobalPath to_south_west = route({xyz(0, 0, 2), xyz(-4, -4, 2)});
	const Trajectory south_west_trajectory = *south_west.cycle(&to_south_west).trajectory;
	for (const Setpoint& point : south_west_trajectory.points) EXPECT_NEAR(wrap_angle(point.yaw + 3 * std::numbers::pi / 4), 0.0, 1e-9);

	/// straight up, then east: the drone keeps its own yaw while it climbs
	World climb;
	climb.yaw = 1.0;
	const GlobalPath up_then_east = route({xyz(0, 0, 2), xyz(0, 0, 3), xyz(3, 0, 3)});
	std::vector<Trajectory> flown{*climb.cycle(&up_then_east).trajectory};
	for (Trajectory& trajectory : climb.fly()) flown.push_back(std::move(trajectory));
	ASSERT_EQ(climb.planner.state(), PathFollowerState::Arrived);
	bool climbed = false;
	bool turned = false;
	for (const Trajectory& trajectory : flown) {
		for (const Setpoint& point : trajectory.points) {
			if (point.position.z() < 3.0 - 1e-9) {
				climbed = true;
				EXPECT_NEAR(point.yaw, 1.0, 1e-9);
			}
			if (point.position.x() > 1e-6) {
				turned = true;
				EXPECT_NEAR(point.yaw, 0.0, 1e-9);
			}
		}
	}
	EXPECT_TRUE(climbed);
	EXPECT_TRUE(turned);
}

// --- Obstacles ---

/// A wall across the line y = 0 at x 2.5-3 m, from y -2 m to 2 m and up to 5 m.
std::vector<LogOddsCell> wall() { return voxels({5, -4, 0}, {5, 3, 9}); }

TEST(PathFollowingLocalPlanner, StopsAndSendsBlockedForAnObstacleWithinTheHorizon) {
	World world;
	const GlobalPath path = route({xyz(0, 0, 2), xyz(10, 0, 2)});
	ASSERT_FALSE(world.cycle(&path).request);

	world.updates.publish(keyframe(wall(), 2));
	const Eigen::Vector3d stopped = world.position;
	for (int i = 0; i < 2; ++i) {
		const TimeSec stamp = world.stamp;
		const LocalStep<ReplanRequest> step = world.cycle();
		ASSERT_TRUE(step.request);
		EXPECT_EQ(step.request->reason, ReplanReason::Blocked);
		ASSERT_TRUE(step.trajectory);
		expect_hover(*step.trajectory, stopped, stamp);
	}
	EXPECT_EQ(world.planner.state(), PathFollowerState::Waiting);
}

TEST(PathFollowingLocalPlanner, ResumesFollowingWithANewRoute) {
	World world;
	const GlobalPath path = route({xyz(0, 0, 2), xyz(10, 0, 2)});
	world.cycle(&path);
	world.updates.publish(keyframe(wall(), 2));
	ASSERT_TRUE(world.cycle().request);

	const GlobalPath around = route({world.position, xyz(1, -3.5, 2), xyz(6, -3.5, 2), xyz(10, 0, 2)});
	const LocalStep<ReplanRequest> step = world.cycle(&around);
	EXPECT_FALSE(step.request);
	EXPECT_EQ(world.planner.state(), PathFollowerState::Following);
	EXPECT_GT((step.trajectory->points.back().position - step.trajectory->points.front().position).norm(), 1.0);

	world.fly();
	EXPECT_EQ(world.planner.state(), PathFollowerState::Arrived);
}

TEST(PathFollowingLocalPlanner, EscapesFromInsideTheMarginBeforeAskingForARoute) {
	/// the drone is in voxel (0, 0, 4), next to an occupied one: inside the 0.5 m margin
	World world({{.x = 1, .y = 0, .z = 4, .log_odds = 3.0f}});
	const Eigen::Vector3d start = xyz(0.25, 0.25, 2.25);
	world.position = start;
	const LocalStep<ReplanRequest> step = world.cycle();
	EXPECT_FALSE(step.request);
	EXPECT_EQ(world.planner.state(), PathFollowerState::Escaping);
	const Trajectory& trajectory = *step.trajectory;
	EXPECT_EQ(trajectory.points.front().position, start);
	EXPECT_GT((trajectory.points.back().position - start).norm(), 0.1);
	for (const Setpoint& point : trajectory.points) EXPECT_LE(point.velocity.norm(), kConfig.escape_speed + 1e-9);
	EXPECT_TRUE(clear_of_obstacles(world.map.map(), positions(trajectory)));

	/// out of the margin, it asks
	std::optional<ReplanRequest> request;
	for (int i = 0; i < 20 && !request; ++i) request = world.cycle().request;
	ASSERT_TRUE(request);
	EXPECT_EQ(request->reason, ReplanReason::NoPath);
	EXPECT_GT((request->position - start).norm(), 0.1);
	EXPECT_EQ(world.planner.state(), PathFollowerState::Waiting);
}

TEST(PathFollowingLocalPlanner, EscapesWhenBlockedInsideTheMarginInsteadOfDeadlocking) {
	World world;
	world.position = xyz(0.25, 0.25, 2.25);
	const GlobalPath path = route({world.position, xyz(10.25, 0.25, 2.25)});
	ASSERT_FALSE(world.cycle(&path).request);

	/// an obstacle appears right next to the drone: its own position is not free any more, so A*
	/// could not start a path here; it moves out first and asks then
	world.updates.publish(keyframe({{.x = 0, .y = -1, .z = 4, .log_odds = 3.0f}}, 2));
	LocalStep<ReplanRequest> step = world.cycle();
	EXPECT_FALSE(step.request);
	EXPECT_EQ(world.planner.state(), PathFollowerState::Escaping);

	for (int i = 0; i < 20 && !step.request; ++i) step = world.cycle();
	ASSERT_TRUE(step.request);
	EXPECT_EQ(step.request->reason, ReplanReason::Blocked);

	const GlobalPath away = route({world.position, xyz(10, 5, 4)});
	EXPECT_FALSE(world.cycle(&away).request);
	EXPECT_EQ(world.planner.state(), PathFollowerState::Following);
}

// --- With the threads ---

TEST(PathFollowingLocalPlannerThreads, AsksForARouteAndFollowsIt) {
	Fixture f(xyz(10, 0, 2));
	f.poses.publish(pose_at(xyz(0, 0, 2), 1.0));
	ASSERT_TRUE(wait_for([&] { return f.moving(); }, 5s));
	EXPECT_GE(f.local_thread.sent_request_count(), 1u);
	EXPECT_GE(f.global_thread.plan_call_count(), 1u);
	const auto trajectory = f.trajectories_latest->latest();
	EXPECT_DOUBLE_EQ(trajectory->points.front().stamp, 1.0);
	EXPECT_GT(trajectory->points.back().position.x(), 1.0);
}

TEST(PathFollowingLocalPlannerThreads, ReplansAroundAnObstacleThatAppears) {
	Fixture f(xyz(10, 0, 2));
	f.poses.publish(pose_at(xyz(0, 0, 2), 1.0));
	ASSERT_TRUE(wait_for([&] { return f.moving(); }, 5s));

	/// the trajectory along y = 0 now runs into the wall
	const LogOddsUpdate update = keyframe(wall(), 2);
	LogOddsGridMap map;
	map.apply(update);
	f.map_updates.publish(update);
	ASSERT_TRUE(wait_for([&] {
		const auto trajectory = f.trajectories_latest->latest();
		return f.moving() && clear_of_obstacles(map, positions(*trajectory));
	},
	                     5s));
	EXPECT_GE(f.local_thread.sent_request_count(), 2u);
	EXPECT_GE(f.global_thread.plan_call_count(), 2u);
}

// --- End to end ---

TEST(PathFollowingLocalPlanner, ReachesTheGoalOfEveryBenchmarkSceneWithAStar) {
	/// one SearchSpaceConfig for both planners, as in the demo
	const SearchSpaceConfig space = kConfig.space;
	for (const Scene& scene : benchmark_scenes()) {
		SCOPED_TRACE(scene.name);
		Topic<LogOddsGridMap::Update> updates;
		Topic<StampedPose> poses;
		MapClient<LogOddsGridMap> local_map(updates);
		MapClient<LogOddsGridMap> global_map(updates);
		UavClient uav(poses);
		UavClient global_uav(poses);
		updates.publish(scene_map(scene, kResolution));
		AStarGlobalPlanner<LogOddsGridMap> global(global_map, global_uav, scene.goal, SearchPlannerConfig{.space = space, .max_altitude = scene.max_altitude});
		Planner local(local_map, uav, kConfig);
		ASSERT_TRUE(global.prepare());

		Eigen::Vector3d position = scene.start;
		TimeSec stamp = 0.0;
		std::uint64_t last_id = 0;
		std::optional<GlobalPath> answer;
		std::vector<Eigen::Vector3d> flown{position};
		for (int i = 0; i < 5000 && local.state() != PathFollowerState::Arrived; ++i) {
			poses.publish(pose_at(position, stamp));
			ASSERT_TRUE(local.prepare());
			LocalStep<ReplanRequest> step = local.plan(answer ? &*answer : nullptr);
			answer.reset();
			/// the pipe: the request gets a number, and the answer reaches the next cycle
			if (step.request) {
				step.request->id = ++last_id;
				answer = global.plan(*step.request);
				answer->request_id = last_id;
			}
			ASSERT_TRUE(step.trajectory);
			const std::optional<Setpoint> next = step.trajectory->sample(stamp + kCycle);
			ASSERT_TRUE(next);
			position = next->position;
			stamp += kCycle;
			flown.push_back(position);
		}
		EXPECT_EQ(local.state(), PathFollowerState::Arrived);
		EXPECT_LE((position - scene.goal).norm(), kConfig.goal_tolerance);
		EXPECT_TRUE(clear_of_obstacles(local_map.map(), flown));
		/// the first request (NoPath) only: on a known map no path the global planner makes looks blocked
		EXPECT_EQ(last_id, 1u);
	}
}
