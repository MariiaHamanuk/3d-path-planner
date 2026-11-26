#include "mapping/log_odds_grid_map.hpp"
#include "pipeline/threads.hpp"
#include "planning/global/search_planners.hpp"
#include "planning/local/trajectory_local_planner.hpp"

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

using Planner = TrajectoryLocalPlanner<LogOddsGridMap>;
using State = TrajectoryPlannerState;

constexpr double kResolution = 0.5;  ///< voxel edge of the test maps, m
constexpr TimeSec kCycle = 0.1;      ///< sim time between two control cycles of a simulated flight, s

const TrajectoryLocalPlannerConfig kConfig;  ///< the defaults, which every test uses

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

/// Whether the straight lines from point to point stay out of the occupied voxels (no margin).
bool clear_of_obstacles(const LogOddsGridMap& map, const std::vector<Eigen::Vector3d>& points) {
	const OccupancySpace<LogOddsGridMap> space(map, Eigen::AlignedBox3d(Eigen::Vector3d::Constant(-1000), Eigen::Vector3d::Constant(1000)), 0.0, true);
	for (std::size_t i = 1; i < points.size(); ++i) {
		if (!space.segment_free(points[i - 1], points[i])) return false;
	}
	return true;
}

/// A drone in a map, with its planner called directly (no threads). The drone flies the trajectories
/// perfectly: after a cycle it is where the trajectory says it is one cycle later.
struct World {
	explicit World(std::vector<LogOddsCell> occupied = {}) : map(updates), uav(poses), planner(map, uav, kConfig, &reports) {
		updates.publish(keyframe(std::move(occupied)));
	}

	/// One control cycle at the drone's pose; then the drone moves on by one cycle.
	LocalStep<ReplanRequest> cycle(const GlobalPath* new_route = nullptr) {
		poses.publish(pose_at(position, stamp, yaw));
		EXPECT_TRUE(planner.prepare());
		LocalStep<ReplanRequest> step = planner.plan(new_route);
		if (step.trajectory) {
			if (const std::optional<Setpoint> next = step.trajectory->sample(stamp + cycle_time)) position = next->position;
		}
		stamp += cycle_time;
		return step;
	}

	/// Cycles while the planner follows its route (at most `max_cycles`); returns the trajectories.
	std::vector<Trajectory> fly(std::size_t max_cycles = 2000) {
		std::vector<Trajectory> flown;
		for (std::size_t i = 0; i < max_cycles && planner.state() == State::Following; ++i) {
			LocalStep<ReplanRequest> step = cycle();
			EXPECT_FALSE(step.request);
			flown.push_back(*step.trajectory);
		}
		return flown;
	}

	/// Cycles until the commanded speed reaches `speed` (at most `max_cycles`).
	void speed_up_to(double speed, std::size_t max_cycles = 100) {
		for (std::size_t i = 0; i < max_cycles; ++i) {
			const LocalStep<ReplanRequest> step = cycle();
			if (step.trajectory->points.front().velocity.norm() >= speed - 1e-9) return;
		}
		ADD_FAILURE() << "never reached " << speed << " m/s";
	}

	Topic<LogOddsGridMap::Update> updates;
	Topic<StampedPose> poses;
	Topic<TrajectoryPlannerReport> reports;
	std::shared_ptr<LatestSlot<TrajectoryPlannerReport>> report = reports.subscribe_latest();
	MapClient<LogOddsGridMap> map;
	UavClient uav;
	Planner planner;
	Eigen::Vector3d position = xyz(0, 0, 2);
	TimeSec stamp = 1.0;
	double yaw = 0.0;
	/// sim time between two cycles; in the simulation, with a real-time factor below 1, much less
	/// than the time between the points of a trajectory
	TimeSec cycle_time = kCycle;
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

/// The speed changes by at most `acceleration` * step from point to point.
void expect_acceleration_limited(const Trajectory& trajectory, double acceleration = kConfig.max_acceleration) {
	for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
		const double change = trajectory.points[i].velocity.norm() - trajectory.points[i - 1].velocity.norm();
		EXPECT_LE(std::abs(change), acceleration * kConfig.step + 1e-6) << "point " << i;
	}
}

/// Both planners with their threads, as in line_planner_test.cpp: A* and the local planner, on a map
/// with the given occupied voxels. Both use the default margin and unknown-space settings.
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

/// A wall across the line y = 0 at x 2.5-3 m, from y -2 m to 2 m and up to 5 m.
std::vector<LogOddsCell> wall() { return voxels({5, -4, 0}, {5, 3, 9}); }

/// A wall across the line y = 0, from x `x` m on (0.5 m thick), from y -2 m to 2 m and up to 5 m.
std::vector<LogOddsCell> wall_at(double x) {
	const int i = static_cast<int>(std::floor(x / kResolution));
	return voxels({i, -4, 0}, {i, 3, 9});
}

}

// --- Without a route ---

TEST(TrajectoryLocalPlanner, RequestsNoPathEveryCycleAndHovers) {
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
	EXPECT_EQ(world.planner.state(), State::Waiting);
}

TEST(TrajectoryLocalPlanner, HoverPointStaysWhereTheDroneStopped) {
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

TEST(TrajectoryLocalPlanner, BacksOffAfterAFailedAnswerInSimTime) {
	World world;
	world.stamp = 9.9;
	ASSERT_TRUE(world.cycle().request);

	const GlobalPath answer = failed();
	world.stamp = 10.0;
	LocalStep<ReplanRequest> step = world.cycle(&answer);
	EXPECT_FALSE(step.request);
	expect_hover(*step.trajectory, xyz(0, 0, 2), 10.0);
	EXPECT_EQ(world.planner.state(), State::BackingOff);

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
	EXPECT_EQ(world.planner.state(), State::Waiting);
}

TEST(TrajectoryLocalPlanner, TreatsARouteWithoutTwoWaypointsAsFailed) {
	World world;
	world.cycle();
	const GlobalPath single = route({xyz(0, 0, 2)});
	LocalStep<ReplanRequest> step = world.cycle(&single);
	EXPECT_FALSE(step.request);
	EXPECT_EQ(world.planner.state(), State::BackingOff);

	World other;
	other.cycle();
	const GlobalPath repeated = route({xyz(0, 0, 2), xyz(0, 0, 2)});
	step = other.cycle(&repeated);
	EXPECT_FALSE(step.request);
	EXPECT_EQ(other.planner.state(), State::BackingOff);
}

// --- Following ---

TEST(TrajectoryLocalPlanner, TrajectoryStartsAtThePoseStampAndCoversTheHorizon) {
	World world;
	world.stamp = 5.0;
	const GlobalPath path = route({xyz(0, 0, 2), xyz(10, 0, 2)});
	const LocalStep<ReplanRequest> step = world.cycle(&path);
	EXPECT_FALSE(step.request);
	ASSERT_TRUE(step.trajectory);
	EXPECT_EQ(world.planner.state(), State::Following);

	const std::vector<Setpoint>& points = step.trajectory->points;
	EXPECT_DOUBLE_EQ(step.trajectory->stamp, 5.0);
	EXPECT_DOUBLE_EQ(points.front().stamp, 5.0);
	for (std::size_t i = 1; i < points.size(); ++i) EXPECT_GT(points[i].stamp, points[i - 1].stamp);
	EXPECT_NEAR(points.back().stamp - points.front().stamp, kConfig.horizon, 1e-9);
	EXPECT_LT((points.front().position - xyz(0, 0, 2)).norm(), 1e-9);
	EXPECT_GT(points.back().position.x(), 1.0);
}

TEST(TrajectoryLocalPlanner, ShortcutsWaypointsThatSeeEachOther) {
	World world;
	const GlobalPath path = route({xyz(0, 0, 2), xyz(6, 0, 2), xyz(6, 6, 2), xyz(6, 6, 5), xyz(10, 6, 5)});
	world.cycle(&path);
	/// nothing in the way: one straight line from the start to the goal
	const Eigen::Vector3d direction = (xyz(10, 6, 5) - xyz(0, 0, 2)).normalized();
	for (const Eigen::Vector3d& point : world.planner.path()) {
		const Eigen::Vector3d offset = point - xyz(0, 0, 2);
		EXPECT_LT((offset - direction * offset.dot(direction)).norm(), 1e-9);
	}
	world.fly();
	EXPECT_EQ(world.planner.state(), State::Arrived);
	EXPECT_LE((world.position - xyz(10, 6, 5)).norm(), kConfig.goal_tolerance);
}

TEST(TrajectoryLocalPlanner, RoundsAndSlowsDownForACornerAroundAnObstacle) {
	/// a block that the straight line from the start to the goal runs into
	World world(voxels({24, 4, 0}, {29, 15, 9}));
	const Eigen::Vector3d corner = xyz(16, 0, 2);
	const GlobalPath path = route({xyz(0, 0, 2), corner, xyz(16, 8, 2)});
	std::vector<Trajectory> flown{*world.cycle(&path).trajectory};
	for (Trajectory& trajectory : world.fly()) flown.push_back(std::move(trajectory));
	ASSERT_EQ(world.planner.state(), State::Arrived);

	double fastest = 0.0;
	double slowest_in_turn = std::numeric_limits<double>::infinity();
	double nearest_to_corner = std::numeric_limits<double>::infinity();
	std::vector<Eigen::Vector3d> commanded;
	for (const Trajectory& trajectory : flown) {
		expect_acceleration_limited(trajectory);
		const Setpoint& now = trajectory.points.front();
		commanded.push_back(now.position);
		fastest = std::max(fastest, now.velocity.norm());
		nearest_to_corner = std::min(nearest_to_corner, (now.position - corner).norm());
		if (now.position.x() > 14.5 && now.position.y() < 1.5) slowest_in_turn = std::min(slowest_in_turn, now.velocity.norm());
	}
	EXPECT_GT(fastest, kConfig.max_speed - 0.01);
	/// the curve has a radius of ~1.4 m: sqrt(2 m/s^2 * 1.4 m) = ~1.7 m/s
	EXPECT_LT(slowest_in_turn, 2.0);
	EXPECT_GT(slowest_in_turn, 0.5);
	/// the corner itself is cut
	EXPECT_GT(nearest_to_corner, 0.3);
	EXPECT_TRUE(clear_of_obstacles(world.map.map(), commanded));
}

TEST(TrajectoryLocalPlanner, KeepsTheDriftedDroneOnThePath) {
	World world;
	const GlobalPath path = route({xyz(0, 0, 2), xyz(10, 0, 2)});
	world.cycle(&path);
	/// 1 m to the side, less than the deviation distance: the trajectory stays on the path
	world.position = xyz(world.position.x(), 1, 2);
	const LocalStep<ReplanRequest> step = world.cycle();
	EXPECT_FALSE(step.request);
	for (const Setpoint& point : step.trajectory->points) EXPECT_NEAR(point.position.y(), 0.0, 1e-9);
}

TEST(TrajectoryLocalPlanner, AsksForANewRouteWhenDeviated) {
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
		EXPECT_EQ(step.request->position, deviated);
		expect_hover(*step.trajectory, deviated, stamp);
	}
	EXPECT_EQ(world.planner.state(), State::Waiting);
}

TEST(TrajectoryLocalPlanner, DoesNotJumpToANearbyPartOfThePath) {
	/// out along z = 2, back along z = 5.5, with a slab between the two that keeps the path from
	/// being shortened
	World world(voxels({-2, -4, 6}, {15, 3, 7}));
	const GlobalPath path = route({xyz(0, 0, 2), xyz(10, 0, 2), xyz(10, 0, 5.5), xyz(0, 0, 5.5)});
	world.cycle(&path);
	for (int i = 0; i < 12; ++i) ASSERT_FALSE(world.cycle().request);
	ASSERT_LT(world.position.x(), 6.0);

	/// 3 m above the way out but 0.5 m below the way back, which is 15 m further along: not a
	/// place on the path, so it asks for a new route
	world.position.z() = 5.0;
	const LocalStep<ReplanRequest> step = world.cycle();
	ASSERT_TRUE(step.request);
	EXPECT_EQ(step.request->reason, ReplanReason::Deviated);
	EXPECT_EQ(world.planner.state(), State::Waiting);
}

TEST(TrajectoryLocalPlanner, AcceleratesFromHoverUpToTheMaxSpeed) {
	World world;
	const GlobalPath path = route({xyz(0, 0, 2), xyz(30, 0, 2)});
	const Trajectory trajectory = *world.cycle(&path).trajectory;
	EXPECT_NEAR(trajectory.points.front().velocity.norm(), 0.0, 1e-9);
	expect_acceleration_limited(trajectory);
	for (const Setpoint& point : trajectory.points) {
		EXPECT_LE(point.velocity.norm(), kConfig.max_speed + 1e-9);
		EXPECT_NEAR(point.velocity.y(), 0.0, 1e-9);
		EXPECT_NEAR(point.yaw, 0.0, 1e-9);
	}
	EXPECT_NEAR(trajectory.points.back().velocity.norm(), kConfig.max_speed, 1e-9);
}

TEST(TrajectoryLocalPlanner, StopsWithZeroVelocityAtTheGoal) {
	World world;
	const Eigen::Vector3d goal = xyz(12, 0, 2);
	const GlobalPath path = route({xyz(0, 0, 2), goal});
	world.cycle(&path);
	const std::vector<Trajectory> flown = world.fly();
	ASSERT_EQ(world.planner.state(), State::Arrived);

	double last_speed = std::numeric_limits<double>::infinity();
	for (const Trajectory& trajectory : flown) {
		expect_acceleration_limited(trajectory);
		for (const Setpoint& point : trajectory.points) {
			EXPECT_LE(point.position.x(), goal.x() + 1e-9);
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

TEST(TrajectoryLocalPlanner, ArrivesAndStopsForObstaclesWhenItCyclesFasterThanTheTrajectoryStep) {
	/// 0.01 s of sim time per cycle, as in a simulation running at a fifth of real time
	World world;
	world.cycle_time = 0.01;
	const GlobalPath path = route({xyz(0, 0, 2), xyz(8, 0, 2)});
	world.cycle(&path);
	world.fly(20000);
	EXPECT_EQ(world.planner.state(), State::Arrived);

	World blocked;
	blocked.cycle_time = 0.01;
	const GlobalPath through = route({xyz(0, 0, 2), xyz(10, 0, 2)});
	blocked.cycle(&through);
	blocked.updates.publish(keyframe(wall(), 2));
	for (int i = 0; i < 2000 && blocked.planner.state() == State::Following; ++i) blocked.cycle();
	/// stopped in front of the wall, and hovers there asking for a route
	EXPECT_EQ(blocked.planner.state(), State::Waiting);
}

TEST(TrajectoryLocalPlanner, HoversAtTheGoalAfterArriving) {
	World world;
	const Eigen::Vector3d goal = xyz(5, 0, 2);
	const GlobalPath path = route({xyz(0, 0, 2), goal});
	world.cycle(&path);
	world.fly();
	ASSERT_EQ(world.planner.state(), State::Arrived);
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

TEST(TrajectoryLocalPlanner, TurnsTheYawTowardsThePathAtTheMaxYawRate) {
	World north;
	const GlobalPath to_north = route({xyz(0, 0, 2), xyz(0, 10, 2)});
	const Trajectory trajectory = *north.cycle(&to_north).trajectory;
	EXPECT_NEAR(trajectory.points.front().yaw, 0.0, 1e-9);
	for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
		EXPECT_LE(std::abs(wrap_angle(trajectory.points[i].yaw - trajectory.points[i - 1].yaw)), kConfig.max_yaw_rate * kConfig.step + 1e-9);
	}
	/// a quarter turn takes ~1.6 s, less than the horizon
	EXPECT_NEAR(trajectory.points.back().yaw, std::numbers::pi / 2, 1e-9);

	/// straight up: the drone keeps its own yaw
	World climb;
	climb.yaw = 1.0;
	const GlobalPath up = route({xyz(0, 0, 2), xyz(0, 0, 6)});
	const Trajectory climbing = *climb.cycle(&up).trajectory;
	for (const Setpoint& point : climbing.points) EXPECT_NEAR(point.yaw, 1.0, 1e-9);
}

// --- Obstacles ---

TEST(TrajectoryLocalPlanner, BrakesBeforeAnObstacleAndAsksForANewRoute) {
	World world;
	const GlobalPath path = route({xyz(0, 0, 2), xyz(10, 0, 2)});
	ASSERT_FALSE(world.cycle(&path).request);

	/// the margin of the wall starts at x = 2 m: the drone stops `stop_buffer` before it
	world.updates.publish(keyframe(wall(), 2));
	const double stop_x = 2.0 - kConfig.stop_buffer;
	for (int i = 0; i < 30; ++i) {
		const LocalStep<ReplanRequest> step = world.cycle();
		ASSERT_TRUE(step.request);
		EXPECT_EQ(step.request->reason, ReplanReason::Blocked);
		EXPECT_LE(step.request->position.x(), stop_x + 1e-9);
		expect_acceleration_limited(*step.trajectory);
		for (const Setpoint& point : step.trajectory->points) EXPECT_LE(point.position.x(), stop_x + 1e-9);
	}
	/// stopped: hovers where it stopped, and asks from there
	EXPECT_EQ(world.planner.state(), State::Waiting);
	const Eigen::Vector3d stopped = world.position;
	const LocalStep<ReplanRequest> step = world.cycle();
	expect_hover(*step.trajectory, stopped, step.trajectory->stamp);
	EXPECT_EQ(step.request->position, stopped);
	EXPECT_GT(stopped.x(), stop_x - 0.3);
}

TEST(TrajectoryLocalPlanner, BrakesHardButFeasiblyForAnObstacleInsideTheBrakingDistance) {
	World world;
	const GlobalPath path = route({xyz(0, 0, 2), xyz(40, 0, 2)});
	world.cycle(&path);
	world.speed_up_to(kConfig.max_speed);

	/// 3.5-4 m ahead: braking with max_acceleration from 5 m/s takes 5 m
	const double wall_x = world.position.x() + 4.0;
	world.updates.publish(keyframe(wall_at(wall_x), 2));
	std::vector<Eigen::Vector3d> commanded{world.position};
	for (int i = 0; i < 40 && world.planner.state() == State::Following; ++i) {
		const LocalStep<ReplanRequest> step = world.cycle();
		/// (the cycle that stops following plans the next state's trajectory)
		if (world.planner.state() == State::Following) expect_acceleration_limited(*step.trajectory, kConfig.emergency_deceleration);
		for (const Setpoint& point : step.trajectory->points) {
			EXPECT_LT(point.position.x(), std::floor(wall_x / kResolution) * kResolution - kConfig.emergency_clearance + 1e-6);
		}
		commanded.push_back(step.trajectory->points.front().position);
	}
	EXPECT_NE(world.planner.state(), State::Following);
	EXPECT_TRUE(clear_of_obstacles(world.map.map(), commanded));
	const Trajectory trajectory = *world.cycle().trajectory;
	EXPECT_LT(trajectory.points.front().velocity.norm(), kConfig.escape_speed + 1e-9);
}

TEST(TrajectoryLocalPlanner, ReportsTheBlockageAndTheEmergencyStop) {
	World world;
	const GlobalPath path = route({xyz(0, 0, 2), xyz(40, 0, 2)});
	world.cycle(&path);
	ASSERT_TRUE(world.report->latest());
	EXPECT_TRUE(world.report->latest()->route);
	EXPECT_EQ(world.report->latest()->state, State::Following);
	world.speed_up_to(kConfig.max_speed);
	EXPECT_FALSE(world.report->latest()->blocked);
	EXPECT_TRUE(std::isinf(world.report->latest()->clearance));

	const double wall_x = std::floor((world.position.x() + 3.5) / kResolution) * kResolution;
	world.updates.publish(keyframe(wall_at(wall_x), 2));
	const Eigen::Vector3d before = world.position;
	world.cycle();
	const auto report = world.report->latest();
	EXPECT_TRUE(report->blocked);
	EXPECT_TRUE(report->emergency);
	EXPECT_NEAR(report->obstacle_distance, wall_x - before.x(), kConfig.path_step);
	EXPECT_NEAR(report->margin_distance, wall_x - kResolution - before.x(), kConfig.path_step);
	EXPECT_NEAR(report->braking_distance, kConfig.max_speed * kConfig.max_speed / (2 * kConfig.max_acceleration), 1e-6);
	ASSERT_TRUE(report->stop_point);
	EXPECT_LT(report->stop_point->x(), wall_x);
	EXPECT_EQ(report->request, ReplanReason::Blocked);
	ASSERT_TRUE(report->path);
	EXPECT_EQ(report->path->size(), world.planner.path().size());
}

TEST(TrajectoryLocalPlanner, NeverFliesOnIntoAnObstacleFromInsideItsMargin) {
	World world;
	const GlobalPath path = route({xyz(0.25, 0.25, 2.25), xyz(20.25, 0.25, 2.25)});
	world.position = xyz(0.25, 0.25, 2.25);
	world.cycle(&path);
	world.speed_up_to(1.0);

	/// an obstacle in the voxel right ahead: the drone is inside its margin at once
	const int ahead = static_cast<int>(std::floor(world.position.x() / kResolution)) + 1;
	world.updates.publish(keyframe(voxels({ahead, 0, 4}, {ahead, 0, 4}), 2));
	std::vector<Eigen::Vector3d> flown{world.position};
	bool escaped = false;
	for (int i = 0; i < 40 && !escaped; ++i) {
		const LocalStep<ReplanRequest> step = world.cycle();
		EXPECT_TRUE(clear_of_obstacles(world.map.map(), positions(*step.trajectory)));
		for (const Setpoint& point : step.trajectory->points) EXPECT_LT(point.position.x(), ahead * kResolution);
		flown.push_back(world.position);
		escaped = world.planner.state() == State::Escaping;
	}
	EXPECT_TRUE(escaped);
	EXPECT_TRUE(clear_of_obstacles(world.map.map(), flown));
}

TEST(TrajectoryLocalPlanner, ResumesFollowingWithANewRoute) {
	World world;
	const GlobalPath path = route({xyz(0, 0, 2), xyz(10, 0, 2)});
	world.cycle(&path);
	world.updates.publish(keyframe(wall(), 2));
	ASSERT_TRUE(world.cycle().request);

	const GlobalPath around = route({world.position, xyz(1, -3.5, 2), xyz(6, -3.5, 2), xyz(10, 0, 2)});
	const LocalStep<ReplanRequest> step = world.cycle(&around);
	EXPECT_FALSE(step.request);
	EXPECT_EQ(world.planner.state(), State::Following);
	EXPECT_GT((step.trajectory->points.back().position - step.trajectory->points.front().position).norm(), 1.0);

	std::vector<Eigen::Vector3d> flown{world.position};
	for (const Trajectory& trajectory : world.fly()) flown.push_back(trajectory.points.front().position);
	EXPECT_EQ(world.planner.state(), State::Arrived);
	EXPECT_TRUE(clear_of_obstacles(world.map.map(), flown));
}

TEST(TrajectoryLocalPlanner, KeepsTheSpeedWhenANewRouteArrives) {
	World world;
	const GlobalPath path = route({xyz(0, 0, 2), xyz(40, 0, 2)});
	world.cycle(&path);
	world.speed_up_to(3.0);

	/// the same line, from a point ahead (as the answer to a request made from there)
	const GlobalPath ahead = route({world.position + xyz(2, 0, 0), xyz(40, 0, 2)});
	const Trajectory trajectory = *world.cycle(&ahead).trajectory;
	EXPECT_GT(trajectory.points.front().velocity.norm(), 3.0 - 1e-9);
	expect_acceleration_limited(trajectory);
}

TEST(TrajectoryLocalPlanner, EscapesFromInsideTheMarginBeforeAskingForARoute) {
	/// the drone is in voxel (0, 0, 4), next to an occupied one: inside the 0.5 m margin
	World world({{.x = 1, .y = 0, .z = 4, .log_odds = 3.0f}});
	const Eigen::Vector3d start = xyz(0.25, 0.25, 2.25);
	world.position = start;
	const LocalStep<ReplanRequest> step = world.cycle();
	EXPECT_FALSE(step.request);
	EXPECT_EQ(world.planner.state(), State::Escaping);
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
	EXPECT_EQ(world.planner.state(), State::Waiting);
}

TEST(TrajectoryLocalPlanner, EscapesWhenBlockedInsideTheMarginInsteadOfDeadlocking) {
	World world;
	world.position = xyz(0.25, 0.25, 2.25);
	const GlobalPath path = route({world.position, xyz(10.25, 0.25, 2.25)});
	ASSERT_FALSE(world.cycle(&path).request);

	/// an obstacle appears right next to the drone: its own position is not free any more, so a
	/// global planner could not start a path here; it stops, moves out and asks then
	world.updates.publish(keyframe({{.x = 0, .y = -1, .z = 4, .log_odds = 3.0f}}, 2));
	for (int i = 0; i < 5 && world.planner.state() != State::Escaping; ++i) world.cycle();
	ASSERT_EQ(world.planner.state(), State::Escaping);

	LocalStep<ReplanRequest> step;
	for (int i = 0; i < 20 && !step.request; ++i) step = world.cycle();
	ASSERT_TRUE(step.request);
	EXPECT_EQ(step.request->reason, ReplanReason::Blocked);

	const GlobalPath away = route({world.position, xyz(10, 5, 4)});
	EXPECT_FALSE(world.cycle(&away).request);
	EXPECT_EQ(world.planner.state(), State::Following);
}

TEST(TrajectoryLocalPlanner, WaitsAfterAFailedAnswerToABlockedRouteAndAsksAgainLater) {
	World world;
	const GlobalPath path = route({xyz(0, 0, 2), xyz(10, 0, 2)});
	world.cycle(&path);
	world.updates.publish(keyframe(wall(), 2));
	ASSERT_TRUE(world.cycle().request);

	const GlobalPath answer = failed();
	const TimeSec failed_at = world.stamp;
	EXPECT_FALSE(world.cycle(&answer).request);
	while (world.stamp < failed_at + kConfig.replan_backoff - 1e-9) EXPECT_FALSE(world.cycle().request) << "at " << world.stamp;
	const LocalStep<ReplanRequest> step = world.cycle();
	ASSERT_TRUE(step.request);
	EXPECT_EQ(step.request->reason, ReplanReason::Blocked);
}

TEST(TrajectoryLocalPlanner, WaitsAfterARouteThatIsBlockedAtOnce) {
	World world;
	const GlobalPath path = route({xyz(0, 0, 2), xyz(10, 0, 2)});
	world.cycle(&path);
	world.updates.publish(keyframe(wall(), 2));
	for (int i = 0; i < 40 && world.planner.state() != State::Waiting; ++i) world.cycle();
	ASSERT_EQ(world.planner.state(), State::Waiting);

	/// the same way through the wall again (a global planner with an old map): asking again at once
	/// would get the same answer, over and over
	const GlobalPath same = route({world.position, xyz(10, 0, 2)});
	const TimeSec answered = world.stamp;
	EXPECT_FALSE(world.cycle(&same).request);
	while (world.stamp < answered + kConfig.replan_backoff - 1e-9) EXPECT_FALSE(world.cycle().request) << "at " << world.stamp;
	const LocalStep<ReplanRequest> step = world.cycle();
	ASSERT_TRUE(step.request);
	EXPECT_EQ(step.request->reason, ReplanReason::Blocked);
}

TEST(TrajectoryLocalPlanner, PlansFromTheRealDroneWhenItFallsBehind) {
	World world;
	const GlobalPath path = route({xyz(0, 0, 2), xyz(40, 0, 2)});
	world.cycle(&path);
	world.speed_up_to(3.0);
	ASSERT_FALSE(world.report->latest()->resynced);

	/// the drone stops following (the autopilot lags): the plan would run away from it
	const Eigen::Vector3d stuck = world.position;
	LocalStep<ReplanRequest> step;
	bool resynced = false;
	for (int i = 0; i < 10 && !resynced; ++i) {
		world.poses.publish(pose_at(stuck, world.stamp));
		ASSERT_TRUE(world.planner.prepare());
		step = world.planner.plan(nullptr);
		world.stamp += kCycle;
		resynced = world.report->latest()->resynced;
	}
	ASSERT_TRUE(resynced);
	/// planned from where the drone is and how fast it really moves
	EXPECT_LT((step.trajectory->points.front().position - stuck).norm(), kConfig.path_step);
	EXPECT_LT(step.trajectory->points.front().velocity.norm(), kConfig.resync_speed);
	expect_acceleration_limited(*step.trajectory);
}

TEST(TrajectoryLocalPlanner, TurnsTowardsThePathBeforeSpeedingUp) {
	World world;
	/// the camera looks along +x, the path goes along -x
	const GlobalPath behind = route({xyz(0, 0, 2), xyz(-30, 0, 2)});
	const Trajectory trajectory = *world.cycle(&behind).trajectory;
	double turned_at = std::numeric_limits<double>::infinity();
	for (const Setpoint& point : trajectory.points) {
		const double t = point.stamp - trajectory.stamp;
		if (std::abs(wrap_angle(std::numbers::pi - point.yaw)) <= kConfig.camera_half_fov) turned_at = std::min(turned_at, t);
		/// flying blind: slowly, until the camera looks along the path
		if (t < turned_at) {
			EXPECT_LE(point.velocity.norm(), kConfig.blind_speed + 1e-9) << "at " << t;
		}
	}
	EXPECT_LT(turned_at, kConfig.horizon);
	EXPECT_GT(trajectory.points.back().velocity.norm(), kConfig.blind_speed);
}

TEST(TrajectoryLocalPlanner, SlowsDownNearObstacles) {
	/// a wall along the path, 1 m to its side (x from 5 m to 15 m): outside the margin, but close
	World world(voxels({10, 2, 0}, {29, 2, 9}));
	const GlobalPath path = route({xyz(0, 0, 2), xyz(25, 0, 2)});
	world.cycle(&path);
	std::vector<Trajectory> flown = world.fly();
	ASSERT_EQ(world.planner.state(), State::Arrived);
	double fastest_before = 0.0;
	for (const Trajectory& trajectory : flown) {
		const Setpoint& now = trajectory.points.front();
		if (now.position.x() > 5.5 && now.position.x() < 14.5) {
			EXPECT_LE(now.velocity.norm(), kConfig.caution_speed + 1e-9) << "at x " << now.position.x();
		}
		if (now.position.x() < 3.0) fastest_before = std::max(fastest_before, now.velocity.norm());
	}
	EXPECT_GT(fastest_before, kConfig.caution_speed);
}

// --- With the threads ---

TEST(TrajectoryLocalPlannerThreads, AsksForARouteAndFollowsIt) {
	Fixture f(xyz(10, 0, 2));
	f.poses.publish(pose_at(xyz(0, 0, 2), 1.0));
	ASSERT_TRUE(wait_for([&] { return f.moving(); }, 5s));
	EXPECT_GE(f.local_thread.sent_request_count(), 1u);
	EXPECT_GE(f.global_thread.plan_call_count(), 1u);
	const auto trajectory = f.trajectories_latest->latest();
	EXPECT_DOUBLE_EQ(trajectory->points.front().stamp, 1.0);
	EXPECT_GT(trajectory->points.back().position.x(), 1.0);
}

TEST(TrajectoryLocalPlannerThreads, ReplansAroundAnObstacleThatAppears) {
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

