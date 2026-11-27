#include "mapping/log_odds_grid_map.hpp"
#include "planning/global/search_planners.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <set>
#include <tuple>

using namespace path3d;

static_assert(SearchGraph<GridMapSearchSpace<LogOddsGridMap>>);
static_assert(std::same_as<GridMapSearchSpace<LogOddsGridMap>::NodeHash, LogOddsGridMap::IndexHash>);  // the map's own hash
static_assert(std::same_as<IndexHashOf<int>::type, VoxelHash>);                                         // a map without one
static_assert(LineCost<GridLineOfSightCost<LogOddsGridMap, LengthCost>, Eigen::Vector3i>);

namespace {

constexpr double kResolution = 0.5;

/// A wall across the way from (0, 0, 2) to (10, 0, 2): voxels with x index 10 (x in [5, 5.5)),
/// y in [-5, 5) and z in [0, 6). Everything else is unknown.
LogOddsUpdate wall_map() {
	LogOddsUpdate update;
	update.keyframe = true;
	update.to_version = 1;
	update.resolution = kResolution;
	for (int y = -10; y < 10; ++y) {
		for (int z = 0; z < 12; ++z) update.cells.push_back({.x = 10, .y = y, .z = z, .log_odds = 2.0f});
	}
	return update;
}

/// The map, its copy for one planner, and the drone pose.
struct World {
	World() : map(updates), uav(poses) { updates.publish(wall_map()); }

	Topic<LogOddsGridMap::Update> updates;
	Topic<StampedPose> poses;
	MapClient<LogOddsGridMap> map;
	UavClient uav;
};

ReplanRequest request_from(const Eigen::Vector3d& position, std::uint64_t id = 1) {
	ReplanRequest request;
	request.id = id;
	request.position = position;
	return request;
}

/// Every segment of the path keeps away from occupied voxels.
void expect_clear(const LogOddsGridMap& map, const GlobalPath& path) {
	const ContinuousSearchSpace<LogOddsGridMap> space(map, Eigen::AlignedBox3d(Eigen::Vector3d::Constant(-100), Eigen::Vector3d::Constant(100)), 0.0, true);
	for (std::size_t i = 1; i < path.waypoints.size(); ++i) {
		EXPECT_TRUE(space.segment_free(path.waypoints[i - 1], path.waypoints[i])) << "segment " << i;
	}
}

const Eigen::Vector3d kStart(0, 0, 2);
const Eigen::Vector3d kGoal(10, 0, 2);

}

TEST(AStarGlobalPlanner, PlansAroundTheWall) {
	World world;
	AStarGlobalPlanner<LogOddsGridMap> planner(world.map, world.uav, kGoal);
	ASSERT_TRUE(planner.prepare());
	const GlobalPath path = planner.plan(request_from(kStart));
	ASSERT_EQ(path.status, PlanStatus::Ok);
	ASSERT_GE(path.waypoints.size(), 3u);
	EXPECT_EQ(path.waypoints.front(), kStart);
	EXPECT_EQ(path.waypoints.back(), kGoal);
	expect_clear(world.map.map(), path);
}

TEST(AStarGlobalPlanner, FailsForAGoalInTheWall) {
	World world;
	AStarGlobalPlanner<LogOddsGridMap> planner(world.map, world.uav, Eigen::Vector3d(5.2, 0, 2));
	ASSERT_TRUE(planner.prepare());
	const GlobalPath path = planner.plan(request_from(kStart));
	EXPECT_EQ(path.status, PlanStatus::Failed);
	EXPECT_TRUE(path.waypoints.empty());
}

TEST(ReachableStart, NeverMovesTheStartThroughAWall) {
	World world;
	ASSERT_TRUE(world.map.sync());
	/// only the far side of the wall (x >= 5.5) and the space 2 m behind the drone count as free
	const auto free = [](const Eigen::Vector3d& p) { return p.x() > 5.5 || p.x() < 3.0; };
	const Eigen::Vector3d drone(4.9, 0, 2);
	/// the nearest free point is 1 m away through the wall; the reachable one is 2 m back
	const auto nearest = nearest_free_point(drone, 2.0, kResolution, free);
	ASSERT_TRUE(nearest);
	EXPECT_GT(nearest->x(), 5.5);
	const auto start = reachable_start(world.map.map(), drone, 2.0, kResolution, true, free);
	ASSERT_TRUE(start);
	EXPECT_LT(start->x(), 3.0);
	EXPECT_EQ(reachable_start(world.map.map(), Eigen::Vector3d(2, 0, 2), 2.0, kResolution, true, free), Eigen::Vector3d(2, 0, 2));
}

TEST(AStarGlobalPlanner, FailsWhenOnlyKnownFreeSpaceIsAllowed) {
	World world;
	SearchPlannerConfig config;
	config.unknown_is_free = false;
	AStarGlobalPlanner<LogOddsGridMap> planner(world.map, world.uav, kGoal, config);
	ASSERT_TRUE(planner.prepare());
	EXPECT_EQ(planner.plan(request_from(kStart)).status, PlanStatus::Failed);
}

TEST(LazyThetaStarGlobalPlanner, PlansAPathWithNoMoreWaypointsThanAStar) {
	World theta_world;
	LazyThetaStarGlobalPlanner<LogOddsGridMap> theta(theta_world.map, theta_world.uav, kGoal);
	ASSERT_TRUE(theta.prepare());
	const GlobalPath path = theta.plan(request_from(kStart));
	ASSERT_EQ(path.status, PlanStatus::Ok);
	EXPECT_EQ(path.waypoints.front(), kStart);
	EXPECT_EQ(path.waypoints.back(), kGoal);
	expect_clear(theta_world.map.map(), path);

	World grid_world;
	AStarGlobalPlanner<LogOddsGridMap> grid(grid_world.map, grid_world.uav, kGoal);
	ASSERT_TRUE(grid.prepare());
	const GlobalPath grid_path = grid.plan(request_from(kStart));
	EXPECT_LE(path.waypoints.size(), grid_path.waypoints.size());
}

TEST(RrtConnectGlobalPlanner, PlansAroundTheWall) {
	World world;
	RrtConnectGlobalPlanner<LogOddsGridMap> planner(world.map, world.uav, kGoal);
	ASSERT_TRUE(planner.prepare());
	const GlobalPath path = planner.plan(request_from(kStart));
	ASSERT_EQ(path.status, PlanStatus::Ok);
	EXPECT_EQ(path.waypoints.front(), kStart);
	EXPECT_EQ(path.waypoints.back(), kGoal);
	expect_clear(world.map.map(), path);
}

TEST(RrtGlobalPlanner, PlansAroundTheWall) {
	World world;
	RrtGlobalPlanner<LogOddsGridMap> planner(world.map, world.uav, kGoal);
	ASSERT_TRUE(planner.prepare());
	const GlobalPath path = planner.plan(request_from(kStart));
	ASSERT_EQ(path.status, PlanStatus::Ok);
	EXPECT_EQ(path.waypoints.front(), kStart);
	EXPECT_EQ(path.waypoints.back(), kGoal);
	expect_clear(world.map.map(), path);
}

TEST(RrtStarGlobalPlanner, PlansAroundTheWall) {
	World world;
	RrtStarParams params;
	params.max_iterations = 3000;
	RrtStarGlobalPlanner<LogOddsGridMap> star(world.map, world.uav, kGoal, {}, params);
	ASSERT_TRUE(star.prepare());
	const GlobalPath path = star.plan(request_from(kStart));
	ASSERT_EQ(path.status, PlanStatus::Ok);
	EXPECT_EQ(path.waypoints.front(), kStart);
	EXPECT_EQ(path.waypoints.back(), kGoal);
	expect_clear(world.map.map(), path);
}

TEST(GridMapSearchSpace, ChargesForUnknownVoxelsAndForClimbing) {
	LogOddsGridMap map(1.0);
	LogOddsUpdate update;
	update.keyframe = true;
	update.resolution = 1.0;
	update.cells = {{.x = 1, .y = 0, .z = 0, .log_odds = -2.0f}};  // the only voxel seen free
	map.apply(update);
	const GridMapSearchSpace<LogOddsGridMap> graph(map, Eigen::AlignedBox3d(Eigen::Vector3d::Constant(-5), Eigen::Vector3d::Constant(5)), 0.0, true,
	                                               LengthCost{.unknown_cost = 3.0, .climb_cost = 0.5});

	std::map<std::tuple<int, int, int>, double> costs;
	graph.for_each_neighbor(Eigen::Vector3i::Zero(), [&](const Eigen::Vector3i& n, double cost) { costs[{n.x(), n.y(), n.z()}] = cost; });
	ASSERT_EQ(costs.size(), 26u);
	EXPECT_DOUBLE_EQ((costs[{1, 0, 0}]), 1.0);                     // seen free: just the length
	EXPECT_DOUBLE_EQ((costs[{-1, 0, 0}]), 3.0);                    // unknown: 3 x 1
	EXPECT_DOUBLE_EQ((costs[{0, 0, 1}]), 3.0 + 0.5);               // unknown and up
	EXPECT_DOUBLE_EQ((costs[{0, 0, -1}]), 3.0);                    // down is not charged
	EXPECT_DOUBLE_EQ((costs[{1, 1, 1}]), 3.0 * std::sqrt(3.0) + 0.5);
}

/// A cost that only the test knows: a move costs its length, and ten times that east of x = 20.
struct EastIsExpensive {
	double operator()(const Eigen::Vector3i& from, const Eigen::Vector3i& to, Occupancy) const {
		const double length = (to - from).cast<double>().norm();
		return to.x() > 20 ? 10.0 * length : length;
	}
};

TEST(GridMapSearchSpace, TakesTheCostOfAMoveFromTheCostFunction) {
	const LogOddsGridMap map(1.0);
	const Eigen::AlignedBox3d bounds(Eigen::Vector3d::Constant(-50), Eigen::Vector3d::Constant(50));
	const GridMapSearchSpace<LogOddsGridMap, EastIsExpensive> space(map, bounds, 0.0, true);
	const GridLineOfSightCost line(space);
	std::map<std::tuple<int, int, int>, double> costs;
	space.for_each_neighbor(Eigen::Vector3i(20, 0, 0), [&](const Eigen::Vector3i& n, double cost) { costs[{n.x() - 20, n.y(), n.z()}] = cost; });
	EXPECT_DOUBLE_EQ((costs[{-1, 0, 0}]), 1.0);
	EXPECT_DOUBLE_EQ((costs[{1, 0, 0}]), 10.0);
	EXPECT_DOUBLE_EQ((costs[{1, 1, 0}]), 10.0 * std::sqrt(2.0));
	/// a line is the cost function averaged over its voxels; with one value for all, that value
	EXPECT_DOUBLE_EQ(*line({0, 0, 0}, {30, 0, 0}), 10.0 * 30.0);
}

TEST(AStarGlobalPlanner, GoesAroundTheWallWhenClimbingIsExpensive) {
	const auto highest = [](const GlobalPath& path) {
		double z = path.waypoints.front().z();
		for (const Eigen::Vector3d& point : path.waypoints) z = std::max(z, point.z());
		return z;
	};
	World over_world;
	AStarGlobalPlanner<LogOddsGridMap> over(over_world.map, over_world.uav, kGoal);
	ASSERT_TRUE(over.prepare());
	const GlobalPath over_path = over.plan(request_from(kStart));
	ASSERT_EQ(over_path.status, PlanStatus::Ok);

	World around_world;
	AStarGlobalPlanner<LogOddsGridMap> around(around_world.map, around_world.uav, kGoal, {}, {}, LengthCost{.climb_cost = 5.0});
	ASSERT_TRUE(around.prepare());
	const GlobalPath around_path = around.plan(request_from(kStart));
	ASSERT_EQ(around_path.status, PlanStatus::Ok);
	expect_clear(around_world.map.map(), around_path);

	EXPECT_GT(highest(over_path), 6.0);    // over the 6 m wall from 2 m
	EXPECT_LT(highest(around_path), 3.0);  // stays near 2 m
}

TEST(Passability, PlannerDecidesWhetherUnknownMayBeFlownThrough) {
	LogOddsGridMap map(1.0);
	LogOddsUpdate update;
	update.keyframe = true;
	update.resolution = 1.0;
	update.cells = {{.x = 5, .y = 0, .z = 0, .log_odds = 2.0f}, {.x = 0, .y = 0, .z = 0, .log_odds = -2.0f}};
	map.apply(update);
	EXPECT_TRUE(is_clear_at(map, {0, 0, 0}, 0, false));
	EXPECT_FALSE(is_clear_at(map, {0, 0, 0}, 1, false));  // the neighbors were never seen
	EXPECT_TRUE(is_clear_at(map, {0, 0, 0}, 1, true));
	EXPECT_FALSE(is_clear_at(map, {4, 0, 0}, 1, true));   // the occupied voxel is in the ball
	EXPECT_TRUE(is_clear_at(map, {3, 0, 0}, 1, true));
	EXPECT_FALSE(is_clear(map, Eigen::Vector3d(4.5, 0.5, 0.5), 0.5, true));
	EXPECT_TRUE(is_passable(Occupancy::Unknown, true));
	EXPECT_FALSE(is_passable(Occupancy::Unknown, false));
	EXPECT_FALSE(is_passable(Occupancy::Occupied, true));
}

TEST(GridMapSearchSpace, HasTwentySixDistinctNeighbors) {
	std::set<std::tuple<int, int, int>> offsets;
	for (const GridStep& step : grid_steps()) {
		offsets.insert({step.offset.x(), step.offset.y(), step.offset.z()});
		EXPECT_DOUBLE_EQ(step.length, step.offset.cast<double>().norm());
	}
	EXPECT_EQ(offsets.size(), 26u);
	EXPECT_EQ(offsets.count({0, 0, 0}), 0u);
}

TEST(SearchBounds, KeepsStartAndGoalInsideTheAltitudeLimits) {
	const auto box = search_bounds(Eigen::Vector3d(0, 0, 0.5), Eigen::Vector3d(10, 0, 40), 5.0, 1.0, 30.0);
	EXPECT_DOUBLE_EQ(box.min().z(), 0.5);
	EXPECT_DOUBLE_EQ(box.max().z(), 40.0);
	EXPECT_DOUBLE_EQ(box.min().x(), -5.0);
	EXPECT_DOUBLE_EQ(box.max().x(), 15.0);
}
