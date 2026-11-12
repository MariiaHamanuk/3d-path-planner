#include "mapping/log_odds_grid_map.hpp"
#include "planning/benchmark_scenes.hpp"
#include "planning/search_planners.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <map>
#include <tuple>

using namespace path3d;

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
	SearchSpaceConfig config;
	config.margin = 0.0;
	const OccupancySpace<LogOddsGridMap> space(map, Eigen::AlignedBox3d(Eigen::Vector3d::Constant(-100), Eigen::Vector3d::Constant(100)), config);
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
	EXPECT_TRUE(planner.last_stats().found);
	EXPECT_GT(planner.last_stats().length, (kGoal - kStart).norm());
}

TEST(AStarGlobalPlanner, FailsForAGoalInTheWall) {
	World world;
	AStarGlobalPlanner<LogOddsGridMap> planner(world.map, world.uav, Eigen::Vector3d(5.2, 0, 2));
	ASSERT_TRUE(planner.prepare());
	const GlobalPath path = planner.plan(request_from(kStart));
	EXPECT_EQ(path.status, PlanStatus::Failed);
	EXPECT_TRUE(path.waypoints.empty());
}

TEST(AStarGlobalPlanner, FailsWhenOnlyKnownFreeSpaceIsAllowed) {
	World world;
	SearchPlannerConfig config;
	config.space.unknown_is_free = false;
	AStarGlobalPlanner<LogOddsGridMap> planner(world.map, world.uav, kGoal, config);
	ASSERT_TRUE(planner.prepare());
	EXPECT_EQ(planner.plan(request_from(kStart)).status, PlanStatus::Failed);
}

TEST(AStarGlobalPlanner, OctileFindsAsShortAPathAsDijkstraWithFewerExpansions) {
	World world;
	AStarGlobalPlanner<LogOddsGridMap> a_star_planner(world.map, world.uav, kGoal);
	ASSERT_TRUE(a_star_planner.prepare());
	a_star_planner.plan(request_from(kStart));

	World other;
	AStarGlobalPlanner<LogOddsGridMap, ZeroHeuristic> dijkstra_planner(other.map, other.uav, kGoal);
	ASSERT_TRUE(dijkstra_planner.prepare());
	dijkstra_planner.plan(request_from(kStart));

	ASSERT_TRUE(a_star_planner.last_stats().found);
	ASSERT_TRUE(dijkstra_planner.last_stats().found);
	EXPECT_NEAR(a_star_planner.last_stats().length, dijkstra_planner.last_stats().length, kResolution);
	EXPECT_LT(a_star_planner.last_stats().explored, dijkstra_planner.last_stats().explored);
}

TEST(LazyThetaStarGlobalPlanner, PlansAShorterPathThanAStar) {
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
	EXPECT_LT(theta.last_stats().length, grid.last_stats().length);
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

TEST(RrtStarGlobalPlanner, PlansAShorterPathThanRrt) {
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

	World other;
	RrtGlobalPlanner<LogOddsGridMap> simple(other.map, other.uav, kGoal);
	ASSERT_TRUE(simple.prepare());
	simple.plan(request_from(kStart));
	EXPECT_LT(star.last_stats().length, simple.last_stats().length);
}

TEST(AStarGlobalPlanner, GoesAroundTheWallWhenClimbingIsExpensive) {
	World over_world;
	AStarGlobalPlanner<LogOddsGridMap> over(over_world.map, over_world.uav, kGoal);
	ASSERT_TRUE(over.prepare());
	over.plan(request_from(kStart));

	World around_world;
	SearchPlannerConfig config;
	config.space.climb_cost = 5.0;
	AStarGlobalPlanner<LogOddsGridMap> around(around_world.map, around_world.uav, kGoal, config);
	ASSERT_TRUE(around.prepare());
	const GlobalPath path = around.plan(request_from(kStart));
	ASSERT_EQ(path.status, PlanStatus::Ok);
	expect_clear(around_world.map.map(), path);

	EXPECT_GT(over.last_stats().climb, 4.0);    // over the 6 m wall from 2 m
	EXPECT_LT(around.last_stats().climb, 1.0);  // stays near 2 m
	EXPECT_GT(around.last_stats().length, over.last_stats().length);
}

TEST(GridGraph, ChargesForUnknownVoxelsAndForClimbing) {
	LogOddsGridMap map(1.0);
	LogOddsUpdate update;
	update.keyframe = true;
	update.resolution = 1.0;
	update.cells = {{.x = 1, .y = 0, .z = 0, .log_odds = -2.0f}};  // the only voxel seen free
	map.apply(update);
	SearchSpaceConfig config;
	config.margin = 0.0;
	config.unknown_cost = 3.0;
	config.climb_cost = 0.5;
	const GridGraph<LogOddsGridMap> graph(map, Eigen::AlignedBox3d(Eigen::Vector3d::Constant(-5), Eigen::Vector3d::Constant(5)), config);

	std::map<std::tuple<int, int, int>, double> costs;
	graph.for_each_neighbor(Eigen::Vector3i::Zero(), [&](const Eigen::Vector3i& n, double cost) { costs[{n.x(), n.y(), n.z()}] = cost; });
	ASSERT_EQ(costs.size(), 26u);
	EXPECT_DOUBLE_EQ((costs[{1, 0, 0}]), 1.0);                     // seen free: just the length
	EXPECT_DOUBLE_EQ((costs[{-1, 0, 0}]), 3.0);                    // unknown: 3 x 1
	EXPECT_DOUBLE_EQ((costs[{0, 0, 1}]), 3.0 + 0.5);               // unknown and up
	EXPECT_DOUBLE_EQ((costs[{0, 0, -1}]), 3.0);                    // down is not charged
	EXPECT_DOUBLE_EQ((costs[{1, 1, 1}]), 3.0 * std::sqrt(3.0) + 0.5);
}

TEST(BenchmarkScenes, EverySceneHasAPath) {
	for (const Scene& scene : benchmark_scenes()) {
		Topic<LogOddsGridMap::Update> updates;
		Topic<StampedPose> poses;
		MapClient<LogOddsGridMap> map(updates);
		UavClient uav(poses);
		updates.publish(scene_map(scene, kResolution));
		SearchPlannerConfig config;
		config.max_altitude = scene.max_altitude;
		AStarGlobalPlanner<LogOddsGridMap> planner(map, uav, scene.goal, config);
		ASSERT_TRUE(planner.prepare());
		const GlobalPath path = planner.plan(request_from(scene.start));
		EXPECT_EQ(path.status, PlanStatus::Ok) << scene.name;
		expect_clear(map.map(), path);
	}
}

TEST(SearchBounds, KeepsStartAndGoalInsideTheAltitudeLimits) {
	const auto box = search_bounds(Eigen::Vector3d(0, 0, 0.5), Eigen::Vector3d(10, 0, 40), 5.0, 1.0, 30.0);
	EXPECT_DOUBLE_EQ(box.min().z(), 0.5);
	EXPECT_DOUBLE_EQ(box.max().z(), 40.0);
	EXPECT_DOUBLE_EQ(box.min().x(), -5.0);
	EXPECT_DOUBLE_EQ(box.max().x(), 15.0);
}
