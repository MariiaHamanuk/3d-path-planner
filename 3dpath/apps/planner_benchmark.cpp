/** @file planner_benchmark.cpp
 * @brief Compares the global planners on the benchmark scenes, without the simulation.
 *
 * Usage: ros2 run path3d planner_benchmark [csv_path] [resolution_m]
 *   csv_path      where to write the results (default benchmark.csv)
 *   resolution_m  voxel size of the scene maps (default 0.5)
 *
 * For every scene (see benchmark_scenes()) and every planner it plans once from the start to the
 * goal of the scene; RRT, RRT-Connect and RRT* plan several times with different seeds and report averages.
 * It prints a table per scene and writes all rows to the CSV file. Run it with
 * scripts/run_benchmark.sh.
 */
#include "mapping/log_odds_grid_map.hpp"
#include "pipeline/clients.hpp"
#include "planning/benchmark_scenes.hpp"
#include "planning/search_planners.hpp"

#include <chrono>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

using namespace path3d;
using Map = LogOddsGridMap;

/// Plans of the sampling planners, averaged over different seeds.
constexpr int kSamplingRuns = 5;

/// One planner on one scene. Averages are over the runs that found a path.
struct Row {
	std::string scene;
	std::string planner;
	int runs = 0;
	int found = 0;
	double length = 0.0;     ///< m
	double climb = 0.0;      ///< m
	double time_ms = 0.0;    ///< over all runs
	double explored = 0.0;   ///< A*: expanded voxels; RRT, RRT*: tree nodes
	double waypoints = 0.0;
};

/**
 * @brief Builds a planner of type P on its own copy of the scene map, plans `runs` times from the
 * start of the scene (request ids 1, 2, ..., so a sampling planner uses a new seed each time)
 * and averages its PlanStats.
 */
template <class P, class... Args>
Row run(const Scene& scene, const LogOddsUpdate& map_update, const std::string& name, int runs, Args&&... args) {
	Topic<Map::Update> updates;
	Topic<StampedPose> poses;
	MapClient<Map> map(updates);
	UavClient uav(poses);
	updates.publish(map_update);
	P planner(map, uav, scene.goal, std::forward<Args>(args)...);
	planner.prepare();

	Row row{.scene = scene.name, .planner = name, .runs = runs};
	for (int i = 0; i < runs; ++i) {
		ReplanRequest request;
		request.id = static_cast<std::uint64_t>(i + 1);
		request.position = scene.start;
		const auto begin = std::chrono::steady_clock::now();
		const GlobalPath path = planner.plan(request);
		row.time_ms += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
		const PlanStats& stats = planner.last_stats();
		row.explored += static_cast<double>(stats.explored);
		if (!stats.found) continue;
		++row.found;
		row.length += stats.length;
		row.climb += stats.climb;
		row.waypoints += static_cast<double>(path.waypoints.size());
	}
	row.time_ms /= runs;
	row.explored /= runs;
	if (row.found > 0) {
		row.length /= row.found;
		row.climb /= row.found;
		row.waypoints /= row.found;
	}
	return row;
}

std::vector<Row> run_scene(const Scene& scene, double resolution) {
	const LogOddsUpdate map_update = scene_map(scene, resolution);
	SearchPlannerConfig config;
	config.max_altitude = scene.max_altitude;
	SearchPlannerConfig climbing = config;
	climbing.space.climb_cost = 2.0;

	return {
	    run<AStarGlobalPlanner<Map>>(scene, map_update, "A* Octile", 1, config),
	    run<AStarGlobalPlanner<Map, EuclideanHeuristic>>(scene, map_update, "A* Euclidean", 1, config),
	    run<AStarGlobalPlanner<Map, ManhattanHeuristic>>(scene, map_update, "A* Manhattan", 1, config),
	    run<AStarGlobalPlanner<Map, ZeroHeuristic>>(scene, map_update, "Dijkstra", 1, config),
	    run<AStarGlobalPlanner<Map>>(scene, map_update, "A* Octile climb x2", 1, climbing),
	    run<LazyThetaStarGlobalPlanner<Map>>(scene, map_update, "Lazy Theta*", 1, config),
	    run<RrtGlobalPlanner<Map>>(scene, map_update, "RRT", kSamplingRuns, config),
	    run<RrtConnectGlobalPlanner<Map>>(scene, map_update, "RRT-Connect", kSamplingRuns, config),
	    run<RrtStarGlobalPlanner<Map>>(scene, map_update, "RRT*", kSamplingRuns, config),
	};
}

void print(const Scene& scene, const std::vector<Row>& rows) {
	std::printf("\n%s: %s\n", scene.name.c_str(), scene.description.c_str());
	std::printf("  %-22s %7s %10s %9s %11s %12s %10s\n", "planner", "found", "length m", "climb m", "time ms", "explored", "waypoints");
	for (const Row& row : rows) {
		const std::string found = std::to_string(row.found) + "/" + std::to_string(row.runs);
		if (row.found == 0) {
			std::printf("  %-22s %7s %10s %9s %11.1f %12.0f %10s\n", row.planner.c_str(), found.c_str(), "-", "-", row.time_ms, row.explored, "-");
		} else {
			std::printf("  %-22s %7s %10.2f %9.2f %11.1f %12.0f %10.1f\n", row.planner.c_str(), found.c_str(), row.length, row.climb, row.time_ms, row.explored, row.waypoints);
		}
	}
}

}

int main(int argc, char** argv) {
	const std::string csv_path = argc > 1 ? argv[1] : "benchmark.csv";
	const double resolution = argc > 2 ? std::stod(argv[2]) : 0.5;

	std::ofstream csv(csv_path);
	if (!csv) {
		std::cerr << "cannot write " << csv_path << "\n";
		return 1;
	}
	csv << "scene,planner,runs,found,length_m,climb_m,time_ms,explored,waypoints\n";

	std::printf("Global planners on the benchmark scenes, voxel %.2f m. RRT, RRT-Connect, RRT*: %d seeds each, averages over the runs that found a path.\n", resolution, kSamplingRuns);
	for (const Scene& scene : benchmark_scenes()) {
		const std::vector<Row> rows = run_scene(scene, resolution);
		print(scene, rows);
		for (const Row& row : rows) {
			csv << row.scene << ',' << row.planner << ',' << row.runs << ',' << row.found << ',' << row.length << ','
			    << row.climb << ',' << row.time_ms << ',' << row.explored << ',' << row.waypoints << '\n';
		}
	}
	std::printf("\nResults written to %s\n", csv_path.c_str());
}
