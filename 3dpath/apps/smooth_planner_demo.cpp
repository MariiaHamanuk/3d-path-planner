/** @file smooth_planner_demo.cpp
 * @brief Flies the drone to a goal with a global planner and SmoothLocalPlanner. The drone must
 * already be flying.
 *
 * Usage: ros2 run path3d smooth_planner_demo <x> <y> <z> [speed] [world] [algo]
 *   x y z   goal in the world frame (ENU), m
 *   speed   cruise speed, m/s (default 5)
 *   world   default | city (must be the world the simulation runs)
 *   algo    global planner: lazy_theta_star (default) | astar | dijkstra | rrt | rrt_star | rrt_connect
 *
 * Before you run it: arm the drone, take off and put it in GUIDED mode. The drone stays at the goal
 * when it arrives. Press Ctrl-C to stop the demo.
 */
#include "io/mavsdk_vehicle.hpp"
#include "io/sim_reader.hpp"
#include "mapping/log_odds_mapper.hpp"
#include "pipeline/clients.hpp"
#include "pipeline/threads.hpp"
#include "planning/search_planners.hpp"
#include "planning/smooth_local_planner.hpp"
#include "visualization/log_odds_map_visualizer.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace {

std::atomic<bool> g_interrupted{false};
void on_signal(int) { g_interrupted = true; }

using Map = path3d::LogOddsGridMap;
using AnyGlobalPlanner = path3d::GlobalPlanner<Map, path3d::ReplanRequest, path3d::GlobalPath>;

std::unique_ptr<AnyGlobalPlanner> make_global_planner(const std::string& algo, path3d::MapClient<Map>& map, path3d::UavClient& uav,
                                                      const Eigen::Vector3d& goal, const path3d::SearchPlannerConfig& config) {
	using namespace path3d;
	if (algo == "lazy_theta_star") return std::make_unique<LazyThetaStarGlobalPlanner<Map>>(map, uav, goal, config);
	if (algo == "astar") return std::make_unique<AStarGlobalPlanner<Map>>(map, uav, goal, config);
	if (algo == "dijkstra") return std::make_unique<AStarGlobalPlanner<Map, ZeroHeuristic>>(map, uav, goal, config);
	if (algo == "rrt") return std::make_unique<RrtGlobalPlanner<Map>>(map, uav, goal, config);
	if (algo == "rrt_star") return std::make_unique<RrtStarGlobalPlanner<Map>>(map, uav, goal, config);
	if (algo == "rrt_connect") return std::make_unique<RrtConnectGlobalPlanner<Map>>(map, uav, goal, config);
	return nullptr;
}

}

int main(int argc, char** argv) {
	if (argc < 4) {
		std::cerr << "usage: smooth_planner_demo <x> <y> <z> [speed] [world] [algo]\n";
		return 2;
	}
	const Eigen::Vector3d goal(std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3]));
	const double speed = argc > 4 ? std::stod(argv[4]) : 5.0;
	const std::string algo = argc > 6 ? argv[6] : "lazy_theta_star";

	using namespace path3d;

	PipelineConfig config;
	if (argc > 5) config.sim_reader.world_frame = argv[5];
	config.mapping.keyframe_period = 1000ms;
	config.mapping.publish_period = 0.1;
	config.map_visualizer.rate_hz = 5.0;
	config.log_odds_mapper.resolution = 0.5;
	config.log_odds_mapper.max_range = 30.0f;

	SensorTopics sensors;
	Topic<Map::Update> map_updates;
	LogOddsMapper mapper(config.log_odds_mapper);
	MappingThread<Map> mapping(sensors, mapper, map_updates, config.mapping);
	mapping.start();
	UavClient uav(sensors.pose);
	Topic<VehicleState> vehicle_state;
	const auto latest_state = vehicle_state.subscribe_latest();
	SimReader reader(config.sim_reader, sensors);
	LogOddsMapVisualizerConfig visualizer_config = config.map_visualizer;
	visualizer_config.frame_id = config.sim_reader.world_frame;
	LogOddsMapVisualizer visualizer(map_updates, visualizer_config);

	Topic<Trajectory> trajectories;
	MavsdkVehicle vehicle(config.mavsdk, sensors.pose, vehicle_state);
	CommanderThread commander(vehicle, trajectories, vehicle_state, sensors.pose, config.commander);
	commander.start();
	std::signal(SIGINT, on_signal);

	while (!g_interrupted) {
		const auto state = latest_state->latest();
		if (state && uav.pose() && state->connected && state->armed && state->frames_ready && state->mode == FlightMode::External) break;
		std::cout << "waiting for a flying drone in GUIDED mode (armed: " << (state && state->armed) << ", mode: " << (state ? to_string(state->mode) : "unknown") << ")\n";
		std::this_thread::sleep_for(2s);
	}
	if (g_interrupted) return 0;
	std::cout << "flying from " << uav.position()->transpose() << " to " << goal.transpose() << " at " << speed << " m/s\n";

	MapClient<Map> global_map(map_updates, config.global_planning.map_client);
	MapClient<Map> local_map(map_updates, config.local_planning.map_client);
	UavClient global_uav(sensors.pose);
	UavClient local_uav(sensors.pose);

	SearchPlannerConfig global_config;
	global_config.space.margin = 1.0;
	const std::unique_ptr<AnyGlobalPlanner> global_planner = make_global_planner(algo, global_map, global_uav, goal, global_config);
	if (!global_planner) {
		std::cerr << "unknown algo '" << algo << "': lazy_theta_star | astar | dijkstra | rrt | rrt_star | rrt_connect\n";
		return 2;
	}
	std::cout << "global planner: " << algo << "\n";

	SmoothLocalPlannerConfig local_config;
	local_config.max_speed = speed;
	local_config.space.margin = 0.5;
	SmoothLocalPlanner<Map> local_planner(local_map, local_uav, local_config);

	Pipe<GlobalPath, ReplanRequest> pipe;
	GlobalPlanningThread<Map, GlobalPath, ReplanRequest> global_thread(*global_planner, pipe, std::nullopt, config.global_planning);
	LocalPlanningThread<Map, GlobalPath, ReplanRequest> local_thread(local_planner, pipe, trajectories, config.local_planning);
	const auto latest_request = pipe.up().subscribe_latest();
	const auto latest_route = pipe.down().subscribe_latest();
	global_thread.start();
	local_thread.start();

	const auto reason_name = [](ReplanReason reason) {
		switch (reason) {
			case ReplanReason::NoPath: return "no path";
			case ReplanReason::Blocked: return "blocked";
			case ReplanReason::Deviated: return "deviated";
		}
		return "?";
	};
	while (!g_interrupted && (*uav.position() - goal).norm() > 0.5) {
		std::cout << "distance to the goal: " << (*uav.position() - goal).norm() << " m | replans " << local_thread.sent_request_count()
		          << " (timed out " << local_thread.request_timeout_count() << ", stale answers " << local_thread.stale_route_count() << ") | global plans " << global_thread.plan_call_count()
		          << " | holds " << commander.hold_count() << "\n";
		if (const auto request = latest_request->latest()) {
			std::cout << "  last request: " << reason_name(request->reason) << " from " << request->position.transpose() << "\n";
		}
		if (const auto route = latest_route->latest()) {
			std::cout << "  last global plan: " << (route->status == PlanStatus::Ok ? "ok, " : "FAILED, ") << route->waypoints.size() << " waypoints\n";
			if (route->status != PlanStatus::Ok) std::cout << "  the goal may be inside or too close to an obstacle: move DST into open space, z above the ground\n";
		}
		std::this_thread::sleep_for(1s);
	}
	std::cout << (g_interrupted ? "stopped\n" : "arrived\n");
}
