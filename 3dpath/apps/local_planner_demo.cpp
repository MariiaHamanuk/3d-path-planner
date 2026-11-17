/** @file local_planner_demo.cpp
 * @brief Flies the drone to a goal around obstacles: a search-based global planner plans the path,
 * the PathFollowingLocalPlanner follows it. The drone must already be flying.
 *
 * Usage: ros2 run path3d local_planner_demo <dx> <dy> <dz> [world] [planner]
 *   dx dy dz  the goal relative to the drone, m, in the world frame (x east, y north, z up)
 *   world     default | city (must be the world the simulation runs)
 *   planner   astar (default) | theta (Lazy Theta*: shorter paths with fewer corners)
 *
 * Before you run it: arm the drone, take off and put it in GUIDED mode (for example with
 * QGroundControl's Takeoff, or in MAVProxy: `mode guided`, `arm throttle`, `takeoff 5`). At the goal
 * the drone hovers. Press Ctrl-C to stop the demo.
 */
#include "io/mavsdk_vehicle.hpp"
#include "io/sim_reader.hpp"
#include "mapping/log_odds_mapper.hpp"
#include "pipeline/clients.hpp"
#include "pipeline/threads.hpp"
#include "planning/path_following_local_planner.hpp"
#include "planning/search_planners.hpp"
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

}

int main(int argc, char** argv) {
	if (argc < 4) {
		std::cerr << "usage: local_planner_demo <dx> <dy> <dz> [world] [astar|theta]\n";
		return 2;
	}
	const Eigen::Vector3d offset(std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3]));
	const std::string planner_name = argc > 5 ? argv[5] : "astar";
	if (planner_name != "astar" && planner_name != "theta") {
		std::cerr << "unknown planner '" << planner_name << "': use astar or theta\n";
		return 2;
	}

	using namespace path3d;
	using Map = LogOddsGridMap;

	PipelineConfig config;
	if (argc > 4) config.sim_reader.world_frame = argv[4];
	config.mapping.keyframe_period = 1000ms;  // the planners' maps are ready sooner

	config.mapping.publish_period = 0.1;
	config.map_visualizer.rate_hz = 5.0;
	config.log_odds_mapper.resolution = 0.5;

	config.log_odds_mapper.max_range = 30.0f;

	// One SearchSpaceConfig for both planners: with a larger margin in the local planner, every
	// global path would look blocked and the drone would replan forever.
	const SearchSpaceConfig space;
	SearchPlannerConfig global_config;
	global_config.space = space;
	PathFollowerConfig local_config;
	local_config.space = space;

	// The map, as in path3d_main.
	SensorTopics sensors;
	Topic<Map::Update> map_updates;
	LogOddsMapper mapper(config.log_odds_mapper);
	MappingThread<Map> mapping(sensors, mapper, map_updates, config.mapping);
	mapping.start();
	UavClient uav(sensors.pose);
	Topic<VehicleState> vehicle_state;
	const auto latest_state = vehicle_state.subscribe_latest();
	SimReader reader(config.sim_reader, sensors);
	// After the reader, which brings up ROS: publishes the map for Foxglove.
	LogOddsMapVisualizerConfig visualizer_config = config.map_visualizer;
	visualizer_config.frame_id = config.sim_reader.world_frame;
	LogOddsMapVisualizer visualizer(map_updates, visualizer_config);

	// The vehicle, and the commander that sends it the trajectory.
	Topic<Trajectory> trajectories;
	MavsdkVehicle vehicle(config.mavsdk, sensors.pose, vehicle_state);
	CommanderThread commander(vehicle, trajectories, vehicle_state, sensors.pose, config.commander);
	commander.start();
	std::signal(SIGINT, on_signal);

	// Wait until the drone is flying and ready for our commands.
	while (!g_interrupted) {
		const auto state = latest_state->latest();
		if (state && uav.pose() && state->connected && state->armed && state->frames_ready && state->mode == FlightMode::External) break;
		std::cout << "waiting for a flying drone in GUIDED mode (armed: " << (state && state->armed) << ", mode: " << (state ? to_string(state->mode) : "unknown") << ")\n";
		std::this_thread::sleep_for(2s);
	}
	if (g_interrupted) return 0;

	// The goal: `offset` from where the drone is now.
	const Eigen::Vector3d start = *uav.position();
	const Eigen::Vector3d goal = start + offset;
	std::cout << "flying from " << start.transpose() << " to " << goal.transpose() << " with " << planner_name << "\n";

	// The planners and their threads.
	MapClient<Map> global_map(map_updates, config.global_planning.map_client);
	MapClient<Map> local_map(map_updates, config.local_planning.map_client);
	UavClient global_uav(sensors.pose);
	UavClient local_uav(sensors.pose);
	std::unique_ptr<GlobalPlanner<Map, ReplanRequest, GlobalPath>> global_planner;
	if (planner_name == "theta") {
		global_planner = std::make_unique<LazyThetaStarGlobalPlanner<Map>>(global_map, global_uav, goal, global_config);
	} else {
		global_planner = std::make_unique<AStarGlobalPlanner<Map>>(global_map, global_uav, goal, global_config);
	}
	PathFollowingLocalPlanner<Map> local_planner(local_map, local_uav, local_config);
	Pipe<GlobalPath, ReplanRequest> pipe;
	GlobalPlanningThread<Map, GlobalPath, ReplanRequest> global_thread(*global_planner, pipe, std::nullopt, config.global_planning);
	LocalPlanningThread<Map, GlobalPath, ReplanRequest> local_thread(local_planner, pipe, trajectories, config.local_planning);
	global_thread.start();
	local_thread.start();

	// Report until the drone arrives.
	while (!g_interrupted && (*uav.position() - goal).norm() > local_config.goal_tolerance) {
		std::cout << "distance to the goal: " << (*uav.position() - goal).norm() << " m, requests: " << local_thread.sent_request_count()
		          << ", global plans: " << global_thread.plan_call_count() << ", commander holds: " << commander.hold_count() << "\n";
		std::this_thread::sleep_for(1s);
	}
	std::cout << (g_interrupted ? "stopped\n" : "arrived\n");
}
