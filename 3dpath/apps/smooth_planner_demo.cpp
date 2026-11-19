/** @file smooth_planner_demo.cpp
 * @brief Flies the drone to a goal with LazyThetaStarGlobalPlanner and SmoothLocalPlanner. The drone
 * must already be flying.
 *
 * Usage: ros2 run path3d smooth_planner_demo <x> <y> <z> [speed] [world]
 *   x y z   goal in the world frame (ENU), m
 *   speed   cruise speed, m/s (default 5)
 *   world   default | city (must be the world the simulation runs)
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
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace {

std::atomic<bool> g_interrupted{false};
void on_signal(int) { g_interrupted = true; }

}

int main(int argc, char** argv) {
	if (argc < 4) {
		std::cerr << "usage: smooth_planner_demo <x> <y> <z> [speed] [world]\n";
		return 2;
	}
	const Eigen::Vector3d goal(std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3]));
	const double speed = argc > 4 ? std::stod(argv[4]) : 5.0;

	using namespace path3d;
	using Map = LogOddsGridMap;

	PipelineConfig config;
	if (argc > 5) config.sim_reader.world_frame = argv[5];
	config.mapping.keyframe_period = 1000ms;
	config.mapping.publish_period = 0.1;
	config.map_visualizer.rate_hz = 5.0;
	config.log_odds_mapper.resolution = 0.5;
	config.log_odds_mapper.max_range = 30.0f;
	config.local_planning.request_timeout = 500ms;

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
	global_config.space.margin = 0.5;
	global_config.max_expansions = 200000;
	LazyThetaStarGlobalPlanner<Map> global_planner(global_map, global_uav, goal, global_config);

	SmoothLocalPlannerConfig local_config;
	local_config.max_speed = speed;
	local_config.space.margin = 0.6;
	SmoothLocalPlanner<Map> local_planner(local_map, local_uav, local_config);

	Pipe<GlobalPath, ReplanRequest> pipe;
	GlobalPlanningThread<Map, GlobalPath, ReplanRequest> global_thread(global_planner, pipe, std::nullopt, config.global_planning);
	LocalPlanningThread<Map, GlobalPath, ReplanRequest> local_thread(local_planner, pipe, trajectories, config.local_planning);
	global_thread.start();
	local_thread.start();

	while (!g_interrupted && (*uav.position() - goal).norm() > 0.5) {
		std::cout << "distance to the goal: " << (*uav.position() - goal).norm() << " m | replans " << local_thread.sent_request_count()
		          << " (timed out " << local_thread.request_timeout_count() << ") | global plans " << global_thread.plan_call_count()
		          << " | holds " << commander.hold_count() << "\n";
		std::this_thread::sleep_for(1s);
	}
	std::cout << (g_interrupted ? "stopped\n" : "arrived\n");
}
