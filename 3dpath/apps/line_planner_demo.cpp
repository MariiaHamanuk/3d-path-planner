/** @file line_planner_demo.cpp
 * @brief Flies the drone along a line with the Line planners. The drone must already be flying.
 *
 * Usage: ros2 run path3d line_planner_demo <yaw_deg> <distance_m> [world]
 *   yaw_deg     direction of the line in the world frame, degrees counter-clockwise from east
 *               (0 = east, 90 = north)
 *   distance_m  length of the line, m
 *   world       default | city (must be the world the simulation runs)
 *
 * Before you run it: arm the drone, take off and put it in GUIDED mode (for example with
 * QGroundControl's Takeoff, or in MAVProxy: `mode guided`, `arm throttle`, `takeoff 5`). The line
 * starts where the drone is, at the same height, and the demo flies it at 2 m/s. At the end of the
 * line the drone stays there. Press Ctrl-C to stop the demo.
 * Tutorial: docs/tutorial_line_planner.md.
 */
#include "io/mavsdk_vehicle.hpp"
#include "io/sim_reader.hpp"
#include "mapping/log_odds_mapper.hpp"
#include "pipeline/clients.hpp"
#include "pipeline/threads.hpp"
#include "planning/line_planner.hpp"
#include "visualization/log_odds_map_visualizer.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <numbers>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace {

std::atomic<bool> g_interrupted{false};
void on_signal(int) { g_interrupted = true; }

}

int main(int argc, char** argv) {
	if (argc < 3) {
		std::cerr << "usage: line_planner_demo <yaw_deg> <distance_m> [world]\n";
		return 2;
	}
	const double yaw = std::stod(argv[1]) * std::numbers::pi / 180.0;
	const double distance = std::stod(argv[2]);
	constexpr double speed = 2.0;  // m/s

	using namespace path3d;
	using Map = LogOddsGridMap;

	PipelineConfig config;
	if (argc > 3) config.sim_reader.world_frame = argv[3];
	config.mapping.keyframe_period = 1000ms;  // the planners' maps are ready sooner

	config.mapping.publish_period = 0.1;
	config.map_visualizer.rate_hz = 5.0;
	config.log_odds_mapper.resolution = 0.5;

	config.log_odds_mapper.max_range = 30.0f;

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

	// The line: from here, `distance` m in the direction `yaw`, at the same height.
	const Eigen::Vector3d start = *uav.position();
	const Eigen::Vector3d goal = start + distance * Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
	std::cout << "flying from " << start.transpose() << " to " << goal.transpose() << "\n";

	// The planners and their threads.
	MapClient<Map> global_map(map_updates, config.global_planning.map_client);
	MapClient<Map> local_map(map_updates, config.local_planning.map_client);
	UavClient global_uav(sensors.pose);
	UavClient local_uav(sensors.pose);
	LineGlobalPlanner global_planner(global_map, global_uav, goal);
	LineLocalPlanner local_planner(local_map, local_uav, speed);
	Pipe<GlobalPath, ReplanRequest> pipe;
	GlobalPlanningThread<Map, GlobalPath, ReplanRequest> global_thread(global_planner, pipe, std::nullopt, config.global_planning);
	LocalPlanningThread<Map, GlobalPath, ReplanRequest> local_thread(local_planner, pipe, trajectories, config.local_planning);
	global_thread.start();
	local_thread.start();

	// Report until the drone arrives.
	while (!g_interrupted && (*uav.position() - goal).norm() > 0.5) {
		std::cout << "distance to the goal: " << (*uav.position() - goal).norm() << " m\n";
		std::this_thread::sleep_for(1s);
	}
	std::cout << (g_interrupted ? "stopped\n" : "arrived\n");
}
