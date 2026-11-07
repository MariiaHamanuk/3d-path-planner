/** @file line_planner_demo.cpp
 * @brief Flies the drone along a straight line with the Line planners.
 *
 * Usage: ros2 run path3d line_planner_demo <yaw_deg> <distance_m> [world]
 *   yaw_deg     direction of the line in the world frame, degrees counter-clockwise from east
 *               (0 = east, 90 = north)
 *   distance_m  length of the line, m
 *   world       default | city (must be the world the simulation runs)
 *
 * The drone arms, takes off to 5 m, flies the line at 2 m/s, and lands at its end.
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

constexpr double kAltitude = 5.0;  ///< m above the start
constexpr double kSpeed = 2.0;     ///< m/s along the line

std::atomic<bool> g_interrupted{false};
void on_signal(int) { g_interrupted = true; }

/// Waits until `done()` is true; false on timeout or Ctrl-C.
template <class Done>
bool wait_until(Done done, std::chrono::seconds timeout) {
	const auto deadline = std::chrono::steady_clock::now() + timeout;
	while (!done()) {
		if (g_interrupted || std::chrono::steady_clock::now() > deadline) return false;
		std::this_thread::sleep_for(100ms);
	}
	return true;
}

}

int main(int argc, char** argv) {
	if (argc < 3) {
		std::cerr << "usage: line_planner_demo <yaw_deg> <distance_m> [world]\n";
		return 2;
	}
	const double yaw = std::stod(argv[1]) * std::numbers::pi / 180.0;
	const double distance = std::stod(argv[2]);

	using namespace path3d;
	using Map = LogOddsGridMap;

	PipelineConfig config;
	if (argc > 3) config.sim_reader.world_frame = argv[3];
	config.mapping.keyframe_period = 1000ms;  ///< the planners' maps are ready sooner

	/// Topics, observers and the map pipeline: the same as in path3d_main.
	SensorTopics sensors;
	Topic<Map::Update> map_updates;
	Topic<Trajectory> trajectories;
	Topic<VehicleState> vehicle_state;
	const auto latest_state = vehicle_state.subscribe_latest();
	UavClient uav(sensors.pose);
	LogOddsMapper mapper(config.log_odds_mapper);
	MappingThread<Map> mapping(sensors, mapper, map_updates, config.mapping);
	mapping.start();
	SimReader reader(config.sim_reader, sensors);
	LogOddsMapVisualizerConfig visualizer_config = config.map_visualizer;
	visualizer_config.frame_id = config.sim_reader.world_frame;
	LogOddsMapVisualizer visualizer(map_updates, visualizer_config);

	/// The vehicle and the commander, which sends the planner's trajectory to it.
	MavsdkVehicle vehicle(config.mavsdk, sensors.pose, vehicle_state);
	CommanderThread commander(vehicle, trajectories, vehicle_state, sensors.pose, config.commander);
	commander.start();
	std::signal(SIGINT, on_signal);

	/// Get the drone into the air and under our control.
	std::cout << "waiting for the simulation and the autopilot...\n";
	if (!wait_until([&] { return uav.pose().has_value(); }, 60s)) {
		std::cerr << "no pose: is the simulation running?\n";
		return 1;
	}
	if (!wait_until([&] {
		    const auto state = latest_state->latest();
		    return state && state->connected && state->frames_ready;
	    },
	                120s)) {
		std::cerr << "the autopilot is not ready (is it connected and the drone standing still?)\n";
		return 1;
	}
	if (!wait_until([&] { return vehicle.arm(5s); }, 120s)) return 1;
	std::cout << "armed, taking off\n";
	if (!vehicle.takeoff(kAltitude, 20s)) return 1;
	if (!wait_until([&] { return uav.position()->z() > kAltitude - 0.5; }, 60s)) return 1;
	if (!vehicle.set_mode(FlightMode::External, 5s)) return 1;

	/// The line: from here, `distance` m in the direction `yaw`, at the same height.
	const Eigen::Vector3d start = *uav.position();
	const Eigen::Vector3d goal = start + distance * Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0.0);
	std::cout << "flying from " << start.transpose() << " to " << goal.transpose() << "\n";

	/// The planners and their threads.
	MapClient<Map> global_map(map_updates, config.global_planning.map_client);
	MapClient<Map> local_map(map_updates, config.local_planning.map_client);
	UavClient global_uav(sensors.pose);
	UavClient local_uav(sensors.pose);
	LineGlobalPlanner global_planner(global_map, global_uav, goal);
	LineLocalPlanner local_planner(local_map, local_uav, kSpeed);
	Pipe<GlobalPath, ReplanRequest> pipe;
	GlobalPlanningThread<Map, GlobalPath, ReplanRequest> global_thread(global_planner, pipe, std::nullopt, config.global_planning);
	LocalPlanningThread<Map, GlobalPath, ReplanRequest> local_thread(local_planner, pipe, trajectories, config.local_planning);
	global_thread.start();
	local_thread.start();

	/// Wait for the end of the line.
	const auto arrived = [&] { return (*uav.position() - goal).norm() < 0.5; };
	const auto timeout = std::chrono::seconds(static_cast<long>(distance / kSpeed) + 60);
	const auto started = std::chrono::steady_clock::now();
	auto last_print = started;
	while (!arrived() && !g_interrupted && std::chrono::steady_clock::now() - started < timeout) {
		std::this_thread::sleep_for(100ms);
		if (std::chrono::steady_clock::now() - last_print >= 1s) {
			last_print = std::chrono::steady_clock::now();
			std::cout << "distance to the goal: " << (*uav.position() - goal).norm() << " m\n";
		}
	}
	std::cout << (arrived() ? "arrived, landing\n" : "stopped, landing\n");

	vehicle.land(10s);
	wait_until([&] {
		const auto state = latest_state->latest();
		return state && !state->armed;
	},
	           60s);
	return 0;
}
