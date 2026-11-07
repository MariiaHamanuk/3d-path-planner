/** @file main.cpp
 * @brief Entry point of the planner: wires the stages that exist so far.
 * SimReader ─► SensorTopics ─► MappingStage(LogOddsMapper) ─► map updates ─► MapClient ─► status line
 *                                                                          └─► LogOddsMapVisualizer ─► Foxglove
 * Usage: ros2 run path3d path3d_main [world]      (world = default | city)
 * The planners are not implemented yet. When a GlobalPlanner<LogOddsGridMap> and a
 * LocalPlanner<LogOddsGridMap> exist, add GlobalPlanningStage and LocalPlanningStage here, each
 * with its part of `config` (planners get a MapClient on `map_updates` and a UavClient on
 * `sensors.pose`). Create every subscriber before the reader. To fly, add a MavsdkVehicle
 * (config.mavsdk) and a CommanderStage that follows the LocalPlanningStage's trajectories;
 * apps/vehicle_demo.cpp shows the vehicle side.
 */
#include "io/sim_reader.hpp"
#include "mapping/log_odds_mapper.hpp"
#include "pipeline/clients.hpp"
#include "pipeline/stages.hpp"
#include "visualization/log_odds_map_visualizer.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <thread>

namespace {

std::atomic<bool> g_interrupted{false};
void on_signal(int) { g_interrupted = true; }

}

int main(int argc, char** argv) {
	path3d::PipelineConfig config;
	if (argc > 1) config.sim_reader.world_frame = argv[1];
	/// The drone flies ~10 m/s: a diff every 0.2 s of sim time or a refresh every 0.5 s would leave the map 2-5 m behind it.
	config.mapping.publish_period = 0.05;
	config.map_visualizer.rate_hz = 10.0;
	
	config.log_odds_mapper.max_range = 30.0f;

	/// Declared in dependency order: destroyed in reverse, so the reader stops first.
	path3d::SensorTopics sensors;
	const auto latest_pose = sensors.pose.subscribe_latest();
	const auto latest_depth = sensors.depth.subscribe_latest();

	using Map = path3d::LogOddsGridMap;
	path3d::Topic<Map::Update> map_updates;
	/// A reader of the map, as a planner will be (created before the mapper publishes).
	path3d::MapClient<Map> map_client(map_updates);
	path3d::LogOddsMapper mapper(config.log_odds_mapper);
	path3d::MappingStage<Map> mapping(sensors, mapper, map_updates, config.mapping);
	mapping.start();
	path3d::SimReader reader(config.sim_reader, sensors);
	/// After the reader, which brings up ROS.
	path3d::LogOddsMapVisualizerConfig visualizer_config = config.map_visualizer;
	visualizer_config.frame_id = config.sim_reader.world_frame;
	path3d::LogOddsMapVisualizer visualizer(map_updates, visualizer_config);
	/// after the reader: rclcpp::init installs its own
	std::signal(SIGINT, on_signal);

	while (!g_interrupted) {
		std::this_thread::sleep_for(std::chrono::seconds(1));
		const auto pose = latest_pose->latest();
		const auto depth = latest_depth->latest();
		map_client.sync();
		const Map& map = map_client.map();
		std::cout << "pose stamp " << (pose ? pose->stamp : 0.0) << ", depth stamp "
		          << (depth ? depth->stamp : 0.0) << " | map v" << map.version << ": "
		          << map.known_count() << " voxels, " << map.occupied_count() << " occupied | frames "
		          << mapping.integrated_count() << " (dropped " << mapping.dropped_count()
		          << ", unsynced " << mapping.unsynced_count() << ")\n";
	}
}
