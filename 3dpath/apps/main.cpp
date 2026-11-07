/** @file main.cpp
 * @brief Entry point of the planner: wires the stages that exist so far.
 * SimReader ─► SensorTopics (pose, depth, color, calibration, mounts) ─► status line
 * Usage: ros2 run path3d path3d_main [world]      (world = default | city)
 * The mapper and the planner are not implemented yet, so only the reader is wired. When a
 * Mapper<M>, a GlobalPlanner<M> and a LocalPlanner<M> exist, add MappingStage (on `sensors`),
 * GlobalPlanningStage and LocalPlanningStage here, each with its part of `config` (planners get a
 * MapClient on the map update topic and a UavClient on `sensors.pose`). Create every subscriber
 * before the reader. To fly, add a MavsdkVehicle (config.mavsdk) and a CommanderStage that
 * follows the LocalPlanningStage's trajectories; apps/vehicle_demo.cpp shows the vehicle side.
 */
#include "io/sim_reader.hpp"
#include "pipeline/stages.hpp"

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

  /// Declared in dependency order: destroyed in reverse, so the reader stops first.
  path3d::SensorTopics sensors;
  const auto latest_pose = sensors.pose.subscribe_latest();
  const auto latest_depth = sensors.depth.subscribe_latest();
  path3d::SimReader reader(config.sim_reader, sensors);
  /// after the reader: rclcpp::init installs its own
  std::signal(SIGINT, on_signal);

  while (!g_interrupted) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    const auto pose = latest_pose->latest();
    const auto depth = latest_depth->latest();
    std::cout << "pose stamp " << (pose ? pose->stamp : 0.0) << ", depth stamp "
              << (depth ? depth->stamp : 0.0) << "\n";
  }
}
