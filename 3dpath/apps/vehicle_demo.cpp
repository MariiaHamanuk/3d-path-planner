/** @file vehicle_demo.cpp
 * @brief Flies a 10 m square in the simulation through Vehicle + CommanderStage, without any planner:
 * ros2 run path3d vehicle_demo [world]      (world = default | city; the sim must be running)
 * SimReader gives the Gazebo pose, MavsdkVehicle talks to ArduCopter SITL through MAVProxy
 * (`--out=udp:127.0.0.1:14560`, see scripts/tmux.yml), and the CommanderStage streams the square.
 * This program plays the local planner: it republishes the trajectory every 100 ms, as a planner
 * must (the commander holds position once a trajectory is older than max_trajectory_age).
 */
#include "core/config.hpp"
#include "io/mavsdk_vehicle.hpp"
#include "io/sim_reader.hpp"
#include "pipeline/stages.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <functional>
#include <iostream>
#include <thread>

namespace {

using namespace std::chrono_literals;

std::atomic<bool> g_interrupted{false};
void on_signal(int) { g_interrupted = true; }

constexpr double kTakeoffAltitude = 5.0;
constexpr double kSide = 10.0;
constexpr double kSpeed = 1.0;
constexpr double kStep = 0.1;

/// Waits until `done` or `timeout` (or Ctrl-C); false on timeout or interrupt.
bool wait_until(const std::function<bool()>& done, std::chrono::seconds timeout) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!done()) {
    if (g_interrupted || std::chrono::steady_clock::now() > deadline) return false;
    std::this_thread::sleep_for(50ms);
  }
  return true;
}

/// The square starting at `start` (east, north, west, south), flown from time `begin` at kSpeed.
path3d::Trajectory square(const Eigen::Vector3d& start, double yaw, path3d::TimeSec begin) {
  const Eigen::Vector3d corners[] = {start, start + Eigen::Vector3d(kSide, 0, 0),
                                     start + Eigen::Vector3d(kSide, kSide, 0),
                                     start + Eigen::Vector3d(0, kSide, 0), start};
  path3d::Trajectory trajectory;
  path3d::TimeSec t = begin;
  for (int edge = 0; edge < 4; ++edge) {
    const Eigen::Vector3d direction = (corners[edge + 1] - corners[edge]).normalized();
    const double duration = kSide / kSpeed;
    for (double s = 0.0; s < duration; s += kStep) {
      trajectory.points.push_back({.stamp = t + s,
                                   .position = corners[edge] + direction * (kSpeed * s),
                                   .velocity = direction * kSpeed,
                                   .acceleration = Eigen::Vector3d::Zero(),
                                   .yaw = yaw});
    }
    t += duration;
  }
  trajectory.points.push_back({.stamp = t, .position = start, .velocity = Eigen::Vector3d::Zero(),
                               .acceleration = Eigen::Vector3d::Zero(), .yaw = yaw});
  return trajectory;
}

}

int main(int argc, char** argv) {
  path3d::PipelineConfig config;
  if (argc > 1) config.sim_reader.world_frame = argv[1];

  /// Subscribers first, the reader last (topics deliver only to existing subscribers).
  path3d::SensorTopics sensors;
  path3d::Topic<path3d::VehicleState> vehicle_state;
  path3d::Topic<path3d::Trajectory> trajectories;
  const auto pose = sensors.pose.subscribe_latest();
  const auto state = vehicle_state.subscribe_latest();
  path3d::MavsdkVehicle vehicle(config.mavsdk, sensors.pose, vehicle_state);
  path3d::CommanderStage commander(vehicle, trajectories, vehicle_state, sensors.pose, config.commander);
  path3d::SimReader reader(config.sim_reader, sensors);
  std::signal(SIGINT, on_signal);  ///< after the reader: rclcpp::init installs its own
  commander.start();

  const auto print_status = [&] {
    const auto s = state->latest();
    const auto p = pose->latest();
    const Eigen::Vector3d position = p ? p->pose.position : Eigen::Vector3d::Zero();
    std::cout << "connected " << (s && s->connected) << " armed " << (s && s->armed) << " mode "
              << (s ? path3d::to_string(s->mode) : "-")
              << " frames " << (s && s->frames_ready) << " | pose "
              << position.transpose()
              << " | sent " << commander.sent_count() << " hold " << commander.hold_count() << "\n";
  };

  std::cout << "waiting for the autopilot and the frame origin (keep the drone on the ground)...\n";
  if (!wait_until([&] { const auto s = state->latest(); return s && s->connected && s->frames_ready && pose->latest(); }, 120s)) {
    std::cerr << "no autopilot link or frame origin: is the sim running with MAVProxy forwarding to "
              << config.mavsdk.connection_url << "?\n";
    print_status();
    return 1;
  }
  std::cout << "NED origin in the world frame: " << vehicle.ned_origin()->transpose() << "\n";

  const Eigen::Vector3d ground = pose->latest()->pose.position;
  std::cout << "arming (retries while pre-arm checks fail)...\n";
  if (!wait_until([&] { return vehicle.arm(2s); }, 120s)) {
    std::cerr << "could not arm\n";
    return 1;
  }
  if (!vehicle.takeoff(kTakeoffAltitude, 3s)) {
    std::cerr << "takeoff refused\n";
    return 1;
  }
  std::cout << "taking off to " << kTakeoffAltitude << " m\n";
  if (!wait_until([&] { return pose->latest()->pose.position.z() > ground.z() + kTakeoffAltitude - 0.5; }, 60s)) {
    std::cerr << "did not reach the takeoff altitude\n";
    return 1;
  }
  /// Hand control to the commander: offboard control starts from the hover position.
  if (!wait_until([&] { return vehicle.set_mode(path3d::FlightMode::External, 2s); }, 30s)) {
    std::cerr << "could not switch to external (GUIDED/offboard) control\n";
    return 1;
  }

  const auto hover = pose->latest();
  const path3d::Trajectory flight =
      square(hover->pose.position, path3d::yaw_of(hover->pose.orientation), hover->stamp + 0.5);
  const path3d::TimeSec end = flight.points.back().stamp;
  std::cout << "flying a " << kSide << " m square (" << end - hover->stamp << " s)\n";
  auto next_print = std::chrono::steady_clock::now();
  while (!g_interrupted && pose->latest()->stamp < end + 2.0) {
    path3d::Trajectory refreshed = flight;
    refreshed.stamp = pose->latest()->stamp;
    trajectories.publish(std::move(refreshed));
    if (std::chrono::steady_clock::now() >= next_print) {
      print_status();
      next_print += 1s;
    }
    std::this_thread::sleep_for(100ms);
  }

  std::cout << "landing\n";
  vehicle.land(3s);
  wait_until([&] { print_status(); return !state->latest()->armed; }, 60s);
  std::cout << "done: sent " << commander.sent_count() << " setpoints, " << commander.hold_count()
            << " holds, " << vehicle.dropped_setpoint_count() << " dropped\n";
  return 0;
}
