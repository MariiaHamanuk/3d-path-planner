/** @file local_planner_demo.cpp
 * @brief Flies the drone to a goal around obstacles: a global planner plans the path, the
 * TrajectoryLocalPlanner flies it. The drone must already be flying.
 *
 * Usage: ros2 run path3d local_planner_demo <x> <y> <z> [world] [planner]
 *   x y z    the goal in the world frame (ENU: x east, y north, z up), m
 *   world    default | city (must be the world the simulation runs)
 *   planner  astar (default) | theta | dijkstra | rrt | rrt_star | rrt_connect
 *
 * Before you run it: arm the drone, take off and put it in GUIDED mode (for example with
 * QGroundControl's Takeoff, or in MAVProxy: `mode guided`, `arm throttle`, `takeoff 5`). At the goal
 * the drone hovers until you press Ctrl-C; the demo also stops when the drone is disarmed (for
 * example after a crash) or leaves GUIDED mode.
 *
 * It logs what explains a crash: when and how far ahead the path got blocked, at what speed and
 * braking distance, emergency stops, how old the map was, flying where the camera does not look,
 * getting close to obstacles, the drone flying away from the plan, and every request with the time
 * its answer took. Foxglove shows the global path (/planner/planned_path), the flown path
 * (/planner/path), the smoothed local path (/planner/local_path), the trajectory
 * (/planner/trajectory) and the stop point (/planner/stop_point).
 */
#include "io/mavsdk_vehicle.hpp"
#include "io/sim_reader.hpp"
#include "mapping/log_odds_mapper.hpp"
#include "pipeline/clients.hpp"
#include "pipeline/threads.hpp"
#include "planning/global/global_planner_factory.hpp"
#include "planning/local/trajectory_local_planner.hpp"
#include "visualization/log_odds_map_visualizer.hpp"
#include "visualization/plan_visualizer.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace {

std::atomic<bool> g_interrupted{false};
void on_signal(int) { g_interrupted = true; }

double degrees(double radians) { return radians * 180.0 / std::numbers::pi; }

/**
 * @brief Prints the events of a flight that tell why it went wrong, from the local planner's
 * reports and the requests it sends. Times are sim time since the first report.
 */
class FlightLog {
public:
	/// Thresholds of the warnings.
	struct Limits {
		double close = 0.5;              ///< nearer to an occupied voxel than this is "close", m
		path3d::TimeSec old_map = 0.5;   ///< a map older than this is "old" while the drone moves, s
		double blind_angle = 0.6;        ///< moving this far off the camera axis is "blind", rad (the camera sees +-0.52)
		double moving_speed = 1.0;       ///< at this speed or more the drone is moving (for "blind" and "old map"), m/s
	};

	FlightLog(path3d::Topic<path3d::TrajectoryPlannerReport>& reports, path3d::Topic<path3d::ReplanRequest>& requests, Limits limits)
	    : m_reports(reports.subscribe_queue(1024)), m_requests(requests.subscribe_queue(64)), m_limits(limits) {}

	/// Prints the events since the last call.
	void update() {
		while (auto request = m_requests->pop(0ms)) {
			m_request_stamps[request->id] = request->stamp;
			std::printf("[%7.2f] request #%llu (%s) from (%.1f %.1f %.1f)\n", time(request->stamp), static_cast<unsigned long long>(request->id),
			            path3d::to_string(request->reason), request->position.x(), request->position.y(), request->position.z());
		}
		while (auto report = m_reports->pop(0ms)) take(*report);
		std::fflush(stdout);
	}

	/// One line about now.
	void status(const Eigen::Vector3d& goal) const {
		if (!m_last) return;
		const path3d::TrajectoryPlannerReport& r = *m_last;
		std::printf("[%7.2f] goal in %.1f m | %s, commanded %.1f m/s, flying %.1f m/s, %.2f m from the plan | map age %.2f s | clearance %s\n",
		            time(r.stamp), (r.position - goal).norm(), path3d::to_string(r.state), r.velocity.norm(), r.actual_velocity.norm(),
		            r.tracking_error, r.map_age, distance(r.clearance).c_str());
	}

	/// The planner has brought the drone to the goal.
	bool arrived() const { return m_last && m_last->state == path3d::TrajectoryPlannerState::Arrived; }

	/// What happened over the whole flight.
	void summary() const {
		std::printf("summary: %zu blocked (%zu emergency stops), closest to an obstacle %s, oldest map %.2f s, %zu cycles flying blind, "
		            "%zu plan restarts from the real drone (farthest %.2f m from the plan)\n",
		            m_blocked_count, m_emergency_count, distance(m_closest).c_str(), m_oldest_map, m_blind_cycles, m_resync_count, m_worst_tracking);
	}

private:
	void take(const path3d::TrajectoryPlannerReport& r) {
		if (!m_start) m_start = r.stamp;
		const double speed = r.velocity.norm();

		if (r.route) {
			const auto sent = m_request_stamps.find(r.route_request_id);
			const std::string waited = sent == m_request_stamps.end() ? "" : ", " + seconds(r.stamp - sent->second) + " after the request";
			std::printf("[%7.2f] answer #%llu: %s, %zu waypoints%s\n", time(r.stamp), static_cast<unsigned long long>(r.route_request_id),
			            *r.route == path3d::PlanStatus::Ok ? "ok" : "FAILED", r.route_waypoints, waited.c_str());
		}
		if (!m_last || m_last->state != r.state) {
			std::printf("[%7.2f] state: %s -> %s\n", time(r.stamp), m_last ? path3d::to_string(m_last->state) : "start", path3d::to_string(r.state));
		}

		const bool was_blocked = m_last && m_last->blocked;
		if (r.blocked && !was_blocked) {
			++m_blocked_count;
			m_blocked_since = r.stamp;
			std::printf("[%7.2f] BLOCKED: margin %s ahead, obstacle %s ahead, speed %.1f m/s needs %.1f m to brake, map age %.2f s, camera %.0f deg off the motion\n",
			            time(r.stamp), distance(r.margin_distance).c_str(), distance(r.obstacle_distance).c_str(), speed, r.braking_distance, r.map_age, degrees(r.yaw_error));
		}
		if (r.emergency && !(m_last && m_last->emergency)) {
			++m_emergency_count;
			std::printf("[%7.2f] EMERGENCY STOP: obstacle %s ahead, margin %s ahead, speed %.1f m/s needs %.1f m to brake\n", time(r.stamp),
			            distance(r.obstacle_distance).c_str(), distance(r.margin_distance).c_str(), speed, r.braking_distance);
		}
		if (!r.blocked && was_blocked) std::printf("[%7.2f] path ahead free again after %s\n", time(r.stamp), seconds(r.stamp - m_blocked_since).c_str());

		m_worst_tracking = std::max(m_worst_tracking, r.tracking_error);
		if (r.resynced) {
			++m_resync_count;
			if (!(m_last && m_last->resynced)) {
				std::printf("[%7.2f] RESYNC: the drone is %.2f m from the plan and flies %.1f m/s (planned %.1f m/s): planning from the drone\n",
				            time(r.stamp), r.tracking_error, r.actual_velocity.norm(), m_last ? m_last->velocity.norm() : 0.0);
			}
		}

		m_closest = std::min(m_closest, r.clearance);
		const bool close = r.clearance < m_limits.close;
		if (close && !m_close) {
			m_close_min = r.clearance;
			std::printf("[%7.2f] CLOSE: %.2f m from an occupied voxel at %.1f m/s (%s)\n", time(r.stamp), r.clearance, speed, path3d::to_string(r.state));
		}
		if (close) m_close_min = std::min(m_close_min, r.clearance);
		if (!close && m_close) std::printf("[%7.2f] away from the obstacle again; nearest was %.2f m\n", time(r.stamp), m_close_min);
		m_close = close;

		/// the map's stamp is its last change: while the drone hovers and sees nothing new it only ages
		const bool moving = r.actual_velocity.norm() >= m_limits.moving_speed;
		if (moving) m_oldest_map = std::max(m_oldest_map, r.map_age);
		const bool old_map = moving && r.map_age > m_limits.old_map;
		if (old_map && !m_old_map) std::printf("[%7.2f] OLD MAP: planning on a map %.2f s old\n", time(r.stamp), r.map_age);
		m_old_map = old_map;

		const bool blind = speed >= m_limits.moving_speed && r.yaw_error > m_limits.blind_angle;
		if (blind) ++m_blind_cycles;
		if (blind && !m_blind) {
			std::printf("[%7.2f] FLYING BLIND: moving %.0f deg off where the camera looks, at %.1f m/s\n", time(r.stamp), degrees(r.yaw_error), speed);
		}
		m_blind = blind;

		m_last = r;
	}

	double time(path3d::TimeSec stamp) const { return stamp - m_start.value_or(stamp); }

	static std::string distance(double meters) {
		if (std::isinf(meters)) return "-";
		char text[32];
		std::snprintf(text, sizeof(text), "%.1f m", meters);
		return text;
	}

	static std::string seconds(path3d::TimeSec value) {
		char text[32];
		std::snprintf(text, sizeof(text), "%.2f s", value);
		return text;
	}

	std::shared_ptr<path3d::BoundedQueue<path3d::TrajectoryPlannerReport>> m_reports;
	std::shared_ptr<path3d::BoundedQueue<path3d::ReplanRequest>> m_requests;
	Limits m_limits;

	std::optional<path3d::TimeSec> m_start;
	std::optional<path3d::TrajectoryPlannerReport> m_last;
	std::map<std::uint64_t, path3d::TimeSec> m_request_stamps;
	path3d::TimeSec m_blocked_since = 0.0;
	bool m_close = false;
	bool m_old_map = false;
	bool m_blind = false;
	double m_close_min = path3d::TrajectoryPlannerReport::kNone;

	std::size_t m_blocked_count = 0;
	std::size_t m_emergency_count = 0;
	std::size_t m_blind_cycles = 0;
	std::size_t m_resync_count = 0;
	double m_worst_tracking = 0.0;
	double m_closest = path3d::TrajectoryPlannerReport::kNone;
	path3d::TimeSec m_oldest_map = 0.0;
};

}

int main(int argc, char** argv) {
	using namespace path3d;
	using Map = LogOddsGridMap;

	if (argc < 4) {
		std::cerr << "usage: local_planner_demo <x> <y> <z> [world] [" << global_planner_names() << "]\n";
		return 2;
	}
	const Eigen::Vector3d goal(std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3]));
	const std::string planner_name = argc > 5 ? argv[5] : "astar";

	PipelineConfig config;
	if (argc > 4) config.sim_reader.world_frame = argv[4];
	config.mapping.keyframe_period = 1000ms;  // the planners' maps are ready sooner
	config.mapping.publish_period = 0.1;
	config.map_visualizer.rate_hz = 5.0;
	config.log_odds_mapper.resolution = 0.5;
	config.log_odds_mapper.max_range = 30.0f;

	// The global planner keeps a larger margin than the local one, so its paths do not look blocked
	// to the local planner as soon as the drone drifts a little. The local planner smooths the path
	// with the global margin, and asks for routes only from points that free.
	SearchPlannerConfig global_config;
	global_config.margin = 1.0;
	TrajectoryLocalPlannerConfig local_config;
	local_config.margin = 0.5;
	local_config.global_margin = global_config.margin;
	if (local_config.margin >= global_config.margin) {
		std::cerr << "the local margin must be smaller than the global one: every global path would look blocked\n";
		return 2;
	}

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

	// The planners, made before the drone is ready so that a wrong name fails at once.
	MapClient<Map> global_map(map_updates, config.global_planning.map_client);
	MapClient<Map> local_map(map_updates, config.local_planning.map_client);
	UavClient global_uav(sensors.pose);
	UavClient local_uav(sensors.pose);
	const auto global_planner = make_global_planner(planner_name, global_map, global_uav, goal, global_config);
	if (!global_planner) {
		std::cerr << "unknown planner '" << planner_name << "': " << global_planner_names() << "\n";
		return 2;
	}
	Topic<TrajectoryPlannerReport> reports;
	TrajectoryLocalPlanner<Map> local_planner(local_map, local_uav, local_config, &reports);

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
	std::cout << "flying from " << uav.position()->transpose() << " to " << goal.transpose() << " with " << planner_name << "\n";

	// The planning threads, and the paths for Foxglove.
	Pipe<GlobalPath, ReplanRequest> pipe;
	FlightLog log(reports, pipe.up(), FlightLog::Limits{.close = local_config.margin});
	PlanVisualizerConfig plan_visualizer_config;
	plan_visualizer_config.frame_id = config.sim_reader.world_frame;
	PlanVisualizer plan_visualizer(pipe.down(), sensors.pose, trajectories, reports, plan_visualizer_config);
	GlobalPlanningThread<Map, GlobalPath, ReplanRequest> global_thread(*global_planner, pipe, std::nullopt, config.global_planning);
	LocalPlanningThread<Map, GlobalPath, ReplanRequest> local_thread(local_planner, pipe, trajectories, config.local_planning);
	global_thread.start();
	local_thread.start();

	// Log with a status line every second. At the goal the planner keeps the drone there (the demo
	// must not end: the autopilot would go on extrapolating the last setpoint) until Ctrl-C, or until
	// the drone stops taking our setpoints (disarmed, e.g. after a crash, or another mode).
	auto next_status = std::chrono::steady_clock::now();
	bool arrived = false;
	std::string ended = "stopped (Ctrl-C)";
	while (!g_interrupted) {
		log.update();
		if (!arrived && log.arrived()) {
			arrived = true;
			std::cout << "arrived: holding the drone at the goal; press Ctrl-C to stop\n";
			log.summary();
		}
		const auto state = latest_state->latest();
		if (!state || !state->armed || state->mode != FlightMode::External) {
			ended = state && !state->armed ? "stopped: the drone was disarmed (a crash?)" : "stopped: the drone left GUIDED mode";
			break;
		}
		if (std::chrono::steady_clock::now() >= next_status) {
			next_status += 1s;
			if (!arrived) {
				log.status(goal);
				std::cout << "          requests " << local_thread.sent_request_count() << " (timed out " << local_thread.request_timeout_count() << ", stale answers "
				          << local_thread.stale_route_count() << "), global plans " << global_thread.plan_call_count() << ", commander holds " << commander.hold_count() << "\n";
			}
		}
		std::this_thread::sleep_for(100ms);
	}
	local_thread.stop();
	global_thread.stop();
	log.update();
	std::cout << ended << "\n";
	log.summary();
}
