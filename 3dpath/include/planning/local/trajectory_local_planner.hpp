#pragma once

#include "core/types.hpp"
#include "mapping/map_view.hpp"
#include "pipeline/clients.hpp"
#include "planning/utils/search_spaces.hpp"
#include "planning/core/local_planner.hpp"
#include "planning/local/trajectory_planner_report.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace path3d {

/**
 * @brief Settings of the TrajectoryLocalPlanner. Times are sim time.
 *
 * The speeds and accelerations must stay well inside what the autopilot can fly (ArduPilot:
 * WPNAV_ACCEL, WPNAV_ACCEL_Z, WPNAV_SPEED_UP/DN, PSC_JERK_XY/Z): a trajectory at or beyond its
 * limits is flown late and too fast, and the drone overshoots every stop.
 */
struct TrajectoryLocalPlannerConfig {
	double max_speed = 5.0;         ///< top speed along the path, m/s
	double max_acceleration = 2.5;  ///< for speeding up, and for braking before corners, obstacles and the goal, m/s^2
	/// Braking for an obstacle that was found closer than the braking distance at `max_acceleration`
	/// may use up to this, m/s^2. Keep it below what the autopilot can brake with: a harder stop does
	/// not make the drone stop sooner, it only makes it overshoot the setpoint.
	double emergency_deceleration = 4.5;
	/// An emergency stop ends at least this far (along the path) before the first occupied voxel, m.
	double emergency_clearance = 0.3;
	/// Turns: on a curve of radius r the speed is at most sqrt(max_lateral_acceleration * r), m/s^2.
	double max_lateral_acceleration = 2.0;
	double max_vertical_speed = 2.0;  ///< climbing and descending, m/s
	double max_yaw_rate = 1.0;        ///< rad/s
	TimeSec horizon = 3.0;            ///< how far ahead each trajectory reaches, s
	TimeSec step = 0.1;               ///< time between the points of a trajectory, s

	double path_step = 0.25;     ///< spacing of the points of the smoothed path, m
	double corner_radius = 2.0;  ///< a corner is rounded from this far before it to this far after it, m

	/// Free space needed around the drone when the path ahead is checked for obstacles, m. It must be
	/// smaller than `global_margin`: otherwise every global path looks blocked.
	double margin = 0.5;
	/// Whether Unknown space counts as free when the path ahead is checked for obstacles.
	bool unknown_is_free = true;
	/// The global planner's margin, m. The path is shortened and rounded with it, so the smoothing
	/// never brings it closer to obstacles than the global planner did; and the drone asks for a
	/// route only from a point this free, so the global planner never has to move the start.
	double global_margin = 1.0;
	double check_distance = 30.0;  ///< how far along the path obstacles are looked for, m
	double stop_buffer = 1.0;      ///< the drone stops this far before the first point of the path inside the margin, m
	/// Closer than this to an obstacle the speed is at most `caution_speed`: the drone never flies
	/// exactly where it is told, m.
	double caution_margin = 1.5;
	double caution_speed = 2.5;  ///< m/s
	/// A Blocked request asks for a path from where the plan will be this much later (never past
	/// the stop point), so the new path leaves the old one before the obstacle, s.
	TimeSec replan_lead = 1.0;

	/// The camera sees this far to each side of where the drone looks, rad. Moving more than this
	/// off the camera axis is flying blind: at most `blind_speed`, until the yaw has caught up.
	double camera_half_fov = 0.5;
	double blind_speed = 0.7;     ///< m/s
	double yaw_lookahead = 2.0;   ///< the drone looks at the path this far ahead, so it turns before a corner, m

	/// The drone this far from the planned point, or this much faster or slower along the path than
	/// planned, restarts the plan from where it really is and how fast it really moves (m, m/s).
	double resync_distance = 1.0;
	double resync_speed = 1.0;
	TimeSec velocity_window = 0.1;  ///< the drone's velocity is measured over this much of its pose history, s
	double settled_speed = 0.3;     ///< slower than this the drone counts as stopped, m/s

	/// The drone this far from the path (after re-projection) asks for a new path (Deviated), m.
	double deviation_distance = 2.0;
	/// The re-projection of a drone that is off the planned point looks this far back and ahead
	/// along the path, m. Keeps it from jumping to another part of the path that passes nearby.
	double projection_window = 3.0;
	/// A new path that starts this close to the current one (as one does that answers a request
	/// made from a point ahead) is joined to it there, m.
	double join_tolerance = 0.5;
	double goal_tolerance = 0.4;  ///< the drone at rest this close to the end of the path has arrived, m

	/// Wait after a Failed answer, or after an answer that is blocked before the drone could fly it,
	/// before asking again, s.
	TimeSec replan_backoff = 1.0;
	double escape_speed = 0.5;   ///< speed out of the margin of an obstacle, m/s
	double escape_radius = 2.5;  ///< how far around the drone to look for a free point to escape to, m
	double escape_step = 0.25;   ///< spacing of the points tried, m
};

/**
 * @brief Local planner that flies a smoothed version of the GlobalPath with a time-parametrized
 * speed profile, and stops for obstacles on its own.
 *
 * A new route is joined to the path the drone is on, shortened (straight lines between waypoints
 * that see each other), its corners rounded, and sampled every `path_step`. Shortening and
 * rounding keep `global_margin` from obstacles. Each point gets a speed limit from its curvature
 * and its slope.
 *
 * Every cycle continues the previous plan at the time of the pose, so speed, position and yaw stay
 * continuous, also across a change of route. When the real drone is too far from the plan or
 * moves too differently (the autopilot flies late, or overshoots), the plan restarts from the
 * drone's real position and speed: every stop is planned from where the drone really is. From
 * there it plans the next `horizon`: it accelerates with `max_acceleration`, follows the curvature
 * limits, slows down near obstacles and while the camera does not look where the drone goes, and
 * brakes to stop `stop_buffer` before the first point of the path inside the margin of an
 * obstacle, or at the goal. An obstacle found inside the braking distance is braked for with up to
 * `emergency_deceleration`, never past the obstacle. A stop never depends on the global planner:
 * while a path is blocked the drone brakes, then hovers where it stopped and asks for a new route.
 *
 * The drone asks for a route only from a point with `global_margin` around it, so a global planner
 * can start there. A drone that stopped closer to an obstacle first moves slowly to the nearest
 * such point. After a Failed answer, or an answer that is blocked at once, it waits
 * `replan_backoff` before asking again. It always returns a trajectory: a hover trajectory instead
 * of none, because the commander keeps flying the old one for up to `max_trajectory_age`.
 *
 * Every cycle can publish a TrajectoryPlannerReport of what it saw and decided (distances to the
 * obstacle, braking distance, age of the map, how far the drone is from the plan, ...).
 *
 * @tparam M Map type; any OccupancyMap that can be replicated (for example LogOddsGridMap).
 */
template <class M>
    requires ReplicatedMap<M> && OccupancyMap<M>
class TrajectoryLocalPlanner : public LocalPlanner<M, GlobalPath, ReplanRequest> {
public:
	/**
	 * @param map The planner's copy of the map.
	 * @param uav Access to the drone pose.
	 * @param config Speeds, horizon, safety settings (see TrajectoryLocalPlannerConfig).
	 * @param reports Where to publish a report of every cycle; null for none.
	 */
	TrajectoryLocalPlanner(MapClient<M>& map, UavClient& uav, TrajectoryLocalPlannerConfig config = {},
	                       Topic<TrajectoryPlannerReport>* reports = nullptr)
	    : LocalPlanner<M, GlobalPath, ReplanRequest>(map, uav), m_config(config), m_reports(reports) {
		/// the points escape() tries, nearest first
		const int reach = static_cast<int>(std::floor(m_config.escape_radius / m_config.escape_step));
		for (int dx = -reach; dx <= reach; ++dx) {
			for (int dy = -reach; dy <= reach; ++dy) {
				for (int dz = -reach; dz <= reach; ++dz) {
					const Eigen::Vector3d offset = Eigen::Vector3i(dx, dy, dz).cast<double>() * m_config.escape_step;
					if ((dx != 0 || dy != 0 || dz != 0) && offset.norm() <= m_config.escape_radius) m_escape_offsets.push_back(offset);
				}
			}
		}
		std::stable_sort(m_escape_offsets.begin(), m_escape_offsets.end(),
		                 [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) { return a.squaredNorm() < b.squaredNorm(); });
	}

	LocalStep<ReplanRequest> plan(const GlobalPath* new_route) override {
		const StampedPose pose = *this->m_uav.pose();
		if (!m_yaw) m_yaw = yaw_of(pose.pose.orientation);
		measure(pose);
		m_report = TrajectoryPlannerReport{};
		m_report.stamp = pose.stamp;
		m_report.position = pose.pose.position;
		m_report.map_age = pose.stamp - this->m_map.map().stamp;
		if (m_velocity) m_report.actual_velocity = *m_velocity;
		if (new_route) take_route(*new_route, pose);

		LocalStep<ReplanRequest> step = run(pose);
		if (m_reports) publish_report(pose, step);
		return step;
	}

	/// What the planner is doing. Changes only in plan(): do not read it while a thread runs the planner.
	TrajectoryPlannerState state() const { return m_state; }

	/// The smoothed path being flown (empty before the first route). Changes only in plan().
	const std::vector<Eigen::Vector3d>& path() const { return m_points; }

private:
	/// Shorter segments and directions count as zero, m.
	static constexpr double kMinLength = 1e-6;
	/// A plan this close to its stop point is at the stop point, at rest, m.
	static constexpr double kStopSnap = 0.01;
	/// A plan slower than this is at rest, m/s. Not 0: when the planner runs more often than the
	/// trajectory has points (sim time, a real-time factor below 1), the state it continues from is
	/// interpolated between a moving point and a resting one, and only nears 0.
	static constexpr double kRestSpeed = 0.05;
	/// Slower than this (sideways) counts as not moving for the yaw error, m/s.
	static constexpr double kMovingSpeed = 0.3;
	/// How far around the drone the report looks for the nearest occupied voxel, m.
	static constexpr double kClearanceRadius = 3.0;
	/// A route blocked before the drone has flown this much of it was useless, m.
	static constexpr double kFreshRoute = 1.0;
	/// Pose history kept for the velocity, s.
	static constexpr TimeSec kHistory = 1.0;

	/// A state of the plan: time, arc length along the path, speed, yaw.
	struct MotionState {
		TimeSec t = 0.0;
		double s = 0.0;
		double v = 0.0;
		double yaw = 0.0;
	};

	/// Where the path ahead stops being free.
	struct Blockage {
		std::optional<double> margin_s;  ///< arc length of the first point inside the margin of an obstacle
		std::optional<double> hit_s;     ///< arc length of the first point in an occupied voxel
	};

	/// One cycle in the current state.
	LocalStep<ReplanRequest> run(const StampedPose& pose) {
		switch (m_state) {
			case TrajectoryPlannerState::Waiting:
				return ask_for_route(pose);
			case TrajectoryPlannerState::Following:
				return follow(pose);
			case TrajectoryPlannerState::BackingOff:
				return back_off(pose);
			case TrajectoryPlannerState::Escaping:
				return escape(pose);
			case TrajectoryPlannerState::Arrived:
				break;
		}
		return {std::nullopt, hover_trajectory(pose.stamp, *m_hold)};
	}

	/// Adds the pose to the history and measures the drone's velocity over `velocity_window`.
	void measure(const StampedPose& pose) {
		if (m_poses.empty() || pose.stamp > m_poses.back().first) m_poses.emplace_back(pose.stamp, pose.pose.position);
		while (m_poses.size() > 2 && m_poses.front().first < pose.stamp - kHistory) m_poses.pop_front();
		m_velocity.reset();
		const auto& [newest_stamp, newest] = m_poses.back();
		for (auto it = m_poses.rbegin(); it != m_poses.rend(); ++it) {
			const TimeSec span = newest_stamp - it->first;
			if (span >= m_config.velocity_window - 1e-9) {
				m_velocity = (newest - it->second) / span;
				return;
			}
		}
	}

	/// The drone's measured speed, 0 before there is measurement
	double actual_speed() const { return m_velocity ? m_velocity->norm() : 0.0; }

	/// Completes the report of this cycle with its outcome and publishes it.
	void publish_report(const StampedPose& pose, const LocalStep<ReplanRequest>& step) {
		m_report.state = m_state;
		m_report.path = m_shared_path;
		m_report.clearance = clearance(pose.pose.position);
		if (step.trajectory && !step.trajectory->points.empty()) m_report.velocity = step.trajectory->points.front().velocity;
		const Eigen::Vector3d& motion = m_velocity ? *m_velocity : m_report.velocity;
		const Eigen::Vector2d sideways = motion.head<2>();
		if (sideways.norm() > kMovingSpeed) {
			m_report.yaw_error = std::abs(wrap_angle(std::atan2(sideways.y(), sideways.x()) - yaw_of(pose.pose.orientation)));
		}
		if (step.request) {
			m_report.request = step.request->reason;
			m_report.request_position = step.request->position;
		}
		m_reports->publish(std::move(m_report));
	}

	/// Distance from `position` to the nearest occupied voxel within `kClearanceRadius`; kNone if
	/// there is none (or the map is not a grid).
	double clearance(const Eigen::Vector3d& position) const {
		double nearest = TrajectoryPlannerReport::kNone;
		if constexpr (GridMap<M>) {
			const M& map = this->m_map.map();
			const int reach = static_cast<int>(std::ceil(kClearanceRadius / map.resolution()));
			const Eigen::Vector3i center = map.index_of(position);
			const double half = 0.5 * map.resolution();
			for (int dx = -reach; dx <= reach; ++dx) {
				for (int dy = -reach; dy <= reach; ++dy) {
					for (int dz = -reach; dz <= reach; ++dz) {
						const Eigen::Vector3i index = center + Eigen::Vector3i(dx, dy, dz);
						if (map.occupancy_at(index) != Occupancy::Occupied) continue;
						const Eigen::Vector3d offset = ((position - map.center_of(index)).cwiseAbs().array() - half).cwiseMax(0.0).matrix();
						nearest = std::min(nearest, offset.norm());
					}
				}
			}
		}
		return nearest;
	}

	/// An answer of the global planner: a usable route is followed, joined to the current path.
	/// Anything else (Failed, fewer than two distinct waypoints) delays the next request by
	/// `replan_backoff`, and starts the backoff unless the drone is following a route or has arrived.
	void take_route(const GlobalPath& route, const StampedPose& pose) {
		m_report.route = route.status;
		m_report.route_request_id = route.request_id;
		m_report.route_waypoints = route.waypoints.size();
		std::vector<Eigen::Vector3d> waypoints;
		for (const Eigen::Vector3d& waypoint : route.waypoints) {
			if (waypoints.empty() || (waypoint - waypoints.back()).norm() > kMinLength) waypoints.push_back(waypoint);
		}
		if (route.status != PlanStatus::Ok || waypoints.size() < 2) {
			m_request_after = pose.stamp + m_config.replan_backoff;
			if (m_state == TrajectoryPlannerState::Following || m_state == TrajectoryPlannerState::Arrived) return;
			m_state = TrajectoryPlannerState::BackingOff;
			return;
		}

		/// the motion goes on from the current plan; from a hover it starts at rest
		MotionState start{pose.stamp, 0.0, 0.0, *m_yaw};
		std::vector<Eigen::Vector3d> raw;
		if (m_state == TrajectoryPlannerState::Following) {
			const MotionState now = current_state(pose);
			start.v = now.v;
			start.yaw = now.yaw;
			raw = path_to_join(now.s, waypoints.front());
		} else {
			raw.push_back(m_hold ? m_hold->position : pose.pose.position);
		}
		for (const Eigen::Vector3d& waypoint : waypoints) {
			if ((waypoint - raw.back()).norm() > kMinLength) raw.push_back(waypoint);
		}
		if (raw.size() < 2) raw.push_back(waypoints.back());

		set_path(raw);
		m_plan.clear();
		m_resume = start;
		m_state = TrajectoryPlannerState::Following;
		m_hold.reset();
		m_escape_target.reset();
		m_request_after = -std::numeric_limits<TimeSec>::infinity();
		m_route_checked = false;
	}

	/// The current path from arc length `s` up to the point nearest `target` (the start of a new
	/// route), if that point is ahead and within `join_tolerance`; otherwise only the point at `s`.
	std::vector<Eigen::Vector3d> path_to_join(double s, const Eigen::Vector3d& target) const {
		std::vector<Eigen::Vector3d> raw{position_at(s)};
		std::optional<std::size_t> join;
		double nearest = m_config.join_tolerance;
		for (std::size_t i = index_at(s) + 1; i < m_points.size() && m_s[i] <= s + m_config.check_distance; ++i) {
			const double distance = (m_points[i] - target).norm();
			if (distance <= nearest) {
				nearest = distance;
				join = i;
			}
		}
		if (!join) return raw;
		for (std::size_t i = index_at(s) + 1; i < *join; ++i) raw.push_back(m_points[i]);
		return raw;
	}

	/// Shortens, rounds and samples `raw` into the path, with the speed limit of every point from
	/// its curvature and slope.
	void set_path(const std::vector<Eigen::Vector3d>& raw) {
		const OccupancySpace<M> space = free_space(std::max(m_config.global_margin, m_config.margin));
		resample(rounded(space, shortcut(space, raw)));
		m_shared_path = std::make_shared<const std::vector<Eigen::Vector3d>>(m_points);
	}

	/// Skips the waypoints that a straight free line can skip.
	static std::vector<Eigen::Vector3d> shortcut(const OccupancySpace<M>& space, const std::vector<Eigen::Vector3d>& points) {
		std::vector<Eigen::Vector3d> result{points.front()};
		std::size_t i = 0;
		while (i + 1 < points.size()) {
			std::size_t j = points.size() - 1;
			while (j > i + 1 && !space.segment_free(points[i], points[j])) --j;
			result.push_back(points[j]);
			i = j;
		}
		return result;
	}

	/// Replaces every corner by a quadratic Bezier curve from `corner_radius` before it to
	/// `corner_radius` after it (less on short segments), where that curve is free.
	std::vector<Eigen::Vector3d> rounded(const OccupancySpace<M>& space, const std::vector<Eigen::Vector3d>& corners) const {
		std::vector<Eigen::Vector3d> curve{corners.front()};
		for (std::size_t i = 1; i + 1 < corners.size(); ++i) {
			const Eigen::Vector3d in = corners[i] - corners[i - 1];
			const Eigen::Vector3d out = corners[i + 1] - corners[i];
			const double cut = std::min({m_config.corner_radius, 0.5 * in.norm(), 0.5 * out.norm()});
			const Eigen::Vector3d a = corners[i] - in.normalized() * cut;
			const Eigen::Vector3d b = corners[i] + out.normalized() * cut;
			const int n = std::max(2, static_cast<int>(std::ceil(2.0 * cut / m_config.path_step)));
			std::vector<Eigen::Vector3d> arc;
			for (int k = 0; k <= n; ++k) {
				const double t = static_cast<double>(k) / n;
				arc.push_back((1.0 - t) * (1.0 - t) * a + 2.0 * (1.0 - t) * t * corners[i] + t * t * b);
			}
			bool free = space.segment_free(curve.back(), arc.front());
			for (std::size_t k = 0; free && k + 1 < arc.size(); ++k) free = space.segment_free(arc[k], arc[k + 1]);
			if (free) {
				curve.insert(curve.end(), arc.begin(), arc.end());
			} else {
				curve.push_back(corners[i]);
			}
		}
		curve.push_back(corners.back());
		return curve;
	}

	/// Samples `polyline` every `path_step` into m_points / m_s, and sets the speed limit of every
	/// point from the curvature and the slope of the path there.
	void resample(const std::vector<Eigen::Vector3d>& polyline) {
		m_points.clear();
		m_s.clear();
		double total = 0.0;
		double next = 0.0;
		for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
			const Eigen::Vector3d d = polyline[i + 1] - polyline[i];
			const double length = d.norm();
			if (length < 1e-9) continue;
			while (next <= total + length) {
				m_points.push_back(polyline[i] + d * ((next - total) / length));
				m_s.push_back(next);
				next += m_config.path_step;
			}
			total += length;
		}
		if (m_s.empty() || total - m_s.back() > 1e-6) {
			m_points.push_back(polyline.back());
			m_s.push_back(total);
		}

		m_path_limit.assign(m_points.size(), m_config.max_speed);
		for (std::size_t i = 0; i + 1 < m_points.size(); ++i) {
			const Eigen::Vector3d out = m_points[i + 1] - m_points[i];
			const double climb = std::abs(out.normalized().z());
			if (climb > 1e-6) m_path_limit[i] = std::min(m_path_limit[i], m_config.max_vertical_speed / climb);
			if (i == 0) continue;
			const Eigen::Vector3d in = m_points[i] - m_points[i - 1];
			const double cosine = std::clamp(in.normalized().dot(out.normalized()), -1.0, 1.0);
			const double curvature = std::acos(cosine) / (0.5 * (in.norm() + out.norm()));
			if (curvature > 1e-6) m_path_limit[i] = std::min(m_path_limit[i], std::sqrt(m_config.max_lateral_acceleration / curvature));
		}
	}

	/// Following: continues the plan (or restarts it from the real drone), checks the arrival and the
	/// path ahead, and plans the next `horizon`.
	LocalStep<ReplanRequest> follow(const StampedPose& pose) {
		MotionState state = current_state(pose);
		const Eigen::Vector3d& position = pose.pose.position;

		/// the real drone is somewhere else, or faster or slower - plan from where it really is
		const double offset = (position_at(state.s) - position).norm();
		m_report.tracking_error = offset;
		const double along = m_velocity ? std::max(0.0, m_velocity->dot(tangent_at(state.s))) : state.v;
		if (offset > m_config.resync_distance || std::abs(along - state.v) > m_config.resync_speed) {
			state.s = closest_s(position, state.s - m_config.projection_window, state.s + m_config.projection_window);
			if ((position_at(state.s) - position).norm() > m_config.deviation_distance) return stop(pose, ReplanReason::Deviated);
			if (m_velocity) state.v = std::min(std::max(0.0, m_velocity->dot(tangent_at(state.s))), m_config.max_speed);
			state.yaw = yaw_of(pose.pose.orientation);
			m_report.resynced = true;
		}

		const bool settled = actual_speed() <= m_config.settled_speed;
		const bool at_end = m_s.back() - state.s <= kStopSnap && state.v <= kRestSpeed;
		if (at_end && settled && (position - m_points.back()).norm() <= m_config.goal_tolerance) {
			m_state = TrajectoryPlannerState::Arrived;
			m_plan.clear();
			m_hold = Setpoint{.position = m_points.back(), .yaw = state.yaw};
			return {std::nullopt, hover_trajectory(pose.stamp, *m_hold)};
		}

		const Blockage blockage = find_blockage(state.s);
		double stop_s = m_s.back();
		double hard_s = m_s.back();
		if (blockage.margin_s) {
			stop_s = std::max(state.s, *blockage.margin_s - m_config.stop_buffer);
			if (blockage.hit_s) hard_s = std::max(stop_s, *blockage.hit_s - m_config.emergency_clearance);
		}
		/// too close to stop at stop_s with max_acceleration - brake harder, at most up to hard_s
		const bool emergency = blockage.margin_s && braking_distance(state.v) > stop_s - state.s + kStopSnap;
		const double end_s = emergency ? hard_s : stop_s;
		m_report.braking_distance = braking_distance(state.v);
		if (blockage.margin_s) {
			m_report.blocked = true;
			m_report.margin_distance = *blockage.margin_s - state.s;
			m_report.emergency = emergency;
			m_report.stop_point = position_at(end_s);
		}
		if (blockage.hit_s) m_report.obstacle_distance = *blockage.hit_s - state.s;

		plan_motion(pose.stamp, state, stop_s, end_s, emergency);
		Trajectory trajectory = to_trajectory(pose.stamp);
		m_yaw = trajectory.points.front().yaw;

		const bool fresh_route = !m_route_checked && state.s < kFreshRoute;
		m_route_checked = true;
		if (!blockage.margin_s) return {std::nullopt, std::move(trajectory)};

		m_reason = ReplanReason::Blocked;
		if (fresh_route && m_plan.back().v <= kRestSpeed) m_request_after = std::max(m_request_after, pose.stamp + m_config.replan_backoff);

		const bool stopped = m_plan.front().v <= kRestSpeed && m_plan.back().v <= 0.0 && m_plan.back().s - m_plan.front().s <= kStopSnap;
		if (stopped && settled) {
			m_plan.clear();
			if (!start_free(position)) {
				m_state = TrajectoryPlannerState::Escaping;
				return escape(pose);
			}
			m_state = TrajectoryPlannerState::Waiting;
			return ask_for_route(pose);
		}
		std::optional<ReplanRequest> request;
		if (pose.stamp >= m_request_after) {
			if (const auto from = request_point()) request = make_request(pose, *from);
		}
		return {std::move(request), std::move(trajectory)};
	}

	/// The next `horizon` of the plan from `state`, into m_plan. The speed follows the limits of the
	/// path (0 from `stop_s` on), changes by at most `max_acceleration` (braking by up to
	/// `emergency_deceleration` in an emergency), and stays at `blind_speed` while the yaw has not
	/// turned to where the drone goes. The plan stops at `end_s`.
	void plan_motion(TimeSec stamp, MotionState state, double stop_s, double end_s, bool emergency) {
		const std::vector<double> limit = speed_limits(stop_s, state.s);
		const double speed_up = m_config.max_acceleration * m_config.step;
		const double slow_down = (emergency ? std::max(m_config.emergency_deceleration, m_config.max_acceleration) : m_config.max_acceleration) * m_config.step;
		const double yaw_step = m_config.max_yaw_rate * m_config.step;
		m_plan.clear();
		for (std::size_t k = 0; k < point_count(); ++k) {
			state.t = stamp + static_cast<double>(k) * m_config.step;
			m_plan.push_back(state);
			const double heading = heading_at(std::min(state.s + m_config.yaw_lookahead, m_s.back()), state.yaw);
			const double ahead = std::min(state.s + state.v * m_config.step, m_s.back());
			const double to_stop = std::sqrt(2.0 * m_config.max_acceleration * std::max(0.0, stop_s - ahead));
			double target = std::min(limit_at(limit, ahead), to_stop);
			if (std::abs(wrap_angle(heading - state.yaw)) > m_config.camera_half_fov) target = std::min(target, m_config.blind_speed);
			const double lowest = std::max(0.0, state.v - slow_down);
			const double highest = std::max(lowest, std::min(state.v + speed_up, m_config.max_speed));
			const double v_next = std::clamp(target, lowest, highest);
			state.s = state.s + 0.5 * (state.v + v_next) * m_config.step;
			if (state.s >= end_s - kStopSnap) {
				state.s = end_s;
				state.v = 0.0;
			} else {
				state.v = v_next;
			}
			state.yaw = turn_towards(state.yaw, heading, yaw_step);
		}
	}

	/// Where a Blocked request asks a new path from: the point of the plan `replan_lead` ahead, or the
	/// latest one before it that a global planner can start at; nullopt if there is none (the drone
	/// asks once it has stopped).
	std::optional<Eigen::Vector3d> request_point() const {
		const std::size_t lead = std::min(m_plan.size() - 1, static_cast<std::size_t>(std::lround(m_config.replan_lead / m_config.step)));
		for (std::size_t i = lead + 1; i-- > 0;) {
			const Eigen::Vector3d point = position_at(m_plan[i].s);
			if (start_free(point)) return point;
		}
		return std::nullopt;
	}

	/// The route cannot be followed any more: hover and ask for a new one.
	LocalStep<ReplanRequest> stop(const StampedPose& pose, ReplanReason reason) {
		m_state = TrajectoryPlannerState::Waiting;
		m_reason = reason;
		m_plan.clear();
		return ask_for_route(pose);
	}

	/// Waiting: hovers and asks every cycle - the thread drops the request while an earlier one is
	/// unanswered - unless an answer was useless less than `replan_backoff` ago. A global planner
	/// needs `global_margin` around its start, so a drone closer to an obstacle escapes first.
	LocalStep<ReplanRequest> ask_for_route(const StampedPose& pose) {
		if (!start_free(pose.pose.position)) {
			m_state = TrajectoryPlannerState::Escaping;
			return escape(pose);
		}
		hold_here(pose);
		std::optional<ReplanRequest> request;
		if (pose.stamp >= m_request_after) request = make_request(pose, pose.pose.position);
		return {std::move(request), hover_trajectory(pose.stamp, *m_hold)};
	}

	/// BackingOff: hovers without asking until `replan_backoff` of sim time has passed since the
	/// Failed answer.
	LocalStep<ReplanRequest> back_off(const StampedPose& pose) {
		if (pose.stamp >= m_request_after) {
			m_state = TrajectoryPlannerState::Waiting;
			return ask_for_route(pose);
		}
		hold_here(pose);
		return {std::nullopt, hover_trajectory(pose.stamp, *m_hold)};
	}

	/// Escaping: once the drone is at rest, flies slowly to the nearest point with `global_margin`
	/// around it, and asks for a route there. A target the drone has reached while it is still not
	/// free (the target lies on the edge of the free space) is replaced by the next one. If there is
	/// no free point nearby it hovers and asks anyway; a Failed answer starts the backoff.
	LocalStep<ReplanRequest> escape(const StampedPose& pose) {
		const Eigen::Vector3d& position = pose.pose.position;
		if (start_free(position)) {
			m_state = TrajectoryPlannerState::Waiting;
			m_escape_target.reset();
			m_hold.reset();
			return ask_for_route(pose);
		}
		if (actual_speed() > m_config.settled_speed && !m_escape_target) {
			hold_here(pose);
			return {std::nullopt, hover_trajectory(pose.stamp, *m_hold)};
		}
		const bool reached = m_escape_target && (*m_escape_target - position).norm() < 0.5 * m_config.escape_step;
		if (!m_escape_target || reached || !escape_free(position, *m_escape_target)) m_escape_target = find_escape_point(position);
		if (!m_escape_target) {
			hold_here(pose);
			std::optional<ReplanRequest> request;
			if (pose.stamp >= m_request_after) request = make_request(pose, position);
			return {std::move(request), hover_trajectory(pose.stamp, *m_hold)};
		}
		m_hold.reset();
		return {std::nullopt, escape_trajectory(pose, *m_escape_target)};
	}

	/// The planned state at the time of `pose`: interpolated in the previous plan, or the start state
	/// of a new route.
	MotionState current_state(const StampedPose& pose) const {
		if (m_plan.empty()) return {pose.stamp, m_resume.s, m_resume.v, m_resume.yaw};
		if (pose.stamp <= m_plan.front().t) return m_plan.front();
		const auto after = std::upper_bound(m_plan.begin(), m_plan.end(), pose.stamp,
		                                    [](TimeSec t, const MotionState& sample) { return t < sample.t; });
		if (after == m_plan.end()) return m_plan.back();
		const MotionState& a = *std::prev(after);
		const MotionState& b = *after;
		const double f = (pose.stamp - a.t) / (b.t - a.t);
		return {pose.stamp, a.s + f * (b.s - a.s), a.v + f * (b.v - a.v), wrap_angle(a.yaw + f * wrap_angle(b.yaw - a.yaw))};
	}

	/// The first points ahead of `s` (up to `check_distance`) inside the margin of an obstacle, and in
	/// an occupied voxel.
	Blockage find_blockage(double s) const {
		const OccupancySpace<M> margin = free_space(m_config.margin);
		const OccupancySpace<M> touching = free_space(0.0);
		Blockage blockage;
		for (std::size_t i = index_at(s); i < m_points.size() && m_s[i] <= s + m_config.check_distance; ++i) {
			if (!blockage.margin_s && !margin.is_free(m_points[i])) blockage.margin_s = std::max(s, m_s[i]);
			if (blockage.margin_s && !touching.is_free(m_points[i])) {
				blockage.hit_s = std::max(s, m_s[i]);
				break;
			}
		}
		return blockage;
	}

	/// The highest speed at every point of the path: its curvature and slope limit, `caution_speed`
	/// near obstacles (checked from `from` up to `check_distance`), 0 from `stop_s` on, and low enough
	/// to brake with `max_acceleration` for every point ahead.
	std::vector<double> speed_limits(double stop_s, double from) const {
		std::vector<double> cap = m_path_limit;
		const OccupancySpace<M> caution = free_space(m_config.caution_margin);
		for (std::size_t i = index_at(from); i < m_points.size() && m_s[i] <= from + m_config.check_distance; ++i) {
			if (cap[i] > m_config.caution_speed && !caution.is_free(m_points[i])) cap[i] = m_config.caution_speed;
		}
		std::vector<double> limit(m_points.size());
		double next = 0.0;
		for (std::size_t i = m_points.size(); i-- > 0;) {
			const double ds = i + 1 < m_points.size() ? m_s[i + 1] - m_s[i] : 0.0;
			const double braking = std::sqrt(next * next + 2.0 * m_config.max_acceleration * ds);
			limit[i] = m_s[i] >= stop_s ? 0.0 : std::min(cap[i], braking);
			next = limit[i];
		}
		return limit;
	}

	/// The trajectory of m_plan, with the acceleration between its points as feed-forward.
	Trajectory to_trajectory(TimeSec stamp) const {
		Trajectory trajectory;
		trajectory.stamp = stamp;
		for (const MotionState& sample : m_plan) {
			trajectory.points.push_back(Setpoint{
			    .stamp = sample.t,
			    .position = position_at(sample.s),
			    .velocity = tangent_at(sample.s) * sample.v,
			    .yaw = sample.yaw,
			});
		}
		for (std::size_t k = 0; k + 1 < trajectory.points.size(); ++k) {
			trajectory.points[k].acceleration = (trajectory.points[k + 1].velocity - trajectory.points[k].velocity) / m_config.step;
		}
		return trajectory;
	}

	/// Straight from the drone to `target` at `escape_speed`, stopping there.
	Trajectory escape_trajectory(const StampedPose& pose, const Eigen::Vector3d& target) const {
		const Eigen::Vector3d offset = target - pose.pose.position;
		const double distance = offset.norm();
		const Eigen::Vector3d direction = distance > kMinLength ? Eigen::Vector3d(offset / distance) : Eigen::Vector3d::Zero();

		Trajectory trajectory;
		trajectory.stamp = pose.stamp;
		for (std::size_t k = 0; k < point_count(); ++k) {
			const double t = static_cast<double>(k) * m_config.step;
			const double travelled = std::min(m_config.escape_speed * t, distance);
			Setpoint point{.stamp = pose.stamp + t, .position = pose.pose.position + direction * travelled, .yaw = *m_yaw};
			if (travelled < distance) point.velocity = direction * m_config.escape_speed;
			trajectory.points.push_back(point);
		}
		return trajectory;
	}

	/// `hold` for the whole horizon, with zero velocity.
	Trajectory hover_trajectory(TimeSec stamp, const Setpoint& hold) const {
		Trajectory trajectory;
		trajectory.stamp = stamp;
		for (std::size_t k = 0; k < point_count(); ++k) {
			Setpoint point = hold;
			point.stamp = stamp + static_cast<double>(k) * m_config.step;
			point.velocity = Eigen::Vector3d::Zero();
			point.acceleration = Eigen::Vector3d::Zero();
			trajectory.points.push_back(point);
		}
		return trajectory;
	}

	/// Fixes the hover point at the drone's position, once: it is kept until the drone moves on, so
	/// the drone does not slide along with a drifting pose (as in CommanderThread).
	void hold_here(const StampedPose& pose) {
		if (!m_hold) m_hold = Setpoint{.position = pose.pose.position, .yaw = *m_yaw};
	}

	std::size_t index_at(double s) const {
		const auto after = std::upper_bound(m_s.begin(), m_s.end(), s);
		const std::size_t i = after == m_s.begin() ? 0 : static_cast<std::size_t>(after - m_s.begin()) - 1;
		return std::min(i, m_s.size() - 2);
	}

	double fraction_at(std::size_t i, double s) const {
		return std::clamp((s - m_s[i]) / (m_s[i + 1] - m_s[i]), 0.0, 1.0);
	}

	Eigen::Vector3d position_at(double s) const {
		const std::size_t i = index_at(s);
		return m_points[i] + fraction_at(i, s) * (m_points[i + 1] - m_points[i]);
	}

	Eigen::Vector3d tangent_at(double s) const {
		const std::size_t i = index_at(s);
		return (m_points[i + 1] - m_points[i]).normalized();
	}

	/// Heading of the path at `s`; `fallback` where the path is (nearly) vertical.
	double heading_at(double s, double fallback) const {
		const Eigen::Vector3d t = tangent_at(s);
		return t.head<2>().norm() < 0.3 ? fallback : std::atan2(t.y(), t.x());
	}

	double limit_at(const std::vector<double>& limit, double s) const {
		const std::size_t i = index_at(s);
		const double f = fraction_at(i, s);
		return limit[i] + f * (limit[i + 1] - limit[i]);
	}

	/// Arc length of the point of the path nearest `p`, searched between `from` and `to`.
	double closest_s(const Eigen::Vector3d& p, double from, double to) const {
		double best_s = std::clamp(from, 0.0, m_s.back());
		double best = std::numeric_limits<double>::infinity();
		for (std::size_t i = 0; i < m_points.size(); ++i) {
			if (m_s[i] < from || m_s[i] > to) continue;
			const double d = (m_points[i] - p).squaredNorm();
			if (d < best) {
				best = d;
				best_s = m_s[i];
			}
		}
		return best_s;
	}

	double braking_distance(double speed) const { return speed * speed / (2.0 * m_config.max_acceleration); }

	static double turn_towards(double yaw, double target, double max_step) {
		return wrap_angle(yaw + std::clamp(wrap_angle(target - yaw), -max_step, max_step));
	}

	/// The map as a space with `margin` around every point; unbounded (the global planner keeps its
	/// paths within its altitude limits).
	OccupancySpace<M> free_space(double margin) const {
		const Eigen::AlignedBox3d everywhere(Eigen::Vector3d::Constant(-1e6), Eigen::Vector3d::Constant(1e6));
		return OccupancySpace<M>(this->m_map.map(), everywhere, margin, m_config.unknown_is_free);
	}

	/// Whether a global planner can start a path at `position`: `global_margin` of free space around it.
	bool start_free(const Eigen::Vector3d& position) const {
		return free_space(std::max(m_config.global_margin, m_config.margin)).is_free(position);
	}

	/// Whether the drone can escape from `from` to `to`: a route can start at `to`, and the straight
	/// line to it does not touch an obstacle (checked without a margin, which the drone is inside of).
	bool escape_free(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const {
		return start_free(to) && free_space(0.0).segment_free(from, to);
	}

	/// The nearest point around `position`, on a grid `escape_step` apart and within `escape_radius`,
	/// that escape_free() allows; nullopt if there is none.
	std::optional<Eigen::Vector3d> find_escape_point(const Eigen::Vector3d& position) const {
		for (const Eigen::Vector3d& offset : m_escape_offsets) {
			if (escape_free(position, position + offset)) return position + offset;
		}
		return std::nullopt;
	}

	ReplanRequest make_request(const StampedPose& pose, const Eigen::Vector3d& position) const {
		ReplanRequest request;
		request.stamp = pose.stamp;
		request.reason = m_reason;
		request.position = position;
		return request;
	}

	/// Points of a trajectory: one every `step`, over the whole `horizon`.
	std::size_t point_count() const {
		return std::max<std::size_t>(2, static_cast<std::size_t>(std::llround(m_config.horizon / m_config.step)) + 1);
	}

	TrajectoryLocalPlannerConfig m_config;
	Topic<TrajectoryPlannerReport>* m_reports;  ///< null: no reports
	TrajectoryPlannerReport m_report;          ///< of the current cycle, filled in during plan()
	std::shared_ptr<const std::vector<Eigen::Vector3d>> m_shared_path;  ///< m_points, for the reports
	std::vector<Eigen::Vector3d> m_escape_offsets;  ///< points around the drone escape() tries, nearest first

	TrajectoryPlannerState m_state = TrajectoryPlannerState::Waiting;
	ReplanReason m_reason = ReplanReason::NoPath;  ///< reason of the requests while waiting or blocked
	/// no request before this (after a Failed answer, or one that was blocked at once)
	TimeSec m_request_after = -std::numeric_limits<TimeSec>::infinity();
	bool m_route_checked = false;  ///< the current route was found blocked or free at least once

	/// The smoothed path: points `path_step` apart, their arc lengths and speed limits.
	std::vector<Eigen::Vector3d> m_points;
	std::vector<double> m_s;
	std::vector<double> m_path_limit;
	std::vector<MotionState> m_plan;  ///< the last plan, continued in the next cycle
	MotionState m_resume;             ///< where a new route starts, until the first plan on it

	std::deque<std::pair<TimeSec, Eigen::Vector3d>> m_poses;  ///< recent poses, oldest first
	std::optional<Eigen::Vector3d> m_velocity;                ///< measured from m_poses

	std::optional<Setpoint> m_hold;                  ///< hover point while not moving
	std::optional<double> m_yaw;                     ///< yaw of the last trajectory (the drone's yaw at first)
	std::optional<Eigen::Vector3d> m_escape_target;  ///< where escape() flies to
};

}
