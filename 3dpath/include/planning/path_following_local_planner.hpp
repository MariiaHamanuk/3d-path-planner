#pragma once

#include "mapping/map_view.hpp"
#include "pipeline/clients.hpp"
#include "planning/algorithms/map_adapters.hpp"
#include "planning/local_planner.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <optional>
#include <utility>
#include <vector>

namespace path3d {

/**
 * @brief Settings of the PathFollowingLocalPlanner. Times are sim time.
 */
struct PathFollowerConfig {
	double cruise_speed = 2.0;      ///< top speed along the path, m/s
	double max_acceleration = 1.5;  ///< for speeding up and for braking (before corners and at the goal), m/s^2
	/// Speed at a turn of `sharp_turn_angle` or more, m/s. Smaller turns are flown faster: the corner
	/// speed goes linearly from `cruise_speed` (straight on) down to this.
	double min_corner_speed = 0.5;
	double sharp_turn_angle = std::numbers::pi / 2.0;  ///< rad
	TimeSec horizon = 2.0;                             ///< how far ahead each trajectory reaches, s
	TimeSec step = 0.1;                                ///< time between the points of a trajectory, s
	/// What counts as free in the collision check (margin, unknown space). Give the global planner the
	/// same one: with a larger margin here, every global path looks blocked.
	SearchSpaceConfig space;
	double deviation_distance = 2.5;  ///< the drone this far from the path asks for a new one (Deviated), m
	double goal_tolerance = 0.4;      ///< the drone this close to the end of the path has arrived, m
	/// How far along the path the projection of the drone may move forward in one cycle, m. Keeps the
	/// progress from jumping to a later part of the path that passes near the drone.
	double projection_window = 1.5;
	TimeSec replan_backoff = 1.0;  ///< wait after a Failed answer before asking again, s
	double escape_speed = 0.5;     ///< speed out of the margin of an obstacle, m/s
	double escape_radius = 2.0;    ///< how far around the drone to look for a free point to escape to, m
	double escape_step = 0.25;     ///< spacing of the points tried, m
};

/**
 * @brief What the PathFollowingLocalPlanner is doing.
 */
enum class PathFollowerState {
	Waiting,     ///< hovers and asks for a route every cycle
	Following,   ///< follows the route
	BackingOff,  ///< hovers after a Failed answer until `replan_backoff` has passed
	Escaping,    ///< moves out of the margin of an obstacle, so that a new route can start here
	Arrived,     ///< hovers at the end of the route
};

/**
 * @brief Local planner that follows the polyline of a GlobalPath with a speed profile.
 *
 * Every cycle it projects the drone onto the path (the progress only moves forward) and plans the
 * next `horizon` seconds from there: points on the path, `step` apart, at a speed limited by the
 * cruise speed, by the acceleration from the current speed, by braking for the corners ahead and
 * by braking to zero at the goal. A drone that drifted off the path is pulled back onto it.
 *
 * Every trajectory is checked against the map. When it is blocked or the drone is too far from the
 * path, the drone hovers and asks for a new route. While it has no route it hovers and asks every
 * cycle, because the thread sends only one request at a time. After a Failed answer it waits
 * `replan_backoff` before asking again. A global planner cannot start a path inside the margin of an
 * obstacle (a_star() needs a free start), so in that case the drone first moves slowly to the
 * nearest free point, and asks from there.
 *
 * It always returns a trajectory: a hover trajectory instead of none, because the commander keeps
 * flying the old one for up to `max_trajectory_age`.
 *
 * @tparam M Map type; any OccupancyMap that can be replicated (for example LogOddsGridMap).
 */
template <class M>
    requires ReplicatedMap<M> && OccupancyMap<M>
class PathFollowingLocalPlanner : public LocalPlanner<M, GlobalPath, ReplanRequest> {
public:
	/**
	 * @param map The planner's copy of the map.
	 * @param uav Access to the drone pose.
	 * @param config Speeds, horizon, safety settings (see PathFollowerConfig).
	 */
	PathFollowingLocalPlanner(MapClient<M>& map, UavClient& uav, PathFollowerConfig config = {})
	    : LocalPlanner<M, GlobalPath, ReplanRequest>(map, uav), m_config(config) {
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
		if (new_route) take_answer(*new_route, pose);

		LocalStep<ReplanRequest> step;
		switch (m_state) {
			case PathFollowerState::Waiting:
				step = ask_for_route(pose);
				break;
			case PathFollowerState::Following:
				step = follow(pose);
				break;
			case PathFollowerState::BackingOff:
				step = back_off(pose);
				break;
			case PathFollowerState::Escaping:
				step = escape(pose);
				break;
			case PathFollowerState::Arrived:
				step.trajectory = hover_trajectory(pose.stamp, *m_hold);
				break;
		}
		m_last_trajectory = step.trajectory;
		return step;
	}

	/// What the planner is doing. Changes only in plan() so do not read it while a thread runs the planner
	PathFollowerState state() const { return m_state; }

private:
	/// Shorter segments and directions count as zero, m.
	static constexpr double kMinLength = 1e-6;

	/// Where the drone is on the path.
	struct Projection {
		double progress;      ///< arc length of the nearest point of the path, m
		std::size_t segment;  ///< the segment that point is on
		double distance;      ///< from the drone to that point, m
	};

	/// A point of the path.
	struct PathPoint {
		Eigen::Vector3d position;
		Eigen::Vector3d direction;  ///< unit direction of the segment the point is on
	};

	/// An answer of the global planner: a usable route is followed. Anything else (Failed, fewer than
	/// two distinct waypoints) starts the backoff, unless the drone is following a route or has arrived.
	void take_answer(const GlobalPath& route, const StampedPose& pose) {
		if (route.status == PlanStatus::Ok && store_route(route)) {
			m_state = PathFollowerState::Following;
			m_hold.reset();
			m_escape_target.reset();
			return;
		}
		if (m_state == PathFollowerState::Following || m_state == PathFollowerState::Arrived) return;
		m_state = PathFollowerState::BackingOff;
		m_failed_stamp = pose.stamp;
	}

	/// Keeps a copy of the waypoints of `route` (repeated points dropped) with the arc length and the
	/// corner speed at each of them, and starts at its beginning. False, and nothing changes, if fewer
	/// than two waypoints remain.
	bool store_route(const GlobalPath& route) {
		std::vector<Eigen::Vector3d> waypoints;
		for (const Eigen::Vector3d& waypoint : route.waypoints) {
			if (waypoints.empty() || (waypoint - waypoints.back()).norm() > kMinLength) waypoints.push_back(waypoint);
		}
		if (waypoints.size() < 2) return false;

		m_waypoints = std::move(waypoints);
		m_arc_lengths.assign(1, 0.0);
		for (std::size_t i = 1; i < m_waypoints.size(); ++i) m_arc_lengths.push_back(m_arc_lengths.back() + (m_waypoints[i] - m_waypoints[i - 1]).norm());
		m_corner_speeds.assign(m_waypoints.size(), m_config.cruise_speed);
		for (std::size_t i = 1; i + 1 < m_waypoints.size(); ++i) {
			m_corner_speeds[i] = corner_speed(m_waypoints[i] - m_waypoints[i - 1], m_waypoints[i + 1] - m_waypoints[i]);
		}
		m_corner_speeds.back() = 0.0;
		m_segment = 0;
		m_progress = 0.0;
		return true;
	}

	/// Speed for a turn from direction `in` to direction `out`: the cruise speed straight on, down to
	/// `min_corner_speed` at `sharp_turn_angle` and more.
	double corner_speed(const Eigen::Vector3d& in, const Eigen::Vector3d& out) const {
		const double angle = std::acos(std::clamp(in.normalized().dot(out.normalized()), -1.0, 1.0));
		const double share = std::min(1.0, angle / m_config.sharp_turn_angle);
		const double slowest = std::min(m_config.min_corner_speed, m_config.cruise_speed);
		return m_config.cruise_speed - (m_config.cruise_speed - slowest) * share;
	}

	/// Following: updates the progress, then checks the deviation, the arrival and the trajectory.
	LocalStep<ReplanRequest> follow(const StampedPose& pose) {
		const Projection projection = project(pose.pose.position);
		m_progress = projection.progress;
		m_segment = projection.segment;
		if (projection.distance > m_config.deviation_distance) return stop(pose, ReplanReason::Deviated);

		const Eigen::Vector3d& goal = m_waypoints.back();
		if (m_arc_lengths.back() - m_progress <= m_config.goal_tolerance && (pose.pose.position - goal).norm() <= m_config.goal_tolerance) {
			m_state = PathFollowerState::Arrived;
			m_hold = Setpoint{.position = goal, .yaw = *m_yaw};
			return {std::nullopt, hover_trajectory(pose.stamp, *m_hold)};
		}

		Trajectory trajectory = reference_trajectory(pose);
		if (!trajectory_free(trajectory)) return stop(pose, ReplanReason::Blocked);
		m_yaw = trajectory.points.front().yaw;
		return {std::nullopt, std::move(trajectory)};
	}

	/// The route cannot be followed any more: hover and ask for a new one.
	LocalStep<ReplanRequest> stop(const StampedPose& pose, ReplanReason reason) {
		m_state = PathFollowerState::Waiting;
		m_reason = reason;
		return ask_for_route(pose);
	}

	/// Waiting: hovers and asks every cycle - the thread drops the request while an earlier one is
	/// unanswered. Inside the margin of an obstacle the global planner cannot start a path, so the
	/// drone escapes first.
	LocalStep<ReplanRequest> ask_for_route(const StampedPose& pose) {
		if (!position_free(pose.pose.position)) {
			m_state = PathFollowerState::Escaping;
			return escape(pose);
		}
		hold_here(pose);
		return {make_request(pose), hover_trajectory(pose.stamp, *m_hold)};
	}

	/// BackingOff: hovers without asking until `replan_backoff` of sim time has passed since the
	/// Failed answer.
	LocalStep<ReplanRequest> back_off(const StampedPose& pose) {
		if (pose.stamp - m_failed_stamp >= m_config.replan_backoff) {
			m_state = PathFollowerState::Waiting;
			return ask_for_route(pose);
		}
		hold_here(pose);
		return {std::nullopt, hover_trajectory(pose.stamp, *m_hold)};
	}

	/// Escaping: flies slowly to the nearest free point, and asks for a route once the drone is free.
	/// A target the drone has reached while it is still not free (the target lies on the edge of the
	/// free space) is replaced by the next one. If there is no free point nearby it hovers and asks
	/// anyway; a Failed answer starts the backoff.
	LocalStep<ReplanRequest> escape(const StampedPose& pose) {
		const Eigen::Vector3d& position = pose.pose.position;
		if (position_free(position)) {
			m_state = PathFollowerState::Waiting;
			m_escape_target.reset();
			m_hold.reset();
			return ask_for_route(pose);
		}
		const bool reached = m_escape_target && (*m_escape_target - position).norm() < 0.5 * m_config.escape_step;
		if (!m_escape_target || reached || !escape_free(position, *m_escape_target)) m_escape_target = find_escape_point(position);
		if (!m_escape_target) {
			hold_here(pose);
			return {make_request(pose), hover_trajectory(pose.stamp, *m_hold)};
		}
		m_hold.reset();
		return {std::nullopt, escape_trajectory(pose, *m_escape_target)};
	}

	/// The nearest point of the path to `position`, searched from the current progress forward, at
	/// most `projection_window` ahead. It is never behind the current progress.
	Projection project(const Eigen::Vector3d& position) const {
		Projection best{m_progress, m_segment, std::numeric_limits<double>::infinity()};
		const double farthest = m_progress + m_config.projection_window;
		for (std::size_t i = m_segment; i + 1 < m_waypoints.size() && m_arc_lengths[i] <= farthest; ++i) {
			const double length = m_arc_lengths[i + 1] - m_arc_lengths[i];
			const Eigen::Vector3d direction = (m_waypoints[i + 1] - m_waypoints[i]) / length;
			const double lowest = std::clamp(m_progress - m_arc_lengths[i], 0.0, length);
			const double highest = std::max(lowest, std::min(length, farthest - m_arc_lengths[i]));
			const double along = std::clamp((position - m_waypoints[i]).dot(direction), lowest, highest);
			const double distance = (m_waypoints[i] + direction * along - position).norm();
			if (distance < best.distance) best = {m_arc_lengths[i] + along, i, distance};
		}
		return best;
	}

	/// The point of the path at arc length `s` (not behind the current segment; clamped to the goal).
	PathPoint point_at(double s) const {
		std::size_t i = m_segment;
		while (i + 2 < m_waypoints.size() && s > m_arc_lengths[i + 1]) ++i;
		const Eigen::Vector3d direction = (m_waypoints[i + 1] - m_waypoints[i]).normalized();
		const double along = std::clamp(s - m_arc_lengths[i], 0.0, m_arc_lengths[i + 1] - m_arc_lengths[i]);
		return {m_waypoints[i] + direction * along, direction};
	}

	/// The highest speed at arc length `s`: the cruise speed, and low enough to brake with
	/// `max_acceleration` to the corner speed of every waypoint ahead (0 at the goal).
	double speed_limit(double s) const {
		double limit = m_config.cruise_speed;
		for (std::size_t i = m_segment + 1; i < m_waypoints.size(); ++i) {
			const double distance = m_arc_lengths[i] - s;
			if (distance < 0.0) continue;
			const double braking = 2.0 * m_config.max_acceleration * distance;
			/// this waypoint cannot lower the limit and neither can any later, which is farther
			if (braking >= limit * limit) break;
			limit = std::min(limit, std::sqrt(m_corner_speeds[i] * m_corner_speeds[i] + braking));
		}
		return limit;
	}

	/// The speed the drone should have now: the previous trajectory sampled at `stamp`; 0 if there is
	/// none or it does not cover `stamp`.
	double current_speed(TimeSec stamp) const {
		if (!m_last_trajectory) return 0.0;
		const std::optional<Setpoint> now = m_last_trajectory->sample(stamp);
		return now ? now->velocity.norm() : 0.0;
	}

	/// The next `horizon` along the path from the current progress. The speed starts at
	/// current_speed(), changes by at most `max_acceleration` and follows speed_limit() one step
	/// ahead, so the drone brakes in time for the corners and stops at the goal. The yaw is the
	/// heading of the path; on a vertical segment the previous one is kept.
	Trajectory reference_trajectory(const StampedPose& pose) const {
		const double length = m_arc_lengths.back();
		const double change = m_config.max_acceleration * m_config.step;
		double s = m_progress;
		double speed = std::min(current_speed(pose.stamp), m_config.cruise_speed);
		double yaw = *m_yaw;

		Trajectory trajectory;
		trajectory.stamp = pose.stamp;
		for (std::size_t k = 0; k < point_count(); ++k) {
			if (k > 0) {
				const double target = speed_limit(std::min(s + speed * m_config.step, length));
				const double next = std::clamp(target, std::max(0.0, speed - change), speed + change);
				s = std::min(s + 0.5 * (speed + next) * m_config.step, length);
				speed = next;
			}
			if (s >= length) speed = 0.0;
			const PathPoint point = point_at(s);
			if (std::hypot(point.direction.x(), point.direction.y()) > kMinLength) yaw = wrap_angle(std::atan2(point.direction.y(), point.direction.x()));
			trajectory.points.push_back({.stamp = pose.stamp + static_cast<double>(k) * m_config.step,
			                             .position = point.position,
			                             .velocity = point.direction * speed,
			                             .yaw = yaw});
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

	/// Whether the drone can fly from point to point of `trajectory` with `space.margin` around it.
	bool trajectory_free(const Trajectory& trajectory) const {
		Eigen::AlignedBox3d box(trajectory.points.front().position);
		for (const Setpoint& point : trajectory.points) box.extend(point.position);
		const OccupancySpace<M> space(this->m_map.map(), check_box(box.min(), box.max()), m_config.space);
		for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
			if (!space.segment_free(trajectory.points[i - 1].position, trajectory.points[i].position)) return false;
		}
		return true;
	}

	/// Whether `position` has `space.margin` of free space around it, as a_star() requires of its start.
	bool position_free(const Eigen::Vector3d& position) const {
		const OccupancySpace<M> space(this->m_map.map(), check_box(position, position), m_config.space);
		return space.is_free(position);
	}

	/// Whether the drone can escape from `from` to `to`: `to` is free, and the straight line to it
	/// does not touch an obstacle (checked without the margin, which the drone is inside of).
	bool escape_free(const Eigen::Vector3d& from, const Eigen::Vector3d& to) const {
		SearchSpaceConfig touching = m_config.space;
		touching.margin = 0.0;
		const OccupancySpace<M> line(this->m_map.map(), check_box(from.cwiseMin(to), from.cwiseMax(to)), touching);
		return position_free(to) && line.segment_free(from, to);
	}

	/// The nearest point around `position`, on a grid `escape_step` apart and within `escape_radius`,
	/// that escape_free() allows; nullopt if there is none.
	std::optional<Eigen::Vector3d> find_escape_point(const Eigen::Vector3d& position) const {
		for (const Eigen::Vector3d& offset : m_escape_offsets) {
			if (escape_free(position, position + offset)) return position + offset;
		}
		return std::nullopt;
	}

	/// The box a check may use: from `lowest` to `highest`, grown by the margin and a meter. Points
	/// outside it count as not free. There are no altitude limits: the global planner keeps its paths
	/// within them, except for a start or goal outside them (see search_bounds()), which must not
	/// look blocked here.
	Eigen::AlignedBox3d check_box(const Eigen::Vector3d& lowest, const Eigen::Vector3d& highest) const {
		constexpr double kUnlimited = std::numeric_limits<double>::infinity();
		return search_bounds(lowest, highest, m_config.space.margin + 1.0, -kUnlimited, kUnlimited);
	}

	ReplanRequest make_request(const StampedPose& pose) const {
		ReplanRequest request;
		request.stamp = pose.stamp;
		request.reason = m_reason;
		request.position = pose.pose.position;
		return request;
	}

	/// Points of a trajectory: one every `step`, over the whole `horizon`.
	std::size_t point_count() const {
		return std::max<std::size_t>(2, static_cast<std::size_t>(std::llround(m_config.horizon / m_config.step)) + 1);
	}

	PathFollowerConfig m_config;
	std::vector<Eigen::Vector3d> m_escape_offsets;  ///< points around the drone escape() tries, nearest first

	PathFollowerState m_state = PathFollowerState::Waiting;
	ReplanReason m_reason = ReplanReason::NoPath;  ///< reason of the request while waiting

	/// The route: waypoints, the arc length at each of them, and the speed to pass each one at.
	std::vector<Eigen::Vector3d> m_waypoints;
	std::vector<double> m_arc_lengths;
	std::vector<double> m_corner_speeds;
	std::size_t m_segment = 0;  ///< segment of the path the drone is on
	double m_progress = 0.0;    ///< arc length of the drone's projection, m; only grows

	std::optional<Setpoint> m_hold;                 ///< hover point while not moving
	std::optional<double> m_yaw;                    ///< yaw of the last moving trajectory (the drone's yaw at first)
	TimeSec m_failed_stamp = 0.0;                   ///< pose stamp of the last Failed answer
	std::optional<Eigen::Vector3d> m_escape_target;  ///< where escape() flies to
	std::optional<Trajectory> m_last_trajectory;     ///< for current_speed()
};

}
