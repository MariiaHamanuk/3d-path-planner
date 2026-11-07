#pragma once

#include "mapping/log_odds_grid_map.hpp"
#include "pipeline/clients.hpp"
#include "planning/global_planner.hpp"
#include "planning/local_planner.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <optional>

namespace path3d {

/**
 * @brief The simplest global planner: the path is the straight line from where the drone is to a
 * fixed goal. It ignores the map, except that it refuses a goal inside an obstacle.
 *
 * Tutorial: docs/tutorial_line_planner.md.
 */
class LineGlobalPlanner : public GlobalPlanner<LogOddsGridMap, ReplanRequest, GlobalPath> {
public:
	LineGlobalPlanner(MapClient<LogOddsGridMap>& map, UavClient& uav, Eigen::Vector3d goal)
	    : GlobalPlanner(map, uav), m_goal(std::move(goal)) {}

	GlobalPath plan(const ReplanRequest& request) override {
		GlobalPath path;
		path.stamp = m_map.map().stamp;
		if (m_map.map().occupancy(m_goal) == Occupancy::Occupied) {
			path.status = PlanStatus::Failed;
			return path;
		}
		path.waypoints = {request.position, m_goal};
		return path;
	}

private:
	Eigen::Vector3d m_goal;
};

/**
 * @brief The simplest local planner: flies along the line of the global path at a constant speed.
 *
 * Every cycle it finds where the drone is on the line and plans the next `horizon` seconds from
 * there, so a drone that drifted sideways is pulled back onto the line. If an obstacle is on the
 * planned part of the line, it publishes nothing (the commander then holds the position) and asks
 * for a new path.
 *
 * Tutorial: docs/tutorial_line_planner.md.
 */
class LineLocalPlanner : public LocalPlanner<LogOddsGridMap, GlobalPath, ReplanRequest> {
public:
	/**
	 * @param map The planner's copy of the map.
	 * @param uav Access to the drone pose.
	 * @param speed Cruise speed along the line, m/s.
	 * @param horizon How far ahead each trajectory reaches, s.
	 * @param step Time between the points of a trajectory, s.
	 */
	LineLocalPlanner(MapClient<LogOddsGridMap>& map, UavClient& uav, double speed = 2.0,
	                 double horizon = 2.0, double step = 0.1)
	    : LocalPlanner(map, uav), m_speed(speed), m_horizon(horizon), m_step(step) {}

	LocalStep<ReplanRequest> plan(const GlobalPath* new_route) override {
		const StampedPose pose = *m_uav.pose();

		if (new_route && new_route->status == PlanStatus::Ok && new_route->waypoints.size() >= 2) {
			m_start = new_route->waypoints.front();
			m_goal = new_route->waypoints.back();
			m_has_route = true;
		}
		if (!m_has_route) return {make_request(pose, ReplanReason::NoPath), std::nullopt};

		const Eigen::Vector3d line = m_goal - m_start;
		const double length = line.norm();
		const Eigen::Vector3d direction = length > 1e-6 ? Eigen::Vector3d(line / length) : Eigen::Vector3d::Zero();
		const double yaw = length > 1e-6 ? std::atan2(direction.y(), direction.x()) : yaw_of(pose.pose.orientation);

		/// How far along the line the drone is (clamped to the line).
		const double progress = std::clamp((pose.pose.position - m_start).dot(direction), 0.0, length);

		Trajectory trajectory;
		trajectory.stamp = pose.stamp;
		for (double t = 0.0; t <= m_horizon + 1e-9; t += m_step) {
			const double travelled = std::min(progress + m_speed * t, length);
			Setpoint point;
			point.stamp = pose.stamp + t;
			point.position = m_start + direction * travelled;
			if (travelled < length) point.velocity = direction * m_speed;
			point.yaw = yaw;
			if (m_map.map().occupancy(point.position) == Occupancy::Occupied) {
				return {make_request(pose, ReplanReason::Blocked), std::nullopt};
			}
			trajectory.points.push_back(point);
		}
		return {std::nullopt, std::move(trajectory)};
	}

private:
	static ReplanRequest make_request(const StampedPose& pose, ReplanReason reason) {
		ReplanRequest request;
		request.stamp = pose.stamp;
		request.reason = reason;
		request.position = pose.pose.position;
		return request;
	}

	double m_speed;
	double m_horizon;
	double m_step;
	bool m_has_route = false;
	Eigen::Vector3d m_start = Eigen::Vector3d::Zero();
	Eigen::Vector3d m_goal = Eigen::Vector3d::Zero();
};

}
