#pragma once

#include "mapping/log_odds_grid_map.hpp"
#include "pipeline/clients.hpp"
#include "planning/core/global_planner.hpp"

#include <Eigen/Core>

#include <utility>

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

}
