#pragma once

#include "planning/search_planners.hpp"

#include <Eigen/Core>

#include <array>
#include <memory>
#include <string>
#include <string_view>

namespace path3d {

/// Names make_global_planner() accepts (the first name of each planner; see global_planner_names()).
inline constexpr std::array<std::string_view, 6> kGlobalPlannerNames = {"astar", "theta", "dijkstra", "rrt", "rrt_star", "rrt_connect"};

/// The accepted names in one line, for usage and error messages.
inline std::string global_planner_names() {
	std::string names;
	for (const std::string_view name : kGlobalPlannerNames) {
		if (!names.empty()) names += " | ";
		names += name;
	}
	return names;
}

/**
 * @brief The global planner called `name`, planning to `goal`: astar, theta (Lazy Theta*; also
 * lazy_theta_star), dijkstra, rrt, rrt_star or rrt_connect. Null for any other name.
 */
template <class M>
    requires ReplicatedMap<M> && GridMap<M>
std::unique_ptr<GlobalPlanner<M, ReplanRequest, GlobalPath>> make_global_planner(std::string_view name, MapClient<M>& map, UavClient& uav,
                                                                                 const Eigen::Vector3d& goal, const SearchPlannerConfig& config = {}) {
	if (name == "astar") return std::make_unique<AStarGlobalPlanner<M>>(map, uav, goal, config);
	if (name == "theta" || name == "lazy_theta_star") return std::make_unique<LazyThetaStarGlobalPlanner<M>>(map, uav, goal, config);
	if (name == "dijkstra") return std::make_unique<AStarGlobalPlanner<M, ZeroHeuristic>>(map, uav, goal, config);
	if (name == "rrt") return std::make_unique<RrtGlobalPlanner<M>>(map, uav, goal, config);
	if (name == "rrt_star") return std::make_unique<RrtStarGlobalPlanner<M>>(map, uav, goal, config);
	if (name == "rrt_connect") return std::make_unique<RrtConnectGlobalPlanner<M>>(map, uav, goal, config);
	return nullptr;
}

}
