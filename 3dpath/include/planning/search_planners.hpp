#pragma once

#include "mapping/map_view.hpp"
#include "pipeline/clients.hpp"
#include "planning/algorithms/a_star.hpp"
#include "planning/algorithms/heuristics.hpp"
#include "planning/algorithms/lazy_theta_star.hpp"
#include "planning/algorithms/map_adapters.hpp"
#include "planning/algorithms/rrt.hpp"
#include "planning/algorithms/rrt_connect.hpp"
#include "planning/algorithms/rrt_star.hpp"
#include "planning/global_planner.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <vector>

namespace path3d {

/**
 * @brief Settings shared by the search-based global planners.
 */
struct SearchPlannerConfig {
	SearchSpaceConfig space;              ///< what counts as free (margin, unknown space)
	double padding = 10.0;                ///< m added around start and goal to make the search box
	double min_altitude = 1.0;            ///< lowest z of the search box, m (unless start or goal is lower)
	double max_altitude = 30.0;           ///< highest z of the search box, m (unless start or goal is higher)
	std::size_t max_expansions = 2000000; ///< A* gives up after expanding this many voxels
};

/**
 * @brief Numbers about the last plan, for comparing planners.
 */
struct PlanStats {
	bool found = false;
	double length = 0.0;       ///< length of the waypoint polyline, m
	double climb = 0.0;        ///< sum of all the climbs along the path, m; 0 for a path at one height
	std::size_t explored = 0;  ///< A*: expanded voxels; RRT, RRT*: tree nodes
};

/// Length of a polyline, m.
inline double polyline_length(const std::vector<Eigen::Vector3d>& points) {
	double length = 0.0;
	for (std::size_t i = 1; i < points.size(); ++i) length += (points[i] - points[i - 1]).norm();
	return length;
}

/// Sum of the climbs along a polyline (descents not counted), m.
inline double polyline_climb(const std::vector<Eigen::Vector3d>& points) {
	double climb = 0.0;
	for (std::size_t i = 1; i < points.size(); ++i) climb += std::max(0.0, points[i].z() - points[i - 1].z());
	return climb;
}

/**
 * @brief Waypoints of a voxel path: the exact start, the centers of the voxels where the path
 * turns, and the exact goal. Voxels on a straight stretch are left out.
 */
template <GridMap M>
std::vector<Eigen::Vector3d> voxel_path_waypoints(const M& map, const std::vector<Eigen::Vector3i>& voxels,
                                                  const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
	std::vector<Eigen::Vector3d> waypoints{start};
	for (std::size_t i = 1; i + 1 < voxels.size(); ++i) {
		const Eigen::Vector3d in = (voxels[i] - voxels[i - 1]).cast<double>().normalized();
		const Eigen::Vector3d out = (voxels[i + 1] - voxels[i]).cast<double>().normalized();
		if ((in - out).norm() > 1e-9) waypoints.push_back(map.center_of(voxels[i]));
	}
	waypoints.push_back(goal);
	return waypoints;
}

/**
 * @brief Global planner that runs A* on the voxels of a GridMap, from the drone to a fixed goal.
 *
 * The search is limited to search_bounds() around the drone and the goal. The waypoints are the
 * drone position, the centers of the voxels where the path turns, and the goal.
 *
 * @tparam M Map type; any GridMap that can be replicated (for example LogOddsGridMap).
 * @tparam H Heuristic on voxel indices. OctileHeuristic finds the shortest path with the fewest
 *   expansions; ZeroHeuristic makes it Dijkstra.
 */
template <class M, class H = OctileHeuristic>
    requires ReplicatedMap<M> && GridMap<M> && SearchHeuristic<H, Eigen::Vector3i>
class AStarGlobalPlanner : public GlobalPlanner<M, ReplanRequest, GlobalPath> {
public:
	AStarGlobalPlanner(MapClient<M>& map, UavClient& uav, Eigen::Vector3d goal, SearchPlannerConfig config = {},
	                   H heuristic = {})
	    : GlobalPlanner<M, ReplanRequest, GlobalPath>(map, uav), m_goal(std::move(goal)), m_config(config), m_heuristic(heuristic) {}

	GlobalPath plan(const ReplanRequest& request) override {
		const M& map = this->m_map.map();
		GlobalPath path;
		path.stamp = map.stamp;

		const GridGraph<M> graph(map, search_bounds(request.position, m_goal, m_config.padding, m_config.min_altitude, m_config.max_altitude), m_config.space);
		const auto result = a_star(graph, map.index_of(request.position), map.index_of(m_goal), m_heuristic, m_config.max_expansions);
		m_stats = {.found = result.found(), .explored = result.expanded};
		if (!result.found()) {
			path.status = PlanStatus::Failed;
			return path;
		}

		path.waypoints = voxel_path_waypoints(map, result.path, request.position, m_goal);
		m_stats.length = polyline_length(path.waypoints);
		m_stats.climb = polyline_climb(path.waypoints);
		return path;
	}

	/// Numbers about the last plan() call.
	const PlanStats& last_stats() const { return m_stats; }

private:
	Eigen::Vector3d m_goal;
	SearchPlannerConfig m_config;
	H m_heuristic;
	PlanStats m_stats;
};

/**
 * @brief Global planner that runs Lazy Theta* on the voxels of a GridMap, from the drone to a fixed
 * goal: like AStarGlobalPlanner, but the path goes at any angle instead of along the grid, so it
 * is shorter and has fewer waypoints.
 *
 * @tparam M Map type; any GridMap that can be replicated.
 * @tparam H Heuristic on voxel indices. It must not overestimate straight lines, so the default is
 *   EuclideanHeuristic (OctileHeuristic overestimates them).
 */
template <class M, class H = EuclideanHeuristic>
    requires ReplicatedMap<M> && GridMap<M> && SearchHeuristic<H, Eigen::Vector3i>
class LazyThetaStarGlobalPlanner : public GlobalPlanner<M, ReplanRequest, GlobalPath> {
public:
	LazyThetaStarGlobalPlanner(MapClient<M>& map, UavClient& uav, Eigen::Vector3d goal, SearchPlannerConfig config = {},
	                           H heuristic = {})
	    : GlobalPlanner<M, ReplanRequest, GlobalPath>(map, uav), m_goal(std::move(goal)), m_config(config), m_heuristic(heuristic) {}

	GlobalPath plan(const ReplanRequest& request) override {
		const M& map = this->m_map.map();
		GlobalPath path;
		path.stamp = map.stamp;

		const GridGraph<M> graph(map, search_bounds(request.position, m_goal, m_config.padding, m_config.min_altitude, m_config.max_altitude), m_config.space);
		const auto result = lazy_theta_star(graph, map.index_of(request.position), map.index_of(m_goal), m_heuristic, m_config.max_expansions);
		m_stats = {.found = result.found(), .explored = result.expanded};
		if (!result.found()) {
			path.status = PlanStatus::Failed;
			return path;
		}

		path.waypoints = voxel_path_waypoints(map, result.path, request.position, m_goal);
		m_stats.length = polyline_length(path.waypoints);
		m_stats.climb = polyline_climb(path.waypoints);
		return path;
	}

	/// Numbers about the last plan() call.
	const PlanStats& last_stats() const { return m_stats; }

private:
	Eigen::Vector3d m_goal;
	SearchPlannerConfig m_config;
	H m_heuristic;
	PlanStats m_stats;
};

/// Parameters of a sampling algorithm; the type picks the algorithm in SamplingGlobalPlanner.
template <class P>
concept SamplingParams = std::same_as<P, RrtParams> || std::same_as<P, RrtStarParams> || std::same_as<P, RrtConnectParams>;

/**
 * @brief Global planner that runs a sampling algorithm (RRT, RRT* or RRT-Connect) in the free space of an
 * OccupancyMap, from the drone to a fixed goal.
 *
 * Random points are drawn from search_bounds() around the drone and the goal. The seed of each
 * plan is `params.seed + request.id`, so a replan does not repeat the same tree.
 *
 * @tparam M Map type; any OccupancyMap that can be replicated.
 * @tparam P RrtParams runs rrt(), RrtStarParams runs rrt_star(), RrtConnectParams runs rrt_connect().
 */
template <class M, SamplingParams P = RrtStarParams>
    requires ReplicatedMap<M> && OccupancyMap<M>
class SamplingGlobalPlanner : public GlobalPlanner<M, ReplanRequest, GlobalPath> {
public:
	SamplingGlobalPlanner(MapClient<M>& map, UavClient& uav, Eigen::Vector3d goal, SearchPlannerConfig config = {},
	                      P params = {})
	    : GlobalPlanner<M, ReplanRequest, GlobalPath>(map, uav), m_goal(std::move(goal)), m_config(config), m_params(params) {}

	GlobalPath plan(const ReplanRequest& request) override {
		const M& map = this->m_map.map();
		GlobalPath path;
		path.stamp = map.stamp;

		const OccupancySpace<M> space(map, search_bounds(request.position, m_goal, m_config.padding, m_config.min_altitude, m_config.max_altitude), m_config.space);
		P params = m_params;
		params.seed += request.id;
		RrtResult<3> result;
		if constexpr (std::same_as<P, RrtStarParams>) {
			result = rrt_star<3>(space, request.position, m_goal, params);
		} else if constexpr (std::same_as<P, RrtConnectParams>) {
			result = rrt_connect<3>(space, request.position, m_goal, params);
		} else {
			result = rrt<3>(space, request.position, m_goal, params);
		}
		m_stats = {.found = result.found(), .explored = result.tree_size};
		if (!result.found()) {
			path.status = PlanStatus::Failed;
			return path;
		}
		path.waypoints = std::move(result.path);
		m_stats.length = result.cost;
		m_stats.climb = polyline_climb(path.waypoints);
		return path;
	}

	/// Numbers about the last plan() call.
	const PlanStats& last_stats() const { return m_stats; }

private:
	Eigen::Vector3d m_goal;
	SearchPlannerConfig m_config;
	P m_params;
	PlanStats m_stats;
};

/// RRT as a global planner: finds a path fast, but a long and crooked one.
template <class M>
using RrtGlobalPlanner = SamplingGlobalPlanner<M, RrtParams>;

/// RRT* as a global planner: slower than RRT, the path gets close to the shortest.
template <class M>
using RrtStarGlobalPlanner = SamplingGlobalPlanner<M, RrtStarParams>;

/// RRT-Connect as a global planner: two trees towards each other; usually the fastest to find a path.
template <class M>
using RrtConnectGlobalPlanner = SamplingGlobalPlanner<M, RrtConnectParams>;

}
