#pragma once

#include "mapping/map_view.hpp"
#include "pipeline/clients.hpp"
#include "planning/core/global_planner.hpp"
#include "planning/costs/length_cost.hpp"
#include "planning/costs/move_cost.hpp"
#include "planning/global/algorithms/a_star.hpp"
#include "planning/global/algorithms/heuristics.hpp"
#include "planning/global/algorithms/lazy_theta_star.hpp"
#include "planning/global/search_planner_common.hpp"
#include "planning/search_spaces/grid_line_of_sight_cost.hpp"
#include "planning/search_spaces/grid_map_search_space.hpp"

#include <Eigen/Core>

#include <utility>

namespace path3d {

/**
 * @brief What the global planners on the voxels of a GridMap share: plan() builds the search space
 * in search_bounds() around the drone and the goal, finds a start the drone can reach, runs
 * search(), and turns the voxel path into waypoints (the drone position, the centers of the voxels
 * where the path turns, and the goal). A planner only says which search runs.
 *
 * @tparam M Map type; any GridMap that can be replicated (for example LogOddsGridMap).
 * @tparam H Heuristic on voxel indices.
 * @tparam Cost What a move between two voxels costs (see MoveCost). The heuristic must not
 *   overestimate it.
 */
template <class M, class H, class Cost>
    requires ReplicatedMap<M> && GridMap<M> && SearchHeuristic<H> && MoveCost<Cost>
class GridGlobalPlanner : public GlobalPlanner<M, ReplanRequest, GlobalPath> {
public:
	GridGlobalPlanner(MapClient<M>& map, UavClient& uav, Eigen::Vector3d goal, SearchPlannerConfig config = {},
	                  H heuristic = {}, Cost cost = {})
	    : GlobalPlanner<M, ReplanRequest, GlobalPath>(map, uav), m_config(config), m_heuristic(heuristic), m_goal(std::move(goal)), m_cost(cost) {}

	GlobalPath plan(const ReplanRequest& request) final {
		const M& map = this->m_map.map();
		GlobalPath path;
		path.stamp = map.stamp;

		const GridMapSearchSpace<M, Cost> space(map, search_bounds(request.position, m_goal, m_config.padding, m_config.min_altitude, m_config.max_altitude), m_config.margin, m_config.unknown_is_free, m_cost);
		const auto start = reachable_start(map, request.position, m_config.start_snap, map.resolution(), m_config.unknown_is_free,
		                                   [&](const Eigen::Vector3d& p) { return space.is_free(map.index_of(p)); });
		if (!start) {
			path.status = PlanStatus::Failed;
			return path;
		}
		const SearchResult result = search(space, map.index_of(*start), map.index_of(m_goal));
		if (!result.found()) {
			path.status = PlanStatus::Failed;
			return path;
		}

		path.waypoints = voxel_path_waypoints(map, result.path, *start, m_goal);
		if (*start != request.position) path.waypoints.insert(path.waypoints.begin(), request.position);
		return path;
	}

protected:
	/// The search from `start` to `goal` in `space`.
	virtual SearchResult search(const GridMapSearchSpace<M, Cost>& space, const Eigen::Vector3i& start, const Eigen::Vector3i& goal) const = 0;

	SearchPlannerConfig m_config;
	H m_heuristic;

private:
	Eigen::Vector3d m_goal;
	Cost m_cost;
};

/**
 * @brief Global planner that runs A* on the voxels of a GridMap, from the drone to a fixed goal.
 *
 * @tparam H OctileHeuristic finds the shortest path with the fewest expansions; ZeroHeuristic makes
 *   it Dijkstra.
 */
template <class M, class H = OctileHeuristic, class Cost = LengthCost>
    requires ReplicatedMap<M> && GridMap<M> && SearchHeuristic<H> && MoveCost<Cost>
class AStarGlobalPlanner : public GridGlobalPlanner<M, H, Cost> {
public:
	using GridGlobalPlanner<M, H, Cost>::GridGlobalPlanner;

protected:
	SearchResult search(const GridMapSearchSpace<M, Cost>& space, const Eigen::Vector3i& start, const Eigen::Vector3i& goal) const override {
		return a_star(space, start, goal, this->m_heuristic, this->m_config.max_expansions);
	}
};

/**
 * @brief Global planner that runs Lazy Theta* on the voxels of a GridMap, from the drone to a fixed
 * goal: like AStarGlobalPlanner, but the path goes at any angle instead of along the grid, so it
 * is shorter and has fewer waypoints.
 *
 * @tparam H It must not overestimate straight lines, so the default is EuclideanHeuristic
 *   (OctileHeuristic overestimates them).
 */
template <class M, class H = EuclideanHeuristic, class Cost = LengthCost>
    requires ReplicatedMap<M> && GridMap<M> && SearchHeuristic<H> && MoveCost<Cost>
class LazyThetaStarGlobalPlanner : public GridGlobalPlanner<M, H, Cost> {
public:
	using GridGlobalPlanner<M, H, Cost>::GridGlobalPlanner;

protected:
	SearchResult search(const GridMapSearchSpace<M, Cost>& space, const Eigen::Vector3i& start, const Eigen::Vector3i& goal) const override {
		return lazy_theta_star(space, start, goal, this->m_heuristic, GridLineOfSightCost(space), this->m_config.max_expansions);
	}
};

}
