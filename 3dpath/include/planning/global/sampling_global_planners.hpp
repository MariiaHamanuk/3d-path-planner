#pragma once

#include "mapping/map_view.hpp"
#include "pipeline/clients.hpp"
#include "planning/core/global_planner.hpp"
#include "planning/global/algorithms/rrt.hpp"
#include "planning/global/algorithms/rrt_connect.hpp"
#include "planning/global/algorithms/rrt_star.hpp"
#include "planning/global/search_planner_common.hpp"
#include "planning/search_spaces/continuous_search_space.hpp"

#include <Eigen/Core>

#include <cstdint>
#include <utility>

namespace path3d {

/**
 * @brief What the global planners that run a sampling algorithm (RRT, RRT*, RRT-Connect) in the
 * free space of an OccupancyMap share: plan() builds the space in search_bounds() around the drone
 * and the goal, finds a start the drone can reach, runs search(), and returns its path. A planner
 * only says which algorithm runs, with which parameters.
 *
 * The seed of each plan is the parameters' seed plus `request.id`, so a replan does not repeat the
 * same tree.
 *
 * @tparam M Map type; any OccupancyMap that can be replicated.
 */
template <class M>
    requires ReplicatedMap<M> && OccupancyMap<M>
class SamplingGlobalPlanner : public GlobalPlanner<M, ReplanRequest, GlobalPath> {
public:
	SamplingGlobalPlanner(MapClient<M>& map, UavClient& uav, Eigen::Vector3d goal, SearchPlannerConfig config = {})
	    : GlobalPlanner<M, ReplanRequest, GlobalPath>(map, uav), m_goal(std::move(goal)), m_config(config) {}

	GlobalPath plan(const ReplanRequest& request) final {
		const M& map = this->m_map.map();
		GlobalPath path;
		path.stamp = map.stamp;

		const ContinuousSearchSpace<M> space(map, search_bounds(request.position, m_goal, m_config.padding, m_config.min_altitude, m_config.max_altitude), m_config.margin, m_config.unknown_is_free);
		const auto start = reachable_start(map, request.position, m_config.start_snap, m_config.start_snap / 8.0, m_config.unknown_is_free,
		                                   [&](const Eigen::Vector3d& p) { return space.is_free(p); });
		if (!start) {
			path.status = PlanStatus::Failed;
			return path;
		}
		RrtResult result = search(space, *start, m_goal, request.id);
		if (!result.found()) {
			path.status = PlanStatus::Failed;
			return path;
		}
		path.waypoints = std::move(result.path);
		if (*start != request.position) path.waypoints.insert(path.waypoints.begin(), request.position);
		return path;
	}

protected:
	/// The search from `start` to `goal` in `space`; `seed_offset` is added to the seed.
	virtual RrtResult search(const ContinuousSearchSpace<M>& space, const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
	                         std::uint64_t seed_offset) const = 0;

private:
	Eigen::Vector3d m_goal;
	SearchPlannerConfig m_config;
};

/// RRT as a global planner: finds a path fast, but a long and crooked one.
template <class M>
    requires ReplicatedMap<M> && OccupancyMap<M>
class RrtGlobalPlanner : public SamplingGlobalPlanner<M> {
public:
	RrtGlobalPlanner(MapClient<M>& map, UavClient& uav, Eigen::Vector3d goal, SearchPlannerConfig config = {}, RrtParams params = {})
	    : SamplingGlobalPlanner<M>(map, uav, std::move(goal), config), m_params(params) {}

protected:
	RrtResult search(const ContinuousSearchSpace<M>& space, const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
	                 std::uint64_t seed_offset) const override {
		RrtParams params = m_params;
		params.seed += seed_offset;
		return rrt(space, start, goal, params);
	}

private:
	RrtParams m_params;
};

/// RRT* as a global planner: slower than RRT, the path gets close to the shortest.
template <class M>
    requires ReplicatedMap<M> && OccupancyMap<M>
class RrtStarGlobalPlanner : public SamplingGlobalPlanner<M> {
public:
	RrtStarGlobalPlanner(MapClient<M>& map, UavClient& uav, Eigen::Vector3d goal, SearchPlannerConfig config = {}, RrtStarParams params = {})
	    : SamplingGlobalPlanner<M>(map, uav, std::move(goal), config), m_params(params) {}

protected:
	RrtResult search(const ContinuousSearchSpace<M>& space, const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
	                 std::uint64_t seed_offset) const override {
		RrtStarParams params = m_params;
		params.seed += seed_offset;
		return rrt_star(space, start, goal, params);
	}

private:
	RrtStarParams m_params;
};

/// RRT-Connect as a global planner: two trees towards each other; usually the fastest to find a path.
template <class M>
    requires ReplicatedMap<M> && OccupancyMap<M>
class RrtConnectGlobalPlanner : public SamplingGlobalPlanner<M> {
public:
	RrtConnectGlobalPlanner(MapClient<M>& map, UavClient& uav, Eigen::Vector3d goal, SearchPlannerConfig config = {}, RrtConnectParams params = {})
	    : SamplingGlobalPlanner<M>(map, uav, std::move(goal), config), m_params(params) {}

protected:
	RrtResult search(const ContinuousSearchSpace<M>& space, const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
	                 std::uint64_t seed_offset) const override {
		RrtConnectParams params = m_params;
		params.seed += seed_offset;
		return rrt_connect(space, start, goal, params);
	}

private:
	RrtConnectParams m_params;
};

}
