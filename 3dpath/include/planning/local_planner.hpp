#pragma once

#include "pipeline/clients.hpp"
#include "mapping/map_view.hpp"
#include "planning/plan_protocol.hpp"

#include <optional>

namespace path3d {

/**
 * @brief Output of one local planning cycle.
 */
template <PlanRequest R>
struct LocalStep {
	std::optional<R> request;              ///< replan request for the global planner, if needed
	std::optional<Trajectory> trajectory;  ///< new trajectory for the drone; empty keeps the last one
};

/**
 * @brief Follows plans of type P on a copy of the map of type M. The stage calls plan() once per
 * control cycle on its own thread, so the planner never needs locks against its inputs.
 * Reads the map and the pose through its clients. The stage loop is paced in wall-clock
 * time, which is not sim time: compute every time step from message stamps (pose, map,
 * plan), never from the clock or from the cycle rate.
 */
template <ReplicatedMap M, PlanAnswer P, PlanRequest R>
class LocalPlanner {
public:
	virtual ~LocalPlanner() = default;
	LocalPlanner(const LocalPlanner&) = delete;
	LocalPlanner& operator=(const LocalPlanner&) = delete;

	/// Brings the map copy up to date; the stage calls it before every cycle, on the planner's
	/// thread. True once a map and a pose exist; the stage calls plan() only then. The map copy
	/// changes only here, never during plan().
	bool prepare() {
		const bool map_ready = m_map.sync();
		return map_ready && m_uav.pose().has_value();
	}

	/// One control cycle. `new_route` is the newest plan that arrived since the last cycle (it
	/// answers the newest request), or nullptr. The planner keeps its own copy of the current plan
	/// between cycles.
	/// Returns:
	///   - `trajectory`: the drone's next states, sent to the CommanderStage (which keeps flying the
	///     last one, and holds position once it is too old, so refresh it every cycle);
	///   - `request`: when the current plan can no longer be followed; the stage numbers it and
	///     publishes it up the pipe. While an earlier request is still unanswered the stage drops new
	///     ones, so keep returning the request every cycle for as long as it is needed.
	virtual LocalStep<R> plan(const P* new_route) = 0;

protected:
	/**
	 * @param map Access to this planner's copy of the map.
	 * @param uav Access to the drone pose.
	 */
	LocalPlanner(MapClient<M>& map, UavClient& uav) : m_map(map), m_uav(uav) {}

	MapClient<M>& m_map;
	UavClient& m_uav;
};

}
