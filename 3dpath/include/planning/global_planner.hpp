#pragma once

#include "pipeline/clients.hpp"
#include "mapping/map_view.hpp"
#include "planning/plan_protocol.hpp"

namespace path3d {

/**
 * @brief Computes a plan in answer to a replanning request R. Called only when a request arrives
 * (and once at start, with the initial request), never on a timer. Reads the map through its
 * MapClient and the current drone position through its UavClient. The map copy (m_map.map())
 * is brought up to date by prepare() and never changes during plan(), so the whole plan is
 * made on one consistent map. Times are sim time from message stamps, never the wall clock.
 */
template <ReplicatedMap M, PlanRequest R, PlanAnswer P>
class GlobalPlanner {
public:
	virtual ~GlobalPlanner() = default;
	GlobalPlanner(const GlobalPlanner&) = delete;
	GlobalPlanner& operator=(const GlobalPlanner&) = delete;

	/// Brings the map copy up to date; call once before each plan(), on the planner's thread. True
	/// once a plan can be made (the map copy holds a full map). The thread holds requests until then.
	bool prepare() { return m_map.sync(); }

	/// The answer to `request`. Always returns one: when no path exists, return a plan that says so
	/// (for GlobalPath: status Failed), so the local planner is never left waiting. The thread fills
	/// in `request_id` and publishes the result.
	virtual P plan(const R& request) = 0;

protected:
	/**
	 * @param map Access to this planner's copy of the map.
	 * @param uav Access to the drone pose.
	 */
	GlobalPlanner(MapClient<M>& map, UavClient& uav) : m_map(map), m_uav(uav) {}

	MapClient<M>& m_map;
	UavClient& m_uav;
};

}
