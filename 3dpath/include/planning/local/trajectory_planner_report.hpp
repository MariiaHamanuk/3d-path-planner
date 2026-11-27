#pragma once

#include "core/types.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

namespace path3d {

/**
 * @brief What the TrajectoryLocalPlanner is doing.
 */
enum class TrajectoryPlannerState {
	Waiting,     ///< hovers and asks for a route every cycle
	Following,   ///< flies the route; brakes and asks for a new one when it is blocked
	BackingOff,  ///< hovers after a Failed answer until `replan_backoff` has passed
	Escaping,    ///< moves out of the margin of an obstacle, so that a new route can start here
	Arrived,     ///< hovers at the end of the route
};

/// Short readable name of a planner state.
inline const char* to_string(TrajectoryPlannerState state) {
	switch (state) {
		case TrajectoryPlannerState::Waiting:
			return "waiting";
		case TrajectoryPlannerState::Following:
			return "following";
		case TrajectoryPlannerState::BackingOff:
			return "backing_off";
		case TrajectoryPlannerState::Escaping:
			return "escaping";
		case TrajectoryPlannerState::Arrived:
			return "arrived";
	}
	return "unknown";
}

/// Short readable name of a replan reason.
inline const char* to_string(ReplanReason reason) {
	switch (reason) {
		case ReplanReason::NoPath:
			return "no_path";
		case ReplanReason::Blocked:
			return "blocked";
		case ReplanReason::Deviated:
			return "deviated";
	}
	return "unknown";
}

/**
 * @brief What one cycle of the TrajectoryLocalPlanner saw and decided, for logs and visualization.
 * Distances are along the path from the planned point, in m; times are sim time.
 */
struct TrajectoryPlannerReport {
	static constexpr double kNone = std::numeric_limits<double>::infinity();

	TimeSec stamp = 0.0;  ///< pose stamp of the cycle
	TrajectoryPlannerState state = TrajectoryPlannerState::Waiting;  ///< after the cycle
	Eigen::Vector3d position = Eigen::Vector3d::Zero();              ///< the drone
	Eigen::Vector3d velocity = Eigen::Vector3d::Zero();              ///< commanded now, m/s
	Eigen::Vector3d actual_velocity = Eigen::Vector3d::Zero();       ///< measured from the poses, m/s
	/// Angle between where the drone (and its camera) looks and where it really moves, rad; 0 when
	/// it barely moves sideways.
	double yaw_error = 0.0;
	/// While following: distance from the drone to where the plan says it is, m.
	double tracking_error = 0.0;
	/// The plan was restarted from the real drone in this cycle (it was too far from the plan, or
	/// too fast or too slow).
	bool resynced = false;
	/// How old the map the cycle planned on is: pose stamp - map stamp, s.
	TimeSec map_age = 0.0;
	/// Distance from the drone to the nearest occupied voxel within a few meters, m; kNone if none.
	double clearance = kNone;

	/// An answer of the global planner arrived in this cycle.
	std::optional<PlanStatus> route;
	std::uint64_t route_request_id = 0;  ///< the request it answers
	std::size_t route_waypoints = 0;

	bool blocked = false;              ///< the path ahead runs into the margin of an obstacle
	double margin_distance = kNone;    ///< to the first point of the path inside the margin
	double obstacle_distance = kNone;  ///< to the first point of the path in an occupied voxel
	double braking_distance = 0.0;     ///< needed to stop from the current speed with max_acceleration
	bool emergency = false;            ///< too close to stop with max_acceleration: braking harder
	std::optional<Eigen::Vector3d> stop_point;  ///< where the plan stops, while blocked

	std::optional<ReplanReason> request;  ///< returned in this cycle (the thread drops it while one is unanswered)
	Eigen::Vector3d request_position = Eigen::Vector3d::Zero();

	/// The smoothed path being flown; shared, changes only with a new route.
	std::shared_ptr<const std::vector<Eigen::Vector3d>> path;
};

}
