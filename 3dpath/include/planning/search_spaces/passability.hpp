#pragma once

#include "mapping/map_view.hpp"

#include <Eigen/Core>

namespace path3d {

/// Whether a voxel with this occupancy may be flown through. Occupied never, Free always; Unknown
/// only when the planner decides so (the map is built while flying, so most of it starts Unknown).
inline bool is_passable(Occupancy occupancy, bool unknown_is_free) {
	return occupancy == Occupancy::Free || (unknown_is_free && occupancy == Occupancy::Unknown);
}

/// Whether the ball of `radius` m around `p` may be flown through: the map says what is in it
/// (ball_occupancy()), the planner what that allows.
template <OccupancyMap M>
bool is_clear(const M& map, const Eigen::Vector3d& p, double radius, bool unknown_is_free) {
	return is_passable(ball_occupancy(map, p, radius), unknown_is_free);
}

/// The same for the ball of `radius_voxels` voxels around a voxel.
template <GridMap M>
bool is_clear_at(const M& map, const Eigen::Vector3i& index, int radius_voxels, bool unknown_is_free) {
	return is_passable(ball_occupancy_at(map, index, radius_voxels), unknown_is_free);
}

}
