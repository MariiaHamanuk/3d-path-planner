#pragma once

#include "mapping/map_view.hpp"

#include <Eigen/Core>

#include <concepts>

namespace path3d {

/**
 * @brief What it costs to fly the straight move from one voxel to another, for GridMapSearchSpace:
 * `cost(from, to, occupancy)`, where `occupancy` (Free or Unknown) is that of the voxels the move
 * passes. Costs are in voxels (a step along an axis is 1).
 *
 * A move must never cost less than the straight distance between the voxels, or the heuristics
 * overestimate and A* no longer finds the cheapest path. A concept cannot check that.
 */
template <class C>
concept MoveCost = requires(const C& cost, const Eigen::Vector3i& voxel, Occupancy occupancy) {
	{ cost(voxel, voxel, occupancy) } -> std::convertible_to<double>;
};

}
