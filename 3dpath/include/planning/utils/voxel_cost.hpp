#pragma once

#include "mapping/map_view.hpp"

#include <Eigen/Core>

#include <algorithm>
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
concept VoxelCost = requires(const C& cost, const Eigen::Vector3i& voxel, Occupancy occupancy) {
	{ cost(voxel, voxel, occupancy) } -> std::convertible_to<double>;
};

/**
 * @brief The default VoxelCost: the length of the move, with two penalties.
 */
struct StepCost {
	/// A move through Unknown voxels costs this many times its length. Above 1, A* prefers space
	/// that was seen free and goes through unknown space only when that is much shorter. Must be
	/// at least 1, or the heuristics overestimate.
	double unknown_cost = 1.0;
	/// Every voxel of climbing costs this many voxels of flight on top of the length, because
	/// climbing takes more energy. 0 compares paths by length only. Must not be negative, or the
	/// heuristics overestimate.
	double climb_cost = 0.0;

	double operator()(const Eigen::Vector3i& from, const Eigen::Vector3i& to, Occupancy occupancy) const {
		double length = (to - from).cast<double>().norm();
		if (occupancy == Occupancy::Unknown) length *= unknown_cost;
		return length + climb(from, to);
	}

private:
	double climb(const Eigen::Vector3i& from, const Eigen::Vector3i& to) const {
		return climb_cost * std::max(0, to.z() - from.z());
	}
};

}
