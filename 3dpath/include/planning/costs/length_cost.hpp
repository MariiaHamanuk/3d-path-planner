#pragma once

#include "mapping/map_view.hpp"

#include <Eigen/Core>

#include <algorithm>

namespace path3d {

/**
 * @brief The default MoveCost: the length of the move, with two penalties.
 */
struct LengthCost {
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
