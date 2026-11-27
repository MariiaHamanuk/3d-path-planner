#pragma once

#include "planning/search_spaces/grid_map_search_space.hpp"
#include "planning/costs/move_cost.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <optional>

namespace path3d {

/**
 * @brief The price of a straight line between two voxels of a GridMapSearchSpace (the `line_cost` of lazy_theta_star()):
 * nullopt when a voxel on the line is not free, otherwise the cost of the whole move, averaged over
 * the voxels nearest to points a quarter voxel apart, each taken with its own occupancy.
 *
 * Keep it for one search only: it holds a reference to the space.
 */
template <GridMap M, MoveCost Cost>
class GridLineOfSightCost {
public:
	explicit GridLineOfSightCost(const GridMapSearchSpace<M, Cost>& space) : m_space(space) {}

	std::optional<double> operator()(const Eigen::Vector3i& a, const Eigen::Vector3i& b) const {
		const Eigen::Vector3d from = a.cast<double>();
		const Eigen::Vector3d delta = (b - a).cast<double>();
		const int samples = std::max(1, static_cast<int>(std::ceil(delta.norm() * 4.0)));
		double total = 0.0;
		for (int i = 1; i <= samples; ++i) {
			const Eigen::Vector3i voxel = (from + delta * (static_cast<double>(i) / samples)).array().round().cast<int>().matrix();
			if (!m_space.is_free(voxel)) return std::nullopt;
			total += m_space.cost()(a, b, m_space.map().occupancy_at(voxel));
		}
		return total / samples;
	}

private:
	const GridMapSearchSpace<M, Cost>& m_space;
};

}
