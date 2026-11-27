#pragma once

#include "mapping/map_view.hpp"
#include "planning/search_spaces/passability.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>

namespace path3d {

/**
 * @brief An OccupancyMap as the question "may the drone be here, or fly this line": points inside
 * `bounds` with `margin` of free space around them. It is the continuous space that rrt(),
 * rrt_connect() and rrt_star() search, and the free-space check of the local planner.
 *
 * The margin is the map's own ball query (see is_clear()). A segment is checked at points half
 * a voxel apart on a grid map, 0.1 m apart on any other.
 */
template <OccupancyMap M>
class ContinuousSearchSpace {
public:
	ContinuousSearchSpace(const M& map, const Eigen::AlignedBox3d& bounds, double margin, bool unknown_is_free)
	    : m_map(map), m_bounds(bounds), m_margin(margin), m_unknown_is_free(unknown_is_free) {
		if constexpr (GridMap<M>) {
			m_check_step = map.resolution() / 2.0;
		}
	}

	bool is_free(const Eigen::Vector3d& p) const {
		return m_bounds.contains(p) && is_clear(m_map, p, m_margin, m_unknown_is_free);
	}

	bool segment_free(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const {
		const double length = (b - a).norm();
		const int steps = std::max(1, static_cast<int>(std::ceil(length / m_check_step)));
		for (int i = 0; i <= steps; ++i) {
			if (!is_free(a + (b - a) * (static_cast<double>(i) / steps))) return false;
		}
		return true;
	}

	Eigen::Vector3d lower_bound() const { return m_bounds.min(); }
	Eigen::Vector3d upper_bound() const { return m_bounds.max(); }

private:
	const M& m_map;
	Eigen::AlignedBox3d m_bounds;
	double m_margin;
	bool m_unknown_is_free;
	double m_check_step = 0.1;  ///< m between the points of a segment that are checked; half a voxel on a grid map
};

}
