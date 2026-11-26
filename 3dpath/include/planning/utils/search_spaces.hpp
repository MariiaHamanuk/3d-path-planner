#pragma once

#include "mapping/grid_topology.hpp"
#include "mapping/map_view.hpp"
#include "planning/utils/voxel_cost.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <unordered_map>

namespace path3d {

/**
 * @brief The box a search may use: the box around start and goal grown by `padding`, cut to
 * [min_altitude, max_altitude] in z. The cut never leaves the start or the goal outside.
 */
inline Eigen::AlignedBox3d search_bounds(const Eigen::Vector3d& start, const Eigen::Vector3d& goal, double padding,
                                         double min_altitude, double max_altitude) {
	Eigen::AlignedBox3d box(start.cwiseMin(goal), start.cwiseMax(goal));
	const double lowest = box.min().z();
	const double highest = box.max().z();
	box.min().array() -= padding;
	box.max().array() += padding;
	box.min().z() = std::min(std::max(box.min().z(), min_altitude), lowest);
	box.max().z() = std::max(std::min(box.max().z(), max_altitude), highest);
	return box;
}

/**
 * @brief A GridMap seen as a graph for a_star(): a node is a voxel index, its neighbors are the 26
 * voxels around it. What a step costs is up to `Cost` (a VoxelCost; StepCost by default).
 *
 * The price of a straight line between two voxels, which lazy_theta_star() needs, is GridLineCost.
 *
 * Only voxels inside `bounds` are nodes, so a search on an unbounded map ends. The free test of
 * a voxel checks a ball of `margin` around it and is cached, because A* asks about the same voxel
 * from each of its neighbors. Keep the graph for one search only: it holds a reference to the map.
 */
template <GridMap M, VoxelCost Cost = StepCost>
class GridMapSearchSpace {
public:
	using Node = typename M::Index;
	using NodeHash = typename M::IndexHash;

	GridMapSearchSpace(const M& map, const Eigen::AlignedBox3d& bounds, double margin, bool unknown_is_free, Cost cost = {})
	    : m_map(map),
	      m_lower(map.index_of(bounds.min())),
	      m_upper(map.index_of(bounds.max())),
	      m_margin_voxels(static_cast<int>(std::ceil(margin / map.resolution()))),
	      m_unknown_is_free(unknown_is_free),
	      m_cost(std::move(cost)) {}

	bool is_free(const Node& node) const {
		if ((node.array() < m_lower.array()).any() || (node.array() > m_upper.array()).any()) return false;
		const auto cached = m_free.find(node);
		if (cached != m_free.end()) return cached->second;
		const bool free = is_clear_at(m_map, node, m_margin_voxels, m_unknown_is_free);
		m_free.emplace(node, free);
		return free;
	}

	template <class Visit>
	void for_each_neighbor(const Node& node, Visit&& visit) const {
		for (const GridStep& step : grid_steps()) {
			const Node neighbor = node + step.offset;
			if (!is_free(neighbor)) continue;
			visit(neighbor, m_cost(node, neighbor, m_map.occupancy_at(neighbor)));
		}
	}

	const M& map() const { return m_map; }
	const Cost& cost() const { return m_cost; }

private:
	const M& m_map;
	Node m_lower;
	Node m_upper;
	int m_margin_voxels;
	bool m_unknown_is_free;
	Cost m_cost;
	mutable std::unordered_map<Node, bool, NodeHash> m_free;
};

/**
 * @brief An OccupancyMap seen as a continuous space for rrt() and rrt_star(): points inside
 * `bounds` with `margin` of free space around them.
 *
 * The margin is the map's own clearance query (see is_clear()). A segment is checked at points half
 * a voxel apart on a grid map, 0.1 m apart on any other.
 */
template <OccupancyMap M>
class OccupancySpace {
public:
	OccupancySpace(const M& map, const Eigen::AlignedBox3d& bounds, double margin, bool unknown_is_free)
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
