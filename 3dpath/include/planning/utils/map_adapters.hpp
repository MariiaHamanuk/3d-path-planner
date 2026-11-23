#pragma once

#include "mapping/map_view.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>

namespace path3d {

/**
 * @brief How a map is turned into a search space (what counts as free, and what safe the path is)
 */
struct SearchSpaceConfig {
	/// Free space the drone needs around its center, m. A point closer than this to an occupied
	/// voxel is not free.
	double margin = 0.5;
	/// The map is built while flying, so most of it is Unknown at first. "True" lets the path go
	/// through unknown space (if it turns out blocked local planner just request global to make a new path).
	bool unknown_is_free = true;
	/// Answers the question "can I fly straight tho"
	double check_step = 0.0;
	/// GridGraph only: a step into an Unknown voxel costs this many times its length. Above 1, A*
	/// prefers space that was seen free and goes through unknown space only when that is much
	/// shorter. Must be at least 1, or the heuristics overestimate.
	double unknown_cost = 1.0;
	/// GridGraph only: every metre of climbing costs this many metres of flight on top of its
	/// length, because climbing takes more energy. 0 compares paths by length only. Must not be
	/// negative, or the heuristics overestimate.
	double climb_cost = 0.0;
};

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

namespace detail {

/// Whether a voxel may be flown through.
template <GridMap M>
bool voxel_passable(const M& map, const Eigen::Vector3i& index, bool unknown_is_free) {
	const Occupancy occupancy = map.occupancy_at(index);
	return occupancy == Occupancy::Free || (unknown_is_free && occupancy == Occupancy::Unknown);
}

/// Whether every voxel within `radius` voxels of `center` (a ball) may be flown through.
template <GridMap M>
bool ball_passable(const M& map, const Eigen::Vector3i& center, int radius, bool unknown_is_free) {
	for (int dx = -radius; dx <= radius; ++dx) {
		for (int dy = -radius; dy <= radius; ++dy) {
			for (int dz = -radius; dz <= radius; ++dz) {
				if (dx * dx + dy * dy + dz * dz > radius * radius) continue;
				if (!voxel_passable(map, center + Eigen::Vector3i(dx, dy, dz), unknown_is_free)) return false;
			}
		}
	}
	return true;
}

}

/**
 * @brief A GridMap seen as a graph for a_star(): a node is a voxel index, its neighbors are the 26
 * voxels around it, and a step costs its length in voxels (1, sqrt(2) or sqrt(3)).
 *
 * Two optional penalties change the cost of a step (see SearchSpaceConfig): `unknown_cost`
 * multiplies a step into an Unknown voxel, and `climb_cost` is added to a step that goes up.
 *
 * It can also fly straight between any two voxels (segment_cost(), straight_cost()), which
 * lazy_theta_star() uses.
 *
 * Only voxels inside `bounds` are nodes, so a search on an unbounded map ends. The free test of
 * a voxel checks a ball of `margin` around it and is cached, because A* asks about the same voxel
 * from each of its neighbors. Keep the graph for one search only: it holds a reference to the map.
 */
template <GridMap M>
class GridGraph {
public:
	using Node = Eigen::Vector3i;

	struct NodeHash {
		std::size_t operator()(const Node& node) const {
			std::uint64_t key = static_cast<std::uint32_t>(node.x());
			key = key * 0x9e3779b97f4a7c15ULL ^ static_cast<std::uint32_t>(node.y());
			key = key * 0x9e3779b97f4a7c15ULL ^ static_cast<std::uint32_t>(node.z());
			key ^= key >> 33;
			key *= 0xff51afd7ed558ccdULL;
			key ^= key >> 33;
			return static_cast<std::size_t>(key);
		}
	};

	GridGraph(const M& map, const Eigen::AlignedBox3d& bounds, const SearchSpaceConfig& config = {})
	    : m_map(map),
	      m_lower(map.index_of(bounds.min())),
	      m_upper(map.index_of(bounds.max())),
	      m_margin_voxels(static_cast<int>(std::ceil(config.margin / map.resolution()))),
	      m_unknown_is_free(config.unknown_is_free),
	      m_unknown_cost(config.unknown_cost),
	      m_climb_cost(config.climb_cost) {
		for (int dx = -1; dx <= 1; ++dx) {
			for (int dy = -1; dy <= 1; ++dy) {
				for (int dz = -1; dz <= 1; ++dz) {
					if (dx == 0 && dy == 0 && dz == 0) continue;
					m_steps[m_step_count++] = {Node(dx, dy, dz), std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz))};
				}
			}
		}
	}

	bool is_free(const Node& node) const {
		if ((node.array() < m_lower.array()).any() || (node.array() > m_upper.array()).any()) return false;
		const auto cached = m_free.find(node);
		if (cached != m_free.end()) return cached->second;
		const bool free = detail::ball_passable(m_map, node, m_margin_voxels, m_unknown_is_free);
		m_free.emplace(node, free);
		return free;
	}

	template <class Visit>
	void for_each_neighbor(const Node& node, Visit&& visit) const {
		for (std::size_t i = 0; i < m_step_count; ++i) {
			const Node neighbor = node + m_steps[i].offset;
			if (!is_free(neighbor)) continue;
			double cost = m_steps[i].cost;
			if (m_unknown_cost != 1.0 && m_map.occupancy_at(neighbor) == Occupancy::Unknown) cost *= m_unknown_cost;
			// a voxel up is a voxel of climbing; the cost is in voxels too
			if (m_steps[i].offset.z() > 0) cost += m_climb_cost;
			visit(neighbor, cost);
		}
	}

	/// Cost of the straight line from `a` to `b` if every voxel along it is free, with the same
	/// penalties as a step: the share of unknown voxels on the line is charged `unknown_cost`, and
	/// every voxel of climbing `climb_cost`. Checks the voxel nearest to points a quarter voxel apart.
	std::optional<double> segment_cost(const Node& a, const Node& b) const {
		const Eigen::Vector3d from = a.cast<double>();
		const Eigen::Vector3d delta = (b - a).cast<double>();
		const double length = delta.norm();
		const int samples = std::max(1, static_cast<int>(std::ceil(length * 4.0)));
		int unknown = 0;
		for (int i = 1; i <= samples; ++i) {
			const Node voxel = (from + delta * (static_cast<double>(i) / samples)).array().round().template cast<int>().matrix();
			if (!is_free(voxel)) return std::nullopt;
			if (m_unknown_cost != 1.0 && m_map.occupancy_at(voxel) == Occupancy::Unknown) ++unknown;
		}
		const double unknown_share = static_cast<double>(unknown) / samples;
		return length * (1.0 + (m_unknown_cost - 1.0) * unknown_share) + m_climb_cost * std::max(0, b.z() - a.z());
	}

	/// segment_cost() without looking at the map: as if every voxel on the line were free and seen.
	double straight_cost(const Node& a, const Node& b) const {
		return (b - a).template cast<double>().norm() + m_climb_cost * std::max(0, b.z() - a.z());
	}

private:
	struct Step {
		Node offset;
		double cost;
	};

	const M& m_map;
	Node m_lower;
	Node m_upper;
	int m_margin_voxels;
	bool m_unknown_is_free;
	double m_unknown_cost;
	double m_climb_cost;
	std::array<Step, 26> m_steps{};
	std::size_t m_step_count = 0;
	mutable std::unordered_map<Node, bool, NodeHash> m_free;
};

/**
 * @brief An OccupancyMap seen as a continuous space for rrt() and rrt_star(): points inside
 * `bounds` with `margin` of free space around them.
 *
 * On a GridMap the margin is a ball of voxels around the voxel of the point; on other maps the
 * point and the six points `margin` away along the axes are checked. A segment is checked at
 * points `check_step` apart.
 */
template <OccupancyMap M>
class OccupancySpace {
public:
	OccupancySpace(const M& map, const Eigen::AlignedBox3d& bounds, const SearchSpaceConfig& config = {})
	    : m_map(map), m_bounds(bounds), m_config(config) {
		if constexpr (GridMap<M>) {
			m_margin_voxels = static_cast<int>(std::ceil(config.margin / map.resolution()));
			if (m_config.check_step <= 0.0) m_config.check_step = map.resolution() / 2.0;
		} else {
			if (m_config.check_step <= 0.0) m_config.check_step = 0.1;
		}
	}

	bool is_free(const Eigen::Vector3d& p) const {
		if (!m_bounds.contains(p)) return false;
		if constexpr (GridMap<M>) {
			return detail::ball_passable(m_map, m_map.index_of(p), m_margin_voxels, m_config.unknown_is_free);
		} else {
			if (!passable(p)) return false;
			for (int axis = 0; axis < 3; ++axis) {
				const Eigen::Vector3d offset = Eigen::Vector3d::Unit(axis) * m_config.margin;
				if (!passable(p + offset) || !passable(p - offset)) return false;
			}
			return true;
		}
	}

	bool segment_free(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const {
		const double length = (b - a).norm();
		const int steps = std::max(1, static_cast<int>(std::ceil(length / m_config.check_step)));
		for (int i = 0; i <= steps; ++i) {
			if (!is_free(a + (b - a) * (static_cast<double>(i) / steps))) return false;
		}
		return true;
	}

	Eigen::Vector3d lower_bound() const { return m_bounds.min(); }
	Eigen::Vector3d upper_bound() const { return m_bounds.max(); }

private:
	bool passable(const Eigen::Vector3d& p) const {
		const Occupancy occupancy = m_map.occupancy(p);
		return occupancy == Occupancy::Free || (m_config.unknown_is_free && occupancy == Occupancy::Unknown);
	}

	const M& m_map;
	Eigen::AlignedBox3d m_bounds;
	SearchSpaceConfig m_config;
	int m_margin_voxels = 0;
};

}
