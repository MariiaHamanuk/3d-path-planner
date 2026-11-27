#pragma once

#include "mapping/map_view.hpp"
#include "planning/search_spaces/passability.hpp"
#include "planning/costs/length_cost.hpp"
#include "planning/costs/move_cost.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <unordered_map>

namespace path3d {

/// A step to a neighboring voxel and its length in voxels (1, sqrt(2) or sqrt(3)). Which voxels
/// count as neighbors is the search's choice: GridMapSearchSpace uses the 26 around a voxel.
struct GridStep {
	Eigen::Vector3i offset;
	double length;
};

/// The 26 steps from a voxel to the voxels around it.
inline const std::array<GridStep, 26>& grid_steps() {
	static const std::array<GridStep, 26> steps = [] {
		std::array<GridStep, 26> result{};
		std::size_t count = 0;
		for (int dx = -1; dx <= 1; ++dx) {
			for (int dy = -1; dy <= 1; ++dy) {
				for (int dz = -1; dz <= 1; ++dz) {
					if (dx == 0 && dy == 0 && dz == 0) continue;
					result[count++] = {Eigen::Vector3i(dx, dy, dz), std::sqrt(static_cast<double>(dx * dx + dy * dy + dz * dz))};
				}
			}
		}
		return result;
	}();
	return steps;
}

/// General hash of a voxel index, for maps that bring none of their own.
struct VoxelHash {
	std::size_t operator()(const Eigen::Vector3i& voxel) const {
		std::uint64_t key = static_cast<std::uint32_t>(voxel.x());
		key = key * 0x9e3779b97f4a7c15ULL ^ static_cast<std::uint32_t>(voxel.y());
		key = key * 0x9e3779b97f4a7c15ULL ^ static_cast<std::uint32_t>(voxel.z());
		key ^= key >> 33;
		key *= 0xff51afd7ed558ccdULL;
		key ^= key >> 33;
		return static_cast<std::size_t>(key);
	}
};

/// The hash a search keeps voxels in a hash map with: the map's own IndexHash if it has one (it
/// knows its storage), otherwise VoxelHash. A map is not required to have one.
template <class M>
struct IndexHashOf {
	using type = VoxelHash;
};
template <class M>
    requires requires { typename M::IndexHash; }
struct IndexHashOf<M> {
	using type = typename M::IndexHash;
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

/**
 * @brief A GridMap seen as a graph for a_star(): a node is a voxel index, its neighbors are the 26
 * voxels around it. What a step costs is up to `Cost` (a MoveCost; LengthCost by default).
 *
 * The price of a straight line between two voxels, which lazy_theta_star() needs, is GridLineOfSightCost.
 *
 * Only voxels inside `bounds` are nodes, so a search on an unbounded map ends. The free test of
 * a voxel checks a ball of `margin` around it and is cached, because A* asks about the same voxel
 * from each of its neighbors. Keep the graph for one search only: it holds a reference to the map.
 */
template <GridMap M, MoveCost Cost = LengthCost>
class GridMapSearchSpace {
public:
	using Node = typename M::Index;
	using NodeHash = typename IndexHashOf<M>::type;

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

}
