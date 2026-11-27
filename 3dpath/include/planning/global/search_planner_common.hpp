#pragma once

#include "mapping/map_view.hpp"
#include "planning/search_spaces/continuous_search_space.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

// What the search-based global planners share: their settings, how they find a start the drone
// can reach, and how a voxel path becomes waypoints.

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
 * @brief Settings shared by the search-based global planners.
 */
struct SearchPlannerConfig {
	double margin = 0.5;                  ///< free space the drone needs around its center, m: a point closer than this to an occupied voxel is not free
	bool unknown_is_free = true;          ///< the map is built while flying, so most of it is Unknown: true plans through it (a blocked path makes the local planner ask for a new one)
	double padding = 10.0;                ///< m added around start and goal to make the search box
	double min_altitude = 1.0;            ///< lowest z of the search box, m (unless start or goal is lower)
	double max_altitude = 30.0;           ///< highest z of the search box, m (unless start or goal is higher)
	std::size_t max_expansions = 2000000; ///< A* gives up after expanding this many voxels
	double start_snap = 2.0;              ///< m; a start that is not free moves to the nearest free point this close that it can reach in a straight line
};

/// The free point nearest to `p` on a grid of `step` within `radius`; `p` itself if it is free.
template <std::predicate<const Eigen::Vector3d&> Free>
std::optional<Eigen::Vector3d> nearest_free_point(const Eigen::Vector3d& p, double radius, double step, Free is_free) {
	if (is_free(p)) return p;
	const int n = static_cast<int>(std::ceil(radius / step));
	std::optional<Eigen::Vector3d> best;
	double best_distance = std::numeric_limits<double>::infinity();
	for (int dx = -n; dx <= n; ++dx) {
		for (int dy = -n; dy <= n; ++dy) {
			for (int dz = -n; dz <= n; ++dz) {
				const Eigen::Vector3d candidate = p + step * Eigen::Vector3d(dx, dy, dz);
				const double distance = (candidate - p).norm();
				if (distance > radius || distance >= best_distance || !is_free(candidate)) continue;
				best = candidate;
				best_distance = distance;
			}
		}
	}
	return best;
}

/// The start of a path from the drone at `p`: `p` itself if `is_free`, otherwise the nearest free
/// point (nearest_free_point()) that the drone can fly to in a straight line without touching an
/// occupied voxel - never one on the other side of a wall. `unknown_is_free` says whether Unknown
/// space is passable for that line.
template <OccupancyMap M, std::predicate<const Eigen::Vector3d&> Free>
std::optional<Eigen::Vector3d> reachable_start(const M& map, const Eigen::Vector3d& p, double radius, double step,
                                               bool unknown_is_free, Free is_free) {
	const Eigen::AlignedBox3d around(p.array() - (radius + 1.0), p.array() + (radius + 1.0));
	const ContinuousSearchSpace<M> line(map, around, 0.0, unknown_is_free);
	const auto reachable = [&](const Eigen::Vector3d& q) {
		/// the drone may itself be in a voxel the map calls occupied: then the line is checked from
		/// where it first gets out
		const double length = (q - p).norm();
		const int samples = std::max(1, static_cast<int>(std::ceil(length / (0.25 * step))));
		bool out = line.is_free(p);
		for (int i = 1; i <= samples; ++i) {
			const bool free = line.is_free(p + (q - p) * (static_cast<double>(i) / samples));
			if (free) out = true;
			if (!free && out) return false;
		}
		return out;
	};
	return nearest_free_point(p, radius, step, [&](const Eigen::Vector3d& q) { return is_free(q) && reachable(q); });
}

/**
 * @brief Waypoints of a voxel path: the exact start, the centers of the voxels where the path
 * turns, and the exact goal. Voxels on a straight stretch are left out.
 */
template <GridMap M>
std::vector<Eigen::Vector3d> voxel_path_waypoints(const M& map, const std::vector<Eigen::Vector3i>& voxels,
                                                  const Eigen::Vector3d& start, const Eigen::Vector3d& goal) {
	std::vector<Eigen::Vector3d> waypoints{start};
	for (std::size_t i = 1; i + 1 < voxels.size(); ++i) {
		const Eigen::Vector3d in = (voxels[i] - voxels[i - 1]).cast<double>().normalized();
		const Eigen::Vector3d out = (voxels[i + 1] - voxels[i]).cast<double>().normalized();
		if ((in - out).norm() > 1e-9) waypoints.push_back(map.center_of(voxels[i]));
	}
	waypoints.push_back(goal);
	return waypoints;
}

}
