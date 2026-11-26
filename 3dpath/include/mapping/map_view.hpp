#pragma once

#include "core/communication.hpp"
#include "core/types.hpp"

#include <Eigen/Core>

#include <concepts>
#include <cstddef>
#include <cstdint>

namespace path3d {

/**
 * @brief Answer of an occupancy query.
 */
enum class Occupancy : std::uint8_t { Free,
                                      Occupied,
                                      Unknown };

/**
 * @brief Base of every map type: a map as of one moment, tagged with the time, version and frame it
 * describes. Planners read their own copy of it through a const reference (see ReplicatedMap
 * below), so no reader ever sees the map change under it. Derived maps add their own
 * queries; the concepts below say which queries an algorithm needs.
 */
class MapView {
public:
	TimeSec stamp = 0.0;             ///< time of the newest observation in the snapshot
	std::uint64_t version = 0;       ///< mapper version the snapshot was taken at
	FrameId frame = FrameId::World;  ///< coordinate frame of all positions

protected:
	MapView() = default;
};

/// A map type: anything derived from MapView.
template <class V>
concept MapViewType = std::derived_from<V, MapView>;

/// Message that carries a map from the mapper to its readers. What it holds is up to the map type;
/// the transport (Mapper, MappingThread, MapClient) only looks at these fields:
///   keyframe      true: the update holds the whole map and can be applied to any copy;
///                 false: a diff, applicable only to a copy at exactly `from_version`
///   from_version  version a diff builds on (unused for a keyframe)
///   to_version    version of the map after applying the update
template <class U>
concept MapUpdate = Stamped<U> && requires(const U& update) {
	{ update.from_version } -> std::convertible_to<std::uint64_t>;
	{ update.to_version } -> std::convertible_to<std::uint64_t>;
	{ update.keyframe } -> std::convertible_to<bool>;
};

/// A map that can be copied through updates: the mapper keeps the working map and publishes
/// `M::Update`s, and every reader (MapClient) keeps its own copy of M and brings it forward with
/// apply(). Every kind of map can qualify:
///   - incremental maps (voxel blocks, octree leaves, points): a diff holds only what changed, a
///     keyframe holds everything;
///   - any other copyable map: every update is a keyframe holding a full copy.
/// Contract for apply(): deterministic. It need not set `stamp` and `version`: MapClient sets them
/// to the update's `stamp` and `to_version` afterwards. Queries on a copy never change it, so a
/// reader needs no locks.
template <class M>
concept ReplicatedMap = MapViewType<M> && std::default_initializable<M> &&
                        requires(M& map, const typename M::Update& update) {
	                        requires MapUpdate<typename M::Update>;
	                        { map.apply(update) } -> std::same_as<void>;
                        };

/// A map that answers occupancy queries at a world position.
template <class V>
concept OccupancyMap = MapViewType<V> && requires(const V& map, const Eigen::Vector3d& p) {
	{ map.occupancy(p) } -> std::same_as<Occupancy>;
};

/// Regular voxel grid with integer indices: enables A*, Theta*, JPS, ...
/// `Index` names a voxel (for now always an Eigen::Vector3i) and `IndexHash` hashes it, in the way
/// the map's storage makes cheapest, so a search can keep per-voxel records in an unordered_map.
template <class V>
concept GridMap = OccupancyMap<V> && std::same_as<typename V::Index, Eigen::Vector3i> &&
                  requires(const V& map, const typename V::IndexHash& hash, const Eigen::Vector3d& p, const typename V::Index& i) {
	                  { map.resolution() } -> std::convertible_to<double>;
	                  { map.index_of(p) } -> std::same_as<typename V::Index>;
	                  { map.center_of(i) } -> std::same_as<Eigen::Vector3d>;
	                  { map.occupancy_at(i) } -> std::same_as<Occupancy>;
	                  { hash(i) } -> std::convertible_to<std::size_t>;
                  };

/// Distance to the nearest obstacle (ESDF-like).
template <class V>
concept DistanceMap = OccupancyMap<V> && requires(const V& map, const Eigen::Vector3d& p) {
	{ map.distance(p) } -> std::convertible_to<double>;
};

/// Distance plus its gradient: enables gradient-based trajectory optimization.
template <class V>
concept GradientMap = DistanceMap<V> && requires(const V& map, const Eigen::Vector3d& p) {
	{ map.gradient(p) } -> std::convertible_to<Eigen::Vector3d>;
};

/// A map that can say whether a ball around a point is free, in the way its storage makes cheapest
/// (scan voxels, read a distance, prune a tree). Unknown space counts as free when `unknown_is_free`.
/// Queries on a copy never change it (see ReplicatedMap), so the map may not cache on the fly.
template <class V>
concept ClearanceMap = OccupancyMap<V> && requires(const V& map, const Eigen::Vector3d& p) {
	{ map.is_clear(p, 0.0, true) } -> std::same_as<bool>;
};

/// The same on the voxel grid: the ball of `radius_voxels` voxels around a voxel index.
template <class V>
concept GridClearanceMap = GridMap<V> && requires(const V& map, const typename V::Index& i) {
	{ map.is_clear_at(i, 0, true) } -> std::same_as<bool>;
};

/// Whether a voxel with this occupancy may be flown through.
inline bool is_passable(Occupancy occupancy, bool unknown_is_free) {
	return occupancy == Occupancy::Free || (unknown_is_free && occupancy == Occupancy::Unknown);
}

/// Whether a ball of `radius` m around `p` is free. Uses the map's own is_clear() when it has one;
/// any other map is probed at `p` and at the six points `radius` away along the axes.
template <OccupancyMap M>
bool is_clear(const M& map, const Eigen::Vector3d& p, double radius, bool unknown_is_free) {
	if constexpr (ClearanceMap<M>) {
		return map.is_clear(p, radius, unknown_is_free);
	} else {
		if (!is_passable(map.occupancy(p), unknown_is_free)) return false;
		for (int axis = 0; axis < 3; ++axis) {
			const Eigen::Vector3d offset = Eigen::Vector3d::Unit(axis) * radius;
			if (!is_passable(map.occupancy(p + offset), unknown_is_free)) return false;
			if (!is_passable(map.occupancy(p - offset), unknown_is_free)) return false;
		}
		return true;
	}
}

/// Whether the ball of `radius_voxels` voxels around a voxel is free. Uses the map's own
/// is_clear_at() when it has one; any other grid is probed like is_clear().
template <GridMap M>
bool is_clear_at(const M& map, const Eigen::Vector3i& index, int radius_voxels, bool unknown_is_free) {
	if constexpr (GridClearanceMap<M>) {
		return map.is_clear_at(index, radius_voxels, unknown_is_free);
	} else {
		return is_clear(map, map.center_of(index), radius_voxels * map.resolution(), unknown_is_free);
	}
}

}
