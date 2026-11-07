#pragma once

#include "core/communication.hpp"
#include "core/types.hpp"

#include <Eigen/Core>

#include <concepts>
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
/// the transport (Mapper, MappingStage, MapClient) only looks at these fields:
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
template <class V>
concept GridMap = OccupancyMap<V> &&
                  requires(const V& map, const Eigen::Vector3d& p, const Eigen::Vector3i& i) {
	                  { map.resolution() } -> std::convertible_to<double>;
	                  { map.index_of(p) } -> std::same_as<Eigen::Vector3i>;
	                  { map.center_of(i) } -> std::same_as<Eigen::Vector3d>;
	                  { map.occupancy_at(i) } -> std::same_as<Occupancy>;
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

}
