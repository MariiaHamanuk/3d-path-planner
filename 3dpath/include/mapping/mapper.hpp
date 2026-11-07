#pragma once

#include "core/communication.hpp"
#include "mapping/map_view.hpp"
#include "core/types.hpp"

#include <cstdint>
#include <optional>

namespace path3d {

/**
 * @brief Builds a map from sensor data and hands out updates for it. The rest of the system depends
 * only on this interface, so map representations (occupancy grid, TSDF + ESDF, ...) can be
 * swapped without touching the stages around them. M is the map type the readers keep copies
 * of (see ReplicatedMap in map_view.hpp); planners are templated on the same M and read it
 * through a MapClient.
 */
template <ReplicatedMap M>
class Mapper {
 public:
  using Update = typename M::Update;

  virtual ~Mapper() = default;
  Mapper(const Mapper&) = delete;
  Mapper& operator=(const Mapper&) = delete;

  /// Fuses one observation into the map. Called from a single thread, in stamp order.
  virtual void integrate(const SensorFrame& frame) = 0;

  /// Increases by one every time integrate() changed the map; 0 for an empty map.
  virtual std::uint64_t version() const = 0;

  /// Update for the readers, called from the same thread as integrate().
  ///   keyframe = false: what changed since the previous make_update() call (of either kind), with
  ///     from_version = the version at that call and to_version = version(). Null if nothing
  ///     changed.
  ///   keyframe = true: the whole map, with keyframe = true and to_version = version(). Never null.
  /// The mapper keeps track of what it has already handed out; a diff after a keyframe builds on
  /// it.
  virtual std::optional<Update> make_update(bool keyframe) = 0;

 protected:
  Mapper() = default;
};

}
