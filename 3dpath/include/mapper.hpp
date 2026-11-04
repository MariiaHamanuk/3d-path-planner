#pragma once

#include "types.hpp"

#include <cstdint>

namespace path3d {

// Builds a map from sensor data. The rest of the system depends only on this interface, so map
// representations (occupancy grid, TSDF + ESDF, ...) can be swapped without touching the stages
// around them.
//
// The input is a `SensorFrame`, i.e. everything the simulation (or any `Reader`) provides. Map
// queries are deliberately not part of this interface: each mapper exposes its own map type,
// which planners consume through concepts (see the planning discussion in the README).
class Mapper {
 public:
  virtual ~Mapper() = default;
  Mapper(const Mapper&) = delete;
  Mapper& operator=(const Mapper&) = delete;

  // Fuses one observation into the map. Called from a single thread, in stamp order.
  virtual void integrate(const SensorFrame& frame) = 0;

  // Increases by one every time integrate() changed the map; 0 for an empty map.
  virtual std::uint64_t version() const = 0;

 protected:
  Mapper() = default;
};

}  // namespace path3d
