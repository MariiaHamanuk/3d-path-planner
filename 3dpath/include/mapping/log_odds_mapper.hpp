#pragma once

#include "core/config.hpp"
#include "mapping/log_odds_grid_map.hpp"
#include "mapping/mapper.hpp"

#include <cstdint>
#include <optional>
#include <unordered_set>

namespace path3d {

/**
 * @brief Builds a LogOddsGridMap from depth images by ray casting.
 *
 * Every (strided) depth pixel is a ray from the camera: the voxels along it are free, the voxel at
 * the measured depth is occupied. Within one frame a voxel is updated at most once, and a hit wins
 * over a miss. Pixels without a return (inf/NaN) or beyond max_range only carve free space up to
 * max_range. Not thread-safe: call from one thread.
 */
class LogOddsMapper : public Mapper<LogOddsGridMap> {
public:
	explicit LogOddsMapper(const LogOddsMapperConfig& config = {});

	void integrate(const SensorFrame& frame) override;
	std::uint64_t version() const override { return m_version; }
	std::optional<Update> make_update(bool keyframe) override;

	/// The working map (do not hand it to other threads: replicate it through updates).
	const LogOddsGridMap& map() const { return m_map; }

private:
	using Key = LogOddsGridMap::Key;

	/// Adds `delta` to the voxel, clamped; remembers it for the next diff if its value changed.
	bool update_voxel(Key key, float delta);

	LogOddsMapperConfig m_config;
	LogOddsGridMap m_map;
	std::uint64_t m_version = 0;
	std::uint64_t m_sent_version = 0;
	/// Voxels changed since the last make_update().
	std::unordered_set<Key> m_dirty;
	/// Per-frame scratch, kept to reuse the allocations.
	std::unordered_set<Key> m_free;
	std::unordered_set<Key> m_hit;
};

}
