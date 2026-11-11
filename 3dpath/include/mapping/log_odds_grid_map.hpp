#pragma once

#include "mapping/map_view.hpp"

#include <Eigen/Core>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace path3d {

/**
 * @brief One voxel of a LogOddsGridMap update: integer grid index and its log-odds of occupancy.
 */
struct LogOddsCell {
	std::int32_t x = 0;
	std::int32_t y = 0;
	std::int32_t z = 0;
	float log_odds = 0.0f;
};

/**
 * @brief Update of a LogOddsGridMap. A keyframe holds every observed voxel; a diff holds the voxels
 * whose value changed (with their new values).
 */
struct LogOddsUpdate {
	TimeSec stamp = 0.0;
	std::uint64_t from_version = 0;
	std::uint64_t to_version = 0;
	bool keyframe = false;
	double resolution = 0.25;  ///< voxel edge, m; a copy takes it from the update
	std::vector<LogOddsCell> cells;
};

/**
 * @brief Sparse 3D occupancy grid in the world frame. A voxel stores the log-odds of being
 * occupied; a voxel that was never observed is not stored and answers Unknown. Voxel (i, j, k)
 * covers [i, i + 1) * resolution along each axis.
 */
class LogOddsGridMap : public MapView {
public:
	using Update = LogOddsUpdate;

	/// Log-odds above this count as occupied, at or below it as free.
	static constexpr float kOccupiedThreshold = 0.0f;

	LogOddsGridMap() = default;
	explicit LogOddsGridMap(double resolution) : m_resolution(resolution) {}

	/// Keyframe: replaces the contents (and the resolution). Diff: overwrites the listed voxels.
	void apply(const Update& update) {
		if (update.keyframe) {
			m_voxels.clear();
			m_resolution = update.resolution;
		}
		for (const LogOddsCell& cell : update.cells) {
			m_voxels[key_of({cell.x, cell.y, cell.z})] = cell.log_odds;
		}
		stamp = update.stamp;
		version = update.to_version;
	}

	double resolution() const { return m_resolution; }
	Eigen::Vector3i index_of(const Eigen::Vector3d& p) const {
		return {static_cast<int>(std::floor(p.x() / m_resolution)),
		        static_cast<int>(std::floor(p.y() / m_resolution)),
		        static_cast<int>(std::floor(p.z() / m_resolution))};
	}
	Eigen::Vector3d center_of(const Eigen::Vector3i& index) const {
		return (index.cast<double>().array() + 0.5).matrix() * m_resolution;
	}

	Occupancy occupancy_at(const Eigen::Vector3i& index) const {
		const auto it = m_voxels.find(key_of(index));
		if (it == m_voxels.end()) return Occupancy::Unknown;
		return it->second > kOccupiedThreshold ? Occupancy::Occupied : Occupancy::Free;
	}
	Occupancy occupancy(const Eigen::Vector3d& p) const { return occupancy_at(index_of(p)); }

	/// 0 (the unknown prior) for a voxel never observed.
	float log_odds_at(const Eigen::Vector3i& index) const { return log_odds(key_of(index)); }

	/// Number of observed voxels.
	std::size_t known_count() const { return m_voxels.size(); }
	std::size_t occupied_count() const {
		std::size_t count = 0;
		for (const auto& [key, value] : m_voxels) count += value > kOccupiedThreshold;
		return count;
	}

	/// Voxel keys: the grid index packed into 64 bits (21 bits per axis, +-1M voxels around 0).
	using Key = std::uint64_t;
	static Key key_of(const Eigen::Vector3i& index) {
		const auto pack = [](int v) { return static_cast<Key>(v + kOffset) & kMask; };
		return pack(index.x()) | pack(index.y()) << 21 | pack(index.z()) << 42;
	}
	static Eigen::Vector3i index_of_key(Key key) {
		const auto unpack = [](Key bits) { return static_cast<int>(bits & kMask) - kOffset; };
		return {unpack(key), unpack(key >> 21), unpack(key >> 42)};
	}

	float log_odds(Key key) const {
		const auto it = m_voxels.find(key);
		return it == m_voxels.end() ? 0.0f : it->second;
	}
	void set_log_odds(Key key, float value) { m_voxels[key] = value; }

	struct KeyHash {
		std::size_t operator()(Key key) const {
			key ^= key >> 33;
			key *= 0xff51afd7ed558ccdULL;
			key ^= key >> 33;
			return static_cast<std::size_t>(key);
		}
	};
	using Voxels = std::unordered_map<Key, float, KeyHash>;
	const Voxels& voxels() const { return m_voxels; }

private:
	static constexpr int kOffset = 1 << 20;
	static constexpr Key kMask = (Key{1} << 21) - 1;

	double m_resolution = 0.25;
	Voxels m_voxels;
};

}
