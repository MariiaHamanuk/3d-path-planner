#pragma once

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <cstddef>

namespace path3d {

/// What every 3D voxel grid (see GridMap) shares, whatever its storage: voxels are
/// `Eigen::Vector3i` indices, and a voxel touches the 26 voxels around it.

/// A step to a neighboring voxel and its length in voxels (1, sqrt(2) or sqrt(3)).
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

}
