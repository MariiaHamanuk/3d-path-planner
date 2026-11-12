#pragma once

#include "mapping/log_odds_grid_map.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace path3d {

/**
 * @brief A test world for comparing global planners: box obstacles, a start and a goal.
 *
 * The map of a scene holds only its obstacles; everything else is Unknown, which the planners
 * treat as free by default. So every planner sees the whole world at once, as if the drone had
 * already mapped it.
 */
struct Scene {
	std::string name;
	std::string description;
	std::vector<Eigen::AlignedBox3d> obstacles;  ///< world frame, m
	Eigen::Vector3d start = Eigen::Vector3d::Zero();
	Eigen::Vector3d goal = Eigen::Vector3d::Zero();
	double max_altitude = 30.0;  ///< ceiling of the search, m
};

/// Keyframe of a LogOddsGridMap with every voxel that overlaps an obstacle of `scene` occupied.
inline LogOddsUpdate scene_map(const Scene& scene, double resolution) {
	LogOddsUpdate update;
	update.keyframe = true;
	update.to_version = 1;
	update.resolution = resolution;
	for (const Eigen::AlignedBox3d& box : scene.obstacles) {
		const Eigen::Vector3i lower = (box.min() / resolution).array().floor().cast<int>();
		const Eigen::Vector3i upper = (box.max() / resolution).array().ceil().cast<int>();
		for (int x = lower.x(); x < upper.x(); ++x) {
			for (int y = lower.y(); y < upper.y(); ++y) {
				for (int z = lower.z(); z < upper.z(); ++z) update.cells.push_back({.x = x, .y = y, .z = z, .log_odds = 3.5f});
			}
		}
	}
	return update;
}

namespace detail {

inline Eigen::AlignedBox3d box(double x0, double y0, double z0, double x1, double y1, double z1) {
	return {Eigen::Vector3d(x0, y0, z0), Eigen::Vector3d(x1, y1, z1)};
}

inline Scene named_scene(std::string name, std::string description) {
	Scene scene;
	scene.name = std::move(name);
	scene.description = std::move(description);
	return scene;
}

}

/**
 * @brief The scenes of the planner comparison. Each one asks for something different in 3D:
 *
 * - `open_field`: a few pillars; the control case, where 2D and 3D give the same path.
 * - `wall`: a wall that is shorter to fly over than around. With a climb cost, A* goes around.
 * - `window`: a tall wall with one opening at 8-12 m; the only way is up through it.
 * - `low_passage`: a tall wall with a low gap off to the side; the drone has to go down.
 * - `city_ceiling`: 4 x 4 blocks of 10 m and 25 m buildings under a 15 m ceiling: over the low
 *   ones, between the high ones.
 */
inline std::vector<Scene> benchmark_scenes() {
	using detail::box;
	std::vector<Scene> scenes;

	Scene open_field = detail::named_scene("open_field", "a few pillars, the path can stay at one height");
	for (const auto& [x, y] : std::vector<std::pair<double, double>>{{10, -2}, {15, 2}, {20, -1}, {25, 3}, {30, -3}}) {
		open_field.obstacles.push_back(box(x, y, 0, x + 1, y + 1, 10));
	}
	open_field.start = {0, 0, 2};
	open_field.goal = {40, 0, 2};
	scenes.push_back(open_field);

	Scene wall = detail::named_scene("wall", "a 6 m wall, shorter over than around");
	wall.obstacles.push_back(box(19, -8, 0, 21, 8, 6));
	wall.start = {0, 0, 2};
	wall.goal = {40, 0, 2};
	scenes.push_back(wall);

	Scene window = detail::named_scene("window", "a 20 m wall with one opening at 8-12 m");
	window.obstacles = {box(19, -20, 0, 21, -2, 20), box(19, 2, 0, 21, 20, 20),  // left and right of the opening
	                    box(19, -2, 0, 21, 2, 8), box(19, -2, 12, 21, 2, 20)};    // below and above it
	window.start = {0, 0, 2};
	window.goal = {40, 0, 2};
	scenes.push_back(window);

	Scene low_passage = detail::named_scene("low_passage", "a 20 m wall with a 3 m high gap at y 4-8");
	low_passage.obstacles = {box(19, -20, 0, 21, 4, 20), box(19, 8, 0, 21, 20, 20),  // beside the gap
	                         box(19, 4, 3, 21, 8, 20)};                              // above it
	low_passage.start = {0, 0, 5};
	low_passage.goal = {40, 0, 5};
	scenes.push_back(low_passage);

	Scene city = detail::named_scene("city_ceiling", "10 m and 25 m buildings under a 15 m ceiling");
	for (int i = 0; i < 4; ++i) {
		for (int j = 0; j < 4; ++j) {
			const double x = 3 + 16 * i;
			const double y = 3 + 16 * j;
			city.obstacles.push_back(box(x, y, 0, x + 10, y + 10, (i + j) % 2 == 0 ? 25 : 10));
		}
	}
	city.start = {-2, -2, 3};
	city.goal = {70, 70, 3};
	city.max_altitude = 15.0;
	scenes.push_back(city);

	return scenes;
}

}
