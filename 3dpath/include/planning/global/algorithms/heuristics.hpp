#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <functional>

namespace path3d {

/**
 * @brief Straight-line distance. ( on a 26-connected grid it
 * underestimates a bit, so A* expands more nodes)
 */
struct EuclideanHeuristic {
	template <class Derived>
	double operator()(const Eigen::MatrixBase<Derived>& a, const Eigen::MatrixBase<Derived>& b) const {
		return (a - b).template cast<double>().norm();
	}
};

/**
 * @brief Exact cost on an empty 26-connected grid with steps 1, sqrt(2) and sqrt(3): moves along
 * the space diagonal as long as all three axes differ, then along a face diagonal, then straight.
 * The tightest admissible heuristic for GridGraph.
 */
struct OctileHeuristic {
	double operator()(const Eigen::Vector3i& a, const Eigen::Vector3i& b) const {
		Eigen::Vector3d d = (a - b).cwiseAbs().cast<double>();
		std::sort(d.data(), d.data() + 3, std::greater<>());
		return d[0] + (std::sqrt(2.0) - 1.0) * d[1] + (std::sqrt(3.0) - std::sqrt(2.0)) * d[2];
	}
};

/**
 * @brief Sum of the differences along the axes. Overestimates on a grid with diagonal steps, so A*
 * is faster but its path is no longer guaranteed to be the shortest.
 */
struct ManhattanHeuristic {
	template <class Derived>
	double operator()(const Eigen::MatrixBase<Derived>& a, const Eigen::MatrixBase<Derived>& b) const {
		return (a - b).template cast<double>().cwiseAbs().sum();
	}
};

}
