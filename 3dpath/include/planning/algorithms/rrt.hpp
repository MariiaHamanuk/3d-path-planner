#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

namespace path3d {

/// A point of a Dim-dimensional continuous space.
template <int Dim>
using Vec = Eigen::Matrix<double, Dim, 1>;

/**
 * @brief A continuous space that sampling planners (RRT, RRT*) can search: a free test for points
 * and for straight segments, and the box that random points are drawn from.
 */
template <class S, int Dim>
concept SamplingSpace = requires(const S& space, const Vec<Dim>& p) {
	{ space.is_free(p) } -> std::convertible_to<bool>;
	{ space.segment_free(p, p) } -> std::convertible_to<bool>;
	{ space.lower_bound() } -> std::convertible_to<Vec<Dim>>;
	{ space.upper_bound() } -> std::convertible_to<Vec<Dim>>;
};

/**
 * @brief Settings of rrt().
 */
struct RrtParams {
	std::size_t max_iterations = 5000;  ///< samples drawn before giving up
	double step_size = 1.0;             ///< longest new edge, m
	double goal_bias = 0.05;            ///< share of samples that are the goal itself
	double goal_tolerance = 1.0;        ///< a node this close to the goal, with a free segment to it, reaches it
	std::uint64_t seed = 0;             ///< same seed, same tree
};

/**
 * @brief Result of a sampling planner. The path is empty when the goal was not reached.
 */
template <int Dim>
struct RrtResult {
	std::vector<Vec<Dim>> path;                             ///< start first, goal last; not smoothed
	double cost = std::numeric_limits<double>::infinity();  ///< length of the path
	std::size_t tree_size = 0;                              ///< nodes in the tree at the end

	bool found() const { return !path.empty(); }
};

/// The point at most `step_size` from `from` on the way to `to`.
template <int Dim>
Vec<Dim> steer(const Vec<Dim>& from, const Vec<Dim>& to, double step_size) {
	const Vec<Dim> delta = to - from;
	const double length = delta.norm();
	if (length <= step_size) return to;
	return from + delta * (step_size / length);
}

namespace detail {

/**
 * @brief Draws the random points of a sampling planner: uniform in the box, or the goal with
 * probability `goal_bias`, which pulls the tree towards the goal.
 */
template <int Dim>
class Sampler {
public:
	Sampler(Vec<Dim> lower, Vec<Dim> upper, Vec<Dim> goal, double goal_bias, std::uint64_t seed)
	    : m_lower(std::move(lower)), m_upper(std::move(upper)), m_goal(std::move(goal)), m_goal_bias(goal_bias), m_random(seed) {}

	Vec<Dim> next() {
		if (m_unit(m_random) < m_goal_bias) return m_goal;
		Vec<Dim> sample;
		for (int i = 0; i < Dim; ++i) sample[i] = m_lower[i] + m_unit(m_random) * (m_upper[i] - m_lower[i]);
		return sample;
	}

private:
	Vec<Dim> m_lower;
	Vec<Dim> m_upper;
	Vec<Dim> m_goal;
	double m_goal_bias;
	std::mt19937_64 m_random;
	std::uniform_real_distribution<double> m_unit{0.0, 1.0};
};

/// Index of the tree node closest to `p`. Compares squared distances: no square root is needed to
/// find the smallest.
template <class TreeNode, int Dim>
std::size_t nearest(const std::vector<TreeNode>& tree, const Vec<Dim>& p) {
	std::size_t best = 0;
	double best_distance = std::numeric_limits<double>::infinity();
	for (std::size_t i = 0; i < tree.size(); ++i) {
		const double d = (tree[i].point - p).squaredNorm();
		if (d < best_distance) {
			best_distance = d;
			best = i;
		}
	}
	return best;
}

/// Path from the root to `last` by following the parents, then on to `goal` if `last` is not it.
template <class TreeNode, int Dim>
std::vector<Vec<Dim>> trace_path(const std::vector<TreeNode>& tree, std::size_t last, const Vec<Dim>& goal) {
	std::vector<Vec<Dim>> path;
	if (tree[last].point != goal) path.push_back(goal);
	for (std::size_t node = last; node != 0; node = tree[node].parent) path.push_back(tree[node].point);
	path.push_back(tree[0].point);
	std::reverse(path.begin(), path.end());
	return path;
}

}

/**
 * @brief RRT: grows a tree from the start towards random points until it reaches the goal.
 *
 * Every iteration draws a point, finds the nearest node of the tree, and adds a node one step
 * from it towards the point if the segment is free. It stops at the first path found, which is
 * usually far from the shortest (see rrt_star()).
 */
template <int Dim, class S>
    requires SamplingSpace<S, Dim>
RrtResult<Dim> rrt(const S& space, const Vec<Dim>& start, const Vec<Dim>& goal, const RrtParams& params = {}) {
	struct TreeNode {
		Vec<Dim> point;
		std::size_t parent;  ///< index in `tree`, which stays valid when the vector reallocates
		double cost;         ///< length of the path from the start
	};

	RrtResult<Dim> result;
	if (!space.is_free(start) || !space.is_free(goal)) return result;

	detail::Sampler<Dim> sampler(space.lower_bound(), space.upper_bound(), goal, params.goal_bias, params.seed);
	std::vector<TreeNode> tree;
	tree.push_back({start, 0, 0.0});

	std::size_t reached = 0;
	bool goal_reached = (goal - start).norm() <= params.goal_tolerance && space.segment_free(start, goal);
	for (std::size_t iteration = 0; iteration < params.max_iterations && !goal_reached; ++iteration) {
		const Vec<Dim> sample = sampler.next();
		const std::size_t nearest = detail::nearest(tree, sample);
		const Vec<Dim> new_point = steer(tree[nearest].point, sample, params.step_size);
		if (new_point == tree[nearest].point || !space.segment_free(tree[nearest].point, new_point)) continue;

		tree.push_back({new_point, nearest, tree[nearest].cost + (new_point - tree[nearest].point).norm()});
		if ((goal - new_point).norm() <= params.goal_tolerance && space.segment_free(new_point, goal)) {
			reached = tree.size() - 1;
			goal_reached = true;
		}
	}
	result.tree_size = tree.size();
	if (!goal_reached) return result;

	result.cost = tree[reached].cost + (goal - tree[reached].point).norm();
	result.path = detail::trace_path(tree, reached, goal);
	return result;
}

}
