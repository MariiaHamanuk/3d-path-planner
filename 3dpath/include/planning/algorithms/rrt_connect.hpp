#pragma once

#include "planning/algorithms/rrt.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <tuple>
#include <utility>
#include <vector>

namespace path3d {

/**
 * @brief Settings of rrt_connect().
 */
struct RrtConnectParams {
	std::size_t max_iterations = 5000;  ///< samples drawn before giving up
	double step_size = 1.0;             ///< longest new edge, m
	std::uint64_t seed = 0;             ///< same seed, same trees
};

/**
 * @brief RRT-Connect: two RRT trees, one from the start and one from the goal, that grow towards
 * each other.
 *
 * Every iteration one tree takes a step towards a random point (as in rrt()), and the other tree
 * then steps towards the new node again and again until it reaches it or hits an obstacle. When
 * it reaches it, the trees are connected and the path is found. Then the trees swap roles.
 * It usually finds a path much faster than RRT, and the path is as crooked.
 */
template <int Dim, class S>
    requires SamplingSpace<S, Dim>
RrtResult<Dim> rrt_connect(const S& space, const Vec<Dim>& start, const Vec<Dim>& goal, const RrtConnectParams& params = {}) {
	struct TreeNode {
		Vec<Dim> point;
		std::size_t parent;
	};
	using Tree = std::vector<TreeNode>;
	enum class Extend { Trapped, Advanced, Reached };

	RrtResult<Dim> result;
	if (!space.is_free(start) || !space.is_free(goal)) return result;

	/// One step of `tree` towards `target`; the second value is the node that got closest.
	const auto extend = [&](Tree& tree, const Vec<Dim>& target) -> std::pair<Extend, std::size_t> {
		const std::size_t nearest = detail::nearest(tree, target);
		if (tree[nearest].point == target) return {Extend::Reached, nearest};
		const Vec<Dim> new_point = steer(tree[nearest].point, target, params.step_size);
		if (!space.segment_free(tree[nearest].point, new_point)) return {Extend::Trapped, nearest};
		tree.push_back({new_point, nearest});
		return {new_point == target ? Extend::Reached : Extend::Advanced, tree.size() - 1};
	};
	/// Points from `node` up to the root of `tree`.
	const auto to_root = [](const Tree& tree, std::size_t node) {
		std::vector<Vec<Dim>> points{tree[node].point};
		for (; node != 0; node = tree[node].parent) points.push_back(tree[tree[node].parent].point);
		return points;
	};

	std::array<Tree, 2> trees{Tree{{start, 0}}, Tree{{goal, 0}}};  ///< 0 grows from the start, 1 from the goal
	detail::Sampler<Dim> sampler(space.lower_bound(), space.upper_bound(), goal, 0.0, params.seed);

	bool connected = space.segment_free(start, goal);
	std::size_t meet_start = 0;  ///< the meeting node in the start tree
	std::size_t meet_goal = 0;   ///< the same point in the goal tree
	std::size_t growing = 0;     ///< the tree that takes the random step
	for (std::size_t iteration = 0; iteration < params.max_iterations && !connected; ++iteration) {
		const auto [status, new_node] = extend(trees[growing], sampler.next());
		if (status != Extend::Trapped) {
			const Vec<Dim> target = trees[growing][new_node].point;
			Extend reach = Extend::Advanced;
			std::size_t other_node = 0;
			while (reach == Extend::Advanced) std::tie(reach, other_node) = extend(trees[1 - growing], target);
			if (reach == Extend::Reached) {
				connected = true;
				meet_start = growing == 0 ? new_node : other_node;
				meet_goal = growing == 0 ? other_node : new_node;
			}
		}
		growing = 1 - growing;
	}
	result.tree_size = trees[0].size() + trees[1].size();
	if (!connected) return result;

	if (meet_start == 0 && meet_goal == 0) {
		result.path = {start, goal};  // a straight free line from the start
	} else {
		result.path = to_root(trees[0], meet_start);
		std::reverse(result.path.begin(), result.path.end());
		const std::vector<Vec<Dim>> rest = to_root(trees[1], meet_goal);
		result.path.insert(result.path.end(), rest.begin() + 1, rest.end());  // the meeting point is in both
	}
	result.cost = 0.0;
	for (std::size_t i = 1; i < result.path.size(); ++i) result.cost += (result.path[i] - result.path[i - 1]).norm();
	return result;
}

}
