#pragma once

#include "planning/algorithms/rrt.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace path3d {

/**
 * @brief Settings of rrt_star().
 */
struct RrtStarParams {
	std::size_t max_iterations = 5000;  ///< samples drawn; all of them are used, the path keeps improving
	double step_size = 1.0;             ///< longest new edge, m
	double goal_bias = 0.05;            ///< share of samples that are the goal itself
	double goal_tolerance = 1.0;        ///< a node this close to the goal, with a free segment to it, reaches it
	double rewire_radius = 2.0;         ///< neighbors of a new node, m; larger than step_size, larger is better but slower
	std::uint64_t seed = 0;             ///< same seed, same tree
};

/**
 * @brief RRT*: RRT that keeps improving its path.
 *
 * Grows the tree as rrt() does, with two more steps for every new node, both among the tree
 * nodes within `rewire_radius` of it (its neighbors):
 *   1. the new node takes as parent the neighbor through which its path from the start is
 *      shortest, not just the nearest node;
 *   2. every neighbor whose path gets shorter through the new node switches its parent to it
 *      (rewiring), and the costs of its whole subtree drop by the same amount.
 * It runs all iterations and returns the shortest path among the nodes that reach the goal.
 */
template <int Dim, class S>
    requires SamplingSpace<S, Dim>
RrtResult<Dim> rrt_star(const S& space, const Vec<Dim>& start, const Vec<Dim>& goal, const RrtStarParams& params = {}) {
	struct TreeNode {
		Vec<Dim> point;
		std::size_t parent;
		double cost;
		std::vector<std::size_t> children;  ///< needed to update the subtree after rewiring
	};

	RrtResult<Dim> result;
	if (!space.is_free(start) || !space.is_free(goal)) return result;

	detail::Sampler<Dim> sampler(space.lower_bound(), space.upper_bound(), goal, params.goal_bias, params.seed);
	const double radius_squared = params.rewire_radius * params.rewire_radius;
	std::vector<TreeNode> tree;
	tree.push_back({start, 0, 0.0, {}});
	std::vector<std::size_t> near;
	std::vector<std::size_t> subtree;

	for (std::size_t iteration = 0; iteration < params.max_iterations; ++iteration) {
		const Vec<Dim> sample = sampler.next();
		const std::size_t nearest = detail::nearest(tree, sample);
		const Vec<Dim> new_point = steer(tree[nearest].point, sample, params.step_size);
		if (new_point == tree[nearest].point || !space.segment_free(tree[nearest].point, new_point)) continue;

		near.clear();
		for (std::size_t i = 0; i < tree.size(); ++i) {
			if ((tree[i].point - new_point).squaredNorm() <= radius_squared) near.push_back(i);
		}

		// 1. the cheapest parent; the cost is compared first, the more expensive segment check only if it pays off
		std::size_t parent = nearest;
		double new_cost = tree[nearest].cost + (new_point - tree[nearest].point).norm();
		for (std::size_t i : near) {
			const double cost = tree[i].cost + (new_point - tree[i].point).norm();
			if (cost < new_cost && space.segment_free(tree[i].point, new_point)) {
				parent = i;
				new_cost = cost;
			}
		}
		const std::size_t new_index = tree.size();
		tree.push_back({new_point, parent, new_cost, {}});
		tree[parent].children.push_back(new_index);

		// 2. rewiring: neighbors that are closer to the start through the new node
		for (std::size_t i : near) {
			if (i == parent) continue;
			const double cost = new_cost + (tree[i].point - new_point).norm();
			if (cost >= tree[i].cost || !space.segment_free(new_point, tree[i].point)) continue;

			auto& old_siblings = tree[tree[i].parent].children;
			old_siblings.erase(std::find(old_siblings.begin(), old_siblings.end(), i));
			tree[i].parent = new_index;
			tree[new_index].children.push_back(i);

			const double delta = cost - tree[i].cost;
			subtree.assign(1, i);
			while (!subtree.empty()) {
				const std::size_t node = subtree.back();
				subtree.pop_back();
				tree[node].cost += delta;
				subtree.insert(subtree.end(), tree[node].children.begin(), tree[node].children.end());
			}
		}
	}
	result.tree_size = tree.size();

	// The costs changed while rewiring, so the best node near the goal is chosen only now.
	std::size_t best = 0;
	for (std::size_t i = 0; i < tree.size(); ++i) {
		const double to_goal = (goal - tree[i].point).norm();
		if (to_goal > params.goal_tolerance) continue;
		const double cost = tree[i].cost + to_goal;
		if (cost < result.cost && space.segment_free(tree[i].point, goal)) {
			result.cost = cost;
			best = i;
		}
	}
	if (result.cost == std::numeric_limits<double>::infinity()) return result;

	result.path = detail::trace_path(tree, best, goal);
	return result;
}

}
