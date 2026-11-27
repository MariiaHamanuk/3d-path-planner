#pragma once

#include "core/types.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <limits>
#include <queue>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace path3d {

/**
 * @brief A graph of voxels that A* can search: whether a voxel is free, and its free neighbors with
 * the cost of the step to each. The neighbors come through a callback, not a vector, so an
 * expansion allocates nothing.
 */
template <class G>
concept SearchGraph = requires(const G& graph, const Eigen::Vector3i& node) {
	{ graph.is_free(node) } -> std::convertible_to<bool>;
	graph.for_each_neighbor(node, [](const Eigen::Vector3i&, double) {});
};

/// Estimate of the remaining cost between two voxels. A* finds the cheapest path only when the
/// heuristic does not overestimate.
template <class H>
concept SearchHeuristic = std::regular_invocable<const H&, const Eigen::Vector3i&, const Eigen::Vector3i&> &&
                          std::convertible_to<std::invoke_result_t<const H&, const Eigen::Vector3i&, const Eigen::Vector3i&>, double>;

/**
 * @brief Result of a graph search (path is empty when the goal was not reached)
 */
struct SearchResult {
	std::vector<Eigen::Vector3i> path;                       ///< start first, goal last
	double cost = std::numeric_limits<double>::infinity();  ///< sum of the step costs of the path
	std::size_t expanded = 0;                               ///< nodes whose neighbors were looked at

	bool found() const { return !path.empty(); }
};

/**
 * @brief A* search from start to goal
 *
 * The queue is ordered by f = g + h, where g is the cost from the start and h the heuristic.
 * A node can be in the queue several times. Entry whose g is higher than the best known one
 * is skipped. The search stops when the goal is taken out of the queue
 * (cheaper way to it may still be in the queue)
 *
 * @param max_expansions Gives up after this many nodes (just not to go into infinity)
 */
template <SearchGraph G, SearchHeuristic H>
SearchResult a_star(const G& graph, const Eigen::Vector3i& start, const Eigen::Vector3i& goal, const H& heuristic,
                    std::size_t max_expansions = std::numeric_limits<std::size_t>::max()) {
	using Node = Eigen::Vector3i;
	struct Visit {
		double cost_from_start;
		Node parent;
	};
	struct QueueEntry {
		double estimated_total;
		double cost_from_start;
		Node node;
		/// priority_queue gives the largest element first (reversed so it gives the smallest f)
		bool operator<(const QueueEntry& other) const { return estimated_total > other.estimated_total; }
	};

	SearchResult result;
	if (!graph.is_free(start) || !graph.is_free(goal)) return result;

	std::unordered_map<Node, Visit, VoxelHash> visits;
	std::priority_queue<QueueEntry> open;
	visits.insert({start, Visit{0.0, start}});
	open.push({heuristic(start, goal), 0.0, start});

	while (!open.empty()) {
		const QueueEntry current = open.top();
		open.pop();
		if (current.cost_from_start > visits.at(current.node).cost_from_start) continue;

		if (current.node == goal) {
			result.cost = current.cost_from_start;
			for (Node node = goal; node != start; node = visits.at(node).parent) result.path.push_back(node);
			result.path.push_back(start);
			std::reverse(result.path.begin(), result.path.end());
			return result;
		}
		if (result.expanded == max_expansions) break;
		++result.expanded;

		graph.for_each_neighbor(current.node, [&](const Node& neighbor, double step_cost) {
			const double new_cost = current.cost_from_start + step_cost;
			const auto found = visits.find(neighbor);
			if (found != visits.end() && new_cost >= found->second.cost_from_start) return;
			visits.insert_or_assign(neighbor, Visit{new_cost, current.node});
			open.push({new_cost + heuristic(neighbor, goal), new_cost, neighbor});
		});
	}
	return result;
}

struct ZeroHeuristic {
	double operator()(const Eigen::Vector3i&, const Eigen::Vector3i&) const { return 0.0; }
};

/**
 * @brief Baseline for comparing heuristics. 
 * 
 * Dijkstra search: A* without a heuristic. Finds the same cost as A*, expands more nodes.
 */
template <SearchGraph G>
SearchResult dijkstra(const G& graph, const Eigen::Vector3i& start, const Eigen::Vector3i& goal,
                      std::size_t max_expansions = std::numeric_limits<std::size_t>::max()) {
	return a_star(graph, start, goal, ZeroHeuristic{}, max_expansions);
}

}
