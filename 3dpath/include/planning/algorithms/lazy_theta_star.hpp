#pragma once

#include "planning/algorithms/a_star.hpp"

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <limits>
#include <optional>
#include <queue>
#include <unordered_map>
#include <vector>

namespace path3d {

/**
 * @brief A SearchGraph that can also fly straight between any two nodes, which any-angle search
 * (lazy_theta_star()) needs:
 *   - segment_cost(a, b): the cost of the straight line from a to b, or nullopt when something
 *     is in the way;
 *   - straight_cost(a, b): the same cost without looking at the map, never more than
 *     segment_cost(a, b). Cheap, used before a line is checked.
 */
template <class G>
concept AnyAngleGraph = SearchGraph<G> && requires(const G& graph, const typename G::Node& node) {
	{ graph.segment_cost(node, node) } -> std::same_as<std::optional<double>>;
	{ graph.straight_cost(node, node) } -> std::convertible_to<double>;
};

/**
 * @brief Lazy Theta*: A* whose path is not tied to the grid.
 *
 * A* links every node to the neighbor it came from, so its path turns only at 45 degree steps.
 * Theta* lets a node take the parent of that neighbor instead, when the straight line between
 * them is free; the path then goes at any angle, and is shorter, with fewer waypoints.
 *
 * "Lazy": the line is not checked when a node is reached, only when it is taken out of the
 * queue. If it turns out blocked then, the node falls back to the best of its already expanded
 * neighbors, as in A*. This checks far fewer lines, because most reached nodes are never expanded.
 *
 * The heuristic must not overestimate straight lines: use EuclideanHeuristic, not
 * OctileHeuristic, which is exact only for paths along the grid.
 *
 * @param max_expansions Gives up after this many expanded nodes.
 */
template <AnyAngleGraph G, class H>
    requires SearchHeuristic<H, typename G::Node>
SearchResult<typename G::Node> lazy_theta_star(const G& graph, const typename G::Node& start, const typename G::Node& goal,
                                               const H& heuristic,
                                               std::size_t max_expansions = std::numeric_limits<std::size_t>::max()) {
	using Node = typename G::Node;
	struct Visit {
		double cost_from_start;
		Node parent;
		bool expanded = false;
	};
	struct QueueEntry {
		double estimated_total;
		double cost_from_start;
		Node node;
		bool operator<(const QueueEntry& other) const { return estimated_total > other.estimated_total; }
	};

	SearchResult<Node> result;
	if (!graph.is_free(start) || !graph.is_free(goal)) return result;

	std::unordered_map<Node, Visit, typename G::NodeHash> visits;
	std::priority_queue<QueueEntry> open;
	visits.insert({start, Visit{0.0, start}});
	open.push({heuristic(start, goal), 0.0, start});

	while (!open.empty()) {
		const QueueEntry current = open.top();
		open.pop();
		Visit& visit = visits.at(current.node);
		if (visit.expanded || current.cost_from_start > visit.cost_from_start) continue;

		// The lazy check: was the straight line from the parent really free, and how much does it cost?
		if (visit.parent != current.node) {
			const std::optional<double> straight = graph.segment_cost(visit.parent, current.node);
			double best = straight ? visits.at(visit.parent).cost_from_start + *straight : std::numeric_limits<double>::infinity();
			Node best_parent = visit.parent;
			graph.for_each_neighbor(current.node, [&](const Node& neighbor, double step_cost) {
				const auto found = visits.find(neighbor);
				if (found == visits.end() || !found->second.expanded) return;
				if (found->second.cost_from_start + step_cost < best) {
					best = found->second.cost_from_start + step_cost;
					best_parent = neighbor;
				}
			});
			visit.cost_from_start = best;
			visit.parent = best_parent;
		}

		if (current.node == goal) {
			result.cost = visit.cost_from_start;
			for (Node node = goal; node != start; node = visits.at(node).parent) result.path.push_back(node);
			result.path.push_back(start);
			std::reverse(result.path.begin(), result.path.end());
			return result;
		}
		if (result.expanded == max_expansions) break;
		++result.expanded;
		visit.expanded = true;

		// Every neighbor is first linked straight to this node's parent, and checked when it is expanded.
		const Node parent = visit.parent;
		const double parent_cost = visits.at(parent).cost_from_start;
		graph.for_each_neighbor(current.node, [&](const Node& neighbor, double step_cost) {
			const auto found = visits.find(neighbor);
			if (found != visits.end() && found->second.expanded) return;
			const double new_cost = parent == current.node ? parent_cost + step_cost
			                                               : parent_cost + graph.straight_cost(parent, neighbor);
			if (found != visits.end() && new_cost >= found->second.cost_from_start) return;
			visits.insert_or_assign(neighbor, Visit{new_cost, parent == current.node ? current.node : parent});
			open.push({new_cost + heuristic(neighbor, goal), new_cost, neighbor});
		});
	}
	return result;
}

}
