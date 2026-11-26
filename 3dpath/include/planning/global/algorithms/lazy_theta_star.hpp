#pragma once

#include "planning/global/algorithms/a_star.hpp"

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <limits>
#include <functional>
#include <optional>
#include <queue>
#include <unordered_map>
#include <vector>

namespace path3d {

/**
 * @brief The price of flying straight from one node to another: the cost of the line, or nullopt
 * when something is in the way. Any-angle search (lazy_theta_star()) takes it next to the graph,
 * because only the code that knows how nodes lie in space can walk along a line between them.
 * The cost of a line must not be below what the heuristic says for the same two nodes.
 */
template <class L, class Node>
concept LineCost = std::regular_invocable<const L&, const Node&, const Node&> &&
                   std::same_as<std::invoke_result_t<const L&, const Node&, const Node&>, std::optional<double>>;

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
 * OctileHeuristic, which is exact only for paths along the grid. It is also the price of a line
 * that has not been checked yet.
 *
 * @param line_cost Checks a straight line and prices it (see LineCost).
 * @param max_expansions Gives up after this many expanded nodes.
 */
template <SearchGraph G, class H, class L>
    requires SearchHeuristic<H, typename G::Node> && LineCost<L, typename G::Node>
SearchResult<typename G::Node> lazy_theta_star(const G& graph, const typename G::Node& start, const typename G::Node& goal,
                                               const H& heuristic, const L& line_cost,
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
			const std::optional<double> straight = line_cost(visit.parent, current.node);
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
			                                               : parent_cost + heuristic(parent, neighbor);
			if (found != visits.end() && new_cost >= found->second.cost_from_start) return;
			visits.insert_or_assign(neighbor, Visit{new_cost, parent == current.node ? current.node : parent});
			open.push({new_cost + heuristic(neighbor, goal), new_cost, neighbor});
		});
	}
	return result;
}

}
