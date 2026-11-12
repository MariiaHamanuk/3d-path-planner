#include "planning/algorithms/a_star.hpp"
#include "planning/algorithms/heuristics.hpp"
#include "planning/algorithms/lazy_theta_star.hpp"
#include "planning/algorithms/rrt.hpp"
#include "planning/algorithms/rrt_connect.hpp"
#include "planning/algorithms/rrt_star.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <optional>

using namespace path3d;

namespace {

/// Nodes 0..9 on a line, node 5 is blocked.
struct LineGraph {
	using Node = int;
	using NodeHash = std::hash<int>;

	bool is_free(int node) const { return node >= 0 && node <= 9 && node != 5; }
	template <class Visit>
	void for_each_neighbor(int node, Visit&& visit) const {
		if (is_free(node - 1)) visit(node - 1, 1.0);
		if (is_free(node + 1)) visit(node + 1, 1.0);
	}
};

const auto line_distance = [](int a, int b) { return static_cast<double>(std::abs(a - b)); };

/// 20 x 20 grid, 8-connected, with a wall at x = 10 for y < 15.
struct WallGrid {
	using Node = Eigen::Vector2i;
	struct NodeHash {
		std::size_t operator()(const Node& node) const { return std::hash<int>()(node.x() * 1000 + node.y()); }
	};

	bool is_free(const Node& node) const {
		if ((node.array() < 0).any() || (node.array() >= 20).any()) return false;
		return !(node.x() == 10 && node.y() < 15);
	}
	template <class Visit>
	void for_each_neighbor(const Node& node, Visit&& visit) const {
		for (int dx = -1; dx <= 1; ++dx) {
			for (int dy = -1; dy <= 1; ++dy) {
				if (dx == 0 && dy == 0) continue;
				const Node neighbor = node + Node(dx, dy);
				if (is_free(neighbor)) visit(neighbor, std::sqrt(static_cast<double>(dx * dx + dy * dy)));
			}
		}
	}
	std::optional<double> segment_cost(const Node& a, const Node& b) const {
		const Eigen::Vector2d delta = (b - a).cast<double>();
		const int samples = std::max(1, static_cast<int>(std::ceil(delta.norm() * 4.0)));
		for (int i = 1; i <= samples; ++i) {
			const Node cell = (a.cast<double>() + delta * (static_cast<double>(i) / samples)).array().round().cast<int>().matrix();
			if (!is_free(cell)) return std::nullopt;
		}
		return delta.norm();
	}
	double straight_cost(const Node& a, const Node& b) const { return (b - a).cast<double>().norm(); }
};

/// Length of a path of grid cells.
double cells_length(const std::vector<WallGrid::Node>& path) {
	double length = 0.0;
	for (std::size_t i = 1; i < path.size(); ++i) length += (path[i] - path[i - 1]).cast<double>().norm();
	return length;
}

/// 10 x 10 m square with a wall at 4 <= x <= 6 for y < 8: the way round is over the top.
struct WallSpace {
	bool is_free(const Vec<2>& p) const { return !(p.x() >= 4 && p.x() <= 6 && p.y() < 8); }
	bool segment_free(const Vec<2>& a, const Vec<2>& b) const {
		for (int i = 0; i <= 100; ++i) {
			if (!is_free(a + (b - a) * (i / 100.0))) return false;
		}
		return true;
	}
	Vec<2> lower_bound() const { return Vec<2>(0, 0); }
	Vec<2> upper_bound() const { return Vec<2>(10, 10); }
};

/// Shortest way round the WallSpace wall: up to its top corner, across, and down again.
const double kWallOptimum = 2 * std::hypot(3.0, 7.0) + 2.0;

void expect_free_path(const WallSpace& space, const std::vector<Vec<2>>& path) {
	for (std::size_t i = 1; i < path.size(); ++i) EXPECT_TRUE(space.segment_free(path[i - 1], path[i])) << "segment " << i;
}

}

TEST(AStar, FindsThePathOnALine) {
	const auto result = a_star(LineGraph{}, 0, 3, line_distance);
	ASSERT_TRUE(result.found());
	EXPECT_EQ(result.path, (std::vector<int>{0, 1, 2, 3}));
	EXPECT_DOUBLE_EQ(result.cost, 3.0);
}

TEST(AStar, ReportsNoPathBehindABlockedNode) {
	EXPECT_FALSE(a_star(LineGraph{}, 0, 7, line_distance).found());
}

TEST(AStar, ReportsNoPathFromABlockedStart) {
	EXPECT_FALSE(a_star(LineGraph{}, 5, 7, line_distance).found());
}

TEST(AStar, StopsAfterMaxExpansions) {
	EXPECT_FALSE(a_star(LineGraph{}, 0, 4, line_distance, 2).found());
	EXPECT_TRUE(a_star(LineGraph{}, 0, 4, line_distance, 4).found());
}

TEST(AStar, FindsTheSameCostAsDijkstraWithFewerExpansions) {
	const WallGrid grid;
	const WallGrid::Node start(2, 2), goal(18, 2);
	const auto a = a_star(grid, start, goal, EuclideanHeuristic{});
	const auto d = dijkstra(grid, start, goal);
	ASSERT_TRUE(a.found());
	ASSERT_TRUE(d.found());
	EXPECT_NEAR(a.cost, d.cost, 1e-9);
	EXPECT_LT(a.expanded, d.expanded);
	for (const auto& node : a.path) EXPECT_TRUE(grid.is_free(node));
}

TEST(Heuristics, OctileIsTheExactCostOnAnEmptyGrid) {
	// (3, 2, 1): one space diagonal, one face diagonal, one straight step
	EXPECT_NEAR(OctileHeuristic{}(Eigen::Vector3i(0, 0, 0), Eigen::Vector3i(3, -2, 1)), std::sqrt(3.0) + std::sqrt(2.0) + 1.0, 1e-12);
	EXPECT_DOUBLE_EQ(EuclideanHeuristic{}(Eigen::Vector3i(0, 0, 0), Eigen::Vector3i(3, 4, 0)), 5.0);
	EXPECT_DOUBLE_EQ(ManhattanHeuristic{}(Eigen::Vector3i(0, 0, 0), Eigen::Vector3i(3, -4, 1)), 8.0);
}

TEST(Rrt, FindsAPathAroundTheWall) {
	const WallSpace space;
	const Vec<2> start(1, 1), goal(9, 1);
	const auto result = rrt<2>(space, start, goal);
	ASSERT_TRUE(result.found());
	EXPECT_EQ(result.path.front(), start);
	EXPECT_EQ(result.path.back(), goal);
	expect_free_path(space, result.path);
	EXPECT_GE(result.cost, kWallOptimum - 1e-9);
}

TEST(Rrt, GivesTheSamePathForTheSameSeed) {
	const Vec<2> start(1, 1), goal(9, 1);
	EXPECT_EQ(rrt<2>(WallSpace{}, start, goal).path, rrt<2>(WallSpace{}, start, goal).path);
}

TEST(Rrt, ReportsNoPathToAGoalInTheWall) {
	EXPECT_FALSE(rrt<2>(WallSpace{}, Vec<2>(1, 1), Vec<2>(5, 5)).found());
}

TEST(RrtStar, FindsAShorterPathThanRrt) {
	const WallSpace space;
	const Vec<2> start(1, 1), goal(9, 1);
	const auto simple = rrt<2>(space, start, goal);
	const auto star = rrt_star<2>(space, start, goal);
	ASSERT_TRUE(star.found());
	EXPECT_EQ(star.path.front(), start);
	EXPECT_EQ(star.path.back(), goal);
	expect_free_path(space, star.path);
	EXPECT_LT(star.cost, simple.cost);
	EXPECT_LT(star.cost, kWallOptimum * 1.05);
}

TEST(LazyThetaStar, GoesStraightWhenNothingIsInTheWay) {
	const auto result = lazy_theta_star(WallGrid{}, WallGrid::Node(1, 16), WallGrid::Node(18, 19), EuclideanHeuristic{});
	ASSERT_TRUE(result.found());
	EXPECT_EQ(result.path, (std::vector<WallGrid::Node>{{1, 16}, {18, 19}}));
	EXPECT_NEAR(result.cost, std::hypot(17.0, 3.0), 1e-9);
}

TEST(LazyThetaStar, FindsAShorterPathThanAStarAroundTheWall) {
	const WallGrid grid;
	const WallGrid::Node start(2, 2), goal(18, 2);
	const auto theta = lazy_theta_star(grid, start, goal, EuclideanHeuristic{});
	const auto grid_path = a_star(grid, start, goal, EuclideanHeuristic{});
	ASSERT_TRUE(theta.found());
	EXPECT_EQ(theta.path.front(), start);
	EXPECT_EQ(theta.path.back(), goal);
	EXPECT_LT(theta.cost, grid_path.cost);
	EXPECT_LT(theta.path.size(), grid_path.path.size());
	EXPECT_NEAR(theta.cost, cells_length(theta.path), 1e-9);
	for (std::size_t i = 1; i < theta.path.size(); ++i) EXPECT_TRUE(grid.segment_cost(theta.path[i - 1], theta.path[i])) << "segment " << i;
}

TEST(LazyThetaStar, ReportsNoPathToABlockedGoal) {
	EXPECT_FALSE(lazy_theta_star(WallGrid{}, WallGrid::Node(2, 2), WallGrid::Node(10, 5), EuclideanHeuristic{}).found());
}

TEST(RrtConnect, FindsAPathAroundTheWall) {
	const WallSpace space;
	const Vec<2> start(1, 1), goal(9, 1);
	const auto result = rrt_connect<2>(space, start, goal);
	ASSERT_TRUE(result.found());
	EXPECT_EQ(result.path.front(), start);
	EXPECT_EQ(result.path.back(), goal);
	expect_free_path(space, result.path);
	double length = 0.0;
	for (std::size_t i = 1; i < result.path.size(); ++i) length += (result.path[i] - result.path[i - 1]).norm();
	EXPECT_NEAR(result.cost, length, 1e-9);
}

TEST(RrtConnect, GoesStraightWhenTheLineIsFree) {
	const auto result = rrt_connect<2>(WallSpace{}, Vec<2>(1, 9), Vec<2>(9, 9));
	ASSERT_TRUE(result.found());
	EXPECT_EQ(result.path.size(), 2u);
}

TEST(RrtConnect, GivesTheSamePathForTheSameSeed) {
	const Vec<2> start(1, 1), goal(9, 1);
	EXPECT_EQ(rrt_connect<2>(WallSpace{}, start, goal).path, rrt_connect<2>(WallSpace{}, start, goal).path);
}

TEST(RrtConnect, ReportsNoPathToAGoalInTheWall) {
	EXPECT_FALSE(rrt_connect<2>(WallSpace{}, Vec<2>(1, 1), Vec<2>(5, 5)).found());
}

TEST(RrtStar, CostMatchesThePathLength) {
	const auto result = rrt_star<2>(WallSpace{}, Vec<2>(1, 1), Vec<2>(9, 1));
	ASSERT_TRUE(result.found());
	double length = 0.0;
	for (std::size_t i = 1; i < result.path.size(); ++i) length += (result.path[i] - result.path[i - 1]).norm();
	EXPECT_NEAR(result.cost, length, 1e-9);
}
