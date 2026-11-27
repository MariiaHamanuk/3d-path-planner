#include "planning/global/algorithms/a_star.hpp"
#include "planning/global/algorithms/heuristics.hpp"
#include "planning/global/algorithms/lazy_theta_star.hpp"
#include "planning/global/algorithms/rrt.hpp"
#include "planning/global/algorithms/rrt_connect.hpp"
#include "planning/global/algorithms/rrt_star.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>

using namespace path3d;

namespace {

/// Voxels 0..9 on the x axis, voxel 5 is blocked.
struct LineGraph {
	bool is_free(const Eigen::Vector3i& node) const {
		return node.y() == 0 && node.z() == 0 && node.x() >= 0 && node.x() <= 9 && node.x() != 5;
	}
	template <class Visit>
	void for_each_neighbor(const Eigen::Vector3i& node, Visit&& visit) const {
		for (const int dx : {-1, 1}) {
			const Eigen::Vector3i neighbor = node + Eigen::Vector3i(dx, 0, 0);
			if (is_free(neighbor)) visit(neighbor, 1.0);
		}
	}
};

/// Voxel `x` on the line.
Eigen::Vector3i on_line(int x) { return {x, 0, 0}; }

/// 20 x 20 voxels in the plane z = 0, 8-connected, with a wall at x = 10 for y < 15.
struct WallGrid {
	bool is_free(const Eigen::Vector3i& node) const {
		if (node.z() != 0 || (node.array() < 0).any() || node.x() >= 20 || node.y() >= 20) return false;
		return !(node.x() == 10 && node.y() < 15);
	}
	template <class Visit>
	void for_each_neighbor(const Eigen::Vector3i& node, Visit&& visit) const {
		for (int dx = -1; dx <= 1; ++dx) {
			for (int dy = -1; dy <= 1; ++dy) {
				if (dx == 0 && dy == 0) continue;
				const Eigen::Vector3i neighbor = node + Eigen::Vector3i(dx, dy, 0);
				if (is_free(neighbor)) visit(neighbor, std::sqrt(static_cast<double>(dx * dx + dy * dy)));
			}
		}
	}
};

/// Voxel (x, y) of the WallGrid plane.
Eigen::Vector3i cell(int x, int y) { return {x, y, 0}; }

/// The price of a straight line on a WallGrid: its length, or nothing when a voxel on it is blocked.
struct WallGridLine {
	std::optional<double> operator()(const Eigen::Vector3i& a, const Eigen::Vector3i& b) const {
		const Eigen::Vector3d delta = (b - a).cast<double>();
		const int samples = std::max(1, static_cast<int>(std::ceil(delta.norm() * 4.0)));
		for (int i = 1; i <= samples; ++i) {
			const Eigen::Vector3i voxel = (a.cast<double>() + delta * (static_cast<double>(i) / samples)).array().round().cast<int>().matrix();
			if (!WallGrid{}.is_free(voxel)) return std::nullopt;
		}
		return delta.norm();
	}
};

/// Length of a path of voxels.
double cells_length(const std::vector<Eigen::Vector3i>& path) {
	double length = 0.0;
	for (std::size_t i = 1; i < path.size(); ++i) length += (path[i] - path[i - 1]).cast<double>().norm();
	return length;
}

/// 10 x 10 m square in the plane z = 0 with a wall at 4 <= x <= 6 for y < 8: the way round is over
/// the top. The box has no height, so every random point is drawn in that plane.
struct WallSpace {
	bool is_free(const Eigen::Vector3d& p) const { return !(p.x() >= 4 && p.x() <= 6 && p.y() < 8); }
	bool segment_free(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const {
		for (int i = 0; i <= 100; ++i) {
			if (!is_free(a + (b - a) * (i / 100.0))) return false;
		}
		return true;
	}
	Eigen::Vector3d lower_bound() const { return Eigen::Vector3d(0, 0, 0); }
	Eigen::Vector3d upper_bound() const { return Eigen::Vector3d(10, 10, 0); }
};

/// Point (x, y) of the WallSpace plane.
Eigen::Vector3d at(double x, double y) { return {x, y, 0}; }

/// Shortest way round the WallSpace wall: up to its top corner, across, and down again.
const double kWallOptimum = 2 * std::hypot(3.0, 7.0) + 2.0;

void expect_free_path(const WallSpace& space, const std::vector<Eigen::Vector3d>& path) {
	for (std::size_t i = 1; i < path.size(); ++i) EXPECT_TRUE(space.segment_free(path[i - 1], path[i])) << "segment " << i;
}

}

TEST(AStar, FindsThePathOnALine) {
	const auto result = a_star(LineGraph{}, on_line(0), on_line(3), EuclideanHeuristic{});
	ASSERT_TRUE(result.found());
	EXPECT_EQ(result.path, (std::vector<Eigen::Vector3i>{on_line(0), on_line(1), on_line(2), on_line(3)}));
	EXPECT_DOUBLE_EQ(result.cost, 3.0);
}

TEST(AStar, ReportsNoPathBehindABlockedNode) {
	EXPECT_FALSE(a_star(LineGraph{}, on_line(0), on_line(7), EuclideanHeuristic{}).found());
}

TEST(AStar, ReportsNoPathFromABlockedStart) {
	EXPECT_FALSE(a_star(LineGraph{}, on_line(5), on_line(7), EuclideanHeuristic{}).found());
}

TEST(AStar, StopsAfterMaxExpansions) {
	EXPECT_FALSE(a_star(LineGraph{}, on_line(0), on_line(4), EuclideanHeuristic{}, 2).found());
	EXPECT_TRUE(a_star(LineGraph{}, on_line(0), on_line(4), EuclideanHeuristic{}, 4).found());
}

TEST(AStar, FindsTheSameCostAsDijkstraWithFewerExpansions) {
	const WallGrid grid;
	const Eigen::Vector3i start = cell(2, 2), goal = cell(18, 2);
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
	const Eigen::Vector3d start = at(1, 1), goal = at(9, 1);
	const auto result = rrt(space, start, goal);
	ASSERT_TRUE(result.found());
	EXPECT_EQ(result.path.front(), start);
	EXPECT_EQ(result.path.back(), goal);
	expect_free_path(space, result.path);
	EXPECT_GE(result.cost, kWallOptimum - 1e-9);
}

TEST(Rrt, GivesTheSamePathForTheSameSeed) {
	const Eigen::Vector3d start = at(1, 1), goal = at(9, 1);
	EXPECT_EQ(rrt(WallSpace{}, start, goal).path, rrt(WallSpace{}, start, goal).path);
}

TEST(Rrt, ReportsNoPathToAGoalInTheWall) {
	EXPECT_FALSE(rrt(WallSpace{}, at(1, 1), at(5, 5)).found());
}

TEST(RrtStar, FindsAShorterPathThanRrt) {
	const WallSpace space;
	const Eigen::Vector3d start = at(1, 1), goal = at(9, 1);
	const auto simple = rrt(space, start, goal);
	const auto star = rrt_star(space, start, goal);
	ASSERT_TRUE(star.found());
	EXPECT_EQ(star.path.front(), start);
	EXPECT_EQ(star.path.back(), goal);
	expect_free_path(space, star.path);
	EXPECT_LT(star.cost, simple.cost);
	EXPECT_LT(star.cost, kWallOptimum * 1.05);
}

TEST(LazyThetaStar, GoesStraightWhenNothingIsInTheWay) {
	const auto result = lazy_theta_star(WallGrid{}, cell(1, 16), cell(18, 19), EuclideanHeuristic{}, WallGridLine{});
	ASSERT_TRUE(result.found());
	EXPECT_EQ(result.path, (std::vector<Eigen::Vector3i>{cell(1, 16), cell(18, 19)}));
	EXPECT_NEAR(result.cost, std::hypot(17.0, 3.0), 1e-9);
}

TEST(LazyThetaStar, FindsAShorterPathThanAStarAroundTheWall) {
	const WallGrid grid;
	const Eigen::Vector3i start = cell(2, 2), goal = cell(18, 2);
	const auto theta = lazy_theta_star(grid, start, goal, EuclideanHeuristic{}, WallGridLine{});
	const auto grid_path = a_star(grid, start, goal, EuclideanHeuristic{});
	ASSERT_TRUE(theta.found());
	EXPECT_EQ(theta.path.front(), start);
	EXPECT_EQ(theta.path.back(), goal);
	EXPECT_LT(theta.cost, grid_path.cost);
	EXPECT_LT(theta.path.size(), grid_path.path.size());
	EXPECT_NEAR(theta.cost, cells_length(theta.path), 1e-9);
	for (std::size_t i = 1; i < theta.path.size(); ++i) EXPECT_TRUE(WallGridLine{}(theta.path[i - 1], theta.path[i])) << "segment " << i;
}

TEST(LazyThetaStar, ReportsNoPathToABlockedGoal) {
	EXPECT_FALSE(lazy_theta_star(WallGrid{}, cell(2, 2), cell(10, 5), EuclideanHeuristic{}, WallGridLine{}).found());
}

TEST(RrtConnect, FindsAPathAroundTheWall) {
	const WallSpace space;
	const Eigen::Vector3d start = at(1, 1), goal = at(9, 1);
	const auto result = rrt_connect(space, start, goal);
	ASSERT_TRUE(result.found());
	EXPECT_EQ(result.path.front(), start);
	EXPECT_EQ(result.path.back(), goal);
	expect_free_path(space, result.path);
	double length = 0.0;
	for (std::size_t i = 1; i < result.path.size(); ++i) length += (result.path[i] - result.path[i - 1]).norm();
	EXPECT_NEAR(result.cost, length, 1e-9);
}

TEST(RrtConnect, GoesStraightWhenTheLineIsFree) {
	const auto result = rrt_connect(WallSpace{}, at(1, 9), at(9, 9));
	ASSERT_TRUE(result.found());
	EXPECT_EQ(result.path.size(), 2u);
}

TEST(RrtConnect, GivesTheSamePathForTheSameSeed) {
	const Eigen::Vector3d start = at(1, 1), goal = at(9, 1);
	EXPECT_EQ(rrt_connect(WallSpace{}, start, goal).path, rrt_connect(WallSpace{}, start, goal).path);
}

TEST(RrtConnect, ReportsNoPathToAGoalInTheWall) {
	EXPECT_FALSE(rrt_connect(WallSpace{}, at(1, 1), at(5, 5)).found());
}

TEST(RrtStar, CostMatchesThePathLength) {
	const auto result = rrt_star(WallSpace{}, at(1, 1), at(9, 1));
	ASSERT_TRUE(result.found());
	double length = 0.0;
	for (std::size_t i = 1; i < result.path.size(); ++i) length += (result.path[i] - result.path[i - 1]).norm();
	EXPECT_NEAR(result.cost, length, 1e-9);
}
