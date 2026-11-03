#pragma once

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

#include "planner/point.hpp"
namespace planner{
template <std::size_t Dim>
using Vec = Point<double, Dim>;

template <typename M, std::size_t Dim>
concept SamplingMap = requires(const M& map, const Vec<Dim>& p) { // те за що говорив Макс - домовляємося з картою
    { map.isFree(p) } -> std::convertible_to<bool>; // нам все одно яка карта але маємо знати чи точка/відрізок вільні і межі карти
    { map.segmentFree(p, p) } -> std::convertible_to<bool>;
    { map.lowerBound() } -> std::convertible_to<Vec<Dim>>;
    { map.upperBound() } -> std::convertible_to<Vec<Dim>>;
};

struct RrtParams {
    std::size_t maxIterations = 5000;
    double stepSize = 1.0; // макс. довжина 1 гілки
    double goalBias = 0.05; // відсоток скільки із спроб ми будемо прямувати прямо до цілі, щоб дерево не росло однаково скрізь а все ж таки 
    // //"більшала ймовірність" потягнутися ближче до цілі і аби воно не ранилося до ранку
    double goalTolerance = 1.0; // після якої відстані ми йдемо вже прямо до цілі щоб не віляти навколо неї а був норм конвьордженс в ціль
    std::uint64_t seed = 0;
};

template <std::size_t Dim>
struct RrtResult {
    std::vector<Vec<Dim>> path; // НЕЗГЛАДЖЕНІ точки 
    double cost = std::numeric_limits<double>::infinity();
    std::size_t treeSize = 0;

    bool found() const { return !path.empty(); }
};
template <std::size_t Dim>
Vec<Dim> steer(const Vec<Dim>& from, const Vec<Dim>& to, double stepSize) {
    const double length = distance(from, to);
    if (length <= stepSize) {
        return to;
    }
    Vec<Dim> result;
    for (std::size_t i = 0; i < Dim; ++i) {
        result[i] = from[i] + (to[i] - from[i]) * (stepSize / length);
    }
    return result;
}
template <std::size_t Dim, typename Map>
    requires SamplingMap<Map, Dim>
RrtResult<Dim> rrt(const Map& map, const Vec<Dim>& start, const Vec<Dim>& goal, const RrtParams& params = {}) {
    struct TreeNode {
        Vec<Dim> point;
        std::size_t parent;  //дуже важливе уточення що це НОМЕР баті у векторі а не його координати поінтер чи ще щось
        double cost; // довжина
    };

    RrtResult<Dim> result;
    if (!map.isFree(start) || !map.isFree(goal)) {
        return result;
    }

    std::mt19937_64 random(params.seed);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const Vec<Dim> lowest = map.lowerBound();
    const Vec<Dim> highest = map.upperBound();

    std::vector<TreeNode> tree;
    tree.push_back({start, 0, 0.0});

    std::size_t reached = 0;
    bool goalReached = distance(start, goal) <= params.goalTolerance && map.segmentFree(start, goal);
    for (std::size_t iteration = 0; iteration < params.maxIterations && !goalReached; ++iteration) {
        Vec<Dim> sample = goal;
        if (unit(random) >= params.goalBias) {
            for (std::size_t i = 0; i < Dim; ++i) {
                sample[i] = lowest[i] + unit(random) * (highest[i] - lowest[i]);
            }
        }

        std::size_t nearest = 0;
        double nearestDistance = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < tree.size(); ++i) {
            const double d = squaredDistance(tree[i].point, sample);
            if (d < nearestDistance) {
                nearestDistance = d;
                nearest = i;
            }
        }

        const Vec<Dim> newPoint = steer(tree[nearest].point, sample, params.stepSize);
        if (newPoint == tree[nearest].point || !map.segmentFree(tree[nearest].point, newPoint)) {
            continue;
        }

        const double newCost = tree[nearest].cost + distance(tree[nearest].point, newPoint);
        tree.push_back({newPoint, nearest, newCost});

        if (distance(newPoint, goal) <= params.goalTolerance && map.segmentFree(newPoint, goal)) {
            reached = tree.size() - 1;
            goalReached = true;
        }
    }
    result.treeSize = tree.size();
    if (!goalReached) {
        return result;
    }

    result.cost = tree[reached].cost + distance(tree[reached].point, goal);
    if (!(tree[reached].point == goal)) {
        result.path.push_back(goal);
    }
    for (std::size_t node = reached; node != 0; node = tree[node].parent) {
        result.path.push_back(tree[node].point);
    }
    result.path.push_back(start);
    std::reverse(result.path.begin(), result.path.end());
    return result;
}

} 