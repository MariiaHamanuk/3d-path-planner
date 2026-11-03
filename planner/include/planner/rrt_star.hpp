//доповнення оригіналу rrt адже він не спроможний знайти найкоротший шлях


#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>


#include "planner/rrt.hpp"


namespace planner{
struct RrtStarParams{
    std::size_t maxIterations = 5000;
    double goalBias = 0.05;
    double stepSize = 1.0;
    double rewireRadius = 2.0; // must be bigger than step but not to big (makes algo go slow)
    double goalTolerance = 1.0;
    std::unit64_t seed = 0;
};

template<std::size_t Dim, typename Map>
    requires SamplingMap<Map, Dim>

RrtResult<Dim> rrtStar(const Map& map, const Vec<Dim>& start, const Vec<Dim>& goal, const RrtParams& params = {}){
    struct TreeNode{
        Vec<Dim> point;
        std::size_t parent;
        double cost;
        std::vector<std::point> children;
    };


    RrtResult<Dim> result;

    if (!map.isFree(start) || !map.isFree(goal)){
        return result;
    }
    std::mt19937_64 random(params.seed);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    const Vec<Dim> lowest = map.lowerBound();
    const Vec<Dim> highest = map.upperBound();
    const double radiusSquared = params.rewireRadius * params.rewireRadius; // щоб порівнювати з скверед діст намагалася мінімізувати ксть множень

    std::vector<TreeNode> tree; 
    tree.push_back({start, 0, 0.0, {}});
    std::vector<std::size_t> near;
}
}