#pragma once

#include <algorithm>
#include <concepts>
#include <cstddef>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>


namespace planner{
template <typename G>
concept SearchGraph = requires(const G& graph, const typename G::Node& node){
    { graph.isFree(node)} -> std::convertible_to<bool>;
    graph.forEachNeighbor(node, [](const typename G::Node&, double) {}); // аби не створювати вектор на пер вузол
};

template <typename Node>
struct SearchResult{
    std::vector<Node> path;
    double cost = std::numeric_limits<double>::infinity();
    std::size_t expanded = 0;
    bool found() const {return !path.empty();}
};
template <SearchGraph Graph, typename Heuristic> //останній це парам який дозволяє передавати у цю ф-ю і ф-ю і лямбда ф-ю
SearchResult<typename Graph::Node> aStar(const Graph& graph, const typename Graph::Node& start, const typename Graph::Node& goal, Heuristic heur){
    using Node = typename Graph::Node;
    struct Visit{
        double costFromStart;
        Node parent;
    };
    struct  QueueEntry {
        double estimatedTotal;
        double costFromStart;
        Node node;
        bool operator<(const QueueEntry& other) const {
        return estimatedTotal > other.estimatedTotal;} // треба свопнути ці два  оператори адже пріоритетна черга видає найбільше значення коли нас цікавить найменше тому це ми так переписуємо і у цьому випадку спейсшіп не треба
    };
    SearchResult<Node> result;
    if (!graph.isFree(start)||!graph.isFree(goal)){
        return result; // в старті або в цілі "перешкода" отже шляху немає
    }
    std::unordered_map<Node, Visit> visits;
    std::priority_queue<QueueEntry> open;
    visits.insert({start, Visit{0.0, start}});
    open.push({heur(start, goal), 0.0, start});
    



    while(!open.empty()){
        const QueueEntry curr = open.top();
        open.pop();
        if (curr.costFromStart > visits.at(curr.node).costFromStart){continue;}
        if (curr.node == goal){
            result.cost = curr.costFromStart;
            for(Node node = goal; node!=start; node = visits.at(node).parent){//вертаємося назад записуємо шлях 
                result.path.push_back(node);
            }
            result.path.push_back(start);
            std::reverse(result.path.begin(), result.path.end());
            return result;
        }
        ++result.expanded;

        graph.forEachNeighbor(curr.node, [&](const Node& neighbor, double stepCost) {//[&] - показує фактично лямбді змінні curr visits open
            const double newCost = curr.costFromStart + stepCost;
            const auto found = visits.find(neighbor);
            if (found!=visits.end() && newCost >= found -> second.costFromStart){return;}
            visits.insert_or_assign(neighbor, Visit{newCost, curr.node});
            open.push({newCost + heur(neighbor, goal), newCost, neighbor});

        });
    }
    return result;
}

}